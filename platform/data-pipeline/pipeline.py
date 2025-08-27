import asyncio
import json
from typing import Dict, Any, List
from datetime import datetime, timedelta
import os
from dataclasses import dataclass
from enum import Enum

from kafka import KafkaProducer, KafkaConsumer
from kafka.errors import KafkaError
import psycopg2
from psycopg2.extras import RealDictCursor
import pandas as pd
import numpy as np
from minio import Minio
from prometheus_client import Counter, Histogram, Gauge

class DataType(Enum):
    TELEMETRY = "telemetry"
    SENSOR_IMAGE = "sensor_image"  
    POINT_CLOUD = "point_cloud"
    IMU = "imu"
    ODOMETRY = "odometry"
    DIAGNOSTICS = "diagnostics"
    COMMAND = "command"
    STATUS = "status"

@dataclass
class SensorData:
    robot_id: str
    sensor_id: str
    data_type: DataType
    timestamp: datetime
    data: Dict[str, Any]
    metadata: Dict[str, Any]

class DataPipeline:
    def __init__(self):
        self.kafka_producer = KafkaProducer(
            bootstrap_servers=os.getenv("KAFKA_BROKERS", "localhost:9092"),
            value_serializer=lambda v: json.dumps(v).encode('utf-8'),
            compression_type='gzip'
        )
        
        self.kafka_consumer = KafkaConsumer(
            'robot-telemetry',
            bootstrap_servers=os.getenv("KAFKA_BROKERS", "localhost:9092"),
            auto_offset_reset='latest',
            value_deserializer=lambda m: json.loads(m.decode('utf-8'))
        )
        
        self.timescale_conn = psycopg2.connect(
            host=os.getenv("TIMESCALE_HOST", "localhost"),
            port=os.getenv("TIMESCALE_PORT", "5433"),
            database=os.getenv("TIMESCALE_DB", "telemetry"),
            user=os.getenv("TIMESCALE_USER", "tsdb"),
            password=os.getenv("TIMESCALE_PASSWORD", "changeme")
        )
        
        self.minio_client = Minio(
            os.getenv("MINIO_ENDPOINT", "localhost:9000"),
            access_key=os.getenv("MINIO_ACCESS_KEY", "minioadmin"),
            secret_key=os.getenv("MINIO_SECRET_KEY", "minioadmin"),
            secure=False
        )
        
        self.metrics_processed = Counter('data_pipeline_processed_total', 
                                        'Total processed data points', 
                                        ['data_type'])
        self.processing_time = Histogram('data_pipeline_processing_seconds',
                                        'Processing time in seconds',
                                        ['data_type'])
        self.buffer_size = Gauge('data_pipeline_buffer_size',
                                'Current buffer size')
        
        self.setup_database()
        self.setup_storage()
        
    def setup_database(self):
        """初始化TimescaleDB表"""
        with self.timescale_conn.cursor() as cursor:
            cursor.execute("""
                CREATE TABLE IF NOT EXISTS robot_telemetry (
                    time TIMESTAMPTZ NOT NULL,
                    robot_id VARCHAR(50) NOT NULL,
                    sensor_id VARCHAR(50),
                    data_type VARCHAR(30),
                    value JSONB,
                    metadata JSONB
                );
            """)
            
            cursor.execute("""
                SELECT create_hypertable('robot_telemetry', 'time', 
                                        if_not_exists => TRUE);
            """)
            
            cursor.execute("""
                CREATE INDEX IF NOT EXISTS idx_robot_telemetry_robot_id 
                ON robot_telemetry (robot_id, time DESC);
            """)
            
            cursor.execute("""
                CREATE TABLE IF NOT EXISTS robot_status (
                    robot_id VARCHAR(50) PRIMARY KEY,
                    last_seen TIMESTAMPTZ,
                    status VARCHAR(20),
                    battery_level FLOAT,
                    location JSONB,
                    active_mission VARCHAR(100)
                );
            """)
            
            self.timescale_conn.commit()
    
    def setup_storage(self):
        """初始化MinIO存储桶"""
        buckets = [
            "robot-images",
            "point-clouds", 
            "models",
            "logs",
            "backups"
        ]
        
        for bucket in buckets:
            if not self.minio_client.bucket_exists(bucket):
                self.minio_client.make_bucket(bucket)
    
    async def ingest_data(self, data: Dict[str, Any]) -> bool:
        """接收并处理传入的数据"""
        try:
            sensor_data = self.parse_sensor_data(data)
            
            if sensor_data.data_type in [DataType.TELEMETRY, DataType.IMU, 
                                         DataType.ODOMETRY, DataType.STATUS]:
                await self.store_timeseries(sensor_data)
                
            elif sensor_data.data_type in [DataType.SENSOR_IMAGE, DataType.POINT_CLOUD]:
                await self.store_blob(sensor_data)
                
            self.kafka_producer.send(f"processed-{sensor_data.data_type.value}", 
                                    value=data)
            
            self.metrics_processed.labels(data_type=sensor_data.data_type.value).inc()
            
            if sensor_data.data_type == DataType.STATUS:
                await self.update_robot_status(sensor_data)
                
            return True
            
        except Exception as e:
            print(f"Error processing data: {e}")
            return False
    
    def parse_sensor_data(self, raw_data: Dict[str, Any]) -> SensorData:
        """解析原始传感器数据"""
        return SensorData(
            robot_id=raw_data.get("robot_id"),
            sensor_id=raw_data.get("sensor_id", "unknown"),
            data_type=DataType(raw_data.get("data_type", "telemetry")),
            timestamp=datetime.fromisoformat(raw_data.get("timestamp")),
            data=raw_data.get("data", {}),
            metadata=raw_data.get("metadata", {})
        )
    
    async def store_timeseries(self, sensor_data: SensorData):
        """存储时序数据到TimescaleDB"""
        with self.timescale_conn.cursor() as cursor:
            cursor.execute("""
                INSERT INTO robot_telemetry (time, robot_id, sensor_id, 
                                            data_type, value, metadata)
                VALUES (%s, %s, %s, %s, %s, %s)
            """, (
                sensor_data.timestamp,
                sensor_data.robot_id,
                sensor_data.sensor_id,
                sensor_data.data_type.value,
                json.dumps(sensor_data.data),
                json.dumps(sensor_data.metadata)
            ))
            self.timescale_conn.commit()
    
    async def store_blob(self, sensor_data: SensorData):
        """存储大型二进制数据到MinIO"""
        bucket_name = "robot-images" if sensor_data.data_type == DataType.SENSOR_IMAGE else "point-clouds"
        
        object_name = f"{sensor_data.robot_id}/{sensor_data.timestamp.isoformat()}/{sensor_data.sensor_id}"
        
        import io
        data_bytes = json.dumps(sensor_data.data).encode('utf-8')
        data_stream = io.BytesIO(data_bytes)
        
        self.minio_client.put_object(
            bucket_name,
            object_name,
            data_stream,
            length=len(data_bytes),
            metadata=sensor_data.metadata
        )
    
    async def update_robot_status(self, sensor_data: SensorData):
        """更新机器人状态"""
        status_data = sensor_data.data
        
        with self.timescale_conn.cursor() as cursor:
            cursor.execute("""
                INSERT INTO robot_status (robot_id, last_seen, status, 
                                         battery_level, location, active_mission)
                VALUES (%s, %s, %s, %s, %s, %s)
                ON CONFLICT (robot_id) 
                DO UPDATE SET 
                    last_seen = EXCLUDED.last_seen,
                    status = EXCLUDED.status,
                    battery_level = EXCLUDED.battery_level,
                    location = EXCLUDED.location,
                    active_mission = EXCLUDED.active_mission
            """, (
                sensor_data.robot_id,
                sensor_data.timestamp,
                status_data.get("status", "unknown"),
                status_data.get("battery_level"),
                json.dumps(status_data.get("location", {})),
                status_data.get("active_mission")
            ))
            self.timescale_conn.commit()
    
    async def query_telemetry(self, robot_id: str, 
                              start_time: datetime,
                              end_time: datetime,
                              data_type: Optional[str] = None) -> List[Dict]:
        """查询历史遥测数据"""
        with self.timescale_conn.cursor(cursor_factory=RealDictCursor) as cursor:
            query = """
                SELECT time, sensor_id, data_type, value, metadata
                FROM robot_telemetry
                WHERE robot_id = %s AND time BETWEEN %s AND %s
            """
            params = [robot_id, start_time, end_time]
            
            if data_type:
                query += " AND data_type = %s"
                params.append(data_type)
                
            query += " ORDER BY time DESC"
            
            cursor.execute(query, params)
            return cursor.fetchall()
    
    async def aggregate_metrics(self, robot_id: str, 
                               metric_name: str,
                               interval: str = "1 hour") -> pd.DataFrame:
        """聚合指标数据"""
        query = f"""
            SELECT 
                time_bucket('{interval}', time) AS bucket,
                avg((value->>{metric_name})::float) as avg_value,
                min((value->>{metric_name})::float) as min_value,
                max((value->>{metric_name})::float) as max_value,
                count(*) as data_points
            FROM robot_telemetry
            WHERE robot_id = %s
            AND value ? %s
            AND time > NOW() - INTERVAL '24 hours'
            GROUP BY bucket
            ORDER BY bucket DESC
        """
        
        df = pd.read_sql_query(query, self.timescale_conn, params=[robot_id, metric_name])
        return df
    
    async def run_consumer(self):
        """运行Kafka消费者主循环"""
        print("Starting data pipeline consumer...")
        
        for message in self.kafka_consumer:
            try:
                await self.ingest_data(message.value)
            except Exception as e:
                print(f"Error processing message: {e}")
    
    def close(self):
        """清理资源"""
        self.kafka_producer.close()
        self.kafka_consumer.close()
        self.timescale_conn.close()

async def main():
    pipeline = DataPipeline()
    
    try:
        await pipeline.run_consumer()
    except KeyboardInterrupt:
        print("Shutting down data pipeline...")
    finally:
        pipeline.close()

if __name__ == "__main__":
    asyncio.run(main())