#!/usr/bin/env python3
"""
数据处理微服务 - 处理数据摄入、存储和查询
"""

from flask import Flask, jsonify, request
from flask_cors import CORS
from kafka import KafkaProducer, KafkaConsumer
import redis
import psycopg2
from psycopg2.extras import RealDictCursor
import json
import threading
import time
from datetime import datetime
import os
from prometheus_flask_exporter import PrometheusMetrics
import logging

app = Flask(__name__)
CORS(app)

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Prometheus监控
metrics = PrometheusMetrics(app)
metrics.info('data_service', 'Data processing microservice', version='1.0.0')

# Kafka配置
KAFKA_BOOTSTRAP_SERVERS = os.environ.get('KAFKA_SERVERS', 'kafka:29092')
KAFKA_TOPIC_DATA_INGESTION = 'data-ingestion'
KAFKA_TOPIC_PROCESSED_DATA = 'processed-data'

# Redis连接
redis_client = redis.Redis(
    host=os.environ.get('REDIS_HOST', 'redis'),
    port=int(os.environ.get('REDIS_PORT', 6379)),
    decode_responses=True
)

# PostgreSQL连接配置
DB_CONFIG = {
    'host': os.environ.get('DB_HOST', 'postgres'),
    'database': os.environ.get('DB_NAME', 'mvpdb'),
    'user': os.environ.get('DB_USER', 'admin'),
    'password': os.environ.get('DB_PASSWORD', 'admin123'),
    'port': os.environ.get('DB_PORT', 5432)
}

def get_db_connection():
    """获取数据库连接"""
    return psycopg2.connect(**DB_CONFIG)

def init_database():
    """初始化数据库表"""
    conn = get_db_connection()
    cur = conn.cursor()
    
    # 创建数据表
    cur.execute('''
        CREATE TABLE IF NOT EXISTS sensor_data (
            id SERIAL PRIMARY KEY,
            device_id VARCHAR(100) NOT NULL,
            sensor_type VARCHAR(50) NOT NULL,
            value FLOAT NOT NULL,
            timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
            metadata JSONB
        )
    ''')
    
    # 创建索引
    cur.execute('CREATE INDEX IF NOT EXISTS idx_device_timestamp ON sensor_data(device_id, timestamp)')
    cur.execute('CREATE INDEX IF NOT EXISTS idx_sensor_type ON sensor_data(sensor_type)')
    
    conn.commit()
    cur.close()
    conn.close()

class KafkaDataProcessor:
    """Kafka数据处理器"""
    
    def __init__(self):
        self.producer = KafkaProducer(
            bootstrap_servers=KAFKA_BOOTSTRAP_SERVERS,
            value_serializer=lambda x: json.dumps(x).encode('utf-8'),
            acks='all',
            retries=3,
            max_in_flight_requests_per_connection=1
        )
        self.running = False
        
    def start_consumer(self):
        """启动消费者线程"""
        self.running = True
        consumer_thread = threading.Thread(target=self._consume_messages)
        consumer_thread.daemon = True
        consumer_thread.start()
        logger.info("Kafka consumer started")
    
    def stop_consumer(self):
        """停止消费者"""
        self.running = False
        
    def _consume_messages(self):
        """消费Kafka消息"""
        consumer = KafkaConsumer(
            KAFKA_TOPIC_DATA_INGESTION,
            bootstrap_servers=KAFKA_BOOTSTRAP_SERVERS,
            auto_offset_reset='latest',
            enable_auto_commit=True,
            group_id='data-processor-group',
            value_deserializer=lambda m: json.loads(m.decode('utf-8'))
        )
        
        while self.running:
            try:
                for message in consumer:
                    if not self.running:
                        break
                    
                    data = message.value
                    logger.info(f"Processing message: {data}")
                    
                    # 处理数据
                    processed_data = self._process_data(data)
                    
                    # 存储到数据库
                    self._store_data(processed_data)
                    
                    # 发送到处理后的topic
                    self.producer.send(KAFKA_TOPIC_PROCESSED_DATA, processed_data)
                    
            except Exception as e:
                logger.error(f"Error in consumer: {e}")
                time.sleep(1)
                
        consumer.close()
    
    def _process_data(self, data):
        """数据处理逻辑"""
        processed = {
            'original_data': data,
            'processed_at': datetime.utcnow().isoformat(),
            'device_id': data.get('device_id'),
            'sensor_type': data.get('sensor_type'),
            'value': float(data.get('value', 0)),
            'metadata': data.get('metadata', {})
        }
        
        # 简单的数据清洗和验证
        if processed['value'] < 0:
            processed['metadata']['anomaly'] = 'negative_value'
        
        if processed['value'] > 1000:
            processed['metadata']['anomaly'] = 'high_value'
            
        return processed
    
    def _store_data(self, data):
        """存储数据到PostgreSQL"""
        try:
            conn = get_db_connection()
            cur = conn.cursor()
            
            cur.execute('''
                INSERT INTO sensor_data (device_id, sensor_type, value, metadata)
                VALUES (%s, %s, %s, %s)
            ''', (
                data['device_id'],
                data['sensor_type'],
                data['value'],
                json.dumps(data['metadata'])
            ))
            
            conn.commit()
            cur.close()
            conn.close()
            
            # 缓存最新数据到Redis
            cache_key = f"latest_data:{data['device_id']}:{data['sensor_type']}"
            redis_client.setex(cache_key, 3600, json.dumps(data))
            
        except Exception as e:
            logger.error(f"Error storing data: {e}")

# 创建Kafka处理器实例
kafka_processor = KafkaDataProcessor()

@app.route('/health', methods=['GET'])
def health_check():
    """健康检查"""
    try:
        # 检查Redis
        redis_client.ping()
        # 检查PostgreSQL
        conn = get_db_connection()
        conn.close()
        # 检查Kafka连接
        producer = KafkaProducer(bootstrap_servers=KAFKA_BOOTSTRAP_SERVERS)
        producer.close()
        
        return jsonify({'status': 'healthy', 'service': 'data-service'}), 200
    except Exception as e:
        return jsonify({'status': 'unhealthy', 'error': str(e)}), 503

@app.route('/ingest', methods=['POST'])
def ingest_data():
    """数据摄入API"""
    try:
        data = request.get_json()
        
        # 验证必需字段
        required_fields = ['device_id', 'sensor_type', 'value']
        for field in required_fields:
            if field not in data:
                return jsonify({'error': f'Missing required field: {field}'}), 400
        
        # 添加时间戳
        data['timestamp'] = datetime.utcnow().isoformat()
        
        # 发送到Kafka
        kafka_processor.producer.send(KAFKA_TOPIC_DATA_INGESTION, data)
        kafka_processor.producer.flush()
        
        return jsonify({'message': 'Data ingested successfully', 'id': data.get('device_id')}), 201
        
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@app.route('/data/<device_id>', methods=['GET'])
def get_device_data(device_id):
    """获取设备数据"""
    try:
        # 从URL参数获取查询选项
        sensor_type = request.args.get('sensor_type')
        limit = int(request.args.get('limit', 100))
        
        # 先检查Redis缓存
        if sensor_type:
            cache_key = f"latest_data:{device_id}:{sensor_type}"
            cached_data = redis_client.get(cache_key)
            if cached_data:
                return jsonify({'data': [json.loads(cached_data)], 'source': 'cache'}), 200
        
        # 从数据库查询
        conn = get_db_connection()
        cur = conn.cursor(cursor_factory=RealDictCursor)
        
        query = 'SELECT * FROM sensor_data WHERE device_id = %s'
        params = [device_id]
        
        if sensor_type:
            query += ' AND sensor_type = %s'
            params.append(sensor_type)
            
        query += ' ORDER BY timestamp DESC LIMIT %s'
        params.append(limit)
        
        cur.execute(query, params)
        results = cur.fetchall()
        cur.close()
        conn.close()
        
        # 转换为JSON可序列化格式
        data = []
        for row in results:
            row_dict = dict(row)
            if row_dict['timestamp']:
                row_dict['timestamp'] = row_dict['timestamp'].isoformat()
            data.append(row_dict)
        
        return jsonify({'data': data, 'source': 'database', 'count': len(data)}), 200
        
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@app.route('/analytics/summary', methods=['GET'])
def get_analytics_summary():
    """获取数据分析摘要"""
    try:
        conn = get_db_connection()
        cur = conn.cursor(cursor_factory=RealDictCursor)
        
        # 统计查询
        cur.execute('''
            SELECT 
                sensor_type,
                COUNT(*) as count,
                AVG(value) as avg_value,
                MIN(value) as min_value,
                MAX(value) as max_value,
                COUNT(DISTINCT device_id) as unique_devices
            FROM sensor_data 
            WHERE timestamp > NOW() - INTERVAL '24 hours'
            GROUP BY sensor_type
        ''')
        
        results = cur.fetchall()
        cur.close()
        conn.close()
        
        # 格式化结果
        summary = []
        for row in results:
            row_dict = dict(row)
            # 格式化浮点数
            if row_dict['avg_value']:
                row_dict['avg_value'] = round(float(row_dict['avg_value']), 2)
            summary.append(row_dict)
        
        return jsonify({'summary': summary, 'period': '24_hours'}), 200
        
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@app.route('/metrics', methods=['GET'])
def metrics_endpoint():
    """Prometheus指标端点"""
    return metrics.generate_latest_responses()

@app.before_first_request
def startup():
    """应用启动时初始化"""
    init_database()
    kafka_processor.start_consumer()

if __name__ == '__main__':
    init_database()
    kafka_processor.start_consumer()
    app.run(host='0.0.0.0', port=8080, debug=True)