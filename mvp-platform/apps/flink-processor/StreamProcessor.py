#!/usr/bin/env python3
"""
Flink流处理任务 - 实时数据分析和异常检测
使用PyFlink进行流处理
"""

from pyflink.datastream import StreamExecutionEnvironment, TimeCharacteristic
from pyflink.table import StreamTableEnvironment, EnvironmentSettings
from pyflink.datastream.connectors import FlinkKafkaConsumer, FlinkKafkaProducer
from pyflink.datastream.formats.json import JsonRowSerializationSchema, JsonRowDeserializationSchema
from pyflink.common.serialization import SimpleStringSchema
from pyflink.common.typeinfo import Types
from pyflink.datastream.functions import KeyedProcessFunction, RuntimeContext
from pyflink.datastream.state import ValueStateDescriptor
import json
import logging
from datetime import datetime, timedelta

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class AnomalyDetector(KeyedProcessFunction):
    """异常检测处理函数"""
    
    def __init__(self):
        self.value_state = None
        self.count_state = None
    
    def open(self, runtime_context: RuntimeContext):
        # 初始化状态
        self.value_state = runtime_context.get_state(
            ValueStateDescriptor("values", Types.LIST(Types.FLOAT()))
        )
        self.count_state = runtime_context.get_state(
            ValueStateDescriptor("count", Types.LONG())
        )
    
    def process_element(self, value, ctx):
        # 获取当前状态
        current_values = self.value_state.value()
        current_count = self.count_state.value()
        
        if current_values is None:
            current_values = []
        if current_count is None:
            current_count = 0
        
        # 解析输入数据
        data = json.loads(value)
        sensor_value = float(data.get('value', 0))
        device_id = data.get('device_id')
        sensor_type = data.get('sensor_type')
        
        # 添加新值到窗口
        current_values.append(sensor_value)
        current_count += 1
        
        # 保持窗口大小为最近100个值
        if len(current_values) > 100:
            current_values.pop(0)
        
        # 计算统计指标
        if len(current_values) >= 10:  # 至少需要10个值才计算异常
            mean = sum(current_values) / len(current_values)
            variance = sum((x - mean) ** 2 for x in current_values) / len(current_values)
            std_dev = variance ** 0.5
            
            # 3-sigma异常检测
            z_score = abs(sensor_value - mean) / (std_dev + 1e-6)  # 避免除零
            
            is_anomaly = z_score > 3.0
            
            # 构建输出数据
            result = {
                'device_id': device_id,
                'sensor_type': sensor_type,
                'value': sensor_value,
                'mean': round(mean, 2),
                'std_dev': round(std_dev, 2),
                'z_score': round(z_score, 2),
                'is_anomaly': is_anomaly,
                'timestamp': datetime.utcnow().isoformat(),
                'window_size': len(current_values)
            }
            
            # 更新状态
            self.value_state.update(current_values)
            self.count_state.update(current_count)
            
            yield json.dumps(result)

def create_kafka_source(env, topic, bootstrap_servers):
    """创建Kafka数据源"""
    properties = {
        'bootstrap.servers': bootstrap_servers,
        'group.id': 'flink-processor-group',
        'auto.offset.reset': 'latest'
    }
    
    kafka_consumer = FlinkKafkaConsumer(
        topics=topic,
        deserialization_schema=SimpleStringSchema(),
        properties=properties
    )
    
    return kafka_consumer

def create_kafka_sink(topic, bootstrap_servers):
    """创建Kafka输出"""
    properties = {
        'bootstrap.servers': bootstrap_servers
    }
    
    kafka_producer = FlinkKafkaProducer(
        topic=topic,
        serialization_schema=SimpleStringSchema(),
        producer_config=properties
    )
    
    return kafka_producer

def main():
    """主函数"""
    # 创建执行环境
    env = StreamExecutionEnvironment.get_execution_environment()
    env.set_time_characteristic(TimeCharacteristic.ProcessingTime)
    env.set_parallelism(1)  # 简化并行度
    
    # Kafka配置
    bootstrap_servers = 'kafka:29092'
    input_topic = 'data-ingestion'
    output_topic = 'anomaly-detection'
    
    # 添加Kafka连接器依赖
    env.add_jars("file:///opt/flink/lib/flink-sql-connector-kafka_2.12-1.17.1.jar")
    
    logger.info("Starting Flink streaming job...")
    
    try:
        # 创建数据源
        kafka_source = create_kafka_source(env, input_topic, bootstrap_servers)
        source_stream = env.add_source(kafka_source)
        
        # 数据处理流水线
        processed_stream = source_stream \
            .key_by(lambda x: json.loads(x).get('device_id', 'unknown')) \
            .process(AnomalyDetector())
        
        # 输出到Kafka
        kafka_sink = create_kafka_sink(output_topic, bootstrap_servers)
        processed_stream.add_sink(kafka_sink)
        
        # 打印到控制台（调试用）
        processed_stream.print()
        
        # 执行作业
        env.execute("Real-time Anomaly Detection")
        
    except Exception as e:
        logger.error(f"Error in Flink job: {e}")
        raise

if __name__ == '__main__':
    main()