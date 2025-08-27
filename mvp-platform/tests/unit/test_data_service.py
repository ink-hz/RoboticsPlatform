#!/usr/bin/env python3
"""
数据服务单元测试
"""

import pytest
import json
from unittest.mock import Mock, patch, MagicMock
import sys
import os

# 添加apps路径到系统路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../apps/data-service'))

from app import create_app, DataService, StreamProcessor

class TestDataService:
    """数据服务测试类"""
    
    @pytest.fixture
    def mock_db(self):
        """Mock数据库"""
        return Mock()
    
    @pytest.fixture
    def mock_kafka(self):
        """Mock Kafka"""
        return Mock()
    
    @pytest.fixture
    def mock_redis(self):
        """Mock Redis"""
        return Mock()
    
    @pytest.fixture
    def data_service(self, mock_db, mock_kafka, mock_redis):
        """创建数据服务实例"""
        with patch('app.threading.Thread'):
            return DataService(mock_db, mock_kafka, mock_redis)
    
    def test_ingest_data_success(self, data_service, mock_kafka):
        """测试数据摄入成功"""
        test_data = {
            "device_id": "sensor_001",
            "sensor_type": "temperature",
            "value": 25.5,
            "metadata": {"location": "building_A"}
        }
        
        result = data_service.ingest_data(test_data)
        
        assert result["success"] is True
        assert "Data ingested" in result["message"]
        mock_kafka.send.assert_called_once_with('data-ingestion', json.dumps(test_data))
    
    def test_ingest_data_failure(self, data_service, mock_kafka):
        """测试数据摄入失败"""
        mock_kafka.send.side_effect = Exception("Kafka error")
        
        test_data = {"device_id": "sensor_001", "value": 25.5}
        result = data_service.ingest_data(test_data)
        
        assert result["success"] is False
        assert "Kafka error" in result["error"]
    
    def test_process_data_success(self, data_service, mock_db, mock_redis, mock_kafka):
        """测试数据处理成功"""
        test_data = {
            "device_id": "sensor_001",
            "sensor_type": "temperature",
            "value": 25.5,
            "metadata": {"location": "building_A"}
        }
        
        data_service.process_data(test_data)
        
        # 验证数据存储到数据库
        mock_db.execute.assert_called()
        
        # 验证缓存到Redis
        mock_redis.set.assert_called()
        
        # 验证没有异常检测（正常值）
        calls = mock_kafka.send.call_args_list
        anomaly_calls = [call for call in calls if 'anomaly-alerts' in str(call)]
        assert len(anomaly_calls) == 0
    
    def test_process_data_with_anomaly(self, data_service, mock_db, mock_redis, mock_kafka):
        """测试异常数据处理"""
        test_data = {
            "device_id": "sensor_001",
            "sensor_type": "temperature",
            "value": 1500,  # 异常值
            "metadata": {"location": "building_A"}
        }
        
        data_service.process_data(test_data)
        
        # 验证异常警报发送
        calls = mock_kafka.send.call_args_list
        anomaly_calls = [call for call in calls if 'anomaly-alerts' in str(call)]
        assert len(anomaly_calls) > 0
    
    def test_detect_anomaly_normal_value(self, data_service):
        """测试正常值异常检测"""
        normal_data = {"value": 25.5}
        assert not data_service.detect_anomaly(normal_data)
    
    def test_detect_anomaly_low_value(self, data_service):
        """测试低异常值检测"""
        anomaly_data = {"value": -50}
        assert data_service.detect_anomaly(anomaly_data)
    
    def test_detect_anomaly_high_value(self, data_service):
        """测试高异常值检测"""
        anomaly_data = {"value": 1500}
        assert data_service.detect_anomaly(anomaly_data)
    
    def test_get_device_data_success(self, data_service, mock_db):
        """测试获取设备数据成功"""
        # Mock数据库返回数据
        mock_row = Mock()
        mock_row.__iter__ = Mock(return_value=iter([('id', 1), ('device_id', 'sensor_001'), ('value', 25.5)]))
        mock_db.execute.return_value = [mock_row]
        
        result = data_service.get_device_data("sensor_001", limit=5)
        
        assert result["success"] is True
        assert "data" in result
        mock_db.execute.assert_called_once()
    
    def test_get_device_data_with_sensor_type(self, data_service, mock_db):
        """测试按传感器类型获取设备数据"""
        mock_db.execute.return_value = []
        
        result = data_service.get_device_data("sensor_001", "temperature", 10)
        
        assert result["success"] is True
        # 验证SQL查询包含sensor_type条件
        call_args = mock_db.execute.call_args
        assert 'sensor_type' in call_args[0][0]
    
    def test_get_analytics_summary_success(self, data_service, mock_db):
        """测试获取分析摘要成功"""
        # Mock数据库返回摘要数据
        mock_row = Mock()
        mock_row.__iter__ = Mock(return_value=iter([
            ('sensor_type', 'temperature'),
            ('count', 10),
            ('avg_value', 25.5),
            ('min_value', 20.0),
            ('max_value', 30.0)
        ]))
        mock_db.execute.return_value = [mock_row]
        
        result = data_service.get_analytics_summary()
        
        assert result["success"] is True
        assert "summary" in result
        assert len(result["summary"]) == 1
    
    def test_get_analytics_summary_failure(self, data_service, mock_db):
        """测试获取分析摘要失败"""
        mock_db.execute.side_effect = Exception("Database error")
        
        result = data_service.get_analytics_summary()
        
        assert result["success"] is False
        assert "Database error" in result["error"]

class TestStreamProcessor:
    """流处理器测试类"""
    
    @pytest.fixture
    def mock_kafka(self):
        """Mock Kafka"""
        return Mock()
    
    @pytest.fixture
    def stream_processor(self, mock_kafka):
        """创建流处理器实例"""
        with patch('app.threading.Thread'):
            return StreamProcessor(mock_kafka)
    
    def test_initial_metrics(self, stream_processor):
        """测试初始指标"""
        assert stream_processor.metrics['processed_count'] == 0
        assert stream_processor.metrics['anomaly_count'] == 0
        assert stream_processor.metrics['average_latency'] == 0
    
    def test_process_stream_normal_data(self, stream_processor, mock_kafka):
        """测试处理正常流数据"""
        test_data = {
            "device_id": "sensor_001",
            "sensor_type": "temperature",
            "value": 25.5
        }
        
        stream_processor.process_stream(test_data)
        
        # 验证指标更新
        assert stream_processor.metrics['processed_count'] == 1
        assert stream_processor.metrics['anomaly_count'] == 0
        
        # 验证发送到处理后的主题
        mock_kafka.send.assert_called_once()
        call_args = mock_kafka.send.call_args
        assert 'processed-data' in call_args[0]
    
    def test_process_stream_anomaly_data(self, stream_processor, mock_kafka):
        """测试处理异常流数据"""
        test_data = {
            "device_id": "sensor_001",
            "sensor_type": "temperature",
            "value": 1500  # 异常值
        }
        
        stream_processor.process_stream(test_data)
        
        # 验证异常计数增加
        assert stream_processor.metrics['anomaly_count'] == 1
    
    def test_process_stream_latency_calculation(self, stream_processor, mock_kafka):
        """测试流处理延迟计算"""
        test_data = {"value": 25.5}
        
        initial_latency = stream_processor.metrics['average_latency']
        stream_processor.process_stream(test_data)
        
        # 验证延迟被更新
        assert stream_processor.metrics['average_latency'] >= 0

class TestDataAPI:
    """数据API测试类"""
    
    @pytest.fixture
    def app(self):
        """创建测试应用"""
        with patch('app.DatabaseManager'), \
             patch('app.redis.Redis'), \
             patch('app.KafkaProducer'), \
             patch('app.KafkaConsumer'):
            app = create_app()
            app.config['TESTING'] = True
            with app.test_client() as client:
                yield client
    
    def test_health_endpoint(self, app):
        """测试健康检查端点"""
        response = app.get('/health')
        data = json.loads(response.data)
        
        assert response.status_code == 200
        assert data["status"] == "healthy"
    
    @patch('app.data_service')
    def test_ingest_endpoint_success(self, mock_data_service, app):
        """测试数据摄入端点成功"""
        mock_data_service.ingest_data.return_value = {"success": True, "message": "Data ingested"}
        
        test_data = {
            "device_id": "sensor_001",
            "sensor_type": "temperature",
            "value": 25.5
        }
        
        response = app.post('/api/data/ingest',
                          data=json.dumps(test_data),
                          content_type='application/json')
        
        data = json.loads(response.data)
        assert response.status_code == 200
        assert data["success"] is True
    
    @patch('app.data_service')
    def test_get_device_data_endpoint(self, mock_data_service, app):
        """测试获取设备数据端点"""
        mock_data_service.get_device_data.return_value = {
            "success": True,
            "data": [{"device_id": "sensor_001", "value": 25.5}]
        }
        
        response = app.get('/api/data/sensor_001?limit=5')
        
        data = json.loads(response.data)
        assert response.status_code == 200
        assert data["success"] is True
        assert "data" in data
    
    @patch('app.data_service')
    def test_analytics_summary_endpoint(self, mock_data_service, app):
        """测试分析摘要端点"""
        mock_data_service.get_analytics_summary.return_value = {
            "success": True,
            "summary": [{"sensor_type": "temperature", "count": 10}]
        }
        
        response = app.get('/api/analytics/summary')
        
        data = json.loads(response.data)
        assert response.status_code == 200
        assert data["success"] is True
        assert "summary" in data
    
    @patch('app.stream_processor')
    def test_metrics_endpoint(self, mock_stream_processor, app):
        """测试指标端点"""
        mock_stream_processor.metrics = {
            'processed_count': 100,
            'anomaly_count': 5,
            'average_latency': 1.23
        }
        
        response = app.get('/api/metrics')
        
        data = json.loads(response.data)
        assert response.status_code == 200
        assert "stream_processor" in data
        assert data["stream_processor"]["processed_count"] == 100

if __name__ == '__main__':
    pytest.main([__file__])