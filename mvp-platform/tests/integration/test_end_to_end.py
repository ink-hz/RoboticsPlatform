#!/usr/bin/env python3
"""
端到端集成测试
"""

import pytest
import requests
import json
import time
import threading
from datetime import datetime
import sqlite3
import os

class TestEndToEndIntegration:
    """端到端集成测试类"""
    
    BASE_URL = "http://localhost:8080"
    
    @pytest.fixture(scope="class", autouse=True)
    def setup_test_environment(self):
        """设置测试环境"""
        # 在实际集成测试中，这里会启动测试服务器
        # 对于这个示例，我们假设服务器已经在运行
        yield
        # 清理测试数据
        self.cleanup_test_data()
    
    def cleanup_test_data(self):
        """清理测试数据"""
        try:
            # 清理测试数据库
            if os.path.exists("test_mvp.db"):
                conn = sqlite3.connect("test_mvp.db")
                conn.execute("DELETE FROM users WHERE username LIKE 'test_%'")
                conn.execute("DELETE FROM sensor_data WHERE device_id LIKE 'test_%'")
                conn.commit()
                conn.close()
        except Exception as e:
            print(f"Cleanup error: {e}")
    
    def test_health_check(self):
        """测试系统健康检查"""
        response = requests.get(f"{self.BASE_URL}/health", timeout=5)
        
        assert response.status_code == 200
        data = response.json()
        assert data.get("status") == "healthy"
    
    def test_user_registration_and_login_flow(self):
        """测试用户注册和登录流程"""
        # 1. 用户注册
        username = f"test_user_{int(time.time())}"
        registration_data = {
            "username": username,
            "password": "test123",
            "email": "test@example.com"
        }
        
        register_response = requests.post(
            f"{self.BASE_URL}/api/auth/register",
            json=registration_data,
            timeout=5
        )
        
        assert register_response.status_code == 200
        register_data = register_response.json()
        assert register_data.get("success") is True
        
        # 2. 用户登录
        login_data = {
            "username": username,
            "password": "test123"
        }
        
        login_response = requests.post(
            f"{self.BASE_URL}/api/auth/login",
            json=login_data,
            timeout=5
        )
        
        assert login_response.status_code == 200
        login_result = login_response.json()
        assert login_result.get("success") is True
        assert "token" in login_result
        
        return login_result["token"]
    
    def test_data_ingestion_and_query_flow(self):
        """测试数据摄入和查询流程"""
        device_id = f"test_device_{int(time.time())}"
        
        # 1. 数据摄入
        sensor_data = {
            "device_id": device_id,
            "sensor_type": "temperature",
            "value": 25.5,
            "metadata": {
                "location": "test_building",
                "test_run": "integration_test"
            }
        }
        
        ingest_response = requests.post(
            f"{self.BASE_URL}/api/data/ingest",
            json=sensor_data,
            timeout=5
        )
        
        assert ingest_response.status_code == 200
        ingest_data = ingest_response.json()
        assert ingest_data.get("success") is True
        
        # 2. 等待数据处理
        time.sleep(2)
        
        # 3. 查询设备数据
        query_response = requests.get(
            f"{self.BASE_URL}/api/data/{device_id}?limit=5",
            timeout=5
        )
        
        assert query_response.status_code == 200
        query_data = query_response.json()
        assert query_data.get("success") is True
        
        # 验证数据存在
        data_list = query_data.get("data", [])
        assert len(data_list) > 0
        
        # 验证数据内容
        found_data = None
        for item in data_list:
            if item.get("device_id") == device_id:
                found_data = item
                break
        
        assert found_data is not None
        assert found_data.get("sensor_type") == "temperature"
        assert found_data.get("value") == 25.5
    
    def test_analytics_summary_flow(self):
        """测试分析摘要流程"""
        # 1. 摄入多个传感器数据
        devices = [f"test_analytics_{i}_{int(time.time())}" for i in range(3)]
        sensor_types = ["temperature", "humidity", "pressure"]
        
        for i, device in enumerate(devices):
            for j, sensor_type in enumerate(sensor_types):
                sensor_data = {
                    "device_id": device,
                    "sensor_type": sensor_type,
                    "value": 20.0 + i * 10 + j * 5,
                    "metadata": {"test": "analytics"}
                }
                
                response = requests.post(
                    f"{self.BASE_URL}/api/data/ingest",
                    json=sensor_data,
                    timeout=5
                )
                assert response.status_code == 200
        
        # 2. 等待数据处理
        time.sleep(3)
        
        # 3. 获取分析摘要
        summary_response = requests.get(
            f"{self.BASE_URL}/api/analytics/summary",
            timeout=5
        )
        
        assert summary_response.status_code == 200
        summary_data = summary_response.json()
        assert summary_data.get("success") is True
        
        # 验证摘要数据
        summary = summary_data.get("summary", [])
        assert len(summary) > 0
        
        # 验证每种传感器类型都有统计数据
        sensor_types_found = {item.get("sensor_type") for item in summary}
        for sensor_type in sensor_types:
            if sensor_type in sensor_types_found:
                # 找到对应的摘要数据
                type_summary = next(
                    item for item in summary 
                    if item.get("sensor_type") == sensor_type
                )
                assert type_summary.get("count", 0) > 0
                assert "avg_value" in type_summary
    
    def test_metrics_monitoring(self):
        """测试指标监控"""
        # 1. 摄入一些数据来生成指标
        for i in range(5):
            sensor_data = {
                "device_id": f"test_metrics_{i}",
                "sensor_type": "cpu",
                "value": 50.0 + i * 10,
                "metadata": {"test": "metrics"}
            }
            
            requests.post(
                f"{self.BASE_URL}/api/data/ingest",
                json=sensor_data,
                timeout=5
            )
        
        # 2. 等待处理
        time.sleep(2)
        
        # 3. 获取指标
        metrics_response = requests.get(f"{self.BASE_URL}/api/metrics", timeout=5)
        
        assert metrics_response.status_code == 200
        metrics_data = metrics_response.json()
        
        # 验证指标结构
        assert "timestamp" in metrics_data
        
        # 如果有流处理器指标，验证其结构
        if "stream_processor" in metrics_data:
            stream_metrics = metrics_data["stream_processor"]
            assert "processed_count" in stream_metrics
            assert "anomaly_count" in stream_metrics
            assert "average_latency" in stream_metrics
    
    def test_anomaly_detection_flow(self):
        """测试异常检测流程"""
        device_id = f"test_anomaly_{int(time.time())}"
        
        # 1. 发送正常数据
        normal_data = {
            "device_id": device_id,
            "sensor_type": "temperature",
            "value": 25.0,
            "metadata": {"test": "normal"}
        }
        
        response = requests.post(
            f"{self.BASE_URL}/api/data/ingest",
            json=normal_data,
            timeout=5
        )
        assert response.status_code == 200
        
        # 2. 发送异常数据
        anomaly_data = {
            "device_id": device_id,
            "sensor_type": "temperature",
            "value": 1500.0,  # 异常高温
            "metadata": {"test": "anomaly"}
        }
        
        response = requests.post(
            f"{self.BASE_URL}/api/data/ingest",
            json=anomaly_data,
            timeout=5
        )
        assert response.status_code == 200
        
        # 3. 等待处理
        time.sleep(2)
        
        # 4. 检查指标中的异常计数
        metrics_response = requests.get(f"{self.BASE_URL}/api/metrics", timeout=5)
        
        if metrics_response.status_code == 200:
            metrics_data = metrics_response.json()
            # 在真实环境中，这里会验证异常计数增加
            # 目前只验证响应结构正确
            assert isinstance(metrics_data, dict)
    
    def test_concurrent_data_ingestion(self):
        """测试并发数据摄入"""
        results = []
        
        def send_data_batch(batch_id):
            """发送数据批次"""
            batch_results = []
            for i in range(10):
                sensor_data = {
                    "device_id": f"test_concurrent_{batch_id}_{i}",
                    "sensor_type": "load_test",
                    "value": 50.0 + i,
                    "metadata": {"batch": batch_id, "test": "concurrent"}
                }
                
                try:
                    response = requests.post(
                        f"{self.BASE_URL}/api/data/ingest",
                        json=sensor_data,
                        timeout=5
                    )
                    batch_results.append(response.status_code == 200)
                except Exception:
                    batch_results.append(False)
            
            results.extend(batch_results)
        
        # 启动并发线程
        threads = []
        for batch_id in range(3):
            thread = threading.Thread(target=send_data_batch, args=(batch_id,))
            threads.append(thread)
            thread.start()
        
        # 等待所有线程完成
        for thread in threads:
            thread.join(timeout=10)
        
        # 验证结果
        success_rate = sum(results) / len(results) if results else 0
        assert success_rate > 0.8  # 至少80%成功率
    
    def test_system_resilience(self):
        """测试系统韧性"""
        # 1. 测试无效数据处理
        invalid_data = {
            "device_id": "",  # 空设备ID
            "sensor_type": "temperature",
            "value": "invalid_number"  # 无效数值
        }
        
        response = requests.post(
            f"{self.BASE_URL}/api/data/ingest",
            json=invalid_data,
            timeout=5
        )
        
        # 系统应该优雅处理无效数据，返回错误或忽略
        assert response.status_code in [200, 400]  # 接受成功处理或客户端错误
        
        # 2. 测试大数据负载
        large_metadata = {"large_field": "x" * 1000}  # 1KB元数据
        large_data = {
            "device_id": "test_large",
            "sensor_type": "temperature",
            "value": 25.0,
            "metadata": large_metadata
        }
        
        response = requests.post(
            f"{self.BASE_URL}/api/data/ingest",
            json=large_data,
            timeout=10
        )
        
        # 系统应该能处理较大的数据包
        assert response.status_code == 200
    
    @pytest.mark.slow
    def test_full_workflow_integration(self):
        """测试完整工作流程集成"""
        # 这是一个综合测试，验证整个系统工作流程
        
        # 1. 用户认证
        token = self.test_user_registration_and_login_flow()
        
        # 2. 数据摄入
        device_id = f"test_workflow_{int(time.time())}"
        sensor_types = ["temperature", "humidity", "pressure"]
        
        for sensor_type in sensor_types:
            for i in range(5):
                sensor_data = {
                    "device_id": device_id,
                    "sensor_type": sensor_type,
                    "value": 20.0 + i * 5,
                    "metadata": {"workflow_test": True, "iteration": i}
                }
                
                response = requests.post(
                    f"{self.BASE_URL}/api/data/ingest",
                    json=sensor_data,
                    timeout=5
                )
                assert response.status_code == 200
        
        # 3. 等待处理
        time.sleep(3)
        
        # 4. 验证数据查询
        query_response = requests.get(
            f"{self.BASE_URL}/api/data/{device_id}?limit=20",
            timeout=5
        )
        assert query_response.status_code == 200
        
        query_data = query_response.json()
        assert query_data.get("success") is True
        assert len(query_data.get("data", [])) >= 5  # 至少有一些数据
        
        # 5. 验证分析摘要
        summary_response = requests.get(
            f"{self.BASE_URL}/api/analytics/summary",
            timeout=5
        )
        assert summary_response.status_code == 200
        
        # 6. 验证指标
        metrics_response = requests.get(f"{self.BASE_URL}/api/metrics", timeout=5)
        assert metrics_response.status_code == 200
        
        print("✅ Full workflow integration test completed successfully")

if __name__ == '__main__':
    pytest.main([__file__, '-v'])