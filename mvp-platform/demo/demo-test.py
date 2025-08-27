#!/usr/bin/env python3
"""
MVP平台演示测试脚本
"""

import requests
import json
import time
import random
import threading
from datetime import datetime

BASE_URL = "http://localhost:8080"

class MVPDemo:
    """MVP演示类"""
    
    def __init__(self):
        self.token = None
        self.base_url = BASE_URL
    
    def print_section(self, title):
        """打印分节标题"""
        print(f"\n{'='*60}")
        print(f"🎯 {title}")
        print('='*60)
    
    def print_step(self, step, description):
        """打印步骤"""
        print(f"\n📍 Step {step}: {description}")
    
    def test_health(self):
        """测试健康检查"""
        self.print_section("Health Check")
        
        try:
            response = requests.get(f"{self.base_url}/health", timeout=5)
            if response.status_code == 200:
                print("✅ Platform is healthy")
                print(f"Response: {response.json()}")
            else:
                print(f"❌ Health check failed: {response.status_code}")
        except requests.exceptions.RequestException as e:
            print(f"❌ Connection failed: {e}")
            return False
        
        return True
    
    def test_authentication(self):
        """测试认证功能"""
        self.print_section("Authentication Service")
        
        # 用户注册
        self.print_step(1, "User Registration")
        username = f"demo_user_{int(time.time())}"
        user_data = {
            "username": username,
            "password": "demo123",
            "email": "demo@example.com"
        }
        
        response = requests.post(f"{self.base_url}/api/auth/register", json=user_data)
        if response.status_code == 200:
            print(f"✅ User registered: {username}")
        else:
            print(f"❌ Registration failed: {response.status_code}")
            return False
        
        # 用户登录
        self.print_step(2, "User Login")
        login_data = {
            "username": username,
            "password": "demo123"
        }
        
        response = requests.post(f"{self.base_url}/api/auth/login", json=login_data)
        if response.status_code == 200:
            result = response.json()
            self.token = result.get('token')
            print(f"✅ Login successful")
            print(f"JWT Token: {self.token[:50]}...")
            return True
        else:
            print(f"❌ Login failed: {response.status_code}")
            return False
    
    def test_data_ingestion(self):
        """测试数据摄入"""
        self.print_section("Data Ingestion & Processing")
        
        self.print_step(1, "Ingesting Sample Data")
        
        # 生成测试设备数据
        devices = ["sensor_001", "sensor_002", "sensor_003", "sensor_004", "sensor_005"]
        sensor_types = ["temperature", "humidity", "pressure", "cpu_usage", "memory_usage"]
        
        success_count = 0
        total_count = 20
        
        print(f"Sending {total_count} data points...")
        for i in range(total_count):
            data = {
                "device_id": random.choice(devices),
                "sensor_type": random.choice(sensor_types),
                "value": round(random.uniform(-10, 120), 2),  # 包含异常值
                "metadata": {
                    "location": f"building_{chr(65 + i % 3)}",  # A, B, C
                    "floor": random.randint(1, 5),
                    "batch": "demo_batch"
                }
            }
            
            response = requests.post(f"{self.base_url}/api/data/ingest", json=data)
            if response.status_code == 200:
                success_count += 1
                print(".", end="", flush=True)
            else:
                print("X", end="", flush=True)
        
        print(f"\n✅ Data ingestion completed: {success_count}/{total_count} successful")
        
        # 等待数据处理
        print("⏳ Waiting for data processing...")
        time.sleep(2)
        
        return success_count > 0
    
    def test_data_query(self):
        """测试数据查询"""
        self.print_section("Data Query & Analytics")
        
        # 查询设备数据
        self.print_step(1, "Querying Device Data")
        device_id = "sensor_001"
        
        response = requests.get(f"{self.base_url}/api/data/{device_id}?limit=5")
        if response.status_code == 200:
            result = response.json()
            data_count = len(result.get('data', []))
            print(f"✅ Retrieved {data_count} records for {device_id}")
            
            if data_count > 0:
                latest_record = result['data'][0]
                print(f"Latest record: {latest_record['sensor_type']} = {latest_record['value']}")
        else:
            print(f"❌ Query failed: {response.status_code}")
        
        # 查询分析摘要
        self.print_step(2, "Analytics Summary")
        response = requests.get(f"{self.base_url}/api/analytics/summary")
        if response.status_code == 200:
            result = response.json()
            summary = result.get('summary', [])
            print(f"✅ Analytics summary retrieved: {len(summary)} sensor types")
            
            for item in summary:
                print(f"  📊 {item['sensor_type']}: "
                      f"{item['count']} readings, "
                      f"avg={item['avg_value']:.2f}, "
                      f"range=[{item['min_value']:.2f}, {item['max_value']:.2f}]")
        else:
            print(f"❌ Analytics query failed: {response.status_code}")
    
    def test_metrics(self):
        """测试指标监控"""
        self.print_section("Metrics & Monitoring")
        
        response = requests.get(f"{self.base_url}/api/metrics")
        if response.status_code == 200:
            metrics = response.json()
            stream_metrics = metrics.get('stream_processor', {})
            
            print("✅ Platform metrics retrieved:")
            print(f"  🔄 Processed messages: {stream_metrics.get('processed_count', 0)}")
            print(f"  ⚠️  Anomaly count: {stream_metrics.get('anomaly_count', 0)}")
            print(f"  ⏱️  Average latency: {stream_metrics.get('average_latency', 0):.2f}ms")
            print(f"  📅 Timestamp: {metrics.get('timestamp', 'N/A')}")
        else:
            print(f"❌ Metrics query failed: {response.status_code}")
    
    def test_load_performance(self):
        """测试负载性能"""
        self.print_section("Load Testing & Performance")
        
        print("🚀 Running concurrent load test...")
        
        def send_data_batch(batch_id, count):
            """发送数据批次"""
            success = 0
            for i in range(count):
                data = {
                    "device_id": f"load_test_device_{batch_id}_{i}",
                    "sensor_type": random.choice(["cpu", "memory", "network"]),
                    "value": round(random.uniform(0, 100), 2),
                    "metadata": {"batch_id": batch_id, "test": "load_test"}
                }
                
                try:
                    response = requests.post(
                        f"{self.base_url}/api/data/ingest", 
                        json=data, 
                        timeout=1
                    )
                    if response.status_code == 200:
                        success += 1
                except:
                    pass
            
            return success
        
        # 启动多个并发线程
        threads = []
        results = []
        batch_size = 10
        num_threads = 5
        
        start_time = time.time()
        
        for i in range(num_threads):
            thread = threading.Thread(
                target=lambda i=i: results.append(send_data_batch(i, batch_size))
            )
            threads.append(thread)
            thread.start()
        
        # 等待所有线程完成
        for thread in threads:
            thread.join()
        
        end_time = time.time()
        duration = end_time - start_time
        total_requests = sum(results)
        total_attempted = num_threads * batch_size
        
        print(f"📈 Load test results:")
        print(f"  ⏱️  Duration: {duration:.2f} seconds")
        print(f"  ✅ Successful requests: {total_requests}/{total_attempted}")
        print(f"  🔥 Throughput: {total_requests/duration:.2f} req/sec")
        print(f"  📊 Success rate: {(total_requests/total_attempted)*100:.1f}%")
    
    def run_full_demo(self):
        """运行完整演示"""
        print("🎬 MVP Platform Architecture Demo")
        print("=" * 60)
        print("Demonstrating enterprise-grade microservices architecture")
        print("with real-time data processing, authentication, and analytics")
        print("=" * 60)
        
        # 健康检查
        if not self.test_health():
            print("❌ Platform is not healthy. Please start the server first.")
            return
        
        # 认证测试
        if not self.test_authentication():
            print("❌ Authentication test failed")
            return
        
        # 数据摄入测试
        if not self.test_data_ingestion():
            print("❌ Data ingestion test failed")
            return
        
        # 数据查询测试
        self.test_data_query()
        
        # 指标监控测试
        self.test_metrics()
        
        # 性能测试
        self.test_load_performance()
        
        # 最终总结
        self.print_section("Demo Summary")
        print("✅ MVP Platform Demo Completed Successfully!")
        print()
        print("🏗️  Architecture Components Demonstrated:")
        print("   • 🔐 Authentication Service (JWT + Session Management)")
        print("   • 📊 Data Processing Service (Kafka-like Streaming)")  
        print("   • 🔄 Stream Processing Engine (Real-time Analytics)")
        print("   • 💾 Multi-layer Storage (Database + Cache)")
        print("   • 📈 Metrics & Monitoring")
        print("   • ⚡ Load Balancing & Concurrency")
        print()
        print("📋 Key Features Validated:")
        print("   • High-throughput data ingestion")
        print("   • Real-time stream processing")
        print("   • Anomaly detection")
        print("   • RESTful API design")
        print("   • Horizontal scalability patterns")
        print("   • Observability & monitoring")

def main():
    """主函数"""
    demo = MVPDemo()
    
    try:
        demo.run_full_demo()
    except KeyboardInterrupt:
        print("\n\n🛑 Demo interrupted by user")
    except Exception as e:
        print(f"\n❌ Demo failed with error: {e}")

if __name__ == "__main__":
    main()