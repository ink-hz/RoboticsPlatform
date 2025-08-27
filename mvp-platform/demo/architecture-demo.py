#!/usr/bin/env python3
"""
MVP平台动态架构图演示脚本
"""

import urllib.request
import json
import time
import random

def send_request(method, url, data=None):
    """发送HTTP请求"""
    try:
        if data:
            data_bytes = json.dumps(data).encode('utf-8')
            req = urllib.request.Request(url, data=data_bytes, method=method)
            req.add_header('Content-Type', 'application/json')
        else:
            req = urllib.request.Request(url, method=method)
        
        with urllib.request.urlopen(req, timeout=5) as response:
            content = response.read().decode('utf-8')
            return response.status, json.loads(content) if content else {}
    
    except Exception as e:
        return None, str(e)

def get_architecture_status():
    """获取架构状态"""
    status, response = send_request('GET', 'http://localhost:8080/api/architecture')
    if status == 200:
        return response
    return None

def demonstrate_architecture_changes():
    """演示架构变化"""
    print("🏗️ MVP Platform Dynamic Architecture Diagram Demo")
    print("=" * 60)
    print("This demo shows how the architecture diagram updates")
    print("in real-time based on actual system activity.")
    print("=" * 60)
    
    # 1. 显示初始状态
    print("\n📊 1. Initial Architecture State")
    print("-" * 40)
    initial_state = get_architecture_status()
    if initial_state:
        components = initial_state['components']
        print(f"Total components: {len(components)}")
        print(f"Active connections: {initial_state['active_connections']}")
        print(f"Data throughput: {initial_state['data_throughput']}")
        print(f"Anomaly rate: {initial_state['anomaly_rate']}")
    
    # 2. 用户注册活动
    print("\n👥 2. Simulating User Registration Activity")
    print("-" * 40)
    print("Registering 3 test users...")
    
    for i in range(3):
        user_data = {
            "username": f"demo_user_{i}_{int(time.time())}",
            "password": "demo123",
            "email": f"demo{i}@example.com"
        }
        
        status, response = send_request('POST', 'http://localhost:8080/api/auth/register', user_data)
        if status == 200:
            print(f"  ✅ User {i+1} registered")
        else:
            print(f"  ❌ User {i+1} failed: {response}")
        
        time.sleep(0.5)
    
    # 检查架构更新
    time.sleep(2)
    print("\nArchitecture after user registration:")
    updated_state = get_architecture_status()
    if updated_state:
        auth_service = updated_state['components'].get('auth-service', {})
        print(f"  Auth Service: {auth_service.get('status')} (load: {auth_service.get('load')})")
        database = updated_state['components'].get('database', {})
        print(f"  Database: {database.get('status')} (load: {database.get('load')})")
    
    # 3. 数据摄入活动
    print("\n📊 3. Simulating Data Ingestion Activity")
    print("-" * 40)
    print("Ingesting sensor data from multiple devices...")
    
    devices = ["sensor_001", "sensor_002", "sensor_003"]
    sensor_types = ["temperature", "humidity", "pressure", "cpu"]
    
    for i in range(10):
        sensor_data = {
            "device_id": random.choice(devices),
            "sensor_type": random.choice(sensor_types),
            "value": round(random.uniform(20, 80), 2),
            "metadata": {
                "demo": "architecture_test",
                "iteration": i
            }
        }
        
        status, response = send_request('POST', 'http://localhost:8080/api/data/ingest', sensor_data)
        if status == 200:
            print(".", end="", flush=True)
        else:
            print("X", end="", flush=True)
        
        time.sleep(0.3)
    
    print("\n\nArchitecture after data ingestion:")
    updated_state = get_architecture_status()
    if updated_state:
        data_service = updated_state['components'].get('data-service', {})
        print(f"  Data Service: {data_service.get('status')} (load: {data_service.get('load')})")
        stream_processor = updated_state['components'].get('stream-processor', {})
        print(f"  Stream Processor: {stream_processor.get('status')} (load: {stream_processor.get('load')})")
        print(f"  Total data throughput: {updated_state['data_throughput']}")
    
    # 4. 异常检测演示
    print("\n⚠️  4. Triggering Anomaly Detection")
    print("-" * 40)
    print("Sending anomalous data to trigger alerts...")
    
    anomaly_values = [1500, -50, 2000, -100]  # 异常值
    for i, value in enumerate(anomaly_values):
        anomaly_data = {
            "device_id": f"anomaly_device_{i}",
            "sensor_type": "temperature",
            "value": value,
            "metadata": {"test": "anomaly_demo"}
        }
        
        status, response = send_request('POST', 'http://localhost:8080/api/data/ingest', anomaly_data)
        if status == 200:
            print(f"  🚨 Anomaly {i+1} sent: {value}")
        
        time.sleep(0.5)
    
    # 检查异常对架构的影响
    time.sleep(2)
    print("\nArchitecture after anomaly detection:")
    final_state = get_architecture_status()
    if final_state:
        anomaly_detector = final_state['components'].get('anomaly-detector', {})
        print(f"  Anomaly Detector: {anomaly_detector.get('status')} (load: {anomaly_detector.get('load')})")
        notification_service = final_state['components'].get('notification-service', {})
        print(f"  Notification Service: {notification_service.get('status')} (load: {notification_service.get('load')})")
        print(f"  Total anomaly rate: {final_state['anomaly_rate']}")
    
    # 5. 总结
    print("\n🎯 5. Architecture Diagram Summary")
    print("-" * 40)
    if final_state:
        healthy_count = sum(1 for comp in final_state['components'].values() if comp['status'] == 'healthy')
        warning_count = sum(1 for comp in final_state['components'].values() if comp['status'] == 'warning')
        error_count = sum(1 for comp in final_state['components'].values() if comp['status'] == 'error')
        
        print(f"Component Health Status:")
        print(f"  🟢 Healthy: {healthy_count}")
        print(f"  🟡 Warning: {warning_count}")
        print(f"  🔴 Error: {error_count}")
        print(f"Overall System Health: {final_state['overall_health']}")
    
    print("\n" + "=" * 60)
    print("✅ Dynamic Architecture Demo Completed!")
    print("=" * 60)
    print("🌐 Visit http://localhost:8080/ to see the live architecture diagram")
    print("📊 The diagram updates every 3 seconds with real system data")
    print("🎯 Click on components to see detailed information")
    print("💡 Component colors indicate health: Green=Healthy, Yellow=Warning, Red=Error")
    print("📈 High-load components pulse to show activity")

def main():
    """主函数"""
    try:
        # 检查平台是否运行
        status, response = send_request('GET', 'http://localhost:8080/health')
        if status != 200:
            print("❌ MVP Platform is not running!")
            print("Please start it with: python3 demo/simple-mvp-demo.py")
            return
        
        print("✅ MVP Platform is online")
        print(f"Platform: {response.get('platform', 'Unknown')}")
        
        # 运行演示
        demonstrate_architecture_changes()
        
    except KeyboardInterrupt:
        print("\n🛑 Demo interrupted by user")
    except Exception as e:
        print(f"\n❌ Demo failed: {e}")

if __name__ == "__main__":
    main()