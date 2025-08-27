#!/usr/bin/env python3
"""
MVP平台API测试脚本
"""

import requests
import json
import time
import random
from datetime import datetime

# 配置
AUTH_SERVICE_URL = "http://localhost:8081"
DATA_SERVICE_URL = "http://localhost:8082"
KONG_GATEWAY_URL = "http://localhost:8000"

def test_auth_service():
    """测试认证服务"""
    print("🔐 Testing Authentication Service...")
    
    # 测试健康检查
    response = requests.get(f"{AUTH_SERVICE_URL}/health")
    print(f"Health check: {response.status_code}")
    
    # 测试用户注册
    user_data = {
        "username": f"testuser_{int(time.time())}",
        "password": "testpass123",
        "email": "test@example.com"
    }
    
    response = requests.post(f"{AUTH_SERVICE_URL}/register", json=user_data)
    print(f"User registration: {response.status_code}")
    
    if response.status_code == 201:
        # 测试登录
        login_data = {
            "username": user_data["username"],
            "password": user_data["password"]
        }
        
        response = requests.post(f"{AUTH_SERVICE_URL}/login", json=login_data)
        print(f"User login: {response.status_code}")
        
        if response.status_code == 200:
            token = response.json().get('token')
            print(f"JWT Token received: {token[:50]}...")
            
            # 测试token验证
            headers = {"Authorization": f"Bearer {token}"}
            response = requests.post(f"{AUTH_SERVICE_URL}/verify", headers=headers)
            print(f"Token verification: {response.status_code}")
            
            return token
    
    return None

def test_data_service(token=None):
    """测试数据服务"""
    print("\n📊 Testing Data Service...")
    
    # 测试健康检查
    response = requests.get(f"{DATA_SERVICE_URL}/health")
    print(f"Health check: {response.status_code}")
    
    # 生成测试数据
    devices = ["sensor_001", "sensor_002", "sensor_003"]
    sensor_types = ["temperature", "humidity", "pressure"]
    
    print("Ingesting test data...")
    for _ in range(10):
        test_data = {
            "device_id": random.choice(devices),
            "sensor_type": random.choice(sensor_types),
            "value": round(random.uniform(20.0, 100.0), 2),
            "metadata": {
                "location": "building_a",
                "floor": random.randint(1, 5)
            }
        }
        
        response = requests.post(f"{DATA_SERVICE_URL}/ingest", json=test_data)
        if response.status_code == 201:
            print(".", end="", flush=True)
        else:
            print(f"X({response.status_code})", end="", flush=True)
    
    print(f"\nData ingestion completed")
    
    # 等待数据处理
    time.sleep(2)
    
    # 测试数据查询
    device_id = devices[0]
    response = requests.get(f"{DATA_SERVICE_URL}/data/{device_id}")
    if response.status_code == 200:
        data = response.json()
        print(f"Query device data: {len(data.get('data', []))} records found")
    
    # 测试分析摘要
    response = requests.get(f"{DATA_SERVICE_URL}/analytics/summary")
    if response.status_code == 200:
        summary = response.json()
        print(f"Analytics summary: {len(summary.get('summary', []))} sensor types")

def test_gateway():
    """测试API网关"""
    print("\n🌐 Testing API Gateway...")
    
    # 测试网关健康检查
    try:
        response = requests.get(f"{KONG_GATEWAY_URL}/health", timeout=5)
        print(f"Gateway health: {response.status_code}")
    except requests.exceptions.RequestException as e:
        print(f"Gateway connection failed: {e}")

def generate_load_test():
    """生成负载测试数据"""
    print("\n⚡ Running Load Test...")
    
    devices = [f"load_test_device_{i}" for i in range(1, 21)]
    sensor_types = ["cpu", "memory", "disk", "network"]
    
    start_time = time.time()
    success_count = 0
    error_count = 0
    
    for i in range(100):
        test_data = {
            "device_id": random.choice(devices),
            "sensor_type": random.choice(sensor_types),
            "value": round(random.uniform(0, 100), 2),
            "metadata": {
                "test_batch": "load_test_1",
                "sequence": i
            }
        }
        
        try:
            response = requests.post(
                f"{DATA_SERVICE_URL}/ingest", 
                json=test_data,
                timeout=5
            )
            if response.status_code == 201:
                success_count += 1
            else:
                error_count += 1
        except requests.exceptions.RequestException:
            error_count += 1
        
        if i % 10 == 0:
            print(f"Progress: {i}/100")
    
    end_time = time.time()
    duration = end_time - start_time
    
    print(f"\n📈 Load Test Results:")
    print(f"Duration: {duration:.2f} seconds")
    print(f"Success: {success_count}")
    print(f"Errors: {error_count}")
    print(f"Throughput: {success_count/duration:.2f} requests/sec")

def main():
    """主函数"""
    print("🚀 Starting MVP Platform API Tests")
    print("=" * 50)
    
    try:
        # 测试认证服务
        token = test_auth_service()
        
        # 测试数据服务
        test_data_service(token)
        
        # 测试网关
        test_gateway()
        
        # 负载测试
        generate_load_test()
        
        print("\n✅ All tests completed!")
        
    except Exception as e:
        print(f"\n❌ Test failed: {e}")

if __name__ == "__main__":
    main()