#!/usr/bin/env python3
"""
向机器人云平台添加演示数据的脚本
"""
import requests
import json
import time
from datetime import datetime

BASE_URL = "http://127.0.0.1:8000"

def send_robot_telemetry(robot_id, data):
    """发送机器人遥测数据"""
    url = f"{BASE_URL}/api/v1/robots/{robot_id}/telemetry"
    response = requests.post(url, json=data)
    return response.status_code == 200

def main():
    print("🤖 添加机器人云平台演示数据...")
    
    # 模拟机器人数据
    robots = [
        {
            "id": "robot-warehouse-001",
            "battery": 85,
            "location": "仓库A区",
            "status": "active"
        },
        {
            "id": "robot-warehouse-002", 
            "battery": 67,
            "location": "仓库B区",
            "status": "active"
        },
        {
            "id": "robot-transport-001",
            "battery": 23,
            "location": "充电站",
            "status": "charging"
        }
    ]
    
    # 发送遥测数据
    for i, robot in enumerate(robots):
        print(f"📡 发送 {robot['id']} 的遥测数据...")
        
        telemetry_data = {
            "timestamp": datetime.utcnow().isoformat(),
            "data": {
                "battery": robot["battery"],
                "position": {
                    "x": 10.5 + i * 5,
                    "y": 20.3 + i * 3,
                    "z": 0.0
                },
                "velocity": {
                    "linear": 0.5 if robot["status"] == "active" else 0.0,
                    "angular": 0.1 if robot["status"] == "active" else 0.0
                },
                "status": robot["status"],
                "location": robot["location"],
                "sensors": {
                    "lidar": "active" if robot["status"] == "active" else "standby",
                    "camera": "active" if robot["status"] == "active" else "standby",
                    "imu": "normal"
                },
                "temperature": 24.5 + i,
                "uptime": 3600 * (24 - i * 2)  # 不同的运行时间
            },
            "metadata": {
                "source": "demo_script",
                "version": "1.0"
            }
        }
        
        success = send_robot_telemetry(robot["id"], telemetry_data)
        if success:
            print(f"✅ {robot['id']} 数据发送成功")
        else:
            print(f"❌ {robot['id']} 数据发送失败")
        
        time.sleep(0.5)  # 避免请求过快
    
    print("\n🎯 发送一些历史遥测数据...")
    
    # 模拟一些历史数据
    import random
    for _ in range(5):
        robot_id = random.choice([r["id"] for r in robots])
        historical_data = {
            "timestamp": datetime.utcnow().isoformat(),
            "data": {
                "battery": random.randint(20, 100),
                "temperature": random.uniform(20, 30),
                "cpu_usage": random.uniform(10, 80),
                "memory_usage": random.uniform(30, 70),
                "task_completed": random.choice(["搬运任务A", "巡检任务B", "充电任务"])
            }
        }
        
        send_robot_telemetry(robot_id, historical_data)
        print(f"📊 发送 {robot_id} 历史数据")
        time.sleep(0.3)
    
    print(f"\n🎉 演示数据添加完成！")
    print(f"👉 访问控制台: {BASE_URL}")
    print(f"📊 查看统计数据: {BASE_URL}/api/v1/dashboard/stats")
    print(f"🤖 查看机器人列表: {BASE_URL}/api/v1/robots")
    print(f"📡 查看遥测数据: {BASE_URL}/api/v1/telemetry/latest")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n⏹️ 脚本被用户中断")
    except Exception as e:
        print(f"\n❌ 脚本执行出错: {e}")