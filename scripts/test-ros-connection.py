#!/usr/bin/env python3
"""
测试ROS2连接和基础话题发布
"""

import sys
import os
sys.path.insert(0, '/opt/ros/jazzy/lib/python3.12/site-packages')

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import json
import requests
import time
from datetime import datetime

class TestRobotPublisher(Node):
    def __init__(self):
        super().__init__('test_robot_publisher')
        
        # 发布器
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/robot_status', 10)
        
        # 定时器
        self.timer = self.create_timer(2.0, self.publish_test_data)
        self.move_timer = self.create_timer(5.0, self.send_test_movement)
        
        self.counter = 0
        self.cloud_url = "http://127.0.0.1:8000"
        
        self.get_logger().info('Test Robot Publisher started!')

    def publish_test_data(self):
        """发布测试数据并发送到云平台"""
        
        # 发布状态消息
        status_msg = String()
        status_data = {
            "robot_id": "test_robot_001",
            "status": "active",
            "battery": 80.5 - (self.counter * 0.1),
            "position": {"x": self.counter * 0.1, "y": 0.0, "z": 0.0},
            "timestamp": datetime.now().isoformat()
        }
        status_msg.data = json.dumps(status_data)
        self.status_pub.publish(status_msg)
        
        # 发送到云平台
        try:
            telemetry_payload = {
                "timestamp": datetime.now().isoformat(),
                "data": {
                    "battery": 80.5 - (self.counter * 0.1),
                    "position": {"x": self.counter * 0.1, "y": 0.0, "z": 0.0},
                    "velocity": {"linear": 0.2, "angular": 0.0},
                    "status": "active",
                    "temperature": 25.0 + (self.counter % 5),
                    "test_counter": self.counter
                },
                "metadata": {
                    "source": "ros2_test_node",
                    "version": "1.0.0"
                }
            }
            
            url = f"{self.cloud_url}/api/v1/robots/test_robot_001/telemetry"
            response = requests.post(url, json=telemetry_payload, timeout=2.0)
            
            if response.status_code == 200:
                self.get_logger().info(f'✅ 数据发送成功 (计数: {self.counter})')
            else:
                self.get_logger().warn(f'❌ 发送失败: {response.status_code}')
                
        except Exception as e:
            self.get_logger().error(f'❌ 网络错误: {e}')
        
        self.counter += 1

    def send_test_movement(self):
        """发送测试移动命令"""
        twist = Twist()
        
        # 循环移动模式
        if (self.counter // 5) % 4 == 0:
            twist.linear.x = 0.2  # 前进
        elif (self.counter // 5) % 4 == 1:
            twist.angular.z = 0.5  # 左转
        elif (self.counter // 5) % 4 == 2:
            twist.linear.x = -0.1  # 后退
        else:
            twist.angular.z = -0.5  # 右转
            
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info(f'🎮 发送移动命令: linear={twist.linear.x:.1f}, angular={twist.angular.z:.1f}')

def main():
    print("🧪 启动ROS2测试节点...")
    print("📡 这将向云平台发送模拟的机器人数据")
    print("🔗 确保云平台API在 http://127.0.0.1:8000 运行")
    
    rclpy.init()
    
    try:
        node = TestRobotPublisher()
        print("✅ 测试节点已启动，开始发送数据...")
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n⏹️ 停止测试节点...")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()