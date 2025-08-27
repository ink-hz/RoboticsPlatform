#!/usr/bin/env python3
"""
Gazebo-Cloud Platform Bridge
连接ROS2/Gazebo仿真环境与机器人云平台
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# ROS2消息类型
from sensor_msgs.msg import BatteryState, LaserScan, Image, PointCloud2, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String, Float32
from diagnostic_msgs.msg import DiagnosticArray

import requests
import json
import threading
import time
from datetime import datetime
from typing import Dict, Any
import math

class GazeboCloudBridge(Node):
    """Gazebo到云平台的数据桥接服务"""
    
    def __init__(self):
        super().__init__('gazebo_cloud_bridge')
        
        # 云平台配置
        self.cloud_api_url = "http://127.0.0.1:8000"
        self.robot_id = "turtlebot3_gazebo_001"
        
        # 数据缓存
        self.robot_data = {
            "battery": 100.0,
            "position": {"x": 0.0, "y": 0.0, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            "velocity": {"linear": 0.0, "angular": 0.0},
            "laser_ranges": [],
            "imu": {"linear_accel": {"x": 0, "y": 0, "z": 0}, "angular_vel": {"x": 0, "y": 0, "z": 0}},
            "status": "active",
            "temperature": 25.0,
            "last_update": datetime.now().isoformat()
        }
        
        # QoS配置 - 适应不同的网络情况
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # 订阅关键话题
        self.create_subscriptions(qos_profile)
        
        # 发布器用于接收云端命令
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # 定时发送数据到云平台
        self.timer = self.create_timer(2.0, self.send_telemetry_to_cloud)
        
        # 定时检查云端命令
        self.command_timer = self.create_timer(1.0, self.check_cloud_commands)
        
        self.get_logger().info(f'Gazebo Cloud Bridge started for robot: {self.robot_id}')
        self.get_logger().info(f'Cloud API: {self.cloud_api_url}')
    
    def create_subscriptions(self, qos_profile):
        """创建所有必要的话题订阅"""
        
        # 里程计 - 机器人位置和速度
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, qos_profile)
        
        # 激光雷达数据
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, qos_profile)
        
        # IMU数据
        self.imu_sub = self.create_subscription(
            Imu, '/imu', self.imu_callback, qos_profile)
        
        # 电池状态（如果有的话）
        self.battery_sub = self.create_subscription(
            BatteryState, '/battery_state', self.battery_callback, qos_profile)
        
        # 系统诊断
        self.diag_sub = self.create_subscription(
            DiagnosticArray, '/diagnostics', self.diagnostics_callback, qos_profile)
        
        # 关节状态和其他传感器可根据需要添加
    
    def odom_callback(self, msg):
        """处理里程计数据"""
        self.robot_data["position"] = {
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y,
            "z": msg.pose.pose.position.z
        }
        
        self.robot_data["orientation"] = {
            "x": msg.pose.pose.orientation.x,
            "y": msg.pose.pose.orientation.y,
            "z": msg.pose.pose.orientation.z,
            "w": msg.pose.pose.orientation.w
        }
        
        # 计算线速度和角速度
        linear_vel = math.sqrt(
            msg.twist.twist.linear.x**2 + 
            msg.twist.twist.linear.y**2
        )
        
        self.robot_data["velocity"] = {
            "linear": linear_vel,
            "angular": msg.twist.twist.angular.z
        }
        
        self.robot_data["last_update"] = datetime.now().isoformat()
    
    def laser_callback(self, msg):
        """处理激光雷达数据"""
        # 只保存一些关键点，避免数据过大
        ranges = list(msg.ranges)
        # 每10度采样一个点
        sample_step = len(ranges) // 36 if len(ranges) > 36 else 1
        sampled_ranges = ranges[::sample_step]
        
        self.robot_data["laser_ranges"] = sampled_ranges
        self.robot_data["laser_info"] = {
            "angle_min": msg.angle_min,
            "angle_max": msg.angle_max,
            "angle_increment": msg.angle_increment * sample_step,
            "range_min": msg.range_min,
            "range_max": msg.range_max,
            "total_points": len(ranges),
            "sampled_points": len(sampled_ranges)
        }
    
    def imu_callback(self, msg):
        """处理IMU数据"""
        self.robot_data["imu"] = {
            "linear_accel": {
                "x": msg.linear_acceleration.x,
                "y": msg.linear_acceleration.y,
                "z": msg.linear_acceleration.z
            },
            "angular_vel": {
                "x": msg.angular_velocity.x,
                "y": msg.angular_velocity.y,
                "z": msg.angular_velocity.z
            },
            "orientation": {
                "x": msg.orientation.x,
                "y": msg.orientation.y,
                "z": msg.orientation.z,
                "w": msg.orientation.w
            }
        }
    
    def battery_callback(self, msg):
        """处理电池状态"""
        if msg.percentage > 0:
            self.robot_data["battery"] = msg.percentage * 100
        else:
            # 模拟电池消耗
            self.robot_data["battery"] = max(0, self.robot_data["battery"] - 0.01)
    
    def diagnostics_callback(self, msg):
        """处理系统诊断信息"""
        diagnostic_info = {}
        for status in msg.status:
            diagnostic_info[status.name] = {
                "level": status.level,
                "message": status.message,
                "values": {kv.key: kv.value for kv in status.values}
            }
        
        self.robot_data["diagnostics"] = diagnostic_info
        
        # 基于诊断信息更新机器人状态
        if any(status.level > 1 for status in msg.status):
            self.robot_data["status"] = "warning"
        elif any(status.level > 0 for status in msg.status):
            self.robot_data["status"] = "degraded"
        else:
            self.robot_data["status"] = "active"
    
    def send_telemetry_to_cloud(self):
        """发送遥测数据到云平台"""
        try:
            # 准备发送的数据
            telemetry_payload = {
                "timestamp": datetime.now().isoformat(),
                "data": self.robot_data.copy(),
                "metadata": {
                    "source": "gazebo_simulation",
                    "robot_type": "turtlebot3",
                    "bridge_version": "1.0.0"
                }
            }
            
            # 发送到云平台
            url = f"{self.cloud_api_url}/api/v1/robots/{self.robot_id}/telemetry"
            response = requests.post(url, json=telemetry_payload, timeout=5.0)
            
            if response.status_code == 200:
                self.get_logger().debug(f'Telemetry sent successfully')
            else:
                self.get_logger().warn(f'Failed to send telemetry: {response.status_code}')
                
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f'Network error sending telemetry: {e}')
        except Exception as e:
            self.get_logger().error(f'Error sending telemetry: {e}')
    
    def check_cloud_commands(self):
        """检查来自云平台的命令"""
        try:
            # 这里可以轮询云平台的命令接口
            # 暂时跳过，避免频繁请求
            pass
        except Exception as e:
            self.get_logger().error(f'Error checking commands: {e}')
    
    def execute_cloud_command(self, command: Dict[str, Any]):
        """执行来自云平台的命令"""
        cmd_type = command.get("type")
        
        if cmd_type == "move":
            # 移动命令
            twist = Twist()
            twist.linear.x = float(command.get("linear", 0.0))
            twist.angular.z = float(command.get("angular", 0.0))
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info(f'Executing move command: {command}')
            
        elif cmd_type == "goto":
            # 导航命令
            goal = PoseStamped()
            goal.header.frame_id = "map"
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x = float(command.get("x", 0.0))
            goal.pose.position.y = float(command.get("y", 0.0))
            goal.pose.orientation.w = 1.0
            self.goal_pub.publish(goal)
            self.get_logger().info(f'Executing goto command: {command}')
            
        elif cmd_type == "stop":
            # 停止命令
            twist = Twist()  # 全零速度
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info('Executing stop command')
            
        else:
            self.get_logger().warn(f'Unknown command type: {cmd_type}')

def main():
    """主函数"""
    rclpy.init()
    
    try:
        bridge = GazeboCloudBridge()
        
        print("🔗 Gazebo Cloud Bridge 已启动!")
        print(f"📡 机器人ID: {bridge.robot_id}")
        print(f"🌐 云平台地址: {bridge.cloud_api_url}")
        print("📊 开始发送遥测数据...")
        print("⭐ 按Ctrl+C停止")
        
        rclpy.spin(bridge)
        
    except KeyboardInterrupt:
        print("\n⏹️ 正在关闭Gazebo Cloud Bridge...")
    except Exception as e:
        print(f"❌ 错误: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()