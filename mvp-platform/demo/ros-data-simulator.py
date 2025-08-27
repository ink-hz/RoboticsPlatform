#!/usr/bin/env python3
"""
ROS机器人数据流模拟器
模拟ROS机器人向MVP平台发送传感器数据
"""

import json
import time
import math
import random
import threading
from datetime import datetime
import urllib.request

class ROSDataSimulator:
    """模拟ROS机器人数据生成器"""
    
    def __init__(self, robot_id="robot_001"):
        self.robot_id = robot_id
        self.position = {"x": 0.0, "y": 0.0, "theta": 0.0}
        self.velocity = {"linear": 0.5, "angular": 0.1}
        self.battery = 100.0
        self.start_time = time.time()
        self.mission_status = "idle"
        self.obstacle_detected = False
        
    def generate_lidar_scan(self):
        """生成激光雷达扫描数据 (360度扫描)"""
        ranges = []
        for angle in range(0, 360, 1):
            # 模拟距离数据，加入一些噪声
            base_distance = 5.0
            if 45 <= angle <= 90:  # 模拟右前方障碍物
                base_distance = 0.5 + random.uniform(-0.1, 0.1)
                self.obstacle_detected = True
            else:
                base_distance = 5.0 + random.uniform(-0.5, 0.5)
                
            ranges.append(base_distance)
            
        return {
            "msg_type": "sensor_msgs/LaserScan",
            "header": {
                "stamp": time.time(),
                "frame_id": "laser_frame"
            },
            "angle_min": 0.0,
            "angle_max": 2 * math.pi,
            "angle_increment": math.pi / 180,
            "ranges": ranges,
            "range_min": 0.1,
            "range_max": 10.0
        }
    
    def generate_odometry(self):
        """生成里程计数据"""
        # 更新位置（简单的运动模型）
        dt = 0.1  # 100ms更新周期
        self.position["x"] += self.velocity["linear"] * math.cos(self.position["theta"]) * dt
        self.position["y"] += self.velocity["linear"] * math.sin(self.position["theta"]) * dt
        self.position["theta"] += self.velocity["angular"] * dt
        
        return {
            "msg_type": "nav_msgs/Odometry",
            "header": {
                "stamp": time.time(),
                "frame_id": "odom"
            },
            "pose": {
                "position": {
                    "x": self.position["x"],
                    "y": self.position["y"],
                    "z": 0.0
                },
                "orientation": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": math.sin(self.position["theta"] / 2),
                    "w": math.cos(self.position["theta"] / 2)
                }
            },
            "twist": {
                "linear": {"x": self.velocity["linear"], "y": 0.0, "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": self.velocity["angular"]}
            }
        }
    
    def generate_imu_data(self):
        """生成IMU传感器数据"""
        return {
            "msg_type": "sensor_msgs/Imu",
            "header": {
                "stamp": time.time(),
                "frame_id": "imu_frame"
            },
            "orientation": {
                "x": 0.0,
                "y": 0.0,
                "z": math.sin(self.position["theta"] / 2),
                "w": math.cos(self.position["theta"] / 2)
            },
            "angular_velocity": {
                "x": random.uniform(-0.01, 0.01),
                "y": random.uniform(-0.01, 0.01),
                "z": self.velocity["angular"] + random.uniform(-0.01, 0.01)
            },
            "linear_acceleration": {
                "x": random.uniform(-0.1, 0.1),
                "y": random.uniform(-0.1, 0.1),
                "z": 9.81 + random.uniform(-0.05, 0.05)
            }
        }
    
    def generate_battery_status(self):
        """生成电池状态"""
        # 模拟电池消耗
        self.battery -= 0.01
        if self.battery < 0:
            self.battery = 0
            
        return {
            "msg_type": "sensor_msgs/BatteryState",
            "header": {
                "stamp": time.time(),
                "frame_id": "battery"
            },
            "voltage": 24.0 + random.uniform(-0.5, 0.5),
            "current": -2.5 + random.uniform(-0.5, 0.5),
            "percentage": self.battery / 100.0,
            "temperature": 35.0 + random.uniform(-2, 2)
        }
    
    def generate_diagnostic_status(self):
        """生成诊断信息"""
        statuses = []
        
        # 电机状态
        statuses.append({
            "name": "Motors",
            "level": 0 if not self.obstacle_detected else 1,  # OK or WARN
            "message": "Motors operating normally" if not self.obstacle_detected else "Reduced speed due to obstacle",
            "values": [
                {"key": "left_motor_temp", "value": str(45 + random.uniform(-5, 5))},
                {"key": "right_motor_temp", "value": str(43 + random.uniform(-5, 5))},
                {"key": "motor_current", "value": str(2.5 + random.uniform(-0.5, 0.5))}
            ]
        })
        
        # 传感器状态
        statuses.append({
            "name": "Sensors",
            "level": 0,  # OK
            "message": "All sensors operational",
            "values": [
                {"key": "lidar_fps", "value": "10"},
                {"key": "imu_fps", "value": "100"},
                {"key": "camera_fps", "value": "30"}
            ]
        })
        
        # 网络状态
        statuses.append({
            "name": "Network",
            "level": 0,
            "message": "Connected",
            "values": [
                {"key": "wifi_strength", "value": str(-50 + random.uniform(-10, 10))},
                {"key": "latency_ms", "value": str(20 + random.uniform(-5, 10))}
            ]
        })
        
        return {
            "msg_type": "diagnostic_msgs/DiagnosticArray",
            "header": {
                "stamp": time.time()
            },
            "status": statuses
        }
    
    def generate_joint_states(self):
        """生成关节状态（用于机械臂）"""
        joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
        positions = [math.sin(time.time() + i) * 0.5 for i in range(6)]
        velocities = [math.cos(time.time() + i) * 0.1 for i in range(6)]
        efforts = [random.uniform(-10, 10) for _ in range(6)]
        
        return {
            "msg_type": "sensor_msgs/JointState",
            "header": {
                "stamp": time.time(),
                "frame_id": ""
            },
            "name": joint_names,
            "position": positions,
            "velocity": velocities,
            "effort": efforts
        }


class ROSBridgeServer:
    """模拟ROS Bridge WebSocket服务器"""
    
    def __init__(self, simulator):
        self.simulator = simulator
        self.clients = []
        self.running = True
        
    def start_websocket_server(self):
        """启动WebSocket服务器（简化版）"""
        print("🌐 ROS Bridge WebSocket Server starting on ws://localhost:9090")
        print("   (Note: This is a simplified simulation)")
        
    def broadcast_data(self):
        """广播ROS数据到所有客户端"""
        while self.running:
            # 生成各种传感器数据
            data_packets = [
                self.simulator.generate_lidar_scan(),
                self.simulator.generate_odometry(),
                self.simulator.generate_imu_data(),
                self.simulator.generate_battery_status(),
                self.simulator.generate_diagnostic_status(),
                self.simulator.generate_joint_states()
            ]
            
            # 发送到MVP平台
            for packet in data_packets:
                self.send_to_mvp_platform(packet)
                
            time.sleep(0.1)  # 10Hz更新率
    
    def send_to_mvp_platform(self, data):
        """发送数据到MVP平台"""
        try:
            # 转换为MVP平台格式
            mvp_data = {
                "device_id": f"{self.simulator.robot_id}_{data['msg_type'].split('/')[1].lower()}",
                "sensor_type": data['msg_type'],
                "timestamp": data['header']['stamp'] if 'header' in data else time.time(),
                "value": 0,  # 默认值
                "metadata": {
                    "ros_data": data,
                    "robot_id": self.simulator.robot_id,
                    "mission_status": self.simulator.mission_status
                }
            }
            
            # 提取关键数值
            if data['msg_type'] == 'sensor_msgs/LaserScan':
                mvp_data['value'] = min(data['ranges'])  # 最近障碍物距离
                mvp_data['sensor_type'] = 'lidar_min_distance'
            elif data['msg_type'] == 'nav_msgs/Odometry':
                mvp_data['value'] = data['twist']['linear']['x']  # 线速度
                mvp_data['sensor_type'] = 'robot_velocity'
            elif data['msg_type'] == 'sensor_msgs/BatteryState':
                mvp_data['value'] = data['percentage'] * 100  # 电池百分比
                mvp_data['sensor_type'] = 'battery_level'
            elif data['msg_type'] == 'sensor_msgs/Imu':
                mvp_data['value'] = data['angular_velocity']['z']  # 角速度
                mvp_data['sensor_type'] = 'angular_velocity'
                
            # 发送到MVP平台的数据接入端点
            url = 'http://localhost:8080/api/data/ingest'
            req = urllib.request.Request(
                url,
                data=json.dumps(mvp_data).encode('utf-8'),
                headers={'Content-Type': 'application/json'},
                method='POST'
            )
            
            with urllib.request.urlopen(req, timeout=1) as response:
                if response.status == 200:
                    # 只打印重要数据
                    if data['msg_type'] in ['sensor_msgs/LaserScan', 'sensor_msgs/BatteryState']:
                        print(f"✅ Sent {data['msg_type']} -> MVP Platform")
                        
        except Exception as e:
            # 静默处理错误，避免刷屏
            pass


def print_ros_architecture():
    """打印ROS集成架构说明"""
    print("""
╔══════════════════════════════════════════════════════════════╗
║             🤖 ROS Robot Data Integration Demo               ║
╠══════════════════════════════════════════════════════════════╣
║                                                              ║
║  Architecture:                                               ║
║  ┌──────────┐    ┌──────────┐    ┌──────────┐              ║
║  │   ROS    │───▶│   ROS    │───▶│   MVP    │              ║
║  │  Robot   │    │  Bridge  │    │ Platform │              ║
║  └──────────┘    └──────────┘    └──────────┘              ║
║       │              │                 │                     ║
║       ▼              ▼                 ▼                     ║
║   [Sensors]    [WebSocket]      [Data Analysis]             ║
║                                                              ║
║  Simulated ROS Topics:                                       ║
║  • /scan              - Laser Scanner (360° lidar)          ║
║  • /odom              - Odometry (position & velocity)      ║
║  • /imu/data          - IMU (acceleration & gyro)           ║
║  • /battery_state     - Battery Status                      ║
║  • /diagnostics       - System Diagnostics                  ║
║  • /joint_states      - Joint Positions (robot arm)         ║
║                                                              ║
║  Data Flow:                                                  ║
║  1. Robot sensors generate data at 10-100 Hz                ║
║  2. ROS Bridge converts to JSON format                      ║
║  3. MVP Platform ingests and analyzes data                  ║
║  4. Real-time visualization and anomaly detection           ║
║                                                              ║
╚══════════════════════════════════════════════════════════════╝
    """)


def main():
    """主函数"""
    print_ros_architecture()
    
    print("\n🚀 Starting ROS Data Simulator...")
    
    # 创建机器人模拟器
    robot = ROSDataSimulator("robot_001")
    
    # 创建ROS Bridge服务器
    bridge = ROSBridgeServer(robot)
    bridge.start_websocket_server()
    
    # 启动数据广播线程
    broadcast_thread = threading.Thread(target=bridge.broadcast_data)
    broadcast_thread.daemon = True
    broadcast_thread.start()
    
    print("\n📡 ROS Robot Data Streaming Started!")
    print("📊 Data is being sent to MVP Platform at http://localhost:8080")
    print("\n📈 Monitor the data at: http://localhost:8080/")
    print("   Look for device IDs starting with 'robot_001_'")
    print("\n⚠️  Simulating robot navigation with obstacle detection...")
    
    try:
        # 模拟任务执行
        missions = ["idle", "patrolling", "docking", "charging", "navigating"]
        mission_idx = 0
        
        while True:
            # 切换任务状态
            if random.random() < 0.1:  # 10%概率切换任务
                mission_idx = (mission_idx + 1) % len(missions)
                robot.mission_status = missions[mission_idx]
                print(f"\n🎯 Mission Status Changed: {robot.mission_status}")
            
            # 更新机器人速度
            if robot.obstacle_detected:
                robot.velocity["linear"] = 0.1  # 减速
                robot.velocity["angular"] = 0.3  # 转向
                print("⚠️  Obstacle detected! Reducing speed...")
                robot.obstacle_detected = False
            else:
                robot.velocity["linear"] = 0.5 + random.uniform(-0.1, 0.1)
                robot.velocity["angular"] = 0.1 + random.uniform(-0.05, 0.05)
            
            # 显示状态
            print(f"\r🤖 Robot Status: "
                  f"Pos({robot.position['x']:.2f}, {robot.position['y']:.2f}) "
                  f"Battery: {robot.battery:.1f}% "
                  f"Mission: {robot.mission_status}    ", end="")
            
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\n\n🛑 Shutting down ROS Data Simulator...")
        bridge.running = False


if __name__ == "__main__":
    main()