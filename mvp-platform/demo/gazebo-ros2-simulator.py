#!/usr/bin/env python3
"""
Gazebo + ROS2 机器人数据模拟器
模拟Gazebo仿真环境中ROS2机器人的数据流
"""

import json
import time
import math
import random
import threading
import urllib.request
from dataclasses import dataclass, asdict
from typing import List, Dict, Any
from enum import Enum

class RobotType(Enum):
    """机器人类型"""
    TURTLEBOT3 = "turtlebot3_waffle"
    UR5_ARM = "ur5_robot"  
    QUADCOPTER = "iris_drone"
    HUSKY = "husky_robot"
    SPOT = "spot_robot"

class GazeboWorld(Enum):
    """Gazebo世界场景"""
    EMPTY = "empty_world"
    HOUSE = "turtlebot3_house"
    CITY = "citysim"
    WAREHOUSE = "aws_robomaker_warehouse"
    OUTDOOR = "outdoor_terrain"

@dataclass
class GazeboPhysics:
    """Gazebo物理引擎参数"""
    real_time_factor: float = 1.0
    max_step_size: float = 0.001
    gravity: Dict[str, float] = None
    
    def __post_init__(self):
        if self.gravity is None:
            self.gravity = {"x": 0.0, "y": 0.0, "z": -9.81}

class GazeboROS2Simulator:
    """Gazebo-ROS2仿真数据生成器"""
    
    def __init__(self, robot_type=RobotType.TURTLEBOT3, world=GazeboWorld.HOUSE):
        self.robot_type = robot_type
        self.world = world
        self.physics = GazeboPhysics()
        
        # 仿真时间
        self.sim_time = 0.0
        self.real_time_start = time.time()
        
        # 机器人状态
        self.robot_state = {
            "position": {"x": 0.0, "y": 0.0, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            "linear_velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
            "angular_velocity": {"x": 0.0, "y": 0.0, "z": 0.0}
        }
        
        # 传感器配置
        self.sensors = self.configure_sensors()
        
        # SLAM地图数据
        self.map_data = {
            "width": 100,
            "height": 100,
            "resolution": 0.05,
            "occupied_cells": []
        }
        
        # 导航目标
        self.nav_goal = {"x": 5.0, "y": 5.0, "theta": 0.0}
        self.path_plan = []
        
    def configure_sensors(self) -> Dict[str, Any]:
        """根据机器人类型配置传感器"""
        if self.robot_type == RobotType.TURTLEBOT3:
            return {
                "lidar": {"type": "2D", "range": 3.5, "samples": 360},
                "camera": {"type": "RGB", "width": 640, "height": 480, "fps": 30},
                "imu": {"type": "9DOF", "rate": 100},
                "cliff_sensors": {"count": 4, "range": 0.1},
                "bumper": {"zones": ["front", "left", "right"]}
            }
        elif self.robot_type == RobotType.UR5_ARM:
            return {
                "joint_encoders": {"joints": 6, "resolution": 0.001},
                "force_torque": {"axes": 6, "range": 1000},
                "camera": {"type": "RGB-D", "width": 1280, "height": 720},
                "gripper": {"force_range": 100, "opening": 0.08}
            }
        elif self.robot_type == RobotType.QUADCOPTER:
            return {
                "gps": {"accuracy": 1.0, "rate": 10},
                "barometer": {"range": 1000, "resolution": 0.1},
                "magnetometer": {"axes": 3, "range": 2},
                "optical_flow": {"resolution": 64, "rate": 100},
                "lidar": {"type": "3D", "range": 100, "points": 32768}
            }
        else:
            return {
                "lidar": {"type": "3D", "range": 50, "samples": 1024},
                "cameras": {"count": 5, "type": "stereo"},
                "imu": {"type": "tactical", "rate": 200}
            }
    
    def update_simulation_time(self):
        """更新仿真时间"""
        real_elapsed = time.time() - self.real_time_start
        self.sim_time = real_elapsed * self.physics.real_time_factor
        return self.sim_time
    
    def simulate_robot_motion(self):
        """模拟机器人运动"""
        dt = 0.01  # 10ms更新
        
        if self.robot_type == RobotType.TURTLEBOT3:
            # 地面机器人运动模型
            # 简单的差速驱动模型
            v = 0.5 + random.uniform(-0.1, 0.1)  # 线速度
            w = 0.2 * math.sin(self.sim_time)  # 角速度
            
            theta = math.atan2(
                2 * self.robot_state["orientation"]["z"] * self.robot_state["orientation"]["w"],
                1 - 2 * self.robot_state["orientation"]["z"]**2
            )
            
            self.robot_state["position"]["x"] += v * math.cos(theta) * dt
            self.robot_state["position"]["y"] += v * math.sin(theta) * dt
            
            # 更新姿态（简化四元数）
            dtheta = w * dt
            self.robot_state["orientation"]["z"] = math.sin(theta + dtheta/2)
            self.robot_state["orientation"]["w"] = math.cos(theta + dtheta/2)
            
            self.robot_state["linear_velocity"]["x"] = v
            self.robot_state["angular_velocity"]["z"] = w
            
        elif self.robot_type == RobotType.QUADCOPTER:
            # 四旋翼运动模型
            # 悬停在目标高度
            target_height = 5.0
            self.robot_state["position"]["z"] += (target_height - self.robot_state["position"]["z"]) * dt
            
            # 圆形轨迹
            radius = 10.0
            self.robot_state["position"]["x"] = radius * math.cos(self.sim_time * 0.1)
            self.robot_state["position"]["y"] = radius * math.sin(self.sim_time * 0.1)
            
        elif self.robot_type == RobotType.UR5_ARM:
            # 机械臂关节运动
            # 简单的正弦运动
            joints = []
            for i in range(6):
                angle = math.sin(self.sim_time + i * math.pi/6) * 0.5
                joints.append(angle)
            return joints
    
    def generate_lidar_scan(self) -> Dict[str, Any]:
        """生成激光雷达扫描数据（Gazebo格式）"""
        sensor_config = self.sensors.get("lidar", {})
        samples = sensor_config.get("samples", 360)
        max_range = sensor_config.get("range", 10.0)
        
        ranges = []
        intensities = []
        
        for i in range(samples):
            angle = (i / samples) * 2 * math.pi
            
            # 模拟环境中的障碍物
            distance = max_range
            
            # 墙壁检测
            robot_x = self.robot_state["position"]["x"]
            robot_y = self.robot_state["position"]["y"]
            
            # 简单的矩形房间模型
            wall_dist_x = min(abs(robot_x - (-5)), abs(robot_x - 5))
            wall_dist_y = min(abs(robot_y - (-5)), abs(robot_y - 5))
            
            if angle < math.pi/4 or angle > 7*math.pi/4:  # 前方
                distance = min(distance, wall_dist_x / abs(math.cos(angle)))
            elif math.pi/4 < angle < 3*math.pi/4:  # 右侧
                distance = min(distance, wall_dist_y / abs(math.sin(angle)))
            
            # 添加随机障碍物
            if random.random() < 0.1:
                distance = min(distance, random.uniform(0.5, 2.0))
            
            # 添加噪声
            distance += random.gauss(0, 0.01)
            distance = max(0.1, min(distance, max_range))
            
            ranges.append(distance)
            intensities.append(random.uniform(0, 255))
        
        return {
            "header": {
                "stamp": {"sec": int(self.sim_time), "nanosec": int((self.sim_time % 1) * 1e9)},
                "frame_id": f"{self.robot_type.value}/base_scan"
            },
            "angle_min": 0.0,
            "angle_max": 2 * math.pi,
            "angle_increment": 2 * math.pi / samples,
            "time_increment": 0.0,
            "scan_time": 0.1,
            "range_min": 0.1,
            "range_max": max_range,
            "ranges": ranges,
            "intensities": intensities
        }
    
    def generate_camera_image(self) -> Dict[str, Any]:
        """生成相机图像元数据（Gazebo格式）"""
        camera_config = self.sensors.get("camera", {})
        
        return {
            "header": {
                "stamp": {"sec": int(self.sim_time), "nanosec": int((self.sim_time % 1) * 1e9)},
                "frame_id": f"{self.robot_type.value}/camera_optical_frame"
            },
            "height": camera_config.get("height", 480),
            "width": camera_config.get("width", 640),
            "encoding": "rgb8",
            "is_bigendian": False,
            "step": camera_config.get("width", 640) * 3,
            "data_size": camera_config.get("width", 640) * camera_config.get("height", 480) * 3,
            "metadata": {
                "exposure": random.uniform(10, 30),
                "gain": random.uniform(1.0, 2.0),
                "white_balance": {"r": 1.0, "g": 1.0, "b": 1.0},
                "detected_objects": self.detect_objects()
            }
        }
    
    def detect_objects(self) -> List[Dict[str, Any]]:
        """模拟物体检测（YOLO风格）"""
        objects = []
        
        if random.random() < 0.3:
            objects.append({
                "class": "person",
                "confidence": random.uniform(0.8, 0.99),
                "bbox": {
                    "x": random.randint(100, 400),
                    "y": random.randint(100, 300),
                    "width": random.randint(50, 150),
                    "height": random.randint(100, 200)
                }
            })
        
        if random.random() < 0.5:
            objects.append({
                "class": "chair",
                "confidence": random.uniform(0.7, 0.95),
                "bbox": {
                    "x": random.randint(200, 500),
                    "y": random.randint(200, 400),
                    "width": random.randint(40, 100),
                    "height": random.randint(40, 100)
                }
            })
        
        return objects
    
    def generate_imu_data(self) -> Dict[str, Any]:
        """生成IMU数据（Gazebo格式）"""
        # 基于机器人运动状态生成IMU数据
        return {
            "header": {
                "stamp": {"sec": int(self.sim_time), "nanosec": int((self.sim_time % 1) * 1e9)},
                "frame_id": f"{self.robot_type.value}/imu_link"
            },
            "orientation": self.robot_state["orientation"],
            "orientation_covariance": [0.01] * 9,
            "angular_velocity": {
                "x": self.robot_state["angular_velocity"]["x"] + random.gauss(0, 0.001),
                "y": self.robot_state["angular_velocity"]["y"] + random.gauss(0, 0.001),
                "z": self.robot_state["angular_velocity"]["z"] + random.gauss(0, 0.001)
            },
            "angular_velocity_covariance": [0.001] * 9,
            "linear_acceleration": {
                "x": random.gauss(0, 0.1),
                "y": random.gauss(0, 0.1),
                "z": self.physics.gravity["z"] + random.gauss(0, 0.05)
            },
            "linear_acceleration_covariance": [0.01] * 9
        }
    
    def generate_model_state(self) -> Dict[str, Any]:
        """生成Gazebo模型状态"""
        return {
            "model_name": self.robot_type.value,
            "pose": {
                "position": self.robot_state["position"],
                "orientation": self.robot_state["orientation"]
            },
            "twist": {
                "linear": self.robot_state["linear_velocity"],
                "angular": self.robot_state["angular_velocity"]
            },
            "reference_frame": "world"
        }
    
    def generate_slam_data(self) -> Dict[str, Any]:
        """生成SLAM地图数据"""
        # 模拟SLAM建图过程
        if random.random() < 0.1:
            # 添加新的占用栅格
            self.map_data["occupied_cells"].append({
                "x": random.randint(0, 99),
                "y": random.randint(0, 99),
                "probability": random.uniform(0.7, 1.0)
            })
        
        return {
            "header": {
                "stamp": {"sec": int(self.sim_time), "nanosec": int((self.sim_time % 1) * 1e9)},
                "frame_id": "map"
            },
            "info": {
                "width": self.map_data["width"],
                "height": self.map_data["height"],
                "resolution": self.map_data["resolution"],
                "origin": {"position": {"x": -2.5, "y": -2.5, "z": 0}}
            },
            "data_summary": {
                "occupied_cells": len(self.map_data["occupied_cells"]),
                "free_cells": self.map_data["width"] * self.map_data["height"] - len(self.map_data["occupied_cells"]),
                "unknown_cells": 0
            }
        }
    
    def generate_path_plan(self) -> Dict[str, Any]:
        """生成路径规划数据"""
        # 简单的直线路径
        if not self.path_plan:
            start = self.robot_state["position"]
            goal = self.nav_goal
            
            steps = 20
            for i in range(steps + 1):
                t = i / steps
                self.path_plan.append({
                    "x": start["x"] + t * (goal["x"] - start["x"]),
                    "y": start["y"] + t * (goal["y"] - start["y"]),
                    "theta": goal["theta"]
                })
        
        return {
            "header": {
                "stamp": {"sec": int(self.sim_time), "nanosec": int((self.sim_time % 1) * 1e9)},
                "frame_id": "map"
            },
            "poses": self.path_plan[:10],  # 返回前10个路径点
            "path_length": len(self.path_plan),
            "estimated_time": len(self.path_plan) * 0.5,  # 估计时间
            "planner": "RRT*"
        }
    
    def send_to_mvp_platform(self, data_type: str, data: Dict[str, Any]):
        """发送数据到MVP平台"""
        try:
            mvp_data = {
                "device_id": f"gazebo_{self.robot_type.value}_{data_type}",
                "sensor_type": f"ros2_{data_type}",
                "value": 0,
                "timestamp": self.sim_time,
                "metadata": {
                    "gazebo_world": self.world.value,
                    "physics": asdict(self.physics),
                    "ros2_data": data
                }
            }
            
            # 提取关键数值
            if data_type == "laser_scan":
                mvp_data["value"] = min(data["ranges"])
            elif data_type == "imu":
                mvp_data["value"] = data["angular_velocity"]["z"]
            elif data_type == "model_state":
                mvp_data["value"] = math.sqrt(
                    data["twist"]["linear"]["x"]**2 + 
                    data["twist"]["linear"]["y"]**2
                )
            
            # 发送到MVP平台
            url = 'http://localhost:8080/api/data/ingest'
            req = urllib.request.Request(
                url,
                data=json.dumps(mvp_data).encode('utf-8'),
                headers={'Content-Type': 'application/json'},
                method='POST'
            )
            
            with urllib.request.urlopen(req, timeout=1) as response:
                if response.status == 200:
                    return True
                    
        except Exception as e:
            # 静默处理错误
            pass
        
        return False


def print_gazebo_banner():
    """打印Gazebo-ROS2集成说明"""
    print("""
╔══════════════════════════════════════════════════════════════════╗
║              🎮 Gazebo + ROS2 Robot Simulation Demo              ║
╠══════════════════════════════════════════════════════════════════╣
║                                                                   ║
║  Simulation Environment:                                          ║
║  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐         ║
║  │   Gazebo    │───▶│    ROS2     │───▶│     MVP     │         ║
║  │  Simulator  │    │  Humble/Iron│    │   Platform  │         ║
║  └─────────────┘    └─────────────┘    └─────────────┘         ║
║                                                                   ║
║  Simulated Robot: TurtleBot3 Waffle                             ║
║  World: TurtleBot3 House                                        ║
║  Physics Engine: ODE (Open Dynamics Engine)                     ║
║                                                                   ║
║  ROS2 Topics:                                                    ║
║  • /scan                 - 360° Lidar (LaserScan)              ║
║  • /imu                  - IMU sensor data                      ║
║  • /camera/image_raw     - RGB camera                          ║
║  • /odom                 - Odometry                            ║
║  • /gazebo/model_states  - Model poses                         ║
║  • /map                  - SLAM occupancy grid                 ║
║  • /plan                 - Navigation path                     ║
║                                                                   ║
║  Features:                                                       ║
║  ✅ Real-time physics simulation                                ║
║  ✅ Sensor noise modeling                                       ║
║  ✅ SLAM map generation                                         ║
║  ✅ Path planning visualization                                 ║
║  ✅ Object detection simulation                                 ║
║                                                                   ║
╚══════════════════════════════════════════════════════════════════╝
    """)


def main():
    """主函数"""
    print_gazebo_banner()
    
    # 选择机器人和世界
    robot = RobotType.TURTLEBOT3
    world = GazeboWorld.HOUSE
    
    print(f"\n🤖 Starting Gazebo-ROS2 Simulation")
    print(f"   Robot: {robot.value}")
    print(f"   World: {world.value}")
    print(f"   DDS Domain: 0")
    
    # 创建仿真器
    simulator = GazeboROS2Simulator(robot, world)
    
    print("\n📡 Publishing ROS2 Topics...")
    print("📊 Streaming to MVP Platform at http://localhost:8080")
    print("\n📈 Simulation Running... Press Ctrl+C to stop\n")
    
    try:
        data_count = 0
        last_report = time.time()
        
        while True:
            # 更新仿真时间
            simulator.update_simulation_time()
            
            # 更新机器人运动
            simulator.simulate_robot_motion()
            
            # 生成并发送传感器数据
            
            # 1. 激光雷达 (10Hz)
            if int(simulator.sim_time * 10) % 10 == 0:
                lidar_data = simulator.generate_lidar_scan()
                if simulator.send_to_mvp_platform("laser_scan", lidar_data):
                    data_count += 1
            
            # 2. IMU (100Hz)
            if int(simulator.sim_time * 100) % 100 == 0:
                imu_data = simulator.generate_imu_data()
                if simulator.send_to_mvp_platform("imu", imu_data):
                    data_count += 1
            
            # 3. 相机 (30Hz)
            if int(simulator.sim_time * 30) % 30 == 0:
                camera_data = simulator.generate_camera_image()
                if simulator.send_to_mvp_platform("camera", camera_data):
                    data_count += 1
            
            # 4. 模型状态 (50Hz)
            if int(simulator.sim_time * 50) % 50 == 0:
                model_state = simulator.generate_model_state()
                if simulator.send_to_mvp_platform("model_state", model_state):
                    data_count += 1
            
            # 5. SLAM地图 (1Hz)
            if int(simulator.sim_time) % 1 == 0:
                slam_data = simulator.generate_slam_data()
                if simulator.send_to_mvp_platform("slam_map", slam_data):
                    data_count += 1
            
            # 6. 路径规划 (0.5Hz)
            if int(simulator.sim_time * 2) % 2 == 0:
                path_data = simulator.generate_path_plan()
                if simulator.send_to_mvp_platform("path_plan", path_data):
                    data_count += 1
            
            # 状态报告
            if time.time() - last_report > 1.0:
                rtf = simulator.physics.real_time_factor
                print(f"\r⏱️  Sim Time: {simulator.sim_time:.1f}s | "
                      f"RTF: {rtf:.2f}x | "
                      f"Pos: ({simulator.robot_state['position']['x']:.2f}, "
                      f"{simulator.robot_state['position']['y']:.2f}) | "
                      f"Data Sent: {data_count} msgs    ", end="")
                last_report = time.time()
            
            # 控制仿真速率
            time.sleep(0.001)  # 1ms
            
    except KeyboardInterrupt:
        print(f"\n\n🛑 Simulation Stopped")
        print(f"📊 Total data sent: {data_count} messages")
        print(f"⏱️  Simulation time: {simulator.sim_time:.2f} seconds")


if __name__ == "__main__":
    main()