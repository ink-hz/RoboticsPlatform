# 🎮 Gazebo + ROS2 机器人仿真数据集成方案

## 📋 概述
将Gazebo仿真环境中的ROS2机器人数据实时接入MVP平台，实现仿真数据的采集、分析和可视化。

## 🏗️ 架构设计

```
┌─────────────────┐     ┌──────────────┐     ┌──────────────┐     ┌─────────────┐
│     Gazebo      │────▶│     ROS2     │────▶│  ROS2-Web    │────▶│     MVP     │
│   Simulation    │     │     DDS      │     │    Bridge    │     │   Platform  │
└─────────────────┘     └──────────────┘     └──────────────┘     └─────────────┘
        │                       │                     │                     │
    [Physics]              [Topics]            [WebSocket]            [Analytics]
    [Sensors]              [Services]           [REST API]             [Storage]
    [Plugins]              [Actions]             [MQTT]               [Visualize]
```

## 🔧 关键技术组件

### 1. Gazebo 11/Ignition Gazebo
- **物理仿真引擎**: 真实物理模拟
- **传感器插件**: 激光雷达、相机、IMU等
- **机器人模型**: URDF/SDF格式
- **世界环境**: 室内/室外场景

### 2. ROS2 (Humble/Iron)
- **DDS通信**: Fast-DDS/Cyclone DDS
- **QoS配置**: 可靠性和实时性保证
- **生命周期节点**: 管理节点状态
- **组件化**: 可组合的节点架构

### 3. 桥接技术
- **ros2-web-bridge**: WebSocket通信
- **rosbridge_suite**: ROS2版本
- **MQTT Bridge**: 轻量级协议
- **gRPC**: 高性能RPC

## 💻 实现方案

### 方案A: ros2-web-bridge (推荐)

#### 1. 安装配置
```bash
# 安装ROS2 Humble
sudo apt update
sudo apt install ros-humble-desktop

# 安装Gazebo
sudo apt install ros-humble-gazebo-ros-pkgs

# 安装ros2-web-bridge
npm install -g ros2-web-bridge

# 安装Python依赖
pip3 install rclpy websocket-client
```

#### 2. 启动Gazebo仿真
```bash
# 启动Gazebo with TurtleBot3
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# 或启动自定义机器人
ros2 launch my_robot_gazebo simulation.launch.py
```

#### 3. 桥接服务实现
```python
#!/usr/bin/env python3
"""
ROS2-Gazebo to MVP Platform Bridge
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import websocket
import json
import threading
from typing import Any, Dict

# ROS2消息类型
from sensor_msgs.msg import LaserScan, Imu, Image, PointCloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import LinkStates, ModelStates
from std_msgs.msg import Float32MultiArray

class GazeboROS2Bridge(Node):
    def __init__(self):
        super().__init__('gazebo_ros2_bridge')
        
        # WebSocket连接到MVP平台
        self.ws_url = "ws://localhost:8080/ws"
        self.ws = None
        self.connect_websocket()
        
        # QoS配置 - 适配Gazebo的发布策略
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # 订阅Gazebo传感器话题
        self.create_subscriptions()
        
        # 性能监控
        self.message_count = 0
        self.create_timer(1.0, self.report_statistics)
        
        self.get_logger().info('Gazebo-ROS2 Bridge initialized')
    
    def connect_websocket(self):
        """连接到MVP平台WebSocket"""
        try:
            self.ws = websocket.WebSocket()
            self.ws.connect(self.ws_url)
            self.get_logger().info(f'Connected to MVP Platform at {self.ws_url}')
        except Exception as e:
            self.get_logger().error(f'WebSocket connection failed: {e}')
    
    def create_subscriptions(self):
        """订阅Gazebo/ROS2话题"""
        
        # 1. 激光雷达数据
        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            self.qos_profile
        )
        
        # 2. IMU数据
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            self.qos_profile
        )
        
        # 3. 里程计数据
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odometry_callback,
            self.qos_profile
        )
        
        # 4. Gazebo模型状态
        self.model_states_sub = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.model_states_callback,
            self.qos_profile
        )
        
        # 5. 相机图像 (压缩传输)
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            self.qos_profile
        )
        
        # 6. 3D点云数据
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/velodyne_points',
            self.pointcloud_callback,
            self.qos_profile
        )
    
    def laser_callback(self, msg: LaserScan):
        """处理激光雷达数据"""
        data = {
            'type': 'laser_scan',
            'timestamp': self.get_clock().now().nanoseconds / 1e9,
            'header': {
                'frame_id': msg.header.frame_id,
                'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            },
            'data': {
                'angle_min': msg.angle_min,
                'angle_max': msg.angle_max,
                'ranges': list(msg.ranges)[:36],  # 采样360个点到36个
                'min_range': float(min(msg.ranges)),
                'max_range': float(max(msg.ranges)),
                'obstacles': self.detect_obstacles(msg.ranges)
            }
        }
        self.send_to_mvp(data)
    
    def detect_obstacles(self, ranges):
        """检测障碍物"""
        obstacles = []
        threshold = 1.0  # 1米内视为障碍物
        
        for i, distance in enumerate(ranges):
            if distance < threshold:
                angle = i * (360.0 / len(ranges))
                obstacles.append({
                    'angle': angle,
                    'distance': float(distance),
                    'sector': self.get_sector(angle)
                })
        
        return obstacles[:5]  # 返回最近的5个障碍物
    
    def get_sector(self, angle):
        """获取障碍物所在扇区"""
        if 315 <= angle or angle < 45:
            return 'front'
        elif 45 <= angle < 135:
            return 'right'
        elif 135 <= angle < 225:
            return 'back'
        else:
            return 'left'
    
    def imu_callback(self, msg: Imu):
        """处理IMU数据"""
        data = {
            'type': 'imu',
            'timestamp': self.get_clock().now().nanoseconds / 1e9,
            'data': {
                'orientation': {
                    'x': msg.orientation.x,
                    'y': msg.orientation.y,
                    'z': msg.orientation.z,
                    'w': msg.orientation.w
                },
                'angular_velocity': {
                    'x': msg.angular_velocity.x,
                    'y': msg.angular_velocity.y,
                    'z': msg.angular_velocity.z
                },
                'linear_acceleration': {
                    'x': msg.linear_acceleration.x,
                    'y': msg.linear_acceleration.y,
                    'z': msg.linear_acceleration.z
                }
            }
        }
        self.send_to_mvp(data)
    
    def odometry_callback(self, msg: Odometry):
        """处理里程计数据"""
        data = {
            'type': 'odometry',
            'timestamp': self.get_clock().now().nanoseconds / 1e9,
            'data': {
                'position': {
                    'x': msg.pose.pose.position.x,
                    'y': msg.pose.pose.position.y,
                    'z': msg.pose.pose.position.z
                },
                'velocity': {
                    'linear': msg.twist.twist.linear.x,
                    'angular': msg.twist.twist.angular.z
                },
                'child_frame_id': msg.child_frame_id
            }
        }
        self.send_to_mvp(data)
    
    def model_states_callback(self, msg: ModelStates):
        """处理Gazebo模型状态"""
        # 找到机器人模型
        robot_index = -1
        for i, name in enumerate(msg.name):
            if 'robot' in name.lower() or 'turtlebot' in name.lower():
                robot_index = i
                break
        
        if robot_index >= 0:
            pose = msg.pose[robot_index]
            twist = msg.twist[robot_index]
            
            data = {
                'type': 'gazebo_model_state',
                'timestamp': self.get_clock().now().nanoseconds / 1e9,
                'data': {
                    'model_name': msg.name[robot_index],
                    'position': {
                        'x': pose.position.x,
                        'y': pose.position.y,
                        'z': pose.position.z
                    },
                    'orientation': {
                        'x': pose.orientation.x,
                        'y': pose.orientation.y,
                        'z': pose.orientation.z,
                        'w': pose.orientation.w
                    },
                    'linear_velocity': {
                        'x': twist.linear.x,
                        'y': twist.linear.y,
                        'z': twist.linear.z
                    },
                    'angular_velocity': {
                        'x': twist.angular.x,
                        'y': twist.angular.y,
                        'z': twist.angular.z
                    }
                }
            }
            self.send_to_mvp(data)
    
    def image_callback(self, msg: Image):
        """处理相机图像 (降采样)"""
        # 仅发送图像元数据，避免传输大量数据
        data = {
            'type': 'camera_image',
            'timestamp': self.get_clock().now().nanoseconds / 1e9,
            'data': {
                'width': msg.width,
                'height': msg.height,
                'encoding': msg.encoding,
                'is_bigendian': msg.is_bigendian,
                'step': msg.step,
                'data_size': len(msg.data),
                # 可选：添加图像分析结果
                'analysis': self.analyze_image(msg)
            }
        }
        self.send_to_mvp(data)
    
    def analyze_image(self, img_msg):
        """简单图像分析 (示例)"""
        return {
            'brightness': 'normal',  # 可以计算实际亮度
            'objects_detected': 0,    # 可以集成目标检测
            'timestamp': self.get_clock().now().nanoseconds / 1e9
        }
    
    def pointcloud_callback(self, msg: PointCloud2):
        """处理3D点云数据"""
        # 发送点云统计信息
        data = {
            'type': 'pointcloud',
            'timestamp': self.get_clock().now().nanoseconds / 1e9,
            'data': {
                'width': msg.width,
                'height': msg.height,
                'point_count': msg.width * msg.height,
                'is_dense': msg.is_dense,
                'fields': [f.name for f in msg.fields],
                # 可以添加点云分析
                'analysis': {
                    'clusters': 0,  # 聚类数量
                    'ground_points': 0,  # 地面点
                    'obstacle_points': 0  # 障碍物点
                }
            }
        }
        self.send_to_mvp(data)
    
    def send_to_mvp(self, data: Dict[str, Any]):
        """发送数据到MVP平台"""
        try:
            # 转换为MVP平台格式
            mvp_data = {
                'device_id': f"gazebo_{data['type']}",
                'sensor_type': data['type'],
                'value': 0,  # 默认值
                'timestamp': data['timestamp'],
                'metadata': data['data']
            }
            
            # 提取关键数值
            if data['type'] == 'laser_scan':
                mvp_data['value'] = data['data']['min_range']
            elif data['type'] == 'odometry':
                mvp_data['value'] = data['data']['velocity']['linear']
            elif data['type'] == 'imu':
                mvp_data['value'] = data['data']['angular_velocity']['z']
            
            # 发送到MVP平台
            if self.ws:
                self.ws.send(json.dumps(mvp_data))
                self.message_count += 1
            
        except Exception as e:
            self.get_logger().error(f'Failed to send data: {e}')
    
    def report_statistics(self):
        """报告统计信息"""
        self.get_logger().info(f'Messages sent: {self.message_count} msg/s')
        self.message_count = 0


def main(args=None):
    rclpy.init(args=args)
    bridge = GazeboROS2Bridge()
    
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 方案B: DDS直连 (高性能)

```python
#!/usr/bin/env python3
"""
Direct DDS Connection for ROS2-Gazebo
使用Fast-DDS直接通信，绕过ROS2 API
"""

import fastdds
import time
from dataclasses import dataclass
from typing import List

@dataclass
class LaserScanData:
    timestamp: float
    ranges: List[float]
    angle_min: float
    angle_max: float

class DDSBridge:
    def __init__(self):
        # 创建DDS参与者
        self.participant = fastdds.DomainParticipant(0)
        
        # 创建订阅者
        self.subscriber = self.participant.create_subscriber()
        
        # 创建话题和数据读取器
        self.setup_topics()
        
    def setup_topics(self):
        """设置DDS话题"""
        # LaserScan话题
        laser_topic = self.participant.create_topic(
            "rt/scan",
            "sensor_msgs::msg::dds_::LaserScan_"
        )
        
        self.laser_reader = self.subscriber.create_datareader(
            laser_topic,
            self.on_laser_data
        )
    
    def on_laser_data(self, reader):
        """处理激光数据"""
        samples = reader.take()
        for sample in samples:
            # 直接处理DDS数据
            self.process_laser(sample)
    
    def process_laser(self, data):
        """处理并转发数据"""
        # 高性能数据处理
        pass
```

## 🚀 Docker Compose配置

```yaml
version: '3.8'

services:
  # Gazebo仿真环境
  gazebo:
    image: osrf/ros:humble-desktop
    environment:
      - DISPLAY=${DISPLAY}
      - QT_X11_NO_MITSHM=1
      - GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      - ./worlds:/workspace/worlds
      - ./models:/workspace/models
    network_mode: host
    command: ros2 launch gazebo_ros gazebo.launch.py

  # ROS2-Web Bridge
  ros2-bridge:
    image: node:16
    working_dir: /app
    volumes:
      - ./bridge:/app
    environment:
      - ROS_DOMAIN_ID=0
    network_mode: host
    command: |
      sh -c "
      npm install ros2-web-bridge &&
      node ./node_modules/ros2-web-bridge/bin/rosbridge.js
      "

  # MVP平台（增强版）
  mvp-platform:
    build: ./mvp-enhanced
    ports:
      - "8080:8080"
    environment:
      - ENABLE_ROS2=true
      - ENABLE_GAZEBO=true
      - DDS_DOMAIN=0
    volumes:
      - ./data:/data
    depends_on:
      - influxdb
      - grafana

  # 时序数据库
  influxdb:
    image: influxdb:2.0
    ports:
      - "8086:8086"
    environment:
      - INFLUXDB_DB=gazebo_data
      - INFLUXDB_ADMIN_USER=admin
      - INFLUXDB_ADMIN_PASSWORD=admin123
    volumes:
      - influxdb-data:/var/lib/influxdb2

  # 可视化
  grafana:
    image: grafana/grafana:latest
    ports:
      - "3000:3000"
    environment:
      - GF_SECURITY_ADMIN_PASSWORD=admin
    volumes:
      - ./grafana/dashboards:/etc/grafana/provisioning/dashboards
      - ./grafana/datasources:/etc/grafana/provisioning/datasources

volumes:
  influxdb-data:
```

## 📊 Gazebo特定数据类型

### 1. 物理仿真数据
- **碰撞检测**: Contact forces
- **关节状态**: Joint positions/velocities/efforts
- **链接状态**: Link poses and twists
- **模型状态**: Model poses in world frame

### 2. 传感器仿真
- **RGB-D相机**: Color + Depth images
- **激光雷达**: 2D/3D laser scans
- **IMU**: Accelerometer + Gyroscope
- **GPS**: Geographic coordinates
- **力/扭矩传感器**: Force/Torque measurements

### 3. 环境数据
- **光照条件**: Ambient/Directional light
- **物理属性**: Gravity, air resistance
- **时间**: Simulation time vs real time
- **性能指标**: Real-time factor, FPS

## 🎮 实际应用示例

### 1. TurtleBot3仿真集成
```bash
# 启动TurtleBot3 Gazebo仿真
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py

# 启动SLAM
ros2 launch turtlebot3_cartographer cartographer.launch.py

# 启动导航
ros2 launch turtlebot3_navigation2 navigation2.launch.py

# 启动数据桥接
python3 gazebo_ros2_bridge.py
```

### 2. 机械臂仿真集成
```bash
# 启动机械臂仿真
ros2 launch moveit2_tutorials demo.launch.py

# 启动运动规划
ros2 run moveit2_tutorials motion_planning_api_tutorial

# 桥接关节状态
python3 arm_data_bridge.py
```

### 3. 无人机仿真集成
```bash
# 启动PX4 + Gazebo
make px4_sitl gazebo

# 启动QGroundControl
./QGroundControl.AppImage

# 桥接MAVLink数据
python3 mavlink_bridge.py
```

## 📈 性能优化

### 1. 数据采样策略
```python
class DataSampler:
    def __init__(self, sample_rate=10):
        self.sample_rate = sample_rate
        self.last_sample = {}
    
    def should_sample(self, topic, timestamp):
        if topic not in self.last_sample:
            self.last_sample[topic] = timestamp
            return True
        
        if timestamp - self.last_sample[topic] > 1.0/self.sample_rate:
            self.last_sample[topic] = timestamp
            return True
        
        return False
```

### 2. 批量传输
```python
class BatchTransmitter:
    def __init__(self, batch_size=100, timeout=0.1):
        self.batch = []
        self.batch_size = batch_size
        self.timeout = timeout
        self.last_send = time.time()
    
    def add(self, data):
        self.batch.append(data)
        
        if len(self.batch) >= self.batch_size or \
           time.time() - self.last_send > self.timeout:
            self.send_batch()
    
    def send_batch(self):
        if self.batch:
            # 批量发送
            send_to_mvp(self.batch)
            self.batch = []
            self.last_send = time.time()
```

### 3. 数据压缩
```python
import zlib
import base64

def compress_data(data):
    """压缩大数据"""
    json_str = json.dumps(data)
    compressed = zlib.compress(json_str.encode())
    return base64.b64encode(compressed).decode()

def decompress_data(compressed_str):
    """解压数据"""
    compressed = base64.b64decode(compressed_str)
    json_str = zlib.decompress(compressed).decode()
    return json.loads(json_str)
```

## 🔧 故障排除

### 常见问题

1. **DDS域ID不匹配**
```bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

2. **QoS不兼容**
```python
# 使用兼容的QoS配置
qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)
```

3. **性能问题**
```bash
# 优化DDS配置
export FASTRTPS_DEFAULT_PROFILES_FILE=fastdds_profile.xml
```

## ✅ 总结

**是的，完全可以接入Gazebo仿真的ROS2机器人数据！**

### 优势
- ✅ **完整仿真环境**: 物理引擎 + 传感器模拟
- ✅ **标准ROS2接口**: 完全兼容ROS2生态
- ✅ **实时数据流**: DDS高性能通信
- ✅ **可扩展性**: 支持多机器人仿真
- ✅ **开发友好**: 无需真实硬件即可测试

### 实现路径
1. **快速原型**: ros2-web-bridge + WebSocket
2. **生产部署**: DDS直连 + 批量传输
3. **大规模仿真**: Ignition Gazebo + 分布式架构

### 应用场景
- 🚗 自动驾驶算法验证
- 🦾 机械臂运动规划
- 🚁 无人机群体协同
- 🤖 SLAM算法开发
- 🏭 数字孪生系统