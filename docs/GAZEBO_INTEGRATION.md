# Gazebo仿真环境接入指南

## ✅ 集成完成状态

你的机器人云平台已经成功集成了Gazebo仿真环境！

### 🎯 当前已实现功能

1. **✅ ROS2桥接服务** - 实时数据传输
2. **✅ 遥测数据流** - 电池、位置、速度等
3. **✅ 云平台可视化** - 实时显示仿真数据
4. **✅ 多机器人支持** - 可同时接入多个机器人

## 🚀 快速启动

### 方法一：使用测试节点（推荐开始）

```bash
# 1. 启动云平台（如果未启动）
cd /home/ink/RoboticsPlatform/services/api-gateway
source venv/bin/activate && python main.py &

# 2. 启动ROS2测试节点
source /opt/ros/jazzy/setup.bash
cd /home/ink/RoboticsPlatform
python3 scripts/test-ros-connection.py
```

### 方法二：使用TurtleBot3 Gazebo

```bash
# 终端1：启动Gazebo仿真
./scripts/start-turtlebot-gazebo.sh

# 终端2：启动桥接服务
cd robot/ros-bridge
source /opt/ros/jazzy/setup.bash
python3 gazebo_cloud_bridge.py
```

### 方法三：一键启动（完整环境）

```bash
./scripts/launch-gazebo-simulation.sh
```

## 📊 查看实时数据

启动后访问：
- **控制台**: http://127.0.0.1:8000
- **API数据**: http://127.0.0.1:8000/api/v1/telemetry/latest

## 🔧 支持的数据类型

### 从Gazebo获取的数据

| 数据类型 | ROS话题 | 说明 |
|---------|---------|------|
| 位置信息 | `/odom` | X/Y/Z坐标和朝向 |
| 速度信息 | `/odom` | 线速度和角速度 |
| 激光雷达 | `/scan` | 距离传感器数据 |
| IMU数据 | `/imu` | 加速度和陀螺仪 |
| 电池状态 | `/battery_state` | 电量百分比 |
| 系统诊断 | `/diagnostics` | 健康状态信息 |

### 发送到云平台的数据

```json
{
  "timestamp": "2025-08-27T11:33:34.736346",
  "data": {
    "battery": 79.3,
    "position": {"x": 1.2, "y": 0.0, "z": 0.0},
    "velocity": {"linear": 0.2, "angular": 0.0},
    "status": "active",
    "temperature": 27.0,
    "laser_ranges": [...],
    "imu": {...}
  },
  "metadata": {
    "source": "gazebo_simulation",
    "robot_type": "turtlebot3"
  }
}
```

## 🎮 控制机器人

### 通过ROS命令控制

```bash
# 发布速度命令
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.2}, angular: {z: 0.5}}'

# 停止机器人
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.0}, angular: {z: 0.0}}'
```

### 通过云平台API控制（开发中）

```bash
# 发送移动命令
curl -X POST http://127.0.0.1:8000/api/v1/robots/turtlebot3_gazebo_001/commands \
  -H "Content-Type: application/json" \
  -d '{"type": "move", "linear": 0.2, "angular": 0.1}'
```

## 🔧 自定义配置

### 修改机器人ID

编辑 `robot/ros-bridge/gazebo_cloud_bridge.py`:

```python
self.robot_id = "your_custom_robot_id"
```

### 修改数据发送频率

```python
# 修改定时器间隔（秒）
self.timer = self.create_timer(1.0, self.send_telemetry_to_cloud)  # 1秒
```

### 添加新的传感器

```python
# 在create_subscriptions方法中添加
self.camera_sub = self.create_subscription(
    Image, '/camera/image_raw', self.camera_callback, qos_profile)
```

## 🐛 故障排除

### 1. ROS2环境问题

```bash
# 检查ROS2安装
source /opt/ros/jazzy/setup.bash
ros2 --help

# 检查可用话题
ros2 topic list
```

### 2. 网络连接问题

```bash
# 测试云平台连接
curl http://127.0.0.1:8000/health

# 检查端口占用
sudo lsof -i :8000
```

### 3. Gazebo启动失败

```bash
# 安装缺少的包
sudo apt update
sudo apt install ros-jazzy-turtlebot3-gazebo

# 设置环境变量
export TURTLEBOT3_MODEL=burger
```

## 📈 性能监控

### 查看数据统计

```bash
# 获取仪表板数据
curl http://127.0.0.1:8000/api/v1/dashboard/stats

# 获取遥测数据
curl http://127.0.0.1:8000/api/v1/telemetry/latest
```

### 监控ROS话题

```bash
# 监听话题频率
ros2 topic hz /odom

# 查看话题数据
ros2 topic echo /scan
```

## 🎯 下一步计划

- [ ] 添加摄像头数据流
- [ ] 实现云端控制命令
- [ ] 支持多机器人编队
- [ ] 添加路径规划可视化
- [ ] 集成ROS2 Navigation

## 💡 最佳实践

1. **数据采样**: 对高频数据（如激光雷达）进行采样，避免网络拥堵
2. **错误处理**: 网络断开时缓存数据，恢复后批量发送
3. **安全考虑**: 生产环境中使用HTTPS和API密钥
4. **性能优化**: 根据网络条件调整发送频率

---

🎉 **恭喜！你的Gazebo仿真环境已成功接入机器人云平台！**

现在你可以在云平台控制台中实时查看仿真机器人的数据，这为后续的真实机器人接入奠定了坚实基础。