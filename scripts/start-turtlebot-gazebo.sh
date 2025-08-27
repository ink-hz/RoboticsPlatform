#!/bin/bash

echo "🐢 启动TurtleBot3 Gazebo仿真..."

# 设置环境
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=burger

# 检查Gazebo是否已在运行
if pgrep -x "gzserver" > /dev/null; then
    echo "⚠️  Gazebo已在运行"
    exit 1
fi

# 安装TurtleBot3 Gazebo包（如果未安装）
if ! ros2 pkg list | grep -q turtlebot3_gazebo; then
    echo "📦 安装TurtleBot3 Gazebo包..."
    sudo apt update
    sudo apt install -y ros-jazzy-turtlebot3-gazebo
fi

echo "🚀 启动TurtleBot3世界..."
echo "💡 启动后可在另一个终端运行机器人桥接服务"

# 启动TurtleBot3 Gazebo世界
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py