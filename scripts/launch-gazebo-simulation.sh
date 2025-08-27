#!/bin/bash

set -e

echo "🚀 启动Gazebo仿真环境和云平台桥接..."

# 颜色定义
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# 检查ROS2环境
if [ ! -f "/opt/ros/jazzy/setup.bash" ]; then
    echo -e "${RED}❌ ROS2 Jazzy未找到!${NC}"
    exit 1
fi

# 设置ROS2环境
echo -e "${BLUE}🔧 设置ROS2环境...${NC}"
source /opt/ros/jazzy/setup.bash

# 检查TurtleBot3包
if ! ros2 pkg list | grep -q turtlebot3; then
    echo -e "${RED}❌ TurtleBot3包未找到!${NC}"
    echo "请安装: sudo apt install ros-jazzy-turtlebot3*"
    exit 1
fi

# 设置TurtleBot3模型
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/jazzy/share/turtlebot3_gazebo/models

echo -e "${GREEN}✅ TurtleBot3模型设置为: ${TURTLEBOT3_MODEL}${NC}"

# 创建会话目录
SESSION_DIR="/tmp/gazebo_cloud_session"
mkdir -p $SESSION_DIR

# 函数：清理后台进程
cleanup() {
    echo -e "\n${YELLOW}🛑 正在停止所有进程...${NC}"
    
    # 杀死所有相关进程
    pkill -f gazebo || true
    pkill -f robot_state_publisher || true
    pkill -f gazebo_cloud_bridge || true
    
    # 清理会话文件
    rm -rf $SESSION_DIR
    
    echo -e "${GREEN}✅ 清理完成${NC}"
    exit 0
}

# 设置信号处理
trap cleanup SIGINT SIGTERM

# 检查Gazebo是否已经在运行
if pgrep -x "gzserver" > /dev/null || pgrep -x "gazebo" > /dev/null; then
    echo -e "${YELLOW}⚠️  Gazebo已在运行，将尝试连接现有实例${NC}"
    GAZEBO_RUNNING=true
else
    GAZEBO_RUNNING=false
fi

# 启动函数
start_gazebo() {
    if [ "$GAZEBO_RUNNING" = false ]; then
        echo -e "${BLUE}🏗️  启动Gazebo仿真环境...${NC}"
        
        # 尝试启动TurtleBot3 Gazebo世界
        if ros2 pkg list | grep -q turtlebot3_gazebo; then
            echo -e "${BLUE}📦 使用TurtleBot3 Gazebo包启动...${NC}"
            ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py &
        else
            echo -e "${YELLOW}⚠️  TurtleBot3 Gazebo包未找到，尝试基础启动...${NC}"
            gazebo --verbose &
        fi
        
        # 等待Gazebo启动
        echo -e "${YELLOW}⏳ 等待Gazebo启动(30秒)...${NC}"
        sleep 30
    fi
}

start_robot_state_publisher() {
    echo -e "${BLUE}🤖 启动机器人状态发布器...${NC}"
    
    # 启动robot_state_publisher
    ros2 run robot_state_publisher robot_state_publisher \
        --ros-args --param use_sim_time:=true &
    
    sleep 2
}

start_cloud_bridge() {
    echo -e "${BLUE}🌉 启动云平台桥接服务...${NC}"
    
    # 切换到桥接脚本目录
    cd /home/ink/RoboticsPlatform/robot/ros-bridge
    
    # 安装Python依赖
    if [ ! -d "venv" ]; then
        echo -e "${BLUE}📦 创建Python虚拟环境...${NC}"
        python3 -m venv venv
    fi
    
    source venv/bin/activate
    pip install -q requests rclpy
    
    # 启动桥接服务
    python3 gazebo_cloud_bridge.py &
    
    # 保存桥接进程PID
    echo $! > $SESSION_DIR/bridge.pid
}

# 主启动流程
echo -e "${GREEN}🎯 开始启动流程...${NC}"

# 1. 启动Gazebo
start_gazebo

# 2. 启动机器人状态发布器
start_robot_state_publisher

# 3. 启动云平台桥接
start_cloud_bridge

echo -e "${GREEN}✅ 所有服务启动完成!${NC}"
echo -e "${BLUE}📊 访问云平台控制台: http://127.0.0.1:8000${NC}"
echo -e "${YELLOW}📡 机器人遥测数据正在发送到云平台...${NC}"
echo -e "${GREEN}🎮 在Gazebo中控制机器人，数据将实时显示在云平台中${NC}"

echo -e "\n${YELLOW}💡 提示:${NC}"
echo -e "  - 在Gazebo中移动机器人"
echo -e "  - 查看云平台仪表板的实时数据更新"
echo -e "  - 按Ctrl+C停止所有服务"

echo -e "\n${BLUE}🔄 保持运行中... 按Ctrl+C停止${NC}"

# 保持脚本运行
while true; do
    sleep 10
    
    # 检查关键进程是否还在运行
    if ! pgrep -f gazebo_cloud_bridge > /dev/null; then
        echo -e "${RED}⚠️  桥接服务已停止${NC}"
        break
    fi
done

cleanup