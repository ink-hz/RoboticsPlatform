# 机器人云平台 - 快速开始指南

## 🚀 30分钟内启动你的机器人云平台

### 前置要求
- Linux/macOS系统 (WSL2也可以)
- Docker和Docker Compose
- 至少8GB RAM
- 20GB可用磁盘空间

### 第一步：启动本地开发环境

```bash
# 1. 启动所有基础服务
make dev

# 2. 检查服务状态
make status
```

服务启动后可访问：
- **Grafana监控面板**: http://localhost:3000 (admin/admin)
- **Prometheus指标**: http://localhost:9090
- **MinIO对象存储**: http://localhost:9001 (minioadmin/minioadmin)

### 第二步：部署核心服务

```bash
# 构建服务镜像
make build

# 如果有K3s环境，部署到Kubernetes
make setup  # 首次运行，安装K3s
make deploy # 部署平台服务
```

### 第三步：测试平台功能

#### 1. 测试API网关
```bash
# 健康检查
curl http://localhost:8000/health

# 发送模拟遥测数据
curl -X POST http://localhost:8000/api/v1/robots/robot-001/telemetry \
  -H "Content-Type: application/json" \
  -d '{
    "data": {
      "battery": 85.5,
      "location": {"x": 10.2, "y": 5.3, "z": 0},
      "temperature": 24.3
    }
  }'
```

#### 2. 注册边缘节点
```bash
curl -X POST http://localhost:8005/nodes/register \
  -H "Content-Type: application/json" \
  -d '{
    "id": "edge-001",
    "name": "Edge Node 1",
    "ip": "192.168.1.100",
    "resources": {
      "cpu_cores": 4,
      "memory_gb": 8,
      "disk_gb": 100,
      "cpu_usage": 25.5,
      "memory_usage": 40.2
    }
  }'
```

### 第四步：连接机器人（模拟）

创建一个Python脚本模拟机器人数据上报：

```python
# robot_simulator.py
import requests
import json
import time
from datetime import datetime

def send_telemetry():
    robot_id = "robot-sim-001"
    api_url = "http://localhost:8000/api/v1/robots/{}/telemetry"
    
    while True:
        data = {
            "timestamp": datetime.utcnow().isoformat(),
            "data": {
                "battery": 80 + (time.time() % 20),
                "velocity": {"linear": 0.5, "angular": 0.1},
                "position": {
                    "x": time.time() % 100,
                    "y": (time.time() * 0.5) % 100,
                    "z": 0
                },
                "sensors": {
                    "lidar": "active",
                    "camera": "active",
                    "imu": "active"
                }
            }
        }
        
        response = requests.post(
            api_url.format(robot_id),
            json=data
        )
        print(f"Sent telemetry: {response.status_code}")
        time.sleep(1)

if __name__ == "__main__":
    send_telemetry()
```

### 第五步：查看监控数据

1. 打开Grafana: http://localhost:3000
2. 添加Prometheus数据源：
   - URL: http://prometheus:9090
3. 导入预设仪表板或创建自定义仪表板

### 项目结构说明

```
RoboticsPlatform/
├── platform/           # 核心平台服务
│   ├── cloud-services/ # 云端服务
│   ├── data-pipeline/  # 数据处理管道
│   └── robot-connector/# 机器人连接器
├── infrastructure/     # 基础设施配置
│   ├── kubernetes/     # K8s部署文件
│   └── monitoring/     # 监控配置
├── services/          # 微服务
│   └── api-gateway/   # API网关服务
├── edge/              # 边缘计算
│   └── controllers/   # 边缘控制器(Go)
├── ml-ops/            # 机器学习运维
└── robot/             # 机器人接口
```

### 常用命令

```bash
# 开发环境
make dev          # 启动开发环境
make dev-down     # 停止开发环境
make logs         # 查看日志

# 构建和部署
make build        # 构建所有服务
make deploy       # 部署到K8s

# 监控和调试
make monitor      # 打开监控面板
make status       # 检查服务状态

# 测试
make test         # 运行所有测试
```

### 下一步

1. **集成ROS2**: 在`robot/ros-bridge`目录实现ROS2桥接服务
2. **添加ML模型**: 在`ml-ops/`目录实现模型训练和部署流水线
3. **扩展数据处理**: 增强`platform/data-pipeline`的实时处理能力
4. **安全加固**: 添加认证、授权和加密

### 故障排除

#### 问题：Docker服务无法启动
```bash
# 检查Docker状态
sudo systemctl status docker

# 重启Docker
sudo systemctl restart docker
```

#### 问题：端口被占用
```bash
# 查看端口占用
sudo lsof -i :8000

# 修改docker-compose.yml中的端口映射
```

#### 问题：K3s安装失败
```bash
# 手动安装K3s
curl -sfL https://get.k3s.io | sh -

# 检查K3s状态
sudo systemctl status k3s
```

### 获取帮助

- 查看完整文档：`docs/`目录
- 提交问题：创建GitHub Issue
- 社区支持：加入Discord/Slack频道

---
🎯 **恭喜！** 你已经成功搭建了机器人云平台的基础架构。现在可以开始根据你的五年规划逐步完善和扩展平台功能了。