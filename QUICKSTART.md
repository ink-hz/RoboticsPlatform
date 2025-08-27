# 🚀 快速开始 - 机器人云平台

30分钟内搭建并运行您的机器人云平台！

## 📋 前置要求

- **操作系统**: Linux (Ubuntu 22.04+) 或 WSL2
- **Go语言**: 1.22+ 版本
- **内存**: 最少 8GB RAM
- **磁盘**: 20GB 可用空间

## 🎯 快速安装

### 1️⃣ 安装 Go (如未安装)

```bash
# Ubuntu/Debian
sudo apt update
sudo apt install golang-go

# 或下载最新版
wget https://go.dev/dl/go1.23.0.linux-amd64.tar.gz
sudo tar -C /usr/local -xzf go1.23.0.linux-amd64.tar.gz
export PATH=$PATH:/usr/local/go/bin
```

### 2️⃣ 克隆项目

```bash
git clone https://github.com/ink-hz/RoboticsPlatform.git
cd RoboticsPlatform
```

### 3️⃣ 启动平台

```bash
# 进入 API 网关目录
cd services/go-api-gateway

# 下载依赖
go mod download

# 编译运行
go build -o robot-cloud-go ./cmd/server
./robot-cloud-go
```

### 4️⃣ 访问控制台

打开浏览器访问: **http://127.0.0.1:8000**

🎉 恭喜！平台已经运行起来了！

## 📊 验证安装

### 检查健康状态

```bash
curl http://127.0.0.1:8000/health
```

预期响应：
```json
{
  "status": "healthy",
  "service": "Robot Cloud Platform Go API Gateway",
  "version": "1.0.0"
}
```

### 查看 API 性能

```bash
# 测试仪表板 API
curl http://127.0.0.1:8000/api/v1/dashboard/stats

# 测试机器人列表
curl http://127.0.0.1:8000/api/v1/robots
```

## 🤖 发送测试数据

### 方法1: 使用 Go 脚本

```bash
cd scripts
go run send-demo-data.go
```

### 方法2: 使用 curl

```bash
curl -X POST http://127.0.0.1:8000/api/v1/robots/test-robot/telemetry \
  -H "Content-Type: application/json" \
  -d '{
    "data": {
      "battery": 85.5,
      "position": {"x": 10.2, "y": 5.3, "z": 0},
      "velocity": {"linear": 0.5, "angular": 0.1},
      "status": "active"
    }
  }'
```

## 🛠️ 配置说明

### 环境变量配置

```bash
# 服务器配置
export SERVER_PORT=8000
export SERVER_HOST=0.0.0.0

# 日志配置
export LOGGER_LEVEL=info
export LOGGER_FORMAT=json

# 启动服务
./robot-cloud-go
```

### 配置文件 (config.yaml)

```yaml
server:
  host: "0.0.0.0"
  port: 8000
  mode: "release"  # debug, release, test

logger:
  level: "info"    # debug, info, warn, error
  format: "json"   # json, text

database:
  enabled: false   # 数据库暂未启用
```

## 📈 性能特点

- **响应时间**: 10-80 微秒
- **并发支持**: 10,000+ 连接
- **内存占用**: < 30MB
- **CPU使用**: 极低
- **启动时间**: < 1秒

## 🔧 常用命令

```bash
# 开发模式运行
go run cmd/server/main.go

# 生产构建
go build -ldflags="-w -s" -o robot-cloud-go ./cmd/server

# 运行测试
go test ./...

# 格式化代码
go fmt ./...

# 检查代码
go vet ./...
```

## 🐛 故障排除

### 端口被占用

```bash
# 检查端口占用
sudo lsof -i :8000

# 使用其他端口
SERVER_PORT=8080 ./robot-cloud-go
```

### 依赖下载失败

```bash
# 设置 Go 代理
go env -w GOPROXY=https://goproxy.cn,direct

# 重新下载
go mod download
```

### 权限问题

```bash
# 给执行权限
chmod +x robot-cloud-go

# 运行
./robot-cloud-go
```

## 🎯 下一步

1. **探索 Web 控制台** - 查看仪表板、机器人状态和遥测数据
2. **连接真实机器人** - 集成 ROS2 或其他机器人系统
3. **部署到生产** - 使用 Docker 或 Kubernetes
4. **添加数据库** - 集成 PostgreSQL 持久化存储

## 📚 相关文档

- [架构设计](docs/ARCHITECTURE.md)
- [API 文档](http://127.0.0.1:8000/api)
- [ROS2 集成指南](docs/GAZEBO_INTEGRATION.md)
- [部署指南](docs/DEPLOYMENT_GUIDE.md)

## 💬 获取帮助

遇到问题？

- 查看 [项目状态](STATUS.md)
- 提交 [Issue](https://github.com/ink-hz/RoboticsPlatform/issues)
- 查看 [FAQ](docs/FAQ.md)

---

**祝您使用愉快！** 🚀

*机器人云平台 - 高性能、易部署、可扩展*