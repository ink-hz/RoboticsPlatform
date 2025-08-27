# MVP平台架构

## 项目概述
这是一个基于云原生技术栈的MVP平台架构，包含以下核心组件：

- **容器编排**: K3s (轻量级Kubernetes)
- **消息队列**: Kafka
- **数据存储**: PostgreSQL + Redis
- **监控系统**: Prometheus + Grafana
- **流处理**: Flink
- **微服务**: 示例应用

## 架构图
```
┌─────────────────────────────────────────────────┐
│                   Ingress Layer                  │
├─────────────────────────────────────────────────┤
│                 API Gateway (Kong)               │
├─────────────────────────────────────────────────┤
│   Microservices    │    Stream Processing        │
│   ┌──────────┐     │    ┌──────────┐           │
│   │ Auth Svc │     │    │  Flink   │           │
│   │ Data Svc │     │    └──────────┘           │
│   └──────────┘     │                            │
├─────────────────────────────────────────────────┤
│              Message Queue (Kafka)               │
├─────────────────────────────────────────────────┤
│   PostgreSQL    │    Redis    │   Monitoring    │
│                 │             │   Prometheus     │
└─────────────────────────────────────────────────┘
```

## 快速开始

### 1. 环境准备
```bash
# 安装K3s
curl -sfL https://get.k3s.io | sh -

# 验证安装
kubectl get nodes
```

### 2. 部署核心组件
```bash
# 部署所有组件
./scripts/deploy-all.sh
```

### 3. 访问服务
- Grafana: http://localhost:3000
- Kafka UI: http://localhost:8080
- API Gateway: http://localhost:8000

## 目录结构
- `k8s/`: Kubernetes配置文件
- `docker/`: Docker镜像定义
- `scripts/`: 部署和管理脚本
- `configs/`: 配置文件
- `apps/`: 微服务应用代码
- `monitoring/`: 监控配置
- `data/`: 数据持久化目录