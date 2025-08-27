#!/bin/bash

# 配置Docker镜像加速器
echo "配置Docker镜像加速..."

# 创建Docker配置目录
sudo mkdir -p /etc/docker

# 配置镜像加速器（使用多个镜像源）
sudo tee /etc/docker/daemon.json > /dev/null <<EOF
{
  "registry-mirrors": [
    "https://docker.m.daocloud.io",
    "https://dockerproxy.com",
    "https://docker.mirrors.ustc.edu.cn",
    "https://mirror.ccs.tencentyun.com"
  ],
  "dns": ["8.8.8.8", "114.114.114.114"],
  "max-concurrent-downloads": 10,
  "log-driver": "json-file",
  "log-opts": {
    "max-size": "100m",
    "max-file": "5"
  },
  "storage-driver": "overlay2"
}
EOF

# 重启Docker服务
echo "重启Docker服务..."
sudo systemctl daemon-reload
sudo systemctl restart docker

echo "Docker镜像加速配置完成！"
echo "等待Docker服务启动..."
sleep 5

# 验证配置
docker info | grep -A 5 "Registry Mirrors"