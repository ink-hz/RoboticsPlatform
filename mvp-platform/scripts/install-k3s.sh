#!/bin/bash

# K3s安装脚本
set -e

echo "========================================"
echo "Installing K3s (Lightweight Kubernetes)"
echo "========================================"

# 检查是否已安装
if command -v k3s &> /dev/null; then
    echo "K3s is already installed"
    k3s --version
else
    echo "Installing K3s..."
    curl -sfL https://get.k3s.io | INSTALL_K3S_EXEC="--docker" sh -
    
    # 配置kubectl
    mkdir -p ~/.kube
    sudo cp /etc/rancher/k3s/k3s.yaml ~/.kube/config
    sudo chown $(id -u):$(id -g) ~/.kube/config
    
    # 设置环境变量
    echo 'export KUBECONFIG=~/.kube/config' >> ~/.bashrc
    echo 'export PATH=$PATH:/usr/local/bin' >> ~/.bashrc
fi

# 安装kubectl如果未安装
if ! command -v kubectl &> /dev/null; then
    echo "Installing kubectl..."
    curl -LO "https://dl.k8s.io/release/$(curl -L -s https://dl.k8s.io/release/stable.txt)/bin/linux/amd64/kubectl"
    chmod +x kubectl
    sudo mv kubectl /usr/local/bin/
fi

# 验证安装
echo ""
echo "Verifying installation..."
kubectl version --client
k3s kubectl get nodes

echo ""
echo "K3s installation completed!"
echo "========================================"