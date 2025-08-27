#!/bin/bash

set -e

echo "========================================="
echo "Robot Cloud Platform - K3s Setup Script"
echo "========================================="

INSTALL_K3S_VERSION=${K3S_VERSION:-"v1.28.3+k3s1"}

check_requirements() {
    echo "Checking system requirements..."
    
    if ! command -v curl &> /dev/null; then
        echo "curl is required but not installed. Installing..."
        sudo apt-get update && sudo apt-get install -y curl
    fi
    
    if ! command -v docker &> /dev/null; then
        echo "Docker is required but not installed. Please install Docker first."
        exit 1
    fi
    
    echo "✓ All requirements met"
}

install_k3s() {
    echo "Installing K3s ${INSTALL_K3S_VERSION}..."
    
    curl -sfL https://get.k3s.io | INSTALL_K3S_VERSION=$INSTALL_K3S_VERSION sh -s - \
        --write-kubeconfig-mode 644 \
        --disable traefik \
        --docker
    
    mkdir -p $HOME/.kube
    sudo cp /etc/rancher/k3s/k3s.yaml $HOME/.kube/config
    sudo chown $(id -u):$(id -g) $HOME/.kube/config
    
    echo "✓ K3s installed successfully"
}

wait_for_k3s() {
    echo "Waiting for K3s to be ready..."
    while ! kubectl get nodes &> /dev/null; do
        echo -n "."
        sleep 2
    done
    echo ""
    
    kubectl wait --for=condition=Ready node --all --timeout=300s
    echo "✓ K3s is ready"
}

install_helm() {
    echo "Installing Helm..."
    
    curl https://raw.githubusercontent.com/helm/helm/main/scripts/get-helm-3 | bash
    
    echo "✓ Helm installed successfully"
}

setup_storage() {
    echo "Setting up local storage class..."
    
    cat <<EOF | kubectl apply -f -
apiVersion: storage.k8s.io/v1
kind: StorageClass
metadata:
  name: local-path
  annotations:
    storageclass.kubernetes.io/is-default-class: "true"
provisioner: rancher.io/local-path
volumeBindingMode: WaitForFirstConsumer
reclaimPolicy: Delete
EOF
    
    echo "✓ Storage class configured"
}

deploy_platform() {
    echo "Deploying Robot Cloud Platform base components..."
    
    kubectl apply -k infrastructure/kubernetes/base/
    
    echo "✓ Platform base components deployed"
}

print_info() {
    echo ""
    echo "========================================="
    echo "K3s Setup Complete!"
    echo "========================================="
    echo ""
    echo "Cluster Info:"
    kubectl cluster-info
    echo ""
    echo "Nodes:"
    kubectl get nodes
    echo ""
    echo "To access the cluster:"
    echo "  export KUBECONFIG=$HOME/.kube/config"
    echo ""
    echo "To deploy the platform:"
    echo "  kubectl apply -k infrastructure/kubernetes/base/"
    echo ""
    echo "To start local development services:"
    echo "  docker-compose up -d"
    echo ""
}

main() {
    check_requirements
    
    if kubectl version --client &> /dev/null; then
        echo "kubectl is already installed"
    else
        echo "Installing kubectl..."
        curl -LO "https://dl.k8s.io/release/$(curl -L -s https://dl.k8s.io/release/stable.txt)/bin/linux/amd64/kubectl"
        chmod +x kubectl
        sudo mv kubectl /usr/local/bin/
    fi
    
    if systemctl is-active --quiet k3s; then
        echo "K3s is already running"
    else
        install_k3s
    fi
    
    wait_for_k3s
    
    if command -v helm &> /dev/null; then
        echo "Helm is already installed"
    else
        install_helm
    fi
    
    setup_storage
    
    if [ "$1" == "--deploy" ]; then
        deploy_platform
    fi
    
    print_info
}

main "$@"