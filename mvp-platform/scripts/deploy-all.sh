#!/bin/bash

# MVP平台一键部署脚本
set -e

echo "========================================"
echo "       MVP Platform Deployment          "
echo "========================================"

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 函数定义
log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 检查Docker
check_docker() {
    log_info "Checking Docker installation..."
    if ! command -v docker &> /dev/null; then
        log_error "Docker is not installed. Please install Docker first."
        exit 1
    fi
    
    if ! docker compose version &> /dev/null; then
        log_error "Docker Compose is not available. Please install Docker Compose first."
        exit 1
    fi
    
    log_info "Docker check passed"
}

# 清理旧容器
cleanup() {
    log_info "Cleaning up old containers..."
    docker compose down -v --remove-orphans || true
    docker system prune -f || true
}

# 构建镜像
build_images() {
    log_info "Building application images..."
    docker compose build --no-cache
}

# 启动服务
start_services() {
    log_info "Starting all services..."
    docker compose up -d
    
    log_info "Waiting for services to be ready..."
    sleep 30
}

# 健康检查
health_check() {
    log_info "Performing health checks..."
    
    # 检查PostgreSQL
    log_info "Checking PostgreSQL..."
    docker exec mvp-postgres pg_isready -h localhost -U admin
    
    # 检查Redis
    log_info "Checking Redis..."
    docker exec mvp-redis redis-cli ping
    
    # 检查Kafka
    log_info "Checking Kafka..."
    sleep 10  # Wait for Kafka to fully start
    docker exec mvp-kafka kafka-topics --bootstrap-server localhost:9092 --list
    
    # 检查服务健康状态
    services=("auth-service:8081" "data-service:8082")
    
    for service in "${services[@]}"; do
        IFS=':' read -r name port <<< "$service"
        log_info "Checking $name on port $port..."
        
        max_attempts=30
        attempt=1
        
        while [ $attempt -le $max_attempts ]; do
            if curl -f "http://localhost:$port/health" > /dev/null 2>&1; then
                log_info "$name is healthy"
                break
            else
                if [ $attempt -eq $max_attempts ]; then
                    log_warn "$name health check failed after $max_attempts attempts"
                else
                    echo -n "."
                    sleep 2
                    ((attempt++))
                fi
            fi
        done
    done
}

# 初始化数据
initialize_data() {
    log_info "Initializing sample data..."
    
    # 创建Kafka topics
    docker exec mvp-kafka kafka-topics --bootstrap-server localhost:9092 \
        --create --topic data-ingestion --partitions 3 --replication-factor 1 || true
    
    docker exec mvp-kafka kafka-topics --bootstrap-server localhost:9092 \
        --create --topic processed-data --partitions 3 --replication-factor 1 || true
    
    docker exec mvp-kafka kafka-topics --bootstrap-server localhost:9092 \
        --create --topic anomaly-detection --partitions 3 --replication-factor 1 || true
    
    log_info "Kafka topics created"
}

# 显示访问信息
show_access_info() {
    echo ""
    log_info "========================================"
    log_info "         Deployment Complete!          "
    log_info "========================================"
    echo ""
    log_info "Service Access Information:"
    echo ""
    echo "🔐 Authentication Service:  http://localhost:8081"
    echo "📊 Data Service:             http://localhost:8082"  
    echo "📈 Grafana Dashboard:        http://localhost:3000 (admin/admin123)"
    echo "⚡ Prometheus Metrics:       http://localhost:9090"
    echo "🔍 Kafka UI:                http://localhost:8080"
    echo "🚀 Flink Dashboard:          http://localhost:8083"
    echo "🌐 API Gateway:              http://localhost:8000"
    echo ""
    log_info "Database Access:"
    echo "🗄️  PostgreSQL:              localhost:5432 (admin/admin123)"
    echo "💾 Redis:                   localhost:6379"
    echo "📨 Kafka:                   localhost:9092"
    echo ""
    log_info "To check logs: docker compose logs -f [service-name]"
    log_info "To stop all services: docker compose down"
    echo ""
}

# 主函数
main() {
    # 切换到项目目录
    cd "$(dirname "$0")/.."
    
    # 检查环境
    check_docker
    
    # 执行部署
    if [ "$1" == "--clean" ]; then
        cleanup
    fi
    
    build_images
    start_services
    health_check
    initialize_data
    show_access_info
    
    log_info "Deployment script completed successfully!"
}

# 错误处理
trap 'log_error "Deployment failed! Check the logs above for details."; exit 1' ERR

# 执行主函数
main "$@"