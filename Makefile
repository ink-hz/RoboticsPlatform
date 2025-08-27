.PHONY: help build deploy clean test setup

help:
	@echo "Robot Cloud Platform - Available Commands:"
	@echo "  make setup      - Install dependencies and setup environment"
	@echo "  make build      - Build all services"
	@echo "  make deploy     - Deploy to Kubernetes"
	@echo "  make dev        - Start development environment"
	@echo "  make test       - Run tests"
	@echo "  make clean      - Clean build artifacts"
	@echo "  make logs       - Show platform logs"
	@echo "  make monitor    - Open monitoring dashboards"

setup:
	@echo "Setting up development environment..."
	@chmod +x scripts/setup-k3s.sh
	@./scripts/setup-k3s.sh
	@docker-compose pull
	@echo "Setup complete!"

build-api-gateway:
	@echo "Building API Gateway..."
	@docker build -t rcp/api-gateway:latest services/api-gateway/

build-edge-controller:
	@echo "Building Edge Controller..."
	@cd edge/controllers && go build -o edge-controller .

build-data-pipeline:
	@echo "Building Data Pipeline..."
	@docker build -t rcp/data-pipeline:latest platform/data-pipeline/

build: build-api-gateway build-edge-controller build-data-pipeline
	@echo "All services built successfully!"

dev:
	@echo "Starting development environment..."
	@docker-compose up -d
	@echo "Development environment started!"
	@echo "Services available at:"
	@echo "  - Grafana: http://localhost:3000 (admin/admin)"
	@echo "  - Prometheus: http://localhost:9090"
	@echo "  - MinIO Console: http://localhost:9001 (minioadmin/minioadmin)"
	@echo "  - PostgreSQL: localhost:5432"
	@echo "  - TimescaleDB: localhost:5433"
	@echo "  - Redis: localhost:6379"
	@echo "  - Kafka: localhost:9092"

dev-down:
	@echo "Stopping development environment..."
	@docker-compose down
	@echo "Development environment stopped!"

deploy:
	@echo "Deploying to Kubernetes..."
	@kubectl apply -k infrastructure/kubernetes/base/
	@echo "Deployment complete!"

deploy-monitoring:
	@echo "Deploying monitoring stack..."
	@kubectl apply -f infrastructure/monitoring/
	@echo "Monitoring stack deployed!"

clean:
	@echo "Cleaning build artifacts..."
	@rm -rf edge/controllers/edge-controller
	@docker-compose down -v
	@echo "Clean complete!"

logs:
	@docker-compose logs -f

logs-k8s:
	@kubectl logs -n robotics-platform -f deployment/api-gateway

test-api:
	@echo "Testing API Gateway..."
	@cd services/api-gateway && python -m pytest tests/

test-edge:
	@echo "Testing Edge Controller..."
	@cd edge/controllers && go test ./...

test: test-api test-edge
	@echo "All tests complete!"

monitor:
	@echo "Opening monitoring dashboards..."
	@echo "Grafana: http://localhost:3000"
	@xdg-open http://localhost:3000 2>/dev/null || open http://localhost:3000 2>/dev/null || echo "Please open http://localhost:3000 in your browser"

status:
	@echo "Checking platform status..."
	@docker-compose ps
	@echo ""
	@kubectl get pods -n robotics-platform 2>/dev/null || echo "Kubernetes not running"

port-forward:
	@echo "Setting up port forwarding for Kubernetes services..."
	@kubectl port-forward -n robotics-platform svc/api-gateway 8080:80 &
	@kubectl port-forward -n robotics-platform svc/prometheus 9090:9090 &
	@kubectl port-forward -n robotics-platform svc/grafana 3000:3000 &
	@echo "Port forwarding established!"

init-models:
	@echo "Initializing ML model registry..."
	@mkdir -p ml-ops/models
	@echo "Model registry initialized!"

backup:
	@echo "Backing up platform data..."
	@mkdir -p backups/$(shell date +%Y%m%d)
	@docker-compose exec postgres pg_dump -U admin robotics > backups/$(shell date +%Y%m%d)/postgres.sql
	@docker-compose exec timescaledb pg_dump -U tsdb telemetry > backups/$(shell date +%Y%m%d)/timescale.sql
	@echo "Backup complete!"

restore:
	@echo "Restoring platform data from backup..."
	@echo "Not implemented yet"