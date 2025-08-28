# Go API Gateway

## Build Instructions

To build the Go API Gateway service:

```bash
# Navigate to the go-api-gateway directory
cd services/go-api-gateway

# Build the binary
go build -o robot-cloud-go cmd/server/main.go

# Or use go run for development
go run cmd/server/main.go
```

## Running the Service

```bash
# Run the compiled binary
./robot-cloud-go

# With custom port
SERVER_PORT=8001 ./robot-cloud-go
```

## Note

The compiled binary `robot-cloud-go` is excluded from version control. Please build it locally after cloning the repository.