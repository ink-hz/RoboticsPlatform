package main

import (
	"context"
	"encoding/json"
	"fmt"
	"log"
	"net/http"
	"os"
	"sync"
	"time"

	"github.com/gorilla/mux"
	"github.com/prometheus/client_golang/prometheus"
	"github.com/prometheus/client_golang/prometheus/promhttp"
	"github.com/redis/go-redis/v9"
)

type EdgeNode struct {
	ID           string    `json:"id"`
	Name         string    `json:"name"`
	IP           string    `json:"ip"`
	Status       string    `json:"status"`
	LastSeen     time.Time `json:"last_seen"`
	Resources    Resources `json:"resources"`
	DeployedModels []string `json:"deployed_models"`
}

type Resources struct {
	CPUCores    int     `json:"cpu_cores"`
	MemoryGB    float64 `json:"memory_gb"`
	GPUMemoryGB float64 `json:"gpu_memory_gb,omitempty"`
	DiskGB      float64 `json:"disk_gb"`
	CPUUsage    float64 `json:"cpu_usage"`
	MemoryUsage float64 `json:"memory_usage"`
}

type ModelDeployment struct {
	ModelID     string            `json:"model_id"`
	Version     string            `json:"version"`
	NodeID      string            `json:"node_id"`
	Status      string            `json:"status"`
	Config      map[string]interface{} `json:"config"`
	DeployedAt  time.Time         `json:"deployed_at"`
}

type EdgeController struct {
	nodes       map[string]*EdgeNode
	deployments map[string]*ModelDeployment
	mu          sync.RWMutex
	redisClient *redis.Client
	metrics     *EdgeMetrics
}

type EdgeMetrics struct {
	nodesTotal      prometheus.Gauge
	deploymentsTotal prometheus.Gauge
	nodesCPUUsage   *prometheus.GaugeVec
	nodesMemUsage   *prometheus.GaugeVec
}

func NewEdgeController() *EdgeController {
	redisClient := redis.NewClient(&redis.Options{
		Addr:     getEnv("REDIS_ADDR", "localhost:6379"),
		Password: getEnv("REDIS_PASSWORD", ""),
		DB:       0,
	})

	metrics := &EdgeMetrics{
		nodesTotal: prometheus.NewGauge(prometheus.GaugeOpts{
			Name: "edge_nodes_total",
			Help: "Total number of edge nodes",
		}),
		deploymentsTotal: prometheus.NewGauge(prometheus.GaugeOpts{
			Name: "edge_deployments_total",
			Help: "Total number of model deployments",
		}),
		nodesCPUUsage: prometheus.NewGaugeVec(prometheus.GaugeOpts{
			Name: "edge_node_cpu_usage",
			Help: "CPU usage of edge nodes",
		}, []string{"node_id"}),
		nodesMemUsage: prometheus.NewGaugeVec(prometheus.GaugeOpts{
			Name: "edge_node_memory_usage",
			Help: "Memory usage of edge nodes",
		}, []string{"node_id"}),
	}

	prometheus.MustRegister(metrics.nodesTotal)
	prometheus.MustRegister(metrics.deploymentsTotal)
	prometheus.MustRegister(metrics.nodesCPUUsage)
	prometheus.MustRegister(metrics.nodesMemUsage)

	return &EdgeController{
		nodes:       make(map[string]*EdgeNode),
		deployments: make(map[string]*ModelDeployment),
		redisClient: redisClient,
		metrics:     metrics,
	}
}

func (ec *EdgeController) RegisterNode(w http.ResponseWriter, r *http.Request) {
	var node EdgeNode
	if err := json.NewDecoder(r.Body).Decode(&node); err != nil {
		http.Error(w, err.Error(), http.StatusBadRequest)
		return
	}

	node.LastSeen = time.Now()
	node.Status = "active"

	ec.mu.Lock()
	ec.nodes[node.ID] = &node
	ec.mu.Unlock()

	ctx := context.Background()
	nodeJSON, _ := json.Marshal(node)
	ec.redisClient.Set(ctx, fmt.Sprintf("edge:node:%s", node.ID), nodeJSON, 5*time.Minute)

	ec.metrics.nodesTotal.Set(float64(len(ec.nodes)))
	ec.metrics.nodesCPUUsage.WithLabelValues(node.ID).Set(node.Resources.CPUUsage)
	ec.metrics.nodesMemUsage.WithLabelValues(node.ID).Set(node.Resources.MemoryUsage)

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(map[string]string{
		"status": "registered",
		"node_id": node.ID,
	})
}

func (ec *EdgeController) ListNodes(w http.ResponseWriter, r *http.Request) {
	ec.mu.RLock()
	nodes := make([]*EdgeNode, 0, len(ec.nodes))
	for _, node := range ec.nodes {
		nodes = append(nodes, node)
	}
	ec.mu.RUnlock()

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(nodes)
}

func (ec *EdgeController) UpdateNodeStatus(w http.ResponseWriter, r *http.Request) {
	vars := mux.Vars(r)
	nodeID := vars["node_id"]

	var update struct {
		Status    string    `json:"status"`
		Resources Resources `json:"resources"`
	}

	if err := json.NewDecoder(r.Body).Decode(&update); err != nil {
		http.Error(w, err.Error(), http.StatusBadRequest)
		return
	}

	ec.mu.Lock()
	if node, exists := ec.nodes[nodeID]; exists {
		node.Status = update.Status
		node.Resources = update.Resources
		node.LastSeen = time.Now()
		
		ec.metrics.nodesCPUUsage.WithLabelValues(nodeID).Set(update.Resources.CPUUsage)
		ec.metrics.nodesMemUsage.WithLabelValues(nodeID).Set(update.Resources.MemoryUsage)
	}
	ec.mu.Unlock()

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(map[string]string{"status": "updated"})
}

func (ec *EdgeController) DeployModel(w http.ResponseWriter, r *http.Request) {
	var deployment ModelDeployment
	if err := json.NewDecoder(r.Body).Decode(&deployment); err != nil {
		http.Error(w, err.Error(), http.StatusBadRequest)
		return
	}

	ec.mu.RLock()
	node, nodeExists := ec.nodes[deployment.NodeID]
	ec.mu.RUnlock()

	if !nodeExists {
		http.Error(w, "Node not found", http.StatusNotFound)
		return
	}

	deployment.DeployedAt = time.Now()
	deployment.Status = "deploying"

	ec.mu.Lock()
	deploymentID := fmt.Sprintf("%s-%s-%d", deployment.ModelID, deployment.NodeID, time.Now().Unix())
	ec.deployments[deploymentID] = &deployment
	
	node.DeployedModels = append(node.DeployedModels, deployment.ModelID)
	ec.mu.Unlock()

	ec.metrics.deploymentsTotal.Set(float64(len(ec.deployments)))

	go ec.executeDeployment(deploymentID, &deployment)

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(map[string]string{
		"deployment_id": deploymentID,
		"status": "initiated",
	})
}

func (ec *EdgeController) executeDeployment(deploymentID string, deployment *ModelDeployment) {
	time.Sleep(5 * time.Second)

	ec.mu.Lock()
	if dep, exists := ec.deployments[deploymentID]; exists {
		dep.Status = "deployed"
	}
	ec.mu.Unlock()

	log.Printf("Model %s deployed to node %s", deployment.ModelID, deployment.NodeID)
}

func (ec *EdgeController) GetDeploymentStatus(w http.ResponseWriter, r *http.Request) {
	vars := mux.Vars(r)
	deploymentID := vars["deployment_id"]

	ec.mu.RLock()
	deployment, exists := ec.deployments[deploymentID]
	ec.mu.RUnlock()

	if !exists {
		http.Error(w, "Deployment not found", http.StatusNotFound)
		return
	}

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(deployment)
}

func (ec *EdgeController) HealthCheck(w http.ResponseWriter, r *http.Request) {
	ctx := context.Background()
	
	redisStatus := "healthy"
	if err := ec.redisClient.Ping(ctx).Err(); err != nil {
		redisStatus = "unhealthy"
	}

	activeNodes := 0
	ec.mu.RLock()
	for _, node := range ec.nodes {
		if time.Since(node.LastSeen) < 5*time.Minute {
			activeNodes++
		}
	}
	totalNodes := len(ec.nodes)
	totalDeployments := len(ec.deployments)
	ec.mu.RUnlock()

	health := map[string]interface{}{
		"status": "healthy",
		"redis": redisStatus,
		"nodes": map[string]int{
			"total": totalNodes,
			"active": activeNodes,
		},
		"deployments": totalDeployments,
		"timestamp": time.Now().Format(time.RFC3339),
	}

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(health)
}

func (ec *EdgeController) MonitorNodes() {
	ticker := time.NewTicker(30 * time.Second)
	defer ticker.Stop()

	for range ticker.C {
		ec.mu.Lock()
		for id, node := range ec.nodes {
			if time.Since(node.LastSeen) > 5*time.Minute {
				node.Status = "offline"
				log.Printf("Node %s marked as offline", id)
			}
		}
		ec.mu.Unlock()
	}
}

func getEnv(key, defaultValue string) string {
	if value := os.Getenv(key); value != "" {
		return value
	}
	return defaultValue
}

func main() {
	controller := NewEdgeController()

	go controller.MonitorNodes()

	router := mux.NewRouter()

	router.HandleFunc("/health", controller.HealthCheck).Methods("GET")
	router.HandleFunc("/nodes", controller.ListNodes).Methods("GET")
	router.HandleFunc("/nodes/register", controller.RegisterNode).Methods("POST")
	router.HandleFunc("/nodes/{node_id}/status", controller.UpdateNodeStatus).Methods("PUT")
	router.HandleFunc("/deployments", controller.DeployModel).Methods("POST")
	router.HandleFunc("/deployments/{deployment_id}", controller.GetDeploymentStatus).Methods("GET")
	
	router.Handle("/metrics", promhttp.Handler())

	port := getEnv("PORT", "8005")
	log.Printf("Edge Controller starting on port %s", port)
	
	if err := http.ListenAndServe(":"+port, router); err != nil {
		log.Fatal("Server failed to start:", err)
	}
}