package main

import (
	"fmt"
	"log"
	"net/http"
	"time"

	"github.com/gin-gonic/gin"
	"github.com/gin-contrib/cors"
	"github.com/sirupsen/logrus"

	"robot-cloud-platform/internal/config"
)

func main() {
	// 加载配置
	cfg, err := config.LoadConfig()
	if err != nil {
		log.Fatalf("Failed to load config: %v", err)
	}

	// 设置日志
	setupLogger(cfg.Logger)

	logrus.Info("Starting Robot Cloud Platform Go API Gateway")
	logrus.WithFields(logrus.Fields{
		"version": "1.0.0",
		"port":    cfg.Server.Port,
		"mode":    cfg.Server.Mode,
	}).Info("Configuration loaded")

	// 设置Gin模式
	gin.SetMode(cfg.Server.Mode)

	// 创建Gin引擎
	r := gin.New()

	// 中间件
	r.Use(gin.Logger())
	r.Use(gin.Recovery())
	
	// 加载HTML模板
	r.LoadHTMLGlob("web/templates/*")
	
	// 静态文件
	r.Static("/static", "./web/static")

	// CORS配置
	corsConfig := cors.DefaultConfig()
	corsConfig.AllowOrigins = []string{"*"}
	corsConfig.AllowMethods = []string{"GET", "POST", "PUT", "DELETE", "OPTIONS"}
	corsConfig.AllowHeaders = []string{"*"}
	r.Use(cors.New(corsConfig))

	// 路由设置
	setupRoutes(r)

	// 启动服务器
	serverAddr := fmt.Sprintf("%s:%d", cfg.Server.Host, cfg.Server.Port)
	
	server := &http.Server{
		Addr:         serverAddr,
		Handler:      r,
		ReadTimeout:  time.Duration(cfg.Server.ReadTimeout) * time.Second,
		WriteTimeout: time.Duration(cfg.Server.WriteTimeout) * time.Second,
	}

	logrus.WithField("address", serverAddr).Info("Starting HTTP server")
	
	if err := server.ListenAndServe(); err != nil && err != http.ErrServerClosed {
		logrus.WithError(err).Fatal("Failed to start server")
	}
}

// setupLogger 配置日志系统
func setupLogger(logConfig config.LoggerConfig) {
	// 设置日志级别
	level, err := logrus.ParseLevel(logConfig.Level)
	if err != nil {
		logrus.Warn("Invalid log level, using info")
		level = logrus.InfoLevel
	}
	logrus.SetLevel(level)

	// 设置日志格式
	if logConfig.Format == "json" {
		logrus.SetFormatter(&logrus.JSONFormatter{
			TimestampFormat: time.RFC3339,
		})
	} else {
		logrus.SetFormatter(&logrus.TextFormatter{
			TimestampFormat: time.RFC3339,
			FullTimestamp:   true,
		})
	}
}

// setupRoutes 设置路由
func setupRoutes(r *gin.Engine) {
	// 健康检查
	r.GET("/health", func(c *gin.Context) {
		c.JSON(http.StatusOK, gin.H{
			"status":    "healthy",
			"timestamp": time.Now().Format(time.RFC3339),
			"service":   "Robot Cloud Platform Go API Gateway",
			"version":   "1.0.0",
		})
	})

	// 根路径 - 渲染前端页面
	r.GET("/", func(c *gin.Context) {
		c.HTML(http.StatusOK, "index.html", gin.H{
			"title": "Robot Cloud Platform - Go Edition",
		})
	})
	
	// API信息端点
	r.GET("/api", func(c *gin.Context) {
		c.JSON(http.StatusOK, gin.H{
			"service":   "Robot Cloud Platform API Gateway",
			"status":    "running",
			"timestamp": time.Now().Format(time.RFC3339),
			"version":   "1.0.0",
			"language":  "Go",
			"endpoints": gin.H{
				"health":    "/health",
				"dashboard": "/api/v1/dashboard/stats",
				"robots":    "/api/v1/robots",
				"telemetry": "/api/v1/telemetry/latest",
			},
		})
	})

	// API v1 路由组
	v1 := r.Group("/api/v1")
	{
		// 仪表板统计
		v1.GET("/dashboard/stats", func(c *gin.Context) {
			c.JSON(http.StatusOK, gin.H{
				"robots_online":     1,
				"robots_offline":    0,
				"total_messages":    100,
				"system_health":     0.95,
				"average_latency":   25.5,
				"last_updated":      time.Now().Format(time.RFC3339),
			})
		})

		// 机器人列表
		v1.GET("/robots", func(c *gin.Context) {
			c.JSON(http.StatusOK, []gin.H{
				{
					"robot_id":  "go_test_robot_001",
					"name":      "Go Test Robot",
					"type":      "test",
					"status":    "online",
					"battery":   85.0,
					"last_seen": time.Now().Format(time.RFC3339),
					"position":  gin.H{"x": 10.2, "y": 5.3, "z": 0},
				},
			})
		})

		// 最新遥测数据
		v1.GET("/telemetry/latest", func(c *gin.Context) {
			c.JSON(http.StatusOK, []gin.H{
				{
					"robot_id":  "go_test_robot_001",
					"data": gin.H{
						"battery":  85.0,
						"position": gin.H{"x": 10.2, "y": 5.3, "z": 0},
						"velocity": gin.H{"linear": 0.5, "angular": 0.1},
						"status":   "active",
					},
					"timestamp": time.Now().Format(time.RFC3339),
				},
			})
		})

		// 接收遥测数据
		v1.POST("/robots/:id/telemetry", func(c *gin.Context) {
			robotID := c.Param("id")
			
			var data map[string]interface{}
			if err := c.ShouldBindJSON(&data); err != nil {
				c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
				return
			}

			logrus.WithFields(logrus.Fields{
				"robot_id": robotID,
				"data_keys": getMapKeys(data),
			}).Info("Received telemetry data")

			c.JSON(http.StatusOK, gin.H{
				"status":    "accepted",
				"robot_id":  robotID,
				"timestamp": time.Now().Format(time.RFC3339),
			})
		})
	}
}

// getMapKeys 获取map的所有键
func getMapKeys(m map[string]interface{}) []string {
	keys := make([]string, 0, len(m))
	for k := range m {
		keys = append(keys, k)
	}
	return keys
}