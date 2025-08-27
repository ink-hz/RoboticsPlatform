from fastapi import FastAPI, HTTPException, BackgroundTasks, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from fastapi.responses import HTMLResponse
from fastapi.templating import Jinja2Templates
from prometheus_client import Counter, Histogram, generate_latest, REGISTRY, CollectorRegistry
from typing import Dict, Any, Optional
import uvicorn
import httpx
import asyncio
import os
from datetime import datetime
import redis
import json

app = FastAPI(title="Robot Cloud Platform API Gateway", version="0.1.0")

# 静态文件和模板
templates = Jinja2Templates(directory="../../frontend/templates")
app.mount("/static", StaticFiles(directory="../../frontend/static"), name="static")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# 内存存储，替代Redis（用于演示）
class MemoryStore:
    def __init__(self):
        self.data = {}
        self.hash_data = {}
    
    def keys(self, pattern="*"):
        if pattern == "*":
            return list(self.data.keys())
        import fnmatch
        return [k for k in self.data.keys() if fnmatch.fnmatch(k, pattern)]
    
    def get(self, key):
        return self.data.get(key)
    
    def set(self, key, value):
        self.data[key] = value
    
    def setex(self, key, time, value):
        self.data[key] = value
    
    def hget(self, key, field):
        return self.hash_data.get(key, {}).get(field)
    
    def hgetall(self, key):
        return self.hash_data.get(key, {})
    
    def hset(self, key, field, value):
        if key not in self.hash_data:
            self.hash_data[key] = {}
        self.hash_data[key][field] = value

# 尝试连接Redis，失败则使用内存存储
try:
    redis_client = redis.Redis(
        host=os.getenv("REDIS_HOST", "localhost"),
        port=int(os.getenv("REDIS_PORT", 6379)),
        decode_responses=True,
        socket_connect_timeout=2
    )
    # 测试连接
    redis_client.ping()
    print("Connected to Redis successfully")
except:
    print("Redis not available, using in-memory storage")
    redis_client = MemoryStore()

try:
    request_counter = Counter('api_requests_total', 'Total API requests', ['method', 'endpoint', 'status'])
    request_duration = Histogram('api_request_duration_seconds', 'API request duration', ['method', 'endpoint'])
except ValueError:
    # Metrics already registered
    from prometheus_client import REGISTRY
    request_counter = REGISTRY._names_to_collectors.get('api_requests_total')
    request_duration = REGISTRY._names_to_collectors.get('api_request_duration_seconds')

SERVICES = {
    "auth": os.getenv("AUTH_SERVICE", "http://auth-service:8001"),
    "robot": os.getenv("ROBOT_SERVICE", "http://robot-connector:8002"),
    "ml": os.getenv("ML_SERVICE", "http://ml-serving:8003"),
    "data": os.getenv("DATA_SERVICE", "http://data-pipeline:8004"),
    "edge": os.getenv("EDGE_SERVICE", "http://edge-controller:8005")
}

@app.get("/", response_class=HTMLResponse)
async def dashboard(request: Request):
    """机器人云平台控制台"""
    return templates.TemplateResponse("index.html", {"request": request})

@app.get("/api")
async def api_root():
    return {
        "service": "Robot Cloud Platform API Gateway",
        "status": "running",
        "timestamp": datetime.utcnow().isoformat(),
        "version": "0.1.0"
    }

@app.get("/health")
async def health_check():
    services_status = {}
    async with httpx.AsyncClient() as client:
        for service_name, service_url in SERVICES.items():
            try:
                response = await client.get(f"{service_url}/health", timeout=2.0)
                services_status[service_name] = "healthy" if response.status_code == 200 else "unhealthy"
            except:
                services_status[service_name] = "unreachable"
    
    all_healthy = all(status == "healthy" for status in services_status.values())
    return {
        "status": "healthy" if all_healthy else "degraded",
        "services": services_status,
        "timestamp": datetime.utcnow().isoformat()
    }

@app.get("/metrics")
async def metrics():
    from starlette.responses import PlainTextResponse
    return PlainTextResponse(generate_latest(), media_type="text/plain")

@app.get("/api/v1/dashboard/stats")
async def get_dashboard_stats():
    """获取仪表板统计数据"""
    # 获取真实的机器人状态统计
    try:
        robots_online = len([k for k in redis_client.keys("robot:status:*") 
                           if redis_client.hget(k, "status") == "online"])
        robots_offline = len([k for k in redis_client.keys("robot:status:*") 
                            if redis_client.hget(k, "status") == "offline"])
        
        # 获取今日消息数量
        today_messages = redis_client.get("stats:messages:today") or 0
        
        # 计算系统健康度（基于服务状态）
        services_up = 0
        total_services = len(SERVICES)
        
        async with httpx.AsyncClient() as client:
            for service_name, service_url in SERVICES.items():
                try:
                    response = await client.get(f"{service_url}/health", timeout=1.0)
                    if response.status_code == 200:
                        services_up += 1
                except:
                    pass
        
        system_health = int((services_up / total_services * 100)) if total_services > 0 else 100
        
        return {
            "onlineRobots": robots_online,
            "offlineRobots": robots_offline,
            "todayMessages": int(today_messages),
            "systemHealth": system_health,
            "timestamp": datetime.utcnow().isoformat(),
            "storage": "memory" if isinstance(redis_client, MemoryStore) else "redis"
        }
    except Exception as e:
        # 返回默认值如果无法获取实际数据
        return {
            "onlineRobots": 0,
            "offlineRobots": 0,
            "todayMessages": 0,
            "systemHealth": 50,
            "timestamp": datetime.utcnow().isoformat(),
            "error": "Unable to fetch real data"
        }

@app.get("/api/v1/robots")
async def list_robots():
    """获取机器人列表"""
    try:
        robots = []
        robot_keys = redis_client.keys("robot:status:*")
        
        for key in robot_keys:
            robot_id = key.split(":")[-1]
            robot_data = redis_client.hgetall(key)
            
            if robot_data:
                robots.append({
                    "id": robot_id,
                    "status": robot_data.get("status", "unknown"),
                    "battery": float(robot_data.get("battery", 0)),
                    "location": robot_data.get("location", "未知"),
                    "lastSeen": robot_data.get("last_seen", "未知"),
                    "activeTask": robot_data.get("active_task", "无")
                })
        
        return {"robots": robots, "total": len(robots)}
    except Exception as e:
        return {"robots": [], "total": 0, "error": str(e)}

@app.get("/api/v1/telemetry/latest")
async def get_latest_telemetry():
    """获取最新遥测数据"""
    try:
        telemetry_data = []
        telemetry_keys = redis_client.keys("telemetry:*:latest")
        
        for key in telemetry_keys:
            data = redis_client.get(key)
            if data:
                import json
                parsed_data = json.loads(data)
                telemetry_data.append({
                    "robotId": parsed_data.get("robot_id"),
                    "timestamp": parsed_data.get("timestamp"),
                    "type": "遥测数据",
                    "data": parsed_data.get("data", {})
                })
        
        # 按时间戳排序，最新的在前
        telemetry_data.sort(key=lambda x: x["timestamp"], reverse=True)
        
        return {"telemetry": telemetry_data[:10]}  # 返回最新10条
    except Exception as e:
        return {"telemetry": [], "error": str(e)}

@app.post("/api/v1/robots/{robot_id}/telemetry")
async def receive_telemetry(robot_id: str, data: Dict[str, Any], background_tasks: BackgroundTasks):
    """接收机器人遥测数据"""
    request_counter.labels(method="POST", endpoint="/telemetry", status="200").inc()
    
    telemetry_data = {
        "robot_id": robot_id,
        "timestamp": datetime.utcnow().isoformat(),
        "data": data
    }
    
    background_tasks.add_task(process_telemetry, telemetry_data)
    
    redis_client.setex(
        f"telemetry:{robot_id}:latest",
        300,
        json.dumps(telemetry_data)
    )
    
    return {"status": "accepted", "robot_id": robot_id}

async def process_telemetry(data: Dict[str, Any]):
    """后台处理遥测数据"""
    try:
        # 跳过HTTP转发，直接在内存中处理
        print(f"Processing telemetry data for robot {data.get('robot_id', 'unknown')}")
        
        # 更新消息计数
        current_count = redis_client.get("stats:messages:today") or 0
        redis_client.set("stats:messages:today", int(current_count) + 1)
        
    except Exception as e:
        print(f"Failed to process telemetry: {e}")

@app.post("/api/v1/robots/{robot_id}/commands")
async def send_command(robot_id: str, command: Dict[str, Any]):
    """发送命令到机器人"""
    request_counter.labels(method="POST", endpoint="/commands", status="200").inc()
    
    command_data = {
        "robot_id": robot_id,
        "command": command,
        "timestamp": datetime.utcnow().isoformat()
    }
    
    async with httpx.AsyncClient() as client:
        try:
            response = await client.post(
                f"{SERVICES['robot']}/execute",
                json=command_data,
                timeout=10.0
            )
            return response.json()
        except httpx.TimeoutException:
            raise HTTPException(status_code=504, detail="Command execution timeout")
        except Exception as e:
            raise HTTPException(status_code=500, detail=str(e))

@app.get("/api/v1/models")
async def list_models():
    """列出可用的AI模型"""
    async with httpx.AsyncClient() as client:
        try:
            response = await client.get(f"{SERVICES['ml']}/models")
            return response.json()
        except Exception as e:
            raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/v1/models/{model_id}/deploy")
async def deploy_model(model_id: str, target: str = "edge"):
    """部署模型到边缘或云端"""
    deployment_request = {
        "model_id": model_id,
        "target": target,
        "timestamp": datetime.utcnow().isoformat()
    }
    
    async with httpx.AsyncClient() as client:
        try:
            response = await client.post(
                f"{SERVICES['ml']}/deploy",
                json=deployment_request,
                timeout=30.0
            )
            return response.json()
        except Exception as e:
            raise HTTPException(status_code=500, detail=str(e))

@app.get("/api/v1/edge/nodes")
async def list_edge_nodes():
    """列出边缘节点状态"""
    async with httpx.AsyncClient() as client:
        try:
            response = await client.get(f"{SERVICES['edge']}/nodes")
            return response.json()
        except Exception as e:
            raise HTTPException(status_code=500, detail=str(e))

@app.websocket("/ws/robots/{robot_id}")
async def robot_websocket(websocket, robot_id: str):
    """机器人实时通信WebSocket"""
    await websocket.accept()
    try:
        while True:
            data = await websocket.receive_text()
            
            await websocket.send_text(f"Acknowledged: {data}")
            
            telemetry = redis_client.get(f"telemetry:{robot_id}:latest")
            if telemetry:
                await websocket.send_text(telemetry)
            
            await asyncio.sleep(0.1)
    except Exception as e:
        print(f"WebSocket error: {e}")
    finally:
        await websocket.close()

if __name__ == "__main__":
    uvicorn.run(
        "main:app",
        host="0.0.0.0",
        port=8000,
        reload=True,
        log_level="info"
    )