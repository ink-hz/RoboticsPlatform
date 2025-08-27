#!/usr/bin/env python3
"""
MVP平台本地模拟器 - 演示完整架构功能
"""

import threading
import time
import queue
import json
import sqlite3
import random
from datetime import datetime, timedelta
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs
import hashlib
import jwt
import logging

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(message)s')

# 模拟组件类
class InMemoryRedis:
    """内存Redis模拟"""
    def __init__(self):
        self.data = {}
        self.expiry = {}
    
    def set(self, key, value, ex=None):
        self.data[key] = value
        if ex:
            self.expiry[key] = datetime.now() + timedelta(seconds=ex)
    
    def get(self, key):
        if key in self.expiry and datetime.now() > self.expiry[key]:
            del self.data[key]
            del self.expiry[key]
            return None
        return self.data.get(key)
    
    def delete(self, key):
        self.data.pop(key, None)
        self.expiry.pop(key, None)

class InMemoryKafka:
    """内存Kafka模拟"""
    def __init__(self):
        self.topics = {}
        self.consumers = {}
    
    def create_topic(self, topic):
        if topic not in self.topics:
            self.topics[topic] = queue.Queue()
    
    def send(self, topic, message):
        self.create_topic(topic)
        self.topics[topic].put(message)
        logging.info(f"Kafka: Sent message to {topic}")
    
    def consume(self, topic, group_id):
        self.create_topic(topic)
        try:
            message = self.topics[topic].get_nowait()
            logging.info(f"Kafka: Consumed message from {topic}")
            return message
        except queue.Empty:
            return None

class DatabaseManager:
    """数据库管理器"""
    def __init__(self, db_path="mvp_demo.db"):
        self.db_path = db_path
        self.init_db()
    
    def init_db(self):
        conn = sqlite3.connect(self.db_path)
        cur = conn.cursor()
        
        # 用户表
        cur.execute('''
            CREATE TABLE IF NOT EXISTS users (
                id INTEGER PRIMARY KEY,
                username TEXT UNIQUE,
                password_hash TEXT,
                email TEXT,
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            )
        ''')
        
        # 传感器数据表
        cur.execute('''
            CREATE TABLE IF NOT EXISTS sensor_data (
                id INTEGER PRIMARY KEY,
                device_id TEXT,
                sensor_type TEXT,
                value REAL,
                timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                metadata TEXT
            )
        ''')
        
        conn.commit()
        conn.close()
    
    def execute(self, query, params=None):
        conn = sqlite3.connect(self.db_path)
        conn.row_factory = sqlite3.Row
        cur = conn.cursor()
        
        if params:
            cur.execute(query, params)
        else:
            cur.execute(query)
        
        if query.strip().upper().startswith('SELECT'):
            results = cur.fetchall()
            conn.close()
            return results
        else:
            conn.commit()
            conn.close()
            return cur.rowcount

class AuthService:
    """认证服务"""
    def __init__(self, db, redis):
        self.db = db
        self.redis = redis
        self.jwt_secret = "demo-secret-key"
    
    def hash_password(self, password):
        return hashlib.sha256(password.encode()).hexdigest()
    
    def register(self, username, password, email):
        try:
            password_hash = self.hash_password(password)
            self.db.execute(
                'INSERT INTO users (username, password_hash, email) VALUES (?, ?, ?)',
                (username, password_hash, email)
            )
            return {"success": True, "message": "User registered"}
        except Exception as e:
            return {"success": False, "error": str(e)}
    
    def login(self, username, password):
        try:
            users = self.db.execute(
                'SELECT id, password_hash FROM users WHERE username = ?',
                (username,)
            )
            
            if not users or users[0]['password_hash'] != self.hash_password(password):
                return {"success": False, "error": "Invalid credentials"}
            
            # 生成JWT
            payload = {
                'user_id': users[0]['id'],
                'username': username,
                'exp': datetime.utcnow() + timedelta(hours=1)
            }
            
            token = jwt.encode(payload, self.jwt_secret, algorithm='HS256')
            
            # 存储到Redis
            self.redis.set(f"token:{username}", token, ex=3600)
            
            return {"success": True, "token": token}
            
        except Exception as e:
            return {"success": False, "error": str(e)}
    
    def verify_token(self, token):
        try:
            payload = jwt.decode(token, self.jwt_secret, algorithms=['HS256'])
            username = payload['username']
            
            # 检查Redis中的token
            stored_token = self.redis.get(f"token:{username}")
            if stored_token != token:
                return {"success": False, "error": "Token revoked"}
            
            return {"success": True, "user": payload}
            
        except jwt.ExpiredSignatureError:
            return {"success": False, "error": "Token expired"}
        except Exception as e:
            return {"success": False, "error": str(e)}

class DataService:
    """数据处理服务"""
    def __init__(self, db, kafka, redis):
        self.db = db
        self.kafka = kafka
        self.redis = redis
        self.start_consumer()
    
    def ingest_data(self, data):
        try:
            # 发送到Kafka
            self.kafka.send('data-ingestion', json.dumps(data))
            return {"success": True, "message": "Data ingested"}
        except Exception as e:
            return {"success": False, "error": str(e)}
    
    def start_consumer(self):
        """启动数据消费者"""
        def consumer_worker():
            while True:
                message = self.kafka.consume('data-ingestion', 'data-processor')
                if message:
                    try:
                        data = json.loads(message)
                        self.process_data(data)
                    except Exception as e:
                        logging.error(f"Error processing message: {e}")
                time.sleep(0.1)
        
        consumer_thread = threading.Thread(target=consumer_worker, daemon=True)
        consumer_thread.start()
    
    def process_data(self, data):
        """处理数据"""
        try:
            # 存储到数据库
            self.db.execute(
                'INSERT INTO sensor_data (device_id, sensor_type, value, metadata) VALUES (?, ?, ?, ?)',
                (data['device_id'], data['sensor_type'], data['value'], json.dumps(data.get('metadata', {})))
            )
            
            # 缓存最新数据到Redis
            cache_key = f"latest:{data['device_id']}:{data['sensor_type']}"
            self.redis.set(cache_key, json.dumps(data), ex=3600)
            
            # 异常检测
            if self.detect_anomaly(data):
                self.kafka.send('anomaly-alerts', json.dumps({
                    **data,
                    'alert_type': 'anomaly_detected',
                    'timestamp': datetime.now().isoformat()
                }))
            
            logging.info(f"Processed data from {data['device_id']}")
            
        except Exception as e:
            logging.error(f"Error processing data: {e}")
    
    def detect_anomaly(self, data):
        """简单异常检测"""
        value = data['value']
        return value < 0 or value > 1000  # 简单阈值检测
    
    def get_device_data(self, device_id, sensor_type=None, limit=10):
        """获取设备数据"""
        try:
            query = 'SELECT * FROM sensor_data WHERE device_id = ?'
            params = [device_id]
            
            if sensor_type:
                query += ' AND sensor_type = ?'
                params.append(sensor_type)
            
            query += ' ORDER BY timestamp DESC LIMIT ?'
            params.append(limit)
            
            results = self.db.execute(query, params)
            
            data = []
            for row in results:
                data.append(dict(row))
            
            return {"success": True, "data": data}
            
        except Exception as e:
            return {"success": False, "error": str(e)}
    
    def get_analytics_summary(self):
        """获取分析摘要"""
        try:
            results = self.db.execute('''
                SELECT 
                    sensor_type,
                    COUNT(*) as count,
                    AVG(value) as avg_value,
                    MIN(value) as min_value,
                    MAX(value) as max_value
                FROM sensor_data 
                WHERE timestamp > datetime('now', '-24 hours')
                GROUP BY sensor_type
            ''')
            
            summary = [dict(row) for row in results]
            return {"success": True, "summary": summary}
            
        except Exception as e:
            return {"success": False, "error": str(e)}

class StreamProcessor:
    """流处理模拟"""
    def __init__(self, kafka):
        self.kafka = kafka
        self.metrics = {
            'processed_count': 0,
            'anomaly_count': 0,
            'average_latency': 0
        }
        self.start_processor()
    
    def start_processor(self):
        """启动流处理器"""
        def processor_worker():
            while True:
                # 消费原始数据
                message = self.kafka.consume('data-ingestion', 'stream-processor')
                if message:
                    try:
                        self.process_stream(json.loads(message))
                    except Exception as e:
                        logging.error(f"Stream processing error: {e}")
                time.sleep(0.05)
        
        processor_thread = threading.Thread(target=processor_worker, daemon=True)
        processor_thread.start()
    
    def process_stream(self, data):
        """处理流数据"""
        start_time = time.time()
        
        # 模拟复杂流处理
        processed_data = {
            **data,
            'processed_at': datetime.now().isoformat(),
            'enriched_data': {
                'processing_latency_ms': 0,  # 将在下面更新
                'batch_id': f"batch_{int(time.time())}"
            }
        }
        
        # 模拟处理时间
        time.sleep(0.001)
        
        # 计算延迟
        latency = (time.time() - start_time) * 1000
        processed_data['enriched_data']['processing_latency_ms'] = round(latency, 2)
        
        # 发送到处理后的主题
        self.kafka.send('processed-data', json.dumps(processed_data))
        
        # 更新指标
        self.metrics['processed_count'] += 1
        self.metrics['average_latency'] = (self.metrics['average_latency'] + latency) / 2
        
        if data.get('value', 0) < 0 or data.get('value', 0) > 1000:
            self.metrics['anomaly_count'] += 1

class MVPHTTPHandler(BaseHTTPRequestHandler):
    """HTTP请求处理器"""
    
    def __init__(self, *args, **kwargs):
        self.auth_service = kwargs.pop('auth_service')
        self.data_service = kwargs.pop('data_service')
        self.stream_processor = kwargs.pop('stream_processor')
        super().__init__(*args, **kwargs)
    
    def do_GET(self):
        """处理GET请求"""
        parsed_path = urlparse(self.path)
        path = parsed_path.path
        query = parse_qs(parsed_path.query)
        
        if path == '/health':
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            response = {"status": "healthy", "service": "mvp-platform"}
            self.wfile.write(json.dumps(response).encode())
        
        elif path.startswith('/api/data/'):
            device_id = path.split('/')[-1]
            sensor_type = query.get('sensor_type', [None])[0]
            limit = int(query.get('limit', [10])[0])
            
            result = self.data_service.get_device_data(device_id, sensor_type, limit)
            
            self.send_response(200 if result['success'] else 400)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps(result).encode())
        
        elif path == '/api/analytics/summary':
            result = self.data_service.get_analytics_summary()
            
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps(result).encode())
        
        elif path == '/api/metrics':
            metrics = {
                "stream_processor": self.stream_processor.metrics,
                "timestamp": datetime.now().isoformat()
            }
            
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps(metrics).encode())
        
        else:
            self.send_response(404)
            self.end_headers()
    
    def do_POST(self):
        """处理POST请求"""
        content_length = int(self.headers['Content-Length'])
        post_data = self.rfile.read(content_length)
        
        try:
            data = json.loads(post_data.decode('utf-8'))
        except json.JSONDecodeError:
            self.send_response(400)
            self.end_headers()
            return
        
        if self.path == '/api/auth/register':
            result = self.auth_service.register(
                data.get('username'),
                data.get('password'), 
                data.get('email')
            )
        
        elif self.path == '/api/auth/login':
            result = self.auth_service.login(
                data.get('username'),
                data.get('password')
            )
        
        elif self.path == '/api/data/ingest':
            result = self.data_service.ingest_data(data)
        
        else:
            self.send_response(404)
            self.end_headers()
            return
        
        status_code = 200 if result.get('success') else 400
        self.send_response(status_code)
        self.send_header('Content-type', 'application/json')
        self.end_headers()
        self.wfile.write(json.dumps(result).encode())
    
    def log_message(self, format, *args):
        """自定义日志格式"""
        logging.info(f"HTTP: {format % args}")

class MVPPlatform:
    """MVP平台主类"""
    
    def __init__(self, port=8080):
        self.port = port
        
        # 初始化组件
        logging.info("Initializing MVP Platform components...")
        
        self.redis = InMemoryRedis()
        self.kafka = InMemoryKafka()
        self.db = DatabaseManager()
        
        # 初始化服务
        self.auth_service = AuthService(self.db, self.redis)
        self.data_service = DataService(self.db, self.kafka, self.redis)
        self.stream_processor = StreamProcessor(self.kafka)
        
        logging.info("All components initialized successfully")
    
    def create_handler(self):
        """创建HTTP处理器"""
        def handler(*args, **kwargs):
            return MVPHTTPHandler(
                *args,
                auth_service=self.auth_service,
                data_service=self.data_service,
                stream_processor=self.stream_processor,
                **kwargs
            )
        return handler
    
    def start_server(self):
        """启动HTTP服务器"""
        handler = self.create_handler()
        httpd = HTTPServer(('localhost', self.port), handler)
        
        logging.info(f"MVP Platform started at http://localhost:{self.port}")
        logging.info("Available endpoints:")
        logging.info("  GET  /health - Health check")
        logging.info("  POST /api/auth/register - User registration")
        logging.info("  POST /api/auth/login - User login")
        logging.info("  POST /api/data/ingest - Data ingestion")
        logging.info("  GET  /api/data/{device_id} - Get device data")
        logging.info("  GET  /api/analytics/summary - Analytics summary")
        logging.info("  GET  /api/metrics - Platform metrics")
        logging.info("")
        logging.info("Press Ctrl+C to stop the server")
        
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            logging.info("Shutting down MVP Platform...")
            httpd.shutdown()

def main():
    """主函数"""
    print("🚀 Starting MVP Platform Local Simulator")
    print("=" * 50)
    
    platform = MVPPlatform(port=8080)
    platform.start_server()

if __name__ == "__main__":
    main()