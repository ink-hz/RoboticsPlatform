#!/usr/bin/env python3
"""
认证微服务 - 提供JWT认证功能
"""

from flask import Flask, jsonify, request
from flask_cors import CORS
import jwt
import redis
import psycopg2
from datetime import datetime, timedelta
import os
import hashlib
from prometheus_flask_exporter import PrometheusMetrics

app = Flask(__name__)
CORS(app)

# Prometheus监控
metrics = PrometheusMetrics(app)
metrics.info('auth_service', 'Authentication microservice', version='1.0.0')

# 配置
JWT_SECRET = os.environ.get('JWT_SECRET', 'your-secret-key-change-in-production')
JWT_ALGORITHM = 'HS256'
JWT_EXP_DELTA_SECONDS = 3600

# Redis连接
redis_client = redis.Redis(
    host=os.environ.get('REDIS_HOST', 'redis'),
    port=int(os.environ.get('REDIS_PORT', 6379)),
    decode_responses=True
)

# PostgreSQL连接配置
DB_CONFIG = {
    'host': os.environ.get('DB_HOST', 'postgres'),
    'database': os.environ.get('DB_NAME', 'mvpdb'),
    'user': os.environ.get('DB_USER', 'admin'),
    'password': os.environ.get('DB_PASSWORD', 'admin123'),
    'port': os.environ.get('DB_PORT', 5432)
}

def get_db_connection():
    """获取数据库连接"""
    return psycopg2.connect(**DB_CONFIG)

def hash_password(password):
    """密码哈希"""
    return hashlib.sha256(password.encode()).hexdigest()

@app.route('/health', methods=['GET'])
def health_check():
    """健康检查"""
    try:
        # 检查Redis
        redis_client.ping()
        # 检查PostgreSQL
        conn = get_db_connection()
        conn.close()
        return jsonify({'status': 'healthy', 'service': 'auth-service'}), 200
    except Exception as e:
        return jsonify({'status': 'unhealthy', 'error': str(e)}), 503

@app.route('/register', methods=['POST'])
def register():
    """用户注册"""
    try:
        data = request.get_json()
        username = data.get('username')
        password = data.get('password')
        email = data.get('email')
        
        if not username or not password:
            return jsonify({'error': 'Username and password are required'}), 400
        
        # 存储用户到数据库
        conn = get_db_connection()
        cur = conn.cursor()
        
        # 创建用户表（如果不存在）
        cur.execute('''
            CREATE TABLE IF NOT EXISTS users (
                id SERIAL PRIMARY KEY,
                username VARCHAR(100) UNIQUE NOT NULL,
                password_hash VARCHAR(255) NOT NULL,
                email VARCHAR(255),
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            )
        ''')
        
        # 插入新用户
        password_hash = hash_password(password)
        cur.execute(
            'INSERT INTO users (username, password_hash, email) VALUES (%s, %s, %s)',
            (username, password_hash, email)
        )
        conn.commit()
        cur.close()
        conn.close()
        
        return jsonify({'message': 'User registered successfully'}), 201
        
    except psycopg2.IntegrityError:
        return jsonify({'error': 'Username already exists'}), 409
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@app.route('/login', methods=['POST'])
def login():
    """用户登录"""
    try:
        data = request.get_json()
        username = data.get('username')
        password = data.get('password')
        
        if not username or not password:
            return jsonify({'error': 'Username and password are required'}), 400
        
        # 验证用户
        conn = get_db_connection()
        cur = conn.cursor()
        cur.execute(
            'SELECT id, password_hash FROM users WHERE username = %s',
            (username,)
        )
        user = cur.fetchone()
        cur.close()
        conn.close()
        
        if not user or user[1] != hash_password(password):
            return jsonify({'error': 'Invalid credentials'}), 401
        
        # 生成JWT
        payload = {
            'user_id': user[0],
            'username': username,
            'exp': datetime.utcnow() + timedelta(seconds=JWT_EXP_DELTA_SECONDS)
        }
        token = jwt.encode(payload, JWT_SECRET, algorithm=JWT_ALGORITHM)
        
        # 存储token到Redis（用于session管理）
        redis_client.setex(
            f'token:{username}',
            JWT_EXP_DELTA_SECONDS,
            token
        )
        
        return jsonify({'token': token, 'expires_in': JWT_EXP_DELTA_SECONDS}), 200
        
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@app.route('/verify', methods=['POST'])
def verify_token():
    """验证JWT token"""
    try:
        auth_header = request.headers.get('Authorization')
        if not auth_header or not auth_header.startswith('Bearer '):
            return jsonify({'error': 'Invalid authorization header'}), 401
        
        token = auth_header.split(' ')[1]
        
        # 验证token
        payload = jwt.decode(token, JWT_SECRET, algorithms=[JWT_ALGORITHM])
        
        # 检查Redis中的session
        username = payload.get('username')
        stored_token = redis_client.get(f'token:{username}')
        
        if stored_token != token:
            return jsonify({'error': 'Token revoked or expired'}), 401
        
        return jsonify({'valid': True, 'user': payload}), 200
        
    except jwt.ExpiredSignatureError:
        return jsonify({'error': 'Token expired'}), 401
    except jwt.InvalidTokenError:
        return jsonify({'error': 'Invalid token'}), 401
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@app.route('/logout', methods=['POST'])
def logout():
    """用户登出"""
    try:
        auth_header = request.headers.get('Authorization')
        if not auth_header or not auth_header.startswith('Bearer '):
            return jsonify({'error': 'Invalid authorization header'}), 401
        
        token = auth_header.split(' ')[1]
        payload = jwt.decode(token, JWT_SECRET, algorithms=[JWT_ALGORITHM])
        username = payload.get('username')
        
        # 从Redis删除token
        redis_client.delete(f'token:{username}')
        
        return jsonify({'message': 'Logged out successfully'}), 200
        
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@app.route('/metrics', methods=['GET'])
def metrics_endpoint():
    """Prometheus指标端点"""
    return metrics.generate_latest_responses()

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8080, debug=True)