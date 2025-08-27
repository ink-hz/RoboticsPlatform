#!/usr/bin/env python3
"""
认证服务单元测试
"""

import pytest
import jwt
import json
from datetime import datetime, timedelta
from unittest.mock import Mock, patch, MagicMock
import sys
import os

# 添加apps路径到系统路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../apps/auth-service'))

from app import create_app, AuthService

class TestAuthService:
    """认证服务测试类"""
    
    @pytest.fixture
    def app(self):
        """创建测试应用"""
        app = create_app()
        app.config['TESTING'] = True
        app.config['JWT_SECRET'] = 'test-secret'
        with app.test_client() as client:
            yield client
    
    @pytest.fixture
    def mock_db(self):
        """Mock数据库"""
        db = Mock()
        return db
    
    @pytest.fixture
    def mock_redis(self):
        """Mock Redis"""
        redis = Mock()
        return redis
    
    @pytest.fixture
    def auth_service(self, mock_db, mock_redis):
        """创建认证服务实例"""
        return AuthService(mock_db, mock_redis)
    
    def test_hash_password(self, auth_service):
        """测试密码哈希"""
        password = "test123"
        hashed = auth_service.hash_password(password)
        
        assert hashed is not None
        assert len(hashed) == 64  # SHA256 hex length
        assert hashed != password
        
        # 测试相同密码产生相同哈希
        hashed2 = auth_service.hash_password(password)
        assert hashed == hashed2
    
    def test_register_success(self, auth_service, mock_db):
        """测试用户注册成功"""
        mock_db.execute.return_value = 1
        
        result = auth_service.register("testuser", "test123", "test@example.com")
        
        assert result["success"] is True
        assert "User registered" in result["message"]
        mock_db.execute.assert_called_once()
    
    def test_register_duplicate_user(self, auth_service, mock_db):
        """测试重复用户注册"""
        mock_db.execute.side_effect = Exception("UNIQUE constraint failed")
        
        result = auth_service.register("testuser", "test123", "test@example.com")
        
        assert result["success"] is False
        assert "UNIQUE constraint failed" in result["error"]
    
    def test_login_success(self, auth_service, mock_db, mock_redis):
        """测试登录成功"""
        # Mock数据库返回用户数据
        mock_user = Mock()
        mock_user.__getitem__ = Mock(side_effect=lambda key: {
            'id': 1,
            'password_hash': auth_service.hash_password("test123")
        }[key])
        mock_db.execute.return_value = [mock_user]
        
        result = auth_service.login("testuser", "test123")
        
        assert result["success"] is True
        assert "token" in result
        mock_redis.set.assert_called_once()
    
    def test_login_invalid_credentials(self, auth_service, mock_db):
        """测试无效凭证登录"""
        mock_db.execute.return_value = []
        
        result = auth_service.login("testuser", "wrongpassword")
        
        assert result["success"] is False
        assert "Invalid credentials" in result["error"]
    
    def test_login_wrong_password(self, auth_service, mock_db):
        """测试错误密码"""
        mock_user = Mock()
        mock_user.__getitem__ = Mock(side_effect=lambda key: {
            'id': 1,
            'password_hash': auth_service.hash_password("correct123")
        }[key])
        mock_db.execute.return_value = [mock_user]
        
        result = auth_service.login("testuser", "wrong123")
        
        assert result["success"] is False
        assert "Invalid credentials" in result["error"]
    
    def test_verify_token_success(self, auth_service, mock_redis):
        """测试token验证成功"""
        # 创建有效token
        payload = {
            'user_id': 1,
            'username': 'testuser',
            'exp': datetime.utcnow() + timedelta(hours=1)
        }
        token = jwt.encode(payload, 'test-secret', algorithm='HS256')
        
        # Mock Redis返回token
        mock_redis.get.return_value = token
        
        result = auth_service.verify_token(token)
        
        assert result["success"] is True
        assert result["user"]["username"] == "testuser"
    
    def test_verify_token_expired(self, auth_service, mock_redis):
        """测试过期token"""
        # 创建过期token
        payload = {
            'user_id': 1,
            'username': 'testuser',
            'exp': datetime.utcnow() - timedelta(hours=1)
        }
        token = jwt.encode(payload, 'test-secret', algorithm='HS256')
        
        mock_redis.get.return_value = token
        
        result = auth_service.verify_token(token)
        
        assert result["success"] is False
        assert "Token expired" in result["error"]
    
    def test_verify_token_revoked(self, auth_service, mock_redis):
        """测试已撤销token"""
        payload = {
            'user_id': 1,
            'username': 'testuser',
            'exp': datetime.utcnow() + timedelta(hours=1)
        }
        token = jwt.encode(payload, 'test-secret', algorithm='HS256')
        
        # Mock Redis返回None（token已被撤销）
        mock_redis.get.return_value = None
        
        result = auth_service.verify_token(token)
        
        assert result["success"] is False
        assert "Token revoked" in result["error"]

class TestAuthAPI:
    """认证API测试类"""
    
    @pytest.fixture
    def app(self):
        """创建测试应用"""
        with patch('app.DatabaseManager'), patch('app.redis.Redis'):
            app = create_app()
            app.config['TESTING'] = True
            with app.test_client() as client:
                yield client
    
    def test_health_endpoint(self, app):
        """测试健康检查端点"""
        response = app.get('/health')
        data = json.loads(response.data)
        
        assert response.status_code == 200
        assert data["status"] == "healthy"
    
    @patch('app.auth_service')
    def test_register_endpoint_success(self, mock_auth, app):
        """测试注册端点成功"""
        mock_auth.register.return_value = {"success": True, "message": "User registered"}
        
        response = app.post('/api/register', 
                          data=json.dumps({
                              "username": "testuser",
                              "password": "test123", 
                              "email": "test@example.com"
                          }),
                          content_type='application/json')
        
        data = json.loads(response.data)
        assert response.status_code == 200
        assert data["success"] is True
    
    @patch('app.auth_service')
    def test_register_endpoint_failure(self, mock_auth, app):
        """测试注册端点失败"""
        mock_auth.register.return_value = {"success": False, "error": "User exists"}
        
        response = app.post('/api/register',
                          data=json.dumps({
                              "username": "testuser",
                              "password": "test123",
                              "email": "test@example.com"
                          }),
                          content_type='application/json')
        
        data = json.loads(response.data)
        assert response.status_code == 400
        assert data["success"] is False
    
    @patch('app.auth_service')
    def test_login_endpoint_success(self, mock_auth, app):
        """测试登录端点成功"""
        mock_auth.login.return_value = {"success": True, "token": "test-token"}
        
        response = app.post('/api/login',
                          data=json.dumps({
                              "username": "testuser",
                              "password": "test123"
                          }),
                          content_type='application/json')
        
        data = json.loads(response.data)
        assert response.status_code == 200
        assert data["success"] is True
        assert "token" in data
    
    @patch('app.auth_service')
    def test_logout_endpoint(self, mock_auth, app):
        """测试登出端点"""
        mock_auth.logout.return_value = {"success": True, "message": "Logged out"}
        
        response = app.post('/api/logout',
                          headers={'Authorization': 'Bearer test-token'})
        
        data = json.loads(response.data)
        assert response.status_code == 200
        assert data["success"] is True
    
    def test_invalid_json_request(self, app):
        """测试无效JSON请求"""
        response = app.post('/api/register',
                          data="invalid json",
                          content_type='application/json')
        
        assert response.status_code == 400
    
    def test_missing_fields(self, app):
        """测试缺失字段"""
        response = app.post('/api/register',
                          data=json.dumps({"username": "testuser"}),
                          content_type='application/json')
        
        assert response.status_code == 400

if __name__ == '__main__':
    pytest.main([__file__])