// K6 性能测试脚本
import http from 'k6/http';
import { check, sleep } from 'k6';
import { Rate, Trend } from 'k6/metrics';

// 自定义指标
const errorRate = new Rate('errors');
const dataIngestionTrend = new Trend('data_ingestion_duration');

// 测试选项
export const options = {
  stages: [
    { duration: '30s', target: 10 },   // 预热：30秒内增加到10用户
    { duration: '2m', target: 50 },    // 负载：2分钟保持50用户
    { duration: '1m', target: 100 },   // 峰值：1分钟增加到100用户
    { duration: '2m', target: 100 },   // 持续：2分钟保持100用户
    { duration: '30s', target: 0 },    // 降压：30秒降到0用户
  ],
  thresholds: {
    http_req_duration: ['p(95)<500'], // 95%的请求在500ms内完成
    http_req_failed: ['rate<0.05'],   // 错误率小于5%
    errors: ['rate<0.1'],             // 自定义错误率小于10%
  },
};

const BASE_URL = 'http://localhost:8080';

// 测试数据生成器
function generateSensorData(deviceId, iterationId) {
  const sensorTypes = ['temperature', 'humidity', 'pressure', 'cpu_usage'];
  const randomSensorType = sensorTypes[Math.floor(Math.random() * sensorTypes.length)];
  
  return {
    device_id: `load_test_${deviceId}_${iterationId}`,
    sensor_type: randomSensorType,
    value: Math.random() * 100,
    metadata: {
      location: `building_${String.fromCharCode(65 + (deviceId % 3))}`,
      floor: Math.floor(Math.random() * 5) + 1,
      test_type: 'load_test',
      timestamp: new Date().toISOString()
    }
  };
}

// 用户注册和登录
export function setup() {
  console.log('🚀 Starting performance test setup...');
  
  // 健康检查
  const healthResponse = http.get(`${BASE_URL}/health`);
  if (healthResponse.status !== 200) {
    console.error('❌ Health check failed');
    return null;
  }
  
  console.log('✅ Platform health check passed');
  
  // 创建测试用户
  const username = `load_test_user_${Date.now()}`;
  const registerData = {
    username: username,
    password: 'loadtest123',
    email: 'loadtest@example.com'
  };
  
  const registerResponse = http.post(
    `${BASE_URL}/api/auth/register`,
    JSON.stringify(registerData),
    {
      headers: { 'Content-Type': 'application/json' },
    }
  );
  
  if (registerResponse.status !== 200) {
    console.log('⚠️ User registration failed, continuing without auth');
    return { token: null };
  }
  
  // 用户登录
  const loginData = {
    username: username,
    password: 'loadtest123'
  };
  
  const loginResponse = http.post(
    `${BASE_URL}/api/auth/login`,
    JSON.stringify(loginData),
    {
      headers: { 'Content-Type': 'application/json' },
    }
  );
  
  let token = null;
  if (loginResponse.status === 200) {
    const loginResult = JSON.parse(loginResponse.body);
    token = loginResult.token;
    console.log('✅ Test user authentication successful');
  }
  
  return { token };
}

// 主测试函数
export default function (data) {
  const deviceId = (__VU - 1) % 10; // 10个虚拟设备
  
  // 1. 数据摄入测试
  const sensorData = generateSensorData(deviceId, __ITER);
  const ingestionStart = new Date();
  
  const ingestionResponse = http.post(
    `${BASE_URL}/api/data/ingest`,
    JSON.stringify(sensorData),
    {
      headers: { 'Content-Type': 'application/json' },
    }
  );
  
  const ingestionDuration = new Date() - ingestionStart;
  dataIngestionTrend.add(ingestionDuration);
  
  const ingestionSuccess = check(ingestionResponse, {
    'data ingestion status is 200': (r) => r.status === 200,
    'data ingestion response time < 1000ms': (r) => r.timings.duration < 1000,
    'data ingestion has success field': (r) => {
      try {
        const body = JSON.parse(r.body);
        return body.hasOwnProperty('success');
      } catch (e) {
        return false;
      }
    },
  });
  
  if (!ingestionSuccess) {
    errorRate.add(1);
  }
  
  // 2. 间歇性数据查询测试（20%概率）
  if (Math.random() < 0.2) {
    const queryDeviceId = `load_test_${deviceId}_${Math.max(0, __ITER - 5)}`;
    const queryResponse = http.get(`${BASE_URL}/api/data/${queryDeviceId}?limit=5`);
    
    check(queryResponse, {
      'data query status is 200': (r) => r.status === 200,
      'data query response time < 2000ms': (r) => r.timings.duration < 2000,
    });
  }
  
  // 3. 间歇性分析摘要测试（10%概率）
  if (Math.random() < 0.1) {
    const analyticsResponse = http.get(`${BASE_URL}/api/analytics/summary`);
    
    check(analyticsResponse, {
      'analytics status is 200': (r) => r.status === 200,
      'analytics response time < 3000ms': (r) => r.timings.duration < 3000,
    });
  }
  
  // 4. 间歇性指标查询测试（5%概率）
  if (Math.random() < 0.05) {
    const metricsResponse = http.get(`${BASE_URL}/api/metrics`);
    
    check(metricsResponse, {
      'metrics status is 200': (r) => r.status === 200,
      'metrics response time < 1000ms': (r) => r.timings.duration < 1000,
    });
  }
  
  // 请求间隔：100-500ms随机延迟
  sleep(Math.random() * 0.4 + 0.1);
}

// 压力测试阶段
export function stressTest() {
  // 这个函数可以用于更激进的压力测试
  const deviceId = Math.floor(Math.random() * 100);
  
  for (let i = 0; i < 5; i++) {
    const sensorData = generateSensorData(deviceId, i);
    
    const response = http.post(
      `${BASE_URL}/api/data/ingest`,
      JSON.stringify(sensorData),
      {
        headers: { 'Content-Type': 'application/json' },
      }
    );
    
    check(response, {
      'stress test ingestion success': (r) => r.status === 200,
    });
  }
}

// 测试清理
export function teardown(data) {
  console.log('🧹 Cleaning up performance test...');
  
  // 发送一些最终查询来验证系统状态
  const finalHealthCheck = http.get(`${BASE_URL}/health`);
  console.log(`Final health check status: ${finalHealthCheck.status}`);
  
  const finalMetrics = http.get(`${BASE_URL}/api/metrics`);
  if (finalMetrics.status === 200) {
    try {
      const metrics = JSON.parse(finalMetrics.body);
      console.log('📊 Final system metrics:', JSON.stringify(metrics, null, 2));
    } catch (e) {
      console.log('⚠️ Could not parse final metrics');
    }
  }
  
  console.log('✅ Performance test cleanup completed');
}

// 辅助函数：生成异常数据（用于异常检测测试）
export function anomalyTest() {
  const anomalyData = {
    device_id: `anomaly_test_${__VU}`,
    sensor_type: 'temperature',
    value: Math.random() > 0.5 ? -50 : 1500, // 异常低温或高温
    metadata: {
      test_type: 'anomaly_detection',
      expected_anomaly: true
    }
  };
  
  const response = http.post(
    `${BASE_URL}/api/data/ingest`,
    JSON.stringify(anomalyData),
    {
      headers: { 'Content-Type': 'application/json' },
    }
  );
  
  check(response, {
    'anomaly data ingestion success': (r) => r.status === 200,
  });
}

// 自定义场景定义
export const scenarios = {
  // 基础负载测试
  basic_load: {
    executor: 'ramping-vus',
    startVUs: 0,
    stages: [
      { duration: '1m', target: 20 },
      { duration: '3m', target: 20 },
      { duration: '1m', target: 0 },
    ],
    gracefulRampDown: '30s',
  },
  
  // 峰值负载测试
  spike_load: {
    executor: 'ramping-vus',
    startVUs: 0,
    stages: [
      { duration: '10s', target: 100 }, // 快速增加到100用户
      { duration: '30s', target: 100 }, // 保持30秒
      { duration: '10s', target: 0 },   // 快速降到0
    ],
    gracefulRampDown: '10s',
  },
  
  // 异常检测测试
  anomaly_detection: {
    executor: 'constant-vus',
    vus: 5,
    duration: '2m',
    exec: 'anomalyTest',
  },
};