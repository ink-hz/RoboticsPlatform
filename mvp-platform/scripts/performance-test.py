#!/usr/bin/env python3
"""
MVP平台性能测试脚本
"""

import asyncio
import aiohttp
import time
import json
import statistics
from concurrent.futures import ThreadPoolExecutor
import threading
from datetime import datetime
import random

class PerformanceTestSuite:
    """性能测试套件"""
    
    def __init__(self, base_url="http://localhost:8080", max_concurrent=100):
        self.base_url = base_url
        self.max_concurrent = max_concurrent
        self.results = {
            'latencies': [],
            'throughput': 0,
            'error_rate': 0,
            'success_count': 0,
            'error_count': 0,
            'start_time': None,
            'end_time': None
        }
        self.lock = threading.Lock()
    
    async def send_request(self, session, method, endpoint, data=None):
        """发送异步请求"""
        url = f"{self.base_url}{endpoint}"
        start_time = time.time()
        
        try:
            if method.upper() == 'POST':
                async with session.post(url, json=data) as response:
                    await response.text()
                    status = response.status
            else:
                async with session.get(url) as response:
                    await response.text()
                    status = response.status
            
            latency = (time.time() - start_time) * 1000  # 转换为毫秒
            
            with self.lock:
                self.results['latencies'].append(latency)
                if status == 200:
                    self.results['success_count'] += 1
                else:
                    self.results['error_count'] += 1
            
            return status == 200, latency
            
        except Exception as e:
            latency = (time.time() - start_time) * 1000
            with self.lock:
                self.results['error_count'] += 1
                self.results['latencies'].append(latency)
            return False, latency
    
    async def data_ingestion_load_test(self, concurrent_users=50, duration_seconds=60):
        """数据摄入负载测试"""
        print(f"🚀 Starting data ingestion load test...")
        print(f"   Concurrent users: {concurrent_users}")
        print(f"   Duration: {duration_seconds}s")
        
        self.results['start_time'] = time.time()
        
        async with aiohttp.ClientSession() as session:
            tasks = []
            end_time = time.time() + duration_seconds
            
            while time.time() < end_time:
                # 控制并发数量
                if len(tasks) >= concurrent_users:
                    done, tasks = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
                
                # 生成测试数据
                device_id = f"perf_test_{random.randint(1, 100)}"
                sensor_data = {
                    "device_id": device_id,
                    "sensor_type": random.choice(["temperature", "humidity", "pressure", "cpu"]),
                    "value": random.uniform(0, 100),
                    "metadata": {
                        "test_type": "performance",
                        "timestamp": datetime.now().isoformat()
                    }
                }
                
                # 创建任务
                task = asyncio.create_task(
                    self.send_request(session, 'POST', '/api/data/ingest', sensor_data)
                )
                tasks.append(task)
            
            # 等待所有任务完成
            if tasks:
                await asyncio.wait(tasks)
        
        self.results['end_time'] = time.time()
        self.calculate_metrics()
        
        return self.results
    
    async def mixed_workload_test(self, concurrent_users=30, duration_seconds=120):
        """混合工作负载测试"""
        print(f"🔄 Starting mixed workload test...")
        print(f"   Concurrent users: {concurrent_users}")
        print(f"   Duration: {duration_seconds}s")
        
        self.results['start_time'] = time.time()
        
        async with aiohttp.ClientSession() as session:
            tasks = []
            end_time = time.time() + duration_seconds
            
            while time.time() < end_time:
                if len(tasks) >= concurrent_users:
                    done, tasks = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
                
                # 随机选择操作类型
                operation = random.choices(
                    ['ingest', 'query', 'analytics', 'metrics'],
                    weights=[60, 25, 10, 5]  # 摄入60%, 查询25%, 分析10%, 指标5%
                )[0]
                
                if operation == 'ingest':
                    sensor_data = {
                        "device_id": f"mixed_test_{random.randint(1, 50)}",
                        "sensor_type": random.choice(["temp", "humid", "pressure"]),
                        "value": random.uniform(10, 90),
                        "metadata": {"test": "mixed_workload"}
                    }
                    task = asyncio.create_task(
                        self.send_request(session, 'POST', '/api/data/ingest', sensor_data)
                    )
                elif operation == 'query':
                    device_id = f"mixed_test_{random.randint(1, 50)}"
                    task = asyncio.create_task(
                        self.send_request(session, 'GET', f'/api/data/{device_id}?limit=10')
                    )
                elif operation == 'analytics':
                    task = asyncio.create_task(
                        self.send_request(session, 'GET', '/api/analytics/summary')
                    )
                else:  # metrics
                    task = asyncio.create_task(
                        self.send_request(session, 'GET', '/api/metrics')
                    )
                
                tasks.append(task)
            
            if tasks:
                await asyncio.wait(tasks)
        
        self.results['end_time'] = time.time()
        self.calculate_metrics()
        
        return self.results
    
    async def spike_test(self, spike_users=200, spike_duration=30):
        """峰值负载测试"""
        print(f"⚡ Starting spike test...")
        print(f"   Spike users: {spike_users}")
        print(f"   Spike duration: {spike_duration}s")
        
        self.results['start_time'] = time.time()
        
        async with aiohttp.ClientSession() as session:
            # 创建大量并发请求
            tasks = []
            
            for i in range(spike_users):
                sensor_data = {
                    "device_id": f"spike_test_{i}",
                    "sensor_type": "load_test",
                    "value": random.uniform(0, 100),
                    "metadata": {"test": "spike", "user_id": i}
                }
                
                task = asyncio.create_task(
                    self.send_request(session, 'POST', '/api/data/ingest', sensor_data)
                )
                tasks.append(task)
            
            # 等待所有请求完成
            await asyncio.wait(tasks)
        
        self.results['end_time'] = time.time()
        self.calculate_metrics()
        
        return self.results
    
    def calculate_metrics(self):
        """计算性能指标"""
        if not self.results['latencies']:
            return
        
        duration = self.results['end_time'] - self.results['start_time']
        total_requests = self.results['success_count'] + self.results['error_count']
        
        self.results['throughput'] = total_requests / duration if duration > 0 else 0
        self.results['error_rate'] = (self.results['error_count'] / total_requests * 100) if total_requests > 0 else 0
        self.results['avg_latency'] = statistics.mean(self.results['latencies'])
        self.results['p50_latency'] = statistics.median(self.results['latencies'])
        self.results['p95_latency'] = statistics.quantiles(self.results['latencies'], n=20)[18] if len(self.results['latencies']) > 20 else max(self.results['latencies'])
        self.results['p99_latency'] = statistics.quantiles(self.results['latencies'], n=100)[98] if len(self.results['latencies']) > 100 else max(self.results['latencies'])
        self.results['max_latency'] = max(self.results['latencies'])
        self.results['min_latency'] = min(self.results['latencies'])
    
    def print_results(self, test_name="Performance Test"):
        """打印测试结果"""
        print(f"\n{'='*60}")
        print(f"📊 {test_name} Results")
        print(f"{'='*60}")
        print(f"🕐 Duration: {self.results['end_time'] - self.results['start_time']:.2f}s")
        print(f"📈 Throughput: {self.results['throughput']:.2f} req/sec")
        print(f"✅ Successful requests: {self.results['success_count']}")
        print(f"❌ Failed requests: {self.results['error_count']}")
        print(f"📉 Error rate: {self.results['error_rate']:.2f}%")
        print(f"\n⏱️  Latency Statistics (ms):")
        print(f"   Average: {self.results.get('avg_latency', 0):.2f}")
        print(f"   P50 (Median): {self.results.get('p50_latency', 0):.2f}")
        print(f"   P95: {self.results.get('p95_latency', 0):.2f}")
        print(f"   P99: {self.results.get('p99_latency', 0):.2f}")
        print(f"   Min: {self.results.get('min_latency', 0):.2f}")
        print(f"   Max: {self.results.get('max_latency', 0):.2f}")
        print(f"{'='*60}")
    
    def reset_results(self):
        """重置测试结果"""
        self.results = {
            'latencies': [],
            'throughput': 0,
            'error_rate': 0,
            'success_count': 0,
            'error_count': 0,
            'start_time': None,
            'end_time': None
        }

async def run_comprehensive_performance_test():
    """运行综合性能测试"""
    print("🎯 MVP Platform Comprehensive Performance Testing")
    print("="*60)
    
    # 健康检查
    tester = PerformanceTestSuite()
    async with aiohttp.ClientSession() as session:
        try:
            async with session.get(f"{tester.base_url}/health") as response:
                if response.status != 200:
                    print("❌ Platform health check failed")
                    return
        except Exception as e:
            print(f"❌ Cannot connect to platform: {e}")
            return
    
    print("✅ Platform health check passed")
    
    # 测试序列
    tests = [
        ("Data Ingestion Load Test", tester.data_ingestion_load_test, {"concurrent_users": 20, "duration_seconds": 30}),
        ("Mixed Workload Test", tester.mixed_workload_test, {"concurrent_users": 15, "duration_seconds": 60}),
        ("Spike Test", tester.spike_test, {"spike_users": 100, "spike_duration": 20}),
    ]
    
    all_results = {}
    
    for test_name, test_func, test_params in tests:
        print(f"\n🚦 Starting {test_name}...")
        tester.reset_results()
        
        try:
            results = await test_func(**test_params)
            tester.print_results(test_name)
            all_results[test_name] = results.copy()
        except Exception as e:
            print(f"❌ {test_name} failed: {e}")
        
        # 测试间隔
        print("\n⏳ Cooling down for 10 seconds...")
        await asyncio.sleep(10)
    
    # 生成测试报告
    generate_performance_report(all_results)

def generate_performance_report(results):
    """生成性能测试报告"""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    report_file = f"performance_report_{timestamp}.json"
    
    report_data = {
        "timestamp": datetime.now().isoformat(),
        "test_results": results,
        "summary": {
            "total_tests": len(results),
            "avg_throughput": sum(r.get('throughput', 0) for r in results.values()) / len(results) if results else 0,
            "avg_error_rate": sum(r.get('error_rate', 0) for r in results.values()) / len(results) if results else 0,
        }
    }
    
    with open(report_file, 'w') as f:
        json.dump(report_data, f, indent=2)
    
    print(f"\n📄 Performance report saved to: {report_file}")
    
    # 打印汇总
    print(f"\n🎯 Overall Performance Summary")
    print(f"{'='*40}")
    print(f"Average Throughput: {report_data['summary']['avg_throughput']:.2f} req/sec")
    print(f"Average Error Rate: {report_data['summary']['avg_error_rate']:.2f}%")

if __name__ == "__main__":
    try:
        asyncio.run(run_comprehensive_performance_test())
    except KeyboardInterrupt:
        print("\n🛑 Performance test interrupted by user")
    except Exception as e:
        print(f"\n❌ Performance test failed: {e}")