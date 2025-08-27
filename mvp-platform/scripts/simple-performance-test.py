#!/usr/bin/env python3
"""
MVP平台简化性能测试脚本（无外部依赖）
"""

import urllib.request
import urllib.parse
import json
import time
import threading
import statistics
import random
from datetime import datetime
from concurrent.futures import ThreadPoolExecutor

class SimplePerformanceTester:
    """简化性能测试器"""
    
    def __init__(self, base_url="http://localhost:8080"):
        self.base_url = base_url
        self.results = {
            'latencies': [],
            'success_count': 0,
            'error_count': 0,
            'start_time': None,
            'end_time': None
        }
        self.lock = threading.Lock()
    
    def send_request(self, method, endpoint, data=None):
        """发送HTTP请求"""
        url = f"{self.base_url}{endpoint}"
        start_time = time.time()
        
        try:
            if data:
                data_bytes = json.dumps(data).encode('utf-8')
                req = urllib.request.Request(url, data=data_bytes, method=method)
                req.add_header('Content-Type', 'application/json')
            else:
                req = urllib.request.Request(url, method=method)
            
            with urllib.request.urlopen(req, timeout=10) as response:
                response.read()
                status = response.status
            
            latency = (time.time() - start_time) * 1000  # 毫秒
            
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
    
    def data_ingestion_worker(self, worker_id, duration_seconds, requests_per_second=5):
        """数据摄入工作线程"""
        end_time = time.time() + duration_seconds
        request_interval = 1.0 / requests_per_second
        
        while time.time() < end_time:
            # 生成测试数据
            sensor_data = {
                "device_id": f"perf_test_{worker_id}_{random.randint(1, 100)}",
                "sensor_type": random.choice(["temperature", "humidity", "pressure", "cpu"]),
                "value": round(random.uniform(0, 100), 2),
                "metadata": {
                    "test_type": "performance",
                    "worker_id": worker_id,
                    "timestamp": datetime.now().isoformat()
                }
            }
            
            # 发送请求
            self.send_request('POST', '/api/data/ingest', sensor_data)
            
            # 控制请求频率
            time.sleep(request_interval)
    
    def query_worker(self, worker_id, duration_seconds, requests_per_second=2):
        """查询工作线程"""
        end_time = time.time() + duration_seconds
        request_interval = 1.0 / requests_per_second
        
        while time.time() < end_time:
            # 随机查询设备
            device_id = f"perf_test_{random.randint(1, 10)}_{random.randint(1, 100)}"
            self.send_request('GET', f'/api/data/{device_id}?limit=5')
            
            # 间歇性分析查询
            if random.random() < 0.3:
                self.send_request('GET', '/api/analytics/summary')
            
            # 间歇性指标查询
            if random.random() < 0.1:
                self.send_request('GET', '/api/metrics')
            
            time.sleep(request_interval)
    
    def run_load_test(self, concurrent_users=10, duration_seconds=60, test_type="mixed"):
        """运行负载测试"""
        print(f"🚀 Starting {test_type} load test...")
        print(f"   Concurrent users: {concurrent_users}")
        print(f"   Duration: {duration_seconds}s")
        
        self.reset_results()
        self.results['start_time'] = time.time()
        
        with ThreadPoolExecutor(max_workers=concurrent_users) as executor:
            futures = []
            
            for i in range(concurrent_users):
                if test_type == "ingestion":
                    future = executor.submit(self.data_ingestion_worker, i, duration_seconds, 3)
                elif test_type == "query":
                    future = executor.submit(self.query_worker, i, duration_seconds, 2)
                else:  # mixed
                    # 70% 摄入，30% 查询
                    if i < concurrent_users * 0.7:
                        future = executor.submit(self.data_ingestion_worker, i, duration_seconds, 2)
                    else:
                        future = executor.submit(self.query_worker, i, duration_seconds, 1)
                
                futures.append(future)
            
            # 等待所有线程完成
            for future in futures:
                future.result()
        
        self.results['end_time'] = time.time()
        self.calculate_metrics()
        return self.results
    
    def run_spike_test(self, spike_requests=200):
        """运行峰值测试"""
        print(f"⚡ Starting spike test...")
        print(f"   Spike requests: {spike_requests}")
        
        self.reset_results()
        self.results['start_time'] = time.time()
        
        def spike_worker(i):
            """峰值测试工作函数"""
            sensor_data = {
                "device_id": f"spike_test_{i}",
                "sensor_type": "load_test",
                "value": random.uniform(0, 100),
                "metadata": {"test": "spike", "request_id": i}
            }
            return self.send_request('POST', '/api/data/ingest', sensor_data)
        
        # 使用线程池并发发送请求
        with ThreadPoolExecutor(max_workers=50) as executor:
            futures = [executor.submit(spike_worker, i) for i in range(spike_requests)]
            
            # 等待所有请求完成
            for future in futures:
                future.result()
        
        self.results['end_time'] = time.time()
        self.calculate_metrics()
        return self.results
    
    def calculate_metrics(self):
        """计算性能指标"""
        if not self.results['latencies']:
            return
        
        duration = self.results['end_time'] - self.results['start_time']
        total_requests = self.results['success_count'] + self.results['error_count']
        
        self.results['duration'] = duration
        self.results['throughput'] = total_requests / duration if duration > 0 else 0
        self.results['error_rate'] = (self.results['error_count'] / total_requests * 100) if total_requests > 0 else 0
        self.results['avg_latency'] = statistics.mean(self.results['latencies'])
        
        if len(self.results['latencies']) > 1:
            self.results['p50_latency'] = statistics.median(self.results['latencies'])
            sorted_latencies = sorted(self.results['latencies'])
            self.results['p95_latency'] = sorted_latencies[int(len(sorted_latencies) * 0.95)]
            self.results['p99_latency'] = sorted_latencies[int(len(sorted_latencies) * 0.99)]
        else:
            self.results['p50_latency'] = self.results['avg_latency']
            self.results['p95_latency'] = self.results['avg_latency']
            self.results['p99_latency'] = self.results['avg_latency']
        
        self.results['max_latency'] = max(self.results['latencies'])
        self.results['min_latency'] = min(self.results['latencies'])
    
    def print_results(self, test_name="Performance Test"):
        """打印测试结果"""
        print(f"\n{'='*60}")
        print(f"📊 {test_name} Results")
        print(f"{'='*60}")
        print(f"🕐 Duration: {self.results.get('duration', 0):.2f}s")
        print(f"📈 Throughput: {self.results.get('throughput', 0):.2f} req/sec")
        print(f"✅ Successful requests: {self.results['success_count']}")
        print(f"❌ Failed requests: {self.results['error_count']}")
        print(f"📉 Error rate: {self.results.get('error_rate', 0):.2f}%")
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
            'success_count': 0,
            'error_count': 0,
            'start_time': None,
            'end_time': None
        }

def main():
    """主测试函数"""
    print("🎯 MVP Platform Simple Performance Testing")
    print("="*60)
    
    # 健康检查
    tester = SimplePerformanceTester()
    try:
        success, latency = tester.send_request('GET', '/health')
        if not success:
            print("❌ Platform health check failed")
            return
    except Exception as e:
        print(f"❌ Cannot connect to platform: {e}")
        return
    
    print("✅ Platform health check passed")
    
    # 性能测试序列
    tests = [
        ("Data Ingestion Load Test", "ingestion", {"concurrent_users": 8, "duration_seconds": 30}),
        ("Mixed Workload Test", "mixed", {"concurrent_users": 6, "duration_seconds": 45}),
        ("Query Load Test", "query", {"concurrent_users": 4, "duration_seconds": 20}),
    ]
    
    all_results = {}
    
    for test_name, test_type, params in tests:
        print(f"\n🚦 Starting {test_name}...")
        try:
            results = tester.run_load_test(test_type=test_type, **params)
            tester.print_results(test_name)
            all_results[test_name] = results.copy()
        except Exception as e:
            print(f"❌ {test_name} failed: {e}")
        
        # 测试间隔
        print("\n⏳ Cooling down for 5 seconds...")
        time.sleep(5)
    
    # 峰值测试
    print(f"\n🚦 Starting Spike Test...")
    try:
        spike_results = tester.run_spike_test(100)
        tester.print_results("Spike Test")
        all_results["Spike Test"] = spike_results.copy()
    except Exception as e:
        print(f"❌ Spike test failed: {e}")
    
    # 生成汇总报告
    generate_summary_report(all_results)

def generate_summary_report(results):
    """生成汇总报告"""
    if not results:
        return
    
    print(f"\n🎯 Overall Performance Summary")
    print(f"{'='*60}")
    
    avg_throughput = sum(r.get('throughput', 0) for r in results.values()) / len(results)
    avg_error_rate = sum(r.get('error_rate', 0) for r in results.values()) / len(results)
    avg_latency = sum(r.get('avg_latency', 0) for r in results.values()) / len(results)
    
    total_requests = sum(r.get('success_count', 0) + r.get('error_count', 0) for r in results.values())
    total_errors = sum(r.get('error_count', 0) for r in results.values())
    
    print(f"📊 Test Summary:")
    print(f"   Total tests run: {len(results)}")
    print(f"   Total requests: {total_requests}")
    print(f"   Total errors: {total_errors}")
    print(f"\n📈 Average Performance:")
    print(f"   Throughput: {avg_throughput:.2f} req/sec")
    print(f"   Latency: {avg_latency:.2f} ms")
    print(f"   Error rate: {avg_error_rate:.2f}%")
    
    # 性能等级评估
    if avg_throughput > 50 and avg_error_rate < 5:
        grade = "🏆 EXCELLENT"
    elif avg_throughput > 25 and avg_error_rate < 10:
        grade = "✅ GOOD"
    elif avg_throughput > 10 and avg_error_rate < 20:
        grade = "⚠️  ACCEPTABLE"
    else:
        grade = "❌ NEEDS IMPROVEMENT"
    
    print(f"\n🎖️  Performance Grade: {grade}")
    print(f"{'='*60}")
    
    # 保存报告到文件
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    report_data = {
        "timestamp": datetime.now().isoformat(),
        "summary": {
            "avg_throughput": avg_throughput,
            "avg_latency": avg_latency,
            "avg_error_rate": avg_error_rate,
            "total_requests": total_requests,
            "total_errors": total_errors,
            "performance_grade": grade
        },
        "detailed_results": results
    }
    
    report_file = f"perf_report_{timestamp}.json"
    try:
        with open(report_file, 'w') as f:
            json.dump(report_data, f, indent=2, default=str)
        print(f"📄 Detailed report saved to: {report_file}")
    except Exception as e:
        print(f"⚠️  Could not save report: {e}")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n🛑 Performance test interrupted by user")
    except Exception as e:
        print(f"\n❌ Performance test failed: {e}")