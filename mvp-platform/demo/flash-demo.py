#!/usr/bin/env python3
"""
闪光效果演示脚本
"""

import urllib.request
import json
import time

def send_request(method, url, data=None):
    """发送HTTP请求"""
    try:
        if data:
            data_bytes = json.dumps(data).encode('utf-8')
            req = urllib.request.Request(url, data=data_bytes, method=method)
            req.add_header('Content-Type', 'application/json')
        else:
            req = urllib.request.Request(url, method=method)
        
        with urllib.request.urlopen(req, timeout=5) as response:
            content = response.read().decode('utf-8')
            return response.status, json.loads(content) if content else {}
    
    except Exception as e:
        return None, str(e)

def main():
    """闪光效果演示"""
    print("🌟 MVP平台闪光效果演示")
    print("="*50)
    print()
    
    print("🔍 当前页面上的闪光效果有两种含义:")
    print()
    print("1️⃣ **数据流箭头闪烁** (⬇️ 符号)")
    print("   含义: 表示数据在系统架构中的流动")
    print("   位置: 架构图各层之间的箭头")
    print("   频率: 每2秒脉冲一次")
    print("   目的: 展示数据从上层到下层的处理流程")
    print()
    
    print("2️⃣ **高负载组件闪烁** (组件框)")
    print("   含义: 表示该组件正在处理大量请求")
    print("   条件: 组件负载 > 10时触发")
    print("   频率: 每1秒脉冲一次") 
    print("   目的: 突出显示系统瓶颈和繁忙组件")
    print()
    
    # 检查当前状态
    print("📊 当前系统负载状态分析:")
    print("-"*30)
    
    status, arch_data = send_request('GET', 'http://localhost:8080/api/architecture')
    if status == 200:
        components = arch_data['components']
        
        # 统计闪烁组件
        flashing_components = {k: v for k, v in components.items() if v['load'] > 10}
        normal_components = {k: v for k, v in components.items() if v['load'] <= 10}
        
        print(f"✨ 闪烁组件 ({len(flashing_components)}个):")
        for name, info in list(flashing_components.items())[:8]:  # 显示前8个
            print(f"   🔥 {name}: 负载 {info['load']} ({info['status']})")
        
        if len(flashing_components) > 8:
            print(f"   ... 还有 {len(flashing_components) - 8} 个组件也在闪烁")
        
        print(f"\n⚪ 正常组件 ({len(normal_components)}个):")
        for name, info in list(normal_components.items())[:5]:  # 显示前5个
            print(f"   📝 {name}: 负载 {info['load']} ({info['status']})")
        
        if len(normal_components) > 5:
            print(f"   ... 还有 {len(normal_components) - 5} 个正常组件")
    
    print()
    print("🎮 交互功能:")
    print("-"*20)
    print("• 点击任意组件可查看详细信息")
    print("• 使用右下角的复选框可开关动画效果")
    print("• 悬停组件可看到负载信息提示")
    print()
    
    print("🚀 演示高负载效果:")
    print("-"*25)
    print("发送一些请求来增加系统负载...")
    
    # 发送一些请求来增加负载
    for i in range(10):
        send_request('GET', 'http://localhost:8080/api/metrics')
        print(".", end="", flush=True)
        time.sleep(0.1)
    
    print(f"\n已发送10个额外请求!")
    
    # 检查更新后的负载
    time.sleep(1)
    status, updated_data = send_request('GET', 'http://localhost:8080/api/architecture')
    if status == 200:
        new_flashing = sum(1 for v in updated_data['components'].values() if v['load'] > 10)
        print(f"现在有 {new_flashing} 个组件在闪烁")
    
    print()
    print("="*50)
    print("✅ 闪光效果说明完毕!")
    print("🌐 现在访问 http://localhost:8080/ 观察闪光效果")
    print("💡 负载越高的组件闪烁越明显")
    print("🎯 这样可以直观看出系统的实时状态")

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"❌ 演示失败: {e}")