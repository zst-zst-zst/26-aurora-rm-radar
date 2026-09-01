#!/usr/bin/env python3
"""
Livox激光雷达设备发现测试脚本
"""
import socket
import time
import threading

def scan_livox_devices():
    """扫描网络中的Livox设备"""
    print("正在扫描Livox激光雷达设备...")
    
    # Livox设备通常使用的端口范围
    ports = [65001, 65002, 65003, 65004]
    
    # 常见的Livox IP地址段
    ip_base = "192.168.1."
    found_devices = []
    
    def scan_port(ip, port):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(1)
            result = sock.connect_ex((ip, port))
            if result == 0:
                print(f"发现设备: {ip}:{port}")
                found_devices.append((ip, port))
            sock.close()
        except:
            pass
    
    # 扫描可能的IP地址范围
    threads = []
    for i in range(100, 200):
        for port in ports:
            ip = ip_base + str(i)
            thread = threading.Thread(target=scan_port, args=(ip, port))
            thread.start()
            threads.append(thread)
    
    # 等待所有线程完成
    for thread in threads:
        thread.join()
    
    return found_devices

def test_livox_connection(ip, port):
    """测试与Livox设备的连接"""
    try:
        print(f"正在测试连接 {ip}:{port}...")
        
        # 创建UDP套接字（Livox使用UDP进行设备发现）
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(2)
        
        # 发送设备发现请求
        discovery_msg = b'\x00\x01\x00\x00'  # 简单的发现请求
        sock.sendto(discovery_msg, (ip, port))
        
        try:
            data, addr = sock.recvfrom(1024)
            print(f"收到响应从 {addr}: {data.hex()}")
            return True
        except socket.timeout:
            print(f"连接 {ip}:{port} 超时")
            return False
        finally:
            sock.close()
            
    except Exception as e:
        print(f"连接 {ip}:{port} 失败: {e}")
        return False

if __name__ == "__main__":
    print("Livox激光雷达设备发现工具")
    print("=" * 50)
    
    # 扫描设备
    devices = scan_livox_devices()
    
    if not devices:
        print("未发现任何Livox设备")
        print("\n可能的解决方案:")
        print("1. 检查激光雷达电源是否开启")
        print("2. 检查网线连接是否正常")
        print("3. 确认电脑IP配置为192.168.1.x网段")
        print("4. 检查激光雷达IP地址是否正确")
    else:
        print(f"\n发现 {len(devices)} 个设备:")
        for ip, port in devices:
            print(f"  - {ip}:{port}")
            
        # 测试连接
        print("\n测试设备连接...")
        for ip, port in devices:
            test_livox_connection(ip, port)
