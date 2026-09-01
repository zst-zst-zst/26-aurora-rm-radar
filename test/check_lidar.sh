#!/bin/bash
# 激光雷达独立测试脚本
# 用法: bash test/check_lidar.sh
# 前提: Mid-70 已通电并通过以太网连接，config/map.pcd 存在

set -euo pipefail

echo "========================================="
echo "  Livox Mid-70 独立测试"
echo "========================================="

# 检查 map.pcd
if [ ! -f "config/map.pcd" ]; then
    echo "[ERROR] config/map.pcd 不存在！"
    echo "  请先录制场地静态点云并保存为 config/map.pcd"
    echo "  方法: 在空场地（无机器人）启动 livox 驱动，录制 rosbag，"
    echo "  然后用 pcl_ros 或自定义工具把点云合并保存为 PCD"
    exit 1
fi

echo "[1/4] 检查网络连接..."
# Livox Mid-70 默认 IP: 192.168.1.1xx
if ping -c 1 -W 2 192.168.1.1 &>/dev/null; then
    echo "  ✓ 192.168.1.1 可达"
elif ip addr | grep -q "192.168.1"; then
    echo "  ✓ 192.168.1.x 网段已配置"
else
    echo "  ✗ 未检测到 192.168.1.x 网段"
    echo "  请配置网卡 IP 为 192.168.1.50 (子网 255.255.255.0)"
    exit 1
fi

echo "[2/4] 启动 Livox 驱动..."
echo "  ros2 launch livox_ros2_driver livox_lidar_launch.py"
echo "  (请在另一个终端执行)"
echo ""
echo "[3/4] 启动激光雷达算法链..."
echo "  ros2 launch dynamic_cloud lidar.launch.py input_is_self_frame:=false self_color_override:=0"
echo "  (请在另一个终端执行)"
echo ""
echo "[4/4] 验证话题..."
echo "  等待节点启动后执行:"
echo "    ros2 topic hz /livox/lidar            # 应有 ~10Hz"
echo "    ros2 topic hz /livox/lidar_dynamic    # 应有数据"
echo "    ros2 topic hz /livox/lidar_cluster    # 应有数据"
echo ""
echo "  可视化 (可选):"
echo "    rviz2  # 添加 PointCloud2 话题 /livox/lidar_dynamic, frame=rm_frame"
echo ""
echo "========================================="
echo "  如果 /livox/lidar 无数据:"
echo "  1. 检查网线连接"
echo "  2. 检查 IP 配置: ip addr show"
echo "  3. 检查 livox_lidar_config.json 中 enable_connect: true"
echo "========================================="
