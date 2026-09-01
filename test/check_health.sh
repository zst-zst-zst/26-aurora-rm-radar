#!/bin/bash
# 话题健康检查脚本 - 比赛中快速确认整条链路是否正常
# 用法: bash test/check_health.sh [lidar]
#   不加参数: 仅检查相机+串口链路
#   加 lidar:  同时检查激光雷达链路

set -euo pipefail
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

TIMEOUT=5  # 每个话题等待秒数

check_topic() {
    local topic="$1"
    local desc="$2"
    printf "%-35s %-20s " "$topic" "$desc"
    local hz
    hz=$(timeout "$TIMEOUT" ros2 topic hz "$topic" --window 3 2>&1 | grep "average rate" | head -1 | awk '{print $3}') || true
    if [ -z "$hz" ]; then
        echo -e "${RED}✗ 无数据${NC}"
        return 1
    else
        echo -e "${GREEN}✓ ${hz} Hz${NC}"
        return 0
    fi
}

check_echo() {
    local topic="$1"
    local desc="$2"
    printf "%-35s %-20s " "$topic" "$desc"
    local data
    data=$(timeout "$TIMEOUT" ros2 topic echo "$topic" --once 2>&1) || true
    if [ -z "$data" ] || echo "$data" | grep -q "Waiting"; then
        echo -e "${RED}✗ 无数据${NC}"
        return 1
    else
        echo -e "${GREEN}✓ 有数据${NC}"
        return 0
    fi
}

echo "========================================="
echo "  雷达站话题健康检查"
echo "========================================="
echo ""

FAIL=0

echo -e "${YELLOW}[相机链路]${NC}"
check_topic "/camera_image"     "相机原始图像"     || ((FAIL++))
check_topic "/detect_result"    "检测结果"         || ((FAIL++))
check_topic "/resolve_result"   "解算结果"         || ((FAIL++))
echo ""

if [ "${1:-}" = "lidar" ]; then
    echo -e "${YELLOW}[激光雷达链路]${NC}"
    check_topic "/livox/lidar"          "Livox 原始点云"    || ((FAIL++))
    check_topic "/livox/lidar_dynamic"  "动态点云"          || ((FAIL++))
    check_topic "/livox/lidar_cluster"  "聚类结果"          || ((FAIL++))
    check_topic "/livox/lidar_kalman"   "Kalman 融合点云"   || ((FAIL++))
    echo ""
fi

echo -e "${YELLOW}[融合与输出]${NC}"
check_topic "/radar2sentry"     "Radar2Sentry"     || ((FAIL++))
check_topic "/kalman_detect"    "Kalman 检测结果"  || ((FAIL++))
check_echo  "/match_info"       "裁判系统信息"     || ((FAIL++))
echo ""

echo -e "${YELLOW}[串口状态]${NC}"
# 检查串口节点参数
PORT=$(ros2 param get /radar_serial_node port 2>/dev/null | awk '{print $NF}') || PORT="未知"
DRY=$(ros2 param get /radar_serial_node dry_run 2>/dev/null | awk '{print $NF}') || DRY="未知"
printf "%-35s %s\n" "串口端口:" "$PORT"
printf "%-35s %s\n" "dry_run:" "$DRY"
echo ""

echo "========================================="
if [ "$FAIL" -eq 0 ]; then
    echo -e "${GREEN}全部通过 ✓${NC}"
else
    echo -e "${RED}${FAIL} 项异常 ✗${NC}"
fi
echo "========================================="
