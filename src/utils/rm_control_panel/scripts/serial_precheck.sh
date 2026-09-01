#!/usr/bin/env bash
set -euo pipefail

cd /home/zst/T
set +u
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u

echo "[1/6] serial node param check"
(timeout 3s ros2 param get /radar_serial_node port) || true
(timeout 3s ros2 param get /radar_serial_node dry_run) || true

echo "[2/6] topic existence"
ros2 topic list | rg -n "^/radar2sentry$|^/Radar2Sentry$|^/match_info$|^/radar/tx_raw$" || true

echo "[3/6] freq check"
(timeout 3s ros2 topic hz /radar2sentry) || true
(timeout 3s ros2 topic hz /Radar2Sentry) || true

echo "[4/6] one-shot payload check"
(timeout 3s ros2 topic echo /radar2sentry --once) || true
(timeout 3s ros2 topic echo /Radar2Sentry --once) || true

echo "[5/6] tx raw packet check"
(timeout 3s ros2 topic echo /radar/tx_raw --once) || true

echo "[6/6] match info check"
(timeout 3s ros2 topic echo /match_info --once) || true

echo "[RX] referee downlink hint"
if timeout 3s ros2 topic echo /match_info --once | rg -q "match_time: -200"; then
  echo "[RX][WARN] /match_info.match_time is -200: no valid referee downlink parsed yet"
else
  echo "[RX][OK] referee downlink parsed"
fi

echo "serial precheck done"
