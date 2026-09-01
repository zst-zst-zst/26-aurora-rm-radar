#!/usr/bin/env bash
set -euo pipefail

ROOT="/home/zst/T"
MODE="${1:-all}"  # replay | match | calib | all

kill_pattern() {
  local pattern="$1"
  [[ -z "${pattern}" ]] && return
  pkill -9 -f "${pattern}" >/dev/null 2>&1 || true
}

cleanup_shm_locks() {
  rm -f /dev/shm/fastrtps_port* >/dev/null 2>&1 || true
  rm -f /tmp/fastrtps_port* >/dev/null 2>&1 || true
  rm -f /dev/shm/fastdds* >/dev/null 2>&1 || true
}

kill_common() {
  kill_pattern "component_container --ros-args -r __node:=camera_detector_container"
  kill_pattern "rosbag_image_view"
  kill_pattern "configure_map_server"
  kill_pattern "/opt/ros/jazzy/lib/nav2_map_server/map_server"
}

kill_replay_only() {
  kill_pattern "bag.launch.py"
  kill_pattern "rosbag_player_node"
}

kill_match_only() {
  kill_pattern "/home/zst/T/src/tdt_vision/launch/match.launch.py"
}

kill_calib_only() {
  kill_pattern "calib.launch.py"
}

echo "cleanup_start mode=${MODE}"
cleanup_shm_locks

case "${MODE}" in
  replay)
    kill_common
    kill_replay_only
    ;;
  match)
    kill_common
    kill_match_only
    ;;
  calib)
    kill_common
    kill_calib_only
    ;;
  all)
    kill_common
    kill_replay_only
    kill_match_only
    kill_calib_only
    ;;
  *)
    kill_common
    kill_replay_only
    kill_match_only
    kill_calib_only
    ;;
esac

echo "cleanup_done mode=${MODE}"
