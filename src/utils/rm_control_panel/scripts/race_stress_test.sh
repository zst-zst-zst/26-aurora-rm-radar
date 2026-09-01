#!/usr/bin/env bash
set -euo pipefail

ROOT="/home/zst/T"
LOOPS="${1:-10}"
RUN_SECS="${2:-25}"
MODE="${3:-replay}"  # replay | match

if [[ "${MODE}" != "replay" && "${MODE}" != "match" ]]; then
  echo "Usage: $0 [loops] [run_secs] [replay|match]"
  exit 1
fi

set +u
source /opt/ros/jazzy/setup.bash
source "${ROOT}/install/setup.bash"
set -u

STAMP="$(date +%Y%m%d_%H%M%S)"
mkdir -p "${ROOT}/log"
LOG_FILE="${ROOT}/log/race_stress_${MODE}_${STAMP}.log"

echo "stress_start mode=${MODE} loops=${LOOPS} run_secs=${RUN_SECS}" | tee -a "${LOG_FILE}"

max_residual=0
fail_count=0

get_patterns() {
  if [[ "${MODE}" == "replay" ]]; then
    cat <<'EOF'
bag.launch.py
rosbag_image_view
component_container --ros-args -r __node:=camera_detector_container
map_server
configure_map_server
EOF
  else
    cat <<'EOF'
match.launch.py
component_container --ros-args -r __node:=camera_detector_container
EOF
  fi
}

for ((i=1; i<=LOOPS; i++)); do
  if [[ "${MODE}" == "replay" ]]; then
    CMD="ros2 launch tdt_vision bag.launch.py show_image:=false"
  else
    CMD="ros2 launch /home/zst/T/src/tdt_vision/launch/match.launch.py"
  fi

  echo "loop=${i} launch_begin" | tee -a "${LOG_FILE}"
  setsid bash -lc "${CMD}" >> "${LOG_FILE}" 2>&1 &
  LAUNCH_PID=$!

  sleep "${RUN_SECS}"

  # Level stop: INT -> TERM -> KILL (by process group)
  kill -INT -"${LAUNCH_PID}" 2>/dev/null || true
  sleep 1
  kill -TERM -"${LAUNCH_PID}" 2>/dev/null || true
  sleep 1
  kill -KILL -"${LAUNCH_PID}" 2>/dev/null || true

  # Kill common leftovers
  while IFS= read -r p; do
    [[ -z "${p}" ]] && continue
    pkill -9 -f "${p}" >/dev/null 2>&1 || true
  done < <(get_patterns)

  sleep 1

  residual=0
  while IFS= read -r p; do
    [[ -z "${p}" ]] && continue
    c="$(pgrep -af "${p}" | wc -l || true)"
    residual=$((residual + c))
  done < <(get_patterns)

  if [[ "${residual}" -gt "${max_residual}" ]]; then
    max_residual="${residual}"
  fi

  if [[ "${residual}" -gt 0 ]]; then
    fail_count=$((fail_count + 1))
  fi

  mem_line="$(free -h | awk 'NR==2 {print $3 "/" $2}')"
  gpu_line="$(nvidia-smi --query-gpu=utilization.gpu,memory.used --format=csv,noheader 2>/dev/null | head -n1 || echo 'gpu_unavailable')"
  echo "loop=${i} residual=${residual} mem=${mem_line} gpu=${gpu_line}" | tee -a "${LOG_FILE}"
done

echo "stress_done fail_loops=${fail_count}/${LOOPS} max_residual=${max_residual} log=${LOG_FILE}" | tee -a "${LOG_FILE}"
