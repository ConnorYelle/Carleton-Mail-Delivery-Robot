#!/usr/bin/env bash
set -euo pipefail

RUN_TIMEOUT_SECONDS="${RUN_TIMEOUT_SECONDS:-240}"
STARTUP_DELAY_SECONDS="${STARTUP_DELAY_SECONDS:-15}"
WORKSPACE_ROOT="/ros2_ws"
REPO_ROOT="${WORKSPACE_ROOT}/src/carleton_mail_robot"
RUNS_DIR="${REPO_ROOT}/mail-delivery-robot/tools/logs/runs"

cleanup() {
  pkill -f "ros2 launch mail-delivery-robot robot.launch.py" 2>/dev/null || true
  pkill -f "create3_gazebo.launch.py" 2>/dev/null || true
  pkill -f "ollama serve" 2>/dev/null || true
}
trap cleanup EXIT

# ROS setup scripts may reference unset vars (e.g. AMENT_TRACE_SETUP_FILES),
# so temporarily disable nounset while sourcing.
set +u
source /opt/ros/humble/setup.bash
source "${WORKSPACE_ROOT}/install/setup.bash"
set -u

mkdir -p "${RUNS_DIR}"

echo "[metrics-runner] starting ollama..."
ollama serve >/tmp/ollama.log 2>&1 &
sleep 3

echo "[metrics-runner] launching gazebo..."
ros2 launch irobot_create_gazebo_bringup create3_gazebo.launch.py \
  world_path:=/root/.gazebo/worlds/demo_video.world \
  spawn_beacons:=true \
  x:=5.586507 \
  y:=-3.07 \
  z:=0.00 >/tmp/gazebo.log 2>&1 &

sleep "${STARTUP_DELAY_SECONDS}"

echo "[metrics-runner] launching robot stack for ${RUN_TIMEOUT_SECONDS}s..."
set +e
timeout "${RUN_TIMEOUT_SECONDS}" \
  ros2 launch mail-delivery-robot robot.launch.py use_ai_lidar:=true use_ai_navigation:=true \
  >/tmp/robot.log 2>&1
launch_status=$?
set -e

if [[ "${launch_status}" -ne 0 && "${launch_status}" -ne 124 ]]; then
  echo "[metrics-runner] robot launch failed with status ${launch_status}"
  tail -n 80 /tmp/robot.log || true
  exit "${launch_status}"
fi

latest_run="$(ls -1t "${RUNS_DIR}"/run_*.txt 2>/dev/null | head -n1 || true)"
if [[ -z "${latest_run}" ]]; then
  echo "[metrics-runner] no run file created in ${RUNS_DIR}"
  echo "--- robot.log ---"
  tail -n 120 /tmp/robot.log || true
  echo "--- gazebo.log ---"
  tail -n 120 /tmp/gazebo.log || true
  exit 1
fi

echo "[metrics-runner] latest run file: ${latest_run}"
cat "${latest_run}"
