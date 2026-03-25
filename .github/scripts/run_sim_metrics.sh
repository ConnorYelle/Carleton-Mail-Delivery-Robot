#!/usr/bin/env bash
set -euo pipefail

RUN_TIMEOUT_SECONDS="${RUN_TIMEOUT_SECONDS:-240}"
STARTUP_DELAY_SECONDS="${STARTUP_DELAY_SECONDS:-25}"
USE_AI_LIDAR="${USE_AI_LIDAR:-true}"
USE_AI_NAVIGATION="${USE_AI_NAVIGATION:-true}"
USE_AI_BEACON="${USE_AI_BEACON:-true}"
USE_AI_AVOIDANCE="${USE_AI_AVOIDANCE:-true}"
USE_AI_TRAVEL_LAYER="${USE_AI_TRAVEL_LAYER:-true}"
BEACON_AI_MODEL="${BEACON_AI_MODEL:-gemma2:2b-instruct-q4_0}"
NAVIGATION_AI_MODEL="${NAVIGATION_AI_MODEL:-gemma2:2b-instruct-q4_0}"
AVOIDANCE_AI_MODEL="${AVOIDANCE_AI_MODEL:-qwen2:0.5b}"
DESTINATION_ROUTE="${DESTINATION_ROUTE:-Canal:Nicol}"
DESTINATION_PUBLISH_DELAY_SECONDS="${DESTINATION_PUBLISH_DELAY_SECONDS:-20}"
WORKSPACE_ROOT="/ros2_ws"
REPO_ROOT="${WORKSPACE_ROOT}/src/carleton_mail_robot"
LOGS_DIR="${REPO_ROOT}/mail-delivery-robot/tools/logs"
RUNS_DIR="${REPO_ROOT}/mail-delivery-robot/tools/logs/runs"
INSTALL_LOGS_DIR="${WORKSPACE_ROOT}/install/mail-delivery-robot/tools/logs"
INSTALL_RUNS_DIR="${INSTALL_LOGS_DIR}/runs"
EXTERNAL_MODELS_DIR="${REPO_ROOT}/external_files"
GAZEBO_MODELS_DIR="/root/.gazebo/models"
WORLD_SOURCE="${EXTERNAL_MODELS_DIR}/demo_video.world"
WORLD_PATCHED="/tmp/demo_video_ci.world"
RUN_START_EPOCH="$(date +%s)"

cleanup() {
  pkill -f "ros2 launch mail-delivery-robot container.launch.py" 2>/dev/null || true
  pkill -f "create3_gazebo.launch.py" 2>/dev/null || true
  pkill -f "robot_description.launch.py" 2>/dev/null || true
  pkill -f "gazebo.launch.py" 2>/dev/null || true
  pkill -f "gzserver" 2>/dev/null || true
  pkill -f "gzclient" 2>/dev/null || true
  pkill -f "ollama serve" 2>/dev/null || true
  if [[ -n "${TOPIC_PROBE_PID:-}" ]]; then
    kill "${TOPIC_PROBE_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

# ROS setup scripts may reference unset vars (e.g. AMENT_TRACE_SETUP_FILES),
# so temporarily disable nounset while sourcing.
set +u
source /opt/ros/humble/setup.bash
set -u

echo "[metrics-runner] rebuilding workspace..."
colcon build --symlink-install --executor sequential

# Source again after build to pick up any changes.
set +u
source "${WORKSPACE_ROOT}/install/setup.bash"
set -u

mkdir -p "${RUNS_DIR}"
mkdir -p "${GAZEBO_MODELS_DIR}"
mkdir -p "${INSTALL_RUNS_DIR}"
# Ensure logger output under install path is visible in mounted source logs directory.
rm -rf "${INSTALL_LOGS_DIR}"
ln -s "${REPO_ROOT}/mail-delivery-robot/tools/logs" "${INSTALL_LOGS_DIR}"

echo "[metrics-runner] provisioning gazebo models..."
copied_models=0
for d in "${EXTERNAL_MODELS_DIR}"/*; do
  [[ -d "${d}" ]] || continue
  if [[ -f "${d}/model.sdf" ]]; then
    cp -r "${d}" "${GAZEBO_MODELS_DIR}/"
    copied_models=$((copied_models + 1))
  fi
done

if [[ "${copied_models}" -eq 0 ]]; then
  echo "[metrics-runner] no models with model.sdf found under ${EXTERNAL_MODELS_DIR}"
  exit 1
fi

export GAZEBO_MODEL_PATH="${GAZEBO_MODELS_DIR}:${EXTERNAL_MODELS_DIR}:${GAZEBO_MODEL_PATH:-}"
export DASHBOARD_LOG_DIR="${LOGS_DIR}"
export MAIL_ROBOT_WORLD_PATH="${WORLD_PATCHED}"

echo "[metrics-runner] preparing world file..."
if [[ ! -f "${WORLD_SOURCE}" ]]; then
  echo "[metrics-runner] world file not found: ${WORLD_SOURCE}"
  exit 1
fi
control_pkg_prefix="$(ros2 pkg prefix irobot_create_control 2>/dev/null || true)"
if [[ -z "${control_pkg_prefix}" ]]; then
  echo "[metrics-runner] unable to resolve irobot_create_control package prefix"
  exit 1
fi
control_yaml_path="${control_pkg_prefix}/share/irobot_create_control/config/control.yaml"
if [[ ! -f "${control_yaml_path}" ]]; then
  echo "[metrics-runner] control yaml not found: ${control_yaml_path}"
  exit 1
fi
cp "${WORLD_SOURCE}" "${WORLD_PATCHED}"
sed -i "s|/home/hari-admin/testing_ws/install/irobot_create_control/share/irobot_create_control/config/control.yaml|${control_yaml_path}|g" "${WORLD_PATCHED}"
# Remove world-level gazebo_ros2_control plugin entirely; control is provided via robot_description (classic).
sed -i "/<plugin name='gazebo_ros2_control' filename='libgazebo_ros2_control.so'>/,/<\\/plugin>/d" "${WORLD_PATCHED}"
# Remove pre-spawned robot so create3_gazebo.launch.py can spawn it with control enabled.
sed -i "/<model name='create3'>/,/<\\/model>/d" "${WORLD_PATCHED}"

echo "[metrics-runner] starting ollama..."
ollama serve >/tmp/ollama.log 2>&1 &
sleep 3

echo "[metrics-runner] preloading ollama models..."
if [[ "${USE_AI_BEACON}" == "true" ]]; then
  ollama pull "${BEACON_AI_MODEL}"
fi
if [[ "${USE_AI_NAVIGATION}" == "true" ]]; then
  ollama pull "${NAVIGATION_AI_MODEL}"
fi
if [[ "${USE_AI_AVOIDANCE}" == "true" ]]; then
  ollama pull "${AVOIDANCE_AI_MODEL}"
fi

echo "[metrics-runner] starting robot description publishers..."
ros2 launch irobot_create_common_bringup robot_description.launch.py \
  gazebo:=classic \
  visualize_rays:=false >/tmp/robot_description.log 2>&1 &

sleep 3

echo "[metrics-runner] launching create3 gazebo..."
ros2 launch irobot_create_gazebo_bringup create3_gazebo.launch.py \
  world_path:="${WORLD_PATCHED}" \
  use_gazebo_gui:=false \
  use_rviz:=false \
  spawn_dock:=true \
  spawn_beacons:=false >/tmp/gazebo.log 2>&1 &

sleep "${STARTUP_DELAY_SECONDS}"

echo "[metrics-runner] launching robot stack for ${RUN_TIMEOUT_SECONDS}s..."
echo "[metrics-runner] starting topic probe..."
(
  probe_iter=0
  while true; do
    ts="$(date +%H:%M:%S)"
    {
      echo "=== ${ts} ==="
      if [[ $((probe_iter % 6)) -eq 0 ]]; then
        echo "--- ros2 node list ---"
        ros2 node list 2>&1 || true
        echo "--- ros2 topic list ---"
        ros2 topic list 2>&1 || true
      fi

      for t in /destinations /lidar_data /actions /cmd_vel /odom; do
        echo "--- ${t} info ---"
        ros2 topic info "${t}" 2>&1 || true
        echo "--- ${t} sample ---"
        timeout 2 ros2 topic echo "${t}" --once 2>&1 || echo "(no message)"
      done
      echo ""
    } >>/tmp/ci_topics.log
    probe_iter=$((probe_iter + 1))
    sleep 10
  done
) &
TOPIC_PROBE_PID=$!
(
  sleep "${DESTINATION_PUBLISH_DELAY_SECONDS}"
  echo "[metrics-runner] waiting for /destinations subscribers..."
  for i in $(seq 1 60); do
    sub_count="$(ros2 topic info /destinations 2>/dev/null | awk '/Subscriber count/ {print $3}')"
    if [[ -n "${sub_count}" && "${sub_count}" -ge 1 ]]; then
      break
    fi
    sleep 1
  done
  published_destination_route="${DESTINATION_ROUTE}"
  echo "[metrics-runner] publishing destination route: ${published_destination_route}"
  # Keep publishing until travel_layer confirms receipt or timeout.
  timeout 60 ros2 topic pub /destinations std_msgs/msg/String "{data: ${published_destination_route}}" -r 1 \
    >>/tmp/destination_pub.log 2>&1 &
  DEST_PUB_PID=$!
  for i in $(seq 1 30); do
    if grep -q "Updated destination" /tmp/robot.log 2>/dev/null; then
      echo "[metrics-runner] destination acknowledged by travel_layer"
      kill "${DEST_PUB_PID}" 2>/dev/null || true
      break
    fi
    sleep 2
  done
) &

set +e
timeout "${RUN_TIMEOUT_SECONDS}" \
  ros2 launch mail-delivery-robot container.launch.py \
  use_ai_lidar:="${USE_AI_LIDAR}" \
  use_ai_navigation:="${USE_AI_NAVIGATION}" \
  use_ai_beacon:="${USE_AI_BEACON}" \
  use_fake_beacons:="true" \
  use_ai_avoidance:="${USE_AI_AVOIDANCE}" \
  use_ai_travel_layer:="${USE_AI_TRAVEL_LAYER}" \
  2>&1 | tee /tmp/robot.log
launch_status=${PIPESTATUS[0]}
set -e

if [[ "${launch_status}" -ne 0 && "${launch_status}" -ne 124 ]]; then
  echo "[metrics-runner] robot launch failed with status ${launch_status}"
  tail -n 80 /tmp/robot.log || true
  exit "${launch_status}"
fi

latest_run=""
for candidate in $(ls -1t "${RUNS_DIR}"/run_*.txt "${RUNS_DIR}"/*run_*.txt "${INSTALL_RUNS_DIR}"/run_*.txt "${INSTALL_RUNS_DIR}"/*run_*.txt 2>/dev/null || true); do
  file_epoch="$(stat -c %Y "${candidate}" 2>/dev/null || echo 0)"
  if [[ "${file_epoch}" -ge "${RUN_START_EPOCH}" ]]; then
    latest_run="${candidate}"
    break
  fi
done
if [[ -z "${latest_run}" ]]; then
  echo "[metrics-runner] no fresh real run file found."
  echo "--- robot.log ---"
  tail -n 120 /tmp/robot.log || true
  echo "--- gazebo.log ---"
  tail -n 120 /tmp/gazebo.log || true
  echo "--- robot_description.log ---"
  tail -n 120 /tmp/robot_description.log || true
  echo "--- destination_pub.log ---"
  tail -n 80 /tmp/destination_pub.log || true
  if [[ -f /tmp/robot.log ]]; then
    cp /tmp/robot.log "${RUNS_DIR}/robot.log" || true
  fi
  if [[ -f /tmp/gazebo.log ]]; then
    cp /tmp/gazebo.log "${RUNS_DIR}/gazebo.log" || true
  fi
  if [[ -f /tmp/robot_description.log ]]; then
    cp /tmp/robot_description.log "${RUNS_DIR}/robot_description.log" || true
  fi
  if [[ -f /tmp/destination_pub.log ]]; then
    cp /tmp/destination_pub.log "${RUNS_DIR}/destination_pub.log" || true
  fi
  if [[ -f /tmp/ci_topics.log ]]; then
    cp /tmp/ci_topics.log "${RUNS_DIR}/ci_topics.log" || true
  fi
  exit 1
fi

echo "[metrics-runner] latest run file: ${latest_run}"
cat "${latest_run}"

if [[ -f /tmp/ci_topics.log ]]; then
  cp /tmp/ci_topics.log "${RUNS_DIR}/ci_topics.log" || true
fi
if [[ -f /tmp/robot.log ]]; then
  cp /tmp/robot.log "${RUNS_DIR}/robot.log" || true
fi
if [[ -f /tmp/gazebo.log ]]; then
  cp /tmp/gazebo.log "${RUNS_DIR}/gazebo.log" || true
fi
if [[ -f /tmp/robot_description.log ]]; then
  cp /tmp/robot_description.log "${RUNS_DIR}/robot_description.log" || true
fi
if [[ -f /tmp/destination_pub.log ]]; then
  cp /tmp/destination_pub.log "${RUNS_DIR}/destination_pub.log" || true
fi
