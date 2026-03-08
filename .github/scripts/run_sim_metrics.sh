#!/usr/bin/env bash
set -euo pipefail

RUN_TIMEOUT_SECONDS="${RUN_TIMEOUT_SECONDS:-240}"
STARTUP_DELAY_SECONDS="${STARTUP_DELAY_SECONDS:-15}"
USE_AI_LIDAR="${USE_AI_LIDAR:-false}"
USE_AI_NAVIGATION="${USE_AI_NAVIGATION:-false}"
USE_AI_BEACON="${USE_AI_BEACON:-true}"
USE_AI_AVOIDANCE="${USE_AI_AVOIDANCE:-false}"
USE_AI_TRAVEL_LAYER="${USE_AI_TRAVEL_LAYER:-false}"
BEACON_AI_MODEL="${BEACON_AI_MODEL:-gemma2:2b-instruct-q4_0}"
NAVIGATION_AI_MODEL="${NAVIGATION_AI_MODEL:-gemma2:2b-instruct-q4_0}"
AVOIDANCE_AI_MODEL="${AVOIDANCE_AI_MODEL:-qwen2:0.5b}"
WORKSPACE_ROOT="/ros2_ws"
REPO_ROOT="${WORKSPACE_ROOT}/src/carleton_mail_robot"
RUNS_DIR="${REPO_ROOT}/mail-delivery-robot/tools/logs/runs"
EXTERNAL_MODELS_DIR="${REPO_ROOT}/external_files"
GAZEBO_MODELS_DIR="/root/.gazebo/models"
WORLD_SOURCE="${EXTERNAL_MODELS_DIR}/demo_video.world"
WORLD_PATCHED="/tmp/demo_video_ci.world"
RUN_START_EPOCH="$(date +%s)"

cleanup() {
  pkill -f "ros2 launch mail-delivery-robot robot.launch.py" 2>/dev/null || true
  pkill -f "create3_gazebo.launch.py" 2>/dev/null || true
  pkill -f "robot_description.launch.py" 2>/dev/null || true
  pkill -f "gazebo.launch.py" 2>/dev/null || true
  pkill -f "gzserver" 2>/dev/null || true
  pkill -f "gzclient" 2>/dev/null || true
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
mkdir -p "${GAZEBO_MODELS_DIR}"

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
# Avoid duplicate control parameter declarations when robot_description also provides control config.
sed -i "/<parameters>.*irobot_create_control.*\\/config\\/control.yaml<\\/parameters>/d" "${WORLD_PATCHED}"

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

echo "[metrics-runner] launching gazebo..."
ros2 launch irobot_create_gazebo_bringup gazebo.launch.py \
  world_path:="${WORLD_PATCHED}" \
  use_gazebo_gui:=false >/tmp/gazebo.log 2>&1 &

sleep "${STARTUP_DELAY_SECONDS}"

echo "[metrics-runner] launching robot stack for ${RUN_TIMEOUT_SECONDS}s..."
set +e
timeout "${RUN_TIMEOUT_SECONDS}" \
  ros2 launch mail-delivery-robot robot.launch.py \
  use_ai_lidar:="${USE_AI_LIDAR}" \
  use_ai_navigation:="${USE_AI_NAVIGATION}" \
  use_ai_beacon:="${USE_AI_BEACON}" \
  use_ai_avoidance:="${USE_AI_AVOIDANCE}" \
  use_ai_travel_layer:="${USE_AI_TRAVEL_LAYER}" \
  >/tmp/robot.log 2>&1
launch_status=$?
set -e

if [[ "${launch_status}" -ne 0 && "${launch_status}" -ne 124 ]]; then
  echo "[metrics-runner] robot launch failed with status ${launch_status}"
  tail -n 80 /tmp/robot.log || true
  exit "${launch_status}"
fi

latest_run=""
for candidate in $(ls -1t "${RUNS_DIR}"/run_*.txt 2>/dev/null || true); do
  file_epoch="$(stat -c %Y "${candidate}" 2>/dev/null || echo 0)"
  if [[ "${file_epoch}" -ge "${RUN_START_EPOCH}" ]]; then
    latest_run="${candidate}"
    break
  fi
done
if [[ -z "${latest_run}" ]]; then
  echo "[metrics-runner] no fresh run file created in ${RUNS_DIR}"
  echo "--- robot.log ---"
  tail -n 120 /tmp/robot.log || true
  echo "--- gazebo.log ---"
  tail -n 120 /tmp/gazebo.log || true
  echo "--- robot_description.log ---"
  tail -n 120 /tmp/robot_description.log || true
  exit 1
fi

echo "[metrics-runner] latest run file: ${latest_run}"
cat "${latest_run}"
