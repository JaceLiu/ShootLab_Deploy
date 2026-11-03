#!/usr/bin/env bash
set -euo pipefail

# Real-robot bring-up reference topology.
#
# Keep onboarding docs aligned with this script: each tmux pane maps to one
# runtime responsibility, and rt/ball_state must have exactly one fuser
# publisher.
#
# Environment overrides:
#   PERCEPTION_MODE=lidar|full  lidar skips the camera pane (default: lidar)
#   AUTO_START=1|0              submit each pane automatically (default: 1)
#   ROBOT_IFACE=<name>          override Unitree network interface
#   REPO_DIR=<path>             override repository root

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="${REPO_DIR:-$(cd "${SCRIPT_DIR}/.." && pwd)}"
SESSION_NAME="${1:-robomimic}"
PERCEPTION_MODE="${PERCEPTION_MODE:-lidar}"
AUTO_START="${AUTO_START:-1}"
TMUX_SHELL="bash --noprofile --norc"
UNITREE_SDK2_DIR="${UNITREE_SDK2_DIR:-/home/unitree/unitree_sdk2-main}"
CYCLONEDDS_IDLC="${CYCLONEDDS_IDLC:-/opt/ros/humble/bin/idlc}"
PYTHON_BIN="${PYTHON_BIN:-/home/unitree/miniconda3/envs/robomimic/bin/python}"
CONDA_ENV_LIB="${CONDA_ENV_LIB:-/home/unitree/miniconda3/envs/robomimic/lib}"
UNITREE_DDS_LIB="${UNITREE_SDK2_DIR}/thirdparty/lib/$(uname -m)"
PY_CYCLONEDDS_LIB="${PY_CYCLONEDDS_LIB:-/home/unitree/share/opt/cyclonedds-0.10.5/lib}"

case "${PERCEPTION_MODE}" in
  lidar|full) ;;
  *)
    echo "PERCEPTION_MODE must be 'lidar' or 'full', got '${PERCEPTION_MODE}'" >&2
    exit 2
    ;;
esac

case "${AUTO_START}" in
  0|1) ;;
  *)
    echo "AUTO_START must be 0 or 1, got '${AUTO_START}'" >&2
    exit 2
    ;;
esac

if [[ ! -f "${REPO_DIR}/tools/start_tmux_layout.sh" ]]; then
  echo "REPO_DIR does not point to RoboNaldo_Deploy: ${REPO_DIR}" >&2
  exit 1
fi

detect_robot_iface() {
  if [[ -n "${ROBOT_IFACE:-}" ]]; then
    printf '%s\n' "${ROBOT_IFACE}"
    return 0
  fi

  local candidate
  for candidate in enP8p1s0 enp5s0f1 eth0 usb0 usb1; do
    if [[ -e "/sys/class/net/${candidate}" ]]; then
      printf '%s\n' "${candidate}"
      return 0
    fi
  done

  if command -v ip >/dev/null 2>&1; then
    candidate="$(ip -o -4 addr show scope global 2>/dev/null |
      awk '/192\.168\.123\./ {print $2; exit}')"
    if [[ -n "${candidate}" ]]; then
      printf '%s\n' "${candidate}"
      return 0
    fi
  fi

  return 1
}

if ! ROBOT_IFACE="$(detect_robot_iface)"; then
  echo "Cannot detect a Unitree network interface; set ROBOT_IFACE explicitly." >&2
  exit 1
fi

if [[ "${PERCEPTION_MODE}" == "full" && ! -e /dev/video3 ]]; then
  echo "PERCEPTION_MODE=full requires the camera at /dev/video3." >&2
  exit 1
fi

BRIDGE_CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${ROBOT_IFACE}\" priority=\"default\" multicast=\"default\" /></Interfaces></General><SharedMemory><Enable>false</Enable></SharedMemory></Domain></CycloneDDS>"
PY_BRIDGE_CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${ROBOT_IFACE}\" priority=\"default\" multicast=\"default\" /></Interfaces></General></Domain></CycloneDDS>"

if ! command -v tmux >/dev/null 2>&1; then
  echo "tmux is not installed or not in PATH" >&2
  exit 1
fi

if tmux has-session -t "${SESSION_NAME}" 2>/dev/null; then
  tmux attach-session -t "${SESSION_NAME}"
  exit 0
fi

tmux new-session -d -s "${SESSION_NAME}" -n main -c "${REPO_DIR}" "${TMUX_SHELL}"
tmux set-option -t "${SESSION_NAME}" remain-on-exit on

# Layout (2 columns × 3 rows, lidar mode):
#   left-top    | right-top     cpp bridge | ball-state monitor
#   left-mid    | right-mid     deploy_policy | lidar ball detection
#   left-bottom | right-bottom  sensor dashboard (port 8091) | ball fuser
#
# In full mode, right-top runs the gray camera instead of the ball monitor.
# Data flow (lidar mode):
#   right-mid  -> rt/lidar_ball_state
#   right-bot  -> rt/ball_state
#   left-bot   subscribes lidar/fused topics -> browser http://<robot>:8091/
LEFT_TOP="$(tmux display-message -p -t "${SESSION_NAME}:0.0" '#{pane_id}')"
RIGHT_TOP="$(tmux split-window -h -P -F '#{pane_id}' -t "${LEFT_TOP}" -c "${REPO_DIR}" "${TMUX_SHELL}")"
LEFT_MID="$(tmux split-window -v -P -F '#{pane_id}' -t "${LEFT_TOP}" -c "${REPO_DIR}" "${TMUX_SHELL}")"
RIGHT_MID="$(tmux split-window -v -P -F '#{pane_id}' -t "${RIGHT_TOP}" -c "${REPO_DIR}" "${TMUX_SHELL}")"
LEFT_BOTTOM="$(tmux split-window -v -P -F '#{pane_id}' -t "${LEFT_MID}" -c "${REPO_DIR}" "${TMUX_SHELL}")"
RIGHT_BOTTOM="$(tmux split-window -v -P -F '#{pane_id}' -t "${RIGHT_MID}" -c "${REPO_DIR}" "${TMUX_SHELL}")"
tmux select-layout -t "${SESSION_NAME}:0" tiled

printf -v repo_q '%q' "${REPO_DIR}"

BRIDGE_CACHE="${REPO_DIR}/bridge/build/CMakeCache.txt"
BRIDGE_CACHE_RESET=""
if [[ -f "${BRIDGE_CACHE}" ]]; then
  cached_source="$(sed -n 's/^CMAKE_HOME_DIRECTORY:INTERNAL=//p' "${BRIDGE_CACHE}")"
  if [[ -n "${cached_source}" && "${cached_source}" != "${REPO_DIR}/bridge" ]]; then
    echo "Detected stale C++ bridge cache from ${cached_source}; regenerating it."
    BRIDGE_CACHE_RESET="rm -f '${BRIDGE_CACHE}' && "
  fi
fi
unset cached_source

BRIDGE_CMD="cd ${repo_q} && ${BRIDGE_CACHE_RESET}cmake -S bridge -B bridge/build -DUNITREE_SDK2_DIR='${UNITREE_SDK2_DIR}' -DCYCLONEDDS_IDLC='${CYCLONEDDS_IDLC}' && cmake --build bridge/build -j2 && env LD_LIBRARY_PATH='${UNITREE_DDS_LIB}:${CONDA_ENV_LIB}:'\"\${LD_LIBRARY_PATH:-}\" CYCLONEDDS_URI='${BRIDGE_CYCLONEDDS_URI}' BRIDGE_NETWORK_INTERFACE='${ROBOT_IFACE}' bridge/build/cpp_bridge_main"
POLICY_CMD="cd ${repo_q} && env LD_LIBRARY_PATH='${PY_CYCLONEDDS_LIB}:${CONDA_ENV_LIB}:/opt/onnxruntime/lib:'\"\${LD_LIBRARY_PATH:-}\" CYCLONEDDS_URI='${PY_BRIDGE_CYCLONEDDS_URI}' ROS_LOCALHOST_ONLY=0 '${PYTHON_BIN}' bridge/python/deploy_policy.py"
LIDAR_CMD="cd ${repo_q} && CYCLONEDDS_IFACE='${ROBOT_IFACE}' ./onboard/perception/lidar/run.sh --show --base-y-bias 0.00 --dds-topic rt/lidar_ball_state"
CAMERA_CMD="cd ${repo_q} && CYCLONEDDS_IFACE='${ROBOT_IFACE}' ./onboard/perception/camera/run_gray.sh --show"
DASHBOARD_CMD="cd ${repo_q} && CYCLONEDDS_IFACE='${ROBOT_IFACE}' bash onboard/perception/run_sensor_dashboard.sh"
FUSER_CMD="cd ${repo_q} && CYCLONEDDS_IFACE='${ROBOT_IFACE}' bash onboard/perception/run_ball_fuser.sh"
MONITOR_CMD="cd ${repo_q} && env LD_LIBRARY_PATH='${PY_CYCLONEDDS_LIB}:${CONDA_ENV_LIB}:'\"\${LD_LIBRARY_PATH:-}\" CYCLONEDDS_URI='${PY_BRIDGE_CYCLONEDDS_URI}' '${PYTHON_BIN}' tools/check_ball_state.py"

queue_pane() {
  local pane="$1"
  local command="$2"
  tmux send-keys -t "${pane}" -l "${command}"
  if [[ "${AUTO_START}" == "1" ]]; then
    sleep 0.2
    tmux send-keys -t "${pane}" C-m
  fi
}

# Queue in dependency order. The policy waits for bridge state; the fuser can
# start before either raw sensor. With AUTO_START=0, press Enter manually.
queue_pane "${LEFT_TOP}" "${BRIDGE_CMD}"
queue_pane "${LEFT_MID}" "${POLICY_CMD}"
queue_pane "${RIGHT_MID}" "${LIDAR_CMD}"
queue_pane "${RIGHT_BOTTOM}" "${FUSER_CMD}"
queue_pane "${LEFT_BOTTOM}" "${DASHBOARD_CMD}"

if [[ "${PERCEPTION_MODE}" == "full" ]]; then
  queue_pane "${RIGHT_TOP}" "${CAMERA_CMD}"
  tmux new-window -d -t "${SESSION_NAME}" -n monitor -c "${REPO_DIR}" "${TMUX_SHELL}"
  queue_pane "${SESSION_NAME}:1.0" "${MONITOR_CMD}"
else
  queue_pane "${RIGHT_TOP}" "${MONITOR_CMD}"
fi

tmux select-pane -t "${LEFT_TOP}"
tmux attach-session -t "${SESSION_NAME}"
