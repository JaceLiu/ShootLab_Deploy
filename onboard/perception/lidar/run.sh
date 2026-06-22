#!/usr/bin/env bash
# run.sh — 启动 LiDAR 球检测（自动启动 Livox MID360 驱动）
#
# 用法：
#   bash onboard/perception/lidar/run.sh
#   bash onboard/perception/lidar/run.sh --dds-topic rt/lidar_ball_state
#
# Ctrl-C 退出时自动关闭 Livox 驱动和检测器。

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"

# ── ROS2 + Livox 环境 ────────────────────────────────────────────────────────
ROS_DISTRO="${ROS_DISTRO:-humble}"
if [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
elif [[ -f /opt/ros/foxy/setup.bash ]]; then
    source /opt/ros/foxy/setup.bash
elif [[ -f /opt/ros/humble/setup.bash ]]; then
    source /opt/ros/humble/setup.bash
else
    echo "[run_lidar] ERROR: ROS2 setup.bash not found under /opt/ros"
    exit 1
fi

LIVOX_WS="${LIVOX_WS:-$HOME/ws_livox}"
if [[ -f "$LIVOX_WS/install/setup.bash" ]]; then
    source "$LIVOX_WS/install/setup.bash"
else
    echo "[run_lidar] WARN: Livox workspace setup not found: $LIVOX_WS/install/setup.bash"
fi

UNITREE_ROS2_WS="${UNITREE_ROS2_WS:-$HOME/unitree_ros2/cyclonedds_ws}"
source "$UNITREE_ROS2_WS/install/setup.sh" 2>/dev/null || true
source "$UNITREE_ROS2_WS/install/setup.bash" 2>/dev/null || true

LIVOX_CONFIG="${LIVOX_CONFIG:-$LIVOX_WS/src/livox_ros_driver2/config/MID360_config.json}"
if [[ ! -f "$LIVOX_CONFIG" ]]; then
    echo "[run_lidar] ERROR: MID360 config not found: $LIVOX_CONFIG"
    exit 1
fi

LIVOX_SDK2_LIB="${LIVOX_SDK2_LIB:-$HOME/Livox-SDK2/build/sdk_core}"
PY_CYCLONEDDS_LIB="${PY_CYCLONEDDS_LIB:-$HOME/share/opt/cyclonedds-0.10.5/lib}"
_extra_ld_paths=()
[[ -d "$PY_CYCLONEDDS_LIB" ]] && _extra_ld_paths+=("$PY_CYCLONEDDS_LIB")
[[ -d "$LIVOX_SDK2_LIB" ]] && _extra_ld_paths+=("$LIVOX_SDK2_LIB")
if (( ${#_extra_ld_paths[@]} )); then
    export LD_LIBRARY_PATH="$(IFS=:; echo "${_extra_ld_paths[*]}"):${LD_LIBRARY_PATH:-}"
fi
unset _extra_ld_paths

source /home/unitree/miniconda3/etc/profile.d/conda.sh 2>/dev/null || true

# Use CycloneDDS — Unitree's intended RMW (FastDDS OOM-kills on 16 GB Jetson).
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=0
CYCLONEDDS_IFACE="${CYCLONEDDS_IFACE:-$(ip -o -4 addr show scope global 2>/dev/null | awk '/192\.168\.123\./ {print $2; exit}')}"
CYCLONEDDS_IFACE="${CYCLONEDDS_IFACE:-enP8p1s0}"
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"$CYCLONEDDS_IFACE\" priority=\"default\" multicast=\"default\" /></Interfaces></General></Domain></CycloneDDS>"
echo "[run_lidar] CycloneDDS interface: $CYCLONEDDS_IFACE"

cd "$ROOT_DIR"

# ── 清理 ─────────────────────────────────────────────────────────────────────
cleanup() {
    echo ""
    echo "[run_lidar] Stopping..."
    pkill -f "msg_MID360_launch" 2>/dev/null || true
    pkill -f "livox_ros_driver2_node" 2>/dev/null || true
    sleep 1
    pkill -9 -f "livox_ros_driver2_node" 2>/dev/null || true
    echo "[run_lidar] Done."
}
trap cleanup EXIT INT TERM

# ── 启动 Livox MID360 驱动（PointCloud2 模式，Python 端明显更快） ───────────
echo "[run_lidar] Starting Livox MID360 driver..."
ros2 run livox_ros_driver2 livox_ros_driver2_node \
    --ros-args \
    -p xfer_format:=0 \
    -p multi_topic:=0 \
    -p data_src:=0 \
    -p publish_freq:=10.0 \
    -p output_data_type:=0 \
    -p frame_id:=livox_frame \
    -p user_config_path:="$LIVOX_CONFIG" \
    -p cmdline_input_bd_code:=livox0000000001 > /tmp/livox_driver.log 2>&1 &
sleep 3
echo "[run_lidar] Livox driver started"

# ── 启动 LiDAR 检测器（前台） ────────────────────────────────────────────────
echo "[run_lidar] Starting lidar ball detector..."
echo "──────────────────────────────────────────────────────────────────────────"

PYTHON_BIN="${PYTHON_BIN:-/home/unitree/miniconda3/envs/robomimic/bin/python}"
"$PYTHON_BIN" -u onboard/perception/lidar/ball_detector.py --msg-type pc2 "$@"
