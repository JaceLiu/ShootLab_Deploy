#!/usr/bin/env bash
# ============================================================
# 启动 chest camera apriltag_detector（含 GPU/Jetson 解锁）
# 默认使用 4-tag 板：
#   tag 0 -> target offset = (+10cm, +17.5cm, 0)
#   tag 1 -> target offset = (+10cm, -17.5cm, 0)
#   tag 2 -> target offset = (-10cm, -17.5cm, 0)
#   tag 3 -> target offset = (-10cm, +17.5cm, 0)
#
# 用法：
#   只检测 tag 0：
#   bash onboard/perception/camera/_launch.sh --tag-id 0 --tag-size 0.12 --show
#
#   使用默认 4-tag 板（0/1/2/3）并融合估计主目标点：
#   bash onboard/perception/camera/_launch.sh --show
#   bash onboard/perception/camera/_launch.sh --tag-size 0.12 --show
#   bash onboard/perception/camera/_launch.sh --tag-size 0.10
#
#   如需覆盖默认 4-tag 板配置，可显式传入你自己的 --tag-id / --tag-offset：
#   bash onboard/perception/camera/_launch.sh \
#       --tag-id 5 --tag-id 8 --tag-size 0.10 \
#       --tag-offset 5 0.20 0.00 0.00 \
#       --tag-offset 8 -0.20 0.00 0.00
#
#   其中 --tag-offset TAG_ID DX DY DZ 表示：
#     从该 tag 中心出发，在 tag 自身坐标系里走 (DX, DY, DZ) 后到达公共目标点。
#     方向定义相对 tag 本身，不是相机系/机器人系：
#       +X = tag 右边
#       +Y = tag 上边
#       +Z = 垂直纸面向外
#
# ============================================================
set -e
cd "$(dirname "$0")/../../.."

echo "[_launch.sh] Unlocking Jetson clocks..."
echo "123" | sudo -S nvpmodel -m 0 2>/dev/null || true
echo "123" | sudo -S jetson_clocks 2>/dev/null || true

source onboard/perception/setup_runtime_env.sh

# Initialize conda for non-interactive shells (e.g. SSH)
source /home/unitree/miniconda3/etc/profile.d/conda.sh 2>/dev/null || true

ROS_DISTRO="${ROS_DISTRO:-humble}"
if [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
elif [[ -f /opt/ros/foxy/setup.bash ]]; then
    source /opt/ros/foxy/setup.bash
elif [[ -f /opt/ros/humble/setup.bash ]]; then
    source /opt/ros/humble/setup.bash
fi

LIVOX_WS="${LIVOX_WS:-$HOME/ws_livox}"
source "$LIVOX_WS/install/setup.sh" 2>/dev/null || true
source "$LIVOX_WS/install/setup.bash" 2>/dev/null || true

UNITREE_ROS2_WS="${UNITREE_ROS2_WS:-$HOME/unitree_ros2/cyclonedds_ws}"
source "$UNITREE_ROS2_WS/install/setup.sh" 2>/dev/null || true
source "$UNITREE_ROS2_WS/install/setup.bash" 2>/dev/null || true

PY_CYCLONEDDS_LIB="${PY_CYCLONEDDS_LIB:-$HOME/share/opt/cyclonedds-0.10.5/lib}"
if [[ -d "$PY_CYCLONEDDS_LIB" ]]; then
    export LD_LIBRARY_PATH="$PY_CYCLONEDDS_LIB:${LD_LIBRARY_PATH:-}"
fi

# Use CycloneDDS — Unitree's intended RMW.
# FastDDS (the Foxy default) pre-allocates ~14 GB of shared memory on a 16 GB
# system, immediately triggering the OOM killer before any Python code runs.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=0
CYCLONEDDS_IFACE="${CYCLONEDDS_IFACE:-$(ip -o -4 addr show scope global 2>/dev/null | awk '/192\.168\.123\./ {print $2; exit}')}"
CYCLONEDDS_IFACE="${CYCLONEDDS_IFACE:-enP8p1s0}"
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"$CYCLONEDDS_IFACE\" priority=\"default\" multicast=\"default\" /></Interfaces></General></Domain></CycloneDDS>"

use_default_board_layout=1
use_default_camera_args=1
for arg in "$@"; do
    case "$arg" in
        --tag-id|--tag-id=*|--tag-offset|--tag-offset=*)
            use_default_board_layout=0
            ;;
        --color-backend|--color-backend=*|--v4l2-device|--v4l2-device=*|--camera-profile|--camera-profile=*|--v4l2-fourcc|--v4l2-fourcc=*)
            use_default_camera_args=0
            ;;
    esac
done

default_board_args=(
    --tag-id 0
    --tag-id 1
    --tag-id 2
    --tag-id 3
    --tag-offset 0 0.10 0.175 0.00
    --tag-offset 1 0.10 -0.175 0.00
    --tag-offset 2 -0.10 -0.175 0.00
    --tag-offset 3 -0.10 0.175 0.00
)

PYTHON_BIN="${PYTHON_BIN:-/home/unitree/miniconda3/envs/robomimic/bin/python}"
cmd=(
    "$PYTHON_BIN" -u onboard/perception/camera/target_ball_detector.py
)

if [[ "$use_default_camera_args" -eq 1 ]]; then
    cmd+=(
    --camera-profile gray-ir
    --color-backend v4l2
    --v4l2-device /dev/video3
    --v4l2-fourcc GREY
    --v4l2-fps 30
    --ball-bright
    --ball-bright-max-hz 0
    --ball-bright-max-abs-y 5.0
    )
fi

if [[ "$use_default_board_layout" -eq 1 ]]; then
    echo "[_launch.sh] Using default 4-tag board layout (ids 0/1/2/3, offsets in metres)."
    cmd+=("${default_board_args[@]}")
fi

cmd+=("$@")
"${cmd[@]}"
