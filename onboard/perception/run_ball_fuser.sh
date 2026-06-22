#!/usr/bin/env bash
# ============================================================
# 启动 ball_fuser — 融合雷达 + 胸部相机球位置估计
#
# 订阅：
#   rt/lidar_ball_state  (lidar ball_detector 输出)
#   rt/cam_ball_state    (apriltag_detector --ball-bright 输出)
#
# 发布：
#   rt/ball_state        (deploy_policy.py 读取)
#
# 用法：
#   bash onboard/perception/run_ball_fuser.sh
#   bash onboard/perception/run_ball_fuser.sh --lidar-max-range 1.5
# ============================================================
set -e
cd "$(dirname "$0")/../.."

source onboard/perception/setup_runtime_env.sh

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

source /home/unitree/miniconda3/etc/profile.d/conda.sh 2>/dev/null || true

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=0
CYCLONEDDS_IFACE="${CYCLONEDDS_IFACE:-$(ip -o -4 addr show scope global 2>/dev/null | awk '/192\.168\.123\./ {print $2; exit}')}"
CYCLONEDDS_IFACE="${CYCLONEDDS_IFACE:-enP8p1s0}"
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"$CYCLONEDDS_IFACE\" priority=\"default\" multicast=\"default\" /></Interfaces></General></Domain></CycloneDDS>"
echo "[run_ball_fuser] CycloneDDS interface: $CYCLONEDDS_IFACE"

conda run -n robomimic --no-capture-output \
    python -u onboard/perception/ball_fuser.py "$@"
