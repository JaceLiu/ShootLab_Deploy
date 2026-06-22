# Onboard Install Guide for Agents

This guide is for installing the onboard perception/runtime stack on another
Unitree G1 Jetson robot as quickly as possible. It records the working path from
the JetPack 6 / Orin install in this repo.

Read this guide together with:

- `onboard/docs/README.md` for the runtime architecture and topic ownership.
- `tools/start_tmux_layout.sh` for the executable real-robot bring-up layout.

After installation, the acceptance test is not just importing packages. Start
the tmux layout, bring up camera, LiDAR, the single fuser, dashboard, and
`tools/check_ball_state.py`, then confirm the final `rt/ball_state` stream is
fresh and stable.

## Target Machine

First identify the robot. Do not blindly follow older JetPack 5 instructions.

```bash
cat /etc/nv_tegra_release
cat /etc/os-release
nvcc --version
```

Known-good setup from this install:

- Ubuntu 22.04 / JetPack 6, L4T R36.4.3
- CUDA 12.6
- Python 3.10 conda environment named `robomimic`
- RealSense D435I on USB 3.2
- Python DDS: `cyclonedds==0.10.5`, linked against the user-built
  `/home/unitree/share/opt/cyclonedds-0.10.5/lib/libddsc.so.0`
- ROS2 RMW: `rmw_cyclonedds_cpp`

If the robot is JetPack 5.1.2 / CUDA 11.4, use the older notes in
`onboard/perception/camera/TROUBLESHOOTING.md` instead of the PyTorch commands
below.

## DDS Version Reference

There are three DDS-related pieces in the runtime. Keep them separate when
installing a new robot:

- Repo Python topics such as `rt/ball_state`, `rt/lidar_ball_state`,
  `rt/cam_ball_state`, and `rt/target_state` use the Python package
  `cyclonedds==0.10.5`.
- That Python package must be compiled against CycloneDDS C library `0.10.5`
  installed at `/home/unitree/share/opt/cyclonedds-0.10.5`. Do not rely on
  Ubuntu Jammy's `cyclonedds-dev` for this path.
- ROS2 nodes still use `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`. The launch
  scripts set `CYCLONEDDS_URI` to the robot network interface to avoid FastDDS
  shared-memory OOM behavior on 16 GB Jetson systems.
- The C++ robot bridge links Unitree SDK2's bundled DDS libraries from
  `${UNITREE_SDK2_DIR}/thirdparty/lib/$(uname -m)`. That is separate from the
  Python `cyclonedds` package used by repo-level perception topics.

Verify the current Python DDS binding with:

```bash
source onboard/perception/setup_runtime_env.sh
conda run -n robomimic --no-capture-output python -m pip show cyclonedds
ldd $(conda run -n robomimic python -c \
  "import sysconfig, glob; print(glob.glob(sysconfig.get_path('platlib') + '/cyclonedds/_clayer*.so')[0])") | grep ddsc
```

Expected `ldd` path:

```text
/home/unitree/share/opt/cyclonedds-0.10.5/lib/libddsc.so.0
```

## Fast Install

Use `uv` for pip packages. Keep the env name `robomimic` because repo scripts
call `conda run -n robomimic`.

```bash
source /home/unitree/miniconda3/etc/profile.d/conda.sh
conda create -n robomimic -c conda-forge python=3.10 pip -y

conda install -n robomimic -c conda-forge \
  numpy scipy pyyaml hydra-core omegaconf pygame pillow opencv pyrealsense2 -y

uv pip install --python /home/unitree/miniconda3/envs/robomimic/bin/python \
  --index-url https://pypi.jetson-ai-lab.io/jp6/cu126 \
  "torch==2.8.0" "torchvision==0.23.0"

uv pip install --python /home/unitree/miniconda3/envs/robomimic/bin/python \
  "numpy<2" onnx onnxruntime mujoco pupil-apriltags
```

Do not use the default PyPI `torch` on Jetson. It will either be CPU-only or
pull a wheel that expects libraries missing from the robot.

## CycloneDDS Python Binding

Ubuntu Jammy's `cyclonedds-dev` is too old for `cyclonedds==0.10.5`. Build
CycloneDDS 0.10.5 into a user prefix, then compile the Python binding against it.
Disable shared memory support to avoid missing `iceoryx` headers.

```bash
mkdir -p /home/unitree/share/src /home/unitree/share/opt
git clone --branch 0.10.5 --depth 1 \
  https://github.com/eclipse-cyclonedds/cyclonedds.git \
  /home/unitree/share/src/cyclonedds-0.10.5

cmake -S /home/unitree/share/src/cyclonedds-0.10.5 \
  -B /home/unitree/share/src/cyclonedds-0.10.5/build \
  -DCMAKE_INSTALL_PREFIX=/home/unitree/share/opt/cyclonedds-0.10.5 \
  -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF -DENABLE_SHM=OFF \
  -DCMAKE_BUILD_TYPE=Release

LD_LIBRARY_PATH=/home/unitree/share/src/cyclonedds-0.10.5/build/lib \
  cmake --build /home/unitree/share/src/cyclonedds-0.10.5/build -j"$(nproc)"

LD_LIBRARY_PATH=/home/unitree/share/src/cyclonedds-0.10.5/build/lib \
  cmake --install /home/unitree/share/src/cyclonedds-0.10.5/build

CYCLONEDDS_HOME=/home/unitree/share/opt/cyclonedds-0.10.5 \
LD_LIBRARY_PATH=/home/unitree/share/opt/cyclonedds-0.10.5/lib:/home/unitree/miniconda3/envs/robomimic/lib \
  uv pip install --python /home/unitree/miniconda3/envs/robomimic/bin/python \
  --no-build-isolation --no-cache "cyclonedds==0.10.5"
```

Verify that Python links to the user-built library:

```bash
source onboard/perception/setup_runtime_env.sh
conda run -n robomimic --no-capture-output python -c \
  "from cyclonedds.domain import DomainParticipant; DomainParticipant(0); print('cyclonedds ok')"

ldd $(conda run -n robomimic python -c \
  "import sysconfig, glob; print(glob.glob(sysconfig.get_path('platlib') + '/cyclonedds/_clayer*.so')[0])") | grep ddsc
```

Expected `ldd` path:

```text
/home/unitree/share/opt/cyclonedds-0.10.5/lib/libddsc.so.0
```

## Runtime Path Setup

Use the shared script added in this repo:

```bash
source onboard/perception/setup_runtime_env.sh
```

It keeps conda's `libstdc++` before Ubuntu's copy, adds CycloneDDS 0.10.5, and
adds Jetson CUDA/TensorRT runtime library paths. This avoids common errors like:

- `CXXABI_1.3.15 not found` from OpenCV
- `undefined symbol: DDS_XTypes_TypeObject_desc` from CycloneDDS
- accidental import of system `protobuf` or `cv2`

## RealSense Setup

In this install, RealSense is not a separate repo workspace under `$HOME`.
The runtime pieces are:

- Python package: conda env `robomimic`, under
  `/home/unitree/miniconda3/envs/robomimic/lib/python3.10/site-packages/pyrealsense2`
- Udev rule, if installed: `/etc/udev/rules.d/99-realsense-libusb.rules`
- Camera device for the grayscale ball/target pipeline: `/dev/video3`
  (`GREY`, 30 Hz by default)

Install udev rules if device access fails:

```bash
curl -L -o /tmp/99-realsense-libusb.rules \
  https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules
echo "123" | sudo -S cp /tmp/99-realsense-libusb.rules /etc/udev/rules.d/99-realsense-libusb.rules
echo "123" | sudo -S udevadm control --reload-rules
echo "123" | sudo -S udevadm trigger
```

Unplug/replug the camera, then verify:

```bash
source onboard/perception/setup_runtime_env.sh
conda run -n robomimic --no-capture-output python - <<'PY'
import pyrealsense2 as rs
ctx = rs.context()
print("devices", len(ctx.devices))
for d in ctx.devices:
    print("name", d.get_info(rs.camera_info.name))
    print("serial", d.get_info(rs.camera_info.serial_number))
    print("usb", d.get_info(rs.camera_info.usb_type_descriptor))
pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
profile = pipe.start(cfg)
frames = pipe.wait_for_frames(5000)
print("pipeline ok", bool(frames.get_color_frame()), bool(frames.get_depth_frame()))
pipe.stop()
PY
```

Expected: one D435/D435I/D455 device, USB `3.x`, and `pipeline ok True True`.

## ROS2 / LiDAR / Unitree Messages

The AprilTag target publisher, camera ball publisher, and LiDAR detector import:

- `rclpy`
- `unitree_hg.msg`
- `sensor_msgs`
- `livox_ros_driver2`

If `/opt/ros` and `ws_livox` are missing, those scripts will not fully start.
Install or restore the robot's ROS2 + Unitree message workspace before running:

```bash
bash onboard/perception/camera/_launch.sh --show
bash onboard/perception/lidar/run.sh
bash onboard/perception/run_sensor_dashboard.sh
```

Known-good LiDAR runtime layout:

```bash
# ROS2: prefer Humble on JetPack 6, with Foxy fallback only for older robots.
source /opt/ros/humble/setup.bash

# Livox MID360 ROS2 driver workspace:
# source tree: $HOME/ws_livox/src/livox_ros_driver2
# install tree: $HOME/ws_livox/install
export LIVOX_WS="${LIVOX_WS:-$HOME/ws_livox}"
source "$LIVOX_WS/install/setup.bash"

# Unitree ROS2 message workspace, needed for unitree_hg.msg imports.
export UNITREE_ROS2_WS="${UNITREE_ROS2_WS:-$HOME/unitree_ros2/cyclonedds_ws}"
source "$UNITREE_ROS2_WS/install/setup.bash"

# Livox SDK2 runtime library, needed by livox_ros_driver2_node:
# source/build tree: $HOME/Livox-SDK2
export LIVOX_SDK2_LIB="${LIVOX_SDK2_LIB:-$HOME/Livox-SDK2/build/sdk_core}"
export LD_LIBRARY_PATH="$LIVOX_SDK2_LIB:$LD_LIBRARY_PATH"

# MID360 config used by onboard/perception/lidar/run.sh.
export LIVOX_CONFIG="${LIVOX_CONFIG:-$LIVOX_WS/src/livox_ros_driver2/config/MID360_config.json}"
test -f "$LIVOX_CONFIG"
```

Quick checks:

```bash
ros2 pkg prefix livox_ros_driver2
ros2 interface show livox_ros_driver2/msg/CustomMsg
python -c "import rclpy; import sensor_msgs.msg; from livox_ros_driver2.msg import CustomMsg; from unitree_hg.msg import LowState; print('ros/lidar imports ok')"
```

The repo launcher normally handles these `source` and `LD_LIBRARY_PATH` steps.
If LiDAR fails on a new robot, check these paths first:

- `/opt/ros/humble/setup.bash` or `/opt/ros/foxy/setup.bash`
- `$HOME/ws_livox/install/setup.bash`
- `$HOME/ws_livox/src/livox_ros_driver2/config/MID360_config.json`
- `$HOME/Livox-SDK2/build/sdk_core`
- `$HOME/unitree_ros2/cyclonedds_ws/install/setup.bash`

After ROS2 is present, keep:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="eth0" priority="default" multicast="default" /></Interfaces></General></Domain></CycloneDDS>'
```

This avoids FastDDS shared-memory OOM behavior on 16 GB Jetson systems.

## Final Checklist

```bash
source onboard/perception/setup_runtime_env.sh

conda run -n robomimic --no-capture-output python -c \
  "import torch; print(torch.__version__, torch.cuda.is_available(), torch.cuda.get_device_name(0))"

conda run -n robomimic --no-capture-output python -c \
  "import cv2, pyrealsense2; print('vision stack ok')"

conda run -n robomimic --no-capture-output python -c \
  "from cyclonedds.domain import DomainParticipant; DomainParticipant(0); print('dds ok')"

conda run -n robomimic --no-capture-output python -c \
  "from common.ball_state_dds import BallStatePublisher; from common.target_state_dds import TargetStatePublisher; print('repo dds ok')"
```

If all four pass, the conda/runtime side is installed. Then validate hardware
with the RealSense smoke test and, if ROS2 is available, the onboard perception
scripts.
