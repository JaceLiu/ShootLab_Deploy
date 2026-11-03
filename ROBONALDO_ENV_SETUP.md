# RoboNaldo_Deploy 环境配置说明（容器版）

机器：Unitree G1 机载 Orin NX 容器（JetPack 6 / L4T R36.4.3 / CUDA 12.6 / Ubuntu 22.04 / aarch64）
配置日期：2026-09-23

## 1. Python 环境

- conda 环境名：`robomimic`（Python 3.10），路径 `/opt/conda/envs/robomimic`
  （仓库脚本调用的是 `conda run -n robomimic`，名字必须保持）
- PyTorch（Jetson 专用轮子，CUDA 12.6）：
  - `torch==2.8.0`，`torchvision==0.23.0`
  - 校验：`torch.cuda.is_available() == True`，设备名 `Orin`
- 其他依赖：
  - `numpy==1.26.4`（<2），`scipy`，`PyYAML`，`hydra-core`，`omegaconf`
  - `pygame`，`pillow`，`opencv-python==4.10.0.84`
  - `onnx`，`onnxruntime`，`mujoco`
  - `pyrealsense2==2.58.4.10922`
- DDS：
  - 自建 CycloneDDS C 库 `0.10.5`，安装前缀 `/home/unitree/share/opt/cyclonedds-0.10.5`
  - Python 绑定 `cyclonedds==0.10.5`，编译时链接上述自建库
- Unitree SDK：
  - Python：`unitree_sdk2py`（源码 `/home/unitree/unitree_sdk2_python`，`pip install -e .`）

## 2. 为兼容仓库硬编码路径创建的软链接

仓库脚本里写死了若干宿主机路径，容器里用软链接对齐：

| 脚本期望路径 | 实际指向 |
|---|---|
| `/home/unitree/miniconda3` | `/opt/conda` |
| `/root/ws_livox`（`$HOME/ws_livox`） | `/workspace/unitree-jetson-container/ros2_ws` |
| `/root/Livox-SDK2`（`$HOME/Livox-SDK2`） | `.../ros2_ws/third_party/Livox-SDK2` |
| `/root/unitree_ros2`（`$HOME/unitree_ros2`） | `/opt/unitree_ros2` |

## 3. C++ Bridge

- 已克隆 `unitree_sdk2`（C++，含 aarch64 预编译库）到 `/home/unitree/unitree_sdk2-main`
- `bridge/` 已用该 SDK 编译成功，产物 `RoboNaldo_Deploy/bridge/build/cpp_bridge_main`

## 4. 注意事项

- 本机操作需 `source onboard/perception/setup_runtime_env.sh` 后运行，才会加载自建
  CycloneDDS（`/home/unitree/share/opt/cyclonedds-0.10.5/lib/libddsc.so.0`）与 conda 的 libstdc++。
- `requirements.txt` 里写的是 `numpy==1.20.0`（面向 Py3.8），此处按安装指南改用 `numpy<2`（1.26.4），
  未直接执行 `pip install -r requirements.txt`，以免破坏环境。
- `unitree_sdk2py` 元数据声明 `cyclonedds==0.10.2`，与仓库所需 `0.10.5` 冲突，故用 `--no-deps` 安装；
  运行期使用 0.10.5，导入验证通过。
- ROS2 humble 与 `unitree_hg` / `livox_ros_driver2` 消息均可导入；apriltag 检测走 OpenCV 内置 `cv2.aruco`，
  无需额外的 pupil-apriltags。
- `tools/start_tmux_layout.sh` 里的 `REPO_DIR` 默认值是别人机器上的路径
  (`/home/unitree/yichao/RoboMimic_Deploy`)，实际使用前需改成
  `/workspace/unitree-jetson-container/RoboNaldo_Deploy`（或 `export`/改脚本）。
- 下载 PyTorch 等海外资源时使用过一次代理 `127.0.0.1:33601`；代理为临时，非持久配置。

## 5. 快速自检命令

```bash
cd /workspace/unitree-jetson-container/RoboNaldo_Deploy
source onboard/perception/setup_runtime_env.sh
P=/opt/conda/envs/robomimic/bin/python
$P -c "import torch; print(torch.__version__, torch.cuda.is_available())"
$P -c "import cv2, pyrealsense2; print('vision ok')"
$P -c "from cyclonedds.domain import DomainParticipant; print('dds ok')"
$P -c "from common.ball_state_dds import BallStatePublisher; print('repo dds ok')"
```

## 6. 已修复：`deploy_policy.py` 段错误（Segmentation fault）

现象：直接运行 `python bridge/python/deploy_policy.py` 时，策略初始化完成后
（打印 `current policy is passive_mode`）立即段错误，崩溃点在
`cyclonedds/topic.py` 的 `Topic.__init__`。

根因：`cyclonedds` 这个 Python 绑定是按本仓库自建的 CycloneDDS 0.10.5 头文件编译的，
但运行时通过 `LD_LIBRARY_PATH` 解析到的却是 ROS 自带的
`/opt/ros/humble/lib/aarch64-linux-gnu/libddsc.so.0`。两者版本号都是 0.10.5，
但编译选项不同、ABI 不兼容，创建带 IDL 类型的 Topic 时崩溃。

修复：绑定自带的 `RPATH` 是 `/opt/conda/envs/robomimic/lib`，而 DT_RPATH 的搜索
优先级高于 `LD_LIBRARY_PATH`。于是把自建库软链进该目录：

```bash
ln -sfn /home/unitree/share/opt/cyclonedds-0.10.5/lib/libddsc.so.0         /opt/conda/envs/robomimic/lib/libddsc.so.0
ln -sfn /home/unitree/share/opt/cyclonedds-0.10.5/lib/libddsc.so           /opt/conda/envs/robomimic/lib/libddsc.so
ln -sfn /home/unitree/share/opt/cyclonedds-0.10.5/lib/libcycloneddsidl.so.0 /opt/conda/envs/robomimic/lib/libcycloneddsidl.so.0
ln -sfn /home/unitree/share/opt/cyclonedds-0.10.5/lib/libcycloneddsidl.so   /opt/conda/envs/robomimic/lib/libcycloneddsidl.so
```

这样无论是否 `source setup_runtime_env.sh`、ROS 是否在 `LD_LIBRARY_PATH` 中，
绑定都会稳定加载自建 libddsc。验证：

```bash
ldd $(python -c "import sysconfig, glob; print(glob.glob(sysconfig.get_path('platlib') + '/cyclonedds/_clayer*.so')[0])") | grep ddsc
# 期望: libddsc.so.0 => /opt/conda/envs/robomimic/lib/libddsc.so.0  (最终指向自建 0.10.5)
```

验证结果：`deploy_policy.py` 不再崩溃，会阻塞在 `wait_for_bridge_state()`——
这是正常行为，表示在等待 C++ bridge 发布 `rt/policy_bridge_state`。
另外 BallState / TargetState / BridgeState 三种 Topic 的收发 round-trip 均已验证通过。
