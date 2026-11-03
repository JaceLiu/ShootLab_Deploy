# G1 MID360–Torso IMU 标定：PC 端接手文档

更新日期：2026-11-03

## 1. 接手目标

PC 端负责以下工作，G1 不安装或运行 iKalibr：

1. 接收 G1 采集的 ROS2 MCAP。
2. 转换为 iKalibr 可读取的 ROS1 bag。
3. 生成 A、B 两套 iKalibr 配置并独立解算。
4. 比较两次外参结果。
5. 只有 A/B 旋转差 ≤1°、平移差 ≤2 cm，才允许把结果标记为已验证。

不要修改策略或 FreeKick 参数，也不要在验证前写入生产标定文件。

## 2. 当前状态

G1 已完成第一段 15 秒静止烟雾采集：

```text
/workspace/unitree-jetson-container/ljc/calibration_data/g1_mid360_smoke_01/
```

关键结果：

| 项目 | 结果 |
|---|---:|
| MCAP 大小 | 99,934,008 bytes |
| MCAP SHA256 | `2e2bb0107336e446bc6801750353d44bcd84533778f322d7eec9f5529ce6de17` |
| 点云频率 | 9.98 Hz |
| torso IMU 频率 | 1052.7 Hz |
| lowstate 频率 | 1052.7 Hz |
| 四路覆盖率 | 99.79% |
| 点云扫描时间 | 0.005–100.32 ms |
| 烟雾验收 | `passed` |

对应文件：

```text
g1_mid360_smoke_01_0.mcap
metadata.yaml
capture_manifest.yaml
analysis.yaml
```

这段数据只用于验证转换链路，不用于最终标定。当前还没有正式 A/B 数据。

## 3. PC Docker 应该怎么选

### 结论

- **通常不需要重新构建 Docker 镜像。**
- **建议新建一个专用的 iKalibr 容器。**
- 如果 PC 上已经有一个专门运行 iKalibr、且已验证可用、没有混装生产代码的容器，可以直接复用。
- **不要把 iKalibr 直接装进现有生产策略、ROS2 或训练容器。**
- “新建容器”不等于“新建镜像”。优先直接使用官方镜像：

```text
ulong2/ie_kalibr_image:latest
```

iKalibr 是 ROS1/catkin 工具链；G1 生产工程使用 ROS2 Humble。两者依赖不同，
不应为了省一个容器而污染生产环境。

### 容器是否隔离

Docker 容器默认与宿主机文件系统隔离。执行 `apt`、`pip` 或 catkin 编译只会修改
该容器的可写层，不会直接修改宿主机。但以下操作会让影响越过容器：

- 把宿主机项目或 `$HOME` 目录挂载进容器后直接改文件。
- 使用 `docker commit` 把临时修改固化到镜像。
- 使用 `--privileged`、共享 Docker socket 或过度挂载。
- 把输出路径指向生产目录。

因此推荐：

- 专用容器名，例如 `g1-lidar-calib`。
- 只挂载标定数据、输出目录和必要的工具源码。
- 不挂载整个 `$HOME`，不使用 `--privileged`。
- iKalibr 不需要机器人 DDS，不使用 `--network=host`。
- 输出放在独立目录，例如 `$HOME/g1-lidar-calib/results/`。

### 先检查 PC 现有环境

在 PC 宿主机执行：

```bash
docker info
docker ps -a
docker images
```

检查现有容器是否是 iKalibr ROS1 环境：

```bash
docker exec -it <container-name> bash
source /opt/ros/noetic/setup.bash
command -v roslaunch
rosrun ikalibr ikalibr_prog --help
```

如果以上命令可用，并且该容器只用于标定工具，可直接复用。否则创建专用容器，
不要在已有生产容器中临时安装 iKalibr。

## 4. PC 目录布局

先区分两个文件系统：

- `/workspace/unitree-jetson-container/ljc/...` 是 **G1 机器人上的路径**。
- `$HOME/g1-lidar-calib` 是 PC 本地目录。

PC 上建议把仓库、数据和结果放在同一个目录：

```text
$HOME/g1-lidar-calib/
├── RoboNaldo_Deploy/
├── calibration_data/
│   ├── g1_mid360_smoke_01/
│   ├── g1_mid360_A_01/
│   └── g1_mid360_B_01/
├── ros1/
│   ├── g1_mid360_A_01_ros1.bag
│   └── g1_mid360_B_01_ros1.bag
├── results/
│   ├── A/
│   └── B/
```

`RoboNaldo_Deploy` 可以从 Git 克隆或从机器人复制到 PC，并作为标定专用副本使用。

不需要在 `g1-lidar-calib` 中生成 Docker 容器。镜像和容器由 Docker 统一管理；
把整个目录映射到容器的 `/work` 后，容器内会看到：

```text
/work/RoboNaldo_Deploy
/work/calibration_data
/work/ros1
/work/results
```

## 5. 从 G1 接收烟雾数据

由有权限的操作人执行，`<ROBOT_HOST>` 替换为实际 SSH 地址：

```bash
mkdir -p "$HOME/g1-lidar-calib/calibration_data"
scp -r <ROBOT_HOST>:/workspace/unitree-jetson-container/ljc/calibration_data/g1_mid360_smoke_01 \
  "$HOME/g1-lidar-calib/calibration_data/"
```

检查文件：

```bash
cd "$HOME/g1-lidar-calib/calibration_data/g1_mid360_smoke_01"
ls -lh
sha256sum g1_mid360_smoke_01_0.mcap
```

SHA256 必须与 `capture_manifest.yaml` 一致。原始 MCAP 不得覆盖或删除。

## 6. PC 端 ROS2→ROS1 转换

转换工具只需要 Python，不需要 ROS2 或 ROS1 运行时。建议在 PC 宿主机使用独立
Python 虚拟环境，避免修改系统 Python：

```bash
cd "$HOME/g1-lidar-calib/RoboNaldo_Deploy"
python3 -m venv .venv-lidar-calib
source .venv-lidar-calib/bin/activate
pip install -r tools/lidar_imu_calibration/requirements-pc.txt
```

转换烟雾 bag：

```bash
python tools/lidar_imu_calibration/convert_g1_lidar_imu.py \
  "$HOME/g1-lidar-calib/calibration_data/g1_mid360_smoke_01" \
  --output "$HOME/g1-lidar-calib/ros1/g1_mid360_smoke_01_ros1.bag"
```

验证 ROS1 bag。若宿主机没有 ROS1 `rosbag` 命令，进入后文的 iKalibr ROS1 容器执行：

```bash
# ROS1 已安装在宿主机时
rosbag info "$HOME/g1-lidar-calib/ros1/g1_mid360_smoke_01_ros1.bag"

# 只能在 iKalibr 容器内执行时
rosbag info /work/ros1/g1_mid360_smoke_01_ros1.bag
```

必须同时存在：

```text
/g1/mid360       sensor_msgs/PointCloud2
/g1/torso_imu    sensor_msgs/Imu
```

点云契约：

```text
x/y/z/intensity  float32
ring             uint16
time             float32，扫描内秒偏移
```

IMU 契约：

- header 使用 rosbag2 消息接收时间。
- orientation 为 `x,y,z,w`。
- gyro 单位 `rad/s`。
- accel 单位 `m/s²`。

烟雾转换未通过时，不要申请正式 A/B 采集。

## 7. 启动专用 iKalibr 容器

首次启动时，把整个标定目录挂载为容器内的 `/work`：

```bash
CALIB_ROOT="$HOME/g1-lidar-calib"

xhost +local:docker

docker run -it \
  --name g1-lidar-calib \
  --shm-size=16g \
  -e DISPLAY="$DISPLAY" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$HOME/.Xauthority:/root/.Xauthority:rw" \
  -v "$CALIB_ROOT:/work:rw" \
  --device=/dev/dri \
  ulong2/ie_kalibr_image:latest \
  bash
```

以后重新进入已有容器：

```bash
docker start -ai g1-lidar-calib
```

或者：

```bash
docker exec -it g1-lidar-calib bash
```

iKalibr 默认会启动 viewer，因此通常需要可用的 X11 `DISPLAY`。如果 PC 没有
`/dev/dri`，移除 `--device=/dev/dri`；这不是当前配置的必要条件，因为生成的配置
默认 `UseCudaInSolving: false`。

完成后可在宿主机撤销临时 X11 权限：

```bash
xhost -local:docker
```

进入 iKalibr 环境：

```bash
cd /work
ls RoboNaldo_Deploy calibration_data ros1 results

ls /home/iKalibr/install
source /home/iKalibr/install/setup.bash
rosrun ikalibr ikalibr_prog --help
```

如果官方镜像只有 `setup.sh`：

```bash
source /home/iKalibr/install/setup.sh
```

不要把该容器当作生产安装环境继续安装其他项目依赖。需要额外依赖时，创建派生镜像
或临时一次性容器，不要执行 `docker commit` 后覆盖生产镜像。

## 8. 生成 iKalibr 配置

PC 收到正式 A/B bag 后，在 iKalibr 容器内生成配置。路径必须使用容器内可见路径：

```bash
cd /work/RoboNaldo_Deploy

python3 tools/lidar_imu_calibration/generate_ikalibr_config.py \
  --bag /work/ros1/g1_mid360_A_01_ros1.bag \
  --output-dir /work/results/A \
  --ikalibr-root /home/iKalibr

python3 tools/lidar_imu_calibration/generate_ikalibr_config.py \
  --bag /work/ros1/g1_mid360_B_01_ros1.bag \
  --output-dir /work/results/B \
  --ikalibr-root /home/iKalibr
```

配置契约：

```text
LiDAR:   VLP_POINTS
IMU:     SENSOR_IMU
ReferIMU: /g1/torso_imu
TimeOffsetPadding: 0.10
```

检查生成文件：

```bash
sed -n '1,180p' /work/results/A/config-ikalibr.yaml
sed -n '1,180p' /work/results/B/config-ikalibr.yaml
```

## 9. 正式 A/B 解算

必须对 A、B 独立解算，不能让 B 继承 A 的结果。

```bash
roslaunch ikalibr ikalibr-prog.launch \
  config_path:=/work/results/A/config-ikalibr.yaml
```

完成后单独运行 B：

```bash
roslaunch ikalibr ikalibr-prog.launch \
  config_path:=/work/results/B/config-ikalibr.yaml
```

iKalibr 会输出逐阶段参数，重点文件为：

```text
results/A/iteration/stage/*.yaml
results/B/iteration/stage/*.yaml
```

选择最终稳定阶段的文件，不要误用初始估计结果。

## 10. A/B 一致性验收

```bash
python tools/lidar_imu_calibration/compare_extrinsics.py \
  /work/results/A/iteration/stage/<final-A>.yaml \
  /work/results/B/iteration/stage/<final-B>.yaml
```

通过条件：

```text
rotation_difference_deg  <= 1.0
translation_difference_m <= 0.02
```

脚本会尝试直接解析 iKalibr 的 `SO3_LkToBr` 和 `POS_LkInBr`。

若无法自动识别最终 YAML，可整理为：

```yaml
rotation_matrix:
  - [1.0, 0.0, 0.0]
  - [0.0, 1.0, 0.0]
  - [0.0, 0.0, 1.0]
translation_m: [0.0, 0.0, 0.0]
```

## 11. 交付物

PC 端最终交付：

1. 原始 smoke/A/B MCAP 保留。
2. A/B ROS1 bag。
3. A/B iKalibr 配置。
4. A/B 最终参数 YAML。
5. `compare_extrinsics.py` 输出。
6. 最终 `R_lidar_to_torso`、`t_lidar_in_torso`。
7. 旋转差、平移差、时间偏移和求解耗时。

未通过 A/B 一致性时：

- 保持 `lidar_calibration.yaml` 为“未验证”。
- 不进入 FreeKick 真机测试。
- 报告哪一轴激振不足、时间偏移异常或哪次解算未收敛。

## 12. 常见阻断

| 现象 | 首先检查 |
|---|---|
| `roslaunch` 找不到 ikalibr | 是否 source `/home/iKalibr/install/setup.bash` |
| 找不到 ROS1 | 当前是否误进 ROS2 Humble 容器 |
| bag 中话题不存在 | 是否运行了转换脚本，而不是直接给 iKalibr MCAP |
| PCL 读取点云失败 | `ring` 是否 uint16、`time` 是否 float32 |
| 时间偏移贴近 ±0.1 s | 是否误用了未归一化的绝对纳秒时间 |
| A/B 结果差异过大 | 场景、安装、运动方式是否变化，最终参数文件是否正确 |
| 生产容器被改动 | 停止在该容器中安装依赖，改用独立 iKalibr 容器 |
