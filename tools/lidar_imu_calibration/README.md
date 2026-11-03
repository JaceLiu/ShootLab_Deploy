# G1 LiDAR-IMU 标定 Bag

本工具采集 Livox MID360 到 G1 torso IMU 的刚体动态标定数据。G1 只运行
ROS2 和 MCAP 录制；ROS2→ROS1 转换和 iKalibr 解算都在 PC 上进行。

PC 端接手、Docker 隔离策略和 A/B 解算流程见
[`PC_HANDOFF.md`](PC_HANDOFF.md)。

## 数据契约

G1 原始 MCAP 固定记录以下话题：

| 话题 | 类型 | 用途 |
|---|---|---|
| `/utlidar/cloud_livox_mid360` | `sensor_msgs/msg/PointCloud2` | MID360 点云 |
| `/secondary_imu` | `unitree_hg/msg/IMUState` | torso 参考 IMU |
| `/lowstate` | `unitree_hg/msg/LowState` | 腰部关节诊断 |
| `/utlidar/imu_livox_mid360` | `sensor_msgs/msg/Imu` | MID360 内置 IMU 诊断 |

真机 `/utlidar` 当前直接发布的字段为
`x/y/z/intensity/ring/time`，其中 `time` 是相对点云 header 的秒偏移。
转换器也兼容 Livox 驱动直出的
`x/y/z/intensity/tag/line/timestamp`，此时会把绝对纳秒时间归一化。
PC 转换后 `/g1/mid360` 固定为 `x/y/z/intensity/ring/time`，并匹配 iKalibr
`VLP_POINTS` 所需的 `ring:uint16`、`time:float32` 类型。

`/secondary_imu` 本身没有 header。转换器使用 rosbag2 接收时间为
`/g1/torso_imu` 生成 `sensor_msgs/Imu` header；gyro 保持 `rad/s`，accel 保持
`m/s²`。四元数从 Unitree 的 `w,x,y,z` 写入 ROS `x,y,z,w`。

数据默认写到仓库外的
`/workspace/unitree-jetson-container/ljc/calibration_data/<capture_id>/`。
每个目录包含 `.mcap`、`metadata.yaml`、`capture_manifest.yaml` 和验收结果
`analysis.yaml`。大 bag 不应提交 Git。

## G1 采集

先在机器人上停止 ball detector、fuser、FreeKick、策略以及额外的
`livox_ros_driver2_node`。不要把本工具与第二套 Livox 驱动同时启动。

```bash
cd /workspace/unitree-jetson-container/ljc/RoboNaldo_Deploy

# 15 秒静止烟雾测试
python tools/lidar_imu_calibration/capture_g1_lidar_imu.py \
  --profile smoke \
  --capture-id g1_mid360_smoke_01 \
  --operator "$USER"
```

封装工具默认 source `/opt/unitree_ros2/setup.sh`，设置
`rmw_cyclonedds_cpp`，检查四个话题类型和实时消息，再启动：

```text
ros2 bag record -s mcap -o <capture_dir> <four topics>
```

如果检测到策略或球感知进程，工具默认拒绝录制。确认机器人处于吊装或可靠保护状态
后，才可显式使用 `--stop-conflicts`。录制结束时工具向 `ros2 bag` 发送 SIGINT，
等待 MCAP 索引和 `metadata.yaml` 落盘，然后自动分析。也可直接按 Ctrl-C。

烟雾测试通过后，在 PC 完成一次转换并确认 `rosbag info`，才采集正式数据：

```bash
# A：10 秒静止；慢速 yaw、roll、pitch 往返；三轴组合；适量平移；10 秒静止
python tools/lidar_imu_calibration/capture_g1_lidar_imu.py \
  --profile A --capture-id g1_mid360_A_01 --operator "$USER"

# B：不同顺序和方向重复，安装与环境不得改变
python tools/lidar_imu_calibration/capture_g1_lidar_imu.py \
  --profile B --capture-id g1_mid360_B_01 --operator "$USER"
```

正式 A/B 必须为 90–120 秒。若机器人无法在策略停止时自主动作，应在稳固吊装状态下由
操作员缓慢完成腰部动作；不要在录制未关闭时断电、拔盘或解除保护。

单独重新分析：

```bash
python tools/lidar_imu_calibration/analyze_g1_lidar_imu.py \
  /workspace/unitree-jetson-container/ljc/calibration_data/g1_mid360_A_01 \
  --profile A
```

分析器检查：

- 点云平均频率 ≥9 Hz，p95 间隔 ≤120 ms，最大间隔 ≤250 ms。
- torso IMU 平均频率 ≥950 Hz，p99 间隔 ≤3 ms，最大间隔 ≤20 ms。
- 点云扫描内时间单调并位于 0–150 ms。
- 点云、torso IMU、lowstate 同时覆盖采集首尾，边缘缺失不超过 2 秒。
- A/B 的 x/y/z 三轴 gyro 均有足够激振，首尾 10 秒可辨认为静止。

## PC 转换

PC 需要 Python 3.10+：

```bash
python3 -m venv .venv-lidar-calib
source .venv-lidar-calib/bin/activate
pip install -r tools/lidar_imu_calibration/requirements-pc.txt
```

原始 MCAP 保留。ROS1 bag 默认写到 `calibration_data_ros1/`：

```bash
python tools/lidar_imu_calibration/convert_g1_lidar_imu.py \
  /workspace/unitree-jetson-container/ljc/calibration_data/g1_mid360_A_01

rosbag info calibration_data_ros1/g1_mid360_A_01_ros1.bag
```

必须同时看到：

```text
/g1/mid360       sensor_msgs/PointCloud2
/g1/torso_imu    sensor_msgs/Imu
```

若输出已存在，转换器拒绝覆盖；确认旧结果无用后才使用 `--overwrite`。

## iKalibr

使用本机 iKalibr 的真实配置结构生成 A/B 配置：

```bash
python tools/lidar_imu_calibration/generate_ikalibr_config.py \
  --bag calibration_data_ros1/g1_mid360_A_01_ros1.bag \
  --output-dir calibration_data_ros1/g1_mid360_A_01_ikalibr \
  --ikalibr-root /workspace/unitree-jetson-container/ljc/iKalibr

python tools/lidar_imu_calibration/generate_ikalibr_config.py \
  --bag calibration_data_ros1/g1_mid360_B_01_ros1.bag \
  --output-dir calibration_data_ros1/g1_mid360_B_01_ikalibr \
  --ikalibr-root /workspace/unitree-jetson-container/ljc/iKalibr
```

生成配置使用 `VLP_POINTS`、`SENSOR_IMU`、`ReferIMU: /g1/torso_imu` 和
`TimeOffsetPadding: 0.10`。`VLP_POINTS` 的 PCL 路径要求转换后的 `ring/time`；
Livox 自定义消息类型不适用于本契约。

在 ROS1/iKalibr 环境中分别运行 A 和 B：

```bash
roslaunch ikalibr ikalibr-prog.launch \
  config_path:=/path/to/g1_mid360_A_01_ikalibr/config-ikalibr.yaml
```

最后比较两次独立解算结果：

```bash
python tools/lidar_imu_calibration/compare_extrinsics.py \
  A/iteration/stage/<final>.yaml B/iteration/stage/<final>.yaml
```

比较器也接受带以下字段的整理结果 YAML：

```yaml
rotation_matrix: [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
translation_m: [0.0, 0.0, 0.0]
```

旋转差必须 ≤1°，平移差必须 ≤2 cm。未通过时，`lidar_calibration.yaml` 必须保持
“未验证”，不得进入 FreeKick 真机测试。
