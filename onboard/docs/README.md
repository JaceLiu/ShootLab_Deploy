# Onboard — 机载感知服务

本目录包含运行在 **G1 机载电脑**上的感知服务代码。
服务启动后通过 **DDS**（与 Unitree SDK 同网段）实时发布球的位置，
本地电脑连接网线后即可直接订阅，无需 SSH 进机器手动启动额外进程。

---

## 新人阅读路径

如果第一次接手这套机载感知/真机部署，请按下面顺序看：

1. 先读本文件，理解运行时拓扑：`camera/lidar raw topic -> ball_fuser -> rt/ball_state`。
2. 再按 `onboard/docs/INSTALL_AGENT_GUIDE.md` 配新机器环境。安装指南是环境配置的唯一权威入口。
3. 环境装好后，看 `tools/start_tmux_layout.sh`。它是真机 bring-up 的可执行拓扑，里面每个 pane 对应一个运行组件。
4. 启动 `bash tools/start_tmux_layout.sh`，按 pane 逐个回车启动服务。
5. 用 `monitor` 窗口里的 `tools/check_ball_state.py` 和 Sensor Dashboard 验证最终 `rt/ball_state`。

常见排障文档：

- 相机/灰度/AprilTag：`onboard/perception/camera/README.md`
- 相机性能与 RealSense 问题：`onboard/perception/camera/TROUBLESHOOTING.md`
- LiDAR 发布频率：`onboard/docs/LIDAR_FREQ_TROUBLESHOOTING.md`
- 感知架构图：`onboard/docs/PERCEPTION_ARCHITECTURE.md`

---

## 目录结构

```
onboard/
└── perception/
    ├── lidar/
    │   ├── ball_detector.py      ← MID360 点云 → raw lidar 球心 → DDS 发布
    │   ├── rviz_publisher.py     ← RViz2 可视化：点云 + Marker 发布
    │   ├── center_kalman_filter.py ← 卡尔曼滤波平滑球心轨迹
    │   ├── mid360_to_base.py     ← 坐标变换：MID360 系 → pelvis (base) 系
    │   └── README.md             ← Lidar 使用手册（含 RViz2 配置）
    └── camera/
        ├── run_gray.sh              ← 入口：灰度 AprilTag + 亮球（+可选 fuser）
        ├── _launch.sh               ← 内部：环境 + 默认参数，启动 detector
        ├── target_ball_detector.py  ← 干活：AprilTag target + 灰度亮球 → DDS
        ├── camera_to_base.py        ← 坐标变换：相机系 → pelvis 系（含胸部占位外参）
        ├── README.md                ← Camera 使用手册（本地文档）
        └── TROUBLESHOOTING.md       ← 性能优化全记录
```

---

## DDS 消息与 Topic

| 字段 | 类型 | 含义 |
|------|------|------|
| `timestamp_us` | uint64 | 发布时刻（µs since epoch） |
| `x / y / z` | float32 | 球心在 **pelvis body 系** 的坐标（m） |
| `valid` | uint8 | `1` = 本帧检测到球；`0` = 未检测到，位置为上一帧 EMA 值 |

- **Raw lidar topic**：`rt/lidar_ball_state`
- **Raw camera topic**：`rt/cam_ball_state`
- **Final policy topic**：`rt/ball_state`（由 `onboard/perception/ball_fuser.py` 发布）
- **QoS**：BestEffort，KeepLast(1)
- **频率**：约 10 Hz（受 Livox MID360 点云帧率限制）

消息定义和 Publisher / Subscriber 封装见 `common/ball_state_dds.py`。

---

## 环境安装入口

新机器安装请以 `onboard/docs/INSTALL_AGENT_GUIDE.md` 为唯一入口。那里记录了
当前真机验证过的 JetPack、Python、CycloneDDS、ROS2、Livox 和 RealSense 路径。

这里仅保留运行时要点，避免两套安装说明互相冲突：

- Python DDS 使用 `cyclonedds==0.10.5`，运行时链接到
  `/home/unitree/share/opt/cyclonedds-0.10.5/lib/libddsc.so.0`。
- ROS2 节点使用 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`，脚本会按机器人网卡生成
  `CYCLONEDDS_URI`。
- Livox MID360 驱动默认位于 `$HOME/ws_livox`，SDK2 runtime lib 默认位于
  `$HOME/Livox-SDK2/build/sdk_core`。
- RealSense 灰度链路默认使用 `/dev/video3`，`GREY`，30 Hz。
- 启动脚本会自动 source 需要的 ROS2 / Livox / Unitree message workspace；手动排障时
  再按 `INSTALL_AGENT_GUIDE.md` 逐项检查。

---

## 快速启动

### 推荐：tmux 总控

在仓库根目录运行：

```bash
bash tools/start_tmux_layout.sh
```

`tools/start_tmux_layout.sh` 是当前真机运行拓扑的参考实现。文档里的手动命令应与它保持一致。

该脚本会预填一套真机 bring-up 窗格：

- C++ bridge：机器人低层状态/控制桥接。
- `deploy_policy.py`：策略推理与 FSM。
- Camera raw perception：发布 `rt/target_state` 和 `rt/cam_ball_state`。
- LiDAR raw perception：发布 `rt/lidar_ball_state`。
- Ball fuser：唯一最终球位置发布者，发布 `rt/ball_state`。
- Sensor Dashboard：浏览器查看传感器与偏置。
- Monitor：运行 `tools/check_ball_state.py` 观察最终 `rt/ball_state`。

### 手动分进程启动

如果不使用 tmux，按下面的职责拆开启动。**同一时间只启动一个 fuser**。

```bash
# Camera: AprilTag target + camera raw ball
bash onboard/perception/camera/run_gray.sh --show

# LiDAR: Livox driver + lidar raw ball
bash onboard/perception/lidar/run.sh --show --base-y-bias 0.00 --dds-topic rt/lidar_ball_state

# Final fuser: lidar/camera raw -> rt/ball_state
bash onboard/perception/run_ball_fuser.sh

# Optional dashboard
bash onboard/perception/run_sensor_dashboard.sh
```

灰度相机链路默认使用 `/dev/video3`、`GREY`、`30Hz`，发布 AprilTag target 到
`rt/target_state`，发布 camera raw ball 到 `rt/cam_ball_state`。LiDAR 链路发布
`rt/lidar_ball_state`。`ball_fuser.py` 按 `lidar > camera` 选择原始观测，并统一
Kalman 平滑后发布策略真正读取的 `rt/ball_state`。

`run_gray.sh --with-fuser` 仍可用于相机单独调试，但在 tmux 或手动已启动
`run_ball_fuser.sh` 时不要使用，避免多个 fuser 同时发布 `rt/ball_state`。

---

## 本地验证

本地电脑通过网线连接 G1 后，在仓库根目录运行：

```bash
python tools/check_ball_state.py
```

正常输出示例：

```
[OK ]  x=+0.823  y=-0.012  z=-0.673  age=85ms
```

- `age` 为数据距当前时刻的延迟，超过 300ms 则显示 `---`（数据过期）。
- 确认能稳定收到数据后，再启动 `deploy_real/deploy_real.py`。

---

## 感知流程（lidar 方案）

```
Livox MID360（点云，~10 Hz）
        │
        │ ROI 滤波 + 反射率阈值
        ▼
候选点云（高反射率球面点）
        │
        │ 最小二乘球心拟合（已知半径 0.115m）
        ▼
球心（MID360 坐标系）
        │
        │ EMA 时间滤波（α=0.6，跳变门限 0.6m）
        ▼
球心（MID360 坐标系，平滑后）
        │
        │ transform_point_mid360_to_base()
        │ （链式正运动学：pelvis → waist → torso → head → MID360）
        │ 使用实时关节角 q_wy / q_wr / q_wp / q_head
        ▼
球心（pelvis body 系，raw lidar）
        │
        │ DDS publish "rt/lidar_ball_state"
        ▼
ball_fuser.py（可同时接收 rt/cam_ball_state）
        │
        │ DDS publish "rt/ball_state"
        ▼
policy runtime → state_cmd.ball_pos_b → FreeKick._build_obs()
```

---

## 添加新的感知方案

如需新增其他感知方式，只需：

1. 在 `onboard/perception/<方案>/` 下新建 `ball_detector.py`
2. 用任意方式获取球在 pelvis 系的坐标
3. 发布到该传感器自己的 raw topic，不要直接发布最终 `rt/ball_state`
4. 在 `ball_fuser.py` 中接入该 raw topic，并决定优先级/距离门限

```python
from common.ball_state_dds import BallStatePublisher, SOURCE_CAM

dds = BallStatePublisher(topic_name="rt/<sensor>_ball_state")
dds.publish(x, y, z, valid=True, source=SOURCE_CAM)
```

如果新增的是第三类传感器，需要先扩展 `common/ball_state_dds.py` 中的 source 常量，
再让 fuser 和 dashboard 识别它；不要复用错误的 source 值。

只要最终仍由 `ball_fuser.py` 发布 `rt/ball_state`，`deploy_real.py`、
`deploy_policy.py` 和 `FreeKick.py` **无需任何修改**。

---

## 参数调整

### Lidar 方案（`onboard/perception/lidar/ball_detector.py`）

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `reflect_thr` | 150 | 反射率阈值，球面反射率较高，可从 60 逐步调高 |
| `min_points` | 3 | 候选点数下限，太少则不可靠 |
| `max_range` | 1.8 m | ROI 最大距离 |
| `min_range` | 0.2 m | ROI 最小距离（过滤自身遮挡） |
| `x_low / x_high` | 0~5 m | 仅检测机器人正前方区域 |
| `z_low / z_high` | ±1.5 m | 高度范围 |
| `alpha` | 0.6 | EMA 平滑系数，越大跟踪越灵敏，越小越平滑 |

> Camera 灰度方案的参数详见 `onboard/perception/camera/README.md`。
