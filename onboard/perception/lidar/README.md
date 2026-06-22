# Lidar Ball Detector — 使用手册

Livox MID360 点云 → 球心检测 → raw DDS 发布 + RViz2 可视化。

LiDAR 模块发布 raw topic `rt/lidar_ball_state`。最终策略读取的
`rt/ball_state` 由 `onboard/perception/ball_fuser.py` 发布。

全系统启动与环境配置见：

- 总览：`onboard/docs/README.md`
- 安装：`onboard/docs/INSTALL_AGENT_GUIDE.md`
- 真机 tmux 拓扑：`tools/start_tmux_layout.sh`

---

## 文件说明

```
onboard/perception/lidar/
├── run.sh                    ← 推荐入口：启动 Livox driver + ball_detector.py
├── ball_detector.py          ← 主服务：订阅 /livox/lidar，发布 rt/lidar_ball_state
├── rviz_publisher.py         ← RViz2 可视化辅助：发布点云 + Marker
├── center_kalman_filter.py   ← 卡尔曼滤波器（平滑球心轨迹）
├── mid360_to_base.py         ← 坐标变换：MID360 系 → pelvis body 系
└── README.md                 ← 本文件
```

---

## 快速启动

### 推荐：使用仓库启动脚本

```bash
bash onboard/perception/lidar/run.sh --show --base-y-bias 0.00 --dds-topic rt/lidar_ball_state
```

`run.sh` 会自动处理：

- ROS2 Humble/Foxy setup
- `$HOME/ws_livox/install/setup.bash`
- `$HOME/unitree_ros2/cyclonedds_ws/install/setup.bash`
- `$HOME/Livox-SDK2/build/sdk_core`
- Livox MID360 driver
- `ball_detector.py --msg-type pc2`

它只发布 raw LiDAR 球：`rt/lidar_ball_state`。要给策略使用，必须再启动唯一 fuser：

```bash
bash onboard/perception/run_ball_fuser.sh
```

全系统真机运行推荐直接使用 `tools/start_tmux_layout.sh`，其中右中 pane 是 LiDAR raw
检测，右下 pane 是唯一 fuser。

启动成功后终端会持续打印状态信息，Sensor Dashboard 可查看 raw/fused 球位置。

```
[INFO] BallDetector ready.
[INFO] ball (raw):  (0.832, -0.011, -0.031)
[INFO] ball (kf):   (0.829, -0.010, -0.030)
[INFO] ball (pelvis): (0.821, -0.009, -0.028)  cand=12  cost=2.3ms
```

### 3. 启动 RViz2（本地电脑）

RViz2 运行在**本地电脑**，通过网线订阅机器人发布的 ROS2 Topic。

#### 网络前提检查

本地电脑通过 `enp5s0f1`（192.168.123.99）连接到机器人（192.168.123.164），
确认已连通：

```bash
ping 192.168.123.164
```

#### 问题：本地多网卡，DDS 可能走错接口

本地机器有多张网卡（`enp5s0f1` 有线 + `wlx...` WiFi），
CycloneDDS 默认会选第一个活跃接口，很可能选到 WiFi，导致**找不到任何 Topic**。
必须显式告知 CycloneDDS 使用连接机器人的有线网卡。

#### 首次使用：安装 CycloneDDS RMW

`rmw_cyclonedds_cpp` 默认未安装，需手动安装一次：

```bash
# 按本地实际版本替换 jazzy（Humble 用 humble，以此类推）
sudo apt install ros-jazzy-rmw-cyclonedds-cpp
```

#### 解决方案：设置 CYCLONEDDS_URI

在本地电脑每次启动 RViz2 的终端里执行：

```bash
# 按本地实际版本 / Shell 替换（zsh → setup.zsh，bash → setup.bash）
source /opt/ros/jazzy/setup.zsh
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export CYCLONEDDS_URI='<CycloneDDS>
  <Domain>
    <General>
      <NetworkInterfaceAddress>enp5s0f1</NetworkInterfaceAddress>
    </General>
  </Domain>
</CycloneDDS>'
rviz2
```

> `enp5s0f1` 是本地电脑连接机器人的有线网卡名（IP 192.168.123.99），
> 根据实际 `ifconfig` 输出确认网卡名。

#### 验证 Topic 可见性

在 RViz2 启动前，先用以下命令确认能看到机器人发布的 Topic：

```bash
# 同一终端（已 source + 设置环境变量）
ros2 topic list
# 应该能看到 /ball_detector/cloud_all 等 Topic

ros2 topic hz /ball_detector/cloud_all
# 应该显示约 10 Hz
```

如果 `ros2 topic list` 为空或报 RMW 错误，检查：
1. `rmw_cyclonedds_cpp` 是否已安装（`sudo apt install ros-jazzy-rmw-cyclonedds-cpp`）
2. 机器人上球检测服务是否已启动
3. `ROS_DOMAIN_ID` 两端是否一致（默认都是 0）
4. `CYCLONEDDS_URI` 中的网卡名是否正确

#### 可选：写入 ~/.zshrc 避免每次手动 export

```bash
cat >> ~/.zshrc << 'EOF'

# ROS2 + CycloneDDS — 连接 Unitree G1 (192.168.123.164)
alias ros_robot='
  source /opt/ros/jazzy/setup.zsh &&
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp &&
  export ROS_DOMAIN_ID=0 &&
  export CYCLONEDDS_URI='"'"'<CycloneDDS><Domain><General><NetworkInterfaceAddress>enp5s0f1</NetworkInterfaceAddress></General></Domain></CycloneDDS>'"'"' &&
  echo "[OK] ROS2 ready, interface=enp5s0f1, domain=0"
'
EOF
```

之后每次开新终端只需输入 `ros_robot` 即可，然后再运行 `rviz2` 或 `ros2 topic list`。

---

## RViz2 配置

### Fixed Frame

打开 RViz2 后，左侧 **Global Options → Fixed Frame** 设置为：

```
livox_frame
```

### 添加 Display

点击左下角 **Add**，依次添加以下 5 个 Display：

| 类型 | Topic | 推荐设置 | 含义 |
|------|-------|---------|------|
| `PointCloud2` | `/ball_detector/cloud_all` | Size: 0.01m，Color: Flat / 灰白 | MID360 原始全量点云 |
| `PointCloud2` | `/ball_detector/cloud_candidates` | Size: 0.03m，Color: Flat / 黄色 | 通过反射率+距离+ROI 过滤的候选点 |
| `Marker` | `/ball_detector/ball_raw` | — | **红色半透明球**：`estimate_ball_center_ls()` 原始估计 |
| `Marker` | `/ball_detector/ball_kf` | — | **绿色实心球**：卡尔曼滤波后的球心 |
| `Marker` | `/ball_detector/text_info` | — | 球心上方的白色文字调试信息 |

> 提示：可以将配置保存为 `.rviz` 文件（File → Save Config As），下次直接 `rviz2 -d your_config.rviz` 加载。

### 可视化效果说明

```
原始点云（灰白小点）
    +
候选点（黄色大点） ← 反射率 ≥ 150 且在 ROI 范围内

红球（半透明）← LS 质心估计，偏移 center_offset
绿球（实心）  ← 卡尔曼滤波平滑后，用于发布到控制器

文字标注（白色，球心上方 25cm）：
  n=12  off=0.050m  cost=2.3ms
  │       │               └─ 每帧检测耗时
  │       └─ 当前 center_offset 值
  └─ 本帧候选点数量
```

---

## 运行时参数调整（键盘）

服务运行时，**在同一终端**按下以下键可实时调整 `center_offset`（球心偏移量）：

| 按键 | 效果 |
|------|------|
| `+` 或 `=` | center_offset += 0.005m |
| `-` 或 `_` | center_offset -= 0.005m（最小 0） |
| `0` | 重置为 0.050m |

调整后 RViz2 中的红球位置会立即变化，用于标定偏移量是否正确。

> 注意：需要 stdin 为 TTY（直接在终端运行），通过 SSH 的 `-T` 参数或后台运行时键盘控制自动禁用。

---

## 检测参数参考

在 `ball_detector.py` 的 `__init__` 中调整：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `reflect_thr` | `150` | 反射率阈值。球面反射率高，背景低；若候选点太少可适当降低 |
| `min_points` | `4` | 候选点数下限，低于此值视为未检测到球 |
| `min_range` | `0.2 m` | ROI 最小距离（过滤自身遮挡） |
| `max_range` | `1.8 m` | ROI 最大距离 |
| `x_low / x_high` | `0 ~ 5 m` | 仅检测机器人正前方（MID360 X 轴方向） |
| `z_low / z_high` | `±1.5 m` | MID360 坐标系下的高度范围 |
| `center_offset` | `0.05 m` | 质心偏移修正量（可键盘实时调整，见上方） |
| `r` (ball radius) | `0.115 m` | 球半径，影响 LS 拟合与球体 Marker 大小 |

**调参建议流程：**

1. 先在 RViz2 观察 `/ball_detector/cloud_all`，确认原始点云正常
2. 观察 `/ball_detector/cloud_candidates`（黄色），若候选点很少或始终没有 → 降低 `reflect_thr`
3. 候选点位置正确后，观察红球和绿球是否与球的实际位置一致
4. 用 `+`/`-` 键微调 `center_offset` 直到绿球中心对准真实球心
5. 确认 `rt/lidar_ball_state` 与 dashboard 显示一致，再通过 fuser 验证最终 `rt/ball_state`

---

## 发布的 ROS2 Topic 汇总

| Topic | 消息类型 | 说明 |
|-------|---------|------|
| `/ball_detector/cloud_all` | `sensor_msgs/PointCloud2` | 全量原始点云（每帧） |
| `/ball_detector/cloud_candidates` | `sensor_msgs/PointCloud2` | 过滤后候选点云 |
| `/ball_detector/ball_raw` | `visualization_msgs/Marker` | 红色球体：LS 原始估计，lifetime=1s |
| `/ball_detector/ball_kf` | `visualization_msgs/Marker` | 绿色球体：KF 平滑结果，lifetime=1s |
| `/ball_detector/text_info` | `visualization_msgs/Marker` | 白色文字：`n= off= cost=`，lifetime=1s |

DDS Topic：

| Topic | 类型 | 说明 |
|-------|------|------|
| `rt/lidar_ball_state` | BallState（自定义 DDS） | LiDAR raw 球心在 pelvis body 系坐标，`valid` 标志位 |
| `rt/ball_state` | BallState（自定义 DDS） | fuser 最终输出，policy 读取；不是 LiDAR detector 直接发布 |

调试 Topic（网页可视化订阅）：

| Topic | 类型 | 说明 |
|-------|------|------|
| `rt/lidar_ball_debug` | LidarBallDebugState（自定义 DDS） | LiDAR raw / KF 球心在 MID360 系的坐标，以及 FK 后 pelvis 坐标 |

如果怀疑 MID360 安装或外参导致 pelvis 系 `y` 有固定偏差，可以先用临时 bias 验证：

```bash
# 例：实际 y=0，但网页/控制看到 y=-0.02，则先加 +0.02m 补偿
bash onboard/perception/lidar/run.sh --base-y-bias 0.02
```

这个 bias 只加在 FK 后、发布到 `rt/lidar_ball_state` 的 pelvis-frame `y` 上；网页中的 `lidar raw MID360` 仍显示 FK 前原始 LiDAR 坐标，方便判断偏差来自检测本身还是安装/FK。
