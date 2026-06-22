# Camera 感知模块 — 使用手册

灰度相机负责发布 raw topic：

- AprilTag 目标：`rt/target_state`
- 相机亮球：`rt/cam_ball_state`

最终策略读取的 `rt/ball_state` 由 `onboard/perception/ball_fuser.py` 发布。
全系统启动与环境配置见：

- 总览：`onboard/docs/README.md`
- 安装：`onboard/docs/INSTALL_AGENT_GUIDE.md`
- 真机 tmux 拓扑：`tools/start_tmux_layout.sh`

---

## 目录

1. [文件结构](#一文件结构)
2. [快速启动](#二快速启动)
   - [AprilTag 目标检测（当前主用）](#21-apriltag-目标检测当前主用)
3. [浏览器预览 --show](#三浏览器预览---show)
4. [检测流程](#四检测流程)
5. [坐标变换与外参](#五坐标变换与外参)
6. [参数速查](#六参数速查)
7. [深度精度说明](#七深度精度说明)
8. [常见问题](#八常见问题)

---

## 一、文件结构

```
onboard/perception/camera/
├── run_gray.sh                ← 入口：灰度 AprilTag + 亮球（+可选 fuser）
├── _launch.sh                 ← 内部：环境 + 默认参数，启动 detector
├── target_ball_detector.py    ← 干活：AprilTag → rt/target_state；
│                                      --ball-bright 灰度亮球 → rt/cam_ball_state
├── camera_to_base.py          ← 坐标变换：相机系 → pelvis 系（含胸部外参）
├── timing.py                  ← StageTimer 阶段耗时分析（--profile-timing）
├── debug/
│   ├── generate_apriltag_template.py ← 生成可打印 A4 AprilTag 模板
│   ├── apriltag_tag36h11_id0_120mm_a4.pdf ← 已生成的 120mm tag0 模板
│   ├── target_state_echo.py      ← 打印 rt/target_state 最新值
│   └── target_extrinsics_eval.py ← 统计目标位姿均值/方差，辅助外参标定
├── README.md                  ← 本文档
└── TROUBLESHOOTING.md         ← 排障与现场调试记录
```

相关感知模块（`onboard/perception/` 层）：

| 文件 | 说明 |
|------|------|
| `lidar/ball_detector.py` | 雷达球检测 → `rt/lidar_ball_state`（近距离球） |
| `lidar/run.sh` | 雷达启动脚本（tmux 使用） |
| `ball_fuser.py` | 融合相机+雷达球 → `rt/ball_state`（policy 读这个） |
| `run_ball_fuser.sh` | 融合器启动脚本，full-system/tmux 中应单独启动且保持单例 |
| `run_sensor_dashboard.sh` | 启动传感器全览仪表盘（port 8091） |
| `debug/sensor_dashboard.py` | 订阅 DDS topic，浏览器显示 target + 球位置 |

---

## 二、快速启动

### 2.1 AprilTag 目标检测（当前主用）

```bash
bash onboard/perception/camera/run_gray.sh --show
```

默认使用 4-tag 板（tag 0/1/2/3），自动融合估计公共目标点，发布到 `rt/target_state`。
同时因为 `run_gray.sh` 默认启用 `--ball-bright`，也会发布相机 raw ball 到
`rt/cam_ball_state`。

如果只做相机单独调试，可以用：

```bash
bash onboard/perception/camera/run_gray.sh --with-fuser --show
```

但在 `tools/start_tmux_layout.sh` 或手动已经启动
`bash onboard/perception/run_ball_fuser.sh` 时，不要使用 `--with-fuser`，避免多个
fuser 同时发布最终 `rt/ball_state`。

常用参数：

```bash
# 只检测单个 tag
bash onboard/perception/camera/run_gray.sh --tag-id 0 --tag-size 0.12 --show

# 自定义多 tag 偏移（见脚本注释）
bash onboard/perception/camera/run_gray.sh \
    --tag-id 5 --tag-id 8 --tag-size 0.10 \
    --tag-offset 5 0.20 0.00 0.00 \
    --tag-offset 8 -0.20 0.00 0.00
```

**AprilTag 参数说明：**
- `--tag-size`：黑色方形 tag 本体边长（米），不是整张 A4 纸大小
- `--tag-offset TAG_ID DX DY DZ`：从该 tag 中心出发的偏移（tag 自身坐标系，+X右/+Y上/+Z外）
- 检测成功时 `class_id` 字段复用为 `tag_id`

**生成打印模板：**

```bash
conda run -n robomimic --no-capture-output \
    python onboard/perception/camera/debug/generate_apriltag_template.py \
    --family tag36h11 --tag-id 0 --tag-size-mm 120
# 生成 PDF，100% 原始尺寸打印，用尺子验证黑框边长
```

**外参标定辅助：**

```bash
# 将 tag 放在机器人正前方，运行后查看均值/方差
python onboard/perception/camera/debug/target_extrinsics_eval.py
# 或订阅原始 topic
python onboard/perception/camera/debug/target_state_echo.py
```

---

## 三、浏览器预览 `--show`

启动时加 `--show` 参数，程序在 **port 8080** 开启 HTTP MJPEG 服务：

```
http://<robot-ip>:8080/
```

**画面说明：**
- 绿色框：本帧检测成功，框内标注置信度
- 橙色框：漏检 COAST 状态，框内标注剩余帧数
- 左下角黄字：目标在 pelvis 系的坐标
- 左上角白字：相机名称 + 实时 FPS

---

## 四、检测流程

```
V4L2 GREY /dev/video3 @30Hz（单进程独占相机）
         │
         ├─▶ [主循环]   AprilTag 检测（cv2.aruco CPU，~5ms）→ rt/target_state
         └─▶ [worker进程] 亮球检测（latest-frame，--ball-bright）→ rt/cam_ball_state
```

---

## 五、坐标变换与外参

所有变换实现在 `camera_to_base.py`。

### 5.1 头部相机（已标定）

运动学链（来自 URDF `g1_sysid_23dof.urdf`）：

```
pelvis
  └─ waist_yaw_joint    (Rz, q_wy)   t=[0, 0, 0]
      └─ waist_roll_joint  (Rx, q_wr)   t=[-0.0039635, 0, 0.044]
          └─ waist_pitch_joint (Ry, q_wp)  t=[0, 0, 0]
              └─ head_joint   (Ry, q_head=0.593412)  t=[0.0039635, 0, 0.3159]
                  └─ head_camera_joint (fixed)
                         xyz=[0.0448353662, 0.01, 0.1219029938]
                         rpy=[0.0119142, 0.8377475, 0.0053045]
```

运行时使用实时关节角 `q_wy / q_wr / q_wp`（从 `/lowstate` 读取），`q_head` 固定为 `0.593412 rad`。

### 5.2 胸部相机（待标定）

```
pelvis
  └─ waist_yaw_joint → waist_roll_joint → waist_pitch_joint
      └─ chest_camera_joint (fixed)  ← ⚠️ 占位值，需要实测/标定
             xyz=[0.10, 0.00, 0.12]  (X前 / Y左 / Z上，单位 m)
             rpy=[0.00, 0.30, 0.00]  (roll / pitch / yaw，单位 rad)
```

修改外参：打开 `camera_to_base.py`，修改：

```python
_CHEST_XYZ = [0.10, 0.00, 0.12]   # TODO: 实测后替换（单位 m）
_CHEST_RPY = (0.00, 0.30, 0.00)   # TODO: 标定后替换（rad）
```

可以通过命令行覆盖：`--chest-xyz 0.13 0.0 0.06 --chest-rpy 0.0 0.28 0.0`

---

## 六、参数速查

### 代码常量（修改源文件后重启生效）

| 常量 | 位置 | 默认值 | 含义 |
|------|------|--------|------|
| `DEPTH_SAMPLE_RADIUS` | 检测器 | 5 px | 深度采样半径 |
| `DEPTH_MIN / MAX` | 检测器 | 0.1 / 10.0 m | 有效深度范围 |
| `BALL_RADIUS` | 检测器 | 0.115 m | 球半径（前表面→球心补偿） |
| `EMA_ALPHA` | 检测器 | 0.6 | 位置平滑系数 |
| `EMA_GATE` | 检测器 | 0.6 m | 跳变重置门限 |
| `COAST_FRAMES` | 检测器 | 10 帧 | 漏检时保持位置的最大帧数 |

### 命令行参数（`target_ball_detector.py`）

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `--tag-id` | 0/1/2/3 | 要识别的 tag ID（可多次指定） |
| `--tag-size` | 0.12 m | tag 黑色方形边长 |
| `--tag-offset` | 见脚本 | 各 tag 到公共目标点的偏移 |
| `--show` | 关闭 | 开启 MJPEG 预览（port 8080） |
| `--ball-bright` | 关闭 | 同时启用灰度/IR 亮球检测 |
| `--chest-xyz` | 见代码 | 覆盖胸部相机位置外参（m） |
| `--chest-rpy` | 见代码 | 覆盖胸部相机姿态外参（rad） |

---

## 七、深度精度说明

D455 的 Color 和 Depth 传感器不共光心（基线约 -14.5 mm，FOV 也不同），直接将 Color 像素坐标用于查找 `depth_arr` 会引入最大约 70 px / 18 cm 的横向误差。

代码中采用两步视差修正：

```
Step 1 — 内参映射
  ndcx = (cx - ppx_c) / fx_c
  dx0  = ndcx × fx_d + ppx_d

Step 2 — 基线视差修正
  dx   = ndcx × fx_d + ppx_d + tx/Z × fx_d   (tx = -14.5mm)
```

该视差修正仅在 `--ball-bright-use-depth` 启用深度校验时生效；灰度链路默认用单目球半径估距，不需要深度流。

---

## 八、常见问题

### Q1：`RuntimeError: Couldn't resolve requests`（RealSense pipeline.start 失败）

常见原因：帧率组合不合法。当前代码优先 **30/30 Hz**，失败则依次尝试降级。若仍失败，检查 USB 3.0 接口、线缆质量、`rs-enumerate-devices`。

有效组合（D455，librealsense 2.54）：

| Color | Depth | 帧率 |
|-------|-------|------|
| 1280×720 | 848×480 | 30 fps ✅ |
| 1280×720 | 424×240 | 15 fps（降帧）⚠️ |

---

### Q2：深度值为 0 或球心 z 坐标异常

1. 检查深度流是否正常（`--show` 画面观察球周围深度孔洞）
2. 适当增大 `DEPTH_SAMPLE_RADIUS`
3. 检查 USB 是否接 3.0 口（2.0 口带宽不足，Depth 流可能掉帧）

---

### Q3：性能与现场调试参考

- `TROUBLESHOOTING.md` 第 0 章：现场速查手册（AprilTag + IR 白球）
- `TROUBLESHOOTING.md` 第十章：诊断命令速查（GPU 频率 / 流水线验证）
- `TROUBLESHOOTING.md` 第 15 章：`--ball-bright` 亮球检测排障
- 灰度链路性能规则见 `onboard/docs/PERCEPTION_ARCHITECTURE.md` 的
  "Grayscale Camera Performance Rules"
