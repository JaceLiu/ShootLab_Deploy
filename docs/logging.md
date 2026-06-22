# FreeKick Log 录制与回放

## 概述

在执行 `SKILL_FREEKICK` 策略期间，可以将机器人状态、球位置和动作输出实时写入二进制 log 文件。每帧立即 flush，即使程序崩溃也不会丢失已录制的数据。

录制完成后，可在 MuJoCo 中可视化回放：

- **绿色半透明机器人**：还原每帧的关节姿态
- **红色球**：机器人传感器感知的球位置（`ball_pos_b` 变换到世界系）
- **蓝色球**：MuJoCo 仿真 ground truth（仅仿真录制时有效）
- **品红色目标球**：`FreeKick` 实际使用的目标（`debug_target_pos_b` 变换到世界系）
- **紫色小球**：原始 `target_pos_b` 传感器/检测值

主要用途：在真实机器人上录制，然后在 MuJoCo 里回放，直观验证球位置感知是否准确。

---

## 开启录制

在对应部署配置中将 `logging.enabled` 改为 `true`：

**MuJoCo 仿真** — `deploy_mujoco/config/mujoco.yaml`
```yaml
logging:
  enabled: true
  log_dir: "logs"
  tag:     "freekick"
```

**真实机器人** — `deploy_real/config/real.yaml`
```yaml
logging:
  enabled: true
  log_dir: "logs"
  tag:     "freekick"
```

运行程序后，进入 `SKILL_FREEKICK` 状态时自动开始录制，退出时自动关闭并写入元信息。

---

## 文件格式

每次录制生成两个文件：

```
logs/
  20260318_153012_freekick.bin    二进制帧数据
  20260318_153012_freekick.json   元信息（字段定义、帧数、控制频率等）
```

每帧为 114 个 float32（456 bytes），字段如下：

| 字段 | 维度 | 说明 |
|---|---|---|
| `step` | 1 | 帧序号 |
| `time_s` | 1 | 时间戳（秒） |
| `q` | 29 | 关节位置 |
| `dq` | 29 | 关节速度 |
| `pelvis_pos_w` | 3 | 浮动基座世界坐标（real 上为零） |
| `pelvis_quat_w` | 4 | 浮动基座姿态 `[w,x,y,z]` |
| `ball_pos_b` | 3 | 球相对 pelvis body frame（传感器值） |
| `ball_pos_w` | 3 | 球世界坐标（仿真 ground truth，real 上为零） |
| `ball_valid` | 1 | 球感知是否有效（0 / 1） |
| `target_pos_b` | 3 | 原始 target 相对 pelvis body frame（来自 `rt/target_state`） |
| `target_valid` | 1 | target 感知是否有效（0 / 1） |
| `vel_cmd` | 3 | 速度命令 |
| `debug_target_pos_b` | 3 | `FreeKick` 实际使用的 target，相对 pelvis body frame |
| `debug_target_source` | 1 | 目标来源编码：0=`none`, 1=`fixed`, 2=`fixed_fallback`, 3=`apriltag`, 4=`imu_hold`, 5=`fixed_sim` |
| `actions` | 29 | policy 输出动作 |

---

## 回放

```bash
# 基本用法
python tools/playback_log.py logs/20260318_153012_freekick.bin

# 指定 xml 和初始速度
python tools/playback_log.py logs/xxx.bin --xml g1_description/scene.xml --speed 0.5
```

**键盘控制：**

| 键 | 功能 |
|---|---|
| `空格` | 暂停 / 继续 |
| `←` / `→` | ±1 帧（暂停时） |
| `[` / `]` | ±10 帧 |
| `f` / `s` | 速度 ×2 / ÷2（范围 0.125× ~ 16×） |
| `r` | 回到第 0 帧 |
| `q` | 退出 |

---

## 在代码中读取 log

```python
from common.logger import Logger

data = Logger.load("logs/20260318_153012_freekick.bin")

data["q"]          # (T, 29) 关节位置
data["ball_pos_b"] # (T, 3)  球 body frame 位置
data["target_pos_b"] # (T, 3) 原始 target body frame 位置
data["debug_target_pos_b"] # (T, 3) FreeKick 实际使用的 target
data["vel_cmd"]    # (T, 3) 速度命令
data["ball_valid"] # (T,)    感知有效性
data["_meta"]      # dict    元信息
```
