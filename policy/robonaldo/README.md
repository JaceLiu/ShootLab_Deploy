# FreeKick 策略配置说明

`FreeKick` 从 `config/freekick.yaml` 读取参数，加载 ONNX 与参考动作 NPZ，构造 **547 维**观测并输出关节目标。本文说明各配置项的含义与典型用法。

---

## 文件与路径

| 项 | 说明 |
|----|------|
| `onnx_path` | 相对 `policy/robonaldo/model/` 的 ONNX 文件名。 |
| `motion_path` | 相对 `policy/robonaldo/model/` 的参考动作 NPZ（需含 `joint_pos` / `joint_vel` / `body_quat_w` / `body_pos_w` 等）。 |

---

## 控制周期与热身

| 键 | 默认 | 说明 |
|----|------|------|
| `control_dt` | 必填 | 控制周期（秒），与部署循环一致；用于把 `motion_start_s` / `motion_end_s` 换成帧索引。 |
| `warmup_steps` | `10` | 进入策略后，先用若干步在「当前关节姿态」与「参考动作第 0 帧姿态」之间插值，再开始跑网络。 |
| `clip_actions` | `3.0` | 网络输出（Isaac 关节顺序）在送入反归一化前的 fallback 限幅，防止发散。 |
| `action_clip` | 可选 | 逐关节 raw action 限幅，键为 Isaac 关节名或 `.*_joint_suffix` 模式；配置后覆盖对应关节的 `clip_actions` fallback。 |

---

## 仿真 vs 真机（球与目标）

| 键 | 说明 |
|----|------|
| `use_body_frame_ball` | **`false`**：仿真。球在 MuJoCo 世界系，由 `deploy_mujoco` 写入 `ball_pos_w`，策略内部变换到骨盆体坐标。<br>**`true`**：真机。球由 DDS `ball_pos_b`（骨盆体系，米）提供，无世界系里程时与训练里「体坐标球」一致。 |
| `target_source` | `fixed` 或 `apriltag`。`fixed` 使用配置里的 `target_pos`；`apriltag` 在真机上读取 `rt/target_state`。 |
| `target_pos` | 固定目标位置 `[x,y,z]`（米）。仿真下解释为世界系目标点；真机下解释为 `enter()` 时刻记录的 pelvis/body-frame 目标偏移，并随航向更新。 |
| `target_hold_on_loss_with_imu` | 当 `target_source=apriltag` 且 tag 丢失时，保留最近一次可见目标在 yaw/world 缓存中的方向，再根据当前 IMU yaw 换回 pelvis 体系，继续瞄准。 |
| `target_use_fixed_fallback` | 当 `target_source=apriltag` 但还没有看到过有效 tag，或关闭 IMU 保持时，是否退回到固定 `target_pos`。 |

---

## 参考动作时间窗与冻结

| 键 | 说明 |
|----|------|
| `motion_start_s` | 可选。从该时间（秒）起的子序列作为参考，缺省为 `0`。 |
| `motion_end_s` | 可选。子序列结束时间（秒）；不填则用 NPZ 全长。 |
| `freeze_motion_at_first_frame` | **`true`** 时，整段 episode 的参考关节指令始终用子序列的**第 0 帧**（不随时间推进动作相位）。用于调试「只盯球、不走动作相位」。**`false`** 时随 `time_step` 在子序列内前进。 |

---

## 锚点位置 `anchor_pos_b`（躯干体系，观测中 3 维）

三者互斥优先级：**`zero_anchor_pos` > `ball_as_anchor_pos` > 位移/绝对公式**（见 `FreeKick._build_obs`）。

| 键 | 说明 |
|----|------|
| `zero_anchor_pos` | **`true`**：`anchor_pos_b` 恒为 `[0,0,0]`，表示锚点与躯干原点重合。开启时 **`ball_as_anchor_pos` 不生效**。 |
| `ball_as_anchor_pos` | **`true`**：锚点由「参考躯干锚点」与「球在躯干体系下的位置」混合（约 `0.1` 参考 + `0.9` 球，球分量会 clip）。真机下球来自 `ball_pos_b`（经 `ball_b_effective`，含丢球默认点，见下）。**`false`**：真机用相对位移公式、仿真用世界系绝对公式（与训练一致）。 |

---

## 锚点朝向 `anchor_ori_6d`（6 维）

| 键 | 说明 |
|----|------|
| `ball_facing_anchor_ori` | **`true`**：锚点朝向使躯干 +X 大致指向**球**（由球世界位置与躯干位置算方向，Z 与参考锚高度对齐）。**`false`**：使用参考动作里锚点连杆的朝向相对躯干。 |

---

## 丢球时的默认球位置（真机）

仅当 **`use_body_frame_ball: true`** 且配置了 `ball_obs_default_when_lost` 时生效。

| 键 | 说明 |
|----|------|
| `ball_obs_default_when_lost` | 骨盆体系下 `[x,y,z]`（米）。当 **`ball_valid` 为假** 且传感器 **`‖ball_pos_b‖ ≤ ball_obs_lost_norm_max`** 时，用该向量替代原始接近零的测量，用于：<br>• `soccer_pos_b` 历史<br>• `ball_as_anchor_pos` 混合<br>• `ball_facing_anchor_ori` 指向的球位<br>**不写此项**则不替换，行为与未加默认时一致。 |
| `ball_obs_lost_norm_max` | 默认 `1e-3`。判定「传感器球接近零」的范数上界（米）。 |

**说明**：短时 **coast**（无效但 `ball_pos_b` 仍为非零滞后估计）范数通常大于该阈值，**不会**被替换成默认点。

---

## PD 与动作缩放（与 ONNX 训练一致）

以下数组均为 **MuJoCo / 硬件关节顺序**（与 `deploy_*` 一致），长度 29。

| 键 | 说明 |
|----|------|
| `default_joint_pos` | 网络输出为相对该中立位的偏移（经 `action_scale` 缩放后加到中立位得到目标角）。 |
| `action_scale` | 逐关节动作缩放。 |
| `kps` / `kds` | 位置环 PD，写入 `policy_output` 供下游发扭矩。 |
| `tau_limit` | 当前 `FreeKick` 仅从 YAML 加载，需与训练元数据一致；若后续在代码里做力矩限幅可与此对齐。 |

---

## 观测维度（便于对照调试）

固定 **547 维**：`command(58)` + `anchor_pos_b(3)` + `anchor_ori_b(6)` + `base_ang_vel(15)` + `joint_pos(145)` + `joint_vel(145)` + `actions(145)` + `soccer_pos_b(15)` + `target_pos_b(15)`，其中 `15 = 5 × 3` 为历史长度 `HISTORY_LEN=5`。

---

## 修改配置后

更换 `onnx_path` / `motion_path` 或改关节维相关数组时，请确认与 **ONNX 输入维数、训练时归一化与动作定义** 一致，否则需重新导出模型或对齐 NPZ。
