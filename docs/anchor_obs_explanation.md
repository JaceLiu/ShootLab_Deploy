# motion_anchor_pos_b / motion_anchor_ori_b 计算详解

> 对应代码：`policy/robonaldo/FreeKick.py` → `_build_obs()` 中的 anchor 部分

---

## 背景：anchor 是什么，为什么需要它

FreeKick 策略的目标是让机器人**模仿参考动作的同时把球踢向目标**。Policy 的输入中需要告诉它：
**"你的 torso（上半身根节点）当前相对于参考动作，差在哪里、差多少"**。

这个"差"就是 anchor 观测：
- **anchor_pos_b**（3维）：参考 torso 的位置，在机器人 torso body 坐标系里的坐标
- **anchor_ori_b**（6维）：参考 torso 的朝向，相对于机器人当前 torso 朝向的差值，用 6D rotation 表示

选 **torso_link** 而非 pelvis 作锚点，是因为训练配置中明确设定了
`anchor_body_name = "torso_link"`。Pelvis 是浮动基座，腰部三关节（waist_yaw/roll/pitch）
弯曲时 pelvis 不动，但上半身已经大幅偏移；torso 在腰部以上，更能代表上半身整体的空间状态。

> **注意**：anchor obs（anchor_pos_b / anchor_ori_b）基于 **torso**；
> 而球和目标点的 obs（soccer_pos_b / target_pos_b）基于 **pelvis（root）**，
> 这与训练代码 `soccer_robot_relative_pos_b` / `target_robot_relative_pos_b`
> 使用 `root_state_w` 保持一致。两者不同，不要混淆。

---

## 坐标系约定

```
Motion World Frame（动作世界系）
    NPZ 文件里 body_pos_w / body_quat_w 所在的坐标系。
    这是采集/训练参考动作时的世界坐标系，原点和 yaw 方向由采集时的场景决定。

Robot World Frame（机器人世界系）
    MuJoCo 仿真当前的世界坐标系。
    d.xpos[torso_id]、d.xquat[torso_id]、d.qpos[0:3]（pelvis）都在这个系里。

Torso Body Frame（torso body 系）
    以机器人当前 torso_link 为原点，随 torso 旋转的局部坐标系。
    anchor_pos_b / anchor_ori_b 在这个系里表达。

Pelvis Body Frame（pelvis body 系）
    以机器人当前 pelvis（浮动基座）为原点，随 pelvis 旋转的局部坐标系。
    soccer_pos_b / target_pos_b 在这个系里表达。
```

两个世界系的**地面（XY 平面）相同**，区别只在水平旋转方向（yaw）：
参考动作录制时机器人可能面朝东，部署时机器人面朝北，需要对齐。

---

## 第一步：`enter()` 什么时候被调用

`enter()` 在玩家切换到 FreeKick 状态时，**只执行一次**：

```
玩家按下 R1+Down
    → FSMCommand.SKILL_8
    → FSM 检测到状态切换
    → old_policy.exit()
    → FreeKick.enter()   ← 在这里，只执行一次
    → 之后每帧 FreeKick.run()
```

`enter()` 里做的事：
- 读取 IMU 当前四元数，计算 `_init_to_world`（yaw 对齐矩阵，之后固定不变）
- 记录参考动作 t=0 的 anchor 世界坐标 `_ref_anchor_world_origin`（之后固定不变）
- 记录当前 `torso_pos_w` 作为 `_entry_torso_pos_w`（之后固定不变）

---

## 第二步：`_init_to_world`——入场时的 yaw 对齐矩阵

在 `enter()` 中只计算一次：

```python
motion_t0_quat = self.motion_body_quat[0, NPZ_ANCHOR_IDX]   # 动作第0帧，torso 的朝向
torso_quat_w   = self.state_cmd.torso_quat_w                 # 机器人入场时 torso 的朝向

yaw_motion_mat = _quat_to_matrix(_yaw_quat(motion_t0_quat))  # 只取 yaw 分量 → 3×3
yaw_robot_mat  = _quat_to_matrix(_yaw_quat(torso_quat_w))    # 只取 yaw 分量 → 3×3

self._init_to_world = yaw_robot_mat @ yaw_motion_mat.T
```

### 直觉

`yaw_motion_mat` 把 +X 轴转到"动作录制时机器人的朝向"。
`.T`（转置 = 逆）把它撤销，让动作面朝 +X。
再乘 `yaw_robot_mat`，把 +X 转到"机器人入场时的朝向"。

最终 `_init_to_world` 是一个纯 yaw 旋转矩阵，把**动作世界系**中的任何向量转到**机器人世界系**。

### 具体数值例子

假设：
- 动作录制时机器人面朝**正东（+X_motion = 正东）**，即 yaw_motion = 0°
- 机器人入场时面朝**正北（+X_robot = 正北）**，即 yaw_robot = 90°

则：

```
yaw_motion_mat = I（单位矩阵，因为 yaw=0）

yaw_robot_mat  = [[cos90, -sin90, 0],   =  [[ 0, -1, 0],
                   [sin90,  cos90, 0],       [ 1,  0, 0],
                   [    0,      0, 1]]        [ 0,  0, 1]]

_init_to_world = yaw_robot_mat @ I.T = yaw_robot_mat
```

这个矩阵的作用：把动作里"向正东走 1 米"（向量 [1,0,0]_motion）
变成机器人世界系里的"向正北走 1 米"（向量 [0,1,0]_robot）——
因为动作的"前方"对应了机器人入场时的"前方"。

---

## 三、`motion_anchor_pos_b` 的计算

```python
# anchor obs 使用 torso 作为参考 body（训练配置：anchor_body_name = "torso_link"）
torso_quat_w = self.state_cmd.torso_quat_w.astype(np.float64)
R_torso_w    = _quat_to_matrix(torso_quat_w)
torso_pos_w  = self.state_cmd.torso_pos_w.astype(np.float64)

init_world_quat      = _matrix_to_quat(self._init_to_world)
ref_anchor_pos_w     = self.motion_body_pos[t, NPZ_ANCHOR_IDX]   # ← 来自 NPZ，不是传感器
aligned_anchor_pos_w = self._init_to_world @ ref_anchor_pos_w

anchor_disp_w = aligned_anchor_pos_w - self._ref_anchor_world_origin  # 参考从起点走了多远
robot_disp_w  = torso_pos_w - self._entry_torso_pos_w                 # 机器人从起点走了多远
anchor_pos_b  = R_torso_w.T @ (anchor_disp_w - robot_disp_w)
```

### `aligned_anchor_pos_w` 来自哪里

`aligned_anchor_pos_w` **完全来自 NPZ 离线文件**，和真机传感器无关：

```
self.motion_body_pos[t, NPZ_ANCHOR_IDX]   ← NPZ 里第 t 帧 torso_link 的世界坐标
self._init_to_world @ (...)               ← 旋转对齐到机器人初始朝向（enter() 时算一次，之后固定）
```

每帧 `run()` 只是查 NPZ 表格里第 t 行的数据，再乘一个固定矩阵。

### 为什么用相对位移而非绝对坐标

**原始想法**（有问题）：
```
anchor_pos_b = R_torso_w.T @ (aligned_anchor_pos_w - torso_pos_w)
               ↑参考的绝对世界坐标            ↑机器人的绝对世界坐标
```

真机没有绝对位置，`torso_pos_w` 永远是 `[0,0,0]`，但 `aligned_anchor_pos_w` 在参考动作中
可能是 `[1.5, 0, 0.8]`，直接相减结果完全错误。

**当前方案**（相对位移）：

```
参考走了多远 = aligned_anchor_pos_w[t] - aligned_anchor_pos_w[0]
机器人走了多远 = torso_pos_w[t] - torso_pos_w[entry]
anchor_pos_b = R_torso_w.T @ (参考走了多远 - 机器人走了多远)
```

| 场景 | 参考位移 | 机器人位移 | 结果 |
|------|---------|-----------|------|
| 仿真 | 来自 NPZ | 来自 MuJoCo | 精确差值 |
| 真机 | 来自 NPZ | 0（无里程计）| ≈ 参考位移（近似，机器人走了多少没减掉）|

真机上近似的含义：**"按参考动作，我的 torso 从出发点应该走到哪里"**，用当前 torso 朝向表达。

### 三步拆解

```
① self._init_to_world @ ref_anchor_pos_w
   动作世界系坐标 → 机器人世界系坐标（yaw 对齐）

② - torso_pos_w
   以机器人当前 torso 为原点，得到世界系下的相对坐标

③ R_torso_w.T @ (...)
   把世界坐标轴方向的相对坐标，旋转到 torso body 坐标轴方向
```

### 具体数值例子

场景设定（延续上面的例子）：
- 动作世界系与机器人世界系的 yaw 差 90°，`_init_to_world` 如上
- 参考动作第 t 帧，torso 在动作世界系的位置：`ref_anchor_pos_w = [2.0, 0.5, 0.9]`
- 机器人当前 torso 在机器人世界系的位置：`torso_pos_w = [1.0, 0.0, 0.85]`
- 机器人当前朝向正北，torso 没有 pitch/roll 倾斜：

```
R_torso_w = [[0, -1, 0],   （torso "前方"+X_body 对应世界系的正北 [0,1,0]）
             [1,  0, 0],
             [0,  0, 1]]
```

**步骤①：yaw 对齐**

```
aligned_anchor_pos_w = _init_to_world @ [2.0, 0.5, 0.9]
                     = [[0,-1,0],[1,0,0],[0,0,1]] @ [2.0, 0.5, 0.9]
                     = [-0.5, 2.0, 0.9]
```

动作里"向东 2m、向北 0.5m"的位置，变成了机器人世界系里"向北 2m、向西 0.5m"。

**步骤②：以机器人 torso 为原点**

```
delta_w = aligned_anchor_pos_w - torso_pos_w
        = [-0.5, 2.0, 0.9] - [1.0, 0.0, 0.85]
        = [-1.5, 2.0, 0.05]
```

参考 torso 在机器人世界系中：向西 1.5m、向北 2m、高 5cm。

**步骤③：转到 torso body 坐标系**

```
anchor_pos_b = R_torso_w.T @ [-1.5, 2.0, 0.05]
             = [[0,1,0],[-1,0,0],[0,0,1]] @ [-1.5, 2.0, 0.05]
             = [2.0, 1.5, 0.05]
```

在 torso body 系里：**正前方 2m、左侧 1.5m、上方 5cm**。

Policy 拿到 `[2.0, 1.5, 0.05]` 后，能直接理解：
"参考 torso 在我正前方 2 米偏左的位置，比我高一点点"，不需要知道自己朝哪个世界方向。

### 为什么第③步不能省略？

如果只做步骤①②，`delta_w = [-1.5, 2.0, 0.05]` 的含义随机器人朝向而变。
当机器人转身面向正东后，同样的"参考 torso 在机器人正前方"，`delta_w` 的数值就会变成 `[2.0, 1.5, 0.05]`。
Policy 看到的输入数值不同，但物理含义相同，训练和推理会混乱。
乘以 `R_torso_w.T` 后，无论机器人朝哪，"正前方 2m"永远输出 `[2.0, 0, 0]`，Policy 感知稳定。

---

## 三、`motion_anchor_ori_b` 的计算

```python
ref_anchor_quat_w = self.motion_body_quat[t, NPZ_ANCHOR_IDX]
aligned_quat      = _quat_mul(init_world_quat, ref_anchor_quat_w)
rel_quat          = _quat_mul(_quat_conj(torso_quat_w), aligned_quat)
rel_quat          = rel_quat / np.linalg.norm(rel_quat)
anchor_ori_6d     = _rot6d_from_quat(rel_quat)
```

朝向的计算结构和位置完全对称，只是旋转的"加减法"用四元数乘法表达。

### 四元数乘法的含义

四元数乘法 `q1 ⊗ q2` 等价于：**先做 q2 的旋转，再做 q1 的旋转**（右结合）。
`_quat_conj(q)` 是 q 的逆，作用是"撤销 q 的旋转"。

### 三步拆解（对应位置的三步）

```
① _quat_mul(init_world_quat, ref_anchor_quat_w)
   等价于矩阵：R_init_to_world @ R_ref_anchor
   将参考 torso 朝向从动作世界系转到机器人世界系（yaw 对齐，与位置步骤①对称）

② _quat_mul(_quat_conj(torso_quat_w), aligned_quat)
   等价于矩阵：R_torso_w.T @ R_aligned
   "撤销 torso 当前朝向"后叠加参考朝向，得到相对旋转差（与位置步骤②③合并的对称）

③ 归一化 + 转 6D rotation
   归一化消除浮点误差；6D rotation 是训练中常用的朝向表示
```

### 具体数值例子

场景设定（同上，延续）：
- 动作录制时机器人朝正东，入场时机器人朝正北，yaw 差 90°
- 参考动作第 t 帧，torso 在动作世界系的朝向：**在朝正东基础上，向左（逆时针）倾斜了 30° yaw**

用四元数表示（绕 Z 轴转 30°）：
```
ref_anchor_quat_w ≈ [cos15°, 0, 0, sin15°] ≈ [0.966, 0, 0, 0.259]
```

机器人当前朝正北（绕 Z 轴转 90°）：
```
torso_quat_w ≈ [cos45°, 0, 0, sin45°] ≈ [0.707, 0, 0, 0.707]
```

`init_world_quat` 对应 yaw=90° 的旋转：
```
init_world_quat ≈ [0.707, 0, 0, 0.707]
```

**步骤①：yaw 对齐**

```
aligned_quat = init_world_quat ⊗ ref_anchor_quat_w
             ≈ [0.707,0,0,0.707] ⊗ [0.966,0,0,0.259]
```

四元数乘法（绕 Z 轴，yaw 直接相加）：yaw_aligned = 90° + 30° = 120°
```
aligned_quat ≈ [cos60°, 0, 0, sin60°] ≈ [0.5, 0, 0, 0.866]
```

参考 torso 在机器人世界系中，面朝 120°（从正北顺时针 120°，即偏向西北）。

**步骤②：计算相对旋转差**

```
rel_quat = conj(torso_quat_w) ⊗ aligned_quat
         = conj([0.707,0,0,0.707]) ⊗ [0.5,0,0,0.866]
         = [0.707,0,0,-0.707] ⊗ [0.5,0,0,0.866]
```

Z 轴四元数相乘（yaw 相减）：120° - 90° = 30°
```
rel_quat ≈ [cos15°, 0, 0, sin15°] ≈ [0.966, 0, 0, 0.259]
```

这说明：**参考 torso 比机器人当前朝向多转了 30°（逆时针）**。
Policy 拿到这个值后，知道自己的 torso 需要再向左转 30° 才能和参考对上。

**步骤③：转 6D rotation**

旋转矩阵（绕 Z 转 30°）：
```
R_rel = [[cos30, -sin30, 0],   =  [[ 0.866, -0.5,  0],
          [sin30,  cos30, 0],       [ 0.5,   0.866, 0],
          [    0,      0, 1]]        [ 0,     0,     1]]
```

取前两列展平：
```
anchor_ori_6d = [0.866, 0.5,  -0.5, 0.866,  0, 0]
                 ↑第1列(3维)        ↑第2列(3维)
```

Policy 接收这 6 个数，通过它们恢复出完整旋转矩阵，理解"torso 还需要转多少"。

### 为什么用 6D rotation 而不是四元数或欧拉角？

| 表示方法 | 维度 | 问题 |
|----------|------|------|
| 欧拉角（RPY） | 3 | 万向锁奇异点；±180° 处梯度跳变，训练不稳定 |
| 四元数 | 4 | q 和 -q 表示同一旋转（antipodal ambiguity）；网络难以区分，损失函数有歧义 |
| 6D rotation | 6 | 无奇异点；唯一映射；梯度处处连续；丢弃第三列不损失信息（可由前两列叉积还原） |

---

## 四、位置与朝向的完整对称性

| 步骤 | 位置 | 朝向 |
|------|------|------|
| ① yaw 对齐 | `R_init @ pos_motion` | `Q_init ⊗ Q_motion` |
| ② 以 torso 为参考 | `aligned_pos - torso_pos_w` | `Q_torso⁻¹ ⊗ Q_aligned` |
| ③ 转到 torso body 系 | `R_torso_w.T @ delta_pos` | （已在步骤②中完成） |
| 输出形式 | (3,) float32 | (6,) 6D rotation |

位置的步骤②和③是分开的两个操作（减法 + 旋转）；
朝向的步骤②把"减法"和"旋转到 body 系"合成了一个四元数乘法，因为旋转空间里"减法"本身就是"用逆旋转"，结果自然已经在 body 系里了。

---

## 五、完整数据流

```
NPZ（动作世界系）
  body_pos_w [t, 9]   ──────────────────────────────────────────────┐
  body_quat_w[t, 9]   ──────────────────────────────────────────────┤
                                                                     │
                              enter() 时计算一次                      │
                        ┌─────────────────────────┐                  │
                        │  _init_to_world（3×3）   │◄─────────────────┘
                        │  = yaw_robot @ yaw_motion.T               │
                        └──────────┬──────────────┘
                                   │ yaw 对齐
                                   ▼
                    机器人世界系中的参考 torso 位姿
                        aligned_pos / aligned_quat
                                   │
              ┌────────────────────┴────────────────────┐
              │ 位置                                     │ 朝向
              │ - torso_pos_w                            │ ⊗ conj(torso_quat_w)
              │ R_torso_w.T @ delta                      │
              ▼                                          ▼
        anchor_pos_b (3,)                    anchor_ori_6d (6,)
              │                                          │
              └──────────────────┬───────────────────────┘
                                 ▼
                            obs[58:67]  →  Policy 输入
                   （参考 torso 在机器人 torso body 系里的位置和朝向差值）


MuJoCo 实时（机器人世界系）
  d.qpos[0:3]  →  pelvis_pos_w  ──┐
  d.qpos[3:7]  →  pelvis_quat_w ──┤  ball/target obs 基于 pelvis
  d.xpos[ball] →  ball_pos_w  ────┤  （与训练 root_state_w 一致）
  freekick.yaml   →  target_pos_w ───┘
                                   │
              ┌────────────────────┴────────────────────┐
              │ 球                                       │ 目标点
              │ ball_pos_w - pelvis_pos_w                │ target_pos_w - pelvis_pos_w
              │ R_pelvis.T @ delta, clip ±8.0            │ R_pelvis.T @ delta, clip ±8.0
              ▼                                          ▼
        soccer_pos_b (3,/帧)               target_pos_b (3,/帧)
              │                                          │
              └──────── 5帧历史 ─────────────────────────┘
                                 ▼
                       obs[517:532] / obs[532:547]  →  Policy 输入
```

---

## 六、仿真 vs 真机：每个量的来源对比

| obs 量 | 仿真（deploy_mujoco） | 真机（deploy_real） | 差异说明 |
|--------|----------------------|---------------------|---------|
| `torso_quat_w` | `d.xquat[torso_id]`（MuJoCo 直接读） | IMU 四元数 + 腰关节 FK（`transform_pelvis_to_torso_complete`） | 真机经过腰关节变换，但物理含义相同 |
| `torso_pos_w` | `d.xpos[torso_id]`（MuJoCo 直接读） | 永远是 `[0,0,0]`（无里程计） | **关键差异**，影响 anchor_pos_b |
| `pelvis_quat_w` | `d.qpos[3:7]`（MuJoCo free joint） | IMU 原始四元数 `[w,x,y,z]` | 相同物理含义，来源不同 |
| `pelvis_pos_w` | `d.qpos[0:3]`（MuJoCo free joint） | 永远是 `[0,0,0]`（无里程计） | 真机 ball/target 通过其他方式绕过 |
| `ball_pos` | MuJoCo 世界坐标，再转到 pelvis body 系 | DDS 直接给出 pelvis body 系坐标 | 来源不同，结果含义相同 |
| `target_pos_b` | 每帧实时计算（用 pelvis 世界坐标） | `enter()` 时算一次，之后固定 | 真机无绝对坐标，只能在入场时估算一次 |

### anchor_pos_b 的计算差异

这是仿真和真机差异最大的量。代码用 `use_body_frame_ball` flag 分两条路径。

---

#### 训练时的公式（Isaac Lab）

训练代码（`/home/ydlu/workspace/Score`）计算：

```python
motion_anchor_pos_b = quat_apply_inverse(robot_torso_quat_w,
                          ref_anchor_pos_w - robot_torso_pos_w)
```

等价于：

```
anchor_pos_b_train = R_torso_w.T @ (ref_anchor_pos_w[t] - robot_torso_pos_w[t])
```

含义：**参考 torso 在哪里，机器人 torso 在哪里，两者之差，用 torso body 系表达。**

关键前提：训练时每次 episode reset，Isaac Lab 把机器人 torso 放在与参考动作 t=0 对齐的位置，
因此 `anchor_pos_b[t=0] ≈ 0`，随后随运动偏差增大/减小。

---

#### 仿真部署（`use_body_frame_ball=false`）

```python
anchor_pos_b = R_torso_w.T @ (aligned_anchor_pos_w - torso_pos_w)
#                               ↑ NPZ + yaw对齐           ↑ MuJoCo d.xpos[torso_id]
```

`torso_pos_w` 来自 MuJoCo，实时精确。**与训练公式完全一致。**

---

#### 真机部署（`use_body_frame_ball=true`）

真机没有里程计，`torso_pos_w` 永远是 `[0, 0, 0]`。
直接套训练公式会得到：

```
anchor_pos_b = R_torso_w.T @ (aligned_anchor_pos_w - [0,0,0])
             = R_torso_w.T @ aligned_anchor_pos_w
```

`aligned_anchor_pos_w` 是参考动作在世界坐标系里的**绝对位置**，比如 `[1.5, 0, 0.85]`。
Policy 会认为"参考 torso 在你东边 1.5m"，而实际上参考和你几乎在同一个地方——**信号完全错误**。

**解决方案：改用相对位移**

```python
# enter() 时记录两个基准（只算一次，之后固定）
self._ref_anchor_world_origin = self._init_to_world @ motion_body_pos[0, NPZ_ANCHOR_IDX]
self._entry_torso_pos_w       = state_cmd.torso_pos_w.copy()   # 真机上 = [0,0,0]

# 每帧 _build_obs() 里
anchor_disp_w = aligned_anchor_pos_w    - self._ref_anchor_world_origin  # 参考从t=0走的位移
robot_disp_w  = torso_pos_w             - self._entry_torso_pos_w        # 机器人从enter()走的位移
anchor_pos_b  = R_torso_w.T @ (anchor_disp_w - robot_disp_w)
```

真机代入（`torso_pos_w = _entry_torso_pos_w = [0,0,0]`）：

```
anchor_disp_w = aligned_anchor_pos_w[t] - aligned_anchor_pos_w[0]
robot_disp_w  = 0
anchor_pos_b  = R_torso_w.T @ (aligned_anchor_pos_w[t] - aligned_anchor_pos_w[0])
```

含义：**参考 torso 从起点走了多远，用当前 torso 朝向表达。**

从 t=0 时 `anchor_pos_b = 0` 开始，随参考动作推进逐渐变化，不再有初始的巨大偏置。

---

#### 两公式的数学关系

设：
- `A[t] = aligned_anchor_pos_w[t]`（参考 torso 世界坐标，经 yaw 对齐）
- `P[t] = torso_pos_w[t]`（机器人 torso 世界坐标）
- `A0 = _ref_anchor_world_origin = A[0]`（参考起点）
- `P0 = _entry_torso_pos_w = P[0]`（机器人入场时 torso 位置）

展开真机公式：

```
真机公式 = R_torso_w.T @ ((A[t] - A0) - (P[t] - P0))
         = R_torso_w.T @ (A[t] - P[t]) - R_torso_w.T @ (A0 - P0)
         =    训练公式    -         常数偏移
```

**常数偏移** = `R_torso_w.T @ (A0 - P0)` = 参考起点与机器人入场位置之差，转到 torso body 系。

注意：真机上 `P0 = [0,0,0]`，`A0 ≈ [0, 0, 0.85]`（NPZ 里 torso 站立高度），**两者不相等**。
但相对位移公式并不要求它们相等，见下方分析。

| 场景 | 偏移 `A0 - P0` | 对公式的影响 |
|------|--------------|------------|
| 训练（Isaac Lab reset 时让 `A0 ≈ P0`） | ≈ 0 | 两公式等价 |
| 仿真部署（`torso_pos_w` 来自 MuJoCo，精确） | 直接用训练公式，不存在此偏移问题 | — |
| 真机（使用训练公式）| `A0 ≠ P0`，偏移 ≈ `[0,0,0.85]` | 第一帧 anchor_pos_b ≈ `[0, 0, 0.85]`，巨大错误 |
| 真机（使用相对位移公式） | `A0 ≠ P0`，但 A0 和 P0 各自消掉自己 | t=0 时强制为 0，误差只来自里程计缺失 |

**为什么相对位移公式不需要 `A0 ≈ P0`？**

代入 t=0：
```
anchor_pos_b[0] = R.T @ ((A[0] - A[0]) - (P[0] - P[0])) = R.T @ 0 = 0
```

A[0] 和 P[0] 各自与自身相减，无论它们是否相等，结果都是 0。
公式只关心"**各自从自己的起点走了多远**"，不要求两个起点在同一位置。

---

#### 真机近似的剩余误差

真机上 `robot_disp_w = 0`，机器人实际走的位移没有被减掉。
理想的完整公式应该是：

```
anchor_pos_b = R_torso_w.T @ (anchor_disp_w - actual_robot_disp_w)
```

`actual_robot_disp_w` 需要支撑腿里程计（stance-leg odometry）估计。
当前近似等价于假设机器人始终停在原点，误差随机器人行走距离增大。
对于踢球这类持续时间 ~2s、前进距离 ~0.3m 的短动作，误差在可接受范围内。

### anchor_ori_b 的计算差异

**没有差异**。朝向只需要 `torso_quat_w`，仿真和真机都能实时拿到（MuJoCo vs IMU+腰关节FK），不依赖绝对位置。

### ball_pos 的计算差异

```python
# freekick.yaml: use_body_frame_ball 控制分支

# 仿真（false）：
ball_pos_b = R_pelvis.T @ (ball_pos_w - pelvis_pos_w)   # MuJoCo 世界坐标 → pelvis body 系

# 真机（true）：
ball_pos_b = state_cmd.ball_pos_b                        # DDS 直接给出 pelvis body 系，跳过变换
```

两者的结果在物理上等价，只是来源不同。

---

## 七、常见疑问

**Q：`_init_to_world` 是在 enter() 时计算的，之后 run() 里每帧都用同一个矩阵，会不会有误差？**

A：不会。这个矩阵只做 yaw 对齐，它把"动作录制时的朝向"对齐到"机器人入场时的朝向"。
这是一个固定的坐标变换，和之后机器人如何移动无关。
每帧 run() 中，机器人当前的朝向通过 `torso_quat_w = state_cmd.torso_quat_w` 实时读取，
所以步骤②的"减去机器人当前量"始终使用的是最新状态，不存在误差积累。

**Q：anchor_pos_b 减的是 torso_pos_w，而 soccer_pos_b 减的是 pelvis_pos_w，为什么不统一？**

A：因为训练时两者用的参考点不同：
- anchor obs 的训练代码（`motion_anchor_pos_b`）使用 `robot_anchor_pos_w`，即 torso_link 的世界坐标。
- ball/target obs 的训练代码（`soccer_robot_relative_pos_b` / `target_robot_relative_pos_b`）使用
  `root_state_w[:, :3]`，即 pelvis（浮动基座）的世界坐标。

部署侧必须与训练侧完全一致，否则 obs 的物理含义对不上，policy 输出会混乱。

**Q：`R_torso_w` 用的是 torso 的旋转矩阵，如果机器人向前倾（pitch），torso 也会倾斜，anchor_pos_b 会随之变化吗？**

A：会。这正是期望的行为：anchor_pos_b 表达的是"在机器人当前姿态的 torso body 系里，参考 torso 差在哪"。
当机器人前倾时，torso body 系的 +X 也向下倾斜，anchor_pos_b 会反映出"参考 torso 在当前 body 系里偏后上方"，
促使 policy 向前倾更多或调整姿态，这和训练时的观测定义是一致的。
