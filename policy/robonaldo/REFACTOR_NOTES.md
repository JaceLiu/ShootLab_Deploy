# FreeKick Refactor Notes

## 背景

这次重构的目标不是改功能，而是把 `policy/robonaldo/FreeKick.py` 中由多个布尔配置隐式组合出来的行为整理清楚，降低阅读和维护成本。

重构前的主要问题：

- `freekick.yaml` 中有多个语义重叠的布尔开关，例如：
  - `zero_anchor_pos`
  - `ball_as_anchor_pos`
  - `ball_facing_anchor_ori`
  - `freeze_motion_at_first_frame`
  - `wait_for_ball`
- `_build_obs()` 同时处理：
  - ball 坐标来源
  - anchor 位置
  - anchor 朝向
  - target 观测
  - motion 帧索引
- `run()` 中 motion trigger / burst / reset 逻辑与时间索引逻辑耦合较重。
- 一些配置项有历史遗留问题：
  - `adapt_play_motion` 实际未接线
  - `tau_limit` 在 `FreeKick.py` 中读取，但并未真正参与 `FreeKick` 的逻辑

## 这次重构做了什么

### 1. 引入内部模式

在 `FreeKick.__init__()` 中，仍然兼容旧 YAML 字段，但会先解析出几个内部模式：

- `runtime_mode`
  - `real`
  - `sim`
- `anchor_mode`
  - `zero`
  - `ball_cmd`
  - `motion_ref`
- `anchor_ori_mode`
  - `ball_facing`
  - `motion_ref`
- `motion_mode`
  - `freeze`
  - `play`
  - `triggered`

这些模式只是代码内部使用，不要求立刻改 YAML 结构。

启动时会打印类似：

```text
[FreeKick config] runtime=sim anchor=ball_cmd anchor_ori=ball_facing motion=freeze
```

方便确认当前配置组合实际落到了哪条逻辑分支。

### 2. 拆分 `_build_obs()`

原来 `_build_obs()` 承担了太多职责。现在拆成了这些辅助函数：

- `_get_effective_ball_pos_b()`
  - 统一处理 real 模式下 `ball_pos_b`
  - 支持 `ball_obs_default_when_lost`
- `_compute_anchor_pos_b(...)`
  - 统一处理三种 anchor 位置来源：
    - `zero`
    - `ball_cmd`
    - `motion_ref`
- `_compute_anchor_ori_6d(...)`
  - 统一处理两种 anchor 朝向来源：
    - `ball_facing`
    - `motion_ref`
- `_compute_ball_target_obs_b(...)`
  - 统一处理 ball / target 的 pelvis-frame 观测构造

这样 `_build_obs()` 现在更像是“拼装观测”，而不是所有逻辑都堆在一起。

### 3. 拆分 trigger / motion gate 逻辑

把 `run()` 中和 trigger 相关的逻辑拆成了：

- `_get_trigger_ball_state_b()`
  - 统一返回 trigger 使用的 body-frame 球状态：
    - `ball_pos_b`
    - `ball_vel_b`
    - `anchor_xy = [0, 0]`
- `_update_motion_trigger_state(policy_step)`
  - 统一处理 finite burst 播放结束后回到等待状态的逻辑

这样 `run()` 中 trigger 主流程更清楚：

1. 更新 trigger 状态
2. 获取 body-frame 球状态
3. 调 `_ball_enters_circle()`
4. 根据结果决定是否触发 motion

### 4. 清理配置

这次顺带清理了几项容易误导阅读的配置：

- `adapt_play_motion`
  - 从 YAML 中移除
  - 在代码里保留 deprecated 兼容读取
  - 如果还传这个字段，会打印 ignored 提示
- `tau_limit`
  - 从 `FreeKick.py` 中移除读取
  - 从 `freekick.yaml` 中移除
  - `tau_limit` 仍由 `deploy_mujoco` 侧控制，不属于 `FreeKick` 策略本身
- `ball_vel_b_alpha`
  - 显式加入 `freekick.yaml`
  - 用于 real 模式下由 `ball_pos_b` 差分估计 `ball_vel_b` 的平滑

## 模式总览

这一版 `FreeKick` 可以先理解成 4 条互相独立的“模式轴”：

| 模式轴 | 内部模式 | 作用 |
|---|---|---|
| `runtime_mode` | `real` / `sim` | 决定 ball / target / trigger 使用 body-frame 还是由 world-frame 变换而来 |
| `anchor_mode` | `zero` / `ball_cmd` / `motion_ref` | 决定 `anchor_pos_b` 从哪里来 |
| `anchor_ori_mode` | `ball_facing` / `motion_ref` | 决定 `anchor_ori_6d` 从哪里来 |
| `motion_mode` | `freeze` / `play` / `triggered` | 决定 reference motion 时间轴怎么推进 |

可以先忽略底层的旧布尔字段，把它理解成：

```text
runtime 决定“球怎么表示”
anchor_mode 决定“锚点位置怎么给”
anchor_ori_mode 决定“锚点朝向怎么给”
motion_mode 决定“参考动作怎么播”
```

## 旧配置如何映射到内部模式

当前仍然兼容旧 YAML 字段，但初始化时会解析成下面这些内部模式：

### 1. `runtime_mode`

| 旧字段 | 内部模式 |
|---|---|
| `use_body_frame_ball: true` | `real` |
| `use_body_frame_ball: false` | `sim` |

### 2. `anchor_mode`

优先级如下：

```text
zero_anchor_pos > ball_as_anchor_pos > motion_ref
```

也就是：

| 旧字段组合 | 内部模式 | 含义 |
|---|---|---|
| `zero_anchor_pos: true` | `zero` | 直接把 `anchor_pos_b` 置零 |
| `zero_anchor_pos: false` 且 `ball_as_anchor_pos: true` | `ball_cmd` | 用球方向生成 anchor 位置 |
| 其它情况 | `motion_ref` | 用参考 motion 的 anchor |

### 3. `anchor_ori_mode`

| 旧字段 | 内部模式 | 含义 |
|---|---|---|
| `ball_facing_anchor_ori: true` | `ball_facing` | anchor 朝向球 |
| `ball_facing_anchor_ori: false` | `motion_ref` | anchor 朝向跟随参考 motion |

### 4. `motion_mode`

| 旧字段组合 | 内部模式 | 含义 |
|---|---|---|
| `freeze_motion_at_first_frame: true` | `freeze` | motion 永远停在第 0 帧 |
| `freeze_motion_at_first_frame: false` 且 `wait_for_ball: false` | `play` | motion 直接播放 |
| `freeze_motion_at_first_frame: false` 且 `wait_for_ball: true` | `triggered` | 先等球，再从关键帧播放 |

## 当前配置语义

建议按“模式轴”阅读，而不是按一个个布尔开关阅读。

### runtime 轴

- `use_body_frame_ball: false`
  - simulation 模式
  - ball 来自 `ball_pos_w`
- `use_body_frame_ball: true`
  - real 模式
  - ball 来自 `ball_pos_b`
  - trigger 也在 body-frame 下判断
  - `target_pos` 被解释成 entry 时刻记录的 body-frame 偏移

### anchor 位置轴

- `zero_anchor_pos: true`
  - anchor 位置固定为零
- `zero_anchor_pos: false` 且 `ball_as_anchor_pos: true`
  - anchor 位置由 ball 指令方向给出
- 否则
  - 使用 motion reference anchor

### anchor 朝向轴

- `ball_facing_anchor_ori: true`
  - anchor 朝向面向球
  - real 模式：使用 `ball_pos_b[:2]` 的 yaw-only 相对朝向
  - sim 模式：使用 torso 到 ball 的 world-frame 方向
- `ball_facing_anchor_ori: false`
  - anchor 朝向跟随参考 motion

### motion 时间轴

- `freeze_motion_at_first_frame: true`
  - 停在 motion 第 0 帧
- `freeze_motion_at_first_frame: false` 且 `wait_for_ball: false`
  - 正常播放 motion
- `freeze_motion_at_first_frame: false` 且 `wait_for_ball: true`
  - 等球触发后从 `trigger_frame` 或 `[trigger_frame, trigger_play_end_frame]` 播放

## 常见模式组合

下面这些是最常见、也最容易和业务语义对应的组合。

### 1. 纯参考姿态调试

适合只看策略在固定参考姿态下的表现，不希望 ball 或 motion 影响太多。

```yaml
use_body_frame_ball: false
freeze_motion_at_first_frame: true
zero_anchor_pos: true
ball_as_anchor_pos: false
ball_facing_anchor_ori: false
wait_for_ball: false
```

内部模式等价于：

```text
runtime=sim
anchor=zero
anchor_ori=motion_ref
motion=freeze
```

### 2. 直接播放参考动作

适合看 reference motion 从头到尾播放，不等球触发。

```yaml
freeze_motion_at_first_frame: false
wait_for_ball: false
zero_anchor_pos: false
ball_as_anchor_pos: false
ball_facing_anchor_ori: false
```

内部模式：

```text
anchor=motion_ref
anchor_ori=motion_ref
motion=play
```

### 3. 球感知驱动 anchor，但不靠球触发动作

适合“球决定命令方向，但 motion 还是直接播或冻结”的模式。

```yaml
ball_as_anchor_pos: true
ball_facing_anchor_ori: true
wait_for_ball: false
```

内部模式：

```text
anchor=ball_cmd
anchor_ori=ball_facing
motion=freeze 或 play
```

### 4. 球靠近后触发关键动作

这是现在 `FreeKick` 最像“踢球技能”的模式。

```yaml
ball_as_anchor_pos: true
ball_facing_anchor_ori: true
freeze_motion_at_first_frame: false
wait_for_ball: true
trigger_frame: 253
trigger_play_end_frame: 278
```

内部模式：

```text
anchor=ball_cmd
anchor_ori=ball_facing
motion=triggered
```

## 模式之间最重要的区别

如果只想快速判断当前配置在干什么，可以看下面这张表：

| 问题 | 看哪个模式轴 |
|---|---|
| 球的位置是 `ball_pos_w` 还是 `ball_pos_b`？ | `runtime_mode` |
| `anchor_pos_b` 是零、参考动作、还是球方向？ | `anchor_mode` |
| `anchor_ori_6d` 是跟球还是跟参考动作？ | `anchor_ori_mode` |
| reference motion 是冻结、直接播、还是等球触发？ | `motion_mode` |

## real 模式下的关键处理

本次重构之后，real 分支更明确地偏向 body-frame 语义。

### 1. ball 观测

real 模式下优先使用 `ball_pos_b`，如果传感器失效且球向量接近零，可以使用：

- `ball_obs_default_when_lost`
- `ball_obs_lost_norm_max`

作为兜底。

### 2. ball 速度

real 模式没有直接的 `ball_vel_b` 传感器，因此使用：

- `ball_pos_b` 的差分
- `ball_vel_b_alpha` 做平滑

对应函数：

- `_estimate_ball_vel_b()`

### 3. anchor 朝向

real 模式下 `ball_facing_anchor_ori` 已改成更明确的 **yaw-only** 语义：

- 只使用 `ball_pos_b[:2]`
- 计算相对水平朝向
- 构造绕 z 轴的相对四元数

也就是说，real 模式下这里不再依赖重建 world-frame ball position。

## 兼容性说明

本次重构尽量保持了行为兼容：

- 旧的 YAML 主要布尔字段仍然有效
- 没有强制引入新的嵌套配置结构
- 外部调用 `FreeKick` 的方式不需要改

但有两点需要注意：

1. `adapt_play_motion` 已被视为废弃字段
2. `tau_limit` 不再属于 `freekick.yaml` 的职责范围

## 阅读代码时建议的顺序

如果以后再看 `FreeKick.py`，建议按这个顺序读：

1. `__init__()`
   - 看配置如何解析成 `runtime_mode / anchor_mode / anchor_ori_mode / motion_mode`
2. `_motion_frame_index()`
   - 看 motion 时间轴怎么推进
3. `_get_effective_ball_pos_b()`
   - 看 real 模式下 ball 输入怎么被整理
4. `_compute_anchor_pos_b()`
   - 看 anchor 位置从哪里来
5. `_compute_anchor_ori_6d()`
   - 看 anchor 朝向从哪里来
6. `_compute_ball_target_obs_b()`
   - 看 ball / target 观测怎么构造
7. `run()`
   - 看 trigger 状态机和策略主循环

这样通常比直接从 `_build_obs()` 一口气往下读更容易建立整体图景。

## 后续建议

如果后续继续整理配置层，可以把当前旧布尔字段进一步升级成显式模式配置，例如：

```yaml
runtime:
  mode: sim

anchor:
  pos_mode: ball_cmd
  ori_mode: ball_facing

motion:
  mode: triggered
```

但这一步建议在当前版本稳定之后再做，因为它会影响配置文件兼容性。

## 涉及文件

- `policy/robonaldo/FreeKick.py`
- `policy/robonaldo/config/freekick.yaml`

## 本次重构总结

这次重构的核心不是“增加功能”，而是把原先散落在多个分支里的意图收拢成：

- 更清楚的内部模式
- 更短的 `_build_obs()`
- 更集中清晰的 trigger 逻辑
- 更少的死配置和误导字段

最终效果是：

- 读配置时更容易理解当前处于什么模式
- 读代码时更容易定位某个行为该去哪个辅助函数看
- 后续继续改 real / sim / trigger / anchor 逻辑时，改动边界更清楚
