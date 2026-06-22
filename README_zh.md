<div align="center">
  <h1 align="center">RoboNaldo Deploy</h1>
  <p align="center">
    <a href="README.md">🌎 English</a> | <span>🇨🇳 中文</span>
  </p>
</div>

<p align="center">
  🎮🚪 <strong>RoboNaldo Deploy 是一个基于状态切换机制的机器人多策略部署框架，目前包含的策略适用于宇树G1机器人(29dof)</strong> 🚪🎮
</p>

> 🤖 **本仓库是 [RoboNaldo](https://opendrivelab.com/RoboNaldo/) 的部署仓库。**
> 策略的**训练**代码在配套仓库
> [**opendrivelab/RoboNaldo**](https://github.com/opendrivelab/RoboNaldo)；本仓库负责把训练好的策略
> 部署到真机与仿真的宇树 G1 上。

<table align="center" width="100%">
  <tr>
    <td width="50%" align="center"><b>仿真 Simulation</b></td>
    <td width="50%" align="center"><b>真机 Real robot</b></td>
  </tr>
  <tr>
    <td width="50%"><video src="https://github.com/user-attachments/assets/f1365c83-3764-43b7-9ecf-57ba89854cf1" controls muted width="100%"></video></td>
    <td width="50%"><video src="https://github.com/user-attachments/assets/45fbf473-060e-49b1-9204-c43bbcc89a3c" controls muted width="100%"></video></td>
  </tr>
</table>

<p align="center"><em>任意球 demo —— RoboNaldo 策略部署在宇树 G1 上</em></p>

## 写在前面

- **本部署框架仅适用于具有三自由度腰部的G1机器人，如果装有腰部固定件的话需要按照官网教程解锁，然后才能正常使用该部署框架。**

- **建议拆下手掌，舞蹈动作会存在干涉**

- **实际机器人部署中出现的问题，十有八九是策略适应性不足所致，大家不必过度怀疑硬件层面的缺陷。**

- **[视频教程](https://www.youtube.com/watch?v=BuHNzqebIqc&feature=youtu.be)**

## 安装配置

## 1. 创建虚拟环境

建议在虚拟环境中运行训练或部署程序，推荐使用 Conda 创建虚拟环境。

### 1.1 创建新环境

使用以下命令创建虚拟环境：

```bash
conda create -n robomimic python=3.8
```

### 1.2 激活虚拟环境

```bash
conda activate robomimic
```

---

## 2. 安装依赖

### 2.1 安装 PyTorch

PyTorch 是一个神经网络计算框架，用于模型训练和推理。使用以下命令安装：

```bash
conda install pytorch==2.3.1 torchvision==0.18.1 torchaudio==2.3.1 pytorch-cuda=12.1 -c pytorch -c nvidia
```

### 2.2 安装 RoboNaldo_Deploy

#### 2.2.1 下载

通过 Git 克隆仓库：

```bash
git clone https://github.com/OpenDriveLab/RoboNaldo_Deploy.git
```

#### 2.2.2 安装组件

进入目录并安装：

```bash
cd RoboNaldo_Deploy
pip install -r requirements.txt
```
#### 2.2.3 安装unitree_sdk2_python

```bash
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
cd unitree_sdk2_python
pip install -e .
```

## 文档地图

真机机载部署请先看 `onboard/docs/README.md`。它说明运行时架构、topic 归属，
并给出新人阅读路径。

- 环境配置：`onboard/docs/INSTALL_AGENT_GUIDE.md`
- 真机启动拓扑：`tools/start_tmux_layout.sh`
- 相机模块手册：`onboard/perception/camera/README.md`
- LiDAR 模块手册：`onboard/perception/lidar/README.md`
- 感知架构：`onboard/docs/PERCEPTION_ARCHITECTURE.md`

---
## 运行代码

## 1. 运行Mujoco仿真代码

标准场景：
```bash
python deploy_mujoco/deploy_mujoco.py
```

FreeKick 场景（含球，运行 FreeKick 策略时使用）：
```bash
python deploy_mujoco/deploy_mujoco.py --config-name mujoco_freekick
```

---
## 2. Policy 说明

| 模式名称            | 触发按键                       | 描述                                                                 |
|--------------------|-------------------------------|----------------------------------------------------------------------|
| **PassiveMode**    | F1（真机）/ 松开 L2（仿真）    | 阻尼保护模式                                                         |
| **FixedPose**      | Start                         | 位控恢复至默认关节值                                                 |
| **LocoMode**       | B                             | 用于稳定行走的控制模式                                               |
| **AMP**            | A                             | AMP 运动策略（默认进入 run 模式）                                    |
| **FreeKick**          | R1                            | 踢球得分策略（547维obs，5帧历史，需配合机载雷达感知）               |
| **BeyondMimicMJ**  | D-pad DOWN                    | 站立躺下模仿策略（MuJoCo 训练版，含参考动作跟踪）                   |
| **StandUpMJ**      | D-pad UP                      | 躺下站起模仿策略（MuJoCo 训练版，含参考动作跟踪）                       |
| **Pinocchio1.6MJ** | R2                            | `g1_result_pinocchio_1_6_mj.yaml` 对应的 MuJoCo 模仿策略            |
| **BeyondMimic**    | —                             | 仍保留在仓库中，但当前默认不分配手柄按键                             |

### 手动偏置调整

仿真和真机均支持在不重启的情况下通过手柄实时调整感知 Y 轴偏置：

| 操作 | 按键 | 范围 | 备注 |
|------|------|------|------|
| 目标 Y +5 cm（左移） | 按住 **L1** + 点按 **D-pad Left** | ±1.5 m | 仿真 + 真机 |
| 目标 Y −5 cm（右移） | 按住 **L1** + 点按 **D-pad Right** | ±1.5 m | 仿真 + 真机 |
| 球 Y +5 cm（左移） | 按住 **L2** + 点按 **D-pad Left** | ±1.5 m | 仅真机 |
| 球 Y −5 cm（右移） | 按住 **L2** + 点按 **D-pad Right** | ±1.5 m | 仅真机 |

偏置以 pelvis body 坐标系为基准（+Y = 机器人左侧）。每次调整时终端会打印当前值，**Sensor Dashboard** 侧边栏橙色"Current Bias"卡片也会实时显示。

---
## 3. 仿真操作说明

1. 连接Xbox手柄

2. 运行仿真程序：
```bash
python deploy_mujoco/deploy_mujoco.py
```
3. Start键进入位控模式

4. 按 `B` 进入 LocoMode，并按下 `BACKSPACE` 在仿真中使机器人站立，之后即可通过摇杆控制机器人行走

5. 仿真中保留的单键切换如下：
   - `A` -> AMP（run 模式）
   - `B` -> LocoMode
   - `D-pad DOWN` -> BeyondMimicMJ
   - `D-pad UP` -> StandUpMJ
   - `R2` -> Pinocchio1.6MJ
   - `R1` -> FreeKick

6. `FreeKick` 需使用 `--config-name mujoco_freekick` 加载含球的仿真场景；`BeyondMimicMJ / StandUpMJ / Pinocchio1.6MJ` 之间支持直接互切

7. 任意时刻可**松开 L2** 进入阻尼保护模式（PassiveMode），按 `Start` 返回 FixedPose

8. `BeyondMimic` 仍保留在仓库中，但默认不再绑定手柄按键；`Dance / Kick / KungFu / ASAP / HOST` 等旧策略已经从仓库中删除

---
## 4. 真机操作说明

1. 开机后将机器人吊起来。先按 **L2+B** 进入阻尼保护模式（头部紫灯变橙），再按 **L2+R2** 进入调试模式（头部橙灯变黄）。

2. **推荐方式（机载 Orin）**：使用 tmux 启动脚本，一键启动 C++ bridge、policy 推理、感知服务和 Sensor Dashboard：
   ```bash
   bash tools/start_tmux_layout.sh
   ```
   在 `deploy_policy` 窗格中按 **Start** 进入位控模式。

   也可分窗口手动启动：
   ```bash
   BRIDGE_NETWORK_INTERFACE=eth0 bridge/build/cpp_bridge_main   # 终端 1
   python bridge/python/deploy_policy.py                          # 终端 2
   ```

   过渡验证版（无需 C++ bridge）：
   ```bash
   python bridge/python/deploy_bridge_py.py   # 终端 1
   python bridge/python/deploy_policy.py       # 终端 2
   ```

   **简单模式（直连 以太网，无 Orin）**：`python deploy_real/deploy_real.py`

3. 后续单键切换与仿真保持一致：`A / B / D-pad DOWN / D-pad UP / R2 / R1`。
   任意时刻可按 **F1** 进入阻尼保护模式（PassiveMode）。

4. **FreeKick 踢球策略（R1）真机额外步骤**：FreeKick 策略依赖机载球感知，需在启动前先运行感知服务；LiDAR / camera raw 观测由 `onboard/perception/ball_fuser.py` 融合，并发布最终 DDS topic `rt/ball_state`。未启动感知服务时请勿激活 FreeKick 策略。

5. **Sensor Dashboard**：启动 `tools/start_tmux_layout.sh` 后，在浏览器中打开 `http://<机器人IP>:8091/` 即可查看所有传感器位置（目标、各球感知源、偏置后坐标）及橙色"Current Bias"卡片中的当前偏置值。手动启动：
   ```bash
   python onboard/perception/debug/sensor_dashboard.py
   ```

   完整 bridge 配置说明见 `bridge/README_zh.md` 和 `bridge/VALIDATION.md`。

---
## 注意事项
### 1. 框架兼容性说明
当前框架暂不支持在搭载Orin NX平台的G1机器人上直接部署。初步分析可能是由于`unitree_python_sdk`在Orin平台上的兼容性问题。针对机载Orin平台的部署需求，建议采用以下替代方案：

- 使用[unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2)替代原Python SDK
- 基于ROS构建双节点架构：
  - **C++节点**：负责机器人与遥控器之间的数据收发
  - **Python节点**：专用于策略推理
- 本仓库已提供一个最小桥接原型，集中在 `bridge/` 目录下，可作为上述拆分方案的参考实现。

### 2. Mimic策略可靠性警告
Mimic策略不保证100%成功率，特别是在湿滑/沙地等复杂地面上。若出现机器人失控情况：
- 按下`F1`键激活**阻尼保护模式**(PassiveMode)
- 按下`Select`键立即终止控制程序

### 3. 当前保留策略的真机提示
当前部署入口已简化为 `LocoMode / AMP / FreeKick / BeyondMimicMJ / StandUpMJ / Pinocchio1.6MJ`。

⚠️ **重要注意事项**：
- **建议拆除手掌**：部分策略的原始训练未考虑手掌碰撞（作者的G1初始无手掌）
- **启动与切换阶段**：策略切换前后建议优先回到 **FixedPose** 或 **PassiveMode**
- **人工保护**：首次在真机验证某个保留策略时，建议全程吊挂并保留人工保护

### 4. 其他动作建议
仓库中现在只保留当前部署会用到的策略实现，其余历史策略已删除，以减少误用和维护成本。

### 5. 强烈建议
**务必**先在仿真环境中熟练操作，再尝试真机部署。

## 致谢

本部署框架基于 [**ccrpRepo/RoboMimic_Deploy**](https://github.com/ccrpRepo/RoboMimic_Deploy)
构建。感谢原作者开源了这套基于状态切换的多策略部署框架，本仓库在其基础上扩展而来。

本仓库是 [**RoboNaldo**](https://opendrivelab.com/RoboNaldo/) 的部署仓库 ——
训练代码见 [opendrivelab/RoboNaldo](https://github.com/opendrivelab/RoboNaldo)。



