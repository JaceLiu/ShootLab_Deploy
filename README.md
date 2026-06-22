<div align="center">
  <h1 align="center">RoboNaldo Deploy</h1>
  <p align="center">
    <span> 🌎English </span> | <a href="README_zh.md"> 🇨🇳中文 </a>
  </p>
</div>

<p align="center">
  <strong>​RoboNaldo Deploy​​ is a multi-policy robot deployment framework based on a state-switching mechanism. Currently, the included policies are designed for the ​​Unitree G1 robot (29-DoF)​​.</strong> 
</p>

> 🤖 **This is the deployment repository for [RoboNaldo](https://opendrivelab.com/RoboNaldo/).**
> The policy **training** code lives in the companion repository
> [**opendrivelab/RoboNaldo**](https://github.com/opendrivelab/RoboNaldo); this repository runs the
> trained policies on the real and simulated Unitree G1.

<table align="center" width="100%">
  <tr>
    <td width="50%" align="center"><b>Simulation</b></td>
    <td width="50%" align="center"><b>Real robot</b></td>
  </tr>
  <tr>
    <td width="50%"><video src="https://github.com/user-attachments/assets/f1365c83-3764-43b7-9ecf-57ba89854cf1" controls muted width="100%"></video></td>
    <td width="50%"><video src="https://github.com/user-attachments/assets/45fbf473-060e-49b1-9204-c43bbcc89a3c" controls muted width="100%"></video></td>
  </tr>
</table>

<p align="center"><em>Free-kick demo — RoboNaldo policy on a Unitree G1</em></p>

## Preface

- **​This deployment framework is only applicable to G1 robots with a 3-DOF waist. If a waist fixing bracket is installed, it must be unlocked according to the official tutorial before this framework can be used normally.​​**

- **It is recommended to remove the hands, as dance movements may cause interference.​**
  
- **When deploying real robots, if something goes wrong, it's probably the policy's fault—not your hardware. Don't waste time second-guessing your robot's physical setup.**

- **[video instruction](https://www.youtube.com/watch?v=BuHNzqebIqc&feature=youtu.be)**

## Installation and Configuration

## 1. Create a Virtual Environment

It is recommended to run training or deployment programs in a virtual environment. We suggest using Conda to create one.

### 1.1 Create a New Environment

Use the following command to create a virtual environment:
```bash
conda create -n robomimic python=3.8
```

### 1.2 Activate the Virtual Environment

```bash
conda activate robomimic
```

---

## 2. Install Dependencies

### 2.1 Install PyTorch
PyTorch is a neural network computation framework used for model training and inference. Install it with the following command:
```bash
conda install pytorch==2.3.1 torchvision==0.18.1 torchaudio==2.3.1 pytorch-cuda=12.1 -c pytorch -c nvidia
```

### 2.2 Install RoboNaldo_Deploy

#### 2.2.1 Download
Clone the repository via git:

```bash
git clone https://github.com/OpenDriveLab/RoboNaldo_Deploy.git
```

#### 2.2.2 Install Components

Navigate to the directory and install:
```bash
cd RoboNaldo_Deploy
pip install -r requirements.txt
```

#### 2.2.3 Install unitree_sdk2_python

```bash
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
cd unitree_sdk2_python
pip install -e .
```

## Documentation Map

For real-robot onboard setup, start with `onboard/docs/README.md`. It explains
the runtime architecture, topic ownership, and the recommended reading path for
new contributors.

- Environment setup: `onboard/docs/INSTALL_AGENT_GUIDE.md`
- Real-robot bring-up topology: `tools/start_tmux_layout.sh`
- Camera module guide: `onboard/perception/camera/README.md`
- LiDAR module guide: `onboard/perception/lidar/README.md`
- Perception architecture: `onboard/docs/PERCEPTION_ARCHITECTURE.md`

---
## Running the Code

## 1. Run Mujoco Simulation

Standard scene:
```bash
python deploy_mujoco/deploy_mujoco.py
```

FreeKick scene (includes ball, required for FreeKick policy):
```bash
python deploy_mujoco/deploy_mujoco.py --config-name mujoco_freekick
```

## 2. Policy Descriptions
| Mode Name           | Trigger Keys                  | Description                                                                              |
|---------------------|-------------------------------|------------------------------------------------------------------------------------------|
| **PassiveMode**     | F1 (real) / L2 release (sim)  | Damping protection mode                                                                  |
| **FixedPose**       | Start                         | Position control reset to default joint values                                           |
| **LocoMode**        | B                             | Stable walking control mode                                                              |
| **AMP**             | A                             | AMP locomotion policy (enters run mode by default)                                       |
| **FreeKick**           | R1                            | Ball-kicking/scoring policy (547-dim obs, 5-frame history, requires onboard LiDAR)      |
| **BeyondMimicMJ**   | D-pad DOWN                    | Stand-to-lying imitation policy (MuJoCo-trained, with reference motion tracking)        |
| **StandUpMJ**       | D-pad UP                      | Lying-to-standing imitation policy (MuJoCo-trained, with reference motion tracking)     |
| **Pinocchio1.6MJ**  | R2                            | MuJoCo imitation policy backed by `g1_result_pinocchio_1_6_mj.yaml`                     |
| **BeyondMimic**     | —                             | Still kept in the repo, but not bound to a default controller shortcut                  |

### Manual Bias Adjustments

Both sim and real support runtime Y-axis bias to correct perception offsets without restarting:

| Action | Keys | Range | Notes |
|--------|------|-------|-------|
| Target Y +5 cm (left) | Hold **L1** + tap **D-pad Left** | ±1.5 m | Sim + real |
| Target Y −5 cm (right) | Hold **L1** + tap **D-pad Right** | ±1.5 m | Sim + real |
| Ball Y +5 cm (left) | Hold **L2** + tap **D-pad Left** | ±1.5 m | Real only |
| Ball Y −5 cm (right) | Hold **L2** + tap **D-pad Right** | ±1.5 m | Real only |

Bias is in the pelvis body frame (+Y = robot's left). It shifts the position fed to the policy without affecting the raw sensor reading. Each change prints to the terminal and is always visible in the **Sensor Dashboard** orange "Current Bias" card.


---
## 3. Operation Instructions in Simulation
1. Connect an Xbox controller.
2. Run the simulation program:
```bash
python deploy_mujoco/deploy_mujoco.py
```
3. Press the **Start** button to enter position control mode.
4. Press **B** to enter **LocoMode**, then press BACKSPACE in the simulation to make the robot stand. Afterward, use the joystick to control walking.
5. The simplified single-button policy map is:
   - **A** -> AMP (run mode)
   - **B** -> LocoMode
   - **D-pad DOWN** -> BeyondMimicMJ
   - **D-pad UP** -> StandUpMJ
   - **R2** -> Pinocchio1.6MJ
   - **R1** -> FreeKick
6. Use `--config-name mujoco_freekick` when entering **FreeKick** so the simulation loads the ball scene.
7. **BeyondMimicMJ / StandUpMJ / Pinocchio1.6MJ** can switch directly between each other with the same single-button bindings.
8. Release **L2** at any time for damping protection (PassiveMode), or press **Start** to return to **FixedPose**.
9. **BeyondMimic** is still present in the repository, but it is intentionally left without a default controller shortcut. Unused legacy policies such as Dance / Kick / KungFu / ASAP / HOST have been deleted from the repository.

---
## 4. Real Robot Operation Instructions

1. Power on the robot and suspend it (e.g., with a harness). First hold **L2+B** to enter damping protection mode (the head light changes from purple to orange), then hold **L2+R2** to enter debug mode (the head light changes from orange to yellow).

2. **Recommended (onboard Orin):** Use the tmux launcher, which starts the C++ bridge, policy runtime, perception services and sensor dashboard in one shot:
   ```bash
   bash tools/start_tmux_layout.sh
   ```
   Then in the `deploy_policy` pane, press **Start** to enter position control mode.

   Alternatively, run the C++ bridge and Python policy manually:
   ```bash
   BRIDGE_NETWORK_INTERFACE=eth0 bridge/build/cpp_bridge_main   # terminal 1
   python bridge/python/deploy_policy.py                          # terminal 2
   ```

   For quick testing without the C++ bridge (Python bridge prototype):
   ```bash
   python bridge/python/deploy_bridge_py.py   # terminal 1
   python bridge/python/deploy_policy.py       # terminal 2
   ```

   **Simple mode (no Orin / direct Ethernet):** `python deploy_real/deploy_real.py`

3. The same single-button mapping applies on the real robot: **A / B / D-pad DOWN / D-pad UP / R2 / R1**.
   Press **F1** at any time for damping protection (PassiveMode).

4. **FreeKick policy (R1) — additional real-robot steps**: The FreeKick policy depends on onboard ball perception. Start the perception services before activating FreeKick; raw LiDAR/camera observations are fused by `onboard/perception/ball_fuser.py`, which publishes the final DDS topic `rt/ball_state`. Do **not** activate the FreeKick policy without the perception services running.

5. **Sensor Dashboard**: After starting `tools/start_tmux_layout.sh`, open `http://<robot-ip>:8091/` in a browser. The dashboard shows all sensor positions (target, cam/lidar/fused ball, corrected positions) and the active bias values in the orange "Current Bias" card. To start it manually:
   ```bash
   python onboard/perception/debug/sensor_dashboard.py
   ```

   See `bridge/README.md` and `bridge/VALIDATION.md` for full bridge setup details.

---
## Important Notes
### 1. Framework Compatibility Notice
The current framework does not natively support deployment on G1 robots equipped with Orin NX platforms. Preliminary analysis suggests compatibility issues with the `unitree_python_sdk` on Orin systems. For onboard Orin deployment, we recommend the following alternative solution:

- Replace with [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2) (official C++ SDK)
- Implement a dual-node ROS architecture:
  - **C++ Node**: Handles data transmission between robot and controller
  - **Python Node**: Dedicated to policy inference
- This repository now includes a minimal bridge prototype under `bridge/` that can serve as a reference implementation for that split.

### 2. Mimic Policy Reliability Warning
The Mimic policy does not guarantee 100% success rate, particularly on slippery/sandy surfaces. In case of robot instability:
- Press `F1` to activate **PassiveMode** (damping protection)
- Press `Select` to immediately terminate the control program

### 3. Notes For The Retained Policies
The default deploy entry points are now limited to `LocoMode / AMP / FreeKick / BeyondMimicMJ / StandUpMJ / Pinocchio1.6MJ`.

⚠️ **Important Precautions**:
- **Palm Removal Recommended**: Some original training setups did not account for palm collisions (the author's G1 had no palms)
- **Prefer Safe Transition Paths**: Before and after testing a retained policy, prefer returning to **FixedPose** or **PassiveMode**
- **Use Human Supervision**: For first-time real-robot validation, keep the robot suspended and maintain manual supervision throughout

### 4. Other Movement Advisories
Only the retained deploy policies remain in the repository. Other historical policy implementations were removed to keep the deployment surface minimal.

### 5. Strong Recommendation
**Always** master operations in simulation before attempting physical robot deployment.

## Acknowledgements

This deployment framework is built on top of
[**ccrpRepo/RoboMimic_Deploy**](https://github.com/ccrpRepo/RoboMimic_Deploy). We thank the
authors for open-sourcing the original state-switching multi-policy deployment framework that
this repository extends.

It is the deployment counterpart of [**RoboNaldo**](https://opendrivelab.com/RoboNaldo/) —
training code: [opendrivelab/RoboNaldo](https://github.com/opendrivelab/RoboNaldo).
