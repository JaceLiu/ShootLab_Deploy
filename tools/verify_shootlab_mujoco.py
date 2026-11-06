"""Run the ShootLab V24/V34 policy through the deployment FSM in MuJoCo."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import mujoco
import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from FSM.FSM import FSM
from common.ctrlcomp import PolicyOutput, StateAndCmd
from common.utils import FSMCommand, FSMStateName


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config-name", default="mujoco_freekick")
    parser.add_argument("--max-steps", type=int, default=320)
    parser.add_argument("--target", type=float, nargs=3, default=[5.0, 0.0, 0.115])
    args = parser.parse_args()

    config_path = ROOT / "deploy_mujoco/config" / f"{args.config_name}.yaml"
    with config_path.open("r") as file:
        deploy_cfg = yaml.safe_load(file)
    model = mujoco.MjModel.from_xml_path(str(ROOT / deploy_cfg["xml_path"]))
    data = mujoco.MjData(model)
    decimation = int(deploy_cfg["control_decimation"])
    model.opt.timestep = float(deploy_cfg["simulation_dt"])
    tau_limit = np.asarray(deploy_cfg["tau_limit"], dtype=np.float64)

    state = StateAndCmd(model.nu)
    output = PolicyOutput(model.nu)
    fsm = FSM(state, output, runtime_mode="sim")
    policy = fsm.shootlab_policy
    policy.set_target_pos_w(args.target)

    ball_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "ball")
    foot_bodies = {
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "left_ankle_roll_link"),
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "right_ankle_roll_link"),
    }
    if ball_body < 0:
        raise RuntimeError("MuJoCo scene must contain a body named 'ball'")

    mujoco.mj_resetData(model, data)
    data.qpos[:3] = [0.0, 0.0, 0.76]
    data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
    data.qpos[7 : 7 + model.nu] = policy.default_q
    ball_joint = int(model.body_jntadr[ball_body])
    ball_qpos = int(model.jnt_qposadr[ball_joint])
    ball_qvel = int(model.jnt_dofadr[ball_joint])
    data.qpos[ball_qpos : ball_qpos + 3] = [0.7, 0.0, 0.115]
    data.qpos[ball_qpos + 3 : ball_qpos + 7] = [1.0, 0.0, 0.0, 0.0]
    data.qvel[ball_qvel : ball_qvel + 6] = 0.0
    mujoco.mj_forward(model, data)

    def update_state(ball_update: bool) -> None:
        state.q = data.qpos[7 : 7 + model.nu].copy()
        state.dq = data.qvel[6 : 6 + model.nu].copy()
        state.root_ang_vel_b = data.qvel[3:6].astype(np.float32)
        state.pelvis_pos_w = data.qpos[:3].astype(np.float32)
        state.pelvis_quat_w = data.qpos[3:7].astype(np.float32)
        state.ball_valid = True
        if ball_update:
            state.ball_pos_w = data.xpos[ball_body].astype(np.float32)

    update_state(ball_update=True)
    state.skill_cmd = FSMCommand.POS_RESET
    fsm.run()
    fsm.run()
    self_check = fsm.cur_policy.name == FSMStateName.FIXEDPOSE
    if not self_check:
        raise RuntimeError(f"Failed to enter FixedPose: {fsm.cur_policy.name}")
    state.skill_cmd = FSMCommand.CMD_SHOOTLAB

    foot_contact = False
    fell = False
    policy_steps = 0
    entered_shootlab = False
    exited_shootlab = False
    for step in range(args.max_steps):
        for _ in range(decimation):
            target_q = np.asarray(output.actions, dtype=np.float64)
            q = data.qpos[7 : 7 + model.nu]
            dq = data.qvel[6 : 6 + model.nu]
            tau = (
                (target_q - q) * np.asarray(output.kps, dtype=np.float64)
                - dq * np.asarray(output.kds, dtype=np.float64)
            )
            data.ctrl[:] = np.clip(tau, -tau_limit, tau_limit)
            mujoco.mj_step(model, data)
            for contact in data.contact[: data.ncon]:
                bodies = {int(model.geom_bodyid[contact.geom1]), int(model.geom_bodyid[contact.geom2])}
                if ball_body in bodies and bodies & foot_bodies:
                    foot_contact = True
            fell |= bool(data.qpos[2] < 0.35)

        update_state(ball_update=step % 5 == 0)
        previous_name = fsm.cur_policy.name
        fsm.run()
        current_name = fsm.cur_policy.name
        if current_name == FSMStateName.SKILL_SHOOTLAB:
            entered_shootlab = True
            if previous_name == current_name:
                policy_steps = policy.time_step - policy.warmup_steps
        if entered_shootlab and current_name == FSMStateName.FIXEDPOSE:
            exited_shootlab = True
            break
        if not np.isfinite(output.actions).all():
            raise RuntimeError(f"Non-finite action at control step {step}")

    summary = {
        "control_steps": step + 1,
        "policy_steps": policy_steps,
        "entered_shootlab": entered_shootlab,
        "auto_returned_to_fixed": exited_shootlab,
        "foot_ball_contact": foot_contact,
        "fell": fell,
    }
    print(summary)
    if not entered_shootlab or not exited_shootlab:
        raise RuntimeError("ShootLab FSM did not enter and time out correctly")
    if policy_steps != policy.max_policy_steps:
        raise RuntimeError(f"Expected {policy.max_policy_steps} policy steps, got {policy_steps}")
    if not foot_contact:
        raise RuntimeError("Policy did not make foot-ball contact")
    if fell:
        raise RuntimeError("Policy fell during the nominal MuJoCo scenario")


if __name__ == "__main__":
    main()
