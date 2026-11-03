# ShootLab V24/V34 部署

该目录部署高层 V24 decoder / V34 kick-curriculum PPO checkpoint
`v24_decoder_v34_kickcurriculum_bound_gpu1_s42_30k_20261023_225406/model_30000.pt`。

## 模型契约

- `policy_obs[1,430]`：7 个观测项，每项 5 帧，按 oldest→newest 拼接。
- `self_obs_raw[1,93]`：`base_ang_vel | joint_pos_rel | joint_vel | last_decoded_action | projected_gravity`。
- `joint_targets[1,29]`：Isaac Lab BFS 顺序；运行时映射到 G1 的 MuJoCo/SDK 顺序。
- 推理是确定性的：actor 使用均值，prior 使用均值，latent 残差按 checkpoint 最终 bound 5.0 计算。

模型、metadata、测试向量和 manifest 位于 `model/`。ONNX 按仓库规则不提交 Git。

## 重新导出

```bash
/opt/conda/envs/instinct/bin/python tools/export_shootlab_shoot_onnx.py
```

导出会重新生成 ONNX、metadata、固定测试向量和 manifest，并执行 ONNX checker 与
PyTorch/ONNX 数值对比。

## 验证

```bash
/opt/conda/envs/rn_deploy/bin/python -m unittest -v tests/test_shootlab_deployment.py
/opt/conda/envs/rn_deploy/bin/python tools/verify_shootlab_mujoco.py
```

真机默认目标源为 AprilTag；在 `config/shootlab_shoot.yaml` 中改为 `fixed` 可只使用固定
pelvis 目标。策略最多执行 250 个 50 Hz step（5 秒），随后自动回 FixedPose。
