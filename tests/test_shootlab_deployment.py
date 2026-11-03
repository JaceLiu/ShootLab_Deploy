"""Contract and runtime tests for the ShootLab V24/V34 deployment."""

from pathlib import Path
import hashlib
import json
import sys
import unittest

import numpy as np
import onnxruntime as ort


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from common.ctrlcomp import PolicyOutput, StateAndCmd
from common.utils import FSMCommand, FSMStateName
from policy.shootlab.ShootLabShoot import (
    ShootLabShoot,
    projected_gravity_b,
)


MODEL_DIR = ROOT / "policy/shootlab/model"
MANIFEST_PATH = MODEL_DIR / "MODEL_MANIFEST.json"
REQUIRE_MODEL = unittest.skipUnless(
    (MODEL_DIR / "policy-shootlab-v24-v34-30000.onnx").is_file(),
    "exported ShootLab model unavailable",
)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def make_policy(runtime_mode: str = "sim") -> tuple[ShootLabShoot, StateAndCmd, PolicyOutput]:
    state = StateAndCmd(29)
    output = PolicyOutput(29)
    policy = ShootLabShoot(state, output, runtime_mode=runtime_mode)
    state.q[:] = policy.default_q
    state.dq[:] = 0.0
    state.root_ang_vel_b[:] = 0.0
    state.pelvis_pos_w[:] = [0.0, 0.0, 0.76]
    state.pelvis_quat_w[:] = [1.0, 0.0, 0.0, 0.0]
    state.ball_pos_w[:] = [0.7, 0.0, 0.115]
    state.ball_pos_b[:] = [0.7, 0.0, -0.645]
    state.ball_valid = True
    return policy, state, output


@REQUIRE_MODEL
class ArtifactTests(unittest.TestCase):
    def test_manifest_hashes_match_artifacts(self):
        manifest = json.loads(MANIFEST_PATH.read_text())
        for artifact_key, hash_key in (
            ("onnx", "onnx_sha256"),
            ("metadata", "metadata_sha256"),
            ("test_vectors", "test_vectors_sha256"),
        ):
            self.assertEqual(sha256(MODEL_DIR / manifest[artifact_key]), manifest[hash_key])

    def test_onnx_matches_saved_pytorch_vectors(self):
        metadata = json.loads(
            (MODEL_DIR / "policy-shootlab-v24-v34-30000_meta.json").read_text()
        )
        vectors = np.load(MODEL_DIR / "test_vectors.npz")
        session = ort.InferenceSession(
            str(MODEL_DIR / "policy-shootlab-v24-v34-30000.onnx"),
            providers=["CPUExecutionProvider"],
        )
        output_names = [
            "joint_targets",
            "raw_z",
            "prior_mu",
            "prior_std",
            "z_decoder",
            "decoded_action",
            "self_obs_normalized",
        ]
        outputs = session.run(
            output_names,
            {
                "policy_obs": vectors["policy_obs"],
                "self_obs_raw": vectors["self_obs_raw"],
            },
        )
        tolerance = float(metadata["test_tolerance"])
        for name, output in zip(output_names, outputs):
            self.assertLessEqual(float(np.max(np.abs(output - vectors[name]))), tolerance)


@REQUIRE_MODEL
class RuntimeTests(unittest.TestCase):
    def test_gravity_matches_isaac_convention(self):
        np.testing.assert_allclose(
            projected_gravity_b(np.array([1.0, 0.0, 0.0, 0.0])),
            [0.0, 0.0, -1.0],
            atol=1e-6,
        )

    def test_observation_layout_and_inference(self):
        policy, state, output = make_policy()
        expected_kps = np.array(
            [
                40.179, 99.098, 40.179, 99.098, 28.501, 28.501,
                40.179, 99.098, 40.179, 99.098, 28.501, 28.501,
                40.179, 28.501, 28.501,
                14.251, 14.251, 14.251, 14.251, 14.251, 16.778, 16.778,
                14.251, 14.251, 14.251, 14.251, 14.251, 16.778, 16.778,
            ],
            dtype=np.float32,
        )
        np.testing.assert_allclose(policy.kps, expected_kps, atol=1e-3)
        expected_default = np.array(
            [
                -0.312, 0.0, 0.0, 0.669, -0.363, 0.0,
                -0.312, 0.0, 0.0, 0.669, -0.363, 0.0,
                0.0, 0.0, 0.0,
                0.2, 0.2, 0.0, 0.6, 0.0, 0.0, 0.0,
                0.2, -0.2, 0.0, 0.6, 0.0, 0.0, 0.0,
            ],
            dtype=np.float32,
        )
        np.testing.assert_allclose(policy.default_q, expected_default, atol=1e-6)
        policy.enter()
        for _ in range(policy.warmup_steps):
            policy.run()

        policy_obs, self_obs = policy._build_observations()
        self.assertEqual(policy_obs.shape, (430,))
        self.assertEqual(self_obs.shape, (93,))
        np.testing.assert_allclose(policy_obs[12:15], [0.0, 0.0, -1.0])
        np.testing.assert_allclose(self_obs[-3:], [0.0, 0.0, -1.0])

        policy.run()
        self.assertTrue(np.isfinite(output.actions).all())
        self.assertIn(policy._target_source_runtime, {"fixed_sim"})

    def test_policy_duration_returns_to_fixed_pose(self):
        policy, state, _ = make_policy()
        policy.warmup_steps = 0
        policy.max_policy_steps = 2
        policy.enter()
        policy.run()
        policy.run()
        policy.run()
        self.assertEqual(state.skill_cmd, FSMCommand.POS_RESET)
        self.assertEqual(policy._exit_reason, "policy_duration_elapsed")

    def test_ball_timeout_returns_to_fixed_pose(self):
        policy, state, _ = make_policy()
        policy.warmup_steps = 0
        policy.max_ball_invalid_steps = 2
        policy.enter()
        state.ball_valid = False
        policy.run()
        policy.run()
        policy.run()
        self.assertEqual(state.skill_cmd, FSMCommand.POS_RESET)
        self.assertEqual(policy._exit_reason, "ball_state_timeout")

    def test_real_fixed_target_and_april_hold(self):
        policy, state, _ = make_policy(runtime_mode="real")
        policy.target_source = "fixed"
        policy.target_pos[:] = [2.0, 0.0, 1.0]
        policy.enter()
        fixed_b = policy._target_pos_b()
        np.testing.assert_allclose(fixed_b, [2.0, 0.0, 1.0], atol=1e-6)

        policy.target_source = "apriltag"
        state.target_valid = True
        state.target_pos_b[:] = [3.0, 0.5, 1.2]
        live_b = policy._target_pos_b()
        np.testing.assert_allclose(live_b, state.target_pos_b, atol=1e-6)
        state.target_valid = False
        held_b = policy._target_pos_b()
        np.testing.assert_allclose(held_b, live_b, atol=1e-6)
        self.assertEqual(policy._target_source_runtime, "imu_hold")

    def test_y_mode_identifiers_are_distinct(self):
        self.assertNotEqual(FSMStateName.SKILL_SHOOTLAB, FSMStateName.SKILL_FREEKICK)
        self.assertNotEqual(FSMCommand.CMD_SHOOTLAB, FSMCommand.CMD_FREEKICK)


if __name__ == "__main__":
    unittest.main()
