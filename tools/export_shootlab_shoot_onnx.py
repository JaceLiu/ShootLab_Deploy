"""Export the ShootLab V24/V34 policy chain as one deterministic ONNX model."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import numpy as np
import onnx
import onnxruntime as ort
import torch
from torch import nn
import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CHECKPOINT = (
    "/workspace/instinct_workspace/ShootLab/logs/instinct_rl/"
    "g1_soccer_v24_v34_transfer/"
    "v24_decoder_v34_kickcurriculum_bound_gpu1_s42_30k_20261023_225406/"
    "model_30000.pt"
)
DEFAULT_DECODER_CHECKPOINT = (
    "/workspace/instinct_workspace/ShootLab/logs/instinct_rl/"
    "g1_beyondmimic_distill_v24/shoot_distill_v2/model_30000.pt"
)
DEFAULT_ENV_YAML = Path(DEFAULT_CHECKPOINT).parent / "params/env.yaml"
DEFAULT_OUTPUT_DIR = PROJECT_ROOT / "policy/shootlab/model"

ISAAC_JOINT_NAMES = [
    "left_hip_pitch_joint",
    "right_hip_pitch_joint",
    "waist_yaw_joint",
    "left_hip_roll_joint",
    "right_hip_roll_joint",
    "waist_roll_joint",
    "left_hip_yaw_joint",
    "right_hip_yaw_joint",
    "waist_pitch_joint",
    "left_knee_joint",
    "right_knee_joint",
    "left_shoulder_pitch_joint",
    "right_shoulder_pitch_joint",
    "left_ankle_pitch_joint",
    "right_ankle_pitch_joint",
    "left_shoulder_roll_joint",
    "right_shoulder_roll_joint",
    "left_ankle_roll_joint",
    "right_ankle_roll_joint",
    "left_shoulder_yaw_joint",
    "right_shoulder_yaw_joint",
    "left_elbow_joint",
    "right_elbow_joint",
    "left_wrist_roll_joint",
    "right_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "right_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_wrist_yaw_joint",
]

POLICY_TERMS = [
    ("projected_gravity", 3),
    ("base_ang_vel", 3),
    ("joint_pos", 29),
    ("joint_vel", 29),
    ("actions", 16),
    ("target_point_pos", 3),
    ("target_destination_pos_local", 3),
]
SELF_OBS_TERMS = [
    ("base_ang_vel", 3),
    ("joint_pos_rel", 29),
    ("joint_vel", 29),
    ("last_decoded_action", 29),
    ("projected_gravity", 3),
]
HISTORY_LEN = 5
POLICY_OBS_DIM = sum(dim * HISTORY_LEN for _, dim in POLICY_TERMS)
SELF_OBS_DIM = sum(dim for _, dim in SELF_OBS_TERMS)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file:
        for chunk in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_yaml(path: Path) -> dict:
    with path.open("r") as file:
        return yaml.load(file, Loader=yaml.UnsafeLoader)


def match_joint_pattern(pattern: str, joint_name: str) -> bool:
    if pattern == joint_name:
        return True
    if pattern == ".*":
        return True
    if pattern.startswith(".*_"):
        return joint_name.endswith(pattern[3:])
    return False


def resolve_joint_value(joint_name: str, values: dict, field: str) -> float:
    matches = [
        (pattern, value)
        for pattern, value in values.items()
        if isinstance(value, (int, float)) and match_joint_pattern(pattern, joint_name)
    ]
    exact = [value for pattern, value in matches if pattern == joint_name]
    if exact:
        return float(exact[0])
    if not matches:
        raise ValueError(f"No {field} value matches joint {joint_name!r}")
    return float(matches[0][1])


def strip_prefix(value: str, prefix: str) -> str:
    if not value.startswith(prefix):
        raise ValueError(f"{value!r} does not start with {prefix!r}")
    return value[len(prefix):]


def actuator_property(env_cfg: dict, joint_name: str, field: str) -> float:
    for actuator in env_cfg["scene"]["robot"]["actuators"].values():
        names = actuator.get("joint_names_expr", [])
        if not any(match_joint_pattern(pattern, joint_name) for pattern in names):
            continue
        values = actuator.get(field)
        if isinstance(values, dict):
            return resolve_joint_value(joint_name, values, field)
        if isinstance(values, (int, float)):
            return float(values)
    raise ValueError(f"No actuator {field} value matches joint {joint_name!r}")


def build_joint_metadata(env_cfg: dict) -> dict[str, list[float]]:
    init_joint_pos = env_cfg["scene"]["robot"]["init_state"]["joint_pos"]
    decoded_scale = env_cfg["actions"]["joint_pos"]["decoded_action_scale"]
    default_pos = []
    for name in ISAAC_JOINT_NAMES:
        try:
            value = resolve_joint_value(name, init_joint_pos, "default_joint_pos")
        except ValueError:
            value = 0.0
        default_pos.append(value)
    action_scale = [
        resolve_joint_value(name, decoded_scale, "decoded_action_scale")
        for name in ISAAC_JOINT_NAMES
    ]
    kps = [actuator_property(env_cfg, name, "stiffness") for name in ISAAC_JOINT_NAMES]
    kds = [actuator_property(env_cfg, name, "damping") for name in ISAAC_JOINT_NAMES]
    effort = [actuator_property(env_cfg, name, "effort_limit_sim") for name in ISAAC_JOINT_NAMES]

    urdf_path = Path(env_cfg["scene"]["robot"]["spawn"]["asset_path"])
    root = ET.parse(urdf_path).getroot()
    limits = {}
    for joint in root.findall(".//joint"):
        name = joint.attrib.get("name")
        limit = joint.find("limit")
        if name in ISAAC_JOINT_NAMES and limit is not None:
            limits[name] = (float(limit.attrib["lower"]), float(limit.attrib["upper"]))
    if set(limits) != set(ISAAC_JOINT_NAMES):
        missing = sorted(set(ISAAC_JOINT_NAMES) - set(limits))
        raise ValueError(f"URDF is missing joint limits for {missing}")

    return {
        "default_joint_pos": default_pos,
        "action_scale": action_scale,
        "kps": kps,
        "kds": kds,
        "effort_limit": effort,
        "joint_pos_min": [limits[name][0] for name in ISAAC_JOINT_NAMES],
        "joint_pos_max": [limits[name][1] for name in ISAAC_JOINT_NAMES],
    }


def mlp(input_size: int, hidden_dims: list[int], output_size: int | None) -> nn.Sequential:
    layers: list[nn.Module] = []
    previous = input_size
    for hidden in hidden_dims:
        layers.extend([nn.Linear(previous, hidden), nn.ELU()])
        previous = hidden
    if output_size is not None:
        layers.append(nn.Linear(previous, output_size))
    return nn.Sequential(*layers)


class FusedShootLabNetwork(nn.Module):
    def __init__(
        self,
        policy_state: dict,
        decoder_checkpoint: dict,
        policy_normalizer: dict,
        distill_normalizer: dict,
        joint_meta: dict,
        prior_residual_bound: float,
    ):
        super().__init__()
        actor_state = {
            strip_prefix(key, "actor."): value
            for key, value in policy_state.items()
            if key.startswith("actor.")
        }
        self.actor = mlp(430, [512, 256, 128], 16)
        self.actor.load_state_dict(actor_state, strict=True)

        decoder_state = decoder_checkpoint["model_state_dict"]
        self.decoder = mlp(109, [512, 256, 128], 29)
        self.decoder.load_state_dict(
            {
                strip_prefix(key, "actor.decoder.model."): value
                for key, value in decoder_state.items()
                if key.startswith("actor.decoder.model.")
            },
            strict=True,
        )
        self.z_prior = mlp(93, [256, 128, 64], None)
        self.z_prior.load_state_dict(
            {
                strip_prefix(key, "actor.z_prior.model."): value
                for key, value in decoder_state.items()
                if key.startswith("actor.z_prior.model.")
            },
            strict=True,
        )
        self.z_prior_mu = nn.Linear(64, 16)
        self.z_prior_mu.load_state_dict(
            {
                "weight": decoder_state["actor.z_prior_mu.weight"],
                "bias": decoder_state["actor.z_prior_mu.bias"],
            }
        )
        self.z_prior_logvar = nn.Linear(64, 16)
        self.z_prior_logvar.load_state_dict(
            {
                "weight": decoder_state["actor.z_prior_logvar.weight"],
                "bias": decoder_state["actor.z_prior_logvar.bias"],
            }
        )

        self.register_buffer(
            "policy_mean",
            policy_normalizer["_mean"].flatten().to(torch.float32),
        )
        self.register_buffer(
            "policy_std",
            policy_normalizer["_std"].flatten().to(torch.float32),
        )
        prior_indices = decoder_state["_vae_prior_indices"].flatten().to(torch.long)
        aux_indices = decoder_state["_vae_aux_indices"].flatten().to(torch.long)
        if not torch.equal(prior_indices, aux_indices):
            raise ValueError("V24 prior and decoder auxiliary self-observation indices differ")
        if not torch.all(prior_indices[1:] - prior_indices[:-1] == 1):
            raise ValueError("V24 self-observation normalization indices must be contiguous")
        distill_mean = distill_normalizer["_mean"].flatten().to(torch.float32)
        distill_var = distill_normalizer["_var"].flatten().to(torch.float32)
        self.register_buffer("self_obs_mean", distill_mean[prior_indices])
        self.register_buffer("self_obs_var", distill_var[prior_indices])
        self.register_buffer(
            "default_joint_pos",
            torch.tensor(joint_meta["default_joint_pos"], dtype=torch.float32),
        )
        self.register_buffer(
            "action_scale",
            torch.tensor(joint_meta["action_scale"], dtype=torch.float32),
        )
        self.policy_norm_eps = 1.0e-2
        self.self_obs_var_eps = 1.0e-5
        self.prior_residual_bound = float(prior_residual_bound)

    def forward(self, policy_obs: torch.Tensor, self_obs_raw: torch.Tensor):
        normalized_policy_obs = (policy_obs - self.policy_mean) / (self.policy_std + self.policy_norm_eps)
        raw_z = self.actor(normalized_policy_obs)

        self_obs_normalized = torch.clamp(
            (self_obs_raw - self.self_obs_mean)
            / torch.sqrt(self.self_obs_var + self.self_obs_var_eps),
            min=-5.0,
            max=5.0,
        )
        prior_hidden = self.z_prior(self_obs_normalized)
        prior_mu = self.z_prior_mu(prior_hidden)
        prior_logvar = torch.clamp(self.z_prior_logvar(prior_hidden), min=-5.0, max=2.0)
        prior_std = torch.exp(0.5 * prior_logvar).clamp_min(1.0e-8)
        z_decoder = prior_mu + prior_std * self.prior_residual_bound * torch.tanh(
            raw_z / self.prior_residual_bound
        )
        decoded_action = self.decoder(torch.cat([z_decoder, self_obs_normalized], dim=-1))
        joint_targets = self.default_joint_pos + self.action_scale * decoded_action
        return (
            joint_targets,
            raw_z,
            prior_mu,
            prior_std,
            z_decoder,
            decoded_action,
            self_obs_normalized,
        )


def make_test_vectors(network: FusedShootLabNetwork) -> dict[str, np.ndarray]:
    generator = torch.Generator().manual_seed(20261023)
    policy_obs = torch.randn((32, POLICY_OBS_DIM), generator=generator) * 0.35
    self_obs = torch.randn((32, SELF_OBS_DIM), generator=generator) * 0.35
    policy_obs[0].zero_()
    self_obs[0].zero_()
    with torch.no_grad():
        outputs = network(policy_obs, self_obs)
    keys = [
        "joint_targets",
        "raw_z",
        "prior_mu",
        "prior_std",
        "z_decoder",
        "decoded_action",
        "self_obs_normalized",
    ]
    vectors = {
        "policy_obs": policy_obs.numpy().astype(np.float32),
        "self_obs_raw": self_obs.numpy().astype(np.float32),
    }
    vectors.update(
        {key: value.numpy().astype(np.float32) for key, value in zip(keys, outputs)}
    )
    return vectors


def validate_onnx(
    onnx_path: Path,
    vectors: dict[str, np.ndarray],
) -> float:
    onnx.checker.check_model(str(onnx_path))
    session = ort.InferenceSession(str(onnx_path), providers=["CPUExecutionProvider"])
    feeds = {
        "policy_obs": vectors["policy_obs"],
        "self_obs_raw": vectors["self_obs_raw"],
    }
    output_names = [
        "joint_targets",
        "raw_z",
        "prior_mu",
        "prior_std",
        "z_decoder",
        "decoded_action",
        "self_obs_normalized",
    ]
    outputs = session.run(output_names, feeds)
    max_error = 0.0
    for name, output in zip(output_names, outputs):
        error = float(np.max(np.abs(output - vectors[name])))
        max_error = max(max_error, error)
        if not np.isfinite(output).all():
            raise RuntimeError(f"ONNX output {name} contains non-finite values")
    if max_error > 5.0e-5:
        raise RuntimeError(f"PyTorch/ONNX maximum absolute error is {max_error}")
    return max_error


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", type=Path, default=Path(DEFAULT_CHECKPOINT))
    parser.add_argument("--decoder-checkpoint", type=Path, default=Path(DEFAULT_DECODER_CHECKPOINT))
    parser.add_argument("--env-yaml", type=Path, default=DEFAULT_ENV_YAML)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument(
        "--artifact-stem",
        default="policy-shootlab-v24-v34-30000",
    )
    args = parser.parse_args()

    policy_checkpoint = torch.load(args.checkpoint, map_location="cpu", weights_only=False)
    decoder_checkpoint = torch.load(
        args.decoder_checkpoint,
        map_location="cpu",
        weights_only=False,
    )
    env_cfg = load_yaml(args.env_yaml)
    joint_meta = build_joint_metadata(env_cfg)
    runtime_action = policy_checkpoint["env_runtime_state"]["actions"]["joint_pos"]
    prior_residual_bound = float(runtime_action["prior_residual_bound"])
    if not np.isclose(prior_residual_bound, 5.0):
        raise ValueError(f"Expected final residual bound 5.0, got {prior_residual_bound}")

    network = FusedShootLabNetwork(
        policy_checkpoint["model_state_dict"],
        {
            "model_state_dict": decoder_checkpoint["model_state_dict"],
        },
        policy_checkpoint["policy_normalizer_state_dict"],
        decoder_checkpoint["policy_normalizer_state_dict"],
        joint_meta,
        prior_residual_bound,
    ).eval()
    vectors = make_test_vectors(network)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    onnx_path = args.output_dir / f"{args.artifact_stem}.onnx"
    meta_path = args.output_dir / f"{args.artifact_stem}_meta.json"
    vectors_path = args.output_dir / "test_vectors.npz"
    manifest_path = args.output_dir / "MODEL_MANIFEST.json"

    example_policy = torch.from_numpy(vectors["policy_obs"][:1])
    example_self = torch.from_numpy(vectors["self_obs_raw"][:1])
    torch.onnx.export(
        network,
        (example_policy, example_self),
        onnx_path,
        input_names=["policy_obs", "self_obs_raw"],
        output_names=[
            "joint_targets",
            "raw_z",
            "prior_mu",
            "prior_std",
            "z_decoder",
            "decoded_action",
            "self_obs_normalized",
        ],
        dynamic_axes={
            "policy_obs": {0: "batch"},
            "self_obs_raw": {0: "batch"},
            "joint_targets": {0: "batch"},
            "raw_z": {0: "batch"},
            "prior_mu": {0: "batch"},
            "prior_std": {0: "batch"},
            "z_decoder": {0: "batch"},
            "decoded_action": {0: "batch"},
            "self_obs_normalized": {0: "batch"},
        },
        opset_version=17,
        do_constant_folding=True,
    )
    max_error = validate_onnx(onnx_path, vectors)
    np.savez_compressed(vectors_path, **vectors, max_abs_error=np.array(max_error))

    metadata = {
        "format": "shootlab_shoot_fused_v3",
        "joint_names": ISAAC_JOINT_NAMES,
        **joint_meta,
        "policy_obs_dim": POLICY_OBS_DIM,
        "policy_obs_terms": [
            {"name": name, "dim": dim, "history": HISTORY_LEN}
            for name, dim in POLICY_TERMS
        ],
        "policy_history_order": "oldest_to_newest",
        "self_obs_terms": [
            {"name": name, "dim": dim} for name, dim in SELF_OBS_TERMS
        ],
        "self_obs_dim": SELF_OBS_DIM,
        "latent_dim": 16,
        "control_dt": 0.02,
        "joint_targets_order": "isaac_lab_bfs",
        "joint_targets_semantics": "default_joint_pos + action_scale * decoded_action",
        "latent_formula": (
            "prior_mu + prior_std * prior_residual_bound * "
            "tanh(raw_z / prior_residual_bound)"
        ),
        "latent_projection": "identity",
        "network": {
            "actor_hidden_dims": [512, 256, 128],
            "decoder_hidden_dims": [512, 256, 128],
            "prior_hidden_dims": [256, 128, 64],
            "policy_norm_eps": 0.01,
            "self_obs_var_eps": 1.0e-5,
            "self_obs_clamp_min": -5.0,
            "self_obs_clamp_max": 5.0,
            "prior_logvar_min": -5.0,
            "prior_logvar_max": 2.0,
            "prior_residual_bound": prior_residual_bound,
        },
        "test_tolerance": 5.0e-5,
        "test_max_abs_error": max_error,
        "source": {
            "policy_checkpoint": str(args.checkpoint.resolve()),
            "policy_checkpoint_sha256": sha256(args.checkpoint),
            "decoder_checkpoint": str(args.decoder_checkpoint.resolve()),
            "decoder_checkpoint_sha256": sha256(args.decoder_checkpoint),
            "env_yaml": str(args.env_yaml.resolve()),
            "env_yaml_sha256": sha256(args.env_yaml),
        },
    }
    meta_path.write_text(json.dumps(metadata, indent=2) + "\n")

    manifest = {
        "policy": "ShootLab V24 decoder / V34 kick curriculum",
        "checkpoint_iteration": int(policy_checkpoint["iter"]),
        "onnx": onnx_path.name,
        "onnx_sha256": sha256(onnx_path),
        "metadata": meta_path.name,
        "metadata_sha256": sha256(meta_path),
        "test_vectors": vectors_path.name,
        "test_vectors_sha256": sha256(vectors_path),
        "contract": {
            "policy_obs_dim": POLICY_OBS_DIM,
            "self_obs_dim": SELF_OBS_DIM,
            "latent_dim": 16,
            "joint_target_dim": 29,
            "prior_residual_bound": prior_residual_bound,
        },
        "source": metadata["source"],
    }
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"Exported {onnx_path}")
    print(f"Exported {meta_path}")
    print(f"Exported {vectors_path}")
    print(f"Exported {manifest_path}")
    print(f"PyTorch/ONNX max absolute error: {max_error:.3e}")


if __name__ == "__main__":
    main()
