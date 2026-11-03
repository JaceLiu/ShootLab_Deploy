#!/usr/bin/env python3
"""Compare two independently solved rigid extrinsics using A/B limits."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any, Sequence

from g1_calibration.manifest import load_yaml

ROTATION_KEYS = (
    "rotation_matrix",
    "R",
    "R_lidar_imu",
    "R_imu_lidar",
    "extrinsic_rotation",
    "rotation",
)
TRANSLATION_KEYS = (
    "translation_m",
    "translation",
    "t_xyz",
    "t",
)
TRANSFORM_KEYS = ("T_imu_lidar", "T_lidar_imu", "transform", "T")
QUATERNION_KEYS = ("q_xyzw", "quaternion_xyzw", "quaternion")


def _walk(value: Any):
    if isinstance(value, dict):
        yield value
        for child in value.values():
            yield from _walk(child)
    elif isinstance(value, list):
        for child in value:
            yield from _walk(child)


def _numeric_list(value: Any) -> list[float] | None:
    if isinstance(value, list) and value and all(
        isinstance(item, (int, float)) for item in value
    ):
        return [float(item) for item in value]
    if isinstance(value, dict):
        for names in (("x", "y", "z", "w"), ("qx", "qy", "qz", "qw")):
            if all(name in value for name in names):
                try:
                    return [float(value[name]) for name in names]
                except (TypeError, ValueError):
                    return None
        if all(name in value for name in ("w", "x", "y", "z")):
            try:
                return [float(value[name]) for name in ("x", "y", "z", "w")]
            except (TypeError, ValueError):
                return None
        row_names = ("r0c0", "r1c0", "r2c0")
        if all(name in value for name in row_names):
            try:
                return [float(value[name]) for name in row_names]
            except (TypeError, ValueError):
                return None
        if value and all(isinstance(key, int) for key in value):
            try:
                return [float(value[key]) for key in sorted(value)]
            except (TypeError, ValueError):
                return None
        for child in value.values():
            candidate = _numeric_list(child)
            if candidate is not None:
                return candidate
    return None


def _ikalibr_value(
    document: Any, key: str, topic: str = "/g1/mid360"
) -> Any | None:
    for mapping in _walk(document):
        if key not in mapping:
            continue
        value = mapping[key]
        if isinstance(value, dict) and topic in value:
            return value[topic]
        if isinstance(value, dict) and len(value) == 1:
            return next(iter(value.values()))
        return value
    return None


def parse_ikalibr_extrinsics(
    document: Any,
) -> tuple[list[list[float]], list[float]] | None:
    """Read cereal fields SO3_LkToBr and POS_LkInBr from iKalibr YAML."""
    rotation_value = _ikalibr_value(document, "SO3_LkToBr")
    translation_value = _ikalibr_value(document, "POS_LkInBr")
    if rotation_value is None or translation_value is None:
        return None
    quaternion = _numeric_list(rotation_value)
    translation = _numeric_list(translation_value)
    if quaternion is None or len(quaternion) != 4:
        return None
    if translation is None or len(translation) != 3:
        return None
    return _rotation_from_quaternion(quaternion), translation


def _rotation_from_matrix(matrix: Sequence[Sequence[float]]) -> list[list[float]]:
    if len(matrix) not in (3, 4) or any(len(row) not in (3, 4) for row in matrix):
        raise ValueError("rotation matrix must be 3x3 or a 4x4 transform")
    return [[float(matrix[row][column]) for column in range(3)] for row in range(3)]


def _translation_from_matrix(matrix: Sequence[Sequence[float]]) -> list[float]:
    if len(matrix) < 3:
        raise ValueError("4x4 transform needs at least three rows")
    if all(len(row) >= 4 for row in matrix[:3]):
        return [float(matrix[index][3]) for index in range(3)]
    raise ValueError("4x4 transform has no translation column")


def _rotation_from_quaternion(values: Sequence[float]) -> list[list[float]]:
    if len(values) != 4:
        raise ValueError("quaternion must have four values")
    x, y, z, w = (float(value) for value in values)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0:
        raise ValueError("quaternion has zero norm")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


def parse_extrinsics(document: Any) -> tuple[list[list[float]], list[float]]:
    ikalibr_result = parse_ikalibr_extrinsics(document)
    if ikalibr_result is not None:
        return ikalibr_result
    for mapping in _walk(document):
        for key in TRANSFORM_KEYS:
            if key in mapping:
                matrix = mapping[key]
                return _rotation_from_matrix(matrix), _translation_from_matrix(matrix)
        rotation = next(
            (mapping[key] for key in ROTATION_KEYS if key in mapping), None
        )
        if rotation is not None and len(rotation) >= 3 and isinstance(rotation[0], list):
            translation = next(
                (mapping[key] for key in TRANSLATION_KEYS if key in mapping), None
            )
            if translation is not None:
                if len(translation) != 3:
                    raise ValueError("translation must contain x, y, z")
                return _rotation_from_matrix(rotation), [
                    float(value) for value in translation
                ]
        for key in QUATERNION_KEYS:
            if key in mapping:
                translation = next(
                    (
                        mapping[translation_key]
                        for translation_key in TRANSLATION_KEYS
                        if translation_key in mapping
                    ),
                    None,
                )
                if translation is not None:
                    return _rotation_from_quaternion(mapping[key]), [
                        float(value) for value in translation
                    ]
    raise ValueError(
        "could not find rotation_matrix/q_xyzw and translation_m/t_xyz in result"
    )


def matrix_multiply(left: Sequence[Sequence[float]], right: Sequence[Sequence[float]]):
    return [
        [
            sum(left[row][index] * right[index][column] for index in range(3))
            for column in range(3)
        ]
        for row in range(3)
    ]


def transpose(matrix: Sequence[Sequence[float]]) -> list[list[float]]:
    return [[float(matrix[column][row]) for column in range(3)] for row in range(3)]


def relative_rotation_deg(
    first: Sequence[Sequence[float]], second: Sequence[Sequence[float]]
) -> float:
    relative = matrix_multiply(second, transpose(first))
    cosine = (sum(relative[index][index] for index in range(3)) - 1.0) / 2.0
    return math.degrees(math.acos(max(-1.0, min(1.0, cosine))))


def translation_delta_m(first: Sequence[float], second: Sequence[float]) -> float:
    return math.sqrt(sum((float(b) - float(a)) ** 2 for a, b in zip(first, second)))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_a", type=Path)
    parser.add_argument("run_b", type=Path)
    parser.add_argument("--max-rotation-deg", type=float, default=1.0)
    parser.add_argument("--max-translation-m", type=float, default=0.02)
    parser.add_argument("--json-out", type=Path)
    args = parser.parse_args()
    try:
        rotation_a, translation_a = parse_extrinsics(load_yaml(args.run_a))
        rotation_b, translation_b = parse_extrinsics(load_yaml(args.run_b))
    except Exception as exc:
        print(f"EXTRINSICS ERROR: {exc}")
        return 2
    rotation_difference = relative_rotation_deg(rotation_a, rotation_b)
    translation_difference = translation_delta_m(translation_a, translation_b)
    passed = (
        rotation_difference <= args.max_rotation_deg
        and translation_difference <= args.max_translation_m
    )
    report = {
        "status": "passed" if passed else "failed",
        "rotation_difference_deg": rotation_difference,
        "translation_difference_m": translation_difference,
        "limits": {
            "rotation_deg": args.max_rotation_deg,
            "translation_m": args.max_translation_m,
        },
    }
    print(json.dumps(report, indent=2))
    if args.json_out:
        args.json_out.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
