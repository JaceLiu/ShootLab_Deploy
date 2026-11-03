"""Capture manifest creation and YAML/JSON interchange without ROS imports."""

from __future__ import annotations

import json
import re
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

CAPTURE_ID_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]*$")


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def validate_capture_id(capture_id: str) -> str:
    if not CAPTURE_ID_PATTERN.fullmatch(capture_id):
        raise ValueError(
            "capture id must start with an alphanumeric character and contain "
            "only letters, digits, '.', '_' or '-'"
        )
    return capture_id


def dump_yaml(data: Any) -> str:
    """Dump YAML when PyYAML is available, otherwise emit valid YAML JSON."""
    try:
        import yaml  # type: ignore
    except ImportError:
        return json.dumps(data, indent=2, ensure_ascii=False, sort_keys=False) + "\n"
    return yaml.safe_dump(
        data, allow_unicode=True, sort_keys=False, default_flow_style=False
    )


def write_yaml(path: Path, data: Any) -> None:
    path.write_text(dump_yaml(data), encoding="utf-8")


def load_yaml(path: Path) -> Any:
    text = path.read_text(encoding="utf-8")
    try:
        import yaml  # type: ignore
    except ImportError:
        return json.loads(text)
    return yaml.safe_load(text)


def build_manifest(
    *,
    capture_id: str,
    profile: str,
    duration_s: float,
    robot_model: str,
    sensor: str,
    scenario: str,
    motion: str,
    operator: str,
    notes: str,
    topics: dict[str, str],
) -> dict[str, Any]:
    now = utc_now()
    return {
        "schema_version": 1,
        "capture_id": capture_id,
        "status": "recording",
        "created_at_utc": now,
        "updated_at_utc": now,
        "profile": profile,
        "requested_duration_s": duration_s,
        "robot": {
            "model": robot_model,
            "sensor": sensor,
            "calibration_target": "MID360 to torso IMU",
        },
        "environment": {
            "scenario": scenario,
            "static_features": "stationary structure at 1-10 m",
        },
        "operation": {
            "operator": operator,
            "motion": motion,
            "safety": "hoisted or otherwise restrained; FreeKick disabled",
        },
        "topics": topics,
        "notes": notes,
        "intended_capture": {
            "storage": "MCAP",
            "ros_distro": "ROS2",
            "ikalibr_runtime": "PC only",
        },
        "files": {},
        "analysis": None,
    }
