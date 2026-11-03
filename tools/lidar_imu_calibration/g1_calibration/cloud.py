"""PointCloud2 field and scan-time handling shared by analysis and conversion."""

from __future__ import annotations

import math
import struct
from dataclasses import asdict, dataclass
from typing import Any, Sequence

from .constants import (
    CLOUD_RING_ALIASES,
    CLOUD_TIME_ALIASES,
    CLOUD_XYZI_FIELDS,
)

_DTYPES = {
    1: ("b", 1),
    2: ("B", 1),
    3: ("h", 2),
    4: ("H", 2),
    5: ("i", 4),
    6: ("I", 4),
    7: ("f", 4),
    8: ("d", 8),
}


@dataclass(frozen=True)
class CloudInspection:
    field_names: tuple[str, ...]
    field_datatypes: dict[str, int]
    ring_field: str
    time_field: str
    point_count: int
    sampled_points: int
    scan_time_min_s: float
    scan_time_max_s: float
    monotonic: bool

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


def _field_dicts(message: Any) -> dict[str, dict[str, int]]:
    fields: dict[str, dict[str, int]] = {}
    for field in message.fields:
        fields[field.name] = {
            "offset": int(field.offset),
            "datatype": int(field.datatype),
            "count": int(field.count),
        }
    return fields


def _value_at(
    payload: memoryview,
    point_index: int,
    point_step: int,
    field: dict[str, int],
    endian: str,
) -> float:
    code, _size = _DTYPES[field["datatype"]]
    offset = point_index * point_step + field["offset"]
    return float(struct.unpack_from(endian + code, payload, offset)[0])


def header_stamp_ns(message: Any) -> int:
    stamp = message.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def find_cloud_fields(message: Any) -> dict[str, dict[str, int]]:
    fields = _field_dicts(message)
    missing = [name for name in CLOUD_XYZI_FIELDS if name not in fields]
    if not any(name in fields for name in CLOUD_RING_ALIASES):
        missing.append("ring/line")
    if not any(name in fields for name in CLOUD_TIME_ALIASES):
        missing.append("time/timestamp")
    if missing:
        raise ValueError(f"PointCloud2 is missing fields: {', '.join(missing)}")
    return fields


def scan_time_offsets_s(
    field_name: str,
    raw_values: Sequence[float],
    header_ns: int,
    *,
    mode: str = "auto",
) -> list[float]:
    """Normalize point times to seconds relative to the cloud header stamp."""
    if not raw_values:
        return []
    if mode not in {"auto", "native", "converted"}:
        raise ValueError("cloud time mode must be auto, native, or converted")

    values = [float(value) for value in raw_values]
    magnitude = max(abs(value) for value in values)
    is_native = field_name == "timestamp" or magnitude > 1e15
    if mode == "native":
        is_native = True
    elif mode == "converted":
        return values

    if is_native:
        # Native Livox timestamps are absolute nanoseconds. Some bag tools expose
        # the same field as scan-relative nanoseconds; both remain unambiguous
        # once compared with the header timestamp.
        if header_ns > 0 and magnitude > 10_000_000:
            return [(value - header_ns) / 1e9 for value in values]
        return [value / 1e9 for value in values]
    # Auto mode also tolerates a scan-relative nanosecond field named 'time'.
    if 10_000_000 < magnitude < 1e15:
        return [value / 1e9 for value in values]
    return values


def inspect_cloud(
    message: Any,
    *,
    max_samples: int = 4096,
    mode: str = "auto",
) -> CloudInspection:
    fields = find_cloud_fields(message)
    point_count = int(message.width) * int(message.height)
    point_step = int(message.point_step)
    payload = memoryview(message.data).cast("B")
    if point_count <= 0:
        raise ValueError("PointCloud2 is empty")
    if point_step <= 0 or len(payload) < point_count * point_step:
        raise ValueError("PointCloud2 payload is shorter than point_count * point_step")

    ring_field = next(name for name in CLOUD_RING_ALIASES if name in fields)
    time_field = next(name for name in CLOUD_TIME_ALIASES if name in fields)
    sample_count = min(point_count, max(1, max_samples))
    if sample_count == 1:
        indices = [0]
    else:
        indices = sorted(
            {
                round(index * (point_count - 1) / (sample_count - 1))
                for index in range(sample_count)
            }
        )
    endian = ">" if bool(message.is_bigendian) else "<"
    raw_times = [
        _value_at(payload, index, point_step, fields[time_field], endian)
        for index in indices
    ]
    offsets = scan_time_offsets_s(
        time_field, raw_times, header_stamp_ns(message), mode=mode
    )
    monotonic = all(left <= right for left, right in zip(offsets, offsets[1:]))
    return CloudInspection(
        field_names=tuple(fields),
        field_datatypes={name: item["datatype"] for name, item in fields.items()},
        ring_field=ring_field,
        time_field=time_field,
        point_count=point_count,
        sampled_points=len(indices),
        scan_time_min_s=min(offsets),
        scan_time_max_s=max(offsets),
        monotonic=monotonic,
    )


def validate_cloud_inspection(
    inspection: CloudInspection,
    *,
    time_min_s: float,
    time_max_s: float,
) -> list[str]:
    failures = []
    for name in CLOUD_XYZI_FIELDS:
        if inspection.field_datatypes.get(name) != 7:
            failures.append(f"cloud: field {name} must be FLOAT32")
    expected_ring = 2 if inspection.ring_field == "line" else 4
    if inspection.field_datatypes.get(inspection.ring_field) != expected_ring:
        width = 8 if expected_ring == 2 else 16
        failures.append(f"cloud: field {inspection.ring_field} must be UINT{width}")
    expected_time = 8 if inspection.time_field == "timestamp" else 7
    if inspection.field_datatypes.get(inspection.time_field) != expected_time:
        kind = "FLOAT64" if expected_time == 8 else "FLOAT32"
        failures.append(f"cloud: field {inspection.time_field} must be {kind}")
    if not inspection.monotonic:
        failures.append("cloud: scan-relative point time is not monotonic")
    if inspection.scan_time_min_s < time_min_s - 1e-6:
        failures.append(
            f"cloud: minimum scan time {inspection.scan_time_min_s:.6f} s "
            f"< {time_min_s:.6f} s"
        )
    if inspection.scan_time_max_s > time_max_s:
        failures.append(
            f"cloud: maximum scan time {inspection.scan_time_max_s:.6f} s "
            f"> {time_max_s:.6f} s"
        )
    if not all(math.isfinite(value) for value in (
        inspection.scan_time_min_s,
        inspection.scan_time_max_s,
    )):
        failures.append("cloud: scan time contains NaN or infinity")
    return failures
