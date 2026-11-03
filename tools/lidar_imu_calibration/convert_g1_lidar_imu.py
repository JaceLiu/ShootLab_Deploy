#!/usr/bin/env python3
"""Convert a native G1 ROS2 MCAP bag to the ROS1 bag consumed by iKalibr."""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from g1_calibration.cloud import find_cloud_fields, header_stamp_ns
from g1_calibration.constants import (
    CLOUD_RING_ALIASES,
    CLOUD_TIME_ALIASES,
    CLOUD_TOPIC,
    DEFAULT_PC_OUTPUT_ROOT,
    IMU_TOPIC,
    LOWSTATE_TOPIC,
    TOPIC_TYPES,
)

IMU_STATE_MSGDEF = """\
float32[4] quaternion
float32[3] gyroscope
float32[3] accelerometer
float32[3] rpy
int16 temperature
"""

NP_DTYPES = {
    1: "i1",
    2: "u1",
    3: "i2",
    4: "u2",
    5: "i4",
    6: "u4",
    7: "f4",
    8: "f8",
}


@dataclass(frozen=True)
class ConvertedCloud:
    width: int
    height: int
    point_step: int
    row_step: int
    is_dense: bool
    data: Any
    fields: list[dict[str, Any]]


def _numpy():
    try:
        import numpy as np
    except ImportError as exc:
        raise RuntimeError("numpy is required for PC conversion") from exc
    return np


def _field_view(matrix: Any, point_step: int, field: dict[str, int], bigendian: bool):
    np = _numpy()
    prefix = ">" if bigendian else "<"
    dtype = np.dtype(prefix + NP_DTYPES[field["datatype"]])
    return np.ndarray(
        shape=(matrix.shape[0],),
        dtype=dtype,
        buffer=matrix,
        offset=field["offset"],
        strides=(point_step,),
    )


def convert_cloud_message(message: Any) -> ConvertedCloud:
    """Map native x/y/z/intensity/tag/line/timestamp to the PC contract."""
    np = _numpy()
    fields = find_cloud_fields(message)
    width = int(message.width)
    height = int(message.height)
    point_count = width * height
    point_step = int(message.point_step)
    if point_count <= 0 or point_step <= 0:
        raise ValueError("PointCloud2 has invalid dimensions")
    payload = np.frombuffer(memoryview(message.data).cast("B"), dtype=np.uint8)
    if payload.size < point_count * point_step:
        raise ValueError("PointCloud2 payload is truncated")
    source = payload[: point_count * point_step].reshape(point_count, point_step)
    ring_field = next(name for name in CLOUD_RING_ALIASES if name in fields)
    time_field = next(name for name in CLOUD_TIME_ALIASES if name in fields)
    bigendian = bool(message.is_bigendian)

    output_step = 24
    output = np.zeros((point_count, output_step), dtype=np.uint8)
    output_fields = [
        {"name": "x", "offset": 0, "datatype": 7, "count": 1},
        {"name": "y", "offset": 4, "datatype": 7, "count": 1},
        {"name": "z", "offset": 8, "datatype": 7, "count": 1},
        {"name": "intensity", "offset": 12, "datatype": 7, "count": 1},
        {"name": "ring", "offset": 16, "datatype": 4, "count": 1},
        {"name": "time", "offset": 20, "datatype": 7, "count": 1},
    ]

    def destination(offset: int, dtype: str):
        return np.ndarray(
            shape=(point_count,),
            dtype=np.dtype("<" + dtype),
            buffer=output,
            offset=offset,
            strides=(output_step,),
        )

    for name, offset, dtype in (
        ("x", 0, "f4"),
        ("y", 4, "f4"),
        ("z", 8, "f4"),
        ("intensity", 12, "f4"),
    ):
        destination(offset, dtype)[:] = _field_view(
            source, point_step, fields[name], bigendian
        ).astype(np.float32, copy=False)
    destination(16, "u2")[:] = _field_view(
        source, point_step, fields[ring_field], bigendian
    ).astype(np.uint16, copy=False)

    raw_times = _field_view(source, point_step, fields[time_field], bigendian).astype(
        np.float64, copy=False
    )
    sample_max = (
        float(np.max(np.abs(raw_times[: min(point_count, 4096)])))
        if point_count
        else 0.0
    )
    if time_field == "timestamp" or sample_max > 10.0:
        if sample_max > 10_000_000:
            time_offsets = (raw_times - float(header_stamp_ns(message))) / 1e9
        else:
            time_offsets = raw_times / 1e9
    else:
        time_offsets = raw_times
    destination(20, "f4")[:] = time_offsets.astype(np.float32)

    time_min = float(np.min(time_offsets))
    time_max = float(np.max(time_offsets))
    if not all(map(math.isfinite, (time_min, time_max))):
        raise ValueError("PointCloud2 scan time contains NaN or infinity")
    if time_min < -1e-6 or time_max > 0.150:
        raise ValueError(
            f"PointCloud2 scan time [{time_min:.6f}, {time_max:.6f}] s is "
            "outside the accepted 0-150 ms interval"
        )
    return ConvertedCloud(
        width=width,
        height=height,
        point_step=output_step,
        row_step=width * output_step,
        is_dense=bool(message.is_dense),
        data=output.reshape(-1).copy(),
        fields=output_fields,
    )


def _stamp_fields(typestore: Any, timestamp_ns: int) -> tuple[Any, Any]:
    time_cls = typestore.types["builtin_interfaces/msg/Time"]
    names = time_cls.__dataclass_fields__
    if "sec" in names and "nanosec" in names:
        stamp = time_cls(sec=timestamp_ns // 1_000_000_000, nanosec=timestamp_ns % 1_000_000_000)
    elif "secs" in names and "nsecs" in names:
        stamp = time_cls(secs=timestamp_ns // 1_000_000_000, nsecs=timestamp_ns % 1_000_000_000)
    else:
        raise RuntimeError("unsupported Time fields in rosbags typestore")
    header_cls = typestore.types["std_msgs/msg/Header"]
    header_kwargs = {"stamp": stamp}
    if "seq" in header_cls.__dataclass_fields__:
        header_kwargs["seq"] = 0
    return header_cls, header_kwargs


def _make_header(typestore: Any, timestamp_ns: int, frame_id: str) -> Any:
    header_cls, kwargs = _stamp_fields(typestore, timestamp_ns)
    kwargs["frame_id"] = frame_id
    return header_cls(**kwargs)


def _make_pointcloud(typestore: Any, source: Any, converted: ConvertedCloud) -> Any:
    np = _numpy()
    field_cls = typestore.types["sensor_msgs/msg/PointField"]
    cloud_cls = typestore.types["sensor_msgs/msg/PointCloud2"]
    fields = [
        field_cls(
            name=item["name"],
            offset=item["offset"],
            datatype=item["datatype"],
            count=item["count"],
        )
        for item in converted.fields
    ]
    header = _make_header(
        typestore, header_stamp_ns(source), str(source.header.frame_id or "mid360")
    )
    kwargs = {
        "header": header,
        "height": converted.height,
        "width": converted.width,
        "fields": fields,
        "is_bigendian": False,
        "point_step": converted.point_step,
        "row_step": converted.row_step,
        "data": np.asarray(converted.data, dtype=np.uint8),
        "is_dense": converted.is_dense,
    }
    allowed = cloud_cls.__dataclass_fields__
    return cloud_cls(**{key: value for key, value in kwargs.items() if key in allowed})


def _make_imu(typestore: Any, source: Any, timestamp_ns: int) -> Any:
    np = _numpy()
    quaternion_cls = typestore.types["geometry_msgs/msg/Quaternion"]
    vector_cls = typestore.types["geometry_msgs/msg/Vector3"]
    imu_cls = typestore.types["sensor_msgs/msg/Imu"]
    qw, qx, qy, qz = (float(value) for value in source.quaternion)
    gx, gy, gz = (float(value) for value in source.gyroscope)
    ax, ay, az = (float(value) for value in source.accelerometer)
    covariance = np.zeros(9, dtype=np.float64)
    return imu_cls(
        header=_make_header(typestore, timestamp_ns, "torso_imu"),
        orientation=quaternion_cls(x=qx, y=qy, z=qz, w=qw),
        orientation_covariance=covariance,
        angular_velocity=vector_cls(x=gx, y=gy, z=gz),
        angular_velocity_covariance=covariance.copy(),
        linear_acceleration=vector_cls(x=ax, y=ay, z=az),
        linear_acceleration_covariance=covariance.copy(),
    )


def _register_unitree_types(reader: Any) -> None:
    try:
        from rosbags.typesys import get_types_from_msg
    except ImportError:
        try:
            from rosbags.typesys.msg import get_types_from_msg
        except ImportError as exc:
            raise RuntimeError(
                "installed rosbags lacks get_types_from_msg; install "
                "rosbags>=0.10,<0.11"
            ) from exc
    for connection in reader.connections:
        if connection.msgtype == "unitree_hg/msg/IMUState":
            reader.typestore.register(
                get_types_from_msg(IMU_STATE_MSGDEF, connection.msgtype)
            )


def convert_bag(input_bag: Path, output_bag: Path, *, overwrite: bool = False) -> None:
    try:
        from rosbags.highlevel import AnyReader
        from rosbags.rosbag1 import Writer
        from rosbags.typesys import Stores, get_typestore
    except ImportError as exc:
        raise RuntimeError(
            "PC conversion requires rosbags and numpy; install "
            "tools/lidar_imu_calibration/requirements-pc.txt"
        ) from exc

    input_bag = input_bag.expanduser().resolve()
    output_bag = output_bag.expanduser().resolve()
    if not input_bag.exists():
        raise FileNotFoundError(input_bag)
    if output_bag.exists():
        if not overwrite:
            raise FileExistsError(
                f"output already exists: {output_bag}; pass --overwrite to replace it"
            )
        if not output_bag.is_file():
            raise IsADirectoryError(f"refusing to remove non-file output: {output_bag}")
        output_bag.unlink()
    output_bag.parent.mkdir(parents=True, exist_ok=True)

    output_typestore = get_typestore(Stores.ROS1_NOETIC)
    with AnyReader([input_bag]) as reader:
        _register_unitree_types(reader)
        topic_types = {connection.topic: connection.msgtype for connection in reader.connections}
        for topic, expected in TOPIC_TYPES.items():
            if topic not in topic_types:
                raise RuntimeError(f"required input topic is absent: {topic}")
            if topic_types[topic] != expected:
                raise RuntimeError(
                    f"{topic}: type {topic_types[topic]}, expected {expected}"
                )
        cloud_connections = [
            connection for connection in reader.connections if connection.topic == CLOUD_TOPIC
        ]
        imu_connections = [
            connection for connection in reader.connections if connection.topic == IMU_TOPIC
        ]
        output_connections = cloud_connections + imu_connections
        with Writer(output_bag) as writer:
            cloud_output = writer.add_connection(
                "/g1/mid360",
                "sensor_msgs/msg/PointCloud2",
                typestore=output_typestore,
            )
            imu_output = writer.add_connection(
                "/g1/torso_imu",
                "sensor_msgs/msg/Imu",
                typestore=output_typestore,
            )
            # Merge both inputs chronologically. Keeping a globally ordered ROS1
            # bag is friendlier to third-party readers than concatenating topics.
            for connection, timestamp, raw in reader.messages(
                connections=output_connections
            ):
                source = reader.deserialize(raw, connection.msgtype)
                if connection.topic == CLOUD_TOPIC:
                    converted = convert_cloud_message(source)
                    message = _make_pointcloud(output_typestore, source, converted)
                    writer.write(
                        cloud_output,
                        timestamp,
                        output_typestore.serialize_ros1(
                            message, "sensor_msgs/msg/PointCloud2"
                        ),
                    )
                else:
                    message = _make_imu(output_typestore, source, timestamp)
                    writer.write(
                        imu_output,
                        timestamp,
                        output_typestore.serialize_ros1(
                            message, "sensor_msgs/msg/Imu"
                        ),
                    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("input_bag", type=Path)
    parser.add_argument(
        "--output",
        type=Path,
        help="Output .bag path (default: calibration_data_ros1/<capture>.bag)",
    )
    parser.add_argument("--overwrite", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    input_bag = args.input_bag.expanduser().resolve()
    output = args.output
    if output is None:
        output = DEFAULT_PC_OUTPUT_ROOT / f"{input_bag.name}_ros1.bag"
    try:
        convert_bag(input_bag, output, overwrite=args.overwrite)
    except Exception as exc:
        print(f"CONVERSION ERROR: {exc}", file=sys.stderr)
        return 1
    print(f"Wrote ROS1 bag: {output.expanduser().resolve()}")
    print("Verify with: rosbag info " + str(output.expanduser().resolve()))
    print("Expected topics: /g1/mid360 and /g1/torso_imu")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
