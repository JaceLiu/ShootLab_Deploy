"""Timestamp, rate, coverage, and motion metrics used by the bag analyzer."""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from typing import Iterable, Sequence


def percentile(values: Sequence[float], fraction: float) -> float:
    """Return a linearly interpolated percentile for an unsorted sequence."""
    if not values:
        raise ValueError("percentile requires at least one value")
    if not 0.0 <= fraction <= 1.0:
        raise ValueError("percentile fraction must be in [0, 1]")
    ordered = sorted(float(value) for value in values)
    position = fraction * (len(ordered) - 1)
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


@dataclass(frozen=True)
class RateMetrics:
    count: int
    duration_s: float
    mean_hz: float | None
    p95_interval_s: float | None
    p99_interval_s: float | None
    max_interval_s: float | None

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


def summarize_timestamps(timestamps_ns: Sequence[int]) -> RateMetrics:
    """Summarize a stream from message timestamps in nanoseconds."""
    if not timestamps_ns:
        return RateMetrics(0, 0.0, None, None, None, None)

    ordered = sorted(int(value) for value in timestamps_ns)
    count = len(ordered)
    duration_s = (ordered[-1] - ordered[0]) / 1e9 if count > 1 else 0.0
    intervals = [
        (right - left) / 1e9 for left, right in zip(ordered, ordered[1:])
    ]
    mean_hz = (count - 1) / duration_s if duration_s > 0.0 else None
    return RateMetrics(
        count=count,
        duration_s=duration_s,
        mean_hz=mean_hz,
        p95_interval_s=percentile(intervals, 0.95) if intervals else None,
        p99_interval_s=percentile(intervals, 0.99) if intervals else None,
        max_interval_s=max(intervals) if intervals else None,
    )


def rate_failures(
    stream: str,
    metrics: RateMetrics,
    *,
    min_mean_hz: float | None = None,
    max_interval_s: float | None = None,
    max_percentile: tuple[float, float] | None = None,
) -> list[str]:
    """Return human-readable violations for one stream."""
    failures: list[str] = []
    if metrics.count < 2 or metrics.mean_hz is None:
        return [f"{stream}: fewer than two messages"]
    if min_mean_hz is not None and metrics.mean_hz < min_mean_hz:
        failures.append(
            f"{stream}: mean rate {metrics.mean_hz:.3f} Hz < {min_mean_hz:.3f} Hz"
        )
    if max_interval_s is not None and (
        metrics.max_interval_s is None or metrics.max_interval_s > max_interval_s
    ):
        failures.append(
            f"{stream}: max interval {metrics.max_interval_s:.6f} s "
            f"> {max_interval_s:.6f} s"
        )
    if max_percentile is not None:
        fraction, limit = max_percentile
        observed = (
            metrics.p95_interval_s if fraction == 0.95 else metrics.p99_interval_s
        )
        if observed is None or observed > limit:
            failures.append(
                f"{stream}: p{int(fraction * 100)} interval {observed:.6f} s "
                f"> {limit:.6f} s"
            )
    return failures


def coverage_report(
    streams: dict[str, Sequence[int]],
    *,
    edge_slack_s: float,
    min_fraction: float,
) -> tuple[dict[str, object], list[str]]:
    """Check that all required streams span the complete capture."""
    nonempty = {name: values for name, values in streams.items() if values}
    failures = [
        f"{name}: no messages" for name, values in streams.items() if not values
    ]
    if not nonempty:
        return {}, failures

    starts = {name: min(values) for name, values in nonempty.items()}
    ends = {name: max(values) for name, values in nonempty.items()}
    bag_start = min(starts.values())
    bag_end = max(ends.values())
    common_start = max(starts.values())
    common_end = min(ends.values())
    bag_duration = (bag_end - bag_start) / 1e9
    common_duration = max(0, common_end - common_start) / 1e9
    fraction = common_duration / bag_duration if bag_duration > 0 else 0.0
    detail: dict[str, object] = {
        "bag_start_ns": bag_start,
        "bag_end_ns": bag_end,
        "bag_duration_s": bag_duration,
        "common_duration_s": common_duration,
        "coverage_fraction": fraction,
        "streams": {
            name: {
                "start_ns": starts[name],
                "end_ns": ends[name],
                "start_gap_s": (starts[name] - bag_start) / 1e9,
                "end_gap_s": (bag_end - ends[name]) / 1e9,
            }
            for name in nonempty
        },
    }

    if bag_duration == 0:
        failures.append("capture has zero duration")
        return detail, failures
    if fraction < min_fraction:
        failures.append(
            f"coverage: common span fraction {fraction:.4f} < {min_fraction:.4f}"
        )
    for name, start in starts.items():
        start_gap_s = (start - bag_start) / 1e9
        end_gap_s = (bag_end - ends[name]) / 1e9
        if start_gap_s > edge_slack_s:
            failures.append(
                f"{name}: missing first {start_gap_s:.3f} s "
                f"> {edge_slack_s:.3f} s"
            )
        if end_gap_s > edge_slack_s:
            failures.append(
                f"{name}: missing final {end_gap_s:.3f} s "
                f"> {edge_slack_s:.3f} s"
            )
    return detail, failures


def axis_span(values: Sequence[float], low: float = 0.05, high: float = 0.95) -> float:
    if not values:
        return 0.0
    return percentile(values, high) - percentile(values, low)


def motion_report(
    imu_timestamps_ns: Sequence[int],
    gyro_xyz: Iterable[Sequence[float]],
    accel_xyz: Iterable[Sequence[float]],
    *,
    static_window_s: float,
    axis_gyro_span_rad_s: float,
    static_max_gyro_span_rad_s: float,
    motion_to_static_ratio: float,
    motion_accel_span_m_s2: float,
) -> tuple[dict[str, object], list[str]]:
    """Measure three-axis excitation and distinguishable static windows."""
    gyro = [list(values) for values in gyro_xyz]
    accel = [list(values) for values in accel_xyz]
    if len(gyro) != 3 or len(accel) != 3:
        return {}, ["motion: expected three gyro and three accel axes"]
    lengths = [
        len(imu_timestamps_ns),
        *(len(values) for values in gyro),
        *(len(values) for values in accel),
    ]
    if not lengths or min(lengths) == 0 or len(set(lengths)) != 1:
        return {}, ["motion: IMU timestamp and axis lengths are inconsistent"]

    start_ns = min(imu_timestamps_ns)
    end_ns = max(imu_timestamps_ns)
    static_start_limit = start_ns + static_window_s * 1e9
    static_end_limit = end_ns - static_window_s * 1e9
    first_indices = [
        index
        for index, timestamp in enumerate(imu_timestamps_ns)
        if timestamp <= static_start_limit
    ]
    last_indices = [
        index
        for index, timestamp in enumerate(imu_timestamps_ns)
        if timestamp >= static_end_limit
    ]
    has_static_windows = bool(first_indices and last_indices)

    def select(values: Sequence[float], indices: Sequence[int]) -> list[float]:
        return [values[index] for index in indices]

    gyro_spans = [axis_span(values) for values in gyro]
    accel_spans = [axis_span(values) for values in accel]
    first_static_spans = (
        [axis_span(select(values, first_indices)) for values in gyro]
        if has_static_windows
        else [0.0, 0.0, 0.0]
    )
    last_static_spans = (
        [axis_span(select(values, last_indices)) for values in gyro]
        if has_static_windows
        else [0.0, 0.0, 0.0]
    )
    static_spans = [
        max(first, last)
        for first, last in zip(first_static_spans, last_static_spans)
    ]
    ratios = []
    for motion_value, static_value in zip(gyro_spans, static_spans):
        if static_value > 1e-6:
            ratios.append(motion_value / static_value)
        elif motion_value >= axis_gyro_span_rad_s:
            ratios.append(math.inf)
        else:
            ratios.append(0.0)

    detail: dict[str, object] = {
        "has_static_windows": has_static_windows,
        "gyro_span_rad_s": dict(zip(("x", "y", "z"), gyro_spans)),
        "accel_span_m_s2": dict(zip(("x", "y", "z"), accel_spans)),
        "first_static_gyro_span_rad_s": dict(
            zip(("x", "y", "z"), first_static_spans)
        ),
        "last_static_gyro_span_rad_s": dict(
            zip(("x", "y", "z"), last_static_spans)
        ),
        "motion_to_static_ratio": dict(
            zip(("x", "y", "z"), [None if math.isinf(x) else x for x in ratios])
        ),
    }
    failures = []
    if not has_static_windows:
        failures.append(
            f"motion: capture is too short for {static_window_s:.1f} s "
            "start/end static windows"
        )
        return detail, failures
    for axis, span, static_span, ratio in zip(
        "xyz", gyro_spans, static_spans, ratios
    ):
        if span < axis_gyro_span_rad_s:
            failures.append(
                f"motion: gyro {axis} span {span:.3f} rad/s "
                f"< {axis_gyro_span_rad_s:.3f}"
            )
        if static_span > static_max_gyro_span_rad_s:
            failures.append(
                f"motion: gyro {axis} static span {static_span:.3f} rad/s "
                f"> {static_max_gyro_span_rad_s:.3f}"
            )
        if ratio < motion_to_static_ratio:
            failures.append(
                f"motion: gyro {axis} motion/static ratio {ratio:.3f} "
                f"< {motion_to_static_ratio:.3f}"
            )
    if max(accel_spans) < motion_accel_span_m_s2:
        failures.append(
            f"motion: maximum accel span {max(accel_spans):.3f} m/s^2 "
            f"< {motion_accel_span_m_s2:.3f}"
        )
    return detail, failures
