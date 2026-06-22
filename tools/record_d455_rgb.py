#!/usr/bin/env python3
"""Record RGB video from an Intel RealSense D455 at the highest resolution.

This script enables only the color stream, automatically selects the highest
available color profile, and writes frames to an MP4 file until interrupted.

Examples:
    python tools/record_d455_rgb.py
    python tools/record_d455_rgb.py --output logs/d455_rgb.mp4
    python tools/record_d455_rgb.py --serial 123456789 --duration 10
    python tools/record_d455_rgb.py --no-preview
    python tools/record_d455_rgb.py --list-cameras
    python tools/record_d455_rgb.py --list-profiles
"""

from __future__ import annotations

import argparse
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs


@dataclass(frozen=True)
class ColorProfile:
    width: int
    height: int
    fps: int
    fmt: rs.format

    @property
    def pixels(self) -> int:
        return self.width * self.height

    @property
    def fmt_name(self) -> str:
        return str(self.fmt).split(".")[-1]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Record RGB video from a RealSense D455 at maximum resolution."
        )
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Output video path (default: logs/d455_rgb_<timestamp>.mp4)",
    )
    parser.add_argument(
        "--serial",
        default=None,
        help="RealSense serial number. Defaults to the first connected D455.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help=(
            "Optional recording duration in seconds. "
            "Defaults to until Ctrl+C."
        ),
    )
    parser.add_argument(
        "--fps",
        type=int,
        default=None,
        help=(
            "Override FPS. Resolution still stays at the maximum supported size."
        ),
    )
    preview_group = parser.add_mutually_exclusive_group()
    preview_group.add_argument(
        "--preview",
        dest="preview",
        action="store_true",
        help="Show a live preview window. Enabled by default.",
    )
    preview_group.add_argument(
        "--no-preview",
        dest="preview",
        action="store_false",
        help="Disable the live preview window.",
    )
    parser.set_defaults(preview=True)
    parser.add_argument(
        "--list-cameras",
        action="store_true",
        help="List connected RealSense devices and exit.",
    )
    parser.add_argument(
        "--list-profiles",
        action="store_true",
        help=(
            "List supported color profiles for the selected camera and exit."
        ),
    )
    return parser.parse_args()


def query_devices(ctx: rs.context) -> list[rs.device]:
    return list(ctx.query_devices())


def device_name(device: rs.device) -> str:
    return device.get_info(rs.camera_info.name)


def device_serial(device: rs.device) -> str:
    return device.get_info(rs.camera_info.serial_number)


def print_devices(devices: list[rs.device]) -> None:
    if not devices:
        print("No RealSense devices found.")
        return

    print("Connected RealSense devices:")
    for idx, dev in enumerate(devices):
        print(
            f"  [{idx}] serial={device_serial(dev)}"
            f"  name={device_name(dev)}"
        )


def select_device(devices: list[rs.device], serial: str | None) -> rs.device:
    if not devices:
        raise RuntimeError("No RealSense device found.")

    if serial is not None:
        for dev in devices:
            if device_serial(dev) == serial:
                return dev
        raise RuntimeError(f"RealSense serial {serial} not found.")

    for dev in devices:
        if "D455" in device_name(dev):
            return dev
    return devices[0]


def list_color_profiles(device: rs.device) -> list[ColorProfile]:
    profiles: dict[tuple[int, int, int, int], ColorProfile] = {}
    valid_formats = {rs.format.bgr8, rs.format.rgb8}

    for sensor in device.query_sensors():
        for profile in sensor.get_stream_profiles():
            try:
                video_profile = profile.as_video_stream_profile()
            except RuntimeError:
                continue

            if video_profile.stream_type() != rs.stream.color:
                continue

            fmt = video_profile.format()
            if fmt not in valid_formats:
                continue

            candidate = ColorProfile(
                width=video_profile.width(),
                height=video_profile.height(),
                fps=video_profile.fps(),
                fmt=fmt,
            )
            key = (
                candidate.width,
                candidate.height,
                candidate.fps,
                candidate.fmt_name,
            )
            profiles[key] = candidate

    return sorted(
        profiles.values(),
        key=lambda p: (
            -p.pixels,
            -p.width,
            -p.height,
            _format_rank(p.fmt),
            -p.fps,
        ),
    )


def _format_rank(fmt: rs.format) -> int:
    if fmt == rs.format.bgr8:
        return 0
    if fmt == rs.format.rgb8:
        return 1
    return 99


def print_profiles(profiles: list[ColorProfile]) -> None:
    if not profiles:
        print("No compatible color profiles found.")
        return

    print("Supported color profiles:")
    for profile in profiles:
        print(
            f"  {profile.width}x{profile.height} @ {profile.fps} FPS"
            f"  format={profile.fmt_name}"
        )


def select_profile(
    profiles: list[ColorProfile],
    fps_override: int | None,
) -> ColorProfile:
    if not profiles:
        raise RuntimeError("No compatible RealSense color profile found.")

    max_pixels = max(profile.pixels for profile in profiles)
    highest_res_profiles = [p for p in profiles if p.pixels == max_pixels]

    if fps_override is not None:
        same_fps = [p for p in highest_res_profiles if p.fps == fps_override]
        if not same_fps:
            supported = sorted(
                {p.fps for p in highest_res_profiles},
                reverse=True,
            )
            raise RuntimeError(
                "Requested FPS is not supported at the highest resolution. "
                f"Available FPS for {highest_res_profiles[0].width}x"
                f"{highest_res_profiles[0].height}: {supported}"
            )
        highest_res_profiles = same_fps

    return sorted(
        highest_res_profiles,
        key=lambda p: (_format_rank(p.fmt), -p.fps),
    )[0]


def default_output_path() -> Path:
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    return Path("logs") / f"d455_rgb_{timestamp}.mp4"


def ensure_output_path(path: Path) -> Path:
    resolved = path.expanduser().resolve()
    resolved.parent.mkdir(parents=True, exist_ok=True)
    return resolved


def open_writer(
    output_path: Path,
    width: int,
    height: int,
    fps: int,
) -> cv2.VideoWriter:
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(str(output_path), fourcc, fps, (width, height))
    if not writer.isOpened():
        raise RuntimeError(f"Could not open VideoWriter for {output_path}")
    return writer


def frame_to_bgr(image: np.ndarray, fmt: rs.format) -> np.ndarray:
    if fmt == rs.format.rgb8:
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    return image


def record_video(
    serial: str,
    profile: ColorProfile,
    output_path: Path,
    duration: float | None,
    preview: bool,
) -> None:
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(serial)
    config.enable_stream(
        rs.stream.color,
        profile.width,
        profile.height,
        profile.fmt,
        profile.fps,
    )

    writer = None
    pipeline_started = False
    recorded_frames = 0
    start_time = None
    last_log_time = 0.0

    try:
        pipeline.start(config)
        pipeline_started = True
        print(
            "Recording started:"
            f" serial={serial}, resolution={profile.width}x{profile.height},"
            f" fps={profile.fps}, format={profile.fmt_name}"
        )
        print(f"Saving to: {output_path}")
        if preview:
            print(
                "Only the RGB/color stream is enabled. "
                "Press q in the preview window or Ctrl+C to stop."
            )
        else:
            print("Only the RGB/color stream is enabled. Press Ctrl+C to stop.")

        while True:
            frames = pipeline.wait_for_frames(timeout_ms=5000)
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            image = np.asanyarray(color_frame.get_data())
            bgr_image = frame_to_bgr(image, profile.fmt)

            if writer is None:
                writer = open_writer(
                    output_path,
                    bgr_image.shape[1],
                    bgr_image.shape[0],
                    profile.fps,
                )
                start_time = time.monotonic()

            writer.write(bgr_image)
            recorded_frames += 1

            if preview:
                cv2.imshow("D455 RGB Recorder", bgr_image)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    print("Stopping because preview window received 'q'.")
                    break

            now = time.monotonic()
            if now - last_log_time >= 1.0 and start_time is not None:
                elapsed = now - start_time
                avg_fps = recorded_frames / max(elapsed, 1e-6)
                print(
                    f"\rRecorded {recorded_frames} frames"
                    f" ({elapsed:.1f}s, avg {avg_fps:.1f} FPS)",
                    end="",
                    flush=True,
                )
                last_log_time = now

            if (
                duration is not None
                and start_time is not None
                and (now - start_time) >= duration
            ):
                print(f"\nReached requested duration: {duration:.1f}s")
                break

        if recorded_frames == 0:
            raise RuntimeError("No frames were recorded.")
        print()
    finally:
        if writer is not None:
            writer.release()
        if pipeline_started:
            pipeline.stop()
        if preview:
            cv2.destroyAllWindows()

    print(f"Saved {recorded_frames} frames to {output_path}")


def main() -> None:
    args = parse_args()

    ctx = rs.context()
    devices = query_devices(ctx)
    print_devices(devices)

    if args.list_cameras:
        return

    device = select_device(devices, args.serial)
    serial = device_serial(device)
    name = device_name(device)
    print(f"Selected camera: serial={serial}  name={name}")

    profiles = list_color_profiles(device)
    if args.list_profiles:
        print_profiles(profiles)
        return

    chosen_profile = select_profile(profiles, args.fps)
    print(
        "Chosen profile:"
        f" {chosen_profile.width}x{chosen_profile.height}"
        f" @ {chosen_profile.fps} FPS"
        f" format={chosen_profile.fmt_name}"
    )

    output_path = ensure_output_path(args.output or default_output_path())
    record_video(
        serial=serial,
        profile=chosen_profile,
        output_path=output_path,
        duration=args.duration,
        preview=args.preview,
    )


if __name__ == "__main__":
    main()
