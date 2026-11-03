#!/usr/bin/env python3
"""Capture G1 native MID360, torso IMU, lowstate, and Livox IMU to MCAP."""

from __future__ import annotations

import argparse
import hashlib
import os
import re
import shlex
import shutil
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

from g1_calibration.constants import (
    CAPTURE_PROFILES,
    CLOUD_TOPIC,
    CONFLICT_PROCESS_PATTERNS,
    DEFAULT_DATA_ROOT,
    IMU_TOPIC,
    LIVOX_IMU_TOPIC,
    LOWSTATE_TOPIC,
    TOPIC_TYPES,
)
from g1_calibration.manifest import (
    build_manifest,
    load_yaml,
    utc_now,
    validate_capture_id,
    write_yaml,
)

ALL_TOPICS = (CLOUD_TOPIC, IMU_TOPIC, LOWSTATE_TOPIC, LIVOX_IMU_TOPIC)


def default_capture_id() -> str:
    return datetime.now().astimezone().strftime("g1_mid360_%Y%m%d_%H%M%S")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Record the four native G1 ROS2 topics required for MID360-to-torso "
            "IMU calibration. The bag is stored outside the Git worktree."
        )
    )
    parser.add_argument("--profile", choices=tuple(CAPTURE_PROFILES), default="smoke")
    parser.add_argument("--capture-id", default=default_capture_id())
    parser.add_argument(
        "--duration-s",
        type=float,
        help="Override the profile duration; 0 records until Ctrl-C",
    )
    parser.add_argument("--data-root", type=Path, default=DEFAULT_DATA_ROOT)
    parser.add_argument(
        "--setup-script",
        default=os.environ.get("UNITREE_SETUP", "/opt/unitree_ros2/setup.sh"),
        help="ROS2/Unitree environment setup script",
    )
    parser.add_argument(
        "--no-setup",
        action="store_true",
        help="Use the current shell environment instead of sourcing --setup-script",
    )
    parser.add_argument("--preflight-seconds", type=float, default=5.0)
    parser.add_argument(
        "--skip-preflight",
        action="store_true",
        help="Emergency override; formal captures must not use this",
    )
    parser.add_argument(
        "--stop-conflicts",
        action="store_true",
        help="Send SIGINT/SIGTERM to detected policy, ball, and extra Livox processes",
    )
    parser.add_argument("--no-analyze", action="store_true")
    parser.add_argument("--robot-model", default="Unitree G1")
    parser.add_argument("--sensor", default="Livox MID360")
    parser.add_argument("--scenario", default="stationary indoor structures at 1-10 m")
    parser.add_argument("--operator", default=os.environ.get("USER", "unknown"))
    parser.add_argument("--notes", default="")
    parser.add_argument(
        "--python",
        default=os.environ.get(
            "G1_CALIB_PYTHON",
            (
                "/home/unitree/miniconda3/envs/robomimic/bin/python"
                if Path("/home/unitree/miniconda3/envs/robomimic/bin/python").exists()
                else sys.executable
            ),
        ),
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print the resolved record command without touching ROS",
    )
    args = parser.parse_args()
    if args.duration_s is None:
        args.duration_s = float(CAPTURE_PROFILES[args.profile]["duration_s"])
    if args.duration_s < 0:
        parser.error("--duration-s must be >= 0")
    if args.preflight_seconds <= 0:
        parser.error("--preflight-seconds must be positive")
    validate_capture_id(args.capture_id)
    return args


def load_ros_environment(setup_script: Path) -> dict[str, str]:
    if not setup_script.is_file():
        raise RuntimeError(f"environment setup script not found: {setup_script}")
    script = "\n".join(
        (
            "set -e",
            f"source {shlex.quote(str(setup_script))}",
            "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp",
            "export ROS_LOCALHOST_ONLY=0",
            "exec env -0",
        )
    )
    result = subprocess.run(
        ["bash", "-c", script],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    if result.returncode != 0:
        detail = result.stderr.decode("utf-8", errors="replace").strip()
        raise RuntimeError(f"failed to source {setup_script}: {detail}")
    environment = {}
    for entry in result.stdout.split(b"\0"):
        if not entry or b"=" not in entry:
            continue
        key, value = entry.split(b"=", 1)
        environment[key.decode()] = value.decode(errors="surrogateescape")
    return environment


def detect_conflicts() -> list[tuple[int, str]]:
    conflicts = []
    own_pid = os.getpid()
    for proc_dir in Path("/proc").iterdir():
        if not proc_dir.name.isdigit():
            continue
        pid = int(proc_dir.name)
        if pid == own_pid:
            continue
        try:
            command = (
                (proc_dir / "cmdline")
                .read_bytes()
                .replace(b"\0", b" ")
                .decode(errors="replace")
                .strip()
            )
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        if any(pattern in command for pattern in CONFLICT_PROCESS_PATTERNS):
            conflicts.append((pid, command))
    return sorted(conflicts)


def stop_conflicts(conflicts: list[tuple[int, str]]) -> None:
    for pid, command in conflicts:
        print(f"Stopping conflicting process {pid}: {command}", flush=True)
        try:
            os.kill(pid, signal.SIGINT)
        except ProcessLookupError:
            continue
    deadline = time.monotonic() + 10.0
    remaining = [pid for pid, _ in conflicts]
    while remaining and time.monotonic() < deadline:
        remaining = [
            pid for pid in remaining if Path(f"/proc/{pid}").exists()
        ]
        time.sleep(0.2)
    for pid in remaining:
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
    if remaining:
        time.sleep(1.0)


def check_topic_types(environment: dict[str, str]) -> list[str]:
    try:
        result = subprocess.run(
            ["ros2", "topic", "list", "-t"],
            env=environment,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=20,
            check=False,
            text=True,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired) as exc:
        return [f"ros2 topic list failed: {exc}"]
    if result.returncode != 0:
        return [f"ros2 topic list failed: {result.stderr.strip()}"]

    observed = {}
    for line in result.stdout.splitlines():
        match = re.match(r"^\s*(\S+)\s+\[(.+)\]\s*$", line)
        if match:
            observed[match.group(1)] = match.group(2)
    failures = []
    for topic, expected in TOPIC_TYPES.items():
        actual = observed.get(topic)
        if actual is None:
            failures.append(f"{topic}: not discovered")
        elif actual != expected:
            failures.append(f"{topic}: type {actual}, expected {expected}")
    return failures


def probe_live_topics(
    environment: dict[str, str], python_bin: str, seconds: float
) -> tuple[dict[str, dict[str, float | int]], list[str]]:
    probe_script = Path(__file__).with_name("probe_topics.py")
    try:
        result = subprocess.run(
            [
                python_bin,
                str(probe_script),
                "--seconds",
                str(seconds),
            ],
            env=environment,
            cwd=Path(__file__).parent,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=seconds + 15,
            check=False,
            text=True,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired) as exc:
        return {}, [f"topic probe failed: {exc}"]
    if result.returncode != 0:
        return {}, [
            f"topic probe failed ({result.returncode}): "
            f"{result.stderr.strip() or result.stdout.strip()}"
        ]
    json_lines = [line for line in result.stdout.splitlines() if line.startswith("{")]
    if not json_lines:
        return {}, [f"topic probe produced no report: {result.stdout.strip()}"]
    import json

    report = json.loads(json_lines[-1])
    failures = [
        f"{topic}: no live messages"
        for topic in ALL_TOPICS
        if int(report.get(topic, {}).get("count", 0)) < 2
    ]
    return report, failures


def find_mcap_files(bag_dir: Path) -> list[Path]:
    return sorted(bag_dir.glob("*.mcap"))


def update_files(manifest: dict[str, object], bag_dir: Path) -> None:
    files = {}
    for path in (
        *find_mcap_files(bag_dir),
        bag_dir / "metadata.yaml",
    ):
        if not path.is_file():
            continue
        entry = {"bytes": path.stat().st_size}
        if path.suffix == ".mcap":
            digest = hashlib.sha256()
            with path.open("rb") as stream:
                for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                    digest.update(chunk)
            entry["sha256"] = digest.hexdigest()
        files[path.name] = entry
    manifest["files"] = files


def write_manifest(path: Path, manifest: dict[str, object]) -> None:
    manifest["updated_at_utc"] = utc_now()
    write_yaml(path, manifest)


def main() -> int:
    args = parse_args()
    profile = CAPTURE_PROFILES[args.profile]
    capture_dir = args.data_root.expanduser().resolve() / args.capture_id
    manifest_path = capture_dir / "capture_manifest.yaml"
    record_command = [
        "ros2",
        "bag",
        "record",
        "-s",
        "mcap",
        "-o",
        str(capture_dir),
        *ALL_TOPICS,
    ]
    if args.dry_run:
        print(
            "Preflight:",
            f"{sys.executable} {Path(__file__).with_name('probe_topics.py')} "
            f"--seconds {args.preflight_seconds:g}",
        )
        print("Record:", shlex.join(record_command))
        print("Output:", capture_dir)
        return 0

    environment = os.environ.copy()
    if not args.no_setup:
        environment = load_ros_environment(Path(args.setup_script).expanduser())
    ros_log_dir = Path(
        environment.get("ROS_LOG_DIR", "/tmp/g1_lidar_imu_ros_logs")
    ).expanduser()
    try:
        ros_log_dir.mkdir(parents=True, exist_ok=True)
        environment["ROS_LOG_DIR"] = str(ros_log_dir)
    except OSError as exc:
        print(f"ERROR: cannot create ROS log directory: {exc}", file=sys.stderr)
        return 2
    ros2 = shutil.which("ros2", path=environment.get("PATH"))
    if ros2 is None:
        print("ERROR: ros2 is not available after environment setup", file=sys.stderr)
        return 2
    record_command[0] = ros2

    conflicts = detect_conflicts()
    if conflicts:
        for pid, command in conflicts:
            print(f"CONFLICT pid={pid}: {command}", file=sys.stderr)
        if not args.stop_conflicts:
            print(
                "ERROR: stop policy, ball perception, and any extra Livox driver "
                "before capture, or rerun with --stop-conflicts",
                file=sys.stderr,
            )
            return 2
        stop_conflicts(conflicts)
        conflicts = detect_conflicts()
        if conflicts:
            print("ERROR: conflicting processes are still running", file=sys.stderr)
            return 2

    if not args.skip_preflight:
        failures = check_topic_types(environment)
        if not failures:
            probe, failures = probe_live_topics(
                environment, args.python, args.preflight_seconds
            )
            if probe:
                for topic in ALL_TOPICS:
                    values = probe.get(topic, {})
                    print(
                        f"Preflight {topic}: {values.get('count', 0)} messages, "
                        f"{float(values.get('mean_hz', 0.0)):.1f} Hz "
                        "(probe callback rate; bag analysis is authoritative)",
                        flush=True,
                    )
        if failures:
            for failure in failures:
                print(f"PREFLIGHT ERROR: {failure}", file=sys.stderr)
            return 2

    if capture_dir.exists():
        print(f"ERROR: capture directory already exists: {capture_dir}", file=sys.stderr)
        return 2
    args.data_root.expanduser().resolve().mkdir(parents=True, exist_ok=True)

    manifest = build_manifest(
        capture_id=args.capture_id,
        profile=args.profile,
        duration_s=args.duration_s,
        robot_model=args.robot_model,
        sensor=args.sensor,
        scenario=args.scenario,
        motion=str(profile["motion"]),
        operator=args.operator,
        notes=args.notes,
        topics=TOPIC_TYPES,
    )
    manifest["requested_options"] = {
        "stop_conflicts": args.stop_conflicts,
        "skip_preflight": args.skip_preflight,
        "preflight_seconds": args.preflight_seconds,
    }

    print(f"Capture directory: {capture_dir}", flush=True)
    print("Starting MCAP recording...", flush=True)
    if args.profile in {"A", "B"}:
        print(
            "Keep the first 10 s static, then execute the planned slow waist "
            "motion and modest translation. Finish with 10 s static.",
            flush=True,
        )

    stop_signal_received = False

    def handle_stop(_signum: int, _frame: object) -> None:
        nonlocal stop_signal_received
        stop_signal_received = True

    previous_sigint = signal.signal(signal.SIGINT, handle_stop)
    previous_sigterm = signal.signal(signal.SIGTERM, handle_stop)
    process = subprocess.Popen(
        record_command,
        env=environment,
        start_new_session=True,
    )
    started = time.monotonic()
    stop_sent = False
    stop_started = 0.0
    terminate_sent = False
    last_update = started
    try:
        while process.poll() is None:
            now = time.monotonic()
            elapsed = now - started
            if (
                args.duration_s > 0 and elapsed >= args.duration_s
            ) or stop_signal_received:
                if not stop_sent:
                    print("\nStopping recorder with SIGINT...", flush=True)
                    process.send_signal(signal.SIGINT)
                    stop_sent = True
                    stop_started = now
            if stop_sent and now - stop_started > 20.0 and not terminate_sent:
                print(
                    "Recorder did not stop after 20 s; sending SIGTERM.",
                    file=sys.stderr,
                    flush=True,
                )
                process.terminate()
                terminate_sent = True
            if stop_sent and now - stop_started > 25.0:
                print(
                    "Recorder did not stop after SIGTERM; killing it.",
                    file=sys.stderr,
                    flush=True,
                )
                process.kill()
            if now - last_update >= 10.0:
                print(f"Recording: {elapsed:.1f} s", flush=True)
                last_update = now
            time.sleep(0.2)
    finally:
        signal.signal(signal.SIGINT, previous_sigint)
        signal.signal(signal.SIGTERM, previous_sigterm)
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5)

    record_exit_code = int(process.returncode or 0)
    if not capture_dir.exists():
        capture_dir.mkdir(parents=True)
    mcap_files = find_mcap_files(capture_dir)
    metadata_path = capture_dir / "metadata.yaml"
    manifest["record_exit_code"] = record_exit_code
    manifest["actual_recording_wall_s"] = time.monotonic() - started
    update_files(manifest, capture_dir)
    failed = (
        not mcap_files
        or not metadata_path.is_file()
        or not any(path.stat().st_size > 0 for path in mcap_files)
    )
    manifest["status"] = "recording_failed" if failed else "recorded_pending_validation"
    write_manifest(manifest_path, manifest)
    print(f"Recorder exit code: {record_exit_code}", flush=True)
    if failed:
        print(
            f"ERROR: expected MCAP and metadata.yaml in {capture_dir}",
            file=sys.stderr,
        )
        return 1

    if not args.no_analyze:
        analyzer = Path(__file__).with_name("analyze_g1_lidar_imu.py")
        result = subprocess.run(
            [
                args.python,
                str(analyzer),
                str(capture_dir),
                "--profile",
                args.profile,
            ],
            env=environment,
            cwd=Path(__file__).parent,
            check=False,
        )
        if result.returncode != 0:
            print(
                f"Validation failed; see {capture_dir / 'analysis.yaml'}",
                file=sys.stderr,
            )
            return result.returncode

    print(f"Capture complete: {capture_dir}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
