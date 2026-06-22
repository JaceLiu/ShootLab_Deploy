"""Lightweight Unitree DDS lowstate waist-joint listener."""

from __future__ import annotations

import os
import multiprocessing as mp
import queue
import threading
import time
from pathlib import Path

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_ as LowStateHG


def read_real_config_net() -> str | None:
    """Read the configured Unitree network interface from deploy_real config."""
    real_cfg_path = (
        Path(__file__).parent.parent.parent / "deploy_real" / "config" / "real.yaml"
    )
    try:
        with open(real_cfg_path, "r", encoding="utf-8") as f:
            for line in f:
                stripped = line.strip()
                if stripped.startswith("net:"):
                    return stripped.split(":", 1)[1].strip().strip("\"'")
    except OSError:
        return None
    return None


def available_net_interfaces() -> set[str]:
    try:
        return {name for name in os.listdir("/sys/class/net") if name != "lo"}
    except OSError:
        return set()


def select_lowstate_net(config_net: str | None) -> str | None:
    """Use the configured interface when present, otherwise pick a likely robot LAN."""
    available = available_net_interfaces()
    if config_net in available:
        return config_net
    for candidate in ("enP8p1s0", "enp5s0f1", "eth0", "usb0", "usb1"):
        if candidate in available:
            print(
                f"[WARN] configured lowstate net '{config_net}' is unavailable; using '{candidate}'",
                flush=True,
            )
            return candidate
    if available:
        selected = sorted(available)[0]
        print(
            f"[WARN] configured lowstate net '{config_net}' is unavailable; using '{selected}'",
            flush=True,
        )
        return selected
    return config_net


def _put_latest(q, item) -> None:
    while True:
        try:
            q.put_nowait(item)
            return
        except queue.Full:
            try:
                q.get_nowait()
            except queue.Empty:
                return


class UnitreeDdsJointListener:
    """Read waist yaw/roll/pitch from Unitree DDS rt/lowstate."""

    def __init__(self, net: str | None, topic: str = "rt/lowstate", max_hz: float = 50.0):
        self.net = net
        self.topic = topic
        self.q_wy = 0.0
        self.q_wr = 0.0
        self.q_wp = 0.0
        self.last_msg_s = 0.0
        self._min_update_period_s = 1.0 / max_hz if max_hz > 0 else 0.0
        self._last_update_s = 0.0
        self._cb_count = 0
        self._applied_count = 0
        self._skipped_count = 0
        self._cb_time_acc_s = 0.0
        self._cb_time_max_s = 0.0
        self._stats_window_start_s = time.time()
        self._lock = threading.Lock()
        ChannelFactoryInitialize(0, net)
        self._subscriber = ChannelSubscriber(topic, LowStateHG)
        self._subscriber.Init(self._cb, 10)
        print(
            f"[INFO] Unitree DDS joint listener started "
            f"(net={net or 'auto'}, topic={topic}, max {max_hz:.0f} Hz)",
            flush=True,
        )

    def _cb(self, msg: LowStateHG):
        t0 = time.perf_counter()
        now_s = time.time()
        self._cb_count += 1
        if now_s - self._last_update_s < self._min_update_period_s:
            self._skipped_count += 1
        else:
            with self._lock:
                self._last_update_s = now_s
                self.q_wy = float(msg.motor_state[12].q)
                self.q_wr = float(msg.motor_state[13].q)
                self.q_wp = float(msg.motor_state[14].q)
                self.last_msg_s = now_s
            self._applied_count += 1
            if self._applied_count == 1:
                print(
                    "[INFO] Unitree DDS lowstate first sample "
                    f"waist_q=({self.q_wy:+.3f},{self.q_wr:+.3f},{self.q_wp:+.3f})",
                    flush=True,
                )

        dt = time.perf_counter() - t0
        self._cb_time_acc_s += dt
        self._cb_time_max_s = max(self._cb_time_max_s, dt)

    def snapshot(self) -> tuple[tuple[float, float, float], float | None]:
        """Return latest waist joints and local age in seconds."""
        with self._lock:
            q = (self.q_wy, self.q_wr, self.q_wp)
            last_msg_s = self.last_msg_s
        age_s = None if last_msg_s <= 0.0 else time.time() - last_msg_s
        return q, age_s

    def pop_stats(self) -> dict[str, float]:
        with self._lock:
            now_s = time.time()
            elapsed_s = max(1e-6, now_s - self._stats_window_start_s)
            stats = {
                "lowstate_cb_hz": self._cb_count / elapsed_s,
                "lowstate_applied_hz": self._applied_count / elapsed_s,
                "lowstate_skipped_hz": self._skipped_count / elapsed_s,
                "lowstate_cb_avg_ms": (
                    (self._cb_time_acc_s / self._cb_count) * 1000.0
                    if self._cb_count else 0.0
                ),
                "lowstate_cb_max_ms": self._cb_time_max_s * 1000.0,
            }
            self._cb_count = 0
            self._applied_count = 0
            self._skipped_count = 0
            self._cb_time_acc_s = 0.0
            self._cb_time_max_s = 0.0
            self._stats_window_start_s = now_s
            return stats


def _lowstate_process_main(net: str | None, topic: str, max_hz: float, out_q, stop_event) -> None:
    try:
        listener = UnitreeDdsJointListener(net, topic=topic, max_hz=max_hz)
        last_msg_s = 0.0
        while not stop_event.is_set():
            with listener._lock:
                item = (
                    listener.q_wy,
                    listener.q_wr,
                    listener.q_wp,
                    listener.last_msg_s,
                )
            if item[3] > 0.0 and item[3] != last_msg_s:
                _put_latest(out_q, item)
                last_msg_s = item[3]
            time.sleep(0.01)
    except Exception as exc:
        _put_latest(out_q, ("error", repr(exc), time.time()))


class UnitreeDdsJointProcessListener:
    """Run Unitree SDK2 lowstate DDS in a child process to avoid ROS DDS conflicts."""

    def __init__(self, net: str | None, topic: str = "rt/lowstate", max_hz: float = 50.0):
        self.net = net
        self.topic = topic
        self.q_wy = 0.0
        self.q_wr = 0.0
        self.q_wp = 0.0
        self.last_msg_s = 0.0
        self.last_error = None
        ctx = mp.get_context("spawn")
        self._q = ctx.Queue(maxsize=1)
        self._stop_event = ctx.Event()
        self._proc = ctx.Process(
            target=_lowstate_process_main,
            args=(net, topic, max_hz, self._q, self._stop_event),
            daemon=True,
        )
        self._proc.start()
        print(
            f"[INFO] Unitree DDS lowstate process started "
            f"(pid={self._proc.pid}, net={net or 'auto'}, topic={topic}, max {max_hz:.0f} Hz)",
            flush=True,
        )

    def snapshot(self) -> tuple[tuple[float, float, float], float | None]:
        while True:
            try:
                item = self._q.get_nowait()
            except queue.Empty:
                break
            if item and item[0] == "error":
                self.last_error = item[1]
                continue
            self.q_wy, self.q_wr, self.q_wp, self.last_msg_s = item
        age_s = None if self.last_msg_s <= 0.0 else time.time() - self.last_msg_s
        return (self.q_wy, self.q_wr, self.q_wp), age_s

    def stop(self) -> None:
        self._stop_event.set()
        self._proc.join(timeout=1.0)
        if self._proc.is_alive():
            self._proc.terminate()
            self._proc.join(timeout=1.0)
