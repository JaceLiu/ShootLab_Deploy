"""Small timing profiler for camera perception services."""

from __future__ import annotations

import statistics
import time
from collections import defaultdict, deque


class StageTimer:
    """Collect per-stage timings and print mean/p95 every N frames."""

    def __init__(self, enabled: bool = False, window: int = 30, label: str = "camera"):
        self.enabled = bool(enabled)
        self.window = max(1, int(window))
        self.label = label
        self._samples = defaultdict(lambda: deque(maxlen=self.window))
        self._frame_count = 0

    @staticmethod
    def now() -> float:
        return time.perf_counter()

    def add(self, name: str, start: float, end: float) -> None:
        if not self.enabled:
            return
        self._samples[name].append(max(0.0, (end - start) * 1000.0))

    def tick(self) -> None:
        if not self.enabled:
            return
        self._frame_count += 1
        if self._frame_count % self.window != 0:
            return
        parts = []
        for name in sorted(self._samples):
            values = list(self._samples[name])
            if not values:
                continue
            mean_ms = statistics.fmean(values)
            p95_ms = values[0] if len(values) == 1 else statistics.quantiles(values, n=20)[-1]
            parts.append(f"{name}={mean_ms:.1f}/{p95_ms:.1f}ms")
        if parts:
            print(f"\n[{self.label}-timing] mean/p95 " + "  ".join(parts), flush=True)
