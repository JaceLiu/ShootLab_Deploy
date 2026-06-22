import json
import os
from typing import Optional
import numpy as np
from datetime import datetime
from pathlib import Path


class Logger:
    """Crash-safe binary logger for FSM policy execution.

    Each call to log() writes one fixed-size float32 record and immediately
    flushes, so no data is lost if the process crashes.

    Record layout (114 × float32 = 456 bytes per frame):
        step(1), time_s(1), q(29), dq(29),
        pelvis_pos_w(3), pelvis_quat_wxyz(4),
        ball_pos_b(3), ball_pos_w(3), ball_valid(1),
        target_pos_b(3), target_valid(1),
        vel_cmd(3),
        debug_target_pos_b(3), debug_target_source(1),
        actions(29)

    Files written:
        <log_dir>/<YYYYMMDD_HHMMSS>_<tag>.bin   binary frames
        <log_dir>/<YYYYMMDD_HHMMSS>_<tag>.json  metadata
    """

    FIELDS = [
        ("step",         1),
        ("time_s",       1),
        ("q",           29),
        ("dq",          29),
        ("pelvis_pos_w", 3),
        ("pelvis_quat_w",4),
        ("ball_pos_b",   3),
        ("ball_pos_w",   3),
        ("ball_valid",   1),
        ("target_pos_b", 3),
        ("target_valid", 1),
        ("vel_cmd",      3),
        ("debug_target_pos_b", 3),
        ("debug_target_source", 1),
        ("actions",     29),
    ]
    RECORD_DIM: int = sum(n for _, n in FIELDS)  # 114

    def __init__(self, log_dir: str, tag: str = "freekick",
                 extra_meta: Optional[dict] = None) -> None:
        Path(log_dir).mkdir(parents=True, exist_ok=True)
        ts   = datetime.now().strftime("%Y%m%d_%H%M%S")
        base = os.path.join(log_dir, f"{ts}_{tag}")

        self._bin_path  = base + ".bin"
        self._meta_path = base + ".json"
        self._file      = open(self._bin_path, "wb")
        self._total_steps = 0
        self._record    = np.empty(self.RECORD_DIM, dtype=np.float32)

        meta: dict = {
            "tag":         tag,
            "created":     ts,
            "record_dim":  self.RECORD_DIM,
            "dtype":       "float32",
            "fields":      {name: size for name, size in self.FIELDS},
            "control_dt":  0.02,
            "total_steps": 0,
        }
        if extra_meta:
            meta.update(extra_meta)

        with open(self._meta_path, "w") as f:
            json.dump(meta, f, indent=2)

        print(f"[Logger] writing → {self._bin_path}")

    # ------------------------------------------------------------------

    def log(self, step: int, time_s: float,
            state_cmd, policy_output) -> None:
        """Write one control-step record.  Call after FSM.run()."""
        r = self._record
        offset = 0

        def put(arr, n: int) -> None:
            nonlocal offset
            r[offset : offset + n] = arr
            offset += n

        put([step],                        1)
        put([time_s],                      1)
        put(state_cmd.q,                  29)
        put(state_cmd.dq,                 29)
        put(state_cmd.pelvis_pos_w,        3)
        put(state_cmd.pelvis_quat_w,       4)
        put(state_cmd.ball_pos_b,          3)
        put(state_cmd.ball_pos_w,          3)
        put([float(state_cmd.ball_valid)], 1)
        put(state_cmd.target_pos_b,        3)
        put([float(state_cmd.target_valid)], 1)
        put(state_cmd.vel_cmd,             3)
        put(policy_output.debug_target_pos_b, 3)
        put(policy_output.debug_target_source, 1)
        put(policy_output.actions,        29)

        self._file.write(r.tobytes())
        self._file.flush()
        self._total_steps += 1

    def close(self) -> None:
        self._file.close()
        with open(self._meta_path) as f:
            meta = json.load(f)
        meta["total_steps"] = self._total_steps
        with open(self._meta_path, "w") as f:
            json.dump(meta, f, indent=2)
        print(f"[Logger] closed — {self._total_steps} frames → {self._bin_path}")

    # ------------------------------------------------------------------

    @staticmethod
    def load(bin_path: str) -> dict:
        """Load a .bin log file.

        Returns a dict mapping field names to numpy arrays:
            scalar fields → shape (T,)
            vector fields → shape (T, N)
        Also includes '_meta' with the raw JSON metadata.
        """
        meta_path = bin_path.replace(".bin", ".json")
        with open(meta_path) as f:
            meta = json.load(f)

        raw = np.fromfile(bin_path, dtype=np.float32)
        dim = meta["record_dim"]
        T   = len(raw) // dim
        if T == 0:
            raise ValueError(f"Log is empty: {bin_path}")

        frames = raw[: T * dim].reshape(T, dim)

        result: dict = {"_meta": meta}
        offset = 0
        for name, size in Logger.FIELDS:
            chunk = frames[:, offset : offset + size]
            result[name] = chunk.squeeze(axis=1) if size == 1 else chunk
            offset += size

        return result
