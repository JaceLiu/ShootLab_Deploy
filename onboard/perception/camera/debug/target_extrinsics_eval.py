"""Rolling statistics tool for chest-camera extrinsics tuning.

Workflow:
1. Place a static target in front of the robot.
2. Run target_ball_detector.py with optional --chest-xyz/--chest-rpy overrides.
3. Run this script to inspect mean/std of the published pelvis-frame target pose.
4. Adjust extrinsics until the position is physically reasonable and stable.
"""

import argparse
import sys
import time
from collections import deque
from pathlib import Path

import numpy as np

sys.path.append(str(Path(__file__).parent.parent.parent.parent.parent.absolute()))

from common.target_state_dds import TargetStateSubscriber


def main():
    parser = argparse.ArgumentParser(description="Evaluate rt/target_state stability for chest extrinsics tuning.")
    parser.add_argument("--window", type=int, default=50,
                        help="Number of valid samples in the rolling statistics window.")
    parser.add_argument("--hz", type=float, default=10.0,
                        help="Refresh rate for printed statistics.")
    args = parser.parse_args()

    sub = TargetStateSubscriber(domain_id=0)
    sub.start()
    samples = deque(maxlen=args.window)

    print("[INFO] Listening on rt/target_state for extrinsics evaluation. Press Ctrl+C to stop.")
    try:
        while True:
            msg = sub.latest()
            if msg.valid:
                samples.append(np.array([msg.x, msg.y, msg.z], dtype=np.float32))

            if samples:
                arr = np.stack(samples, axis=0)
                mean = arr.mean(axis=0)
                std = arr.std(axis=0)
                span = arr.max(axis=0) - arr.min(axis=0)
                print(
                    f"\r[n={len(samples):03d}] "
                    f"mean=({mean[0]:+.3f}, {mean[1]:+.3f}, {mean[2]:+.3f}) "
                    f"std=({std[0]:.3f}, {std[1]:.3f}, {std[2]:.3f}) "
                    f"span=({span[0]:.3f}, {span[1]:.3f}, {span[2]:.3f})   ",
                    end="",
                    flush=True,
                )
            else:
                print("\r[waiting] no valid target samples yet...   ", end="", flush=True)

            time.sleep(max(1e-3, 1.0 / args.hz))
    except KeyboardInterrupt:
        print("\n[INFO] Interrupted.")
    finally:
        sub.stop()


if __name__ == "__main__":
    main()
