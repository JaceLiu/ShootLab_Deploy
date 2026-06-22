"""Simple DDS subscriber for rt/target_state.

Use this to verify that the target detector is publishing stable positions before
connecting any future controller-side consumer.
"""

import sys
import time
from pathlib import Path

import numpy as np

sys.path.append(str(Path(__file__).parent.parent.parent.parent.parent.absolute()))

from common.target_state_dds import INVALID_CLASS_ID, TargetStateSubscriber


def main():
    sub = TargetStateSubscriber(domain_id=0)
    sub.start()
    print("[INFO] Listening on rt/target_state. Press Ctrl+C to stop.")
    try:
        while True:
            msg = sub.latest()
            if msg.valid:
                pos = np.array([msg.x, msg.y, msg.z], dtype=np.float32)
                norm = float(np.linalg.norm(pos))
                print(
                    f"\r[valid] xyz=({msg.x:+.3f}, {msg.y:+.3f}, {msg.z:+.3f}) "
                    f"|norm|={norm:.3f}m cls={msg.class_id} conf={msg.confidence:.2f} src={msg.source}   ",
                    end="",
                    flush=True,
                )
            else:
                class_txt = "none" if msg.class_id == INVALID_CLASS_ID else str(msg.class_id)
                print(
                    f"\r[stale] waiting for valid target... cls={class_txt} conf={msg.confidence:.2f}   ",
                    end="",
                    flush=True,
                )
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n[INFO] Interrupted.")
    finally:
        sub.stop()


if __name__ == "__main__":
    main()
