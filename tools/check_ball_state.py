"""Quick diagnostic: print BallState DDS messages received from the robot.

Run on laptop (same network as G1):
    python tools/check_ball_state.py
"""

import sys
import time
from pathlib import Path
sys.path.append(str(Path(__file__).parent.parent.absolute()))

from common.ball_state_dds import BallStateSubscriber


def main():
    print("Subscribing to 'rt/ball_state' ... (Ctrl-C to stop)")
    sub = BallStateSubscriber(domain_id=0)
    sub.start()

    try:
        while True:
            ball = sub.latest()
            age_ms = (time.time() * 1e6 - ball.timestamp_us) / 1000.0
            status = "OK " if ball.valid else "---"
            print(
                f"[{status}]  "
                f"x={ball.x:+.3f}  y={ball.y:+.3f}  z={ball.z:+.3f}  "
                f"age={age_ms:.0f}ms",
                end="\033[K\r",
            )
            time.sleep(0.05)
    except KeyboardInterrupt:
        print()
    finally:
        sub.stop()


if __name__ == "__main__":
    main()
