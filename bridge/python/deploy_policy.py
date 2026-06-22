import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parents[2]))

import time

from bridge.python.bridge_cmd_dds import BridgeCmdPublisher
from deploy_real.config import Config
from bridge.python.policy_runtime import PolicyRuntime


def main():
    config = Config()
    runtime = PolicyRuntime(config)
    cmd_pub = BridgeCmdPublisher(
        domain_id=config.bridge_domain_id,
        topic_name=config.bridge_cmd_topic,
    )

    try:
        while True:
            loop_start = time.time()
            frame = runtime.step()
            cmd_pub.publish(
                seq=frame.seq,
                q_des=frame.q_des,
                kp=frame.kp,
                kd=frame.kd,
                request_damping=frame.request_damping,
                exit_requested=frame.exit_requested,
            )
            if frame.exit_requested:
                break

            loop_dt = time.time() - loop_start
            if loop_dt < config.control_dt:
                time.sleep(config.control_dt - loop_dt)
    except KeyboardInterrupt:
        pass
    finally:
        runtime.close()
        cmd_pub.publish(
            seq=0,
            q_des=[0.0] * config.num_joints,
            kp=[0.0] * config.num_joints,
            kd=[8.0] * config.num_joints,
            request_damping=True,
            exit_requested=True,
        )
        print("Exit")


if __name__ == "__main__":
    main()
