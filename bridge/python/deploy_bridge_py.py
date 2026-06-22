import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parents[2]))

import time

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_, unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_ as LowCmdHG
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_ as LowStateHG
from unitree_sdk2py.utils.crc import CRC

from bridge.python.bridge_state_dds import BridgeStatePublisher
from bridge.python.bridge_cmd_dds import BridgeCmdSubscriber
from common.command_helper import create_damping_cmd, init_cmd_hg, MotorMode
from deploy_real.config import Config


class PythonSdk2Bridge:
    """Prototype bridge using unitree_sdk2py for immediate end-to-end validation."""

    def __init__(self, config: Config):
        self.config = config
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.low_state = unitree_hg_msg_dds__LowState_()
        self.mode_pr = MotorMode.PR
        self.mode_machine = 0

        self.lowcmd_publisher = ChannelPublisher(config.lowcmd_topic, LowCmdHG)
        self.lowcmd_publisher.Init()

        self.lowstate_subscriber = ChannelSubscriber(config.lowstate_topic, LowStateHG)
        self.lowstate_subscriber.Init(self.lowstate_handler, 10)

        self.bridge_state_pub = BridgeStatePublisher(
            domain_id=config.bridge_domain_id,
            topic_name=config.bridge_state_topic,
        )
        self.bridge_cmd_sub = BridgeCmdSubscriber(
            domain_id=config.bridge_domain_id,
            topic_name=config.bridge_cmd_topic,
        )
        self.bridge_cmd_sub.start()

        init_cmd_hg(self.low_cmd, self.mode_machine, self.mode_pr)
        self.wait_for_low_state()

    def lowstate_handler(self, msg: LowStateHG):
        self.low_state = msg
        self.mode_machine = self.low_state.mode_machine

    def wait_for_low_state(self):
        while self.low_state.tick == 0:
            time.sleep(0.01)
        print("Python bridge connected to robot lowstate.")

    def close(self):
        self.bridge_cmd_sub.stop()
        create_damping_cmd(self.low_cmd)
        self.send_lowcmd()

    def send_lowcmd(self):
        self.low_cmd.crc = CRC().Crc(self.low_cmd)
        self.lowcmd_publisher.Write(self.low_cmd)

    def publish_bridge_state(self):
        q = [float(m.q) for m in self.low_state.motor_state]
        dq = [float(m.dq) for m in self.low_state.motor_state]
        quat = [float(v) for v in self.low_state.imu_state.quaternion]
        gyro = [float(v) for v in self.low_state.imu_state.gyroscope]
        remote_raw = list(self.low_state.wireless_remote[:24])
        self.bridge_state_pub.publish(
            tick=int(self.low_state.tick),
            q=q,
            dq=dq,
            imu_quat_wxyz=quat,
            imu_gyro=gyro,
            remote_raw=remote_raw,
        )

    def apply_bridge_cmd(self):
        cmd_msg, received_at = self.bridge_cmd_sub.latest()
        now_us = int(time.time() * 1e6)
        stale = (
            received_at == 0
            or (now_us - received_at) > self.config.bridge_cmd_timeout_ms * 1000
        )

        if stale or bool(cmd_msg.request_damping) or bool(cmd_msg.exit_requested):
            create_damping_cmd(self.low_cmd)
            self.send_lowcmd()
            return bool(cmd_msg.exit_requested)

        for i in range(self.config.num_joints):
            self.low_cmd.motor_cmd[i].q = float(cmd_msg.q_des[i])
            self.low_cmd.motor_cmd[i].qd = 0.0
            self.low_cmd.motor_cmd[i].kp = float(cmd_msg.kp[i])
            self.low_cmd.motor_cmd[i].kd = float(cmd_msg.kd[i])
            self.low_cmd.motor_cmd[i].tau = 0.0
        self.send_lowcmd()
        return False

    def run(self):
        loop_period_s = max(self.config.bridge_loop_period_ms, 1) / 1000.0
        try:
            while True:
                loop_start = time.time()
                self.publish_bridge_state()
                should_exit = self.apply_bridge_cmd()
                if should_exit:
                    break
                elapsed = time.time() - loop_start
                if elapsed < loop_period_s:
                    time.sleep(loop_period_s - elapsed)
        except KeyboardInterrupt:
            pass
        finally:
            self.close()
            print("Python SDK2 bridge exited.")


if __name__ == "__main__":
    config = Config()
    ChannelFactoryInitialize(0, config.net)
    PythonSdk2Bridge(config).run()
