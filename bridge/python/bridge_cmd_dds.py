"""BridgeCmd DDS message + publisher/subscriber helpers.

Topic   : "rt/policy_bridge_cmd"
Content : per-joint policy outputs emitted by the Python policy process and
          consumed by a low-level C++ bridge built on `unitree_sdk2`.
"""

import time
import threading
from dataclasses import dataclass, field
from typing import Optional

import cyclonedds.idl as idl
import cyclonedds.idl.types as types
from cyclonedds.domain import DomainParticipant
from cyclonedds.topic import Topic
from cyclonedds.pub import Publisher, DataWriter
from cyclonedds.sub import Subscriber, DataReader
from cyclonedds.core import Qos, Policy

BRIDGE_CMD_TOPIC = "rt/policy_bridge_cmd"
NUM_JOINTS = 29


_BRIDGE_QOS = Qos(
    Policy.Reliability.BestEffort,
    Policy.History.KeepLast(1),
)


@dataclass
class BridgeCmd(idl.IdlStruct, typename="mjlab::msg::dds_::BridgeCmd_"):
    timestamp_us: types.uint64 = 0
    seq: types.uint64 = 0
    q_des: types.sequence[types.float32] = field(default_factory=lambda: [0.0] * NUM_JOINTS)
    kp: types.sequence[types.float32] = field(default_factory=lambda: [0.0] * NUM_JOINTS)
    kd: types.sequence[types.float32] = field(default_factory=lambda: [0.0] * NUM_JOINTS)
    request_damping: types.uint8 = 0
    exit_requested: types.uint8 = 0


class BridgeCmdPublisher:
    def __init__(self, domain_id: int = 0, topic_name: str = BRIDGE_CMD_TOPIC):
        self._dp = DomainParticipant(domain_id)
        self._topic = Topic(self._dp, topic_name, BridgeCmd, qos=_BRIDGE_QOS)
        self._pub = Publisher(self._dp)
        self._writer = DataWriter(self._pub, self._topic, qos=_BRIDGE_QOS)

    def publish(
        self,
        seq: int,
        q_des,
        kp,
        kd,
        request_damping: bool = False,
        exit_requested: bool = False,
        timestamp_us: Optional[int] = None,
    ):
        msg = BridgeCmd(
            timestamp_us=int(time.time() * 1e6) if timestamp_us is None else int(timestamp_us),
            seq=int(seq),
            q_des=[float(v) for v in q_des],
            kp=[float(v) for v in kp],
            kd=[float(v) for v in kd],
            request_damping=1 if request_damping else 0,
            exit_requested=1 if exit_requested else 0,
        )
        self._writer.write(msg)


class BridgeCmdSubscriber:
    def __init__(
        self,
        domain_id: int = 0,
        callback=None,
        topic_name: str = BRIDGE_CMD_TOPIC,
    ):
        self._dp = DomainParticipant(domain_id)
        self._topic = Topic(self._dp, topic_name, BridgeCmd, qos=_BRIDGE_QOS)
        self._sub = Subscriber(self._dp)
        self._reader = DataReader(self._sub, self._topic, qos=_BRIDGE_QOS)
        self._callback = callback

        self._lock = threading.Lock()
        self._last = BridgeCmd()
        self._received_at = 0
        self._thread: threading.Thread | None = None
        self._running = False

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False

    def _poll_loop(self):
        while self._running:
            try:
                samples = self._reader.take(N=10)
                valid_samples = [s for s in samples if isinstance(s, BridgeCmd)]
                if valid_samples:
                    newest = valid_samples[-1]
                    with self._lock:
                        self._last = newest
                        self._received_at = int(time.time() * 1e6)
                    if self._callback:
                        self._callback(newest)
            except Exception:
                pass
            time.sleep(0.002)

    def latest(self):
        with self._lock:
            return self._last, self._received_at
