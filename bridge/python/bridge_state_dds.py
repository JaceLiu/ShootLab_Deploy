"""BridgeState DDS message + publisher/subscriber helpers.

Topic   : "rt/policy_bridge_state"
Content : low-level robot state forwarded by a C++ `unitree_sdk2` bridge to the
          Python policy process.
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

BRIDGE_STATE_TOPIC = "rt/policy_bridge_state"
BRIDGE_STATE_STALE_MS = 200
NUM_JOINTS = 29
REMOTE_RAW_BYTES = 24


_BRIDGE_QOS = Qos(
    Policy.Reliability.BestEffort,
    Policy.History.KeepLast(1),
)


@dataclass
class BridgeState(idl.IdlStruct, typename="mjlab::msg::dds_::BridgeState_"):
    timestamp_us: types.uint64 = 0
    tick: types.uint64 = 0
    q: types.sequence[types.float32] = field(default_factory=lambda: [0.0] * NUM_JOINTS)
    dq: types.sequence[types.float32] = field(default_factory=lambda: [0.0] * NUM_JOINTS)
    imu_quat_wxyz: types.sequence[types.float32] = field(default_factory=lambda: [1.0, 0.0, 0.0, 0.0])
    imu_gyro: types.sequence[types.float32] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    remote_raw: types.sequence[types.uint8] = field(default_factory=lambda: [0] * REMOTE_RAW_BYTES)


class BridgeStatePublisher:
    def __init__(self, domain_id: int = 0, topic_name: str = BRIDGE_STATE_TOPIC):
        self._dp = DomainParticipant(domain_id)
        self._topic = Topic(self._dp, topic_name, BridgeState, qos=_BRIDGE_QOS)
        self._pub = Publisher(self._dp)
        self._writer = DataWriter(self._pub, self._topic, qos=_BRIDGE_QOS)

    def publish(
        self,
        tick: int,
        q,
        dq,
        imu_quat_wxyz,
        imu_gyro,
        remote_raw,
        timestamp_us: Optional[int] = None,
    ):
        msg = BridgeState(
            timestamp_us=int(time.time() * 1e6) if timestamp_us is None else int(timestamp_us),
            tick=int(tick),
            q=[float(v) for v in q],
            dq=[float(v) for v in dq],
            imu_quat_wxyz=[float(v) for v in imu_quat_wxyz],
            imu_gyro=[float(v) for v in imu_gyro],
            remote_raw=[int(v) for v in remote_raw],
        )
        self._writer.write(msg)


class BridgeStateSubscriber:
    """Non-blocking subscriber with staleness check."""

    def __init__(
        self,
        domain_id: int = 0,
        callback=None,
        topic_name: str = BRIDGE_STATE_TOPIC,
        stale_ms: int = BRIDGE_STATE_STALE_MS,
    ):
        self._dp = DomainParticipant(domain_id)
        self._topic = Topic(self._dp, topic_name, BridgeState, qos=_BRIDGE_QOS)
        self._sub = Subscriber(self._dp)
        self._reader = DataReader(self._sub, self._topic, qos=_BRIDGE_QOS)
        self._callback = callback
        self._stale_ms = stale_ms

        self._lock = threading.Lock()
        self._last = BridgeState()
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
                valid_samples = [s for s in samples if isinstance(s, BridgeState)]
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

    def latest(self) -> BridgeState:
        with self._lock:
            state = self._last
            received_at = self._received_at
        if not isinstance(state, BridgeState):
            return BridgeState()
        now_us = int(time.time() * 1e6)
        if received_at == 0 or (now_us - received_at) > self._stale_ms * 1000:
            return BridgeState(timestamp_us=state.timestamp_us, tick=0)
        return state
