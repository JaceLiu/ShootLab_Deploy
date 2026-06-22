"""TargetState DDS message + publisher/subscriber helpers.

Topic  : "rt/target_state"
Content: target position in pelvis (base) body frame, published at sensor rate.

This file mirrors the BallState DDS helper style so the onboard perception stack
and future control-side consumers can share one lightweight interface.
"""

import time
import threading
from dataclasses import dataclass

import cyclonedds.idl as idl
import cyclonedds.idl.types as types
from cyclonedds.domain import DomainParticipant
from cyclonedds.topic import Topic
from cyclonedds.pub import Publisher, DataWriter
from cyclonedds.sub import Subscriber, DataReader
from cyclonedds.core import Qos, Policy

TARGET_STATE_TOPIC = "rt/target_state"
TARGET_STALE_MS = 300

SOURCE_NONE = 0
SOURCE_CHEST_CAMERA = 1

INVALID_CLASS_ID = -1


@dataclass
class TargetState(idl.IdlStruct, typename="mjlab::msg::dds_::TargetState_"):
    """Target position in pelvis (base) body frame."""

    timestamp_us: types.uint64 = 0
    x: types.float32 = 0.0
    y: types.float32 = 0.0
    z: types.float32 = 0.0
    valid: types.uint8 = 0
    class_id: types.int32 = INVALID_CLASS_ID
    confidence: types.float32 = 0.0
    source: types.uint8 = SOURCE_NONE


_SENSOR_QOS = Qos(
    Policy.Reliability.BestEffort,
    Policy.History.KeepLast(1),
)


class TargetStatePublisher:
    def __init__(self, domain_id: int = 0, topic_name: str = TARGET_STATE_TOPIC):
        self._dp = DomainParticipant(domain_id)
        self._topic = Topic(self._dp, topic_name, TargetState, qos=_SENSOR_QOS)
        self._pub = Publisher(self._dp)
        self._writer = DataWriter(self._pub, self._topic, qos=_SENSOR_QOS)

    def publish(
        self,
        x: float,
        y: float,
        z: float,
        valid: bool = True,
        class_id: int = INVALID_CLASS_ID,
        confidence: float = 0.0,
        source: int = SOURCE_NONE,
    ):
        msg = TargetState(
            timestamp_us=int(time.time() * 1e6),
            x=float(x),
            y=float(y),
            z=float(z),
            valid=1 if valid else 0,
            class_id=int(class_id),
            confidence=float(confidence),
            source=source,
        )
        self._writer.write(msg)


class TargetStateSubscriber:
    """Non-blocking subscriber with staleness check."""

    def __init__(
        self,
        domain_id: int = 0,
        callback=None,
        topic_name: str = TARGET_STATE_TOPIC,
    ):
        self._dp = DomainParticipant(domain_id)
        self._topic = Topic(self._dp, topic_name, TargetState, qos=_SENSOR_QOS)
        self._sub = Subscriber(self._dp)
        self._reader = DataReader(self._sub, self._topic, qos=_SENSOR_QOS)
        self._callback = callback

        self._lock = threading.Lock()
        self._last = TargetState()
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
                valid_samples = [s for s in samples if isinstance(s, TargetState)]
                if valid_samples:
                    newest = valid_samples[-1]
                    with self._lock:
                        self._last = newest
                        self._received_at = int(time.time() * 1e6)
                    if self._callback:
                        self._callback(newest)
            except Exception:
                pass
            time.sleep(0.005)

    def latest(self) -> TargetState:
        with self._lock:
            state = self._last
            received_at = self._received_at
        if not isinstance(state, TargetState):
            return TargetState()
        now_us = int(time.time() * 1e6)
        if (now_us - received_at) > TARGET_STALE_MS * 1000:
            return TargetState(timestamp_us=state.timestamp_us, valid=0)
        return state
