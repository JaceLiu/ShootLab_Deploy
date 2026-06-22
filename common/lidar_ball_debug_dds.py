"""LiDAR ball debug DDS message + helpers.

Topic: "rt/lidar_ball_debug"

This is intentionally separate from rt/ball_state so controller-facing ball
state stays stable while debug tools can inspect pre-FK LiDAR measurements.
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


LIDAR_BALL_DEBUG_TOPIC = "rt/lidar_ball_debug"
LIDAR_BALL_DEBUG_STALE_MS = 300


@dataclass
class LidarBallDebugState(
    idl.IdlStruct,
    typename="mjlab::msg::dds_::LidarBallDebugState_",
):
    """LiDAR-only ball debug values in MID360 and pelvis frames."""

    timestamp_us: types.uint64 = 0
    raw_x: types.float32 = 0.0       # raw estimated ball center in MID360 frame
    raw_y: types.float32 = 0.0
    raw_z: types.float32 = 0.0
    kf_x: types.float32 = 0.0        # KF-smoothed center in MID360 frame, before FK
    kf_y: types.float32 = 0.0
    kf_z: types.float32 = 0.0
    base_x: types.float32 = 0.0      # published pelvis frame after FK + bias
    base_y: types.float32 = 0.0
    base_z: types.float32 = 0.0
    base_y_bias: types.float32 = 0.0
    candidate_count: types.uint32 = 0
    in_shell_count: types.uint32 = 0
    valid: types.uint8 = 0


_SENSOR_QOS = Qos(
    Policy.Reliability.BestEffort,
    Policy.History.KeepLast(1),
)


class LidarBallDebugPublisher:
    def __init__(self, domain_id: int = 0, topic_name: str = LIDAR_BALL_DEBUG_TOPIC):
        self._dp = DomainParticipant(domain_id)
        self._topic = Topic(self._dp, topic_name, LidarBallDebugState, qos=_SENSOR_QOS)
        self._pub = Publisher(self._dp)
        self._writer = DataWriter(self._pub, self._topic, qos=_SENSOR_QOS)

    def publish(
        self,
        raw,
        kf,
        base,
        *,
        base_y_bias: float = 0.0,
        candidate_count: int = 0,
        in_shell_count: int = 0,
        valid: bool = True,
    ):
        msg = LidarBallDebugState(
            timestamp_us=int(time.time() * 1e6),
            raw_x=float(raw[0]),
            raw_y=float(raw[1]),
            raw_z=float(raw[2]),
            kf_x=float(kf[0]),
            kf_y=float(kf[1]),
            kf_z=float(kf[2]),
            base_x=float(base[0]),
            base_y=float(base[1]),
            base_z=float(base[2]),
            base_y_bias=float(base_y_bias),
            candidate_count=int(candidate_count),
            in_shell_count=int(in_shell_count),
            valid=1 if valid else 0,
        )
        self._writer.write(msg)


class LidarBallDebugSubscriber:
    def __init__(self, domain_id: int = 0, callback=None,
                 topic_name: str = LIDAR_BALL_DEBUG_TOPIC):
        self._dp = DomainParticipant(domain_id)
        self._topic = Topic(self._dp, topic_name, LidarBallDebugState, qos=_SENSOR_QOS)
        self._sub = Subscriber(self._dp)
        self._reader = DataReader(self._sub, self._topic, qos=_SENSOR_QOS)
        self._callback = callback
        self._lock = threading.Lock()
        self._last = LidarBallDebugState()
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
                valid_samples = [
                    s for s in samples if isinstance(s, LidarBallDebugState)
                ]
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

    def latest(self) -> LidarBallDebugState:
        with self._lock:
            s = self._last
            received_at = self._received_at
        if not isinstance(s, LidarBallDebugState):
            return LidarBallDebugState()
        now_us = int(time.time() * 1e6)
        if (now_us - received_at) > LIDAR_BALL_DEBUG_STALE_MS * 1000:
            return LidarBallDebugState(timestamp_us=s.timestamp_us, valid=0)
        return s
