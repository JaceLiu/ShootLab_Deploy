"""BallState DDS message + publisher/subscriber helpers.

Topic  : "rt/ball_state"
Content: ball position in pelvis (base) body frame, published at ~10 Hz.

This file is shared between the on-robot detector service and the laptop
deploy_real.py.  Keep it identical on both sides.
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

BALL_STATE_TOPIC = "rt/ball_state"
BALL_STALE_MS    = 300   # ms — if no update within this window, report invalid


SOURCE_NONE  = 0
SOURCE_CAM   = 1
SOURCE_LIDAR = 2


@dataclass
class BallState(idl.IdlStruct, typename="mjlab::msg::dds_::BallState_"):
    """Ball position in pelvis (base) body frame."""
    timestamp_us : types.uint64  = 0    # µs since epoch
    x            : types.float32 = 0.0  # [m] pelvis body frame
    y            : types.float32 = 0.0
    z            : types.float32 = 0.0
    valid        : types.uint8   = 0    # 1 = detection valid, 0 = no ball
    source       : types.uint8   = 0    # 0=none, 1=camera, 2=lidar


# Best-effort, keep only the latest sample — appropriate for sensor data.
_SENSOR_QOS = Qos(
    Policy.Reliability.BestEffort,
    Policy.History.KeepLast(1),
)


# ---------------------------------------------------------------------------
# Publisher (on-robot side)
# ---------------------------------------------------------------------------

class BallStatePublisher:
    def __init__(self, domain_id: int = 0, topic_name: str = BALL_STATE_TOPIC):
        self._dp     = DomainParticipant(domain_id)
        self._topic  = Topic(self._dp, topic_name, BallState, qos=_SENSOR_QOS)
        self._pub    = Publisher(self._dp)
        self._writer = DataWriter(self._pub, self._topic, qos=_SENSOR_QOS)

    def publish(self, x: float, y: float, z: float, valid: bool = True,
                source: int = SOURCE_NONE):
        msg = BallState(
            timestamp_us=int(time.time() * 1e6),
            x=float(x),
            y=float(y),
            z=float(z),
            valid=1 if valid else 0,
            source=source,
        )
        self._writer.write(msg)


# ---------------------------------------------------------------------------
# Subscriber (laptop / deploy_real side)
# ---------------------------------------------------------------------------

class BallStateSubscriber:
    """Non-blocking subscriber with staleness check.

    Usage:
        sub = BallStateSubscriber()
        sub.start()

        # In control loop:
        ball = sub.latest()   # returns BallState; check .valid before using
    """

    def __init__(self, domain_id: int = 0, callback=None,
                 topic_name: str = BALL_STATE_TOPIC):
        """
        Args:
            callback: optional callable(BallState) invoked on each new sample.
                      If None, use latest() to poll.
            topic_name: DDS topic to subscribe to.
        """
        self._dp     = DomainParticipant(domain_id)
        self._topic  = Topic(self._dp, topic_name, BallState, qos=_SENSOR_QOS)
        self._sub    = Subscriber(self._dp)
        self._reader = DataReader(self._sub, self._topic, qos=_SENSOR_QOS)
        self._callback = callback

        self._lock        = threading.Lock()
        self._last        = BallState()   # zeros, valid=0
        self._received_at = 0             # local time (µs) when last sample arrived
        self._thread : threading.Thread | None = None
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
                # Filter out InvalidSample objects (emitted when a remote writer
                # is disposed/unregistered, e.g. when ball_detector is restarted).
                valid_samples = [s for s in samples if isinstance(s, BallState)]
                if valid_samples:
                    newest = valid_samples[-1]
                    with self._lock:
                        self._last = newest
                        self._received_at = int(time.time() * 1e6)
                    if self._callback:
                        self._callback(newest)
            except Exception:
                pass
            time.sleep(0.005)   # 200 Hz poll — fast enough for 10 Hz sensor

    def latest(self) -> BallState:
        """Return the most recent BallState.  Sets valid=0 if data is stale.

        Staleness is judged by local reception time to avoid clock-sync issues
        between the robot and the subscriber machine.
        """
        with self._lock:
            s = self._last
            received_at = self._received_at
        # Guard against any non-BallState object that slipped through.
        if not isinstance(s, BallState):
            return BallState()
        now_us = int(time.time() * 1e6)
        if (now_us - received_at) > BALL_STALE_MS * 1000:
            return BallState(timestamp_us=s.timestamp_us, valid=0)
        return s
