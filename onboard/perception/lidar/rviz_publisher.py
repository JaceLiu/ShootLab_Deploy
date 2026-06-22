"""RViz2 visualization helpers for the lidar ball detector.

Publishes the following topics (all latched at QoS depth=5):

  /ball_detector/cloud_all        sensor_msgs/PointCloud2
      Every point received from /livox/lidar this frame.

  /ball_detector/cloud_candidates sensor_msgs/PointCloud2
      Points that passed reflectivity + distance + ROI filters.

  /ball_detector/ball_raw         visualization_msgs/Marker  (SPHERE)
      Ball centre estimated by estimate_ball_center_ls(), before KF.
      Rendered as a red semi-transparent sphere (radius = ball_r).

  /ball_detector/ball_kf          visualization_msgs/Marker  (SPHERE)
      Ball centre after Kalman filter.
      Rendered as a green opaque sphere (radius = ball_r).

  /ball_detector/text_info        visualization_msgs/Marker  (TEXT)
      One-line debug string just above the KF sphere:
        n=<cand_pts>  off=<offset>m  cost=<ms>ms

All markers use frame_id='livox_frame' by default (change via constructor).
"""

import struct
import numpy as np

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from builtin_interfaces.msg import Duration
from std_msgs.msg import ColorRGBA, Header
from geometry_msgs.msg import Point, Vector3
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker


# ── helpers ──────────────────────────────────────────────────────────────────

def _make_header(frame_id: str, stamp) -> Header:
    h = Header()
    h.frame_id = frame_id
    h.stamp    = stamp
    return h


def _xyz_to_pointcloud2(points: np.ndarray, header: Header) -> PointCloud2:
    """Pack an (N,3) float32 array into a PointCloud2 message."""
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width  = len(points)
    msg.is_dense     = False
    msg.is_bigendian  = False
    msg.point_step   = 12   # 3 × float32
    msg.row_step     = msg.point_step * msg.width
    msg.fields = [
        PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
    ]
    msg.data = points.astype(np.float32).tobytes()
    return msg


def _sphere_marker(
    center: np.ndarray,
    radius: float,
    header: Header,
    marker_id: int,
    r: float, g: float, b: float, a: float,
) -> Marker:
    mk = Marker()
    mk.header = header
    mk.ns     = "ball_detector"
    mk.id     = marker_id
    mk.type   = Marker.SPHERE
    mk.action = Marker.ADD
    mk.pose.position.x = float(center[0])
    mk.pose.position.y = float(center[1])
    mk.pose.position.z = float(center[2])
    mk.pose.orientation.w = 1.0
    mk.scale.x = radius * 2.0
    mk.scale.y = radius * 2.0
    mk.scale.z = radius * 2.0
    mk.color   = ColorRGBA(r=r, g=g, b=b, a=a)
    mk.lifetime = Duration(sec=1)   # auto-vanish after 1s if no new msg
    return mk


def _text_marker(
    center: np.ndarray,
    text: str,
    header: Header,
    marker_id: int,
) -> Marker:
    mk = Marker()
    mk.header = header
    mk.ns     = "ball_detector"
    mk.id     = marker_id
    mk.type   = Marker.TEXT_VIEW_FACING
    mk.action = Marker.ADD
    mk.pose.position.x = float(center[0])
    mk.pose.position.y = float(center[1])
    mk.pose.position.z = float(center[2]) + 0.25   # float text above sphere
    mk.pose.orientation.w = 1.0
    mk.scale.z = 0.08                              # text height in metres
    mk.color   = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
    mk.text    = text
    mk.lifetime = Duration(sec=1)
    return mk


# ── main class ────────────────────────────────────────────────────────────────

class RvizPublisher:
    """Thin wrapper around ROS2 publishers; attach to any Node instance."""

    def __init__(
        self,
        node: Node,
        frame_id: str = "livox_frame",
        ball_r: float = 0.115,
        enable: bool = False,
        cloud_publish_hz: float = 5.0,
    ):
        self._node     = node
        self._frame_id = frame_id
        self._ball_r   = ball_r
        self._enable   = enable
        self._cloud_period_s = 0.0 if cloud_publish_hz <= 0 else 1.0 / cloud_publish_hz
        self._last_cloud_pub_t = 0.0

        self._pub_all   = node.create_publisher(PointCloud2, "/ball_detector/cloud_all",        5)
        self._pub_cand  = node.create_publisher(PointCloud2, "/ball_detector/cloud_candidates", 5)
        self._pub_raw   = node.create_publisher(Marker,      "/ball_detector/ball_raw",         5)
        self._pub_kf    = node.create_publisher(Marker,      "/ball_detector/ball_kf",          5)
        self._pub_text  = node.create_publisher(Marker,      "/ball_detector/text_info",        5)

        node.get_logger().info(
            f"[RvizPublisher] ready — frame_id='{frame_id}'  "
            "topics: /ball_detector/{{cloud_all, cloud_candidates, ball_raw, ball_kf, text_info}}"
        )

    # ------------------------------------------------------------------

    @staticmethod
    def _has_subscribers(pub) -> bool:
        try:
            return pub.get_subscription_count() > 0
        except Exception:
            return True

    def should_publish_clouds(self) -> bool:
        if not self._enable:
            return False
        if not (self._has_subscribers(self._pub_all) or self._has_subscribers(self._pub_cand)):
            return False
        if self._cloud_period_s <= 0.0:
            return True
        return (self._node.get_clock().now().nanoseconds * 1e-9 - self._last_cloud_pub_t) >= self._cloud_period_s

    def publish_clouds(self, all_xyz: np.ndarray, cand_xyz: np.ndarray, stamp):
        """Publish full cloud and filtered candidate cloud."""
        if not self.should_publish_clouds():
            return
        hdr = _make_header(self._frame_id, stamp)
        if len(all_xyz) and self._has_subscribers(self._pub_all):
            self._pub_all.publish(_xyz_to_pointcloud2(all_xyz, hdr))
        if len(cand_xyz) and self._has_subscribers(self._pub_cand):
            self._pub_cand.publish(_xyz_to_pointcloud2(cand_xyz, hdr))
        self._last_cloud_pub_t = self._node.get_clock().now().nanoseconds * 1e-9

    def publish_ball_raw(self, center: np.ndarray, stamp):
        """Red semi-transparent sphere at the raw LS-estimated ball centre."""
        if not self._enable or not self._has_subscribers(self._pub_raw):
            return
        hdr = _make_header(self._frame_id, stamp)
        self._pub_raw.publish(
            _sphere_marker(center, self._ball_r, hdr,
                           marker_id=0, r=1.0, g=0.2, b=0.2, a=0.5)
        )

    def publish_ball_kf(self, center: np.ndarray, stamp):
        """Green opaque sphere at the Kalman-filtered ball centre."""
        if not self._enable or not self._has_subscribers(self._pub_kf):
            return
        hdr = _make_header(self._frame_id, stamp)
        self._pub_kf.publish(
            _sphere_marker(center, self._ball_r, hdr,
                           marker_id=1, r=0.2, g=1.0, b=0.2, a=0.85)
        )

    def publish_text(self, center: np.ndarray, n_cand: int,
                     offset: float, cost_ms: float, stamp):
        """White text label above the KF sphere with key debug values."""
        if not self._enable or not self._has_subscribers(self._pub_text):
            return
        hdr  = _make_header(self._frame_id, stamp)
        text = f"n={n_cand}  off={offset:.3f}m  cost={cost_ms:.1f}ms"
        self._pub_text.publish(_text_marker(center, text, hdr, marker_id=2))
