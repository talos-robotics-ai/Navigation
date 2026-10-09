"""Laptop: /fpose/crate_pose (the laptop detector; frame rgbd_head_front, camera-clock stamp) -> /x2/crate_pose (odom).

Uses the head-camera pose T_odom_cam that PC2 streams through the relay (/x2/cam_pose), taken at
arrival_time - latency_s (the detector's image->pose latency, ~0.3 s: TODO measure). The camera-clock stamp of the
input is ignored (it lags the system clock and drifts); the output is stamped with the arrival time.
"""
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from . import se3
from .cam_buffer import CamPoseBuffer, crate_in_odom_from_cam


class CrateToOdom(Node):
    def __init__(self, **kw):
        super().__init__('crate_to_odom', **kw)
        P = self.declare_parameter
        P('crate_in_topic', '/fpose/crate_pose')
        P('crate_out_topic', '/x2/crate_pose')
        P('cam_topic', '/x2/cam_pose')
        P('latency_s', 0.3)
        P('max_cam_gap_s', 0.5)
        g = lambda k: self.get_parameter(k).value  # noqa: E731
        self._lat, self._gap = float(g('latency_s')), float(g('max_cam_gap_s'))
        self._buf = CamPoseBuffer()
        self._n_in = self._n_out = self._n_nocam = 0
        self._pub = self.create_publisher(PoseStamped, g('crate_out_topic'), 10)
        self.create_subscription(PoseStamped, g('cam_topic'), self._on_cam, 10)
        self.create_subscription(PoseStamped, g('crate_in_topic'), self._on_crate, 10)
        self.create_timer(5.0, self._log)

    def _on_cam(self, m):
        p, o = m.pose.position, m.pose.orientation
        self._buf.add(time.monotonic(), se3.make([p.x, p.y, p.z], [o.x, o.y, o.z, o.w]))

    def _on_crate(self, m):
        self._n_in += 1
        t = time.monotonic()
        hit = self._buf.nearest(t - self._lat, self._gap)
        if hit is None:
            self._n_nocam += 1
            return
        p, o = m.pose.position, m.pose.orientation
        T = crate_in_odom_from_cam(hit[0], [p.x, p.y, p.z], [o.x, o.y, o.z, o.w])
        out = PoseStamped()
        out.header.stamp, out.header.frame_id = self.get_clock().now().to_msg(), 'odom'
        pp, qq = se3.position(T), se3.quat(T)
        out.pose.position.x, out.pose.position.y, out.pose.position.z = map(float, pp)
        out.pose.orientation.x, out.pose.orientation.y, out.pose.orientation.z, out.pose.orientation.w = map(float, qq)
        self._pub.publish(out)
        self._n_out += 1

    def _log(self):
        self.get_logger().info(f'crate in/out {self._n_in}/{self._n_out}, dropped for lack of a camera pose {self._n_nocam}')
        self._n_in = self._n_out = self._n_nocam = 0
