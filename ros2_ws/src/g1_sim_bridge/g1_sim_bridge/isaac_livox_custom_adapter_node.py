"""Isaac Sim -> FAST-LIO adapter (simulation only).

FAST_LIO_LOCALIZATION_HUMANOID's Livox path (lidar_type:1 / AVIA) consumes a
``livox_ros_driver2/msg/CustomMsg`` with per-point ``offset_time`` + ``line`` +
``tag`` + ``reflectivity``, and subscribes to both the cloud and the IMU with
RELIABLE QoS (see FAST_LIO/config/mid360.yaml: lid_topic=/livox/custom_msg,
imu_topic=/livox/imu).

Isaac Sim's ROS 2 bridge instead publishes:
  * ``/livox/lidar``     : a plain XYZ ``sensor_msgs/PointCloud2`` (no
    intensity/tag/line/time fields), and
  * ``/livox/imu_raw``   : ``sensor_msgs/Imu`` (BEST_EFFORT),
neither of which FAST-LIO can consume directly ("Failed to find match for field
'intensity'/'tag'/'line'" + "incompatible QoS … RELIABILITY" on the IMU).

This node bridges the two, in sim only, with NO change to mid360.yaml:
  * Cloud:  /livox/lidar (PointCloud2, XYZ) -> CustomMsg on /livox/custom_msg,
            synthesizing offset_time (0 by default; instantaneous snapshots),
            line (round-robin over num_lines, < FAST-LIO scan_line), tag=0,
            reflectivity. Published RELIABLE so FAST-LIO's subscriber matches.
  * IMU:    /livox/imu_raw (BEST_EFFORT) -> /livox/imu (RELIABLE), unchanged.

On the real robot this node is NOT used -- the Livox SDK driver emits a native
CustomMsg + a RELIABLE /livox/imu directly. This file is adapted from the
g1_real_lidar adapter in the G1_navigation repo.

Parameters
----------
input_topic     Isaac cloud (PointCloud2, XYZ).    Default: /livox/lidar
output_topic    CustomMsg for FAST-LIO.            Default: /livox/custom_msg
imu_in_topic    Isaac IMU (BEST_EFFORT).           Default: /livox/imu_raw
imu_out_topic   RELIABLE IMU for FAST-LIO.         Default: /livox/imu
scan_period     Seconds per scan (offset_time).    Default: 0.1 (10 Hz)
fake_sweep_time Synthesize a linear 0..scan_period per-point offset_time. Isaac
                clouds are instantaneous snapshots (no rolling shutter), so the
                default is false -> offset_time=0 (no de-skew). Setting it true
                makes FAST-LIO de-skew motion that did not happen, injecting
                distortion that diverges the estimate while turning.
num_lines       Synthetic scan lines (<= FAST-LIO scan_line). Default: 4
reflectivity    Constant reflectivity for every point (0-255). Default: 100
stride          Keep every Nth point (1 = all).    Default: 1
"""
from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py import point_cloud2

from livox_ros_driver2.msg import CustomMsg, CustomPoint


class IsaacLivoxCustomAdapter(Node):

    def __init__(self):
        super().__init__('isaac_livox_custom_adapter')

        self.declare_parameter('input_topic', '/livox/lidar')
        self.declare_parameter('output_topic', '/livox/custom_msg')
        self.declare_parameter('imu_in_topic', '/livox/imu_raw')
        self.declare_parameter('imu_out_topic', '/livox/imu')
        self.declare_parameter('scan_period', 0.1)
        self.declare_parameter('fake_sweep_time', False)
        self.declare_parameter('num_lines', 4)
        self.declare_parameter('reflectivity', 100)
        self.declare_parameter('stride', 1)

        self._out_topic = str(self.get_parameter('output_topic').value)
        self._scan_period = float(self.get_parameter('scan_period').value)
        self._fake_sweep = bool(self.get_parameter('fake_sweep_time').value)
        self._num_lines = max(1, int(self.get_parameter('num_lines').value))
        self._reflectivity = int(self.get_parameter('reflectivity').value) & 0xFF
        self._stride = max(1, int(self.get_parameter('stride').value))

        # Isaac publishes the cloud/IMU BEST_EFFORT; subscribe to match.
        be_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=5,
        )
        # FAST-LIO's CustomMsg + IMU subscribers are RELIABLE; publish RELIABLE.
        rel_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=10,
        )

        self._cloud_pub = self.create_publisher(CustomMsg, self._out_topic, rel_qos)
        self._imu_pub = self.create_publisher(
            Imu, str(self.get_parameter('imu_out_topic').value), rel_qos)

        self.create_subscription(
            PointCloud2, str(self.get_parameter('input_topic').value),
            self._cloud_cb, be_qos)
        self.create_subscription(
            Imu, str(self.get_parameter('imu_in_topic').value),
            self._imu_cb, be_qos)

        self._warned_empty = False
        self.get_logger().info(
            f'Isaac->FAST-LIO adapter: PointCloud2 -> CustomMsg on '
            f'{self._out_topic!r} (RELIABLE), IMU -> '
            f'{str(self.get_parameter("imu_out_topic").value)!r} (RELIABLE); '
            f'num_lines={self._num_lines} stride={self._stride} '
            f'fake_sweep_time={self._fake_sweep}'
        )

    def _imu_cb(self, msg: Imu) -> None:
        # Pure QoS bridge (BEST_EFFORT -> RELIABLE); contents unchanged.
        self._imu_pub.publish(msg)

    def _cloud_cb(self, msg: PointCloud2) -> None:
        try:
            raw = point_cloud2.read_points(
                msg, field_names=('x', 'y', 'z'), skip_nans=True)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(
                f'cloud parse error: {exc}', throttle_duration_sec=5.0)
            return

        pts = np.column_stack((raw['x'], raw['y'], raw['z'])).astype(np.float32)
        if self._stride > 1:
            pts = pts[::self._stride]
        n = int(pts.shape[0])
        if n == 0:
            if not self._warned_empty:
                self.get_logger().warning('empty cloud -- nothing to forward')
                self._warned_empty = True
            return

        # Per-point offset_time. Isaac clouds are instantaneous snapshots, so
        # default to 0 (no de-skew). Only synthesize a linear sweep if asked.
        if self._fake_sweep:
            period_ns = self._scan_period * 1e9
            offsets = (np.arange(n, dtype=np.float64) / max(n - 1, 1)
                       * period_ns).astype(np.uint32)
        else:
            offsets = np.zeros(n, dtype=np.uint32)
        # Round-robin line index (< FAST-LIO scan_line).
        lines = (np.arange(n, dtype=np.int64) % self._num_lines).astype(np.uint8)

        out = CustomMsg()
        out.header = msg.header
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(
            msg.header.stamp.nanosec)
        out.timebase = stamp_ns
        out.point_num = n
        out.lidar_id = 0

        points = []
        ref = self._reflectivity
        xs, ys, zs = pts[:, 0], pts[:, 1], pts[:, 2]
        for i in range(n):
            cp = CustomPoint()
            cp.offset_time = int(offsets[i])
            cp.x = float(xs[i])
            cp.y = float(ys[i])
            cp.z = float(zs[i])
            cp.reflectivity = ref
            cp.tag = 0          # (tag & 0x30) == 0x00 -> accepted by avia_handler
            cp.line = int(lines[i])
            points.append(cp)
        out.points = points

        self._cloud_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = IsaacLivoxCustomAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
