"""Publishes a cloud with ONE far point (odom frame) at 10 Hz on /local_voxel_map/obstacles.

The A* planner refuses to plan without a fresh obstacle cloud; this satisfies it while
reporting "no obstacles" (the point is beyond the local grid and max_lidar_range).
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


class DummyObstacles(Node):
    def __init__(self):
        super().__init__('dummy_obstacles')
        self.declare_parameter('topic', '/local_voxel_map/obstacles')
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('point', [100.0, 100.0, 1.0])
        self._frame = self.get_parameter('frame_id').value
        self._pt = [float(v) for v in self.get_parameter('point').value]
        self._pub = self.create_publisher(PointCloud2, self.get_parameter('topic').value, 10)
        self.create_timer(1.0 / float(self.get_parameter('rate_hz').value), self._tick)

    def _tick(self):
        h = Header(frame_id=self._frame)
        h.stamp = self.get_clock().now().to_msg()
        self._pub.publish(point_cloud2.create_cloud_xyz32(h, [self._pt]))


def main():
    rclpy.init()
    n = DummyObstacles()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()
