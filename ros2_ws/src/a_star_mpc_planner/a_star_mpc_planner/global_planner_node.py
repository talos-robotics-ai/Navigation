"""
Global planner node — long-horizon routing layer (global+local architecture).

Plans a LONG route from the robot to the global goal across a world-fixed,
coarse global costmap that remembers the whole traversed scene (and the safe
corridor the robot came through), then publishes it as ``/global_path``. The
existing local A* (a_star_node) follows a carrot along that path while its own
LIVE, clean costmap + the MPCC handle precise reactive avoidance.

This keeps the two concerns cleanly separated:
  * global costmap  → coarse, persistent, drift-tolerant (hit-thresholded);
                      used ONLY to choose the route. Never fused into the local
                      costmap (that fusion injected ghost obstacles — see
                      docs/planning/A_STAR_MPC_PLANNER.md).
  * local  costmap  → live, clean, reactive; unchanged.

Subscribes:
  /dlio/odom_node/odom        nav_msgs/Odometry   — robot pose (→ position)
  /local_voxel_map/obstacles  sensor_msgs/PointCloud2 — clean ground-removed cloud
  /global_goal                geometry_msgs/PoseStamped — goal

Publishes:
  /global_path                nav_msgs/Path           — the long route to follow
  /global_planner/costmap     nav_msgs/OccupancyGrid  — the global costmap (RViz)
"""

import array

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from a_star_mpc_planner.a_star_planner import AStarPlanner
from a_star_mpc_planner.global_costmap import GlobalCostmap


def _read_xyz(msg: PointCloud2) -> np.ndarray:
    rec = point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
    if not isinstance(rec, np.ndarray):
        rec = np.array(list(rec))
    if rec.size == 0:
        return np.empty((0, 3), dtype=float)
    if rec.dtype.names:
        return np.column_stack([rec['x'], rec['y'], rec['z']]).astype(float)
    return rec.astype(float).reshape(-1, 3)


class GlobalPlannerNode(Node):

    def __init__(self):
        super().__init__('global_planner_node')

        self.declare_parameter('odom_topic', '/dlio/odom_node/odom')
        self.declare_parameter('obstacle_topic', '/local_voxel_map/obstacles')
        self.declare_parameter('global_goal_topic', '/global_goal')
        self.declare_parameter('global_path_topic', '/global_path')
        self.declare_parameter('global_replan_hz', 1.0)
        self.declare_parameter('global_reso', 0.20)
        self.declare_parameter('global_half_width', 25.0)
        self.declare_parameter('robot_radius', 0.35)
        self.declare_parameter('inflation_radius', 0.70)
        self.declare_parameter('global_hit_threshold', 2.0)
        self.declare_parameter('global_decay', 0.997)
        self.declare_parameter('global_free_radius', 0.40)
        self.declare_parameter('obstacle_cost_weight', 50.0)
        self.declare_parameter('goal_reached_radius', 0.25)
        self.declare_parameter('max_range', 25.0)

        self._max_range = float(self.get_parameter('max_range').value)
        self._goal_reached_radius = float(self.get_parameter('goal_reached_radius').value)

        self._costmap = GlobalCostmap(
            reso=float(self.get_parameter('global_reso').value),
            half_width=float(self.get_parameter('global_half_width').value),
            robot_radius=float(self.get_parameter('robot_radius').value),
            inflation_radius=float(self.get_parameter('inflation_radius').value),
            hit_threshold=float(self.get_parameter('global_hit_threshold').value),
            decay=float(self.get_parameter('global_decay').value),
            free_radius=float(self.get_parameter('global_free_radius').value),
        )
        self._planner = AStarPlanner(
            obstacle_threshold=0.5,
            obstacle_cost_weight=float(self.get_parameter('obstacle_cost_weight').value),
            step_over_height=0.0,
        )

        self._pose_xy: np.ndarray | None = None
        self._goal_xy: np.ndarray | None = None
        self._latest_obs: np.ndarray | None = None
        self._frame = 'odom'

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=10)
        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1)

        self.create_subscription(
            Odometry, str(self.get_parameter('odom_topic').value), self._odom_cb, odom_qos)
        self.create_subscription(
            PointCloud2, str(self.get_parameter('obstacle_topic').value),
            self._obs_cb, sensor_qos)
        self.create_subscription(
            PoseStamped, str(self.get_parameter('global_goal_topic').value),
            self._goal_cb, 10)

        self._path_pub = self.create_publisher(
            Path, str(self.get_parameter('global_path_topic').value), 10)
        self._costmap_pub = self.create_publisher(
            OccupancyGrid, '/global_planner/costmap', latched_qos)
        # Raw CONFIRMED obstacle cells (pre-inflation, anti-ghost-gated) for the
        # local A*'s global+local fusion mode. Latched so a late-joining local
        # planner gets the map immediately. Coarse grid → a few k points at 1 Hz.
        self._known_obs_pub = self.create_publisher(
            PointCloud2, '/global_planner/known_obstacles', latched_qos)

        rate = float(self.get_parameter('global_replan_hz').value)
        self.create_timer(1.0 / max(0.1, rate), self._replan_cb)
        self.get_logger().info(
            f'global planner ready | grid={2*self._costmap.half_width:.0f} m '
            f'@ {self._costmap.reso} m/cell | replan {rate} Hz')

    # ── Callbacks ──────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        self._frame = msg.header.frame_id or 'odom'
        self._pose_xy = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])

    def _obs_cb(self, msg: PointCloud2):
        try:
            pts = _read_xyz(msg)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f'obstacle parse error: {exc}', throttle_duration_sec=5.0)
            return
        if len(pts) > 0:
            self._latest_obs = pts[:, :2]

    def _goal_cb(self, msg: PoseStamped):
        self._goal_xy = np.array([msg.pose.position.x, msg.pose.position.y])

    # ── Global replanning ──────────────────────────────────────────────

    def _replan_cb(self):
        if self._pose_xy is None or self._goal_xy is None:
            return

        # Range-limit obstacles to the global window around the robot.
        obs = self._latest_obs
        if obs is not None and len(obs) > 0:
            d = np.hypot(obs[:, 0] - self._pose_xy[0], obs[:, 1] - self._pose_xy[1])
            obs = obs[d < self._max_range]

        self._costmap.update(obs, self._pose_xy)
        self._costmap.build()

        # Publish the global costmap for RViz (row-major: x=col, y=row → transpose).
        if self._costmap.gmap is not None:
            ogm = OccupancyGrid()
            ogm.header.stamp = self.get_clock().now().to_msg()
            ogm.header.frame_id = self._frame
            ogm.info.resolution = self._costmap.reso
            ogm.info.width = self._costmap.cells
            ogm.info.height = self._costmap.cells
            ogm.info.origin.position.x = self._costmap.minx
            ogm.info.origin.position.y = self._costmap.miny
            ogm.info.origin.orientation.w = 1.0
            scaled = (self._costmap.gmap.T.flatten() * 100.0).clip(0, 100).astype(np.int8)
            ogm.data = array.array('b', scaled.tobytes())
            self._costmap_pub.publish(ogm)

        # Publish the raw confirmed-hit cells for the local planner's fusion mode.
        hits = self._costmap.confirmed_hit_points()
        if hits is not None:
            hdr = Header(stamp=self.get_clock().now().to_msg(), frame_id=self._frame)
            pts3 = np.column_stack([hits, np.zeros(len(hits))]).astype(np.float32)
            self._known_obs_pub.publish(point_cloud2.create_cloud_xyz32(hdr, pts3))

        dist_to_goal = float(np.linalg.norm(self._pose_xy - self._goal_xy))
        if dist_to_goal <= self._goal_reached_radius:
            return  # at goal — let the local layer's goal-reached stop handle it

        path = self._planner.plan(self._costmap, self._pose_xy, self._goal_xy)
        if not path:
            self.get_logger().warning('[GLOBAL] no path to goal', throttle_duration_sec=3.0)
            return

        stamp = self.get_clock().now().to_msg()
        path_msg = Path()
        path_msg.header.stamp = stamp
        path_msg.header.frame_id = self._frame
        for wx, wy in path:
            ps = PoseStamped()
            ps.header.stamp = stamp
            ps.header.frame_id = self._frame
            ps.pose.position.x = float(wx)
            ps.pose.position.y = float(wy)
            ps.pose.orientation.w = 1.0
            path_msg.poses.append(ps)
        self._path_pub.publish(path_msg)
        self.get_logger().info(
            f'[GLOBAL] route: {len(path)} wpts, dist_to_goal={dist_to_goal:.1f} m',
            throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
