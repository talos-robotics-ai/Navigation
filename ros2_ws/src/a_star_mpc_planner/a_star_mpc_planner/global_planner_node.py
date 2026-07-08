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

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
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
        # T1 — grow the persistent map to the whole scene (cap, then roll).
        self.declare_parameter('global_max_half_width', 80.0)
        # T2 — dead-end / stuck memory.
        self.declare_parameter('global_penalty_decay', 0.985)   # per-build fade
        self.declare_parameter('global_penalty_max', 4.0)
        self.declare_parameter('stuck_time_s', 6.0)             # no-progress window -> penalize
        self.declare_parameter('stuck_progress_eps', 0.30)      # min dist-to-goal gain = progress (m)
        self.declare_parameter('stuck_penalty_ahead', 1.5)      # stamp this far ahead toward goal (m)
        self.declare_parameter('stuck_penalty_radius', 1.2)     # penalty disk radius (m)
        self.declare_parameter('stuck_penalty_amount', 2.0)     # penalty added per stuck event
        # T3 — re-anchor accumulated memory on a DLIO odom discontinuity.
        self.declare_parameter('odom_jump_threshold', 0.30)     # per-msg pose step = jump (m)
        # 2.5D height-graded cost — real structure (tall) is lethal + inflated; a
        # measured LOW return is soft cost, not a hard block (curbs / low clutter / noise).
        self.declare_parameter('global_use_height_cost', True)
        self.declare_parameter('global_foot_offset', 0.70)      # sensor(odom z) -> foot drop (m)
        self.declare_parameter('global_low_height', 0.20)       # below this = soft, not lethal (m)
        self.declare_parameter('global_low_cost', 0.45)         # soft cost applied to low returns
        # ── STATIC-MAP mode (OFF by default; online map is the default) ──
        # When true, route over a pre-built OccupancyGrid (nav2_map_server /map) in the
        # MAP frame, with the robot pose from a map-frame localizer (nav2_amcl /amcl_pose).
        # Live odom-frame obstacles are NOT fused here (the local planner handles dynamics);
        # requires a map->odom localization (AMCL) to be running. See planner_params yaml.
        self.declare_parameter('use_static_map', False)
        self.declare_parameter('static_map_topic', '/map')
        self.declare_parameter('map_pose_topic', '/amcl_pose')
        self.declare_parameter('static_occupied_thresh', 50)    # 0-100; >= is an obstacle

        self._max_range = float(self.get_parameter('max_range').value)
        self._goal_reached_radius = float(self.get_parameter('goal_reached_radius').value)
        self._stuck_time_s = float(self.get_parameter('stuck_time_s').value)
        self._stuck_eps = float(self.get_parameter('stuck_progress_eps').value)
        self._stuck_ahead = float(self.get_parameter('stuck_penalty_ahead').value)
        self._stuck_radius = float(self.get_parameter('stuck_penalty_radius').value)
        self._stuck_amount = float(self.get_parameter('stuck_penalty_amount').value)
        self._jump_threshold = float(self.get_parameter('odom_jump_threshold').value)

        self._costmap = GlobalCostmap(
            reso=float(self.get_parameter('global_reso').value),
            half_width=float(self.get_parameter('global_half_width').value),
            robot_radius=float(self.get_parameter('robot_radius').value),
            inflation_radius=float(self.get_parameter('inflation_radius').value),
            hit_threshold=float(self.get_parameter('global_hit_threshold').value),
            decay=float(self.get_parameter('global_decay').value),
            free_radius=float(self.get_parameter('global_free_radius').value),
            max_half_width=float(self.get_parameter('global_max_half_width').value),
            penalty_decay=float(self.get_parameter('global_penalty_decay').value),
            penalty_max=float(self.get_parameter('global_penalty_max').value),
            use_height_cost=bool(self.get_parameter('global_use_height_cost').value),
            foot_offset=float(self.get_parameter('global_foot_offset').value),
            low_height=float(self.get_parameter('global_low_height').value),
            low_cost=float(self.get_parameter('global_low_cost').value),
        )
        self._planner = AStarPlanner(
            obstacle_threshold=0.5,
            obstacle_cost_weight=float(self.get_parameter('obstacle_cost_weight').value),
            step_over_height=0.0,
        )

        self._pose_xy: np.ndarray | None = None
        self._pose_z: float | None = None        # for 2.5D height reference (foot)
        self._goal_xy: np.ndarray | None = None
        self._latest_obs: np.ndarray | None = None
        self._frame = 'odom'
        # T2 stuck tracking (per goal): best distance-to-goal and when it happened.
        self._goal_best_d: float | None = None
        self._goal_best_t: float = 0.0

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

        # Static-map mode (default OFF): route over a pre-built /map in the map frame,
        # pose from a map-frame localizer. Only wired when the flag is set.
        self._use_static = bool(self.get_parameter('use_static_map').value)
        self._static_thresh = int(self.get_parameter('static_occupied_thresh').value)
        if self._use_static:
            self.create_subscription(
                OccupancyGrid, str(self.get_parameter('static_map_topic').value),
                self._on_static_map, latched_qos)
            self.create_subscription(
                PoseWithCovarianceStamped, str(self.get_parameter('map_pose_topic').value),
                self._on_map_pose, 10)
            self.get_logger().warning(
                'STATIC-MAP mode ON: routing over a pre-built map — needs map_server + AMCL.')

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
        if self._use_static:
            return  # static mode: pose + frame come from /amcl_pose + /map instead
        self._frame = msg.header.frame_id or 'odom'
        xy = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        # T3: a per-message pose step larger than a walking robot can make is a DLIO
        # discontinuity (loop closure) — re-anchor the accumulated map by that delta
        # so the jump doesn't smear memory into ghost walls.
        if self._pose_xy is not None and self._costmap.ready:
            dx = float(xy[0] - self._pose_xy[0])
            dy = float(xy[1] - self._pose_xy[1])
            if (dx * dx + dy * dy) ** 0.5 > self._jump_threshold:
                self._costmap.shift(dx, dy)
                self.get_logger().warning(
                    f'[GLOBAL] odom jump {(dx * dx + dy * dy) ** 0.5:.2f} m — re-anchored global map',
                    throttle_duration_sec=1.0)
        self._pose_xy = xy
        self._pose_z = float(msg.pose.pose.position.z)

    def _obs_cb(self, msg: PointCloud2):
        try:
            pts = _read_xyz(msg)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f'obstacle parse error: {exc}', throttle_duration_sec=5.0)
            return
        if len(pts) > 0:
            self._latest_obs = pts[:, :3]   # keep z for the 2.5D height layer

    def _goal_cb(self, msg: PoseStamped):
        self._goal_xy = np.array([msg.pose.position.x, msg.pose.position.y])
        self._goal_best_d = None   # new goal → reset stuck tracking

    # ── Static-map mode callbacks (only active when use_static_map) ─────
    def _on_static_map(self, msg: OccupancyGrid):
        self._frame = msg.header.frame_id or 'map'
        self._costmap.load_static_map(
            msg.data, msg.info.width, msg.info.height, msg.info.resolution,
            msg.info.origin.position.x, msg.info.origin.position.y,
            occupied_thresh=self._static_thresh)
        self.get_logger().info(
            f'[GLOBAL] static map loaded: {msg.info.width}x{msg.info.height} '
            f'@ {msg.info.resolution:.2f} m/cell, frame={self._frame}')

    def _on_map_pose(self, msg: PoseWithCovarianceStamped):
        self._pose_xy = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        self._pose_z = float(msg.pose.pose.position.z)

    # ── Global replanning ──────────────────────────────────────────────

    def _replan_cb(self):
        if self._pose_xy is None or self._goal_xy is None:
            return
        if self._use_static and not self._costmap.ready:
            return  # waiting for the static map to load

        # Range-limit obstacles to the window. In STATIC mode we do NOT fuse the live
        # odom-frame cloud into the map-frame static grid — the local planner handles
        # dynamics; the global map is the pre-built structure.
        obs = None
        if not self._use_static:
            obs = self._latest_obs
            if obs is not None and len(obs) > 0:
                d = np.hypot(obs[:, 0] - self._pose_xy[0], obs[:, 1] - self._pose_xy[1])
                obs = obs[d < self._max_range]

        self._costmap.update(obs, self._pose_xy, self._pose_z)

        # T2: dead-end / stuck memory. Track progress toward the goal; if it stalls
        # for stuck_time_s, stamp a decaying penalty just ahead (toward the goal) so
        # the global A* below reroutes around the corridor that isn't working. The
        # penalty is stamped BEFORE build() so it takes effect on this cycle's route.
        dist_to_goal = float(np.linalg.norm(self._pose_xy - self._goal_xy))
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._goal_best_d is None or dist_to_goal < self._goal_best_d - self._stuck_eps:
            self._goal_best_d = dist_to_goal
            self._goal_best_t = now
        elif (now - self._goal_best_t) > self._stuck_time_s:
            gd = self._goal_xy - self._pose_xy
            n = float(np.hypot(gd[0], gd[1]))
            if n > 1e-3:
                ahead = self._pose_xy + (gd / n) * self._stuck_ahead
                self._costmap.stamp_penalty(ahead, self._stuck_radius, self._stuck_amount)
                self.get_logger().warning(
                    f'[GLOBAL] no progress for {self._stuck_time_s:.0f}s — penalized dead-end ahead, rerouting',
                    throttle_duration_sec=2.0)
            self._goal_best_t = now   # reset the window so we don't stamp every cycle

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
