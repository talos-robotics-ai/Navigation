"""NavigationInterface — the single, robot-agnostic seam to the geometric stack.

Every higher layer (skills, coordinator, agent) talks to navigation ONLY through
this interface; nothing above it touches ROS topics directly. It wraps the
existing contract the A*+MPC planner already speaks:

    goal  IN  : /global_goal        geometry_msgs/PoseStamped
    state OUT : /navigation/state   std_msgs/String  (STOPPED|NAVIGATING|GOAL_REACHED|...)
    stop  IN  : /estop              std_msgs/Bool

Keeping this seam thin means the whole geometric planner can be swapped, tuned,
or run off-board without any skill/agent code changing — and it is what lets us
TEST low-level navigation on its own before adding language/VLM on top.
"""
from __future__ import annotations

import math
from typing import Optional

from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class NavigationInterface:
    """Publish goals to / read state from the geometric navigation stack."""

    # Mirror the states the planner publishes on /navigation/state.
    UNKNOWN = 'UNKNOWN'
    STOPPED = 'STOPPED'
    NAVIGATING = 'NAVIGATING'
    GOAL_REACHED = 'GOAL_REACHED'

    def __init__(
        self,
        node,
        goal_topic: str = '/global_goal',
        state_topic: str = '/navigation/state',
        estop_topic: str = '/estop',
        odom_topic: str = '/dlio/odom_node/odom',
        frame: str = 'odom',
    ):
        self._node = node
        self._frame = frame
        self._state = self.UNKNOWN
        self._goal: Optional[tuple] = None
        self._pose: Optional[tuple] = None   # (x, y) latest robot pose

        self._goal_pub = node.create_publisher(PoseStamped, goal_topic, 10)
        self._estop_pub = node.create_publisher(Bool, estop_topic, 10)
        node.create_subscription(String, state_topic, self._on_state, 10)
        node.create_subscription(Odometry, odom_topic, self._on_odom, 10)

    # ── Commands ───────────────────────────────────────────────────────
    def set_goal(self, x: float, y: float, yaw: float = 0.0) -> None:
        """Send a metric goal in the world (`odom`) frame."""
        ps = PoseStamped()
        ps.header.frame_id = self._frame
        ps.header.stamp = self._node.get_clock().now().to_msg()
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.orientation = yaw_to_quaternion(float(yaw))
        self._goal = (float(x), float(y), float(yaw))
        self._goal_pub.publish(ps)
        self._node.get_logger().info(f'[nav] goal -> ({x:.2f}, {y:.2f}, {yaw:.2f} rad)')

    def set_goal_pose(self, pose: PoseStamped) -> None:
        pose.header.frame_id = pose.header.frame_id or self._frame
        self._goal = (pose.pose.position.x, pose.pose.position.y, 0.0)
        self._goal_pub.publish(pose)

    def cancel(self) -> None:
        """Engage the software stop (the same /estop the gait bridge honours)."""
        self._estop_pub.publish(Bool(data=True))
        self._node.get_logger().info('[nav] cancel -> e-stop engaged')

    def resume(self) -> None:
        self._estop_pub.publish(Bool(data=False))

    # ── State ──────────────────────────────────────────────────────────
    def get_state(self) -> str:
        return self._state

    def is_goal_reached(self) -> bool:
        return self._state == self.GOAL_REACHED

    def is_navigating(self) -> bool:
        return self._state == self.NAVIGATING

    def robot_xy(self) -> Optional[tuple]:
        return self._pose

    def distance_to_goal(self) -> Optional[float]:
        if self._pose is None or self._goal is None:
            return None
        return math.hypot(self._pose[0] - self._goal[0], self._pose[1] - self._goal[1])

    # ── Callbacks ──────────────────────────────────────────────────────
    def _on_state(self, msg: String) -> None:
        self._state = msg.data or self.UNKNOWN

    def _on_odom(self, msg: Odometry) -> None:
        self._pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)
