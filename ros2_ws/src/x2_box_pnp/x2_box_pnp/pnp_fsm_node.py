"""ROS 2 wrapper around the pure PnpFsm. Pure logic lives in fsm.py / geometry.py / keyframes.py.

In : /x2/odom, /x2/crate_pose, /x2/q_arm, /mpc/cmd_vel, /navigation/state, /estop,
     /pnp/start (std_srvs/Trigger service AND std_msgs/Bool topic), /pnp/reset (same)
Out: /x2/cmd_vel_out (ALWAYS at rate_hz, zero when not moving), /x2/arm_cmd, /x2/hand_cmd,
     /global_goal, /pnp/state

/mpc/cmd_vel reaches /x2/cmd_vel_out only through this node (the gate).
"""
import math
import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray, String
from std_srvs.srv import Trigger

from . import geometry as g
from .fsm import Inputs, Params, PnpFsm
from .keyframes import Keyframes


class PnpNode(Node):
    def __init__(self, **node_kw):
        super().__init__('pnp_fsm', **node_kw)
        defaults = Params()
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('keyframes_file', '')
        self._params = Params()
        for k, v in defaults.__dict__.items():
            if k in ('arm_move_s_by_key',):
                continue
            if isinstance(v, tuple):
                v = list(v)
            self.declare_parameter(k, v)
            val = self.get_parameter(k).value
            setattr(self._params, k, tuple(val) if isinstance(v, list) and k == 'place_pose' else val)
        kf_file = self.get_parameter('keyframes_file').value or os.path.join(
            get_package_share_directory('x2_box_pnp'), 'config', 'arm_keyframes.yaml')
        with open(kf_file) as f:
            frames = yaml.safe_load(f)['keyframes']
        self._fsm = PnpFsm(self._params, Keyframes(frames, self._params.arm_default),
                           log=lambda m: self.get_logger().info(m))

        self._robot = self._crate = self._q_arm = self._planner = None
        self._crate_rx = None
        self._crate_frames = 0
        self._planner_rx = None
        self._nav_state = ''
        self._estop = False

        self.create_subscription(Odometry, '/x2/odom', self._on_odom, 10)
        self.create_subscription(PoseStamped, '/x2/crate_pose', self._on_crate, 10)
        self.create_subscription(Float64MultiArray, '/x2/q_arm', self._on_q, 10)
        self.create_subscription(Twist, '/mpc/cmd_vel', self._on_planner, 10)
        self.create_subscription(String, '/navigation/state', self._on_nav, 10)
        self.create_subscription(Bool, '/estop', lambda m: setattr(self, '_estop', bool(m.data)), 10)
        self.create_subscription(Bool, '/pnp/start', lambda m: m.data and self._start(), 10)
        self.create_subscription(Bool, '/pnp/reset', lambda m: m.data and self._fsm.request_reset(), 10)
        self.create_service(Trigger, '/pnp/start', self._srv_start)
        self.create_service(Trigger, '/pnp/reset', self._srv_reset)
        self._cmd_pub = self.create_publisher(Twist, '/x2/cmd_vel_out', 10)
        self._arm_pub = self.create_publisher(Float64MultiArray, '/x2/arm_cmd', 10)
        self._hand_pub = self.create_publisher(Float64MultiArray, '/x2/hand_cmd', 10)
        self._goal_pub = self.create_publisher(PoseStamped, '/global_goal', 10)
        self._state_pub = self.create_publisher(String, '/pnp/state', 10)
        self.create_timer(1.0 / float(self.get_parameter('rate_hz').value), self._tick)
        self.get_logger().info(f'PNP: FSM up in {self._fsm.state.name} (auto_start={self._params.auto_start})')

    def _start(self):
        self.get_logger().info('PNP: start requested')
        self._fsm.request_start()

    def _srv_start(self, _req, resp):
        self._start()
        resp.success, resp.message = True, f'start latched (state {self._fsm.state.name})'
        return resp

    def _srv_reset(self, _req, resp):
        self.get_logger().info('PNP: reset requested')
        self._fsm.request_reset()
        resp.success, resp.message = True, 'reset queued'
        return resp

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_odom(self, m):
        o = m.pose.pose.orientation
        self._robot = (m.pose.pose.position.x, m.pose.pose.position.y, g.yaw_from_quat(o.x, o.y, o.z, o.w))

    def _on_crate(self, m):
        o = m.pose.orientation
        self._crate = (m.pose.position.x, m.pose.position.y, g.yaw_from_quat(o.x, o.y, o.z, o.w))
        self._crate_rx = self._now()
        self._crate_frames += 1

    def _on_q(self, m):
        self._q_arm = list(m.data)

    def _on_planner(self, m):
        self._planner = (m.linear.x, m.angular.z)
        self._planner_rx = self._now()

    def _on_nav(self, m):
        self._nav_state = m.data

    def _tick(self):
        t = self._now()
        inp = Inputs(
            t=t, robot=self._robot, crate=self._crate,
            crate_age=math.inf if self._crate_rx is None else t - self._crate_rx,
            crate_frames=self._crate_frames, q_arm=self._q_arm, planner_cmd=self._planner,
            planner_age=math.inf if self._planner_rx is None else t - self._planner_rx,
            nav_state=self._nav_state, estop=self._estop)
        out = self._fsm.update(inp)
        tw = Twist()
        tw.linear.x, tw.angular.z = float(out.vx), float(out.wz)      # vy = 0 always
        self._cmd_pub.publish(tw)                                    # also zeros: explicit balancing
        self._arm_pub.publish(Float64MultiArray(data=[float(v) for v in (out.arm or [])]))
        self._hand_pub.publish(Float64MultiArray(data=[float(v) for v in (out.hand or [])]))
        if out.goal is not None:
            gm = PoseStamped()
            gm.header.frame_id = 'odom'
            gm.header.stamp = self.get_clock().now().to_msg()
            gm.pose.position.x, gm.pose.position.y = out.goal[0], out.goal[1]
            q = g.quat_from_yaw(out.goal[2])
            gm.pose.orientation.x, gm.pose.orientation.y, gm.pose.orientation.z, gm.pose.orientation.w = q
            self._goal_pub.publish(gm)
        self._state_pub.publish(String(data=out.state.name))


def main():
    rclpy.init()
    n = PnpNode()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()
