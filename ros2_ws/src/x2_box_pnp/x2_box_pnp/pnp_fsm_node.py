"""ROS 2 wrapper around the pure PnpFsm. Pure logic lives in fsm.py / geometry.py / keyframes.py.

In : /x2/odom, /x2/crate_pose, /x2/q_arm, /mpc/cmd_vel, /navigation/state, /estop,
     /x2/walker_state (String, JSON {"phase", "engage_result"} from the walker; bring-up only),
     /pnp/start (std_srvs/Trigger service AND std_msgs/Bool topic), /pnp/reset (same)
Out: /x2/cmd_vel_out (ALWAYS at rate_hz, zero when not moving), /x2/arm_cmd, /x2/hand_cmd,
     /global_goal, /pnp/state, /x2/walker_engage (Bool true, one-shot, bring-up only)
     /x2/goal (PoseStamped, debug goal; frame "odom", or "crate" = relative to the crate estimate: e.g. click
     a pre-grasp pose in Foxglove's 3D panel with the display frame set to "crate"), /x2/pad_override (Bool)
Grasp pose: while SETTLE (robot standing at the pre-grasp pose) every confirmed 6D crate pose is collected; when
SETTLE ends they are fused (grasp_pose.fuse: median position, symmetry-aligned quaternion mean) and published
latched on /x2/crate_grasp_pose (odom) and /x2/crate_grasp_pose_base (frame base_link: pelvis, x fwd, y left).
TF out: odom -> crate (the FSM's averaged crate estimate), so Foxglove can display and publish in "crate".
Triggers (files in trigger_dir): start, reset, estop, clear, engage, released.

/mpc/cmd_vel reaches /x2/cmd_vel_out only through this node (the gate).
"""
import json
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
from .fsm import Inputs, Params, PnpFsm, State
from .grasp_pose import fuse, to_base
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
        self._walker_phase = self._engage_result = None
        self.create_subscription(String, '/x2/walker_state', self._on_walker, 10)
        self._engage_pub = self.create_publisher(Bool, '/x2/walker_engage', 10)
        self._manual = False
        self.create_subscription(Bool, '/x2/pad_override', lambda m: setattr(self, '_manual', bool(m.data)), 10)
        self.create_subscription(PoseStamped, '/x2/goal', self._on_goal, 10)
        from tf2_ros import TransformBroadcaster
        self._tf = TransformBroadcaster(self)
        self.create_subscription(Bool, '/pnp/start', lambda m: m.data and self._start(), 10)
        self.create_subscription(Bool, '/pnp/reset', lambda m: m.data and self._fsm.request_reset(), 10)
        self.create_service(Trigger, '/pnp/start', self._srv_start)
        self.create_service(Trigger, '/pnp/reset', self._srv_reset)
        self._cmd_pub = self.create_publisher(Twist, '/x2/cmd_vel_out', 10)
        # Operator estop/clear (file triggers) is also announced on /estop, so it reaches whatever consumes it
        # (mc_velocity_node, the PC2 relay) and not just this FSM. We subscribe to it too: idempotent.
        self._estop_pub = self.create_publisher(Bool, '/estop', 10)
        self._arm_pub = self.create_publisher(Float64MultiArray, '/x2/arm_cmd', 10)
        self._hand_pub = self.create_publisher(Float64MultiArray, '/x2/hand_cmd', 10)
        self._goal_pub = self.create_publisher(PoseStamped, '/global_goal', 10)
        self._state_pub = self.create_publisher(String, '/pnp/state', 10)
        from rclpy.qos import DurabilityPolicy, QoSProfile
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._grasp_pub = self.create_publisher(PoseStamped, '/x2/crate_grasp_pose', latched)
        self._grasp_base_pub = self.create_publisher(PoseStamped, '/x2/crate_grasp_pose_base', latched)
        self._grasp_samples = []
        self._prev_state = None
        self._robot_z = 0.0
        self.create_timer(1.0 / float(self.get_parameter('rate_hz').value), self._tick)
        # File triggers, so the operator can start/stop on PC2 without a ros2 CLI process joining
        # the vendor DDS graph (a new participant there has made a standing X2 fall). Each file
        # is consumed (deleted) when seen: <dir>/start, <dir>/reset, <dir>/estop, <dir>/clear.
        self.declare_parameter('trigger_dir', '')
        self._trigger_dir = os.path.expanduser(self.get_parameter('trigger_dir').value or '')
        if self._trigger_dir:
            os.makedirs(self._trigger_dir, exist_ok=True)
            self.create_timer(0.2, self._poll_triggers)
            self.get_logger().info(f'PNP: file triggers in {self._trigger_dir}')
        self.get_logger().info(f'PNP: FSM up in {self._fsm.state.name} (auto_start={self._params.auto_start})')

    def _poll_triggers(self):
        for name in ('estop', 'clear', 'reset', 'start', 'engage', 'released'):
            path = os.path.join(self._trigger_dir, name)
            if not os.path.exists(path):
                continue
            try:
                os.remove(path)
            except OSError:
                pass
            self.get_logger().info(f'PNP: trigger file {name}')
            if name in ('estop', 'clear'):
                self._estop = name == 'estop'
                self._estop_pub.publish(Bool(data=self._estop))
            elif name == 'reset':
                self._fsm.request_reset()
            elif name == 'engage':
                self._fsm.request_engage()
            elif name == 'released':
                self._fsm.request_released()
            else:
                self._start()

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
        self._robot_z = m.pose.pose.position.z
        o = m.pose.pose.orientation
        self._robot = (m.pose.pose.position.x, m.pose.pose.position.y, g.yaw_from_quat(o.x, o.y, o.z, o.w))

    def _on_crate(self, m):
        if self._fsm.state == State.SETTLE:
            p, q = m.pose.position, m.pose.orientation
            self._grasp_samples.append(((p.x, p.y, p.z), (q.x, q.y, q.z, q.w)))
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

    def _on_goal(self, m):
        o = m.pose.orientation
        gx, gy, gyaw = m.pose.position.x, m.pose.position.y, g.yaw_from_quat(o.x, o.y, o.z, o.w)
        frame = (m.header.frame_id or 'odom').lstrip('/')
        if frame == 'crate':
            c = self._fsm._crate_est
            if c is None:
                self.get_logger().warn('PNP: goal in "crate" ignored: no crate estimate yet')
                return
            cs, sn = math.cos(c[2]), math.sin(c[2])
            gx, gy, gyaw = c[0] + cs * gx - sn * gy, c[1] + sn * gx + cs * gy, c[2] + gyaw
        elif frame != 'odom':
            self.get_logger().warn(f'PNP: goal frame {frame!r} not supported (odom | crate)')
            return
        self.get_logger().info(f'PNP: goal from /x2/goal ({frame}) -> odom ({gx:.2f}, {gy:.2f}, '
                               f'yaw {math.degrees(gyaw):.0f} deg)')
        self._fsm.request_goal((gx, gy, gyaw))

    def _publish_grasp_pose(self):
        r = fuse(self._grasp_samples)
        if r is None or self._robot is None:
            self.get_logger().warn('PNP: no grasp pose (no crate samples while settled)')
            return
        p, q, st = r
        now = self.get_clock().now().to_msg()
        for pub, frame, (pp, qq) in ((self._grasp_pub, 'odom', (p, q)),
                                     (self._grasp_base_pub, 'base_link',
                                      to_base(p, q, (self._robot[0], self._robot[1], self._robot[2], self._robot_z)))):
            m = PoseStamped()
            m.header.stamp, m.header.frame_id = now, frame
            m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, pp)
            m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = map(float, qq)
            pub.publish(m)
        pb, _ = to_base(p, q, (self._robot[0], self._robot[1], self._robot[2], self._robot_z))
        self.get_logger().info(
            f'PNP: GRASP POSE from {st["n"]} detections (spread {st["pos_std_m"] * 100:.1f} cm, '
            f'{st["max_ang_dev_deg"]:.1f} deg): crate bottom-centre {pb[0]:+.3f} fwd {pb[1]:+.3f} left '
            f'{pb[2]:+.3f} up of the pelvis -> /x2/crate_grasp_pose(_base)')

    def _publish_crate_tf(self):
        c = self._fsm._crate_est
        if c is None:
            return
        from geometry_msgs.msg import TransformStamped
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = 'odom', 'crate'
        t.transform.translation.x, t.transform.translation.y = float(c[0]), float(c[1])
        q = g.quat_from_yaw(c[2])
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q
        self._tf.sendTransform(t)

    def _on_walker(self, m):
        try:
            d = json.loads(m.data)
        except ValueError:
            return
        self._walker_phase = d.get('phase')
        self._engage_result = d.get('engage_result')

    def _tick(self):
        t = self._now()
        inp = Inputs(
            t=t, robot=self._robot, crate=self._crate,
            crate_age=math.inf if self._crate_rx is None else t - self._crate_rx,
            crate_frames=self._crate_frames, q_arm=self._q_arm, planner_cmd=self._planner,
            planner_age=math.inf if self._planner_rx is None else t - self._planner_rx,
            nav_state=self._nav_state, estop=self._estop,
            walker_phase=self._walker_phase, engage_result=self._engage_result, manual=self._manual)
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
        if out.engage:
            self.get_logger().info('PNP: engage sent to the walker')
            self._engage_pub.publish(Bool(data=True))
        self._state_pub.publish(String(data=out.state.name))
        if out.state != self._prev_state:
            if out.state == State.SETTLE:
                self._grasp_samples = []
            elif self._prev_state == State.SETTLE and out.state in (State.REACH, State.DONE):
                self._publish_grasp_pose()
            self._prev_state = out.state
        self._publish_crate_tf()


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
