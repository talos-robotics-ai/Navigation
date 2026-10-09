"""Laptop side of the navigation relay (the planner stack runs next to it). See nav_relay_server.py.

    PC2 --> /x2/odom, /x2/cam_pose (+ TF odom -> rgbd_head_front), /x2/crate_pose (stamped on arrival), /kilvo/cloud_registered_ds (PointCloud2, frame odom)
    /x2/cmd_vel_out (the pnp FSM gate's output) --> {"cmd"} at 20 Hz; zeros when the local Twist is stale
    (> stale_s) or never arrived. /estop --> {"estop"} (a true is re-sent every second and on reconnect).

Connects to host:port and reconnects forever; heartbeats at 1 Hz, a server silent for server_silence_s is dropped
and re-dialled.
"""
import json
import struct
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool, Header, String

from . import nav_relay_proto as proto
from .nav_relay_link import LinkClient

CMD_HZ = 20.0
XYZ_FIELDS = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, n in enumerate('xyz')]


class NavRelayClient(Node):
    def __init__(self, **kw):
        super().__init__('nav_relay_client', **kw)
        P = self.declare_parameter
        P('host', '10.0.1.41')
        P('port', 5596)
        P('odom_topic', '/x2/odom')
        P('crate_topic', '/x2/crate_pose')
        P('cam_topic', '/x2/cam_pose')
        P('cam_frame', 'rgbd_head_front')
        P('publish_tf', True)       # local graph only
        P('cloud_topic', '/kilvo/cloud_registered_ds')
        P('cmd_topic', '/x2/cmd_vel_out')
        P('estop_topic', '/estop')
        P('frame', 'odom')
        P('stale_s', 0.3)
        P('server_silence_s', 3.0)
        P('pad_port', 8771)         # PS5 pad override (pad_walker.py --nav); 0 = off
        g = lambda k: self.get_parameter(k).value  # noqa: E731
        self._frame, self._stale = str(g('frame')), float(g('stale_s'))
        self._silence = float(g('server_silence_s'))
        self._cmd = None      # (vx, vy, wz, t_rx)
        self._estop = False
        self._stat = dict(odom=0, cam=0, crate=0, cloud=0, pts=0, cmd=0)
        self._t_stat = time.monotonic()
        self._odom_pub = self.create_publisher(Odometry, g('odom_topic'), 10)
        self._crate_pub = self.create_publisher(PoseStamped, g('crate_topic'), 10)
        self._cam_pub = self.create_publisher(PoseStamped, g('cam_topic'), 10)
        self._tf = None
        if g('publish_tf'):
            from tf2_ros import TransformBroadcaster
            self._tf = TransformBroadcaster(self)
        self._cam_frame = str(g('cam_frame'))
        self._cloud_pub = self.create_publisher(PointCloud2, g('cloud_topic'), 2)
        self.create_subscription(Twist, g('cmd_topic'), self._on_twist, 10)
        self.create_subscription(Bool, g('estop_topic'), self._on_estop, 10)
        # on-robot walker (relay walker:=onrobot): its phase/gate verdict in, the FSM's engage out
        self._walker_pub = self.create_publisher(String, '/x2/walker_state', 10)
        self.create_subscription(Bool, '/x2/walker_engage',
                                 lambda m: m.data and self._cli.send(proto.encode_frame({'t': 'engage'})), 10)
        self._pad = None
        self._override = None
        self._pnp_state = ''
        self._walker_phase = None
        if int(g('pad_port')) > 0:
            from .pad_input import PadInput
            self._pad = PadInput(port=int(g('pad_port')), stale_s=float(g('stale_s')),
                                 log=lambda m: self.get_logger().info(m))
        self._override_pub = self.create_publisher(Bool, '/x2/pad_override', 10)
        self.create_subscription(String, '/pnp/state', lambda m: setattr(self, '_pnp_state', m.data), 10)
        self._t_pad_status = 0.0
        self.create_timer(1.0 / CMD_HZ, self._cmd_tick)
        self.create_timer(1.0, self._slow_tick)
        self._cli = LinkClient(str(g('host')), int(g('port')), self._on_frame, self._on_connect, self._on_close)
        self.get_logger().info(f'nav relay client -> {g("host")}:{g("port")}; cloud {g("cloud_topic")} frame {self._frame}')

    # ---------------------------------------------------------------- PC2 -> laptop
    def _on_frame(self, h, payload):
        kind = h.get('t')
        try:
            if kind in ('odom', 'crate', 'cam'):
                p, q = h['p'], h['q']
                stamp = self.get_clock().now().to_msg()   # arrival time: the planner's timeouts use it
                if kind == 'odom':
                    o = Odometry()
                    o.header.stamp, o.header.frame_id, o.child_frame_id = stamp, self._frame, 'base_link'
                    pose = o.pose.pose
                    pub = self._odom_pub
                else:
                    o = PoseStamped()
                    o.header.stamp, o.header.frame_id = stamp, self._frame
                    pose = o.pose
                    pub = self._crate_pub if kind == 'crate' else self._cam_pub
                pose.position.x, pose.position.y, pose.position.z = map(float, p)
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(float, q)
                pub.publish(o)
                self._stat[kind] += 1
                if kind == 'cam' and self._tf is not None:
                    t = TransformStamped()
                    t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, self._frame, self._cam_frame
                    tr, ro = t.transform.translation, t.transform.rotation
                    tr.x, tr.y, tr.z = map(float, p)
                    ro.x, ro.y, ro.z, ro.w = map(float, q)
                    self._tf.sendTransform(t)
            elif kind == 'walker':
                self._walker_phase = h.get('phase')
                self._walker_pub.publish(String(data=json.dumps(
                    {'phase': h.get('phase'), 'engage_result': h.get('engage_result'), 'age': h.get('age')})))
            elif kind == 'cloud':
                n = int(h['n'])
                proto.unpack_cloud(payload, n)   # validates the size
                m = PointCloud2()
                m.header = Header(stamp=self.get_clock().now().to_msg(), frame_id=self._frame)
                m.height, m.width, m.fields = 1, n, XYZ_FIELDS
                m.is_bigendian, m.point_step, m.row_step, m.is_dense = False, 12, 12 * n, True
                m.data = payload
                self._cloud_pub.publish(m)
                self._stat['cloud'] += 1
                self._stat['pts'] += n
        except (KeyError, TypeError, ValueError, struct.error, proto.FrameError) as e:
            self.get_logger().warn(f'bad {kind} frame: {e}', throttle_duration_sec=5)

    def _on_connect(self, link):
        self.get_logger().info(f'connected to PC2 {link.peer}')
        if self._estop:
            self._send_estop(True)

    def _on_close(self, link):
        self.get_logger().warn(f'link to {link.peer} lost; retrying (PC2 commands zero by itself within 0.3 s)')

    # ---------------------------------------------------------------- laptop -> PC2
    def _on_twist(self, m):
        self._cmd = (m.linear.x, m.linear.y, m.angular.z, time.monotonic())

    def _on_estop(self, m):
        self._estop = bool(m.data)
        self._send_estop(self._estop)

    def _send_estop(self, v):
        self._cli.send(proto.encode_frame({'t': 'estop', 'value': bool(v)}))

    def _cmd_tick(self):
        c = self._cmd
        vx, vy, wz = proto.local_cmd(None if c is None else (c[0], c[1], c[2], time.monotonic() - c[3]), self._stale)
        pad = self._pad.get() if self._pad is not None else None
        override = pad is not None
        if override:                                   # L1 held: the pad wins over navigation
            vx, vy, wz = pad[0], 0.0, pad[1]
        if override != self._override:
            self._override = override
            self._override_pub.publish(Bool(data=override))
            self.get_logger().warn('PAD OVERRIDE: the pad drives, navigation paused (reset to resume)'
                                   if override else 'pad released: the robot stands')
        if self._pad is not None:
            if self._pad.take_engage():
                self._cli.send(proto.encode_frame({'t': 'engage'}))
                self.get_logger().info('engage from the pad')
            now = time.monotonic()
            if now - self._t_pad_status > 0.2:
                self._t_pad_status = now
                self._pad.send_status(phase=self._walker_phase, pnp=self._pnp_state, override=override,
                                      vx=round(vx, 3), wz=round(wz, 3))
        if self._cli.send(proto.cmd_frame(vx, vy, wz), replace_key='cmd'):
            self._stat['cmd'] += 1

    def _slow_tick(self):
        link = self._cli.link
        if link is not None and link.alive:
            link.send(proto.encode_frame({'t': 'hb', 'stamp': time.time()}), replace_key='hb')
            if time.monotonic() - link.t_rx > self._silence:
                self.get_logger().warn(f'PC2 silent for {self._silence} s: reconnecting')
                link.close()
            if self._estop:
                self._send_estop(True)
        now = time.monotonic()
        if now - self._t_stat >= 5.0:
            dt, s = now - self._t_stat, self._stat
            self._t_stat = now
            self.get_logger().info(
                f'relay {"link " + link.peer if link and link.alive else "DISCONNECTED"} rx/s odom {s["odom"]/dt:.1f} cam {s["cam"]/dt:.1f} '
                f'crate {s["crate"]/dt:.1f} cloud {s["cloud"]/dt:.1f} ({s["pts"]/max(s["cloud"], 1):.0f} pts) | tx cmd {s["cmd"]/dt:.1f}/s'
                + (' | ESTOP' if self._estop else ''))
            for k in s:
                s[k] = 0

    def on_shutdown(self):
        if self._pad is not None:
            self._pad.close()
        self._cli.close()


def main(args=None):
    rclpy.init(args=args)
    n = NavRelayClient()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.on_shutdown()
        n.destroy_node()
        rclpy.try_shutdown()
