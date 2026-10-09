"""PC2 side of the navigation relay: the cheap half. The planner stack runs on the laptop.

    /x2/odom, /x2/cam_pose, /x2/crate_pose --> TCP --> laptop   (every message; odom and cam rate-limited to odom_max_hz;
                              /x2/cam_pose = T_odom_cam of the head camera, from kilvo_base_odom's FK, for a detector
                              that runs on the laptop; /x2/crate_pose only exists if the detector still runs on PC2)
    /kilvo/cloud_registered   --> TCP --> laptop   (cropped to cloud_radius around /x2/odom, voxel-downsampled
                                                    to cloud_voxel, at cloud_hz: float32 xyz, odom frame)
    laptop --> TCP --> {"cmd": vx vy wz} --> /x2/cmd_vel_out (Twist, 20 Hz)   (mc_velocity_node's input)
                       {"estop": bool}   --> /estop (Bool; true is latched and re-published at 1 Hz)
                       {"engage"}        --> walker {"engage": true}             (walker:=onrobot only)
    walker state {"phase", "engage_result"} --> TCP --> laptop (5 Hz + on change) (walker:=onrobot only)

walker:=onrobot (the on-robot RL walker, packages/x2_pnp/onrobot_walker, 127.0.0.1:8770): the held command
also goes to the walker as {"vx", "wz"} lines at 20 Hz (vy dropped: not trained), and after the zero tail
nothing is sent, so the walker's own 0.3 s stale guard makes it stand. /x2/cmd_vel_out is still published
(nothing consumes it unless walker:=mc).

CPU: the big cloud is subscribed RAW (no ROS deserialization of the ~10 Hz stream); only the one message per
1/cloud_hz that is processed is deserialized, then numpy crop + voxel unique. No ROS timers faster than 20 Hz.

Safety: no command for stale_s (0.3 s) or the client disconnecting -> zero Twists for 0.3 s, then NOTHING is
published; mc_velocity_node then runs its own stale (0.3 s) -> zeros (0.3 s) -> silent. So after the laptop
vanishes the robot is commanded zero for ~1.2 s in total and never anything else. Speeds are clamped to
+-max_v* here too. /x2/cmd_vel_out has no other publisher in relay mode (the FSM runs on the laptop).

One client; a new connection replaces the old. Heartbeats both ways (1 Hz); a client silent for client_silence_s
(5 s) is dropped. Logs one line per 5 s.
"""
import json
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool

from . import nav_relay_proto as proto
from .nav_relay_link import LinkServer

BEST_EFFORT = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=2)
LATCHED = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST, depth=1)
CMD_HZ = 20.0


def pose_header(kind, m):
    p, o = m.pose.position, m.pose.orientation
    return {'t': kind, 'stamp': time.time(), 'p': [p.x, p.y, p.z], 'q': [o.x, o.y, o.z, o.w]}


class NavRelayServer(Node):
    def __init__(self, **kw):
        super().__init__('nav_relay_server', **kw)
        P = self.declare_parameter
        P('host', '0.0.0.0')
        P('port', 5596)
        P('odom_topic', '/x2/odom')
        P('crate_topic', '/x2/crate_pose')
        P('cam_topic', '/x2/cam_pose')
        P('cloud_topic', '/kilvo/cloud_registered')
        P('cmd_topic', '/x2/cmd_vel_out')
        P('estop_topic', '/estop')
        P('odom_max_hz', 50.0)
        P('cloud_hz', 2.0)
        P('cloud_voxel', 0.1)
        P('cloud_radius', 6.0)
        P('stale_s', 0.3)
        P('zero_tail_s', 0.3)
        P('client_silence_s', 5.0)
        P('max_vx', 0.5)
        P('max_vy', 0.3)
        P('max_wz', 0.5)
        P('walker', '')                  # '' | 'onrobot'
        P('walker_host', '127.0.0.1')
        P('walker_port', 8770)
        g = lambda k: self.get_parameter(k).value  # noqa: E731
        self._odom_dt = 1.0 / float(g('odom_max_hz'))
        self._cloud_dt = 1.0 / float(g('cloud_hz'))
        self._voxel, self._radius = float(g('cloud_voxel')), float(g('cloud_radius'))
        self._silence = float(g('client_silence_s'))
        self._hold = proto.CmdHold(float(g('stale_s')), float(g('zero_tail_s')),
                                   (float(g('max_vx')), float(g('max_vy')), float(g('max_wz'))))
        self._center = None
        self._t_odom = self._t_cam = self._t_cloud = 0.0
        self._estop = False
        self._offsets = None
        self._stat = dict(odom=0, cam=0, crate=0, cloud=0, pts=0, cmd=0, pub=0, cloud_in=0, cloud_ms=0.0)
        self._t_stat = time.monotonic()
        self._walker = None
        self._walker_sent = None
        if str(g('walker')) == 'onrobot':
            from .walker_link import WalkerLink
            self._walker = WalkerLink(str(g('walker_host')), int(g('walker_port')),
                                      log=lambda m: self.get_logger().info(m))
        self._cmd_pub = self.create_publisher(Twist, g('cmd_topic'), 10)
        self._estop_pub = self.create_publisher(Bool, g('estop_topic'), LATCHED)
        self.create_subscription(Odometry, g('odom_topic'), self._on_odom, 10)
        self.create_subscription(PoseStamped, g('crate_topic'), self._on_crate, 10)
        self.create_subscription(PoseStamped, g('cam_topic'), self._on_cam, 10)
        self.create_subscription(PointCloud2, g('cloud_topic'), self._on_cloud, BEST_EFFORT, raw=True)
        self.create_timer(1.0 / CMD_HZ, self._cmd_tick)
        self.create_timer(1.0, self._slow_tick)
        self._server = LinkServer(str(g('host')), int(g('port')), self._on_frame, self._on_connect,
                                  self._on_close, log=lambda s: self.get_logger().info(s))
        self.get_logger().info(
            f'nav relay server on {g("host")}:{self._server.port}; odom<={g("odom_max_hz")} Hz, cloud {g("cloud_hz")} Hz '
            f'voxel {self._voxel} m radius {self._radius} m; cmd -> {g("cmd_topic")} (stale {g("stale_s")} s)')

    # ---------------------------------------------------------------- PC2 -> laptop
    def _on_odom(self, m):
        self._center = (m.pose.pose.position.x, m.pose.pose.position.y)
        t = time.monotonic()
        if t - self._t_odom < self._odom_dt:
            return
        self._t_odom = t
        if self._server.send(proto.encode_frame(pose_header('odom', m.pose)), replace_key='odom'):
            self._stat['odom'] += 1

    def _on_cam(self, m):
        t = time.monotonic()
        if t - self._t_cam < self._odom_dt:
            return
        self._t_cam = t
        if self._server.send(proto.encode_frame(pose_header('cam', m)), replace_key='cam'):
            self._stat['cam'] += 1

    def _on_crate(self, m):
        if self._server.send(proto.encode_frame(pose_header('crate', m)), replace_key='crate'):
            self._stat['crate'] += 1

    def _on_cloud(self, raw):
        self._stat['cloud_in'] += 1
        t = time.monotonic()
        if t - self._t_cloud < self._cloud_dt or self._center is None or self._server.link is None:
            return
        self._t_cloud = t
        msg = deserialize_message(raw, PointCloud2)
        if self._offsets is None:
            f = {x.name: x for x in msg.fields}
            if not all(k in f and f[k].datatype == 7 for k in 'xyz') or msg.is_bigendian:
                self.get_logger().error('cloud x/y/z are not little-endian float32: nothing relayed', throttle_duration_sec=10)
                return
            self._offsets = tuple(f[k].offset for k in 'xyz')
        n = msg.width * msg.height
        xyz = proto.cloud_xyz(msg.data, msg.point_step, n, self._offsets)
        out = proto.obstacle_cloud(xyz, self._center, self._radius, self._voxel)
        hdr = {'t': 'cloud', 'stamp': time.time(), 'n': int(len(out))}
        if self._server.send(proto.encode_frame(hdr, proto.pack_cloud(out)), replace_key='cloud'):
            self._stat['cloud'] += 1
            self._stat['pts'] += len(out)
            self._stat['cloud_ms'] += (time.monotonic() - t) * 1e3

    # ---------------------------------------------------------------- laptop -> PC2
    def _on_frame(self, h, _payload):
        kind = h.get('t')
        now = time.monotonic()
        if kind == 'cmd':
            try:
                self._hold.set(h['vx'], h['vy'], h['wz'], now)
                self._stat['cmd'] += 1
            except (KeyError, TypeError, ValueError):
                pass
        elif kind == 'estop':
            self._set_estop(bool(h.get('value', True)))
        elif kind == 'engage' and self._walker is not None:
            ok = self._walker.send({'engage': True})
            self.get_logger().info(f'engage forwarded to the walker (phase {self._walker.phase()}, sent={ok})')

    def _set_estop(self, v):
        if v != self._estop:
            self.get_logger().warn(f'ESTOP {"LATCHED" if v else "released"} (from the laptop)')
        self._estop = v
        self._estop_pub.publish(Bool(data=v))

    def _on_connect(self, link):
        self.get_logger().info(f'client connected: {link.peer}')

    def _on_close(self, link):
        self._hold.drop(time.monotonic())
        self.get_logger().warn(f'client {link.peer} gone: zero Twist for {self._hold.zero_tail_s} s, then silence')

    def _cmd_tick(self):
        self._walker_tick()
        out = self._hold.step(time.monotonic(), self._estop)
        if out is None:
            return
        tw = Twist()
        tw.linear.x, tw.linear.y, tw.angular.z = out
        self._cmd_pub.publish(tw)
        self._stat['pub'] += 1
        if self._walker is not None:
            self._walker.send({'vx': float(out[0]), 'wz': float(out[2])})

    def _walker_tick(self):
        """Forward the walker's phase / gate verdict to the laptop: on change, else at 5 Hz."""
        w, link = self._walker, self._server.link
        if w is None or link is None:
            return
        st = w.state or {}
        cur = {'t': 'walker', 'phase': st.get('phase') if w.connected else 'disconnected',
               'engage_result': st.get('engage_result'), 'age': round(min(w.age(), 99.0), 2)}
        key = (cur['phase'], json.dumps(cur['engage_result'], sort_keys=True))
        now = time.monotonic()
        if key != self._walker_sent or now - getattr(self, '_t_walker_tx', 0.0) > 0.2:
            link.send(proto.encode_frame(cur), replace_key='walker')
            self._walker_sent, self._t_walker_tx = key, now

    def _slow_tick(self):
        link = self._server.link
        if link is not None:
            link.send(proto.encode_frame({'t': 'hb', 'stamp': time.time()}), replace_key='hb')
            if time.monotonic() - link.t_rx > self._silence:
                self.get_logger().warn(f'client {link.peer} silent for {self._silence} s: dropping it')
                link.close()
        if self._estop:
            self._estop_pub.publish(Bool(data=True))
        now = time.monotonic()
        if now - self._t_stat >= 5.0:
            dt, s = now - self._t_stat, self._stat
            self._t_stat = now
            nc = max(s['cloud'], 1)
            self.get_logger().info(
                f'relay client={link.peer if link else "none"} tx/s odom {s["odom"]/dt:.1f} cam {s["cam"]/dt:.1f} crate {s["crate"]/dt:.1f} '
                f'cloud {s["cloud"]/dt:.1f} ({s["pts"]/nc:.0f} pts, {s["cloud_ms"]/nc:.0f} ms each, in {s["cloud_in"]/dt:.1f}/s) | '
                f'rx/s cmd {s["cmd"]/dt:.1f} -> Twist {s["pub"]/dt:.1f}/s | '
                + (f'bytes tx {link.bytes_tx/1e3:.0f} kB rx {link.bytes_rx/1e3:.0f} kB, dropped {link.dropped}' if link else 'no client')
                + (' | ESTOP' if self._estop else ''))
            for k in s:
                s[k] = 0 if k != 'cloud_ms' else 0.0

    def on_shutdown(self):
        if self._walker is not None:
            self._walker.close()
        self._server.close()


def main(args=None):
    rclpy.init(args=args)
    n = NavRelayServer()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.on_shutdown()
        n.destroy_node()
        rclpy.try_shutdown()
