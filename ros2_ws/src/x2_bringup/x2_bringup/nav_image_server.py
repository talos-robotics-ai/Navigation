"""PC2: the head RGB-D for the laptop detector, on its OWN TCP port (5597), so images never delay odom/cmd (5596).

Subscribes (BEST_EFFORT, depth 1, RAW = no ROS deserialization of frames that are dropped)
    <ns>/rgb_image   <ns>/depth_image   <ns>/rgb_camera_info          ns = /aima/hal/sensor/rgbd_head_front
ONLY WHILE A CLIENT IS CONNECTED (subscriptions are created on connect, destroyed on disconnect: zero cost otherwise),
encodes at most image_hz (10) per stream: RGB -> JPEG (cv2, quality 80), depth -> 16-bit PNG in mm (compression 1),
and sends them latest-wins. The images are sent as published (the module is mounted upside down: the detector
handles orientation). Wire format: nav_image_proto.py. `rgb_source: compressed` forwards the vendor's own
<ns>/rgb_image/compressed instead of re-encoding (cheaper, if the vendor publishes it).
"""
import collections
import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

from . import nav_image_proto as ip
from . import nav_relay_proto as proto
from .nav_relay_link import LinkServer

BEST_EFFORT1 = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)


class NavImageServer(Node):
    def __init__(self, **kw):
        super().__init__('nav_image_server', **kw)
        P = self.declare_parameter
        P('host', '0.0.0.0')
        P('port', 5597)
        P('ns', '/aima/hal/sensor/rgbd_head_front')
        P('image_hz', 10.0)
        P('jpeg_quality', 80)
        P('png_level', 1)
        P('rgb_source', 'raw')        # raw | compressed
        g = lambda k: self.get_parameter(k).value  # noqa: E731
        self._ns, self._dt = str(g('ns')), 1.0 / float(g('image_hz'))
        self._q, self._lvl, self._rgb_src = int(g('jpeg_quality')), int(g('png_level')), str(g('rgb_source'))
        self._subs = []
        # Recent raw messages per stream, (stamp, raw): the tick encodes only an RGB/depth PAIR with the same
        # capture stamp. (Throttling each stream on its own sent unmatched halves: the laptop paired ~1 of 2.)
        self._buf = {'rgb': collections.deque(maxlen=6), 'depth': collections.deque(maxlen=6)}
        self._sent_stamp = None
        self._pair_tol = 0.012     # s; the RGB-D module stamps both from one capture
        self.create_timer(self._dt, self._tick)
        self._info = None
        self._stat = dict(rgb=0, depth=0, rgb_in=0, depth_in=0, rgb_ms=0.0, depth_ms=0.0, rgb_b=0, depth_b=0)
        self._t_stat = time.monotonic()
        self._server = LinkServer(str(g('host')), int(g('port')), lambda h, p: None, self._on_connect,
                                  lambda l: self.get_logger().info(f'image client {l.peer} gone'),
                                  log=lambda s: self.get_logger().info(s))
        self.create_timer(0.5, self._sync)
        self.create_timer(1.0, self._slow)
        self.get_logger().info(f'nav image server on {g("host")}:{self._server.port}; {self._ns} rgb {self._rgb_src} '
                               f'<= {g("image_hz")} Hz, nothing subscribed until a client connects')

    def _on_connect(self, link):
        self.get_logger().info(f'image client connected: {link.peer}')
        if self._info is not None:
            link.send(self._info, replace_key='info')

    def _sync(self):
        """Subscribe while a client is connected, unsubscribe when it is gone (runs on the executor)."""
        want = self._server.link is not None
        if want and not self._subs:
            ns = self._ns
            rgb_topic, rgb_type = (f'{ns}/rgb_image/compressed', CompressedImage) if self._rgb_src == 'compressed' \
                else (f'{ns}/rgb_image', Image)
            self._subs = [
                self.create_subscription(CameraInfo, f'{ns}/rgb_camera_info', self._on_info, BEST_EFFORT1),
                self.create_subscription(rgb_type, rgb_topic, lambda r: self._on_img('rgb', r), BEST_EFFORT1, raw=True),
                self.create_subscription(Image, f'{ns}/depth_image', lambda r: self._on_img('depth', r), BEST_EFFORT1, raw=True),
            ]
            self.get_logger().info('client present: subscribed to the head camera')
        elif not want and self._subs:
            for s in self._subs:
                self.destroy_subscription(s)
            self._subs = []
            self.get_logger().info('no client: unsubscribed from the head camera')

    def _on_info(self, m):
        self._info = proto.encode_frame({
            't': 'info', 'frame_id': m.header.frame_id, 'width': m.width, 'height': m.height,
            'K': [float(v) for v in m.k], 'D': [float(v) for v in m.d], 'distortion_model': m.distortion_model})

    def _on_img(self, kind, raw):
        """Only buffer: the header stamp is read straight from the CDR bytes (encapsulation 4 B, then
        builtin_interfaces/Time sec int32 + nanosec uint32), so nothing is deserialized here."""
        self._stat[kind + '_in'] += 1
        try:
            sec, nsec = struct.unpack_from('<iI', raw, 4)
        except struct.error:
            return
        self._buf[kind].append((sec + nsec * 1e-9, raw))

    def _tick(self):
        if self._server.link is None or not self._buf['rgb'] or not self._buf['depth']:
            return
        for st_rgb, raw_rgb in reversed(self._buf['rgb']):          # newest RGB that has a depth twin
            if self._sent_stamp is not None and st_rgb <= self._sent_stamp:
                return
            match = min(self._buf['depth'], key=lambda d: abs(d[0] - st_rgb))
            if abs(match[0] - st_rgb) <= self._pair_tol:
                self._sent_stamp = st_rgb
                self._encode_send('rgb', raw_rgb)
                self._encode_send('depth', match[1])
                self._stat['pairs'] = self._stat.get('pairs', 0) + 1
                return

    def _encode_send(self, kind, raw):
        t = time.monotonic()
        try:
            if kind == 'rgb' and self._rgb_src == 'compressed':
                m = deserialize_message(raw, CompressedImage)
                hdr = {'t': 'rgb', 'width': 0, 'height': 0, 'encoding': 'jpeg', 'src_encoding': m.format}
                payload = bytes(m.data)
            else:
                m = deserialize_message(raw, Image)
                if kind == 'rgb':
                    payload = ip.encode_rgb(m.data, m.height, m.width, m.step, m.encoding, self._q)
                    hdr = {'t': 'rgb', 'encoding': 'jpeg', 'src_encoding': m.encoding}
                else:
                    payload = ip.encode_depth(m.data, m.height, m.width, m.step, m.encoding, self._lvl)
                    hdr = {'t': 'depth', 'encoding': 'png16', 'unit': 'mm'}
                hdr.update(width=m.width, height=m.height)
        except Exception as e:   # noqa: BLE001 -- a bad frame must not kill the executor thread
            self.get_logger().warn(f'{kind} frame dropped: {e}', throttle_duration_sec=5)
            return
        hdr.update(stamp_sec=m.header.stamp.sec, stamp_nsec=m.header.stamp.nanosec, frame_id=m.header.frame_id)
        if self._server.send(proto.encode_frame(hdr, payload), replace_key=kind):
            self._stat[kind] += 1
            self._stat[kind + '_b'] += len(payload)
            self._stat[kind + '_ms'] += (time.monotonic() - t) * 1e3

    def _slow(self):
        link = self._server.link
        if link is not None:
            link.send(proto.encode_frame({'t': 'hb', 'stamp': time.time()}), replace_key='hb')
            if self._info is not None:
                link.send(self._info, replace_key='info')
        now = time.monotonic()
        if now - self._t_stat >= 5.0 and link is not None:
            dt, s = now - self._t_stat, self._stat
            self._t_stat = now
            n = {k: max(s[k], 1) for k in ('rgb', 'depth')}
            self.get_logger().info(
                f'images client={link.peer} tx/s rgb {s["rgb"]/dt:.1f} ({s["rgb_b"]/n["rgb"]/1e3:.0f} kB, {s["rgb_ms"]/n["rgb"]:.0f} ms) '
                f'depth {s["depth"]/dt:.1f} ({s["depth_b"]/n["depth"]/1e3:.0f} kB, {s["depth_ms"]/n["depth"]:.0f} ms) '
                f'in/s {s["rgb_in"]/dt:.1f}/{s["depth_in"]/dt:.1f} dropped {link.dropped}')
            for k in s:
                s[k] = 0 if not k.endswith('_ms') else 0.0
        elif link is None:
            self._t_stat = now

    def on_shutdown(self):
        self._server.close()


def main(args=None):
    rclpy.init(args=args)
    n = NavImageServer()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.on_shutdown()
        n.destroy_node()
        rclpy.try_shutdown()
