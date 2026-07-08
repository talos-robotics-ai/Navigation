#!/usr/bin/env python3
# Generic ROS 2 <-> ZMQ relay for the distributed-nav split (docs/planning/
# DISTRIBUTED_NAV_PLAN.md). Serializes ROS messages over ZMQ so NO DDS crosses the
# WiFi — the Jetson ships odom/TF/obstacles to the off-board planner and the laptop
# ships /mpc/cmd_vel back. Both ends run Humble, so the CDR bytes are compatible.
#
# THREADING MODEL (2026-07 refactor — fixes the "heavy topic collapses odom" bottleneck):
#   Measured on the real robot: with the old design (serialize_message + send INSIDE the
#   rclpy callback), relaying one big topic (e.g. /dlio/.../path at 156 KB) stalled the
#   single executor thread and collapsed /dlio/.../odom from 108 Hz to 5.6 Hz with 300 ms
#   jitter — starving the gait-yaw and the MPC pose. It was NOT a WiFi limit (6.8 Mbit/s
#   on a 94 Mbit/s link). Fix: the ROS callback only OFFERS the latest message to a
#   per-topic bounded queue (cheap); a dedicated per-topic thread does the expensive
#   serialize_message() and the ZMQ send. So a slow serialize on one topic can no longer
#   block another. The one PUB socket (not thread-safe) is guarded by a short lock held
#   ONLY around send() — the expensive serialize happens outside it, in parallel per topic.
#
# Per-topic options:
#   "topic:pkg/msg/Type"       relay every message (drop-oldest if the queue fills)
#   "topic:pkg/msg/Type@20"    THROTTLE to <=20 Hz (drop-before-enqueue). Use for odom:
#                              the planner needs ~10-20 Hz, not DLIO's raw 100 Hz.
#
# One process can both SEND (ROS sub -> ZMQ PUB) and RECV (ZMQ SUB -> ROS pub):
#
#   Jetson:
#     python3 zmq_ros_bridge.py \
#       --send "/dlio/odom_node/odom:nav_msgs/msg/Odometry@20,/tf:tf2_msgs/msg/TFMessage,/tf_static:tf2_msgs/msg/TFMessage,/local_voxel_map/obstacles:sensor_msgs/msg/PointCloud2" \
#       --pub-bind "tcp://*:5601" \
#       --recv "/mpc/cmd_vel:geometry_msgs/msg/Twist" \
#       --sub-connect "tcp://<LAPTOP_IP>:5602"
#
# Deps: pyzmq, rclpy. Source ROS (Humble) before running.
import argparse
import collections
import importlib
import threading
import time

import zmq
import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message, deserialize_message
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy


def load_type(spec: str):
    # "nav_msgs/msg/Odometry" or "nav_msgs/Odometry" -> the message class
    parts = [p for p in spec.split('/') if p and p != 'msg']
    pkg, name = parts[0], parts[-1]
    return getattr(importlib.import_module(pkg + '.msg'), name)


def parse_send_spec(s: str):
    """"topic:pkg/msg/Type[@hz],..." -> [(topic, type_str, min_interval_s_or_None)].
    @hz throttles the topic to at most hz messages/s (min_interval = 1/hz). @0 = off."""
    out = []
    for item in (s or '').split(','):
        item = item.strip()
        if not item:
            continue
        min_interval = None
        if '@' in item:
            item, _, hz = item.rpartition('@')
            item = item.strip()
            try:
                hz = float(hz)
                min_interval = (1.0 / hz) if hz > 0 else None
            except ValueError:
                min_interval = None
        topic, _, typ = item.partition(':')
        out.append((topic.strip(), typ.strip(), min_interval))
    return out


def parse_recv_spec(s: str):
    """"topic:pkg/msg/Type,..." -> [(topic, type_str)]  (no throttle on the recv side)."""
    out = []
    for item in (s or '').split(','):
        item = item.strip()
        if not item:
            continue
        topic, _, typ = item.partition(':')
        out.append((topic.strip(), typ.strip()))
    return out


def qos_for(topic: str) -> QoSProfile:
    # /tf_static is latched (TRANSIENT_LOCAL); everything else is plain volatile.
    q = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST)
    if topic.endswith('tf_static'):
        q.durability = DurabilityPolicy.TRANSIENT_LOCAL
    return q


class TopicSender:
    """Bounded, throttled, thread-safe hand-off from a ROS callback to a sender thread.

    offer() runs on the rclpy executor thread and must stay CHEAP: it applies the
    optional rate throttle and appends the raw msg to a keep-latest deque (drop-oldest
    when full). get() runs on a dedicated sender thread which serializes + sends, so the
    expensive work is off the executor and parallel across topics."""

    def __init__(self, topic_bytes, typ, min_interval, maxlen, latched=False):
        self.topic = topic_bytes
        self.typ = typ
        self.min_interval = min_interval
        # latched (TRANSIENT_LOCAL, e.g. /tf_static): the ROS publisher sends it ONCE, but
        # a ZMQ PUB does NOT replay to subscribers that connect later, so a laptop relay /
        # RViz that joins after startup never gets it -> broken TF tree, no robot model, no
        # lidar-frame clouds. The sender thread re-broadcasts the last latched message
        # periodically so any late subscriber picks it up within RESEND_S.
        self.latched = latched
        self.q = collections.deque(maxlen=max(1, maxlen))
        self._last = None      # monotonic time of last accepted offer (for throttle)
        self.dropped = 0       # dropped by queue-full (backpressure)
        self.sent = 0
        self._cv = threading.Condition()

    def offer(self, msg, now):
        """Throttle + enqueue latest. Returns True if enqueued, False if throttled."""
        if self.min_interval is not None and self._last is not None \
                and (now - self._last) < self.min_interval:
            return False
        with self._cv:
            if len(self.q) == self.q.maxlen:
                self.dropped += 1   # deque evicts the oldest on append below
            self.q.append(msg)
            self._last = now
            self._cv.notify()
        return True

    def get(self, timeout):
        """Blocking pop (up to timeout s). Returns a msg or None if none arrived."""
        with self._cv:
            if not self.q:
                self._cv.wait(timeout)
            return self.q.popleft() if self.q else None

    def snapshot(self):
        return list(self.q)


class ZmqRosBridge(Node):
    def __init__(self, args):
        super().__init__('zmq_ros_bridge')
        self.ctx = zmq.Context.instance()
        self._running = True

        # SEND: ROS subscription -> per-topic queue -> sender thread -> ZMQ PUB.
        self.pub_sock = None
        self._send_lock = threading.Lock()   # the PUB socket is not thread-safe
        self._senders = {}
        if args.send:
            self.pub_sock = self.ctx.socket(zmq.PUB)
            self.pub_sock.set_hwm(args.hwm)
            self.pub_sock.bind(args.pub_bind)
            for topic, typ_str, min_interval in parse_send_spec(args.send):
                typ = load_type(typ_str)
                latched = qos_for(topic).durability == DurabilityPolicy.TRANSIENT_LOCAL
                sender = TopicSender(topic.encode(), typ, min_interval, maxlen=args.hwm, latched=latched)
                self._senders[topic] = sender
                self.create_subscription(
                    typ, topic, lambda msg, s=sender: self._on_ros(s, msg), qos_for(topic))
                threading.Thread(target=self._send_loop, args=(sender,), daemon=True).start()
                rate = f' @{1.0 / min_interval:.0f}Hz' if min_interval else (' (latched)' if latched else '')
                self.get_logger().info(f'SEND {topic} ({typ.__name__}){rate} -> ZMQ {args.pub_bind}')

        # RECV: ZMQ SUB -> ROS publisher (deserialize on its own thread; already off-executor)
        self.recv_pubs = {}
        self.sub_sock = None
        if args.recv:
            self.sub_sock = self.ctx.socket(zmq.SUB)
            self.sub_sock.set_hwm(args.hwm)
            self.sub_sock.connect(args.sub_connect)
            for topic, typ_str in parse_recv_spec(args.recv):
                typ = load_type(typ_str)
                self.recv_pubs[topic] = (self.create_publisher(typ, topic, qos_for(topic)), typ)
                self.sub_sock.setsockopt(zmq.SUBSCRIBE, topic.encode())
                self.get_logger().info(f'RECV ZMQ {args.sub_connect} -> {topic} ({typ.__name__})')
            threading.Thread(target=self._zmq_recv_loop, daemon=True).start()

    def _on_ros(self, sender, msg):
        # Executor thread: stay cheap — just hand off the latest message.
        sender.offer(msg, time.monotonic())

    _LATCH_RESEND_S = 2.0   # re-broadcast latched topics this often (for late subscribers)

    def _emit(self, sender, data):
        try:
            with self._send_lock:                       # hold the lock only around send()
                self.pub_sock.send_multipart([sender.topic, data], zmq.NOBLOCK)
            sender.sent += 1
        except zmq.Again:
            sender.dropped += 1                          # drop under backpressure, don't block

    def _send_loop(self, sender):
        last_latched = None                             # last serialized bytes of a latched topic
        last_resend = 0.0
        while self._running and rclpy.ok():
            msg = sender.get(0.2)
            if msg is None:
                # Idle tick: re-broadcast the latched message so a subscriber that
                # connected after the one-shot publish still receives it.
                if sender.latched and last_latched is not None:
                    now = time.monotonic()
                    if now - last_resend >= self._LATCH_RESEND_S:
                        self._emit(sender, last_latched)
                        last_resend = now
                continue
            try:
                data = serialize_message(msg)          # expensive — OFF the executor thread
            except Exception as e:                      # noqa: BLE001
                self.get_logger().warn(f'serialize {sender.topic}: {e}')
                continue
            self._emit(sender, data)
            if sender.latched:
                last_latched = data
                last_resend = time.monotonic()

    def _zmq_recv_loop(self):
        poller = zmq.Poller()
        poller.register(self.sub_sock, zmq.POLLIN)
        while self._running and rclpy.ok():
            if dict(poller.poll(200)):
                try:
                    topic_b, data = self.sub_sock.recv_multipart()
                except Exception:  # noqa: BLE001
                    continue
                entry = self.recv_pubs.get(topic_b.decode())
                if entry:
                    pub, typ = entry
                    try:
                        pub.publish(deserialize_message(data, typ))
                    except Exception as e:  # noqa: BLE001
                        self.get_logger().warn(f'deserialize {topic_b}: {e}')

    def destroy_node(self):
        self._running = False
        super().destroy_node()


def main():
    ap = argparse.ArgumentParser(description='Generic ROS2 <-> ZMQ relay (serialize_message over ZMQ).')
    ap.add_argument('--send', default='', help='ROS->ZMQ: "topic:pkg/msg/Type[@hz],..."')
    ap.add_argument('--recv', default='', help='ZMQ->ROS: "topic:pkg/msg/Type,..."')
    ap.add_argument('--pub-bind', default='tcp://*:5601', help='ZMQ PUB bind for --send')
    ap.add_argument('--sub-connect', default='', help='ZMQ SUB connect for --recv')
    ap.add_argument('--hwm', type=int, default=20, help='ZMQ high-water mark + per-topic queue depth (msgs)')
    args = ap.parse_args()

    rclpy.init()
    node = ZmqRosBridge(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass


if __name__ == '__main__':
    main()
