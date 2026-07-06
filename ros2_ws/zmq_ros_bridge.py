#!/usr/bin/env python3
# Generic ROS 2 <-> ZMQ relay for the distributed-nav split (docs/planning/
# DISTRIBUTED_NAV_PLAN.md). Serializes ROS messages over ZMQ so NO DDS crosses the
# WiFi — the Jetson ships odom/TF/obstacles to the off-board planner and the laptop
# ships /mpc/cmd_vel back. Both ends run Humble, so the CDR bytes are compatible.
#
# One process can both SEND (ROS sub -> ZMQ PUB) and RECV (ZMQ SUB -> ROS pub):
#
#   Jetson:
#     python3 zmq_ros_bridge.py \
#       --send "/dlio/odom_node/odom:nav_msgs/msg/Odometry,/tf:tf2_msgs/msg/TFMessage,/tf_static:tf2_msgs/msg/TFMessage,/local_voxel_map/obstacles:sensor_msgs/msg/PointCloud2" \
#       --pub-bind "tcp://*:5601" \
#       --recv "/mpc/cmd_vel:geometry_msgs/msg/Twist" \
#       --sub-connect "tcp://<LAPTOP_IP>:5602"
#
#   Laptop (inside the Humble container):
#     python3 zmq_ros_bridge.py \
#       --recv "/dlio/odom_node/odom:nav_msgs/msg/Odometry,/tf:tf2_msgs/msg/TFMessage,/tf_static:tf2_msgs/msg/TFMessage,/local_voxel_map/obstacles:sensor_msgs/msg/PointCloud2" \
#       --sub-connect "tcp://<JETSON_IP>:5601" \
#       --send "/mpc/cmd_vel:geometry_msgs/msg/Twist" \
#       --pub-bind "tcp://*:5602"
#
# Deps: pyzmq, rclpy. Source ROS (Humble) before running.
import argparse
import importlib
import threading

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


def parse_topics(s: str):
    out = []
    for item in (s or '').split(','):
        item = item.strip()
        if not item:
            continue
        topic, _, typ = item.partition(':')
        out.append((topic.strip(), load_type(typ.strip())))
    return out


def qos_for(topic: str) -> QoSProfile:
    # /tf_static is latched (TRANSIENT_LOCAL); everything else is plain volatile.
    q = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST)
    if topic.endswith('tf_static'):
        q.durability = DurabilityPolicy.TRANSIENT_LOCAL
    return q


class ZmqRosBridge(Node):
    def __init__(self, args):
        super().__init__('zmq_ros_bridge')
        self.ctx = zmq.Context.instance()

        # SEND: ROS subscription -> ZMQ PUB (multipart [topic, cdr_bytes])
        self.pub_sock = None
        if args.send:
            self.pub_sock = self.ctx.socket(zmq.PUB)
            self.pub_sock.set_hwm(args.hwm)
            self.pub_sock.bind(args.pub_bind)
            for topic, typ in parse_topics(args.send):
                self.create_subscription(typ, topic,
                                         lambda msg, t=topic: self._on_ros(t, msg),
                                         qos_for(topic))
                self.get_logger().info(f'SEND {topic} ({typ.__name__}) -> ZMQ {args.pub_bind}')

        # RECV: ZMQ SUB -> ROS publisher
        self.recv_pubs = {}
        self.sub_sock = None
        if args.recv:
            self.sub_sock = self.ctx.socket(zmq.SUB)
            self.sub_sock.set_hwm(args.hwm)
            self.sub_sock.connect(args.sub_connect)
            for topic, typ in parse_topics(args.recv):
                self.recv_pubs[topic] = (self.create_publisher(typ, topic, qos_for(topic)), typ)
                self.sub_sock.setsockopt(zmq.SUBSCRIBE, topic.encode())
                self.get_logger().info(f'RECV ZMQ {args.sub_connect} -> {topic} ({typ.__name__})')
            threading.Thread(target=self._zmq_recv_loop, daemon=True).start()

    def _on_ros(self, topic, msg):
        try:
            self.pub_sock.send_multipart([topic.encode(), serialize_message(msg)], zmq.NOBLOCK)
        except zmq.Again:
            pass  # drop under backpressure rather than block the ROS callback

    def _zmq_recv_loop(self):
        poller = zmq.Poller()
        poller.register(self.sub_sock, zmq.POLLIN)
        while rclpy.ok():
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


def main():
    ap = argparse.ArgumentParser(description='Generic ROS2 <-> ZMQ relay (serialize_message over ZMQ).')
    ap.add_argument('--send', default='', help='ROS->ZMQ: "topic:pkg/msg/Type,..."')
    ap.add_argument('--recv', default='', help='ZMQ->ROS: "topic:pkg/msg/Type,..."')
    ap.add_argument('--pub-bind', default='tcp://*:5601', help='ZMQ PUB bind for --send')
    ap.add_argument('--sub-connect', default='', help='ZMQ SUB connect for --recv')
    ap.add_argument('--hwm', type=int, default=20, help='ZMQ high-water mark (msgs)')
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
