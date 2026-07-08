#!/usr/bin/env python3
"""Unit tests for the pure logic of zmq_ros_bridge.py (parse + throttle + drop-oldest).
No ROS graph / no ZMQ socket needed. Run:  python3 -m pytest test_zmq_ros_bridge.py -q
(zmq + rclpy must import, which they do in the Humble env the relay runs in.)"""
from zmq_ros_bridge import parse_send_spec, parse_recv_spec, TopicSender


def test_parse_send_spec_with_and_without_rate():
    assert parse_send_spec("/a:nav_msgs/msg/Odometry@20,/b:pkg/msg/T") == \
        [("/a", "nav_msgs/msg/Odometry", 0.05), ("/b", "pkg/msg/T", None)]


def test_parse_send_spec_skips_blanks_and_whitespace():
    assert parse_send_spec(" , /x:pkg/msg/T ,") == [("/x", "pkg/msg/T", None)]


def test_parse_send_spec_zero_rate_is_unthrottled():
    assert parse_send_spec("/a:pkg/msg/T@0") == [("/a", "pkg/msg/T", None)]


def test_parse_recv_spec_ignores_rate_suffix_absent():
    assert parse_recv_spec("/c:pkg/msg/T") == [("/c", "pkg/msg/T")]


def test_throttle_rejects_within_interval_keeps_accepted():
    s = TopicSender(b"/a", None, min_interval=0.05, maxlen=10)
    assert s.offer("m1", now=0.00) is True
    assert s.offer("m2", now=0.01) is False   # too soon -> dropped, not queued
    assert s.offer("m3", now=0.06) is True     # interval elapsed
    assert s.snapshot() == ["m1", "m3"]


def test_unthrottled_drops_oldest_when_full():
    s = TopicSender(b"/a", None, min_interval=None, maxlen=2)
    assert s.offer("m1", 0.0) is True
    assert s.offer("m2", 0.0) is True
    assert s.offer("m3", 0.0) is True          # accepted; deque evicts oldest
    assert s.snapshot() == ["m2", "m3"]         # m1 dropped (keep-latest)
    assert s.dropped == 1


def test_topicsender_latched_flag_defaults_false_and_settable():
    assert TopicSender(b"/tf", None, None, 5).latched is False
    assert TopicSender(b"/tf_static", None, None, 5, latched=True).latched is True


def test_get_returns_fifo_then_none_on_timeout():
    s = TopicSender(b"/a", None, min_interval=None, maxlen=5)
    s.offer("m1", 0.0)
    s.offer("m2", 0.0)
    assert s.get(timeout=0.0) == "m1"
    assert s.get(timeout=0.0) == "m2"
    assert s.get(timeout=0.01) is None          # empty -> None after wait
