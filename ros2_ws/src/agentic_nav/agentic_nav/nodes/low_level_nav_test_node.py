"""low_level_nav_test — Phase-1 harness: exercise the GEOMETRIC nav on its own.

This is the whole point of shipping the interface/skill layer before any VLM or
agent: prove that a goal in → the robot drives there → GOAL_REACHED works
end-to-end through the clean seam, with nothing intelligent above it.

It drives a list of metric goals sequentially via NavigationSkillContainer
(which goes through NavigationInterface → /global_goal) and reports each outcome.

Run (inside the Humble container, planner + robot up):
    ros2 run agentic_nav low_level_nav_test --ros-args -p goals:="2.0,0.0,0.0; 0.0,0.0,3.14"
"""
from __future__ import annotations

import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor

from ..control.blueprints.mobile import build_mobile_nav


def _parse_goals(spec: str):
    goals = []
    for chunk in (spec or '').split(';'):
        chunk = chunk.strip()
        if not chunk:
            continue
        parts = [float(v) for v in chunk.split(',')]
        x, y = parts[0], parts[1]
        yaw = parts[2] if len(parts) > 2 else 0.0
        goals.append((x, y, yaw))
    return goals


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('low_level_nav_test')
    node.declare_parameter('goals', '2.0,0.0,0.0')
    node.declare_parameter('timeout_s', 60.0)
    node.declare_parameter('tag_demo', True)   # tag the first goal as 'home' to exercise memory

    stack = build_mobile_nav(node)
    timeout_s = float(node.get_parameter('timeout_s').value)
    goals = _parse_goals(str(node.get_parameter('goals').value))

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()

    # Wait until we actually hear the planner (state) + odom before commanding.
    node.get_logger().info('waiting for /navigation/state + odom ...')
    t0 = time.time()
    while rclpy.ok() and stack.nav.robot_xy() is None and time.time() - t0 < 15.0:
        time.sleep(0.2)
    if stack.nav.robot_xy() is None:
        node.get_logger().error('no odom — is the planner + relay running? aborting.')
        rclpy.shutdown()
        return

    if node.get_parameter('tag_demo').value and stack.nav.robot_xy() is not None:
        hx, hy = stack.nav.robot_xy()
        stack.memory.tag_location('home', hx, hy)
        node.get_logger().info(f'tagged current pose as "home" ({hx:.2f}, {hy:.2f})')

    passed = 0
    for i, (x, y, yaw) in enumerate(goals):
        node.get_logger().info(f'=== goal {i + 1}/{len(goals)} -> ({x:.2f}, {y:.2f}, {yaw:.2f}) ===')
        ok = stack.skills.navigate_to_pose(x, y, yaw, timeout_s=timeout_s)
        node.get_logger().info(f'=== goal {i + 1}: {"REACHED" if ok else "TIMEOUT"} ===')
        passed += int(ok)

    node.get_logger().info(f'low-level nav test done: {passed}/{len(goals)} reached')
    stack.nav.cancel()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
