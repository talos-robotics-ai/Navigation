#!/usr/bin/env python3
"""Bridge /mpc/cmd_vel (Twist) -> the SONIC whole-body locomotion policy.

The third sibling of cmd_vel_to_amo_node / cmd_vel_to_unitree_loco_node. SONIC's
real-time C++ controller (g1_deploy_onnx_ref) is NOT a ROS 2 process: it SUBs a
ZMQ PUB socket (tcp://*:5556) and drives the robot over CycloneDDS — the same
"last hop to a non-ROS gait" role AMO (WebSocket) and Unitree (LocoClient) play.
So this bridge is mutually exclusive with those two; run exactly one.

    AMO     : /mpc/cmd_vel -> cmd_vel_to_amo_node        -> WebSocket :8766 -> AMO joint policy
    Unitree : /mpc/cmd_vel -> cmd_vel_to_unitree_loco    -> Unitree SDK DDS -> native LocoClient
    SONIC   : /mpc/cmd_vel -> THIS node                  -> ZMQ :5556       -> g1_deploy_onnx_ref

FRAME RECONCILIATION (why this bridge also needs odometry, unlike the other two):
  The MPC emits a BODY-frame Twist (vx forward, vy left, wz yaw-rate). SONIC's
  `planner` message instead wants WORLD-frame DIRECTION unit vectors — `movement`
  (where to translate) and `facing` (where to point). Turning ONLY happens via
  `facing`, so a body-frame command alone can never make SONIC turn.

  The stock SONIC bridge derives world heading OPEN-LOOP (theta += wz*dt), which
  drifts. Here we anchor `facing` on the MEASURED DLIO yaw (/dlio/odom_node/odom),
  so the conversion is CLOSED-LOOP and does not drift over long runs:

      dth      = wrap(yaw_meas - yaw0)           # heading vs SONIC's start frame
      movement = R(dth) . [vx, vy]               # body velocity -> world dir
      face     = dth + wz * facing_lookahead     # inject MPC turn intent ahead
      facing   = [cos(face), sin(face)]

  yaw0 is the DLIO yaw latched when this bridge starts control (SONIC defines its
  own world frame at policy start as facing=[1,0,0]); anchoring relative makes the
  odom frame and the SONIC frame agree. `facing_lookahead` is REQUIRED: without
  it `facing` == current heading and the robot never turns. It is the closed-loop
  replacement for open-loop integration and converges as the measured yaw catches
  up and the MPC shrinks wz.

Subscribes:
    /mpc/cmd_vel           geometry_msgs/Twist   — body velocity command (MPC)
    /dlio/odom_node/odom   nav_msgs/Odometry     — measured pose (yaw source)
    /estop                 std_msgs/Bool         — latched e-stop (true => idle+hold)

Drives (ZMQ PUB, tcp://<zmq_host>:<zmq_port>):
    SONIC `command` (start/stop/planner) + `planner` (per-tick locomotion).

SAFETY: same contract as the AMO/Unitree bridges — a 0.5 s watchdog forces IDLE if
/mpc/cmd_vel goes stale, and /estop holds the robot at zero. Run only ONE gait.

Deps: pyzmq (via g1_sim_bridge.sonic_wire; rosdep key python3-zmq).
"""
from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool

from g1_sim_bridge.sonic_wire import (
    MODE_IDLE,
    UB_PRESETS,
    SonicPlannerPublisher,
    build_upper_body,
)

_EPS = 1e-3


def _wrap(a: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def _yaw_from_quat(q) -> float:
    """Yaw (rad) from a geometry_msgs Quaternion, no tf dependency."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class CmdVelToSonic(Node):
    def __init__(self):
        super().__init__("cmd_vel_to_sonic")

        # ── Params (names mirror cmd_vel_to_amo where they overlap) ──────────
        self.declare_parameter("zmq_host", "*")      # bind address for the PUB socket
        self.declare_parameter("zmq_port", 5556)
        self.declare_parameter("cmd_vel_topic", "/mpc/cmd_vel")
        self.declare_parameter("odom_topic", "/dlio/odom_node/odom")
        self.declare_parameter("estop_topic", "/estop")
        self.declare_parameter("rate_hz", 30.0)
        self.declare_parameter("max_forward_vel", 0.8)
        self.declare_parameter("max_lateral_vel", 0.4)
        self.declare_parameter("max_yaw_rate", 0.8)
        # Fail-safe watchdog: if no command arrives for this long, force IDLE
        # instead of coasting on a stale command (see cmd_vel_to_amo_node).
        self.declare_parameter("cmd_timeout_sec", 0.5)
        # SONIC-specific.
        self.declare_parameter("mode", 2)           # LocomotionMode (2 = WALK)
        # SONIC realises ~0.85x commanded m/s; feed-forward correction so the
        # robot actually reaches the requested speed. The MPC also closes the
        # loop on measured vx, so this is only a first-order compensation.
        self.declare_parameter("speed_gain", 1.18)
        self.declare_parameter("max_speed", 1.0)    # speed clamp (m/s)
        # Turn-intent look-ahead: `facing` leads the measured heading by
        # wz * this. REQUIRED (>0) or the robot never turns.
        self.declare_parameter("facing_lookahead_sec", 0.4)
        # Optional carry-while-walking: pin the 17-DOF upper body to a fixed
        # pose. hold_arms uses arm_preset (default 'default'); a non-empty
        # arm_preset also enables it. Empty + hold_arms=false => arms follow the
        # policy's own gait (default).
        self.declare_parameter("hold_arms", False)
        self.declare_parameter("arm_preset", "")

        gp = self.get_parameter
        host = str(gp("zmq_host").value)
        port = int(gp("zmq_port").value)
        self._cmd_vel_topic = str(gp("cmd_vel_topic").value)
        self._odom_topic = str(gp("odom_topic").value)
        estop_topic = str(gp("estop_topic").value)
        rate = float(gp("rate_hz").value)
        self._vmax = (float(gp("max_forward_vel").value),
                      float(gp("max_lateral_vel").value),
                      float(gp("max_yaw_rate").value))
        self._cmd_timeout = float(gp("cmd_timeout_sec").value)
        self._mode = int(gp("mode").value)
        self._speed_gain = float(gp("speed_gain").value)
        self._max_speed = float(gp("max_speed").value)
        self._lookahead = float(gp("facing_lookahead_sec").value)

        # Build the optional fixed upper-body target once.
        preset = str(gp("arm_preset").value).strip()
        if bool(gp("hold_arms").value) or preset:
            try:
                self._upper_body = build_upper_body(preset or "default")
                self.get_logger().info(
                    f"holding upper body (preset={preset or 'default'})")
            except ValueError as exc:
                self.get_logger().error(
                    f"{exc}; arms will follow the gait. "
                    f"valid presets: {', '.join(sorted(UB_PRESETS))}")
                self._upper_body = None
        else:
            self._upper_body = None

        # ── State ────────────────────────────────────────────────────────────
        self._cmd = (0.0, 0.0, 0.0)
        self._last_cmd_t = time.monotonic()
        self._timed_out = False
        self._estop = False
        self._yaw: float | None = None        # latest measured yaw
        self._yaw0: float | None = None        # anchor (SONIC world-frame origin)
        self._need_anchor = True

        # ── ZMQ sink + start control ─────────────────────────────────────────
        self._pub = SonicPlannerPublisher(host=host, port=port)
        # Start SONIC in planner mode (mirrors the stock bridge's start_control).
        self._pub.send_command(start=True, stop=False, planner=True)

        # ── Subscriptions ────────────────────────────────────────────────────
        self.create_subscription(Twist, self._cmd_vel_topic, self._on_twist, 10)
        self.create_subscription(Odometry, self._odom_topic, self._on_odom, 10)
        # E-stop latched (TRANSIENT_LOCAL) so the bridge picks up the current
        # stop state even if it (re)starts after the e-stop was engaged.
        estop_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(Bool, estop_topic, self._on_estop, estop_qos)

        self.create_timer(1.0 / max(1.0, rate), self._tick)
        self.get_logger().info(
            f"bridging {self._cmd_vel_topic} (+ yaw from {self._odom_topic}) -> "
            f"SONIC ZMQ tcp://{host}:{port} at {rate:.0f} Hz "
            f"(caps vx={self._vmax[0]} vy={self._vmax[1]} yaw={self._vmax[2]}; "
            f"speed_gain={self._speed_gain}; lookahead={self._lookahead:.2f}s; "
            f"watchdog={self._cmd_timeout:.2f}s; estop_topic={estop_topic})")

    @staticmethod
    def _clip(v, lim):
        return max(-lim, min(lim, float(v)))

    def _on_twist(self, msg: Twist):
        self._cmd = (
            self._clip(msg.linear.x, self._vmax[0]),
            self._clip(msg.linear.y, self._vmax[1]),
            self._clip(msg.angular.z, self._vmax[2]),
        )
        self._last_cmd_t = time.monotonic()
        if self._timed_out:
            self._timed_out = False
            self.get_logger().info("command stream resumed")

    def _on_odom(self, msg: Odometry):
        self._yaw = _yaw_from_quat(msg.pose.pose.orientation)

    def _on_estop(self, msg: Bool):
        engaged = bool(msg.data)
        if engaged != self._estop:
            self._estop = engaged
            if engaged:
                self.get_logger().warn("E-STOP ENGAGED — forcing SONIC IDLE")
                self._cmd = (0.0, 0.0, 0.0)
                self._last_cmd_t = time.monotonic()
                self._send_idle()
            else:
                self.get_logger().info("E-STOP RELEASED — navigation re-enabled")

    def _facing_hold(self):
        """`facing` unit vector for the current measured heading (no look-ahead),
        used to hold heading while idle/estopped. Falls back to SONIC's start
        heading before any odom/anchor is available."""
        if self._yaw is not None and self._yaw0 is not None:
            dth = _wrap(self._yaw - self._yaw0)
            return [math.cos(dth), math.sin(dth), 0.0]
        return [1.0, 0.0, 0.0]

    def _send_idle(self):
        self._pub.send_planner(MODE_IDLE, [0.0, 0.0, 0.0], self._facing_hold(),
                               speed=-1.0, upper_body=self._upper_body)

    def _tick(self):
        # E-stop has top priority: hold at zero regardless of the MPC.
        if self._estop:
            self._send_idle()
            return
        # Watchdog: stale command stream -> zero the command (fail-safe stop).
        # The zeroed command below then resolves to IDLE via the normal path.
        if time.monotonic() - self._last_cmd_t > self._cmd_timeout:
            self._cmd = (0.0, 0.0, 0.0)
            if not self._timed_out:
                self._timed_out = True
                self.get_logger().warn(
                    f"no cmd_vel for >{self._cmd_timeout:.2f}s — forcing IDLE")
        # No heading yet -> cannot build the world frame; hold at the start pose.
        if self._yaw is None:
            self._send_idle()
            return
        if self._need_anchor:
            self._yaw0 = self._yaw
            self._need_anchor = False
            self.get_logger().info(f"anchored SONIC world frame at yaw0={self._yaw0:.3f} rad")

        vx, vy, wz = self._cmd
        dth = _wrap(self._yaw - self._yaw0)
        c, s = math.cos(dth), math.sin(dth)
        # Body velocity rotated into SONIC's world frame by the MEASURED heading.
        dirx = vx * c - vy * s
        diry = vx * s + vy * c
        speed_mag = math.hypot(vx, vy)
        if speed_mag > _EPS:
            n = math.hypot(dirx, diry)
            movement = [dirx / n, diry / n, 0.0]
            speed = min(speed_mag * self._speed_gain, self._max_speed)
            mode = self._mode
        else:
            movement = [0.0, 0.0, 0.0]
            speed = -1.0
            mode = self._mode if abs(wz) > _EPS else MODE_IDLE
        # `facing` leads the measured heading by the commanded turn rate so the
        # policy actually turns; converges to dth as wz -> 0.
        face = dth + wz * self._lookahead
        facing = [math.cos(face), math.sin(face), 0.0]
        self._pub.send_planner(mode, movement, facing, speed=speed,
                               upper_body=self._upper_body)

    def destroy_node(self):
        try:
            self._send_idle()          # stop translating/turning
            time.sleep(0.1)            # let the PUB socket flush
            self._pub.send_command(start=False, stop=True, planner=True)
            time.sleep(0.1)
            self._pub.close()
        except Exception:  # noqa: BLE001
            pass
        super().destroy_node()


def main(argv=None):
    rclpy.init(args=argv)
    node = CmdVelToSonic()
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


if __name__ == "__main__":
    main()
