#!/usr/bin/env python3
"""Bridge /mpc/cmd_vel (Twist) -> the Unitree G1 NATIVE (factory) walking policy.

The counterpart to cmd_vel_to_amo_node, but for the robot's on-board high-level
locomotion controller instead of the AMO/RoboJuDo joint policy. It subscribes to
the same command + safety topics the AMO bridge uses, so the A*+MPC navigation
stack drives the native gait with ZERO changes upstream — only the last hop
differs:

    AMO :     /mpc/cmd_vel -> cmd_vel_to_amo_node   -> WebSocket :8766 -> AMO joint policy
    Unitree:  /mpc/cmd_vel -> THIS node             -> Unitree SDK DDS  -> native LocoClient

Because the native gait is HIGH-LEVEL (velocity in, firmware owns the joints),
there are no joint targets to filter as with AMO; the analog is VELOCITY-COMMAND
smoothing (ramp-in + slew + low-pass), applied here via VelocitySmoother — see
unitree_loco.py.

Subscribes:
    /mpc/cmd_vel   geometry_msgs/Twist  — velocity command from the MPC
    /estop         std_msgs/Bool        — latched e-stop (true => hold at zero)

Drives (Unitree SDK DDS, domain 0 on the robot NIC):
    LocoClient.SetVelocity(vx, vy, omega, duration)

SAFETY: the native gait and AMO both command the motors — run only ONE. Bring the
robot up to the requested walking FSM first, either manually with g1_loco_client /
unitree_gait_test, or set auto_bring_up:=true here. Keep the hardware e-stop in
hand.
"""
from __future__ import annotations

import os
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from g1_sim_bridge.unitree_loco import (
    FSM_DEFAULT_WALK,
    LocoDriver,
    SmootherConfig,
    VelocitySmoother,
    fsm_name,
)


class CmdVelToUnitreeLoco(Node):
    def __init__(self):
        super().__init__("cmd_vel_to_unitree_loco")

        # ── Params (names mirror cmd_vel_to_amo where they overlap) ──────────
        self.declare_parameter("net_if", os.environ.get("UNITREE_NET_IFACE", "eth0"))
        self.declare_parameter("unitree_domain_id", 0)   # Unitree DDS domain (robot)
        self.declare_parameter("cmd_vel_topic", "/mpc/cmd_vel")
        self.declare_parameter("estop_topic", "/estop")
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("max_forward_vel", 0.5)
        self.declare_parameter("max_lateral_vel", 0.12)
        self.declare_parameter("max_yaw_rate", 0.5)
        self.declare_parameter("cmd_timeout_sec", 0.5)
        # SetVelocity duration: the native controller keeps the last velocity for
        # this long, so if THIS node dies the robot also coasts to a stop. Kept a
        # bit above the tick period as a secondary firmware-side watchdog.
        self.declare_parameter("velocity_hold_s", 0.5)
        # Auto FSM bring-up on start (damp->stand_up->control_fsm). OFF by
        # default: standing the robot unexpectedly is unsafe — bring it up manually
        # (unitree_gait_test / g1_loco_client) unless you explicitly enable this.
        self.declare_parameter("auto_bring_up", False)
        self.declare_parameter(
            "control_fsm", int(os.environ.get("UNITREE_LOCO_FSM", FSM_DEFAULT_WALK)))
        self.declare_parameter("damp_on_shutdown", True)
        # Velocity-command smoothing (high-level analog of AMO joint filtering).
        self.declare_parameter("startup_ramp_s", 3.0)
        self.declare_parameter("lin_accel_max", 0.6)
        self.declare_parameter("yaw_accel_max", 1.2)
        self.declare_parameter("lowpass_alpha", 0.35)

        gp = self.get_parameter
        self._net_if = str(gp("net_if").value)
        self._domain = int(gp("unitree_domain_id").value)
        self._rate = float(gp("rate_hz").value)
        self._vmax = (float(gp("max_forward_vel").value),
                      float(gp("max_lateral_vel").value),
                      float(gp("max_yaw_rate").value))
        self._cmd_timeout = float(gp("cmd_timeout_sec").value)
        self._hold_s = float(gp("velocity_hold_s").value)
        self._control_fsm = int(gp("control_fsm").value)
        self._damp_on_shutdown = bool(gp("damp_on_shutdown").value)

        self._smoother = VelocitySmoother(SmootherConfig(
            startup_ramp_s=float(gp("startup_ramp_s").value),
            lin_accel_max=float(gp("lin_accel_max").value),
            yaw_accel_max=float(gp("yaw_accel_max").value),
            lowpass_alpha=float(gp("lowpass_alpha").value),
        ))

        self._cmd = (0.0, 0.0, 0.0)
        self._last_cmd_t = time.monotonic()
        self._timed_out = False
        self._estop = False
        self._last_tick_t = time.monotonic()

        # ── Connect to the native gait ───────────────────────────────────────
        self._driver = LocoDriver(self._net_if, domain_id=self._domain)
        try:
            self._driver.connect()
        except ImportError as exc:
            self.get_logger().fatal(str(exc))
            raise SystemExit(1) from exc
        self.get_logger().info(
            f"connected to Unitree native gait (net_if={self._net_if}, "
            f"DDS domain={self._domain})")

        # Liveness probe: warn loudly (with the fix) if the loco service isn't
        # answering, rather than silently forwarding velocities that go nowhere.
        ok, code, fsm = self._driver.probe()
        if ok:
            self.get_logger().info(f"loco service responding (fsm_id={fsm})")
        else:
            self.get_logger().warn(f"{self._driver._SERVICE_HINT} (GetFsmId code={code})")

        if bool(gp("auto_bring_up").value):
            self.get_logger().warn(
                "auto_bring_up=true — standing the robot up. Ensure it is clear "
                f"and the AMO policy is NOT running. Target FSM: "
                f"{self._control_fsm} ({fsm_name(self._control_fsm)}).")
            threading.Thread(target=self._bring_up_safe, daemon=True).start()
        else:
            self.get_logger().warn(
                "bring the robot to walking control FIRST, e.g. "
                "`ros2 run g1_sim_bridge unitree_gait_test`. Target FSM: "
                f"{self._control_fsm} ({fsm_name(self._control_fsm)}). "
                "This bridge only sends velocities.")

        # ── Subscriptions ────────────────────────────────────────────────────
        self.create_subscription(Twist, str(gp("cmd_vel_topic").value), self._on_twist, 10)
        estop_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Bool, str(gp("estop_topic").value), self._on_estop, estop_qos)

        self.create_timer(1.0 / max(1.0, self._rate), self._tick)
        self.get_logger().info(
            f"bridging {gp('cmd_vel_topic').value} -> Unitree LocoClient at "
            f"{self._rate:.0f} Hz (caps vx={self._vmax[0]} vy={self._vmax[1]} "
            f"yaw={self._vmax[2]}; watchdog={self._cmd_timeout:.2f}s)")

    def _bring_up_safe(self):
        try:
            self._driver.bring_up(logger=self.get_logger(), target_fsm=self._control_fsm)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"auto bring-up failed: {exc}")

    @staticmethod
    def _clip(v, lim):
        return max(-lim, min(lim, float(v)))

    def _on_twist(self, msg: Twist):
        self._cmd = (self._clip(msg.linear.x, self._vmax[0]),
                     self._clip(msg.linear.y, self._vmax[1]),
                     self._clip(msg.angular.z, self._vmax[2]))
        self._last_cmd_t = time.monotonic()
        if self._timed_out:
            self._timed_out = False
            self.get_logger().info("command stream resumed")

    def _on_estop(self, msg: Bool):
        engaged = bool(msg.data)
        if engaged != self._estop:
            self._estop = engaged
            if engaged:
                self.get_logger().warn("E-STOP ENGAGED — zero velocity to native gait")
                self._cmd = (0.0, 0.0, 0.0)
                self._smoother.reset()
                self._driver.stop_move()
            else:
                self.get_logger().info("E-STOP RELEASED — navigation re-enabled")

    def _tick(self):
        now = time.monotonic()
        dt = now - self._last_tick_t
        self._last_tick_t = now

        # E-stop has top priority: hold at zero regardless of the MPC.
        if self._estop:
            self._driver.set_velocity(0.0, 0.0, 0.0, self._hold_s)
            return
        # Watchdog: stale command stream -> zero (and reset the ramp so a resume
        # eases back in from standstill instead of jumping to the old command).
        if now - self._last_cmd_t > self._cmd_timeout:
            if not self._timed_out:
                self._timed_out = True
                self.get_logger().warn(
                    f"no cmd_vel for >{self._cmd_timeout:.2f}s — zero velocity")
            self._cmd = (0.0, 0.0, 0.0)
            self._smoother.reset()
            self._driver.set_velocity(0.0, 0.0, 0.0, self._hold_s)
            return

        vx, vy, wz = self._smoother.step(*self._cmd, dt=dt)
        self._driver.set_velocity(vx, vy, wz, self._hold_s)

    def destroy_node(self):
        try:
            self._driver.stop_move()
            if self._damp_on_shutdown:
                self._driver.damp()   # fail-safe: release into damping on exit
        except Exception:  # noqa: BLE001
            pass
        super().destroy_node()


def main(argv=None):
    rclpy.init(args=argv)
    node = CmdVelToUnitreeLoco()
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
