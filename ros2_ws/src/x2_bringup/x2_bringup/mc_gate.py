"""Pure (no rclpy) velocity shaping + stale/zero-tail gate of mc_velocity_node.

Port of the shaping in tools/x2_teleop/mc_velocity_bridge.py (the PS5 bridge), plus the deadband mode.

mc ignores a command whose magnitude is below its start thresholds (linear 0.2 m/s, angular 0.1 rad/s):
    deadband_mode "lift" (default)  a nonzero command >= EPS but below the threshold is RAISED to the
                                    threshold (sign kept), so a slow planner command still moves the
                                    robot -- at the slowest speed mc accepts, i.e. faster than asked;
                                    below EPS it is 0.
    deadband_mode "zero"            the PS5 bridge's behaviour: below the threshold -> 0 (robot stands).
    deadband_mode "none"            pass through (mc drops it itself).
"""
import math

MIN_LINEAR, MIN_ANGULAR, MAX_ANY = 0.2, 0.1, 1.0
ZERO_TAIL_S = 0.3
EPS_LINEAR, EPS_ANGULAR = 0.03, 0.02    # below these a command means "stand still"


def shape(value, cap, minimum, eps, mode='lift'):
    if not math.isfinite(value):
        return 0.0
    mag = min(abs(value), cap, MAX_ANY)
    if mag < eps:
        return 0.0
    if mag < minimum:
        if mode == 'lift':
            mag = minimum
        elif mode == 'zero':
            return 0.0
    return math.copysign(mag, value)


class VelocityGate:
    """cmd (vx, vy, wz, rx_time) + estop -> (vx, vy, wz) to publish, or None (silent)."""

    def __init__(self, max_vx=0.5, max_vy=0.3, max_wz=0.5, stale_s=0.3, mode='lift', zero_tail_s=ZERO_TAIL_S):
        self.max_vx, self.max_vy, self.max_wz = max_vx, max_vy, max_wz
        self.stale_s, self.mode, self.zero_tail_s = stale_s, mode, zero_tail_s
        self.zero_until = 0.0
        self.active = False

    def step(self, now, cmd, estop=False):
        """cmd = (vx, vy, wz, t_rx) or None."""
        fresh = cmd is not None and now - cmd[3] <= self.stale_s
        if fresh and not estop:
            out = (shape(cmd[0], self.max_vx, MIN_LINEAR, EPS_LINEAR, self.mode),
                   shape(cmd[1], self.max_vy, MIN_LINEAR, EPS_LINEAR, self.mode),
                   shape(cmd[2], self.max_wz, MIN_ANGULAR, EPS_ANGULAR, self.mode))
            self.zero_until = now + self.zero_tail_s
            self.active = True
            return out
        if now < self.zero_until:
            return (0.0, 0.0, 0.0)
        self.active = False
        return None
