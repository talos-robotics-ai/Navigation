"""Planar geometry for the box pick-and-place FSM (pure Python, no rclpy). Frames: odom (z up)."""
import math


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quat_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def crate_axis_directions(crate_yaw, axis='x'):
    """Unit vectors (both signs) of the crate's horizontal x or y axis. The box is 180-deg symmetric
    for the detector, so which sign is 'front' is ambiguous: callers pick the one facing the robot."""
    a = crate_yaw + (0.0 if axis == 'x' else math.pi / 2)
    return (math.cos(a), math.sin(a)), (-math.cos(a), -math.sin(a))


def pregrasp_pose(robot_xy, crate_xy, standoff, mode='line', crate_yaw=0.0, axis='x'):
    """Pre-grasp pose (x, y, yaw) in odom, on the ground plane.

    mode 'line': stand `standoff` short of the crate on the robot->crate line, facing the crate.
    mode 'axis': stand `standoff` from the crate along its x (or y) axis, on the side facing the
                 robot, facing the crate (use when the grasp needs a particular box side).
    `standoff` is measured from the crate's bottom-centre origin (= box centre in xy) to the
    pelvis (base_link) origin.
    """
    dx, dy = crate_xy[0] - robot_xy[0], crate_xy[1] - robot_xy[1]
    if mode == 'axis':
        d1, d2 = crate_axis_directions(crate_yaw, axis)
        # direction from crate towards the standing point: the axis sign pointing at the robot
        away = -dx, -dy
        u = d1 if (d1[0] * away[0] + d1[1] * away[1]) >= (d2[0] * away[0] + d2[1] * away[1]) else d2
        px, py = crate_xy[0] + standoff * u[0], crate_xy[1] + standoff * u[1]
        return (px, py, math.atan2(crate_xy[1] - py, crate_xy[0] - px))
    n = math.hypot(dx, dy)
    if n < 1e-6:                      # robot exactly on the crate: keep current heading
        ux, uy = 1.0, 0.0
    else:
        ux, uy = dx / n, dy / n
    return (crate_xy[0] - standoff * ux, crate_xy[1] - standoff * uy, math.atan2(uy, ux))


def crate_in_base(robot, crate_xy):
    """Crate xy in the robot base frame -> (forward, left, bearing, distance)."""
    dx, dy = crate_xy[0] - robot[0], crate_xy[1] - robot[1]
    c, s = math.cos(-robot[2]), math.sin(-robot[2])
    f, l = c * dx - s * dy, s * dx + c * dy
    return f, l, math.atan2(l, f), math.hypot(f, l)


def pose_error(robot, target):
    """(xy distance, |yaw error|) between two (x, y, yaw) poses."""
    return math.hypot(target[0] - robot[0], target[1] - robot[1]), abs(wrap(target[2] - robot[2]))
