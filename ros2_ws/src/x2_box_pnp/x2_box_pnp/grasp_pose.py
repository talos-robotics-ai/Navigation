"""Fuse several 6D crate poses (taken while the robot stands still at the pre-grasp pose) into one grasp pose.

Pure numpy, no rclpy. The crate is symmetric under a 180 deg turn about its own z axis (the detector may report
either), so every pose is first brought to the same branch as the first one before averaging:
  position    = per-axis median (robust to the odd bad registration)
  orientation = sign-aligned quaternion mean (Markley-style for small spreads), after the z-symmetry alignment
The spread (position std, max angular deviation from the result) is returned so the caller can refuse a noisy set.
"""
import math

import numpy as np


def _qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([aw * bx + ax * bw + ay * bz - az * by,
                     aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw,
                     aw * bw - ax * bx - ay * by - az * bz])


Z180 = np.array([0.0, 0.0, 1.0, 0.0])     # 180 deg about the crate's own z (x, y, z, w)


def _angle(a, b):
    d = abs(float(np.dot(a, b)))
    return 2.0 * math.acos(min(1.0, d))


def fuse(poses):
    """poses: list of (position[3], quat_xyzw[4]) in one frame. Returns (position, quat_xyzw, stats) or None."""
    if not poses:
        return None
    P = np.array([p for p, _ in poses], float)
    ref = np.asarray(poses[0][1], float)
    ref /= np.linalg.norm(ref)
    Q = []
    for _, q in poses:
        q = np.asarray(q, float)
        q /= np.linalg.norm(q)
        alt = _qmul(q, Z180)                       # the same box turned 180 deg about its z
        q = q if _angle(q, ref) <= _angle(alt, ref) else alt
        Q.append(q if np.dot(q, ref) >= 0 else -q)
    Q = np.array(Q)
    q = Q.mean(axis=0)
    q /= np.linalg.norm(q)
    p = np.median(P, axis=0)
    stats = {'n': len(poses),
             'pos_std_m': float(np.linalg.norm(P.std(axis=0))),
             'max_ang_dev_deg': float(max(math.degrees(_angle(qi, q)) for qi in Q))}
    return p, q, stats


def to_base(p, q, base_xy_yaw_z):
    """Express an odom pose in the robot's planar base frame (x forward, y left; base = /x2/odom pelvis
    position (x, y, z) and yaw). Returns (position, quat_xyzw)."""
    bx, by, byaw, bz = base_xy_yaw_z
    c, s = math.cos(byaw), math.sin(byaw)
    d = np.asarray(p, float) - np.array([bx, by, bz])
    pb = np.array([c * d[0] + s * d[1], -s * d[0] + c * d[1], d[2]])
    qb = _qmul(np.array([0.0, 0.0, -math.sin(byaw / 2), math.cos(byaw / 2)]), np.asarray(q, float))
    return pb, qb / np.linalg.norm(qb)
