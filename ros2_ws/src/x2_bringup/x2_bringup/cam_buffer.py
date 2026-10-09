"""Pure (no rclpy): short history of T_odom_cam, and the laptop-side crate conversion.

The detector (on the laptop) reports the crate in the camera's optical frame `rgbd_head_front`, stamped with the
CAMERA clock; its latency (image -> pose) is ~0.3 s. Odom/camera poses arrive over the relay with the laptop's
arrival time. The crate is placed in odom with the camera pose closest to  arrival_time - latency  (TODO: measure
the real latency; with a stationary robot it does not matter).
"""
import bisect

import numpy as np

from . import se3


class CamPoseBuffer:
    def __init__(self, keep_s=3.0):
        self.keep_s = keep_s
        self._t, self._T = [], []

    def add(self, t, T):
        self._t.append(t)
        self._T.append(T)
        k = bisect.bisect_left(self._t, t - self.keep_s)
        if k:
            del self._t[:k], self._T[:k]

    def nearest(self, t, max_gap_s=1.0):
        """(T, |gap|) of the stored pose closest in time to t, or None if empty or further than max_gap_s."""
        if not self._t:
            return None
        i = bisect.bisect_left(self._t, t)
        cand = [j for j in (i - 1, i) if 0 <= j < len(self._t)]
        j = min(cand, key=lambda k: abs(self._t[k] - t))
        gap = abs(self._t[j] - t)
        return (self._T[j], gap) if gap <= max_gap_s else None


def crate_in_odom_from_cam(T_odom_cam, position, quat_xyzw):
    """T_odom_crate = T_odom_cam * T_cam_crate."""
    return T_odom_cam @ se3.make(position, quat_xyzw)
