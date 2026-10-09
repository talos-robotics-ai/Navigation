"""Pure (no rclpy) core of kilvo_base_odom_node: HAL joint parsing and the KILVO-IMU -> pelvis / crate chain.

Kept apart so it can be unit-tested where rclpy must not be imported (PC2's vendor graph).
"""
from .odom_frames import R_IL, T_IL, Extrinsics, base_in_odom, crate_in_odom

#: HAL group -> joint names in the HAL's array order (vhit_bridge.VENDOR; also the URDF joint names)
HAL_JOINTS = {
    'waist': ['waist_yaw_joint', 'waist_pitch_joint', 'waist_roll_joint'],
    'head': ['head_yaw_joint', 'head_pitch_joint'],
}


def joints_from_hal(group, names, positions):
    """{joint: position} of one HAL JointStateArray.

    By name when the message carries the expected names (any order); otherwise by position in the
    HAL's documented order, if the counts match; else {} (the caller keeps the previous values).
    """
    expected = HAL_JOINTS[group]
    named = {n: float(p) for n, p in zip(names, positions) if n in expected}
    if len(named) == len(expected):
        return named
    if len(positions) == len(expected):
        return {n: float(p) for n, p in zip(expected, positions)}
    return {}


class BasePoseChain:
    """Pure core (no rclpy): joints -> extrinsics (cached) -> pelvis / crate pose in odom."""

    def __init__(self, urdf, r_il=R_IL, t_il=T_IL):
        self.urdf, self.r_il, self.t_il = urdf, r_il, t_il
        self.q = {}
        self._key, self._ext = None, None

    def set_joints(self, q):
        self.q.update(q)

    @property
    def ext(self):
        key = tuple(round(self.q.get(n, 0.0), 4) for g in HAL_JOINTS.values() for n in g)
        if key != self._key:   # waist/head move slowly: FK only when a joint moved > 0.1 mrad
            self._ext = Extrinsics.from_fk(self.urdf, self.q, self.r_il, self.t_il)
            self._key = key
        return self._ext

    @staticmethod
    def odom_dict(msg):
        p, o = msg.pose.pose.position, msg.pose.pose.orientation
        return {'position': [p.x, p.y, p.z], 'quat_xyzw': [o.x, o.y, o.z, o.w], 'frame_id': 'tracked'}

    def base(self, odom_d):
        """4x4 T_odom_base from a dict {position, quat_xyzw} of the KILVO IMU pose."""
        return base_in_odom(dict(odom_d, frame_id='tracked'), self.ext)

    def crate(self, pose_d, odom_d):
        """4x4 T_odom_crate from the crate in rgbd_head_front and the KILVO IMU pose, or None."""
        return crate_in_odom(dict(pose_d, frame_id='rgbd_head_front'), dict(odom_d, frame_id='tracked'), self.ext)
