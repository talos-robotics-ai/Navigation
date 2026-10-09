"""Pure frame logic of the RoboJuDo link: server state dict -> poses in the planner's `odom`.

Two cases, selected by the frame_id the server reports:

* "odom" (sim / fake server): the pose is already the base pose and the crate is already
  in odom. Identity -- the same code runs unchanged.
* anything else (robot: KILVO /kilvo/aft_mapped_to_init, frame `camera_init`, child
  `aft_mapped` = the chest LiDAR's IMU): the world frame is used AS odom (gravity aligned,
  z up, fixed), and
      T_odom_base  = T_odom_tracked * T_tracked_base
      T_odom_crate = T_odom_tracked * T_tracked_cam * T_cam_crate   (crate given in rgbd_head_front)
  The extrinsics are ROS params of robojudo_link_node; defaults from tools/compute_x2_extrinsics.py.
"""
import numpy as np

from . import se3


# KILVO config/x2.yaml extrin_calib extrinsic_R/T: p_imu = R p_lidar + T (LiDAR -> IMU; direction
# confirmed in LIVMapper.cpp: pos_imu = extR * p_lidar + extT).
R_IL = [0.005948, -0.999979, 0.002737,
        0.000525, 0.002741, 0.999960,
        -0.999982, -0.005946, 0.000542]
T_IL = [0.00427, -0.01575, -0.01121]
BASE_LINK, LIDAR_LINK, CAM_LINK = 'base_link', 'lidar_chest_front', 'rgbd_head_front'


class Extrinsics:
    """T_tracked_base / T_tracked_cam (tracked = KILVO IMU) -- recomputed from URDF FK + joints."""

    def __init__(self, tracked_to_base, tracked_to_cam):
        self.T_tracked_base = tracked_to_base
        self.T_tracked_cam = tracked_to_cam

    @classmethod
    def from_fk(cls, urdf, q=None, R_il=R_IL, t_il=T_IL):
        """urdf: urdf_fk.Urdf; q: {joint name: position} (waist/head matter; others ignored).

        T_imu_base = T_imu_lidar * inv(T_base_lidar);  T_imu_cam = T_imu_base * T_base_cam.
        base_link == pelvis in the X2 URDF (identity joint).
        """
        T_il = np.eye(4)
        T_il[:3, :3], T_il[:3, 3] = np.asarray(R_il, float).reshape(3, 3), t_il
        T_imu_base = T_il @ se3.inv(urdf.fk(BASE_LINK, LIDAR_LINK, q))
        return cls(T_imu_base, T_imu_base @ urdf.fk(BASE_LINK, CAM_LINK, q))


def _pose(d):
    return se3.make(d['position'], d['quat_xyzw'])


def base_in_odom(odom_dict, ext: Extrinsics):
    """T_odom_base from the server's `odom` entry."""
    T = _pose(odom_dict)
    if odom_dict.get('frame_id', 'odom') == 'odom':
        return T
    return T @ ext.T_tracked_base


def crate_in_odom(crate_dict, odom_dict, ext: Extrinsics):
    """T_odom_crate, or None if it cannot be computed (camera-frame crate but no odom)."""
    Tc = _pose(crate_dict)
    if crate_dict.get('frame_id', 'odom') == 'odom':
        return Tc
    if odom_dict is None:
        return None
    return _pose(odom_dict) @ ext.T_tracked_cam @ Tc
