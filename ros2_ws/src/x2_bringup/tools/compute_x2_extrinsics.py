#!/usr/bin/env python3
"""Derive the KILVO-tracked-frame -> pelvis / head-camera extrinsics for the X2.

Chain (all fixed at waist = head = 0, see x2_bringup/README.md):
  T_imu_base = T_imu_lidar * T_lidar_torso * T_torso_pelvis
  T_imu_cam  = T_imu_lidar * T_lidar_torso * T_torso_cam
Sources:
  * KILVO config/x2.yaml extrin_calib extrinsic_R/T : p_imu = R p_lidar + T  (LiDAR -> IMU)
  * x2_ultra.urdf : lidar_chest_front, rgbd_head_front (both under torso_link), waist/head joints.
Usage: compute_x2_extrinsics.py [path/to/x2_ultra.urdf]
"""
import sys
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation as R

URDF = sys.argv[1] if len(sys.argv) > 1 else (
    "/home/ggal/Workspace/talos-dev/talos_control/packages/talos_assets/src/"
    "talos_assets/robots/x2/models/x2_ultra.urdf")
R_IL = np.array([[0.005948, -0.999979, 0.002737],
                 [0.000525, 0.002741, 0.999960],
                 [-0.999982, -0.005946, 0.000542]])
T_IL = np.array([0.00427, -0.01575, -0.01121])


def se3(Rm, t):
    T = np.eye(4); T[:3, :3] = Rm; T[:3, 3] = t
    return T


joints = {}
for j in ET.parse(URDF).getroot().iter("joint"):
    o = j.find("origin")
    xyz = [float(v) for v in o.get("xyz", "0 0 0").split()]
    rpy = [float(v) for v in o.get("rpy", "0 0 0").split()]
    joints[j.find("child").get("link")] = (j.find("parent").get("link"),
                                           se3(R.from_euler("xyz", rpy).as_matrix(), xyz))


def chain(link, stop):
    """T_stop_link at all joint angles = 0."""
    T = np.eye(4)
    while link != stop:
        parent, Tpc = joints[link]
        T = Tpc @ T
        link = parent
    return T


T_imu_lidar = se3(R_IL, T_IL)
T_torso_lidar = chain("lidar_chest_front", "torso_link")
T_torso_cam = chain("rgbd_head_front", "torso_link")
T_pelvis_torso = chain("torso_link", "pelvis")
T_torso_pelvis = np.linalg.inv(T_pelvis_torso)
T_imu_base = T_imu_lidar @ np.linalg.inv(T_torso_lidar) @ T_torso_pelvis
T_imu_cam = T_imu_lidar @ np.linalg.inv(T_torso_lidar) @ T_torso_cam
for name, T in (("tracked_to_base", T_imu_base), ("tracked_to_cam", T_imu_cam)):
    q = R.from_matrix(T[:3, :3]).as_quat()
    print(f"{name}_xyz:  [{T[0,3]:.5f}, {T[1,3]:.5f}, {T[2,3]:.5f}]")
    print(f"{name}_quat: [{q[0]:.6f}, {q[1]:.6f}, {q[2]:.6f}, {q[3]:.6f}]   # xyzw")
# sanity: pelvis forward (+x_base) and camera optical axis (+z_cam) in the torso/base frame
T_base_cam = np.linalg.inv(T_imu_base) @ T_imu_cam
print("cam position in base_link:", np.round(T_base_cam[:3, 3], 4))
print("cam +z (optical axis) in base_link:", np.round(T_base_cam[:3, 2], 3))
print("cam +x in base_link:", np.round(T_base_cam[:3, 0], 3), " cam +y:", np.round(T_base_cam[:3, 1], 3))
print("IMU->base translation magnitude:", np.round(np.linalg.norm(T_imu_base[:3, 3]), 3))
