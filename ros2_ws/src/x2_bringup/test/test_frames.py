import math
import os

import numpy as np

from x2_bringup import se3
from x2_bringup.odom_frames import Extrinsics, base_in_odom, crate_in_odom
from x2_bringup.urdf_fk import Urdf

URDF = Urdf.load(os.path.join(os.path.dirname(__file__), '..', 'urdf', 'x2_ultra.urdf'))
EXT = Extrinsics.from_fk(URDF, {})
# previously fixed defaults (tools/compute_x2_extrinsics.py at waist = head = 0)
FIX_BASE = ([-0.33299, -0.01488, 0.08942], [0.001157, 0.705007, -0.000780, 0.709199])
FIX_CAM = ([0.17236, -0.02741, 0.02646], [0.666033, 0.664329, -0.239351, -0.240370])


def test_fk_q0_reproduces_fixed_defaults():
    for T, (xyz, quat) in ((EXT.T_tracked_base, FIX_BASE), (EXT.T_tracked_cam, FIX_CAM)):
        assert np.allclose(T[:3, 3], xyz, atol=1e-4)
        assert np.allclose(T, se3.make(xyz, quat), atol=1e-4)


def test_waist_yaw_rotates_pelvis_heading():
    e2 = Extrinsics.from_fk(URDF, {'waist_yaw_joint': 0.2})
    R_rel = EXT.T_tracked_base[:3, :3].T @ e2.T_tracked_base[:3, :3]     # pelvis(q) in pelvis(0)
    # R_IL is only given to 6 decimals (not exactly orthonormal) -> 1e-4 tolerance
    ang = math.atan2(math.hypot(R_rel[2, 1] - R_rel[1, 2], R_rel[0, 2] - R_rel[2, 0], R_rel[1, 0] - R_rel[0, 1]),
                     np.trace(R_rel) - 1)
    assert abs(ang - 0.2) < 1e-4                       # rotation angle = waist yaw
    assert abs(abs(R_rel[0, 1]) - math.sin(0.2)) < 1e-4 and abs(R_rel[2, 2] - 1) < 1e-4   # about the vertical


def test_head_yaw_moves_camera_only():
    e2 = Extrinsics.from_fk(URDF, {'head_yaw_joint': 0.3})
    assert np.allclose(e2.T_tracked_base, EXT.T_tracked_base)
    assert not np.allclose(e2.T_tracked_cam, EXT.T_tracked_cam)


def test_sim_identity():
    o = {'position': [1, 2, 0], 'quat_xyzw': [0, 0, 0, 1], 'frame_id': 'odom'}
    assert np.allclose(base_in_odom(o, EXT)[:3, 3], [1, 2, 0])
    c = {'position': [3, 4, 0], 'quat_xyzw': [0, 0, 0, 1], 'frame_id': 'odom'}
    assert np.allclose(crate_in_odom(c, o, EXT)[:3, 3], [3, 4, 0])


def test_kilvo_base_and_camera_consistent():
    o = {'position': [0, 0, 0], 'quat_xyzw': [0, 0, 0, 1], 'frame_id': 'camera_init'}
    Tb = base_in_odom(o, EXT)
    Tbc = se3.inv(Tb) @ EXT.T_tracked_cam
    assert np.allclose(Tbc[:3, 3], [0.066, -0.0112, 0.505], atol=2e-3)
    assert np.allclose(Tbc[:3, 2], [0.766, 0, -0.643], atol=2e-3)       # optical axis, 40 deg down
    crate = {'position': [0, 0, 1.2], 'quat_xyzw': [0, 0, 0, 1], 'frame_id': 'rgbd_head_front'}
    in_base = se3.inv(Tb) @ crate_in_odom(crate, o, EXT)
    assert abs(in_base[0, 3] - 0.985) < 5e-3 and abs(in_base[1, 3]) < 0.02


def test_quat_roundtrip():
    q = np.array([0.1, -0.3, 0.5, 0.8])
    q = q / np.linalg.norm(q)
    assert np.allclose(se3.mat_to_quat(se3.quat_to_mat(q)) * np.sign(q[3]), q, atol=1e-9)
