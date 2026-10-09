"""Pure tests (no rclpy: they run on PC2 too) of the onboard KILVO -> pelvis chain."""
import math
import os

import numpy as np

from x2_bringup import se3
from x2_bringup.onboard_chain import HAL_JOINTS, BasePoseChain, joints_from_hal
from x2_bringup.urdf_fk import Urdf

URDF = Urdf.load(os.path.join(os.path.dirname(__file__), '..', 'urdf', 'x2_ultra.urdf'))
FIX_BASE = ([-0.33299, -0.01488, 0.08942], [0.001157, 0.705007, -0.000780, 0.709199])   # IMU->pelvis at q=0


def imu_pose(xyz=(0, 0, 0), yaw=0.0):
    return {'position': list(xyz), 'quat_xyzw': [0, 0, math.sin(yaw / 2), math.cos(yaw / 2)]}


def test_hal_names_match_urdf():
    for names in HAL_JOINTS.values():
        for n in names:
            assert n in {v[1] for v in URDF.joints.values()}


def test_joints_by_name_any_order_and_by_position():
    assert joints_from_hal('head', ['head_pitch_joint', 'head_yaw_joint'], [0.2, 0.1]) == \
        {'head_pitch_joint': 0.2, 'head_yaw_joint': 0.1}
    assert joints_from_hal('waist', ['', '', ''], [0.1, 0.2, 0.3])['waist_roll_joint'] == 0.3   # unnamed: HAL order
    assert joints_from_hal('waist', ['a'], [1.0]) == {}


def test_identity_imu_gives_fixed_extrinsic_at_zero_joints():
    T = BasePoseChain(URDF).base(imu_pose())
    assert np.allclose(T, se3.make(*FIX_BASE), atol=1e-4)


def test_pelvis_follows_imu_translation_and_yaw():
    c = BasePoseChain(URDF)
    T = c.base(imu_pose((1.0, 2.0, 0.5), yaw=math.pi / 2))
    T0 = c.base(imu_pose())
    assert np.allclose(T[:3, :3], se3.quat_to_mat([0, 0, math.sin(math.pi / 4), math.cos(math.pi / 4)]) @ T0[:3, :3], atol=1e-6)
    assert np.allclose(T[:3, 3], np.array([1.0, 2.0, 0.5]) + T[:3, :3] @ np.zeros(3) + se3.quat_to_mat([0, 0, math.sin(math.pi / 4), math.cos(math.pi / 4)]) @ T0[:3, 3], atol=1e-6)


def test_waist_yaw_moves_pelvis_but_cache_follows_joints():
    c = BasePoseChain(URDF)
    T0 = c.base(imu_pose())
    c.set_joints({'waist_yaw_joint': 0.3})
    T1 = c.base(imu_pose())
    assert not np.allclose(T0, T1, atol=1e-3)
    c.set_joints({'waist_yaw_joint': 0.0})
    assert np.allclose(c.base(imu_pose()), T0, atol=1e-9)


def test_crate_in_odom_uses_head_camera_chain():
    c = BasePoseChain(URDF)
    # crate 1 m in front of the camera along the optical z axis
    T = c.crate({'position': [0, 0, 1.0], 'quat_xyzw': [0, 0, 0, 1]}, imu_pose())
    cam = c.ext.T_tracked_cam
    assert np.allclose(T[:3, 3], (cam @ np.array([0, 0, 1.0, 1]))[:3], atol=1e-9)


def test_ground_removal_keeps_a_box_drops_the_floor():
    from g1_local_map.ground_segmentation import GroundParams, segment_ground
    xs, ys = np.meshgrid(np.arange(-4, 4, 0.1), np.arange(-4, 4, 0.1))
    floor = np.c_[xs.ravel(), ys.ravel(), np.full(xs.size, -0.66)]
    bx, by, bz = np.meshgrid(np.arange(1.5, 1.9, 0.1), np.arange(-0.2, 0.2, 0.1), np.arange(-0.6, 0.0, 0.1))
    box = np.c_[bx.ravel(), by.ravel(), bz.ravel()]
    p = GroundParams(cell=0.4, min_pts=4, ground_band=0.14, leg_offset=0.66, max_height=1.5, min_total=200)
    obs, _ = segment_ground(np.vstack([floor, box]), np.array([0, 0, -1.0]), 0.0, p, return_info=True)
    assert len(obs) >= 0.8 * len(box) and (obs[:, 2] > -0.55).all() and (np.abs(obs[:, 0] - 1.7) < 0.4).all()


def test_mc_deadband_lift_zero_none_and_caps():
    from x2_bringup.mc_gate import VelocityGate
    cmd = lambda vx, vy, wz, t=0.0: (vx, vy, wz, t)  # noqa: E731
    g = VelocityGate(mode='lift')
    assert g.step(0.0, cmd(0.1, 0.0, 0.05)) == (0.2, 0.0, 0.1)          # raised to mc's thresholds
    assert g.step(0.0, cmd(0.01, -0.1, -0.01)) == (0.0, -0.2, 0.0)      # below epsilon -> 0, sign kept
    assert g.step(0.0, cmd(2.0, 2.0, -2.0)) == (0.5, 0.3, -0.5)         # caps
    assert VelocityGate(mode='zero').step(0.0, cmd(0.1, 0.0, 0.05)) == (0.0, 0.0, 0.0)
    assert VelocityGate(mode='none').step(0.0, cmd(0.1, 0.0, 0.05)) == (0.1, 0.0, 0.05)
    assert g.step(0.0, cmd(float('nan'), 0.0, 0.0)) == (0.0, 0.0, 0.0)


def test_mc_gate_stale_zero_tail_then_silence_and_estop():
    from x2_bringup.mc_gate import VelocityGate
    g = VelocityGate()
    assert g.step(0.0, (0.3, 0.0, 0.0, 0.0)) == (0.3, 0.0, 0.0)
    assert g.step(0.2, (0.3, 0.0, 0.0, 0.0)) == (0.3, 0.0, 0.0)
    assert g.step(0.4, (0.3, 0.0, 0.0, 0.0)) == (0.0, 0.0, 0.0)         # stale (> 0.3 s) -> zeros ...
    assert g.step(0.45, (0.3, 0.0, 0.0, 0.0)) == (0.0, 0.0, 0.0)
    assert g.step(0.6, (0.3, 0.0, 0.0, 0.0)) is None                    # ... for 0.3 s from the last good one, then silence
    g2 = VelocityGate()
    g2.step(0.0, (0.3, 0, 0, 0.0))
    assert g2.step(0.1, (0.3, 0, 0, 0.1), estop=True) == (0.0, 0.0, 0.0)
    assert g2.step(0.5, (0.3, 0, 0, 0.5), estop=True) is None
    assert g2.step(0.6, (0.3, 0, 0, 0.6), estop=False) == (0.3, 0.0, 0.0)
