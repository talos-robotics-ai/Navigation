"""Pure tests (no rclpy) of the camera-pose history, the laptop crate conversion and the RGB-D encoders."""
import math
import os

import numpy as np
import pytest

from x2_bringup import nav_image_proto as ip
from x2_bringup import se3
from x2_bringup.cam_buffer import CamPoseBuffer, crate_in_odom_from_cam
from x2_bringup.odom_frames import Extrinsics, cam_in_odom, crate_in_odom
from x2_bringup.urdf_fk import Urdf

URDF = Urdf.load(os.path.join(os.path.dirname(__file__), '..', 'urdf', 'x2_ultra.urdf'))


def test_cam_buffer_nearest_and_trim():
    b = CamPoseBuffer(keep_s=1.0)
    assert b.nearest(0.0) is None
    for i in range(30):
        b.add(i * 0.1, np.eye(4) * 0 + i)
    assert b.nearest(2.04)[0][0, 0] == 20
    assert b.nearest(100.0, max_gap_s=0.5) is None
    assert len(b._t) <= 12 and b._t[0] >= 1.9 - 1e-9


def test_laptop_conversion_equals_pc2_conversion():
    ext = Extrinsics.from_fk(URDF, {})
    odom = {'position': [1.0, 2.0, 0.3], 'quat_xyzw': [0, 0, math.sin(0.4), math.cos(0.4)], 'frame_id': 'tracked'}
    crate = {'position': [0.1, -0.2, 1.7], 'quat_xyzw': [0, 0, 0, 1], 'frame_id': 'rgbd_head_front'}
    T_pc2 = crate_in_odom(crate, odom, ext)
    T_lap = crate_in_odom_from_cam(cam_in_odom(odom, ext), crate['position'], crate['quat_xyzw'])
    assert np.allclose(T_pc2, T_lap, atol=1e-9)


cv2 = pytest.importorskip('cv2')


def test_rgb_jpeg_roundtrip_bgr_and_rgb_sources():
    rng = np.random.default_rng(0)
    bgr = np.full((48, 64, 3), (10, 120, 240), np.uint8)
    step = 64 * 3 + 8   # padded rows
    buf = np.zeros((48, step), np.uint8)
    buf[:, :192] = bgr.reshape(48, 192)
    out = ip.decode_rgb(ip.encode_rgb(buf.tobytes(), 48, 64, step, 'bgr8', 95))
    assert out.shape == (48, 64, 3) and abs(int(out[5, 5, 2]) - 240) < 6
    rgb = bgr[..., ::-1].copy()
    out2 = ip.decode_rgb(ip.encode_rgb(rgb.tobytes(), 48, 64, 192, 'rgb8', 95))
    assert abs(int(out2[5, 5, 2]) - 240) < 6
    with pytest.raises(ValueError):
        ip.encode_rgb(b'', 1, 1, 3, 'mono8')


def test_depth_png_lossless_mm_both_encodings():
    mm = (np.arange(40 * 50, dtype=np.uint16).reshape(40, 50) % 5000)
    d = ip.decode_depth_m(ip.encode_depth(mm.tobytes(), 40, 50, 100, '16UC1'))
    assert np.allclose(d, mm / 1000.0)
    m = np.full((4, 4), 1.234, np.float32)
    m[0, 0] = np.nan
    m[1, 1] = np.inf
    d = ip.decode_depth_m(ip.encode_depth(m.tobytes(), 4, 4, 16, '32FC1'))
    assert d[2, 2] == pytest.approx(1.234) and d[0, 0] == 0 and d[1, 1] == 0
