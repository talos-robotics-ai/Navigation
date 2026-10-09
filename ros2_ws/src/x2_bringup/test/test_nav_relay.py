"""Pure tests (no rclpy: they run on PC2 too) of the navigation relay: framing, downsampling, stale handling, TCP link."""
import socket
import threading
import time

import numpy as np
import pytest

from x2_bringup import nav_relay_proto as P
from x2_bringup.nav_relay_link import LinkClient, LinkServer


# ---------------------------------------------------------------- framing
def test_frame_roundtrip_in_odd_chunks():
    payload = np.arange(30, dtype='<f4').tobytes()
    data = P.encode_frame({'t': 'cloud', 'n': 10}, payload) + P.encode_frame({'t': 'hb'}) + P.cmd_frame(0.1, 0, -0.2)
    dec, got = P.FrameDecoder(), []
    for i in range(0, len(data), 7):
        got += dec.feed(data[i:i + 7])
    assert [h['t'] for h, _ in got] == ['cloud', 'hb', 'cmd']
    assert got[0][1] == payload and got[1][1] == b''
    assert got[2][0]['vx'] == pytest.approx(0.1) and got[2][0]['wz'] == pytest.approx(-0.2)


def test_garbage_and_oversize_rejected():
    with pytest.raises(P.FrameError):
        P.FrameDecoder().feed(b'\xff' * 16)
    with pytest.raises(P.FrameError):
        P.FrameDecoder().feed(P.HDR.pack(5, 10) + b'notjs' + b'x' * 10)
    with pytest.raises(P.FrameError):
        P.FrameDecoder().feed(P.encode_frame([1, 2]) if False else P.HDR.pack(2, 0) + b'[]')


# ---------------------------------------------------------------- clouds
def make_buffer(xyz, step=32, offsets=(0, 4, 8)):
    n = len(xyz)
    buf = np.zeros((n, step), np.uint8)
    for i, off in enumerate(offsets):
        buf[:, off:off + 4] = np.ascontiguousarray(xyz[:, i], '<f4').view(np.uint8).reshape(n, 4)
    return buf.tobytes()


def test_cloud_xyz_with_extra_fields_and_offsets():
    xyz = np.random.default_rng(0).normal(size=(50, 3)).astype(np.float32)
    out = P.cloud_xyz(make_buffer(xyz, 32, (4, 12, 20)), 32, 50, (4, 12, 20))
    assert np.array_equal(out, xyz)
    assert P.cloud_xyz(b'', 32, 0, (0, 4, 8)).shape == (0, 3)


def test_voxel_downsample_one_per_voxel_and_idempotent():
    rng = np.random.default_rng(1)
    xyz = rng.uniform(-3, 3, size=(20000, 3)).astype(np.float32)
    ds = P.voxel_downsample(xyz, 0.1)
    keys = {tuple(k) for k in np.floor(ds / 0.1).astype(int)}
    assert len(keys) == len(ds) < len(xyz)
    assert len(P.voxel_downsample(ds, 0.1)) == len(ds)
    assert set(map(tuple, ds)) <= set(map(tuple, xyz))     # real points, not centroids
    assert len(P.voxel_downsample(xyz[:0], 0.1)) == 0
    assert len(P.voxel_downsample(xyz, 0.0)) == len(xyz)   # off


def test_negative_coordinates_do_not_collide():
    xyz = np.array([[-0.05, 0, 0], [0.05, 0, 0], [-0.05, -0.15, -0.05], [0.05, 0.15, 0.05]], np.float32)
    assert len(P.voxel_downsample(xyz, 0.1)) == 4


def test_crop_radius_and_nonfinite():
    xyz = np.array([[0, 0, 0], [5.9, 0, 9], [6.1, 0, 0], [np.nan, 0, 0], [np.inf, 1, 1], [11, 10, 0]], np.float32)
    out = P.crop_radius(xyz, (5.0, 0.0), 6.0)
    assert len(out) == 3 and np.isfinite(out).all()
    assert len(P.obstacle_cloud(xyz, (0, 0), 6.0, 0.1)) == 2


def test_pack_unpack_cloud():
    xyz = np.random.default_rng(2).normal(size=(7, 3)).astype(np.float32)
    assert np.array_equal(P.unpack_cloud(P.pack_cloud(xyz), 7), xyz)
    with pytest.raises(P.FrameError):
        P.unpack_cloud(b'\0' * 11, 1)


# ---------------------------------------------------------------- stale handling
def run_hold(hold, t0, t1, dt=0.05, estop=False):
    return [(round(t, 3), hold.step(t, estop)) for t in np.arange(t0, t1, dt)]


def test_hold_fresh_passes_then_zero_tail_then_silence():
    h = P.CmdHold(0.3, 0.3)
    assert h.step(0.0) is None                       # never heard: silent
    for t in (1.0, 1.05, 1.10):
        h.set(0.3, 0.0, 0.1, t)
        assert h.step(t) == (0.3, 0.0, 0.1)
    assert h.step(1.35) == (0.3, 0.0, 0.1)           # still fresh (0.25 s old)
    out = {t: v for t, v in run_hold(h, 1.40, 2.2)}
    assert out[1.45] == (0.0, 0.0, 0.0)              # stale -> zeros
    zeros = [t for t, v in out.items() if v == (0.0, 0.0, 0.0)]
    assert zeros and max(zeros) - 1.40 <= 0.4        # ... for about zero_tail_s
    assert out[1.95] is None and out[2.15] is None   # then silence


def test_hold_no_motion_after_silence_and_first_zero_within_stale_s():
    h = P.CmdHold(0.3, 0.3)
    h.set(0.5, 0, 0, 0.0)
    assert h.step(0.0) == (0.5, 0, 0)
    first_zero = next(t for t, v in run_hold(h, 0.0, 1.0, 0.01) if v == (0.0, 0.0, 0.0))
    assert first_zero <= 0.31 + 1e-9
    assert all(v is None or v == (0.0, 0.0, 0.0) for _, v in run_hold(h, 0.35, 3.0))


def test_hold_drop_zeroes_immediately_and_estop_overrides():
    h = P.CmdHold(0.3, 0.3)
    h.set(0.5, 0.2, 0.3, 0.0)
    assert h.step(0.01) == (0.5, 0.2, 0.3)
    h.drop(0.02)
    assert h.step(0.03) == (0.0, 0.0, 0.0)
    assert h.step(0.40) is None
    h.set(0.5, 0, 0, 1.0)
    assert h.step(1.01, estop=True) == (0.0, 0.0, 0.0)


def test_hold_clamps_and_rejects_nonfinite():
    h = P.CmdHold(0.3, 0.3, cap=(0.5, 0.3, 0.5))
    h.set(9, -9, 9, 0.0)
    assert h.step(0.0) == (0.5, -0.3, 0.5)
    h.set(float('nan'), 0, 0, 1.0)
    assert h.step(1.0) in ((0.0, 0.0, 0.0), None)


def test_client_side_stale_local_cmd_is_zero():
    assert P.local_cmd(None, 0.3) == (0, 0, 0)
    assert P.local_cmd((0.2, 0, 0.1, 0.1), 0.3) == (0.2, 0, 0.1)
    assert P.local_cmd((0.2, 0, 0.1, 0.5), 0.3) == (0, 0, 0)
    assert P.local_cmd((float('inf'), 0, 0, 0.0), 0.3) == (0, 0, 0)


# ---------------------------------------------------------------- TCP link (loopback)
def wait_for(cond, t=3.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < t:
        if cond():
            return True
        time.sleep(0.01)
    return False


def test_link_roundtrip_reconnect_and_replacement():
    srv_rx, cli_rx, events = [], [], []
    srv = LinkServer('127.0.0.1', 0, lambda h, p: srv_rx.append((h, p)), lambda l: events.append('conn'),
                     lambda l: events.append('close'), log=lambda s: None)
    cli = LinkClient('127.0.0.1', srv.port, lambda h, p: cli_rx.append((h, p)), lambda l: None, lambda l: None, backoff=0.05)
    assert wait_for(lambda: cli.connected and srv.link is not None)
    assert cli.send(P.cmd_frame(0.1, 0.0, 0.2))
    big = np.zeros((20000, 3), np.float32)
    assert srv.send(P.encode_frame({'t': 'cloud', 'n': 20000}, P.pack_cloud(big)))
    assert wait_for(lambda: srv_rx and cli_rx)
    assert srv_rx[0][0]['t'] == 'cmd' and len(cli_rx[0][1]) == 240000
    # the client goes away: the server hears about it, and a new one can connect
    cli.close()
    assert wait_for(lambda: events.count('close') >= 1 and srv.link is None)
    cli2 = LinkClient('127.0.0.1', srv.port, lambda h, p: None, lambda l: None, lambda l: None, backoff=0.05)
    assert wait_for(lambda: srv.link is not None)
    cli2.close()
    srv.close()


def test_link_latest_wins_and_slow_peer_never_blocks_sender():
    srv = LinkServer('127.0.0.1', 0, lambda h, p: None, lambda l: None, lambda l: None, log=lambda s: None)
    s = socket.create_connection(('127.0.0.1', srv.port))     # connected but never reads
    assert wait_for(lambda: srv.link is not None)
    t0 = time.monotonic()
    for _ in range(300):
        srv.send(P.encode_frame({'t': 'cloud', 'n': 0}, b'\0' * 200000), replace_key='cloud')
        srv.send(P.encode_frame({'t': 'hb'}))
    assert time.monotonic() - t0 < 1.0                           # queueing never blocks
    assert srv.link.dropped > 0
    s.close()
    srv.close()


def test_server_survives_garbage_client():
    events = []
    srv = LinkServer('127.0.0.1', 0, lambda h, p: None, lambda l: None, lambda l: events.append('close'), log=lambda s: None)
    s = socket.create_connection(('127.0.0.1', srv.port))
    s.sendall(b'GET / HTTP/1.1\r\n\r\n' * 10)
    assert wait_for(lambda: 'close' in events)
    s.close()
    srv.close()
