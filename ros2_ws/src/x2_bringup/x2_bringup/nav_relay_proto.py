"""Pure (no rclpy, numpy only) core of the PC2 <-> laptop navigation relay.

Wire format, one TCP stream, both directions: length-prefixed frames

    uint32 BE  header_len | uint32 BE  payload_len | header (UTF-8 JSON object) | payload (bytes)

Headers carry a type "t":
    PC2 -> laptop   odom   {"t","stamp","p":[x,y,z],"q":[x,y,z,w]}          (/x2/odom, odom frame)
                    crate  same fields                                       (/x2/crate_pose)
                    cloud  {"t","stamp","n"} + payload n*3 little-endian float32 xyz   (odom frame)
                    hb     {"t","stamp"}
    laptop -> PC2   cmd    {"t","vx","vy","wz"}                              (20 Hz)
                    estop  {"t","value":true|false}
                    hb     {"t","stamp"}

Safety of the command direction (CmdHold): fresh command -> pass; no command for `stale_s` or the link
dropped -> zeros for `zero_tail_s`, then nothing (the consumer, mc_velocity_node, repeats that with its
own stale -> zeros -> silence gate).
"""
import json
import math
import struct

import numpy as np

HDR = struct.Struct('>II')
MAX_HEADER = 64 * 1024
MAX_PAYLOAD = 32 * 1024 * 1024


class FrameError(ValueError):
    pass


def encode_frame(header, payload=b''):
    h = json.dumps(header, separators=(',', ':')).encode()
    return HDR.pack(len(h), len(payload)) + h + bytes(payload)


class FrameDecoder:
    """Incremental decoder: feed(bytes) -> [(header_dict, payload_bytes), ...]. Raises FrameError on garbage."""

    def __init__(self):
        self._buf = bytearray()

    def feed(self, data):
        self._buf += data
        out = []
        while len(self._buf) >= HDR.size:
            hl, pl = HDR.unpack_from(self._buf)
            if hl == 0 or hl > MAX_HEADER or pl > MAX_PAYLOAD:
                raise FrameError(f'bad frame sizes header={hl} payload={pl}')
            end = HDR.size + hl + pl
            if len(self._buf) < end:
                break
            try:
                header = json.loads(bytes(self._buf[HDR.size:HDR.size + hl]))
            except ValueError as e:
                raise FrameError(f'bad header json: {e}') from None
            if not isinstance(header, dict):
                raise FrameError('header is not an object')
            out.append((header, bytes(self._buf[HDR.size + hl:end])))
            del self._buf[:end]
        return out


# ---------------------------------------------------------------------------------------- clouds
def cloud_xyz(data, point_step, n_points, offsets):
    """(N,3) float32 xyz from a PointCloud2 `data` buffer whose x,y,z are little-endian float32 at `offsets`."""
    if n_points == 0:
        return np.empty((0, 3), np.float32)
    raw = np.frombuffer(data, dtype=np.uint8, count=n_points * point_step).reshape(n_points, point_step)
    out = np.empty((n_points, 3), np.float32)
    for i, off in enumerate(offsets):
        out[:, i] = raw[:, off:off + 4].copy().view('<f4')[:, 0]
    return out


def crop_radius(xyz, center_xy, radius):
    """Points within `radius` (horizontal) of center_xy; non-finite points dropped."""
    if not len(xyz):
        return xyz
    keep = np.isfinite(xyz).all(axis=1)
    d = xyz[:, :2] - np.asarray(center_xy, np.float32)
    keep &= (d * d).sum(axis=1) <= radius * radius
    return xyz[keep]


def voxel_downsample(xyz, voxel):
    """One point (the first) per voxel of edge `voxel`. Keys packed into one int64 (21 bits per axis)."""
    if len(xyz) == 0 or voxel <= 0:
        return xyz
    idx = np.floor(xyz / np.float32(voxel)).astype(np.int64) + (1 << 20)
    idx = np.clip(idx, 0, (1 << 21) - 1)
    key = (idx[:, 0] << 42) | (idx[:, 1] << 21) | idx[:, 2]
    _, first = np.unique(key, return_index=True)
    return xyz[np.sort(first)]


def obstacle_cloud(xyz, center_xy, radius, voxel):
    """The relayed cloud: cropped around `center_xy`, then voxel-downsampled."""
    return voxel_downsample(crop_radius(xyz, center_xy, radius), voxel)


def pack_cloud(xyz):
    return np.ascontiguousarray(xyz, dtype='<f4').tobytes()


def unpack_cloud(payload, n):
    if len(payload) != n * 12:
        raise FrameError(f'cloud payload {len(payload)} B for {n} points')
    return np.frombuffer(payload, dtype='<f4').reshape(n, 3)


# ---------------------------------------------------------------------------------------- commands
class CmdHold:
    """Latest {vx,vy,wz} from the link -> what to publish now, or None (stay silent).

    fresh (age <= stale_s)         -> the command (finite, clamped to +-cap)
    stale / link dropped / estop   -> zeros until zero_tail_s after the last fresh output (or the drop)
    afterwards                     -> None
    """

    def __init__(self, stale_s=0.3, zero_tail_s=0.3, cap=(1.0, 1.0, 1.0)):
        self.stale_s, self.zero_tail_s, self.cap = stale_s, zero_tail_s, cap
        self.cmd = None          # (vx, vy, wz, t_rx)
        self.zero_until = -math.inf

    def set(self, vx, vy, wz, now):
        self.cmd = (float(vx), float(vy), float(wz), now)

    def drop(self, now):
        """Link lost: zeros from now for zero_tail_s."""
        self.cmd = None
        self.zero_until = max(self.zero_until, now + self.zero_tail_s)

    def step(self, now, estop=False):
        c = self.cmd
        if c is not None and now - c[3] <= self.stale_s and all(map(math.isfinite, c[:3])):
            self.zero_until = now + self.zero_tail_s
            if estop:
                return (0.0, 0.0, 0.0)
            return tuple(max(-k, min(k, v)) for v, k in zip(c[:3], self.cap))
        if now < self.zero_until:
            return (0.0, 0.0, 0.0)
        return None


def cmd_frame(vx, vy, wz):
    return encode_frame({'t': 'cmd', 'vx': float(vx), 'vy': float(vy), 'wz': float(wz)})


def local_cmd(twist_xyz_age, stale_s):
    """Client side: (vx, vy, wz, age) of the local /x2/cmd_vel_out -> what to send (zeros if stale or none)."""
    if twist_xyz_age is None:
        return (0.0, 0.0, 0.0)
    vx, vy, wz, age = twist_xyz_age
    if age > stale_s or not all(map(math.isfinite, (vx, vy, wz))):
        return (0.0, 0.0, 0.0)
    return (vx, vy, wz)
