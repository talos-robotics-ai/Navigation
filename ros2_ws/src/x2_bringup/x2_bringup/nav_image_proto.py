"""Pure (no rclpy; cv2 + numpy, imported lazily) encoders of the head RGB-D stream (port 5597).

Same framing as nav_relay_proto (length-prefixed JSON header + binary payload). Frame types, PC2 -> laptop:

  info   {"t":"info","frame_id","width","height","K":[9],"D":[...],"distortion_model"}          no payload (1 Hz + on connect)
  rgb    {"t":"rgb","stamp_sec","stamp_nsec","frame_id","width","height","encoding":"jpeg","src_encoding"}
         payload: JPEG of the image AS PUBLISHED (not rotated), decodes with cv2.imdecode to BGR
  depth  {"t":"depth","stamp_sec","stamp_nsec","frame_id","width","height","encoding":"png16","unit":"mm"}
         payload: 16-bit PNG, millimetres, 0 = invalid, decodes with cv2.imdecode(..., cv2.IMREAD_UNCHANGED) to uint16
  hb     {"t":"hb","stamp"}                                                                       (1 Hz)
laptop -> PC2: hb only.
"""
import numpy as np


def _plane(data, h, w, step, channels, dtype):
    isz = np.dtype(dtype).itemsize
    a = np.frombuffer(data, np.uint8, count=h * step).reshape(h, step)
    return np.ascontiguousarray(a[:, :w * channels * isz]).view(dtype).reshape(h, w, channels) if channels > 1 \
        else np.ascontiguousarray(a[:, :w * isz]).view(dtype).reshape(h, w)


def encode_rgb(data, h, w, step, src_encoding, quality=80):
    """JPEG (BGR, as cv2 wants it) of an rgb8 / bgr8 image buffer."""
    import cv2
    img = _plane(data, h, w, step, 3, np.uint8)
    enc = src_encoding.lower()
    if enc == 'rgb8':
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    elif enc != 'bgr8':
        raise ValueError(f'rgb encoding {src_encoding!r}: rgb8 | bgr8')
    ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, int(quality)])
    if not ok:
        raise RuntimeError('jpeg encode failed')
    return buf.tobytes()


def encode_depth(data, h, w, step, src_encoding, level=1):
    """16-bit PNG in millimetres from a 16UC1 (mm) or 32FC1 (m) buffer."""
    import cv2
    if src_encoding == '16UC1':
        mm = _plane(data, h, w, step, 1, '<u2')
    elif src_encoding == '32FC1':
        m = _plane(data, h, w, step, 1, '<f4')
        mm = np.where(np.isfinite(m), np.clip(np.rint(m * 1000.0), 0, 65535), 0).astype(np.uint16)
    else:
        raise ValueError(f'depth encoding {src_encoding!r}: 16UC1 | 32FC1')
    ok, buf = cv2.imencode('.png', mm, [cv2.IMWRITE_PNG_COMPRESSION, int(level)])
    if not ok:
        raise RuntimeError('png encode failed')
    return buf.tobytes()


def decode_depth_m(payload):
    """Laptop side helper: metres float32 from a depth frame payload."""
    import cv2
    mm = cv2.imdecode(np.frombuffer(payload, np.uint8), cv2.IMREAD_UNCHANGED)
    return mm.astype(np.float32) * 1e-3


def decode_rgb(payload):
    """Laptop side helper: BGR uint8 from an rgb frame payload."""
    import cv2
    return cv2.imdecode(np.frombuffer(payload, np.uint8), cv2.IMREAD_COLOR)
