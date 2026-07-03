"""SONIC ZMQ wire format — vendored into the Navigation stack.

The SONIC whole-body controller (g1_deploy_onnx_ref) is NOT a ROS 2 process: it
SUBs a ZMQ PUB socket and drives the robot over CycloneDDS. This module packs the
`command` / `planner` messages it expects, so cmd_vel_to_sonic_node can be the
"last hop" to that gait — the same role the WebSocket client plays for AMO and
the LocoClient for the Unitree native gait.

This is a self-contained copy of the wire encoder from the SONIC deployment repo
(cmd_vel_to_sonic.py), kept here because that repo is not on the ROS 2 container's
Python path. It intentionally carries NO ROS dependency. If the SONIC wire format
changes upstream (header layout, field order, enum values), mirror it here.

Wire format (matches gear_sonic_deploy ZMQPackedMessageSubscriber):
    message = topic_bytes + header(1280B, null-padded JSON) + little-endian payload
The header field order MUST match the payload concatenation order — the deploy
slices the buffer by field order.

Deps: pyzmq (rosdep key python3-zmq).
"""
import json
import struct
import time

import zmq

HEADER_SIZE = 1280  # must match ZMQPackedMessageSubscriber::HEADER_SIZE

# LocomotionMode enum (subset). 0=IDLE, 1=SLOW_WALK, 2=WALK, 3=RUN.
MODE_IDLE = 0
MODE_SLOW_WALK = 1
MODE_WALK = 2
MODE_RUN = 3

# ---------------------------------------------------------------------------
# Upper-body (17-DOF) target ordering.
#
# The deploy's optional `upper_body_position` planner field is a float[17] that
# REPLACES the upper-body joints in the reference motion the policy tracks.
# Sending it every tick pins the waist/arms/wrists to a fixed pose while the
# lower body keeps walking under planner control — i.e. carry an object without
# the arms swinging.
#
# The 17 values are ordered by HARDWARE joint index: waist(3), then L/R
# interleaved shoulder-pitch, shoulder-roll, shoulder-yaw, elbow, wrist-roll,
# wrist-pitch, wrist-yaw. Names below are the wire order; edit by name.
UPPER_BODY_JOINTS = [
    "waist_yaw",           # 0  (hw 12)
    "waist_roll",          # 1  (hw 13)  locked on 29-DOF waist-locked G1
    "waist_pitch",         # 2  (hw 14)  locked on 29-DOF waist-locked G1
    "left_shoulder_pitch", # 3  (hw 15)
    "right_shoulder_pitch",# 4  (hw 22)
    "left_shoulder_roll",  # 5  (hw 16)
    "right_shoulder_roll", # 6  (hw 23)
    "left_shoulder_yaw",   # 7  (hw 17)
    "right_shoulder_yaw",  # 8  (hw 24)
    "left_elbow",          # 9  (hw 18)
    "right_elbow",         # 10 (hw 25)
    "left_wrist_roll",     # 11 (hw 19)
    "right_wrist_roll",    # 12 (hw 26)
    "left_wrist_pitch",    # 13 (hw 20)
    "right_wrist_pitch",   # 14 (hw 27)
    "left_wrist_yaw",      # 15 (hw 21)
    "right_wrist_yaw",     # 16 (hw 28)
]
UB_INDEX = {name: i for i, name in enumerate(UPPER_BODY_JOINTS)}

# Default standing pose (radians): arms slightly out, elbows bent ~0.6, wrists
# level. Matches the deploy's default_angles for these joints.
UB_DEFAULT = [
    0.0,   # waist_yaw
    0.0,   # waist_roll
    0.0,   # waist_pitch
    0.2,   # left_shoulder_pitch
    0.2,   # right_shoulder_pitch
    0.2,   # left_shoulder_roll
    -0.2,  # right_shoulder_roll
    0.0,   # left_shoulder_yaw
    0.0,   # right_shoulder_yaw
    0.6,   # left_elbow
    0.6,   # right_elbow
    0.0, 0.0,  # wrist_roll  L,R
    0.0, 0.0,  # wrist_pitch L,R
    0.0, 0.0,  # wrist_yaw   L,R
]

# Named presets. "carry": arms brought forward, elbows bent, forearms level — a
# stable two-handed carry pose. Tune per payload; verify on a hoist.
UB_PRESETS = {
    "default": list(UB_DEFAULT),
    "carry": [
        0.0, 0.0, 0.0,      # waist
        -0.4, -0.4,         # shoulder_pitch  (raise arms forward)
        0.25, -0.25,        # shoulder_roll   (elbows slightly out)
        0.0, 0.0,           # shoulder_yaw
        1.2, 1.2,           # elbow           (~70 deg bend)
        0.0, 0.0,           # wrist_roll
        0.0, 0.0,           # wrist_pitch
        0.0, 0.0,           # wrist_yaw
    ],
}


def build_upper_body(preset="default", overrides=None):
    """Return a 17-vector (radians) in wire order. `overrides` is a dict of
    joint-name -> radians applied on top of the preset."""
    if preset not in UB_PRESETS:
        raise ValueError(
            f"unknown arm preset '{preset}'. valid: {', '.join(sorted(UB_PRESETS))}")
    pose = list(UB_PRESETS[preset])
    for name, val in (overrides or {}).items():
        if name not in UB_INDEX:
            raise ValueError(
                f"unknown upper-body joint '{name}'. valid: {', '.join(UPPER_BODY_JOINTS)}")
        pose[UB_INDEX[name]] = float(val)
    return pose


def _header(fields):
    h = {"v": 1, "endian": "le", "count": 1, "fields": fields}
    hb = json.dumps(h).encode("utf-8")
    if len(hb) > HEADER_SIZE:
        raise ValueError("header too large")
    return hb + b"\x00" * (HEADER_SIZE - len(hb))


class SonicPlannerPublisher:
    """Binds a ZMQ PUB socket and packs `command` / `planner` messages for the
    SONIC deploy controller (which SUBs tcp://<host>:<port>)."""

    def __init__(self, host="*", port=5556):
        self.ctx = zmq.Context.instance()
        self.sock = self.ctx.socket(zmq.PUB)
        self.sock.bind(f"tcp://{host}:{port}")
        time.sleep(0.5)  # let subscribers connect (PUB/SUB slow-joiner)

    def send_command(self, start, stop, planner):
        fields = [
            {"name": "start", "dtype": "u8", "shape": [1]},
            {"name": "stop", "dtype": "u8", "shape": [1]},
            {"name": "planner", "dtype": "u8", "shape": [1]},
        ]
        data = struct.pack("BBB", 1 if start else 0, 1 if stop else 0, 1 if planner else 0)
        self.sock.send(b"command" + _header(fields) + data)

    def send_planner(self, mode, movement, facing, speed=-1.0, height=-1.0,
                     upper_body=None, upper_body_vel=None):
        fields = [
            {"name": "mode", "dtype": "i32", "shape": [1]},
            {"name": "movement", "dtype": "f32", "shape": [3]},
            {"name": "facing", "dtype": "f32", "shape": [3]},
            {"name": "speed", "dtype": "f32", "shape": [1]},
            {"name": "height", "dtype": "f32", "shape": [1]},
        ]
        data = struct.pack("<i", int(mode))
        data += struct.pack("<fff", *[float(x) for x in movement])
        data += struct.pack("<fff", *[float(x) for x in facing])
        data += struct.pack("<f", float(speed))
        data += struct.pack("<f", float(height))
        # Optional upper-body (17-DOF) hold. Header field order must match the
        # payload concatenation order (the deploy slices buffers by field order).
        if upper_body is not None:
            if len(upper_body) != 17:
                raise ValueError("upper_body must have 17 elements")
            if upper_body_vel is None:
                upper_body_vel = [0.0] * 17
            fields.append({"name": "upper_body_position", "dtype": "f32", "shape": [17]})
            fields.append({"name": "upper_body_velocity", "dtype": "f32", "shape": [17]})
            data += struct.pack("<17f", *[float(x) for x in upper_body])
            data += struct.pack("<17f", *[float(x) for x in upper_body_vel])
        self.sock.send(b"planner" + _header(fields) + data)

    def close(self):
        self.sock.close(0)
