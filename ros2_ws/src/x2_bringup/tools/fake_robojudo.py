#!/usr/bin/env python3
"""Fake RoboJuDo X2 server (stdlib + math only) for testing without robot / sim.

Same TCP JSON-lines protocol as the real server (default 127.0.0.1:8770):
  client -> server : {"vx","wz","arm":[14]|null,"hand_l":[10]|null,"hand_r":[10]|null}
  server -> client : {"t","fresh","vx_applied","wz_applied","arm_names","q_arm","odom","odom_age_ms",
                      "crate","crate_age_ms","mode"}   at 20 Hz
Planar unicycle integrated at 50 Hz; commands older than 0.3 s -> stand still (like the real one).
odom / crate are reported in frame "odom" (child "base") -- the SIM convention.

  --crate X Y YAW        crate (bottom-centre) world pose, default 2.5 0.4 0.3
  --start X Y YAW        initial base pose, default 0 0 0
  --crate-dropout A:B    crate not reported for t in [A, A+B) s after the first client connects;
                         may be repeated. SIGUSR1 toggles dropout manually.
"""
import argparse
import json
import math
import signal
import socket
import threading
import time

ARM_NAMES = [f'{s}_{j}' for s in ('left', 'right') for j in (
    'shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow_pitch', 'wrist_yaw', 'wrist_pitch', 'wrist_roll')]
JOINT_NAMES = ['waist_yaw_joint', 'waist_pitch_joint', 'waist_roll_joint', 'head_yaw_joint', 'head_pitch_joint']
ARM_DEFAULT = [0.3, 0.2, 0, -0.8, 0, 0, 0, 0.3, -0.2, 0, -0.8, 0, 0, 0]   # X2HzdWalkDoF default_pos


def quat_z(yaw):
    return [0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)]


class Sim:
    def __init__(self, a):
        self.x, self.y, self.yaw = a.start
        self.crate = a.crate
        self.q_other = [a.waist_yaw, 0.0, 0.0, 0.0, 0.0]
        self.dropouts = a.crate_dropout
        self.manual_hide = False
        self.cmd = dict(vx=0.0, wz=0.0, arm=None, hand_l=None, hand_r=None)
        self.t_cmd = -1e9
        self.vx_a = self.wz_a = 0.0
        self.q_arm = list(ARM_DEFAULT)
        self.t0 = None
        self.lock = threading.Lock()

    def step(self, dt):
        with self.lock:
            fresh = time.monotonic() - self.t_cmd < 0.3
            vx = max(0.0, min(0.5, float(self.cmd['vx'] or 0.0))) if fresh else 0.0
            wz = max(-0.5, min(0.5, float(self.cmd['wz'] or 0.0))) if fresh else 0.0
            # first-order actuator lag like a gait
            self.vx_a += (vx - self.vx_a) * min(1.0, dt / 0.15)
            self.wz_a += (wz - self.wz_a) * min(1.0, dt / 0.12)
            self.x += self.vx_a * math.cos(self.yaw) * dt
            self.y += self.vx_a * math.sin(self.yaw) * dt
            self.yaw += self.wz_a * dt
            tgt = self.cmd['arm'] if (self.cmd['arm'] and fresh) else ARM_DEFAULT
            for i in range(14):                       # 2 rad/s joint rate limit
                d = tgt[i] - self.q_arm[i]
                self.q_arm[i] += max(-2.0 * dt, min(2.0 * dt, d))
            return fresh

    def crate_hidden(self):
        if self.manual_hide:
            return True
        if self.t0 is None:
            return False
        t = time.monotonic() - self.t0
        return any(a <= t < a + b for a, b in self.dropouts)

    def snapshot(self):
        with self.lock:
            now = time.time()
            hid = self.crate_hidden()
            cx, cy, cyaw = self.crate
            return {
                't': now, 'fresh': time.monotonic() - self.t_cmd < 0.3,
                'vx_applied': self.vx_a, 'wz_applied': self.wz_a,
                'arm_names': ARM_NAMES, 'q_arm': list(self.q_arm),
                'joint_names': JOINT_NAMES, 'q': list(self.q_other),
                'odom': {'position': [self.x, self.y, 0.0], 'quat_xyzw': quat_z(self.yaw),
                         'lin_vel': [self.vx_a * math.cos(self.yaw), self.vx_a * math.sin(self.yaw), 0.0],
                         'frame_id': 'odom', 'child_frame_id': 'base', 'stamp': now},
                'odom_age_ms': 5.0,
                'crate': None if hid else {'position': [cx, cy, 0.0], 'quat_xyzw': quat_z(cyaw),
                                           'frame_id': 'odom', 'stamp': now},
                'crate_age_ms': None if hid else 50.0,
                'mode': 'fake'}


def serve_client(conn, sim):
    if sim.t0 is None:
        sim.t0 = time.monotonic()
    stop = threading.Event()

    def reader():
        buf = b''
        try:
            while not stop.is_set():
                chunk = conn.recv(65536)
                if not chunk:
                    break
                buf += chunk
                *lines, buf = buf.split(b'\n')
                for ln in lines:
                    if ln.strip():
                        try:
                            m = json.loads(ln)
                        except ValueError:
                            continue
                        with sim.lock:
                            for k in ('vx', 'wz', 'arm', 'hand_l', 'hand_r'):
                                if k in m:
                                    sim.cmd[k] = m[k]
                            sim.t_cmd = time.monotonic()
        except OSError:
            pass
        stop.set()

    threading.Thread(target=reader, daemon=True).start()
    try:
        while not stop.is_set():
            conn.sendall((json.dumps(sim.snapshot()) + '\n').encode())
            time.sleep(0.05)
    except OSError:
        pass
    stop.set()
    conn.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=8770)
    ap.add_argument('--crate', nargs=3, type=float, default=[2.5, 0.4, 0.3], metavar=('X', 'Y', 'YAW'))
    ap.add_argument('--start', nargs=3, type=float, default=[0.0, 0.0, 0.0], metavar=('X', 'Y', 'YAW'))
    ap.add_argument('--waist-yaw', type=float, default=0.0, help='reported waist_yaw_joint [rad]')
    ap.add_argument('--crate-dropout', action='append', default=[], metavar='A:B')
    a = ap.parse_args()
    a.crate_dropout = [tuple(float(v) for v in s.split(':')) for s in a.crate_dropout]
    sim = Sim(a)
    signal.signal(signal.SIGUSR1, lambda *_: (setattr(sim, 'manual_hide', not sim.manual_hide),
                                              print('crate hidden =', sim.manual_hide, flush=True)))

    def loop():
        dt = 0.02
        nxt = time.monotonic()
        while True:
            sim.step(dt)
            nxt += dt
            time.sleep(max(0.0, nxt - time.monotonic()))

    threading.Thread(target=loop, daemon=True).start()
    srv = socket.socket()
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((a.host, a.port))
    srv.listen(2)
    print(f'fake RoboJuDo on {a.host}:{a.port}, crate={a.crate}, start={a.start}', flush=True)
    while True:
        c, _ = srv.accept()
        print('client connected', flush=True)
        threading.Thread(target=serve_client, args=(c, sim), daemon=True).start()


if __name__ == '__main__':
    main()
