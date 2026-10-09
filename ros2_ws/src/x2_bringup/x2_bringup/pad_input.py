"""The PS5 pad's input to the laptop relay client (no rclpy): JSON lines on 127.0.0.1:8771, one client.

pad -> here:  {"vx", "wz", "deadman": bool, "engage": bool?}   at ~20 Hz (pad_walker.py --nav)
here -> pad:  {"phase", "pnp", "override", "nav_vx", "nav_wz"}  status at ~5 Hz

Arbitration (nav_relay_client): while the deadman (L1) is held and the pad is fresh (< stale_s), the pad's
vx/wz replace the navigation command and /x2/pad_override is true (the FSM pauses in MANUAL until reset).
A pad that goes silent counts as released. An estop on the robot side still zeroes everything.
"""
import json
import socket
import threading
import time


class PadInput:
    def __init__(self, host='127.0.0.1', port=8771, stale_s=0.3, log=print):
        self.stale_s = stale_s
        self.log = log
        self._lock = threading.Lock()
        self._cmd = None            # (vx, wz, deadman, t)
        self._engage = False
        self._conn = None
        self.status = {}
        self._srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._srv.bind((host, int(port)))
        self._srv.listen(1)
        self._stop = False
        threading.Thread(target=self._serve, daemon=True, name='pad_input').start()

    def get(self, now=None):
        """(vx, wz) while the deadman is held and the pad is fresh, else None."""
        now = now or time.monotonic()
        with self._lock:
            c = self._cmd
        if c is None or not c[2] or now - c[3] > self.stale_s:
            return None
        return c[0], c[1]

    def take_engage(self):
        with self._lock:
            e, self._engage = self._engage, False
        return e

    def send_status(self, **kw):
        line = (json.dumps(kw, separators=(',', ':')) + '\n').encode()
        with self._lock:
            c = self._conn
        if c is not None:
            try:
                c.sendall(line)
            except OSError:
                pass

    def _serve(self):
        while not self._stop:
            try:
                conn, addr = self._srv.accept()
            except OSError:
                return
            conn.settimeout(1.0)
            with self._lock:
                self._conn = conn
            self.log(f'pad connected from {addr[0]}:{addr[1]}')
            buf = b''
            while not self._stop:
                try:
                    chunk = conn.recv(4096)
                except socket.timeout:
                    continue
                except OSError:
                    chunk = b''
                if not chunk:
                    break
                buf += chunk
                *lines, buf = buf.split(b'\n')
                for ln in lines:
                    try:
                        m = json.loads(ln)
                    except ValueError:
                        continue
                    with self._lock:
                        if m.get('engage'):
                            self._engage = True
                        if 'vx' in m or 'deadman' in m:
                            self._cmd = (float(m.get('vx', 0.0)), float(m.get('wz', 0.0)),
                                         bool(m.get('deadman', False)), time.monotonic())
            with self._lock:
                self._conn, self._cmd = None, None
            try:
                conn.close()
            except OSError:
                pass
            self.log('pad disconnected (counts as released)')

    def close(self):
        self._stop = True
        try:
            self._srv.close()
        except OSError:
            pass
