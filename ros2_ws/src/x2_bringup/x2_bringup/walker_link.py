"""Client of the on-robot walker's command port (PnpCtrl protocol, JSON lines on 127.0.0.1:8770). No rclpy.

The walker (packages/x2_pnp/onrobot_walker, on PC2) reads lines {"vx", "wz", "arm"?, "hand_l"?, "hand_r"?,
"engage"?} and writes state lines at ~20 Hz carrying at least {"phase", "engage_result"}. It enforces its own
safety: no command line for 0.3 s -> vx = wz = 0 (stands balancing); vx/wz ignored unless phase == live;
"engage" only acts in phase == stance. This class only moves lines; it never invents a command.
"""
import json
import socket
import threading
import time


class WalkerLink:
    def __init__(self, host='127.0.0.1', port=8770, reconnect_s=1.0, log=print):
        self.addr = (host, int(port))
        self.reconnect_s = reconnect_s
        self.log = log
        self._sock = None
        self._lock = threading.Lock()
        self.state = None           # last state line (dict)
        self.t_state = 0.0          # monotonic time of the last state line
        self._stop = False
        self._thread = threading.Thread(target=self._run, daemon=True, name='walker_link')
        self._thread.start()

    @property
    def connected(self):
        return self._sock is not None

    def age(self, now=None):
        return (now or time.monotonic()) - self.t_state if self.state is not None else float('inf')

    def phase(self):
        return None if self.state is None else self.state.get('phase')

    def send(self, obj):
        line = (json.dumps(obj, separators=(',', ':')) + '\n').encode()
        with self._lock:
            s = self._sock
            if s is None:
                return False
            try:
                s.sendall(line)
                return True
            except OSError:
                self._drop()
                return False

    def _drop(self):
        s, self._sock = self._sock, None
        if s is not None:
            try:
                s.close()
            except OSError:
                pass

    def _run(self):
        warned = False
        while not self._stop:
            try:
                s = socket.create_connection(self.addr, timeout=2.0)
            except OSError as e:
                if not warned:
                    self.log(f'walker link: {self.addr[0]}:{self.addr[1]} not reachable ({e}); retrying')
                    warned = True
                time.sleep(self.reconnect_s)
                continue
            warned = False
            s.settimeout(1.0)
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            with self._lock:
                self._sock = s
            self.log(f'walker link: connected to {self.addr[0]}:{self.addr[1]}')
            buf = b''
            while not self._stop:
                try:
                    chunk = s.recv(65536)
                except socket.timeout:
                    continue
                except OSError:
                    chunk = b''
                if not chunk:
                    break
                buf += chunk
                *lines, buf = buf.split(b'\n')
                for ln in lines:
                    if not ln.strip():
                        continue
                    try:
                        d = json.loads(ln)
                    except ValueError:
                        continue
                    self.state, self.t_state = d, time.monotonic()
            with self._lock:
                if self._sock is s:
                    self._drop()
            self.log('walker link: disconnected')

    def close(self):
        self._stop = True
        with self._lock:
            self._drop()
