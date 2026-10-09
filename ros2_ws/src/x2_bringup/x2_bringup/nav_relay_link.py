"""Pure (no rclpy) TCP plumbing of the navigation relay: one framed connection with a receive thread and a
send thread (bounded queue, so a stalled peer can never block a ROS callback), plus a listening server and a
reconnecting client built on it."""
import collections
import socket
import threading
import time

from .nav_relay_proto import FrameDecoder, FrameError


class FrameLink:
    """A connected socket. on_frame(header, payload) runs on the receive thread; on_close(link) once."""

    def __init__(self, sock, on_frame, on_close, send_maxlen=64, send_timeout=1.0, name='link'):
        self.sock, self.name = sock, name
        self.peer = '%s:%s' % sock.getpeername()[:2]
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.settimeout(send_timeout)
        self._on_frame, self._on_close = on_frame, on_close
        self._q = collections.deque(maxlen=send_maxlen)   # oldest dropped when the peer is slow
        self._cv = threading.Condition()
        self.alive = True
        self.t_rx = time.monotonic()
        self.bytes_rx = self.bytes_tx = self.dropped = 0
        self._closed_once = False
        self._threads = [threading.Thread(target=self._rx, daemon=True, name=f'{name}-rx'),
                         threading.Thread(target=self._tx, daemon=True, name=f'{name}-tx')]
        for t in self._threads:
            t.start()

    def send(self, data, replace_key=None):
        """Queue bytes. A queued item with the same replace_key is replaced (latest-wins streams)."""
        if not self.alive:
            return False
        with self._cv:
            if replace_key is not None:
                for i, (k, _) in enumerate(self._q):
                    if k == replace_key:
                        self._q[i] = (k, data)
                        self.dropped += 1
                        break
                else:
                    self._q.append((replace_key, data))
            else:
                if len(self._q) == self._q.maxlen:
                    self.dropped += 1
                self._q.append((None, data))
            self._cv.notify()
        return True

    def _tx(self):
        while self.alive:
            with self._cv:
                while self.alive and not self._q:
                    self._cv.wait(0.5)
                if not self.alive:
                    return
                _, data = self._q.popleft()
            try:
                self.sock.sendall(data)
                self.bytes_tx += len(data)
            except OSError:
                self.close()
                return

    def _rx(self):
        dec = FrameDecoder()
        # recv shares the socket timeout with sendall; a timeout there only means "nothing yet"
        while self.alive:
            try:
                data = self.sock.recv(1 << 16)
            except socket.timeout:
                continue
            except OSError:
                break
            if not data:
                break
            self.t_rx = time.monotonic()
            self.bytes_rx += len(data)
            try:
                for h, p in dec.feed(data):
                    self._on_frame(h, p)
            except FrameError:
                break
        self.close()

    def close(self):
        with self._cv:
            self.alive = False
            self._cv.notify_all()
            first = not self._closed_once
            self._closed_once = True
        try:
            self.sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        try:
            self.sock.close()
        except OSError:
            pass
        if first:
            self._on_close(self)


class LinkServer:
    """Listens; ONE client at a time (a new connection replaces the old one)."""

    def __init__(self, host, port, on_frame, on_connect, on_close, log=print):
        self.on_frame, self.on_connect, self.on_close, self.log = on_frame, on_connect, on_close, log
        self.link = None
        self._lock = threading.Lock()
        self._srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._srv.bind((host, port))
        self._srv.listen(1)
        self._srv.settimeout(0.5)
        self.port = self._srv.getsockname()[1]
        self._run = True
        threading.Thread(target=self._accept, daemon=True, name='relay-accept').start()

    def _accept(self):
        while self._run:
            try:
                s, _ = self._srv.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            with self._lock:
                old, self.link = self.link, None
            if old is not None:
                self.log(f'new client replaces {old.peer}')
                old.close()
            link = FrameLink(s, self.on_frame, self._closed, name='relay-srv')
            with self._lock:
                self.link = link
            self.on_connect(link)

    def _closed(self, link):
        with self._lock:
            if self.link is link:
                self.link = None
        self.on_close(link)

    def send(self, data, replace_key=None):
        with self._lock:
            link = self.link
        return link.send(data, replace_key) if link else False

    def close(self):
        self._run = False
        try:
            self._srv.close()
        except OSError:
            pass
        with self._lock:
            link = self.link
        if link:
            link.close()


class LinkClient:
    """Connects to host:port, reconnects forever (fixed backoff)."""

    def __init__(self, host, port, on_frame, on_connect, on_close, backoff=1.0, log=print):
        self.addr, self.on_frame, self.on_connect, self.on_close = (host, port), on_frame, on_connect, on_close
        self.backoff, self.log = backoff, log
        self.link = None
        self._run = True
        self._gone = threading.Event()
        threading.Thread(target=self._loop, daemon=True, name='relay-connect').start()

    def _loop(self):
        while self._run:
            try:
                s = socket.create_connection(self.addr, timeout=2.0)
            except OSError:
                time.sleep(self.backoff)
                continue
            self._gone.clear()
            self.link = FrameLink(s, self.on_frame, self._closed, name='relay-cli')
            self.on_connect(self.link)
            while self._run and not self._gone.wait(0.5):
                pass

    def _closed(self, link):
        self.on_close(link)
        self._gone.set()

    def send(self, data, replace_key=None):
        link = self.link
        return link.send(data, replace_key) if link and link.alive else False

    @property
    def connected(self):
        return self.link is not None and self.link.alive

    def close(self):
        self._run = False
        if self.link:
            self.link.close()
