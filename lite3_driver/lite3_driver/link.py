"""UDP link to the Lite3 motion host: commands out, reports in on a receive thread."""

import select
import socket
import threading
import time
from typing import Callable, Optional


class MotionLink:
    """One UDP socket bound to the report port; sends to the motion host from the same socket."""

    def __init__(
        self,
        host: str,
        port: int,
        local_ip: str,
        local_port: int,
        on_datagram: Callable[[bytes, tuple], None],
        poll_sec: float = 0.2,
    ):
        """Store the addresses; nothing is opened until start()."""
        self.remote = (host, int(port))
        self.local = (local_ip, int(local_port))
        self._on_datagram = on_datagram
        self._poll_sec = poll_sec
        self._sock: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self._running = threading.Event()
        self._send_lock = threading.Lock()
        self.rx_count = 0
        self.tx_count = 0
        self.tx_errors = 0
        self.rx_errors = 0
        self.last_rx: Optional[float] = None
        self.last_error = ''

    def start(self):
        """Bind the socket and start the receive thread."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(self.local)
        sock.setblocking(False)
        self._sock = sock
        self.local = sock.getsockname()
        self._running.set()
        self._thread = threading.Thread(target=self._receive_loop, name='lite3_rx', daemon=True)
        self._thread.start()

    def stop(self):
        """Stop the receive thread and close the socket."""
        self._running.clear()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._sock is not None:
            self._sock.close()
            self._sock = None

    def send(self, data: bytes) -> bool:
        """Send one datagram to the motion host; False when it could not be sent."""
        sock = self._sock
        if sock is None:
            return False
        try:
            with self._send_lock:
                sock.sendto(data, self.remote)
        except OSError as error:
            self.tx_errors += 1
            self.last_error = str(error)
            return False
        self.tx_count += 1
        return True

    def send_all(self, datagrams) -> int:
        """Send datagrams in order; return how many went out."""
        return sum(1 for data in datagrams if self.send(data))

    def age(self, now: Optional[float] = None) -> float:
        """Return seconds since the last report, infinity before the first."""
        if self.last_rx is None:
            return float('inf')
        return (time.monotonic() if now is None else now) - self.last_rx

    def _receive_loop(self):
        """Read datagrams until stopped; a failing callback never kills the loop."""
        while self._running.is_set():
            sock = self._sock
            if sock is None:
                return
            try:
                readable, _, _ = select.select([sock], [], [], self._poll_sec)
            except (OSError, ValueError):
                return
            if not readable:
                continue
            try:
                data, address = sock.recvfrom(65535)
            except (BlockingIOError, InterruptedError):
                continue
            except OSError as error:
                self.rx_errors += 1
                self.last_error = str(error)
                continue
            self.rx_count += 1
            self.last_rx = time.monotonic()
            try:
                self._on_datagram(data, address)
            except Exception as error:  # noqa: B902  a bad datagram must not stop the link
                self.rx_errors += 1
                self.last_error = f'{type(error).__name__}: {error}'
