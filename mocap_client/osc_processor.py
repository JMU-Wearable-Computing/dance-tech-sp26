"""OSC_Processor thread: consume payloads and hand them to middleware.

This module exposes `OSCProcessor`, a thread that consumes queued payloads
and forwards them to the middleware layer for OSC output.
"""
import threading
import time
from queue import Queue, Empty

from pythonosc import udp_client

from utils import middleware


class OSCProcessor(threading.Thread):
    """Consume items from an input Queue and forward them to middleware.

    Expected payloads are dicts with keys like `segment`, `quat`, `pos`,
    and `timestamp`.
    """

    def __init__(self, in_queue: Queue, isadora_ip: str = "127.0.0.1",
                 isadora_port: int = 1234, name: str = "OSC_Processor",
                 target_fps: float = 30.0):
        super().__init__(name=name, daemon=True)
        self.in_queue = in_queue
        self.osc_client = udp_client.SimpleUDPClient(isadora_ip, isadora_port)
        self._stop_event = threading.Event()
        self._min_interval = 1.0 / target_fps
        self._last_sent: dict = {}  # segment -> last send time

    def stop(self):
        self._stop_event.set()

    def run(self):
        while not self._stop_event.is_set():
            try:
                item = self.in_queue.get(timeout=0.5)
            except Empty:
                continue

            try:
                self._process_item(item)
            finally:
                try:
                    self.in_queue.task_done()
                except Exception:
                    pass

    def _process_item(self, item: dict):
        """Forward a payload to the middleware dispatcher."""
        now = time.time()
        segment = item.get("segment")
        if now - self._last_sent.get(segment, 0.0) < self._min_interval:
            return
        self._last_sent[segment] = now

        middleware.main(item, self.osc_client)
