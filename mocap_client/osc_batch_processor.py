"""OSC_Batch_Processor thread: accumulate payloads by frame and batch to middleware.

This module exposes `OSCBatchProcessor`, a thread that consumes queued payloads,
batches them by frame number, and forwards the entire frame to the batch
middleware layer for combined processing.
"""
import threading
import time
from queue import Queue, Empty
from typing import Any

from pythonosc import udp_client

from utils import batch_middleware


class OSCBatchProcessor(threading.Thread):
    """Accumulate items by frame and forward batches to batch middleware.

    Expected payloads are dicts with keys like `segment`, `quat`, `pos`,
    `frame`, and `timestamp`.
    """

    def __init__(self, in_queue: Queue, isadora_ip: str = "127.0.0.1",
                 isadora_port: int = 1234, name: str = "OSC_BatchProcessor",
                 target_fps: float = 30.0, batch_timeout: float = 0.05):
        super().__init__(name=name, daemon=True)
        self.in_queue = in_queue
        self.osc_client = udp_client.SimpleUDPClient(isadora_ip, isadora_port)
        self._stop_event = threading.Event()
        self._min_interval = 1.0 / target_fps
        self._batch_timeout = batch_timeout
        
        # Batching state
        self._current_frame: dict[int, list[dict]] = {}  # frame_num -> list of items
        self._current_frame_num: int | None = None
        self._frame_start_time: float = 0.0
        self._last_sent: dict[int, float] = {}  # frame_num -> last send time

    def stop(self):
        self._stop_event.set()

    def run(self):
        while not self._stop_event.is_set():
            try:
                item = self.in_queue.get(timeout=0.5)
            except Empty:
                # On timeout, flush any pending batch
                self._flush_batch_if_expired()
                continue

            try:
                self._accumulate_item(item)
            finally:
                try:
                    self.in_queue.task_done()
                except Exception:
                    pass

    def _accumulate_item(self, item: dict):
        """Accumulate item into current frame batch, flush if frame boundary crossed."""
        frame_num = item.get("frame")
        
        # Frame boundary: flush previous batch and start new one
        if self._current_frame_num is not None and frame_num != self._current_frame_num:
            self._flush_batch()
            self._current_frame = {}
            self._current_frame_num = frame_num
            self._frame_start_time = time.time()
        elif self._current_frame_num is None:
            # First item
            self._current_frame_num = frame_num
            self._frame_start_time = time.time()
        
        # Accumulate item in current batch
        if frame_num not in self._current_frame:
            self._current_frame[frame_num] = []
        self._current_frame[frame_num].append(item)

    def _flush_batch_if_expired(self):
        """Flush batch if timeout exceeded without new frame."""
        if self._current_frame_num is None or not self._current_frame:
            return
        
        elapsed = time.time() - self._frame_start_time
        if elapsed > self._batch_timeout:
            self._flush_batch()

    def _flush_batch(self):
        """Send accumulated frame batch to batch middleware."""
        if self._current_frame_num is None or not self._current_frame:
            return
        
        now = time.time()
        if now - self._last_sent.get(self._current_frame_num, 0.0) < self._min_interval:
            return
        self._last_sent[self._current_frame_num] = now
        
        # Get all items for this frame
        items = self._current_frame.get(self._current_frame_num, [])
        batch_middleware.main(items, self.osc_client)
