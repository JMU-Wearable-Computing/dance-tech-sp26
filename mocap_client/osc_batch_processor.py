"""OSC_Batch_Processor thread: accumulate payloads by frame and batch to middleware.

This module exposes `OSCBatchProcessor`, a thread that consumes queued payloads,
batches them by frame number, and forwards the entire frame to the batch
middleware layer for combined processing.
"""
import logging
import threading
import time
from queue import Queue, Empty
from typing import Any

from pythonosc import udp_client

from utils import batch_middleware

logger = logging.getLogger(__name__)


class OSCBatchProcessor(threading.Thread):
    """Accumulate items by frame and forward batches to batch middleware.

    Expected payloads are dicts with keys like `segment`, `quat`, `pos`,
    `frame`, and `timestamp`.
    """

    def __init__(self, in_queue: Queue, isadora_ip: str = "127.0.0.1",
                 isadora_port: int = 1234, name: str = "OSC_BatchProcessor",
                 batch_timeout: float = 0.05):
        super().__init__(name=name, daemon=True)
        self.in_queue = in_queue
        self.osc_client = udp_client.SimpleUDPClient(isadora_ip, isadora_port)
        self._stop_event = threading.Event()
        self._batch_timeout = batch_timeout
        logger.info(f"OSCBatchProcessor initialized: {isadora_ip}:{isadora_port}, timeout={batch_timeout}s")
         
        # Batching state
        self._current_frame: dict[int, list[dict]] = {}  # frame_num -> list of items
        self._current_frame_num: int | None = None
        self._frame_start_time: float = 0.0
        self._last_sent: dict[int, float] = {}  # frame_num -> last send time
        self._log_every_n_frames: int = 100

    def stop(self):
        self._stop_event.set()

    def run(self):
        logger.info("OSCBatchProcessor thread started")
        while not self._stop_event.is_set():
            try:
                item = self.in_queue.get(timeout=0.5)
                logger.debug(f"Got item from queue: frame={item.get('frame')}, segment={item.get('segment')}")
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
        logger.info("OSCBatchProcessor thread stopped")

    def _accumulate_item(self, item: dict):
        """Accumulate item into current frame batch, flush if frame boundary crossed."""
        frame_num = item.get("frame")
        logger.debug(f"Accumulating item for frame {frame_num}, current frame: {self._current_frame_num}")
        
        # Frame boundary: flush previous batch and start new one
        if self._current_frame_num is not None and frame_num != self._current_frame_num:
            self._flush_batch()
            self._current_frame = {}
            self._current_frame_num = frame_num
            self._frame_start_time = time.time()
        elif self._current_frame_num is None:
            # First item
            logger.info(f"First item received, starting frame {frame_num}")
            self._current_frame_num = frame_num
            self._frame_start_time = time.time()
        
        # Accumulate item in current batch
        if frame_num not in self._current_frame:
            self._current_frame[frame_num] = []
        self._current_frame[frame_num].append(item)
        logger.debug(f"Frame {frame_num} now has {len(self._current_frame[frame_num])} items")

    def _flush_batch_if_expired(self):
        """Flush batch if timeout exceeded without new frame."""
        if self._current_frame_num is None or not self._current_frame:
            return
        
        elapsed = time.time() - self._frame_start_time
        logger.debug(f"Batch timeout check: frame {self._current_frame_num}, elapsed {elapsed:.3f}s, timeout {self._batch_timeout}s")
        if elapsed > self._batch_timeout:
            logger.debug(f"Batch timeout exceeded, flushing frame {self._current_frame_num}")
            self._flush_batch()

    def _flush_batch(self):
        """Send accumulated frame batch to batch middleware."""
        if self._current_frame_num is None or not self._current_frame:
            logger.debug("_flush_batch: no frame to flush")
            return
        
        # Get all items for this frame
        items = self._current_frame.get(self._current_frame_num, [])
        if self._current_frame_num % self._log_every_n_frames == 0:
            logger.info(f"Frame {self._current_frame_num}")
        try:
            batch_middleware.main(items, self.osc_client)
            logger.debug(f"Successfully sent frame {self._current_frame_num}")
        except Exception as e:
            logger.error(f"Error sending frame {self._current_frame_num}: {e}", exc_info=True)
