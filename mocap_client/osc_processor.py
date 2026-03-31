"""OSC_Processor thread skeleton.

This module exposes `OSCProcessor`, a thread that consumes quaternion
payloads from a queue. Implementation of actual OSC sending will be added
later; currently it prints and ACKs processed items.
"""
import threading
import time
from queue import Queue, Empty
from typing import Optional


class OSCProcessor(threading.Thread):
    """Consume items from an input Queue and process them.

    Expected payloads are dicts with keys: `segment`, `quat`, `timestamp`.
    """

    def __init__(self, in_queue: Queue, name: str = "OSC_Processor"):
        super().__init__(name=name, daemon=True)
        self.in_queue = in_queue
        self._stop_event = threading.Event()

    def stop(self):
        self._stop_event.set()

    def run(self):
        while not self._stop_event.is_set():
            try:
                item = self.in_queue.get(timeout=0.5)
            except Empty:
                continue

            try:
                # placeholder processing: print and mark task done
                self._process_item(item)
            finally:
                try:
                    self.in_queue.task_done()
                except Exception:
                    pass

    def _process_item(self, item: dict):
        # Replace this with OSC sending logic in a future step.
        segment = item.get("segment")
        quat = item.get("quat")
        ts = item.get("timestamp")
        print(f"OSC_Processor: received {segment} @ {ts:.3f} quat={quat}")
        # simulate a small processing delay
        time.sleep(0.005)
