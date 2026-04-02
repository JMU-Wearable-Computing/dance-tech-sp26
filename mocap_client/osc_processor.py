"""OSC_Processor thread: consume quaternion payloads and send to Isadora.

This module exposes `OSCProcessor`, a thread that consumes quaternion
payloads from a queue and sends them to Isadora via OSC on the address
/isadora-multi/1 (4 float values: qx, qy, qz, qw).
"""
import threading
import time
from queue import Queue, Empty
from typing import Optional

from pythonosc import udp_client


class OSCProcessor(threading.Thread):
    """Consume items from an input Queue and send quaternion data via OSC.

    Expected payloads are dicts with keys: `segment`, `quat`, `timestamp`.
    Sends the quaternion (4 floats) to Isadora via OSC message.
    """

    def __init__(self, in_queue: Queue, isadora_ip: str = "127.0.0.1",
                 isadora_port: int = 1234, name: str = "OSC_Processor",
                 target_fps: float = 30.0):
        super().__init__(name=name, daemon=True)
        self.in_queue = in_queue
        self.osc_client = udp_client.SimpleUDPClient(isadora_ip, isadora_port)
        self._stop_event = threading.Event()
        self._min_interval = 1.0 / target_fps
        self._last_sent: float = 0.0

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
        """Extract position and quaternion from item and send to Isadora via OSC.
        
        Sends the position as a string to /isadora/1 and the 4 quaternion values
        (qx, qy, qz, qw) to /isadora-multi/1.
        """
        now = time.time()
        if now - self._last_sent < self._min_interval:
            return
        self._last_sent = now

        segment = item.get("segment")
        quat = item.get("quat")
        pos = item.get("pos")
        ts = item.get("timestamp")
        
        if quat is None or len(quat) != 4:
            print(f"OSC_Processor: invalid quat for {segment}, skipping")
            return
        
        try:
            # Send position as three separate OSC messages: /<name>x, /<name>y, /<name>z
            if pos is not None and len(pos) == 3:
                base = segment.lower() if segment else "unknown"
                self.osc_client.send_message(f"/{base}x", float(pos[0]))
                self.osc_client.send_message(f"/{base}y", float(pos[1]))
                self.osc_client.send_message(f"/{base}z", float(pos[2]))
                print(f"OSC_Processor: sent /{base}x={pos[0]:.4f} /{base}y={pos[1]:.4f} /{base}z={pos[2]:.4f} @ {ts:.3f}")
        except Exception as e:
            print(f"OSC_Processor: send failed for {segment}: {e}")
