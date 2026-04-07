"""OSC_Processor thread: consume mocap payloads and send to MaxMSP for audio processing.

This module exposes `OSCProcessor`, a thread that consumes quaternion
payloads from a queue and sends them to MaxMSP via OSC.

Follows Max OSC conventions for parameter addressing:
  /<patcher>/param/<segment>/<attribute>/raw

Message paths:
  - /audio/param/<segment>/quat/raw   : quaternion (qx, qy, qz, qw) for spatial audio
  - /audio/param/<segment>/pos/raw    : position (x, y, z) for panning / distance attenuation
  - /audio/param/<segment>/sync/raw   : frame number and timestamp for sync
"""
import threading
import time
from queue import Queue, Empty
from typing import Optional

from pythonosc import udp_client


class OSCProcessor(threading.Thread):
    """Consume items from an input Queue and send mocap data via OSC to MaxMSP.

    Expected payloads are dicts with keys: `segment`, `quat`, `pos`, `frame`, `timestamp`.
    Sends quaternion and position data to MaxMSP via OSC messages.
    """

    def __init__(self, in_queue: Queue, maxmsp_ip: str = "127.0.0.1",
                 maxmsp_port: int = 9999, name: str = "OSC_Processor",
                 target_fps: float = 30.0):
        super().__init__(name=name, daemon=True)
        self.in_queue = in_queue
        self.osc_client = udp_client.SimpleUDPClient(maxmsp_ip, maxmsp_port)
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
        """Extract position and quaternion from item and send to MaxMSP via OSC.
        
        Sends to OSC paths following Max conventions /<patcher>/param/<segment>/<attribute>/raw:
          - /audio/param/<segment>/quat/raw  : quaternion array (qx, qy, qz, qw)
          - /audio/param/<segment>/pos/raw   : position array (x, y, z) 
          - /audio/param/<segment>/sync/raw  : frame number and timestamp
        """
        now = time.time()
        segment = item.get("segment")
        if now - self._last_sent.get(segment, 0.0) < self._min_interval:
            return
        self._last_sent[segment] = now
        
        quat = item.get("quat")
        pos = item.get("pos")
        frame = item.get("frame")
        ts = item.get("timestamp")
        
        if quat is None or len(quat) != 4:
            print(f"OSC_Processor: invalid quat for {segment}, skipping")
            return
        
        try:
            segment_name = segment.lower() if segment else "unknown"
            
            # Send quaternion: /audio/param/<segment>/quat/raw qx qy qz qw
            quat_path = f"/audio/param/{segment_name}/quat/raw"
            self.osc_client.send_message(quat_path, [float(q) for q in quat])
            
            # Send position: /audio/param/<segment>/pos/raw x y z
            if pos is not None and len(pos) == 3:
                pos_path = f"/audio/param/{segment_name}/pos/raw"
                self.osc_client.send_message(pos_path, [float(p) for p in pos])
            
            # Send sync info: /audio/param/<segment>/sync/raw frame timestamp
            if frame is not None and ts is not None:
                sync_path = f"/audio/param/{segment_name}/sync/raw"
                self.osc_client.send_message(sync_path, [int(frame), float(ts)])
            
            print(f"OSC_Processor: sent {quat_path}={[f'{q:.4f}' for q in quat]} @ frame {frame}")
            
        except Exception as e:
            print(f"OSC_Processor: send failed for {segment}: {e}")
