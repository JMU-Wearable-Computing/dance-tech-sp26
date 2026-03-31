"""NatClient - bridges Motive/NatNet to the mocap pipeline.

Wraps the NatNetClient from the polymidi project.  For each frame
received it optionally filters by a single target name (rigid body or
skeleton bone) and enqueues quaternion payloads for OSC_Processor.

Payload schema pushed onto out_queue:
    {
        "segment":   str,           # rigid-body / bone name from Motive
        "quat":      (qx,qy,qz,qw),# orientation quaternion
        "frame":     int,           # Motive frame number
        "timestamp": float,         # Motive timestamp (seconds)
    }
"""
import queue
import threading
import time
from typing import Optional

from .NatNetClient import NatNetClient as _NatNetClient
from . import DataDescriptions as _DD
from . import MoCapData as _MCD


class NatClient:
    """Connect to a Motive NatNet stream and enqueue quaternion data.

    Parameters
    ----------
    out_queue:
        Destination queue for quaternion payloads.
    target_name:
        Name of the single rigid body or skeleton bone to forward.
        Must match exactly the name configured in Motive.
        Pass None to forward every tracked entity.
    server_ip:
        IP address of the Motive server (default: loopback).
    local_ip:
        IP address of this machine's network interface.
    use_multicast:
        Use multicast transport (default True, matches Motive default).
    """

    def __init__(
        self,
        out_queue: queue.Queue,
        target_name: Optional[str] = None,
        server_ip: str = "127.0.0.1",
        local_ip: str = "127.0.0.1",
        use_multicast: bool = True,
    ):
        self.out_queue = out_queue
        self.target_name: Optional[str] = target_name

        self._server_ip = server_ip
        self._local_ip = local_ip
        self._use_multicast = use_multicast

        # Populated once DataDescriptions arrive: rigid-body id -> segment name
        self._id_to_name: dict = {}

        self._data_q: queue.Queue = queue.Queue()
        self._command_q: queue.Queue = queue.Queue()

        self._client = _NatNetClient()
        self._client.set_server_address(self._server_ip)
        self._client.set_client_address(self._local_ip)
        self._client.set_use_multicast(self._use_multicast)

        self._stop_event = threading.Event()
        self._frame_thread: Optional[threading.Thread] = None
        self._last_modeldef_request: float = 0.0

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def start(self) -> bool:
        """Connect to Motive and begin processing frames.

        Returns True on successful connection, False otherwise.
        """
        is_running = self._client.run(self._data_q, self._command_q)
        if not is_running:
            print("[NatClient] ERROR: Could not open NatNet sockets.")
            return False

        time.sleep(1)
        if not self._client.connected():
            print("[NatClient] ERROR: Could not connect to Motive server.")
            return False

        # Ask Motive for model definitions so we can build the ID->name map
        self._client.send_request(
            self._client.command_socket,
            self._client.NAT_REQUEST_MODELDEF,
            "",
            (self._server_ip, self._client.command_port),
        )
        self._last_modeldef_request = time.time()

        self._stop_event.clear()
        self._frame_thread = threading.Thread(
            target=self._frame_loop, name="NatClientFrameLoop", daemon=True
        )
        self._frame_thread.start()
        return True

    def stop(self, timeout: float = 2.0):
        """Disconnect from Motive and shut down the processing thread."""
        self._stop_event.set()
        self._client.shutdown()
        if self._frame_thread is not None:
            self._frame_thread.join(timeout)
            self._frame_thread = None

    def set_target_name(self, name: Optional[str]):
        """Update the target filter at runtime (thread-safe)."""
        self.target_name = name

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _frame_loop(self):
        """Drain both NatNetClient queues and dispatch each item."""
        while not self._stop_event.is_set():
            # Re-request DataDescriptions every 5 s until ID map is populated
            if not self._id_to_name and (time.time() - self._last_modeldef_request) > 5.0:
                print("[NatClient] Requesting DataDescriptions...")
                self._client.send_request(
                    self._client.command_socket,
                    self._client.NAT_REQUEST_MODELDEF,
                    "",
                    (self._server_ip, self._client.command_port),
                )
                self._last_modeldef_request = time.time()

            for src_q in (self._command_q, self._data_q):
                while True:
                    try:
                        item = src_q.get_nowait()
                    except queue.Empty:
                        break
                    self._dispatch(item)
            time.sleep(0.001)

    def _dispatch(self, item):
        if isinstance(item, _DD.DataDescriptions):
            print(f"[NatClient] DataDescriptions packet received")
            self._build_id_map(item)
            print(f"[NatClient] ID map: {self._id_to_name}")
        elif isinstance(item, _MCD.MoCapData):
            self._process_frame(item)
        else:
            print(f"[NatClient] Unknown packet type: {type(item)}")

    def _build_id_map(self, desc: _DD.DataDescriptions):
        """Build self._id_to_name from a DataDescriptions packet."""
        def _decode(name):
            return name.decode("utf-8") if isinstance(name, bytes) else name

        new_map: dict = {}
        for rb_desc in desc.rigid_body_list:
            new_map[rb_desc.id_num] = _decode(rb_desc.sz_name)
        for skel_desc in desc.skeleton_list:
            for rb_desc in skel_desc.rigid_body_description_list:
                new_map[rb_desc.id_num] = _decode(rb_desc.sz_name)
        self._id_to_name = new_map

    def _process_frame(self, frame: _MCD.MoCapData):
        """Filter rigid bodies by segment name and enqueue quaternion payloads."""
        frame_num = frame.prefix_data.frame_number if frame.prefix_data else 0
        timestamp = (
            frame.suffix_data.timestamp
            if frame.suffix_data is not None
            else 0.0
        )

        # Skeleton bones
        if frame.skeleton_data:
            for skeleton in frame.skeleton_data.skeleton_list:
                for rb in skeleton.rigid_body_list:
                    self._maybe_enqueue(rb, frame_num, timestamp)

        # Top-level rigid bodies
        if frame.rigid_body_data:
            for rb in frame.rigid_body_data.rigid_body_list:
                self._maybe_enqueue(rb, frame_num, timestamp)

    def _maybe_enqueue(self, rb, frame_num: int, timestamp: float):
        """Enqueue a quaternion payload if the rigid body passes the filter."""
        seg_name = self._id_to_name.get(rb.id_num)
        if seg_name is None:
            return
        if self.target_name is not None and seg_name != self.target_name:
            return
        if not rb.tracking_valid:
            return
        self.out_queue.put({
            "segment":   seg_name,
            "quat":      rb.rot,
            "frame":     frame_num,
            "timestamp": timestamp,
        })
