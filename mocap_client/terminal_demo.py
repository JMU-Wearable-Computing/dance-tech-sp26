"""Small standalone Motive/NatNet terminal demo.

Run from the repository root:

    python -m mocap_client.terminal_demo

The demo prints every rigid body and skeleton bone received from Motive.
"""
import argparse
import queue
import time

from .DataDescriptions import DataDescriptions
from .MoCapData import MoCapData
from .NatNetClient import NatNetClient


def _decode_name(name):
    return name.decode("utf-8") if isinstance(name, bytes) else name


def _build_name_map(descriptions):
    """Map the IDs used in frame packets to Motive segment names."""
    names = {
        description.id_num: _decode_name(description.sz_name)
        for description in descriptions.rigid_body_list
    }
    for skeleton in descriptions.skeleton_list:
        for bone in skeleton.rigid_body_description_list:
            frame_id = (skeleton.id_num << 16) | bone.id_num
            names[frame_id] = _decode_name(bone.sz_name)
    return names


def _print_rigid_body(rigid_body, names, prefix=""):
    segment_name = names.get(rigid_body.id_num, f"id={rigid_body.id_num}")
    print(
        f"{prefix}{segment_name} "
        f"pos={tuple(round(value, 4) for value in rigid_body.pos)} "
        f"quat={tuple(round(value, 4) for value in rigid_body.rot)} "
        f"tracked={rigid_body.tracking_valid}",
        flush=True,
    )


def _print_frame(frame, names):
    frame_number = frame.prefix_data.frame_number
    timestamp = frame.suffix_data.timestamp if frame.suffix_data else 0.0
    print(f"\nframe={frame_number} timestamp={timestamp:.6f}", flush=True)

    if frame.rigid_body_data:
        for rigid_body in frame.rigid_body_data.rigid_body_list:
            _print_rigid_body(rigid_body, names, prefix="  rigid_body ")

    if frame.skeleton_data:
        for skeleton in frame.skeleton_data.skeleton_list:
            for rigid_body in skeleton.rigid_body_list:
                _print_rigid_body(rigid_body, names, prefix="  skeleton ")


def main():
    parser = argparse.ArgumentParser(description="Print live Motive NatNet data.")
    parser.add_argument("--server", default="127.0.0.1", help="Motive server IP")
    parser.add_argument("--local", default="127.0.0.1", help="local network IP")
    parser.add_argument(
        "--unicast",
        action="store_true",
        help="use unicast instead of Motive's default multicast mode",
    )
    args = parser.parse_args()

    data_queue = queue.Queue()
    command_queue = queue.Queue()
    client = NatNetClient()
    client.set_server_address(args.server)
    client.set_client_address(args.local)
    client.set_use_multicast(not args.unicast)

    if not client.run(data_queue, command_queue):
        raise RuntimeError("Could not open the NatNet sockets.")

    client.send_request(
        client.command_socket,
        client.NAT_REQUEST_MODELDEF,
        "",
        (args.server, client.command_port),
    )

    print(
        f"Listening for Motive data from {args.server}. "
        "Press Ctrl+C to stop.",
        flush=True,
    )
    names = {}
    try:
        while True:
            packet = None
            for packet_queue in (command_queue, data_queue):
                try:
                    packet = packet_queue.get_nowait()
                    break
                except queue.Empty:
                    continue
            if packet is None:
                time.sleep(0.001)
                continue

            if isinstance(packet, DataDescriptions):
                names.update(_build_name_map(packet))
                print(f"Discovered {len(names)} Motive segments.", flush=True)
            elif isinstance(packet, MoCapData):
                _print_frame(packet, names)
    except KeyboardInterrupt:
        print("\nStopping.", flush=True)
    finally:
        client.shutdown()


if __name__ == "__main__":
    main()
