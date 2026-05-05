"""XYZ OSC sender for position payloads."""

from typing import Any


def send_xyz(item: dict, osc_client: Any) -> bool:
    """Send a payload's position as /<segment>x, /<segment>y, /<segment>z."""
    pos = item.get("pos")
    if pos is None or len(pos) != 3:
        return False

    segment = item.get("segment")
    base = segment.lower() if segment else "unknown"

    osc_client.send_message(f"/{base}x", float(pos[0]))
    osc_client.send_message(f"/{base}y", float(pos[1]))
    osc_client.send_message(f"/{base}z", float(pos[2]))
    print(f"Sent OSC: /{base}x {pos[0]}, /{base}y {pos[1]}, /{base}z {pos[2]}")  # Debug print
    return True
