"""bounding box maker for OSC sender for position payloads."""

from typing import Any


def bounding_box(item: dict, osc_client: Any) -> bool:
    """Send a payload's position as /<segment>x, /<segment>y, /<segment>width, /<segment>height."""
    pos = item.get("pos")
    if pos is None or len(pos) != 3:
        return False

    segment = item.get("segment")
    base = segment.lower() if segment else "unknown"

    # Rough Manual Perspective Matrix
    


    # logic

    # sending

    return True
