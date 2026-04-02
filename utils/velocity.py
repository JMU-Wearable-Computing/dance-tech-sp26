"""Velocity OSC sender for payloads that already carry a velocity value."""

from typing import Any

from utils.middleware import OSCPlugin


def velocity(item: dict, osc_client: Any) -> bool:
    """Send a velocity value as /<segment>velocity when one is present.

    Expected payload shape:
        {
            "segment": str,
            "velocity": float | int,
        }
    """
    value = item.get("velocity")
    if value is None:
        value = item.get("vel")
    if value is None:
        return False

    segment = item.get("segment")
    base = segment.lower() if segment else "unknown"

    osc_client.send_message(f"/{base}-velocity", float(value))
    return True


velocity_plugin: OSCPlugin = velocity