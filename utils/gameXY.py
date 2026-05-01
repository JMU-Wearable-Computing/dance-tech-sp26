"""XYZ OSC sender for position payloads."""

from typing import Any


def gameXY(items: list[dict], osc_client: Any, dancer_config: dict | None = None) -> bool:
    """Send three rigid body positions as /<segment>x, /<segment>y. overlaid. 1 in middle, 2 left, 3 right."""
    for item in items:
        pos = item.get("pos")
        segment = item.get("segment")
        base = segment.lower() if segment else "unknown"
        
        # Apply x-axis offset based on rigid body number: 1=center, 2=left (-1.6m), 3=right (+1.6m)
        x_offset = 0.0
        if segment:
            if '1' in segment:
                x_offset = 0.0
            elif '2' in segment:
                x_offset = -1.6
            elif '3' in segment:
                x_offset = 1.6
        else:
            continue  # Skip if no segment info
        
        x_pos = float(pos[0]) + x_offset

        # [-0.8, 0.8] m range
        osc_client.send_message(f"/{base}x", x_pos)
        osc_client.send_message(f"/{base}y", float(pos[1]))
    
    return True
