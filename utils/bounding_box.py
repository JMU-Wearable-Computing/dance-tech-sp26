"""bounding box maker for OSC sender for position payloads."""
import numpy as np
from typing import Any


# match this width/height to Isadora's stage size for correct mapping of positions to screen coordinates
SCREEN_WIDTH = 1020.0
SCREEN_HEIGHT = 570.0
FX = 777.4
FY = 995.2
CX = SCREEN_WIDTH * 0.5
CY = SCREEN_HEIGHT * 0.5

# Camera intrinsic matrix K:
# [[fx, 0, cx],
#  [0, fy, cy],
#  [0,  0,  1]]
K = np.array([
    [FX, 0.0, CX],
    [0.0, FY, CY],
    [0.0, 0.0, 1.0],
], dtype=float) 


def bounding_box(item: dict, osc_client: Any) -> bool:
    """Send a payload's position as /<segment>x, /<segment>y, /<segment>depth."""
    pos = item.get("pos")
    if pos is None or len(pos) != 3:
        return False

    segment = item.get("segment")
    base = segment.lower() if segment else "unknown"

    # Mocap frame: [x (right), y (up), z (towards camera)]
    # Screen frame: [x (right), y (up)]
    # Depth: positive away from camera
    mocap_x, mocap_y, mocap_z = pos


    # Since +z mocap is towards camera, negate to get depth (positive away)
    depth = -mocap_z
    
    # Perspective projection: p = [x, y, depth] then K @ p
    # Result: u = fx*(x/depth) + cx, v = fy*(y/depth) + cy
    p = np.array([mocap_x, mocap_y, depth], dtype=float)
    uv1 = K @ p  # [u*depth, v*depth, depth]
    
    # Divide by depth to get normalized screen pixel coordinates
    depth = uv1[2]
    u = uv1[0] / depth
    v = uv1[1] / depth

    # Send normalized OSC messages
    osc_client.send_message(f"/{base}x", float(u))
    osc_client.send_message(f"/{base}y", float(v))
    osc_client.send_message(f"/{base}depth", float(depth))

    return True
