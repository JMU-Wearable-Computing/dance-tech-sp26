"""bounding box maker for OSC sender for position payloads."""
import numpy as np
from typing import Any, Dict


# Track min/max statistics for each segment
_stats: Dict[str, Dict[str, tuple]] = {}


# Screen dimensions for Isadora stage
SCREEN_WIDTH = 1020.0
SCREEN_HEIGHT = 570.0

# Camera specifications
CAMERA_DISTANCE_CM = 170.0  # Camera 170 cm away at +z from mocap origin
FOV_H_DEG = 102.0  # Horizontal field of view in degrees
FOV_V_DEG = 57.0   # Vertical field of view in degrees

# Calculate focal lengths from FOV
# fx = (width/2) / tan(FOV_h/2)
# fy = (height/2) / tan(FOV_v/2)
FOV_H_RAD = np.radians(FOV_H_DEG)
FOV_V_RAD = np.radians(FOV_V_DEG)
FX = (SCREEN_WIDTH * 0.5) / np.tan(FOV_H_RAD * 0.5)
FY = (SCREEN_HEIGHT * 0.5) / np.tan(FOV_V_RAD * 0.5)

# Principal point (image center)
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

# Extrinsic: Camera is at +z=170cm from mocap origin, aligned with mocap axes
# Rotation: Identity (no rotation)
# Translation: [0, 0, -CAMERA_DISTANCE_CM] in world-to-camera transform
# (negative because we measure depth as distance from camera)
R = np.eye(3, dtype=float)  # Identity rotation
t = np.array([0.0, 0.0, -CAMERA_DISTANCE_CM], dtype=float)  # Translation 


def bounding_box(item: dict, osc_client: Any) -> bool:
    """
    Project mocap position to screen coordinates using camera intrinsics/extrinsics.
    
    Coordinate systems:
    - Mocap: +x right, +y up, +z towards camera
    - Camera: origin at optical center, +x right, +y down, +z forward (away from camera)
    
    Sends: /<segment>x, /<segment>y, /<segment>depth
    """
    pos = item.get("pos")
    if pos is None or len(pos) != 3:
        return False

    segment = item.get("segment")
    base = segment.lower() if segment else "unknown"

    # Mocap position
    mocap_x, mocap_y, mocap_z = pos
    P_mocap = np.array([mocap_x, mocap_y, mocap_z], dtype=float)

    # Transform to camera coordinates: P_camera = R @ P_mocap + t
    P_camera = R @ P_mocap + t
    
    # Project to image plane: p = K @ P_camera
    p_proj = K @ P_camera
    
    # Normalize by depth to get pixel coordinates
    depth = p_proj[2]
    if depth <= 0:  # Point is behind camera
        return False
    
    u = p_proj[0] / depth
    v = p_proj[1] / depth

    # Track min/max statistics
    if base not in _stats:
        _stats[base] = {
            'u_min': u, 'u_max': u,
            'v_min': v, 'v_max': v,
            'depth_min': depth, 'depth_max': depth,
        }
    else:
        _stats[base]['u_min'] = min(_stats[base]['u_min'], u)
        _stats[base]['u_max'] = max(_stats[base]['u_max'], u)
        _stats[base]['v_min'] = min(_stats[base]['v_min'], v)
        _stats[base]['v_max'] = max(_stats[base]['v_max'], v)
        _stats[base]['depth_min'] = min(_stats[base]['depth_min'], depth)
        _stats[base]['depth_max'] = max(_stats[base]['depth_max'], depth)

    # Send OSC messages
    osc_client.send_message(f"/{base}x", float(u))
    osc_client.send_message(f"/{base}y", float(v))
    osc_client.send_message(f"/{base}depth", float(depth))

    return True


def print_summary():
    """Print min/max statistics for all tracked segments."""
    if not _stats:
        print("\nBounding Box Summary: No data collected")
        return
    
    print("\n" + "="*70)
    print("BOUNDING BOX STATISTICS SUMMARY")
    print("="*70)
    
    for segment in sorted(_stats.keys()):
        stats = _stats[segment]
        print(f"\nSegment: {segment}")
        print(f"  Screen X (u):      min={stats['u_min']:8.2f}  max={stats['u_max']:8.2f}")
        print(f"  Screen Y (v):      min={stats['v_min']:8.2f}  max={stats['v_max']:8.2f}")
        print(f"  Depth (cm):        min={stats['depth_min']:8.2f}  max={stats['depth_max']:8.2f}")
    
    print("\n" + "="*70)


def reset_stats():
    """Reset statistics tracking."""
    global _stats
    _stats = {}
