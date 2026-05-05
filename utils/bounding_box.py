"""bounding box maker for OSC sender for position payloads."""
import numpy as np
from typing import Any, Dict


# Track min/max statistics for each segment
_stats: Dict[str, Dict[str, tuple]] = {}


# Screen dimensions for Isadora stage
SCREEN_WIDTH = 1920.0
SCREEN_HEIGHT = 1080.0

# Camera specifications
CAMERA_DISTANCE_CM = 170.0  # Camera 170 cm away at +z from mocap origin
FOV_H_DEG = 78.0  # Horizontal field of view in degrees
FOV_V_DEG = 40.0   # Vertical field of view in degrees
CAMERA_PITCH_DEG = 15.0  # Positive = camera looks downward toward stage

# Calculate focal lengths from FOV
# fx = (width/2) / tan(FOV_h/2)
# fy = (height/2) / tan(FOV_v/2)
FOV_H_RAD = np.radians(FOV_H_DEG)
FOV_V_RAD = np.radians(FOV_V_DEG)
FX = (SCREEN_WIDTH * 0.5) / np.tan(FOV_H_RAD * 0.5)
FY = (SCREEN_HEIGHT * 0.5) / np.tan(FOV_V_RAD * 0.5)

# Stage-space calibration after normalization to [-50, 50].
Y_STAGE_OFFSET = -25.0

# Zoom factor: < 1.0 to zoom out (wider view), > 1.0 to zoom in (narrower view)
ZOOM = 2  # Adjust this to control magnification (0.5 = 2x zoom out)
FX /= ZOOM
FY /= ZOOM

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

# Extrinsic: Camera positioned at [0, 0, 1.7m] looking DOWN the -z direction
# Only flip Z to point camera down. Keep X and Y as-is since mocap Y is always positive (above floor).
# R = [[1, 0, 0], [0, 1, 0], [0, 0, -1]]
R = np.array([
    [1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
    [0.0, 0.0, -1.0],
], dtype=float)
# Camera position in mocap frame
camera_pos = np.array([0.0, 0.0, CAMERA_DISTANCE_CM / 100.0], dtype=float) 


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

    # Transform to camera coordinates: P_camera = R @ (P_mocap - camera_position)
    P_camera = R @ (P_mocap - camera_pos)
    
    # Project to image plane: p = K @ P_camera
    p_proj = K @ P_camera
    
    # Normalize by depth to get pixel coordinates
    depth = p_proj[2]
    if depth <= 0:  # Point is behind camera
        return False
    
    u = p_proj[0] / depth
    v = p_proj[1] / depth

    # Normalize screen coordinates to -50 to 50
    # Map from pixel space [0, SCREEN_WIDTH] and [0, SCREEN_HEIGHT] to [-50, 50]
    u_norm = (u / SCREEN_WIDTH) * 100.0 - 50.0
    v_norm = (v / SCREEN_HEIGHT) * 100.0 - 50.0
    v_norm += Y_STAGE_OFFSET

    # Normalize intensity (depth) to 0-100 
    # 100 = at camera (brightest), 0 = 10m away (darkest)
    MAX_DEPTH_M = 10.0  # Maximum expected distance from camera in meters
    intensity = ((MAX_DEPTH_M - depth) / MAX_DEPTH_M) * 100.0 
    intensity = max(0.0, min(100.0, intensity))  # Clamp to [0, 100]

    # Track min/max statistics
    if base not in _stats:
        _stats[base] = {
            'u_min': u_norm, 'u_max': u_norm,
            'v_min': v_norm, 'v_max': v_norm,
            'intensity_min': intensity, 'intensity_max': intensity,
        }
    else:
        _stats[base]['u_min'] = min(_stats[base]['u_min'], u_norm)
        _stats[base]['u_max'] = max(_stats[base]['u_max'], u_norm)
        _stats[base]['v_min'] = min(_stats[base]['v_min'], v_norm)
        _stats[base]['v_max'] = max(_stats[base]['v_max'], v_norm)
        _stats[base]['intensity_min'] = min(_stats[base]['intensity_min'], intensity)
        _stats[base]['intensity_max'] = max(_stats[base]['intensity_max'], intensity)

    # Send OSC messages
    osc_client.send_message(f"/{base}x", float(u_norm))
    osc_client.send_message(f"/{base}y", float(v_norm))
    osc_client.send_message(f"/{base}intensity", float(intensity))

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
        print(f"  Screen X (-50~50):  min={stats['u_min']:8.2f}  max={stats['u_max']:8.2f}")
        print(f"  Screen Y (-50~50):  min={stats['v_min']:8.2f}  max={stats['v_max']:8.2f}")
        print(f"  Intensity (0-100):  min={stats['intensity_min']:8.2f}  max={stats['intensity_max']:8.2f}")
    
    print("\n" + "="*70)


def reset_stats():
    """Reset statistics tracking."""
    global _stats
    _stats = {}
