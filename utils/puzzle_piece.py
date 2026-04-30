"""Dancer router: computes relative offsets and sends anchored joint data.

Arms anchor to shoulder_center, legs anchor to hip_center, torso sends absolute
positions plus anchor points for the other performers to use.
"""

from typing import Any
import numpy as np

DANCER_CONFIG = {
    "Ally": {
        "role": "torso",
        "skeleton_prefix": "Ally",
        "body_parts": ["HeadTop", "Ab", "BackTop", "LHip", "WaistLFront", "WaistRFront"],
    },
    "Riley": {
        "role": "arms",
        "skeleton_prefix": "Riley",
        "body_parts": ["LShoulder", "RShoulder", "LElbowOut", "RElbowOut", "LHand", "RHand"],
    },
    "Emma": {
        "role": "legs",
        "skeleton_prefix": "Emma",
        "body_parts": ["LHeel", "RHeel", "LKneeOut", "RKneeOut", "WaistLFront", "WaistRFront"],
    }
}


def puzzle_piece(items: list[dict], osc_client: Any, dancer_config: dict | None = None) -> bool:
    """Process a batch of joints for all dancers, computing relative offsets.
    
    Torso (Ally): sends absolute positions + anchor points (shoulder_center, hip_center)
    Arms (Emma): sends relative offsets from shoulder_center
    Legs (Riley): sends relative offsets from hip_center
    """
    if not dancer_config:
        dancer_config = DANCER_CONFIG
    
    if not items:
        return False
    
    # Build a mapping: dancer_name -> {bone_name -> position}
    dancer_joints: dict[str, dict[str, Any]] = {}
    
    for item in items:
        segment = item.get("segment")
        pos = item.get("pos")
        
        if not segment or pos is None or len(pos) != 3:
            continue
        
        # Parse segment name: "Emma_Chest" -> prefix="Emma", bone="Chest"
        parts = segment.rsplit(":", 1)
        if len(parts) != 2:
            continue
        
        skeleton_prefix, bone_name = parts
        
        # Find matching dancer
        dancer_name = None
        for name, config in dancer_config.items():
            if skeleton_prefix == config.get("skeleton_prefix"):
                dancer_name = name
                break
        
        if not dancer_name:
            continue
        
        if dancer_name not in dancer_joints:
            dancer_joints[dancer_name] = {}
        
        dancer_joints[dancer_name][bone_name] = np.array(pos, dtype=float)
    
    handled = False
    
    # Get torso anchor points (Ally)
    ally_joints = dancer_joints.get("Ally", {})
    if not ally_joints:
        return False
    ally_lshoulder = ally_joints.get("LShoulder")
    ally_rshoulder = ally_joints.get("RShoulder")
    ally_lhip = ally_joints.get("LHip")
    ally_rhip = ally_joints.get("RHip")
    
    # Get arm and leg performer's own anchor points for adjustment
    riley_joints = dancer_joints.get("Riley", {})
    riley_lshoulder = riley_joints.get("LShoulder")
    riley_rshoulder = riley_joints.get("RShoulder")
    
    emma_joints = dancer_joints.get("Emma", {})
    emma_lhip = emma_joints.get("LHip")
    emma_rhip = emma_joints.get("RHip")
    
    # Process each dancer based on their role
    for dancer_name, config in dancer_config.items():
        if dancer_name not in dancer_joints:
            continue
        
        joints = dancer_joints[dancer_name]
        role = config.get("role")
        
        if role == "torso":
            # Ally: send absolute positions for shoulders and hips
            for bone_name in ["LShoulder", "RShoulder", "LHip", "RHip"]:
                if bone_name in joints:
                    pos = joints[bone_name]
                    address = f"/{dancer_name.lower()}/{bone_name}"
                    osc_client.send_message(address, [float(pos[0]), float(pos[1]), float(pos[2])])
                    handled = True
        
        elif role == "arms":
            # Riley: send adjusted positions accounting for shoulder position difference
            for bone_name, pos in joints.items():
                if bone_name.startswith("L"):
                    # Left arm joint: adjust for difference between Riley's L shoulder and Ally's L shoulder
                    if ally_lshoulder is not None and riley_lshoulder is not None:
                        shoulder_adjustment = ally_lshoulder - riley_lshoulder
                        adjusted_pos = pos + shoulder_adjustment
                        address = f"/{dancer_name.lower()}/{bone_name}_rel_LShoulder"
                        osc_client.send_message(address, [float(adjusted_pos[0]), float(adjusted_pos[1]), float(adjusted_pos[2])])
                        handled = True
                elif bone_name.startswith("R"):
                    # Right arm joint: adjust for difference between Riley's R shoulder and Ally's R shoulder
                    if ally_rshoulder is not None and riley_rshoulder is not None:
                        shoulder_adjustment = ally_rshoulder - riley_rshoulder
                        adjusted_pos = pos + shoulder_adjustment
                        address = f"/{dancer_name.lower()}/{bone_name}_rel_RShoulder"
                        osc_client.send_message(address, [float(adjusted_pos[0]), float(adjusted_pos[1]), float(adjusted_pos[2])])
                        handled = True

        elif role == "legs":
            # Emma: send adjusted positions accounting for hip position difference
            for bone_name, pos in joints.items():
                if bone_name.startswith("L"):
                    # Left leg joint: adjust for difference between Emma's L hip and Ally's L hip
                    if ally_lhip is not None and emma_lhip is not None:
                        hip_adjustment = ally_lhip - emma_lhip
                        adjusted_pos = pos + hip_adjustment
                        address = f"/{dancer_name.lower()}/{bone_name}_rel_LHip"
                        osc_client.send_message(address, [float(adjusted_pos[0]), float(adjusted_pos[1]), float(adjusted_pos[2])])
                        handled = True
                elif bone_name.startswith("R"):
                    # Right leg joint: adjust for difference between Emma's R hip and Ally's R hip
                    if ally_rhip is not None and emma_rhip is not None:
                        hip_adjustment = ally_rhip - emma_rhip
                        adjusted_pos = pos + hip_adjustment
                        address = f"/{dancer_name.lower()}/{bone_name}_rel_RHip"
                        osc_client.send_message(address, [float(adjusted_pos[0]), float(adjusted_pos[1]), float(adjusted_pos[2])])
                        handled = True
    
    return handled