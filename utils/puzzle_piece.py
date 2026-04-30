"""Dancer router: computes relative offsets and sends anchored joint data.

Arms anchor to shoulder_center, legs anchor to hip_center, torso sends absolute
positions plus anchor points for the other performers to use.
"""

import logging
from typing import Any
import numpy as np

logger = logging.getLogger(__name__)


def normalize_position(value: float, input_min: float = -3.0, input_max: float = 3.0, 
                       output_min: float = -50.0, output_max: float = 50.0) -> float:
    """Normalize a position value from input range to output range.
    
    Args:
        value: Input value in meters
        input_min: Minimum input value (default -3 meters)
        input_max: Maximum input value (default 3 meters)
        output_min: Minimum output value (default -50)
        output_max: Maximum output value (default 50)
    
    Returns:
        Normalized value in output range
    """
    if input_max == input_min:
        return (output_min + output_max) / 2  # Return midpoint if range is zero
    
    # Linear interpolation: map [input_min, input_max] to [output_min, output_max]
    normalized = (value - input_min) / (input_max - input_min) * (output_max - output_min) + output_min
    return normalized

DANCER_CONFIG = {
    "Ally": {
        "role": "torso",
        "skeleton_prefix": "Ally", # need to draw lines for neck and backtop
        "body_parts": ["head", "neck", "chest", "ab", "hip", "lshoulder", "rshoulder"],
    },
    "Riley": {
        "role": "arms",
        "skeleton_prefix": "Riley",
        "body_parts": ["luarm", "ruarm", "lfarmp", "rfarmp", "lhand", "rhand"],
    },
    "Emma": {
        "role": "legs",
        "skeleton_prefix": "Emma",
        "body_parts": ["lthigh", "lshin", "lfoot", "ltoe", "rthigh", "rshin", "rfoot", "rtoe"],
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
        logger.debug("puzzle_piece: no items received")
        return False
    
    # Build a mapping: dancer_name -> {bone_name -> position}
    dancer_joints: dict[str, dict[str, Any]] = {}    
    for item in items:
        segment = item.get("segment")
        pos = item.get("pos")

        #  Parse segment name: "Emma_Chest" -> prefix="Emma", bone="chest"
        parts = segment.rsplit("_", 1)
        if len(parts) != 2:
            logger.debug(f"Segment name not in expected format (prefix_bone): {segment}")
            continue
        
        skeleton_prefix, bone_name = parts
        skeleton_prefix = skeleton_prefix.strip()
        bone_name = bone_name.strip().lower()
        logger.debug(f"Parsed segment: prefix={skeleton_prefix}, bone={bone_name}")
        
        # Find matching dancer (case-insensitive comparison)
        dancer_name = None
        for name, config in dancer_config.items():
            if skeleton_prefix.lower() == config.get("skeleton_prefix", "").lower():
                dancer_name = name
                break
        
        if not dancer_name:
            logger.debug(f"No matching dancer for skeleton_prefix: {skeleton_prefix}")
            continue
        
        if dancer_name not in dancer_joints:
            dancer_joints[dancer_name] = {}
        
        dancer_joints[dancer_name][bone_name] = np.array(pos, dtype=float)
    
    handled = False
    
    # Get torso anchor points (Ally)
    ally_joints = dancer_joints.get("Ally", {})
    if not ally_joints:
        logger.warning("No Ally (torso) joints found, cannot proceed")
        return False
    
    ally_lshoulder = ally_joints.get("lshoulder")
    ally_rshoulder = ally_joints.get("rshoulder")
    ally_lhip = ally_joints.get("hip")  # Use hip as hip anchor
    ally_rhip = ally_joints.get("hip")  # Use hip as hip anchor
    
    # Get arm and leg performer's own anchor points for adjustment
    riley_joints = dancer_joints.get("Riley", {})
    riley_lshoulder = riley_joints.get("lshoulder")
    riley_rshoulder = riley_joints.get("rshoulder")
    
    emma_joints = dancer_joints.get("Emma", {})
    emma_lhip = emma_joints.get("hip")  # Use hip as left hip anchor
    emma_rhip = emma_joints.get("hip")  # Use hip as right hip anchor
    # Process each dancer based on their role
    for dancer_name, config in dancer_config.items():
        if dancer_name not in dancer_joints:
            logger.debug(f"Skipping {dancer_name}: not in parsed joints")
            continue
        
        joints = dancer_joints[dancer_name]
        role = config.get("role")
        body_parts = config.get("body_parts", [])
        
        if role == "torso":
            # Ally: send absolute positions for all configured body parts as anchor points for other dancers
            for bone_name in body_parts:
                if bone_name in joints:
                    pos = joints[bone_name]
                    # Skip if position has invalid values
                    if pos is None or not all(np.isfinite(pos)):
                        logger.debug(f"Torso: {bone_name} has invalid position")
                        continue
                    address = f"/{dancer_name.lower()}/{bone_name}"
                    try:
                        osc_client.send_message(f"{address}/x", normalize_position(float(pos[0])))
                        osc_client.send_message(f"{address}/y", normalize_position(float(pos[1])))
                        osc_client.send_message(f"{address}/z", normalize_position(float(pos[2])))
                        handled = True
                    except Exception as e:
                        logger.error(f"Error sending OSC message to {address}: {e}")
                else:
                    logger.debug(f"Torso: {bone_name} not found in joints")
        
        elif role == "arms":
            # Riley: send adjusted positions accounting for shoulder position difference
            for bone_name in body_parts:
                if bone_name in joints:
                    pos = joints[bone_name]
                    if bone_name.lower().startswith("l"):
                        # Left arm joint: adjust for difference between Riley's L shoulder and Ally's L shoulder
                        if ally_lshoulder is not None and riley_lshoulder is not None:
                            shoulder_adjustment = ally_lshoulder - riley_lshoulder
                            adjusted_pos = pos + shoulder_adjustment
                            # Skip if position has invalid values
                            if adjusted_pos is None or not all(np.isfinite(adjusted_pos)):
                                logger.debug(f"Arms (L): {bone_name} has invalid adjusted position")
                                continue
                            address = f"/{dancer_name.lower()}/{bone_name}_rel_lshoulder"
                            try:
                                osc_client.send_message(f"{address}/x", normalize_position(float(adjusted_pos[0])))
                                osc_client.send_message(f"{address}/y", normalize_position(float(adjusted_pos[1])))
                                osc_client.send_message(f"{address}/z", normalize_position(float(adjusted_pos[2])))
                                handled = True
                            except Exception as e:
                                logger.error(f"Error sending OSC message to {address}: {e}")
                        else:
                            logger.debug(f"Cannot process {bone_name}: missing shoulder anchors")
                    elif bone_name.lower().startswith("r"):
                        # Right arm joint: adjust for difference between Riley's R shoulder and Ally's R shoulder
                        if ally_rshoulder is not None and riley_rshoulder is not None:
                            shoulder_adjustment = ally_rshoulder - riley_rshoulder
                            adjusted_pos = pos + shoulder_adjustment
                            # Skip if position has invalid values
                            if adjusted_pos is None or not all(np.isfinite(adjusted_pos)):
                                logger.debug(f"Arms (R): {bone_name} has invalid adjusted position")
                                continue
                            address = f"/{dancer_name.lower()}/{bone_name}_rel_rshoulder"
                            try:
                                osc_client.send_message(f"{address}/x", normalize_position(float(adjusted_pos[0])))
                                osc_client.send_message(f"{address}/y", normalize_position(float(adjusted_pos[1])))
                                osc_client.send_message(f"{address}/z", normalize_position(float(adjusted_pos[2])))
                                handled = True
                            except Exception as e:
                                logger.error(f"Error sending OSC message to {address}: {e}")
                        else:
                            logger.debug(f"Cannot process {bone_name}: missing shoulder anchors")

        elif role == "legs":
            # Emma: send adjusted positions accounting for hip position difference
            for bone_name in body_parts:
                if bone_name in joints:
                    if bone_name.lower().startswith("l"):
                        pos = joints[bone_name]
                        # Left leg joint: adjust for difference between Emma's L hip and Ally's L hip
                        if ally_lhip is not None and emma_lhip is not None:
                            hip_adjustment = ally_lhip - emma_lhip
                            adjusted_pos = pos + hip_adjustment
                            # Skip if position has invalid values
                            if adjusted_pos is None or not all(np.isfinite(adjusted_pos)):
                                logger.debug(f"Legs (L): {bone_name} has invalid adjusted position")
                                continue
                            address = f"/{dancer_name.lower()}/{bone_name}_rel_lhip"
                            try:
                                osc_client.send_message(f"{address}/x", normalize_position(float(adjusted_pos[0])))
                                osc_client.send_message(f"{address}/y", normalize_position(float(adjusted_pos[1])))
                                osc_client.send_message(f"{address}/z", normalize_position(float(adjusted_pos[2])))
                                handled = True
                            except Exception as e:
                                logger.error(f"Error sending OSC message to {address}: {e}")
                        else:
                            logger.debug(f"Cannot process {bone_name}: missing hip anchors")
                    elif bone_name.lower().startswith("r"):
                        pos = joints[bone_name]
                        # Right leg joint: adjust for difference between Emma's R hip and Ally's R hip
                        if ally_rhip is not None and emma_rhip is not None:
                            hip_adjustment = ally_rhip - emma_rhip
                            adjusted_pos = pos + hip_adjustment
                            # Skip if position has invalid values
                            if adjusted_pos is None or not all(np.isfinite(adjusted_pos)):
                                logger.debug(f"Legs (R): {bone_name} has invalid adjusted position")
                                continue
                            address = f"/{dancer_name.lower()}/{bone_name}_rel_rhip"
                            try:
                                osc_client.send_message(f"{address}/x", normalize_position(float(adjusted_pos[0])))
                                osc_client.send_message(f"{address}/y", normalize_position(float(adjusted_pos[1])))
                                osc_client.send_message(f"{address}/z", normalize_position(float(adjusted_pos[2])))
                                handled = True
                            except Exception as e:
                                logger.error(f"Error sending OSC message to {address}: {e}")
                        else:
                            logger.debug(f"Cannot process {bone_name}: missing hip anchors")
    
    return handled