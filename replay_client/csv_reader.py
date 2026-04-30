"""
CSV Motion Capture Replay Reader

Reads exported CSV from Motive and provides payloads compatible with the
standard middleware pipeline. Handles frame-accurate timing.

Expected CSV format (Motive standard):
  - Metadata rows at top (Format Version, Take Name, etc.)
  - Type row: indicates "Bone", "Bone Marker", "Marker"
  - Name row: skeleton:bone names (e.g., "Will:Chest")
  - Data rows: Frame, then alternating X, Y, Z for each bone (possibly with quaternion)

The reader skips metadata, parses the header structure, and extracts XYZ positions.
"""

import csv
import time
import threading
import logging
from queue import Queue
from typing import Dict, List, Tuple, Optional

log = logging.getLogger("replay_client.csv_reader")

class CSVPayload:
    """Mimics MoCapData frame structure from NatNet for compatibility."""
    
    def __init__(self, frame_num: int, timestamp: float, bodies: Dict[str, Dict]):
        """
        Args:
            frame_num: Frame number from CSV
            timestamp: Recording timestamp (milliseconds / 1000) from csv
            bodies: Dict[name] -> {x, y, z} position data
        """
        self.frame_num = frame_num
        self.timestamp = timestamp
        self.bodies = bodies  # {name: {x, y, z, ...}}


class CSVReader:
    """
    Reads Motive CSV exports and enqueues payloads at correct timing.
    
    Handles both rigid bodies and skeleton bones. Replays with frame-accurate
    timing based on original timestamps.
    """
    
    def __init__(
        self,
        csv_path: str,
        out_queue: Queue,
        target_name: Optional[str] = None,
        skeleton_bones: Optional[List[str]] = None,
        playback_speed: float = 1.0,
        position_scale: float = 0.001,
    ):
        """
        Args:
            csv_path: Path to Motive CSV export
            out_queue: Queue to enqueue payloads
            target_name: Filter to single rigid body (None = all)
            skeleton_bones: Bone suffix whitelist (None = all)
            playback_speed: Playback speed multiplier (1.0 = normal, 2.0 = 2x faster)
            position_scale: Scalar applied to CSV position values before enqueue.
                Use 0.001 for CSV values in millimeters to output meters.
        """
        self.csv_path = csv_path
        self.out_queue = out_queue
        self.target_name = target_name
        self.skeleton_bones = skeleton_bones or []
        self.playback_speed = playback_speed
        self.position_scale = position_scale
        self.running = False
        self._thread: Optional[threading.Thread] = None
        self.start_wall_time: Optional[float] = None
        self.start_csv_time: Optional[float] = None
        self.frame_count = 0
        self._debug_enqueued_samples = 0
    
    def start(self):
        """Start playback thread. Returns True if successfully started."""
        if self.running:
            return True
        self.running = True
        self._thread = threading.Thread(target=self._playback_loop, daemon=True)
        log.debug("Starting CSV reader thread for %s", self.csv_path)
        self._thread.start()
        return True
    
    def stop(self):
        """Stop playback."""
        self.running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        log.debug("Stopped CSV reader")
    
    def _playback_loop(self):
        """Main playback loop: reads CSV and enqueues frames at correct timing."""
        log.info("Entering read loop for: %s", self.csv_path)
        try:
            with open(self.csv_path, 'r') as f:
                lines = f.readlines()
            
            # Parse metadata from first row (key,value pairs)
            metadata = {}
            if lines:
                meta_line = lines[0].strip()
                # Split by comma but preserve pairs
                parts = meta_line.split(',')
                for i in range(0, len(parts), 2):
                    if i + 1 < len(parts):
                        metadata[parts[i]] = parts[i + 1]
            
            log.debug("Metadata: %s", metadata)
            
            # Read column structure from rows 4, 6, 7 (0-indexed: 3, 5, 6)
            col_bones = []  # Will store (bone_name, descriptor, axis) for each data column
            col_descriptors = []
            col_axes = []
            
            if len(lines) > 3:
                bone_line = lines[3].strip().split(',')
                log.debug("Row 4 (bones): %s...", bone_line[:10])
                col_bones = bone_line
            
            if len(lines) > 5:
                desc_line = lines[5].strip().split(',')
                log.debug("Row 6 (descriptors): %s...", desc_line[:10])
                col_descriptors = desc_line
            
            if len(lines) > 6:
                axis_line = lines[6].strip().split(',')
                log.debug("Row 7 (axes): %s...", axis_line[:10])
                col_axes = axis_line
            
            # Build a column map: col_index -> (bone_name, descriptor, axis)
            col_map = {}
            for i in range(len(col_bones)):
                bone = col_bones[i] if i < len(col_bones) else ""
                desc = col_descriptors[i] if i < len(col_descriptors) else ""
                axis = col_axes[i] if i < len(col_axes) else ""
                col_map[i] = (bone, desc, axis)
            
            log.debug("Column map (first 15):")
            for i in range(min(15, len(col_map))):
                log.debug("  Col %s: %s", i, col_map[i])
            
            # Start reading data from row 8 (0-indexed: 7)
            reader = csv.reader(lines[7:])
            frame_num_row = 0
            
            log.info("Total lines in file: %s", len(lines))
            log.info("Starting to read from line 7 (row 8)")
            
            for row in reader:
                if not self.running:
                    break
                
                if not row or len(row) < 2:
                    if frame_num_row < 3:
                        log.debug("Skipping empty/short row %s: len=%s", frame_num_row, len(row))
                    frame_num_row += 1
                    continue
                
                if frame_num_row < 5:
                    log.debug("Data row %s: first 5 cols = %s", frame_num_row, row[:5])
                
                try:
                    frame_num = int(float(row[0])) if row[0] else 0
                    csv_time = float(row[1]) if len(row) > 1 and row[1] else 0.0
                    
                    if frame_num_row < 3:
                        log.debug("Parsed frame %s, time %s", frame_num, csv_time)
                except (ValueError, IndexError) as e:
                    if frame_num_row < 3:
                        log.debug("Parse error on row %s: %s, values=%s", frame_num_row, e, row[:3])
                    frame_num_row += 1
                    continue
                
                frame_num_row += 1
                
                # Initialize playback timing on first frame
                if self.start_wall_time is None:
                    self.start_wall_time = time.time()
                    self.start_csv_time = csv_time
                    log.info("Starting playback at frame %s, CSV time %.3fs", frame_num, csv_time)
                
                # Calculate when this frame should play
                elapsed_csv_time = csv_time - self.start_csv_time
                adjusted_csv_time = elapsed_csv_time / self.playback_speed
                target_time = self.start_wall_time + adjusted_csv_time
                
                # Wait until it's time to send this frame
                now = time.time()
                wait_time = target_time - now
                if wait_time > 0.001:
                    time.sleep(wait_time)
                
                # Extract position data from row using col_map
                payload = self._parse_row(row, frame_num, csv_time, col_map)
                if payload.bodies:
                    # Convert CSVPayload to dict format expected by OSCProcessor
                    if frame_num <= 2:
                        log.debug(f"Frame {frame_num}: payload.bodies keys = {list(payload.bodies.keys())}")
                    for segment_name, body_data in payload.bodies.items():
                        # body_data has nested shape {'pos': {'x':..,'y':..,'z':..}, 'rot': {...}}
                        pos_dict = body_data.get('pos', {})
                        if not pos_dict:
                            if frame_num <= 2:
                                log.debug(f"  {segment_name}: no pos_dict")
                            continue
                        pos = (
                            pos_dict.get('x', 0.0),
                            pos_dict.get('y', 0.0),
                            pos_dict.get('z', 0.0),
                        )
                        if frame_num <= 2:
                            log.debug(f"  {segment_name}: enqueuing pos={pos}")
                        msg = {
                            "segment": segment_name,
                            "pos": pos,
                            "frame": frame_num,
                            "timestamp": payload.timestamp,
                        }
                        self.out_queue.put(msg)
                    self.frame_count += 1
                else:
                    if frame_num <= 2:
                        log.debug(f"Frame {frame_num}: payload.bodies is empty")
        
        except FileNotFoundError:
            log.error("CSV file not found: %s", self.csv_path)
        except Exception as e:
            log.exception("Error during playback: %s", e)
            import traceback
            traceback.print_exc()
        finally:
            self.running = False
            log.info("Exiting read loop: processed %s frames", self.frame_count)
    
    def _parse_row(self, row: list, frame_num: int, timestamp: float, col_map: Dict) -> CSVPayload:
        """
        Parse a CSV row using the column map.
        
        col_map: Dict[col_index] -> (bone_name, descriptor, axis)
        For each bone, we collect position (x,y,z) and rotation (x,y,z,w) values.
        """
        bodies: Dict[str, Dict] = {}
        
        # Skip first two columns (Frame and Time)
        for col_idx in range(2, len(row)):
            if col_idx not in col_map:
                continue
            
            bone_name, descriptor, axis = col_map[col_idx]
            
            # Debug first few columns
            if frame_num <= 1 and col_idx < 10:
                log.debug(
                    "Col %s: bone='%s', desc='%s', axis='%s', value='%s'",
                    col_idx,
                    bone_name,
                    descriptor,
                    axis,
                    row[col_idx] if col_idx < len(row) else 'OOB',
                )
            
            # Skip if no bone name
            if not bone_name or bone_name.strip() == '':
                continue
            
            axis_lower = axis.lower().strip()
            descriptor_lower = descriptor.lower().strip() if descriptor else ""
            
            # Only process Position (x,y,z) or Rotation (x,y,z,w) descriptors
            if descriptor_lower not in ('position', 'rotation'):
                if frame_num <= 1 and col_idx < 20:
                    log.debug(f"Col {col_idx}: skipping descriptor '{descriptor_lower}'")
                continue
            
            if descriptor_lower == 'position' and axis_lower not in ('x', 'y', 'z'):
                if frame_num <= 1 and col_idx < 20:
                    log.debug(f"Col {col_idx}: skipping position axis '{axis_lower}'")
                continue
            if descriptor_lower == 'rotation' and axis_lower not in ('x', 'y', 'z', 'w'):
                if frame_num <= 1 and col_idx < 20:
                    log.debug(f"Col {col_idx}: skipping rotation axis '{axis_lower}'")
                continue
            
            # Apply target_name filter
            if self.target_name and not bone_name.lower().startswith(self.target_name.lower()):
                continue
            
            # Apply skeleton_bones filter
            if ':' in bone_name:
                bone_part = bone_name.split(':', 1)[1]
                if self.skeleton_bones and not any(
                    bone_part.lower().endswith(suffix.lower()) for suffix in self.skeleton_bones
                ):
                    continue
            
            # Extract numeric value
            try:
                if col_idx >= len(row):
                    continue
                value = float(row[col_idx])
                if descriptor_lower == 'position':
                    value *= self.position_scale
                # Skip NaN and inf values
                if not (-1e10 < value < 1e10):
                    continue
            except (ValueError, TypeError, OverflowError, IndexError):
                if frame_num <= 1 and col_idx < 10:
                    log.debug(
                        "Col %s: parse error for value '%s'",
                        col_idx,
                        row[col_idx] if col_idx < len(row) else 'OOB',
                    )
                continue
            
            # Store component (position or rotation)
            bone_key = bone_name.lower()
            if bone_key not in bodies:
                bodies[bone_key] = {}
            
            if descriptor_lower == 'position':
                if 'pos' not in bodies[bone_key]:
                    bodies[bone_key]['pos'] = {}
                bodies[bone_key]['pos'][axis_lower] = value
            elif descriptor_lower == 'rotation':
                if 'rot' not in bodies[bone_key]:
                    bodies[bone_key]['rot'] = {}
                bodies[bone_key]['rot'][axis_lower] = value
        
        if frame_num <= 1:
            log.debug("Frame %s: parsed %s bodies, total row length: %s", frame_num, len(bodies), len(row))
        
        return CSVPayload(frame_num, timestamp, bodies)

