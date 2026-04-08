"""
CSV Motion Capture Replay Reader

Reads exported CSV from Motive and provides payloads compatible with the
standard middleware pipeline. Handles frame-accurate timing.

Expected CSV format (Motive standard):
  Frame, Time(Seconds), <RigidBody>x, <RigidBody>y, <RigidBody>z, ...
  or with skeletons:
  Frame, Time(Seconds), <Skeleton>:<Bone>x, <Skeleton>:<Bone>y, <Skeleton>:<Bone>z, ...

The reader extracts XYZ positions and enqueues payloads with the same shape as
nat_client.py for compatibility with middleware.
"""

import csv
import time
import threading
from queue import Queue
from typing import Dict, List, Tuple, Optional


class CSVPayload:
    """Mimics MoCapData frame structure from NatNet for compatibility."""
    
    def __init__(self, frame_num: int, timestamp: float, bodies: Dict[str, Dict]):
        """
        Args:
            frame_num: Frame number from CSV
            timestamp: Recording timestamp (seconds) from csv
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
    ):
        """
        Args:
            csv_path: Path to Motive CSV export
            out_queue: Queue to enqueue payloads
            target_name: Filter to single rigid body (None = all)
            skeleton_bones: Bone suffix whitelist (None = all)
            playback_speed: Playback speed multiplier (1.0 = normal, 2.0 = 2x faster)
        """
        self.csv_path = csv_path
        self.out_queue = out_queue
        self.target_name = target_name
        self.skeleton_bones = skeleton_bones or []
        self.playback_speed = playback_speed
        self.running = False
        self._thread: Optional[threading.Thread] = None
        self.start_wall_time: Optional[float] = None
        self.start_csv_time: Optional[float] = None
    
    def start(self):
        """Start playback thread."""
        if self.running:
            return
        self.running = True
        self._thread = threading.Thread(target=self._playback_loop, daemon=True)
        self._thread.start()
    
    def stop(self):
        """Stop playback."""
        self.running = False
        if self._thread:
            self._thread.join(timeout=2.0)
    
    def _playback_loop(self):
        """Main playback loop: reads CSV and enqueues frames at correct timing."""
        try:
            with open(self.csv_path, 'r') as f:
                reader = csv.DictReader(f)
                
                for row in reader:
                    if not self.running:
                        break
                    
                    # Parse frame metadata
                    try:
                        frame_num = int(row.get('Frame', 0))
                        csv_time = float(row.get('Time(Seconds)', 0))
                    except (ValueError, KeyError):
                        continue
                    
                    # Initialize playback timing on first frame
                    if self.start_wall_time is None:
                        self.start_wall_time = time.time()
                        self.start_csv_time = csv_time
                    
                    # Calculate when this frame should play
                    elapsed_csv_time = csv_time - self.start_csv_time
                    adjusted_csv_time = elapsed_csv_time / self.playback_speed
                    target_time = self.start_wall_time + adjusted_csv_time
                    
                    # Wait until it's time to send this frame
                    now = time.time()
                    wait_time = target_time - now
                    if wait_time > 0:
                        time.sleep(wait_time)
                    
                    # Extract position data from row
                    payload = self._parse_row(row, frame_num, csv_time)
                    if payload.bodies:  # Only enqueue if we have data
                        self.out_queue.put(payload)
        
        except FileNotFoundError:
            print(f"[CSV Reader] Error: CSV file not found: {self.csv_path}")
        except Exception as e:
            print(f"[CSV Reader] Error during playback: {e}")
        finally:
            self.running = False
    
    def _parse_row(self, row: Dict, frame_num: int, timestamp: float) -> CSVPayload:
        """
        Parse a CSV row and extract position data.
        
        Handles both formats:
          - Rigid bodies: name+axis (e.g., "Box x", "Box y", "Box z")
          - Skeletons: "SkeletonName:BoneName x/y/z"
        """
        bodies: Dict[str, Dict] = {}
        
        for key, value in row.items():
            if key in ('Frame', 'Time(Seconds)'):
                continue
            
            # Parse column name to extract body/bone name and axis
            name, axis = self._parse_column_name(key)
            if not name or not axis:
                continue
            
            # Apply target_name filter
            if self.target_name and not name.lower().startswith(self.target_name.lower()):
                continue
            
            # Apply skeleton_bones filter
            if ':' in name:  # It's a skeleton bone
                bone_part = name.split(':', 1)[1]
                if self.skeleton_bones and not any(
                    bone_part.endswith(suffix) for suffix in self.skeleton_bones
                ):
                    continue
            
            # Extract numeric value
            try:
                pos_value = float(value)
            except (ValueError, TypeError):
                continue
            
            # Store position component
            if name not in bodies:
                bodies[name] = {}
            bodies[name][axis] = pos_value
        
        return CSVPayload(frame_num, timestamp, bodies)
    
    @staticmethod
    def _parse_column_name(col_name: str) -> Tuple[Optional[str], Optional[str]]:
        """
        Parse column name into (body_name, axis).
        
        Handles:
          "Box x" → ("box", "x")
          "Alyx_Chest z" → ("alyx_chest", "z")
          "Alyx:Chest y" → ("alyx:chest", "y")
        """
        parts = col_name.rsplit(' ', 1)
        if len(parts) != 2:
            return None, None
        
        name, axis = parts
        axis = axis.lower()
        
        if axis not in ('x', 'y', 'z'):
            return None, None
        
        return name.lower(), axis


# Example usage and testing
if __name__ == "__main__":
    # Quick test: print first few payloads without timing
    import sys
    
    if len(sys.argv) < 2:
        print("Usage: python csv_reader.py <csv_file>")
        sys.exit(1)
    
    test_queue: Queue = Queue()
    reader = CSVReader(
        csv_path=sys.argv[1],
        out_queue=test_queue,
        playback_speed=1.0,
    )
    
    # Read a few frames without actual timing
    print(f"[Test] Reading from: {sys.argv[1]}")
    with open(sys.argv[1], 'r') as f:
        csv_reader = csv.DictReader(f)
        for i, row in enumerate(csv_reader):
            if i >= 5:
                break
            payload = reader._parse_row(row, i, float(row.get('Time(Seconds)', 0)))
            print(f"Frame {i}: {payload.bodies}")
