"""CSV reader for mocap data.

Parses a Motive CSV export file and streams quaternion payloads to a queue
at a configurable playback rate.
"""
import csv
import time
import logging
from pathlib import Path
from queue import Queue
from typing import Optional


class CSVReader:
    """Read mocap data from a Motive CSV export and stream via queue.

    Expected CSV format (from Motive export):
    - Row 0: Metadata (Format Version, Take Name, etc.)
    - Row 1: Empty
    - Row 2: Type (Bone, RigidBody, etc.)
    - Row 3: Segment/Bone names
    - Row 4: IDs
    - Row 5: Column headers (Rotation, Position, and component names X/Y/Z/W)
    - Row 6+: Data rows with frame number, time, and mocap values

    Payload schema pushed onto out_queue:
        {
            "segment":   str,           # segment/bone name from CSV
            "quat":      (qx,qy,qz,qw), # orientation quaternion
            "pos":       (x, y, z),     # position vector
            "frame":     int,           # frame number
            "timestamp": float,         # timestamp in seconds
        }
    """

    def __init__(
        self,
        csv_path: str,
        out_queue: Queue,
        target_segment: Optional[str] = None,
        skeleton_bones: Optional[list] = None,
        playback_rate: float = 1.0,
    ):
        """Initialize the CSV reader.

        Parameters
        ----------
        csv_path : str
            Path to the CSV file (relative or absolute).
        out_queue : Queue
            Destination queue for quaternion payloads.
        target_segment : str, optional
            Name of the single segment to forward. Pass None to forward all.
        skeleton_bones : list, optional
            Whitelist of bone names to forward. Matched by suffix.
            Pass None to forward all bones.
        playback_rate : float
            Playback speed multiplier (1.0 = real-time).
        """
        self.csv_path = Path(csv_path)
        self.out_queue = out_queue
        self.target_segment = target_segment
        self.skeleton_bones = skeleton_bones
        self.playback_rate = max(0.01, playback_rate)  # Prevent divide by zero

        self.log = logging.getLogger("csv_reader")

        # Will be populated during header parsing
        self.segment_map = {}  # segment_name -> {"rotation": [col_indices], "position": [col_indices]}
        self.segment_order = []  # Ordered list of segment names for iteration
        self.segment_type = {}  # segment_name -> "Bone" or "RigidBody"

    def read_and_stream(self):
        """Read the CSV file and stream mocap data to the output queue."""
        if not self.csv_path.exists():
            raise FileNotFoundError(f"CSV file not found: {self.csv_path}")

        with open(self.csv_path, 'r', newline='') as f:
            reader = csv.reader(f)

            # Parse header rows (rows 0-5)
            self._parse_headers(reader)

            # Skip the column label row (row 6: Frame, Time, X, Y, Z, W, etc.)
            next(reader)

            if not self.segment_map:
                self.log.warning("No segments found in CSV")
                return

            self.log.info(f"Found {len(self.segment_map)} segment(s)")

            # Get the frame timing from the first data row
            first_row = next(reader)
            start_time = time.time()
            frame_num = int(first_row[0])
            timestamp = float(first_row[1])
            last_frame_time = timestamp

            # Process first row
            self._enqueue_frame_data(first_row, frame_num, timestamp)

            # Process remaining rows
            for row in reader:
                if not row or not row[0]:
                    continue

                frame_num = int(row[0])
                timestamp = float(row[1])
                delta_time = timestamp - last_frame_time
                last_frame_time = timestamp

                # Enqueue the data
                self._enqueue_frame_data(row, frame_num, timestamp)

                # Sleep to respect playback rate
                if delta_time > 0:
                    sleep_time = delta_time / self.playback_rate
                    time.sleep(sleep_time)

    def _parse_headers(self, reader):
        """Parse CSV header rows to build the segment map."""
        # Read header rows
        row_0 = next(reader)  # Metadata
        row_1 = next(reader)  # Empty
        row_2 = next(reader)  # Type (Bone, RigidBody, etc.)
        row_3 = next(reader)  # Segment/Bone names
        row_4 = next(reader)  # IDs
        row_5 = next(reader)  # Component type (Rotation or Position)

        # Build segment map from header rows
        # Column 0: Frame, Column 1: Time
        # Columns 2+: Data organized as [Rotation(4) + Position(3)] per segment

        # First pass: collect all unique segments in order of first appearance
        seen_segments = set()
        for col_idx in range(2, len(row_3)):
            segment_name = row_3[col_idx] if col_idx < len(row_3) else ""
            segment_type = row_2[col_idx] if col_idx < len(row_2) else ""
            if segment_name and segment_name not in seen_segments:
                self.segment_order.append(segment_name)
                self.segment_map[segment_name] = {"rotation": [], "position": []}
                self.segment_type[segment_name] = segment_type
                seen_segments.add(segment_name)

        # Second pass: populate rotation and position column indices
        # Track position count per segment to only take first 3 position columns
        position_count = {}
        for col_idx in range(2, len(row_3)):
            segment_name = row_3[col_idx] if col_idx < len(row_3) else ""
            component_type = row_5[col_idx] if col_idx < len(row_5) else ""

            if not segment_name or segment_name not in self.segment_map:
                continue

            # Categorize by component type
            if component_type == "Rotation":
                self.segment_map[segment_name]["rotation"].append(col_idx)
            elif component_type == "Position":
                # Only take first 3 position columns per segment (X, Y, Z)
                if position_count.get(segment_name, 0) < 3:
                    self.segment_map[segment_name]["position"].append(col_idx)
                    position_count[segment_name] = position_count.get(segment_name, 0) + 1

        self.log.debug(f"Segment map: {self.segment_map}")

    def _enqueue_frame_data(self, row: list, frame_num: int, timestamp: float):
        """Enqueue mocap payloads for a single frame."""
        for segment_name in self.segment_order:
            if self.target_segment and segment_name != self.target_segment:
                continue

            # Determine segment type and apply filtering
            seg_type = self.segment_type.get(segment_name, "")
            is_bone = seg_type == "Bone"

            # Apply skeleton bone filter only to skeleton bones
            if is_bone and self.skeleton_bones is not None:
                bone_suffix = segment_name.rsplit(":", 1)[-1] if ":" in segment_name else segment_name
                if bone_suffix not in self.skeleton_bones:
                    continue

            seg_data = self.segment_map[segment_name]
            rotation_cols = seg_data["rotation"]
            position_cols = seg_data["position"]

            # Extract rotation (quaternion as x,y,z,w)
            if len(rotation_cols) == 4:
                try:
                    quat = tuple(float(row[col_idx]) for col_idx in rotation_cols)
                except (ValueError, IndexError):
                    continue
            else:
                continue

            # Extract position (x,y,z)
            if len(position_cols) == 3:
                try:
                    pos = tuple(float(row[col_idx]) for col_idx in position_cols)
                except (ValueError, IndexError):
                    continue
            else:
                pos = (0.0, 0.0, 0.0)

            # Enqueue the payload
            payload = {
                "segment": segment_name,
                "quat": quat,
                "pos": pos,
                "frame": frame_num,
                "timestamp": timestamp,
            }
            self.out_queue.put(payload)
