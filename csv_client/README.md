# CSV Client

A mocap data streamer that reads from Motive CSV export files instead of live UDP streams.

## Overview

The `csv_client` mimics the functionality of `mocap_client` but reads quaternion and position data from a CSV file instead of connecting to a live Motive server. This is useful for:

- Replaying recorded mocap sessions
- Testing without a live motion capture system
- Batch processing mocap data
- Debugging Isadora integration

## Usage

From the repository root:

```bash
python -m csv_client.main
```

## Configuration

Edit the configuration constants in [main.py](main.py):

- **CSV_PATH**: Path to your Motive CSV export file (default: `motive-captures/roses-take2-scene1.csv`)
- **PLAYBACK_RATE**: Speed multiplier (1.0 = real-time, 2.0 = 2x speed, etc.)
- **TARGET_SEGMENT**: Filter to a single segment name, or `None` for all segments
- **SKELETON_BONES**: Whitelist of bone names to forward, or `None` for all
- **ISADORA_IP** and **ISADORA_PORT**: OSC destination address

## CSV Format

The CSV reader expects Motive's standard export format:

```
Row 0:    Metadata (Format Version, Take Name, etc.)
Row 1:    Empty
Row 2:    Data type labels (Bone, RigidBody)
Row 3:    Segment/Bone names
Row 4:    IDs
Row 5:    Column headers (Rotation, Position, X/Y/Z/W)
Row 6+:   Data rows
```

Each segment is represented as:
- Frame number (column 0)
- Timestamp in seconds (column 1)
- Rotation: 4 floats (X, Y, Z, W) for the quaternion
- Position: 3 floats (X, Y, Z)

## Output

OSC messages are sent via the middleware layer (by default, XYZ position data):

```
/<segment>x <float>
/<segment>y <float>
/<segment>z <float>
```

To add custom handlers (e.g., quaternion output), modify [../utils/middleware.py](../utils/middleware.py).

## Example

Stream motion capture data at 2x speed, filtering only the "Emma:Head" segment:

```python
CSV_PATH = "motive-captures/roses-take2-scene1.csv"
PLAYBACK_RATE = 2.0
TARGET_SEGMENT = "Emma:Head"
```

Generate two output sample runs to understand the data flow:

```bash
# List available segments in your CSV
python -c "from csv_client.csv_reader import CSVReader; r = CSVReader('motive-captures/roses-take2-scene1.csv', __import__('queue').Queue()); r._parse_headers(open('motive-captures/roses-take2-scene1.csv').__enter__().__next__() for _ in range(6)); print(list(r.segment_map.keys())[:10])"
```

## See Also

- [mocap_client](../mocap_client/) — Live Motive server integration
- [OSC Processor](../mocap_client/osc_processor.py) — Handles OSC output
- [Middleware](../utils/middleware.py) — OSC effect handlers
