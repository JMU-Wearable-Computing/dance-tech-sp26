# CSV Client - OSC Streaming from Motive CSV Files

## Summary

The `csv_client` is a new program that streams mocap data from Motive CSV export files to Isadora via OSC, mimicking the functionality of `mocap_client` but reading from a file instead of a live Motive server.

### Created Files

```
csv_client/
├── __init__.py          # Package initialization
├── main.py              # Entry point and configuration
├── csv_reader.py        # CSV parsing and data streaming logic
├── test.py              # Quick test suite  
└── README.md            # Detailed documentation
```

## Quick Start

### 1. Basic Usage

From the repository root:

```bash
python -m csv_client.main
```

This will stream the configured CSV file to Isadora at `127.0.0.1:1234` at real-time playback speed.

### 2. Configuration

Edit `csv_client/main.py` to customize:

```python
# Path to CSV file
CSV_PATH = "motive-captures/roses-take2-scene1.csv"

# Playback speed (1.0 = real-time, 2.0 = 2x speed)
PLAYBACK_RATE = 1.0

# Filter to specific segment (None = all segments)
TARGET_SEGMENT = None  # e.g., "Emma:Head"

# Filter to specific bones by suffix (None = all bones)
SKELETON_BONES = None  # e.g., ["Head", "Chest"]

# Isadora OSC connection
ISADORA_IP = "127.0.0.1"
ISADORA_PORT = 1234
```

### 3. Testing

Run the test suite to verify data parsing:

```bash
python -m csv_client.test
```

Output shows found segments, processed frames, and sample payloads.

## How It Works

### Data Flow

```
CSV File
  ↓
CSVReader.read_and_stream()
  ↓
Parse headers (rows 0-6)
  ↓
For each data row:
  - Extract frame number and timestamp
  - Parse rotation (quaternion) and position for each segment  
  - Enqueue payload { segment, quat, pos, frame, timestamp }
  - Sleep to match playback rate
  ↓
OSCProcessor thread
  ↓
Middleware handlers (e.g., send_xyz)
  ↓
OSC messages to Isadora
```

### CSV Format Expected

The program expects Motive's standard CSV export format:

```
Row 0:    Metadata (Format Version, Take Name, FPS, etc.)
Row 1:    (empty)
Row 2:    Type labels (Bone, RigidBody, etc.)
Row 3:    Segment/Bone names
Row 4:    IDs
Row 5:    Component types (Rotation, Position)
Row 6:    Column labels (Frame, Time, X, Y, Z, W, etc.)
Row 7+:   Data rows
```

Each segment has:
- 4 float values for **Rotation** (Quaternion: X, Y, Z, W)
- 3 float values for **Position** (X, Y, Z)

### Payload Format

Payloads pushed to the queue match `mocap_client` format:

```python
{
    "segment":   str,           # e.g., "Emma:Head"
    "quat":      (qx,qy,qz,qw), # Orientation quaternion
    "pos":       (x, y, z),     # Position vector  
    "frame":     int,           # Frame number from CSV
    "timestamp": float,         # Timestamp in seconds
}
```

These are passed through the middleware layer (same as `mocap_client`), which by default sends OSC messages for position:

```
/emma:head x <float>
/emma:head y <float>
/emma:head z <float>
```

### Segment Name Mapping

Segment names from the CSV are used directly in OSC addresses, converted to lowercase:

- `Emma:Head` → `/emma:head`
- `Emma:LShoulder` → `/emma:lshoulder`

## Advanced Configuration

### Filtering by Segment Name

Stream only one segment:

```python
TARGET_SEGMENT = "Emma:Head"
```

### Filtering by Bone Suffix

Stream only specific bones (matched by suffix after `:`):

```python
SKELETON_BONES = ["Head", "Chest", "LShoulder"]  # Will match Emma:Head, Emma:Chest, etc.
```

### Playback Speeds

```python
PLAYBACK_RATE = 0.5   # Half speed (slow motion)
PLAYBACK_RATE = 1.0   # Real-time (default)
PLAYBACK_RATE = 2.0   # 2x speed (fast forward)
PLAYBACK_RATE = 100.0 # Very fast (for testing)
```

### Custom OSC Handlers

To output quaternion data instead of position, modify `/utils/middleware.py` or create a custom handler:

```python
def send_quat(item: dict, osc_client: Any) -> bool:
    """Send quaternion data as OSC."""
    quat = item.get("quat")
    if quat is None or len(quat) != 4:
        return False
    
    segment = item.get("segment")
    base = segment.lower() if segment else "unknown"
    
    osc_client.send_message(f"/{base}qx", float(quat[0]))
    osc_client.send_message(f"/{base}qy", float(quat[1]))
    osc_client.send_message(f"/{base}qz", float(quat[2]))
    osc_client.send_message(f"/{base}qw", float(quat[3]))
    return True
```

Then add it to `middleware.py`'s `DEFAULT_HANDLERS`.

## Troubleshooting

### "CSV file not found"

Ensure the CSV path is correct relative to the workspace root, or use an absolute path:

```python
CSV_PATH = "/path/to/your/file.csv"
```

### No segments found

Check that the CSV file is in Motive's export format. The first 7 rows should contain metadata and headers, not data.

### OSC messages not reaching Isadora

1. Verify Isadora is configured to receive OSC on the specified IP and port
2. Ensure firewall allows the connection (127.0.0.1 is usually fine)
3. Check Isadora's OSC receiver objects are set to the correct port

### Slow performance with large CSV files

The large roses-take2-scene1.csv file will take time to fully stream. For testing, use a smaller CSV file or set `PLAYBACK_RATE` to a high value to process faster.

## Integration with Mocap Client

The `csv_client` and `mocap_client` can be used interchangeably:

- **`mocap_client`**: Use with a live Motive server for real-time capture
- **`csv_client`**: Use with recorded CSV files for offline processing/testing

Both feed into the same OSC processing pipeline, so Isadora configuration remains identical.

## Files Reference

- [csv_reader.py](csv_reader.py) — Core CSV parsing and streaming logic
- [main.py](main.py) — Program entry point with configuration
- [test.py](test.py) — Test suite to verify functionality
- [README.md](README.md) — Detailed API documentation
- [../mocap_client/osc_processor.py](../mocap_client/osc_processor.py) — Shared OSC handling
- [../utils/middleware.py](../utils/middleware.py) — OSC output handlers
