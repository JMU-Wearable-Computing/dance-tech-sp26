# max_client

Motion capture data client for MaxMSP audio processing.

## Overview

`max_client` is a lightweight Python application that:

1. **Connects to Motive** — reads real-time rigid-body and skeleton data via NatNet
2. **Filters segments** — optionally tracks only specific body parts (e.g., "Chest")
3. **Sends to MaxMSP** — broadcasts quaternion and position data via OSC for audio synthesis

The data flow is identical to `mocap_client`, but the OSC processor sends to MaxMSP instead of Isadora, with OSC paths following Max's standard parameter conventions:

```
/audio/param/<segment>/<attribute>/raw
```

Specific message paths:

- `/audio/param/<segment>/quat/raw` — quaternion (qx, qy, qz, qw) for spatial audio
- `/audio/param/<segment>/pos/raw` — position (x, y, z) for panning / distance attenuation
- `/audio/param/<segment>/sync/raw` — frame number and timestamp for sync

## Configuration

Edit the constants at the top of `main.py`:

```python
# Motive server
SERVER_IP      = "127.0.0.1"
LOCAL_IP       = "127.0.0.1"
USE_MULTICAST  = True

# Which body parts to track (None = all)
TARGET_NAME = None
SKELETON_BONES = ["Chest"]

# MaxMSP receiver
MAXMSP_IP = "127.0.0.1"
MAXMSP_PORT = 9999
```

## Running

```bash
# From repo root:
python -m max_client.main

# Press Ctrl+C to stop
```

## Testing

A test Max patcher (`mocap_test.maxpat`) is included to verify OSC communication:

1. Open `mocap_test.maxpat` in Max
2. Run `python -m max_client.main` from the repo root
3. The patcher displays live quaternion, position, and sync data
4. Check Max console for connection status messages

## Architecture

```
Motive (NatNet)
    ↓
NatClient (reads frames, filters by segment)
    ↓
Queue (thread-safe payload buffer)
    ↓
OSCProcessor (formats and sends via OSC)
    ↓
MaxMSP (receives on port 9999)
```

## Dependencies

- See `../mocap_client/requirements.txt` for NatNet and OSC libraries
- This client reuses the mocap utilities from `mocap_client/`
