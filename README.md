# dance-tech-sp26
Course repo for ENGR 490 / DANC 303 — Spring 2026

---

## mocap_client

Real-time motion-capture pipeline that connects to an OptiTrack Motive server
via NatNet, filters tracked rigid bodies and skeleton bones, and streams XYZ
position as individual OSC float messages to Isadora.

### Quick start

```powershell
# 1. Install dependencies (once)
python -m venv .venv
.venv\Scripts\Activate.ps1
pip install -r mocap_client/requirements.txt

# 2. Run
.\start_mocap.bat

# 3. Verify output (separate window)
python osc_listener.py
```


### How to make a new tool/plugin

OSC output (data getting sent to isadora) is built from small plugin functions in `utils/`.

Each plugin should follow the shared handler shape used by `utils/middleware.py`:

```python
def my_plugin(item: dict, osc_client) -> bool:
  ...
```

- `item` is the payload from `nat_client.py`
- `osc_client` is the active `pythonosc` client
- return `True` when the plugin handled the payload, otherwise return `False`

To add a new effect:

1. Create a new file in `utils/` for the effect logic.
2. Keep the plugin signature consistent with `utils/middleware.py`.
3. Register the plugin in `DEFAULT_HANDLERS` inside [utils/middleware.py](utils/middleware.py).
4. Leave `mocap_client/osc_processor.py` focused on queue handling and dispatch.

Example plugins already in the repo are [utils/xyz.py](utils/xyz.py) and [utils/velocity.py](utils/velocity.py).

*IMPORTANT*: these functions dont return, they send data directly out
```python
    osc_client.send_message(f"/{base}x", float(pos[0]))

```


### Architecture

```
Motive (NatNet UDP)
        │
        ▼
  NatNetClient          ← low-level NatNet protocol (from polymidi)
  (NatNetClient.py)       runs two internal threads: data + command
        │
        ├─ command_q ─► DataDescriptions
        │                  refreshed every 5 s → id→name map
        │                  (picks up new assets added mid-session)
        │
        └─ data_q ────► MoCapData frames
                           │
                           ▼
                       NatClient              (nat_client.py)
                       _frame_loop thread
                         1. refresh id→name map every 5 s
                         2. drain frames → rigid bodies + skeleton bones
                         3. look up name; mask high 16 bits for bone IDs
                         4. apply TARGET_NAME / SKELETON_BONES filter
                         5. enqueue payload
                           │
                           ▼
                       out_queue (thread-safe Queue)
                           │
                           ▼
                       OSCProcessor           (osc_processor.py)
                       consumer daemon thread @ 30 Hz
                         - hands payloads to middleware.py
                           │
                           ▼
                       middleware.py          (dispatch layer)
                         - routes each payload to enabled effects
                           │
                           ▼
                       xyz.py                 (position sender)
                         - sends /<name>x, /<name>y, /<name>z as floats
```

### OSC output format

Position is sent as **three separate float messages** at up to 30 Hz:

```
/<name>x   float   (e.g. /boxx, /alyxchestx)
/<name>y   float
/<name>z   float
```

- Rigid bodies use the Motive asset name lowercased (e.g. `Box` → `/boxx`)
- Skeleton bones use the full bone name lowercased (e.g. `Alyx_Chest` → `/alyxchestx`)

### What gets streamed

- **Rigid bodies**: all tracked rigid bodies pass through (filter with `TARGET_NAME`)
- **Skeleton bones**: only bones whose suffix matches `SKELETON_BONES` are forwarded
  (e.g. `["Chest"]` matches `Alyx_Chest`, `Bob_Chest`, etc.)
- Skeleton bones that also appear in the top-level rigid body stream are suppressed
  to avoid duplicates

### Configuration (`mocap_client/main.py`)

| Variable | Default | Description |
|---|---|---|
| `SERVER_IP` | `127.0.0.1` | IP of the Motive machine |
| `LOCAL_IP` | `127.0.0.1` | IP of this machine's network interface |
| `USE_MULTICAST` | `True` | Must match Motive's streaming mode |
| `TARGET_NAME` | `None` | `None` = all rigid bodies; `"Box"` = one specific body |
| `SKELETON_BONES` | `["Chest"]` | Bone suffix whitelist; `None` = all bones |
| `ISADORA_IP` | `127.0.0.1` | Isadora machine IP |
| `ISADORA_PORT` | `1234` | Isadora OSC listen port |

### Motive setup

1. Enable **Data Streaming** (Edit → Project → Streaming).
2. Name rigid bodies clearly (e.g. `Box`). Bones are named automatically as `<SkeletonName>_<BoneName>` (e.g. `Alyx_Chest`).
3. The pipeline refreshes asset names every 5 s — adding or renaming assets mid-session is picked up without restarting.

### Components

| File | Role |
|---|---|
| `mocap_client/NatNetClient.py` | Low-level NatNet protocol. Sourced from polymidi. |
| `mocap_client/MoCapData.py` | Data-model classes for a MoCap frame. Sourced from polymidi. |
| `mocap_client/DataDescriptions.py` | Data-model classes for Motive asset definitions. Sourced from polymidi. Has a known `NameError` bug in `get_as_string()` that does not affect operation. |
| `mocap_client/nat_client.py` | Wraps NatNetClient; builds id→name map; filters and enqueues payloads. |
| `utils/middleware.py` | Dispatch layer for outgoing OSC effects. |
| `utils/xyz.py` | XYZ position sender for OSC output. |
| `mocap_client/osc_processor.py` | Daemon thread consuming payloads and dispatching them through middleware at 30 Hz. |
| `mocap_client/main.py` | Entry point and configuration. |

---

## replay_client

Motion-capture CSV playback pipeline. Replays pre-recorded motion capture data
from Motive CSV exports through the same middleware pipeline as the live system,
enabling testing, development, and demonstrations without a running Motive server.

### Quick start

```bash
# 1. Install dependencies (once)
python -m venv .venv
source .venv/bin/activate  # or .venv\Scripts\Activate.ps1 on Windows
pip install -r replay_client/requirements.txt

# 2. Run playback
python -m replay_client.main replay_client/Will\ Performance.csv

# 3. Verify output (separate window)
python osc_listener.py
```

### Usage

```bash
python -m replay_client.main <csv_file> [options]
```

#### Options

| Option | Default | Description |
|--------|---------|-------------|
| `csv_file` | required | Path to Motive CSV export file |
| `--target-name` | `None` | Filter to single rigid body (e.g., `"dancer1"`). `None` = all |
| `--skeleton-bones` | `["Chest"]` | Bone suffix whitelist (e.g., `["Chest", "Hips"]`) |
| `--speed` | `1.0` | Playback speed multiplier (`1.0` = normal, `2.0` = 2× faster) |
| `--isadora-ip` | `127.0.0.1` | IP address of Isadora machine |
| `--isadora-port` | `1234` | OSC port on Isadora |

#### Examples

```bash
# Replay all data at normal speed
python -m replay_client.main recordings/performance.csv

# Replay only "dancer1" rigid body at 1.5× speed
python -m replay_client.main recordings/performance.csv \
  --target-name dancer1 \
  --speed 1.5

# Replay skeleton bones (Chest, Hips) to remote Isadora
python -m replay_client.main recordings/performance.csv \
  --skeleton-bones Chest Hips \
  --isadora-ip 192.168.1.100
```

### Goals

**replay_client** solves three key problems:

1. **Development without Motive** — Test the full streaming pipeline using recorded data
2. **Deterministic testing** — Replay the same performance repeatedly for consistent debugging
3. **Demonstrations** — Show dance-tech effects without requiring real-time motion capture

### How it works

The replay pipeline mirrors `mocap_client` exactly:

1. **CSV Reader** (`replay_client/csv_reader.py`)
   - Parses Motive CSV export format (metadata, column structure, data rows)
   - Extracts position (x, y, z) and rotation (x, y, z, w) data per frame
   - Applies target name and bone filtering
   - Maintains frame-accurate timing based on original CSV timestamps
   - Respects playback speed multiplier for slow-motion or fast-forward

2. **Payload Conversion**
   - Translates CSV frame data into NatClient-compatible dict format:
     ```python
     {
         "segment": segment_name,
         "pos": (x, y, z),
         "quat": (x, y, z, w),  # if available
         "frame": frame_num,
         "timestamp": timestamp,
     }
     ```

3. **OSC Output** (via shared middleware)
   - Routes payloads through `utils/middleware.py` (same as live system)
   - Sends /<name>x, /<name>y, /<name>z as individual OSC floats
   - Respects 30 Hz output rate and segment filtering

### CSV format

Expected Motive CSV export structure:

```
Row 1   (0-indexed: 0)    Metadata (key,value pairs)
Row 2   (0-indexed: 1)    Empty
Row 3   (0-indexed: 2)    Bone/No-Bone
Row 4   (0-indexed: 3)    Bone/segment names (e.g., "Will:Chest", "Will:Hips")
Row 5   (0-indexed: 4)    Empty
Row 6   (0-indexed: 5)    Descriptors (e.g., "Position", "Rotation")
Row 7   (0-indexed: 6)    Axes (e.g., "X", "Y", "Z", "W")
Row 8+  (0-indexed: 7+)   Data rows (Frame, Time, then position/rotation values)
```

The CSV reader automatically parses this structure and handles variations gracefully.

### Configuration

Unlike `mocap_client`, `replay_client` uses command-line arguments for all configuration
(no hardcoded constants). This allows flexible testing without code changes.

See **Options** section above for all available parameters.
| `osc_listener.py` | Debug tool — prints all incoming OSC messages on port 1234. |
| `start_mocap.bat` | Windows launcher — activates venv and runs `python -m mocap_client.main`. |

### Known gotchas

**Skeleton bone IDs in frame data**
NatNet encodes skeleton bone IDs in frame packets as `(skeleton_id << 16) | bone_id`,
but `DataDescriptions` stores only the raw `bone_id`. The pipeline masks off the
high 16 bits when doing the ID→name lookup.

**Bone names include skeleton prefix**
Motive names bones as `<SkeletonName>_<BoneName>` (e.g. `Alyx_Chest`).
`SKELETON_BONES` matches on the suffix only, so `"Chest"` works for any skeleton.

**`tracking_valid` not set for skeleton bones**
Motive only sets this flag for standalone rigid bodies. The pipeline skips the
check for skeleton bones.

**`DataDescriptions.py` crash**
A `NameError: name 'file'` in `get_as_string()` crashes the command thread's
pretty-printer. This does not affect data flow — `_build_id_map` runs before the crash.

