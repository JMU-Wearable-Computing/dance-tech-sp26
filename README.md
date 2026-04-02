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
| `mocap_client/osc_processor.py` | Daemon thread consuming payloads and sending OSC at 30 Hz. |
| `mocap_client/main.py` | Entry point and configuration. |
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
                         4. if name matches target_name → enqueue payload
                           │
                           ▼
                       out_queue (thread-safe Queue)
                           │
                           ▼
                       OSCProcessor           (osc_processor.py)
                       consumer daemon thread
                         - dequeues payload
                         - _process_item() ← put OSC send logic here
```

**Payload schema** pushed onto the queue:

```python
{
    "segment":   str,            # rigid-body / bone name from Motive
    "quat":      (qx, qy, qz, qw),  # orientation quaternion (floats)
    "frame":     int,            # Motive frame number
    "timestamp": float,          # Motive timestamp in seconds
}
```

### Components

| File | Role |
|---|---|
| `NatNetClient.py` | Low-level NatNet protocol (UDP socket management, packet decode). Sourced from the polymidi project. |
| `MoCapData.py` | Data-model classes for a single MoCap frame (rigid bodies, skeletons, markers). Sourced from polymidi. |
| `DataDescriptions.py` | Data-model classes for the Motive model definition (asset names, bone hierarchy). Sourced from polymidi. |
| `nat_client.py` | **NatClient** — wraps NatNetClient, builds the id→name map from DataDescriptions, filters frames by `target_name`, and pushes quaternion payloads onto `out_queue`. |
| `osc_processor.py` | **OSCProcessor** — daemon thread that consumes payloads from the queue. Currently prints to console; OSC send logic goes in `_process_item()`. |
| `main.py` | Entry point. Configure `SERVER_IP`, `LOCAL_IP`, `USE_MULTICAST`, and `TARGET_NAME` here. |

### Setup

```powershell
# From the repo root
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r mocap_client/requirements.txt
```

### Running

1. Open Motive and ensure **Data Streaming** is enabled (Edit → Project → Streaming).
2. Note the name of the rigid body or skeleton bone you want to track (e.g. `Box`).
3. Edit `mocap_client/main.py`:
   ```python
   SERVER_IP   = "127.0.0.1"   # Motive machine IP (loopback if same machine)
   TARGET_NAME = "Box"          # exact name in Motive; None = forward all
   ```
4. Run:
   ```powershell
   python -m mocap_client.main
   ```

### Extending OSC_Processor

`OSCProcessor._process_item(item)` in `osc_processor.py` is the integration point. Replace the `print` statement with your OSC send logic:

```python
from pythonosc.udp_client import SimpleUDPClient

client = SimpleUDPClient("192.168.1.x", 9000)

def _process_item(self, item: dict):
    qx, qy, qz, qw = item["quat"]
    client.send_message(f"/mocap/{item['segment']}/quat", [qx, qy, qz, qw])
```

---

## sketchbook

Arduino sketches for wearable sensor nodes used in the course.

| Sketch | Description |
|---|---|
| `AccelLED_Magic` | ADXL313 accelerometer driving LED output |
| `DistenceBuzzer_Magic` | Ultrasonic distance sensor with buzzer feedback |
| `Mario_Magic` | Mario theme buzzer sketch |
| `PotServo_Magic` | Potentiometer-controlled servo |
| `The_Magic` | Combined sensor demo |

