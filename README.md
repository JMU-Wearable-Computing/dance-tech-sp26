# dance-tech-sp26
Course repo for ENGR 490 / DANC 303 — Spring 2026

---

## mocap_client

Real-time motion-capture pipeline that connects to an OptiTrack Motive server via NatNet, filters tracked bodies by name, and streams orientation data as quaternions to a consumer thread (`OSC_Processor`).

### Architecture

```
Motive (NatNet UDP)
        │
        ▼
  NatNetClient          ← low-level NatNet protocol (from polymidi)
  (NatNetClient.py)       runs two internal threads: data + command
        │
        ├─ command_q ─► DataDescriptions
        │                  builds id→name map on first receipt
        │
        └─ data_q ────► MoCapData frames (120 fps)
                           │
                           ▼
                       NatClient              (nat_client.py)
                       _frame_loop thread
                         1. drain command_q → _build_id_map()
                         2. drain data_q   → rigid bodies + skeleton bones
                         3. look up name from id_to_name map
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

