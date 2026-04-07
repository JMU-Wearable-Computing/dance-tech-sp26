# dance-tech-sp26 Copilot Instructions

## Project Goal
This repository supports ENGR 490 / DANC 303 course work. The main software project is `mocap_client`, which receives motion-capture data from an OptiTrack Motive server over NatNet, filters tracked bodies by name, and forwards selected motion data to an OSC consumer for Isadora. The `sketchbook/` folder contains Arduino sketches for separate wearable and sensor demos and is not part of the mocap runtime pipeline.

## Pipeline Overview
The mocap pipeline is:

Motive -> NatNet UDP -> `mocap_client/NatNetClient.py` -> `mocap_client/nat_client.py` -> thread-safe `Queue` -> `mocap_client/osc_processor.py` -> OSC output to Isadora.

Data flow in detail:

1. Motive streams NatNet packets containing model definitions and live frame data.
2. `NatNetClient.py` decodes those packets into Python objects from `DataDescriptions.py` and `MoCapData.py`.
3. `nat_client.py` builds an `id -> name` map from `DataDescriptions` packets.
4. Skeleton bone IDs are tracked separately so skeleton bones can be handled differently from top-level rigid bodies.
5. Each MoCap frame provides a frame number, timestamp, rigid-body data, and skeleton data.
6. `NatClient` filters the frame data:
   - `target_name` filters rigid bodies by exact Motive name.
   - `skeleton_bones` filters skeleton bones by suffix match, such as `Chest` matching `Alyx_Chest`.
   - Untracked rigid bodies are skipped.
7. Matching items are pushed into the output queue as dicts with this shape:

```python
{
	"segment": "Box",
	"quat": (qx, qy, qz, qw),
	"pos": (px, py, pz),
	"frame": 1234,
	"timestamp": 12.345,
}
```

8. `OSCProcessor` consumes the queue on a background thread, rate-limits per segment, and sends OSC messages to Isadora.
9. Current OSC output is position-based: it sends `/segmentx`, `/segmenty`, and `/segmentz` messages. The quaternion is validated and preserved in the payload, but it is not currently emitted by the processor.

## Important Reminders
Keep edits minimal and focused on the requested change. Preserve existing formatting, naming, and public interfaces unless a change is explicitly required.

Treat the NatNet-derived files (`NatNetClient.py`, `DataDescriptions.py`, `MoCapData.py`) as vendored third-party code. Avoid rewriting them unless the task depends on it.

Do not hard-code machine-specific network settings, secrets, or production IPs into source files. Keep values such as `SERVER_IP`, `LOCAL_IP`, `USE_MULTICAST`, `TARGET_NAME`, and `ISADORA_IP` as configuration.

Be careful with the pipeline contract:

- Preserve the ID-to-name mapping logic.
- Preserve the distinction between rigid bodies and skeleton bones.
- Keep queue payload keys stable unless the whole pipeline is being updated together.
- If output behavior changes, update the README and any related docs.

Prefer clear logging and avoid noisy debug prints unless they help diagnose a real issue. Use ASCII unless the file already requires something else.

Update the README after every change that affects setup, usage, or architecture. The README should reflect the current state of the codebase and provide accurate instructions for running the mocap pipeline.

## Commands
Use these commands for the current project flow:

```bash
# Run the mocap pipeline from the repo root
bash start_mocap.sh

# Or run the entry point directly
python -m mocap_client.main

# Create and activate a virtual environment
python -m venv .venv
source .venv/bin/activate

# Install Python dependencies
pip install -r mocap_client/requirements.txt
```

Future commands should be added here when new tooling, tests, or automation scripts are introduced.
