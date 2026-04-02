# mocap_client

Real-time motion-capture pipeline: receives NatNet data from Motive, filters
rigid bodies and skeleton bones, and streams XYZ position via OSC to Isadora.

## Running

```powershell
# From the repo root
.\start_mocap.bat
```

## Configuration (`main.py`)

| Variable | Default | Description |
|---|---|---|
| `SERVER_IP` | `127.0.0.1` | IP of the Motive machine |
| `LOCAL_IP` | `127.0.0.1` | IP of this machine's network interface |
| `USE_MULTICAST` | `True` | Match Motive's streaming mode |
| `TARGET_NAME` | `None` | Rigid body filter — `None` = all rigid bodies, `"Box"` = one |
| `SKELETON_BONES` | `["Chest"]` | Bone suffix whitelist — matched against the part after `_` in the bone name |
| `ISADORA_IP` | `127.0.0.1` | Isadora machine IP |
| `ISADORA_PORT` | `1234` | Isadora OSC listen port |

## OSC output format

Position is sent as **three separate float messages** at up to 30 Hz:

```
/<name>x   float   # e.g. /boxx, /alyxchestx
/<name>y   float
/<name>z   float
```

- Rigid bodies use the Motive asset name lowercased (e.g. `Box` → `/boxx`)
- Skeleton bones use the full bone name lowercased (e.g. `Alyx_Chest` → `/alyxchestx`)

## What gets streamed

- **Rigid bodies**: all tracked rigid bodies pass through (filter with `TARGET_NAME`)
- **Skeleton bones**: only bones whose suffix matches `SKELETON_BONES` are sent
  (e.g. `["Chest"]` matches `Alyx_Chest`, `Bob_Chest`, etc.)
- Skeleton bones that also appear in the top-level rigid body stream are suppressed
  to avoid duplicates

## Live asset detection

The ID→name map is refreshed from Motive every 5 seconds while running.
New rigid bodies or skeletons added in Motive mid-session are picked up
automatically within 5 seconds — no restart needed.

## Debugging — known gotchas

### Skeleton bone IDs in frame data
NatNet encodes skeleton bone IDs in *frame* packets as `(skeleton_id << 16) | bone_id`,
but `DataDescriptions` stores only the raw `bone_id`. The pipeline masks off the
high 16 bits when doing the ID→name lookup.

### Bone names are prefixed with skeleton name
Motive names bones as `<SkeletonName>_<BoneName>` (e.g. `Alyx_Chest`).
`SKELETON_BONES` matches on the suffix only, so `"Chest"` works for any skeleton.

### tracking_valid flag
Motive does not set `tracking_valid` on solved skeleton bones — only on
standalone rigid bodies. The pipeline skips that check for bones.

### DataDescriptions crash
There is a bug in the vendored `DataDescriptions.py` (`NameError: name 'file'`
in `get_as_string`) that crashes the command thread when pretty-printing.
This only affects the string representation; the pipeline still works because
`_build_id_map` runs before the crash.

## Verifying output

Run the listener to see all OSC messages arriving on port 1234:

```powershell
python osc_listener.py
```

