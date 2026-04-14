# Multi-CSV Synchronization Driver Plan

## 1) Objective

Create a unified driver (`roses_driver.py`) that orchestrates playback of multiple Motive CSV files in a coordinated sequence: reading certain CSVs sequentially and others in parallel, maintaining frame-accurate timing and merging data into a single OSC output stream to Isadora.

**Deliverable:** A deployable `roses_driver.py` that demonstrates reading one CSV completely, then reading two CSVs simultaneously, with synchronized playback and unified OSC output.

---

## 2) Assumptions

- The driver will extend/reuse existing `CSVReader` and `OSCProcessor` classes from `replay_client/`
- All input CSVs follow the standard Motive export format (metadata rows, type/name headers, frame-based data)
- The unified timer is a **master clock** that triggers CSVs. The CSVs should play themselves independently once triggered
- Output merges all CSV segments into a single OSC stream (no separate streams per CSV)
- The driver is configured via a simple configuration object
- Playback rate is uniform across all CSVs (not per-CSV speed control)
- Frame synchronization is based on timestamps, not frame numbers (to handle different frame rates)

---

## 3) Plan

### Phase 1: Design & Architecture

**Step 1.1: Define the execution model**
- **Action:** Decide on the exact sequence:
  - Example: Read `roses_take1.csv` → then read `roses_take2.csv` AND `dancers_backup.csv` in parallel
  - Document which CSVs are "sequence phases" and which are "parallel groups"
- **Files/Areas Affected:** `roses_driver.py` (structure)
- **Why:** Ensures clear timing logic and prevents confusion during implementation

**Step 1.2: Define the unified timer interface**
- **Action:** Design how the master clock works:
  - Single reference point (`start_time`) when driver starts
  - Each CSV reader adjusted to the master clock (not independent)
  - Enum or config to mark each CSV as "sequential" vs "parallel"
- **Files/Areas Affected:** `roses_driver.py` (main loop & timing logic)
- **Why:** Prevents out-of-sync playback and simplifies OSC merging

### Phase 2: Implementation

**Step 2.1: Create a configuration object for multi-CSV playback**
- **Action:** Define a simple data structure (dataclass or dict-based) that specifies:
  - List of CSVs with their paths
  - Phase assignment (phase 0, 1, 2, etc.)
  - Optional playback rate, target segments, bone filters per CSV
- **Files/Areas Affected:** 
  - `roses_driver.py` (new `CSVPlaylistConfig` or similar class)
- **Why:** Makes it easy to test different CSV combinations without code changes

**Step 2.2: Implement a timing coordinator**
- **Action:** Create a coordinator class that:
  - Tracks global start time
  - Manages phase transitions (phase 0 complete → start phase 1)
- **Files/Areas Affected:** 
  - `roses_driver.py` (new `Timer` class)
- **Why:** Centralizes timing logic; easy to debug and adjust playback

**Step 2.3: Implement phase-based playback**
- **Action:** Create execution loop that:
  - Starts CSV readers for phase 0 in sequence (one-at-a-time)
  - Waits for phase 0 completion
  - Starts CSV readers for phase 1 in parallel (two-at-a-time)
  - Collects data from all active readers and merges into single OSC output
- **Files/Areas Affected:** 
  - `roses_driver.py` (main `main()` loop)
  - Extend or wrap existing `CSVReader` to accept a timing coordinator
- **Why:** Demonstrates the core feature: sequential then parallel playback

**Step 2.4: Merge OSC data from multiple readers**
- **Action:** Ensure the output queue handler (OSCProcessor) correctly:
  - Receives payloads from multiple CSV readers
  - Does not duplicate or drop frames
  - Sends all segment data in a single OSC batch per frame
- **Files/Areas Affected:** 
  - `roses_driver.py` (shared queue management)
  - Possibly `mocap_client/osc_processor.py` (if changes needed, minimal)
- **Why:** Guarantees no data loss and consistent timing in Isadora

### Phase 3: Testing & Validation

**Step 3.1: Test phase transitions**
- **Action:** Create a small test config with 2–3 dummy/small CSVs; verify phase timing
- **Files/Areas Affected:** Test config file
- **Why:** Ensures phases start/stop at the correct times

**Step 3.2: Stress test parallel reading**
- **Action:** Run two larger CSVs in parallel; monitor queue depth and timing
- **Files/Areas Affected:** Logging in `roses_driver.py`
- **Why:** Confirms no timing skew or dropped frames under load

---

## 4) Verification

| Check | How | Expected Outcome |
|-------|-----|------------------|
| **Phase 0 completes** | Log output shows "Phase 0 complete" | All phase 0 CSVs finish playback before phase 1 starts |
| **Phase 1 starts in parallel** | Log shows both CSV readers writing to queue simultaneously | Both CSVs stream data to OSC at the same time |
| **No frame drops** | Monitor queue length and OSC packet count | Queue stays < threshold; all data reaches Isadora |
| **Timing accuracy** | Compare recorded vs. expected frame intervals | Frame timing matches CSV timestamps (within 1–2ms) |
| **OSC output merges** | Inspect OSC traffic to Isadora | All segments (phase 0 + phase 1) appear in a single stream |

**Manual checks:**
```bash
# Run the driver with test CSVs
python -m replay_client.roses_driver --config roses-performance.json --isadora-ip 127.0.0.1

# In another terminal, listen for OSC
python osc_listener.py
```

---

## 5) Risks & Mitigations

| Risk | Impact | Mitigation |
|------|--------|-----------|
| **Queue overflow** if two CSVs produce data faster than OSC can send | Frames dropped; Isadora loses sync | Monitor queue; add backpressure or buffering; test with target playback speed first |
| **CSV files have different frame rates** | Hard to synchronize if using frame numbers instead of timestamps | Use timestamp-based synchronization (already in CSVReader); allow per-CSV frame-rate config |
| **Phase transition timing edge cases** | Playback stutters or skips | Test heavily at phase boundaries; add logging at transitions |
| **Configuration errors** (wrong CSV path, invalid phase) | Driver crashes silently | Validate configuration at startup; clear error messages |

---

## 6) Clarifying Questions

Before implementation begins, please clarify:

1. **Sequencing:** What is the exact sequence of CSVs and phases?
   - *Example:* "Phase 0: roses-take1.csv → Phase 1: roses-take2.csv + dancers-backup.csv"
   - Or is it flexible/configurable per performance?

   This is exact performance scheduling.

2. **Configuration method:** How should the driver know which CSVs to load and when?
   - Command-line args? (e.g., `--phase-0 file1.csv --phase-1 file2.csv file3.csv`)
   - JSON/YAML config file?
   - Hardcoded for now?

   The JSON file.

3. **Unified timer:** Should playback be:
   - All CSVs reference a **single global clock** (master time)?
   - Each CSV **offset from master** (e.g., CSV B starts 5 seconds after CSV A)?

   Start time should be dictated by global clock.

4. **Output naming:** When merging OSVs, should segment names be:
   - Preserved as-is (e.g., "Will:Chest", "Dancer2:Head")?
   - Prefixed by CSV filename for disambiguation?

   Yes preserve

---

## 7) Implementation Order

1. **Clarify questions above** → finalize requirements
2. **Step 1.1:** Define execution sequence
3. **Step 1.2:** Design unified timer
4. **Step 2.1:** Implement config structure
5. **Step 2.2:** Implement timing coordinator
6. **Step 2.3:** Implement phase loop with CSV readers
7. **Step 2.4:** Verify OSC merging
8. **Step 3.x:** Test and validate

---

## Notes

- Leverage existing `CSVReader` from `replay_client/` to avoid duplication
- Reuse `OSCProcessor` for output handling
- Keep `roses_driver.py` focused on orchestration; avoid mixing timing logic into CSV readers
- Consider making the phase-based approach reusable for future multi-CSV performances
