"""Entry point for the csv_client pipeline.

Reads mocap data from a Motive CSV export file, filters by configured
segment names, and feeds quaternion payloads into the OSC_Processor
consumer thread at a configurable playback rate.

Usage (from the repo root):
    python -m csv_client.main

Edit the constants below to match your CSV file and the body segments
you want to track.
"""
import time
import logging
from queue import Queue
from pathlib import Path

from csv_client.csv_reader import CSVReader
from mocap_client.osc_processor import OSCProcessor

# --- Configuration -------------------------------------------------------
# Path to the CSV file (relative or absolute)
CSV_PATH = "C:/Users/walte/OneDrive/Documents/CRobotic Arm Parts/Isadora Stuff/dance-tech-sp26/theremin_Box.csv"

# Playback rate (1.0 = real-time at 120 FPS, 2.0 = 2x speed, 0.5 = half speed)
PLAYBACK_RATE = 1.0

# Name filter: if specified, only this segment is sent.
# Set to None to send all tracked segments.
TARGET_SEGMENT = None  # None = all segments, "Name" = single segment

# Skeleton bone whitelist — only these bones are forwarded.
# Check the CSV for exact bone names (e.g., "Emma:Chest", "Emma:Head").
# Set to None to forward all bones.
# Matched by bone suffix (e.g., "Chest" matches "Emma:Chest")
SKELETON_BONES = ["LThumb1", "LShoulder", "RThumb1"]  # Only LThumb1 segments

# Isadora OSC receiver config
ISADORA_IP = "172.28.98.126"
ISADORA_PORT = 1235
# -------------------------------------------------------------------------


def main():
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")
    log = logging.getLogger("csv_client")

    q = Queue()

    osc = OSCProcessor(q, isadora_ip=ISADORA_IP, isadora_port=ISADORA_PORT)
    osc.start()
    log.info(f"OSC_Processor started, Isadora at {ISADORA_IP}:{ISADORA_PORT}")

    try:
        while True:
            csv_reader = CSVReader(
                csv_path=CSV_PATH,
                out_queue=q,
                target_segment=TARGET_SEGMENT,
                skeleton_bones=SKELETON_BONES,
                playback_rate=PLAYBACK_RATE,
            )
            log.info(f"Reading CSV from {Path(CSV_PATH).resolve()}")
            csv_reader.read_and_stream()
            log.info("Playback complete, looping...")
    except KeyboardInterrupt:
        log.info("Interrupted by user.")
    except Exception as e:
        log.error(f"Error during playback: {e}", exc_info=True)
    finally:
        osc.stop()
        osc.join(timeout=2.0)
        log.info("Shutdown complete.")


if __name__ == "__main__":
    main()
