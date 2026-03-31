"""Entry point for the mocap_client pipeline.

Connects to a Motive NatNet server, filters incoming rigid-body frames
by the configured segment names, and feeds quaternion payloads into the
OSC_Processor consumer thread.

Usage (from the repo root):
    python -m mocap_client.main

Edit the constants below to match your network configuration and the
body segments you want to track.
"""
import time
import logging
from queue import Queue

from mocap_client.nat_client import NatClient
from mocap_client.osc_processor import OSCProcessor

# --- Configuration -------------------------------------------------------
# Motive NatNet server config
SERVER_IP      = "127.0.0.1"
LOCAL_IP       = "127.0.0.1"
USE_MULTICAST  = True

# Name must match exactly the rigid-body or bone name in Motive.
# Set to None to receive all tracked entities.
TARGET_NAME = "Box"

# Isadora OSC receiver config
ISADORA_IP = "127.0.0.1"
ISADORA_PORT = 1234
# -------------------------------------------------------------------------


def main():
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")
    log = logging.getLogger("mocap_client")

    q = Queue()

    osc = OSCProcessor(q, isadora_ip=ISADORA_IP, isadora_port=ISADORA_PORT)
    osc.start()
    log.info(f"OSC_Processor started, Isadora at {ISADORA_IP}:{ISADORA_PORT}")

    nat = NatClient(
        out_queue=q,
        target_name=TARGET_NAME,
        server_ip=SERVER_IP,
        local_ip=LOCAL_IP,
        use_multicast=USE_MULTICAST,
    )

    if not nat.start():
        log.error("Failed to connect to Motive.  Exiting.")
        osc.stop()
        return

    log.info("Connected to Motive.  Streaming — press Ctrl+C to stop.")
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        log.info("Shutting down.")
    finally:
        nat.stop()
        osc.stop()
        osc.join(timeout=2.0)


if __name__ == "__main__":
    main()
