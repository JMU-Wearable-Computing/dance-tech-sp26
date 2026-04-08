"""
Motion Capture CSV Playback Entry Point

Replays pre-recorded motion capture from CSV files through the same
middleware pipeline as the live system.

Usage:
  python -m replay_client.main <csv_file> [options]

Example:
  python -m replay_client.main recordings/my_dance.csv \\
    --target-name dancer1 \\
    --speed 1.5 \\
    --isadora-ip 192.168.1.100
"""
import argparse
import sys
import time
import logging
from pathlib import Path
from queue import Queue

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from replay_client.csv_reader import CSVReader
from mocap_client.osc_processor import OSCProcessor


def parse_args():
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="Replay recorded motion capture from CSV"
    )
    
    parser.add_argument(
        "csv_file",
        help="Path to Motive CSV export file"
    )
    
    parser.add_argument(
        "--target-name",
        default=None,
        help="Filter to single rigid body (e.g., 'dancer1'). Default: all"
    )
    
    parser.add_argument(
        "--skeleton-bones",
        nargs="+",
        default=["Chest"],
        help="Skeleton bone suffix whitelist. Default: ['Chest']"
    )
    
    parser.add_argument(
        "--speed",
        type=float,
        default=1.0,
        help="Playback speed multiplier (1.0=normal, 2.0=2x faster). Default: 1.0"
    )
    
    parser.add_argument(
        "--isadora-ip",
        default="127.0.0.1",
        help="IP address of Isadora machine. Default: 127.0.0.1"
    )
    
    parser.add_argument(
        "--isadora-port",
        type=int,
        default=1234,
        help="OSC port on Isadora. Default: 1234"
    )

    parser.add_argument(
        "--csv-position-units",
        choices=["mm", "m"],
        default="mm",
        help="Units used by CSV position columns. 'mm' converts to meters before OSC. Default: mm"
    )
    
    return parser.parse_args()


def main():
    """Main playback loop."""
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(name)s %(levelname)s %(message)s"
    )
    log = logging.getLogger("replay_client")
    
    args = parse_args()
    
    # Verify CSV file exists
    csv_path = Path(args.csv_file)
    if not csv_path.exists():
        log.error(f"CSV file not found: {csv_path}")
        sys.exit(1)
    
    log.info(f"CSV: {csv_path.resolve()}")
    log.info(f"Target: {args.target_name or 'all rigid bodies'}")
    log.info(f"Speed: {args.speed}x")
    log.info(f"Isadora: {args.isadora_ip}:{args.isadora_port}")

    position_scale = 0.001 if args.csv_position_units == "mm" else 1.0
    log.info(
        "CSV position units: %s (applied position scale=%s)",
        args.csv_position_units,
        position_scale,
    )
    
    # Create shared queue
    q = Queue()
    
    # Create OSC processor
    osc = OSCProcessor(
        in_queue=q,
        isadora_ip=args.isadora_ip,
        isadora_port=args.isadora_port,
    )
    osc.start()
    log.info(f"OSC_Processor started, Isadora at {args.isadora_ip}:{args.isadora_port}")
    
    # Create CSV reader (ingest data from CSV instead of NatNet)
    csv = CSVReader(
        csv_path=str(csv_path),
        out_queue=q,
        target_name=args.target_name,
        skeleton_bones=args.skeleton_bones if args.skeleton_bones else None,
        playback_speed=args.speed,
        position_scale=position_scale,
    )
    
    if not csv.start():
        log.error("Failed to start CSV reader.  Exiting.")
        osc.stop()
        return
    
    log.info("CSV playback started — press Ctrl+C to stop.")
    try:
        # Wait for reader to finish playback
        while csv.running:
            time.sleep(0.1)
        
        # Allow queue to drain before exiting
        log.info("Draining queue...")
        q.join()
        log.info("Playback completed.")
    
    except KeyboardInterrupt:
        log.info("Shutting down.")
    
    finally:
        csv.stop()
        osc.stop()
        osc.join(timeout=2.0)


if __name__ == "__main__":
    main()
