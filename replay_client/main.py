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
from queue import Queue

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from replay_client.csv_reader import CSVReader
from mocap_client.osc_processor import OSCProcessor
from pathlib import Path


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
        "--no-xyz",
        action="store_true",
        help="Disable XYZ position streaming effect"
    )
    
    return parser.parse_args()


def main():
    """Main playback loop."""
    args = parse_args()
    
    # Verify CSV file exists
    csv_path = Path(args.csv_file)
    if not csv_path.exists():
        print(f"Error: CSV file not found: {csv_path}", file=sys.stderr)
        sys.exit(1)
    
    print(f"\n[Replay Client]")
    print(f"  CSV: {csv_path.resolve()}")
    print(f"  Target: {args.target_name or 'all rigid bodies'}")
    print(f"  Speed: {args.speed}x")
    print(f"  Isadora: {args.isadora_ip}:{args.isadora_port}")
    print(f"  XYZ streaming: {'disabled' if args.no_xyz else 'enabled'}")
    print()
    
    # Create shared output queue
    out_queue: Queue = Queue(maxsize=100)
    
    # Create CSV reader
    reader = CSVReader(
        csv_path=str(csv_path),
        out_queue=out_queue,
        target_name=args.target_name,
        skeleton_bones=args.skeleton_bones if args.skeleton_bones else None,
        playback_speed=args.speed,
    )
    
    # Create OSC processor (same as live pipeline)
    osc_proc = OSCProcessor(
        out_queue=out_queue,
        isadora_ip=args.isadora_ip,
        isadora_port=args.isadora_port,
        enable_xyz=not args.no_xyz,
    )
    
    try:
        # Start both reader and processor
        reader.start()
        osc_proc.start()
        
        print("Starting playback... (Press Ctrl+C to stop)")
        print()
        
        # Keep main thread alive
        while reader.running:
            time.sleep(0.1)
        
        print("\nPlayback completed")
    
    except KeyboardInterrupt:
        print("\n\nStopping playback...")
    
    finally:
        reader.stop()
        osc_proc.stop()
        print("Stopped")


if __name__ == "__main__":
    main()
