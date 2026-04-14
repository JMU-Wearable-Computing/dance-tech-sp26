"""
Multi-CSV Synchronization Driver for Unified Performance Playback

Orchestrates playback of multiple Motive CSV files with phases:
- Phase 0: Read CSVs sequentially (one at a time)
- Phase 1: Read multiple CSVs in parallel (simultaneously)

Maintains frame-accurate timing with a unified master clock and merges all
data into a single OSC output stream to Isadora.

Usage:
    python -m replay_client.roses_driver --config roses-performance.json \\
        --isadora-ip 127.0.0.1 \\
        --isadora-port 1234 \\
        --speed 1.0

Example config (roses-performance.json):
    {
        "phase_0": [
            {"path": "roses_take1.csv", "skeleton_bones": ["Chest"]},
            {"path": "roses_take2.csv", "skeleton_bones": ["Chest"]}
        ],
        "phase_1": [
            {"path": "dancers_backup.csv", "skeleton_bones": ["Chest"]},
            {"path": "roses_take3.csv", "skeleton_bones": ["Chest"]}
        ]
    }
"""

import sys
import json
import time
import logging
import argparse
import threading
from pathlib import Path
from dataclasses import dataclass, field
from typing import Dict, List, Optional
from queue import Queue

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from replay_client.csv_reader import CSVReader
from mocap_client.osc_processor import OSCProcessor


log = logging.getLogger("roses_driver")


@dataclass
class CSVConfig:
    """Configuration for a single CSV file in the performance."""
    path: str
    start_time: float = 0.0  # Time (seconds) when this CSV should start relative to master clock
    skeleton_bones: Optional[List[str]] = field(default_factory=lambda: ["Chest"])
    target_name: Optional[str] = None
    position_scale: float = 0.001  # Convert mm to m


@dataclass
class PerformanceConfig:
    """Configuration for multi-phase CSV playback performance."""
    phase_0: List[CSVConfig] = field(default_factory=list)
    phase_1: List[CSVConfig] = field(default_factory=list)
    playback_speed: float = 1.0
    
    @classmethod
    def from_json(cls, json_path: str) -> "PerformanceConfig":
        """Load performance config from JSON file."""
        with open(json_path, 'r') as f:
            data = json.load(f)
        
        phase_0 = [
            CSVConfig(
                path=item['path'],
                start_time=item.get('start_time', 0.0),
                skeleton_bones=item.get('skeleton_bones', ["Chest"]),
                target_name=item.get('target_name'),
                position_scale=item.get('position_scale', 0.001),
            )
            for item in data.get('phase_0', [])
        ]
        
        phase_1 = [
            CSVConfig(
                path=item['path'],
                start_time=item.get('start_time', 0.0),
                skeleton_bones=item.get('skeleton_bones', ["Chest"]),
                target_name=item.get('target_name'),
                position_scale=item.get('position_scale', 0.001),
            )
            for item in data.get('phase_1', [])
        ]
        
        return cls(
            phase_0=phase_0,
            phase_1=phase_1,
            playback_speed=data.get('playback_speed', 1.0),
        )


class TimingCoordinator:
    """
    Manages unified master clock for multi-CSV playback.
    
    The coordinator:
    - Tracks global start time when playback begins
    - Manages phase transitions
    - Signals when phases are complete
    """
    
    def __init__(self):
        """Initialize the timing coordinator."""
        self.start_time: Optional[float] = None
        self.phase_0_complete = threading.Event()
        self.phase_1_complete = threading.Event()
        self.all_complete = threading.Event()
        self._lock = threading.Lock()
    
    def start(self):
        """Mark the start of playback (establishes master clock reference)."""
        with self._lock:
            if self.start_time is None:
                self.start_time = time.time()
                log.info(f"Master clock started at {self.start_time:.3f}")
    
    def signal_phase_0_complete(self):
        """Signal that phase 0 playback is complete."""
        with self._lock:
            self.phase_0_complete.set()
            log.info("Phase 0 complete")
    
    def signal_phase_1_complete(self):
        """Signal that phase 1 playback is complete."""
        with self._lock:
            self.phase_1_complete.set()
            log.info("Phase 1 complete")
    
    def signal_all_complete(self):
        """Signal that all phases are complete."""
        with self._lock:
            self.all_complete.set()
            log.info("All phases complete")
    
    def wait_phase_0_complete(self, timeout: Optional[float] = None) -> bool:
        """Wait for phase 0 to complete. Returns True if signaled, False if timeout."""
        return self.phase_0_complete.wait(timeout=timeout)
    
    def wait_phase_1_complete(self, timeout: Optional[float] = None) -> bool:
        """Wait for phase 1 to complete. Returns True if signaled, False if timeout."""
        return self.phase_1_complete.wait(timeout=timeout)
    
    def wait_all_complete(self, timeout: Optional[float] = None) -> bool:
        """Wait for all phases to complete. Returns True if signaled, False if timeout."""
        return self.all_complete.wait(timeout=timeout)
    
    def elapsed_time(self) -> float:
        """Return elapsed time since master clock start (in seconds)."""
        if self.start_time is None:
            return 0.0
        return time.time() - self.start_time


class CSVPlaylistManager:
    """
    Orchestrates phase-based playback of multiple CSV files.
    
    Manages:
    - Starting CSV readers for each phase
    - Synchronizing phase transitions
    - Monitoring reader health
    """
    
    def __init__(
        self,
        out_queue: Queue,
        timing_coordinator: TimingCoordinator,
        playback_speed: float = 1.0,
    ):
        """
        Initialize the playlist manager.
        
        Args:
            out_queue: Shared output queue for all CSV readers
            timing_coordinator: Master clock coordinator
            playback_speed: Global playback speed multiplier
        """
        self.out_queue = out_queue
        self.timing_coordinator = timing_coordinator
        self.playback_speed = playback_speed
        self.active_readers: List[CSVReader] = []
        self._lock = threading.Lock()
    
    def start_csv_sequentially(self, csv_configs: List[CSVConfig]) -> bool:
        """
        Start CSV readers sequentially (one at a time, wait for completion).
        Respects each CSV's start_time relative to the master clock.
        
        Args:
            csv_configs: List of CSV configurations to play in sequence
        
        Returns:
            True if all CSVs completed successfully, False if error/timeout
        """
        log.info(f"Starting {len(csv_configs)} CSV(s) sequentially (Phase 0)")
        
        for idx, cfg in enumerate(csv_configs):
            # Wait until it's time to start this CSV
            elapsed = self.timing_coordinator.elapsed_time()
            if elapsed < cfg.start_time:
                wait_time = cfg.start_time - elapsed
                log.info(f"  [{idx+1}/{len(csv_configs)}] Waiting {wait_time:.2f}s before starting: {cfg.path}")
                time.sleep(wait_time)
            
            log.info(f"  [{idx+1}/{len(csv_configs)}] Starting at {cfg.start_time}s: {cfg.path}")
            
            reader = CSVReader(
                csv_path=cfg.path,
                out_queue=self.out_queue,
                target_name=cfg.target_name,
                skeleton_bones=cfg.skeleton_bones,
                playback_speed=self.playback_speed,
                position_scale=cfg.position_scale,
            )
            
            with self._lock:
                self.active_readers.append(reader)
            
            reader.start()
            
            # Wait for this reader to complete before starting next
            # (join with a reasonable timeout based on typical performance length)
            timeout = 600.0  # 10 minutes max per CSV
            if reader._thread:
                reader._thread.join(timeout=timeout)
                if reader._thread.is_alive():
                    log.error(f"CSV reader timeout (>{timeout}s): {cfg.path}")
                    return False
            
            with self._lock:
                self.active_readers.remove(reader)
            
            log.info(f"  [{idx+1}/{len(csv_configs)}] Complete: {cfg.path}")
        
        return True
    
    def start_csvs_parallel(self, csv_configs: List[CSVConfig]) -> bool:
        """
        Start multiple CSV readers in parallel (simultaneously).
        Respects each CSV's start_time relative to the master clock.
        
        Args:
            csv_configs: List of CSV configurations to play in parallel
        
        Returns:
            True if all CSVs completed successfully, False if error/timeout
        """
        log.info(f"Starting {len(csv_configs)} CSV(s) in parallel (Phase 1)")
        
        # First, wait for synchronized start time if needed
        max_start_time = max((cfg.start_time for cfg in csv_configs), default=0.0)
        if max_start_time > 0:
            elapsed = self.timing_coordinator.elapsed_time()
            if elapsed < max_start_time:
                wait_time = max_start_time - elapsed
                log.info(f"  Waiting {wait_time:.2f}s before parallel start")
                time.sleep(wait_time)
        
        readers = []
        for idx, cfg in enumerate(csv_configs):
            log.info(f"  [{idx+1}/{len(csv_configs)}] Starting at {cfg.start_time}s: {cfg.path}")
            
            reader = CSVReader(
                csv_path=cfg.path,
                out_queue=self.out_queue,
                target_name=cfg.target_name,
                skeleton_bones=cfg.skeleton_bones,
                playback_speed=self.playback_speed,
                position_scale=cfg.position_scale,
            )
            
            readers.append(reader)
            reader.start()
        
        with self._lock:
            self.active_readers.extend(readers)
        
        # Wait for all readers to complete
        timeout = 600.0  # 10 minutes max
        for reader in readers:
            if reader._thread:
                reader._thread.join(timeout=timeout)
                if reader._thread.is_alive():
                    log.error(f"CSV reader timeout (>{timeout}s)")
                    return False
        
        with self._lock:
            for reader in readers:
                self.active_readers.remove(reader)
        
        log.info(f"  All {len(csv_configs)} CSV(s) completed in parallel")
        return True
    
    def stop_all(self):
        """Stop all active CSV readers."""
        with self._lock:
            stopped_count = len(self.active_readers)
            for reader in self.active_readers:
                reader.stop()
            self.active_readers.clear()
            if stopped_count > 0:
                log.info(f"Stopped {stopped_count} active reader(s)")


def parse_args():
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="Multi-CSV Synchronization Driver for unified performance playback"
    )
    
    parser.add_argument(
        "--config",
        type=str,
        required=True,
        help="Path to performance config JSON file (required)"
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
        "--speed",
        type=float,
        default=1.0,
        help="Playback speed multiplier (1.0=normal, 2.0=2x faster). Default: 1.0"
    )
    
    parser.add_argument(
        "--log-level",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        default="INFO",
        help="Logging level. Default: INFO"
    )
    
    return parser.parse_args()


def main():
    """Main entry point for roses_driver."""
    args = parse_args()
    
    # Configure logging
    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s %(name)s %(levelname)s %(message)s"
    )
    
    log.info("=" * 70)
    log.info("ROSES DRIVER - Multi-CSV Synchronization")
    log.info("=" * 70)
    
    # Load performance configuration
    config_path = Path(args.config)
    if not config_path.exists():
        log.error(f"Config file not found: {config_path}")
        sys.exit(1)
    
    try:
        config = PerformanceConfig.from_json(str(config_path))
        log.info(f"Loaded performance config from {config_path}")
        log.info(f"  Phase 0 CSVs: {len(config.phase_0)}")
        log.info(f"  Phase 1 CSVs: {len(config.phase_1)}")
        log.info(f"  Playback speed: {args.speed}x")
    except Exception as e:
        log.error(f"Failed to load config: {e}")
        sys.exit(1)
    
    # Create OSC processor (output to Isadora)
    out_queue = Queue()
    osc_processor = OSCProcessor(
        out_queue,
        isadora_ip=args.isadora_ip,
        isadora_port=args.isadora_port
    )
    osc_processor.start()
    log.info(f"OSC Processor started -> {args.isadora_ip}:{args.isadora_port}")
    
    # Create timing coordinator
    timing_coordinator = TimingCoordinator()
    
    # Create playlist manager
    playlist_manager = CSVPlaylistManager(
        out_queue=out_queue,
        timing_coordinator=timing_coordinator,
        playback_speed=args.speed if args.speed else config.playback_speed,
    )
    
    try:
        # Start master clock
        timing_coordinator.start()
        
        # Execute Phase 0: Sequential playback
        if config.phase_0:
            if not playlist_manager.start_csv_sequentially(config.phase_0):
                log.error("Phase 0 failed")
                sys.exit(1)
            timing_coordinator.signal_phase_0_complete()
        else:
            log.info("No Phase 0 CSVs configured")
            timing_coordinator.signal_phase_0_complete()
        
        # Small pause between phases
        time.sleep(0.1)
        
        # Before Phase 1 starts, kill any remaining Phase 0 readers
        if config.phase_1:
            phase_1_start_time = min((cfg.start_time for cfg in config.phase_1), default=0.0)
            if playlist_manager.active_readers:
                log.info(f"Phase 1 starting at {phase_1_start_time}s - stopping any remaining Phase 0 readers")
                playlist_manager.stop_all()
        
        # Execute Phase 1: Parallel playback
        if config.phase_1:
            if not playlist_manager.start_csvs_parallel(config.phase_1):
                log.error("Phase 1 failed")
                sys.exit(1)
            timing_coordinator.signal_phase_1_complete()
        else:
            log.info("No Phase 1 CSVs configured")
            timing_coordinator.signal_phase_1_complete()
        
        timing_coordinator.signal_all_complete()
        
        elapsed = timing_coordinator.elapsed_time()
        log.info("=" * 70)
        log.info(f"PERFORMANCE COMPLETE - Total time: {elapsed:.2f}s")
        log.info("=" * 70)
        
    except KeyboardInterrupt:
        log.info("Interrupted by user")
        playlist_manager.stop_all()
    except Exception as e:
        log.error(f"Error during playback: {e}", exc_info=True)
        playlist_manager.stop_all()
        sys.exit(1)
    finally:
        osc_processor.stop()
        osc_processor.join(timeout=2.0)
        log.info("Shutdown complete")


if __name__ == "__main__":
    main()
