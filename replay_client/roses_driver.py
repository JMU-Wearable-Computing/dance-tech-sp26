"""
Multi-CSV Synchronization Driver for Unified Performance Playback

Orchestrates playback of multiple Motive CSV files with scenes:
- Scene 1: Read CSVs sequentially (one at a time)
- Scene 2: Read multiple CSVs in parallel (simultaneously)

Maintains frame-accurate timing with a unified master clock and merges all
data into a single OSC output stream to Isadora.

Usage:
    python -m replay_client.roses_driver --config roses-performance.json \\
        --isadora-ip 127.0.0.1 \\
        --isadora-port 1234 \\
        --speed 1.0

Example config (roses-performance.json):
    {
        "scene_1": {
            "start_time": 0.0,
            "csvs": [
                {"path": "roses_take1.csv", "start_time": 0.0, "skeleton_bones": ["Chest"]},
                {"path": "roses_take2.csv", "start_time": 2.5, "skeleton_bones": ["Chest"]}
            ]
        },
        "scene_2": {
            "start_time": 30.0,
            "csvs": [
                {"path": "dancers_backup.csv", "start_time": 0.0, "skeleton_bones": ["Chest"]},
                {"path": "roses_take3.csv", "start_time": 1.0, "skeleton_bones": ["Chest"]}
            ]
        }
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
    start_time: float = 0.0  # Time (seconds) when this CSV should start relative to scene start
    skeleton_bones: Optional[List[str]] = field(default_factory=lambda: ["Chest"])
    target_name: Optional[str] = None
    position_scale: float = 0.001  # Convert mm to m


@dataclass
class SceneConfig:
    """Configuration for a scene containing one or more CSV files."""
    start_time: float = 0.0  # Time (seconds) when this scene starts relative to master clock
    csvs: List[CSVConfig] = field(default_factory=list)


@dataclass
class PerformanceConfig:
    """Configuration for multi-scene CSV playback performance."""
    scene_1: SceneConfig = field(default_factory=SceneConfig)
    scene_2: SceneConfig = field(default_factory=SceneConfig)
    playback_speed: float = 1.0
    
    @classmethod
    def from_json(cls, json_path: str) -> "PerformanceConfig":
        """Load performance config from JSON file."""
        with open(json_path, 'r') as f:
            data = json.load(f)
        
        def _parse_csv_list(csv_data: List[dict]) -> List[CSVConfig]:
            return [
                CSVConfig(
                    path=item['path'],
                    start_time=item.get('start_time', 0.0),
                    skeleton_bones=item.get('skeleton_bones', ["Chest"]),
                    target_name=item.get('target_name'),
                    position_scale=item.get('position_scale', 0.001),
                )
                for item in csv_data
            ]

        def _parse_scene(scene_key: str) -> SceneConfig:
            scene_data = data.get(scene_key, {})

            # Backward compatibility: if scene key maps directly to a list, treat it as csvs.
            if isinstance(scene_data, list):
                return SceneConfig(start_time=0.0, csvs=_parse_csv_list(scene_data))

            if not isinstance(scene_data, dict):
                return SceneConfig()

            return SceneConfig(
                start_time=scene_data.get('start_time', 0.0),
                csvs=_parse_csv_list(scene_data.get('csvs', [])),
            )
        
        return cls(
            scene_1=_parse_scene('scene_1'),
            scene_2=_parse_scene('scene_2'),
            playback_speed=data.get('playback_speed', 1.0),
        )


class TimingCoordinator:
    """
    Manages unified master clock for multi-CSV playback.
    
    The coordinator:
    - Tracks global start time when playback begins
    - Manages scene transitions
    - Signals when scenes are complete
    """
    
    def __init__(self):
        """Initialize the timing coordinator."""
        self.start_time: Optional[float] = None
        self.scene_1_complete = threading.Event()
        self.scene_2_complete = threading.Event()
        self.all_complete = threading.Event()
        self._lock = threading.Lock()
    
    def start(self):
        """Mark the start of playback (establishes master clock reference)."""
        with self._lock:
            if self.start_time is None:
                self.start_time = time.time()
                log.info(f"Master clock started at {self.start_time:.3f}")
    
    def signal_scene_1_complete(self):
        """Signal that scene 1 playback is complete."""
        with self._lock:
            self.scene_1_complete.set()
            log.info("Scene 1 complete")
    
    def signal_scene_2_complete(self):
        """Signal that scene 2 playback is complete."""
        with self._lock:
            self.scene_2_complete.set()
            log.info("Scene 2 complete")
    
    def signal_all_complete(self):
        """Signal that all phases are complete."""
        with self._lock:
            self.all_complete.set()
            log.info("All phases complete")
    
    def wait_scene_1_complete(self, timeout: Optional[float] = None) -> bool:
        """Wait for scene 1 to complete. Returns True if signaled, False if timeout."""
        return self.scene_1_complete.wait(timeout=timeout)
    
    def wait_scene_2_complete(self, timeout: Optional[float] = None) -> bool:
        """Wait for scene 2 to complete. Returns True if signaled, False if timeout."""
        return self.scene_2_complete.wait(timeout=timeout)
    
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
    Orchestrates scene-based playback of multiple CSV files.
    
    Manages:
    - Starting CSV readers for each scene
    - Synchronizing scene transitions
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
        self._stop_event = threading.Event()

    def _sleep_interruptibly(self, total_seconds: float) -> bool:
        """Sleep in short chunks so Ctrl+C and stop requests are responsive."""
        if total_seconds <= 0:
            return True
        end_time = time.time() + total_seconds
        while time.time() < end_time:
            if self._stop_event.is_set():
                return False
            remaining = end_time - time.time()
            time.sleep(min(0.1, max(0.0, remaining)))
        return True

    def _wait_for_reader(self, reader: CSVReader, timeout: float, label: str) -> bool:
        """Join a reader thread using short polling intervals for responsiveness."""
        if not reader._thread:
            return True

        deadline = time.time() + timeout
        while reader._thread.is_alive():
            if self._stop_event.is_set():
                return False
            if time.time() >= deadline:
                log.error(f"CSV reader timeout (>{timeout}s): {label}")
                return False
            reader._thread.join(timeout=0.1)
        return True
    
    def start_scene_sequentially(self, scene: SceneConfig, scene_name: str = "Scene 1") -> bool:
        """
        Start scene CSV readers sequentially (one at a time, wait for completion).
        Respects scene start_time (relative to master clock) and CSV start_time
        (relative to scene start).
        
        Args:
            scene: Scene configuration to play in sequence
            scene_name: Human-readable scene name for logging
        
        Returns:
            True if all CSVs completed successfully, False if error/timeout
        """
        csv_configs = scene.csvs
        scene_start_time = scene.start_time
        self._stop_event.clear()
        log.info(f"Starting {len(csv_configs)} CSV(s) sequentially ({scene_name})")

        elapsed = self.timing_coordinator.elapsed_time()
        if elapsed < scene_start_time:
            wait_time = scene_start_time - elapsed
            log.info(f"  Waiting {wait_time:.2f}s for {scene_name} start at {scene_start_time:.2f}s")
            if not self._sleep_interruptibly(wait_time):
                return False
        
        for idx, cfg in enumerate(csv_configs):
            # Wait until the CSV's scene-relative start offset is reached.
            target_start = scene_start_time + cfg.start_time
            elapsed = self.timing_coordinator.elapsed_time()
            if elapsed < target_start:
                wait_time = target_start - elapsed
                log.info(f"  [{idx+1}/{len(csv_configs)}] Waiting {wait_time:.2f}s before starting: {cfg.path}")
                if not self._sleep_interruptibly(wait_time):
                    return False
            
            log.info(
                f"  [{idx+1}/{len(csv_configs)}] Starting at scene+{cfg.start_time:.2f}s "
                f"(master {target_start:.2f}s): {cfg.path}"
            )
            
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
            if not self._wait_for_reader(reader, timeout=timeout, label=cfg.path):
                return False
            
            with self._lock:
                if reader in self.active_readers:
                    self.active_readers.remove(reader)
            
            log.info(f"  [{idx+1}/{len(csv_configs)}] Complete: {cfg.path}")
        
        return True
    
    def start_scene_parallel(self, scene: SceneConfig, scene_name: str = "Scene 2") -> bool:
        """
        Start scene CSV readers in parallel.
        Respects scene start_time (relative to master clock) and CSV start_time
        (relative to scene start).
        
        Args:
            scene: Scene configuration to play in parallel
            scene_name: Human-readable scene name for logging
        
        Returns:
            True if all CSVs completed successfully, False if error/timeout
        """
        csv_configs = scene.csvs
        scene_start_time = scene.start_time
        self._stop_event.clear()
        log.info(f"Starting {len(csv_configs)} CSV(s) in parallel ({scene_name})")

        elapsed = self.timing_coordinator.elapsed_time()
        if elapsed < scene_start_time:
            wait_time = scene_start_time - elapsed
            log.info(f"  Waiting {wait_time:.2f}s for {scene_name} start at {scene_start_time:.2f}s")
            if not self._sleep_interruptibly(wait_time):
                return False
        
        # Start each reader when its scene-relative offset is reached.
        csv_configs_sorted = sorted(csv_configs, key=lambda cfg: cfg.start_time)
        
        readers = []
        for idx, cfg in enumerate(csv_configs_sorted):
            target_start = scene_start_time + cfg.start_time
            elapsed = self.timing_coordinator.elapsed_time()
            if elapsed < target_start:
                wait_time = target_start - elapsed
                log.info(f"  [{idx+1}/{len(csv_configs)}] Waiting {wait_time:.2f}s before starting: {cfg.path}")
                if not self._sleep_interruptibly(wait_time):
                    return False

            log.info(
                f"  [{idx+1}/{len(csv_configs)}] Starting at scene+{cfg.start_time:.2f}s "
                f"(master {target_start:.2f}s): {cfg.path}"
            )
            
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
            if not self._wait_for_reader(reader, timeout=timeout, label=getattr(reader, "csv_path", "unknown")):
                return False
        
        with self._lock:
            for reader in readers:
                if reader in self.active_readers:
                    self.active_readers.remove(reader)
        
        log.info(f"  All {len(csv_configs)} CSV(s) completed in parallel")
        return True
    
    def stop_all(self):
        """Stop all active CSV readers."""
        self._stop_event.set()
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
    time.sleep(0.5)
    try:
        config = PerformanceConfig.from_json(str(config_path))
        log.info(f"Loaded performance config from {config_path}")
        log.info(f"  Scene 1 CSVs: {len(config.scene_1.csvs)}")
        log.info(f"  Scene 2 CSVs: {len(config.scene_2.csvs)}")
        log.info(f"  Scene 1 start: {config.scene_1.start_time:.2f}s")
        log.info(f"  Scene 2 start: {config.scene_2.start_time:.2f}s")
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
        for count in (3, 2, 1, 0):
            print(count, flush=True)
            if count > 0:
                time.sleep(0.99)

        # Start master clock
        timing_coordinator.start()
        
        # Execute Scene 1: Sequential playback
        if config.scene_1.csvs:
            if not playlist_manager.start_scene_sequentially(config.scene_1, scene_name="Scene 1"):
                log.error("Scene 1 failed")
                sys.exit(1)
            timing_coordinator.signal_scene_1_complete()
        else:
            log.info("No Scene 1 CSVs configured")
            timing_coordinator.signal_scene_1_complete()
        
        # Small pause between scenes
        time.sleep(0.1)
        
        # Before Scene 2 starts, kill any remaining Scene 1 readers
        if config.scene_2.csvs:
            scene_2_start_time = config.scene_2.start_time
            if playlist_manager.active_readers:
                log.info(f"Scene 2 starting at {scene_2_start_time:.2f}s - stopping any remaining Scene 1 readers")
                playlist_manager.stop_all()
        
        # Execute Scene 2: Parallel playback
        if config.scene_2.csvs:
            if not playlist_manager.start_scene_parallel(config.scene_2, scene_name="Scene 2"):
                log.error("Scene 2 failed")
                sys.exit(1)
            timing_coordinator.signal_scene_2_complete()
        else:
            log.info("No Scene 2 CSVs configured")
            timing_coordinator.signal_scene_2_complete()
        
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
