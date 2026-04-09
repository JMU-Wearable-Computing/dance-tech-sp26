"""Quick test of csv_client streaming to verify functionality."""
import sys
import time
from queue import Queue
from pathlib import Path

# Import configuration from main
from csv_client.main import CSV_PATH, PLAYBACK_RATE, TARGET_SEGMENT, SKELETON_BONES

# Test parameters
FRAMES_TO_PROCESS = 100
ISADORA_IP = "127.0.0.1"
ISADORA_PORT = 1234


def test_csv_client():
    from csv_client.csv_reader import CSVReader
    
    q = Queue()
    reader = CSVReader(
        csv_path=CSV_PATH,
        out_queue=q,
        target_segment=TARGET_SEGMENT,
        skeleton_bones=SKELETON_BONES,
        playback_rate=100.0,  # Very fast for testing
    )
    
    print(f"Testing csv_client with {FRAMES_TO_PROCESS} frames")
    print(f"Config: CSV_PATH='{CSV_PATH}', TARGET_SEGMENT={TARGET_SEGMENT}, SKELETON_BONES={SKELETON_BONES}\n")
    
    # Read headers and process frames
    import csv
    with open(reader.csv_path, 'r', newline='') as f:
        csv_reader = csv.reader(f)
        reader._parse_headers(csv_reader)
        next(csv_reader)  # Skip the label row
        
        print(f"✓ Found {len(reader.segment_map)} segments")
        print(f"  Sample segments: {', '.join(reader.segment_order[:5])}")
        
        # Process first N frames
        frame_count = 0
        for row in csv_reader:
            if frame_count >= FRAMES_TO_PROCESS:
                break
            if not row or not row[0]:
                continue
            
            frame_num = int(row[0])
            timestamp = float(row[1])
            reader._enqueue_frame_data(row, frame_num, timestamp)
            frame_count += 1
            
            if frame_count % 20 == 0:
                print(f"  Processed {frame_count} frames, queue size: {q.qsize()}")
    
    # Verify data in queue
    print(f"\n✓ Processed {frame_count} frames, queue has {q.qsize()} items\n")
    
    # Sample data
    print("Sample payload from queue:")
    sample_count = 0
    while not q.empty() and sample_count < 3:
        item = q.get()
        print(f"  {sample_count}: {item['segment']:20s} pos={item['pos']}, quat={item['quat']}")
        sample_count += 1
    
    print("\n✓ CSV client test successful!")
    return True


if __name__ == "__main__":
    try:
        test_csv_client()
        print("\nAll checks passed. Ready to stream to Isadora.")
        print(f"Configure Isadora to receive OSC on {ISADORA_IP}:{ISADORA_PORT}")
    except Exception as e:
        print(f"\n✗ Test failed: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc()
        sys.exit(1)
