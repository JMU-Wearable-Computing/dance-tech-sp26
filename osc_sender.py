"""Simple OSC sender that transmits integers 1-10 to localhost:1234"""
import time
from pythonosc import udp_client

def main():
    # Create OSC client pointing to localhost on port 1234
    client = udp_client.SimpleUDPClient("127.0.0.1", 1234)
    
    print("Sending integers 1-10 to 127.0.0.1:1234...")
    for i in range(1, 30):
        client.send_message("/data", i)
        print(f"  [{i}/10] Sent: {i}")
        time.sleep(0.5)  # Small delay between sends
    
    print("Done!")

if __name__ == "__main__":
    main()