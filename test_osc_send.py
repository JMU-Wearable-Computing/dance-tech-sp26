#!/usr/bin/env python3
"""
Simple OSC test script — sends "hello" to Isadora 10 times, once per second.
"""

import time
from pythonosc import udp_client

# OSC client on localhost:1234
client = udp_client.SimpleUDPClient("127.0.0.1", 1234)

# Send "hello" to /isadora/1 ten times, 1 second apart
for i in range(10):
    print(f"[{i+1}/10] Sending 'hello' to /isadora/1")
    client.send_message("/isadora/1", "hello")
    time.sleep(1)

print("Done!")
