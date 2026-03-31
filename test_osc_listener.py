#!/usr/bin/env python3
"""
Simple OSC listener server to verify incoming OSC messages.
Listens on localhost:1234 for messages on /isadora/1 and /isadora-multi/1
"""

from pythonosc import dispatcher
from pythonosc import osc_server
import time

def handler_isadora_1(unused_addr, value):
    """Handler for /isadora/1 messages"""
    print(f"✓ /isadora/1 received: {value}")

def handler_isadora_multi_1(unused_addr, *values):
    """Handler for /isadora-multi/1 messages (list of values)"""
    print(f"✓ /isadora-multi/1 received: {values}")

def main():
    print("Starting OSC Listener on localhost:1234...")
    print("Listening for /isadora/1 and /isadora-multi/1")
    print("Press Ctrl-C to stop\n")
    
    # Create dispatcher and add handlers
    disp = dispatcher.Dispatcher()
    disp.map("/isadora/1", handler_isadora_1)
    disp.map("/isadora-multi/1", handler_isadora_multi_1)
    
    # Create and run server
    server = osc_server.ThreadingOSCUDPServer(("127.0.0.1", 1234), disp)
    print(f"OSC Server listening on {server.server_address}")
    
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n\nShutting down...")
        server.shutdown()

if __name__ == "__main__":
    main()
