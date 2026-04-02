"""Simple OSC listener that receives messages on port 1234"""
from pythonosc import dispatcher
from pythonosc import osc_server

def message_handler(addr, *args):
    """Handle received OSC messages"""
    print(f"[{addr}] {' '.join(str(a) for a in args)}")

def main():
    # Create dispatcher and map message handler to all addresses
    disp = dispatcher.Dispatcher()
    disp.set_default_handler(message_handler)
    
    # Create server on localhost port 1234
    server = osc_server.ThreadingOSCUDPServer(("127.0.0.1", 1234), disp)
    print("OSC Listener started on 127.0.0.1:1234")
    print("Waiting for all OSC messages (press Ctrl+C to stop)...")
    
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        server.server_close()

if __name__ == "__main__":
    main()
