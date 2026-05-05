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
    
    # Create server on the specified IP port 1234
    server_ip = "172.28.98.126"  # Change this to match your Isadora machine IP
    server = osc_server.ThreadingOSCUDPServer((server_ip, 1235), disp)
    print(f"OSC Listener started on {server_ip}:1234")
    print("Waiting for all OSC messages (press Ctrl+C to stop)...")
    
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        server.server_close()

if __name__ == "__main__":
    main()
