# mocap_client

Scaffold package to receive motion-capture data (NatNet/Motive) and queue
quaternion data for processing by an `OSC_Processor` thread.

Next steps to integrate with `polymidi`:

- Copy or import the NatNet `NatClient` implementation from your existing
  `polymidi` project (C:\\Users\\Wearables Lab\\Documents\\Wearables\\polymidi) into
  `mocap_client/nat_client.py` and replace the scaffold receiver logic.
- Ensure the real `NatClient` pushes payloads to the same Queue format:
  `{"segment": str, "quat": (x,y,z,w), "timestamp": float}`.
- Implement OSC sending logic in `mocap_client/osc_processor.py` (e.g., using
  `python-osc`) to forward data to downstream tools.

Running the scaffold (simulation):

```bash
python -m mocap_client.main
```
