"""mocap_client package

Receives OptiTrack Motive data via NatNet, filters rigid-body frames
by configured segment names, extracts orientation quaternions, and
hands them to OSC_Processor via a Queue.

Raw NatNet support is provided by the bundled NatNetClient.py,
MoCapData.py, and DataDescriptions.py (from the NaturalPoint SDK /
polymidi project).
"""
from .nat_client import NatClient
from .osc_processor import OSCProcessor

__all__ = ["NatClient", "OSCProcessor"]
