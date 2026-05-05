"""Middleware dispatcher for OSC output effects."""

from typing import Callable, Any

from utils.xyz import send_xyz
from utils.bounding_box import bounding_box

OSCPlugin = Callable[[dict, Any], bool]

# Add all plugins you want to use here. Each must match OSCPlugin.
DEFAULT_HANDLERS: tuple[OSCPlugin, ...] = (bounding_box, send_xyz)


def main(item: dict, osc_client, handlers: tuple[OSCPlugin, ...] | None = None) -> bool:
    """Dispatch a payload to the configured OSC handlers.

    The default handler set currently sends XYZ position data, but this
    function is the plug-in point for future effects.
    """
    active_handlers = DEFAULT_HANDLERS if handlers is None else handlers
    handled = False

    for handler in active_handlers:
        try:
            handled = handler(item, osc_client) or handled
        except Exception as exc:
            segment = item.get("segment")
            print(f"middleware: handler {getattr(handler, '__name__', handler)} failed for {segment}: {exc}")

    return handled