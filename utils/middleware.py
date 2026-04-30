"""Middleware dispatcher for OSC output effects."""

from typing import Callable, Any

from utils.xyz import send_xyz

OSCPlugin = Callable[[dict, Any], bool]

# Add all plugins you want to use here. Each must match OSCPlugin.
DEFAULT_HANDLERS: tuple[OSCPlugin, ...] = ()


def main(item: dict, osc_client, handlers: tuple[OSCPlugin, ...] | None = None) -> bool:
    """Dispatch a payload to the configured OSC handlers.

    This is kept for single-item processing. For batch processing with puzzle_piece,
    use batch_middleware with OSCBatchProcessor instead.
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