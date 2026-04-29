"""Batch middleware dispatcher for OSC output effects.

Processes entire frame batches at once, enabling cross-item calculations
like relative offsets and anchor points.
"""

from typing import Callable, Any

from utils.puzzle_piece import puzzle_piece

OSCBatchPlugin = Callable[[list[dict], Any], bool]

# Add all batch plugins you want to use here. Each must match OSCBatchPlugin.
DEFAULT_BATCH_HANDLERS: tuple[OSCBatchPlugin, ...] = (puzzle_piece,)


def main(items: list[dict], osc_client, handlers: tuple[OSCBatchPlugin, ...] | None = None, **kwargs) -> bool:
    """Dispatch a batch of payloads (entire frame) to the configured batch handlers.

    This allows handlers to see all bones/joints for a frame at once,
    enabling relative offset calculations and anchor point computations.
    """
    if not items:
        return False
    
    active_handlers = DEFAULT_BATCH_HANDLERS if handlers is None else handlers
    handled = False

    for handler in active_handlers:
        try:
            handled = handler(items, osc_client, **kwargs) or handled
        except Exception as exc:
            frame_num = items[0].get("frame") if items else "unknown"
            print(f"batch_middleware: handler {getattr(handler, '__name__', handler)} failed for frame {frame_num}: {exc}")

    return handled
