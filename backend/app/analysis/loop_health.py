"""
Shared cascade health state.

The cascaded controllers must be tuned inside-out: an outer-loop verdict
is meaningless while an inner loop is unhealthy (a position "overshoot"
on top of a saturated rate loop is the rate loop's fault). Each loop
analyzer records its health here; outer loops consult their inner
neighbour before emitting recommendations. Metrics still stream to the
UI regardless — only *advice* is gated.

Kept in its own module so live_pid (rate loop) and cascade.py (outer
loops) can share it without a circular import.
"""
from __future__ import annotations

# loop name -> True (healthy) / False (impaired). Missing = unknown,
# treated as healthy so a loop that never produced data doesn't block
# the cascade forever.
HEALTH: dict[str, bool] = {}

_INNER = {"attitude": "rate", "velocity": "attitude", "position": "velocity"}


def set_health(loop: str, healthy: bool) -> None:
    HEALTH[loop] = healthy


def inner_loop_healthy(loop: str) -> bool:
    """True when every loop inside `loop` is healthy (or unknown)."""
    inner = _INNER.get(loop)
    while inner:
        if HEALTH.get(inner) is False:
            return False
        inner = _INNER.get(inner)
    return True
