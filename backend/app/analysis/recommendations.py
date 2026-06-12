"""
Live recommendation bus.

Analysis engines call `recommend()` to surface a *structured* tuning
suggestion (param + relative or absolute change) instead of prose-only
alerts. The UI renders these with a "Stage as proposal" action; staging
resolves the relative change against the live on-vehicle value and runs
the full safety-registry + Approve & Write pipeline — engines never
write anything, same as everywhere else in the app.

Per-parameter cooldown stops a persistent condition from spamming the
pilot with identical cards.
"""
from __future__ import annotations

import time
from typing import Optional

from ..mavlink.telemetry_hub import HUB

_last_emit: dict[str, float] = {}


def recommend(param: str, rationale: str, *,
              scale_factor: Optional[float] = None,
              delta: Optional[float] = None,
              target_value: Optional[float] = None,
              source: str = "live",
              cooldown_s: float = 30.0) -> None:
    """Publish a structured recommendation (exactly one change form)."""
    forms = [v is not None for v in (scale_factor, delta, target_value)]
    if sum(forms) != 1:
        raise ValueError("recommend() needs exactly one of "
                         "scale_factor / delta / target_value")
    now = time.monotonic()
    if now - _last_emit.get(param, 0.0) < cooldown_s:
        return
    _last_emit[param] = now
    HUB.publish("recommendation", {
        "param": param,
        "scale_factor": scale_factor,
        "delta": delta,
        "target_value": target_value,
        "rationale": rationale,
        "source": source,
    })
