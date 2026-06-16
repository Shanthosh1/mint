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
              cooldown_s: float = 30.0,
              is_saturation_gain_reduction: bool = False,
              confidence: Optional[str] = None,
              limitations: Optional[str] = None,
              pre_step_motion: bool = False,
              ramped_input: bool = False,
              low_coherence: bool = False) -> None:
    """Publish a structured recommendation (exactly one change form)."""
    forms = [v is not None for v in (scale_factor, delta, target_value)]
    if sum(forms) != 1:
        raise ValueError("recommend() needs exactly one of "
                         "scale_factor / delta / target_value")
    now = time.monotonic()
    # Note: Cooldown is keyed by param name. For most params (e.g. MC_ROLLRATE_P vs MC_PITCHRATE_P,
    # or FW_YR_P vs FW_PR_P), the param strings are distinct so they cooldown independently.
    # If two axes ever share the exact same parameter name, the second axis would be suppressed
    # during the cooldown, which is acceptable since they share the same physical parameter.
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
        "is_saturation_gain_reduction": is_saturation_gain_reduction,
        "confidence": confidence,
        "limitations": limitations,
        "pre_step_motion": pre_step_motion,
        "ramped_input": ramped_input,
        "low_coherence": low_coherence,
    })

