"""
Actuator deflection saturation analysis (actuator_controls_0).

Control outputs pinned at ±1.0 mean the controller is demanding more
authority than the airframe has — a classic symptom of gains too high,
excessive vibration feeding the D-term, or an underpowered vehicle.
"""
from __future__ import annotations

import numpy as np
import pandas as pd

_CHANNELS = {
    "roll": "control[0]",
    "pitch": "control[1]",
    "yaw": "control[2]",
    "thrust": "control[3]",
}
_SAT_EPS = 0.005          # |u| >= 1 - eps counts as saturated
_BURST_WARN_S = 0.2       # sustained saturation longer than this is flagged


def analyze_saturation(actuators: pd.DataFrame) -> dict:
    """Per-channel saturation statistics + the longest continuous burst."""
    ts = actuators["timestamp"].to_numpy(dtype=np.float64) / 1e6
    out: dict[str, dict] = {}
    worst: tuple[str, float] | None = None

    for name, col in _CHANNELS.items():
        if col not in actuators.columns:
            continue
        u = actuators[col].to_numpy(dtype=np.float64)
        # Thrust saturates only at the top; attitude axes at both rails.
        saturated = (u >= 1.0 - _SAT_EPS) if name == "thrust" \
            else (np.abs(u) >= 1.0 - _SAT_EPS)

        pct = float(100.0 * np.count_nonzero(saturated) / max(1, len(u)))
        longest = _longest_burst_s(ts, saturated)
        out[name] = {
            "saturated_pct": round(pct, 2),
            "longest_burst_s": round(longest, 3),
            "flagged": longest >= _BURST_WARN_S or pct > 2.0,
        }
        if out[name]["flagged"] and (worst is None or longest > worst[1]):
            worst = (name, longest)

    advice = None
    if worst:
        axis, dur = worst
        advice = (
            f"{axis.capitalize()} output saturated for up to {dur:.2f} s at a time. "
            + ("Check hover throttle, payload weight, and battery sag."
               if axis == "thrust" else
               f"Reduce {axis} rate gains or address vibration before "
               f"increasing authority demands.")
        )

    return {"channels": out, "advice": advice}


def _longest_burst_s(ts: np.ndarray, mask: np.ndarray) -> float:
    """Length in seconds of the longest contiguous True run in `mask`."""
    if not mask.any():
        return 0.0
    edges = np.diff(mask.astype(np.int8))
    starts = np.flatnonzero(edges == 1) + 1
    ends = np.flatnonzero(edges == -1) + 1
    if mask[0]:
        starts = np.r_[0, starts]
    if mask[-1]:
        ends = np.r_[ends, len(mask)]
    durations = ts[np.minimum(ends - 1, len(ts) - 1)] - ts[starts]
    return float(durations.max()) if durations.size else 0.0
