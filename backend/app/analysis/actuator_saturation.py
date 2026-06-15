"""
Actuator deflection saturation analysis (actuator_outputs / actuator_controls_0).

Control outputs pinned at maximum mean the controller is demanding more
authority than the airframe has — a classic symptom of gains too high,
excessive vibration feeding the D-term, or an underpowered vehicle.
"""
from __future__ import annotations

import numpy as np
import pandas as pd

from ..core import config

_SAT_EPS = config.SAT_EPS
_BURST_WARN_S = config.BURST_WARN_S



def analyze_saturation(actuators: pd.DataFrame, actuator_map: dict | None = None,
                       airframe_class: str | None = None, is_physical: bool = False) -> dict:
    """Analyze actuator outputs or control setpoints for saturation."""
    ts = actuators["timestamp"].to_numpy(dtype=np.float64) / 1e6
    out: dict[str, dict] = {}
    worst: tuple[str, float] | None = None

    if is_physical and actuator_map:
        # Check if any columns are output[0] to output[15]
        # Map roles to keys
        roles = [
            ("hover_motors", "M", True),
            ("thrust_motors", "P", True),
            ("control_surfaces", "S", False),
            ("tilt_servos", "Tilt", False),
        ]
        
        for group_name, prefix, is_motor in roles:
            channels = actuator_map.get(group_name, [])
            for idx, ch in enumerate(channels):
                col = f"output[{ch}]"
                if col not in actuators.columns:
                    continue
                
                u = actuators[col].to_numpy(dtype=np.float64)
                if u.size == 0:
                    continue
                
                # Auto-detect raw PWM vs normalized values
                # If values are raw PWM (e.g. >200), we normalize them
                is_raw_pwm = np.any(u > 200.0)
                if is_raw_pwm:
                    if is_motor:
                        u_norm = np.clip((u - 1000.0) / 1000.0, 0.0, 1.0)
                    else:
                        u_norm = np.clip((u - 1500.0) / 500.0, -1.0, 1.0)
                else:
                    if is_motor:
                        u_norm = np.clip(u, 0.0, 1.0)
                    else:
                        u_norm = np.clip(u, -1.0, 1.0)

                # Saturation condition
                if is_motor:
                    saturated = (u_norm >= 1.0 - _SAT_EPS)
                else:
                    saturated = (np.abs(u_norm) >= 1.0 - _SAT_EPS)

                pct = float(100.0 * np.count_nonzero(saturated) / max(1, len(u_norm)))
                longest = _longest_burst_s(ts, saturated)
                
                label = f"{prefix}{idx + 1} (Ch{ch + 1})"
                out[label] = {
                    "saturated_pct": round(pct, 2),
                    "longest_burst_s": round(longest, 3),
                    "flagged": longest >= _BURST_WARN_S or pct > 2.0,
                }
                
                if out[label]["flagged"] and (worst is None or longest > worst[1]):
                    worst = (label, longest)
    else:
        # Legacy control setpoint analysis fallback
        legacy_channels = {
            "roll": "control[0]",
            "pitch": "control[1]",
            "yaw": "control[2]",
            "thrust": "control[3]",
        }
        for name, col in legacy_channels.items():
            if col not in actuators.columns:
                continue
            u = actuators[col].to_numpy(dtype=np.float64)
            if u.size == 0:
                continue
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
        if "M" in axis or "P" in axis or "thrust" in axis:
            advice = (
                f"{axis} output saturated for up to {dur:.2f} s at a time. "
                "Check hover throttle, payload weight, battery sag, or forward thrust authority."
            )
        else:
            advice = (
                f"{axis} control surface railed for up to {dur:.2f} s at a time. "
                "Reduce rate gains or address mechanical linkages before increasing authority demands."
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
