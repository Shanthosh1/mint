"""
Post-flight TECS (Total Energy Control System) analysis and parameter suggestions.
"""
from __future__ import annotations

import numpy as np
import pandas as pd
from typing import Optional

from .step_utils import _extract_euler_angles

_RESAMPLE_HZ = 10.0  # 10 Hz is sufficient for long-term trim and TECS analysis


def analyze_tecs(local_pos: pd.DataFrame | None,
                 controls: pd.DataFrame | None,
                 vehicle_att: pd.DataFrame | None,
                 airspeed: pd.DataFrame | None,
                 airspeed_validated: pd.DataFrame | None,
                 params: dict,
                 airframe_class: str | None,
                 status: pd.DataFrame | None = None) -> dict:
    """Analyse level flight, climbs, and descents to tune TECS trim parameters."""
    if airframe_class not in ("FIXED_WING", "DELTA_WING", "VTOL"):
        return {"skipped": "TECS analysis is only applicable to Fixed-Wing and VTOL vehicles."}

    if local_pos is None or local_pos.empty:
        return {"skipped": "vehicle_local_position not logged"}

    # Extract common time window
    t_pos = local_pos["timestamp"].to_numpy(np.float64) / 1e6
    t0 = t_pos[0]
    t1 = t_pos[-1]

    # Check for other required data
    if vehicle_att is None or vehicle_att.empty:
        return {"skipped": "vehicle_attitude not logged"}
    if controls is None or controls.empty:
        return {"skipped": "actuator_controls_0 not logged"}

    t_att = vehicle_att["timestamp"].to_numpy(np.float64) / 1e6
    t_ctrl = controls["timestamp"].to_numpy(np.float64) / 1e6
    t0 = max(t0, t_att[0], t_ctrl[0])
    t1 = min(t1, t_att[-1], t_ctrl[-1])

    grid = np.arange(t0, t1, 1.0 / _RESAMPLE_HZ)
    if len(grid) < 100:
        return {"skipped": "Log flight duration is too short for TECS analysis"}

    # VTOL mode segmentation: only fixed-wing mode (vehicle_type == 2)
    fw_mask = np.ones_like(grid, dtype=bool)
    vtol_note = None
    if airframe_class == "VTOL":
        if status is not None and "vehicle_type" in status.columns:
            t_status = status["timestamp"].to_numpy(np.float64) / 1e6
            vtype = status["vehicle_type"].to_numpy(np.float64)
            grid_vtype = np.round(np.interp(grid, t_status, vtype))
            fw_mask = (grid_vtype == 2)  # 2 = FIXED_WING
            if not fw_mask.any():
                return {"skipped": "No VTOL fixed-wing segments found in this log"}
        else:
            vtol_note = "VTOL log detected but vehicle_type status was not logged. Analysing entire log as fixed-wing mode."

    # Resample climb rate (-vz)
    vz = local_pos["vz"].to_numpy(np.float64)
    climb_rate_val = np.interp(grid, t_pos, -vz)

    # Resample throttle (control[3] from actuator_controls_0)
    ctrl3 = controls["control[3]"].to_numpy(np.float64)
    throttle_val = np.interp(grid, t_ctrl, ctrl3)
    # Normalize if throttle range is [-1, 1] instead of [0, 1]
    if np.min(throttle_val) < -0.1:
        throttle_val = (throttle_val + 1.0) / 2.0
    throttle_val = np.clip(throttle_val, 0.0, 1.0)

    # Resample pitch (in degrees)
    angles = _extract_euler_angles(vehicle_att)
    if angles is None or "pitch" not in angles:
        return {"skipped": "Could not extract pitch from vehicle_attitude"}
    pitch_deg = np.degrees(angles["pitch"])
    pitch_val = np.interp(grid, t_att, pitch_deg)

    # Resample airspeed
    airspeed_val = None
    # Find valid airspeed source
    as_ts, as_vals = None, None
    for df in (airspeed, airspeed_validated):
        if df is None or df.empty or "timestamp" not in df.columns:
            continue
        for col in ["indicated_airspeed_m_s", "true_airspeed_m_s", "calibrated_airspeed_m_s", "airspeed"]:
            if col in df.columns:
                as_ts = df["timestamp"].to_numpy(dtype=np.float64) / 1e6
                as_vals = df[col].to_numpy(dtype=np.float64)
                break
        if as_ts is not None:
            break

    if as_ts is not None:
        airspeed_val = np.interp(grid, as_ts, as_vals)

    # Segment classification using sliding windows
    # Level flight: climb rate < 0.5 m/s for >= 5s (50 samples), throttle std < 5% (0.05)
    is_level_idx = np.zeros_like(grid, dtype=bool)
    for i in range(49, len(grid)):
        if (np.all(np.abs(climb_rate_val[i-49 : i+1]) < 0.5) and
            np.all(fw_mask[i-49 : i+1]) and
            np.std(throttle_val[i-49 : i+1]) < 0.05):
            is_level_idx[i-49 : i+1] = True

    # Climb: throttle > 80% (0.8), climb rate > 1.0 m/s for >= 3s (30 samples)
    is_climb_idx = np.zeros_like(grid, dtype=bool)
    for i in range(29, len(grid)):
        if (np.all(throttle_val[i-29 : i+1] > 0.8) and
            np.all(climb_rate_val[i-29 : i+1] > 1.0) and
            np.all(fw_mask[i-29 : i+1])):
            is_climb_idx[i-29 : i+1] = True

    # Descent: throttle < 20% (0.2), climb rate < -1.0 m/s for >= 3s (30 samples)
    is_descent_idx = np.zeros_like(grid, dtype=bool)
    for i in range(29, len(grid)):
        if (np.all(throttle_val[i-29 : i+1] < 0.2) and
            np.all(climb_rate_val[i-29 : i+1] < -1.0) and
            np.all(fw_mask[i-29 : i+1])):
            is_descent_idx[i-29 : i+1] = True

    recommendations = []
    notes = []
    if vtol_note:
        notes.append(vtol_note)

    stats = {
        "level_flight_duration_s": round(float(np.sum(is_level_idx)) / _RESAMPLE_HZ, 1),
        "climb_duration_s": round(float(np.sum(is_climb_idx)) / _RESAMPLE_HZ, 1),
        "descent_duration_s": round(float(np.sum(is_descent_idx)) / _RESAMPLE_HZ, 1),
    }

    # 1. Level flight analysis
    if is_level_idx.any():
        level_throttle = throttle_val[is_level_idx]
        level_pitch = pitch_val[is_level_idx]
        
        # Calculate level flight confidence
        # Use standard deviation of pitch as a consistency metric
        pitch_std = float(np.std(level_pitch))
        # 30 seconds level flight is ideal
        conf_score = min(1.0, len(level_throttle) / 300.0)
        if pitch_std > 3.0:
            conf_score *= max(0.5, 1.0 - (pitch_std - 3.0) / 6.0)
        
        confidence = {
            "score": round(conf_score, 2),
            "flags": {
                "short_duration": len(level_throttle) < 150,
                "high_pitch_variance": pitch_std > 3.0
            }
        }

        # FW_THR_TRIM
        med_throttle = float(np.median(level_throttle))
        proposed_thr = round(med_throttle, 2)
        curr_thr = params.get("FW_THR_TRIM")
        if curr_thr is None or abs(curr_thr - proposed_thr) >= 0.02:
            recommendations.append({
                "param": "FW_THR_TRIM",
                "proposed_value": proposed_thr,
                "rationale": f"Detected trim throttle in level flight is {proposed_thr * 100:.0f}%. Adjust FW_THR_TRIM to maintain cruise altitude.",
                "confidence": confidence
            })

        # FW_PSP_OFF
        med_pitch = float(np.median(level_pitch))
        proposed_psp = round(med_pitch, 1)
        curr_psp = params.get("FW_PSP_OFF")
        if curr_psp is None or abs(curr_psp - proposed_psp) >= 0.2:
            recommendations.append({
                "param": "FW_PSP_OFF",
                "proposed_value": proposed_psp,
                "rationale": f"Detected trim pitch attitude in level flight is {proposed_psp:.1f}°. Adjust FW_PSP_OFF to remove steady-state height error.",
                "confidence": confidence
            })

        # FW_AIRSPD_TRIM
        if airspeed_val is not None:
            level_as = airspeed_val[is_level_idx]
            as_std = float(np.std(level_as))
            # Include airspeed variance in confidence
            as_conf_score = conf_score
            if as_std > 2.0:
                as_conf_score *= max(0.5, 1.0 - (as_std - 2.0) / 4.0)
            as_confidence = {
                "score": round(as_conf_score, 2),
                "flags": {
                    "short_duration": len(level_as) < 150,
                    "high_airspeed_variance": as_std > 2.0
                }
            }

            med_as = float(np.median(level_as))
            proposed_as = round(med_as, 1)
            curr_as = params.get("FW_AIRSPD_TRIM")
            if curr_as is None or abs(curr_as - proposed_as) >= 0.5:
                recommendations.append({
                    "param": "FW_AIRSPD_TRIM",
                    "proposed_value": proposed_as,
                    "rationale": f"Detected cruise airspeed in level flight is {proposed_as:.1f} m/s. Adjust FW_AIRSPD_TRIM to match.",
                    "confidence": as_confidence
                })
        else:
            notes.append("Airspeed data not found or invalid. Skipping cruise airspeed calibration.")
    else:
        notes.append("No level flight segments found. Maintain level flight (climb rate < 0.5 m/s, stable throttle) for at least 5 seconds to calibrate trim values.")

    # 2. Climb analysis
    if is_climb_idx.any():
        climb_rates = climb_rate_val[is_climb_idx]
        max_climb = float(np.max(climb_rates))
        proposed_climb = round(max_climb, 1)
        curr_climb = params.get("FW_CLIMB_MAX")
        
        # Climb confidence
        climb_conf_score = min(1.0, len(climb_rates) / 100.0)
        climb_confidence = {
            "score": round(climb_conf_score, 2),
            "flags": {
                "short_duration": len(climb_rates) < 50
            }
        }

        if curr_climb is None or abs(curr_climb - proposed_climb) >= 0.2:
            recommendations.append({
                "param": "FW_CLIMB_MAX",
                "proposed_value": proposed_climb,
                "rationale": f"Observed maximum climb rate is {proposed_climb:.1f} m/s during full throttle climb. Update FW_CLIMB_MAX to match vehicle capabilities.",
                "confidence": climb_confidence
            })
    else:
        notes.append("No maximum-rate climb segments found (throttle > 80%, climb rate > 1.0 m/s for >= 3s).")

    # 3. Descent analysis
    if is_descent_idx.any():
        sink_rates = -climb_rate_val[is_descent_idx]
        min_sink = float(np.min(sink_rates))
        proposed_sink = round(min_sink, 1)
        curr_sink = params.get("FW_SINK_MIN")
        
        # Descent confidence
        descent_conf_score = min(1.0, len(sink_rates) / 100.0)
        descent_confidence = {
            "score": round(descent_conf_score, 2),
            "flags": {
                "short_duration": len(sink_rates) < 50
            }
        }

        if curr_sink is None or abs(curr_sink - proposed_sink) >= 0.2:
            recommendations.append({
                "param": "FW_SINK_MIN",
                "proposed_value": proposed_sink,
                "rationale": f"Observed minimum sink rate at idle throttle is {proposed_sink:.1f} m/s. Update FW_SINK_MIN to prevent stall/undershoot in TECS.",
                "confidence": descent_confidence
            })
    else:
        notes.append("No idle-throttle descent segments found (throttle < 20%, climb rate < -1.0 m/s for >= 3s).")

    return {
        "stats": stats,
        "recommendations": recommendations,
        "notes": notes
    }
