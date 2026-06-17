"""
Post-flight position loop tracking analysis and gain suggestions.
"""
from __future__ import annotations

import math
from typing import Optional
import numpy as np
import pandas as pd

from .step_utils import _uniform, _find_step_indices, _step_response_offline, compute_fd_gain, compute_axis_confidence
from .live_pid import LivePidEngine

_RESAMPLE_HZ = 100.0


def analyze_position(local_pos: pd.DataFrame | None,
                     local_pos_sp: pd.DataFrame | None,
                     params: dict,
                     airframe_class: str | None,
                     status: pd.DataFrame | None = None) -> dict:
    """Analyse position loop step responses during mission waypoint transitions."""
    if airframe_class in ("FIXED_WING", "DELTA_WING"):
        return {"skipped": "Position loop analysis is only applicable to Multirotors and VTOL vehicles."}

    if local_pos is None or local_pos_sp is None:
        return {"skipped": "vehicle_local_position or vehicle_local_position_setpoint not logged"}

    # Extract common time window
    t_sp = local_pos_sp["timestamp"].to_numpy(np.float64) / 1e6
    t_act = local_pos["timestamp"].to_numpy(np.float64) / 1e6
    t0 = max(t_sp[0], t_act[0])
    t1 = min(t_sp[-1], t_act[-1])
    grid = np.arange(t0, t1, 1.0 / _RESAMPLE_HZ)
    if len(grid) < 200:
        return {"skipped": "Log flight duration is too short for position analysis"}

    # Resample status/vehicle_type for VTOL mode segmentation and nav_state for mission checks
    mc_mask = np.ones_like(grid, dtype=bool)
    is_mission = np.zeros_like(grid, dtype=bool)
    vtol_note = None

    if status is not None:
        t_status = status["timestamp"].to_numpy(np.float64) / 1e6
        if "vehicle_type" in status.columns and airframe_class == "VTOL":
            vtype = status["vehicle_type"].to_numpy(np.float64)
            grid_vtype = np.round(np.interp(grid, t_status, vtype))
            mc_mask = (grid_vtype == 1)  # 1 = Multicopter
            if not mc_mask.any():
                return {"skipped": "No VTOL multicopter (hover) segments found in this log"}
        elif airframe_class == "VTOL":
            vtol_note = "VTOL log detected but vehicle_type status was not logged. Analysing entire log as multicopter mode."
            
        if "nav_state" in status.columns:
            nav_state = status["nav_state"].to_numpy(np.float64)
            grid_nav = np.round(np.interp(grid, t_status, nav_state))
            is_mission = (grid_nav == 3)  # 3 = NAVIGATION_STATE_AUTO_MISSION
    elif airframe_class == "VTOL":
        vtol_note = "VTOL log detected but vehicle_type status was not logged. Analysing entire log as multicopter mode."

    # Estimate wind speed
    wind_speed = None
    if "windspeed_east" in local_pos.columns and "windspeed_north" in local_pos.columns:
        w_east = local_pos["windspeed_east"].to_numpy(np.float64)
        w_north = local_pos["windspeed_north"].to_numpy(np.float64)
        w_mag = np.sqrt(w_east**2 + w_north**2)
        wind_speed = float(np.mean(w_mag))

    axes: dict[str, dict] = {}
    recommendations: list[dict] = []
    notes: list[str] = []
    if vtol_note:
        notes.append(vtol_note)

    mission_steps_count = 0

    for axis in ("x", "y", "z"):
        sp_col, act_col = axis, axis
        if sp_col not in local_pos_sp.columns or act_col not in local_pos.columns:
            continue

        sp_u = np.interp(grid, t_sp, local_pos_sp[sp_col].to_numpy(np.float64))
        act_u = np.interp(grid, t_act, local_pos[act_col].to_numpy(np.float64))

        min_amp = 0.15 if axis == "z" else 0.30
        indices = _find_step_indices(sp_u, threshold=min_amp)
        # Apply multicopter mask
        indices = [i for i in indices if mc_mask[i]]

        candidates_count = len(indices)
        step_results = []
        taus, settlings, overshoots = [], [], []
        win_sp, win_act = [], []
        last_step_t = None

        rej_counts = {"window_too_short": 0, "level_change": 0, "snr": 0, "ramp": 0, "too_small": 0}

        for i in indices:
            slice_pre = int(1.5 * _RESAMPLE_HZ)
            slice_post = int(4.5 * _RESAMPLE_HZ)  # larger post window for position loop settling
            lo, hi = max(0, i - slice_pre), min(len(sp_u), i + slice_post)

            res, reason = _step_response_offline(
                grid[lo:hi], sp_u[lo:hi], act_u[lo:hi],
                min_amp=min_amp,
                last_step_t=last_step_t,
                derived=False,
                sp_stable_window_s=2.0,
                osc_window_s=6.0
            )

            last_step_t = float(grid[i])

            if res is None:
                if reason in rej_counts:
                    rej_counts[reason] += 1
                continue

            # Record if step occurred during mission mode
            if is_mission[i]:
                mission_steps_count += 1

            step_results.append(res)
            if res.get("plateau_duration_ok", True):
                if res["tau_s"] is not None:
                    taus.append(res["tau_s"])
                if res["settling_s"] is not None:
                    settlings.append(res["settling_s"])
                if res["overshoot"] is not None:
                    overshoots.append(res["overshoot"])
                win_sp.append(sp_u[lo:hi])
                win_act.append(act_u[lo:hi])

        # Confidence assessment
        confidence = compute_axis_confidence(
            step_results, candidates_count,
            wind_speed=wind_speed,
            low_coherence=False,
            derived=False
        )

        axis_stats = {
            "n_steps": len(step_results),
            "n_steps_time_domain": len(overshoots),
            "candidates": candidates_count,
            "rejections": rej_counts,
            "confidence": confidence,
            "tau_s_median": round(float(np.median(taus)), 3) if taus else None,
            "settling_s_median": round(float(np.median(settlings)), 3) if settlings else None,
            "overshoot_max": round(float(np.max(overshoots)), 3) if overshoots else None,
        }

        if win_sp and win_act:
            tm = LivePidEngine._tracking_metrics(np.concatenate(win_sp), np.concatenate(win_act))
            axis_stats["r"] = tm["r"]
            axis_stats["nrmse"] = tm["nrmse"]
        else:
            axis_stats["r"] = None
            axis_stats["nrmse"] = None

        axes[axis] = axis_stats

        # Propose recommendations based on loop metrics
        # Resolve target parameter name
        p_param = "MPC_XY_P" if axis in ("x", "y") else "MPC_Z_P"
        curr_p = params.get(p_param)

        # 1. Check rejections first
        if len(step_results) == 0 and candidates_count > 0:
            rejs = [f"{v} {k}" for k, v in rej_counts.items() if v > 0]
            notes.append(f"Position {axis}: {candidates_count} steps found but all rejected ({', '.join(rejs)}).")
            continue

        if len(step_results) == 0:
            notes.append(f"Position {axis}: No step maneuvers detected. Run a mission or execute distinct waypoints.")
            continue

        tau = axis_stats["tau_s_median"]
        settling = axis_stats["settling_s_median"]
        overshoot = axis_stats["overshoot_max"]

        # Overshoot rule
        if curr_p is not None and overshoot is not None and overshoot > 0.25:
            recommendations.append({
                "param": p_param,
                "proposed_value": round(float(curr_p) * 0.9, 4),
                "rationale": f"Position {axis} overshoot ({overshoot*100:.0f}% > 25%) indicates a P gain that is too high. Soften P gain to prevent overshoot.",
                "confidence": confidence,
            })
        # Sluggish settling rule
        elif curr_p is not None and tau is not None and settling is not None and settling > 4.0 * tau:
            recommendations.append({
                "param": p_param,
                "proposed_value": round(float(curr_p) * 1.1, 4),
                "rationale": f"Position {axis} settling is sluggish (settling {settling:.1f}s is > 4x time constant τ={tau:.1f}s). Increase P gain.",
                "confidence": confidence,
            })

    # Add note if no steps occurred during mission mode
    if candidates_count > 0 and mission_steps_count == 0:
        notes.append("Position loop analysis is most reliable with mission-generated waypoint steps. Manual POSCTL steps are often too small to produce meaningful metrics.")

    return {
        "axes": axes,
        "recommendations": recommendations,
        "notes": notes,
    }
