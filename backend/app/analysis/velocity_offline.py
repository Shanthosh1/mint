"""
Post-flight velocity loop tracking analysis and gain suggestions.
"""
from __future__ import annotations

import math
from typing import Optional
import numpy as np
import pandas as pd

from .step_utils import _uniform, _find_step_indices, _step_response_offline, compute_fd_gain, compute_axis_confidence
from .live_pid import LivePidEngine

_RESAMPLE_HZ = 100.0


def analyze_velocity(local_pos: pd.DataFrame | None,
                     local_pos_sp: pd.DataFrame | None,
                     params: dict,
                     airframe_class: str | None,
                     status: pd.DataFrame | None = None) -> dict:
    """Analyse velocity loop step responses and steady-state offsets."""
    if airframe_class in ("FIXED_WING", "DELTA_WING"):
        return {"skipped": "Velocity loop analysis is only applicable to Multirotors and VTOL vehicles."}

    if local_pos is None or local_pos_sp is None:
        return {"skipped": "vehicle_local_position or vehicle_local_position_setpoint not logged"}

    # Extract common time window
    t_sp = local_pos_sp["timestamp"].to_numpy(np.float64) / 1e6
    t_act = local_pos["timestamp"].to_numpy(np.float64) / 1e6
    t0 = max(t_sp[0], t_act[0])
    t1 = min(t_sp[-1], t_act[-1])
    grid = np.arange(t0, t1, 1.0 / _RESAMPLE_HZ)
    if len(grid) < 200:
        return {"skipped": "Log flight duration is too short for velocity analysis"}

    # Resample status/vehicle_type for VTOL mode segmentation
    mc_mask = np.ones_like(grid, dtype=bool)
    vtol_note = None
    if airframe_class == "VTOL":
        if status is not None and "vehicle_type" in status.columns:
            t_status = status["timestamp"].to_numpy(np.float64) / 1e6
            vtype = status["vehicle_type"].to_numpy(np.float64)
            grid_vtype = np.round(np.interp(grid, t_status, vtype))
            mc_mask = (grid_vtype == 1)  # 1 = Multicopter
            if not mc_mask.any():
                return {"skipped": "No VTOL multicopter (hover) segments found in this log"}
        else:
            vtol_note = "VTOL log detected but vehicle_type status was not logged. Analysing entire log as multicopter mode."

    # Estimate wind speed if wind fields exist
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

    for axis in ("vx", "vy", "vz"):
        sp_col, act_col = axis, axis
        if sp_col not in local_pos_sp.columns or act_col not in local_pos.columns:
            continue

        sp_u = np.interp(grid, t_sp, local_pos_sp[sp_col].to_numpy(np.float64))
        act_u = np.interp(grid, t_act, local_pos[act_col].to_numpy(np.float64))

        min_amp = 0.15 if axis == "vz" else 0.20
        indices = _find_step_indices(sp_u, threshold=min_amp)
        # Apply multicopter mask
        indices = [i for i in indices if mc_mask[i]]

        candidates_count = len(indices)
        step_results = []
        taus, settlings, overshoots, oscillations_list = [], [], [], []
        win_sp, win_act = [], []
        last_step_t = None

        rej_counts = {"window_too_short": 0, "level_change": 0, "snr": 0, "ramp": 0, "too_small": 0}

        for i in indices:
            slice_pre = int(1.5 * _RESAMPLE_HZ)
            slice_post = int(3.5 * _RESAMPLE_HZ)
            lo, hi = max(0, i - slice_pre), min(len(sp_u), i + slice_post)

            res, reason = _step_response_offline(
                grid[lo:hi], sp_u[lo:hi], act_u[lo:hi],
                min_amp=min_amp,
                last_step_t=last_step_t,
                derived=False,
                sp_stable_window_s=1.0,
                osc_window_s=4.0
            )

            last_step_t = float(grid[i])

            if res is None:
                if reason in rej_counts:
                    rej_counts[reason] += 1
                continue

            step_results.append(res)
            if res.get("plateau_duration_ok", True):
                if res["tau_s"] is not None:
                    taus.append(res["tau_s"])
                if res["settling_s"] is not None:
                    settlings.append(res["settling_s"])
                if res["overshoot"] is not None:
                    overshoots.append(res["overshoot"])
                if res["oscillations"] is not None:
                    oscillations_list.append(res["oscillations"])
                win_sp.append(sp_u[lo:hi])
                win_act.append(act_u[lo:hi])

        # Compute steady-state error (I-gain deficit)
        # 2-second rolling window is 200 samples
        sp_series = pd.Series(sp_u)
        act_series = pd.Series(act_u)
        sp_std = sp_series.rolling(200).std().to_numpy()
        act_std = act_series.rolling(200).std().to_numpy()

        # Filter only multicopter hover segments
        settled_mask = (sp_std < 0.01) & (act_std < 0.05) & mc_mask
        mean_offset = 0.0
        if settled_mask.any():
            mean_offset = float(np.mean(np.abs(sp_u[settled_mask] - act_u[settled_mask])))

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
            "steady_state_offset": round(mean_offset, 3),
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
        p_param, i_param, d_param = None, None, None
        if axis in ("vx", "vy"):
            p_param = "MPC_XY_VEL_P_ACC"
            i_param = "MPC_XY_VEL_I_ACC"
            d_param = "MPC_XY_VEL_D_ACC"
        elif axis == "vz":
            p_param = "MPC_Z_VEL_P_ACC"
            i_param = "MPC_Z_VEL_I_ACC"
            d_param = "MPC_Z_VEL_D_ACC"

        curr_p = params.get(p_param)
        curr_i = params.get(i_param)
        curr_d = params.get(d_param)

        # 1. Check rejections first
        if len(step_results) == 0 and candidates_count > 0:
            rejs = [f"{v} {k}" for k, v in rej_counts.items() if v > 0]
            notes.append(f"Velocity {axis}: {candidates_count} steps found but all rejected ({', '.join(rejs)}).")
            continue

        if len(step_results) == 0:
            notes.append(f"Velocity {axis}: No step maneuvers detected. Perform distinct velocity steps in POSCTL.")
            continue

        tau = axis_stats["tau_s_median"]
        settling = axis_stats["settling_s_median"]
        overshoot = axis_stats["overshoot_max"]
        oscillations = int(np.median(oscillations_list)) if oscillations_list else 0

        # Overshoot rule
        if curr_p is not None and overshoot is not None and overshoot > 0.25:
            recommendations.append({
                "param": p_param,
                "proposed_value": round(float(curr_p) * 0.9, 4),
                "rationale": f"Velocity {axis} overshoot ({overshoot*100:.0f}% > 25%) indicates excessive P gain. Soften P gain to stabilize.",
                "confidence": confidence,
            })
        # Sluggish settling rule
        elif curr_p is not None and tau is not None and settling is not None and settling > 4.0 * tau:
            recommendations.append({
                "param": p_param,
                "proposed_value": round(float(curr_p) * 1.1, 4),
                "rationale": f"Velocity {axis} response is sluggish (settling {settling:.1f}s is > 4x time constant τ={tau:.1f}s). Increase P gain.",
                "confidence": confidence,
            })

        # Steady-state error rule (I-gain deficit)
        if curr_i is not None and mean_offset > 0.05:
            # Only suggest raising I-gain if response is not completely sluggish
            if tau is not None and tau < 1.5:
                recommendations.append({
                    "param": i_param,
                    "proposed_value": round(float(curr_i) * 1.15, 4),
                    "rationale": f"Velocity {axis} exhibits a persistent steady-state tracking offset of {mean_offset:.2f} m/s (>0.05 m/s). Raise I-gain to eliminate offset.",
                    "confidence": confidence,
                })

        # Damping rule (D-gain increase)
        if curr_d is not None and overshoot is not None and settling is not None and tau is not None:
            if overshoot > 0.15 and settling > 3.0 * tau and oscillations >= 2:
                recommendations.append({
                    "param": d_param,
                    "proposed_value": round(float(curr_d) * 1.15, 4),
                    "rationale": f"Velocity {axis} is underdamped and oscillatory (overshoot {overshoot*100:.0f}% and settling {settling:.1f}s > 3x τ={tau:.1f}s). Raise D-gain to improve damping.",
                    "confidence": confidence,
                })

    return {
        "axes": axes,
        "recommendations": recommendations,
        "notes": notes,
    }
