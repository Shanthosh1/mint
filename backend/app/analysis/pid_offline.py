"""
Post-flight PID tracking analysis + gain suggestions.

Applies the same size-agnostic mathematics as the live engine
(live_pid._step_response / _tracking_metrics) to the full-rate log:

  setpoints : vehicle_rates_setpoint  (roll/pitch/yaw, rad/s)
  actuals   : vehicle_angular_velocity (xyz[0..2]) — falls back to raw
              sensor_combined gyro when the filtered topic wasn't logged.

Every commanded-rate step in the log is located, analysed in a
0.5 s-pre / 1.5 s-post window, and the per-axis aggregate (median tau,
median settling, worst overshoot, Pearson r over maneuver windows) is
converted into concrete parameter suggestions:

  overshoot > 25% of step      -> reduce rate P by 10%
  settling > 4*tau (sluggish)  -> increase rate P by 10%
  r < 0.85 during maneuvers    -> no gain change; authority/vibration
                                  problem flagged instead (raising gains
                                  on a saturated vehicle makes it worse)

Suggestions are *proposals only* — they flow through the safety registry
and the pilot's Approve & Write step exactly like live advice.
"""
from __future__ import annotations

import math
from typing import Optional
import numpy as np
import pandas as pd
import scipy.signal

from .live_pid import LivePidEngine, compute_coherence_in_band

_RESAMPLE_HZ = 100.0
_STEP_SCAN_SPAN_S = 0.25     # rate change measured across this span
_STEP_MIN_RAD_S = 0.15      # smallest commanded change worth analysing
_STEP_MIN_RAD_S_YAW = 0.08  # smallest commanded change worth analysing for yaw
_EVENT_SPACING_S = 1.0
_PRE_S, _POST_S = 0.5, 2.5
_MAX_EVENTS_PER_AXIS = 40
_OVERSHOOT_LIMIT = 0.25
_R_DEPLETION = 0.85
_GAIN_TRIM = 0.10           # proposed +/-10% per analysis pass
_SETTLE_BAND = 0.20

_SP_COLS = {"roll": "roll", "pitch": "pitch", "yaw": "yaw"}
_ACT_COLS = {"roll": "xyz[0]", "pitch": "xyz[1]", "yaw": "xyz[2]"}
_GYRO_COLS = {"roll": "gyro_rad[0]", "pitch": "gyro_rad[1]", "yaw": "gyro_rad[2]"}

# Axis -> rate parameter (P for MC, P for FW), per safety class.
_RATE_P_PARAM = {
    "MULTIROTOR": {"roll": "MC_ROLLRATE_P", "pitch": "MC_PITCHRATE_P", "yaw": "MC_YAWRATE_P"},
    "VTOL": {"roll": "MC_ROLLRATE_P", "pitch": "MC_PITCHRATE_P", "yaw": None},
    "FIXED_WING": {"roll": "FW_RR_P", "pitch": "FW_PR_P", "yaw": "FW_YR_P"},
    "DELTA_WING": {"roll": "FW_RR_P", "pitch": "FW_PR_P", "yaw": None},
}

# Axis -> attitude parameter (P for MC, TC for FW), per safety class.
_ATTITUDE_PARAM = {
    "MULTIROTOR": {"roll": "MC_ROLL_P", "pitch": "MC_PITCH_P", "yaw": "MC_YAW_P"},
    "VTOL": {"roll": "MC_ROLL_P", "pitch": "MC_PITCH_P", "yaw": "MC_YAW_P"},
    "FIXED_WING": {"roll": "FW_R_TC", "pitch": "FW_P_TC", "yaw": None},
    "DELTA_WING": {"roll": "FW_R_TC", "pitch": "FW_P_TC", "yaw": None},
}


from .step_utils import _uniform, _find_step_indices, _step_response_offline, compute_fd_gain, _extract_euler_angles



def _analyze_axis(t: np.ndarray, sp: np.ndarray, act: np.ndarray,
                  step_mask: np.ndarray | None = None,
                  derived: bool = False,
                  axis: str = "roll") -> dict:
    taus, settlings, overshoots = [], [], []
    win_sp, win_act = [], []
    step_results = []
    step_results_full = []

    if "(body)" in axis:
        min_amp = 0.087
    else:
        min_amp = _STEP_MIN_RAD_S_YAW if "yaw" in axis else _STEP_MIN_RAD_S

    indices = _find_step_indices(sp, threshold=min_amp)
    if step_mask is not None:
        indices = [i for i in indices if step_mask[i]]

    rej_counts = {"window_too_short": 0, "level_change": 0, "snr": 0, "ramp": 0, "too_small": 0}
    candidates_count = len(indices)
    last_step_t = None

    for i in indices:
        slice_pre = int(1.5 * _RESAMPLE_HZ)
        slice_post = int(2.5 * _RESAMPLE_HZ)
        lo, hi = max(0, i - slice_pre), min(len(sp), i + slice_post)
        
        res, reason = _step_response_offline(
            t[lo:hi], sp[lo:hi], act[lo:hi],
            min_amp=min_amp,
            last_step_t=last_step_t,
            derived=derived
        )
        
        last_step_t = float(t[i])
        
        if res is None:
            if reason in rej_counts:
                rej_counts[reason] += 1
            continue
            
        step_results_full.append(res)
        step_results.append({
            "t": t[lo:hi].tolist(),
            "sp": sp[lo:hi].tolist(),
            "act": act[lo:hi].tolist(),
            "tau_s": res["tau_s"],
            "overshoot": res["overshoot"],
            "oscillations": res["oscillations"],
            "settling_s": res["settling_s"],
            "amplitude": res["amplitude"],
            "ramped_input": res["ramped_input"],
            "noise_ratio": res["noise_ratio"],
            "confidence": res["confidence"],
            "t0": res["t0"],
            "sp_pre": res["sp_pre"],
            "plateau_duration_ok": res["plateau_duration_ok"],
        })
        
        if res.get("plateau_duration_ok", True):
            if res["tau_s"] is not None:
                taus.append(res["tau_s"])
            if res["settling_s"] is not None:
                settlings.append(res["settling_s"])
            overshoots.append(res["overshoot"])
            win_sp.append(sp[lo:hi])
            win_act.append(act[lo:hi])

    coherence = compute_coherence_in_band(t, sp, act)
    low_coherence = coherence < 0.6 if coherence is not None else False
    fd_gain = compute_fd_gain(t, sp, act)

    out: dict = {
        "n_steps": len(step_results),
        "n_steps_time_domain": len(overshoots),
        "candidates": candidates_count,
        "rejections": rej_counts,
        "steps": step_results,
        "derived_from_attitude": derived,
        "coherence": round(coherence, 3) if coherence is not None else None,
        "fd_gain": round(fd_gain, 3) if fd_gain is not None else None,
    }
    
    # Aggregate step flags:
    pre_step_motion = False
    ramped_input = False
    short_post_window = False
    osc_reliable = False
    
    if step_results_full:
        pre_step_motion = any(s["confidence"]["flags"]["pre_step_motion"] for s in step_results_full)
        ramped_input = any(s["confidence"]["flags"]["ramped_input"] for s in step_results_full)
        short_post_window = any(s["confidence"]["flags"]["short_post_window"] for s in step_results_full)
        osc_reliable = any(s["confidence"]["flags"]["osc_reliable"] for s in step_results_full)
        
        # Calculate axis score using min-of-components:
        score = 1.0
        if low_coherence:
            score = min(score, 0.5)
        
        max_nr = max(s.get("noise_ratio", 0.0) for s in step_results_full)
        if max_nr > 0.25:
            score = min(score, 0.3)
        elif max_nr > 0.15:
            score = min(score, 0.6)
            
        if ramped_input:
            score = min(score, 0.8)
            
        if short_post_window:
            score = min(score, 0.7)
            
        if derived:
            score = min(score, 0.5)
    else:
        score = max(0.0, min(1.0, coherence)) if coherence is not None else 0.0

    confidence = {
        "score": round(score, 2),
        "flags": {
            "pre_step_motion": pre_step_motion,
            "ramped_input": ramped_input,
            "low_coherence": low_coherence,
            "short_post_window": short_post_window,
            "derived_from_attitude": derived,
            "osc_reliable": osc_reliable,
        }
    }
    out["confidence"] = confidence

    if not overshoots:
        out.update({
            "tau_s_median": None,
            "settling_s_median": None,
            "overshoot_max": None,
            "r": None,
            "nrmse": None,
        })
        return out

    # Pearson/NRMSE over maneuver windows only — including quiet hover
    # would inflate r artificially (both signals near zero agree trivially).
    tm = LivePidEngine._tracking_metrics(
        np.concatenate(win_sp), np.concatenate(win_act))
    out.update({
        "tau_s_median": round(float(np.median(taus)), 3) if taus else None,
        "settling_s_median": round(float(np.median(settlings)), 3) if settlings else None,
        "overshoot_max": round(float(np.max(overshoots)), 3),
        "r": tm["r"],
        "nrmse": tm["nrmse"],
    })
    return out



def _suggest(axis: str, stats: dict, airframe_class: str | None,
             params: dict) -> tuple[list[dict], list[str]]:
    """Turn one axis' aggregate stats into proposals and/or notes."""
    recs: list[dict] = []
    notes: list[str] = []

    # Get details
    loop_type = "rate"
    base_axis = axis
    if " (rate)" in axis:
        base_axis = axis.replace(" (rate)", "")
        loop_type = "rate"
    elif " (body)" in axis:
        base_axis = axis.replace(" (body)", "")
        loop_type = "body"

    if loop_type == "body":
        param = _ATTITUDE_PARAM.get(airframe_class or "", {}).get(base_axis)
    else:
        param = _RATE_P_PARAM.get(airframe_class or "", {}).get(base_axis)
        
    current = params.get(param) if param else None
    
    n_steps = stats.get("n_steps", 0)
    n_steps_td = stats.get("n_steps_time_domain", 0)
    coherence = stats.get("coherence")
    fd_gain = stats.get("fd_gain")
    coherence_ok = coherence is not None and coherence > 0.6

    candidates = stats.get("candidates", 0)
    # If there are candidates but all were rejected, show the rejections accounting note
    if n_steps == 0 and candidates > 0:
        rejections = stats.get("rejections", {})
        derived_str = " (derived from attitude)" if stats.get("derived_from_attitude") else ""
        parts = []
        if rejections.get("too_small", 0) > 0:
            parts.append(f"{rejections['too_small']} too small")
        if rejections.get("snr", 0) > 0:
            parts.append(f"{rejections['snr']} noisy/low SNR")
        if rejections.get("ramp", 0) > 0:
            parts.append(f"{rejections['ramp']} slow/ramped")
        
        window_count = rejections.get("window_too_short", 0) + rejections.get("window", 0)
        if window_count > 0:
            parts.append(f"{window_count} window too short")
        if rejections.get("level_change", 0) > 0:
            parts.append(f"{rejections['level_change']} level change")
            
        if not parts:
            parts.append("rejected")
            
        reason_msg = ", ".join(parts)
        notes.append(
            f"{axis}: {candidates} step candidates detected{derived_str} but all were rejected, "
            f"predominantly because they were {reason_msg}. "
            f"Ensure inputs are sharp, distinct, and performed from a steady hover/flight state."
        )
        return recs, notes

    # If no steps were reliable for time-domain and coherence is low, report that coherence is too low
    if n_steps_td == 0 and not coherence_ok:
        coherence_val = f"{coherence:.2f}" if coherence is not None else "N/A"
        notes.append(f"{axis}: No steps detected. Coherence too low ({coherence_val} ≤ 0.6). Fly sharp inputs or use Live Dashboard.")
        return recs, notes

    # Inform the user when frequency fallback is used due to step details
    if n_steps == 0 and coherence_ok:
        notes.append(f"{axis}: no step maneuvers detected — frequency-domain fallback used for gain advice. Fly sharp alternating {axis} inputs to enable step-response envelope plots.")
    elif n_steps > 0 and n_steps_td == 0:
        notes.append(f"{axis}: steps were too short/pulsed to reliably measure overshoot and settling in the time domain. Suggest frequency analysis for overshoot.")

    # 2. Process recommendations
    if n_steps_td > 0:
        # Standard time-domain analysis path
        r = stats.get("r")
        if r is not None and r < _R_DEPLETION:
            notes.append(
                f"{axis}: tracking correlation r={r:.2f} (<{_R_DEPLETION}) across "
                f"maneuvers. Check the actuator-saturation and vibration sections "
                f"before touching gains.")
            return recs, notes

        if param is None or current is None:
            notes.append(f"{axis}: stats computed but no tunable parameter "
                         f"resolved for airframe class {airframe_class}.")
            return recs, notes

        tau, settling = stats.get("tau_s_median"), stats.get("settling_s_median")
        overshoot = stats.get("overshoot_max", 0.0)
        is_time_constant = param.endswith("_TC")

        if overshoot > _OVERSHOOT_LIMIT:
            if is_time_constant:
                proposed = round(float(current) + 0.05, 3)
                rationale = f"{axis} overshoot peaked at {overshoot*100:.0f}% — slow down by raising time constant."
            else:
                proposed = round(float(current) * (1 - _GAIN_TRIM), 4)
                rationale = f"{axis} overshoot peaked at {overshoot*100:.0f}% of step amplitude — soften the P gain."
                
            recs.append({
                "param": param,
                "proposed_value": proposed,
                "rationale": rationale,
                "confidence": stats.get("confidence"),
            })
        elif tau is not None and settling is not None and settling > 4 * tau:
            if is_time_constant:
                proposed = round(max(0.1, float(current) - 0.05), 3)
                rationale = f"{axis} settles in {settling}s vs τ={tau}s — speed up by tightening time constant."
            else:
                proposed = round(float(current) * (1 + _GAIN_TRIM), 4)
                rationale = f"{axis} settles in {settling}s vs τ={tau}s (>{4}×τ) — raise the P gain."
                
            recs.append({
                "param": param,
                "proposed_value": proposed,
                "rationale": rationale,
                "confidence": stats.get("confidence"),
            })
        else:
            notes.append(f"{axis}: tracking healthy (r={r}, τ={tau}s, "
                         f"overshoot {overshoot*100:.0f}%) — no change advised.")
    else:
        # Fallback frequency-domain gain path
        if coherence_ok:
            if fd_gain is not None:
                if param is None or current is None:
                    notes.append(f"{axis}: frequency-domain analysis computed (gain={fd_gain:.2f}) but no tunable parameter resolved for airframe class {airframe_class}.")
                    return recs, notes

                is_time_constant = param.endswith("_TC")
                if fd_gain < 0.85:
                    if is_time_constant:
                        proposed = round(max(0.1, float(current) - 0.05), 3)
                        rationale = f"{axis} low-frequency gain is sluggish ({fd_gain:.2f} < 0.85) — speed up by lowering time constant."
                    else:
                        proposed = round(float(current) * (1 + _GAIN_TRIM), 4)
                        rationale = f"{axis} low-frequency gain is sluggish ({fd_gain:.2f} < 0.85) — raise the P gain."
                    
                    recs.append({
                        "param": param,
                        "proposed_value": proposed,
                        "rationale": rationale,
                        "confidence": stats.get("confidence"),
                    })
                elif fd_gain > 1.15:
                    if is_time_constant:
                        proposed = round(float(current) + 0.05, 3)
                        rationale = f"{axis} low-frequency gain is elevated ({fd_gain:.2f} > 1.15) — slow down by raising time constant."
                    else:
                        proposed = round(float(current) * (1 - _GAIN_TRIM), 4)
                        rationale = f"{axis} low-frequency gain is elevated ({fd_gain:.2f} > 1.15) — soften the P gain."
                    
                    recs.append({
                        "param": param,
                        "proposed_value": proposed,
                        "rationale": rationale,
                        "confidence": stats.get("confidence"),
                    })
                else:
                    notes.append(f"{axis}: frequency-domain tracking healthy (gain={fd_gain:.2f}, coherence={coherence:.2f}) — no change advised.")
            else:
                notes.append(f"{axis}: no step maneuvers detected and frequency-domain gain could not be computed.")

    return recs, notes


def analyze_pid(rates_sp: pd.DataFrame | None,
                ang_vel: pd.DataFrame | None,
                sensor: pd.DataFrame | None,
                params: dict,
                airframe_class: str | None,
                status: pd.DataFrame | None = None,
                vtol_status: pd.DataFrame | None = None,
                att_sp: pd.DataFrame | None = None,
                vehicle_att: pd.DataFrame | None = None) -> dict:
    """Full-log rate-tracking and attitude-tracking analysis. Returns per-axis stats + proposals."""
    if rates_sp is None:
        return {"skipped": "vehicle_rates_setpoint not logged — enable a "
                           "higher logging profile (SDLOG_PROFILE) for PID analysis"}

    # Prefer the filtered angular velocity; fall back to raw gyro.
    act_source, act_cols = ang_vel, _ACT_COLS
    if act_source is None or not all(c in act_source.columns for c in _ACT_COLS.values()):
        act_source, act_cols = sensor, _GYRO_COLS
    if act_source is None:
        return {"skipped": "no angular velocity source in log "
                           "(vehicle_angular_velocity / sensor_combined)"}

    axes: dict[str, dict] = {}
    recommendations: list[dict] = []
    notes: list[str] = []

    for axis in ("roll", "pitch", "yaw"):
        # --- 1. Rate Loop ---
        rate_axis = f"{axis} (rate)"
        sp_u_rate = _uniform(rates_sp, _SP_COLS[axis])
        act_u_rate = _uniform(act_source, act_cols[axis])

        derived_rate = False
        has_data_rate = (sp_u_rate is not None and act_u_rate is not None)

        if axis == "yaw" and att_sp is not None:
            need_fallback = not has_data_rate
            if has_data_rate:
                t0 = max(sp_u_rate[0][0], act_u_rate[0][0])
                t1 = min(sp_u_rate[0][-1], act_u_rate[0][-1])
                grid = np.arange(t0, t1, 1.0 / _RESAMPLE_HZ)
                if len(grid) >= 200:
                    sp_dry = np.interp(grid, *sp_u_rate)
                    if airframe_class == "VTOL":
                        grid_vtype = None
                        if status is not None and "vehicle_type" in status.columns:
                            t_status = status["timestamp"].to_numpy(np.float64) / 1e6
                            vtype = status["vehicle_type"].to_numpy(np.float64)
                            if len(t_status) > 0:
                                grid_vtype = np.round(np.interp(grid, t_status, vtype))
                        elif vtol_status is not None and "vtol_in_rw_mode" in vtol_status.columns:
                            t_vtol = vtol_status["timestamp"].to_numpy(np.float64) / 1e6
                            rw_mode = vtol_status["vtol_in_rw_mode"].to_numpy(np.float64)
                            if len(t_vtol) > 0:
                                vtype = np.where(rw_mode > 0.5, 1.0, 2.0)
                                grid_vtype = np.round(np.interp(grid, t_vtol, vtype))

                        if grid_vtype is None:
                            mc_mask = None
                            fw_mask = np.zeros_like(grid, dtype=bool)
                        else:
                            mc_mask = (grid_vtype == 1)
                            fw_mask = (grid_vtype == 2)

                        mc_indices = _find_step_indices(sp_dry, threshold=_STEP_MIN_RAD_S_YAW)
                        if mc_mask is not None:
                            mc_indices = [idx for idx in mc_indices if mc_mask[idx]]
                        fw_indices = _find_step_indices(sp_dry, threshold=_STEP_MIN_RAD_S_YAW)
                        if fw_mask is not None:
                            fw_indices = [idx for idx in fw_indices if fw_mask[idx]]

                        n_steps = len(mc_indices) + len(fw_indices)
                    else:
                        n_steps = len(_find_step_indices(sp_dry, threshold=_STEP_MIN_RAD_S_YAW))

                    if n_steps < 3:
                        need_fallback = True
                else:
                    need_fallback = True

            if need_fallback:
                att_euler = _extract_euler_angles(att_sp)
                yaw_att = att_euler["yaw"] if att_euler is not None else None
                if yaw_att is not None and len(att_sp) >= 10:
                    t_att = att_sp["timestamp"].to_numpy(np.float64) / 1e6
                    yaw_unwrapped = np.unwrap(yaw_att)
                    grid_att = np.arange(t_att[0], t_att[-1], 1.0 / _RESAMPLE_HZ)
                    if len(grid_att) >= 10:
                        yaw_uniform = np.interp(grid_att, t_att, yaw_unwrapped)
                        yaw_rate = np.gradient(yaw_uniform, 1.0 / _RESAMPLE_HZ)
                        sp_u_rate = (grid_att, yaw_rate)
                        derived_rate = True
                        has_data_rate = (act_u_rate is not None)

        if has_data_rate:
            t0 = max(sp_u_rate[0][0], act_u_rate[0][0])
            t1 = min(sp_u_rate[0][-1], act_u_rate[0][-1])
            grid = np.arange(t0, t1, 1.0 / _RESAMPLE_HZ)
            if len(grid) >= 200:
                sp = np.interp(grid, *sp_u_rate)
                act = np.interp(grid, *act_u_rate)

                grid_vtype = None
                if status is not None and "vehicle_type" in status.columns:
                    t_status = status["timestamp"].to_numpy(np.float64) / 1e6
                    vtype = status["vehicle_type"].to_numpy(np.float64)
                    if len(t_status) > 0:
                        grid_vtype = np.round(np.interp(grid, t_status, vtype))
                elif vtol_status is not None and "vtol_in_rw_mode" in vtol_status.columns:
                    t_vtol = vtol_status["timestamp"].to_numpy(np.float64) / 1e6
                    rw_mode = vtol_status["vtol_in_rw_mode"].to_numpy(np.float64)
                    if len(t_vtol) > 0:
                        vtype = np.where(rw_mode > 0.5, 1.0, 2.0)
                        grid_vtype = np.round(np.interp(grid, t_vtol, vtype))

                if airframe_class == "VTOL":
                    if grid_vtype is None:
                        notes.append(f"VTOL split failed (no vehicle_status or vtol_vehicle_status data). Treating all steps as MC-only for {rate_axis}.")
                        mc_mask = None
                        fw_mask = np.zeros_like(grid, dtype=bool)
                    else:
                        mc_mask = (grid_vtype == 1)
                        fw_mask = (grid_vtype == 2)

                    mc_stats = _analyze_axis(grid, sp, act, mc_mask, derived=derived_rate, axis=rate_axis)
                    fw_stats = _analyze_axis(grid, sp, act, fw_mask, derived=derived_rate, axis=rate_axis)

                    axes[rate_axis] = {
                        "mc": mc_stats,
                        "fw": fw_stats,
                        "n_steps": mc_stats.get("n_steps", 0) + fw_stats.get("n_steps", 0)
                    }
                    mc_recs, mc_notes = _suggest(rate_axis, mc_stats, "MULTIROTOR", params)
                    fw_recs, fw_notes = _suggest(rate_axis, fw_stats, "FIXED_WING", params)

                    for r in mc_recs:
                        r["rationale"] = f"MC mode: {r['rationale']}"
                    for r in fw_recs:
                        r["rationale"] = f"FW mode: {r['rationale']}"

                    mc_notes = [f"{n.split(':', 1)[0]} (MC):{n.split(':', 1)[1]}" if ":" in n else f"(MC) {n}" for n in mc_notes]
                    fw_notes = [f"{n.split(':', 1)[0]} (FW):{n.split(':', 1)[1]}" if ":" in n else f"(FW) {n}" for n in fw_notes]

                    recommendations += mc_recs + fw_recs
                    notes += mc_notes + fw_notes
                else:
                    stats = _analyze_axis(grid, sp, act, None, derived=derived_rate, axis=rate_axis)
                    axes[rate_axis] = stats
                    recs, axis_notes = _suggest(rate_axis, stats, airframe_class, params)
                    recommendations += recs
                    notes += axis_notes
            else:
                axes[rate_axis] = {"n_steps": 0}
        else:
            axes[rate_axis] = {"n_steps": 0}

        # --- 2. Body Attitude Loop ---
        body_axis = f"{axis} (body)"
        has_data_body = False
        if att_sp is not None and vehicle_att is not None:
            att_sp_euler = _extract_euler_angles(att_sp)
            vehicle_att_euler = _extract_euler_angles(vehicle_att)
            if att_sp_euler is not None and vehicle_att_euler is not None:
                att_sp_copy = att_sp.copy()
                vehicle_att_copy = vehicle_att.copy()

                # Unwrap all angles to prevent wrap-around boundary discontinuities
                sp_angles = np.unwrap(att_sp_euler[axis])
                act_angles = np.unwrap(vehicle_att_euler[axis])

                att_sp_copy["euler_angle"] = sp_angles
                vehicle_att_copy["euler_angle"] = act_angles

                sp_u_body = _uniform(att_sp_copy, "euler_angle")
                act_u_body = _uniform(vehicle_att_copy, "euler_angle")

                has_data_body = (sp_u_body is not None and act_u_body is not None)

        if has_data_body:
            t0 = max(sp_u_body[0][0], act_u_body[0][0])
            t1 = min(sp_u_body[0][-1], act_u_body[0][-1])
            grid = np.arange(t0, t1, 1.0 / _RESAMPLE_HZ)
            if len(grid) >= 200:
                sp = np.interp(grid, *sp_u_body)
                act = np.interp(grid, *act_u_body)

                grid_vtype = None
                if status is not None and "vehicle_type" in status.columns:
                    t_status = status["timestamp"].to_numpy(np.float64) / 1e6
                    vtype = status["vehicle_type"].to_numpy(np.float64)
                    if len(t_status) > 0:
                        grid_vtype = np.round(np.interp(grid, t_status, vtype))
                elif vtol_status is not None and "vtol_in_rw_mode" in vtol_status.columns:
                    t_vtol = vtol_status["timestamp"].to_numpy(np.float64) / 1e6
                    rw_mode = vtol_status["vtol_in_rw_mode"].to_numpy(np.float64)
                    if len(t_vtol) > 0:
                        vtype = np.where(rw_mode > 0.5, 1.0, 2.0)
                        grid_vtype = np.round(np.interp(grid, t_vtol, vtype))

                if airframe_class == "VTOL":
                    if grid_vtype is None:
                        notes.append(f"VTOL split failed (no vehicle_status or vtol_vehicle_status data). Treating all steps as MC-only for {body_axis}.")
                        mc_mask = None
                        fw_mask = np.zeros_like(grid, dtype=bool)
                    else:
                        mc_mask = (grid_vtype == 1)
                        fw_mask = (grid_vtype == 2)

                    mc_stats = _analyze_axis(grid, sp, act, mc_mask, derived=False, axis=body_axis)
                    fw_stats = _analyze_axis(grid, sp, act, fw_mask, derived=False, axis=body_axis)

                    axes[body_axis] = {
                        "mc": mc_stats,
                        "fw": fw_stats,
                        "n_steps": mc_stats.get("n_steps", 0) + fw_stats.get("n_steps", 0)
                    }
                    mc_recs, mc_notes = _suggest(body_axis, mc_stats, "MULTIROTOR", params)
                    fw_recs, fw_notes = _suggest(body_axis, fw_stats, "FIXED_WING", params)

                    for r in mc_recs:
                        r["rationale"] = f"MC mode: {r['rationale']}"
                    for r in fw_recs:
                        r["rationale"] = f"FW mode: {r['rationale']}"

                    mc_notes = [f"{n.split(':', 1)[0]} (MC):{n.split(':', 1)[1]}" if ":" in n else f"(MC) {n}" for n in mc_notes]
                    fw_notes = [f"{n.split(':', 1)[0]} (FW):{n.split(':', 1)[1]}" if ":" in n else f"(FW) {n}" for n in fw_notes]

                    recommendations += mc_recs + fw_recs
                    notes += mc_notes + fw_notes
                else:
                    stats = _analyze_axis(grid, sp, act, None, derived=False, axis=body_axis)
                    axes[body_axis] = stats
                    recs, axis_notes = _suggest(body_axis, stats, airframe_class, params)
                    recommendations += recs
                    notes += axis_notes
            else:
                axes[body_axis] = {"n_steps": 0}
        else:
            axes[body_axis] = {"n_steps": 0}

    return {
        "airframe_class": airframe_class,
        "axes": axes,
        "recommendations": recommendations,
        "notes": notes,
    }
