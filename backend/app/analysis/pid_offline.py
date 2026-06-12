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

import numpy as np
import pandas as pd

from .live_pid import LivePidEngine

_RESAMPLE_HZ = 100.0
_STEP_SCAN_SPAN_S = 0.1     # rate change measured across this span
_STEP_MIN_RAD_S = 0.3       # smallest commanded change worth analysing
_EVENT_SPACING_S = 1.0
_PRE_S, _POST_S = 0.5, 1.5
_MAX_EVENTS_PER_AXIS = 40
_OVERSHOOT_LIMIT = 0.25
_R_DEPLETION = 0.85
_GAIN_TRIM = 0.10           # proposed +/-10% per analysis pass

_SP_COLS = {"roll": "roll", "pitch": "pitch", "yaw": "yaw"}
_ACT_COLS = {"roll": "xyz[0]", "pitch": "xyz[1]", "yaw": "xyz[2]"}
_GYRO_COLS = {"roll": "gyro_rad[0]", "pitch": "gyro_rad[1]", "yaw": "gyro_rad[2]"}

# Axis -> rate P parameter, per safety class.
_RATE_P_PARAM = {
    "MULTIROTOR": {"roll": "MC_ROLLRATE_P", "pitch": "MC_PITCHRATE_P", "yaw": "MC_YAWRATE_P"},
    "VTOL": {"roll": "MC_ROLLRATE_P", "pitch": "MC_PITCHRATE_P", "yaw": None},
    "FIXED_WING": {"roll": "FW_RR_P", "pitch": "FW_PR_P", "yaw": "FW_YR_P"},
    "DELTA_WING": {"roll": "FW_RR_P", "pitch": "FW_PR_P", "yaw": None},
}


def _uniform(df: pd.DataFrame, col: str) -> tuple[np.ndarray, np.ndarray] | None:
    if col not in df.columns or len(df) < 10:
        return None
    t = df["timestamp"].to_numpy(np.float64) / 1e6
    x = df[col].to_numpy(np.float64)
    grid = np.arange(t[0], t[-1], 1.0 / _RESAMPLE_HZ)
    if len(grid) < 100:
        return None
    return grid, np.interp(grid, t, x)


def _find_step_indices(sp: np.ndarray) -> list[int]:
    """Indices where the commanded rate changes sharply."""
    span = int(_STEP_SCAN_SPAN_S * _RESAMPLE_HZ)
    change = np.abs(sp[span:] - sp[:-span])
    hot = np.flatnonzero(change > _STEP_MIN_RAD_S)
    if hot.size == 0:
        return []
    # Collapse runs of hot samples into one event each, keeping spacing.
    spacing = int(_EVENT_SPACING_S * _RESAMPLE_HZ)
    events, last = [], -spacing
    for i in hot:
        if i - last >= spacing:
            events.append(int(i))
            last = i
        if len(events) >= _MAX_EVENTS_PER_AXIS:
            break
    return events


def _analyze_axis(t: np.ndarray, sp: np.ndarray, act: np.ndarray) -> dict:
    pre = int(_PRE_S * _RESAMPLE_HZ)
    post = int(_POST_S * _RESAMPLE_HZ)
    taus, settlings, overshoots = [], [], []
    win_sp, win_act = [], []

    for i in _find_step_indices(sp):
        lo, hi = max(0, i - pre), min(len(sp), i + post)
        if hi - lo < pre + post // 2:
            continue
        res = LivePidEngine._step_response(t[lo:hi], sp[lo:hi], act[lo:hi])
        if res is None:
            continue
        if res["tau_s"] is not None:
            taus.append(res["tau_s"])
        if res["settling_s"] is not None:
            settlings.append(res["settling_s"])
        overshoots.append(res["overshoot"])
        win_sp.append(sp[lo:hi])
        win_act.append(act[lo:hi])

    out: dict = {"n_steps": len(overshoots)}
    if not overshoots:
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

    if stats.get("n_steps", 0) == 0:
        notes.append(f"{axis}: no usable step maneuvers in this log — fly "
                     f"sharp alternating {axis} inputs to enable analysis.")
        return recs, notes

    param = _RATE_P_PARAM.get(airframe_class or "", {}).get(axis)
    current = params.get(param) if param else None
    r = stats.get("r")
    tau, settling = stats.get("tau_s_median"), stats.get("settling_s_median")
    overshoot = stats.get("overshoot_max", 0.0)

    if r is not None and r < _R_DEPLETION:
        # Authority problem — gain changes would be treating the symptom.
        notes.append(
            f"{axis}: tracking correlation r={r:.2f} (<{_R_DEPLETION}) across "
            f"maneuvers. Check the actuator-saturation and vibration sections "
            f"before touching gains.")
        return recs, notes

    if param is None or current is None:
        notes.append(f"{axis}: stats computed but no tunable rate-P parameter "
                     f"resolved for airframe class {airframe_class}.")
        return recs, notes

    if overshoot > _OVERSHOOT_LIMIT:
        recs.append({
            "param": param,
            "proposed_value": round(float(current) * (1 - _GAIN_TRIM), 4),
            "rationale": (
                f"{axis} overshoot peaked at {overshoot*100:.0f}% of step "
                f"amplitude across {stats['n_steps']} maneuvers "
                f"(τ={tau}s). Reduce {param} ~{int(_GAIN_TRIM*100)}% "
                f"(or raise rate D) to damp the response."),
        })
    elif tau is not None and settling is not None and settling > 4 * tau:
        recs.append({
            "param": param,
            "proposed_value": round(float(current) * (1 + _GAIN_TRIM), 4),
            "rationale": (
                f"{axis} settles in {settling}s vs τ={tau}s (>{4}×τ) — "
                f"under-tuned. Increase {param} ~{int(_GAIN_TRIM*100)}% and "
                f"re-fly the test."),
        })
    else:
        notes.append(f"{axis}: tracking healthy (r={r}, τ={tau}s, "
                     f"overshoot {overshoot*100:.0f}%) — no change advised.")
    return recs, notes


def analyze_pid(rates_sp: pd.DataFrame | None,
                ang_vel: pd.DataFrame | None,
                sensor: pd.DataFrame | None,
                params: dict,
                airframe_class: str | None) -> dict:
    """Full-log rate-tracking analysis. Returns per-axis stats + proposals."""
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
        sp_u = _uniform(rates_sp, _SP_COLS[axis])
        act_u = _uniform(act_source, act_cols[axis])
        if sp_u is None or act_u is None:
            axes[axis] = {"n_steps": 0}
            continue
        # Overlap the two grids.
        t0 = max(sp_u[0][0], act_u[0][0])
        t1 = min(sp_u[0][-1], act_u[0][-1])
        grid = np.arange(t0, t1, 1.0 / _RESAMPLE_HZ)
        if len(grid) < 200:
            axes[axis] = {"n_steps": 0}
            continue
        sp = np.interp(grid, *sp_u)
        act = np.interp(grid, *act_u)

        stats = _analyze_axis(grid, sp, act)
        axes[axis] = stats
        recs, axis_notes = _suggest(axis, stats, airframe_class, params)
        recommendations += recs
        notes += axis_notes

    return {
        "airframe_class": airframe_class,
        "axes": axes,
        "recommendations": recommendations,
        "notes": notes,
    }
