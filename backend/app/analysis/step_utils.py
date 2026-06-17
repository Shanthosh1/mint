"""
Shared utilities for post-flight step response and tracking analysis.
"""
from __future__ import annotations

import math
from typing import Optional
import numpy as np
import pandas as pd
import scipy.signal

from .live_pid import compute_coherence_in_band

_RESAMPLE_HZ = 100.0
_SETTLE_BAND = 0.20


def _uniform(df: pd.DataFrame, col: str, resample_hz: float = _RESAMPLE_HZ) -> tuple[np.ndarray, np.ndarray] | None:
    if col not in df.columns or len(df) < 10:
        return None
    t = df["timestamp"].to_numpy(np.float64) / 1e6
    x = df[col].to_numpy(np.float64)
    grid = np.arange(t[0], t[-1], 1.0 / resample_hz)
    if len(grid) < 100:
        return None
    return grid, np.interp(grid, t, x)


def _find_step_indices(sp: np.ndarray, threshold: float,
                       resample_hz: float = _RESAMPLE_HZ,
                       step_scan_span_s: float = 0.25,
                       event_spacing_s: float = 1.0,
                       max_events: int = 40) -> list[int]:
    """Indices where the commanded setpoint changes value sharply."""
    span = int(step_scan_span_s * resample_hz)
    if len(sp) <= span:
        return []
    change = np.abs(sp[span:] - sp[:-span])
    hot = np.flatnonzero(change > threshold)
    if hot.size == 0:
        return []
    # Collapse runs of hot samples into one event each, keeping spacing.
    spacing = int(event_spacing_s * resample_hz)
    events, last = [], -spacing
    for i in hot:
        if i - last >= spacing:
            events.append(int(i))
            last = i
        if len(events) >= max_events:
            break
    return events


def compute_fd_gain(t: np.ndarray, x: np.ndarray, y: np.ndarray) -> Optional[float]:
    """
    Compute frequency-domain closed-loop transfer function gain |H(f)| = |P_xy(f) / P_xx(f)|
    averaged over the low-frequency band [0.5, 1.0] Hz.
    """
    if len(t) < 20 or (t[-1] - t[0]) <= 0:
        return None
    dt = np.diff(t)
    mean_dt = np.mean(dt) if dt.size > 0 else 0.01
    if mean_dt <= 0:
        return None
    fs = 1.0 / mean_dt
    try:
        x_detrend = scipy.signal.detrend(x)
        y_detrend = scipy.signal.detrend(y)
    except Exception:
        return None
    nperseg = min(len(x), max(64, len(x) // 4))
    if nperseg < 8:
        return None
    try:
        f_welch, Pxx = scipy.signal.welch(x_detrend, fs=fs, nperseg=nperseg)
        f_csd, Pxy = scipy.signal.csd(x_detrend, y_detrend, fs=fs, nperseg=nperseg)
    except Exception:
        return None
    if len(Pxx) != len(Pxy):
        return None
    H = np.zeros_like(Pxy, dtype=complex)
    valid_mask = Pxx > 1e-12
    H[valid_mask] = Pxy[valid_mask] / Pxx[valid_mask]
    gain = np.abs(H)
    band_mask = (f_welch >= 0.5) & (f_welch <= 1.0)
    if not band_mask.any():
        closest_idx = np.argmin(np.abs(f_welch - 0.75))
        return float(gain[closest_idx])
    return float(np.mean(gain[band_mask]))


def _step_response_offline(t: np.ndarray, sp: np.ndarray, act: np.ndarray,
                           min_amp: float = 0.15,
                           last_step_t: Optional[float] = None,
                           derived: bool = False,
                           sp_stable_window_s: float = 0.20,
                           osc_window_s: float = 2.0,
                           resample_hz: float = _RESAMPLE_HZ) -> tuple[Optional[dict], Optional[str]]:
    """
    Offline step response extraction.
    Returns (res_dict, rejection_reason).
    """
    if len(sp) < 20:
        return None, "window_too_short"
    i = int(np.argmax(np.abs(np.diff(sp))))
    t0_temp = t[i + 1]

    # Go backwards from i to find the start of the stick movement
    diff_sp = np.diff(sp)
    step_dir = np.sign(diff_sp[i]) if diff_sp[i] != 0 else 1.0
    
    i_start = i
    while i_start > 0:
        if np.sign(diff_sp[i_start - 1]) != step_dir or abs(diff_sp[i_start - 1]) < 0.005:
            break
        i_start -= 1
        
    t_start = t[i_start]
    pre_window = 0.3 if (last_step_t is not None and (t_start - last_step_t) < 3.0) else 1.0
    pre_mask = (t >= t_start - pre_window) & (t < t_start)
    
    if pre_mask.sum() < 3:
        return None, "window_too_short"

    sp_pre = float(np.mean(sp[pre_mask]))
    
    # 1. Temporary step response calculation to locate 5% deflection point
    post_mask_temp = t >= t0_temp
    sp_post_vals_temp = sp[post_mask_temp]
    if len(sp_post_vals_temp) < 8:
        return None, "window_too_short"
        
    abs_deflections_temp = np.abs(sp_post_vals_temp - sp_pre)
    idx_peak_temp = int(np.argmax(abs_deflections_temp))
    peak_def_temp = abs_deflections_temp[idx_peak_temp]
    
    # Apply temporary 50% decay window truncation to avoid pulse return-to-zero in sp_post estimation
    cut_temp = None
    for idx in range(idx_peak_temp, len(abs_deflections_temp)):
        if abs_deflections_temp[idx] < 0.5 * peak_def_temp:
            cut_temp = idx
            break
    if cut_temp is not None:
        sp_post_vals_temp = sp_post_vals_temp[:cut_temp]
        
    if len(sp_post_vals_temp) < 3:
        return None, "window_too_short"
        
    settled_start_temp = max(0, int(0.7 * len(sp_post_vals_temp)))
    sp_post_temp = float(np.mean(sp_post_vals_temp[settled_start_temp:]))
    amp_temp = sp_post_temp - sp_pre
    
    # 5% deflection transition start detection
    idx_5pct = i_start
    for idx in range(i_start, len(sp)):
        if abs(sp[idx] - sp_pre) >= 0.05 * abs(amp_temp):
            idx_5pct = idx
            break
    t0 = t[idx_5pct]

    # Final post window definition
    post_mask = t >= t0
    sp_post_vals = sp[post_mask]
    act_post_vals = act[post_mask]
    t_post_vals = t[post_mask]
    
    if len(sp_post_vals) < 8:
        return None, "window_too_short"

    # 50% decay pulse-width window truncation
    abs_deflections = np.abs(sp_post_vals - sp_pre)
    idx_peak = int(np.argmax(abs_deflections))
    peak_def = abs_deflections[idx_peak]
    
    cut = None
    for idx in range(idx_peak, len(abs_deflections)):
        if abs_deflections[idx] < 0.5 * peak_def:
            cut = idx
            break
            
    if cut is not None:
        sp_post_vals = sp_post_vals[:cut]
        act_post_vals = act_post_vals[:cut]
        t_post_vals = t_post_vals[:cut]
        
    if len(sp_post_vals) < 8:
        return None, "window_too_short"

    # Find where the setpoint deflection drops below 90% of peak deflection after the peak
    # to isolate the plateau from any return-to-zero decay
    abs_deflections = np.abs(sp_post_vals - sp_pre)
    idx_decay_start = len(abs_deflections)
    for idx in range(idx_peak, len(abs_deflections)):
        if abs_deflections[idx] < 0.90 * peak_def:
            idx_decay_start = idx
            break

    # Standard 30% settled plateau average calculated over the plateau region
    plateau_len = idx_decay_start - idx_peak
    settled_start_idx = idx_peak + max(0, int(0.7 * plateau_len))
    sp_post = float(np.mean(sp_post_vals[settled_start_idx:max(settled_start_idx + 1, idx_decay_start)]))
    amp = sp_post - sp_pre

    # Ensure sp_post is a meaningful step away from sp_pre
    if abs(amp) < 0.8 * min_amp:
        return None, "too_small"
        
    # Use stable early portion for initial amplitude in the ramp check
    stable_start_t = t0 + 0.1
    stable_end_t = t0 + 0.1 + sp_stable_window_s
    stable_idx = (t_post_vals >= stable_start_t) & (t_post_vals < stable_end_t)
    if stable_idx.sum() >= 5:
        sp_early = float(np.mean(sp_post_vals[stable_idx]))
    else:
        sp_early = float(np.mean(sp_post_vals[:max(1, len(sp_post_vals)//4)]))
    amp_initial = sp_early - sp_pre

    # Circularity-safe reference amp ramp check:
    pre_span = np.ptp(sp[pre_mask]) if pre_mask.sum() > 2 else 0.0
    reference_amp = max(abs(amp_initial), pre_span + min_amp)
    post_sp_std = np.std(sp_post_vals)
    
    if post_sp_std > 0.7 * reference_amp:
        return None, "ramp"

    # SNR gate:
    act_noise = float(np.std(act[pre_mask])) or 1e-6
    if abs(amp) < 2.0 * act_noise:
        return None, "snr"
        
    noise_ratio = act_noise / abs(amp)

    # Oscillation window
    osc_mask = (t >= t0) & (t <= t0 + osc_window_s)
    osc_act = act[osc_mask]
    if len(osc_act) < int(1.0 * resample_hz):
        oscillations = None
    else:
        # Zero crossings of osc_act relative to sp_post
        resid = osc_act - sp_post
        crossing_noise = min(act_noise, 0.03) if act_noise > 0.05 else act_noise
        sig = resid[np.abs(resid) > max(0.03 * abs(amp), 2 * crossing_noise, 0.02)]
        crossings = int(np.count_nonzero(np.diff(np.sign(sig)) != 0)) if sig.size > 1 else 0
        oscillations = crossings

    ta, aa = t_post_vals, act_post_vals
    sign = 1.0 if amp > 0 else -1.0

    # Check that we started below the 63.2% threshold relative to the step direction
    started_below = sign * (aa[0] - (sp_pre + 0.632 * amp)) < 0
    crossed = sign * (aa - (sp_pre + 0.632 * amp)) >= 0
    crossed_90 = sign * (aa - (sp_pre + 0.90 * amp)) >= 0

    post_duration = float(ta[-1] - t0) if len(ta) > 0 else 0.0

    if started_below and crossed.any() and crossed_90.any():
        tau = float(ta[np.argmax(crossed)] - t0)
        plateau_duration_ok = post_duration >= 4 * tau
    elif not started_below:
        # Instantaneous/very fast response
        tau = 0.0
        plateau_duration_ok = True
    else:
        tau = None
        plateau_duration_ok = False

    peak = float(np.max(sign * (aa - sp_post)))
    overshoot = max(0.0, peak / abs(amp))

    outside = np.abs(aa - sp_post) > _SETTLE_BAND * abs(amp)
    if np.all(outside):
        settling = None
        settled = False
    else:
        settling = float(ta[np.where(outside)[0][-1]] - t0) if outside.any() else 0.0
        settled = not bool(outside[-1])

    ramped_input = not plateau_duration_ok
    limitations = None
    if ramped_input:
        limitations = "short pulse/ramp — τ & overshoot may be unreliable"

    pre_step_motion = noise_ratio > 0.15
    short_post_window = post_duration < 1.0
    osc_reliable = post_duration >= 1.0 and oscillations is not None
    
    score = 1.0
    if noise_ratio > 0.25:
        score = min(score, 0.3)
    elif noise_ratio > 0.15:
        score = min(score, 0.6)
        
    if ramped_input:
        score = min(score, 0.8)
        
    if short_post_window:
        score = min(score, 0.7)
        
    if derived:
        score = min(score, 0.5)
        
    confidence = {
        "score": score,
        "flags": {
            "pre_step_motion": pre_step_motion,
            "ramped_input": ramped_input,
            "low_coherence": False, # filled at axis level
            "short_post_window": short_post_window,
            "derived_from_attitude": derived,
            "osc_reliable": osc_reliable,
        }
    }

    res_dict = {
        "tau_s": round(tau, 3) if tau is not None else None,
        "settling_s": round(settling, 3) if settled else None,
        "overshoot": round(overshoot, 3),
        "oscillations": oscillations,
        "step_amp_deg_s": round(math.degrees(amp), 1),
        "amplitude": amp,
        "post_sp_std": post_sp_std,
        "ramped_input": ramped_input,
        "limitations": limitations,
        "noise_ratio": noise_ratio,
        "t_start": float(t_start),
        "confidence": confidence,
        "t0": float(t0),
        "sp_pre": float(sp_pre),
        "plateau_duration_ok": plateau_duration_ok,
    }
    return res_dict, None


def _extract_euler_angles(df: pd.DataFrame | None) -> dict[str, np.ndarray] | None:
    if df is None or len(df) == 0:
        return None

    # Check for direct Euler columns first (common in some setpoint logs)
    euler_cols_body = ["roll_body", "pitch_body", "yaw_body"]
    if any(c in df.columns for c in euler_cols_body):
        return {
            "roll": df["roll_body"].to_numpy(np.float64) if "roll_body" in df.columns else np.zeros(len(df), dtype=np.float64),
            "pitch": df["pitch_body"].to_numpy(np.float64) if "pitch_body" in df.columns else np.zeros(len(df), dtype=np.float64),
            "yaw": df["yaw_body"].to_numpy(np.float64) if "yaw_body" in df.columns else np.zeros(len(df), dtype=np.float64)
        }

    euler_cols_raw = ["roll", "pitch", "yaw"]
    if any(c in df.columns for c in euler_cols_raw):
        return {
            "roll": df["roll"].to_numpy(np.float64) if "roll" in df.columns else np.zeros(len(df), dtype=np.float64),
            "pitch": df["pitch"].to_numpy(np.float64) if "pitch" in df.columns else np.zeros(len(df), dtype=np.float64),
            "yaw": df["yaw"].to_numpy(np.float64) if "yaw" in df.columns else np.zeros(len(df), dtype=np.float64)
        }

    # Check for quaternion columns
    # Setpoint quaternions usually prefix with q_d
    q_d_cols = ["q_d[0]", "q_d[1]", "q_d[2]", "q_d[3]"]
    if all(c in df.columns for c in q_d_cols):
        w = df["q_d[0]"].to_numpy(np.float64)
        x = df["q_d[1]"].to_numpy(np.float64)
        y = df["q_d[2]"].to_numpy(np.float64)
        z = df["q_d[3]"].to_numpy(np.float64)
        
        roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0))
        yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        return {"roll": roll, "pitch": pitch, "yaw": yaw}

    # Actual quaternions usually prefix with q
    q_cols = ["q[0]", "q[1]", "q[2]", "q[3]"]
    if all(c in df.columns for c in q_cols):
        w = df["q[0]"].to_numpy(np.float64)
        x = df["q[1]"].to_numpy(np.float64)
        y = df["q[2]"].to_numpy(np.float64)
        z = df["q[3]"].to_numpy(np.float64)
        
        roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0))
        yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        return {"roll": roll, "pitch": pitch, "yaw": yaw}

    return None


def compute_axis_confidence(step_results: list[dict], candidates: int,
                            wind_speed: Optional[float] = None,
                            low_coherence: bool = False,
                            derived: bool = False) -> dict:
    """Compute aggregate axis-level confidence score and flags."""
    n_steps = len(step_results)
    score_step = min(1.0, n_steps / max(1, candidates))
    
    consistency_wind_factor = 1.0
    
    # Consistency checks: spread of tau and overshoot
    if n_steps >= 2:
        taus = [r["tau_s"] for r in step_results if r["tau_s"] is not None]
        overshoots = [r["overshoot"] for r in step_results if r["overshoot"] is not None]
        
        if len(taus) >= 2:
            median_tau = np.median(taus)
            p25_tau = np.percentile(taus, 25)
            p75_tau = np.percentile(taus, 75)
            spread_tau = p75_tau - p25_tau
            if median_tau > 0 and spread_tau > 0.5 * median_tau:
                consistency_wind_factor -= 0.2
                
        if len(overshoots) >= 2:
            median_os = np.median(overshoots)
            p25_os = np.percentile(overshoots, 25)
            p75_os = np.percentile(overshoots, 75)
            spread_os = p75_os - p25_os
            if median_os > 0 and spread_os > 0.5 * median_os:
                consistency_wind_factor -= 0.2
                
    # Wind degradation
    if wind_speed is not None and wind_speed > 8.0:
        consistency_wind_factor -= 0.2
        
    score = min(score_step, max(0.0, consistency_wind_factor))
    
    # Cap score for low coherence / derived values
    if low_coherence:
        score = min(score, 0.5)
    if derived:
        score = min(score, 0.5)
        
    pre_step_motion = any(s["confidence"]["flags"]["pre_step_motion"] for s in step_results) if step_results else False
    ramped_input = any(s["confidence"]["flags"]["ramped_input"] for s in step_results) if step_results else False
    short_post_window = any(s["confidence"]["flags"]["short_post_window"] for s in step_results) if step_results else False
    osc_reliable = any(s["confidence"]["flags"]["osc_reliable"] for s in step_results) if step_results else False
    
    return {
        "score": round(score, 2),
        "flags": {
            "pre_step_motion": pre_step_motion,
            "ramped_input": ramped_input,
            "low_coherence": low_coherence,
            "short_post_window": short_post_window,
            "derived_from_attitude": derived,
            "osc_reliable": osc_reliable,
            "high_wind": wind_speed is not None and wind_speed > 8.0
        }
    }


