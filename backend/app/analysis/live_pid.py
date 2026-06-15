"""
Size- and airframe-agnostic live PID tracking engine.

All verdicts are *non-dimensional* so the same thresholds hold for a
250 g racer and a 25 kg endurance VTOL:

  * Time constant tau — seconds for the actual rate to reach 63.2% of a
    detected step in the commanded rate. Under-tuning = settling > 4*tau,
    a ratio of the vehicle's own dynamics, never an absolute bound.
  * Pearson correlation r between commanded rates (ATTITUDE_TARGET body
    rates) and measured gyro rates (ATTITUDE body rates). r < 0.85
    during high-rate deflection = tracking authority depletion.
  * NRMSE — RMSE normalized by the commanded-rate range in the window.
  * Overshoot > 15% of step amplitude with >= 2 residual oscillations
    = damping problem (raise rate D); without oscillation = P too hot.

Regime gating:
  DYNAMIC_MANEUVER  -> step/tracking analysis on 3 s sliding windows.
  STEADY_HOLD       -> steady-state *offset* detection only: a constant
                       attitude offset (>2 deg for >2 s, low variance)
                       points at integrator deficit (wind / CG error)
                       -> rate I recommendation. No step analysis.
  PRE_FLIGHT        -> everything idle.

Fixed-wing extra: NRMSE samples are bucketed by airspeed. Tracking that
is markedly worse in the high-airspeed bucket than the low one (or vice
versa) indicates broken airspeed scaling — flagged as an advisory since
the fix is the FW_AIRSPD_* envelope, not a blind gain change.

Findings are published as alerts AND structured "recommendation" events
(param + relative change) the pilot can stage as safety-checked
proposals from the UI. This engine never writes anything.
"""
from __future__ import annotations

import asyncio
import math
import time
import logging
from collections import deque, defaultdict
from typing import Optional

log = logging.getLogger("mint.live_pid")


def _damping_ratio(overshoot: float) -> Optional[float]:
    if overshoot <= 0.01:  # essentially no overshoot
        return None
    log_os = math.log(overshoot)
    return -log_os / math.sqrt(math.pi**2 + log_os**2)

import numpy as np
import scipy.signal

from ..core import config
from ..core.safety_registry import REGISTRY
from ..mavlink.connection import CONNECTION
from ..mavlink.telemetry_hub import HUB
from . import loop_health
from .recommendations import recommend
from .regime import REGIME, Regime
from .vibration_live import VIB_GATE
from .stick_monitor import STICK_MONITOR
from ..advisors.param_advisor import ADVISOR

_AXES = {
    "roll": ("rollspeed", "body_roll_rate"),
    "pitch": ("pitchspeed", "body_pitch_rate"),
    "yaw": ("yawspeed", "body_yaw_rate"),
}

_WINDOW_S = config.PID_WINDOW_S
_EVAL_PERIOD_S = config.PID_EVAL_PERIOD_S
_R_DEPLETION = config.PID_R_DEPLETION
_HIGH_RATE_RAD_S = config.PID_HIGH_RATE_RAD_S
_OVERSHOOT_LIMIT = config.PID_OVERSHOOT_LIMIT
_OSC_CROSSINGS = config.PID_OSC_CROSSINGS
_SETTLE_BAND = config.PID_SETTLE_BAND

# Steady-state offset (I-gain) detection — STEADY_HOLD regime.
_OFFSET_WINDOW_S = config.PID_OFFSET_WINDOW_S
_OFFSET_MIN_SPAN_S = config.PID_OFFSET_MIN_SPAN_S
_OFFSET_RAD = math.radians(config.PID_OFFSET_DEG)
_OFFSET_MAX_STD = math.radians(config.PID_OFFSET_MAX_STD_DEG)

# Airspeed-binned tracking asymmetry (fixed-wing classes).
_ASPD_SAMPLES = config.PID_ASPD_SAMPLES
_ASPD_MIN_PER_BIN = config.PID_ASPD_MIN_PER_BIN
_ASPD_RATIO = config.PID_ASPD_RATIO
_ASPD_MIN_NRMSE = config.PID_ASPD_MIN_NRMSE

_ALERT_COOLDOWN_S = config.PID_ALERT_COOLDOWN_S

_FW_PREFIX = {"roll": "FW_RR_", "pitch": "FW_PR_", "yaw": "FW_YR_"}


def _airframe_class() -> Optional[str]:
    af = CONNECTION.state.airframe
    return af.airframe_class if af else None





def _quat_to_roll_pitch(q: list[float]) -> tuple[float, float]:
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    return roll, pitch


def compute_coherence_in_band(t: np.ndarray, x: np.ndarray, y: np.ndarray) -> Optional[float]:
    """Compute mean coherence between x and y in the 0.5-3.0 Hz frequency band."""
    if len(t) < 20 or (t[-1] - t[0]) <= 0:
        return None
    
    # Estimate sampling frequency
    dt = np.diff(t)
    mean_dt = np.mean(dt) if dt.size > 0 else 0.02
    if mean_dt <= 0:
        return None
    fs = 1.0 / mean_dt
    
    # Ensure signals are detrended
    try:
        x_detrend = scipy.signal.detrend(x)
        y_detrend = scipy.signal.detrend(y)
    except Exception:
        return None
    
    # Compute coherence
    nperseg = min(len(x), max(64, len(x) // 4))
    if nperseg < 8:
        return None
        
    try:
        f, Cxy = scipy.signal.coherence(x_detrend, y_detrend, fs=fs, nperseg=nperseg)
    except Exception:
        return None
    
    # Filter for the control band [0.5, 3.0] Hz
    band_mask = (f >= 0.5) & (f <= 3.0)
    if not band_mask.any():
        # Fallback to closest frequency if none in band
        closest_idx = np.argmin(np.abs(f - 1.75))
        return float(Cxy[closest_idx])
        
    return float(np.mean(Cxy[band_mask]))


def check_actuator_saturation(history: list[tuple[float, list[float]]], limit_threshold: float = 0.85) -> tuple[bool, float, float, float, int]:
    """
    Check if any actuator has exceeded 85% of its limit for >100 ms continuously.
    Returns:
      (is_saturated, duration_s, peak_value, sustained_value, actuator_index)
    """
    if len(history) < 2:
        return False, 0.0, 0.0, 0.0, 0
        
    num_actuators = len(history[0][1])
    max_duration = 0.0
    peak_value = 0.0
    sustained_value = 0.0
    sat_actuator_idx = 0
    
    for i in range(num_actuators):
        # Extract times and values for actuator i
        t_seq = []
        v_seq = []
        for t, vals in history:
            if i < len(vals):
                t_seq.append(t)
                v_seq.append(abs(vals[i])) # Use absolute value for deflections/motors
                
        if len(t_seq) < 2:
            continue
            
        # Find continuous segments where value >= limit_threshold
        in_segment = False
        start_t = 0.0
        start_idx = 0
        
        for idx in range(len(t_seq)):
            val = v_seq[idx]
            t = t_seq[idx]
            if val >= limit_threshold:
                if not in_segment:
                    in_segment = True
                    start_t = t
                    start_idx = idx
            else:
                if in_segment:
                    in_segment = False
                    dur = t - start_t
                    if dur > max_duration:
                        max_duration = dur
                        segment_vals = v_seq[start_idx:idx]
                        peak_value = np.max(segment_vals)
                        sustained_value = np.percentile(segment_vals, 85)
                        sat_actuator_idx = i
                    
        # Check if the last segment was still active at the end of the window
        if in_segment:
            dur = t_seq[-1] - start_t
            if dur > max_duration:
                max_duration = dur
                segment_vals = v_seq[start_idx:]
                peak_value = np.max(segment_vals)
                sustained_value = np.percentile(segment_vals, 85)
                sat_actuator_idx = i
                
    return (max_duration > 0.1), max_duration, float(peak_value), float(sustained_value), sat_actuator_idx





class LivePidEngine:
    """Fuses rate setpoints + gyro rates into non-dimensional metrics."""

    def __init__(self) -> None:
        # dynamic-regime windows: per-axis (t, sp_rate, act_rate)
        self._win: dict[str, deque] = {ax: deque() for ax in _AXES}
        self._latest_sp: dict[str, float] = {}
        # steady-hold windows: per-axis (t, angle_offset)
        self._offset_win: dict[str, deque] = {"roll": deque(), "pitch": deque()}
        self._latest_angle_sp: dict[str, float] = {}
        # fixed-wing airspeed buckets: per-axis (airspeed, nrmse)
        self._aspd_samples: dict[str, deque] = {ax: deque(maxlen=_ASPD_SAMPLES)
                                                for ax in _AXES}
        self._airspeed = 0.0
        self._vtol_state = 0
        self._last_eval = 0.0
        self._last_alert: dict[str, float] = {}
        self._actuation_history: deque[tuple[float, list[float]]] = deque()
        self._was_saturated = False
        self._axis_active_start: dict[str, float] = {}
        self._consecutive_overshoot_no_osc: dict[str, int] = defaultdict(int)
        self._window_s_override: dict[str, float] = defaultdict(lambda: _WINDOW_S)
        self._recommended_axes_this_cycle: set[str] = set()
        self._task: asyncio.Task | None = None

    def _rate_param(self, axis: str, term: str) -> Optional[str]:
        """Resolve the rate-loop parameter for the detected airframe class.
        Returns None when no class is known — no class, no recommendations."""
        cls = _airframe_class()
        if cls == "VTOL":
            resolved_cls = "FIXED_WING" if self._vtol_state == 4 else "MULTIROTOR"
        else:
            resolved_cls = cls

        if resolved_cls in ("FIXED_WING", "DELTA_WING") and axis == "yaw" and term == "D":
            return None
        if resolved_cls in ("MULTIROTOR", "VTOL"):
            return f"MC_{axis.upper()}RATE_{term}"
        if resolved_cls in ("FIXED_WING", "DELTA_WING"):
            return _FW_PREFIX[axis] + term
        return None

    def _auto_rate_param(self, axis: str) -> Optional[str]:
        cls = _airframe_class()
        if cls == "VTOL":
            resolved_cls = "FIXED_WING" if self._vtol_state == 4 else "MULTIROTOR"
        else:
            resolved_cls = cls

        if resolved_cls in ("MULTIROTOR", "VTOL"):
            return f"MC_{axis.upper()}RAUTO_MAX"
        return None

    def start(self) -> None:
        if self._task is None or self._task.done():
            self._task = asyncio.create_task(self._run(), name="live-pid-engine")

    def stop(self) -> None:
        if self._task:
            self._task.cancel()

    # ------------------------------------------------------------------ #
    async def _run(self) -> None:
        async for event in HUB.subscribe():
            if event.channel == "attitude_target":
                p = event.payload
                for ax, (_, sp_field) in _AXES.items():
                    v = p.get(sp_field)
                    if v is not None:
                        self._latest_sp[ax] = float(v)
                q = p.get("q")
                if q and len(q) == 4:
                    r, pch = _quat_to_roll_pitch(q)
                    self._latest_angle_sp = {"roll": r, "pitch": pch}
            elif event.channel == "extended_sys_state":
                self._vtol_state = int(event.payload.get("vtol_state", 0) or 0)
            elif event.channel == "vfr_hud":
                self._airspeed = max(0.0, float(event.payload.get("airspeed", 0) or 0))
            elif event.channel == "attitude":
                self._ingest(event.payload)
            elif event.channel == "actuation":
                self._ingest_actuation(event.payload)

    def _ingest_actuation(self, payload: dict) -> None:
        now = time.monotonic()
        outputs = payload.get("motor_norms") or payload.get("surface_deflections") or []
        if outputs:
            self._actuation_history.append((now, [float(o) for o in outputs]))
            while self._actuation_history and self._actuation_history[0][0] < now - 10.0:
                self._actuation_history.popleft()

    def _ingest(self, att: dict) -> None:
        now = time.monotonic()
        regime = REGIME.current

        if regime == Regime.DYNAMIC_MANEUVER:
            self._offset_win["roll"].clear()
            self._offset_win["pitch"].clear()
            self._ingest_dynamic(att, now)
        elif regime == Regime.STEADY_HOLD:
            for win in self._win.values():
                win.clear()
            # self._actuation_history.clear() <- Do NOT clear
            self._ingest_steady(att, now)
            for ax in _AXES:
                ADVISOR.clear_diagnostic_card(ax)
                self._axis_active_start.pop(ax, None)
        else:  # PRE_FLIGHT
            for win in self._win.values():
                win.clear()
            self._offset_win["roll"].clear()
            self._offset_win["pitch"].clear()
            self._actuation_history.clear()
            for ax in _AXES:
                ADVISOR.clear_diagnostic_card(ax)
                self._axis_active_start.pop(ax, None)

    # ------------------------------------------------------------------ #
    # DYNAMIC_MANEUVER: step response + tracking correlation
    # ------------------------------------------------------------------ #
    def _ingest_dynamic(self, att: dict, now: float) -> None:
        for ax, (act_field, _) in _AXES.items():
            act, sp = att.get(act_field), self._latest_sp.get(ax)
            if act is None or sp is None:
                continue
            win = self._win[ax]
            win.append((now, sp, float(act)))
            window_s = self._window_s_override.get(ax, _WINDOW_S)
            horizon = now - window_s
            while win and win[0][0] < horizon:
                win.popleft()

        if now - self._last_eval >= _EVAL_PERIOD_S:
            self._last_eval = now
            self._evaluate(now)

    def _get_saturation_severity(self) -> str:
        """
        Unify saturation severity based on check_actuator_saturation metrics.
        85% = start of mild saturation (transient allowed).
        95% = moderate saturation threshold (blocks P-gain increases).
        100% for 1s = severe (block everything).
        """
        is_sat, sat_dur, peak_val, sustained_val, sat_act_idx = check_actuator_saturation(list(self._actuation_history))
        if not is_sat:
            return "none"
        if peak_val >= 1.0 and sat_dur >= 1.0:
            return "severe"
        if sustained_val >= 0.95:
            return "moderate"
        if sustained_val >= 0.85:
            return "mild"
        return "none"

    @staticmethod
    def _is_proposal_allowed(param: str, scale_factor: Optional[float], delta: Optional[float], severity: str) -> bool:
        """
        Enforce unified saturation rules:
        - severe: block everything (proposals not allowed).
        - moderate or mild: block P-gain increases. Reductions or rate limit changes are allowed.
        - none: all allowed.
        """
        if severity == "severe":
            return False
            
        # Determine if it's a P-gain increase
        is_p_gain = param.endswith("RATE_P") or param.endswith("_P")
        
        # Check if the proposed change is an increase
        is_increase = False
        if scale_factor is not None and scale_factor > 1.0:
            is_increase = True
        if delta is not None and delta > 0.0:
            is_increase = True
            
        if severity in ("moderate", "mild"):
            if is_p_gain and is_increase:
                return False
                
        return True

    def _safe_recommend(self, ax: str, param: str, rationale: str, *,
                        scale_factor: Optional[float] = None,
                        delta: Optional[float] = None,
                        target_value: Optional[float] = None,
                        is_saturation_gain_reduction: bool = False,
                        cooldown_s: float = 30.0,
                        confidence: Optional[str] = None,
                        limitations: Optional[str] = None,
                        severity: Optional[str] = None) -> None:
        if severity is None:
            severity = self._get_saturation_severity()
        if not self._is_proposal_allowed(param, scale_factor, delta, severity):
            return
            
        recommend(
            param, rationale,
            scale_factor=scale_factor,
            delta=delta,
            target_value=target_value,
            source="live_pid",
            cooldown_s=cooldown_s,
            is_saturation_gain_reduction=is_saturation_gain_reduction,
            confidence=confidence,
            limitations=limitations
        )
        self._recommended_axes_this_cycle.add(ax)

    def _determine_diagnostic(self, ax: str, now: float, severity: Optional[str] = None) -> str:
        """
        Return the appropriate diagnostic text explaining why no proposal is generated.
        """
        if severity is None:
            severity = self._get_saturation_severity()
        # 1. Severe saturation
        if severity == "severe":
            return "Severe saturation – fix mechanical issues or reduce rate demand before tuning."
            
        # 2. Vibration too high
        worst_vib = max(VIB_GATE._latest.get(k, 0.0) for k in ("x", "y", "z")) if VIB_GATE._latest else 0.0
        if not VIB_GATE.ok() and worst_vib >= 30.0:
            return f"Vibration too high (metric {worst_vib:.0f}) – land and inspect props / mounting."
            
        # 3. Insufficient data (not enough samples in window)
        win = self._win[ax]
        if len(win) < 15:
            return "Insufficient data – wait for more flight time or increase telemetry rate."
            
        # 4. Default / Low excitation instruction
        if REGIME.current == Regime.PRE_FLIGHT:
            return f"Hold the vehicle steady on the ground for 1.5 seconds, then apply a sharp stick input on {ax}."
        else:
            return f"Stop maneuvering and stabilise for 1.5 seconds, then apply a sharp stick input on {ax}."

    def _evaluate(self, now: float) -> None:
        metrics: dict[str, dict] = {}
        worst_r = None
        self._recommended_axes_this_cycle.clear()

        # Capture saturation metrics once per evaluation cycle
        act_sat_info = check_actuator_saturation(list(self._actuation_history))
        is_sat, sat_dur, peak_val, sustained_val, sat_act_idx = act_sat_info
        
        severity = "none"
        if is_sat:
            if peak_val >= 1.0 and sat_dur >= 1.0:
                severity = "severe"
            elif sustained_val >= 0.95:
                severity = "moderate"
            elif sustained_val >= 0.85:
                severity = "mild"

        for ax, win in self._win.items():
            if STICK_MONITOR._tuning_active and STICK_MONITOR._needed_axis != ax:
                self._axis_active_start.pop(ax, None)
                ADVISOR.clear_diagnostic_card(ax)
                continue

            if len(win) < 15:
                continue
            t = np.array([s[0] for s in win])
            sp = np.array([s[1] for s in win])
            act = np.array([s[2] for s in win])

            m = self._tracking_metrics(sp, act)
            m["coherence"] = compute_coherence_in_band(t, sp, act)
            step = self._step_response(
                t, sp, act,
                min_amp=config.PID_MIN_AMP,
                ax=ax,
                recommended_axes=self._recommended_axes_this_cycle
            )
            if step:
                m.update(step)
            m["sp_deg_s"] = round(math.degrees(sp[-1]), 1)
            m["act_deg_s"] = round(math.degrees(act[-1]), 1)
            metrics[ax] = m
            if m.get("r") is not None:
                worst_r = m["r"] if worst_r is None else min(worst_r, m["r"])

            if m.get("nrmse") is not None and self._airspeed > 5.0:
                self._aspd_samples[ax].append((self._airspeed, m["nrmse"]))
                self._check_airspeed_asymmetry(ax, now)

            # Compute quality scores
            std_sp = float(np.std(sp))
            if std_sp < 0.05:
                excitation_score = 0.0
            elif std_sp > 0.25:
                excitation_score = 1.0
            else:
                excitation_score = (std_sp - 0.05) / 0.20

            if step is not None:
                step_score = max(0.0, min(1.0, 1.0 - (step["post_sp_std"] / abs(step["amplitude"]))))
            else:
                step_score = 1.0

            coherence = m.get("coherence")
            if coherence is not None:
                coherence_score = max(0.0, min(1.0, coherence))
            else:
                coherence_score = 1.0

            confidence_value = excitation_score * step_score * coherence_score
            if step is not None and "noise_ratio" in step:
                confidence_value *= (1.0 - step["noise_ratio"])
            
            if STICK_MONITOR._tuning_active:
                confidence_label = "High confidence (test window)"
            elif confidence_value > 0.6:
                confidence_label = "Medium confidence (passive observation)"
            else:
                confidence_label = "Low confidence (needs clearer data)"

            limitations_note = None
            if step is not None and step.get("ramped_adjusted"):
                limitations_note = "Input was slightly ramped; τ estimate adjusted."
            if step is not None and step.get("noise_ratio") is not None:
                limitations_note = "Elevated pre-step noise; confidence degraded."

            # Passive analysis gating:
            # When excitation is low (std(sp) < 0.12 rad/s) and coherence < 0.6,
            # show diagnostic card (via diagnostic logic) and do not make recommendations.
            skip_recommendations = False
            if not STICK_MONITOR._tuning_active:
                if std_sp < 0.12 and (coherence is None or coherence < 0.6):
                    skip_recommendations = True

            self._verdicts(ax, m, now, confidence_label, limitations_note, skip_recommendations, severity=severity, act_sat_info=act_sat_info)

        # Update diagnostic cards for active/inactive axes
        for ax in _AXES:
            if STICK_MONITOR._tuning_active and STICK_MONITOR._needed_axis != ax:
                self._axis_active_start.pop(ax, None)
                ADVISOR.clear_diagnostic_card(ax)
                continue

            win = self._win[ax]
            is_active = False
            if len(win) >= 15:
                sp = np.array([s[1] for s in win])
                if np.std(sp) > 0.12:
                    is_active = True
            
            if is_active:
                if ax not in self._axis_active_start:
                    self._axis_active_start[ax] = now
                
                # Active for >10s
                if now - self._axis_active_start[ax] > 10.0:
                    if ax not in self._recommended_axes_this_cycle:
                        text = self._determine_diagnostic(ax, now, severity=severity)
                        ADVISOR.set_diagnostic_card(ax, text)
                    else:
                        ADVISOR.clear_diagnostic_card(ax)
            else:
                self._axis_active_start.pop(ax, None)
                ADVISOR.clear_diagnostic_card(ax)

        if worst_r is not None:
            # Innermost loop's health gates every outer-loop verdict.
            loop_health.set_health("rate", worst_r >= _R_DEPLETION)
        if metrics:
            HUB.publish("loop_metrics", {"loop": "rate", "axes": metrics,
                                         "unit": "deg/s"})

    @staticmethod
    def _tracking_metrics(sp: np.ndarray, act: np.ndarray) -> dict:
        """Pearson r + NRMSE, both dimensionless."""
        out: dict = {"r": None, "nrmse": None, "high_rate": False}
        sp_range = float(np.ptp(sp))
        out["high_rate"] = sp_range > _HIGH_RATE_RAD_S
        if sp_range < 1e-3 or np.std(sp) < 1e-6 or np.std(act) < 1e-6:
            return out
        out["r"] = round(float(np.corrcoef(sp, act)[0, 1]), 3)
        rmse = float(np.sqrt(np.mean((sp - act) ** 2)))
        out["nrmse"] = round(rmse / sp_range, 3)
        return out

    @staticmethod
    def _step_response(t: np.ndarray, sp: np.ndarray, act: np.ndarray,
                       min_amp: float = 0.15, ax: Optional[str] = None,
                       recommended_axes: Optional[set[str]] = None) -> Optional[dict]:
        """
        Isolate the largest commanded step and measure tau (63.2% rise),
        settling (into the +/-20% band), overshoot, and residual
        oscillation count — all relative to the step's own amplitude.
        `min_amp` is in the signal's own units (rad/s for rates, rad for
        attitude, m/s for velocity, m for position).

        NOTE: This design processes a single step per evaluation window by finding the
        largest transition (argmax of absolute diff). In rapid sequences with multiple
        consecutive steps, only the largest step within the current window is analyzed.
        This single-step-per-window behavior is inherent to the windowed architecture.
        """
        if len(sp) < 20:
            return None
        i = int(np.argmax(np.abs(np.diff(sp))))
        t0 = t[i + 1]

        # Go backwards from i to find the start of the stick movement
        diff_sp = np.diff(sp)
        step_dir = np.sign(diff_sp[i]) if diff_sp[i] != 0 else 1.0
        
        i_start = i
        while i_start > 0:
            if np.sign(diff_sp[i_start - 1]) != step_dir or abs(diff_sp[i_start - 1]) < 0.005:
                break
            i_start -= 1
            
        t_start = t[i_start]
        pre_mask = (t >= t_start - 1.0) & (t < t_start)
        post_mask = t >= t0
        if pre_mask.sum() < 3 or len(sp[post_mask]) < 8:
            return None

        sp_pre = float(np.mean(sp[pre_mask]))
        sp_post = float(np.mean(sp[post_mask][3:]))
        amp = sp_post - sp_pre

        # Size-agnostic significance gate: the step must dwarf the
        # vehicle's own pre-step measurement noise.
        act_noise = float(np.std(act[pre_mask])) or 1e-6

        noise_ratio = None

        # Check rejection reasons:
        # Case 1: Genuine vibration
        if act_noise > 0.05 and not VIB_GATE.ok():
            if ax and recommended_axes is not None:
                ADVISOR.set_diagnostic_card(ax, "Step rejected: excessive vibration – fix mechanical issues first")
                recommended_axes.add(ax)
            return None

        # Case 2: Pre-step motion
        if act_noise > 0.05 and VIB_GATE.ok():
            if abs(amp) < 2.0 * act_noise:
                if ax and recommended_axes is not None:
                    ADVISOR.set_diagnostic_card(ax, "Step rejected: vehicle was moving significantly in the 1 second before your input. Hold steady for 1.5 seconds, then step sharply.")
                    recommended_axes.add(ax)
                return None
            # else: amplitude clears the noise floor — continue analysis,
            # set degraded confidence flag downstream
            noise_ratio = act_noise / abs(amp)

        # Calculate adaptive multiplier
        multiplier = 2.0 + 2.0 * min(abs(amp) / 0.3, 1.0)

        # Case 3: Step too small
        if abs(amp) < multiplier * act_noise or abs(amp) < min_amp:
            if ax and recommended_axes is not None:
                ADVISOR.set_diagnostic_card(ax, f"Step rejected: input too small or too slow. Apply a larger, sharper stick deflection (≥{min_amp:.2f} rad/s).")
                recommended_axes.add(ax)
            return None

        post_sp_std = float(np.std(sp[post_mask][3:]))
        if post_sp_std > 0.5 * abs(amp):
            return None   # ramp / continuous stirring, not even a relaxed step

        ta, aa = t[post_mask], act[post_mask]
        sign = 1.0 if amp > 0 else -1.0

        # Check that we started below the 63.2% threshold relative to the step direction
        started_below = sign * (aa[0] - (sp_pre + 0.632 * amp)) < 0
        crossed = sign * (aa - (sp_pre + 0.632 * amp)) >= 0
        crossed_90 = sign * (aa - (sp_pre + 0.90 * amp)) >= 0

        if started_below and crossed.any() and crossed_90.any():
            tau = float(ta[np.argmax(crossed)] - t0)
        else:
            tau = None

        ramped_adjusted = False
        if post_sp_std > 0.35 * abs(amp):
            ramped_adjusted = True
            if tau is not None:
                tau = tau * 1.5

        peak = float(np.max(sign * (aa - sp_post)))
        overshoot = max(0.0, peak / abs(amp))

        outside = np.abs(aa - sp_post) > _SETTLE_BAND * abs(amp)
        if np.all(outside):
            settling = None
            settled = False
        else:
            settling = float(ta[np.where(outside)[0][-1]] - t0) if outside.any() else 0.0
            settled = not bool(outside[-1])

        # Residual oscillation: zero-crossings of significant excursions
        # around the new setpoint. >= _OSC_CROSSINGS means the response
        # rings ("oscillates twice") rather than just overshooting once.
        resid = aa - sp_post
        crossing_noise = min(act_noise, 0.03) if (act_noise > 0.05 and VIB_GATE.ok()) else act_noise
        sig = resid[np.abs(resid) > max(0.03 * abs(amp), 2 * crossing_noise, 0.01)]
        crossings = int(np.count_nonzero(np.diff(np.sign(sig)) != 0)) if sig.size > 1 else 0

        post_duration = float(ta[-1] - t0) if len(ta) > 0 else 0.0
        res_dict = {
            "tau_s": round(tau, 3) if tau is not None else None,
            "settling_s": round(settling, 3) if settled else None,
            "overshoot": round(overshoot, 3),
            "oscillations": None if post_duration < 1.0 else crossings,
            "step_amp_deg_s": round(math.degrees(amp), 1),
            "amplitude": amp,
            "post_sp_std": post_sp_std,
            "ramped_adjusted": ramped_adjusted,
        }
        if noise_ratio is not None:
            res_dict["noise_ratio"] = noise_ratio
        return res_dict

    async def _advise_damping(self, ax: str, overshoot: float, osc: int, zeta: Optional[float], confidence_label: str, limitations_note: Optional[str], severity: Optional[str] = None) -> None:
        p_d = self._rate_param(ax, "D")
        p_p = self._rate_param(ax, "P")
        if not p_d or not p_p:
            return

        treat_as_maxed = False
        current_d = 0.0
        try:
            current_d = await asyncio.wait_for(CONNECTION.read_param(p_d), timeout=2.0)
        except Exception:
            treat_as_maxed = True

        if not treat_as_maxed:
            cls = _airframe_class()
            if cls:
                limits = REGISTRY.params_for(cls).get(p_d)
                if limits and "abs_max" in limits:
                    abs_max = limits["abs_max"]
                    if current_d >= abs_max - 1e-6:
                        treat_as_maxed = True

        vib_high = not VIB_GATE.ok() or VIB_GATE.just_cleared()
        if not VIB_GATE.ok():
            log.warning("VIB_GATE hysteresis inconsistency: step accepted but vibration gate not ok at advice time. Falling back to P-gain reduction.")

        if vib_high or treat_as_maxed:
            # Action: Reduce P-gain
            if vib_high:
                rationale = f"{ax.capitalize()} overshoot {overshoot*100:.0f}% with low damping, but Rate D-gain increases are blocked by high vibration — back off P-gain instead."
            else:
                rationale = f"{ax.capitalize()} overshoot {overshoot*100:.0f}% with low damping, but Rate D-gain is at its safety limit ({current_d:.4f}) — back off P-gain instead."
            
            self._safe_recommend(
                ax, p_p, rationale,
                scale_factor=0.9,
                confidence=confidence_label,
                limitations=limitations_note,
                severity=severity
            )
        else:
            # Action: Increase D-gain
            if osc >= 2:
                rationale = f"{ax.capitalize()} overshoot {overshoot*100:.0f}% with {osc} residual reversals — add derivative damping."
            else:
                zeta_val = zeta if zeta is not None else 0.0
                rationale = f"{ax.capitalize()} overshoot {overshoot*100:.0f}% with low damping ratio (ζ={zeta_val:.2f}) — add derivative damping."

            self._safe_recommend(
                ax, p_d, rationale,
                scale_factor=1.15,
                confidence=confidence_label,
                limitations=limitations_note,
                severity=severity
            )

    # ------------------------------------------------------------------ #
    def _verdicts(self, ax: str, m: dict, now: float, confidence_label: str, limitations_note: Optional[str] = None, skip_recommendations: bool = False, severity: Optional[str] = None, act_sat_info: Optional[tuple] = None) -> None:
        """Turn one window's metrics into alerts + stageable advice."""
        if STICK_MONITOR._tuning_active and STICK_MONITOR._needed_loop != "rate":
            return

        # Check consecutive windows showing overshoot > 20% but zero oscillations
        overshoot = m.get("overshoot", 0.0)
        osc = m.get("oscillations", 0)

        if overshoot > 0.20 and osc == 0:
            self._consecutive_overshoot_no_osc[ax] += 1
        else:
            self._consecutive_overshoot_no_osc[ax] = 0
            self._window_s_override[ax] = _WINDOW_S
            # Immediately prune the window to _WINDOW_S
            win = self._win[ax]
            horizon = now - _WINDOW_S
            while win and win[0][0] < horizon:
                win.popleft()

        if self._consecutive_overshoot_no_osc[ax] >= 2:
            self._window_s_override[ax] = 4.0

        # Check actuator saturation
        if act_sat_info is None:
            act_sat_info = check_actuator_saturation(list(self._actuation_history))
        is_sat, sat_dur, peak_val, sustained_val, sat_act_idx = act_sat_info
        
        if severity is None:
            severity = "none"
            if is_sat:
                if peak_val >= 1.0 and sat_dur >= 1.0:
                    severity = "severe"
                elif sustained_val >= 0.95:
                    severity = "moderate"
                elif sustained_val >= 0.85:
                    severity = "mild"

        coherence = m.get("coherence")
        
        # Check joint conditions:
        sat_active = is_sat and (coherence is not None) and (coherence < 0.6)

        # Track resolution of saturation:
        if self._was_saturated and REGIME.current == Regime.DYNAMIC_MANEUVER and m.get("high_rate"):
            recent_act = [vals for t, vals in self._actuation_history if t >= now - 2.0]
            has_2s = len(self._actuation_history) > 0 and self._actuation_history[0][0] <= now - 2.0
            if has_2s and recent_act:
                max_recent = max(max(abs(v) for v in vals) for vals in recent_act)
                if max_recent < 0.75:
                    HUB.publish("alert", {
                        "severity": "success",
                        "source": "pid",
                        "type": "resolution",
                        "text": "Saturation Resolved: Actuator authority restored. Peak outputs remaining below 75% limit."
                    })
                    self._was_saturated = False

        if sat_active:
            # Publish saturation alert with full details
            if now - self._last_alert.get(f"sat_{ax}", 0) >= _ALERT_COOLDOWN_S:
                self._last_alert[f"sat_{ax}"] = now
                HUB.publish("alert", {
                    "severity": "warning",
                    "source": "pid",
                    "type": "saturation",
                    "axis": ax,
                    "motor_idx": sat_act_idx,
                    "peak_pct": round(peak_val * 100, 1),
                    "sustained_pct": round(sustained_val * 100, 1),
                    "duration_ms": round(sat_dur * 1000),
                    "coherence": round(coherence, 3) if coherence is not None else None,
                    "text": f"Actuator Saturation: {ax.capitalize()} motor #{sat_act_idx} hit {peak_val*100:.1f}% peak ({sustained_val*100:.1f}% sustained) for {sat_dur*1000:.0f} ms."
                })
                self._was_saturated = True
            
            if not skip_recommendations:
                # Propose auto rate limit parameter reduction
                p_auto = self._auto_rate_param(ax)
                if p_auto:
                    reduction = min(0.9, max(0.6, 0.75 / sustained_val))
                    self._safe_recommend(
                        ax, p_auto,
                        f"Actuator saturation detected on {ax}. Reduce rate limit to restore headroom.",
                        scale_factor=round(reduction, 3),
                        confidence=confidence_label,
                        limitations=limitations_note,
                        severity=severity
                    )

                if config.EXPERT_MODE:
                    overshoot = m.get("overshoot", 0.0)
                    if overshoot > _OVERSHOOT_LIMIT:
                        self._last_alert[ax] = now
                        p_p = self._rate_param(ax, "P")
                        if p_p:
                            self._safe_recommend(
                                ax, p_p,
                                f"{ax.capitalize()} overshoot {overshoot*100:.0f}% without oscillation during saturation — back off P-gain (expert mode).",
                                scale_factor=0.9,
                                is_saturation_gain_reduction=True,
                                confidence=confidence_label,
                                limitations=limitations_note,
                                severity=severity
                            )
            # Under saturation, block all other tuning proposals on this axis
            return

        # If only coherence is low and actuators are <70%, alert for vibration/slop instead
        if (coherence is not None) and (coherence < 0.6) and (not is_sat) and (sustained_val < 0.70):
            if now - self._last_alert.get(f"vib_{ax}", 0) >= _ALERT_COOLDOWN_S:
                self._last_alert[f"vib_{ax}"] = now
                HUB.publish("alert", {
                    "severity": "warning",
                    "source": "pid",
                    "type": "vibration_slop",
                    "text": f"Low coherence ({coherence:.2f}) on {ax} with low actuator demand ({sustained_val*100:.1f}%). Suspect mechanical vibration, linkage slop, or sensor noise."
                })
            return

        if skip_recommendations:
            return

        r, tau, settling = m.get("r"), m.get("tau_s"), m.get("settling_s")
        overshoot, osc = m.get("overshoot", 0.0), m.get("oscillations", 0)

        if now - self._last_alert.get(ax, 0) < _ALERT_COOLDOWN_S:
            return

        if m.get("high_rate") and r is not None and r < _R_DEPLETION:
            # Skip depletion check if a step was detected — low r during a step is expected
            if m.get("step_amp_deg_s") is None:
                # Authority problem — never recommend gains here.
                self._last_alert[ax] = now
                HUB.publish("alert", {
                    "severity": "warning", "source": "pid",
                    "text": (f"Tracking authority depletion on {ax}: r={r:.2f} "
                             f"(<{_R_DEPLETION}) during high-rate input. Check "
                             f"actuator saturation and vibration before touching gains."),
                })
                return

        # Compute zeta and check underdamped condition
        zeta = None
        if overshoot > 0.20:
            zeta = _damping_ratio(overshoot)

        is_underdamped = False
        branch = None
        if overshoot > _OVERSHOOT_LIMIT and osc is not None and osc >= _OSC_CROSSINGS:
            is_underdamped = True
            branch = "A"
        elif overshoot > 0.20 and zeta is not None and zeta < 0.3 and osc is not None and osc >= 1:
            is_underdamped = True
            branch = "B"

        if is_underdamped:
            self._last_alert[ax] = now

            if branch == "A":
                conf = "High – clear ringing observed"
                alert_text = (f"{ax.capitalize()} overshoots {overshoot*100:.0f}% and "
                              f"rings ({osc} reversals) before settling — damping deficit.")
            else:
                conf = "Medium – extrapolated from zeta, no observed ringing"
                alert_text = f"{ax.capitalize()} overshoots {overshoot*100:.0f}% with low damping ratio (ζ={zeta:.2f}) — damping deficit."

            HUB.publish("alert", {
                "severity": "warning", "source": "pid",
                "text": alert_text,
            })

            try:
                asyncio.create_task(self._advise_damping(ax, overshoot, osc, zeta, conf, limitations_note, severity))
            except RuntimeError:
                try:
                    asyncio.run(self._advise_damping(ax, overshoot, osc, zeta, conf, limitations_note, severity))
                except Exception:
                    pass
            return

        if overshoot > _OVERSHOOT_LIMIT and osc is not None:
            self._last_alert[ax] = now
            p_p = self._rate_param(ax, "P")
            HUB.publish("alert", {
                "severity": "warning", "source": "pid",
                "text": (f"Dynamic overshoot on {ax}: {overshoot*100:.0f}% of step "
                         f"amplitude (no ringing) — proportional gain too hot."),
            })
            if p_p:
                self._safe_recommend(
                    ax, p_p,
                    f"{ax} overshoot {overshoot*100:.0f}% without oscillation — back off proportional gain ~10%.",
                    scale_factor=0.9,
                    confidence=confidence_label,
                    limitations=limitations_note,
                    severity=severity
                )
            return

        if tau is not None and settling is not None and settling > 4 * tau:
            self._last_alert[ax] = now
            
            if severity == "none":
                p_p = self._rate_param(ax, "P")
                if p_p:
                    if VIB_GATE.ok():
                        self._safe_recommend(
                            ax, p_p,
                            f"{ax.capitalize()} settles in {settling:.2f}s vs τ={tau:.2f}s (>4×τ) — raise proportional gain ~10%.",
                            scale_factor=1.1,
                            confidence=confidence_label,
                            limitations=limitations_note,
                            severity=severity
                        )
                    else:
                        HUB.publish("alert", {
                            "severity": "info", "source": "pid",
                            "text": (f"{ax.capitalize()} looks under-tuned, but gain raises "
                                     f"are suspended while vibration is high."),
                        })
            elif severity in ("mild", "moderate"):
                p_auto = self._auto_rate_param(ax)
                if p_auto:
                    self._safe_recommend(
                        ax, p_auto,
                        "Sluggish because rate demand exceeds vehicle capability – lowering demand will restore tracking.",
                        scale_factor=0.85,
                        confidence=confidence_label,
                        limitations=limitations_note,
                        severity=severity
                    )
            return

        if tau is None and m.get("step_amp_deg_s"):
            self._last_alert[ax] = now
            HUB.publish("alert", {
                "severity": "warning", "source": "pid",
                "text": (f"{ax.capitalize()} never reached 63.2% of the commanded "
                         f"step within the window — severely sluggish or saturated."),
            })
            if not skip_recommendations:
                if severity in ("mild", "moderate"):
                    # Saturation present — rate demand exceeds capability
                    p_auto = self._auto_rate_param(ax)
                    if p_auto:
                        self._safe_recommend(
                            ax, p_auto,
                            f"{ax.capitalize()} never reached command — rate demand likely exceeds vehicle capability. Reduce rate limit to restore tracking.",
                            scale_factor=0.85,
                            confidence=confidence_label,
                            limitations=limitations_note,
                            severity=severity
                        )
                    else:
                        # Yaw has no RAUTO_MAX — fall through to P reduction below
                        p_p = self._rate_param(ax, "P")
                        if p_p and VIB_GATE.ok():
                            self._safe_recommend(
                                ax, p_p,
                                f"{ax.capitalize()} never reached command under saturation — reduce P-gain to lower demand.",
                                scale_factor=0.9,
                                confidence=confidence_label,
                                limitations=limitations_note,
                                severity=severity
                            )
                else:
                    # No saturation — under-gained
                    p_p = self._rate_param(ax, "P")
                    if p_p:
                        if VIB_GATE.ok():
                            self._safe_recommend(
                                ax, p_p,
                                f"{ax.capitalize()} never reached 63.2% of command — likely under-gained. Raise P ~10%.",
                                scale_factor=1.1,
                                confidence=confidence_label,
                                limitations=limitations_note,
                                severity=severity
                            )
                        else:
                            HUB.publish("alert", {
                                "severity": "info", "source": "pid",
                                "text": (f"{ax.capitalize()} appears under-gained but gain raises "
                                         f"are suspended while vibration is high."),
                            })

    # ------------------------------------------------------------------ #
    # STEADY_HOLD: integrator-deficit (steady-state offset) detection
    # ------------------------------------------------------------------ #
    def _ingest_steady(self, att: dict, now: float) -> None:
        if _airframe_class() not in ("MULTIROTOR", "VTOL"):
            return   # hover-offset logic is a rotor-borne concept
        # Gate steady-state hover offset checks to only run when airspeed is < 5.0 m/s
        if self._airspeed >= 5.0:
            return
        horizon = now - _OFFSET_WINDOW_S
        for ax in ("roll", "pitch"):
            act = att.get(ax)
            sp = self._latest_angle_sp.get(ax)
            if act is None or sp is None:
                continue
            win = self._offset_win[ax]
            win.append((now, sp - float(act)))
            while win and win[0][0] < horizon:
                win.popleft()
            self._check_offset(ax, win, now)

    def _check_offset(self, ax: str, win: deque, now: float) -> None:
        if len(win) < 10 or (win[-1][0] - win[0][0]) < _OFFSET_MIN_SPAN_S:
            return
        offs = np.array([o for _, o in win])
        mean, std = float(np.mean(offs)), float(np.std(offs))
        # Constant offset = large stable mean; an oscillating or drifting
        # error is a different disease and must not trigger I advice.
        if abs(mean) < _OFFSET_RAD or std > _OFFSET_MAX_STD:
            return
        if now - self._last_alert.get(f"offset_{ax}", 0) < _ALERT_COOLDOWN_S * 2:
            return
        self._last_alert[f"offset_{ax}"] = now
        p_i = self._rate_param(ax, "I")
        HUB.publish("alert", {
            "severity": "info", "source": "pid",
            "text": (f"Steady-state {ax} offset of {math.degrees(mean):.1f}° held "
                     f">{_OFFSET_MIN_SPAN_S:.0f}s in hover — integrator deficit "
                     f"(wind or CG imbalance)."),
        })
        if p_i:
            recommend(p_i, f"Constant {math.degrees(mean):.1f}° {ax} offset during "
                           f"steady hover — raise integral gain to trim it out.",
                      scale_factor=1.15, source="live_pid", cooldown_s=60.0)

    # ------------------------------------------------------------------ #
    # Fixed-wing: airspeed-binned tracking asymmetry
    # ------------------------------------------------------------------ #
    def _check_airspeed_asymmetry(self, ax: str, now: float) -> None:
        if _airframe_class() not in ("FIXED_WING", "DELTA_WING", "VTOL"):
            return
        samples = self._aspd_samples[ax]
        if len(samples) < 2 * _ASPD_MIN_PER_BIN:
            return
        speeds = sorted(s for s, _ in samples)
        median_v = speeds[len(speeds) // 2]
        
        speeds_lo = [v for v, _ in samples if v < median_v]
        speeds_hi = [v for v, _ in samples if v >= median_v]
        if len(speeds_lo) < _ASPD_MIN_PER_BIN or len(speeds_hi) < _ASPD_MIN_PER_BIN:
            return
        if (np.mean(speeds_hi) - np.mean(speeds_lo)) < 10.0:
            return

        lo = [n for v, n in samples if v < median_v]
        hi = [n for v, n in samples if v >= median_v]
        med_lo = float(np.median(lo))
        med_hi = float(np.median(hi))

        key = f"aspd_{ax}"
        if now - self._last_alert.get(key, 0) < 60.0:
            return
        worse_hi = med_hi > _ASPD_RATIO * med_lo and med_hi > _ASPD_MIN_NRMSE
        worse_lo = med_lo > _ASPD_RATIO * med_hi and med_lo > _ASPD_MIN_NRMSE
        if not (worse_hi or worse_lo):
            return
        self._last_alert[key] = now
        where, other = ("high", "low") if worse_hi else ("low", "high")
        HUB.publish("alert", {
            "severity": "warning", "source": "pid",
            "text": (f"{ax.capitalize()} tracking is markedly worse at {where} "
                     f"airspeed (NRMSE {med_hi if worse_hi else med_lo:.2f} vs "
                     f"{med_lo if worse_hi else med_hi:.2f} at {other}) — airspeed "
                     f"scaling looks wrong. Verify FW_AIRSPD_TRIM / FW_AIRSPD_MIN "
                     f"against the real flight envelope before changing rate gains."),
        })


LIVE_PID = LivePidEngine()
