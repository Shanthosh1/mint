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
from collections import deque
from typing import Optional

import numpy as np

from ..mavlink.connection import CONNECTION
from ..mavlink.telemetry_hub import HUB
from . import loop_health
from .recommendations import recommend
from .regime import REGIME, Regime
from .vibration_live import VIB_GATE

_AXES = {
    "roll": ("rollspeed", "body_roll_rate"),
    "pitch": ("pitchspeed", "body_pitch_rate"),
    "yaw": ("yawspeed", "body_yaw_rate"),
}

_WINDOW_S = 3.0            # sliding capture window (dynamic regime)
_EVAL_PERIOD_S = 0.5
_R_DEPLETION = 0.85
_HIGH_RATE_RAD_S = 0.8
_OVERSHOOT_LIMIT = 0.15    # fraction of step amplitude
_OSC_CROSSINGS = 3         # residual zero-crossings ~ "oscillates twice"
_SETTLE_BAND = 0.2

# Steady-state offset (I-gain) detection — STEADY_HOLD regime.
_OFFSET_WINDOW_S = 3.0
_OFFSET_MIN_SPAN_S = 2.0
_OFFSET_RAD = math.radians(2.0)
_OFFSET_MAX_STD = math.radians(1.5)

# Airspeed-binned tracking asymmetry (fixed-wing classes).
_ASPD_SAMPLES = 60
_ASPD_MIN_PER_BIN = 10
_ASPD_RATIO = 1.8
_ASPD_MIN_NRMSE = 0.25

_ALERT_COOLDOWN_S = 15.0

_FW_PREFIX = {"roll": "FW_RR_", "pitch": "FW_PR_", "yaw": "FW_YR_"}


def _airframe_class() -> Optional[str]:
    af = CONNECTION.state.airframe
    return af.airframe_class if af else None


def _rate_param(axis: str, term: str) -> Optional[str]:
    """Resolve the rate-loop parameter for the detected airframe class.
    Returns None when no class is known — no class, no recommendations."""
    cls = _airframe_class()
    if cls in ("MULTIROTOR", "VTOL"):
        return f"MC_{axis.upper()}RATE_{term}"
    if cls in ("FIXED_WING", "DELTA_WING"):
        return _FW_PREFIX[axis] + term
    return None


def _quat_to_roll_pitch(q: list[float]) -> tuple[float, float]:
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    return roll, pitch


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
        self._last_eval = 0.0
        self._last_alert: dict[str, float] = {}
        self._task: asyncio.Task | None = None

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
            elif event.channel == "vfr_hud":
                self._airspeed = float(event.payload.get("airspeed", 0) or 0)
            elif event.channel == "attitude":
                self._ingest(event.payload)

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
            self._ingest_steady(att, now)
        else:  # PRE_FLIGHT
            for win in self._win.values():
                win.clear()
            self._offset_win["roll"].clear()
            self._offset_win["pitch"].clear()

    # ------------------------------------------------------------------ #
    # DYNAMIC_MANEUVER: step response + tracking correlation
    # ------------------------------------------------------------------ #
    def _ingest_dynamic(self, att: dict, now: float) -> None:
        horizon = now - _WINDOW_S
        for ax, (act_field, _) in _AXES.items():
            act, sp = att.get(act_field), self._latest_sp.get(ax)
            if act is None or sp is None:
                continue
            win = self._win[ax]
            win.append((now, sp, float(act)))
            while win and win[0][0] < horizon:
                win.popleft()

        if now - self._last_eval >= _EVAL_PERIOD_S:
            self._last_eval = now
            self._evaluate(now)

    def _evaluate(self, now: float) -> None:
        metrics: dict[str, dict] = {}
        worst_r = None
        for ax, win in self._win.items():
            if len(win) < 15:
                continue
            t = np.array([s[0] for s in win])
            sp = np.array([s[1] for s in win])
            act = np.array([s[2] for s in win])

            m = self._tracking_metrics(sp, act)
            step = self._step_response(t, sp, act)
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

            self._verdicts(ax, m, now)

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
                       min_amp: float = 0.15) -> Optional[dict]:
        """
        Isolate the largest commanded step and measure tau (63.2% rise),
        settling (into the +/-20% band), overshoot, and residual
        oscillation count — all relative to the step's own amplitude.
        `min_amp` is in the signal's own units (rad/s for rates, rad for
        attitude, m/s for velocity, m for position).
        """
        if len(sp) < 20:
            return None
        i = int(np.argmax(np.abs(np.diff(sp))))
        t0 = t[i + 1]

        pre_mask = (t >= t0 - 0.5) & (t < t0)
        post_mask = t >= t0
        if pre_mask.sum() < 3 or post_mask.sum() < 8:
            return None

        sp_pre = float(np.mean(sp[pre_mask]))
        sp_post = float(np.mean(sp[post_mask][3:]))
        amp = sp_post - sp_pre

        # Size-agnostic significance gate: the step must dwarf the
        # vehicle's own pre-step measurement noise.
        act_noise = float(np.std(act[pre_mask])) or 1e-6
        if abs(amp) < 4 * act_noise or abs(amp) < min_amp:
            return None
        if float(np.std(sp[post_mask][3:])) > 0.25 * abs(amp):
            return None   # ramp / continuous stirring, not a step

        ta, aa = t[post_mask], act[post_mask]
        sign = 1.0 if amp > 0 else -1.0

        crossed = sign * (aa - (sp_pre + 0.632 * amp)) >= 0
        tau = float(ta[np.argmax(crossed)] - t0) if crossed.any() else None

        peak = float(np.max(sign * (aa - sp_post)))
        overshoot = max(0.0, peak / abs(amp))

        outside = np.abs(aa - sp_post) > _SETTLE_BAND * abs(amp)
        settling = (float(ta[np.where(outside)[0][-1]] - t0)
                    if outside.any() else 0.0)
        settled = not bool(outside[-1])

        # Residual oscillation: zero-crossings of significant excursions
        # around the new setpoint. >= _OSC_CROSSINGS means the response
        # rings ("oscillates twice") rather than just overshooting once.
        resid = aa - sp_post
        sig = resid[np.abs(resid) > max(0.08 * abs(amp), 2 * act_noise)]
        crossings = int(np.count_nonzero(np.diff(np.sign(sig)) != 0)) if sig.size > 1 else 0

        return {
            "tau_s": round(tau, 3) if tau is not None else None,
            "settling_s": round(settling, 3) if settled else None,
            "overshoot": round(overshoot, 3),
            "oscillations": crossings,
            "step_amp_deg_s": round(math.degrees(amp), 1),
        }

    # ------------------------------------------------------------------ #
    def _verdicts(self, ax: str, m: dict, now: float) -> None:
        """Turn one window's metrics into alerts + stageable advice."""
        r, tau, settling = m.get("r"), m.get("tau_s"), m.get("settling_s")
        overshoot, osc = m.get("overshoot", 0.0), m.get("oscillations", 0)

        if now - self._last_alert.get(ax, 0) < _ALERT_COOLDOWN_S:
            return

        if m.get("high_rate") and r is not None and r < _R_DEPLETION:
            # Authority problem — never recommend gains here.
            self._last_alert[ax] = now
            HUB.publish("alert", {
                "severity": "warning", "source": "pid",
                "text": (f"Tracking authority depletion on {ax}: r={r:.2f} "
                         f"(<{_R_DEPLETION}) during high-rate input. Check "
                         f"actuator saturation and vibration before touching gains."),
            })
            return

        if overshoot > _OVERSHOOT_LIMIT and osc >= _OSC_CROSSINGS:
            self._last_alert[ax] = now
            p_d = _rate_param(ax, "D")
            HUB.publish("alert", {
                "severity": "warning", "source": "pid",
                "text": (f"{ax.capitalize()} overshoots {overshoot*100:.0f}% and "
                         f"rings ({osc} reversals) before settling — damping deficit."),
            })
            # The D-term differentiates gyro noise straight into the
            # motors — never advise more D on a vibrating airframe.
            if p_d and VIB_GATE.ok():
                recommend(p_d, f"{ax} overshoot {overshoot*100:.0f}% with "
                               f"{osc} residual reversals — add derivative damping.",
                          delta=+0.001, source="live_pid")
            elif p_d:
                HUB.publish("alert", {
                    "severity": "warning", "source": "pid",
                    "text": (f"Withholding rate-D advice for {ax}: vibration is too "
                             f"high — more D would amplify it into the motors. Fix "
                             f"vibration/filtering first (see ULog analysis)."),
                })
            return

        if overshoot > _OVERSHOOT_LIMIT:
            self._last_alert[ax] = now
            p_p = _rate_param(ax, "P")
            HUB.publish("alert", {
                "severity": "warning", "source": "pid",
                "text": (f"Dynamic overshoot on {ax}: {overshoot*100:.0f}% of step "
                         f"amplitude (no ringing) — proportional gain too hot."),
            })
            if p_p:
                recommend(p_p, f"{ax} overshoot {overshoot*100:.0f}% without "
                               f"oscillation — back off proportional gain ~10%.",
                          scale_factor=0.9, source="live_pid")
            return

        if tau is not None and settling is not None and settling > 4 * tau:
            self._last_alert[ax] = now
            p_p = _rate_param(ax, "P")
            HUB.publish("alert", {
                "severity": "info", "source": "pid",
                "text": (f"Under-tuned {ax}: settling {settling:.2f}s exceeds "
                         f"4×τ ({tau:.2f}s)."),
            })
            if p_p and VIB_GATE.ok():
                recommend(p_p, f"{ax} settles in {settling:.2f}s vs τ={tau:.2f}s "
                               f"(>4×τ) — raise proportional gain ~10%.",
                          scale_factor=1.1, source="live_pid")
            elif p_p:
                HUB.publish("alert", {
                    "severity": "info", "source": "pid",
                    "text": (f"{ax.capitalize()} looks under-tuned, but gain raises "
                             f"are suspended while vibration is high."),
                })
            return

        if tau is None and m.get("step_amp_deg_s"):
            self._last_alert[ax] = now
            HUB.publish("alert", {
                "severity": "warning", "source": "pid",
                "text": (f"{ax.capitalize()} never reached 63.2% of the commanded "
                         f"step within the window — severely sluggish or saturated."),
            })

    # ------------------------------------------------------------------ #
    # STEADY_HOLD: integrator-deficit (steady-state offset) detection
    # ------------------------------------------------------------------ #
    def _ingest_steady(self, att: dict, now: float) -> None:
        if _airframe_class() not in ("MULTIROTOR", "VTOL"):
            return   # hover-offset logic is a rotor-borne concept
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
        p_i = _rate_param(ax, "I")
        HUB.publish("alert", {
            "severity": "info", "source": "pid",
            "text": (f"Steady-state {ax} offset of {math.degrees(mean):.1f}° held "
                     f">{_OFFSET_MIN_SPAN_S:.0f}s in hover — integrator deficit "
                     f"(wind or CG imbalance)."),
        })
        if p_i:
            recommend(p_i, f"Constant {math.degrees(mean):.1f}° {ax} offset during "
                           f"steady hover — raise integral gain to trim it out.",
                      delta=+0.05, source="live_pid", cooldown_s=60.0)

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
        lo = [n for v, n in samples if v < median_v]
        hi = [n for v, n in samples if v >= median_v]
        if len(lo) < _ASPD_MIN_PER_BIN or len(hi) < _ASPD_MIN_PER_BIN:
            return
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
