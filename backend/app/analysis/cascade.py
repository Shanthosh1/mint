"""
Cascaded outer-loop analyzers: attitude, velocity, position.

PX4's controllers are nested — rate inside attitude inside velocity
inside position — and each outer loop runs at a lower bandwidth than the
one it wraps. This module applies the same non-dimensional step/tracking
mathematics as the rate engine (live_pid) to each outer loop's own
signal pair, at its own timescale:

    loop      setpoint source                 actual source         window
    attitude  ATTITUDE_TARGET q (rad)         ATTITUDE angles        4 s
    velocity  POSITION_TARGET_LOCAL_NED v*    LOCAL_POSITION_NED v*  6 s
    position  POSITION_TARGET_LOCAL_NED x/y/z LOCAL_POSITION_NED     10 s

Three gates decide whether a loop is analysed at all:

1. Regime — only during DYNAMIC_MANEUVER (or AUTO modes where the FC
   itself generates the maneuvers).
2. Flight mode — a loop the FC isn't closing is skipped entirely
   (in ACRO the "attitude controller" is the pilot's thumbs).
3. Cascade health — recommendations (not metrics) are suppressed while
   an inner loop is impaired: tune inside-out, always.

For VTOLs the control domain (MC vs FW) follows MAV_VTOL_STATE live, so
attitude advice automatically targets MC_ROLL_P/MC_PITCH_P in hover and
FW_R_TC/FW_P_TC in forward flight.
"""
from __future__ import annotations

import asyncio
import math
import time
from collections import deque
from typing import Callable, Optional

import numpy as np

from ..mavlink.connection import CONNECTION
from ..mavlink.flight_modes import active_loops
from ..mavlink.telemetry_hub import HUB
from . import loop_health
from .live_pid import LivePidEngine, _quat_to_roll_pitch
from .recommendations import recommend
from .regime import REGIME, Regime
from ..core import config

_R_HEALTHY = 0.85
_VTOL_STATE_MC, _VTOL_STATE_FW = 3, 4

# POSITION_TARGET_LOCAL_NED type_mask bits: 0-2 ignore position,
# 3-5 ignore velocity.
_MASK_POS = 0b000_000_111
_MASK_VEL = 0b000_111_000


class CascadeState:
    """Live mode/domain context shared by the outer-loop analyzers."""

    def __init__(self) -> None:
        self.mode: str = "UNKNOWN"
        self.vtol_state: int = 0

    @property
    def domain(self) -> str:
        """Control domain right now: 'MC' or 'FW'."""
        af = CONNECTION.state.airframe
        cls = af.airframe_class if af else "MULTIROTOR"
        if cls == "VTOL":
            return "FW" if self.vtol_state == _VTOL_STATE_FW else "MC"
        return "FW" if cls in ("FIXED_WING", "DELTA_WING") else "MC"

    def loops_active(self) -> set[str]:
        return active_loops(self.mode, self.domain)

    def auto_flight(self) -> bool:
        """AUTO/OFFBOARD: the FC generates maneuvers, so outer loops can
        be analysed even when the pilot's sticks are quiet."""
        return self.mode.startswith(("AUTO", "OFFBOARD"))

    def snapshot(self) -> dict:
        return {
            "mode": self.mode,
            "domain": self.domain,
            "active_loops": sorted(self.loops_active()),
        }


STATE = CascadeState()


class _OuterLoop:
    """One outer tracking loop: windows, metrics, verdicts, advice."""

    def __init__(self, name: str, axes: list[str], window_s: float,
                 min_amp: float, unit: str,
                 advisor: Callable[[str, str, dict, str], None]):
        self.name = name
        self.axes = axes
        self.window_s = window_s
        self.min_amp = min_amp
        self.unit = unit
        self.advisor = advisor            # (axis, verdict, metrics, domain)
        self.win: dict[str, deque] = {ax: deque() for ax in axes}
        self.latest_sp: dict[str, float] = {}
        self.last_eval = 0.0
        self.last_advice: dict[str, float] = {}

    def clear(self) -> None:
        for w in self.win.values():
            w.clear()

    def feed_sp(self, values: dict[str, float]) -> None:
        self.latest_sp.update(values)

    def feed_act(self, values: dict[str, float], now: float) -> None:
        horizon = now - self.window_s
        for ax in self.axes:
            act, sp = values.get(ax), self.latest_sp.get(ax)
            if act is None or sp is None:
                continue
            w = self.win[ax]
            w.append((now, sp, act))
            while w and w[0][0] < horizon:
                w.popleft()
        if now - self.last_eval >= self.window_s / 4:
            self.last_eval = now
            self._evaluate(now)

    def _evaluate(self, now: float) -> None:
        metrics: dict[str, dict] = {}
        worst_r = None
        for ax, w in self.win.items():
            if len(w) < 12:
                continue
            t = np.array([s[0] for s in w])
            sp = np.array([s[1] for s in w])
            act = np.array([s[2] for s in w])
            m = LivePidEngine._tracking_metrics(sp, act)
            step = LivePidEngine._step_response(t, sp, act, min_amp=self.min_amp)
            if step:
                m.update(step)
            m["sp"], m["act"] = round(float(sp[-1]), 2), round(float(act[-1]), 2)
            metrics[ax] = m
            if m.get("r") is not None:
                worst_r = m["r"] if worst_r is None else min(worst_r, m["r"])
            self._verdict(ax, m, now)

        if worst_r is not None:
            loop_health.set_health(self.name, worst_r >= _R_HEALTHY)
        if metrics:
            HUB.publish("loop_metrics", {"loop": self.name, "axes": metrics,
                                         "unit": self.unit})

    def _verdict(self, ax: str, m: dict, now: float) -> None:
        if now - self.last_advice.get(ax, 0) < 30:
            return
        if not loop_health.inner_loop_healthy(self.name):
            return   # tune inside-out: inner loop first

        # Suppress cascade loop advice if a tuning window is open for a different loop or axis
        from .stick_monitor import STICK_MONITOR
        if STICK_MONITOR._tuning_active and (STICK_MONITOR._needed_loop != self.name or STICK_MONITOR._needed_axis != ax):
            return
        tau, settling = m.get("tau_s"), m.get("settling_s")
        overshoot = m.get("overshoot", 0.0)
        verdict = None
        if overshoot > 0.2:
            verdict = "overshoot"
        elif tau is not None and settling is not None and settling > 4 * tau:
            verdict = "sluggish"
        if verdict:
            self.last_advice[ax] = now
            self.advisor(ax, verdict, m, STATE.domain)


# ---------------------------------------------------------------------- #
# Per-loop advisors: map verdicts onto the right parameter family.
# ---------------------------------------------------------------------- #
def _attitude_advice(ax: str, verdict: str, m: dict, domain: str) -> None:
    pretty = f"{ax} attitude {verdict} (τ={m.get('tau_s')}s, " \
             f"overshoot {m.get('overshoot', 0)*100:.0f}%)"
    if domain == "MC":
        param = {"roll": "MC_ROLL_P", "pitch": "MC_PITCH_P", "yaw": "MC_YAW_P"}.get(ax)
        if param is None:
            return
        scale = 0.9 if verdict == "overshoot" else 1.1
        recommend(param, f"{pretty} — {'soften' if scale < 1 else 'raise'} the "
                         f"attitude P gain.", scale_factor=scale, source="cascade")
    else:
        # FW attitude is a time constant: bigger = slower. Sluggish means
        # the TC should shrink; overshoot means it should grow.
        param = {"roll": "FW_R_TC", "pitch": "FW_P_TC"}.get(ax)
        if param is None:
            return
        delta = +0.05 if verdict == "overshoot" else -0.05
        recommend(param, f"{pretty} — {'slow' if delta > 0 else 'tighten'} the "
                         f"{ax} time constant.", delta=delta, source="cascade")


def _velocity_advice(ax: str, verdict: str, m: dict, domain: str) -> None:
    if domain != "MC":
        return   # FW speed/altitude is TECS territory — out of live scope
    param = "MPC_Z_VEL_P_ACC" if ax == "vz" else "MPC_XY_VEL_P_ACC"
    scale = 0.9 if verdict == "overshoot" else 1.1
    recommend(param,
              f"{ax} velocity {verdict} (overshoot {m.get('overshoot', 0)*100:.0f}%, "
              f"τ={m.get('tau_s')}s) — {'soften' if scale < 1 else 'raise'} the "
              f"velocity P gain.", scale_factor=scale, source="cascade")


def _position_advice(ax: str, verdict: str, m: dict, domain: str) -> None:
    if domain != "MC":
        return
    param = "MPC_Z_P" if ax == "z" else "MPC_XY_P"
    scale = 0.9 if verdict == "overshoot" else 1.1
    recommend(param,
              f"{ax} position {verdict} across recent setpoint changes — "
              f"{'soften' if scale < 1 else 'raise'} the position P gain.",
              scale_factor=scale, source="cascade")


# ---------------------------------------------------------------------- #
class CascadeEngine:
    """Single hub consumer dispatching to the three outer loops."""

    def __init__(self) -> None:
        self.attitude = _OuterLoop("attitude", ["roll", "pitch"], 4.0,
                                   math.radians(config.CASCADE_ATTITUDE_MIN_AMP_DEG), "deg", _attitude_advice)
        self.velocity = _OuterLoop("velocity", ["vx", "vy", "vz"], 6.0,
                                   config.CASCADE_VELOCITY_MIN_AMP, "m/s", _velocity_advice)
        self.position = _OuterLoop("position", ["x", "y", "z"], 10.0,
                                   config.CASCADE_POSITION_MIN_AMP, "m", _position_advice)
        self._task: asyncio.Task | None = None

    def start(self) -> None:
        if self._task is None or self._task.done():
            self._task = asyncio.create_task(self._run(), name="cascade-engine")

    def stop(self) -> None:
        if self._task:
            self._task.cancel()

    # ------------------------------------------------------------------ #
    async def _run(self) -> None:
        async for event in HUB.subscribe():
            now = time.monotonic()
            ch, p = event.channel, event.payload
            if ch == "flight_mode":
                STATE.mode = p.get("mode", STATE.mode)
                HUB.publish("cascade_state", STATE.snapshot())
            elif ch == "extended_sys_state":
                prev = STATE.vtol_state
                STATE.vtol_state = int(p.get("vtol_state", 0) or 0)
                if STATE.vtol_state != prev:
                    HUB.publish("cascade_state", STATE.snapshot())
            elif ch == "attitude_target":
                self._feed_attitude_sp(p)
            elif ch == "attitude":
                self._feed(self.attitude,
                           {"roll": p.get("roll"), "pitch": p.get("pitch")}, now)
            elif ch == "position_target":
                self._feed_position_sp(p)
            elif ch == "local_position":
                self._feed(self.velocity,
                           {a: p.get(a) for a in ("vx", "vy", "vz")}, now)
                self._feed(self.position,
                           {a: p.get(a) for a in ("x", "y", "z")}, now)

    def _loop_enabled(self, loop: _OuterLoop) -> bool:
        if loop.name not in STATE.loops_active():
            return False
        # Pilot maneuvers OR autopilot-generated setpoint changes both
        # exercise the loops; in AUTO the sticks are silent by design.
        return (REGIME.current == Regime.DYNAMIC_MANEUVER
                or STATE.auto_flight())

    def _feed(self, loop: _OuterLoop, values: dict, now: float) -> None:
        if not self._loop_enabled(loop):
            loop.clear()
            return
        clean = {k: float(v) for k, v in values.items() if v is not None}
        if clean:
            loop.feed_act(clean, now)

    def _feed_attitude_sp(self, p: dict) -> None:
        q = p.get("q")
        if q and len(q) == 4:
            r, pitch = _quat_to_roll_pitch(q)
            self.attitude.feed_sp({"roll": r, "pitch": pitch})

    def _feed_position_sp(self, p: dict) -> None:
        mask = int(p.get("type_mask", 0) or 0)
        if (mask & _MASK_VEL) != _MASK_VEL:   # velocity fields valid
            self.velocity.feed_sp({a: float(p[a]) for a in ("vx", "vy", "vz")
                                   if p.get(a) is not None})
        if (mask & _MASK_POS) != _MASK_POS:   # position fields valid
            self.position.feed_sp({a: float(p[a]) for a in ("x", "y", "z")
                                   if p.get(a) is not None})


CASCADE = CascadeEngine()
