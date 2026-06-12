"""
Pilot input assistant.

Two responsibilities, both fed by MANUAL_CONTROL:

1. Tuning-window prompts — if stick variance is ~zero for longer than
   STICK_IDLE_TIMEOUT_S during an active tuning window, prompt the pilot
   to fly deliberate step inputs and validate compliance.

2. Control-authority watch (fixed-wing classes) — if the pilot is
   repeatedly pinning the sticks at the rails (+/-1000) just to maneuver,
   the manual deflection scaling is robbing them of authority: recommend
   raising FW_MAN_R_SC / FW_MAN_P_SC (clamped to <=1.0 by the registry).
"""
from __future__ import annotations

import asyncio
import statistics
import time
from collections import deque
from typing import Optional

from ..core import config
from ..mavlink.connection import CONNECTION
from ..mavlink.telemetry_hub import HUB
from .recommendations import recommend
from .regime import REGIME, Regime

# Rail-pinning detection (fixed-wing manual scaling).
_RAIL_THRESHOLD = 950          # |stick| >= this counts as pinned
_RAIL_WINDOW_S = 30.0
_RAIL_FRACTION = 0.2           # pinned this share of a dynamic window
_RAIL_MIN_SAMPLES = 50
_RAIL_PARAM = {"roll": "FW_MAN_R_SC", "pitch": "FW_MAN_P_SC"}
_RAIL_FIELD = {"roll": "y", "pitch": "x"}

# MANUAL_CONTROL fields are scaled -1000..1000 by convention.
_VARIANCE_FLOOR = 25.0          # below this the sticks are "idle"
_COMPLIANCE_PEAK = 400.0        # a real step input swings at least this far
_COMPLIANCE_REVERSALS = 3       # "three sharp, alternating inputs"

_AXIS_FIELDS = {"roll": "y", "pitch": "x", "yaw": "r"}

_PROMPTS = {
    "roll": "System requires roll data: execute three sharp, alternating "
            "roll inputs when safe.",
    "pitch": "System requires pitch data: execute three sharp, alternating "
             "pitch inputs when safe.",
    "yaw": "System requires yaw data: execute three sharp, alternating "
           "yaw inputs when safe.",
}


class StickMonitor:
    """Idle detection + maneuver compliance validation, one axis at a time."""

    def __init__(self) -> None:
        self._history: deque[tuple[float, dict[str, float]]] = deque(maxlen=400)
        self._tuning_active = False
        self._needed_axis: Optional[str] = None
        self._prompt_issued_at: Optional[float] = None
        # rail-pinning: per-axis (t, pinned) over a long rolling window
        self._rail: dict[str, deque] = {"roll": deque(), "pitch": deque()}
        self._task: asyncio.Task | None = None

    # -- session control (driven by the API layer) -----------------------
    def begin_window(self, axis: str) -> None:
        """Open a dynamic-testing window that needs data on `axis`."""
        self._tuning_active = True
        self._needed_axis = axis
        self._prompt_issued_at = None
        self._history.clear()

    def end_window(self) -> None:
        self._tuning_active = False
        self._needed_axis = None
        self._prompt_issued_at = None

    def start(self) -> None:
        if self._task is None or self._task.done():
            self._task = asyncio.create_task(self._run(), name="stick-monitor")

    # -- internals --------------------------------------------------------
    async def _run(self) -> None:
        async for event in HUB.subscribe():
            if event.channel != "manual_control":
                continue
            now = time.monotonic()
            self._watch_rails(event.payload, now)
            if not self._tuning_active:
                continue
            sticks = {
                ax: float(event.payload.get(field, 0) or 0)
                for ax, field in _AXIS_FIELDS.items()
            }
            self._history.append((now, sticks))
            self._evaluate(now)

    # -- control-authority watch (rail pinning) ---------------------------
    def _watch_rails(self, payload: dict, now: float) -> None:
        af = CONNECTION.state.airframe
        if af is None or af.airframe_class not in ("FIXED_WING", "DELTA_WING"):
            return
        if REGIME.current != Regime.DYNAMIC_MANEUVER:
            return   # idle/cruise stick positions say nothing about authority

        horizon = now - _RAIL_WINDOW_S
        for axis, field in _RAIL_FIELD.items():
            pinned = abs(float(payload.get(field, 0) or 0)) >= _RAIL_THRESHOLD
            win = self._rail[axis]
            win.append((now, pinned))
            while win and win[0][0] < horizon:
                win.popleft()
            if len(win) < _RAIL_MIN_SAMPLES:
                continue
            frac = sum(1 for _, p in win if p) / len(win)
            if frac >= _RAIL_FRACTION:
                win.clear()   # restart the window after advising
                recommend(
                    _RAIL_PARAM[axis],
                    f"Sticks pinned at full {axis} deflection {frac*100:.0f}% of "
                    f"the last {int(_RAIL_WINDOW_S)}s of maneuvering — pilot is "
                    f"running out of control authority. Raise manual surface "
                    f"scaling (capped at 1.0).",
                    scale_factor=1.15, source="stick_monitor", cooldown_s=60.0,
                )
                HUB.publish("alert", {
                    "severity": "info", "source": "sticks",
                    "text": (f"Pilot repeatedly at full {axis} stick during normal "
                             f"maneuvering — consider raising "
                             f"{_RAIL_PARAM[axis]} for more authority."),
                })

    def _evaluate(self, now: float) -> None:
        axis = self._needed_axis
        if axis is None:
            return
        # Never ask for step inputs on the ground — the prompt is only
        # meaningful (and safe) once the vehicle is actually flying.
        if REGIME.current == Regime.PRE_FLIGHT:
            return

        window = [(t, s[axis]) for t, s in self._history
                  if t >= now - config.STICK_IDLE_TIMEOUT_S]
        if len(window) < 5:
            return
        values = [v for _, v in window]
        variance = statistics.pvariance(values)

        if self._prompt_issued_at is None:
            if variance < _VARIANCE_FLOOR:
                self._prompt_issued_at = now
                HUB.publish("pilot_prompt", {
                    "axis": axis,
                    "text": _PROMPTS[axis],
                    "kind": "step_input_request",
                })
        else:
            if self._is_compliant(values):
                HUB.publish("pilot_prompt", {
                    "axis": axis,
                    "text": f"{axis.capitalize()} step inputs received — "
                            f"collecting response data.",
                    "kind": "compliance_ack",
                })
                self._prompt_issued_at = None

    @staticmethod
    def _is_compliant(values: list[float]) -> bool:
        """Require >= _COMPLIANCE_REVERSALS sign reversals at meaningful
        amplitude — i.e. genuine alternating step inputs, not drift."""
        peaks = [v for v in values if abs(v) > _COMPLIANCE_PEAK]
        if len(peaks) < _COMPLIANCE_REVERSALS:
            return False
        reversals = sum(
            1 for a, b in zip(peaks, peaks[1:]) if (a > 0) != (b > 0)
        )
        return reversals >= _COMPLIANCE_REVERSALS - 1


STICK_MONITOR = StickMonitor()
