"""
Flight regime state machine.

Classifies the live flight condition from the rolling variance (sigma^2)
of pilot stick inputs (MANUAL_CONTROL) plus coarse energy cues (VFR_HUD):

    PRE_FLIGHT       on the ground / idle throttle, no stick activity
    STEADY_HOLD      airborne, sticks quiet (hover or trimmed cruise)
    DYNAMIC_MANEUVER airborne, significant stick activity

Downstream engines gate on the current regime:
  * PID tracking analysis runs ONLY during DYNAMIC_MANEUVER — analysing a
    quiet hover produces false "sluggish" verdicts because nothing is
    being commanded.
  * EKF innovation alerts get extra weight during STEADY_HOLD — with the
    vehicle quiet, an innovation spike is environmental (magnetic
    interference, GPS multipath) or sensor timing, never tuning.

Transitions are debounced: a candidate state must persist for DWELL_S
before it becomes current, so a single stick blip doesn't thrash the
analysis pipelines.
"""
from __future__ import annotations

import asyncio
import statistics
import time
from collections import deque
from enum import Enum

from ..mavlink.telemetry_hub import HUB
from ..core import config


class Regime(str, Enum):
    PRE_FLIGHT = "pre_flight"
    STEADY_HOLD = "steady_hold"
    DYNAMIC_MANEUVER = "dynamic_maneuver"


# MANUAL_CONTROL axes are scaled -1000..1000.
_VARIANCE_WINDOW_S = config.REGIME_VARIANCE_WINDOW_S
_DYNAMIC_VARIANCE = config.REGIME_DYNAMIC_VARIANCE
_DWELL_S = config.REGIME_DWELL_S
_INFLIGHT_THROTTLE_PCT = config.REGIME_INFLIGHT_THROTTLE_PCT
_INFLIGHT_SPEED_M_S = config.REGIME_INFLIGHT_SPEED_M_S
_VFR_STALE_S = config.REGIME_VFR_STALE_S


class RegimeClassifier:
    """Stick-variance + energy-cue state machine publishing "regime"."""

    def __init__(self) -> None:
        self.current: Regime = Regime.PRE_FLIGHT
        self._sticks: deque[tuple[float, float, float, float]] = deque()
        self._candidate: Regime | None = None
        self._candidate_since: float = 0.0
        self._last_vfr: dict = {}
        self._last_vfr_at: float = 0.0
        self._last_heartbeat_pub: float = 0.0
        self._task: asyncio.Task | None = None

    def start(self) -> None:
        if self._task is None or self._task.done():
            self._task = asyncio.create_task(self._run(), name="regime-classifier")

    def stop(self) -> None:
        if self._task:
            self._task.cancel()

    # ------------------------------------------------------------------ #
    async def _run(self) -> None:
        async for event in HUB.subscribe(channels=frozenset({
            "vfr_hud", "manual_control"
        })):
            now = time.monotonic()
            if event.channel == "vfr_hud":
                self._last_vfr = event.payload
                self._last_vfr_at = now
            elif event.channel == "manual_control":
                p = event.payload
                self._sticks.append((
                    now,
                    float(p.get("x", 0) or 0),   # pitch
                    float(p.get("y", 0) or 0),   # roll
                    float(p.get("r", 0) or 0),   # yaw
                ))
                horizon = now - _VARIANCE_WINDOW_S
                while self._sticks and self._sticks[0][0] < horizon:
                    self._sticks.popleft()
                self._evaluate(now)

    # ------------------------------------------------------------------ #
    def _stick_variance(self) -> float:
        """Max per-axis population variance over the rolling window."""
        if len(self._sticks) < 5:
            return 0.0
        import numpy as np
        arr = np.array(self._sticks)
        return float(np.max(np.var(arr[:, 1:4], axis=0)))

    def _in_flight(self, now: float) -> bool:
        if now - self._last_vfr_at > _VFR_STALE_S:
            return False
        throttle = float(self._last_vfr.get("throttle", 0) or 0)
        speed = max(
            float(self._last_vfr.get("airspeed", 0) or 0),
            float(self._last_vfr.get("groundspeed", 0) or 0),
        )
        return throttle > _INFLIGHT_THROTTLE_PCT or speed > _INFLIGHT_SPEED_M_S

    def _evaluate(self, now: float) -> None:
        variance = self._stick_variance()
        if not self._in_flight(now):
            target = Regime.PRE_FLIGHT
        elif variance > _DYNAMIC_VARIANCE:
            target = Regime.DYNAMIC_MANEUVER
        else:
            target = Regime.STEADY_HOLD

        # Debounce: hold the candidate for DWELL_S before committing.
        if target != self.current:
            if target != self._candidate:
                self._candidate, self._candidate_since = target, now
            elif now - self._candidate_since >= _DWELL_S:
                self.current = target
                self._candidate = None
                self._publish(variance, changed=True)
                return
        else:
            self._candidate = None

        # Periodic heartbeat so a freshly opened UI learns the state.
        if now - self._last_heartbeat_pub > 2.0:
            self._publish(variance, changed=False)

    def _publish(self, variance: float, changed: bool) -> None:
        self._last_heartbeat_pub = time.monotonic()
        HUB.publish("regime", {
            "state": self.current.value,
            "stick_variance": round(variance, 1),
            "changed": changed,
        })


REGIME = RegimeClassifier()
