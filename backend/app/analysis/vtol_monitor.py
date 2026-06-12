"""
VTOL transition mechanics monitor.

Tracks the MAV_VTOL_STATE field of EXTENDED_SYS_STATE through the
hover -> transition -> fixed-wing sequence and watches the three classic
transition failure signatures:

1. Stuck transition: in TRANSITION_TO_FW too long while losing altitude
   and never reaching blend airspeed -> the timeout will abort the
   transition mid-air. Advise raising VT_TRANS_TIMEOUT *and* checking
   forward propulsion (a longer timeout cannot fix a dead pusher motor).

2. Post-transition pitch dip: lift motors cut (state flips to FW) and
   the nose immediately drops hard -> the wing wasn't generating enough
   lift at the configured transition airspeed. Advise raising
   VT_ARSP_TRANS so motors stay on until the wing is flying.

3. Elevon fight in hover: large surface deflections at near-zero
   airspeed while in MC mode -> surfaces are flailing against the
   multirotor controller (wind, bad mixing), wasting current and adding
   disturbance. Advise enabling VT_ELEV_MC_LOCK.

MAV_VTOL_STATE: 0 undefined, 1 transition->FW, 2 transition->MC,
3 MC, 4 FW.
"""
from __future__ import annotations

import asyncio
import math
import time
from collections import deque

from ..mavlink.connection import CONNECTION
from ..mavlink.telemetry_hub import HUB
from .recommendations import recommend

_TRANS_TO_FW, _TRANS_TO_MC, _STATE_MC, _STATE_FW = 1, 2, 3, 4

_STUCK_AFTER_S = 8.0
_STUCK_ALT_LOSS_M = 3.0
_DIP_WATCH_S = 3.0
_DIP_PITCH_RAD = math.radians(-10.0)
_DIP_ALT_LOSS_M = 2.0
_HOVER_AIRSPEED_MAX = 4.0
_ELEVON_FIGHT_STD = 0.25      # normalized deflection std over the window
_ALERT_COOLDOWN_S = 30.0


class VtolMonitor:
    """Transition-phase watchdog; only acts when airframe class is VTOL."""

    def __init__(self) -> None:
        self._state = 0
        self._trans_started: float | None = None
        self._trans_start_alt: float | None = None
        self._fw_entered: float | None = None
        self._fw_entry_alt: float | None = None
        self._dip_worst_pitch = 0.0
        self._alt = 0.0
        self._airspeed = 0.0
        self._deflections: deque[tuple[float, list[float]]] = deque()
        self._last_alert: dict[str, float] = {}
        self._task: asyncio.Task | None = None

    def start(self) -> None:
        if self._task is None or self._task.done():
            self._task = asyncio.create_task(self._run(), name="vtol-monitor")

    def stop(self) -> None:
        if self._task:
            self._task.cancel()

    @staticmethod
    def _is_vtol() -> bool:
        af = CONNECTION.state.airframe
        return af is not None and af.airframe_class == "VTOL"

    # ------------------------------------------------------------------ #
    async def _run(self) -> None:
        async for event in HUB.subscribe():
            if not self._is_vtol():
                continue
            now = time.monotonic()
            if event.channel == "extended_sys_state":
                self._on_state(int(event.payload.get("vtol_state", 0) or 0), now)
            elif event.channel == "vfr_hud":
                self._alt = float(event.payload.get("alt", 0) or 0)
                self._airspeed = float(event.payload.get("airspeed", 0) or 0)
                self._check_stuck_transition(now)
            elif event.channel == "attitude":
                self._watch_pitch_dip(event.payload, now)
            elif event.channel == "actuation":
                self._watch_elevon_fight(event.payload, now)

    # ------------------------------------------------------------------ #
    # 1) stuck front transition -> VT_TRANS_TIMEOUT
    # ------------------------------------------------------------------ #
    def _on_state(self, state: int, now: float) -> None:
        prev, self._state = self._state, state
        if state == _TRANS_TO_FW and prev != _TRANS_TO_FW:
            self._trans_started, self._trans_start_alt = now, self._alt
        elif state != _TRANS_TO_FW:
            self._trans_started = None
        if state == _STATE_FW and prev == _TRANS_TO_FW:
            # Lift motors just cut — open the pitch-dip watch window.
            self._fw_entered, self._fw_entry_alt = now, self._alt
            self._dip_worst_pitch = 0.0

    def _check_stuck_transition(self, now: float) -> None:
        if self._trans_started is None or self._trans_start_alt is None:
            return
        elapsed = now - self._trans_started
        alt_loss = self._trans_start_alt - self._alt
        if elapsed < _STUCK_AFTER_S or alt_loss < _STUCK_ALT_LOSS_M:
            return
        if now - self._last_alert.get("stuck", 0) < _ALERT_COOLDOWN_S:
            return
        self._last_alert["stuck"] = now
        HUB.publish("alert", {
            "severity": "critical", "source": "vtol",
            "text": (f"Transition to fixed-wing stuck for {elapsed:.0f}s with "
                     f"{alt_loss:.0f} m altitude loss at {self._airspeed:.0f} m/s — "
                     f"the timeout may abort it mid-air. Check forward propulsion "
                     f"authority; if thrust is healthy, allow more time."),
        })
        recommend("VT_TRANS_TIMEOUT",
                  f"Front transition still incomplete after {elapsed:.0f}s "
                  f"({alt_loss:.0f} m lost) — extend the timeout window. "
                  f"Only stage this if forward thrust is confirmed healthy.",
                  delta=+3.0, source="vtol_monitor", cooldown_s=60.0)

    # ------------------------------------------------------------------ #
    # 2) post-transition pitch dip -> VT_ARSP_TRANS
    # ------------------------------------------------------------------ #
    def _watch_pitch_dip(self, att: dict, now: float) -> None:
        if self._fw_entered is None:
            return
        if now - self._fw_entered > _DIP_WATCH_S:
            self._fw_entered = None
            return
        pitch = float(att.get("pitch", 0) or 0)
        self._dip_worst_pitch = min(self._dip_worst_pitch, pitch)
        alt_loss = (self._fw_entry_alt or 0) - self._alt
        if self._dip_worst_pitch > _DIP_PITCH_RAD and alt_loss < _DIP_ALT_LOSS_M:
            return
        if now - self._last_alert.get("dip", 0) < _ALERT_COOLDOWN_S:
            return
        self._last_alert["dip"] = now
        self._fw_entered = None
        HUB.publish("alert", {
            "severity": "critical", "source": "vtol",
            "text": (f"Pitch dipped to {math.degrees(self._dip_worst_pitch):.0f}° "
                     f"({alt_loss:.1f} m lost) right after lift motors cut — the "
                     f"wing was not generating enough lift at transition airspeed."),
        })
        recommend("VT_ARSP_TRANS",
                  f"Nose dropped {math.degrees(self._dip_worst_pitch):.0f}° the "
                  f"moment lift motors shut off — keep them running to a higher "
                  f"airspeed so the wing is flying before handover.",
                  delta=+2.0, source="vtol_monitor", cooldown_s=60.0)

    # ------------------------------------------------------------------ #
    # 3) elevon fight in hover -> VT_ELEV_MC_LOCK
    # ------------------------------------------------------------------ #
    def _watch_elevon_fight(self, payload: dict, now: float) -> None:
        if self._state != _STATE_MC or self._airspeed > _HOVER_AIRSPEED_MAX:
            self._deflections.clear()
            return
        defl = payload.get("surface_deflections")
        if not defl:
            return
        self._deflections.append((now, defl))
        while self._deflections and self._deflections[0][0] < now - 2.0:
            self._deflections.popleft()
        if len(self._deflections) < 10:
            return

        # Largest per-channel deflection std over the window.
        n_ch = len(self._deflections[-1][1])
        worst = max(
            _std([d[i] for _, d in self._deflections if i < len(d)])
            for i in range(n_ch)
        )
        if worst < _ELEVON_FIGHT_STD:
            return
        if now - self._last_alert.get("elevon", 0) < _ALERT_COOLDOWN_S:
            return
        self._last_alert["elevon"] = now
        HUB.publish("alert", {
            "severity": "warning", "source": "vtol",
            "text": (f"Control surfaces flailing in hover (deflection σ={worst:.2f} "
                     f"at {self._airspeed:.0f} m/s) — they are fighting the lift "
                     f"motors and burning current without authority."),
        })
        recommend("VT_ELEV_MC_LOCK",
                  "Surfaces are working hard in hover where they have no "
                  "aerodynamic authority — lock them in multirotor mode until "
                  "forward flight is established.",
                  target_value=1, source="vtol_monitor", cooldown_s=120.0)


def _std(xs: list[float]) -> float:
    if len(xs) < 2:
        return 0.0
    m = sum(xs) / len(xs)
    return (sum((x - m) ** 2 for x in xs) / len(xs)) ** 0.5


VTOL_MONITOR = VtolMonitor()
