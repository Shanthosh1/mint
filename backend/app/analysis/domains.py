"""
Dynamic domain partitioning + live actuation monitor.

Analytical checks are grouped by how the vehicle generates control
torque, not by airframe name:

  differential_thrust  (multirotor; VTOL in hover)
      -> motor saturation index from SERVO_OUTPUT_RAW,
         rate-tracking curves (live_pid), gyro vibration fields.

  aerodynamic_surface  (fixed-wing, delta; VTOL in transition/cruise)
      -> control surface rail detection (+/-1.0 equivalent) scaled by
         live dynamic-pressure energy from VFR_HUD.airspeed. A railed
         elevator at 8 m/s is physics; a railed elevator at cruise
         airspeed is a tuning/authority problem. Same rail, different
         verdict — the q-ratio makes the check size- and speed-agnostic.

VTOL is treated as "hybrid": both monitors run and the airspeed energy
naturally selects which one produces meaningful output.
"""
from __future__ import annotations

import asyncio
import time
from collections import deque

from ..mavlink.telemetry_hub import HUB

DOMAIN_BY_CLASS = {
    "MULTIROTOR": "differential_thrust",
    "FIXED_WING": "aerodynamic_surface",
    "DELTA_WING": "aerodynamic_surface",
    "VTOL": "hybrid",
}

_PWM_MIN, _PWM_MID, _PWM_RANGE = 1000.0, 1500.0, 500.0
_MOTOR_SAT = 0.98          # normalized output counted as saturated
_SURFACE_RAIL = 0.96       # |deflection| counted as railed
_SUSTAIN_WINDOW_S = 1.5
_SUSTAIN_FRACTION = 0.6    # saturated this share of the window => alert
_AIRSPEED_REF_WINDOW_S = 30.0
_ALERT_COOLDOWN_S = 15.0


def domain_for(airframe_class: str | None) -> str | None:
    return DOMAIN_BY_CLASS.get(airframe_class or "")


class ActuationMonitor:
    """Streams per-domain actuation health from SERVO_OUTPUT_RAW."""

    def __init__(self) -> None:
        self._domain: str | None = None
        self._airspeed: float = 0.0
        self._airspeed_hist: deque[tuple[float, float]] = deque()
        self._sat_hist: deque[tuple[float, bool]] = deque()
        self._last_alert: dict[str, float] = {}
        self._task: asyncio.Task | None = None

    def start(self) -> None:
        if self._task is None or self._task.done():
            self._task = asyncio.create_task(self._run(), name="actuation-monitor")

    def stop(self) -> None:
        if self._task:
            self._task.cancel()

    # ------------------------------------------------------------------ #
    async def _run(self) -> None:
        async for event in HUB.subscribe():
            if event.channel == "airframe":
                self._domain = domain_for(event.payload.get("airframe_class"))
            elif event.channel == "vfr_hud":
                self._track_airspeed(event.payload)
            elif event.channel == "servo_output" and self._domain:
                self._process(event.payload)

    def _track_airspeed(self, vfr: dict) -> None:
        now = time.monotonic()
        self._airspeed = float(vfr.get("airspeed", 0) or 0)
        if self._airspeed > 4.0:   # only meaningful forward flight samples
            self._airspeed_hist.append((now, self._airspeed))
        horizon = now - _AIRSPEED_REF_WINDOW_S
        while self._airspeed_hist and self._airspeed_hist[0][0] < horizon:
            self._airspeed_hist.popleft()

    def _reference_q_ratio(self) -> float | None:
        """Current dynamic pressure relative to recent-cruise reference.

        q is proportional to v^2, so the ratio needs no air-density or
        wing-area knowledge — it cancels out. None until a reference
        airspeed has been observed.
        """
        if len(self._airspeed_hist) < 10:
            return None
        ref = sorted(v for _, v in self._airspeed_hist)[len(self._airspeed_hist) // 2]
        if ref <= 0:
            return None
        return (self._airspeed / ref) ** 2

    # ------------------------------------------------------------------ #
    def _process(self, servo: dict) -> None:
        raws = [float(servo.get(f"servo{i}_raw", 0) or 0) for i in range(1, 9)]
        valid = [v for v in raws if 800 < v < 2300]
        if not valid:
            return
        now = time.monotonic()
        payload: dict = {"domain": self._domain}

        if self._domain in ("differential_thrust", "hybrid"):
            norms = [min(1.0, max(0.0, (v - _PWM_MIN) / 1000.0)) for v in valid]
            sat_index = sum(1 for n in norms if n >= _MOTOR_SAT) / len(norms)
            payload["motor_sat_index"] = round(sat_index, 2)
            payload["motor_max"] = round(max(norms), 2)
            self._sustained_check("motor", sat_index > 0, now, severity="warning",
                                  text="Motor saturation: one or more motors pinned at "
                                       "maximum output. Reduce aggressive rate gains, "
                                       "payload, or maneuver intensity.")

        if self._domain in ("aerodynamic_surface", "hybrid"):
            defl = [(v - _PWM_MID) / _PWM_RANGE for v in valid]
            railed = [i for i, d in enumerate(defl) if abs(d) >= _SURFACE_RAIL]
            q_ratio = self._reference_q_ratio()
            payload["surface_deflections"] = [round(d, 2) for d in defl]
            payload["railed_channels"] = railed
            payload["q_ratio"] = round(q_ratio, 2) if q_ratio is not None else None

            if railed and q_ratio is not None:
                if q_ratio >= 0.8:
                    self._sustained_check(
                        "surface_hi_q", True, now, severity="warning",
                        text="Control surface at full travel at cruise energy — "
                             "authority/tuning problem. Check FW rate gains and trim.")
                elif q_ratio < 0.5:
                    self._sustained_check(
                        "surface_lo_q", True, now, severity="info",
                        text="Control surface railed at low airspeed — expected "
                             "physics, not a tuning fault. Gain more airspeed before "
                             "judging response.")
            else:
                self._sustained_check("surface_hi_q", False, now, "", "")
                self._sustained_check("surface_lo_q", False, now, "", "")

        HUB.publish("actuation", payload)

    def _sustained_check(self, key: str, condition: bool, now: float,
                         severity: str, text: str) -> None:
        """Alert only when `condition` holds for most of a rolling window."""
        hist = self._sat_hist if key == "motor" else None
        if hist is None:
            # surface checks share the motor mechanism via per-key attrs
            hist = getattr(self, f"_hist_{key}", None)
            if hist is None:
                hist = deque()
                setattr(self, f"_hist_{key}", hist)
        hist.append((now, condition))
        horizon = now - _SUSTAIN_WINDOW_S
        while hist and hist[0][0] < horizon:
            hist.popleft()
        if not text or len(hist) < 5:
            return
        frac = sum(1 for _, c in hist if c) / len(hist)
        if frac >= _SUSTAIN_FRACTION and now - self._last_alert.get(key, 0) > _ALERT_COOLDOWN_S:
            self._last_alert[key] = now
            HUB.publish("alert", {"severity": severity, "source": "actuation", "text": text})


ACTUATION = ActuationMonitor()
