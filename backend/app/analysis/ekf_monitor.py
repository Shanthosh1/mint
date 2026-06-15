"""
Real-time EKF diagnostician.

Consumes EKF_STATUS_REPORT and tracks innovation test ratios for the
velocity/position (GPS), magnetometer, and barometer (height) channels.
Ratio semantics: PX4 normalizes the innovation variance tests so 1.0
means the innovation hit its gate; sustained values near/above 1.0 mean
the EKF is on the verge of rejecting that sensor.

Beyond plain threshold alerts, two pattern detectors:

1. Mag spike during arming / throttle run-up — the mag ratio crossing
   0.8 while throttle ramps from idle is the signature of
   electromagnetic interference from battery/ESC current, the classic
   precursor to a toilet-bowl flyaway. Recommends raising
   EKF2_MAG_NOISE (and physically separating compass from power wiring).

2. Maneuver-correlated velocity innovations — per-regime running
   averages of the GPS-velocity and baro-height ratios. If the dynamic
   average is much worse than the steady average, position data is
   arriving out of phase with the IMU: a sensor *timing* problem
   (EKF2_GPS_DELAY / EKF2_BARO_DELAY), not a noise problem. The precise
   delay is measured offline (ULog cross-correlation); live we flag the
   signature and point at the post-flight tool.

Publishes "ekf_metrics" for the UI gauges plus alerts/recommendations.
"""
from __future__ import annotations

import asyncio
import time

from ..core import config
from ..mavlink.connection import CONNECTION
from ..mavlink.telemetry_hub import HUB
from .recommendations import recommend
from .regime import REGIME, Regime

# EKF_STATUS_REPORT fields -> friendly channel names.
_RATIO_FIELDS = {
    "velocity_variance": "gps_velocity",
    "pos_horiz_variance": "gps_position",
    "mag_variance": "magnetometer",
    "pos_vert_variance": "barometer",
}

_RUNUP_THROTTLE_LO = 5.0     # % — idle
_RUNUP_THROTTLE_HI = 30.0    # % — spool-up reached
_RUNUP_WINDOW_S = 10.0       # mag watched this long after spool-up starts
_RUNUP_MAG_RATIO = 0.8

_EMA_ALPHA = 0.05            # per-regime running averages
_PHASE_LAG_FACTOR = 2.0      # dynamic avg vs steady avg
_PHASE_LAG_MIN = 0.5         # dynamic avg must also be materially high
_PHASE_LAG_STEADY_MAX = 0.4  # steady avg must be healthy (else it's noise)

# EKF feed staleness (a dead estimator stream, distinct from bad ratios).
_STALE_CHECK_S = 0.5         # how often to test the EKF feed age
_STALE_MAX_AGE_S = 2.0       # no EKF_STATUS_REPORT this long => UNKNOWN


class EkfMonitor:
    """Streams innovation ratios; raises threshold + pattern alerts."""

    def __init__(self) -> None:
        self._last_alert: dict[str, float] = {}
        self._throttle = 0.0
        self._runup_started: float | None = None
        # per-(channel, regime) EMA of innovation ratios
        self._ema: dict[tuple[str, str], float] = {}
        self._task: asyncio.Task | None = None
        self._stale_task: asyncio.Task | None = None
        self._stale = False   # are we currently flagging EKF data as dead?

    def start(self) -> None:
        if self._task is None or self._task.done():
            self._task = asyncio.create_task(self._run(), name="ekf-monitor")
        if self._stale_task is None or self._stale_task.done():
            self._stale_task = asyncio.create_task(
                self._staleness_loop(), name="ekf-staleness")

    def stop(self) -> None:
        if self._task:
            self._task.cancel()
        if self._stale_task:
            self._stale_task.cancel()

    async def _run(self) -> None:
        async for event in HUB.subscribe():
            if event.channel == "vfr_hud":
                self._track_throttle(event.payload)
            elif event.channel == "ekf_status":
                if self._stale:
                    # Data resumed — clear the UNKNOWN state.
                    self._stale = False
                    HUB.publish("alert", {
                        "severity": "info", "source": "ekf",
                        "text": "EKF status reports resumed — gauges live again.",
                    })
                self._process(event.payload)
            elif event.channel == "estimator_status":
                if self._stale:
                    # Data resumed — clear the UNKNOWN state.
                    self._stale = False
                    HUB.publish("alert", {
                        "severity": "info", "source": "ekf",
                        "text": "EKF status reports resumed — gauges live again.",
                    })
                self._process_estimator_status(event.payload)

    async def _staleness_loop(self) -> None:
        """Flag a dead EKF feed so the gauges show UNKNOWN, not stale green.

        EKF_STATUS_REPORT or ESTIMATOR_STATUS can stop while attitude still streams (so the
        generic telemetry watchdog won't catch it). Once EKF data has been
        seen at least once, a gap longer than the threshold publishes an
        UNKNOWN metrics state plus a one-shot alert.
        """
        start_time = None
        has_alerted_missing = False
        try:
            while True:
                await asyncio.sleep(_STALE_CHECK_S)
                if not CONNECTION.state.connected:
                    start_time = None
                    has_alerted_missing = False
                    self._stale = False
                    continue
                if start_time is None:
                    start_time = time.monotonic()
                seen_ekf = HUB.last_seen("ekf_status")
                seen_est = HUB.last_seen("estimator_status")
                seen = None
                if seen_ekf is not None and seen_est is not None:
                    seen = max(seen_ekf, seen_est)
                elif seen_ekf is not None:
                    seen = seen_ekf
                elif seen_est is not None:
                    seen = seen_est

                if seen is None:
                    if not has_alerted_missing and (time.monotonic() - start_time > 5.0):
                        has_alerted_missing = True
                        HUB.publish("alert", {
                            "severity": "critical", "source": "ekf",
                            "text": "No EKF data received within 5 seconds of connection. Check PX4 configuration."
                        })
                    continue  # never started — nothing to call stale yet
                
                active_channel = "ekf_status" if seen_ekf is not None and (seen_est is None or seen_ekf > seen_est) else "estimator_status"
                if HUB.is_stale(active_channel, _STALE_MAX_AGE_S):
                    if not self._stale:
                        self._stale = True
                        HUB.publish("ekf_metrics", {
                            "ratios": None, "flags": None,
                            "regime": REGIME.current.value, "status": "unknown",
                        })
                        HUB.publish("alert", {
                            "severity": "critical", "source": "ekf",
                            "text": (f"No EKF status for >{_STALE_MAX_AGE_S:.0f}s — "
                                     f"estimator gauges are UNKNOWN, not healthy. "
                                     f"The EKF feed has stopped; do not trust the "
                                     f"last shown values."),
                        })
        except asyncio.CancelledError:
            pass

    # ------------------------------------------------------------------ #
    def _track_throttle(self, vfr: dict) -> None:
        """Detect the idle -> spool-up edge that opens the mag watch."""
        prev, self._throttle = self._throttle, float(vfr.get("throttle", 0) or 0)
        if prev < _RUNUP_THROTTLE_LO and self._throttle >= _RUNUP_THROTTLE_HI:
            self._runup_started = time.monotonic()

    def _in_runup_window(self, now: float) -> bool:
        return (self._runup_started is not None
                and now - self._runup_started <= _RUNUP_WINDOW_S)

    # ------------------------------------------------------------------ #
    def _handle_ratios_and_flags(self, ratios: dict[str, float], flags: int, now: float) -> None:
        HUB.publish("ekf_metrics", {
            "ratios": ratios, "flags": flags, "regime": REGIME.current.value,
            "status": "ok",
        })

        self._update_regime_stats(ratios)
        self._check_runup_mag(ratios, now)
        self._check_phase_lag(now)

        for label, ratio in ratios.items():
            severity = None
            if ratio >= config.EKF_RATIO_FAIL:
                severity = "critical"
            elif ratio >= config.EKF_RATIO_WARN:
                severity = "warning"
            if severity is None:
                continue
            if now - self._last_alert.get(label, 0) < 10:
                continue
            self._last_alert[label] = now
            HUB.publish("alert", {
                "severity": severity,
                "source": "ekf",
                "text": self._alert_text(label, ratio, severity),
            })

    def _process(self, report: dict) -> None:
        now = time.monotonic()
        ratios = {
            label: round(float(report.get(field, 0.0) or 0.0), 3)
            for field, label in _RATIO_FIELDS.items()
        }
        flags = int(report.get("flags", 0) or 0)
        self._handle_ratios_and_flags(ratios, flags, now)

    def _process_estimator_status(self, report: dict) -> None:
        now = time.monotonic()
        ratios = {
            "gps_velocity": round(float(report.get("vel_ratio", 0.0) or 0.0), 3),
            "gps_position": round(float(report.get("pos_horiz_ratio", 0.0) or 0.0), 3),
            "magnetometer": round(float(report.get("mag_ratio", 0.0) or 0.0), 3),
            "barometer": round(float(report.get("pos_vert_ratio", 0.0) or 0.0), 3),
        }
        flags = int(report.get("flags", 0) or 0)
        self._handle_ratios_and_flags(ratios, flags, now)

    # ------------------------------------------------------------------ #
    # Pattern 1: mag interference during throttle run-up
    # ------------------------------------------------------------------ #
    def _check_runup_mag(self, ratios: dict, now: float) -> None:
        if not self._in_runup_window(now):
            return
        mag = ratios.get("magnetometer", 0.0)
        if mag < _RUNUP_MAG_RATIO:
            return
        if now - self._last_alert.get("runup_mag", 0) < 60:
            return
        self._last_alert["runup_mag"] = now
        HUB.publish("alert", {
            "severity": "critical", "source": "ekf",
            "text": (f"Magnetometer innovation spiked to {mag:.2f} during throttle "
                     f"run-up — electromagnetic interference from battery/ESC "
                     f"current. Flyaway risk: do NOT take off until resolved. "
                     f"Physically separate the compass from power wiring first."),
        })
        recommend("EKF2_MAG_NOISE",
                  f"Mag innovation ratio {mag:.2f} correlates with throttle "
                  f"run-up (EM interference). Raising the noise floor desensitizes "
                  f"fusion — a mitigation, not a fix for the wiring.",
                  scale_factor=1.2, source="ekf_monitor", cooldown_s=120.0)

    # ------------------------------------------------------------------ #
    # Pattern 2: maneuver-correlated innovations = sensor timing lag
    # ------------------------------------------------------------------ #
    def _update_regime_stats(self, ratios: dict) -> None:
        regime = REGIME.current.value
        if regime == Regime.PRE_FLIGHT.value:
            return
        for label in ("gps_velocity", "barometer"):
            key = (label, regime)
            prev = self._ema.get(key, ratios[label])
            self._ema[key] = (1 - _EMA_ALPHA) * prev + _EMA_ALPHA * ratios[label]

    def _check_phase_lag(self, now: float) -> None:
        for label, param in (("gps_velocity", "EKF2_GPS_DELAY"),
                             ("barometer", "EKF2_BARO_DELAY")):
            steady = self._ema.get((label, Regime.STEADY_HOLD.value))
            dynamic = self._ema.get((label, Regime.DYNAMIC_MANEUVER.value))
            if steady is None or dynamic is None:
                continue
            if not (dynamic > _PHASE_LAG_FACTOR * steady
                    and dynamic > _PHASE_LAG_MIN
                    and steady < _PHASE_LAG_STEADY_MAX):
                continue
            key = f"lag_{label}"
            if now - self._last_alert.get(key, 0) < 120:
                continue
            self._last_alert[key] = now
            HUB.publish("alert", {
                "severity": "warning", "source": "ekf",
                "text": (f"EKF {label.replace('_', ' ')} innovations average "
                         f"{dynamic:.2f} during maneuvers but only {steady:.2f} in "
                         f"steady flight — the sensor data is arriving out of sync "
                         f"with the IMU ({param} phase lag). Upload this flight's "
                         f"ULog for a measured delay correction."),
            })

    # ------------------------------------------------------------------ #
    @staticmethod
    def _alert_text(label: str, ratio: float, severity: str) -> str:
        """Regime-aware diagnosis: the same ratio means different things
        depending on what the vehicle was doing when it spiked."""
        base = (
            f"EKF {label.replace('_', ' ')} innovation ratio at {ratio:.2f} — "
            f"{'sensor fusion REJECTING data' if severity == 'critical' else 'approaching rejection gate'}."
        )
        if REGIME.current == Regime.STEADY_HOLD:
            hints = {
                "magnetometer": "Vehicle is in steady hold, so this is environmental "
                                "or structural — magnetic interference from power wiring, "
                                "payload, or nearby steel, not tuning.",
                "gps_velocity": "Vehicle is in steady hold — suspect GPS multipath, "
                                "antenna placement, or a wrong EKF2_GPS_DELAY.",
                "gps_position": "Vehicle is in steady hold — suspect GPS multipath or "
                                "degraded satellite geometry, not tuning.",
                "barometer": "Vehicle is in steady hold — suspect prop wash over the "
                             "baro, airflow into the canopy, or thermal drift.",
            }
            return f"{base} {hints.get(label, '')}".strip()
        if REGIME.current == Regime.DYNAMIC_MANEUVER:
            return (f"{base} Raised during aggressive maneuvering — re-check during "
                    f"a steady hold before attributing it to a sensor fault.")
        return f"{base} Check sensor health before continuing tuning."


EKF_MONITOR = EkfMonitor()
