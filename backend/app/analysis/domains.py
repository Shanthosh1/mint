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
import logging
import time
from collections import deque

from ..mavlink.telemetry_hub import HUB
from ..mavlink.connection import CONNECTION
from .regime import REGIME, Regime
from ..core import config

_log = logging.getLogger(__name__)

DOMAIN_BY_CLASS = {
    "MULTIROTOR": "differential_thrust",
    "FIXED_WING": "aerodynamic_surface",
    "DELTA_WING": "aerodynamic_surface",
    "VTOL": "hybrid",
}

_PWM_MIN = config.DOMAINS_PWM_MIN
_PWM_MID = config.DOMAINS_PWM_MID
_PWM_RANGE = config.DOMAINS_PWM_RANGE
_MOTOR_SAT = config.DOMAINS_MOTOR_SAT
_SURFACE_RAIL = config.DOMAINS_SURFACE_RAIL
_SUSTAIN_WINDOW_S = config.DOMAINS_SUSTAIN_WINDOW_S
_SUSTAIN_FRACTION = config.DOMAINS_SUSTAIN_FRACTION
_AIRSPEED_REF_WINDOW_S = config.DOMAINS_AIRSPEED_REF_WINDOW_S
_ALERT_COOLDOWN_S = config.DOMAINS_ALERT_COOLDOWN_S

# Motor balance (steady hover only). A quad runs motors at different outputs
# in level hover to counter prop yaw torque and CG offset; that differential
# is symmetric about the mean and cancels when each motor's output is
# *time-averaged* over a hover window. What remains after averaging is a
# persistent asymmetry — the fault we flag (prop wear, ESC drift, CG offset).
_BALANCE_WINDOW_S = config.DOMAINS_BALANCE_WINDOW_S
_BALANCE_MIN_SAMPLES = config.DOMAINS_BALANCE_MIN_SAMPLES
_BALANCE_WARN_FRAC = config.DOMAINS_BALANCE_WARN_FRAC
_BALANCE_ALERT_COOLDOWN_S = config.DOMAINS_BALANCE_ALERT_COOLDOWN_S


def domain_for(airframe_class: str | None) -> str | None:
    return DOMAIN_BY_CLASS.get(airframe_class or "")


class ActuationMonitor:
    """Streams per-domain actuation health from SERVO_OUTPUT_RAW."""

    def __init__(self) -> None:
        self._domain: str | None = None
        self._airspeed: float = 0.0
        self._airspeed_hist: deque[tuple[float, float]] = deque()
        self._sat_hist: deque[tuple[float, bool]] = deque()
        # Per-motor rolling (ts, normalized-output) history for the hover
        # balance check; index i tracks motor i+1.
        self._motor_hist: dict[int, deque[tuple[float, float]]] = {}
        self._last_alert: dict[str, float] = {}
        self._task: asyncio.Task | None = None
        self._instance_buf: list[list[float]] = [[0.0] * 16, [0.0] * 16]
        self._last_instance0_time: float = 0.0

    @property
    def domain(self) -> str | None:
        if self._domain is not None:
            return self._domain
        if CONNECTION.state.airframe:
            self._domain = domain_for(CONNECTION.state.airframe.airframe_class)
        return self._domain

    @domain.setter
    def domain(self, val: str | None) -> None:
        self._domain = val

    def start(self) -> None:
        if self._task is None or self._task.done():
            self._task = asyncio.create_task(self._run(), name="actuation-monitor")

    def stop(self) -> None:
        if self._task:
            self._task.cancel()

    # ------------------------------------------------------------------ #
    async def _run(self) -> None:
        self._vtol_state = 0

        async for event in HUB.subscribe(channels=frozenset({
            "airframe", "vfr_hud", "extended_sys_state",
            "actuator_output_status",
        })):
            if event.channel == "airframe":
                self.domain = domain_for(event.payload.get("airframe_class"))
            elif event.channel == "vfr_hud":
                self._track_airspeed(event.payload)
            elif event.channel == "extended_sys_state":
                self._vtol_state = int(event.payload.get("vtol_state", 0) or 0)
            elif event.channel == "actuator_output_status":
                self._process_actuator_status(event.payload)

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

    def _process_actuator_status(self, payload: dict) -> None:
        actuator = payload.get("actuator") or []
        if not actuator:
            return
        import math, time

        family = getattr(CONNECTION.state, "actuator_family", None)

        if family == "PWM_IOMCU":
            # IOMCU sends MAIN and AUX as two separate 8-wide messages.
            block_size = 8
            now = time.monotonic()
            vals = list(actuator[:block_size]) + [0.0] * max(0, block_size - len(actuator))
            if now - self._last_instance0_time > 0.05:
                self._instance_buf[0] = vals
                self._last_instance0_time = now
            else:
                self._instance_buf[1] = vals
            values = self._instance_buf[0] + self._instance_buf[1]
        else:
            values = list(actuator)

        is_raw = any(not math.isnan(v) and abs(v) > 2.0 for v in values)
        self._process_channels(values, is_raw=is_raw)

    def _process_channels(self, values: list[float], is_raw: bool) -> None:
        now = time.monotonic()
        
        # Determine mapping channels — exclusively from dynamic discovery
        actuator_map = getattr(CONNECTION.state, "actuator_map", None)
        has_mapped_channels = False
        if actuator_map:
            has_mapped_channels = any(len(v) > 0 for v in actuator_map.values())
            
        payload: dict = {
            "domain": self.domain,
            "motor_norms": [],
            "motor_channels": [],
            "thrust_norms": [],
            "thrust_channels": [],
            "surface_deflections": [],
            "surface_channels": [],
            "surface_names": [],
            "tilt_deflections": [],
            "tilt_channels": [],
            "railed_channels": [],
            "raw_channels": [],
            "unmapped": not has_mapped_channels,
        }
        
        if has_mapped_channels:
            hover_channels = actuator_map.get("hover_motors", [])
            thrust_channels = actuator_map.get("thrust_motors", [])
            surface_channels = actuator_map.get("control_surfaces", [])
            tilt_channels = actuator_map.get("tilt_servos", [])
        else:
            hover_channels = []
            thrust_channels = []
            surface_channels = []
            tilt_channels = []

        # ---- Always include ALL mapped channels + active unmapped as raw bars ----
        classified = set(hover_channels) | set(thrust_channels) | set(surface_channels) | set(tilt_channels)
        raw_channels = []
        raw_pwms_by_ch = {}
        no_data_surface_ch: set[int] = set()
        for i, v in enumerate(values):
            is_mapped = i in classified
            
            # Fetch limits from connection state
            limits = getattr(CONNECTION.state, "actuator_limits", {}).get(i)
            if limits:
                ch_min = limits["min"]
                ch_max = limits["max"]
                ch_trim = limits["trim"]
            else:
                ch_min = _PWM_MIN
                ch_max = _PWM_MIN + _PWM_RANGE * 2
                ch_trim = _PWM_MID

            if is_raw:
                raw_pwm = v
            else:
                import math
                raw_v = 0.0 if math.isnan(v) else v
                is_motor = (i in hover_channels) or (i in thrust_channels)
                if is_motor:
                    # Unidirectional mapping: [-1.0, 1.0] -> [ch_min, ch_max]
                    raw_pwm = ch_min + (raw_v + 1.0) / 2.0 * (ch_max - ch_min)
                else:
                    # Bidirectional mapping with ch_trim as center
                    if raw_v >= 0.0:
                        raw_pwm = ch_trim + raw_v * (ch_max - ch_trim)
                    else:
                        raw_pwm = ch_trim + raw_v * (ch_trim - ch_min)

            if raw_pwm < ch_min:
                norm = 0.0
            else:
                norm = (raw_pwm - ch_min) / max(1.0, ch_max - ch_min)
                norm = min(1.0, max(0.0, norm))

            raw_pwms_by_ch[i] = raw_pwm

            if is_raw:
                admit = is_mapped or v >= ch_min
            else:
                import math
                admit = is_mapped or (not math.isnan(v) and abs(v) > 0.001)

            if admit:
                is_surface_or_tilt = (i in surface_channels) or (i in tilt_channels)
                data_available = not (is_raw and round(raw_pwm) == 0 and is_surface_or_tilt)
                if not data_available:
                    no_data_surface_ch.add(i)
                raw_channels.append({
                    "ch": i + 1,
                    "raw": round(raw_pwm),
                    "norm": round(norm, 3),
                    "min": ch_min,
                    "max": ch_max,
                    "trim": ch_trim,
                    "classified": is_mapped,
                    "data_available": data_available,
                })
        payload["raw_channels"] = raw_channels

        # 1. Process Hover Motors — always include mapped channels
        norms = []
        hover_mapped_ch = []
        if hover_channels:
            for ch in hover_channels:
                if ch < len(values):
                    raw_pwm = raw_pwms_by_ch.get(ch, _PWM_MID)
                    limits = getattr(CONNECTION.state, "actuator_limits", {}).get(ch)
                    if limits:
                        ch_min = limits["min"]
                        ch_max = limits["max"]
                    else:
                        ch_min = _PWM_MIN
                        ch_max = _PWM_MIN + _PWM_RANGE * 2
                    if raw_pwm < ch_min:
                        norms.append(0.0)
                    else:
                        norms.append(min(1.0, max(0.0, (raw_pwm - ch_min) / max(1.0, ch_max - ch_min))))
                    hover_mapped_ch.append(ch + 1)
        elif not has_mapped_channels:
            # Fallback: treat all active channels as motors for saturation check
            norms = [ch["norm"] for ch in raw_channels]
            hover_mapped_ch = [ch["ch"] for ch in raw_channels]

            # Heuristic to guess surface-like channels for UI raw bars
            guessed_defls = []
            guessed_surf_ch = []
            guessed_names = []
            for ch_info in raw_channels:
                ch_idx = ch_info["ch"] - 1
                raw_val = ch_info["raw"]
                limits = getattr(CONNECTION.state, "actuator_limits", {}).get(ch_idx)
                is_guessed_motor = True
                if limits:
                    ch_min = limits.get("min", _PWM_MIN)
                    ch_trim = limits.get("trim", _PWM_MID)
                    if abs(ch_trim - ch_min) > 100:
                        is_guessed_motor = False
                else:
                    if is_raw:
                        if abs(raw_val - _PWM_MID) < 100:
                            is_guessed_motor = False
                    else:
                        v = values[ch_idx]
                        import math
                        if not math.isnan(v) and v < -0.05:
                            is_guessed_motor = False
                if not is_guessed_motor:
                    ch_min = limits.get("min", _PWM_MIN) if limits else _PWM_MIN
                    ch_max = limits.get("max", _PWM_MIN + _PWM_RANGE * 2) if limits else (_PWM_MIN + _PWM_RANGE * 2)
                    ch_trim = limits.get("trim", _PWM_MID) if limits else _PWM_MID
                    if raw_val <= ch_trim:
                        deflection = (raw_val - ch_trim) / max(1.0, ch_trim - ch_min)
                    else:
                        deflection = (raw_val - ch_trim) / max(1.0, ch_max - ch_trim)
                    deflection = min(1.0, max(-1.0, deflection))
                    guessed_defls.append(round(deflection, 2))
                    guessed_surf_ch.append(ch_info["ch"])
                    guessed_names.append(f"Surface {len(guessed_names) + 1}")

            if guessed_defls:
                payload["surface_deflections"] = guessed_defls
                payload["surface_channels"] = guessed_surf_ch
                payload["surface_names"] = guessed_names
                        
        if self.domain == "hybrid" and self._vtol_state == 4:
            norms = [0.0] * len(norms)

        if norms:
            sat_index = sum(1 for n in norms if n >= _MOTOR_SAT) / len(norms)
            payload["motor_sat_index"] = round(sat_index, 2)
            payload["motor_max"] = round(max(norms), 2)
            payload["motor_norms"] = [round(n, 3) for n in norms]
            payload["motor_channels"] = hover_mapped_ch if has_mapped_channels else []
            self._sustained_check("motor", sat_index > 0, now, severity="warning",
                                   text="Motor saturation: one or more motors pinned at "
                                        "maximum output. Reduce aggressive rate gains, "
                                        "payload, or maneuver intensity.")
            if has_mapped_channels:
                balance = self._check_motor_balance(norms, now)
                if balance is not None:
                    payload["motor_balance"] = balance

        # 2. Process Thrust Motors — always include mapped channels
        thrust_norms = []
        thrust_mapped_ch = []
        if thrust_channels:
            for ch in thrust_channels:
                if ch < len(values):
                    raw_pwm = raw_pwms_by_ch.get(ch, _PWM_MID)
                    limits = getattr(CONNECTION.state, "actuator_limits", {}).get(ch)
                    if limits:
                        ch_min = limits["min"]
                        ch_max = limits["max"]
                    else:
                        ch_min = _PWM_MIN
                        ch_max = _PWM_MIN + _PWM_RANGE * 2
                    if raw_pwm < ch_min:
                        thrust_norms.append(0.0)
                    else:
                        thrust_norms.append(min(1.0, max(0.0, (raw_pwm - ch_min) / max(1.0, ch_max - ch_min))))
                    thrust_mapped_ch.append(ch + 1)
        if thrust_norms:
            payload["thrust_norms"] = [round(tn, 3) for tn in thrust_norms]
            payload["thrust_channels"] = thrust_mapped_ch

        # 3. Process Control Surfaces — always include mapped channels
        defl = []
        surf_mapped_ch = []
        surf_names = []
        if surface_channels:
            for ch in surface_channels:
                if ch < len(values):
                    raw_pwm = raw_pwms_by_ch.get(ch, _PWM_MID)
                    limits = getattr(CONNECTION.state, "actuator_limits", {}).get(ch)
                    if limits:
                        ch_min = limits["min"]
                        ch_max = limits["max"]
                        ch_trim = limits["trim"]
                    else:
                        ch_min = _PWM_MIN
                        ch_max = _PWM_MIN + _PWM_RANGE * 2
                        ch_trim = _PWM_MID
                    if raw_pwm < ch_min:
                        defl.append(0.0)
                    else:
                        if raw_pwm <= ch_trim:
                            deflection = (raw_pwm - ch_trim) / max(1.0, ch_trim - ch_min)
                        else:
                            deflection = (raw_pwm - ch_trim) / max(1.0, ch_max - ch_trim)
                        deflection = min(1.0, max(-1.0, deflection))
                        defl.append(deflection)
                    surf_mapped_ch.append(ch + 1)
                    name = getattr(CONNECTION.state, "actuator_names", {}).get(ch, f"Surface {len(surf_names) + 1}")
                    surf_names.append(name)

        if defl:
            # Skip channels with no upstream data from the railing check.
            railed = [i for i, d in enumerate(defl)
                      if abs(d) >= _SURFACE_RAIL and surface_channels[i] not in no_data_surface_ch]
            q_ratio = self._reference_q_ratio()
            payload["surface_deflections"] = [round(d, 2) for d in defl]
            payload["surface_channels"] = surf_mapped_ch
            payload["surface_names"] = surf_names
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

        # 4. Process Tilt Servos — always include mapped channels
        tilt_defl = []
        tilt_mapped_ch = []
        if tilt_channels:
            for ch in tilt_channels:
                if ch < len(values):
                    raw_pwm = raw_pwms_by_ch.get(ch, _PWM_MID)
                    limits = getattr(CONNECTION.state, "actuator_limits", {}).get(ch)
                    if limits:
                        ch_min = limits["min"]
                        ch_max = limits["max"]
                        ch_trim = limits["trim"]
                    else:
                        ch_min = _PWM_MIN
                        ch_max = _PWM_MIN + _PWM_RANGE * 2
                        ch_trim = _PWM_MID
                    if raw_pwm < ch_min:
                        tilt_defl.append(0.0)
                    else:
                        if raw_pwm <= ch_trim:
                            deflection = (raw_pwm - ch_trim) / max(1.0, ch_trim - ch_min)
                        else:
                            deflection = (raw_pwm - ch_trim) / max(1.0, ch_max - ch_trim)
                        deflection = min(1.0, max(-1.0, deflection))
                        tilt_defl.append(deflection)
                    tilt_mapped_ch.append(ch + 1)
        if tilt_defl:
            payload["tilt_deflections"] = [round(td, 2) for td in tilt_defl]
            payload["tilt_channels"] = tilt_mapped_ch

        HUB.publish("actuation", payload)

    def _check_motor_balance(self, norms: list[float], now: float) -> dict | None:
        """Sustained per-motor asymmetry during steady hover.

        Only meaningful in STEADY_HOLD: outside it, differential thrust is
        the pilot/controller maneuvering, not a fault. Accumulates each
        motor's output over a hover window, then compares time-averaged
        outputs — the legitimate yaw-torque differential averages out, so a
        motor whose *average* sits >15% off the fleet mean indicates uneven
        prop wear, ESC calibration drift, or CG imbalance.

        Returns a balance summary for the UI (or None when not enough hover
        data yet), and raises a cooldown-gated advisory when out of balance.
        """
        # Reset history the moment we leave steady hover so a maneuver's
        # differential thrust never leaks into the next hover average.
        if REGIME.current != Regime.STEADY_HOLD:
            for hist in self._motor_hist.values():
                while hist and hist[0][0] < now - 0.5:
                    hist.popleft()
            return None

        horizon = now - _BALANCE_WINDOW_S
        for i, n in enumerate(norms):
            hist = self._motor_hist.setdefault(i, deque())
            hist.append((now, n))
            while hist and hist[0][0] < horizon:
                hist.popleft()

        # Every motor needs a full window of samples before we judge.
        counts = [len(self._motor_hist.get(i, ())) for i in range(len(norms))]
        if len(norms) < 3 or min(counts) < _BALANCE_MIN_SAMPLES:
            return None

        avgs = [sum(v for _, v in self._motor_hist[i]) / counts[i]
                for i in range(len(norms))]
        fleet_mean = sum(avgs) / len(avgs)
        if fleet_mean <= 1e-3:
            return None   # not actually spinning — ignore

        deviations = [(a - fleet_mean) / fleet_mean for a in avgs]
        worst_i = max(range(len(deviations)), key=lambda i: abs(deviations[i]))
        worst_dev = deviations[worst_i]

        summary = {
            "mean": round(fleet_mean, 3),
            "avgs": [round(a, 3) for a in avgs],
            "deviations": [round(d, 3) for d in deviations],
            "worst_motor": worst_i + 1,
            "worst_dev": round(worst_dev, 3),
            "balanced": abs(worst_dev) < _BALANCE_WARN_FRAC,
        }

        if (abs(worst_dev) >= _BALANCE_WARN_FRAC
                and now - self._last_alert.get("balance", 0) > _BALANCE_ALERT_COOLDOWN_S):
            self._last_alert["balance"] = now
            hi_lo = "higher" if worst_dev > 0 else "lower"
            HUB.publish("alert", {
                "severity": "warning", "source": "actuation",
                "text": (f"Motor imbalance in hover: motor {worst_i + 1} averages "
                         f"{abs(worst_dev) * 100:.0f}% {hi_lo} than the others. A "
                         f"motor working consistently harder points to uneven prop "
                         f"wear, ESC calibration drift, or a CG offset — inspect "
                         f"props/mount and check balance before tuning."),
            })
        return summary

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
