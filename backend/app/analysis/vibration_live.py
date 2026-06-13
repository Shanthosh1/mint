"""
Live vibration gate.

The MAVLink VIBRATION message is the one vibration signal that survives
a low-rate telemetry stream (the FC computes it onboard at full sensor
rate): per-axis accel-deviation metrics plus cumulative accelerometer
clipping counters.

Two jobs:

1. Pilot warnings — vibration metric over the QGC-conventional 30 / 60
   thresholds, or any new accel clipping (clipping = the worst case:
   the EKF is being fed truncated data).

2. Gain-advice gate — `ok()` is consulted by the live PID engine before
   any gain-RAISING recommendation (rate P up, rate D up). Raising
   gains on a vibrating airframe amplifies the noise into the motors;
   the right order is always: fix vibration -> filter -> then tune.
   When VIBRATION isn't streamed at all, the gate defaults to open so
   it never silently blocks advice on minimal telemetry setups.
"""
from __future__ import annotations

import asyncio
import time

from ..mavlink.telemetry_hub import HUB

_VIB_WARN = 30.0
_VIB_CRIT = 60.0
_CLIP_WINDOW_S = 30.0       # recent clipping blocks tuning this long
_ALERT_COOLDOWN_S = 30.0
_STALE_S = 2.0             # VIBRATION gone this long mid-flight => fail safe
_NEVER_STREAMED_S = 10.0   # never seen this long => assume link doesn't carry it


class VibrationGate:
    def __init__(self) -> None:
        self._latest: dict = {}
        self._last_seen = 0.0
        self._ever_seen = False
        self._last_clip_counts: tuple[int, int, int] | None = None
        self._last_clip_event = 0.0
        self._last_alert: dict[str, float] = {}
        self._task: asyncio.Task | None = None

    def start(self) -> None:
        if self._task is None or self._task.done():
            self._task = asyncio.create_task(self._run(), name="vibration-gate")

    def stop(self) -> None:
        if self._task:
            self._task.cancel()

    # ------------------------------------------------------------------ #
    def ok(self) -> bool:
        """Safe to recommend raising gains?

        Staleness policy distinguishes "lost the signal" from "link never
        carried it":
          * Seen before, then silent > _STALE_S  -> fail safe (block advice).
            Losing a vibration feed we were relying on is exactly when a
            gain raise could be dangerous, so we refuse to vouch for it.
          * Never seen for > _NEVER_STREAMED_S    -> open (don't block).
            Many minimal-telemetry links simply don't stream VIBRATION;
            treating its absence as "unsafe" would kill all gain advice.
        """
        now = time.monotonic()
        age = now - self._last_seen
        if self._ever_seen and age > _STALE_S:
            return False  # had a feed, lost it — do not vouch for gain raises
        if not self._ever_seen and age > _NEVER_STREAMED_S:
            return True   # link never carries VIBRATION — do not block advice
        worst = max(self._latest.get(k, 0.0) for k in ("x", "y", "z"))
        recently_clipped = now - self._last_clip_event < _CLIP_WINDOW_S
        return worst < _VIB_WARN and not recently_clipped

    # ------------------------------------------------------------------ #
    async def _run(self) -> None:
        async for event in HUB.subscribe():
            if event.channel != "vibration":
                continue
            self._process(event.payload, time.monotonic())

    def _process(self, p: dict, now: float) -> None:
        self._last_seen = now
        self._ever_seen = True
        self._latest = {
            "x": float(p.get("vibration_x", 0) or 0),
            "y": float(p.get("vibration_y", 0) or 0),
            "z": float(p.get("vibration_z", 0) or 0),
        }
        clips = tuple(int(p.get(f"clipping_{i}", 0) or 0) for i in range(3))

        new_clipping = (self._last_clip_counts is not None
                        and any(c > l for c, l in zip(clips, self._last_clip_counts)))
        self._last_clip_counts = clips
        if new_clipping:
            self._last_clip_event = now

        worst = max(self._latest.values())
        HUB.publish("vibration_metrics", {
            **self._latest, "clipping": list(clips), "ok": self.ok(),
        })

        if new_clipping and now - self._last_alert.get("clip", 0) > _ALERT_COOLDOWN_S:
            self._last_alert["clip"] = now
            HUB.publish("alert", {
                "severity": "critical", "source": "vibration",
                "text": "Accelerometer CLIPPING detected — the IMU is hitting its "
                        "measurement limits and the EKF is consuming truncated "
                        "data. Land and fix mounting/balance before any tuning.",
            })
        elif worst >= _VIB_CRIT and now - self._last_alert.get("crit", 0) > _ALERT_COOLDOWN_S:
            self._last_alert["crit"] = now
            HUB.publish("alert", {
                "severity": "critical", "source": "vibration",
                "text": f"Severe vibration (metric {worst:.0f} ≥ {_VIB_CRIT:.0f}). "
                        f"Tuning is pointless until the mechanical source is fixed.",
            })
        elif worst >= _VIB_WARN and now - self._last_alert.get("warn", 0) > _ALERT_COOLDOWN_S:
            self._last_alert["warn"] = now
            HUB.publish("alert", {
                "severity": "warning", "source": "vibration",
                "text": f"Elevated vibration (metric {worst:.0f} ≥ {_VIB_WARN:.0f}) — "
                        f"gain-raising advice is suspended. Check props, balance, "
                        f"and mount; run a ULog vibration analysis.",
            })


VIB_GATE = VibrationGate()
