"""
Vehicle connection manager: MAVSDK (control plane) + pymavlink (data plane).

Separation of concerns:
  * MAVSDK   — connection state, parameter get/set, airframe detection.
               Bound to udp:14540.
  * pymavlink— raw high-rate message firehose feeding the analysis engines.
               Bound to udp:14541. Runs in a dedicated thread because
               pymavlink's recv_match is blocking; samples are handed to
               the asyncio world via loop.call_soon_threadsafe.

Nothing in this module ever writes a parameter without going through
`write_approved_param`, which is only called from the explicit
"Approve & Write" REST endpoint.
"""
from __future__ import annotations

import asyncio
import logging
import threading
from dataclasses import dataclass
from typing import Optional

from mavsdk import System
from pymavlink import mavutil

from ..core import config
from .airframe import AirframeInfo, classify_airframe, classify_mav_type
from .flight_modes import decode_mode
from .router_manager import ROUTER
from .telemetry_hub import HUB

log = logging.getLogger("mint.connection")

# Raw messages the analyzer thread forwards into the hub.
_WATCHED_MESSAGES = {
    "ATTITUDE": "attitude",
    "ATTITUDE_TARGET": "attitude_target",
    "MANUAL_CONTROL": "manual_control",
    "EKF_STATUS_REPORT": "ekf_status",
    "ESTIMATOR_STATUS": "estimator_status",
    "VFR_HUD": "vfr_hud",
    "SYS_STATUS": "sys_status",
    "SERVO_OUTPUT_RAW": "servo_output",   # actuation monitor (domains.py)
    "VIBRATION": "vibration",             # live vibration fields
    "EXTENDED_SYS_STATE": "extended_sys_state",  # VTOL transition state
    "LOCAL_POSITION_NED": "local_position",       # velocity/position loops
    "POSITION_TARGET_LOCAL_NED": "position_target",  # velocity/position setpoints
}

_MAV_TYPE_GCS = 6  # ignore heartbeats from other ground stations on the link


@dataclass
class VehicleState:
    connected: bool = False
    airframe: Optional[AirframeInfo] = None
    system_id: Optional[int] = None


class ConnectionManager:
    """Owns the MAVSDK system object and the pymavlink listener thread."""

    def __init__(self) -> None:
        self.state = VehicleState()
        self._system: Optional[System] = None
        self._pymav_thread: Optional[threading.Thread] = None
        self._stop_flag = threading.Event()
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._mav_type_seen = False
        self._last_mode: tuple[int, bool] | None = None

    # ------------------------------------------------------------------ #
    # Lifecycle
    # ------------------------------------------------------------------ #
    async def connect(self) -> VehicleState:
        """Connect MAVSDK to the routed UDP endpoint and start the raw listener.

        Ports come from the RouterManager, not static config: a udp_listen
        source (e.g. local SITL) may have shifted the fan-out to alternate
        ports to avoid colliding with the source socket.
        """
        self._loop = asyncio.get_running_loop()
        mavsdk_url = f"udpin://0.0.0.0:{ROUTER.mavsdk_port}"

        self._system = System(mavsdk_server_address=None)
        await self._system.connect(system_address=mavsdk_url)

        log.info("Waiting for vehicle heartbeat on %s ...", mavsdk_url)
        async for cs in self._system.core.connection_state():
            if cs.is_connected:
                break
        self.state.connected = True
        HUB.publish("connection", {"connected": True})

        await self._detect_airframe()
        self._start_pymavlink_thread()
        return self.state

    async def disconnect(self) -> None:
        self._stop_flag.set()
        if self._pymav_thread:
            self._pymav_thread.join(timeout=3)
            self._pymav_thread = None
        self._system = None
        self.state = VehicleState()
        HUB.publish("connection", {"connected": False})

    # ------------------------------------------------------------------ #
    # Airframe detection (SYS_AUTOSTART primary, MAV_TYPE cross-check)
    # ------------------------------------------------------------------ #
    async def _detect_airframe(self) -> None:
        try:
            value = await self._system.param.get_param_int("SYS_AUTOSTART")
            self.state.airframe = classify_airframe(value)
            log.info("Airframe: %s", self.state.airframe)
            self._publish_airframe()
        except Exception as exc:  # param timeout, unsupported FC, etc.
            log.warning("SYS_AUTOSTART fetch failed: %s", exc)
            HUB.publish("alert", {
                "severity": "warning",
                "text": "Could not read SYS_AUTOSTART — waiting for HEARTBEAT "
                        "MAV_TYPE fallback. Parameter writes stay disabled "
                        "until an airframe class is established.",
            })

    def _publish_airframe(self) -> None:
        af = self.state.airframe
        HUB.publish("airframe", {
            "sys_autostart": af.sys_autostart,
            "airframe_class": af.airframe_class,
            "label": af.label,
            "mav_type": af.mav_type,
            "source": af.source,
        })

    def _apply_mav_type(self, mav_type: int) -> None:
        """First vehicle HEARTBEAT seen (called on the event loop).

        Fallback when SYS_AUTOSTART failed; cross-check when it worked —
        a class mismatch between the two means the FC config is suspect,
        which the pilot should know before trusting any advice.
        """
        if self.state.airframe is None:
            fallback = classify_mav_type(mav_type)
            if fallback is not None:
                self.state.airframe = fallback
                log.info("Airframe (MAV_TYPE fallback): %s", fallback)
                self._publish_airframe()
            return

        self.state.airframe.mav_type = mav_type
        cross = classify_mav_type(mav_type)
        if cross and cross.airframe_class != self.state.airframe.airframe_class:
            HUB.publish("alert", {
                "severity": "warning",
                "source": "airframe",
                "text": (
                    f"Airframe mismatch: SYS_AUTOSTART says "
                    f"{self.state.airframe.airframe_class} but HEARTBEAT "
                    f"MAV_TYPE={mav_type} suggests {cross.airframe_class}. "
                    f"Verify the FC configuration before applying advice."
                ),
            })
        self._publish_airframe()

    # ------------------------------------------------------------------ #
    # Parameter access (control plane)
    # ------------------------------------------------------------------ #
    async def read_param(self, name: str) -> float:
        """Read a parameter, transparently handling float vs int storage."""
        if not self._system:
            raise ConnectionError("Vehicle not connected")
        try:
            return await self._system.param.get_param_float(name)
        except Exception:
            return float(await self._system.param.get_param_int(name))

    async def write_approved_param(self, name: str, value: float,
                                   as_int: bool = False) -> None:
        """
        The ONLY parameter-write path in the application.

        Callers must have already validated `value` through the safety
        registry; this method re-asserts connectivity but trusts the
        API layer to have done the safety pass (it does it twice).
        `as_int` comes from the registry's type annotation.
        """
        if not self._system:
            raise ConnectionError("Vehicle not connected")
        if as_int:
            await self._system.param.set_param_int(name, int(round(value)))
        else:
            await self._system.param.set_param_float(name, value)
        log.info("Parameter written: %s = %s", name, value)
        HUB.publish("param_written", {"param": name, "value": value})

    # ------------------------------------------------------------------ #
    # Raw firehose (data plane, dedicated thread)
    # ------------------------------------------------------------------ #
    def _start_pymavlink_thread(self) -> None:
        self._stop_flag.clear()
        self._mav_type_seen = False
        self._last_mode = None
        self._pymav_thread = threading.Thread(
            target=self._pymavlink_worker, name="pymavlink-rx", daemon=True
        )
        self._pymav_thread.start()

    def _pymavlink_worker(self) -> None:
        """Blocking receive loop. Publishes into the hub thread-safely."""
        try:
            conn = mavutil.mavlink_connection(
                f"udpin:127.0.0.1:{ROUTER.pymavlink_port}"
            )
        except OSError as exc:
            log.error("pymavlink bind failed: %s", exc)
            return

        while not self._stop_flag.is_set():
            msg = conn.recv_match(blocking=True, timeout=1.0)
            if msg is None:
                continue
            mtype = msg.get_type()

            # Vehicle heartbeats: one-shot MAV_TYPE capture + continuous
            # flight-mode tracking (ignore other GCS instances on the link).
            if mtype == "HEARTBEAT" and msg.type != _MAV_TYPE_GCS:
                if not self._mav_type_seen:
                    self._mav_type_seen = True
                    if self._loop and self._loop.is_running():
                        self._loop.call_soon_threadsafe(self._apply_mav_type, msg.type)
                armed = bool(msg.base_mode & 0x80)
                if (msg.custom_mode, armed) != self._last_mode:
                    self._last_mode = (msg.custom_mode, armed)
                    payload = {
                        "mode": decode_mode(msg.custom_mode),
                        "custom_mode": int(msg.custom_mode),
                        "armed": armed,
                    }
                    if self._loop and self._loop.is_running():
                        self._loop.call_soon_threadsafe(
                            HUB.publish, "flight_mode", payload)
                continue

            channel = _WATCHED_MESSAGES.get(mtype)
            if channel is None:
                continue
            payload = msg.to_dict()
            payload.pop("mavpackettype", None)
            # Cross the thread boundary into asyncio land.
            if self._loop and self._loop.is_running():
                self._loop.call_soon_threadsafe(HUB.publish, channel, payload)

        conn.close()


CONNECTION = ConnectionManager()
