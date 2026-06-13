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
from .airframe import (AirframeInfo, UnsupportedAirframeError,
                       classify_airframe, classify_mav_type)
from .flight_modes import decode_mode
from .router_manager import ROUTER
from .telemetry_hub import HUB

log = logging.getLogger("mint.connection")


class UnsupportedVehicleError(Exception):
    """Raised when the connected vehicle is not a supported PX4 target.

    Covers a non-PX4 autopilot, a PX4 firmware older than the supported
    floor, or an out-of-scope airframe (rover/boat/sub/balloon)."""

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

# Telemetry staleness watchdog.
_WATCHDOG_PERIOD_S = 1.0    # how often to re-check the firehose
_STALE_MAX_AGE_S = 2.0      # channel silent longer than this => stale

# Control-plane param read retry (transient MAVSDK timeouts in flight).
_PARAM_READ_ATTEMPTS = 3
_PARAM_RETRY_BASE_S = 0.25  # doubled each retry: 0.25s, 0.5s


@dataclass
class VehicleState:
    connected: bool = False
    airframe: Optional[AirframeInfo] = None
    system_id: Optional[int] = None
    fw_version: Optional[str] = None     # "1.14.0" once read from MAVSDK Info


class ConnectionManager:
    """Owns the MAVSDK system object and the pymavlink listener thread."""

    def __init__(self) -> None:
        self.state = VehicleState()
        self._system: Optional[System] = None
        self._pymav_thread: Optional[threading.Thread] = None
        self._stop_flag = threading.Event()
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._mav_type_seen = False
        self._autopilot_rejected = False
        self._last_mode: tuple[int, bool] | None = None
        self._watchdog: Optional[asyncio.Task] = None
        self._conn_monitor: Optional[asyncio.Task] = None

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

        # Gate on PX4 + firmware version BEFORE marking connected or starting
        # any analysis. A non-PX4 stack or pre-1.14 firmware is rejected here
        # so the engines never see telemetry they would misinterpret.
        try:
            await self._verify_supported_firmware()
            await self._detect_airframe()
        except (UnsupportedVehicleError, UnsupportedAirframeError) as exc:
            self._system = None
            self.state = VehicleState()
            HUB.publish("alert", {
                "severity": "critical", "source": "connection",
                "text": f"Unsupported vehicle — refusing to connect: {exc}",
            })
            raise UnsupportedVehicleError(str(exc)) from exc

        self.state.connected = True
        HUB.publish("connection", {"connected": True})

        self._start_pymavlink_thread()
        self._watchdog = asyncio.create_task(
            self._staleness_watchdog(), name="telemetry-watchdog")
        self._conn_monitor = asyncio.create_task(
            self._connection_monitor(), name="connection-monitor")
        return self.state

    async def _verify_supported_firmware(self) -> None:
        """Reject non-PX4 stacks and PX4 firmware older than the floor.

        PX4 reports its flight-stack version through the MAVSDK Info plugin.
        ArduPilot and other autopilots either expose a different vendor or
        no Info at all; both fail the gate. The vendor-agnostic PX4 check is
        the HEARTBEAT.autopilot field, captured separately on the firehose —
        this method handles the version floor.
        """
        # Info can lag the connection_state flip by a beat while the autopilot
        # finishes its handshake; retry briefly before treating absence as a
        # hard "not PX4" verdict.
        version = None
        last_exc: Exception | None = None
        for _ in range(5):
            try:
                version = await self._system.info.get_version()
                break
            except Exception as exc:  # plugin not ready / unsupported / timeout
                last_exc = exc
                await asyncio.sleep(0.5)
        if version is None:
            raise UnsupportedVehicleError(
                "Could not read a firmware version from the vehicle. MINT "
                "supports PX4 "
                f"v{config.MIN_PX4_VERSION[0]}.{config.MIN_PX4_VERSION[1]}+ "
                f"only (underlying error: {last_exc})."
            ) from last_exc

        major = int(version.flight_sw_major)
        minor = int(version.flight_sw_minor)
        patch = int(version.flight_sw_patch)
        self.state.fw_version = f"{major}.{minor}.{patch}"

        if (major, minor) < config.MIN_PX4_VERSION:
            floor = config.MIN_PX4_VERSION
            raise UnsupportedVehicleError(
                f"PX4 firmware v{self.state.fw_version} is older than the "
                f"supported minimum v{floor[0]}.{floor[1]}. Earlier releases "
                f"emit a different message set MINT cannot analyse reliably."
            )
        log.info("Firmware version: PX4 v%s", self.state.fw_version)

    async def disconnect(self) -> None:
        if not self.state.connected and self._system is None:
            return  # already torn down (e.g. monitor + manual disconnect race)
        self._stop_flag.set()
        if self._watchdog:
            self._watchdog.cancel()
            self._watchdog = None
        # Don't cancel the connection monitor if we're running inside it;
        # it will exit on its own once this coroutine returns.
        current = asyncio.current_task()
        if self._conn_monitor and self._conn_monitor is not current:
            self._conn_monitor.cancel()
        self._conn_monitor = None
        if self._pymav_thread:
            self._pymav_thread.join(timeout=3)
            self._pymav_thread = None
        self._system = None
        self.state = VehicleState()
        HUB.publish("connection", {"connected": False})

    # ------------------------------------------------------------------ #
    # Telemetry staleness watchdog
    # ------------------------------------------------------------------ #
    async def _staleness_watchdog(self) -> None:
        """Watch the core firehose channels and flag a data drought.

        If the router dies or PX4 stops streaming, the analysis engines just
        block on an empty queue and the UI keeps showing the last value. This
        loop turns that silent failure into an explicit "telemetry_stale"
        event (and a one-shot alert) so the pilot never trusts frozen metrics.
        """
        # Channels that should be flowing continuously on a healthy link.
        watched = ("attitude", "vfr_hud")
        was_stale = False
        try:
            while True:
                await asyncio.sleep(_WATCHDOG_PERIOD_S)
                # Only meaningful once data has started: require at least one
                # sample on a channel before judging it stale.
                stale = [c for c in watched
                         if HUB.last_seen(c) is not None
                         and HUB.is_stale(c, _STALE_MAX_AGE_S)]
                any_data = any(HUB.last_seen(c) is not None for c in watched)
                now_stale = any_data and bool(stale)
                if now_stale != was_stale:
                    was_stale = now_stale
                    HUB.publish("telemetry_stale", {
                        "stale": now_stale,
                        "channels": stale,
                    })
                    if now_stale:
                        HUB.publish("alert", {
                            "severity": "critical", "source": "telemetry",
                            "text": (
                                "Telemetry has gone silent — no "
                                f"{', '.join(stale)} for >{_STALE_MAX_AGE_S:.0f}s. "
                                "Displayed metrics are FROZEN, not live. Check the "
                                "link/router before trusting any reading."
                            ),
                        })
                    else:
                        HUB.publish("alert", {
                            "severity": "info", "source": "telemetry",
                            "text": "Telemetry resumed — metrics are live again.",
                        })
        except asyncio.CancelledError:
            pass

    # ------------------------------------------------------------------ #
    # Control-plane liveness monitor
    # ------------------------------------------------------------------ #
    async def _connection_monitor(self) -> None:
        """Watch the MAVSDK control plane and tear down on link loss.

        The staleness watchdog covers the data plane (firehose went quiet).
        This covers the control plane: if the vehicle reboots or the link
        drops, MAVSDK's connection_state() reports is_connected=False. Without
        this, the control plane keeps reporting "connected" and param reads/
        writes would queue against a dead vehicle. On a drop we alert and run
        the normal teardown so the UI flips to disconnected.
        """
        try:
            async for cs in self._system.core.connection_state():
                if not cs.is_connected:
                    log.warning("MAVSDK reports control link lost")
                    HUB.publish("alert", {
                        "severity": "critical", "source": "connection",
                        "text": ("Lost the control link to the vehicle (reboot or "
                                 "radio dropout). Disconnecting — reconnect once the "
                                 "vehicle is back."),
                    })
                    await self.disconnect()
                    return
        except asyncio.CancelledError:
            pass
        except Exception:
            log.exception("Connection monitor error")

    # ------------------------------------------------------------------ #
    # Airframe detection (SYS_AUTOSTART primary, MAV_TYPE cross-check)
    # ------------------------------------------------------------------ #
    async def _detect_airframe(self) -> None:
        try:
            value = await self._system.param.get_param_int("SYS_AUTOSTART")
        except Exception as exc:  # param timeout, unsupported FC, etc.
            log.warning("SYS_AUTOSTART fetch failed: %s", exc)
            HUB.publish("alert", {
                "severity": "warning",
                "text": "Could not read SYS_AUTOSTART — waiting for HEARTBEAT "
                        "MAV_TYPE fallback. Parameter writes stay disabled "
                        "until an airframe class is established.",
            })
            return
        # A successful read that resolves to an out-of-scope airframe must
        # abort the connection (propagates UnsupportedAirframeError).
        self.state.airframe = classify_airframe(value)
        log.info("Airframe: %s", self.state.airframe)
        self._publish_airframe()

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
            try:
                fallback = classify_mav_type(mav_type)
            except UnsupportedAirframeError as exc:
                self._reject_running_vehicle(str(exc))
                return
            if fallback is not None:
                self.state.airframe = fallback
                log.info("Airframe (MAV_TYPE fallback): %s", fallback)
                self._publish_airframe()
            return

        self.state.airframe.mav_type = mav_type
        try:
            cross = classify_mav_type(mav_type)
        except UnsupportedAirframeError as exc:
            # SYS_AUTOSTART said supported but the HEARTBEAT says rover/boat/etc.
            # Trust the deny-list and refuse rather than analyse a hybrid.
            self._reject_running_vehicle(str(exc))
            return
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

    def _reject_running_vehicle(self, reason: str) -> None:
        """Tear down an already-running connection on the event loop.

        Used when an out-of-scope airframe or non-PX4 autopilot is only
        discovered from the HEARTBEAT firehose (after MAVSDK already handed
        us a connection). Publishes a critical alert, then disconnects.
        """
        log.error("Rejecting connected vehicle: %s", reason)
        HUB.publish("alert", {
            "severity": "critical", "source": "connection",
            "text": f"Unsupported vehicle — disconnecting: {reason}",
        })
        if self._loop and self._loop.is_running():
            asyncio.create_task(self.disconnect())

    # ------------------------------------------------------------------ #
    # Parameter access (control plane)
    # ------------------------------------------------------------------ #
    async def read_param(self, name: str) -> float:
        """Read a parameter, transparently handling float vs int storage.

        MAVSDK param reads can transiently time out when the autopilot is
        busy (flight-mode change, high CPU) — a single failure must not sink
        a whole proposal, so the read is retried with exponential backoff.
        We deliberately do NOT cache: the safety model re-reads the live
        on-vehicle value at both staging and approval, so a stale cached
        value could let an approval validate against a number the pilot has
        since changed in QGC.
        """
        if not self._system:
            raise ConnectionError("Vehicle not connected")

        delay = _PARAM_RETRY_BASE_S
        last_exc: Exception | None = None
        for attempt in range(_PARAM_READ_ATTEMPTS):
            try:
                return await self._read_param_once(name)
            except Exception as exc:  # timeout / busy
                last_exc = exc
                if attempt < _PARAM_READ_ATTEMPTS - 1:
                    log.warning("Param read %s failed (attempt %d/%d): %s — retrying",
                                name, attempt + 1, _PARAM_READ_ATTEMPTS, exc)
                    await asyncio.sleep(delay)
                    delay *= 2
        raise TimeoutError(
            f"Could not read {name} after {_PARAM_READ_ATTEMPTS} attempts: {last_exc}"
        )

    async def _read_param_once(self, name: str) -> float:
        """One param read, transparently handling float vs int storage.

        A float read that fails specifically because the value is stored as
        an int is retried as an int; that type-probe is distinct from the
        transient-timeout retry in read_param.
        """
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
        self._autopilot_rejected = False
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

            # Vehicle heartbeats: one-shot autopilot/MAV_TYPE capture +
            # continuous flight-mode tracking (ignore other GCS on the link).
            if mtype == "HEARTBEAT" and msg.type != _MAV_TYPE_GCS:
                # PX4-only gate: a non-PX4 autopilot on the link is refused.
                # The version floor is checked separately via MAVSDK Info.
                if msg.autopilot != config.MAV_AUTOPILOT_PX4:
                    if not self._autopilot_rejected:
                        self._autopilot_rejected = True
                        if self._loop and self._loop.is_running():
                            self._loop.call_soon_threadsafe(
                                self._reject_running_vehicle,
                                f"autopilot type {msg.autopilot} is not PX4 "
                                f"(MAV_AUTOPILOT_PX4={config.MAV_AUTOPILOT_PX4}). "
                                f"MINT supports PX4 only.",
                            )
                    continue
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
