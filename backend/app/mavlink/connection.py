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
from dataclasses import dataclass, field
from typing import Optional
import struct
import time

import os
os.environ["MAVLINK20"] = "1"
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
    "ACTUATOR_OUTPUT_STATUS": "actuator_output_status", # dynamic actuation monitor
    "SERVO_OUTPUT_RAW": "servo_output_raw",             # fallback for SIM_GZ servo channels
"VIBRATION": "vibration",             # live vibration fields
    "EXTENDED_SYS_STATE": "extended_sys_state",  # VTOL transition state
    "LOCAL_POSITION_NED": "local_position",       # velocity/position loops
    "POSITION_TARGET_LOCAL_NED": "position_target",  # velocity/position setpoints
    "LOCAL_POSITION_NED_COV": "local_position_setpoint", # velocity/position setpoints in POSCTL
}

_MAV_TYPE_GCS = 6  # ignore heartbeats from other ground stations on the link

# Telemetry staleness watchdog.
_WATCHDOG_PERIOD_S = config.CONN_WATCHDOG_PERIOD_S    # how often to re-check the firehose
_STALE_MAX_AGE_S = config.CONN_STALE_MAX_AGE_S      # channel silent longer than this => stale

# Control-plane param read retry (transient MAVSDK timeouts in flight).
_PARAM_READ_ATTEMPTS = config.CONN_PARAM_READ_ATTEMPTS
_PARAM_RETRY_BASE_S = config.CONN_PARAM_RETRY_BASE_S  # doubled each retry: 0.25s, 0.5s

CS_TYPE_TO_NAME = {
    0: "Custom Servo",
    1: "Left Aileron",
    2: "Right Aileron",
    3: "Elevator",
    4: "Left Elevator",
    5: "Right Elevator",
    6: "Rudder",
    7: "Left Rudder",
    8: "Right Rudder",
    9: "Flap",
    10: "Left Flap",
    11: "Right Flap",
    12: "Airbrake",
    13: "Left Airbrake",
    14: "Right Airbrake",
    15: "V-Tail Left",
    16: "V-Tail Right",
}


@dataclass
class VehicleState:
    connected: bool = False
    airframe: Optional[AirframeInfo] = None
    system_id: Optional[int] = None
    fw_version: Optional[str] = None     # "1.14.0" once read from MAVSDK Info
    discovery_failed: bool = False
    actuator_family: str | None = None   # e.g. "SIM_GZ", "PWM", "ACT_FUNC"
    actuator_map: dict[str, list[int]] = field(default_factory=lambda: {
        "hover_motors": [],
        "thrust_motors": [],
        "control_surfaces": [],
        "tilt_servos": []
    })
    actuator_limits: dict[int, dict[str, float]] = field(default_factory=dict)
    actuator_names: dict[int, str] = field(default_factory=dict)
def _safe_set_result(fut: asyncio.Future, val) -> None:
    if not fut.done():
        fut.set_result(val)


def _safe_set_exception(fut: asyncio.Future, exc: Exception) -> None:
    if not fut.done():
        fut.set_exception(exc)


def _extract_param_value(msg) -> float:
    val = msg.param_value
    ptype = getattr(msg, "param_type", None)
    if ptype is not None and 1 <= ptype <= 8:
        try:
            packed = struct.pack('f', val)
            if ptype in (1, 3, 5):
                return float(struct.unpack('I', packed)[0])
            else:
                return float(struct.unpack('i', packed)[0])
        except Exception:
            pass
    return float(val)


class ConnectionManager:
    """Owns the pymavlink system connection and the receive thread."""

    def __init__(self) -> None:
        self.state = VehicleState()
        self._pymav_conn: Optional[mavutil.mavfile] = None
        self._pymav_thread: Optional[threading.Thread] = None
        self._stop_flag = threading.Event()
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._mav_type_seen = False
        self._autopilot_rejected = False
        self._last_mode: tuple[int, bool] | None = None
        self._watchdog: Optional[asyncio.Task] = None
        self._conn_monitor: Optional[asyncio.Task] = None
        self._heartbeat_task: Optional[asyncio.Task] = None
        self._param_cache: dict[str, float] | None = None

        self._target_system: Optional[int] = None
        self._target_component: Optional[int] = None
        self._connected_future: Optional[asyncio.Future[tuple[int, int]]] = None
        self._pending_version_read: Optional[asyncio.Future[object]] = None
        self._pending_param_reads: dict[str, asyncio.Future[float]] = {}
        self._pending_param_writes: dict[str, asyncio.Future[float]] = {}
        self._last_heartbeat_time: float = 0.0

        # Bulk download variables
        self._bulk_download_active: bool = False
        self._bulk_params: dict[str, float] = {}
        self._bulk_param_count: int = -1
        self._bulk_event: Optional[asyncio.Event] = None

    # ------------------------------------------------------------------ #
    # Lifecycle
    # ------------------------------------------------------------------ #
    async def connect(self) -> VehicleState:
        """Connect to the routed UDP endpoint using pymavlink and start the receive loop.

        Ports come from the RouterManager, not static config: a udp_listen
        source (e.g. local SITL) may have shifted the fan-out to alternate
        ports to avoid colliding with the source socket.
        """
        self._loop = asyncio.get_running_loop()
        pymav_url = f"udpin:127.0.0.1:{ROUTER.pymavlink_port}"

        # Initialize futures and collections
        self._connected_future = self._loop.create_future()
        self._pending_version_read = None
        self._pending_param_reads = {}
        self._pending_param_writes = {}
        self._bulk_download_active = False
        self._bulk_params = {}
        self._bulk_param_count = -1
        self._bulk_event = None

        log.info("Opening pymavlink connection on %s ...", pymav_url)
        try:
            self._pymav_conn = mavutil.mavlink_connection(pymav_url, source_system=251)
            import socket
            try:
                self._pymav_conn.port.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)
                log.info("Set pymavlink UDP socket receive buffer to 1MB")
            except Exception as e:
                log.warning("Failed to set pymavlink UDP socket RCVBUF: %s", e)
        except Exception as exc:
            log.error("pymavlink connection open failed: %s", exc)
            self._pymav_conn = None
            raise ConnectionError(f"Failed to open pymavlink connection: {exc}")

        # Start background receive loop early to handle connection/HEARTBEAT
        self._start_pymavlink_thread()

        # Start GCS heartbeat sender task to register routing in the vehicle
        self._heartbeat_task = asyncio.create_task(
            self._heartbeat_sender(), name="heartbeat-sender"
        )

        log.info("Waiting for vehicle heartbeat on %s ...", pymav_url)
        try:
            self._target_system, self._target_component = await asyncio.wait_for(
                self._connected_future, timeout=10.0
            )
            self._last_heartbeat_time = time.monotonic()
        except asyncio.TimeoutError as exc:
            await self.disconnect()
            raise TimeoutError("Timeout waiting for vehicle heartbeat") from exc
        except UnsupportedVehicleError as exc:
            await self.disconnect()
            raise exc
        except Exception as exc:
            await self.disconnect()
            raise exc

        try:
            await self._verify_supported_firmware()
            await self._detect_airframe()
            await self._discover_actuators()
        except (UnsupportedVehicleError, UnsupportedAirframeError) as exc:
            await self.disconnect()
            HUB.publish("alert", {
                "severity": "critical", "source": "connection",
                "text": f"Unsupported vehicle — refusing to connect: {exc}",
            })
            raise UnsupportedVehicleError(str(exc)) from exc
        except Exception as exc:
            log.exception("Error during connection setup")
            await self.disconnect()
            raise exc

        self.state.connected = True
        HUB.publish("connection", {"connected": True})

        self._watchdog = asyncio.create_task(
            self._staleness_watchdog(), name="telemetry-watchdog")
        self._conn_monitor = asyncio.create_task(
            self._connection_monitor(), name="connection-monitor")
        return self.state

    async def _verify_supported_firmware(self) -> None:
        """Reject non-PX4 stacks and PX4 firmware older than the floor."""
        version_msg = None
        last_exc: Exception | None = None
        for attempt in range(5):
            try:
                self._pending_version_read = self._loop.create_future()
                self._pymav_conn.mav.autopilot_version_request_send(
                    self._target_system,
                    self._target_component
                )
                version_msg = await asyncio.wait_for(self._pending_version_read, timeout=2.0)
                break
            except Exception as exc:
                last_exc = exc
                await asyncio.sleep(0.5)
            finally:
                self._pending_version_read = None

        if version_msg is None:
            raise UnsupportedVehicleError(
                "Could not read a firmware version from the vehicle. MINT "
                "supports PX4 "
                f"v{config.MIN_PX4_VERSION[0]}.{config.MIN_PX4_VERSION[1]}+ "
                f"only (underlying error: {last_exc})."
            )

        ver = version_msg.flight_sw_version
        major = (ver >> 24) & 0xFF
        minor = (ver >> 16) & 0xFF
        patch = (ver >> 8) & 0xFF
        self.state.fw_version = f"{major}.{minor}.{patch}"

        if (major, minor) < config.MIN_PX4_VERSION:
            floor = config.MIN_PX4_VERSION
            raise UnsupportedVehicleError(
                f"PX4 firmware v{self.state.fw_version} is older than the "
                f"supported minimum v{floor[0]}.{floor[1]}. Earlier releases "
                f"emit a different message set MINT cannot analyse reliably."
            )
        log.info("Firmware version: PX4 v%s", self.state.fw_version)

    async def _heartbeat_sender(self) -> None:
        """Periodically send a GCS heartbeat to the vehicle to keep the routing active."""
        try:
            while not self._stop_flag.is_set():
                if self._pymav_conn:
                    try:
                        self._pymav_conn.mav.heartbeat_send(
                            6,  # MAV_TYPE_GCS
                            0,  # MAV_AUTOPILOT_INVALID
                            0,  # base_mode
                            0,  # custom_mode
                            0   # system_status
                        )
                    except Exception as e:
                        log.debug("Failed to send GCS heartbeat: %s", e)
                await asyncio.sleep(1.0)
        except asyncio.CancelledError:
            pass

    async def disconnect(self) -> None:
        if not self.state.connected and self._pymav_conn is None:
            return  # already torn down (e.g. monitor + manual disconnect race)
        self._stop_flag.set()
        if self._watchdog:
            self._watchdog.cancel()
            self._watchdog = None
        if self._heartbeat_task:
            self._heartbeat_task.cancel()
            self._heartbeat_task = None
        # Don't cancel the connection monitor if we're running inside it;
        # it will exit on its own once this coroutine returns.
        current = asyncio.current_task()
        if self._conn_monitor and self._conn_monitor is not current:
            self._conn_monitor.cancel()
        self._conn_monitor = None
        if self._pymav_conn:
            try:
                self._pymav_conn.close()
            except Exception:
                pass
            self._pymav_conn = None
        if self._pymav_thread:
            self._pymav_thread.join(timeout=3)
            self._pymav_thread = None
        self.state = VehicleState()

        # Clear proposals and cached telemetry
        from ..advisors.param_advisor import ADVISOR
        ADVISOR.clear()
        HUB.clear_latest()

        HUB.publish("connection", {"connected": False})
        HUB.publish("proposal", {"clear": True})

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
                seen_channels = [c for c in watched if HUB.last_seen(c) is not None]
                stale = [c for c in seen_channels if HUB.is_stale(c, _STALE_MAX_AGE_S)]
                now_stale = len(seen_channels) > 0 and len(stale) == len(seen_channels)
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
        """Watch the pymavlink heartbeat liveness and tear down on link loss.

        The staleness watchdog covers the data plane (firehose went quiet).
        This covers the control plane: if the vehicle reboots or the link
        drops (no heartbeat from target for 5s), we alert and run the normal
        teardown so the UI flips to disconnected.
        """
        try:
            while True:
                await asyncio.sleep(1.0)
                if time.monotonic() - self._last_heartbeat_time > 5.0:
                    log.warning("pymavlink reports control link lost (no heartbeat for 5s)")
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
            value = int(await self.read_param("SYS_AUTOSTART"))
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
            "sys_autostart": af.sys_autostart if af else None,
            "airframe_class": af.airframe_class if af else None,
            "label": af.label if af else None,
            "mav_type": af.mav_type if af else None,
            "source": af.source if af else None,
            "discovery_failed": self.state.discovery_failed,
            "actuator_map": self.state.actuator_map,
        })

    async def _discover_actuators(self) -> None:
        """Query parameters at connection to build the dynamic actuator lookup table."""
        log.info("Starting dynamic actuator discovery...")
        self.state.actuator_map = {
            "hover_motors": [],
            "thrust_motors": [],
            "control_surfaces": [],
            "tilt_servos": []
        }
        self.state.actuator_limits = {}
        self.state.actuator_names = {}
        self.state.discovery_failed = False
        num_esc = 0

        # Check if we are connected over serial
        is_serial = False
        try:
            if ROUTER.is_running and ROUTER.status().mode == "serial":
                is_serial = True
        except Exception:
            pass

        # For parameter existence scans, one attempt is sufficient
        scan_attempts = 1
        scan_timeout = 1.0 if is_serial else 0.5

        # On serial links, bulk-download all parameters before probing.
        if is_serial:
            log.info("Serial link: pre-downloading parameter cache for discovery")
            cache = await self._bulk_download_params()
            self._param_cache = cache if cache else None

        # 1. Identify active parameter family
        selected_family = None
        
        # Determine properties of the connected vehicle to narrow down potential families
        is_hil = False
        is_sim = False
        
        if self.state.airframe:
            # Check if autostart ID is in _SIM_IDS
            from .airframe import _SIM_IDS
            if self.state.airframe.sys_autostart in _SIM_IDS:
                is_sim = True
                # Check if it's HIL simulation
                label = self.state.airframe.label.lower()
                if "hil" in label or "hil sim" in label:
                    is_hil = True
        
        # Cross-check SYS_HITL parameter to be sure
        if not is_hil:
            try:
                # Read SYS_HITL parameter to see if HITL is enabled on the autopilot.
                hitl_val = await self.read_param("SYS_HITL", attempts=scan_attempts, timeout=scan_timeout)
                if int(hitl_val) == 1:
                    is_hil = True
            except Exception:
                pass

        log.info("Actuator discovery - vehicle properties: is_sim=%s, is_hil=%s", is_sim, is_hil)

        if is_hil:
            selected_family = "HIL"
        else:
            # Check SYS_CTRL_ALLOC to determine if control allocation (ACT_FUNC) is enabled.
            # 1 = ACT_FUNC, 0 = PWM (or SIM_GZ for simulator if present)
            ctrl_alloc_enabled = False
            has_ctrl_alloc_param = False
            try:
                ctrl_alloc = await self.read_param("SYS_CTRL_ALLOC", attempts=scan_attempts, timeout=scan_timeout)
                has_ctrl_alloc_param = True
                if int(ctrl_alloc) == 1:
                    ctrl_alloc_enabled = True
            except Exception:
                # If SYS_CTRL_ALLOC doesn't exist, assume legacy PWM (or SITL simulator)
                pass

            if has_ctrl_alloc_param:
                if ctrl_alloc_enabled:
                    selected_family = "ACT_FUNC"
                else:
                    if is_sim:
                        try:
                            # Probe SIM_GZ only if it's a simulator connection
                            await self.read_param("SIM_GZ_EC_FUNC1", attempts=scan_attempts, timeout=scan_timeout)
                            selected_family = "SIM_GZ"
                        except Exception:
                            selected_family = "PWM"
                    else:
                        selected_family = "PWM"
            else:
                # Fallback: probe parameter existence sequentially if SYS_CTRL_ALLOC doesn't exist
                # This ensures compatibility with custom/older firmware or tests.
                try:
                    await self.read_param("ACT_FUNC1", attempts=scan_attempts, timeout=scan_timeout)
                    selected_family = "ACT_FUNC"
                except Exception:
                    if is_sim:
                        try:
                            await self.read_param("SIM_GZ_EC_FUNC1", attempts=scan_attempts, timeout=scan_timeout)
                            selected_family = "SIM_GZ"
                        except Exception:
                            selected_family = "PWM"
                    else:
                        selected_family = "PWM"

        log.info("Discovered parameter family: %s", selected_family)
        if not selected_family:
            self.state.discovery_failed = True
            log.warning("Actuator auto-discovery failed: no parameter family detected.")
            self._param_cache = None
            self._publish_airframe()
            return

        self.state.actuator_family = selected_family
        airframe_class = self.state.airframe.airframe_class if self.state.airframe else "MULTIROTOR"

        async def map_func(func_val: int, channel_idx: int):
            if 101 <= func_val <= 112:
                # Motor
                if airframe_class == "MULTIROTOR":
                    self.state.actuator_map["hover_motors"].append(channel_idx)
                elif airframe_class in ("FIXED_WING", "DELTA_WING"):
                    self.state.actuator_map["thrust_motors"].append(channel_idx)
                elif airframe_class == "VTOL":
                    if 101 <= func_val <= 104:
                        self.state.actuator_map["hover_motors"].append(channel_idx)
                    else:
                        self.state.actuator_map["thrust_motors"].append(channel_idx)
            elif 201 <= func_val <= 208:
                # Servo / Surface
                self.state.actuator_map["control_surfaces"].append(channel_idx)
                servo_idx = func_val - 201
                try:
                    cs_type = int(await self.read_param(f"CA_SV_CS{servo_idx}_TYPE", attempts=scan_attempts, timeout=scan_timeout))
                    name = CS_TYPE_TO_NAME.get(cs_type, f"Surface {servo_idx + 1}")
                    self.state.actuator_names[channel_idx] = name
                except Exception:
                    self.state.actuator_names[channel_idx] = f"Surface {servo_idx + 1}"
            elif 301 <= func_val <= 308:
                # Tilt Servo
                self.state.actuator_map["tilt_servos"].append(channel_idx)

        # 2. Query and map parameters of selected family
        present_channels = set()
        try:
            if selected_family == "ACT_FUNC":
                for i in range(1, 17):
                    try:
                        val = int(await self.read_param(f"ACT_FUNC{i}", attempts=scan_attempts, timeout=scan_timeout))
                        if val > 0:
                            present_channels.add(i - 1)
                            await map_func(val, i - 1)
                    except Exception:
                        pass
            elif selected_family == "SIM_GZ":
                # First detect number of active ESC slots (up to 8)
                num_esc = 0
                esc_values = {}
                for i in range(1, 9):
                    try:
                        val = int(await self.read_param(f"SIM_GZ_EC_FUNC{i}", attempts=scan_attempts, timeout=scan_timeout))
                        esc_values[i] = val
                        if val > 0:
                            present_channels.add(i - 1)
                            num_esc = max(num_esc, i)
                    except Exception:
                        pass

                # Map EC functions to physical channels 1 to num_esc
                for i in range(1, num_esc + 1):
                    val = esc_values.get(i, 0)
                    if val > 0:
                        await map_func(val, i - 1)

                # Map SV functions at fixed offset 16 (instance 1 in ACTUATOR_OUTPUT_STATUS).
                for j in range(1, 9):
                    try:
                        val = int(await self.read_param(f"SIM_GZ_SV_FUNC{j}", attempts=scan_attempts, timeout=scan_timeout))
                        if val > 0:
                            present_channels.add(16 + j - 1)
                            await map_func(val, 16 + j - 1)
                    except Exception:
                        pass
            elif selected_family == "HIL":
                for i in range(1, 17):
                    try:
                        val = int(await self.read_param(f"HIL_ACT_FUNC{i}", attempts=scan_attempts, timeout=scan_timeout))
                        if val > 0:
                            present_channels.add(i - 1)
                            await map_func(val, i - 1)
                    except Exception:
                        pass
            elif selected_family == "PWM":
                for i in range(1, 9):
                    try:
                        val = int(await self.read_param(f"PWM_MAIN_FUNC{i}", attempts=scan_attempts, timeout=scan_timeout))
                        if val > 0:
                            present_channels.add(i - 1)
                            await map_func(val, i - 1)
                    except Exception:
                        pass
                
                # Check if AUX channels exist by probing PWM_AUX_FUNC1 first
                has_aux = False
                try:
                    val = int(await self.read_param("PWM_AUX_FUNC1", attempts=scan_attempts, timeout=scan_timeout))
                    has_aux = True
                    if val > 0:
                        present_channels.add(8)
                        await map_func(val, 8)
                except Exception:
                    pass

                if has_aux:
                    for j in range(2, 9):
                        try:
                            val = int(await self.read_param(f"PWM_AUX_FUNC{j}", attempts=scan_attempts, timeout=scan_timeout))
                            if val > 0:
                                present_channels.add(8 + j - 1)
                                await map_func(val, 8 + j - 1)
                        except Exception:
                            pass

                # Check if AUX / IO board is active
                try:
                    if int(await self.read_param("SYS_USE_IO", attempts=scan_attempts, timeout=scan_timeout)) == 1:
                        self.state.actuator_family = "PWM_IOMCU"
                except Exception:
                    pass
        except Exception as exc:
            log.exception("Error scanning parameters inside family %s", selected_family)

        # 3. Check if we mapped anything
        has_mapped = any(len(v) > 0 for v in self.state.actuator_map.values())
        if not has_mapped:
            self.state.discovery_failed = True
            log.warning("Actuator discovery completed but mapped actuator set was empty.")
        else:
            self.state.discovery_failed = False
            active_map = {k: v for k, v in self.state.actuator_map.items() if v}
            log.info("Discovered actuator map: %s", active_map)

        # 4. Discover limits for present/mapped channels
        classified = present_channels
        if not classified:
            mapped = set()
            for v in self.state.actuator_map.values():
                mapped.update(v)
            classified = mapped if mapped else set(range(8))

        async def fetch_and_store_limits(ch_idx: int):
            min_val, max_val, trim_val = None, None, None

            def normalize_to_pwm(val: float, role: str) -> float:
                if selected_family == "SIM_GZ":
                    return float(val)
                if 800 <= val <= 2200:
                    return val
                if -1.0 <= val <= 1.0:
                    return 1500.0 + val * 500.0
                if role == "min":
                    return 1000.0
                elif role == "max":
                    return 2000.0
                else:
                    return 1500.0

            if selected_family == "ACT_FUNC":
                try:
                    min_val = await self.read_param(f"OUT{ch_idx + 1}_MIN", attempts=scan_attempts, timeout=scan_timeout)
                except Exception:
                    pass
                try:
                    max_val = await self.read_param(f"OUT{ch_idx + 1}_MAX", attempts=scan_attempts, timeout=scan_timeout)
                except Exception:
                    pass
                try:
                    trim_val = await self.read_param(f"OUT{ch_idx + 1}_TRIM", attempts=scan_attempts, timeout=scan_timeout)
                except Exception:
                    pass
            elif selected_family == "SIM_GZ":
                if ch_idx < 16:
                    esc_idx = ch_idx + 1
                    try:
                        min_val = await self.read_param(f"SIM_GZ_EC_MIN{esc_idx}", attempts=scan_attempts, timeout=scan_timeout)
                    except Exception:
                        pass
                    try:
                        max_val = await self.read_param(f"SIM_GZ_EC_MAX{esc_idx}", attempts=scan_attempts, timeout=scan_timeout)
                    except Exception:
                        pass
                    try:
                        trim_val = await self.read_param(f"SIM_GZ_EC_DIS{esc_idx}", attempts=scan_attempts, timeout=scan_timeout)
                    except Exception:
                        pass
                else:
                    sv_idx = ch_idx - 16 + 1
                    try:
                        min_val = await self.read_param(f"SIM_GZ_SV_MIN{sv_idx}", attempts=scan_attempts, timeout=scan_timeout)
                    except Exception:
                        pass
                    try:
                        max_val = await self.read_param(f"SIM_GZ_SV_MAX{sv_idx}", attempts=scan_attempts, timeout=scan_timeout)
                    except Exception:
                        pass
                    try:
                        trim_val = await self.read_param(f"SIM_GZ_SV_DIS{sv_idx}", attempts=scan_attempts, timeout=scan_timeout)
                    except Exception:
                        pass
            elif selected_family in ("PWM", "PWM_IOMCU"):
                try:
                    min_val = await self.read_param(f"OUT{ch_idx + 1}_MIN", attempts=scan_attempts, timeout=scan_timeout)
                except Exception:
                    pass
                try:
                    max_val = await self.read_param(f"OUT{ch_idx + 1}_MAX", attempts=scan_attempts, timeout=scan_timeout)
                except Exception:
                    pass
                try:
                    trim_val = await self.read_param(f"OUT{ch_idx + 1}_TRIM", attempts=scan_attempts, timeout=scan_timeout)
                except Exception:
                    pass

                # Legacy fallback
                if min_val is None:
                    if ch_idx < 8:
                        idx = ch_idx + 1
                        try:
                            min_val = await self.read_param(f"PWM_MAIN_MIN{idx}", attempts=scan_attempts, timeout=scan_timeout)
                        except Exception:
                            pass
                        try:
                            max_val = await self.read_param(f"PWM_MAIN_MAX{idx}", attempts=scan_attempts, timeout=scan_timeout)
                        except Exception:
                            pass
                        try:
                            trim_val = await self.read_param(f"PWM_MAIN_TRIM{idx}", attempts=scan_attempts, timeout=scan_timeout)
                        except Exception:
                            pass
                    else:
                        idx = ch_idx - 7
                        try:
                            min_val = await self.read_param(f"PWM_AUX_MIN{idx}", attempts=scan_attempts, timeout=scan_timeout)
                        except Exception:
                            pass
                        try:
                            max_val = await self.read_param(f"PWM_AUX_MAX{idx}", attempts=scan_attempts, timeout=scan_timeout)
                        except Exception:
                            pass
                        try:
                            trim_val = await self.read_param(f"PWM_AUX_TRIM{idx}", attempts=scan_attempts, timeout=scan_timeout)
                        except Exception:
                            pass
            elif selected_family == "HIL":
                try:
                    min_val = await self.read_param(f"OUT{ch_idx + 1}_MIN", attempts=scan_attempts, timeout=scan_timeout)
                except Exception:
                    pass
                try:
                    max_val = await self.read_param(f"OUT{ch_idx + 1}_MAX", attempts=scan_attempts, timeout=scan_timeout)
                except Exception:
                    pass
                try:
                    trim_val = await self.read_param(f"OUT{ch_idx + 1}_TRIM", attempts=scan_attempts, timeout=scan_timeout)
                except Exception:
                    pass
                if min_val is None and ch_idx < 8:
                    idx = ch_idx + 1
                    try:
                        min_val = await self.read_param(f"PWM_MAIN_MIN{idx}", attempts=scan_attempts, timeout=scan_timeout)
                    except Exception:
                        pass
                    try:
                        max_val = await self.read_param(f"PWM_MAIN_MAX{idx}", attempts=scan_attempts, timeout=scan_timeout)
                    except Exception:
                        pass
                    try:
                        trim_val = await self.read_param(f"PWM_MAIN_TRIM{idx}", attempts=scan_attempts, timeout=scan_timeout)
                    except Exception:
                        pass

            min_pwm = normalize_to_pwm(min_val if min_val is not None else config.DOMAINS_PWM_MIN, "min")
            max_pwm = normalize_to_pwm(max_val if max_val is not None else (config.DOMAINS_PWM_MIN + config.DOMAINS_PWM_RANGE * 2), "max")
            trim_pwm = normalize_to_pwm(trim_val if trim_val is not None else config.DOMAINS_PWM_MID, "trim")
            pwm_range = max(1.0, (max_pwm - min_pwm) / 2.0)

            self.state.actuator_limits[ch_idx] = {
                "min": min_pwm,
                "max": max_pwm,
                "trim": trim_pwm,
                "range": pwm_range
            }
            log.info("Ch %d limits: min=%s, max=%s, trim=%s, range=%s", ch_idx + 1, min_pwm, max_pwm, trim_pwm, pwm_range)

        if classified:
            for ch in sorted(classified):
                await fetch_and_store_limits(ch)

        self._param_cache = None  # release discovery cache; future reads must be live
        self._publish_airframe()

    async def _bulk_download_params(self) -> dict[str, float]:
        """Download all vehicle parameters in one PARAM_REQUEST_LIST exchange.

        Returns a name→value dict on success, or an empty dict if the download
        fails (caller falls back to individual reads).
        """
        if not self._pymav_conn:
            return {}
        try:
            log.info("Bulk parameter download starting (this may take a few seconds on serial)...")
            self._bulk_params = {}
            self._bulk_param_count = -1
            self._bulk_event = asyncio.Event()
            self._bulk_download_active = True

            # Send PARAM_REQUEST_LIST
            self._pymav_conn.mav.param_request_list_send(
                self._target_system,
                self._target_component
            )

            # Wait for completion (up to 120s on slow serial links)
            await asyncio.wait_for(self._bulk_event.wait(), timeout=120.0)

            log.info("Bulk parameter download complete: %d parameters cached", len(self._bulk_params))
            return self._bulk_params
        except asyncio.TimeoutError:
            log.warning("Bulk parameter download timed out; falling back to individual reads")
            return {}
        except Exception as exc:
            log.warning("Bulk parameter download failed (%s); falling back to individual reads", exc)
            return {}
        finally:
            self._bulk_download_active = False
            self._bulk_event = None

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
    async def read_param(self, name: str, attempts: int | None = None, timeout: float | None = None) -> float:
        """Read a parameter, transparently handling float vs int storage.

        Param reads can transiently time out when the autopilot is busy —
        a single failure must not sink a whole proposal, so the read is
        retried with exponential backoff.
        We check the cache first if populated.
        """
        if not self._pymav_conn:
            raise ConnectionError("Vehicle not connected")

        name_upper = name.upper()
        if self._param_cache is not None:
            if name_upper in self._param_cache:
                return self._param_cache[name_upper]
            else:
                raise KeyError(name)

        attempts = attempts or _PARAM_READ_ATTEMPTS
        if timeout is None:
            is_serial = False
            try:
                if ROUTER.is_running and ROUTER.status().mode == "serial":
                    is_serial = True
            except Exception:
                pass
            timeout = 2.0 if is_serial else 1.0

        delay = _PARAM_RETRY_BASE_S
        last_exc: Exception | None = None

        for attempt in range(attempts):
            fut = self._loop.create_future()
            self._pending_param_reads[name_upper] = fut
            try:
                param_id = name_upper.encode('utf-8')
                if len(param_id) > 16:
                    param_id = param_id[:16]

                self._pymav_conn.mav.param_request_read_send(
                    self._target_system,
                    self._target_component,
                    param_id,
                    -1  # read by name
                )
                
                return await asyncio.wait_for(fut, timeout=timeout)
            except (asyncio.TimeoutError, Exception) as exc:
                last_exc = exc
                if attempt < attempts - 1:
                    log.warning("Param read %s failed (attempt %d/%d): %s — retrying",
                                name, attempt + 1, attempts, exc)
                    await asyncio.sleep(delay)
                    delay *= 2
            finally:
                self._pending_param_reads.pop(name_upper, None)

        raise TimeoutError(
            f"Could not read {name} after {attempts} attempts: {last_exc}"
        )

    async def write_approved_param(self, name: str, value: float,
                                   as_int: bool = False) -> None:
        """
        The ONLY parameter-write path in the application.

        Callers must have already validated `value` through the safety
        registry; this method re-asserts connectivity but trusts the
        API layer to have done the safety pass (it does it twice).
        `as_int` comes from the registry's type annotation.
        """
        if not self._pymav_conn:
            raise ConnectionError("Vehicle not connected")

        name_upper = name.upper()
        param_id = name_upper.encode('utf-8')
        if len(param_id) > 16:
            param_id = param_id[:16]

        if as_int:
            int_val = int(round(value))
            float_val = struct.unpack('f', struct.pack('i', int_val))[0]
            param_type = 6  # MAV_PARAM_TYPE_INT32
        else:
            float_val = value
            param_type = 9  # MAV_PARAM_TYPE_REAL32

        attempts = 3
        timeout = 1.0
        delay = 0.2
        last_exc: Exception | None = None

        for attempt in range(attempts):
            fut = self._loop.create_future()
            self._pending_param_writes[name_upper] = fut
            try:
                self._pymav_conn.mav.param_set_send(
                    self._target_system,
                    self._target_component,
                    param_id,
                    float_val,
                    param_type
                )
                
                confirmed_val = await asyncio.wait_for(fut, timeout=timeout)
                log.info("Parameter written: %s = %s (confirmed: %s)", name, value, confirmed_val)
                HUB.publish("param_written", {"param": name, "value": value})
                return
            except (asyncio.TimeoutError, Exception) as exc:
                last_exc = exc
                if attempt < attempts - 1:
                    log.warning("Param write %s failed (attempt %d/%d): %s — retrying",
                                name, attempt + 1, attempts, exc)
                    await asyncio.sleep(delay)
                    delay *= 2
            finally:
                self._pending_param_writes.pop(name_upper, None)

        raise TimeoutError(
            f"Could not write parameter {name} = {value} after {attempts} attempts: {last_exc}"
        )

    # ------------------------------------------------------------------ #
    # Raw firehose (data plane, dedicated thread)
    # ------------------------------------------------------------------ #
    def _request_message_intervals(self, conn, target_system: int, target_component: int) -> None:
        is_serial = False
        try:
            if ROUTER.is_running and ROUTER.status().mode == "serial":
                is_serial = True
        except Exception:
            pass

        if is_serial:
            log.info("Serial link: scaling down requested telemetry streams to conserve bandwidth")
            # Downsampled rates for low-bandwidth serial modems (total ~24 Hz vs ~90 Hz)
            intervals = {
                30: 100000,   # ATTITUDE: 10 Hz
                83: 500000,   # ATTITUDE_TARGET: 2 Hz
                32: 500000,   # LOCAL_POSITION_NED: 2 Hz
                85: 500000,   # POSITION_TARGET_LOCAL_NED: 2 Hz
                193: 1000000, # EKF_STATUS_REPORT: 1 Hz
                230: 1000000, # ESTIMATOR_STATUS: 1 Hz
                241: 1000000, # VIBRATION: 1 Hz
                36: 500000,   # SERVO_OUTPUT_RAW: 2 Hz
                375: 500000,  # ACTUATOR_OUTPUT_STATUS: 2 Hz
                74: 1000000,  # VFR_HUD: 1 Hz
            }
        else:
            log.info("High-bandwidth link: requesting full-rate telemetry streams")
            # Message intervals: key: message_id, value: interval in microseconds
            # ATTITUDE (30): 20 Hz (50,000 us)
            # ATTITUDE_TARGET (83): 10 Hz (100,000 us)
            # LOCAL_POSITION_NED (32): 10 Hz (100,000 us)
            # POSITION_TARGET_LOCAL_NED (85): 10 Hz (100,000 us)
            # EKF_STATUS_REPORT (193): 5 Hz (200,000 us)
            # ESTIMATOR_STATUS (230): 5 Hz (200,000 us)
            # VIBRATION (241): 5 Hz (200,000 us)
            # SERVO_OUTPUT_RAW (36): 10 Hz (100,000 us)
            # ACTUATOR_OUTPUT_STATUS (375): 10 Hz (100,000 us)
            # VFR_HUD (74): 5 Hz (200,000 us)
            intervals = {
                30: 50000,    # ATTITUDE
                83: 100000,   # ATTITUDE_TARGET
                32: 100000,   # LOCAL_POSITION_NED
                85: 100000,   # POSITION_TARGET_LOCAL_NED
                193: 200000,  # EKF_STATUS_REPORT
                230: 200000,  # ESTIMATOR_STATUS
                241: 200000,  # VIBRATION
                36: 100000,   # SERVO_OUTPUT_RAW
                375: 100000,  # ACTUATOR_OUTPUT_STATUS
                74: 200000,   # VFR_HUD
            }
        for msg_id, interval_us in intervals.items():
            if self._stop_flag.is_set():
                break
            try:
                conn.mav.command_long_send(
                    target_system,
                    target_component,
                    511, # MAV_CMD_SET_MESSAGE_INTERVAL
                    0,   # confirmation
                    msg_id,
                    interval_us,
                    0, 0, 0, 0, 0
                )
                log.debug("Sent message interval request for msg %d: %d us", msg_id, interval_us)
            except Exception as e:
                log.warning("Failed to send message interval request for msg %d: %s", msg_id, e)
            import time
            time.sleep(0.05)

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
        conn = self._pymav_conn
        if not conn:
            log.error("pymavlink worker started without an active connection")
            return

        rates_requested = False

        while not self._stop_flag.is_set():
            try:
                msg = conn.recv_match(blocking=True, timeout=1.0)
            except Exception as e:
                if self._stop_flag.is_set():
                    break
                log.debug("Error in recv_match: %s", e)
                time.sleep(0.1)
                continue

            if msg is None:
                continue
            mtype = msg.get_type()

            # Vehicle heartbeats: one-shot autopilot/MAV_TYPE capture +
            # continuous flight-mode tracking (ignore other GCS on the link).
            if mtype == "HEARTBEAT" and msg.type != _MAV_TYPE_GCS:
                # PX4-only gate: a non-PX4 autopilot on the link is refused.
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

                self._last_heartbeat_time = time.monotonic()

                if self._connected_future and not self._connected_future.done():
                    target_sys = msg.get_srcSystem()
                    target_comp = msg.get_srcComponent()
                    self._loop.call_soon_threadsafe(
                        _safe_set_result, self._connected_future, (target_sys, target_comp)
                    )

                if not rates_requested:
                    rates_requested = True
                    target_sys = msg.get_srcSystem()
                    target_comp = msg.get_srcComponent()
                    threading.Thread(
                        target=self._request_message_intervals,
                        args=(conn, target_sys, target_comp),
                        name="telemetry-rate-request",
                        daemon=True
                    ).start()

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

            if mtype == "PARAM_VALUE":
                pid = msg.param_id
                if isinstance(pid, bytes):
                    try:
                        pid = pid.decode('utf-8')
                    except Exception:
                        pid = str(pid)
                pid = pid.replace('\x00', '').strip()
                pid_upper = pid.upper()
                
                val = _extract_param_value(msg)
                
                if self._bulk_download_active:
                    self._bulk_params[pid_upper] = val
                    self._bulk_param_count = msg.param_count
                    if len(self._bulk_params) >= self._bulk_param_count and self._bulk_event:
                        self._loop.call_soon_threadsafe(self._bulk_event.set)
                
                if pid_upper in self._pending_param_reads:
                    fut = self._pending_param_reads.get(pid_upper)
                    if fut and not fut.done():
                        self._loop.call_soon_threadsafe(_safe_set_result, fut, val)
                
                if pid_upper in self._pending_param_writes:
                    fut = self._pending_param_writes.get(pid_upper)
                    if fut and not fut.done():
                        self._loop.call_soon_threadsafe(_safe_set_result, fut, val)

            elif mtype == "AUTOPILOT_VERSION":
                if self._pending_version_read and not self._pending_version_read.done():
                    self._loop.call_soon_threadsafe(_safe_set_result, self._pending_version_read, msg)

            channel = _WATCHED_MESSAGES.get(mtype)
            if channel is None:
                continue
            payload = msg.to_dict()
            payload.pop("mavpackettype", None)
            # Cross the thread boundary into asyncio land.
            if self._loop and self._loop.is_running():
                self._loop.call_soon_threadsafe(HUB.publish, channel, payload)


CONNECTION = ConnectionManager()
