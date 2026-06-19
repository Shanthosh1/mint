"""
System endpoints: host info, serial scan, router control, connection.
"""
from __future__ import annotations

import logging
from typing import Literal, Optional

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field, model_validator

from ..core import config, platform_utils
from ..mavlink.connection import CONNECTION, UnsupportedVehicleError
from ..mavlink.router_manager import ROUTER, ConnectionTarget

log = logging.getLogger("mint.api.system")
router = APIRouter(prefix="/api/system", tags=["system"])


class RouterStartRequest(BaseModel):
    """One MAVLink source, four flavors.

    serial      — USB FC / telemetry radio (device + baud)
    udp_listen  — vehicle or SITL pushes to us (bind host + port)
    udp_connect — we push/pull against a remote UDP peer (host + port)
    tcp_connect — serial-over-ethernet bridges, some SITL setups (host + port)
    """
    mode: Literal["serial", "udp_listen", "udp_connect", "tcp_connect"] = "serial"
    serial_device: Optional[str] = Field(None, examples=["/dev/tty.usbmodem01", "COM7"])
    baud: int = Field(default=config.DEFAULT_BAUD, ge=9600, le=3_000_000)
    host: str = Field(default="0.0.0.0", examples=["0.0.0.0", "192.168.144.12"])
    port: int = Field(default=14550, ge=1, le=65535)

    @model_validator(mode="after")
    def _check_mode_fields(self):
        if self.mode == "serial" and not self.serial_device:
            raise ValueError("serial mode requires serial_device")
        if self.mode in ("udp_connect", "tcp_connect") and self.host in ("", "0.0.0.0"):
            raise ValueError(f"{self.mode} requires a remote host address")
        return self


class ActuatorMapConfig(BaseModel):
    hover_motors: list[int] = Field(default_factory=list)
    thrust_motors: list[int] = Field(default_factory=list)
    control_surfaces: list[int] = Field(default_factory=list)
    tilt_servos: list[int] = Field(default_factory=list)


import uuid

BACKEND_SESSION_ID = str(uuid.uuid4())


@router.get("/host")
def host_info() -> dict:
    """OS autodetection + router binary availability."""
    info = platform_utils.host_info_dict()
    info["backend_session_id"] = BACKEND_SESSION_ID
    return info


@router.get("/serial-ports")
def serial_ports() -> list[dict]:
    """Scan available serial/COM ports with FC-likelihood hints."""
    try:
        return platform_utils.serial_ports_dict()
    except OSError as exc:
        raise HTTPException(500, f"Serial enumeration failed: {exc}")


@router.get("/router")
def router_status() -> dict:
    return ROUTER.status_dict()


@router.post("/router/start")
async def router_start(req: RouterStartRequest) -> dict:
    """Launch mavp2p splitting the source -> QGC + backend UDP."""
    target = ConnectionTarget(
        mode=req.mode,
        device=req.serial_device,
        baud=req.baud,
        host=req.host,
        port=req.port,
    )
    try:
        await ROUTER.start(target)
    except FileNotFoundError as exc:
        raise HTTPException(424, str(exc))          # binary missing
    except PermissionError as exc:
        raise HTTPException(403, str(exc))          # serial permission
    except (RuntimeError, ValueError) as exc:
        raise HTTPException(502, str(exc))          # router died / bad target
    return ROUTER.status_dict()


@router.post("/router/stop")
async def router_stop() -> dict:
    await ROUTER.stop()
    return ROUTER.status_dict()


@router.post("/vehicle/connect")
async def vehicle_connect() -> dict:
    """Begin the MAVLink handshake against the routed endpoint."""
    if not ROUTER.is_running:
        raise HTTPException(409, "Start the telemetry router first")
    try:
        state = await CONNECTION.connect()
    except UnsupportedVehicleError as exc:
        # Non-PX4 stack, pre-1.14 firmware, or out-of-scope airframe.
        raise HTTPException(422, str(exc))
    return {
        "connected": state.connected,
        "fw_version": state.fw_version,
        "airframe": (
            {
                "sys_autostart": state.airframe.sys_autostart,
                "airframe_class": state.airframe.airframe_class,
                "label": state.airframe.label,
            } if state.airframe else None
        ),
        "discovery_failed": state.discovery_failed,
        "actuator_map": state.actuator_map,
    }


@router.post("/vehicle/disconnect")
async def vehicle_disconnect() -> dict:
    await CONNECTION.disconnect()
    return {"connected": False}


@router.post("/actuator-map")
async def update_actuator_map(req: ActuatorMapConfig) -> dict:
    """Explicitly override the vehicle actuator map with manual config from pilot."""
    CONNECTION.state.actuator_map = {
        "hover_motors": req.hover_motors,
        "thrust_motors": req.thrust_motors,
        "control_surfaces": req.control_surfaces,
        "tilt_servos": req.tilt_servos,
    }
    CONNECTION.state.discovery_failed = False
    CONNECTION._publish_airframe()
    return {"success": True, "actuator_map": CONNECTION.state.actuator_map}


@router.get("/config/frontend")
def get_frontend_config() -> dict:
    """Read frontend/config.yaml and return it as a JSON dict."""
    import yaml
    try:
        with open(config.FRONTEND_CONFIG_PATH, "r") as f:
            return yaml.safe_load(f) or {}
    except Exception as e:
        log.error("Failed to load frontend config.yaml: %s", e)
        return {}
