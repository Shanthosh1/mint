"""
System endpoints: host info, serial scan, router control, connection.
"""
from __future__ import annotations

import logging
from typing import Literal, Optional

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field, model_validator

from ..core import config, platform_utils
from ..mavlink.connection import CONNECTION
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


@router.get("/host")
def host_info() -> dict:
    """OS autodetection + router binary availability."""
    return platform_utils.host_info_dict()


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
    """Begin the MAVSDK handshake against the routed endpoint."""
    if not ROUTER.is_running:
        raise HTTPException(409, "Start the telemetry router first")
    state = await CONNECTION.connect()
    return {
        "connected": state.connected,
        "airframe": (
            {
                "sys_autostart": state.airframe.sys_autostart,
                "airframe_class": state.airframe.airframe_class,
                "label": state.airframe.label,
            } if state.airframe else None
        ),
    }


@router.post("/vehicle/disconnect")
async def vehicle_disconnect() -> dict:
    await CONNECTION.disconnect()
    return {"connected": False}
