"""
Central runtime configuration for MINT.

All tunables live here so the PyInstaller bundle, the dev server, and tests
share one source of truth. Values can be overridden via environment
variables prefixed with MINT_ (e.g. MINT_HTTP_PORT=9000).
"""
from __future__ import annotations

import os
import sys
from pathlib import Path


def _env(name: str, default: str) -> str:
    return os.environ.get(f"MINT_{name}", default)


# ---------------------------------------------------------------------------
# Resource resolution (PyInstaller-aware)
# ---------------------------------------------------------------------------
def resource_root() -> Path:
    """
    Root folder for bundled static resources.

    When frozen by PyInstaller, data files are unpacked to sys._MEIPASS.
    In development we resolve relative to the repository root.
    """
    if getattr(sys, "frozen", False):  # PyInstaller bundle
        return Path(getattr(sys, "_MEIPASS"))
    return Path(__file__).resolve().parents[3]


RESOURCES_DIR = resource_root() / "resources"
ROUTER_BIN_DIR = RESOURCES_DIR / "bin"
FRONTEND_DIST = resource_root() / "frontend" / "dist"

# ---------------------------------------------------------------------------
# HTTP / WebSocket server
# ---------------------------------------------------------------------------
HTTP_HOST = _env("HTTP_HOST", "127.0.0.1")
HTTP_PORT = int(_env("HTTP_PORT", "8400"))

# ---------------------------------------------------------------------------
# Telemetry routing endpoints (all loopback)
# ---------------------------------------------------------------------------
QGC_UDP_PORT = int(_env("QGC_UDP_PORT", "14550"))        # QGroundControl
MAVSDK_UDP_PORT = int(_env("MAVSDK_UDP_PORT", "14540"))  # MAVSDK-Python (params, actions)
PYMAVLINK_UDP_PORT = int(_env("PYMAVLINK_UDP_PORT", "14541"))  # raw message firehose

DEFAULT_BAUD = int(_env("DEFAULT_BAUD", "57600"))

# NOTE: the ports above are *defaults*. When the router source is a local
# UDP listener that collides with one of them (e.g. SITL on 14540), the
# RouterManager shifts the fan-out to alternate ports at start time — the
# connection layer reads live ports from ROUTER, never from here directly.

# ---------------------------------------------------------------------------
# Live analysis windows
# ---------------------------------------------------------------------------
WS_STREAM_HZ = float(_env("WS_STREAM_HZ", "10"))          # downsampled UI rate
RMSE_WINDOW_S = float(_env("RMSE_WINDOW_S", "5.0"))       # running RMSE window
STICK_IDLE_TIMEOUT_S = float(_env("STICK_IDLE_TIMEOUT_S", "10.0"))
EKF_RATIO_WARN = float(_env("EKF_RATIO_WARN", "0.8"))     # amber threshold
EKF_RATIO_FAIL = float(_env("EKF_RATIO_FAIL", "1.0"))     # red threshold

# ---------------------------------------------------------------------------
# Supported autopilot / firmware
# ---------------------------------------------------------------------------
# MINT targets PX4 only, from this firmware version onward. ArduPilot and
# other stacks use different message sets, parameter names and EKF semantics,
# so their telemetry would be silently misanalysed. The gate is enforced at
# both the live connection and the ULog upload paths.
MIN_PX4_VERSION = (
    int(_env("MIN_PX4_MAJOR", "1")),
    int(_env("MIN_PX4_MINOR", "14")),
)
MAV_AUTOPILOT_PX4 = 12          # MAV_AUTOPILOT_PX4 (HEARTBEAT.autopilot)

# ---------------------------------------------------------------------------
# ULog upload pipeline
# ---------------------------------------------------------------------------
ULOG_UPLOAD_CHUNK = 1024 * 1024            # 1 MiB chunks while spooling to disk
# Default upload cap. pyulog materialises whole datasets into pandas frames in
# one shot, so a multi-hundred-MiB log can spike memory well past the file size
# and OOM the backend — taking live telemetry down with it. The conservative
# default protects 8 GiB field laptops; raise MINT_ULOG_MAX_MIB on a workstation.
ULOG_MAX_BYTES = int(_env("ULOG_MAX_MIB", "800")) * 1024 * 1024
ULOG_TMP_DIR = Path(_env("ULOG_TMP_DIR", str(Path.home() / ".mint" / "uploads")))
