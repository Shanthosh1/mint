"""
OS detection, serial-port discovery, and platform-specific binary resolution.

Everything that needs to know "what machine am I on?" goes through this
module so the rest of the codebase stays platform-agnostic.
"""
from __future__ import annotations

import os
import platform
import stat
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Optional

from serial.tools import list_ports

from . import config


@dataclass
class HostInfo:
    os_name: str          # "Windows" | "Linux" | "Darwin"
    os_label: str         # human-friendly
    machine: str          # arch, e.g. arm64 / AMD64
    router_binary: Optional[str]   # resolved absolute path, or None if missing


@dataclass
class SerialPortInfo:
    device: str           # e.g. COM7 or /dev/tty.usbmodem01
    description: str
    hwid: str
    is_px4_likely: bool   # USB VID heuristic for common FC vendors
    permission_ok: bool   # Linux: user can actually open it


# USB vendor IDs commonly seen on PX4-capable flight controllers.
_FC_USB_VIDS = {0x26AC, 0x3162, 0x2DAE, 0x1209, 0x0483}  # 3DR, Holybro, CubePilot, generic, STM


def detect_host() -> HostInfo:
    """Detect host OS and resolve the matching mavlink-routerd binary."""
    os_name = platform.system()
    labels = {"Windows": "Windows", "Linux": "Linux", "Darwin": "macOS"}
    binary = resolve_router_binary(os_name)
    return HostInfo(
        os_name=os_name,
        os_label=labels.get(os_name, os_name),
        machine=platform.machine(),
        router_binary=str(binary) if binary else None,
    )


def resolve_router_binary(os_name: Optional[str] = None) -> Optional[Path]:
    """
    Locate the bundled mavp2p executable for this platform.

    Layout inside /resources/bin/:
        mavp2p.exe   (Windows)
        mavp2p       (Linux / macOS)

    Returns None when the binary is absent so the UI can surface a clear
    "router binary not installed" state instead of a crash.
    """
    os_name = os_name or platform.system()
    name = "mavp2p.exe" if os_name == "Windows" else "mavp2p"
    candidate = config.ROUTER_BIN_DIR / name
    if not candidate.is_file():
        return None
    # Ensure the unpacked binary is executable on POSIX (PyInstaller can
    # drop the execute bit when extracting bundled data files).
    if os_name != "Windows":
        mode = candidate.stat().st_mode
        if not mode & stat.S_IXUSR:
            candidate.chmod(mode | stat.S_IXUSR | stat.S_IXGRP)
    return candidate


def _can_open(device: str) -> bool:
    """Cheap permission probe without holding the port open."""
    try:
        fd = os.open(device, os.O_RDWR | os.O_NONBLOCK)
        os.close(fd)
        return True
    except PermissionError:
        return False
    except OSError:
        # Busy/odd states still count as "reachable" for permission purposes.
        return True


def scan_serial_ports() -> list[SerialPortInfo]:
    """
    Enumerate serial/COM ports with FC-likelihood and permission hints.

    On Linux a PermissionError usually means the user is missing the
    `dialout` group — the API layer turns that into actionable advice.
    """
    ports: list[SerialPortInfo] = []
    for p in list_ports.comports():
        permission_ok = True
        if platform.system() == "Linux":
            permission_ok = _can_open(p.device)
        ports.append(
            SerialPortInfo(
                device=p.device,
                description=p.description or "Unknown device",
                hwid=p.hwid or "",
                is_px4_likely=(p.vid in _FC_USB_VIDS) if p.vid else False,
                permission_ok=permission_ok,
            )
        )
    # FC-likely ports first so the UI default selection is usually right.
    ports.sort(key=lambda x: (not x.is_px4_likely, x.device))
    return ports


def host_info_dict() -> dict:
    res = asdict(detect_host())
    res["expert_mode"] = config.EXPERT_MODE
    return res


def serial_ports_dict() -> list[dict]:
    return [asdict(p) for p in scan_serial_ports()]
