"""
Airframe discovery: SYS_AUTOSTART (primary) + MAV_TYPE (cross-check).

PX4 encodes the selected airframe as a numeric autostart ID; the leading
digits group airframes into families mapped onto the safety classes used
by the safety registry. The HEARTBEAT MAV_TYPE field provides a coarser
second opinion used as a fallback when the parameter read fails and as a
sanity cross-check when it succeeds.

MINT supports multirotor (MR), fixed-wing (FW) and VTOL airframes only.
Ground, surface, underwater and lighter-than-air vehicles (rovers, boats,
submarines, balloons, airships) are explicitly out of scope: their control
architecture shares no loops with the rate/attitude/position cascade the
analysis engines model, so applying that advice to them is unsafe. Such
airframes are *rejected* at discovery time rather than coerced into a
default class.

Reference: https://docs.px4.io/main/en/airframes/airframe_reference.html
"""
from __future__ import annotations

import json
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional


class UnsupportedAirframeError(Exception):
    """Raised when a vehicle is not an MR/FW/VTOL airframe."""


@dataclass
class AirframeInfo:
    sys_autostart: int           # -1 when only MAV_TYPE was available
    airframe_class: str          # key into safety_registry.json
    label: str                   # human-friendly description
    mav_type: Optional[int] = None
    source: str = "SYS_AUTOSTART"   # or "MAV_TYPE" (fallback path)


# ---------------------------------------------------------------------------
# Airframe ID tables (externalised to airframe_ids.json for maintenance)
# ---------------------------------------------------------------------------
def _locate_ids() -> Path:
    """Find airframe_ids.json in dev and in the PyInstaller bundle.

    Mirrors core.safety_registry._locate_registry: module-relative first,
    then the frozen bundle path under sys._MEIPASS.
    """
    sibling = Path(__file__).with_name("airframe_ids.json")
    if sibling.is_file():
        return sibling
    if getattr(sys, "frozen", False):
        candidate = Path(getattr(sys, "_MEIPASS")) / "app" / "mavlink" / "airframe_ids.json"
        if candidate.is_file():
            return candidate
    raise FileNotFoundError(
        f"airframe_ids.json not found near {sibling} — "
        f"check the PyInstaller datas mapping in mint.spec"
    )


with open(_locate_ids(), "r", encoding="utf-8") as _f:
    _IDS = json.load(_f)

# Safety classes MINT analyses. Everything else is out of scope.
SUPPORTED_CLASSES = frozenset(_IDS["supported_classes"])

# Simulation/HIL/SIH frames: explicit per-ID map (the [1000,1999] range mixes
# vehicle types, so the Ackermann rover sim stays rejected by omission).
#   {sys_autostart: (class, label)}
_SIM_IDS: dict[int, tuple[str, str]] = {
    int(k): (v["class"], v["label"]) for k, v in _IDS["sim_ids"].items()
}

# SYS_AUTOSTART family ranges. (start, end_inclusive, class, label); first
# match wins. Out-of-scope IDs are absent by design (no match -> rejected).
_RANGES: list[tuple[int, int, str, str]] = [
    (r["start"], r["end"], r["class"], r["label"]) for r in _IDS["ranges"]
]

# MAV_TYPE -> safety class (coarse HEARTBEAT cross-check / fallback).
_MAV_TYPE_CLASS: dict[int, tuple[str, str]] = {
    int(k): (v["class"], v["label"]) for k, v in _IDS["mav_type_class"].items()
}

# MAV_TYPE values MINT explicitly refuses (ground/surface/underwater/LTA),
# kept so the rejection message can name the vehicle kind.
_MAV_TYPE_OUT_OF_SCOPE: dict[int, str] = {
    int(k): v for k, v in _IDS["mav_type_out_of_scope"].items()
}


def classify_mav_type(mav_type: int) -> Optional[AirframeInfo]:
    """Coarse fallback classification from HEARTBEAT.MAV_TYPE.

    Raises UnsupportedAirframeError for a recognised out-of-scope vehicle
    (rover/boat/sub/balloon/airship). Returns None when the type is simply
    unrecognised — the caller decides whether absence is fatal.
    """
    if mav_type in _MAV_TYPE_OUT_OF_SCOPE:
        raise UnsupportedAirframeError(
            f"{_MAV_TYPE_OUT_OF_SCOPE[mav_type]} (MAV_TYPE={mav_type}) is not a "
            f"multirotor, fixed-wing or VTOL airframe — MINT does not support it."
        )
    entry = _MAV_TYPE_CLASS.get(mav_type)
    if entry is None:
        return None
    cls, label = entry
    return AirframeInfo(
        sys_autostart=-1,
        airframe_class=cls,
        label=f"{label} (from MAV_TYPE — SYS_AUTOSTART unavailable)",
        mav_type=mav_type,
        source="MAV_TYPE",
    )


def classify_airframe(sys_autostart: int) -> AirframeInfo:
    """Map a SYS_AUTOSTART value to its safety class.

    Only multirotor, fixed-wing/delta and VTOL families resolve to a class
    (including their HIL/SIH simulation frames). Any other (or unrecognised)
    autostart ID raises UnsupportedAirframeError: coercing an out-of-scope
    airframe into MULTIROTOR would hand it advice derived from a control
    cascade it does not run."""
    sim = _SIM_IDS.get(sys_autostart)
    if sim is not None:
        cls, label = sim
        return AirframeInfo(sys_autostart, cls, label)
    for lo, hi, cls, label in _RANGES:
        if lo <= sys_autostart <= hi:
            return AirframeInfo(sys_autostart, cls, label)
    raise UnsupportedAirframeError(
        f"SYS_AUTOSTART={sys_autostart} is not a multirotor, fixed-wing or "
        f"VTOL airframe (or is unrecognised) — MINT does not support it."
    )
