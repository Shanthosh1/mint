"""
Airframe discovery: SYS_AUTOSTART (primary) + MAV_TYPE (cross-check).

PX4 encodes the selected airframe as a numeric autostart ID; the leading
digits group airframes into families mapped onto the safety classes used
by the safety registry. The HEARTBEAT MAV_TYPE field provides a coarser
second opinion used as a fallback when the parameter read fails and as a
sanity cross-check when it succeeds.

Reference: https://docs.px4.io/main/en/airframes/airframe_reference.html
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass
class AirframeInfo:
    sys_autostart: int           # -1 when only MAV_TYPE was available
    airframe_class: str          # key into safety_registry.json
    label: str                   # human-friendly description
    mav_type: Optional[int] = None
    source: str = "SYS_AUTOSTART"   # or "MAV_TYPE" (fallback path)


# (range_start, range_end_inclusive, class, label) — first match wins.
_RANGES: list[tuple[int, int, str, str]] = [
    (2100, 2199, "DELTA_WING", "Flying Wing / Delta"),
    (2200, 2999, "FIXED_WING", "Fixed-Wing (standard)"),
    (3000, 3999, "FIXED_WING", "Fixed-Wing (simulation/custom)"),
    (4000, 4999, "MULTIROTOR", "Quadrotor X"),
    (5000, 5999, "MULTIROTOR", "Quadrotor +"),
    (6000, 6999, "MULTIROTOR", "Hexarotor"),
    (7000, 7999, "MULTIROTOR", "Hexarotor +/Coax"),
    (8000, 8999, "MULTIROTOR", "Octorotor"),
    (9000, 9999, "MULTIROTOR", "Octorotor Coax"),
    (10000, 10999, "MULTIROTOR", "Wide/Custom Multirotor"),
    (11000, 11999, "MULTIROTOR", "Hexa Coax"),
    (12000, 12999, "MULTIROTOR", "Octo Coax Wide"),
    (13000, 13999, "VTOL", "VTOL (tiltrotor/standard/tailsitter)"),
    (14000, 14999, "VTOL", "VTOL Tiltwing"),
]


# MAV_TYPE -> safety class (coarse; HEARTBEAT enum values).
_MAV_TYPE_CLASS: dict[int, tuple[str, str]] = {
    1: ("FIXED_WING", "Fixed-Wing"),            # MAV_TYPE_FIXED_WING
    2: ("MULTIROTOR", "Quadrotor"),             # MAV_TYPE_QUADROTOR
    13: ("MULTIROTOR", "Hexarotor"),            # MAV_TYPE_HEXAROTOR
    14: ("MULTIROTOR", "Octorotor"),            # MAV_TYPE_OCTOROTOR
    15: ("MULTIROTOR", "Tricopter"),            # MAV_TYPE_TRICOPTER
    19: ("VTOL", "VTOL Tailsitter (duo)"),      # MAV_TYPE_VTOL_*
    20: ("VTOL", "VTOL Tailsitter (quad)"),
    21: ("VTOL", "VTOL Tiltrotor"),
    22: ("VTOL", "VTOL Fixed-rotor"),
    23: ("VTOL", "VTOL Tailsitter"),
    24: ("VTOL", "VTOL Tiltwing"),
    25: ("VTOL", "VTOL"),
}


def classify_mav_type(mav_type: int) -> Optional[AirframeInfo]:
    """Coarse fallback classification from HEARTBEAT.MAV_TYPE."""
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
    """Map a SYS_AUTOSTART value to its safety class. Unknown -> MULTIROTOR
    with a warning label, because multirotor limits are the most conservative
    set in the registry (smallest step deltas on shared params)."""
    for lo, hi, cls, label in _RANGES:
        if lo <= sys_autostart <= hi:
            return AirframeInfo(sys_autostart, cls, label)
    return AirframeInfo(
        sys_autostart,
        "MULTIROTOR",
        f"Unknown airframe ID {sys_autostart} — defaulting to most-conservative limits",
    )
