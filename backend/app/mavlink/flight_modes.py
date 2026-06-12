"""
PX4 flight mode decoding + control-loop activity mapping.

PX4 packs its mode into HEARTBEAT.custom_mode: main mode in bits 16-23,
sub mode (AUTO flavors) in bits 24-31.

`active_loops()` answers the question the cascade analyzer cares about:
which control loops is the FC actually closing right now? Analysing a
loop the FC isn't running produces garbage verdicts — e.g. in ACRO the
attitude loop is the *pilot*, so attitude tracking error is pilot style,
not tuning. The control domain matters too: a fixed-wing in MANUAL is
direct stick-to-surface passthrough with no loops closed at all.
"""
from __future__ import annotations

MAIN_MODES = {
    1: "MANUAL", 2: "ALTCTL", 3: "POSCTL", 4: "AUTO",
    5: "ACRO", 6: "OFFBOARD", 7: "STABILIZED", 8: "RATTITUDE",
}
AUTO_SUB_MODES = {
    1: "READY", 2: "TAKEOFF", 3: "LOITER", 4: "MISSION",
    5: "RTL", 6: "LAND", 8: "FOLLOW",
}

LOOPS = ("rate", "attitude", "velocity", "position")


def decode_mode(custom_mode: int) -> str:
    """custom_mode -> human label, e.g. 'POSCTL' or 'AUTO.MISSION'."""
    main = (custom_mode >> 16) & 0xFF
    sub = (custom_mode >> 24) & 0xFF
    name = MAIN_MODES.get(main, f"UNKNOWN({main})")
    if name == "AUTO" and sub in AUTO_SUB_MODES:
        name = f"AUTO.{AUTO_SUB_MODES[sub]}"
    return name


# Which loops the FC closes, per main mode, per control domain.
_MC_LOOPS = {
    "MANUAL": {"rate", "attitude"},        # MC manual == stabilized
    "STABILIZED": {"rate", "attitude"},
    "ACRO": {"rate"},
    "RATTITUDE": {"rate", "attitude"},
    "ALTCTL": {"rate", "attitude", "velocity"},        # z-velocity/alt
    "POSCTL": {"rate", "attitude", "velocity", "position"},
    "AUTO": {"rate", "attitude", "velocity", "position"},
    "OFFBOARD": {"rate", "attitude", "velocity", "position"},
}
_FW_LOOPS = {
    "MANUAL": set(),                       # direct surface passthrough
    "ACRO": {"rate"},
    "STABILIZED": {"rate", "attitude"},
    "ALTCTL": {"rate", "attitude", "velocity"},        # TECS altitude
    "POSCTL": {"rate", "attitude", "velocity"},
    "AUTO": {"rate", "attitude", "velocity", "position"},
    "OFFBOARD": {"rate", "attitude", "velocity", "position"},
}


def active_loops(mode_name: str, domain: str) -> set[str]:
    """`domain` is "MC" or "FW" — for VTOLs this flips with vtol_state."""
    table = _FW_LOOPS if domain == "FW" else _MC_LOOPS
    return set(table.get(mode_name.split(".")[0], {"rate", "attitude"}))
