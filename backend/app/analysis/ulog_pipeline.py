"""
Post-flight ULog ingestion and orchestration.

Responsibilities:
  1. Spool uploaded .ulg bytes to disk in chunks (the API layer streams the
     request body — the full file is never held in RAM, which keeps >100 MB
     logs safe on low-spec field laptops).
  2. Parse only the datasets each analyzer needs (pyulog loads lazily per
     dataset name) and hand pandas frames to the analysis modules.
  3. Aggregate results into a single report dict for the frontend.
"""
from __future__ import annotations

import logging
import uuid
from pathlib import Path

import pandas as pd
from pyulog import ULog

import re

from ..core import config
from ..mavlink.airframe import UnsupportedAirframeError, classify_airframe
from . import (actuator_saturation, ekf_offline, filter_advisor,
               pid_offline, vibration_fft)

log = logging.getLogger("mint.ulog")


class UnsupportedLogError(Exception):
    """Raised when a ULog is not from a supported PX4 target.

    Covers a non-PX4 log, a PX4 firmware older than the supported floor,
    or an out-of-scope airframe (rover/boat/sub/balloon)."""


def _parse_px4_version(ver_sw: str | None) -> tuple[int, int, int] | None:
    """Pull (major, minor, patch) out of a ULog ver_sw string.

    PX4 stores values like "v1.14.0", "1.14.0-rc1", or "1.14"; return None when no
    dotted version can be found.
    """
    if not ver_sw:
        return None
    m = re.search(r"(\d+)\.(\d+)(?:\.(\d+))?", str(ver_sw))
    if not m:
        return None
    major = int(m.group(1))
    minor = int(m.group(2))
    patch = int(m.group(3)) if m.group(3) else 0
    return major, minor, patch

# Only these datasets are ever materialized into memory.
_DATASETS = [
    "sensor_combined",
    "actuator_controls_0",
    "actuator_outputs",
    "vehicle_local_position",
    "vehicle_gps_position",
    "sensor_baro",
    "vehicle_attitude",
    "vehicle_attitude_setpoint",
    "vehicle_angular_velocity",   # filtered body rates (PID analysis)
    "vehicle_rates_setpoint",     # commanded body rates (PID analysis)
    "vehicle_air_data",           # baro altitude (EKF2_BARO_DELAY)
    "vehicle_status",
    "vtol_vehicle_status",
]


class UlogTooLargeError(Exception):
    pass


def allocate_upload_path() -> Path:
    """Reserve a unique temp path for an incoming upload."""
    config.ULOG_TMP_DIR.mkdir(parents=True, exist_ok=True)
    return config.ULOG_TMP_DIR / f"{uuid.uuid4().hex}.ulg"


async def spool_upload(stream, dest: Path) -> int:
    """
    Write an async byte stream to `dest` in ULOG_UPLOAD_CHUNK pieces.

    `stream` is any async iterator of bytes (FastAPI's UploadFile.read
    loop). Enforces ULOG_MAX_BYTES and cleans up on failure.
    """
    written = 0
    try:
        with open(dest, "wb") as f:
            while True:
                chunk = await stream.read(config.ULOG_UPLOAD_CHUNK)
                if not chunk:
                    break
                written += len(chunk)
                if written > config.ULOG_MAX_BYTES:
                    raise UlogTooLargeError(
                        f"Upload exceeds {config.ULOG_MAX_BYTES // 2**20} MiB cap"
                    )
                f.write(chunk)
    except Exception:
        dest.unlink(missing_ok=True)
        raise
    return written


def dataset_frame(ulog: ULog, name: str, instance: int = 0) -> pd.DataFrame | None:
    """Extract one ULog dataset as a DataFrame, or None if absent."""
    try:
        ds = ulog.get_dataset(name, instance)
    except (KeyError, IndexError, ValueError):
        return None
    return pd.DataFrame(ds.data)


def _discover_actuators_from_params(params: dict, airframe_class: str) -> dict:
    """Reconstruct the actuator_map dictionary from ULog initial parameters dictionary."""
    actuator_map = {
        "hover_motors": [],
        "thrust_motors": [],
        "control_surfaces": [],
        "tilt_servos": []
    }

    # 1. Identify parameter family
    selected_family = None
    if "ACT_FUNC1" in params:
        selected_family = "ACT_FUNC"
    elif "SIM_GZ_EC_FUNC1" in params:
        selected_family = "SIM_GZ"
    elif "HIL_ACT_FUNC1" in params:
        selected_family = "HIL"
    elif "PWM_MAIN_FUNC1" in params:
        selected_family = "PWM"

    if not selected_family:
        return actuator_map

    def map_func(func_val: int, channel_idx: int):
        if 101 <= func_val <= 112:
            if airframe_class == "MULTIROTOR":
                actuator_map["hover_motors"].append(channel_idx)
            elif airframe_class in ("FIXED_WING", "DELTA_WING"):
                actuator_map["thrust_motors"].append(channel_idx)
            elif airframe_class == "VTOL":
                if 101 <= func_val <= 104:
                    actuator_map["hover_motors"].append(channel_idx)
                else:
                    actuator_map["thrust_motors"].append(channel_idx)
        elif 201 <= func_val <= 208:
            actuator_map["control_surfaces"].append(channel_idx)
        elif 301 <= func_val <= 308:
            actuator_map["tilt_servos"].append(channel_idx)

    # 2. Extract configuration
    if selected_family == "ACT_FUNC":
        for i in range(1, 17):
            val = params.get(f"ACT_FUNC{i}")
            if val is not None and int(val) > 0:
                map_func(int(val), i - 1)
    elif selected_family == "SIM_GZ":
        num_esc = 0
        for i in range(1, 9):
            val = params.get(f"SIM_GZ_EC_FUNC{i}")
            if val is not None and int(val) > 0:
                num_esc = max(num_esc, i)
        
        for i in range(1, num_esc + 1):
            val = params.get(f"SIM_GZ_EC_FUNC{i}")
            if val is not None and int(val) > 0:
                map_func(int(val), i - 1)
                
        for j in range(1, 9):
            val = params.get(f"SIM_GZ_SV_FUNC{j}")
            if val is not None and int(val) > 0:
                map_func(int(val), num_esc + j - 1)
    elif selected_family == "HIL":
        for i in range(1, 17):
            val = params.get(f"HIL_ACT_FUNC{i}")
            if val is not None and int(val) > 0:
                map_func(int(val), i - 1)
    elif selected_family == "PWM":
        for i in range(1, 9):
            val = params.get(f"PWM_MAIN_FUNC{i}")
            if val is not None and int(val) > 0:
                map_func(int(val), i - 1)
        for j in range(1, 9):
            val = params.get(f"PWM_AUX_FUNC{j}")
            if val is not None and int(val) > 0:
                map_func(int(val), 8 + j - 1)

    return actuator_map


def analyze(path: Path) -> dict:
    """
    Run the full post-flight pipeline on a spooled .ulg file.

    Returns a JSON-serializable report. Individual analyzers degrade
    gracefully: a missing dataset yields a "skipped" section instead of
    failing the whole report.
    """
    log.info("Parsing ULog: %s", path)
    ulog = ULog(str(path), message_name_filter_list=_DATASETS)

    # --- Supported-target gate (PX4 only, >= MIN_PX4_VERSION, MR/FW/VTOL) ---
    sys_name = str(ulog.msg_info_dict.get("sys_name", "")).strip()
    if sys_name and sys_name.upper() != "PX4":
        raise UnsupportedLogError(
            f"Log was produced by '{sys_name}', not PX4. MINT supports PX4 only."
        )
    
    ver_sw = ulog.msg_info_dict.get("ver_sw")
    if ver_sw:
        ver = _parse_px4_version(ver_sw)
        if ver is not None:
            if ver[:2] < config.MIN_PX4_VERSION:
                floor = config.MIN_PX4_VERSION
                raise UnsupportedLogError(
                    f"Log is from PX4 v{ver[0]}.{ver[1]}.{ver[2]}, older than the "
                    f"supported minimum v{floor[0]}.{floor[1]}."
                )
        else:
            log.warning("Could not parse PX4 firmware version string '%s' — proceeding with analysis.", ver_sw)
    else:
        log.warning("PX4 firmware version (ver_sw) is missing from log metadata — proceeding with caution.")

    # Airframe class from the log itself — drives which gain parameters
    # the PID advisor targets, independent of any live connection. An
    # out-of-scope airframe (rover/boat/sub/balloon) is rejected outright.
    sys_autostart = ulog.initial_parameters.get("SYS_AUTOSTART")
    if not sys_autostart:
        raise UnsupportedLogError(
            "Log has no SYS_AUTOSTART parameter — cannot establish a "
            "supported airframe class."
        )
    try:
        airframe = classify_airframe(int(sys_autostart))
    except UnsupportedAirframeError as exc:
        raise UnsupportedLogError(str(exc)) from exc

    report: dict = {
        "file": path.name,
        "duration_s": round((ulog.last_timestamp - ulog.start_timestamp) / 1e6, 1),
        "px4_version": ulog.msg_info_dict.get("ver_sw", "unknown"),
        "sys_autostart": sys_autostart,
        "airframe_class": airframe.airframe_class,
        "airframe_label": airframe.label,
        "initial_params": {
            k: v for k, v in ulog.initial_parameters.items()
            if k.startswith(("MC_", "FW_", "IMU_", "EKF2_"))
        },
        "sections": {},
    }

    sensor = dataset_frame(ulog, "sensor_combined")
    
    # Try reading actuator_outputs first, fallback to actuator_controls_0
    actuators = dataset_frame(ulog, "actuator_outputs")
    is_physical = True
    if actuators is None:
        actuators = dataset_frame(ulog, "actuator_controls_0")
        is_physical = False

    gps = dataset_frame(ulog, "vehicle_gps_position")
    local_pos = dataset_frame(ulog, "vehicle_local_position")
    baro = dataset_frame(ulog, "sensor_baro")
    rates_sp = dataset_frame(ulog, "vehicle_rates_setpoint")
    ang_vel = dataset_frame(ulog, "vehicle_angular_velocity")
    air_data = dataset_frame(ulog, "vehicle_air_data")
    status = dataset_frame(ulog, "vehicle_status")
    vtol_status = dataset_frame(ulog, "vtol_vehicle_status")

    sections = report["sections"]
    sections["pid"] = pid_offline.analyze_pid(
        rates_sp=rates_sp, ang_vel=ang_vel, sensor=sensor,
        params=report["initial_params"],
        airframe_class=report["airframe_class"],
        status=status,
        vtol_status=vtol_status,
    )
    sections["vibration"] = (
        vibration_fft.analyze_vibration(sensor)
        if sensor is not None else {"skipped": "sensor_combined not logged"}
    )
    sections["filters"] = filter_advisor.advise_filters(
        vibration=sections["vibration"], sensor=sensor, ang_vel=ang_vel,
        params=report["initial_params"],
    )

    # Reconstruct actuator map from initial log parameters
    actuator_map = _discover_actuators_from_params(ulog.initial_parameters, report["airframe_class"])

    if actuators is not None:
        sections["actuator_saturation"] = actuator_saturation.analyze_saturation(
            actuators, actuator_map=actuator_map,
            airframe_class=report["airframe_class"], is_physical=is_physical
        )
        if not any(actuator_map.values()):
            sections["actuator_saturation"]["actuator_discovery_failed"] = True
    else:
        sections["actuator_saturation"] = {"skipped": "actuator outputs/controls not logged"}
    sections["ekf_delays"] = ekf_offline.analyze_delays(
        sensor=sensor, gps=gps, local_pos=local_pos,
        current_params=report["initial_params"],
        air_data=air_data, baro=baro,
    )
    sections["ekf_noise"] = (
        ekf_offline.analyze_hover_noise(sensor, local_pos, report["initial_params"])
        if sensor is not None else {"skipped": "sensor_combined not logged"}
    )

    return report
