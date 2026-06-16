"""
Central runtime configuration for MINT.

All tunables live here so the PyInstaller bundle, the dev server, and tests
share one source of truth. Values can be overridden via environment
variables prefixed with MINT_ (e.g. MINT_HTTP_PORT=9000).
"""
from __future__ import annotations

import os
import sys
import yaml
from pathlib import Path


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

# Config yaml paths
BACKEND_CONFIG_PATH = resource_root() / "backend" / "config.yaml"
FRONTEND_CONFIG_PATH = resource_root() / "frontend" / "config.yaml"

# Load backend configuration
try:
    with open(BACKEND_CONFIG_PATH, "r") as f:
        _CONFIG_DATA = yaml.safe_load(f) or {}
except Exception:
    _CONFIG_DATA = {}


def _get_val(key_path: str, default):
    parts = key_path.split(".")
    env_name = "MINT_" + "_".join(parts).upper()
    if env_name in os.environ:
        val = os.environ[env_name]
        if isinstance(default, bool):
            return val.lower() in ("true", "1", "yes")
        elif isinstance(default, int):
            return int(val)
        elif isinstance(default, float):
            return float(val)
        return val

    curr = _CONFIG_DATA
    for p in parts:
        if isinstance(curr, dict) and p in curr:
            curr = curr[p]
        else:
            return default
    return curr


# ---------------------------------------------------------------------------
# HTTP / WebSocket server
# ---------------------------------------------------------------------------
HTTP_HOST = _get_val("server.http_host", "127.0.0.1")
HTTP_PORT = int(_get_val("server.http_port", 8400))

# ---------------------------------------------------------------------------
# Telemetry routing endpoints (all loopback)
# ---------------------------------------------------------------------------
QGC_UDP_PORT = int(_get_val("mavlink.qgc_udp_port", 14550))        # QGroundControl
MAVSDK_UDP_PORT = int(_get_val("mavlink.mavsdk_udp_port", 14540))  # MAVSDK-Python (params, actions)
PYMAVLINK_UDP_PORT = int(_get_val("mavlink.pymavlink_udp_port", 14541))  # raw message firehose
DEFAULT_BAUD = int(_get_val("mavlink.default_baud", 57600))

# ---------------------------------------------------------------------------
# Live analysis windows
# ---------------------------------------------------------------------------
WS_STREAM_HZ = float(_get_val("analysis.ws_stream_hz", 10.0))          # downsampled UI rate
RMSE_WINDOW_S = float(_get_val("analysis.rmse_window_s", 5.0))       # running RMSE window
STICK_IDLE_TIMEOUT_S = float(_get_val("analysis.stick_idle_timeout_s", 10.0))
EKF_RATIO_WARN = float(_get_val("analysis.ekf_ratio_warn", 0.8))     # amber threshold
EKF_RATIO_FAIL = float(_get_val("analysis.ekf_ratio_fail", 1.0))     # red threshold

# ---------------------------------------------------------------------------
# Supported autopilot / firmware
# ---------------------------------------------------------------------------
MIN_PX4_VERSION = (
    int(_get_val("mavlink.min_px4_major", 1)),
    int(_get_val("mavlink.min_px4_minor", 14)),
)
MAV_AUTOPILOT_PX4 = int(_get_val("mavlink.mav_autopilot_px4", 12))

# ---------------------------------------------------------------------------
# ULog upload pipeline
# ---------------------------------------------------------------------------
ULOG_UPLOAD_CHUNK = int(_get_val("ulog.upload_chunk", 1024 * 1024))
ULOG_MAX_BYTES = int(_get_val("ulog.max_mib", 800)) * 1024 * 1024

_tmp_dir_str = _get_val("ulog.tmp_dir", str(Path.home() / ".mint" / "uploads"))
if _tmp_dir_str.startswith("~"):
    ULOG_TMP_DIR = Path(os.path.expanduser(_tmp_dir_str))
else:
    ULOG_TMP_DIR = Path(_tmp_dir_str)


# ---------------------------------------------------------------------------
# Expert Mode Settings
# ---------------------------------------------------------------------------
EXPERT_MODE = _get_val("expert_mode.enabled", False)


# ---------------------------------------------------------------------------
# Centralized Analysis Constants
# ---------------------------------------------------------------------------

# Actuator Saturation
SAT_EPS = float(_get_val("analysis.actuator_saturation.sat_eps", 0.005))
BURST_WARN_S = float(_get_val("analysis.actuator_saturation.burst_warn_s", 0.2))

# VTOL Monitor
VTOL_STUCK_AFTER_S = float(_get_val("analysis.vtol.stuck_after_s", 8.0))
VTOL_STUCK_ALT_LOSS_M = float(_get_val("analysis.vtol.stuck_alt_loss_m", 3.0))
VTOL_DIP_WATCH_S = float(_get_val("analysis.vtol.dip_watch_s", 3.0))
VTOL_DIP_PITCH_DEG = float(_get_val("analysis.vtol.dip_pitch_deg", -10.0))
VTOL_DIP_ALT_LOSS_M = float(_get_val("analysis.vtol.dip_alt_loss_m", 2.0))
VTOL_HOVER_AIRSPEED_MAX = float(_get_val("analysis.vtol.hover_airspeed_max", 4.0))
VTOL_ELEVON_FIGHT_STD = float(_get_val("analysis.vtol.elevon_fight_std", 0.25))
VTOL_ALERT_COOLDOWN_S = float(_get_val("analysis.vtol.alert_cooldown_s", 30.0))

# Filter Advisor
FILTER_HF_BAND_HZ = float(_get_val("analysis.filter_advisor.hf_band_hz", 40.0))
FILTER_GYRO_NOISY_RMS = float(_get_val("analysis.filter_advisor.gyro_noisy_rms", 0.04))
FILTER_GYRO_CLEAN_RMS = float(_get_val("analysis.filter_advisor.gyro_clean_rms", 0.012))
FILTER_ACCEL_NOISY_RMS = float(_get_val("analysis.filter_advisor.accel_noisy_rms", 0.5))
FILTER_HARMONIC_TOL = float(_get_val("analysis.filter_advisor.harmonic_tol", 0.08))
FILTER_DELAY_RAISE_MS = float(_get_val("analysis.filter_advisor.delay_raise_ms", 8.0))

# Vibration Live
VIB_WARN = float(_get_val("analysis.vibration_live.vib_warn", 30.0))
VIB_CRIT = float(_get_val("analysis.vibration_live.vib_crit", 60.0))
VIB_CLIP_WINDOW_S = float(_get_val("analysis.vibration_live.clip_window_s", 30.0))
VIB_ALERT_COOLDOWN_S = float(_get_val("analysis.vibration_live.alert_cooldown_s", 30.0))
VIB_STALE_S = float(_get_val("analysis.vibration_live.stale_s", 2.0))
VIB_NEVER_STREAMED_S = float(_get_val("analysis.vibration_live.never_streamed_s", 10.0))

# EKF Offline
EKF_OFFLINE_MAX_LAG_S = float(_get_val("analysis.ekf_offline.max_lag_s", 0.5))
EKF_OFFLINE_HOVER_MIN_S = float(_get_val("analysis.ekf_offline.hover_min_s", 5.0))
EKF_OFFLINE_HOVER_GYRO_RMS = float(_get_val("analysis.ekf_offline.hover_gyro_rms", 0.06))

# Stick Monitor
STICK_RAIL_THRESHOLD = float(_get_val("analysis.stick_monitor.rail_threshold", 950.0))
STICK_RAIL_WINDOW_S = float(_get_val("analysis.stick_monitor.rail_window_s", 30.0))
STICK_RAIL_FRACTION = float(_get_val("analysis.stick_monitor.rail_fraction", 0.2))
STICK_RAIL_MIN_SAMPLES = int(_get_val("analysis.stick_monitor.rail_min_samples", 50))
STICK_VARIANCE_FLOOR = float(_get_val("analysis.stick_monitor.variance_floor", 25.0))
STICK_COMPLIANCE_PEAK = float(_get_val("analysis.stick_monitor.compliance_peak", 300.0))
STICK_COMPLIANCE_REVERSALS = int(_get_val("analysis.stick_monitor.compliance_reversals", 2))

# Live PID
PID_WINDOW_S = float(_get_val("analysis.live_pid.window_s", 3.0))
PID_EVAL_PERIOD_S = float(_get_val("analysis.live_pid.eval_period_s", 0.5))
PID_R_DEPLETION = float(_get_val("analysis.live_pid.r_depletion", 0.85))
PID_HIGH_RATE_RAD_S = float(_get_val("analysis.live_pid.high_rate_rad_s", 0.8))
PID_OVERSHOOT_LIMIT = float(_get_val("analysis.live_pid.overshoot_limit", 0.15))
PID_OSC_CROSSINGS = int(_get_val("analysis.live_pid.osc_crossings", 2))
PID_SETTLE_BAND = float(_get_val("analysis.live_pid.settle_band", 0.2))
PID_OFFSET_WINDOW_S = float(_get_val("analysis.live_pid.offset_window_s", 3.0))
PID_OFFSET_MIN_SPAN_S = float(_get_val("analysis.live_pid.offset_min_span_s", 2.0))
PID_OFFSET_DEG = float(_get_val("analysis.live_pid.offset_deg", 2.0))
PID_OFFSET_MAX_STD_DEG = float(_get_val("analysis.live_pid.offset_max_std_deg", 1.5))
PID_ASPD_SAMPLES = int(_get_val("analysis.live_pid.aspd_samples", 60))
PID_ASPD_MIN_PER_BIN = int(_get_val("analysis.live_pid.aspd_min_per_bin", 10))
PID_ASPD_RATIO = float(_get_val("analysis.live_pid.aspd_ratio", 1.8))
PID_ASPD_MIN_NRMSE = float(_get_val("analysis.live_pid.aspd_min_nrmse", 0.25))
PID_MIN_AMP = float(_get_val("analysis.live_pid.min_amp", 0.10))
PID_ALERT_COOLDOWN_S = float(_get_val("analysis.live_pid.alert_cooldown_s", 15.0))

# Cascade
CASCADE_ATTITUDE_MIN_AMP_DEG = float(_get_val("analysis.cascade.attitude_min_amp_deg", 5.0))
CASCADE_VELOCITY_MIN_AMP = float(_get_val("analysis.cascade.velocity_min_amp", 0.2))
CASCADE_POSITION_MIN_AMP = float(_get_val("analysis.cascade.position_min_amp", 0.3))

# Domains
DOMAINS_PWM_MIN = float(_get_val("analysis.domains.pwm_min", 1000.0))
DOMAINS_PWM_MID = float(_get_val("analysis.domains.pwm_mid", 1500.0))
DOMAINS_PWM_RANGE = float(_get_val("analysis.domains.pwm_range", 500.0))
DOMAINS_MOTOR_SAT = float(_get_val("analysis.domains.motor_sat", 0.98))
DOMAINS_SURFACE_RAIL = float(_get_val("analysis.domains.surface_rail", 0.96))
DOMAINS_SUSTAIN_WINDOW_S = float(_get_val("analysis.domains.sustain_window_s", 1.5))
DOMAINS_SUSTAIN_FRACTION = float(_get_val("analysis.domains.sustain_fraction", 0.6))
DOMAINS_AIRSPEED_REF_WINDOW_S = float(_get_val("analysis.domains.airspeed_ref_window_s", 30.0))
DOMAINS_ALERT_COOLDOWN_S = float(_get_val("analysis.domains.alert_cooldown_s", 15.0))
DOMAINS_BALANCE_WINDOW_S = float(_get_val("analysis.domains.balance_window_s", 4.0))
DOMAINS_BALANCE_MIN_SAMPLES = int(_get_val("analysis.domains.balance_min_samples", 20))
DOMAINS_BALANCE_WARN_FRAC = float(_get_val("analysis.domains.balance_warn_frac", 0.15))
DOMAINS_BALANCE_ALERT_COOLDOWN_S = float(_get_val("analysis.domains.balance_alert_cooldown_s", 60.0))

# Regime
REGIME_VARIANCE_WINDOW_S = float(_get_val("analysis.regime.variance_window_s", 2.0))
REGIME_DYNAMIC_VARIANCE = float(_get_val("analysis.regime.dynamic_variance", 15000.0))
REGIME_DWELL_S = float(_get_val("analysis.regime.dwell_s", 0.7))
REGIME_INFLIGHT_THROTTLE_PCT = float(_get_val("analysis.regime.inflight_throttle_pct", 12.0))
REGIME_INFLIGHT_SPEED_M_S = float(_get_val("analysis.regime.inflight_speed_m_s", 1.5))
REGIME_VFR_STALE_S = float(_get_val("analysis.regime.vfr_stale_s", 5.0))

# Connection watchdogs
CONN_WATCHDOG_PERIOD_S = float(_get_val("mavlink.connection.watchdog_period_s", 1.0))
CONN_STALE_MAX_AGE_S = float(_get_val("mavlink.connection.stale_max_age_s", 3.0))
CONN_PARAM_READ_ATTEMPTS = int(_get_val("mavlink.connection.param_read_attempts", 3))
CONN_PARAM_RETRY_BASE_S = float(_get_val("mavlink.connection.param_retry_base_s", 0.25))

# Parameter Advisor
