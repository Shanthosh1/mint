"""
Offline EKF optimization engine.

Two analyses over a parsed ULog:

1. analyze_delays() — cross-correlates the IMU-derived vertical
   acceleration timeline against GPS vertical velocity changes to
   estimate the real physical GPS propagation delay, then advises a
   correction for EKF2_GPS_DELAY (ms).

2. analyze_hover_noise() — finds quiet flat-hover segments and measures
   baseline accel/gyro variance there, advising tailored values for
   EKF2_ACC_NOISE and EKF2_GYR_NOISE process noise.
"""
from __future__ import annotations

import numpy as np
import pandas as pd
from scipy import signal

from ..core import config

_MAX_LAG_S = config.EKF_OFFLINE_MAX_LAG_S
_HOVER_MIN_S = config.EKF_OFFLINE_HOVER_MIN_S
_HOVER_GYRO_RMS = config.EKF_OFFLINE_HOVER_GYRO_RMS


def _resample_uniform(t_us: np.ndarray, x: np.ndarray, fs: float
                      ) -> tuple[np.ndarray, np.ndarray]:
    """Resample an irregular series onto a uniform grid (for xcorr)."""
    t = t_us / 1e6
    grid = np.arange(t[0], t[-1], 1.0 / fs)
    return grid, np.interp(grid, t, x)


def analyze_delays(sensor: pd.DataFrame | None,
                   gps: pd.DataFrame | None,
                   local_pos: pd.DataFrame | None,
                   current_params: dict,
                   air_data: pd.DataFrame | None = None,
                   baro: pd.DataFrame | None = None) -> dict:
    """Estimate true GPS + baro latency via cross-correlation against IMU."""
    out = _gps_delay(sensor, gps, current_params)
    out["baro"] = _baro_delay(sensor, air_data, baro, current_params)
    return out


def _gps_delay(sensor: pd.DataFrame | None,
               gps: pd.DataFrame | None,
               current_params: dict) -> dict:
    """GPS latency: vertical accel vs differentiated GPS vertical velocity."""
    if sensor is None or gps is None:
        return {"skipped": "sensor_combined and vehicle_gps_position both required"}

    az_col = "accelerometer_m_s2[2]"
    vel_col = "vel_d_m_s" if "vel_d_m_s" in gps.columns else None
    if az_col not in sensor.columns or vel_col is None:
        return {"skipped": "required accel/GPS velocity columns missing"}

    fs = 50.0  # common grid; GPS is ~5-10 Hz, accel far higher
    t_a, az = _resample_uniform(sensor["timestamp"].to_numpy(np.float64),
                                sensor[az_col].to_numpy(np.float64), fs)
    t_g, vd = _resample_uniform(gps["timestamp"].to_numpy(np.float64),
                                gps[vel_col].to_numpy(np.float64), fs)

    # Differentiate GPS vertical velocity -> acceleration proxy, then
    # overlap the two grids.
    ad_gps = np.gradient(vd, 1.0 / fs)
    t0, t1 = max(t_a[0], t_g[0]), min(t_a[-1], t_g[-1])
    if t1 - t0 < 20:
        return {"skipped": "insufficient overlapping flight time (<20 s)"}
    a = az[(t_a >= t0) & (t_a <= t1)]
    g = ad_gps[(t_g >= t0) & (t_g <= t1)]
    n = min(len(a), len(g))
    a, g = signal.detrend(a[:n]), signal.detrend(g[:n])

    # correlate(a, g) peaks at *negative* lag when g lags a (scipy's
    # convention: lag k aligns a[n+k] with g[n]), so the sensor delay
    # is the negated peak location.
    corr = signal.correlate(a, g, mode="full")
    lags = signal.correlation_lags(n, n, mode="full") / fs
    window = np.abs(lags) <= _MAX_LAG_S
    best_lag_s = float(lags[window][np.argmax(corr[window])])
    measured_delay_ms = max(0.0, -best_lag_s * 1000.0)

    current = float(current_params.get("EKF2_GPS_DELAY", 110.0))
    out = {
        "measured_gps_delay_ms": round(measured_delay_ms, 1),
        "current_EKF2_GPS_DELAY": current,
        "recommendation": None,
    }
    if abs(measured_delay_ms - current) > 20.0:
        out["recommendation"] = {
            "param": "EKF2_GPS_DELAY",
            "proposed_value": round(measured_delay_ms, 0),
            "rationale": (
                f"Cross-correlation places true GPS latency at "
                f"~{measured_delay_ms:.0f} ms vs configured {current:.0f} ms. "
                f"Aligning EKF2_GPS_DELAY tightens velocity fusion."
            ),
        }
    return out


def _baro_delay(sensor: pd.DataFrame | None,
                air_data: pd.DataFrame | None,
                baro: pd.DataFrame | None,
                current_params: dict) -> dict:
    """
    Baro latency via vertical-velocity cross-correlation.

    Comparison happens in the velocity domain: IMU accel integrated once
    vs baro altitude differentiated once. Each signal is one derivative
    from its source, so baro noise is amplified by fs (not fs^2 as a
    double derivative would) and the correlation peak stays sharp.

    Prefers vehicle_air_data.baro_alt_meter; falls back to converting
    sensor_baro pressure to altitude (ISA approximation).
    """
    az_col = "accelerometer_m_s2[2]"
    if sensor is None or az_col not in sensor.columns:
        return {"skipped": "sensor_combined accel required"}

    if air_data is not None and "baro_alt_meter" in air_data.columns:
        t_b_us = air_data["timestamp"].to_numpy(np.float64)
        alt = air_data["baro_alt_meter"].to_numpy(np.float64)
    elif baro is not None and "pressure" in baro.columns:
        t_b_us = baro["timestamp"].to_numpy(np.float64)
        p = baro["pressure"].to_numpy(np.float64)
        alt = 44330.0 * (1.0 - (p / p[0]) ** (1.0 / 5.255))  # ISA approx
    else:
        return {"skipped": "no barometric altitude source in log"}

    fs = 50.0
    t_a, az = _resample_uniform(sensor["timestamp"].to_numpy(np.float64),
                                sensor[az_col].to_numpy(np.float64), fs)
    t_b, alt_u = _resample_uniform(t_b_us, alt, fs)

    t0, t1 = max(t_a[0], t_b[0]), min(t_a[-1], t_b[-1])
    if t1 - t0 < 20:
        return {"skipped": "insufficient overlapping flight time (<20 s)"}
    az_o = az[(t_a >= t0) & (t_a <= t1)]
    alt_o = alt_u[(t_b >= t0) & (t_b <= t1)]
    n = min(len(az_o), len(alt_o))

    # IMU vertical velocity: accelerometer z is FRD (down +), so "up"
    # acceleration is -az; integrate, then detrend to kill bias drift.
    v_imu = np.cumsum(signal.detrend(-az_o[:n])) / fs
    # Baro vertical velocity: light smoothing, single derivative.
    k = max(1, int(0.2 * fs))
    alt_s = np.convolve(alt_o[:n], np.ones(k) / k, mode="same")
    v_baro = np.gradient(alt_s, 1.0 / fs)

    a, b = signal.detrend(v_imu), signal.detrend(v_baro)

    # Same lag convention as the GPS path: baro lagging the IMU shows
    # up as a negative-lag peak, so negate to get the physical delay.
    corr = signal.correlate(a, b, mode="full")
    lags = signal.correlation_lags(len(a), len(b), mode="full") / fs
    window = np.abs(lags) <= _MAX_LAG_S
    best_lag_s = float(lags[window][np.argmax(corr[window])])
    measured_ms = max(0.0, -best_lag_s * 1000.0)

    current = float(current_params.get("EKF2_BARO_DELAY", 0.0))
    out = {
        "measured_baro_delay_ms": round(measured_ms, 1),
        "current_EKF2_BARO_DELAY": current,
        "recommendation": None,
    }
    if abs(measured_ms - current) > 15.0:
        out["recommendation"] = {
            "param": "EKF2_BARO_DELAY",
            "proposed_value": round(measured_ms, 0),
            "rationale": (
                f"Cross-correlation places baro latency at ~{measured_ms:.0f} ms "
                f"vs configured {current:.0f} ms. Aligning EKF2_BARO_DELAY "
                f"removes height-innovation phase lag during climbs."
            ),
        }
    return out


def analyze_hover_noise(sensor: pd.DataFrame,
                        local_pos: pd.DataFrame | None,
                        current_params: dict) -> dict:
    """Baseline sensor variance during flat hover -> process noise advice."""
    gyro_cols = [f"gyro_rad[{i}]" for i in range(3)]
    acc_cols = [f"accelerometer_m_s2[{i}]" for i in range(3)]
    if not all(c in sensor.columns for c in gyro_cols + acc_cols):
        return {"skipped": "gyro/accel columns missing from sensor_combined"}

    t = sensor["timestamp"].to_numpy(np.float64) / 1e6
    gyro_mag = np.linalg.norm(sensor[gyro_cols].to_numpy(np.float64), axis=1)

    # 1 s rolling RMS of gyro magnitude marks "quiet" flight.
    fs = 1.0 / np.median(np.diff(t))
    win = max(1, int(fs))
    quiet = (pd.Series(gyro_mag).rolling(win, center=True).std()
             .to_numpy() < _HOVER_GYRO_RMS)

    segs = _segments(t, quiet, _HOVER_MIN_S)
    if not segs:
        return {"skipped": "no flat-hover segment >= 5 s found in log"}

    # Measure standard deviations inside the longest quiet segment.
    s0, s1 = max(segs, key=lambda s: s[1] - s[0])
    mask = (t >= s0) & (t <= s1)
    acc_std = float(np.mean(sensor.loc[mask, acc_cols].std()))
    gyr_std = float(np.mean(sensor.loc[mask, gyro_cols].std()))

    recs = []
    cur_acc = float(current_params.get("EKF2_ACC_NOISE", 0.35))
    cur_gyr = float(current_params.get("EKF2_GYR_NOISE", 0.015))
    # Heuristic: process noise should sit modestly above measured floor.
    tgt_acc, tgt_gyr = round(acc_std * 1.5, 3), round(gyr_std * 1.5, 4)
    if abs(tgt_acc - cur_acc) / cur_acc > 0.3:
        recs.append({"param": "EKF2_ACC_NOISE", "proposed_value": tgt_acc,
                     "rationale": f"Hover accel noise floor {acc_std:.3f} m/s² "
                                  f"vs configured {cur_acc}."})
    if abs(tgt_gyr - cur_gyr) / cur_gyr > 0.3:
        recs.append({"param": "EKF2_GYR_NOISE", "proposed_value": tgt_gyr,
                     "rationale": f"Hover gyro noise floor {gyr_std:.4f} rad/s "
                                  f"vs configured {cur_gyr}."})

    return {
        "hover_segment_s": [round(s0, 1), round(s1, 1)],
        "accel_std_m_s2": round(acc_std, 4),
        "gyro_std_rad_s": round(gyr_std, 5),
        "recommendations": recs,
    }


def _segments(t: np.ndarray, mask: np.ndarray, min_len_s: float
              ) -> list[tuple[float, float]]:
    """Contiguous True runs in `mask` lasting at least `min_len_s`."""
    mask = np.nan_to_num(mask.astype(np.float64)).astype(bool)
    if not mask.any():
        return []
    edges = np.diff(mask.astype(np.int8))
    starts = np.flatnonzero(edges == 1) + 1
    ends = np.flatnonzero(edges == -1) + 1
    if mask[0]:
        starts = np.r_[0, starts]
    if mask[-1]:
        ends = np.r_[ends, len(mask)]
    return [(float(t[a]), float(t[min(b, len(t) - 1)]))
            for a, b in zip(starts, ends)
            if t[min(b, len(t) - 1)] - t[a] >= min_len_s]
