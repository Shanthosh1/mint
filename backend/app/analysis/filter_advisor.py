"""
Offline filter tuning advisor.

Owns ALL filter-parameter advice (the vibration module supplies spectra
and peaks; this module turns them into a *coordinated* filter plan —
piecemeal filter changes interact, so one module decides):

1. Notch placement — strongest spectral peak(s) -> static notch(es).
   Version-aware: firmware that exposes IMU_GYRO_NF0_FRQ (PX4 >= 1.13)
   gets NF0/NF1 names and can take two notches; older firmware gets the
   legacy IMU_GYRO_NF_FREQ/NF_BW pair.

2. Dynamic notch detection — peaks sitting at integer harmonic ratios
   are motor/prop blade-pass harmonics that move with throttle; a static
   notch can't follow them. Recommends IMU_GYRO_DNF_EN (requires ESC
   RPM telemetry, flagged in the rationale) with a harmonic count.

3. Low-pass cutoffs — the noise-vs-latency budget:
     * noisy spectrum (high broadband floor above the control band)
       -> lower IMU_GYRO_CUTOFF and especially IMU_DGYRO_CUTOFF (the
       D-term is the most noise-sensitive path in the controller);
     * clean spectrum AND measurable filter-induced delay -> raise
       cutoffs to buy back control latency.

4. Measured filter delay — cross-correlates raw gyro (sensor_combined)
   against the filtered rates the controller actually consumes
   (vehicle_angular_velocity). This attributes how much of the rate
   loop's tau is filter lag rather than gains — "sluggish, raise P"
   advice is wrong when the gyro path is over-filtered.

5. Accel cutoff — same noise logic on the accelerometer spectrum.
"""
from __future__ import annotations

import numpy as np
import pandas as pd
from scipy import signal

from .ekf_offline import _resample_uniform

# Gyro broadband HF noise (rms, rad/s, integrated above _HF_BAND_HZ).
_HF_BAND_HZ = 40.0
_GYRO_NOISY_RMS = 0.04
_GYRO_CLEAN_RMS = 0.012
_ACCEL_NOISY_RMS = 0.5        # m/s^2
_HARMONIC_TOL = 0.08          # +/-8% counts as an integer harmonic
_DELAY_RAISE_MS = 8.0         # measured filter lag worth buying back

_DEFAULTS = {
    "IMU_GYRO_CUTOFF": 40.0,
    "IMU_DGYRO_CUTOFF": 30.0,
    "IMU_ACCEL_CUTOFF": 30.0,
}


# ---------------------------------------------------------------------- #
# helpers
# ---------------------------------------------------------------------- #
def _hf_rms(freqs: list[float], psd_db: list[float],
            above_hz: float = _HF_BAND_HZ) -> float | None:
    """Integrate a (decimated) PSD above `above_hz` -> rms amplitude."""
    f = np.asarray(freqs, dtype=np.float64)
    if len(f) < 4 or f[-1] <= above_hz:
        return None
    df = float(f[1] - f[0])
    lin = 10.0 ** (np.asarray(psd_db, dtype=np.float64) / 10.0)
    band = f >= above_hz
    return float(np.sqrt(np.sum(lin[band]) * df))


def _merge_peaks(vib_axes: dict) -> list[dict]:
    """Collect per-axis peaks, merge near-duplicates (<5 Hz apart),
    return sorted by power descending."""
    raw = [p for ax in vib_axes.values() for p in ax.get("peaks", [])]
    raw.sort(key=lambda p: -p["power_db"])
    merged: list[dict] = []
    for p in raw:
        if all(abs(p["freq_hz"] - m["freq_hz"]) >= 5.0 for m in merged):
            merged.append(p)
    return merged


def _harmonic_count(peaks: list[dict]) -> tuple[float | None, int]:
    """Fundamental frequency + how many integer harmonics of it appear."""
    if not peaks:
        return None, 0
    f0 = min(p["freq_hz"] for p in peaks)
    count = 0
    for p in peaks:
        ratio = p["freq_hz"] / f0
        k = round(ratio)
        if k >= 2 and abs(ratio - k) <= _HARMONIC_TOL * k:
            count += 1
    return f0, count


def measure_filter_delay_ms(sensor: pd.DataFrame | None,
                            ang_vel: pd.DataFrame | None) -> float | None:
    """Lag of the filtered rate signal behind the raw gyro (ms)."""
    if sensor is None or ang_vel is None:
        return None
    if "gyro_rad[0]" not in sensor.columns or "xyz[0]" not in ang_vel.columns:
        return None
    fs = 200.0
    t_r, raw = _resample_uniform(sensor["timestamp"].to_numpy(np.float64),
                                 sensor["gyro_rad[0]"].to_numpy(np.float64), fs)
    t_f, flt = _resample_uniform(ang_vel["timestamp"].to_numpy(np.float64),
                                 ang_vel["xyz[0]"].to_numpy(np.float64), fs)
    t0, t1 = max(t_r[0], t_f[0]), min(t_r[-1], t_f[-1])
    if t1 - t0 < 10:
        return None
    a = raw[(t_r >= t0) & (t_r <= t1)]
    b = flt[(t_f >= t0) & (t_f <= t1)]
    n = min(len(a), len(b))
    a, b = signal.detrend(a[:n]), signal.detrend(b[:n])

    corr = signal.correlate(a, b, mode="full")
    lags = signal.correlation_lags(n, n, mode="full") / fs
    window = np.abs(lags) <= 0.1
    # Filtered lags raw -> negative-lag peak (same convention as
    # ekf_offline); negate for the physical delay.
    best = float(lags[window][np.argmax(corr[window])])
    return round(max(0.0, -best * 1000.0), 1)


# ---------------------------------------------------------------------- #
def advise_filters(vibration: dict,
                   sensor: pd.DataFrame | None,
                   ang_vel: pd.DataFrame | None,
                   params: dict) -> dict:
    """Build the coordinated filter plan. `vibration` is the output of
    vibration_fft.analyze_vibration (spectra + peaks per axis)."""
    if vibration.get("skipped"):
        return {"skipped": f"vibration analysis unavailable: {vibration['skipped']}"}

    recs: list[dict] = []
    notes: list[str] = []
    new_notch_names = "IMU_GYRO_NF0_FRQ" in params

    # ---- noise floor --------------------------------------------------
    axes = vibration.get("axes", {})
    rms_vals = [r for r in (
        _hf_rms(a["freqs_hz"], a["psd_db"]) for a in axes.values()
    ) if r is not None]
    gyro_hf_rms = max(rms_vals) if rms_vals else None
    noisy = gyro_hf_rms is not None and gyro_hf_rms > _GYRO_NOISY_RMS
    clean = gyro_hf_rms is not None and gyro_hf_rms < _GYRO_CLEAN_RMS

    # ---- filter-induced latency ----------------------------------------
    delay_ms = measure_filter_delay_ms(sensor, ang_vel)

    # ---- 1+2) notches & dynamic notch ----------------------------------
    peaks = _merge_peaks(axes)
    f0, harmonics = _harmonic_count(peaks)
    if peaks:
        nf_freq = "IMU_GYRO_NF0_FRQ" if new_notch_names else "IMU_GYRO_NF_FREQ"
        nf_bw = "IMU_GYRO_NF0_BW" if new_notch_names else "IMU_GYRO_NF_BW"
        top = peaks[0]
        recs.append({
            "param": nf_freq, "proposed_value": top["freq_hz"],
            "rationale": f"Dominant vibration peak at {top['freq_hz']} Hz "
                         f"({top['power_db']:.0f} dB). Center the static notch here.",
            "companion": {"param": nf_bw,
                          "proposed_value": round(max(10.0, top["freq_hz"] * 0.2), 1)},
        })
        if new_notch_names and len(peaks) > 1 and harmonics == 0:
            second = peaks[1]
            recs.append({
                "param": "IMU_GYRO_NF1_FRQ", "proposed_value": second["freq_hz"],
                "rationale": f"Second independent peak at {second['freq_hz']} Hz "
                             f"— firmware supports a second static notch.",
                "companion": {"param": "IMU_GYRO_NF1_BW",
                              "proposed_value": round(max(10.0, second["freq_hz"] * 0.2), 1)},
            })
        elif len(peaks) > 1 and not new_notch_names:
            notes.append("Multiple vibration peaks found but this firmware has a "
                         "single static notch — consider upgrading PX4 (NF0/NF1) "
                         "or fixing the mechanical source.")

    if harmonics >= 1 and f0 is not None:
        recs.append({
            "param": "IMU_GYRO_DNF_EN", "proposed_value": 1,
            "rationale": f"Peaks at integer multiples of {f0:.0f} Hz "
                         f"({harmonics} harmonic(s)) — blade-pass harmonics move "
                         f"with throttle, which a static notch cannot follow. "
                         f"Requires ESC RPM telemetry (DShot).",
            "companion": {"param": "IMU_GYRO_DNF_HMC",
                          "proposed_value": min(3, harmonics + 1)},
        })

    # ---- 3) low-pass cutoffs -------------------------------------------
    gyro_cut = float(params.get("IMU_GYRO_CUTOFF", _DEFAULTS["IMU_GYRO_CUTOFF"]))
    dgyro_cut = float(params.get("IMU_DGYRO_CUTOFF", _DEFAULTS["IMU_DGYRO_CUTOFF"]))
    if noisy:
        if gyro_cut > 30.0:
            recs.append({
                "param": "IMU_GYRO_CUTOFF", "proposed_value": 30.0,
                "rationale": f"Broadband gyro noise above {_HF_BAND_HZ:.0f} Hz is "
                             f"{gyro_hf_rms:.3f} rad/s rms — lower the gyro LPF "
                             f"until the mechanical source is fixed.",
            })
        if dgyro_cut > 20.0:
            recs.append({
                "param": "IMU_DGYRO_CUTOFF", "proposed_value": 20.0,
                "rationale": "The D-term differentiates this noise straight into "
                             "the motors (heat, oscillation). Lower its dedicated "
                             "cutoff first — it costs less latency than the main LPF.",
            })
        notes.append("Spectrum is NOISY: place the notch(es), re-fly, and only "
                     "then consider raising any gains. Do not raise rate D on "
                     "this airframe yet.")
    elif clean and delay_ms is not None and delay_ms > _DELAY_RAISE_MS:
        if gyro_cut < 80.0:
            recs.append({
                "param": "IMU_GYRO_CUTOFF", "proposed_value": round(gyro_cut + 10, 0),
                "rationale": f"Spectrum is clean ({gyro_hf_rms:.3f} rad/s rms) but "
                             f"the filter chain adds {delay_ms:.0f} ms of measured "
                             f"lag — raise the cutoff to buy back control latency.",
            })
        if dgyro_cut < 40.0:
            recs.append({
                "param": "IMU_DGYRO_CUTOFF", "proposed_value": round(dgyro_cut + 10, 0),
                "rationale": "Clean spectrum: the D-term path can tolerate a higher "
                             "cutoff, sharpening derivative response.",
            })
    elif clean:
        notes.append("Spectrum clean and filter lag "
                     f"{'unmeasured' if delay_ms is None else f'{delay_ms:.0f} ms'} — "
                     "current low-pass settings are appropriate.")

    # ---- 5) accel cutoff -------------------------------------------------
    accel_rms = _accel_hf_rms(sensor)
    if accel_rms is not None and accel_rms > _ACCEL_NOISY_RMS:
        accel_cut = float(params.get("IMU_ACCEL_CUTOFF", _DEFAULTS["IMU_ACCEL_CUTOFF"]))
        if accel_cut > 20.0:
            recs.append({
                "param": "IMU_ACCEL_CUTOFF", "proposed_value": 20.0,
                "rationale": f"Accelerometer HF noise {accel_rms:.2f} m/s² rms feeds "
                             f"the EKF and position controller — lower the accel LPF.",
            })

    return {
        "gyro_hf_rms_rad_s": round(gyro_hf_rms, 4) if gyro_hf_rms is not None else None,
        "accel_hf_rms_m_s2": round(accel_rms, 3) if accel_rms is not None else None,
        "filter_delay_ms": delay_ms,
        "spectrum_class": "noisy" if noisy else "clean" if clean else "moderate",
        "notch_naming": "NF0/NF1" if new_notch_names else "legacy NF",
        "harmonics_of_hz": round(f0, 1) if harmonics and f0 else None,
        "recommendations": recs,
        "notes": notes,
    }


def _accel_hf_rms(sensor: pd.DataFrame | None) -> float | None:
    """Worst-axis accelerometer rms above the HF band (Welch)."""
    if sensor is None:
        return None
    cols = [f"accelerometer_m_s2[{i}]" for i in range(3)]
    if not all(c in sensor.columns for c in cols):
        return None
    t = sensor["timestamp"].to_numpy(np.float64) / 1e6
    dt = np.diff(t)
    dt = dt[(dt > 0) & (dt < 1.0)]
    if dt.size == 0:
        return None
    fs = 1.0 / float(np.median(dt))
    if fs < 2 * _HF_BAND_HZ:
        return None
    worst = 0.0
    for c in cols:
        x = sensor[c].to_numpy(np.float64)
        x = x - np.mean(x)
        freqs, psd = signal.welch(x, fs=fs, nperseg=min(int(fs * 2), len(x)))
        band = freqs >= _HF_BAND_HZ
        worst = max(worst, float(np.sqrt(np.trapezoid(psd[band], freqs[band]))))
    return worst
