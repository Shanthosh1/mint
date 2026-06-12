"""
High-rate gyro FFT vibration analysis.

Uses Welch's method (scipy.signal.welch) on raw sensor_combined gyro
channels to estimate the power spectral density, then hunts for narrow
high-amplitude mechanical bands (motor/prop resonances) and recommends
a dynamic notch filter center frequency (IMU_GYRO_NF_FREQ) plus
bandwidth (IMU_GYRO_NF_BW).
"""
from __future__ import annotations

import numpy as np
import pandas as pd
from scipy import signal

# Peaks below this frequency are flight dynamics, not vibration.
_MIN_NOTCH_HZ = 25.0
# PSD peak must stand this many times above the broadband median to count.
_PEAK_PROMINENCE_RATIO = 8.0

_GYRO_AXES = {
    "x": "gyro_rad[0]",
    "y": "gyro_rad[1]",
    "z": "gyro_rad[2]",
}


def _sample_rate(df: pd.DataFrame) -> float:
    """Median sample rate from the microsecond timestamp column."""
    dt = np.diff(df["timestamp"].to_numpy(dtype=np.float64)) / 1e6
    dt = dt[(dt > 0) & (dt < 1.0)]
    return float(1.0 / np.median(dt)) if dt.size else 0.0


def analyze_vibration(sensor: pd.DataFrame) -> dict:
    """
    Per-axis PSD peak hunt + notch filter recommendation.

    Returns spectra decimated for plotting (<=512 bins per axis) and a
    recommendation block the param advisor can turn into a proposal.
    """
    fs = _sample_rate(sensor)
    if fs < 100:
        return {"skipped": f"Gyro sample rate too low for FFT analysis ({fs:.0f} Hz)"}

    axes_out: dict[str, dict] = {}

    for axis, col in _GYRO_AXES.items():
        if col not in sensor.columns:
            continue
        x = sensor[col].to_numpy(dtype=np.float64)
        x = x - np.mean(x)

        # Welch PSD: 2 s segments, 50% overlap — good resolution/variance
        # trade-off for multi-minute logs.
        nperseg = min(int(fs * 2), len(x))
        freqs, psd = signal.welch(x, fs=fs, nperseg=nperseg)

        psd_db = 10 * np.log10(psd + 1e-12)
        median_db = float(np.median(psd_db))

        # Restrict the peak hunt to the mechanical band.
        band = freqs >= _MIN_NOTCH_HZ
        peaks, props = signal.find_peaks(
            psd_db[band],
            height=median_db + 10 * np.log10(_PEAK_PROMINENCE_RATIO),
            distance=max(1, int(5 / (freqs[1] - freqs[0]))),  # >=5 Hz apart
        )
        band_freqs = freqs[band]
        axis_peaks = [
            {"freq_hz": round(float(band_freqs[i]), 1),
             "power_db": round(float(props["peak_heights"][k]), 1)}
            for k, i in enumerate(peaks)
        ]

        # Decimate spectrum for the UI plot.
        step = max(1, len(freqs) // 512)
        axes_out[axis] = {
            "freqs_hz": np.round(freqs[::step], 1).tolist(),
            "psd_db": np.round(psd_db[::step], 1).tolist(),
            "peaks": axis_peaks,
        }

    # Filter-parameter advice (notches, cutoffs, dynamic notch) is owned
    # by filter_advisor.py, which consumes these spectra — one module
    # decides the coordinated plan instead of piecemeal suggestions.
    return {
        "sample_rate_hz": round(fs, 1),
        "axes": axes_out,
    }
