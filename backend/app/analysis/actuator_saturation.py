"""
Actuator deflection saturation analysis (actuator_outputs / actuator_controls_0).

Control outputs pinned at maximum mean the controller is demanding more
authority than the airframe has — a classic symptom of gains too high,
excessive vibration feeding the D-term, or an underpowered vehicle.
"""
from __future__ import annotations

import numpy as np
import pandas as pd

from ..core import config

_SAT_EPS = config.SAT_EPS
_BURST_WARN_S = config.BURST_WARN_S
_PWM_MIN = config.DOMAINS_PWM_MIN
_PWM_MID = config.DOMAINS_PWM_MID
_PWM_RANGE = config.DOMAINS_PWM_RANGE


def _get_channel_limits(params: dict | None, ch: int) -> tuple[float, float, float]:
    """Get min, max, trim limits for a channel (0-indexed).

    Returns values in the same native scale as actuator_outputs in the ULog —
    µs for real hardware (PWM/ACT_FUNC), raw Gazebo units for SIM_GZ.
    """
    p_min = _PWM_MIN
    p_max = _PWM_MIN + _PWM_RANGE * 2
    p_trim = _PWM_MID

    if not params:
        return p_min, p_max, p_trim

    # SIM_GZ: parameters and actuator_outputs share the same native Gazebo
    # scale (0–MAX), no µs offset applied.
    if "SIM_GZ_EC_FUNC1" in params:
        num_esc = 0
        for i in range(1, 9):
            if params.get(f"SIM_GZ_EC_FUNC{i}", 0):
                num_esc = max(num_esc, i)
        if ch < num_esc:
            idx = ch + 1
            p_min = float(params.get(f"SIM_GZ_EC_MIN{idx}", 0.0))
            p_max = float(params.get(f"SIM_GZ_EC_MAX{idx}", 1000.0))
            p_trim = float(params.get(f"SIM_GZ_EC_DIS{idx}", 0.0))
        else:
            idx = ch - num_esc + 1
            p_min = float(params.get(f"SIM_GZ_SV_MIN{idx}", 0.0))
            p_max = float(params.get(f"SIM_GZ_SV_MAX{idx}", 1000.0))
            p_trim = float(params.get(f"SIM_GZ_SV_DIS{idx}", 500.0))
        return p_min, p_max, p_trim

    ch_1 = ch + 1

    # ACT_FUNC (newer PX4 centralized outputs) uses OUT{n}_MIN/MAX/TRIM.
    # Check this first so it takes priority over PWM_MAIN on shared channels.
    if "ACT_FUNC1" in params:
        out_min = params.get(f"OUT{ch_1}_MIN")
        out_max = params.get(f"OUT{ch_1}_MAX")
        out_trim = params.get(f"OUT{ch_1}_TRIM")
        if out_min is not None:
            p_min = float(out_min)
        if out_max is not None:
            p_max = float(out_max)
        if out_trim is not None:
            p_trim = float(out_trim)
        return p_min, p_max, p_trim

    # PWM / HIL: main outputs on channels 0–7, aux on 8–15.
    if ch < 8:
        min_keys = [f"PWM_MAIN_MIN{ch_1}"]
        max_keys = [f"PWM_MAIN_MAX{ch_1}"]
        trim_keys = [f"PWM_MAIN_TRIM{ch_1}"]
    else:
        ch_aux = ch - 7
        min_keys = [f"PWM_AUX_MIN{ch_aux}"]
        max_keys = [f"PWM_AUX_MAX{ch_aux}"]
        trim_keys = [f"PWM_AUX_TRIM{ch_aux}"]

    # HIL fallback: OUT{n}_MIN/MAX/TRIM mirrors ACT_FUNC naming.
    min_keys.append(f"OUT{ch_1}_MIN")
    max_keys.append(f"OUT{ch_1}_MAX")
    trim_keys.append(f"OUT{ch_1}_TRIM")

    for k in min_keys:
        if k in params:
            p_min = float(params[k])
            break
    for k in max_keys:
        if k in params:
            p_max = float(params[k])
            break
    for k in trim_keys:
        if k in params:
            p_trim = float(params[k])
            break

    return p_min, p_max, p_trim


def analyze_saturation(actuators: pd.DataFrame, actuator_map: dict | None = None,
                       airframe_class: str | None = None, is_physical: bool = False,
                       params: dict | None = None, controls: pd.DataFrame | None = None,
                       airspeed_data: tuple[np.ndarray, np.ndarray] | None = None) -> dict:
    """Analyze actuator outputs or control setpoints for saturation."""
    if actuators is None or actuators.empty or "timestamp" not in actuators.columns:
        return {"channels": {}, "advice": None, "t": []}

    ts = actuators["timestamp"].to_numpy(dtype=np.float64) / 1e6
    out: dict[str, dict] = {}
    worst: tuple[str, float] | None = None

    # Calculate decimation factor for 10 Hz time-series
    duration = ts[-1] - ts[0] if ts.size > 1 else 1.0
    target_len = int(max(10, duration * 10.0))
    k = max(1, len(ts) // target_len)
    ts_decimated = ts[::k] - ts[0]

    # Align controls to ts if available
    controls_aligned = {}
    if controls is not None and not controls.empty and "timestamp" in controls.columns:
        ctrl_ts = controls["timestamp"].to_numpy(dtype=np.float64) / 1e6
        for axis_idx, axis_name in [(0, "roll"), (1, "pitch"), (2, "yaw"), (3, "thrust")]:
            col = f"control[{axis_idx}]"
            if col in controls.columns:
                ctrl_val = controls[col].to_numpy(dtype=np.float64)
                if ctrl_val.size > 0:
                    controls_aligned[axis_name] = np.interp(ts, ctrl_ts, ctrl_val)

    if is_physical and actuator_map:
        _CS_TYPES = {
            1: "Left Aileron",
            2: "Right Aileron",
            3: "Elevator",
            4: "Rudder",
            5: "Left Elevon",
            6: "Right Elevon",
            7: "Left V-Tail",
            8: "Right V-Tail",
            9: "Left Flap",
            10: "Right Flap",
            11: "Airbrake",
            12: "Custom Surface",
            13: "Left A-Tail",
            14: "Right A-Tail",
            15: "Single Channel Aileron",
            16: "Steering Wheel",
            17: "Left Spoiler",
            18: "Right Spoiler",
        }

        # Build dynamic labels based on parameters and logical servo ordering
        dynamic_labels = {}
        if params:
            selected_family = None
            if "ACT_FUNC1" in params:
                selected_family = "ACT_FUNC"
            elif "SIM_GZ_EC_FUNC1" in params:
                selected_family = "SIM_GZ"
            elif "HIL_ACT_FUNC1" in params:
                selected_family = "HIL"
            elif "PWM_MAIN_FUNC1" in params:
                selected_family = "PWM"

            servo_to_channel = {}
            if selected_family:
                if selected_family == "ACT_FUNC":
                    for c in range(16):
                        val = params.get(f"ACT_FUNC{c + 1}")
                        if val is not None and int(float(val)) > 0:
                            func_val = int(float(val))
                            if 201 <= func_val <= 208:
                                servo_to_channel[func_val - 201] = c
                elif selected_family == "HIL":
                    for c in range(16):
                        val = params.get(f"HIL_ACT_FUNC{c + 1}")
                        if val is not None and int(float(val)) > 0:
                            func_val = int(float(val))
                            if 201 <= func_val <= 208:
                                servo_to_channel[func_val - 201] = c
                elif selected_family == "PWM":
                    for c in range(8):
                        val = params.get(f"PWM_MAIN_FUNC{c + 1}")
                        if val is not None and int(float(val)) > 0:
                            func_val = int(float(val))
                            if 201 <= func_val <= 208:
                                servo_to_channel[func_val - 201] = c
                    for c in range(8):
                        val = params.get(f"PWM_AUX_FUNC{c + 1}")
                        if val is not None and int(float(val)) > 0:
                            func_val = int(float(val))
                            if 201 <= func_val <= 208:
                                servo_to_channel[func_val - 201] = 8 + c
                elif selected_family == "SIM_GZ":
                    num_esc = 0
                    esc_values = {}
                    for i in range(1, 9):
                        val = params.get(f"SIM_GZ_EC_FUNC{i}")
                        if val is not None and int(float(val)) > 0:
                            num_esc = max(num_esc, i)
                            esc_values[i] = int(float(val))
                    for j in range(1, 9):
                        val = params.get(f"SIM_GZ_SV_FUNC{j}")
                        if val is not None and int(float(val)) > 0:
                            func_val = int(float(val))
                            if 201 <= func_val <= 208:
                                servo_to_channel[func_val - 201] = num_esc + j - 1

            # Map each active servo output directly based on its logical servo index parameter
            if servo_to_channel:
                cs_count = int(float(params.get("CA_SV_CS_COUNT", 0)))
                if cs_count == 0:
                    cs_count = sum(1 for i in range(8) if int(float(params.get(f"CA_SV_CS{i}_TYPE", 0))) > 0)
                
                for servo_idx, ch in servo_to_channel.items():
                    if cs_count > 0:
                        if servo_idx < cs_count:
                            cs_type_val = int(float(params.get(f"CA_SV_CS{servo_idx}_TYPE", 0)))
                            if cs_type_val > 0:
                                cs_name = _CS_TYPES.get(cs_type_val, f"CS{servo_idx + 1}")
                                dynamic_labels[ch] = f"{cs_name} (Ch{ch + 1})"
                            else:
                                dynamic_labels[ch] = f"S{servo_idx + 1} (Ch{ch + 1})"
                        else:
                            tilt_idx = sum(1 for s_idx in servo_to_channel if s_idx < servo_idx and s_idx >= cs_count)
                            dynamic_labels[ch] = f"Tilt Servo {tilt_idx + 1} (Ch{ch + 1})"
                    else:
                        cs_type_val = int(float(params.get(f"CA_SV_CS{servo_idx}_TYPE", 0)))
                        if cs_type_val > 0:
                            cs_name = _CS_TYPES.get(cs_type_val, f"CS{servo_idx + 1}")
                            dynamic_labels[ch] = f"{cs_name} (Ch{ch + 1})"
                        else:
                            # Count how many tilt servos exist at or before this logical servo index
                            tilt_idx = sum(1 for s_idx in servo_to_channel if s_idx < servo_idx and int(float(params.get(f"CA_SV_CS{s_idx}_TYPE", 0))) == 0)
                            dynamic_labels[ch] = f"Tilt Servo {tilt_idx + 1} (Ch{ch + 1})"

        roles = [
            ("hover_motors", "M", True),
            ("thrust_motors", "P", True),
            ("control_surfaces", "S", False),
            ("tilt_servos", "Tilt Servo ", False),
        ]
        
        for group_name, prefix, is_motor in roles:
            channels = actuator_map.get(group_name, [])
            for idx, ch in enumerate(channels):
                col = f"output[{ch}]"
                if col not in actuators.columns:
                    continue
                
                u = actuators[col].to_numpy(dtype=np.float64)
                if u.size == 0:
                    continue
                
                # Fetch per-channel limits
                p_min, p_max, p_trim = _get_channel_limits(params, ch)
                
                # Auto-detect raw PWM vs normalized values
                is_raw_pwm = np.any(u > 200.0)
                if is_raw_pwm:
                    if is_motor:
                        u_norm = np.clip((u - p_min) / max(1.0, p_max - p_min), 0.0, 1.0)
                    else:
                        # Bidirectional scaling centered at p_trim
                        u_norm = np.zeros_like(u)
                        above_trim = u >= p_trim
                        below_trim = ~above_trim
                        if np.any(above_trim):
                            u_norm[above_trim] = (u[above_trim] - p_trim) / max(1.0, p_max - p_trim)
                        if np.any(below_trim):
                            u_norm[below_trim] = (u[below_trim] - p_trim) / max(1.0, p_trim - p_min)
                        u_norm = np.clip(u_norm, -1.0, 1.0)
                else:
                    if is_motor:
                        u_norm = np.clip(u, 0.0, 1.0)
                    else:
                        u_norm = np.clip(u, -1.0, 1.0)

                # Saturation condition
                if is_motor:
                    saturated = (u_norm >= 1.0 - _SAT_EPS)
                else:
                    saturated = (np.abs(u_norm) >= 1.0 - _SAT_EPS)

                pct = float(100.0 * np.count_nonzero(saturated) / max(1, len(u_norm)))
                longest = _longest_burst_s(ts, saturated)
                
                # Command vs Output Comparison
                correlated_axis = None
                max_corr = 0.0
                mismatch_longest_burst_s = 0.0
                cmd_val_dec = None
                
                if controls_aligned:
                    ch_corrs = {}
                    for axis_name, cmd_series in controls_aligned.items():
                        if np.std(u_norm) > 1e-4 and np.std(cmd_series) > 1e-4:
                            c = np.corrcoef(u_norm, cmd_series)[0, 1]
                            if not np.isnan(c):
                                ch_corrs[axis_name] = c
                    if ch_corrs:
                        best_axis = max(ch_corrs.keys(), key=lambda k: abs(ch_corrs[k]))
                        best_val = ch_corrs[best_axis]
                        if abs(best_val) >= 0.3:
                            correlated_axis = best_axis
                            max_corr = best_val
                            
                            cmd_series = controls_aligned[best_axis]
                            cmd_val = cmd_series if best_axis == "thrust" else np.abs(cmd_series)
                            output_val = u_norm if is_motor else np.abs(u_norm)
                            
                            mismatch = (cmd_val > 0.95) & (output_val < 0.95)
                            mismatch_longest_burst_s = _longest_burst_s(ts, mismatch)
                            cmd_val_dec = cmd_val[::k]

                label = dynamic_labels.get(ch, f"{prefix}{idx + 1} (Ch{ch + 1})")
                u_norm_decimated = u_norm[::k]
                out[label] = {
                    "saturated_pct": round(pct, 2),
                    "longest_burst_s": round(longest, 3),
                    "flagged": longest >= _BURST_WARN_S or pct > 2.0 or mismatch_longest_burst_s >= 0.5,
                    "values": [round(float(v), 3) for v in u_norm_decimated],
                    "correlated_axis": correlated_axis,
                    "correlation": round(float(max_corr), 3) if correlated_axis else None,
                    "command_mismatch_duration_s": round(float(mismatch_longest_burst_s), 3) if correlated_axis else None,
                    "command_values": [round(float(v), 3) for v in cmd_val_dec] if cmd_val_dec is not None else None,
                }

                # Airspeed context for control surfaces
                if not is_motor and airspeed_data is not None and airframe_class in ("FIXED_WING", "DELTA_WING", "VTOL"):
                    as_ts, as_val = airspeed_data
                    if longest > 0:
                        edges = np.diff(saturated.astype(np.int8))
                        starts = np.flatnonzero(edges == 1) + 1
                        ends = np.flatnonzero(edges == -1) + 1
                        if saturated[0]:
                            starts = np.r_[0, starts]
                        if saturated[-1]:
                            ends = np.r_[ends, len(saturated)]
                        
                        durations = ts[np.minimum(ends - 1, len(ts) - 1)] - ts[starts]
                        if durations.size > 0:
                            longest_idx = np.argmax(durations)
                            burst_start = starts[longest_idx]
                            burst_end = ends[longest_idx]
                            
                            burst_ts = ts[burst_start:burst_end]
                            if burst_ts.size > 0:
                                burst_airspeeds = np.interp(burst_ts, as_ts, as_val)
                                mean_burst_airspeed = float(np.mean(burst_airspeeds))
                                
                                cruise_speed = 15.0
                                if params:
                                    cruise_speed = float(params.get("FW_AIRSPD_TRIM", 15.0))
                                    
                                if mean_burst_airspeed < 0.7 * cruise_speed:
                                    note = f"Surface railed at {mean_burst_airspeed:.1f} m/s – expected at low speed."
                                else:
                                    note = f"Surface railed at {mean_burst_airspeed:.1f} m/s – possible authority issue at cruise."
                                
                                out[label]["note"] = note
                                out[label]["airspeed_at_saturation"] = round(mean_burst_airspeed, 2)
                
                if out[label]["flagged"] and (worst is None or longest > worst[1]):
                    worst = (label, longest)
    else:
        # Legacy control setpoint analysis fallback
        legacy_channels = {
            "roll": "control[0]",
            "pitch": "control[1]",
            "yaw": "control[2]",
            "thrust": "control[3]",
        }
        for name, col in legacy_channels.items():
            if col not in actuators.columns:
                continue
            u = actuators[col].to_numpy(dtype=np.float64)
            if u.size == 0:
                continue
            saturated = (u >= 1.0 - _SAT_EPS) if name == "thrust" \
                else (np.abs(u) >= 1.0 - _SAT_EPS)

            pct = float(100.0 * np.count_nonzero(saturated) / max(1, len(u)))
            longest = _longest_burst_s(ts, saturated)
            u_decimated = u[::k]
            out[name] = {
                "saturated_pct": round(pct, 2),
                "longest_burst_s": round(longest, 3),
                "flagged": longest >= _BURST_WARN_S or pct > 2.0,
                "values": [round(float(v), 3) for v in u_decimated],
            }
            if out[name]["flagged"] and (worst is None or longest > worst[1]):
                worst = (name, longest)

    advice = None
    if worst:
        axis, dur = worst
        if "M" in axis or "P" in axis or "thrust" in axis:
            advice = (
                f"{axis} output saturated for up to {dur:.2f} s at a time. "
                "Check hover throttle, payload weight, battery sag, or forward thrust authority."
            )
        else:
            advice = (
                f"{axis} control surface railed for up to {dur:.2f} s at a time. "
                "Reduce rate gains or address mechanical linkages before increasing authority demands."
            )

    out_payload = {
        "channels": out,
        "advice": advice,
        "t": ts_decimated.tolist(),
    }

    # Add Decimated Airspeed Time Series if available
    if airspeed_data is not None:
        as_ts, as_val = airspeed_data
        decimated_airspeed = np.interp(ts_decimated + ts[0], as_ts, as_val)
        out_payload["airspeed"] = [round(float(v), 2) for v in decimated_airspeed]

    # Motor Output Balance History
    if is_physical and actuator_map and airframe_class in ("MULTIROTOR", "VTOL"):
        hover_channels = actuator_map.get("hover_motors", [])
        hover_cols = [f"output[{ch}]" for ch in hover_channels if f"output[{ch}]" in actuators.columns]
        if len(hover_cols) >= 3:
            # Calculate sample rate to define 4.0s rolling window
            fs = len(ts) / duration if duration > 0.1 else 100.0
            window_samples = int(4.0 * fs)
            
            df_hover = actuators[hover_cols]
            df_rolled = df_hover.rolling(window=max(5, window_samples), min_periods=max(1, window_samples // 4)).mean()
            rolled_decimated = df_rolled.iloc[::k]
            
            deviations_timeline = {f"M{i+1}": [] for i in range(len(hover_cols))}
            for idx in range(len(rolled_decimated)):
                row = rolled_decimated.iloc[idx].to_numpy()
                if np.any(np.isnan(row)):
                    for i in range(len(hover_cols)):
                        deviations_timeline[f"M{i+1}"].append(None)
                    continue
                
                fleet_mean = np.mean(row)
                if fleet_mean <= 1e-3:
                    for i in range(len(hover_cols)):
                        deviations_timeline[f"M{i+1}"].append(0.0)
                else:
                    for i in range(len(hover_cols)):
                        dev = (row[i] - fleet_mean) / fleet_mean
                        deviations_timeline[f"M{i+1}"].append(round(float(dev), 3))
            
            out_payload["motor_balance"] = {
                "t": ts_decimated.tolist(),
                "deviations": deviations_timeline,
            }

    return out_payload


def _longest_burst_s(ts: np.ndarray, mask: np.ndarray) -> float:
    """Length in seconds of the longest contiguous True run in `mask`."""
    if not mask.any():
        return 0.0
    edges = np.diff(mask.astype(np.int8))
    starts = np.flatnonzero(edges == 1) + 1
    ends = np.flatnonzero(edges == -1) + 1
    if mask[0]:
        starts = np.r_[0, starts]
    if mask[-1]:
        ends = np.r_[ends, len(mask)]
    durations = ts[np.minimum(ends - 1, len(ts) - 1)] - ts[starts]
    return float(durations.max()) if durations.size else 0.0

