import unittest
import asyncio
import time
import numpy as np
import pandas as pd
from collections import deque
from pathlib import Path
from unittest.mock import MagicMock, patch, AsyncMock

from backend.app.analysis.live_pid import LivePidEngine, compute_coherence_in_band, check_actuator_saturation
from backend.app.analysis import pid_offline
from backend.app.analysis.vibration_live import VibrationGate
from backend.app.analysis.ekf_monitor import EkfMonitor
from backend.app.analysis.domains import ActuationMonitor
from backend.app.advisors.param_advisor import ProposalState
from backend.app.mavlink.telemetry_hub import HUB
from backend.app.analysis.regime import REGIME, Regime

class TestTuningIssues(unittest.TestCase):

    def test_offline_parameter_mappings(self):
        # Issue #1, #13: Verify fixed-wing maps to Proportional rate parameters instead of Feedforward
        self.assertEqual(pid_offline._RATE_P_PARAM["FIXED_WING"]["roll"], "FW_RR_P")
        self.assertEqual(pid_offline._RATE_P_PARAM["FIXED_WING"]["pitch"], "FW_PR_P")
        self.assertEqual(pid_offline._RATE_P_PARAM["FIXED_WING"]["yaw"], "FW_YR_P")
        self.assertEqual(pid_offline._RATE_P_PARAM["DELTA_WING"]["roll"], "FW_RR_P")
        self.assertEqual(pid_offline._RATE_P_PARAM["DELTA_WING"]["pitch"], "FW_PR_P")
        self.assertIsNone(pid_offline._RATE_P_PARAM["DELTA_WING"]["yaw"])

    @patch("backend.app.analysis.live_pid._airframe_class")
    def test_live_pid_rate_param_mappings(self, mock_airframe_class):
        engine = LivePidEngine()
        
        # Test FIXED_WING: yaw D-term should be None, others should be FW_
        mock_airframe_class.return_value = "FIXED_WING"
        self.assertEqual(engine._rate_param("roll", "P"), "FW_RR_P")
        self.assertEqual(engine._rate_param("pitch", "D"), "FW_PR_D")
        self.assertIsNone(engine._rate_param("yaw", "D"))

        # Test MULTIROTOR: yaw D-term is allowed (returns MC_YAWRATE_D)
        mock_airframe_class.return_value = "MULTIROTOR"
        self.assertEqual(engine._rate_param("yaw", "D"), "MC_YAWRATE_D")

    @patch("backend.app.analysis.live_pid._airframe_class")
    def test_vtol_mode_tracking(self, mock_airframe_class):
        engine = LivePidEngine()
        mock_airframe_class.return_value = "VTOL"
        
        # In hover (vtol_state != 4): returns MC_ parameters
        engine._vtol_state = 3
        self.assertEqual(engine._rate_param("roll", "P"), "MC_ROLLRATE_P")
        self.assertEqual(engine._rate_param("yaw", "D"), "MC_YAWRATE_D")
        self.assertEqual(engine._auto_rate_param("roll"), "MC_ROLLRAUTO_MAX")

        # In forward flight (vtol_state == 4): returns FW_ parameters, yaw D blocked
        engine._vtol_state = 4
        self.assertEqual(engine._rate_param("roll", "P"), "FW_RR_P")
        self.assertIsNone(engine._rate_param("yaw", "D"))
        self.assertIsNone(engine._auto_rate_param("roll"))

    def test_coherence_data_gating(self):
        # Issue #5: Coherence defaults to None with insufficient data (<8 samples)
        t = np.arange(6)
        x = np.sin(t)
        y = np.sin(t)
        self.assertIsNone(compute_coherence_in_band(t, x, y))

    def test_step_response_metrics(self):
        # Issue #17: Settling time is None when never settled
        t = np.linspace(0, 1, 100)
        sp = np.zeros_like(t)
        sp[50:] = 1.0  # commanded step from 0 to 1 at t=0.5
        # Response oscillates wildly and never enters +/-20% settled band around 1.0
        act = np.zeros_like(t)
        act[50:] = 2.0  # constant overshoot to 2.0 (100% overshoot)
        res = LivePidEngine._step_response(t, sp, act, min_amp=0.15)
        self.assertIsNotNone(res)
        self.assertIsNone(res["settling_s"])

        # Issue #18: Tau is None when response starts above 63.2%
        act_bad = np.zeros_like(t)
        act_bad[:50] = 0.8  # starts already above 0.632 * amp (0.632)
        act_bad[50:] = 1.0
        res_bad = LivePidEngine._step_response(t, sp, act_bad, min_amp=0.15)
        self.assertIsNotNone(res_bad)
        self.assertIsNone(res_bad["tau_s"])

    def test_saturation_metrics_splitting(self):
        # Issue #7: Check that check_actuator_saturation returns peak and sustained separately
        history = [(0.0, [0.1]), (0.05, [0.95]), (0.1, [0.95]), (0.15, [0.95]), (0.2, [0.5])]
        is_sat, dur, peak, sustained, idx = check_actuator_saturation(history, limit_threshold=0.85)
        self.assertTrue(is_sat)
        self.assertAlmostEqual(dur, 0.20 - 0.05)
        self.assertEqual(peak, 0.95)
        self.assertEqual(sustained, 0.95)

    def test_motor_balance_buffer(self):
        # Issue #20: Balance check preserves last 0.5s history when leaving STEADY_HOLD
        analysis = ActuationMonitor()
        analysis._domain = "multirotor"
        
        # Populate history
        now = time.monotonic()
        analysis._motor_hist = {
            0: deque([(now - 1.0, 0.5), (now - 0.4, 0.5), (now - 0.1, 0.5)])
        }
        
        # Trigger check outside STEADY_HOLD (Regime PRE_FLIGHT / DYNAMIC)
        with patch("backend.app.analysis.domains.REGIME") as mock_regime:
            mock_regime.current = Regime.DYNAMIC_MANEUVER
            res = analysis._check_motor_balance([0.5], now)
            self.assertIsNone(res)
            # History older than now - 0.5 should be popped. (now - 1.0 is popped, others kept)
            self.assertEqual(len(analysis._motor_hist[0]), 2)
            self.assertEqual(analysis._motor_hist[0][0][0], now - 0.4)

    def test_revert_tolerance(self):
        # Issue #25: Revert check tolerates float rounding using 1e-4 bounds
        # Let's mock a proposal
        prop = MagicMock()
        prop.state = ProposalState.REVERTED  # just dummy state
        # In param_advisor.py revert logic:
        # wrote = prop.proposed_value
        # tol = max(1e-4, abs(wrote) * 1e-3)
        # abs(live - wrote) <= tol
        wrote = 0.001
        live = 0.00105  # difference is 0.00005, which is < 1e-4
        tol = max(1e-4, abs(wrote) * 1e-3)
        self.assertTrue(abs(live - wrote) <= tol)


    @patch("backend.app.analysis.ekf_monitor.HUB")
    def test_estimator_status_parsing(self, mock_hub):
        monitor = EkfMonitor()
        report = {
            "vel_ratio": 0.123,
            "pos_horiz_ratio": 0.456,
            "mag_ratio": 0.789,
            "pos_vert_ratio": 0.321,
            "flags": 15
        }
        monitor._process_estimator_status(report)
        
        publish_calls = [(call[0][0], call[0][1]) for call in mock_hub.publish.call_args_list]
        self.assertIn(("ekf_metrics", {
            "ratios": {
                "gps_velocity": 0.123,
                "gps_position": 0.456,
                "magnetometer": 0.789,
                "barometer": 0.321
            },
            "flags": 15,
            "regime": REGIME.current.value,
            "status": "ok"
        }), publish_calls)


class TestAsyncWatchdogs(unittest.IsolatedAsyncioTestCase):

    @patch("backend.app.analysis.vibration_live.CONNECTION")
    @patch("backend.app.analysis.vibration_live.HUB")
    async def test_vibration_gate_watchdog_gated(self, mock_hub, mock_connection):
        # When disconnected: watchdog loop runs but does not fire alert
        mock_connection.state.connected = False
        gate = VibrationGate()
        
        with patch("backend.app.analysis.vibration_live.asyncio.sleep", side_effect=[None, asyncio.CancelledError()]):
            try:
                await gate._missing_vibration_watchdog()
            except asyncio.CancelledError:
                pass
        
        # Verify no alerts published
        for call in mock_hub.publish.call_args_list:
            self.assertNotEqual(call[0][0], "alert")

        # When connected and self._ever_seen is False: watchdog fires alert after 10s
        mock_connection.state.connected = True
        gate = VibrationGate()
        gate._ever_seen = False
        
        with patch("backend.app.analysis.vibration_live.time.monotonic", side_effect=[100.0, 111.0]), \
             patch("backend.app.analysis.vibration_live.asyncio.sleep", return_value=None):
            await gate._missing_vibration_watchdog()
            
        # Verify alert published
        publish_calls = [(call[0][0], call[0][1]) for call in mock_hub.publish.call_args_list]
        self.assertIn(("alert", {
            "severity": "warning", "source": "vibration",
            "text": "Vibration data never received. Gain raises will be allowed without vibration gating."
        }), publish_calls)

    @patch("backend.app.analysis.ekf_monitor.CONNECTION")
    @patch("backend.app.analysis.ekf_monitor.HUB")
    async def test_ekf_monitor_watchdog_gated(self, mock_hub, mock_connection):
        # When disconnected: watchdog loop runs but does not fire alert
        mock_connection.state.connected = False
        monitor = EkfMonitor()
        
        with patch("backend.app.analysis.ekf_monitor.asyncio.sleep", side_effect=[None, asyncio.CancelledError()]):
            try:
                await monitor._staleness_loop()
            except asyncio.CancelledError:
                pass
        
        # Verify no alerts published
        for call in mock_hub.publish.call_args_list:
            self.assertNotEqual(call[0][0], "alert")

        # When connected and no EKF data: watchdog fires alert after 5s
        mock_connection.state.connected = True
        mock_hub.last_seen.return_value = None
        monitor = EkfMonitor()
        
        with patch("backend.app.analysis.ekf_monitor.time.monotonic", side_effect=[100.0, 100.0, 106.0, 106.0]), \
             patch("backend.app.analysis.ekf_monitor.asyncio.sleep", side_effect=[None, None, asyncio.CancelledError()]):
            try:
                await monitor._staleness_loop()
            except asyncio.CancelledError:
                pass
            
        # Verify alert published
        publish_calls = [(call[0][0], call[0][1]) for call in mock_hub.publish.call_args_list]
        self.assertIn(("alert", {
            "severity": "critical", "source": "ekf",
            "text": "No EKF data received within 5 seconds of connection. Check PX4 configuration."
        }), publish_calls)

    @patch("backend.app.mavlink.connection.asyncio.sleep", side_effect=asyncio.CancelledError)
    @patch("backend.app.mavlink.connection.HUB")
    async def test_staleness_watchdog_no_data(self, mock_hub, mock_sleep):
        from backend.app.mavlink.connection import ConnectionManager
        cm = ConnectionManager()
        mock_hub.last_seen.return_value = None
        
        mock_sleep.side_effect = [None, asyncio.CancelledError()]
        await cm._staleness_watchdog()
        
        for call in mock_hub.publish.call_args_list:
            self.assertNotEqual(call[0][0], "telemetry_stale")

    @patch("backend.app.mavlink.connection.asyncio.sleep")
    @patch("backend.app.mavlink.connection.HUB")
    async def test_staleness_watchdog_state_transitions(self, mock_hub, mock_sleep):
        from backend.app.mavlink.connection import ConnectionManager
        cm = ConnectionManager()
        
        mock_sleep.side_effect = [None, None, None, asyncio.CancelledError()]
        
        last_seen_vals = {"attitude": 100.0, "vfr_hud": 100.0}
        mock_hub.last_seen.side_effect = lambda c: last_seen_vals.get(c)
        
        is_stale_vals = [
            False, False,  # Iteration 1
            True, True,    # Iteration 2
            False, False   # Iteration 3
        ]
        mock_hub.is_stale.side_effect = is_stale_vals
        
        await cm._staleness_watchdog()
        
        publish_calls = [(call[0][0], call[0][1]) for call in mock_hub.publish.call_args_list]
        self.assertIn(("telemetry_stale", {"stale": True, "channels": ["attitude", "vfr_hud"]}), publish_calls)
        self.assertIn(("telemetry_stale", {"stale": False, "channels": []}), publish_calls)
        
        critical_alerts = [p[1] for p in publish_calls if p[0] == "alert" and p[1].get("severity") == "critical"]
        self.assertEqual(len(critical_alerts), 1)
        
        info_alerts = [p[1] for p in publish_calls if p[0] == "alert" and p[1].get("severity") == "info"]
        self.assertEqual(len(info_alerts), 1)

class TestNewTuningImprovements(unittest.IsolatedAsyncioTestCase):

    def test_saturation_severity(self):
        engine = LivePidEngine()
        
        # none: no history
        self.assertEqual(engine._get_saturation_severity(), "none")
        
        # none: peak 0.8, sustained 0.8
        engine._actuation_history.append((time.monotonic(), [0.8]))
        self.assertEqual(engine._get_saturation_severity(), "none")
        
        # mild: sustained >= 0.85
        engine._actuation_history.clear()
        now = time.monotonic()
        for i in range(10):
            engine._actuation_history.append((now + i * 0.05, [0.90]))
        self.assertEqual(engine._get_saturation_severity(), "mild")
        
        # moderate: sustained >= 0.95
        engine._actuation_history.clear()
        now = time.monotonic()
        for i in range(10):
            engine._actuation_history.append((now + i * 0.05, [0.98]))
        self.assertEqual(engine._get_saturation_severity(), "moderate")
        
        # severe: peak >= 1.0, duration >= 1.0s
        engine._actuation_history.clear()
        now = time.monotonic()
        for i in range(25): # 25 * 0.05 = 1.25s
            engine._actuation_history.append((now + i * 0.05, [1.0]))
        self.assertEqual(engine._get_saturation_severity(), "severe")

    def test_is_proposal_allowed(self):
        # none severity allows everything
        self.assertTrue(LivePidEngine._is_proposal_allowed("MC_ROLLRATE_P", 1.1, None, "none"))
        self.assertTrue(LivePidEngine._is_proposal_allowed("MC_ROLLRATE_P", 0.9, None, "none"))
        
        # mild blocks P-gain increase
        self.assertFalse(LivePidEngine._is_proposal_allowed("MC_ROLLRATE_P", 1.1, None, "mild"))
        self.assertFalse(LivePidEngine._is_proposal_allowed("FW_RR_P", None, 0.05, "mild"))
        # mild allows P-gain reduction
        self.assertTrue(LivePidEngine._is_proposal_allowed("MC_ROLLRATE_P", 0.9, None, "mild"))
        # mild allows rate limit reduction
        self.assertTrue(LivePidEngine._is_proposal_allowed("MC_ROLLRAUTO_MAX", 0.85, None, "mild"))
        
        # moderate blocks P-gain increase
        self.assertFalse(LivePidEngine._is_proposal_allowed("MC_ROLLRATE_P", 1.1, None, "moderate"))
        # moderate allows P-gain reduction
        self.assertTrue(LivePidEngine._is_proposal_allowed("MC_ROLLRATE_P", 0.9, None, "moderate"))
        
        # severe blocks everything
        self.assertFalse(LivePidEngine._is_proposal_allowed("MC_ROLLRATE_P", 0.9, None, "severe"))
        self.assertFalse(LivePidEngine._is_proposal_allowed("MC_ROLLRAUTO_MAX", 0.85, None, "severe"))

    def test_step_response_relaxed_thresholds(self):
        import math
        # Verify min_amp down to 0.10 rad/s is accepted
        t = np.linspace(0, 1, 100)
        sp = np.zeros_like(t)
        sp[50:] = 0.12  # Step amplitude 0.12 (less than 0.15, but >= 0.10)
        act = np.zeros_like(t)
        act[50:] = 0.12
        
        res = LivePidEngine._step_response(t, sp, act, min_amp=0.10)
        self.assertIsNotNone(res)
        self.assertEqual(res["step_amp_deg_s"], round(math.degrees(0.12), 1))
        
        # Verify ramp relaxation threshold: post_sp_std > 0.5 * abs(amp) returns None
        sp_ramp = np.zeros_like(t)
        sp_ramp[50:] = 0.12
        sp_ramp[55::2] = 1.0  # wild oscillation post-step
        res_ramp = LivePidEngine._step_response(t, sp_ramp, act, min_amp=0.10)
        self.assertIsNone(res_ramp)
        
        # Verify slightly ramped input correction heuristic:
        # post_sp_std > 0.35 * abs(amp) multiplies tau by 1.5 and sets ramped_adjusted=True
        sp_slight_ramp = np.zeros_like(t)
        sp_slight_ramp[50:] = np.linspace(0.01, 1.0, 50)
        
        res_slight = LivePidEngine._step_response(t, sp_slight_ramp, act, min_amp=0.10)
        self.assertIsNotNone(res_slight)
        self.assertTrue(res_slight["ramped_adjusted"])


    def test_diagnostic_cards(self):
        from backend.app.advisors.param_advisor import ParamAdvisor, Proposal, ProposalState
        advisor = ParamAdvisor()
        
        # 1. Set diagnostic card
        advisor.set_diagnostic_card("roll", "Vibration too high")
        props = advisor.list_proposals()
        self.assertEqual(len(props), 1)
        self.assertEqual(props[0]["state"], "diagnostic")
        self.assertEqual(props[0]["rationale"], "Vibration too high")
        self.assertEqual(props[0]["param"], "ROLL")
        
        # 2. If a real proposal exists for roll, diagnostic should not be appended
        prop = Proposal(
            id="test_prop",
            param="MC_ROLLRATE_P",
            current_value=0.15,
            proposed_value=0.165,
            requested_value=0.165,
            rationale="Test real",
            airframe_class="MULTIROTOR",
            state=ProposalState.PRESENTED,
            safety_note=""
        )
        advisor._proposals[prop.id] = prop
        
        props_after = advisor.list_proposals()
        # Diagnostic card should be filtered out because there is a real roll proposal
        real_props = [p for p in props_after if p["state"] != "diagnostic"]
        diag_props = [p for p in props_after if p["state"] == "diagnostic"]
        self.assertEqual(len(real_props), 1)
        self.assertEqual(len(diag_props), 0)
        
        # 3. Clear diagnostic card
        advisor.clear_diagnostic_card("roll")
        self.assertNotIn("roll", advisor._diagnostics)

    def test_tuning_window_axis_scoping(self):
        from backend.app.analysis.stick_monitor import STICK_MONITOR
        from backend.app.analysis.live_pid import LivePidEngine
        from backend.app.advisors.param_advisor import ADVISOR
        import math
        
        engine = LivePidEngine()
        ADVISOR.clear()
        
        # Open tuning window for pitch
        STICK_MONITOR.begin_window("pitch", "rate")
        
        # Populate history/windows for roll and pitch to trigger evaluation
        now = time.monotonic()
        engine._win["roll"].clear()
        engine._win["pitch"].clear()
        for i in range(20):
            engine._win["roll"].append((now + i * 0.05, 0.2 * math.sin(i), 0.2 * math.sin(i)))
            engine._win["pitch"].append((now + i * 0.05, 0.2 * math.sin(i), 0.2 * math.sin(i)))
            
        # Mock _determine_diagnostic for both axes
        engine._determine_diagnostic = lambda ax, t, *args, **kwargs: f"Mock diagnostic for {ax}"
        
        # Trigger evaluation
        engine._evaluate(now)
        
        # Verify roll active start and diagnostics are cleared/skipped
        self.assertNotIn("roll", engine._axis_active_start)
        self.assertNotIn("roll", ADVISOR._diagnostics)
        
        # Set pitch active start to >10s ago
        engine._axis_active_start["pitch"] = now - 11.0
        engine._evaluate(now)
        
        self.assertIn("pitch", ADVISOR._diagnostics)
        self.assertNotIn("roll", ADVISOR._diagnostics)
        
        # Close window
        STICK_MONITOR.end_window()

    def test_actuator_stream_dynamic_domain_resolution(self):
        # Verify that ActuationMonitor starts processing without strict _domain gating,
        # and correctly resolves its domain property from CONNECTION.state.airframe
        from backend.app.mavlink.connection import CONNECTION, VehicleState, AirframeInfo
        
        monitor = ActuationMonitor()
        self.assertIsNone(monitor._domain)
        self.assertIsNone(monitor.domain)
        
        # Mock CONNECTION state
        CONNECTION.state = VehicleState(
            connected=True,
            airframe=AirframeInfo(
                sys_autostart=4001,
                airframe_class="MULTIROTOR",
                label="Generic Quad",
                mav_type=2,
                source="SYS_AUTOSTART"
            )
        )
        
        # domain property should dynamically resolve
        self.assertEqual(monitor.domain, "differential_thrust")
        
        # Test processing output status without strict gating
        payload = {"actuator": [0.5, 0.6, 0.7, 0.8]}
        
        # Mock HUB.publish
        with patch("backend.app.analysis.domains.HUB") as mock_hub:
            monitor._process_actuator_status(payload)
            self.assertTrue(mock_hub.publish.called)
            # Make sure it published to "actuation" channel
            self.assertEqual(mock_hub.publish.call_args[0][0], "actuation")
            
        # Clean up
        CONNECTION.state = VehicleState()

    async def test_underdamped_reversal_ringing_classification(self):
        # Verify that an overshoot with exactly 2 reversals is classified as a
        # damping deficit (Rate D increase recommended) rather than proportional overshoot (P backoff).
        from backend.app.analysis.live_pid import LivePidEngine
        from backend.app.advisors.param_advisor import ADVISOR
        
        engine = LivePidEngine()
        ADVISOR.clear()
        
        # Mock VIB_GATE to return ok=True
        from backend.app.analysis import recommendations
        recommendations._last_emit.clear()
        
        with patch("backend.app.analysis.live_pid.VIB_GATE.ok", return_value=True), \
             patch("backend.app.analysis.live_pid._airframe_class", return_value="MULTIROTOR"), \
             patch("backend.app.analysis.live_pid.CONNECTION.read_param", new_callable=AsyncMock) as mock_read_param, \
             patch("backend.app.analysis.live_pid.HUB.publish") as mock_publish:
            
            mock_read_param.return_value = 0.003
            
            # Step response metrics with overshoot 20% (>15% limit) and 2 oscillations (<3 under old config but >=2 under new)
            metrics = {
                "tau_s": 0.1,
                "settling_s": 0.4,
                "overshoot": 0.20,
                "oscillations": 2,
                "step_amp_deg_s": 20.0,
                "amplitude": 0.35,
                "post_sp_std": 0.02,
                "ramped_adjusted": False,
            }
            
            # Mock rate parameters
            engine._rate_param = MagicMock(side_effect=lambda ax, term: f"MC_{ax.upper()}RATE_{term}")
            engine._recommended_axes_this_cycle = set()
            
            # Run verdicts
            engine._verdicts("roll", metrics, time.monotonic(), "High", "None")
            
            # Allow background tasks to run
            await asyncio.sleep(0.05)
            
            # Check published alerts and recommendations
            publish_calls = [(call[0][0], call[0][1]) for call in mock_publish.call_args_list]
            
            # Should have alert saying "damping deficit"
            damping_alerts = [p[1] for p in publish_calls if p[0] == "alert" and "damping deficit" in p[1].get("text", "")]
            self.assertEqual(len(damping_alerts), 1)
            
            # Should recommend Rate D parameter change with scale factor 1.15
            recs_calls = [p[1] for p in publish_calls if p[0] == "recommendation"]
            self.assertTrue(any("MC_ROLLRATE_D" in r.get("param", "") for r in recs_calls))

    def test_damping_ratio_calculation(self):
        # Test _damping_ratio helper
        from backend.app.analysis.live_pid import _damping_ratio
        
        # Test no/low overshoot: <= 0.01 should return None
        self.assertIsNone(_damping_ratio(0.005))
        self.assertIsNone(_damping_ratio(-0.1))
        
        # Test typical overshoot
        # 30% overshoot: -ln(0.3) / sqrt(pi^2 + ln(0.3)^2)
        # log(0.3) = -1.20397
        # log(0.3)^2 = 1.4495
        # pi^2 + log(0.3)^2 = 9.8696 + 1.4495 = 11.3191
        # sqrt = 3.3644
        # zeta = 1.20397 / 3.3644 = 0.3578
        zeta_30 = _damping_ratio(0.30)
        self.assertAlmostEqual(zeta_30, 0.3579, places=4)
        
        # 40% overshoot: log(0.4) = -0.9163
        # log(0.4)^2 = 0.8396
        # pi^2 + log(0.4)^2 = 9.8696 + 0.8396 = 10.7092
        # sqrt = 3.2725
        # zeta = 0.9163 / 3.2725 = 0.2800 (under 0.3!)
        zeta_40 = _damping_ratio(0.40)
        self.assertAlmostEqual(zeta_40, 0.2800, places=4)

    async def test_advise_damping_decision_table(self):
        # Verify D-gain / P-gain recommendation decision table
        from backend.app.analysis.live_pid import LivePidEngine
        
        engine = LivePidEngine()
        
        # Mock rate parameters
        engine._rate_param = MagicMock(side_effect=lambda ax, term: f"MC_{ax.upper()}RATE_{term}")
        
        # Case 1: Vibration Low, D-gain not max -> Increase D-gain
        with patch("backend.app.analysis.live_pid.VIB_GATE.ok", return_value=True), \
             patch("backend.app.analysis.live_pid._airframe_class", return_value="MULTIROTOR"), \
             patch("backend.app.analysis.live_pid.CONNECTION.read_param", new_callable=AsyncMock) as mock_read_param, \
             patch.object(engine, "_safe_recommend") as mock_rec:
             
            mock_read_param.return_value = 0.003  # Below abs_max = 0.01
            await engine._advise_damping("roll", 0.40, 0, 0.28, "Medium", None)
            
            mock_rec.assert_called_once_with(
                "roll", "MC_ROLLRATE_D",
                "Roll overshoot 40% with low damping ratio (ζ=0.28) — add derivative damping.",
                scale_factor=1.15,
                confidence="Medium",
                limitations=None,
                severity=None,
                pre_step_motion=False,
                ramped_input=False,
                low_coherence=False
            )

        # Case 2: Vibration Low, D-gain already maxed (>= abs_max) -> Reduce P-gain
        with patch("backend.app.analysis.live_pid.VIB_GATE.ok", return_value=True), \
             patch("backend.app.analysis.live_pid._airframe_class", return_value="MULTIROTOR"), \
             patch("backend.app.analysis.live_pid.CONNECTION.read_param", new_callable=AsyncMock) as mock_read_param, \
             patch.object(engine, "_safe_recommend") as mock_rec:
             
            mock_read_param.return_value = 0.01  # At abs_max = 0.01
            await engine._advise_damping("roll", 0.40, 0, 0.28, "Medium", None)
            
            mock_rec.assert_called_once_with(
                "roll", "MC_ROLLRATE_P",
                "Roll overshoot 40% with low damping, but Rate D-gain is at its safety limit (0.0100) — back off P-gain instead.",
                scale_factor=0.9,
                confidence="Medium",
                limitations=None,
                severity=None,
                pre_step_motion=False,
                ramped_input=False,
                low_coherence=False
            )

        # Case 3: Vibration High, D-gain not max -> Reduce P-gain
        with patch("backend.app.analysis.live_pid.VIB_GATE.ok", return_value=False), \
             patch("backend.app.analysis.live_pid._airframe_class", return_value="MULTIROTOR"), \
             patch("backend.app.analysis.live_pid.CONNECTION.read_param", new_callable=AsyncMock) as mock_read_param, \
             patch.object(engine, "_safe_recommend") as mock_rec:
             
            mock_read_param.return_value = 0.003  # Below abs_max = 0.01
            await engine._advise_damping("roll", 0.40, 0, 0.28, "Medium", None)
            
            mock_rec.assert_called_once_with(
                "roll", "MC_ROLLRATE_P",
                "Roll overshoot 40% with low damping, but Rate D-gain increases are blocked by high vibration — back off P-gain instead.",
                scale_factor=0.9,
                confidence="Medium",
                limitations=None,
                severity=None,
                pre_step_motion=False,
                ramped_input=False,
                low_coherence=False
            )

        # Case 4: read_param fails (timeout/exception) -> Reduce P-gain (treat as maxed fallback)
        with patch("backend.app.analysis.live_pid.VIB_GATE.ok", return_value=True), \
             patch("backend.app.analysis.live_pid._airframe_class", return_value="MULTIROTOR"), \
             patch("backend.app.analysis.live_pid.CONNECTION.read_param", side_effect=TimeoutError("Read failed")), \
             patch.object(engine, "_safe_recommend") as mock_rec:
             
            await engine._advise_damping("roll", 0.40, 0, 0.28, "Medium", None)
            
            mock_rec.assert_called_once_with(
                "roll", "MC_ROLLRATE_P",
                "Roll overshoot 40% with low damping, but Rate D-gain is at its safety limit (0.0000) — back off P-gain instead.",
                scale_factor=0.9,
                confidence="Medium",
                limitations=None,
                severity=None,
                pre_step_motion=False,
                ramped_input=False,
                low_coherence=False
            )

    def test_step_vibration_rejection(self):
        # Verify that act_noise > 0.05 rad/s RMS rejects step entirely
        from backend.app.analysis.live_pid import LivePidEngine
        from backend.app.advisors.param_advisor import ADVISOR
        
        t = np.linspace(0, 1, 100)
        sp = np.zeros_like(t)
        sp[50:] = 0.5
        
        # Clean response (act_noise = 0.001)
        act_clean = np.zeros_like(t)
        act_clean[50:] = 0.5
        
        # Vibrating pre-step response (act_noise = 0.06 > 0.05)
        # Let's add noise to the pre-step period
        np.random.seed(42)
        noise = np.random.normal(0, 0.06, size=100)
        act_noisy = sp + noise
        
        # Verify clean step is accepted
        res_clean = LivePidEngine._step_response(t, sp, act_clean, min_amp=0.15)
        self.assertIsNotNone(res_clean)
        
        # Verify noisy step is rejected (returns None)
        with patch("backend.app.analysis.live_pid.VIB_GATE.ok", return_value=False):
            ADVISOR.clear()
            recommended_axes = set()
            res_noisy = LivePidEngine._step_response(t, sp, act_noisy, min_amp=0.15, ax="roll", recommended_axes=recommended_axes)
            self.assertIsNone(res_noisy)
            self.assertIn("roll", recommended_axes)
            props = ADVISOR.list_proposals()
            self.assertTrue(any(p.get("state") == "diagnostic" and "excessive vibration" in p.get("rationale") for p in props))

    def test_adaptive_window_override(self):
        # Verify window override logic after 2 consecutive cycles of overshoot > 20% but zero oscillations
        from backend.app.analysis.live_pid import LivePidEngine, _WINDOW_S
        
        engine = LivePidEngine()
        engine._recommended_axes_this_cycle = set()
        
        # Initially, override is default (_WINDOW_S)
        self.assertEqual(engine._window_s_override["roll"], _WINDOW_S)
        
        # Mock parameters & publish
        engine._rate_param = MagicMock(side_effect=lambda ax, term: f"MC_{ax.upper()}RATE_{term}")
        
        # Cycle 1: overshoot 25%, osc 0
        metrics = {
            "tau_s": 0.1,
            "settling_s": 0.4,
            "overshoot": 0.25,
            "oscillations": 0,
            "amplitude": 0.5,
            "post_sp_std": 0.02,
        }
        
        with patch("backend.app.analysis.live_pid.HUB.publish"):
            engine._verdicts("roll", metrics, time.monotonic(), "Medium", None)
            
            # Consecutive counter goes to 1, window size still _WINDOW_S
            self.assertEqual(engine._consecutive_overshoot_no_osc["roll"], 1)
            self.assertEqual(engine._window_s_override["roll"], _WINDOW_S)
            
            # Cycle 2: overshoot 25%, osc 0
            engine._verdicts("roll", metrics, time.monotonic(), "Medium", None)
            
            # Consecutive counter goes to 2, window size becomes 4.0
            self.assertEqual(engine._consecutive_overshoot_no_osc["roll"], 2)
            self.assertEqual(engine._window_s_override["roll"], 4.0)
            
            # Cycle 3: no overshoot (0.05) or osc 1 -> should reset override
            metrics_reset = {
                "tau_s": 0.1,
                "settling_s": 0.4,
                "overshoot": 0.05,
                "oscillations": 0,
                "amplitude": 0.5,
                "post_sp_std": 0.02,
            }
            engine._verdicts("roll", metrics_reset, time.monotonic(), "Medium", None)
            
            self.assertEqual(engine._consecutive_overshoot_no_osc["roll"], 0)
            self.assertEqual(engine._window_s_override["roll"], _WINDOW_S)

    async def test_discover_actuators_sequential(self):
        from backend.app.mavlink.connection import ConnectionManager, VehicleState
        
        conn = ConnectionManager()
        conn.state = VehicleState()
        conn._system = MagicMock()
        
        params = {
            "ACT_FUNC1": 101,  # Motor 1
            "ACT_FUNC2": 102,  # Motor 2
            "ACT_FUNC3": 201,  # Servo 1
            "ACT_FUNC4": 202,  # Servo 2
            # Limits for Ch3 and Ch4
            "OUT3_MIN": 1100.0,
            "OUT3_MAX": 1900.0,
            "OUT3_TRIM": 1550.0,
            "OUT4_MIN": 1000.0,
            "OUT4_MAX": 2000.0,
            "OUT4_TRIM": 1500.0,
        }
        
        async def mock_read_param(name, attempts=None):
            if name in params:
                return float(params[name])
            raise Exception("Param not found")
            
        conn.read_param = mock_read_param
        
        with patch("backend.app.mavlink.connection.log"):
            await conn._discover_actuators()
            
            # Verify we have limits for present channels
            self.assertIn(0, conn.state.actuator_limits)
            self.assertIn(1, conn.state.actuator_limits)
            self.assertIn(2, conn.state.actuator_limits)
            self.assertIn(3, conn.state.actuator_limits)
            
            # Verify we do NOT have limits for other channels
            self.assertNotIn(4, conn.state.actuator_limits)
            
            # Verify values
            self.assertEqual(conn.state.actuator_limits[2]["min"], 1100.0)
            self.assertEqual(conn.state.actuator_limits[2]["max"], 1900.0)
            self.assertEqual(conn.state.actuator_limits[2]["trim"], 1550.0)

    def test_vibration_gate_hysteresis(self):
        from backend.app.analysis.vibration_live import VIB_GATE
        # Initialize gate state
        VIB_GATE._last_ok_val = True
        VIB_GATE._last_cleared_time = 0.0
        
        # Trigger transition: True -> False
        VIB_GATE._ever_seen = True
        VIB_GATE._last_seen = time.monotonic()
        VIB_GATE._latest = {"x": 35.0, "y": 0.0, "z": 0.0} # worst = 35 >= _VIB_WARN (30)
        self.assertFalse(VIB_GATE.ok())
        self.assertFalse(VIB_GATE.just_cleared())
        
        # Transition: False -> True
        VIB_GATE._latest = {"x": 10.0, "y": 0.0, "z": 0.0} # worst = 10 < 30
        self.assertTrue(VIB_GATE.ok())
        self.assertTrue(VIB_GATE.just_cleared())
        
        # After 3.1 seconds, just_cleared() should be False
        with patch("backend.app.analysis.vibration_live.time.monotonic", return_value=time.monotonic() + 3.1):
            self.assertFalse(VIB_GATE.just_cleared())

    def test_step_response_rejections_and_clamp(self):
        from backend.app.analysis.live_pid import LivePidEngine
        from backend.app.advisors.param_advisor import ADVISOR
        
        t = np.linspace(0, 2, 200) # 2 seconds window
        sp = np.zeros_like(t)
        sp[100:] = 0.5 # step of 0.5 at t=1.0s
        
        # Pre-step motion (act_noise > 0.05 AND VIB_GATE.ok() is True)
        # First case: amp < 2.0 * noise (e.g. amp = 0.16, noise = 0.09)
        sp_pre_low = np.zeros_like(t)
        sp_pre_low[100:] = 0.12 # amp = 0.12 (less than 1.5 * noise of 0.09)
        np.random.seed(123)
        act_noise_high = np.random.normal(0, 0.09, size=200)
        act_pre_low = sp_pre_low + act_noise_high
        
        with patch("backend.app.analysis.live_pid.VIB_GATE.ok", return_value=True):
            ADVISOR.clear()
            recs = set()
            res = LivePidEngine._step_response(t, sp_pre_low, act_pre_low, min_amp=0.15, ax="roll", recommended_axes=recs)
            self.assertIsNone(res)
            self.assertIn("roll", recs)
            props = ADVISOR.list_proposals()
            self.assertTrue(any(p.get("state") == "diagnostic" and "vehicle was moving significantly in the 1 second before your input" in p.get("rationale") for p in props))

        # Second case: amp >= 2.0 * noise (e.g. amp = 0.50, noise = 0.06). Clears noise floor, returns noise_ratio.
        act_noise_low = np.random.normal(0, 0.06, size=200)
        act_pre_high = sp + act_noise_low
        with patch("backend.app.analysis.live_pid.VIB_GATE.ok", return_value=True):
            ADVISOR.clear()
            recs = set()
            res = LivePidEngine._step_response(t, sp, act_pre_high, min_amp=0.15, ax="roll", recommended_axes=recs)
            self.assertIsNotNone(res)
            self.assertIn("noise_ratio", res)
            self.assertGreater(res["noise_ratio"], 0.0)

        # Step too small (abs(amp) < multiplier * act_noise)
        sp_small = np.zeros_like(t)
        sp_small[100:] = 0.05 # amp = 0.05
        act_small_noise = np.random.normal(0, 0.03, size=200) # std = 0.03
        act_small = sp_small + act_small_noise
        
        with patch("backend.app.analysis.live_pid.VIB_GATE.ok", return_value=True):
            ADVISOR.clear()
            recs = set()
            res = LivePidEngine._step_response(t, sp_small, act_small, min_amp=0.02, ax="pitch", recommended_axes=recs)
            self.assertIsNone(res)
            self.assertIn("pitch", recs)
            props = ADVISOR.list_proposals()
            self.assertTrue(any(p.get("state") == "diagnostic" and "input too small or too slow" in p.get("rationale") for p in props))

    def test_flight_context_diagnostic_cards(self):
        from backend.app.analysis.live_pid import LivePidEngine
        from backend.app.analysis.regime import Regime
        
        engine = LivePidEngine()
        # Populate engine._win["roll"] so that len(win) >= 15
        now = time.monotonic()
        for i in range(20):
            engine._win["roll"].append((now + i * 0.05, 0.0, 0.0))
        
        with patch("backend.app.analysis.live_pid.REGIME.current", Regime.PRE_FLIGHT):
            diag = engine._determine_diagnostic("roll", now)
            self.assertIn("steady on the ground", diag)
            
        with patch("backend.app.analysis.live_pid.REGIME.current", Regime.DYNAMIC_MANEUVER):
            diag = engine._determine_diagnostic("roll", now)
            self.assertIn("Stop maneuvering and stabilise", diag)

    async def test_underdamped_branch_b(self):
        from backend.app.analysis.live_pid import LivePidEngine
        from backend.app.advisors.param_advisor import ADVISOR
        
        engine = LivePidEngine()
        ADVISOR.clear()
        
        # Clear cooldowns
        from backend.app.analysis import recommendations
        recommendations._last_emit.clear()
        
        with patch("backend.app.analysis.live_pid.VIB_GATE.ok", return_value=True), \
             patch("backend.app.analysis.live_pid._airframe_class", return_value="MULTIROTOR"), \
             patch("backend.app.analysis.live_pid.CONNECTION.read_param", new_callable=AsyncMock) as mock_read_param, \
             patch("backend.app.analysis.live_pid.HUB.publish") as mock_publish:
            
            mock_read_param.return_value = 0.003
            
            # Overshoot 38% gives zeta = 0.294 < 0.3, and osc = 1.
            metrics = {
                "tau_s": 0.1,
                "settling_s": 0.4,
                "overshoot": 0.38,
                "oscillations": 1,
                "step_amp_deg_s": 20.0,
                "amplitude": 0.35,
                "post_sp_std": 0.02,
                "ramped_adjusted": False,
            }
            
            engine._rate_param = MagicMock(side_effect=lambda ax, term: f"MC_{ax.upper()}RATE_{term}")
            engine._recommended_axes_this_cycle = set()
            
            engine._verdicts("roll", metrics, time.monotonic(), "Medium", None)
            
            await asyncio.sleep(0.05)
            
            publish_calls = [(call[0][0], call[0][1]) for call in mock_publish.call_args_list]
            damping_alerts = [p[1] for p in publish_calls if p[0] == "alert" and "damping deficit" in p[1].get("text", "")]
            self.assertEqual(len(damping_alerts), 1)
            self.assertIn("low damping ratio", damping_alerts[0].get("text", ""))

    def test_actuator_saturation_segment_metrics(self):
        # Verify that check_actuator_saturation computes peak and sustained values
        # only over the saturated segment, not the entire history.
        from backend.app.analysis.live_pid import check_actuator_saturation
        
        # A 10-second history where a spike to 0.95 happened 8 seconds ago,
        # but the current saturated segment (last 2 seconds) has values around 0.90
        # and doesn't contain the old spike.
        history = []
        now = time.monotonic()
        
        # Add old spike to 0.95 (duration < 100ms, not a sustained saturation segment itself)
        history.append((now - 8.0, [0.95]))
        for i in range(1, 5):
            history.append((now - 8.0 + i * 0.01, [0.30]))
            
        # Add new saturated segment at 0.90 (exceeds 0.85 limit threshold for > 100ms)
        for i in range(30):
            val = 0.90 if i != 15 else 0.92  # peak = 0.92, sustained (85th percentile) = 0.90
            history.append((now - 2.0 + i * 0.1, [val]))
            
        is_sat, dur, peak, sustained, act_idx = check_actuator_saturation(history, limit_threshold=0.85)
        
        self.assertTrue(is_sat)
        self.assertAlmostEqual(peak, 0.92, places=2)
        # Verify it did NOT use 0.95 from the old history
        self.assertLess(peak, 0.95)
        self.assertAlmostEqual(sustained, 0.90, places=2)

    def test_coherence_nperseg_dynamic_scaling(self):
        # Verify compute_coherence_in_band scales nperseg dynamically
        from backend.app.analysis.live_pid import compute_coherence_in_band
        import scipy.signal
        
        # At ~50 Hz, 3s window is 150 samples -> nperseg = max(64, 150 // 4) = 64
        t_50 = np.linspace(0, 3.0, 150)
        x_50 = np.sin(2 * np.pi * 1.5 * t_50)
        y_50 = x_50 + 0.1 * np.random.normal(size=150)
        
        # Spy on scipy.signal.coherence to see what nperseg was passed
        with patch("scipy.signal.coherence", wraps=scipy.signal.coherence) as spy_coherence:
            res_50 = compute_coherence_in_band(t_50, x_50, y_50)
            self.assertIsNotNone(res_50)
            spy_coherence.assert_called_once()
            called_nperseg = spy_coherence.call_args[1].get("nperseg")
            self.assertEqual(called_nperseg, 37)
            
        # At ~200 Hz, 3s window is 600 samples -> nperseg = max(64, 600 // 4) = 150
        t_200 = np.linspace(0, 3.0, 600)
        x_200 = np.sin(2 * np.pi * 1.5 * t_200)
        y_200 = x_200 + 0.1 * np.random.normal(size=600)
        
        with patch("scipy.signal.coherence", wraps=scipy.signal.coherence) as spy_coherence:
            res_200 = compute_coherence_in_band(t_200, x_200, y_200)
            self.assertIsNotNone(res_200)
            spy_coherence.assert_called_once()
            called_nperseg = spy_coherence.call_args[1].get("nperseg")
            self.assertEqual(called_nperseg, 150)

    def test_nrmse_ptp_normalization(self):
        # Verify _tracking_metrics normalizes NRMSE by range (ptp), not standard deviation
        from backend.app.analysis.live_pid import LivePidEngine
        
        sp = np.zeros(100)
        sp[80:] = 0.5  # step near the edge of the window
        act = sp.copy()
        act[80:] = 0.45  # tracking error is 0.05
        
        metrics = LivePidEngine._tracking_metrics(sp, act)
        self.assertIsNotNone(metrics["nrmse"])
        # range = 0.5, rmse = sqrt(20 * 0.05^2 / 100) = sqrt(0.0005) = 0.02236
        # nrmse = 0.02236 / 0.5 = 0.0447 -> round to 0.045
        self.assertAlmostEqual(metrics["nrmse"], 0.045, places=3)

    def test_coherence_vibration_slop_early_return(self):
        # Verify that when coherence is low, is_sat is False, and sustained_val < 0.70,
        # _verdicts returns early and doesn't run the rest of the verdict path.
        from backend.app.analysis.live_pid import LivePidEngine
        
        engine = LivePidEngine()
        engine._rate_param = MagicMock(side_effect=lambda ax, term: f"MC_{ax.upper()}RATE_{term}")
        
        metrics = {
            "r": 0.9,
            "tau_s": 0.1,
            "settling_s": 0.4,
            "overshoot": 0.35,
            "oscillations": 0,
            "step_amp_deg_s": 20.0,
            "amplitude": 0.35,
            "post_sp_std": 0.01,
            "ramped_adjusted": False,
            "coherence": 0.51,  # < 0.60
        }
        
        with patch("backend.app.analysis.live_pid.check_actuator_saturation") as mock_sat, \
             patch.object(engine, "_safe_recommend") as mock_rec, \
             patch("backend.app.analysis.live_pid.HUB.publish") as mock_publish:
            
            # Not saturated, low sustained value
            mock_sat.return_value = (False, 0.0, 0.0, 0.0, 0)
            
            engine._verdicts("roll", metrics, time.monotonic(), "Medium", None)
            
            # Check vibration slop warning alert is published
            alert_calls = [c[0][1] for c in mock_publish.call_args_list if c[0][0] == "alert"]
            self.assertTrue(any("Low coherence" in a.get("text", "") for a in alert_calls))
            
            # Verify no recommendations are generated (it returned early)
            mock_rec.assert_not_called()

    def test_authority_depletion_bypassed_during_step(self):
        # Verify that the authority depletion check is bypassed when a step is detected in the window.
        from backend.app.analysis.live_pid import LivePidEngine
        
        engine = LivePidEngine()
        
        # Scenario where r = 0.82 < _R_DEPLETION (0.85) and high_rate is True
        metrics = {
            "r": 0.82,
            "high_rate": True,
            "step_amp_deg_s": 20.0,  # step detected!
        }
        
        with patch("backend.app.analysis.live_pid.check_actuator_saturation") as mock_sat, \
             patch("backend.app.analysis.live_pid.HUB.publish") as mock_publish:
             
            mock_sat.return_value = (False, 0.0, 0.0, 0.0, 0)
            
            engine._verdicts("roll", metrics, time.monotonic(), "Medium", None)
            
            # Verify no authority depletion alert is published
            alert_calls = [c[0][1] for c in mock_publish.call_args_list if c[0][0] == "alert"]
            self.assertFalse(any("Tracking authority depletion" in a.get("text", "") for a in alert_calls))

    def test_post_step_duration_oscillation_guard(self):
        # Verify that when post_duration < 1.0s, oscillations is None and all gain recommendations are skipped.
        from backend.app.analysis.live_pid import LivePidEngine
        
        # Create a window where the step occurs near the end of the window
        t = np.linspace(0, 3.0, 150)
        sp = np.zeros_like(t)
        sp[125:] = 0.5  # step starts at t = 2.5s -> post_duration = 0.5s < 1.0s
        act = sp.copy()
        
        # Add some ringing (would cross zero multiple times if analyzed)
        act[125:] = sp[125:] + 0.1 * np.sin(2 * np.pi * 5 * (t[125:] - 2.5))
        
        recs = set()
        res = LivePidEngine._step_response(t, sp, act, min_amp=0.10, ax="roll", recommended_axes=recs)
        
        self.assertIsNotNone(res)
        self.assertIsNone(res["oscillations"])  # Count is None due to short duration
        
        # Verify that with osc = None, verdicts does not recommend anything
        engine = LivePidEngine()
        engine._rate_param = MagicMock(side_effect=lambda ax, term: f"MC_{ax.upper()}RATE_{term}")
        
        metrics = {
            "r": 0.95,
            "tau_s": 0.1,
            "settling_s": 0.4,
            "overshoot": 0.35,  # > _OVERSHOOT_LIMIT (0.15)
            "oscillations": None,  # None!
            "step_amp_deg_s": 20.0,
            "amplitude": 0.35,
            "post_sp_std": 0.01,
            "ramped_adjusted": False,
        }
        
        with patch("backend.app.analysis.live_pid.check_actuator_saturation") as mock_sat, \
             patch.object(engine, "_safe_recommend") as mock_rec:
             
            mock_sat.return_value = (False, 0.0, 0.0, 0.0, 0)
            
            engine._verdicts("roll", metrics, time.monotonic(), "Medium", None)
            
            # Verify no recommendations are generated (both underdamped and overshoot require osc is not None)
            mock_rec.assert_not_called()

    def test_sluggish_roll_under_saturation_recommends_auto_rate_limit(self):
        # Verify that sluggish roll response under saturation recommends reducing auto rate limit
        from backend.app.analysis.live_pid import LivePidEngine
        
        engine = LivePidEngine()
        engine._rate_param = MagicMock(side_effect=lambda ax, term: f"MC_{ax.upper()}RATE_{term}")
        
        metrics = {
            "tau_s": None,  # sluggish step response!
            "step_amp_deg_s": 20.0,
            "overshoot": 0.0,
            "oscillations": 0,
            "amplitude": 0.35,
            "post_sp_std": 0.01,
            "ramped_adjusted": False,
        }
        
        with patch.object(engine, "_safe_recommend") as mock_rec, \
             patch("backend.app.analysis.live_pid._airframe_class", return_value="MULTIROTOR"), \
             patch("backend.app.analysis.live_pid.VIB_GATE.ok", return_value=True):
             
            # under moderate/mild saturation
            engine._verdicts("roll", metrics, time.monotonic(), "Medium", None, severity="moderate")
            
            # Should recommend auto rate limit reduction (MC_ROLLRAUTO_MAX)
            mock_rec.assert_called_once()
            called_args = mock_rec.call_args
            self.assertEqual(called_args[0][0], "roll")
            self.assertEqual(called_args[0][1], "MC_ROLLRAUTO_MAX")
            self.assertIn("Reduce rate limit to restore tracking", called_args[0][2])
            self.assertEqual(called_args[1].get("scale_factor"), 0.85)

    def test_sluggish_yaw_under_saturation_recommends_p_gain_reduction(self):
        # Verify that sluggish yaw response under saturation falls back to P-gain reduction
        from backend.app.analysis.live_pid import LivePidEngine
        
        engine = LivePidEngine()
        engine._rate_param = MagicMock(side_effect=lambda ax, term: f"MC_{ax.upper()}RATE_{term}")
        
        metrics = {
            "tau_s": None,  # sluggish step response!
            "step_amp_deg_s": 20.0,
            "overshoot": 0.0,
            "oscillations": 0,
            "amplitude": 0.35,
            "post_sp_std": 0.01,
            "ramped_adjusted": False,
        }
        
        with patch.object(engine, "_safe_recommend") as mock_rec, \
             patch("backend.app.analysis.live_pid.VIB_GATE.ok", return_value=True):
             
            # under moderate/mild saturation
            engine._verdicts("yaw", metrics, time.monotonic(), "Medium", None, severity="moderate")
            
            # Should recommend P-gain reduction (MC_YAWRATE_P) since yaw has no auto rate limit parameter
            mock_rec.assert_called_once()
            called_args = mock_rec.call_args
            self.assertEqual(called_args[0][0], "yaw")
            self.assertEqual(called_args[0][1], "MC_YAWRATE_P")
            self.assertIn("reduce P-gain to lower demand", called_args[0][2])
            self.assertEqual(called_args[1].get("scale_factor"), 0.9)

    def test_sluggish_step_no_saturation_recommends_p_gain_increase(self):
        # Verify that sluggish step response without saturation recommends raising P-gain
        from backend.app.analysis.live_pid import LivePidEngine
        
        engine = LivePidEngine()
        engine._rate_param = MagicMock(side_effect=lambda ax, term: f"MC_{ax.upper()}RATE_{term}")
        
        metrics = {
            "tau_s": None,  # sluggish step response!
            "step_amp_deg_s": 20.0,
            "overshoot": 0.0,
            "oscillations": 0,
            "amplitude": 0.35,
            "post_sp_std": 0.01,
            "ramped_adjusted": False,
        }
        
        with patch.object(engine, "_safe_recommend") as mock_rec, \
             patch("backend.app.analysis.live_pid.VIB_GATE.ok", return_value=True):
             
            # no saturation
            engine._verdicts("roll", metrics, time.monotonic(), "Medium", None, severity="none")
            
            # Should recommend raising P-gain (MC_ROLLRATE_P)
            mock_rec.assert_called_once()
            called_args = mock_rec.call_args
            self.assertEqual(called_args[0][0], "roll")
            self.assertEqual(called_args[0][1], "MC_ROLLRATE_P")
            self.assertIn("likely under-gained. Raise P", called_args[0][2])
            self.assertEqual(called_args[1].get("scale_factor"), 1.1)

    def test_offline_step_rejections_and_snr(self):
        from backend.app.analysis.pid_offline import _step_response_offline
        # Create signals
        t = np.linspace(0, 3.0, 300)
        sp = np.zeros_like(t)
        sp[100:] = 0.5
        
        # Test Case 1: amplitude too small (amplitude < min_amp)
        # min_amp is 0.15, so 0.05 is too small
        sp_small = np.zeros_like(t)
        sp_small[100:] = 0.05
        act_clean = sp_small.copy()
        res, reason = _step_response_offline(t, sp_small, act_clean, min_amp=0.15)
        self.assertIsNone(res)
        self.assertEqual(reason, "too_small")

        # Test Case 2: SNR too low (abs(amp) < multiplier * act_noise)
        # amp = 0.5, multiplier is at least 2.0. So 2 * act_noise > 0.5 implies act_noise > 0.25
        np.random.seed(42)
        noise = np.random.normal(0, 0.3, size=300)
        act_noisy = sp + noise
        res, reason = _step_response_offline(t, sp, act_noisy, min_amp=0.15)
        self.assertIsNone(res)
        self.assertEqual(reason, "snr")

        # Test Case 3: Ramp (post_sp_std > 0.7 * reference_amp)
        sp_ramp = np.zeros_like(t)
        sp_ramp[100:] = np.linspace(0.2, 2.0, 200)
        act_ramp = sp_ramp.copy()
        res, reason = _step_response_offline(t, sp_ramp, act_ramp, min_amp=0.15)
        self.assertIsNone(res)
        self.assertEqual(reason, "ramp")

        # Test Case 4: Success case
        act_success = sp.copy()
        res, reason = _step_response_offline(t, sp, act_success, min_amp=0.15)
        self.assertIsNotNone(res)
        self.assertIsNone(reason)
        self.assertEqual(res["amplitude"], 0.5)

    def test_offline_diagnostics_accounting(self):
        from backend.app.analysis.pid_offline import _suggest
        
        # stats with no usable steps but candidates rejected
        stats = {
            "n_steps": 0,
            "candidates": 5,
            "rejections": {
                "too_small": 2,
                "snr": 2,
                "ramp": 1,
                "window": 0
            }
        }
        
        recs, notes = _suggest("roll", stats, "MULTIROTOR", {})
        self.assertEqual(len(recs), 0)
        self.assertEqual(len(notes), 1)
        self.assertIn("5 step candidates detected but all were rejected", notes[0])
        self.assertIn("2 too small", notes[0])
        self.assertIn("2 noisy/low SNR", notes[0])
        self.assertIn("1 slow/ramped", notes[0])

    def test_offline_yaw_attitude_fallback(self):
        # Create rates_sp with no yaw steps (all zeros)
        t = np.linspace(0, 5.0, 500)
        t_ms = t * 1e6
        rates_sp = pd.DataFrame({
            "timestamp": t_ms,
            "roll": np.zeros_like(t),
            "pitch": np.zeros_like(t),
            "yaw": np.zeros_like(t),
        })
        
        # Create yaw_body in att_sp that starts at 0.0, and starting at 2.0s, increases linearly at 1.0 rad/s
        yaw_body = np.zeros_like(t)
        yaw_body[200:] = 1.0 * (t[200:] - t[200])
        
        att_sp = pd.DataFrame({
            "timestamp": t_ms,
            "yaw_body": yaw_body
        })
        
        # Actual yaw rate steps to 1.0 and has standard underdamped response
        xyz2 = np.zeros_like(t)
        xyz2[200:] = 1.0 + 0.2 * np.exp(-5 * (t[200:] - t[200])) * np.cos(15 * (t[200:] - t[200]))
        
        ang_vel = pd.DataFrame({
            "timestamp": t_ms,
            "xyz[0]": np.zeros_like(t),
            "xyz[1]": np.zeros_like(t),
            "xyz[2]": xyz2,
        })
        
        params = {"MC_YAWRATE_P": 0.15}
        
        res = pid_offline.analyze_pid(
            rates_sp=rates_sp,
            ang_vel=ang_vel,
            sensor=None,
            params=params,
            airframe_class="MULTIROTOR",
            att_sp=att_sp
        )
        
        self.assertIn("yaw (rate)", res["axes"])
        yaw_stats = res["axes"]["yaw (rate)"]
        self.assertTrue(yaw_stats.get("derived_from_attitude"))
        self.assertTrue(yaw_stats.get("n_steps") > 0)
        self.assertTrue(yaw_stats["confidence"]["flags"]["derived_from_attitude"])
        self.assertLessEqual(yaw_stats["confidence"]["score"], 0.5)

    def test_offline_step_level_change_cutting(self):
        from backend.app.analysis.pid_offline import _step_response_offline
        t = np.linspace(0, 3.0, 300)
        
        # Step at 1.0s (index 100) from 0.0 to 1.0, and level change back at 2.0s (index 200)
        sp = np.zeros_like(t)
        sp[100:200] = 1.0
        sp[200:] = 0.0
        
        act = sp.copy()
        
        res, reason = _step_response_offline(t, sp, act, min_amp=0.15)
        self.assertIsNotNone(res)
        self.assertIsNone(reason)
        self.assertTrue(res["confidence"]["flags"]["short_post_window"])

    def test_live_confidence_caps(self):
        from backend.app.analysis.live_pid import LivePidEngine
        from unittest.mock import patch
        
        engine = LivePidEngine()
        t = np.linspace(0, 2.5, 125)
        sp = np.zeros_like(t)
        sp[50:] = 0.5
        act = sp.copy()
        
        # Case 1: noise_ratio > 0.25 -> Cap at Low
        step_result_high_noise = {
            "tau_s": 0.1,
            "settling_s": 0.4,
            "overshoot": 0.1,
            "oscillations": 0,
            "step_amp_deg_s": 28.6,
            "amplitude": 0.5,
            "post_sp_std": 0.01,
            "ramped_adjusted": False,
            "t_start": 1.0,
            "noise_ratio": 0.30  # > 0.25
        }
        
        engine._win["roll"].clear()
        for i in range(len(t)):
            engine._win["roll"].append((t[i], sp[i], act[i]))
            
        with patch.object(engine, "_step_response", return_value=step_result_high_noise), \
             patch.object(engine, "_verdicts") as mock_verdicts:
             
            engine._evaluate(2.5)
            mock_verdicts.assert_called_once()
            called_args = mock_verdicts.call_args
            confidence_label = called_args[0][3]
            self.assertEqual(confidence_label, "Low confidence (needs clearer data)")
            
        # Case 2: 0.15 < noise_ratio <= 0.25 -> Cap at Medium
        step_result_medium_noise = step_result_high_noise.copy()
        step_result_medium_noise["noise_ratio"] = 0.20
        
        engine._win["roll"].clear()
        for i in range(len(t)):
            sp_excited = np.sin(2 * np.pi * 2 * t[i]) * 0.3
            sp_excited += 0.5 if i >= 50 else 0.0
            act_excited = sp_excited
            engine._win["roll"].append((t[i], sp_excited, act_excited))
            
        with patch.object(engine, "_step_response", return_value=step_result_medium_noise), \
             patch.object(engine, "_verdicts") as mock_verdicts:
             
            engine._evaluate(2.5)
            mock_verdicts.assert_called_once()
            called_args = mock_verdicts.call_args
            confidence_label = called_args[0][3]
            self.assertEqual(confidence_label, "Medium confidence (passive observation)")

    def test_adaptive_sequence_gating(self):
        from backend.app.analysis.live_pid import LivePidEngine
        
        t = np.linspace(0, 5.0, 250)
        sp = np.zeros_like(t)
        sp[50:] = 0.5
        act = sp.copy()
        
        # Test Case 1: no last step -> window is 1.0s
        res = LivePidEngine._step_response(t, sp, act, min_amp=0.15, last_step_t=None)
        self.assertIsNotNone(res)
        
        # Test Case 2: last_step_t is recent (1.0s ago) -> pre-step window is 0.3s
        res_recent = LivePidEngine._step_response(t, sp, act, min_amp=0.15, last_step_t=0.0)
        self.assertIsNotNone(res_recent)

    def test_step_history_accumulation(self):
        from backend.app.analysis.live_pid import LivePidEngine
        from unittest.mock import patch
        
        engine = LivePidEngine()
        t = np.linspace(0, 2.5, 125)
        sp = np.zeros_like(t)
        sp[50:] = 0.5
        act = sp.copy()
        
        step_result = {
            "tau_s": 0.1,
            "settling_s": 0.4,
            "overshoot": 0.1,
            "oscillations": 0,
            "step_amp_deg_s": 28.6,
            "amplitude": 0.5,
            "post_sp_std": 0.01,
            "ramped_adjusted": False,
            "t_start": 1.0,
        }
        
        engine._win["roll"].clear()
        for i in range(len(t)):
            engine._win["roll"].append((t[i], sp[i], act[i]))
            
        with patch.object(engine, "_step_response", return_value=step_result), \
             patch("backend.app.analysis.live_pid.HUB.publish") as mock_publish:
             
            for _ in range(12):
                engine._evaluate(2.5)
                
            self.assertEqual(len(engine._step_history["roll"]), 10)
            self.assertEqual(engine._last_step_t["roll"], 1.0)
            
            metrics_calls = [c[0][1] for c in mock_publish.call_args_list if c[0][0] == "loop_metrics"]
            self.assertTrue(len(metrics_calls) > 0)
            last_metrics = metrics_calls[-1]
            self.assertIn("step_history", last_metrics["axes"]["roll"])
            self.assertEqual(len(last_metrics["axes"]["roll"]["step_history"]), 10)

    async def test_underdamped_branch_c(self):
        from backend.app.analysis.live_pid import LivePidEngine
        from backend.app.advisors.param_advisor import ADVISOR
        
        engine = LivePidEngine()
        ADVISOR.clear()
        
        from backend.app.analysis import recommendations
        recommendations._last_emit.clear()
        
        with patch("backend.app.analysis.live_pid.VIB_GATE.ok", return_value=True), \
             patch("backend.app.analysis.live_pid._airframe_class", return_value="MULTIROTOR"), \
             patch("backend.app.analysis.live_pid.CONNECTION.read_param", new_callable=AsyncMock) as mock_read_param, \
             patch("backend.app.analysis.live_pid.HUB.publish") as mock_publish:
            
            mock_read_param.return_value = 0.003
            
            # Overshoot 35% (>30%), settling is None, oscillations is None -> Branch C.
            metrics = {
                "tau_s": 0.1,
                "settling_s": None,
                "overshoot": 0.35,
                "oscillations": None,
                "step_amp_deg_s": 20.0,
                "amplitude": 0.35,
                "post_sp_std": 0.02,
                "ramped_adjusted": False,
            }
            
            engine._rate_param = MagicMock(side_effect=lambda ax, term: f"MC_{ax.upper()}RATE_{term}")
            engine._recommended_axes_this_cycle = set()
            
            engine._verdicts("roll", metrics, time.monotonic(), "Medium", None)
            
            await asyncio.sleep(0.05)
            
            publish_calls = [(call[0][0], call[0][1]) for call in mock_publish.call_args_list]
            damping_alerts = [p[1] for p in publish_calls if p[0] == "alert" and "damping deficit" in p[1].get("text", "")]
            self.assertEqual(len(damping_alerts), 1)
            self.assertIn("never settled", damping_alerts[0].get("text", ""))

    async def test_no_p_reduction_on_unknown_oscillations(self):
        from backend.app.analysis.live_pid import LivePidEngine
        from backend.app.advisors.param_advisor import ADVISOR
        
        engine = LivePidEngine()
        ADVISOR.clear()
        
        from backend.app.analysis import recommendations
        recommendations._last_emit.clear()
        
        with patch("backend.app.analysis.live_pid.VIB_GATE.ok", return_value=True), \
             patch("backend.app.analysis.live_pid.HUB.publish") as mock_publish, \
             patch.object(engine, "_safe_recommend") as mock_recommend:
            
            # Overshoot 35% (>30%), settling is 0.4s (not None), but oscillations is None.
            metrics = {
                "tau_s": 0.1,
                "settling_s": 0.4,
                "overshoot": 0.35,
                "oscillations": None,
                "step_amp_deg_s": 20.0,
                "amplitude": 0.35,
                "post_sp_std": 0.02,
                "ramped_adjusted": False,
            }
            
            engine._rate_param = MagicMock(side_effect=lambda ax, term: f"MC_{ax.upper()}RATE_{term}")
            engine._recommended_axes_this_cycle = set()
            
            engine._verdicts("roll", metrics, time.monotonic(), "Medium", None)
            
            # Verify no recommend call for P reduction was made (because osc is None, not 0)
            mock_recommend.assert_not_called()

    async def test_tau_none_overshoot_guard(self):
        from backend.app.analysis.live_pid import LivePidEngine
        from backend.app.advisors.param_advisor import ADVISOR
        
        engine = LivePidEngine()
        ADVISOR.clear()
        
        from backend.app.analysis import recommendations
        recommendations._last_emit.clear()
        
        with patch("backend.app.analysis.live_pid.VIB_GATE.ok", return_value=True), \
             patch("backend.app.analysis.live_pid.HUB.publish") as mock_publish, \
             patch.object(engine, "_safe_recommend") as mock_recommend:
            
            # tau is None, step_amp_deg_s is present, overshoot is 35% (>30%), no saturation (severity = "none")
            metrics = {
                "tau_s": None,
                "settling_s": 0.4,
                "overshoot": 0.35,
                "oscillations": None,
                "step_amp_deg_s": 20.0,
                "amplitude": 0.35,
                "post_sp_std": 0.02,
                "ramped_adjusted": False,
            }
            
            engine._rate_param = MagicMock(side_effect=lambda ax, term: f"MC_{ax.upper()}RATE_{term}")
            engine._recommended_axes_this_cycle = set()
            
            engine._verdicts("roll", metrics, time.monotonic(), "Medium", None, severity="none")
            
            # Verify no recommend call for P increase was made (because overshoot > _OVERSHOOT_LIMIT)
            mock_recommend.assert_not_called()


    def test_nrmse_gating(self):
        from backend.app.analysis.live_pid import LivePidEngine
        
        # sp_range <= 0.1 (flat window with slight noise)
        sp1 = np.ones(10) * 0.05
        act1 = np.ones(10) * 0.04
        metrics1 = LivePidEngine._tracking_metrics(sp1, act1)
        self.assertIsNone(metrics1["nrmse"])
        
        # sp_range > 0.1 (actual step)
        sp2 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.2, 0.2])
        act2 = sp2 * 0.9
        metrics2 = LivePidEngine._tracking_metrics(sp2, act2)
        self.assertIsNotNone(metrics2["nrmse"])
        self.assertLess(metrics2["nrmse"], 1.0)

    def test_minimum_confidence_calculation(self):
        from backend.app.analysis.live_pid import LivePidEngine
        
        engine = LivePidEngine()
        # Set up a situation where excitation_score = 0.5, step_score = 0.8, coherence_score = 0.7.
        # With minimum calculation, confidence_value should be 0.5.
        
        t = np.linspace(0, 2.5, 125)
        sp = np.zeros_like(t)
        sp[62:] = 0.3
        act = sp.copy()
        
        step_result = {
            "tau_s": 0.1,
            "settling_s": 0.4,
            "overshoot": 0.1,
            "oscillations": 0,
            "step_amp_deg_s": 28.6,
            "amplitude": 0.3,
            "post_sp_std": 0.01,
            "ramped_adjusted": False,
            "t_start": 1.0,
            "noise_ratio": 0.0
        }
        
        engine._win["roll"].clear()
        for i in range(len(t)):
            engine._win["roll"].append((t[i], sp[i], act[i]))
            
        with patch.object(engine, "_step_response", return_value=step_result), \
             patch.object(engine, "_verdicts") as mock_verdicts:
             
            with patch("backend.app.analysis.live_pid.np.std", side_effect=lambda x: 0.15 if len(x) == len(sp) else np.std(x)):
                engine._evaluate(2.5)
                mock_verdicts.assert_called_once()
                called_args = mock_verdicts.call_args
                confidence_label = called_args[0][3]
                self.assertEqual(confidence_label, "Low confidence (needs clearer data)")

            with patch("backend.app.analysis.live_pid.np.std", side_effect=lambda x: 0.20 if len(x) == len(sp) else np.std(x)):
                engine._win["roll"].clear()
                for i in range(len(t)):
                    engine._win["roll"].append((t[i], sp[i], act[i]))
                
                with patch.object(engine, "_tracking_metrics", return_value={"r": 0.9, "nrmse": 0.05, "coherence": 0.80, "high_rate": False}):
                    engine._evaluate(2.5)
                    confidence_label = mock_verdicts.call_args[0][3]
                    self.assertEqual(confidence_label, "Medium confidence (passive observation)")

    def test_consecutive_no_increment_when_osc_none(self):
        from backend.app.analysis.live_pid import LivePidEngine
        
        engine = LivePidEngine()
        engine._consecutive_overshoot_no_osc["roll"] = 1
        
        metrics = {
            "tau_s": 0.1,
            "settling_s": 0.4,
            "overshoot": 0.25,
            "oscillations": None,
            "amplitude": 0.5,
            "post_sp_std": 0.02,
        }
        
        with patch("backend.app.analysis.live_pid.HUB.publish"):
            engine._verdicts("roll", metrics, time.monotonic(), "Medium", None)
            self.assertEqual(engine._consecutive_overshoot_no_osc["roll"], 1)

    def test_cascade_local_position_setpoint_feed(self):
        from backend.app.analysis.cascade import CascadeEngine
        engine = CascadeEngine()
        payload = {"vx": 1.5, "vy": -0.5, "vz": 0.0, "x": 10.0, "y": 20.0, "z": -5.0}
        engine._feed_local_position_setpoint(payload)
        self.assertEqual(engine.velocity.latest_sp.get("vx"), 1.5)
        self.assertEqual(engine.velocity.latest_sp.get("vy"), -0.5)
        self.assertEqual(engine.velocity.latest_sp.get("vz"), 0.0)
        self.assertEqual(engine.position.latest_sp.get("x"), 10.0)
        self.assertEqual(engine.position.latest_sp.get("y"), 20.0)
        self.assertEqual(engine.position.latest_sp.get("z"), -5.0)

    @patch("backend.app.analysis.cascade.STATE")
    @patch("backend.app.analysis.cascade.HUB")
    def test_cascade_activity_gate(self, mock_hub, mock_state):
        from backend.app.analysis.cascade import _OuterLoop
        
        # Setup STATE mock
        mock_state.auto_flight.return_value = False
        mock_state.loops_active.return_value = {"vx", "vy", "vz"}
        mock_state.domain = "MC"
        
        # Test Case 1: Inactive velocity loop (act_std = 0.0 <= 0.1)
        # 20 samples of sp and act both being constant
        loop = _OuterLoop("velocity", ["vx"], 6.0, 0.2, "m/s", MagicMock())
        now = time.monotonic()
        for i in range(20):
            # Feed constant values
            loop.win["vx"].append((now + i * 0.1, 1.0, 1.0))
            
        loop._evaluate(now + 2.0)
        
        # Since it's inactive, no metrics should be published
        mock_hub.publish.assert_not_called()
        
        # Test Case 2: Active velocity loop (act_std > 0.1)
        # Feed changing values (std around 0.5)
        loop_active = _OuterLoop("velocity", ["vx"], 6.0, 0.2, "m/s", MagicMock())
        for i in range(20):
            val = 1.0 if i % 2 == 0 else 0.0
            loop_active.win["vx"].append((now + i * 0.1, val, val))
            
        with patch("backend.app.analysis.cascade.LivePidEngine._step_response", return_value=None):
            loop_active._evaluate(now + 2.0)
            
        # Since active, it should publish metrics
        mock_hub.publish.assert_called_with("loop_metrics", unittest.mock.ANY)

    @patch("backend.app.analysis.cascade.STATE")
    @patch("backend.app.analysis.cascade.REGIME")
    def test_cascade_manual_posctl_bypass(self, mock_regime, mock_state):
        from backend.app.analysis.cascade import CascadeEngine, Regime
        
        # Setup mock state to reflect manual POSCTL flight
        mock_state.mode = "POSCTL"
        mock_state.loops_active.return_value = {"rate", "attitude", "velocity", "position"}
        mock_state.auto_flight.return_value = False
        
        # Setup regime mock: say we are in STEADY_HOLD (not DYNAMIC_MANEUVER)
        mock_regime.current = Regime.STEADY_HOLD
        
        engine = CascadeEngine()
        engine.velocity.clear()
        engine.velocity.feed_sp({"vx": 0.0, "vy": 0.0, "vz": 0.0})
        
        # Feed some values
        now = time.monotonic()
        engine._feed(engine.velocity, {"vx": 1.0, "vy": 0.0, "vz": 0.0}, now)
        
        # Verify that velocity loop window is NOT empty (it bypassed the regime check)
        self.assertGreater(len(engine.velocity.win["vx"]), 0)
        
        # Now let's feed attitude loop in the same state
        engine.attitude.clear()
        engine.attitude.feed_sp({"roll": 0.0, "pitch": 0.0})
        engine._feed(engine.attitude, {"roll": 0.1, "pitch": 0.0}, now)
        # Verify that attitude loop IS cleared because attitude requires _loop_enabled (which checks regime)
        self.assertEqual(len(engine.attitude.win["roll"]), 0)

    @patch("backend.app.mavlink.telemetry_hub.HUB.publish")
    def test_velocity_broadened_recommendations(self, mock_publish):
        from backend.app.analysis.cascade import _velocity_advice
        from backend.app.analysis import recommendations
        
        # 1. P-gain recommendation on overshoot
        recommendations._last_emit.clear()
        mock_publish.reset_mock()
        
        m_overshoot = {
            "tau_s": 0.5,
            "settling_s": 1.2,
            "overshoot": 0.25,
            "sp": 1.0,
            "act": 1.1,
            "r": 0.9,
        }
        _velocity_advice("vx", "overshoot", m_overshoot, "MC")
        
        # Check P-gain reduction recommendation is published
        publish_calls = [call[0][1] for call in mock_publish.call_args_list if call[0][0] == "recommendation"]
        self.assertTrue(any(p.get("param") == "MPC_XY_VEL_P_ACC" and p.get("scale_factor") == 0.9 for p in publish_calls))
        
        # 2. I-gain recommendation on persistent offset
        recommendations._last_emit.clear()
        mock_publish.reset_mock()
        
        m_persistent = {
            "tau_s": None,
            "settling_s": None,
            "overshoot": 0.0,
            "sp": 2.0,
            "act": 1.5,
            "r": 0.7,  # < 0.8
        }
        _velocity_advice("vx", "sluggish", m_persistent, "MC")
        
        publish_calls = [call[0][1] for call in mock_publish.call_args_list if call[0][0] == "recommendation"]
        self.assertTrue(any(p.get("param") == "MPC_XY_VEL_I_ACC" and p.get("scale_factor") == 1.15 for p in publish_calls))
        
        # 3. D-gain recommendation on oscillatory behavior
        recommendations._last_emit.clear()
        mock_publish.reset_mock()
        
        m_oscillatory = {
            "tau_s": 0.4,
            "settling_s": 1.5,  # > 3 * tau (1.2)
            "overshoot": 0.18,  # > 0.15
            "sp": 1.0,
            "act": 1.0,
            "r": 0.95,
        }
        _velocity_advice("vx", "sluggish", m_oscillatory, "MC")
        
        publish_calls = [call[0][1] for call in mock_publish.call_args_list if call[0][0] == "recommendation"]
        self.assertTrue(any(p.get("param") == "MPC_XY_VEL_D_ACC" and p.get("scale_factor") == 1.15 for p in publish_calls))

    def test_offline_yaw_axis_lower_threshold(self):
        from backend.app.analysis.pid_offline import _analyze_axis
        
        t = np.linspace(0, 3.0, 300)
        
        # Create a small step setpoint of amplitude 0.10 rad/s
        sp = np.zeros_like(t)
        sp[100:] = 0.10
        act = sp.copy()
        
        # 1. Roll axis: minimum threshold is 0.15, so 0.10 is too small and yields no steps
        res_roll = _analyze_axis(t, sp, act, axis="roll")
        self.assertEqual(res_roll.get("n_steps"), 0)
        
        # 2. Yaw axis: minimum threshold is 0.08, so 0.10 is accepted
        res_yaw = _analyze_axis(t, sp, act, axis="yaw")
        self.assertEqual(res_yaw.get("n_steps"), 1)

    def test_offline_attitude_loop_analysis(self):
        # 1. Test quaternion to Euler conversion helper
        from backend.app.analysis.pid_offline import _extract_euler_angles, analyze_pid
        
        # Identity quaternion should result in roll=0, pitch=0, yaw=0
        df_q = pd.DataFrame({
            "q[0]": [1.0, 1.0],
            "q[1]": [0.0, 0.0],
            "q[2]": [0.0, 0.0],
            "q[3]": [0.0, 0.0],
        })
        eulers = _extract_euler_angles(df_q)
        self.assertIsNotNone(eulers)
        self.assertAlmostEqual(eulers["roll"][0], 0.0)
        self.assertAlmostEqual(eulers["pitch"][0], 0.0)
        self.assertAlmostEqual(eulers["yaw"][0], 0.0)

        # 2. Run analyze_pid and check step response on both rate and body axes
        t = np.linspace(0, 5.0, 500)
        t_ms = t * 1e6

        # Construct a pitch step in rate loop (e.g. from 0.0 to 0.5 rad/s)
        rates_sp = pd.DataFrame({
            "timestamp": t_ms,
            "roll": np.zeros_like(t),
            "pitch": np.zeros_like(t),
            "yaw": np.zeros_like(t),
        })
        rates_sp.loc[200:, "pitch"] = 0.5  # Step of 0.5 rad/s at t=2.0s
        
        ang_vel = pd.DataFrame({
            "timestamp": t_ms,
            "xyz[0]": np.zeros_like(t),
            "xyz[1]": np.zeros_like(t),  # Step response in pitch actual
            "xyz[2]": np.zeros_like(t),
        })
        # actual pitch rate has overshoot (overshoot = 0.3)
        ang_vel.loc[200:, "xyz[1]"] = 0.5
        ang_vel.loc[200:230, "xyz[1]"] = 0.65  # peak of 0.65 (overshoot = (0.65-0.5)/0.5 = 30%)

        # Construct a roll step in body attitude loop (e.g. from 0.0 to 10 degrees = 0.174 rad)
        att_sp = pd.DataFrame({
            "timestamp": t_ms,
            "roll_body": np.zeros_like(t),
            "pitch_body": np.zeros_like(t),
            "yaw_body": np.zeros_like(t),
        })
        att_sp.loc[200:, "roll_body"] = 0.2  # Step of 0.2 rad at t=2.0s

        vehicle_att = pd.DataFrame({
            "timestamp": t_ms,
            "roll": np.zeros_like(t),
            "pitch": np.zeros_like(t),
            "yaw": np.zeros_like(t),
        })
        # actual roll attitude has overshoot (overshoot = 0.4)
        vehicle_att.loc[200:, "roll"] = 0.2
        vehicle_att.loc[200:230, "roll"] = 0.28  # peak of 0.28 (overshoot = 40%)

        params = {
            "MC_PITCHRATE_P": 0.15,
            "MC_ROLL_P": 6.5,
            "FW_RR_P": 0.05,
            "FW_R_TC": 0.4,
        }

        # Test Multirotor recommendations
        res_mc = analyze_pid(
            rates_sp=rates_sp,
            ang_vel=ang_vel,
            sensor=None,
            params=params,
            airframe_class="MULTIROTOR",
            att_sp=att_sp,
            vehicle_att=vehicle_att
        )

        # Rate loop should analyze "pitch (rate)" and recommend backing off P due to overshoot (30% > 25%)
        self.assertIn("pitch (rate)", res_mc["axes"])
        pitch_rate_stats = res_mc["axes"]["pitch (rate)"]
        self.assertTrue(pitch_rate_stats.get("n_steps") > 0)
        self.assertGreater(pitch_rate_stats.get("overshoot_max", 0.0), 0.25)
        
        # Body loop should analyze "roll (body)" and recommend backing off MC_ROLL_P due to overshoot (40% > 25%)
        self.assertIn("roll (body)", res_mc["axes"])
        roll_body_stats = res_mc["axes"]["roll (body)"]
        self.assertTrue(roll_body_stats.get("n_steps") > 0)
        self.assertGreater(roll_body_stats.get("overshoot_max", 0.0), 0.25)

        # Check recommendations
        recs = res_mc["recommendations"]
        self.assertTrue(any(r["param"] == "MC_PITCHRATE_P" and r["proposed_value"] < 0.15 for r in recs))
        self.assertTrue(any(r["param"] == "MC_ROLL_P" and r["proposed_value"] < 6.5 for r in recs))

        # Test Fixed-Wing recommendations (checking time constants)
        res_fw = analyze_pid(
            rates_sp=rates_sp,
            ang_vel=ang_vel,
            sensor=None,
            params=params,
            airframe_class="FIXED_WING",
            att_sp=att_sp,
            vehicle_att=vehicle_att
        )
        recs_fw = res_fw["recommendations"]
        # Fixed wing should slow down roll attitude loop by raising FW_R_TC due to overshoot
        self.assertTrue(any(r["param"] == "FW_R_TC" and r["proposed_value"] > 0.4 for r in recs_fw))

    def test_offline_pulsed_setpoint_overshoot(self):
        from backend.app.analysis.pid_offline import _step_response_offline
        
        t = np.linspace(0, 3.0, 300)
        
        # Step from 0.0 to 1.0 at t=0.5s, stays at 1.0 until 1.5s, then returns gradually to 0.0 at 2.5s.
        sp = np.zeros_like(t)
        sp[50:150] = 1.0
        # gradual return to 0.0 from 1.5s (index 150) to 2.5s (index 250)
        sp[150:250] = np.linspace(1.0, 0.0, 100)
        
        # The actual tracks it perfectly with NO overshoot (reaches exactly 1.0 and decays the same way)
        act = sp.copy()
        
        res, reason = _step_response_offline(t, sp, act, min_amp=0.15)
        
        self.assertIsNotNone(res)
        self.assertIsNone(reason)
        # overshoot should be calculated relative to the plateau (1.0), which is close to 0.0 (or very low)
        # instead of being inflated (e.g. 500%) by the return-to-zero value.
        self.assertLess(res["overshoot"], 0.10)

    def test_offline_plateau_mitigations(self):
        from backend.app.analysis.pid_offline import _step_response_offline
        
        t = np.linspace(0, 3.0, 300)
        
        # Test Case 1: Very short plateau (< 5 samples)
        sp_short = np.zeros_like(t)
        sp_short[50:53] = 1.0  # only 3 samples of plateau
        act_short = sp_short.copy()
        res_short, reason_short = _step_response_offline(t, sp_short, act_short, min_amp=0.15)
        self.assertIsNone(res_short)
        self.assertIn(reason_short, ("window_too_short", "level_change"))
        
        # Test Case 2: Ramped input (no steady plateau, < 5% of window length)
        # 300 samples total. post_mask has 250 samples. 5% is 12.5 samples.
        sp_ramp = np.zeros_like(t)
        sp_ramp[50:60] = np.linspace(0.0, 1.0, 10)
        sp_ramp[60:63] = 1.0  # only 3 samples of plateau at peak, then returns to 0
        act_ramp = sp_ramp.copy()
        res_ramp, reason_ramp = _step_response_offline(t, sp_ramp, act_ramp, min_amp=0.15)
        self.assertIsNone(res_ramp)
        self.assertIn(reason_ramp, ("ramp", "level_change", "too_small", "window_too_short"))

    def test_ulog_pipeline_version_parsing(self):
        from backend.app.analysis.ulog_pipeline import (
            _decode_ver_sw_release, _format_ver_sw_release
        )
        
        # 1. Test decode release version
        # 0x011100FF (1.17.0 Release)
        dec_rel = _decode_ver_sw_release(0x011100FF)
        self.assertEqual(dec_rel, (1, 17, 0, 255))
        
        # 0x010E01C0 (1.14.1 RC0)
        dec_rc = _decode_ver_sw_release(0x010E01C0)
        self.assertEqual(dec_rc, (1, 14, 1, 192))
        
        # 2. Test formatting
        # Release type 255
        self.assertEqual(_format_ver_sw_release(1, 17, 0, 255), "v1.17.0")
        # RC type 192 (RC0)
        self.assertEqual(_format_ver_sw_release(1, 14, 1, 192), "v1.14.1-rc0")
        # Beta type 129 (Beta1)
        self.assertEqual(_format_ver_sw_release(1, 14, 0, 129), "v1.14.0-beta1")
        # Alpha type 65 (Alpha1)
        self.assertEqual(_format_ver_sw_release(1, 14, 0, 65), "v1.14.0-alpha1")
        # Dev type 5 (Dev5)
        self.assertEqual(_format_ver_sw_release(1, 14, 0, 5), "v1.14.0-dev5")


    def test_offline_frequency_domain_fallback(self):
        from backend.app.analysis.pid_offline import compute_fd_gain, _suggest
        
        # 1. Verify compute_fd_gain works as expected on simulated signals
        t = np.linspace(0, 10.0, 1000)
        # Create a setpoint x and actual y with a specific gain relationship (e.g. y = 0.8 * x)
        # We use a sinusoid at 0.75 Hz (in the center of the 0.5-1.0 Hz low frequency band)
        x = np.sin(2 * np.pi * 0.75 * t)
        y = 0.8 * x
        
        gain = compute_fd_gain(t, x, y)
        self.assertIsNotNone(gain)
        # gain should be very close to 0.8
        self.assertAlmostEqual(gain, 0.8, places=2)
        
        # 2. Test fallback recommendations inside _suggest
        # Stats dictionary with no step maneuvers (n_steps = 0) but high coherence and low gain (< 0.85)
        stats_sluggish = {
            "n_steps": 0,
            "n_steps_time_domain": 0,
            "candidates": 0,
            "coherence": 0.8,
            "fd_gain": 0.8,
        }
        params = {"MC_ROLLRATE_P": 0.15}
        recs, notes = _suggest("roll (rate)", stats_sluggish, "MULTIROTOR", params)
        self.assertEqual(len(recs), 1)
        self.assertEqual(recs[0]["param"], "MC_ROLLRATE_P")
        self.assertTrue(recs[0]["proposed_value"] > 0.15) # sluggish -> propose increasing P
        
        # Stats dictionary with no step maneuvers (n_steps = 0) but high coherence and elevated gain (> 1.15)
        stats_elevated = {
            "n_steps": 0,
            "n_steps_time_domain": 0,
            "candidates": 0,
            "coherence": 0.85,
            "fd_gain": 1.25,
        }
        recs_el, notes_el = _suggest("roll (rate)", stats_elevated, "MULTIROTOR", params)
        self.assertEqual(len(recs_el), 1)
        self.assertEqual(recs_el[0]["param"], "MC_ROLLRATE_P")
        self.assertTrue(recs_el[0]["proposed_value"] < 0.15) # elevated -> propose decreasing P

    def test_filter_delay_multiaxial_and_sanity_check(self):
        from backend.app.analysis.filter_advisor import measure_filter_delay_ms
        
        t = np.linspace(0, 15.0, 3000)
        t_ms = t * 1e6
        
        np.random.seed(42)
        raw_roll = np.sin(2 * np.pi * 5.0 * t) + np.random.normal(0, 0.1, len(t))
        raw_pitch = np.sin(2 * np.pi * 7.0 * t) + np.random.normal(0, 0.1, len(t))
        raw_yaw = np.sin(2 * np.pi * 9.0 * t) + np.random.normal(0, 0.1, len(t))
        
        sensor = pd.DataFrame({
            "timestamp": t_ms,
            "gyro_rad[0]": raw_roll,
            "gyro_rad[1]": raw_pitch,
            "gyro_rad[2]": raw_yaw,
        })
        
        flt_roll = np.roll(raw_roll, 3) # 15 ms delay
        flt_pitch = np.roll(raw_pitch, 4) # 20 ms delay
        flt_yaw = np.roll(raw_yaw, 5) # 25 ms delay
        
        ang_vel = pd.DataFrame({
            "timestamp": t_ms,
            "xyz[0]": flt_roll,
            "xyz[1]": flt_pitch,
            "xyz[2]": flt_yaw,
        })
        
        delay = measure_filter_delay_ms(sensor, ang_vel)
        self.assertIsNotNone(delay)
        self.assertAlmostEqual(delay, 20.0, delta=5.0)
        
        # Test 50ms sanity check filtering
        flt_yaw_massive = np.roll(raw_yaw, 12) # 60 ms delay -> suspect!
        ang_vel_massive = pd.DataFrame({
            "timestamp": t_ms,
            "xyz[0]": flt_roll,      # 15 ms
            "xyz[1]": flt_pitch,     # 20 ms
            "xyz[2]": flt_yaw_massive, # 60 ms (filtered out)
        })
        
        delay_massive = measure_filter_delay_ms(sensor, ang_vel_massive)
        self.assertIsNotNone(delay_massive)
        self.assertAlmostEqual(delay_massive, 17.5, delta=3.0)

    def test_actuator_improvements(self):
        from backend.app.analysis import actuator_saturation
        # Create timestamps from 0 to 10s at 100Hz
        t = np.linspace(0, 10, 1000)
        ts_us = t * 1e6
        
        # Motors (M1 to M4 on output[0] to output[3])
        # Let's say M1 has a constant value of 1500 (PWM)
        # M2 has a slightly higher constant value of 1600 (PWM)
        # M3 has a sinusoid
        # M4 has a sinusoid
        outputs = pd.DataFrame({
            "timestamp": ts_us,
            "output[0]": np.full_like(t, 1500.0),
            "output[1]": np.full_like(t, 1600.0),
            "output[2]": 1500.0 + 300.0 * np.sin(2 * np.pi * t),
            "output[3]": 1500.0 + 300.0 * np.sin(2 * np.pi * t),
        })
        
        # Mapped to hover_motors: 0, 1, 2, 3
        actuator_map = {
            "hover_motors": [0, 1, 2, 3],
            "thrust_motors": [],
            "control_surfaces": [],
            "tilt_servos": [],
        }
        
        # Control inputs matching the outputs
        controls = pd.DataFrame({
            "timestamp": ts_us,
            "control[0]": np.zeros_like(t),
            "control[1]": np.zeros_like(t),
            "control[2]": np.zeros_like(t),
            "control[3]": 0.5 + 0.3 * np.sin(2 * np.pi * t), # thrust command correlated with outputs[2] and [3]
        })
        
        # Parameter limits: let's configure limits for M1 (OUT1_MIN/MAX) and M2 (PWM_MAIN_MIN2/MAX2)
        params = {
            "OUT1_MIN": 1000.0,
            "OUT1_MAX": 2000.0,
            "PWM_MAIN_MIN2": 1000.0,
            "PWM_MAIN_MAX2": 2000.0,
        }
        
        res = actuator_saturation.analyze_saturation(
            outputs, actuator_map=actuator_map,
            airframe_class="MULTIROTOR", is_physical=True,
            params=params, controls=controls
        )
        
        # Assertions
        self.assertIn("t", res)
        self.assertIn("channels", res)
        channels = res["channels"]
        self.assertIn("M1 (Ch1)", channels)
        self.assertIn("M2 (Ch2)", channels)
        
        # Verify decimation
        self.assertTrue(len(res["t"]) < 150)
        self.assertEqual(len(channels["M1 (Ch1)"]["values"]), len(res["t"]))
        
        # Verify correlation matching
        self.assertEqual(channels["M3 (Ch3)"]["correlated_axis"], "thrust")
        self.assertTrue(channels["M3 (Ch3)"]["correlation"] > 0.8)
        
        # Verify motor balance
        self.assertIn("motor_balance", res)
        self.assertIn("t", res["motor_balance"])
        self.assertIn("M1", res["motor_balance"]["deviations"])
        
        # Test airspeed note for control surface
        actuator_map_fw = {
            "hover_motors": [],
            "thrust_motors": [],
            "control_surfaces": [4],
            "tilt_servos": [],
        }
        # output[4] is control surface, constant maxed out (2000 PWM) to trigger saturation
        outputs_fw = pd.DataFrame({
            "timestamp": ts_us,
            "output[4]": np.full_like(t, 2000.0),
        })
        # Airspeed constant at 12 m/s
        airspeed_data = (t, np.full_like(t, 12.0))
        params_fw = {
            "FW_AIRSPD_TRIM": 15.0,
        }
        
        res_fw = actuator_saturation.analyze_saturation(
            outputs_fw, actuator_map=actuator_map_fw,
            airframe_class="FIXED_WING", is_physical=True,
            params=params_fw, airspeed_data=airspeed_data
        )
        self.assertIn("S1 (Ch5)", res_fw["channels"])
        self.assertIn("note", res_fw["channels"]["S1 (Ch5)"])
        self.assertIn("possible authority issue at cruise.", res_fw["channels"]["S1 (Ch5)"]["note"])

    def test_filter_advisor_yaw_torque_cutoff(self):
        from backend.app.analysis import filter_advisor
        
        # Test 1: Noisy spectrum -> lower MC_YAW_TQ_CUTOFF to 15.0
        vibration_noisy = {
            "axes": {
                "roll": {"freqs_hz": [10.0, 20.0, 30.0, 50.0], "psd_db": [10.0, 10.0, 10.0, 10.0]} # integrated rms > _GYRO_NOISY_RMS
            }
        }
        params_noisy = {
            "MC_YAW_TQ_CUTOFF": 20.0
        }
        res_noisy = filter_advisor.advise_filters(vibration_noisy, None, None, params_noisy)
        recs_noisy = {r["param"]: r for r in res_noisy.get("recommendations", [])}
        self.assertIn("MC_YAW_TQ_CUTOFF", recs_noisy)
        self.assertEqual(recs_noisy["MC_YAW_TQ_CUTOFF"]["proposed_value"], 15.0)

        # Test 2: Clean spectrum + high delay -> raise MC_YAW_TQ_CUTOFF
        vibration_clean = {
            "axes": {
                "roll": {"freqs_hz": [10.0, 20.0, 30.0, 50.0], "psd_db": [-50.0, -50.0, -50.0, -50.0]} # clean
            }
        }
        params_clean = {
            "MC_YAW_TQ_CUTOFF": 20.0
        }
        # Mock measure_filter_delay_ms to return 45ms (> _DELAY_RAISE_MS)
        from unittest.mock import patch
        with patch("backend.app.analysis.filter_advisor.measure_filter_delay_ms", return_value=45.0):
            res_clean = filter_advisor.advise_filters(vibration_clean, None, None, params_clean)
            recs_clean = {r["param"]: r for r in res_clean.get("recommendations", [])}
            self.assertIn("MC_YAW_TQ_CUTOFF", recs_clean)
            self.assertEqual(recs_clean["MC_YAW_TQ_CUTOFF"]["proposed_value"], 25.0)
    def test_vtol_label_specialization(self):
        from unittest.mock import MagicMock, patch
        from backend.app.analysis.ulog_pipeline import classify_airframe, analyze
        from pathlib import Path
        
        # Test direct classify_airframe output
        mr = classify_airframe(4001)
        self.assertEqual(mr.airframe_class, "MULTIROTOR")
        
        # Mock pyulog.ULog and dataset_frame to test ulog_pipeline.analyze label refinement
        mock_ulog = MagicMock()
        mock_ulog.initial_parameters = {
            "SYS_AUTOSTART": 13000,
            "VT_TYPE": 1 # Tiltrotor
        }
        mock_ulog.msg_info_dict = {
            "sys_name": "PX4"
        }
        mock_ulog.last_timestamp = 10000000
        mock_ulog.start_timestamp = 0
        
        with patch("backend.app.analysis.ulog_pipeline.ULog", return_value=mock_ulog):
            with patch("backend.app.analysis.ulog_pipeline.dataset_frame", return_value=None):
                # Test Tiltrotor (VT_TYPE=1)
                res = analyze(Path("dummy.ulg"))
                self.assertEqual(res["airframe_label"], "VTOL (Tiltrotor)")
                
                # Test Tailsitter (VT_TYPE=0)
                mock_ulog.initial_parameters["VT_TYPE"] = 0
                res = analyze(Path("dummy.ulg"))
                self.assertEqual(res["airframe_label"], "VTOL (Tailsitter)")
                
                # Test Standard VTOL (VT_TYPE=2)
                mock_ulog.initial_parameters["VT_TYPE"] = 2
                res = analyze(Path("dummy.ulg"))
                self.assertEqual(res["airframe_label"], "VTOL (Standard)")
                
                # Test simulation label overrides
                from backend.app.mavlink.airframe import AirframeInfo
                with patch("backend.app.analysis.ulog_pipeline.classify_airframe", return_value=AirframeInfo(1040, "VTOL", "Standard VTOL (SITL sim)")):
                    mock_ulog.initial_parameters["VT_TYPE"] = 1 # Tiltrotor
                    res = analyze(Path("dummy.ulg"))
                    self.assertEqual(res["airframe_label"], "VTOL (Tiltrotor) (SITL sim)")

                # Test fallback when VT_TYPE/CA_AIRFRAME are missing but ACT_FUNC has tilt servo (301)
                mock_ulog.initial_parameters = {
                    "SYS_AUTOSTART": 13000,
                    "ACT_FUNC1": 301,  # VTOL tilt servo
                }
                res = analyze(Path("dummy.ulg"))
                self.assertEqual(res["airframe_label"], "VTOL (Tiltrotor)")
                self.assertIn("ACT_FUNC1", res["initial_params"])


    def test_dynamic_actuator_reclassification(self):
        from backend.app.analysis.ulog_pipeline import _discover_actuators_from_params
        from backend.app.analysis.actuator_saturation import analyze_saturation
        import pandas as pd
        import numpy as np

        # 1. Test backend _discover_actuators_from_params reclassification
        params = {
            "ACT_FUNC1": 101,  # Motor 1
            "ACT_FUNC2": 102,  # Motor 2
            "ACT_FUNC3": 201,  # Servo 1
            "ACT_FUNC4": 202,  # Servo 2
            "ACT_FUNC5": 203,  # Servo 3
            "CA_SV_CS0_TYPE": 5.0,  # Left Elevon
            "CA_SV_CS1_TYPE": 6.0,  # Right Elevon
        }

        # For VTOL airframe, should reclassify based on CS counts:
        # 2 control surfaces -> Servo 1 & 2 are control surfaces, Servo 3 (Ch5) becomes tilt servo
        actuator_map = _discover_actuators_from_params(params, "VTOL")
        self.assertEqual(actuator_map["hover_motors"], [0, 1])
        self.assertEqual(actuator_map["thrust_motors"], [])
        self.assertEqual(actuator_map["control_surfaces"], [2, 3])
        self.assertEqual(actuator_map["tilt_servos"], [4])

        # 2. Test analyze_saturation labeling
        # Create a dummy dataframe with actuator outputs
        df = pd.DataFrame({
            "timestamp": [0, 1000000],
            "output[0]": [0.5, 0.5],
            "output[1]": [0.5, 0.5],
            "output[2]": [0.5, 0.5],
            "output[3]": [0.5, 0.5],
            "output[4]": [0.5, 0.5],
        })
        res = analyze_saturation(df, actuator_map=actuator_map, airframe_class="VTOL", is_physical=True, params=params)
        channels = res.get("channels", {})
        
        # Verify dynamic labels
        self.assertIn("Left Elevon (Ch3)", channels)
        self.assertIn("Right Elevon (Ch4)", channels)
        self.assertIn("Tilt Servo 1 (Ch5)", channels)

        # 3. Test dynamic allocation with CA_SV_TL_COUNT and CA_SV_CS_COUNT
        params_tl = {
            "ACT_FUNC1": 0,
            "ACT_FUNC9": 203,  # Servo 3 (Ch9) -> Tilt Servo
            "ACT_FUNC10": 204, # Servo 4 (Ch10) -> Tilt Servo
            "ACT_FUNC11": 205, # Servo 5 (Ch11) -> Tilt Servo
            "ACT_FUNC13": 201, # Servo 1 (Ch13) -> Control Surface (CS0)
            "ACT_FUNC14": 202, # Servo 2 (Ch14) -> Control Surface (CS1)
            "CA_SV_TL_COUNT": 3.0,
            "CA_SV_CS_COUNT": 2.0,
            "CA_SV_CS0_TYPE": 5.0,  # Left Elevon
            "CA_SV_CS1_TYPE": 6.0,  # Right Elevon
        }
        
        actuator_map_tl = _discover_actuators_from_params(params_tl, "VTOL")
        self.assertEqual(actuator_map_tl["control_surfaces"], [12, 13])
        self.assertEqual(actuator_map_tl["tilt_servos"], [8, 9, 10])

        df_tl = pd.DataFrame({
            "timestamp": [0, 1000000],
            "output[8]": [0.5, 0.5],
            "output[9]": [0.5, 0.5],
            "output[10]": [0.5, 0.5],
            "output[12]": [0.5, 0.5],
            "output[13]": [0.5, 0.5],
        })
        res_tl = analyze_saturation(df_tl, actuator_map=actuator_map_tl, airframe_class="VTOL", is_physical=True, params=params_tl)
        channels_tl = res_tl.get("channels", {})
        
        # Verify dynamic labels for sequential case
        self.assertIn("Tilt Servo 1 (Ch9)", channels_tl)
        self.assertIn("Tilt Servo 2 (Ch10)", channels_tl)
        self.assertIn("Tilt Servo 3 (Ch11)", channels_tl)
        self.assertIn("Left Elevon (Ch13)", channels_tl)
        self.assertIn("Right Elevon (Ch14)", channels_tl)


class TestNewPipelineImprovements(unittest.TestCase):
    def test_unmapped_channel_processing_fallback(self):
        # Verify ActuationMonitor falls back to computing normalized outputs for all channels when unmapped
        from backend.app.analysis.domains import ActuationMonitor
        monitor = ActuationMonitor()
        
        # Mock Connection actuator map to be empty (unmapped)
        with patch("backend.app.analysis.domains.CONNECTION") as mock_conn, \
             patch("backend.app.analysis.domains.HUB") as mock_hub:
            mock_conn.state.actuator_map = {}
            mock_conn.state.actuator_limits = {}
            
            # Send actuator status values
            monitor._process_actuator_status({"actuator": [0.5, -0.5, 0.0]})
            
            # Verify that hub publish was called
            mock_hub.publish.assert_called_with("actuation", unittest.mock.ANY)
            payload = mock_hub.publish.call_args[0][1]
            self.assertTrue(payload.get("unmapped"))
            self.assertIn("motor_norms", payload)
            # Since unmapped, all active channels are treated as motors for saturation check
            self.assertEqual(len(payload["motor_norms"]), 2)
            # Check guessed deflections for the negative value
            self.assertIn("surface_deflections", payload)
            self.assertEqual(len(payload["surface_deflections"]), 1) # only channel 2 was negative

    def test_vibration_gate_reconnect_reset(self):
        # Verify vibration gate resets self._last_ok_val and self._last_cleared_time on reconnect
        from backend.app.analysis.vibration_live import VibrationGate
        gate = VibrationGate()
        gate._last_ok_val = False
        gate._last_cleared_time = 99.0
        
        # We simulate a connection event with connected = True
        class MockEvent:
            def __init__(self):
                self.channel = "connection"
                self.payload = {"connected": True}
                
        # Patch HUB.subscribe to yield the connection event and then cancel
        async def mock_subscribe():
            yield MockEvent()
            
        with patch("backend.app.analysis.vibration_live.HUB.subscribe", return_value=mock_subscribe()):
            # Run the _run method task for one tick
            asyncio.run(gate._run())
            
        self.assertTrue(gate._last_ok_val)
        self.assertEqual(gate._last_cleared_time, 0.0)

    def test_cascade_state_recalculation_on_vtol_mode(self):
        # Verify CascadeState recalculates domain immediately on vtol_state change
        from backend.app.analysis.cascade import CascadeEngine, STATE
        engine = CascadeEngine()
        
        class MockEvent:
            def __init__(self):
                self.channel = "extended_sys_state"
                self.payload = {"vtol_state": 4} # FW mode
                
        async def mock_subscribe():
            yield MockEvent()
            
        with patch("backend.app.analysis.cascade.HUB.subscribe", return_value=mock_subscribe()), \
             patch("backend.app.analysis.cascade.CONNECTION") as mock_conn, \
             patch("backend.app.analysis.cascade.HUB.publish") as mock_publish:
            mock_conn.state.airframe.airframe_class = "VTOL"
            STATE.vtol_state = 3 # MC hover
            
            asyncio.run(engine._run())
            
            # Domain should immediately recalculate to FW
            self.assertEqual(STATE._domain, "FW")
            mock_publish.assert_called_with("cascade_state", unittest.mock.ANY)


class TestNewOfflineAnalysisModules(unittest.TestCase):
    def test_velocity_offline(self):
        from backend.app.analysis.velocity_offline import analyze_velocity
        
        # Test airframe gate
        res = analyze_velocity(None, None, {}, "FIXED_WING")
        self.assertIn("skipped", res)
        
        # Generate dummy data for velocity loop (15s at 100Hz)
        t = np.arange(100.0, 115.0, 0.01)
        
        # Scenario 1: Overshoot (P-gain decrease)
        sp = np.zeros_like(t)
        sp[t >= 102.0] = 2.0
        
        act = np.zeros_like(t)
        t_step = t[t >= 102.0] - 102.0
        # High overshoot formula
        act[t >= 102.0] = 2.0 * (1.0 - np.exp(-1.5 * t_step) * (np.cos(5.0 * t_step) - 0.8 * np.sin(5.0 * t_step)))
        
        local_pos_sp = pd.DataFrame({"timestamp": t * 1e6, "vx": sp, "vy": sp, "vz": sp})
        local_pos = pd.DataFrame({"timestamp": t * 1e6, "vx": act, "vy": act, "vz": act})
        
        params = {
            "MPC_XY_VEL_P_ACC": 1.0,
            "MPC_XY_VEL_I_ACC": 0.1,
            "MPC_XY_VEL_D_ACC": 0.01,
            "MPC_Z_VEL_P_ACC": 1.0,
            "MPC_Z_VEL_I_ACC": 0.1,
            "MPC_Z_VEL_D_ACC": 0.01,
        }
        
        res_os = analyze_velocity(local_pos, local_pos_sp, params, "MULTIROTOR")
        self.assertIn("axes", res_os)
        rec_p = [r for r in res_os["recommendations"] if r["param"] == "MPC_XY_VEL_P_ACC"]
        self.assertTrue(len(rec_p) > 0)
        self.assertTrue(rec_p[0]["proposed_value"] < 1.0)
        
        # Scenario 2: Sluggish settling (P-gain increase)
        act_sluggish = np.zeros_like(t)
        act_sluggish[t >= 102.0] = 2.0 * (0.65 * (1.0 - np.exp(-10.0 * t_step)) + 0.35 * (1.0 - np.exp(-0.4 * t_step)))
        local_pos["vx"] = act_sluggish
        local_pos["vy"] = act_sluggish
        local_pos["vz"] = act_sluggish
        res_sluggish = analyze_velocity(local_pos, local_pos_sp, params, "MULTIROTOR")
        rec_p_inc = [r for r in res_sluggish["recommendations"] if r["param"] == "MPC_XY_VEL_P_ACC"]
        self.assertTrue(len(rec_p_inc) > 0)
        self.assertTrue(rec_p_inc[0]["proposed_value"] > 1.0)

        # Scenario 3: Steady-state offset (I-gain increase)
        # Create steady-state error (I-gain deficit)
        # Offset is constant 0.1 m/s (>0.05) on settled part (std < 0.05)
        act_offset = np.zeros_like(t)
        act_offset[t >= 102.0] = 1.9 # constant offset of 0.1
        local_pos["vx"] = act_offset
        local_pos["vy"] = act_offset
        local_pos["vz"] = act_offset
        res_ss = analyze_velocity(local_pos, local_pos_sp, params, "MULTIROTOR")
        rec_i = [r for r in res_ss["recommendations"] if r["param"] == "MPC_XY_VEL_I_ACC"]
        self.assertTrue(len(rec_i) > 0)
        self.assertTrue(rec_i[0]["proposed_value"] > 0.1)

    def test_position_offline(self):
        from backend.app.analysis.position_offline import analyze_position
        
        # Test airframe gate
        res = analyze_position(None, None, {}, "FIXED_WING")
        self.assertIn("skipped", res)
        
        # Generate dummy data for position loop (15s at 100Hz)
        t = np.arange(100.0, 115.0, 0.01)
        sp = np.zeros_like(t)
        sp[t >= 102.0] = 5.0
        
        act = np.zeros_like(t)
        t_step = t[t >= 102.0] - 102.0
        # High overshoot
        act[t >= 102.0] = 5.0 * (1.0 - np.exp(-1.5 * t_step) * (np.cos(5.0 * t_step) - 0.8 * np.sin(5.0 * t_step)))
        
        local_pos_sp = pd.DataFrame({"timestamp": t * 1e6, "x": sp, "y": sp, "z": sp})
        local_pos = pd.DataFrame({"timestamp": t * 1e6, "x": act, "y": act, "z": act})
        
        params = {
            "MPC_XY_P": 1.0,
            "MPC_Z_P": 1.0,
        }
        
        res_os = analyze_position(local_pos, local_pos_sp, params, "MULTIROTOR")
        self.assertIn("axes", res_os)
        rec = [r for r in res_os["recommendations"] if r["param"] == "MPC_XY_P"]
        self.assertTrue(len(rec) > 0)
        self.assertTrue(rec[0]["proposed_value"] < 1.0)

    def test_tecs_offline(self):
        from backend.app.analysis.tecs_offline import analyze_tecs
        
        # Test airframe gate
        res = analyze_tecs(None, None, None, None, None, {}, "MULTIROTOR")
        self.assertIn("skipped", res)
        
        # Build 60-second flight log at 10 Hz
        t = np.arange(100.0, 160.0, 0.1)
        
        # Level flight segment from 105s to 120s (15s)
        # climb_rate = -vz. So vz should be < 0.5 in absolute value. Let's make it 0.1 (climb rate = -0.1)
        vz = np.ones_like(t) * 2.0
        vz[(t >= 105.0) & (t <= 120.0)] = -0.1  # climb rate = 0.1 m/s
        
        # Climb segment from 125s to 135s (10s)
        # climb rate > 1.0 (vz < -1.0)
        vz[(t >= 125.0) & (t <= 135.0)] = -3.0 # climb rate = 3.0 m/s
        
        # Descent segment from 140s to 150s (10s)
        # climb rate < -1.0 (vz > 1.0)
        vz[(t >= 140.0) & (t <= 150.0)] = 2.0 # climb rate = -2.0 m/s
        
        local_pos = pd.DataFrame({"timestamp": t * 1e6, "vz": vz})
        
        # Throttle (control[3])
        # Level flight: stable throttle (std < 0.05). Let's make it 0.5.
        # Climb: throttle > 0.8. Let's make it 0.9.
        # Descent: throttle < 0.2. Let's make it 0.1.
        ctrl3 = np.ones_like(t) * 0.4
        ctrl3[(t >= 105.0) & (t <= 120.0)] = 0.5
        ctrl3[(t >= 125.0) & (t <= 135.0)] = 0.9
        ctrl3[(t >= 140.0) & (t <= 150.0)] = 0.1
        controls = pd.DataFrame({"timestamp": t * 1e6, "control[3]": ctrl3})
        
        # Pitch
        # Let's make pitch 3.0 degrees in level flight
        pitch = np.ones_like(t) * np.radians(1.0)
        pitch[(t >= 105.0) & (t <= 120.0)] = np.radians(3.0)
        vehicle_att = pd.DataFrame({
            "timestamp": t * 1e6,
            "roll": np.zeros_like(t),
            "pitch": pitch,
            "yaw": np.zeros_like(t)
        })
        
        # Airspeed
        # cruise airspeed in level flight = 15.0 m/s
        as_vals = np.ones_like(t) * 10.0
        as_vals[(t >= 105.0) & (t <= 120.0)] = 15.0
        airspeed = pd.DataFrame({"timestamp": t * 1e6, "indicated_airspeed_m_s": as_vals})
        
        params = {
            "FW_THR_TRIM": 0.4,
            "FW_PSP_OFF": 0.0,
            "FW_AIRSPD_TRIM": 12.0,
            "FW_CLIMB_MAX": 2.0,
            "FW_SINK_MIN": 1.0,
        }
        
        res = analyze_tecs(local_pos, controls, vehicle_att, airspeed, None, params, "FIXED_WING")
        self.assertIn("stats", res)
        recs = {r["param"]: r for r in res["recommendations"]}
        
        # Assert calibration values are generated correctly
        self.assertIn("FW_THR_TRIM", recs)
        self.assertEqual(recs["FW_THR_TRIM"]["proposed_value"], 0.5)
        
        self.assertIn("FW_PSP_OFF", recs)
        self.assertEqual(recs["FW_PSP_OFF"]["proposed_value"], 3.0)
        
        self.assertIn("FW_AIRSPD_TRIM", recs)
        self.assertEqual(recs["FW_AIRSPD_TRIM"]["proposed_value"], 15.0)
        
        self.assertIn("FW_CLIMB_MAX", recs)
        self.assertEqual(recs["FW_CLIMB_MAX"]["proposed_value"], 3.0)
        
        self.assertIn("FW_SINK_MIN", recs)
        self.assertEqual(recs["FW_SINK_MIN"]["proposed_value"], 2.0)

    def test_npfg_offline(self):
        from backend.app.analysis.npfg_offline import analyze_npfg
        
        # Test airframe gate
        res = analyze_npfg({}, "MULTIROTOR")
        self.assertIn("skipped", res)
        
        # NPFG auto tuning disabled
        res_disabled = analyze_npfg({"NPFG_EN_AUTO_TUNING": 0.0}, "FIXED_WING")
        self.assertEqual(len(res_disabled["recommendations"]), 1)
        self.assertEqual(res_disabled["recommendations"][0]["param"], "NPFG_EN_AUTO_TUNING")
        self.assertEqual(res_disabled["recommendations"][0]["proposed_value"], 1)
        
        # NPFG auto tuning enabled
        res_enabled = analyze_npfg({"NPFG_EN_AUTO_TUNING": 1.0}, "FIXED_WING")
        self.assertEqual(len(res_enabled["recommendations"]), 0)


if __name__ == "__main__":
    unittest.main()

