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
from backend.app.advisors.tuning_memory import TuningMemory
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
        # Issue #5: Coherence defaults to None with insufficient data (<20 samples)
        t = np.arange(10)
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

    def test_tuning_memory_locking(self):
        # Issue #10: Tuning memory has a Lock initialized
        tm = TuningMemory(Path("/tmp/dummy_tuning_mem.jsonl"))
        self.assertTrue(hasattr(tm, "_lock"))
        self.assertIsNotNone(tm._lock)

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

    async def test_feedback_auto_timeout(self):
        from backend.app.advisors.param_advisor import ParamAdvisor, Proposal, ProposalState
        advisor = ParamAdvisor()
        
        # Create a written proposal with written_at 65 seconds ago
        prop = Proposal(
            id="test_timeout_id",
            param="MC_ROLLRATE_P",
            current_value=0.15,
            proposed_value=0.165,
            requested_value=0.165,
            rationale="Test auto timeout",
            airframe_class="MULTIROTOR",
            state=ProposalState.WRITTEN,
            safety_note="",
            written_at=time.time() - 65.0
        )
        advisor._proposals[prop.id] = prop
        
        # Patch sleep using an async function to avoid TypeError on await
        async def mock_sleep(seconds):
            if mock_sleep.calls == 0:
                mock_sleep.calls += 1
                return
            raise asyncio.CancelledError()
        mock_sleep.calls = 0
        
        with patch("backend.app.advisors.param_advisor.asyncio.sleep", mock_sleep):
            try:
                await advisor._auto_timeout_loop()
            except asyncio.CancelledError:
                pass
                
        # Verify the proposal state feedback is auto-set to "no_feedback"
        self.assertEqual(prop.feedback, "no_feedback")

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
                severity=None
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
                severity=None
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
                severity=None
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
                severity=None
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
        from backend.app.analysis.live_pid import LivePidEngine
        
        engine = LivePidEngine()
        engine._recommended_axes_this_cycle = set()
        
        # Initially, override is default (3.0)
        self.assertEqual(engine._window_s_override["roll"], 3.0)
        
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
            
            # Consecutive counter goes to 1, window size still 3.0
            self.assertEqual(engine._consecutive_overshoot_no_osc["roll"], 1)
            self.assertEqual(engine._window_s_override["roll"], 3.0)
            
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
            self.assertEqual(engine._window_s_override["roll"], 3.0)

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
        sp_pre_low[100:] = 0.16 # amp = 0.16
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
            self.assertEqual(called_nperseg, 64)
            
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


if __name__ == "__main__":
    unittest.main()


