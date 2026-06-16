import { useEffect, useState } from 'react';
import { api } from '../api.js';
import { useChannelState, useTelemetryChannel } from '../hooks/useTelemetry.js';
import LiveLoopChart, { LOOP_CONFIG } from './LiveLoopChart.jsx';

const LOOPS = ['rate', 'attitude', 'velocity', 'position'];

const fmt = (v, suffix = '', digits = 2) =>
  v === null || v === undefined ? '—' : `${Number(v).toFixed(digits)}${suffix}`;

/**
 * Cascaded tuning workspace.
 *
 * One tab per control loop (rate → attitude → velocity → position).
 * The flight mode decides which loops the FC is actually closing —
 * inactive tabs are dimmed and the workspace auto-selects the
 * outermost active loop when the mode changes. For VTOLs the control
 * domain badge (MC/FW) flips live with the transition state, and the
 * backend targets the matching parameter family automatically.
 */
export default function PidPanel() {
  const [loop, setLoop] = useState('rate');
  const [axis, setAxis] = useState('roll');
  const [windowOpen, setWindowOpen] = useState(false);
  const [loopMetrics, setLoopMetrics] = useState({});   // loop -> axes metrics
  const regime = useChannelState('regime');
  const cascade = useChannelState('cascade_state');

  useTelemetryChannel('loop_metrics', (d) => {
    if (!d) {
      setLoopMetrics({});
      return;
    }
    setLoopMetrics((prev) => ({ ...prev, [d.loop]: d.axes }));
  });

  const activeLoops = new Set(cascade?.active_loops ?? ['rate', 'attitude']);
  const domain = cascade?.domain ?? 'MC';
  const dynamic = regime?.state === 'dynamic_maneuver';
  const autoMode = (cascade?.mode ?? '').startsWith('AUTO')
    || cascade?.mode === 'OFFBOARD';

  // Follow the flight mode: if the selected loop stops being closed by
  // the FC, fall back to the innermost active loop.
  useEffect(() => {
    if (!activeLoops.has(loop)) {
      const fallback = LOOPS.find((l) => activeLoops.has(l)) ?? 'rate';
      setLoop(fallback);
      setAxis(LOOP_CONFIG[fallback].axes[0]);
    }
  }, [cascade?.mode, cascade?.domain]);   // eslint-disable-line

  useEffect(() => {
    if (windowOpen) {
      api.startTuningWindow(axis, loop).catch(() => {});
    }
  }, [axis, loop, windowOpen]);

  const cfg = LOOP_CONFIG[loop];
  const axes = cfg.axes;
  const metrics = loopMetrics[loop] ?? {};
  const m = metrics[axis];

  const selectLoop = (l) => { setLoop(l); setAxis(LOOP_CONFIG[l].axes[0]); };

  const toggleWindow = async () => {
    if (windowOpen) { await api.stopTuningWindow(); setWindowOpen(false); }
    else { await api.startTuningWindow(axis === 'pitch' ? 'pitch' : axis === 'yaw' ? 'yaw' : 'roll', loop); setWindowOpen(true); }
  };

  const rColor = (r) =>
    r === null || r === undefined ? 'var(--text-lo)'
      : r < 0.85 ? 'var(--crit)' : r < 0.95 ? 'var(--warn)' : 'var(--ok)';

  return (
    <div className="glass panel">
      <div className="row spread">
        <h2>Cascade Tuning — {cascade?.mode ?? '—'}
          <span className="badge" style={{ marginLeft: 10 }}>
            <span className={`dot ${domain === 'FW' ? 'warn' : 'ok'}`} />
            {domain} frame
          </span>
        </h2>
        {(loop === 'rate' || loop === 'attitude') && (
          <button className={`btn ${windowOpen ? 'danger' : 'primary'}`} onClick={toggleWindow}>
            {windowOpen ? 'End test window' : 'Start test window'}
          </button>
        )}
      </div>

      <div className="row" style={{ marginBottom: 10, flexWrap: 'wrap' }}>
        {LOOPS.map((l) => {
          const active = activeLoops.has(l);
          return (
            <button key={l} className="btn"
              disabled={!active}
              title={active ? `${l} loop` : `${l} loop not closed in ${cascade?.mode ?? 'current'} mode`}
              style={{
                flex: 1, textTransform: 'capitalize',
                opacity: active ? 1 : 0.35,
                borderColor: l === loop ? 'var(--accent)' : undefined,
                boxShadow: l === loop ? '0 0 14px var(--accent-glow)' : undefined,
              }}
              onClick={() => selectLoop(l)}>
              {l}
            </button>
          );
        })}
      </div>

      {!dynamic && !autoMode && (
        <div className="alert info" style={{ marginBottom: 12 }}>
          {regime?.state === 'steady_hold'
            ? 'Steady hold — loop analysis suppressed (no false positives from a quiet vehicle). EKF monitoring is active.'
            : 'Pre-flight — analysis starts once the vehicle is flying and maneuvering (or running an AUTO mission).'}
        </div>
      )}

      <div className="row" style={{ marginBottom: 12, flexWrap: 'wrap' }}>
        {axes.map((ax) => {
          const am = metrics[ax];
          return (
            <button key={ax} className="btn"
              style={{
                flex: 1,
                borderColor: ax === axis ? 'var(--accent)' : undefined,
                boxShadow: ax === axis ? '0 0 14px var(--accent-glow)' : undefined,
              }}
              onClick={() => setAxis(ax)}>
              <div style={{ fontSize: '0.72rem', textTransform: 'uppercase', letterSpacing: '0.08em' }}>
                {ax}
              </div>
              <div className="mono" style={{ color: rColor(am?.r) }}>
                r {fmt(am?.r)} · ε {fmt(am?.nrmse)}
              </div>
            </button>
          );
        })}
      </div>

      {m && (m.tau_s !== undefined || m.r !== undefined) && (
        <div className="row muted mono" style={{ marginBottom: 10, gap: 18, flexWrap: 'wrap' }}>
          <span title="time constant (63.2% rise)">τ {fmt(m.tau_s, 's')}</span>
          <span title="time to stay within ±20% of step amplitude"
                style={{ color: m.tau_s && m.settling_s > 4 * m.tau_s ? 'var(--warn)' : undefined }}>
            settle {fmt(m.settling_s, 's')}
          </span>
          <span title="travel beyond target / step amplitude"
                style={{ color: m.overshoot > 0.2 ? 'var(--warn)' : undefined }}>
            overshoot {m.overshoot !== undefined ? `${(m.overshoot * 100).toFixed(0)}%` : '—'}
          </span>
          {m.oscillations !== undefined && <span title="residual reversals">osc {m.oscillations}</span>}
        </div>
      )}

      <LiveLoopChart key={`${loop}-${axis}`} loop={loop} axis={axis} />
    </div>
  );
}
