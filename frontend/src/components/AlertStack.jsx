import { useState } from 'react';
import { api } from '../api.js';
import { useTelemetryChannel } from '../hooks/useTelemetry.js';

const MAX_ALERTS = 6;
const MAX_RECOS = 4;

/**
 * Rolling alert feed + pilot step-input prompt + live tuning advice.
 *
 * "recommendation" events carry a structured change (param + relative
 * or absolute target). Staging resolves it against the live on-vehicle
 * value, runs the safety registry, and lands in the Proposals panel —
 * the pilot still has to Approve & Write there. Nothing here writes.
 */
export default function AlertStack() {
  const [alerts, setAlerts] = useState([]);
  const [recos, setRecos] = useState([]);
  const [prompt, setPrompt] = useState(null);
  const [error, setError] = useState(null);

  useTelemetryChannel('alert', (d) => {
    setAlerts((prev) =>
      [{ ...d, id: Date.now() + Math.random() }, ...prev].slice(0, MAX_ALERTS));
  });

  useTelemetryChannel('recommendation', (d) => {
    setRecos((prev) => [
      { ...d, id: Date.now() + Math.random() },
      ...prev.filter((r) => r.param !== d.param),   // newest per param wins
    ].slice(0, MAX_RECOS));
  });

  useTelemetryChannel('pilot_prompt', (d) => {
    if (d.kind === 'compliance_ack') setPrompt(null);
    else setPrompt(d);
  });

  const changeLabel = (r) => {
    if (r.target_value !== null && r.target_value !== undefined) return `→ ${r.target_value}`;
    if (r.scale_factor !== null && r.scale_factor !== undefined) return `× ${r.scale_factor}`;
    return `${r.delta >= 0 ? '+' : ''}${r.delta}`;
  };

  const stage = async (r) => {
    setError(null);
    try {
      await api.createProposal({
        param: r.param,
        target_value: r.target_value ?? undefined,
        scale_factor: r.scale_factor ?? undefined,
        delta: r.delta ?? undefined,
        rationale: r.rationale,
      });
      setRecos((prev) => prev.filter((x) => x.id !== r.id));
    } catch (e) { setError(e.message); }
  };

  return (
    <div className="glass panel">
      <h2>Flight Advisories</h2>
      <div className="stack">
        {prompt && (
          <div className="pilot-prompt row spread">
            <span>🕹 {prompt.text}</span>
            <button className="btn" onClick={() => setPrompt(null)}>✕</button>
          </div>
        )}

        {recos.map((r) => (
          <div key={r.id} className="proposal">
            <div className="row spread">
              <span className="mono" style={{ fontWeight: 600 }}>
                {r.param} <span style={{ color: 'var(--accent)' }}>{changeLabel(r)}</span>
              </span>
              <span className="badge">{r.source}</span>
            </div>
            <div className="muted" style={{ margin: '6px 0' }}>{r.rationale}</div>
            <div className="row">
              <button className="btn primary" style={{ flex: 1 }} onClick={() => stage(r)}>
                Stage as proposal
              </button>
              <button className="btn"
                      onClick={() => setRecos((p) => p.filter((x) => x.id !== r.id))}>
                Dismiss
              </button>
            </div>
          </div>
        ))}

        {error && <div className="alert critical">{error}</div>}
        {alerts.length === 0 && recos.length === 0 && !prompt && (
          <div className="muted">No advisories. Telemetry nominal.</div>
        )}
        {alerts.map((a) => (
          <div key={a.id} className={`alert ${a.severity || 'info'}`}>
            {a.source && <strong style={{ textTransform: 'uppercase', fontSize: '0.7rem', marginRight: 8 }}>{a.source}</strong>}
            {a.text}
          </div>
        ))}
      </div>
    </div>
  );
}
