import { useEffect, useState } from 'react';
import { api } from '../api.js';
import { useTelemetryChannel } from '../hooks/useTelemetry.js';

const MAX_ALERTS = 6;
const MAX_RECOS = 4;

// Inline SVG Icon components for premium look
const BellIcon = () => (
  <svg viewBox="0 0 24 24" width="16" height="16" stroke="currentColor" strokeWidth="2.5" fill="none" strokeLinecap="round" strokeLinejoin="round" style={{ flexShrink: 0, color: 'var(--accent)' }}>
    <path d="M18 8A6 6 0 0 0 6 8c0 7-3 9-3 9h18s-3-2-3-9" />
    <path d="M13.73 21a2 2 0 0 1-3.46 0" />
  </svg>
);

const InfoIcon = () => (
  <svg viewBox="0 0 24 24" width="16" height="16" stroke="#4f8cff" strokeWidth="2.5" fill="none" strokeLinecap="round" strokeLinejoin="round" style={{ flexShrink: 0 }}>
    <circle cx="12" cy="12" r="10" />
    <line x1="12" y1="16" x2="12" y2="12" />
    <line x1="12" y1="8" x2="12.01" y2="8" />
  </svg>
);

const WarningIcon = () => (
  <svg viewBox="0 0 24 24" width="16" height="16" stroke="#fbbf24" strokeWidth="2.5" fill="none" strokeLinecap="round" strokeLinejoin="round" style={{ flexShrink: 0 }}>
    <path d="M10.29 3.86L1.82 18a2 2 0 0 0 1.71 3h16.94a2 2 0 0 0 1.71-3L13.71 3.86a2 2 0 0 0-3.42 0z" />
    <line x1="12" y1="9" x2="12" y2="13" />
    <line x1="12" y1="17" x2="12.01" y2="17" />
  </svg>
);

const CriticalIcon = () => (
  <svg viewBox="0 0 24 24" width="16" height="16" stroke="#f87171" strokeWidth="2.5" fill="none" strokeLinecap="round" strokeLinejoin="round" style={{ flexShrink: 0 }}>
    <polygon points="7.86 2 16.14 2 22 7.86 22 16.14 16.14 22 7.86 22 2 16.14 2 7.86 7.86 2" />
    <line x1="12" y1="8" x2="12" y2="12" />
    <line x1="12" y1="16" x2="12.01" y2="16" />
  </svg>
);

const LightbulbIcon = () => (
  <svg viewBox="0 0 24 24" width="16" height="16" stroke="var(--accent)" strokeWidth="2.5" fill="none" strokeLinecap="round" strokeLinejoin="round" style={{ flexShrink: 0 }}>
    <path d="M15 14c.2-1 .7-1.7 1.5-2.5 1-.9 1.5-2.2 1.5-3.5A5 5 0 0 0 8 8c0 1 .3 2.2 1.5 3.5.7.7 1.3 1.5 1.5 2.5" />
    <line x1="9" y1="18" x2="15" y2="18" />
    <line x1="10" y1="22" x2="14" y2="22" />
  </svg>
);

const JoystickIcon = () => (
  <svg viewBox="0 0 24 24" width="16" height="16" stroke="var(--accent)" strokeWidth="2.5" fill="none" strokeLinecap="round" strokeLinejoin="round" style={{ flexShrink: 0 }}>
    <circle cx="12" cy="12" r="10" />
    <line x1="12" y1="2" x2="12" y2="22" />
    <line x1="2" y1="12" x2="22" y2="12" />
    <circle cx="12" cy="12" r="3" fill="var(--accent)" />
  </svg>
);

const CheckCircleIcon = () => (
  <svg viewBox="0 0 24 24" width="16" height="16" stroke="#22c55e" strokeWidth="2.5" fill="none" strokeLinecap="round" strokeLinejoin="round" style={{ flexShrink: 0 }}>
    <path d="M22 11.08V12a10 10 0 1 1-5.93-9.14" />
    <polyline points="22 4 12 14.01 9 11.01" />
  </svg>
);

const getAlertIcon = (severity) => {
  if (severity === 'success') return <CheckCircleIcon />;
  if (severity === 'warning') return <WarningIcon />;
  if (severity === 'critical' || severity === 'error') return <CriticalIcon />;
  return <InfoIcon />;
};

/** "just now" / "12s ago" / "3m ago" from an epoch-ms timestamp. */
function timeAgo(epochMs, nowMs) {
  const s = Math.max(0, Math.round((nowMs - epochMs) / 1000));
  if (s < 3) return 'just now';
  if (s < 60) return `${s}s ago`;
  if (s < 3600) return `${Math.floor(s / 60)}m ago`;
  return `${Math.floor(s / 3600)}h ago`;
}

/**
 * Rolling alert feed + pilot step-input prompt + live tuning advice.
 *
 * "recommendation" events carry a structured change (param + relative
 * or absolute target). Staging resolves it against the live on-vehicle
 * value, runs the safety registry, and lands in the Proposals panel —
 * the pilot still has to Approve & Write there. Nothing here writes.
 */
export default function AlertStack() {
  const [recos, setRecos] = useState([]);
  const [prompt, setPrompt] = useState(null);
  const [error, setError] = useState(null);

  useTelemetryChannel('recommendation', (d) => {
    if (!d) {
      setRecos([]);
      return;
    }
    setRecos((prev) => [
      { ...d, id: Date.now() + Math.random() },
      ...prev.filter((r) => r.param !== d.param),   // newest per param wins
    ].slice(0, MAX_RECOS));
  });

  useTelemetryChannel('pilot_prompt', (d) => {
    if (!d) {
      setPrompt(null);
      return;
    }
    if (d.kind === 'compliance_ack') setPrompt(null);
    else setPrompt(d);
  });

  const changeLabel = (r) => {
    if (r.target_value !== null && r.target_value !== undefined) return `→ ${r.target_value}`;
    if (r.scale_factor !== null && r.scale_factor !== undefined) {
      const pct = Math.round((r.scale_factor - 1.0) * 100);
      return `(${pct >= 0 ? '+' : ''}${pct}%)`;
    }
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
        is_saturation_gain_reduction: r.is_saturation_gain_reduction ?? false,
        confidence: r.confidence ?? undefined,
        limitations: r.limitations ?? undefined,
      });
      setRecos((prev) => prev.filter((x) => x.id !== r.id));
    } catch (e) { setError(e.message); }
  };

  return (
    <div className="glass panel">
      <div className="row spread" style={{ marginBottom: '14px', alignItems: 'center' }}>
        <h2 style={{ display: 'flex', alignItems: 'center', gap: '8px', margin: 0 }}>
          <BellIcon />
          Flight Advisories
        </h2>
        {(recos.length > 0 || prompt) && (
          <button className="btn" style={{ padding: '4px 8px', fontSize: '0.72rem', borderRadius: '6px' }} onClick={() => { setRecos([]); setPrompt(null); }}>
            Clear all
          </button>
        )}
      </div>
      <div className="stack scrollable-stack" style={{ maxHeight: '350px' }}>
        {prompt && (
          <div className="pilot-prompt row spread" style={{ display: 'flex', gap: '10px', alignItems: 'center' }}>
            <div style={{ display: 'flex', alignItems: 'center', gap: '8px', flexGrow: 1 }}>
              <JoystickIcon />
              <span>{prompt.text}</span>
            </div>
            <button className="btn" style={{ padding: '4px 8px', borderRadius: '6px' }} onClick={() => setPrompt(null)}>✕</button>
          </div>
        )}

        {recos.map((r) => (
          <div key={r.id} className="proposal">
            <div className="row spread" style={{ marginBottom: '6px' }}>
              <span className="mono" style={{ fontWeight: 600, display: 'flex', alignItems: 'center', gap: '6px' }}>
                <LightbulbIcon />
                {r.param} <span style={{ color: 'var(--accent)' }}>{changeLabel(r)}</span>
              </span>
              <div style={{ display: 'flex', gap: '6px', alignItems: 'center' }}>
                {r.confidence && <span className="badge">{r.confidence}</span>}
                <span className="badge">{r.source}</span>
              </div>
            </div>
            {r.limitations && (
              <div style={{ fontSize: '0.76rem', color: '#fbbf24', marginBottom: '6px', fontWeight: 500 }}>
                ⚠️ {r.limitations}
              </div>
            )}
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

        {error && (
          <div className="alert critical" style={{ display: 'flex', gap: '10px', alignItems: 'flex-start' }}>
            <div style={{ marginTop: '2px', flexShrink: 0 }}>
              <CriticalIcon />
            </div>
            <div style={{ flexGrow: 1 }}>{error}</div>
          </div>
        )}
        {recos.length === 0 && !prompt && (
          <div className="muted" style={{ display: 'flex', alignItems: 'center', gap: '6px' }}>
            <span>No recommendations. Telemetry nominal.</span>
          </div>
        )}
      </div>
    </div>
  );
}
