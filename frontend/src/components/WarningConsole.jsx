import { useEffect, useState } from 'react';
import { useTelemetryChannel } from '../hooks/useTelemetry.js';

// SVG Icons
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

function timeAgo(epochMs, nowMs) {
  const s = Math.max(0, Math.round((nowMs - epochMs) / 1000));
  if (s < 3) return 'just now';
  if (s < 60) return `${s}s ago`;
  if (s < 3600) return `${Math.floor(s / 60)}m ago`;
  return `${Math.floor(s / 3600)}h ago`;
}

export default function WarningConsole() {
  const [alerts, setAlerts] = useState([]);
  const [now, setNow] = useState(Date.now());

  // Tick relative timestamps
  useEffect(() => {
    const id = setInterval(() => setNow(Date.now()), 1000);
    return () => clearInterval(id);
  }, []);

  useTelemetryChannel('alert', (d, _t, _ch, ts) => {
    if (!d) {
      setAlerts([]);
      return;
    }
    const atMs = ts != null ? ts * 1000 : Date.now();
    setAlerts((prev) => {
      // Filter duplicates of the exact same message text
      const filtered = prev.filter((a) => a.text !== d.text);
      return [{ ...d, id: Date.now() + Math.random(), atMs }, ...filtered].slice(0, 50);
    });
  });

  return (
    <div className="glass panel animate-fade">
      <div className="row spread" style={{ marginBottom: '14px', alignItems: 'center' }}>
        <h2 style={{ display: 'flex', alignItems: 'center', gap: '8px', margin: 0 }}>
          Warning Console
        </h2>
        {alerts.length > 0 && (
          <button
            className="btn"
            style={{ padding: '4px 8px', fontSize: '0.72rem', borderRadius: '6px' }}
            onClick={() => setAlerts([])}
          >
            Clear console
          </button>
        )}
      </div>

      <div className="stack scrollable-stack" style={{ maxHeight: '350px' }}>
        {alerts.length === 0 ? (
          <div className="muted" style={{ padding: '10px 0', textAlign: 'center' }}>
            No warnings or errors. Telemetry nominal.
          </div>
        ) : (
          alerts.map((a) => (
            <div
              key={a.id}
              className={`alert ${a.severity || 'info'}`}
              style={{ display: 'flex', gap: '10px', alignItems: 'flex-start' }}
            >
              <div style={{ marginTop: '2px', flexShrink: 0 }}>
                {getAlertIcon(a.severity)}
              </div>
              <div style={{ flexGrow: 1 }}>
                <div className="row spread" style={{ marginBottom: '4px' }}>
                  {a.source && (
                    <strong style={{ textTransform: 'uppercase', fontSize: '0.7rem', color: 'var(--text-mid)' }}>
                      {a.source}
                    </strong>
                  )}
                  {a.atMs && <span className="muted" style={{ fontSize: '0.7rem' }}>{timeAgo(a.atMs, now)}</span>}
                </div>
                <div style={{ wordBreak: 'break-word', color: 'var(--text-hi)' }}>{a.text}</div>
                {a.type === 'saturation' && (
                  <details style={{ marginTop: '8px', fontSize: '0.78rem', borderTop: '1px dashed rgba(255,255,255,0.15)', paddingTop: '6px' }}>
                    <summary style={{ cursor: 'pointer', color: 'var(--accent)', fontWeight: 500, outline: 'none' }}>
                      Troubleshooting details
                    </summary>
                    <div style={{ marginTop: '6px', display: 'flex', flexDirection: 'column', gap: '6px', color: 'var(--text-mid)' }}>
                      <p style={{ margin: 0 }}>
                        The control coherence in the 0.5-3.0 Hz band is low ({a.coherence}), indicating actuator saturation is actively degrading stabilization.
                      </p>
                      <p style={{ margin: 0, fontWeight: 500 }}>
                        ⚠️ Inspect propellers, shafts, and frame stiffness. Do not increase P-gain while saturation is present.
                      </p>
                    </div>
                  </details>
                )}
              </div>
            </div>
          ))
        )}
      </div>
    </div>
  );
}
