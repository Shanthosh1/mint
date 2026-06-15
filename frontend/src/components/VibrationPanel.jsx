import { useState } from 'react';
import { useChannelState, useTelemetryChannel } from '../hooks/useTelemetry.js';

const AXES = ['x', 'y', 'z'];
const GAUGE_MAX = 75.0; // warning = 30, critical = 60

export default function VibrationPanel() {
  const vib = useChannelState('vibration_metrics');
  const [alerts, setAlerts] = useState([]);

  useTelemetryChannel('alert', (d) => {
    if (!d) {
      setAlerts([]);
      return;
    }
    if (d.source !== 'vibration') return;
    setAlerts((prev) => {
      // Prevent duplicates by text match
      const filtered = prev.filter((a) => a.text !== d.text);
      return [{ ...d, id: Date.now() + Math.random() }, ...filtered];
    });
  });

  const unknown = !vib;
  const xVal = vib?.x ?? 0;
  const yVal = vib?.y ?? 0;
  const zVal = vib?.z ?? 0;
  const clipping = vib?.clipping ?? [0, 0, 0];
  
  const worst = Math.max(xVal, yVal, zVal);
  const isOk = vib?.ok ?? true;

  let statusLabel = 'NORMAL';
  let dotColor = 'ok';
  if (!isOk || worst >= 30.0 || clipping.some(c => c > 0)) {
    if (worst >= 60.0 || clipping.some(c => c > 0)) {
      statusLabel = 'CRITICAL';
      dotColor = 'crit';
    } else {
      statusLabel = 'WARNING';
      dotColor = 'warn';
    }
  }

  return (
    <div className="glass panel animate-fade">
      <div className="row spread" style={{ marginBottom: '14px', alignItems: 'center' }}>
        <h2 style={{ margin: 0, display: 'flex', alignItems: 'center', gap: '8px' }}>
          Vibration Monitor
          <span className="badge">
            <span className={`dot ${dotColor}`} />
            {statusLabel}
          </span>
        </h2>
        {alerts.length > 0 && (
          <button
            className="btn"
            style={{ padding: '4px 8px', fontSize: '0.72rem', borderRadius: '6px' }}
            onClick={() => setAlerts([])}
          >
            Clear alerts
          </button>
        )}
      </div>

      {unknown && <div className="muted">Waiting for vibration telemetry stream…</div>}

      {!unknown && (
        <>
          {alerts.length > 0 && (
            <div className="stack" style={{ marginBottom: '14px' }}>
              {alerts.slice(0, 2).map((a) => (
                <div
                  key={a.id}
                  className={`alert ${a.severity || 'info'}`}
                  style={{ display: 'flex', gap: '8px', padding: '8px 12px', fontSize: '0.8rem', alignItems: 'center' }}
                >
                  <div style={{ flexGrow: 1 }}>{a.text}</div>
                  <button
                    className="btn"
                    style={{
                      padding: '2px 6px',
                      fontSize: '0.7rem',
                      borderRadius: '4px',
                      background: 'rgba(255,255,255,0.08)',
                      border: '1px solid var(--glass-border)',
                      color: 'var(--text-hi)',
                      cursor: 'pointer',
                    }}
                    onClick={() => setAlerts((prev) => prev.filter((x) => x.id !== a.id))}
                  >
                    ✕
                  </button>
                </div>
              ))}
            </div>
          )}

          {AXES.map((axis, idx) => {
            const val = vib[axis] ?? 0;
            const cls = val >= 60.0 ? 'crit' : val >= 30.0 ? 'warn' : 'ok';
            const fillWidth = Math.min(100, (val / GAUGE_MAX) * 100);
            
            return (
              <div className="gauge" key={axis}>
                <div className="row spread" style={{ marginBottom: 4 }}>
                  <span className="muted">{axis.toUpperCase()} Accelerometer</span>
                  <span className="mono" style={{
                    color: cls === 'crit' ? 'var(--crit)' : cls === 'warn' ? 'var(--warn)' : 'var(--ok)',
                    fontWeight: 600
                  }}>
                    {val.toFixed(1)}
                  </span>
                </div>
                <div className="gauge-track">
                  <div className={`gauge-fill ${cls}`} style={{ width: `${fillWidth}%` }} />
                  {/* Warning line (30.0 / 75.0 = 40%) */}
                  <div className="gauge-gate" style={{ left: '40%', borderLeft: '1px dashed rgba(251, 191, 36, 0.4)' }} />
                  {/* Critical line (60.0 / 75.0 = 80%) */}
                  <div className="gauge-gate" style={{ left: '80%', borderLeft: '1px dashed rgba(248, 113, 113, 0.4)' }} />
                </div>
              </div>
            );
          })}

          <div style={{ marginTop: '14px', paddingTop: '10px', borderTop: '1px solid var(--glass-border)' }}>
            <div className="row spread" style={{ fontSize: '0.85rem' }}>
              <span className="muted">Clipping:</span>
              <span className="mono" style={{ fontWeight: 600 }}>
                X: <span style={{ color: clipping[0] > 0 ? 'var(--crit)' : 'inherit' }}>{clipping[0]}</span> |&nbsp;
                Y: <span style={{ color: clipping[1] > 0 ? 'var(--crit)' : 'inherit' }}>{clipping[1]}</span> |&nbsp;
                Z: <span style={{ color: clipping[2] > 0 ? 'var(--crit)' : 'inherit' }}>{clipping[2]}</span>
              </span>
            </div>
          </div>
        </>
      )}
    </div>
  );
}
