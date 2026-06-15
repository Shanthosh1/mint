import { useState } from 'react';
import { useChannelState, useTelemetryChannel } from '../hooks/useTelemetry.js';

const LABELS = {
  gps_velocity: 'GPS Velocity',
  gps_position: 'GPS Position',
  magnetometer: 'Magnetometer',
  barometer: 'Barometer',
};
const GAUGE_MAX = 1.5; // gauges scale 0..1.5 so the 1.0 gate sits at 2/3

/**
 * Real-time EKF diagnostician: innovation test ratio gauges with the
 * rejection gate (1.0) marked. Green < 0.8, amber < 1.0, red >= 1.0.
 */
export default function EkfPanel() {
  const ekf = useChannelState('ekf_metrics');
  const unknown = ekf?.status === 'unknown' || ekf?.ratios == null;
  const ratios = ekf?.ratios ?? {};
  const [alerts, setAlerts] = useState([]);

  useTelemetryChannel('alert', (d) => {
    if (!d) {
      setAlerts([]);
      return;
    }
    if (d.source !== 'ekf') return;
    setAlerts((prev) => {
      let filtered = prev;

      const isEkfResumed = d.text && d.text.includes('resumed');
      const isEkfStale = d.text && (d.text.includes('No EKF status') || d.text.includes('feed has stopped'));
      const isEkfMissing = d.text && d.text.includes('No EKF data received');

      if (isEkfResumed) {
        filtered = prev.filter((a) => !(a.text.includes('No EKF status') || a.text.includes('feed has stopped') || a.text.includes('No EKF data received')));
      } else if (isEkfStale || isEkfMissing) {
        filtered = prev.filter((a) => !a.text.includes('resumed'));
      }

      // Filter out any alert with the exact same text to prevent duplicates
      filtered = filtered.filter((a) => a.text !== d.text);

      return [{ ...d, id: Date.now() + Math.random() }, ...filtered];
    });
  });

  const hasErrorAlert = alerts.some((a) => a.severity === 'critical' || a.severity === 'warning');
  const isError = unknown || hasErrorAlert;

  return (
    <div className="glass panel">
      <div className="row spread" style={{ marginBottom: '14px', alignItems: 'center' }}>
        <h2 style={{ margin: 0, display: 'flex', alignItems: 'center', gap: '8px' }}>
          EKF Innovation
          <span className="badge">
            <span className={`dot ${isError ? 'crit' : 'ok'}`} />
            {isError ? 'ERROR' : 'NORMAL'}
          </span>
        </h2>
        {alerts.length > 0 && (
          <button className="btn" style={{ padding: '4px 8px', fontSize: '0.72rem', borderRadius: '6px' }} onClick={() => setAlerts([])}>
            Clear alerts
          </button>
        )}
      </div>
      {!ekf && <div className="muted">Waiting for EKF telemetry stream…</div>}
      {ekf && (
        <>
          {alerts.length > 0 && (
            <div className="stack" style={{ marginBottom: '14px' }}>
              {alerts.slice(0, 2).map((a) => (
                <div key={a.id} className={`alert ${a.severity || 'info'}`} style={{ display: 'flex', gap: '8px', padding: '8px 12px', fontSize: '0.8rem', alignItems: 'center' }}>
                  <div style={{ flexGrow: 1 }}>{a.text}</div>
                  <button className="btn" style={{ padding: '2px 6px', fontSize: '0.7rem', borderRadius: '4px', background: 'rgba(255,255,255,0.08)', border: '1px solid var(--glass-border)', color: 'var(--text-hi)', cursor: 'pointer' }} onClick={() => setAlerts((prev) => prev.filter((x) => x.id !== a.id))}>✕</button>
                </div>
              ))}
              {alerts.length > 2 && (
                <div className="stack scrollable-stack" style={{ maxHeight: '120px', paddingRight: '4px', gap: '8px' }}>
                  {alerts.slice(2).map((a) => (
                    <div key={a.id} className={`alert ${a.severity || 'info'}`} style={{ display: 'flex', gap: '8px', padding: '8px 12px', fontSize: '0.8rem', alignItems: 'center' }}>
                      <div style={{ flexGrow: 1 }}>{a.text}</div>
                      <button className="btn" style={{ padding: '2px 6px', fontSize: '0.7rem', borderRadius: '4px', background: 'rgba(255,255,255,0.08)', border: '1px solid var(--glass-border)', color: 'var(--text-hi)', cursor: 'pointer' }} onClick={() => setAlerts((prev) => prev.filter((x) => x.id !== a.id))}>✕</button>
                    </div>
                  ))}
                </div>
              )}
            </div>
          )}
          {unknown && alerts.length === 0 && (
            <div className="alert critical" style={{ marginBottom: '14px' }}>
              EKF feed lost — gauges UNKNOWN. The last values are not live; do not
              trust estimator health until the stream resumes.
            </div>
          )}
          {Object.entries(LABELS).map(([key, label]) => {
            const r = unknown ? null : (ratios[key] ?? 0);
            const cls = unknown ? 'unknown' : (r >= 1.0 ? 'crit' : r >= 0.8 ? 'warn' : 'ok');
            const displayVal = unknown ? 'UNKNOWN' : r.toFixed(2);
            const fillWidth = unknown ? 0 : Math.min(100, (r / GAUGE_MAX) * 100);
            return (
              <div className="gauge" key={key} style={{ opacity: unknown ? 0.55 : 1 }}>
                <div className="row spread" style={{ marginBottom: 4 }}>
                  <span className="muted">{label}</span>
                  <span className="mono" style={{
                    color: unknown ? 'var(--text-lo)' : (cls === 'crit' ? 'var(--crit)' : cls === 'warn' ? 'var(--warn)' : 'var(--ok)'),
                    fontWeight: unknown ? 500 : 600,
                  }}>
                    {displayVal}
                  </span>
                </div>
                <div className="gauge-track">
                  <div className={`gauge-fill ${cls}`}
                       style={{ width: `${fillWidth}%` }} />
                  {!unknown && <div className="gauge-gate" style={{ left: `${(1.0 / GAUGE_MAX) * 100}%` }} />}
                </div>
              </div>
            );
          })}
        </>
      )}
    </div>
  );
}
