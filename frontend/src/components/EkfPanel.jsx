import { useRef } from 'react';
import { useChannelState } from '../hooks/useTelemetry.js';

const LABELS = {
  gps_velocity: 'GPS Velocity',
  gps_position: 'GPS Position',
  magnetometer: 'Magnetometer',
  barometer: 'Barometer',
};
const GAUGE_MAX = 1.5; // gauges scale 0..1.5 so the 1.0 gate sits at 2/3

export default function EkfPanel() {
  const ekf = useChannelState('ekf_metrics');
  const lastRatiosRef = useRef(null);

  // Preserve last known ratios so gauges stay visible when the feed gaps.
  if (ekf?.ratios != null) {
    lastRatiosRef.current = ekf.ratios;
  }

  const stale = ekf?.status === 'unknown';
  const noData = ekf == null;
  const ratios = stale ? (lastRatiosRef.current ?? {}) : (ekf?.ratios ?? {});
  const isError = !stale && Object.values(ratios).some((r) => r >= 0.8);

  return (
    <div className="glass panel">
      <div className="row spread" style={{ marginBottom: '14px', alignItems: 'center' }}>
        <h2 style={{ margin: 0, display: 'flex', alignItems: 'center', gap: '8px' }}>
          EKF Innovation
          <span className="badge">
            <span className={`dot ${stale ? 'warn' : isError ? 'crit' : 'ok'}`} />
            {stale ? 'STALE' : isError ? 'ERROR' : 'NORMAL'}
          </span>
        </h2>
      </div>
      {noData && <div className="muted">Waiting for EKF telemetry stream…</div>}
      {!noData && (
        <>
          {stale && (
            <div className="alert warning" style={{ marginBottom: '14px' }}>
              EKF feed gapped — showing last known values (dimmed). Do not rely
              on these until the stream resumes.
            </div>
          )}
          {Object.entries(LABELS).map(([key, label]) => {
            const r = ratios[key] ?? 0;
            const cls = r >= 1.0 ? 'crit' : r >= 0.8 ? 'warn' : 'ok';
            const fillWidth = Math.min(100, (r / GAUGE_MAX) * 100);
            return (
              <div className="gauge" key={key} style={{ opacity: stale ? 0.55 : 1 }}>
                <div className="row spread" style={{ marginBottom: 4 }}>
                  <span className="muted">{label}</span>
                  <span className="mono" style={{
                    color: stale ? 'var(--text-lo)' : (cls === 'crit' ? 'var(--crit)' : cls === 'warn' ? 'var(--warn)' : 'var(--ok)'),
                    fontWeight: stale ? 500 : 600,
                  }}>
                    {r.toFixed(2)}
                  </span>
                </div>
                <div className="gauge-track">
                  <div className={`gauge-fill ${cls}`} style={{ width: `${fillWidth}%` }} />
                  <div className="gauge-gate" style={{ left: `${(1.0 / GAUGE_MAX) * 100}%` }} />
                </div>
              </div>
            );
          })}
        </>
      )}
    </div>
  );
}
