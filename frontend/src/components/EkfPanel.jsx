import { useChannelState } from '../hooks/useTelemetry.js';

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

  return (
    <div className="glass panel">
      <h2>EKF Innovation Ratios</h2>
      {!ekf && <div className="muted">Waiting for EKF_STATUS_REPORT stream…</div>}
      {ekf && unknown && (
        <div className="alert critical">
          EKF feed lost — gauges UNKNOWN. The last values are not live; do not
          trust estimator health until the stream resumes.
        </div>
      )}
      {!unknown && Object.entries(LABELS).map(([key, label]) => {
        const r = ratios[key] ?? 0;
        const cls = r >= 1.0 ? 'crit' : r >= 0.8 ? 'warn' : 'ok';
        return (
          <div className="gauge" key={key}>
            <div className="row spread" style={{ marginBottom: 4 }}>
              <span className="muted">{label}</span>
              <span className="mono" style={{
                color: cls === 'crit' ? 'var(--crit)' : cls === 'warn' ? 'var(--warn)' : 'var(--ok)',
              }}>
                {r.toFixed(2)}
              </span>
            </div>
            <div className="gauge-track">
              <div className={`gauge-fill ${cls}`}
                   style={{ width: `${Math.min(100, (r / GAUGE_MAX) * 100)}%` }} />
              <div className="gauge-gate" style={{ left: `${(1.0 / GAUGE_MAX) * 100}%` }} />
            </div>
          </div>
        );
      })}
    </div>
  );
}
