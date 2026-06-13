import { useChannelState } from '../hooks/useTelemetry.js';

/**
 * Real-time actuator output display.
 *
 * Consumes the `actuation` channel (derived from SERVO_OUTPUT_RAW by the
 * backend ActuationMonitor). Two layouts by control domain:
 *
 *   differential_thrust / hybrid  — per-motor output bars, 0..100%.
 *   aerodynamic_surface / hybrid  — per-surface deflection bars, |±100%|.
 *
 * Bars turn amber at >= 80% and red at >= 100% (saturated / railed), so a
 * pilot can see authority running out mid-maneuver before it bites.
 */
const WARN = 0.8;   // amber zone
const SAT = 1.0;    // red zone (saturated motor / railed surface)

function band(frac) {
  if (frac >= SAT - 1e-6) return 'crit';
  if (frac >= WARN) return 'warn';
  return 'ok';
}

function Bar({ label, frac, text }) {
  // frac is 0..1 of full travel; gate marker sits at the 80% warn line.
  const pct = Math.min(100, Math.max(0, frac * 100));
  const cls = band(frac);
  return (
    <div className="gauge">
      <div className="row spread" style={{ marginBottom: 4 }}>
        <span className="muted">{label}</span>
        <span className="mono" style={{
          color: cls === 'crit' ? 'var(--crit)' : cls === 'warn' ? 'var(--warn)' : 'var(--ok)',
        }}>
          {text}
        </span>
      </div>
      <div className="gauge-track">
        <div className={`gauge-fill ${cls}`} style={{ width: `${pct}%` }} />
        <div className="gauge-gate" style={{ left: `${WARN * 100}%` }} />
      </div>
    </div>
  );
}

export default function ActuatorPanel() {
  const act = useChannelState('actuation');
  const domain = act?.domain;
  const motors = act?.motor_norms;
  const surfaces = act?.surface_deflections;
  const railed = new Set(act?.railed_channels ?? []);

  const showMotors = motors && motors.length > 0;
  const showSurfaces = surfaces && surfaces.length > 0;

  return (
    <div className="glass panel">
      <h2>Actuator Outputs</h2>
      {!act && <div className="muted">Waiting for SERVO_OUTPUT_RAW stream…</div>}
      {act && !showMotors && !showSurfaces && (
        <div className="muted">No actuator channels reporting on this link.</div>
      )}

      {showMotors && (
        <>
          <div className="muted" style={{ fontSize: '0.74rem', marginBottom: 8 }}>
            Motor output {domain === 'hybrid' ? '(MC)' : ''} — red = saturated
          </div>
          {motors.map((n, i) => (
            <Bar key={`m${i}`} label={`M${i + 1}`} frac={n}
                 text={`${Math.round(n * 100)}%`} />
          ))}
        </>
      )}

      {showSurfaces && (
        <>
          <div className="muted" style={{ fontSize: '0.74rem', margin: '10px 0 8px' }}>
            Control surfaces — bar = travel toward rail
            {act.q_ratio != null && (
              <span> · q-ratio {act.q_ratio}</span>
            )}
          </div>
          {surfaces.map((d, i) => (
            <Bar key={`s${i}`} label={`S${i + 1}${railed.has(i) ? ' ⚠' : ''}`}
                 frac={Math.abs(d)}
                 text={`${d >= 0 ? '+' : ''}${Math.round(d * 100)}%`} />
          ))}
        </>
      )}
    </div>
  );
}
