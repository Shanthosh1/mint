import { useState } from 'react';
import { useChannelState, useFrontendConfig } from '../hooks/useTelemetry.js';

/**
 * Real-time actuator output display.
 *
 * Consumes the `actuation` channel (derived from SERVO_OUTPUT_RAW or ACTUATOR_OUTPUT_STATUS).
 *
 * Merges all outputs and real-time saturation/health status into a single unified card.
 * Servos (control surfaces/tilts) are displayed as bidirectional center-filled bars (deflection from trim).
 * Motors (hover/thrust) are displayed as unidirectional progress bars.
 */

function band(frac, warn = 0.8, sat = 1.0) {
  if (frac >= sat - 1e-6) return 'crit';
  if (frac >= warn) return 'warn';
  return 'ok';
}

function Bar({ label, frac, val, bipolar, text, dim, warn = 0.8, sat = 1.0 }) {
  let pct = 0;
  let left = '0%';
  let cls = 'ok';
  
  if (bipolar) {
    const d = val ?? 0;
    const absD = Math.abs(d);
    cls = band(absD, warn, sat);
    
    const scale = Math.min(1.0, Math.max(-1.0, d));
    if (scale >= 0) {
      left = '50%';
      pct = scale * 50;
    } else {
      left = `${50 + scale * 50}%`;
      pct = Math.abs(scale) * 50;
    }
  } else {
    const f = frac ?? 0;
    pct = Math.min(100, Math.max(0, f * 100));
    cls = band(f, warn, sat);
  }

  return (
    <div className="gauge" style={dim ? { opacity: 0.5 } : undefined}>
      <div className="row spread" style={{ marginBottom: 4 }}>
        <span className="muted" style={{ display: 'flex', alignItems: 'center' }}>{label}</span>
        <span className="mono" style={{
          color: cls === 'crit' ? 'var(--crit)' : cls === 'warn' ? 'var(--warn)' : 'var(--text-mid)',
        }}>
          {text}
        </span>
      </div>
      <div className="gauge-track" style={{ position: 'relative' }}>
        {bipolar && (
          <div style={{
            position: 'absolute',
            left: '50%',
            top: 0,
            bottom: 0,
            width: '1px',
            backgroundColor: 'var(--glass-border)',
            opacity: 0.8,
            zIndex: 1
          }} />
        )}
        <div className={`gauge-fill ${cls}`} style={{ position: 'absolute', left, width: `${pct}%` }} />
        {!bipolar && (
          <div className="gauge-gate" style={{ left: `${warn * 100}%` }} />
        )}
        {bipolar && (
          <>
            <div className="gauge-gate" style={{ left: `${(0.5 - warn * 0.5) * 100}%` }} />
            <div className="gauge-gate" style={{ left: `${(0.5 + warn * 0.5) * 100}%` }} />
          </>
        )}
      </div>
    </div>
  );
}

function TypeBadge({ type }) {
  if (type === 'Motor') return <span className="pill ok" style={{ marginLeft: 6 }}>Motor</span>;
  if (type === 'Thrust') return <span className="pill warn" style={{ marginLeft: 6 }}>Thrust</span>;
  if (type === 'Surface') return <span className="pill neutral" style={{ marginLeft: 6 }}>Surface</span>;
  if (type === 'Tilt') return <span className="pill neutral" style={{ marginLeft: 6 }}>Tilt</span>;
  return <span className="pill neutral" style={{ marginLeft: 6, opacity: 0.6 }}>Unclassified</span>;
}

export default function ActuatorPanel() {
  const fConfig = useFrontendConfig();
  const warn = fConfig?.actuator_panel?.warn_threshold ?? 0.8;
  const sat = fConfig?.actuator_panel?.sat_threshold ?? 1.0;

  const [showPercentage, setShowPercentage] = useState(false);

  const act = useChannelState('actuation');
  const domain = act?.domain;
  const motors = act?.motor_norms;
  const motor_channels = act?.motor_channels;
  const thrusts = act?.thrust_norms;
  const thrust_channels = act?.thrust_channels;
  const surfaces = act?.surface_deflections;
  const surface_channels = act?.surface_channels;
  const surface_names = act?.surface_names;
  const tilts = act?.tilt_deflections;
  const tilt_channels = act?.tilt_channels;
  const railed = new Set(act?.railed_channels ?? []);
  const balance = act?.motor_balance;
  const rawChannels = act?.raw_channels ?? [];
  const unmapped = act?.unmapped;

  const surfaceDefMap = {};
  if (surface_channels && surfaces) {
    surface_channels.forEach((chNum, index) => {
      surfaceDefMap[chNum] = surfaces[index] ?? 0;
    });
  }

  const tiltDefMap = {};
  if (tilt_channels && tilts) {
    tilt_channels.forEach((chNum, index) => {
      tiltDefMap[chNum] = tilts[index] ?? 0;
    });
  }

  // Saturated actuators calculation for combining outputs & saturation status
  const motorSaturatedCount = motors ? motors.filter(n => n >= sat).length : 0;
  const surfaceRailedCount = railed ? railed.size : 0;
  const saturatedChannelsCount = motorSaturatedCount + surfaceRailedCount;
  const hasSaturatedActuators = saturatedChannelsCount > 0;

  const getChannelInfo = (chNum) => {
    if (unmapped) {
      if (surface_channels && surface_channels.includes(chNum)) {
        const idx = surface_channels.indexOf(chNum);
        const customLabel = surface_names?.[idx] || `Surface ${idx + 1}`;
        return { label: customLabel, type: 'Surface', isServo: true };
      }
      return { label: `Ch${chNum}`, type: 'Unclassified', isServo: false };
    }

    if (motor_channels && motor_channels.includes(chNum)) {
      const idx = motor_channels.indexOf(chNum);
      return { label: `M${idx + 1}`, type: 'Motor', isServo: false };
    }
    if (thrust_channels && thrust_channels.includes(chNum)) {
      const idx = thrust_channels.indexOf(chNum);
      return { label: `P${idx + 1}`, type: 'Thrust', isServo: false };
    }
    if (surface_channels && surface_channels.includes(chNum)) {
      const idx = surface_channels.indexOf(chNum);
      const customLabel = surface_names?.[idx] || `S${idx + 1}`;
      return { label: customLabel, type: 'Surface', isServo: true };
    }
    if (tilt_channels && tilt_channels.includes(chNum)) {
      const idx = tilt_channels.indexOf(chNum);
      return { label: `Tilt${idx + 1}`, type: 'Tilt', isServo: true };
    }
    return { label: `Ch${chNum}`, type: 'Unclassified', isServo: false };
  };

  return (
    <div className="glass panel">
      <div className="row spread" style={{ marginBottom: '14px', alignItems: 'center' }}>
        <h2 style={{ margin: 0 }}>Actuator Outputs</h2>
        {rawChannels.length > 0 && (
          <button
            className="btn"
            style={{ padding: '4px 8px', fontSize: '0.72rem', borderRadius: '6px' }}
            onClick={() => setShowPercentage(!showPercentage)}
          >
            {showPercentage ? 'Show Raw (µs)' : 'Show Percentage'}
          </button>
        )}
      </div>

      {/* Dynamic Saturated/Health status display */}
      {act && (
        <div className="row gap-md" style={{ marginBottom: 16, flexWrap: 'wrap', gap: 8 }}>
          {hasSaturatedActuators ? (
            <span className="pill crit" style={{ fontSize: '0.72rem', padding: '4px 8px', display: 'inline-flex', alignItems: 'center', gap: 4 }}>
              🔴 Saturation Alert: {saturatedChannelsCount} channels railed
            </span>
          ) : (
            <span className="pill ok" style={{ fontSize: '0.72rem', padding: '4px 8px', display: 'inline-flex', alignItems: 'center', gap: 4 }}>
              🟢 Actuators Healthy
            </span>
          )}
          
          {act.motor_max != null && (
            <span className="pill neutral" style={{ fontSize: '0.72rem', padding: '4px 8px' }}>
              Peak Motor: {Math.round(act.motor_max * 100)}%
            </span>
          )}
          {act.motor_sat_index > 0 && (
            <span className="pill warn" style={{ fontSize: '0.72rem', padding: '4px 8px' }}>
              Motor Saturation Index: {Math.round(act.motor_sat_index * 100)}%
            </span>
          )}
        </div>
      )}

      {!act && <div className="muted">Waiting for actuator output stream…</div>}

      {unmapped && rawChannels.length > 0 && (
        <div className="alert warning" style={{ marginBottom: 12, fontSize: '0.8rem' }}>
          ⚠ Actuator mapping not available — fallback to guessed surfaces and unclassified channels.
          Use the <strong>Connection panel → Configure Actuators</strong> to assign
          channel roles (motors, surfaces, tilts).
        </div>
      )}

      {/* Unified Channels List */}
      {rawChannels.length > 0 && (
        <div className="stack" style={{ gap: 10 }}>
          {rawChannels.map((rc) => {
            const info = getChannelInfo(rc.ch);
            
            let val = undefined;
            if (info.isServo) {
              if (info.type === 'Tilt') {
                val = tiltDefMap[rc.ch] ?? 0;
              } else {
                val = surfaceDefMap[rc.ch] ?? 0;
              }
            }

            let textVal = '';
            if (showPercentage) {
              if (info.isServo) {
                textVal = rc.data_available === false ? 'N/A' : `${Math.round(val * 100)}%`;
              } else {
                textVal = `${Math.round(rc.norm * 100)}%`;
              }
            } else {
              textVal = rc.data_available === false ? 'N/A' : `${rc.raw} µs`;
            }
            
            return (
              <div key={rc.ch}>
                <Bar
                  label={
                    <span style={{ display: 'inline-flex', alignItems: 'center' }}>
                      <span className="mono" style={{ fontWeight: 600 }}>{info.label}</span>
                      <span className="muted" style={{ fontSize: '0.68rem', marginLeft: 6 }}>(Ch{rc.ch})</span>
                      <TypeBadge type={info.type} />
                    </span>
                  }
                  bipolar={info.isServo}
                  val={info.isServo ? val : undefined}
                  frac={info.isServo ? undefined : rc.norm}
                  text={textVal}
                  warn={warn}
                  sat={sat}
                />
              </div>
            );
          })}
        </div>
      )}

      {/* Hover Balance Diagnosis */}
      {act && balance && motors && motors.length > 0 && (
        <div className="muted" style={{ fontSize: '0.74rem', marginTop: 14, borderTop: '1px solid var(--glass-border)', paddingTop: 10 }}>
          {balance.balanced ? (
            <>⚖ Hover balance OK — motors within ±{Math.round((balance.worst_dev ?? 0) * 100)}% of mean</>
          ) : (
            <span style={{ color: 'var(--warn)' }}>
              ⚖ Hover imbalance — M{balance.worst_motor} runs{' '}
              {Math.round(Math.abs(balance.worst_dev) * 100)}% off the others
              (prop wear / ESC drift / CG)
            </span>
          )}
        </div>
      )}
    </div>
  );
}
