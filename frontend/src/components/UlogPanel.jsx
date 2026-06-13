import { useRef, useState } from 'react';
import { api } from '../api.js';

/**
 * Post-flight ULog deep dive: drag-and-drop upload, then a structured
 * report (vibration peaks + notch advice, actuator saturation, EKF
 * delay/noise recommendations). Each recommendation can be staged as a
 * proposal with one click — still subject to safety + pilot approval.
 */
export default function UlogPanel() {
  const fileInput = useRef(null);
  const [drag, setDrag] = useState(false);
  const [busy, setBusy] = useState(false);
  const [report, setReport] = useState(null);
  const [error, setError] = useState(null);
  const [activeTab, setActiveTab] = useState('pid');

  const analyze = async (file) => {
    if (!file) return;
    setBusy(true); setError(null); setReport(null);
    try { setReport(await api.analyzeUlog(file)); }
    catch (e) { setError(e.message); }
    finally { setBusy(false); }
  };

  const stage = async (rec) => {
    try {
      await api.createProposal({
        param: rec.param, target_value: rec.proposed_value, rationale: rec.rationale,
      });
      if (rec.companion) {
        await api.createProposal({
          param: rec.companion.param,
          target_value: rec.companion.proposed_value,
          rationale: `Companion to ${rec.param}: notch bandwidth`,
        });
      }
    } catch (e) { setError(e.message); }
  };

  const recBlock = (rec) => rec && (
    <div className="proposal" style={{ marginTop: 8 }}>
      <div className="mono">{rec.param} → <strong style={{ color: 'var(--accent)' }}>{rec.proposed_value}</strong></div>
      <div className="muted" style={{ margin: '6px 0' }}>{rec.rationale}</div>
      <button className="btn primary" onClick={() => stage(rec)}>Stage as proposal</button>
    </div>
  );

  const sections = report?.sections;

  return (
    <div className="glass panel">
      <h2>Post-Flight ULog Deep Dive</h2>

      <div
        className={`dropzone ${drag ? 'active' : ''}`}
        onClick={() => fileInput.current?.click()}
        onDragOver={(e) => { e.preventDefault(); setDrag(true); }}
        onDragLeave={() => setDrag(false)}
        onDrop={(e) => { e.preventDefault(); setDrag(false); analyze(e.dataTransfer.files[0]); }}
      >
        {busy ? 'Analyzing… (large logs can take a minute)'
              : 'Drop a .ulg file here or click to browse'}
        <input ref={fileInput} type="file" accept=".ulg" hidden
               onChange={(e) => analyze(e.target.files[0])} />
      </div>

      {error && <div className="alert critical" style={{ marginTop: 12 }}>{error}</div>}

      {report && (
        <div className="stack" style={{ marginTop: 14 }}>
          <div className="muted mono" style={{ fontSize: '0.78rem', background: 'rgba(0,0,0,0.15)', padding: '8px 12px', borderRadius: 8, border: '1px solid var(--glass-border)' }}>
            📄 {report.original_filename} &nbsp;·&nbsp; ⏱️ {report.duration_s}s flight &nbsp;·&nbsp;
            💾 {(report.size_bytes / 2 ** 20).toFixed(1)} MiB &nbsp;·&nbsp; ⚙️ PX4 {report.px4_version}
            {report.airframe_label && <>&nbsp;·&nbsp; 🚀 {report.airframe_label}</>}
          </div>

          <div className="tabs-container" style={{ marginTop: 6 }}>
            <button className={`tab-btn ${activeTab === 'pid' ? 'active' : ''}`} onClick={() => setActiveTab('pid')}>
              🎯 PID Tuning
            </button>
            <button className={`tab-btn ${activeTab === 'vibration' ? 'active' : ''}`} onClick={() => setActiveTab('vibration')}>
              📶 Vibrations &amp; Filters
            </button>
            <button className={`tab-btn ${activeTab === 'actuator' ? 'active' : ''}`} onClick={() => setActiveTab('actuator')}>
              ⚡ Actuators
            </button>
            <button className={`tab-btn ${activeTab === 'ekf' ? 'active' : ''}`} onClick={() => setActiveTab('ekf')}>
              🧠 EKF Health
            </button>
          </div>

          {/* --- Tab Content: PID Loops --- */}
          {activeTab === 'pid' && (
            <div className="stack animate-fade">
              {sections.pid.skipped ? (
                <div className="muted">{sections.pid.skipped}</div>
              ) : (
                <>
                  <table className="modern-table">
                    <thead>
                      <tr>
                        <th>Axis / Mode</th>
                        <th>Steps</th>
                        <th>τ (Response)</th>
                        <th>Settling</th>
                        <th>Overshoot</th>
                        <th>Tracking (r)</th>
                      </tr>
                    </thead>
                    <tbody>
                      {Object.entries(sections.pid.axes ?? {}).flatMap(([ax, s]) => {
                        if (s && (s.mc || s.fw)) {
                          return [
                            { label: `${ax} (MC)`, data: s.mc },
                            { label: `${ax} (FW)`, data: s.fw }
                          ];
                        }
                        return [{ label: ax, data: s }];
                      }).map(({ label, data }) => (
                        <tr key={label}>
                          <td className="mono" style={{ fontWeight: 600 }}>{label}</td>
                          {(!data || data.n_steps === 0) ? (
                            <td colSpan={5} className="muted" style={{ textAlign: 'center', fontSize: '0.8rem' }}>no step maneuvers found</td>
                          ) : (
                            <>
                              <td>{data.n_steps}</td>
                              <td className="mono">{data.tau_s_median !== undefined && data.tau_s_median !== null ? `${data.tau_s_median}s` : '—'}</td>
                              <td className="mono">{data.settling_s_median !== undefined && data.settling_s_median !== null ? `${data.settling_s_median}s` : '—'}</td>
                              <td>
                                <span className={`pill ${data.overshoot_max > 0.25 ? 'warn' : 'ok'}`}>
                                  {data.overshoot_max !== undefined && data.overshoot_max !== null ? `${(data.overshoot_max * 100).toFixed(0)}%` : '—'}
                                </span>
                              </td>
                              <td>
                                <span className={`pill ${data.r !== null && data.r < 0.85 ? 'crit' : 'ok'}`}>
                                  {data.r !== undefined && data.r !== null ? data.r : '—'}
                                </span>
                              </td>
                            </>
                          )}
                        </tr>
                      ))}
                    </tbody>
                  </table>

                  <div className="stack" style={{ marginTop: 14, gap: 8 }}>
                    {sections.pid.notes?.map((n, i) => {
                      let severity = 'info';
                      if (n.includes('Check the actuator-saturation') || n.includes('no usable step')) {
                        severity = 'warning';
                      }
                      return (
                        <div key={i} className={`insight-item ${severity}`}>
                          <span className="insight-icon">{severity === 'warning' ? '⚠' : 'ℹ'}</span>
                          <div className="insight-content">{n}</div>
                        </div>
                      );
                    })}
                  </div>

                  {sections.pid.recommendations?.length > 0 && (
                    <div style={{ marginTop: 14 }}>
                      <h3 style={{ margin: '0 0 8px 0', fontSize: '0.78rem', color: 'var(--text-mid)', letterSpacing: '0.05em', textTransform: 'uppercase' }}>
                        Proposed Gain Adjustments
                      </h3>
                      {sections.pid.recommendations.map((r, i) => (
                        <div key={i} style={{ marginBottom: 8 }}>{recBlock(r)}</div>
                      ))}
                    </div>
                  )}
                </>
              )}
            </div>
          )}

          {/* --- Tab Content: Vibrations & Filters --- */}
          {activeTab === 'vibration' && (
            <div className="stack animate-fade">
              {sections.vibration.skipped ? (
                <div className="muted">{sections.vibration.skipped}</div>
              ) : (
                <>
                  <div style={{ display: 'grid', gridTemplateColumns: 'repeat(auto-fit, minmax(220px, 1fr))', gap: 12 }}>
                    {Object.entries(sections.vibration.axes).map(([ax, a]) => (
                      <div key={ax} className="proposal" style={{ padding: '12px 14px' }}>
                        <div style={{ fontSize: '0.7rem', textTransform: 'uppercase', color: 'var(--text-mid)', fontWeight: 600, letterSpacing: '0.05em', marginBottom: 4 }}>
                          Gyro {ax} Peak Noise
                        </div>
                        <div className="mono" style={{ fontSize: '0.95rem', fontWeight: 700, color: a.peaks.length ? 'var(--warn)' : 'var(--ok)' }}>
                          {a.peaks.length ? a.peaks.map((p) => `${p.freq_hz} Hz`).join(', ') : 'No significant peaks'}
                        </div>
                      </div>
                    ))}
                  </div>

                  {!sections.filters.skipped && (
                    <div className="proposal" style={{ display: 'grid', gridTemplateColumns: 'repeat(2, 1fr)', gap: 12, marginTop: 4 }}>
                      <div>
                        <span className="muted" style={{ fontSize: '0.7rem', textTransform: 'uppercase', letterSpacing: '0.05em' }}>Spectrum Class</span>
                        <div className="mono" style={{ fontSize: '0.85rem', fontWeight: 600, marginTop: 2 }}>{sections.filters.spectrum_class}</div>
                      </div>
                      <div>
                        <span className="muted" style={{ fontSize: '0.7rem', textTransform: 'uppercase', letterSpacing: '0.05em' }}>Gyro HF Noise</span>
                        <div className="mono" style={{ fontSize: '0.85rem', fontWeight: 600, marginTop: 2, color: sections.filters.gyro_hf_rms_rad_s > 0.05 ? 'var(--warn)' : 'var(--ok)' }}>
                          {sections.filters.gyro_hf_rms_rad_s ?? '—'} rad/s
                        </div>
                      </div>
                      <div>
                        <span className="muted" style={{ fontSize: '0.7rem', textTransform: 'uppercase', letterSpacing: '0.05em' }}>Filter Phase Lag</span>
                        <div className="mono" style={{ fontSize: '0.85rem', fontWeight: 600, marginTop: 2 }}>{sections.filters.filter_delay_ms ?? '—'} ms</div>
                      </div>
                      <div>
                        <span className="muted" style={{ fontSize: '0.7rem', textTransform: 'uppercase', letterSpacing: '0.05em' }}>Notch Scheme</span>
                        <div className="mono" style={{ fontSize: '0.85rem', fontWeight: 600, marginTop: 2 }}>{sections.filters.notch_naming}</div>
                      </div>
                    </div>
                  )}

                  <div className="stack" style={{ gap: 8 }}>
                    {sections.filters.notes?.map((n, i) => (
                      <div key={i} className="insight-item info">
                        <span className="insight-icon">ℹ</span>
                        <div className="insight-content">{n}</div>
                      </div>
                    ))}
                  </div>

                  {(sections.vibration.recommendation || (sections.filters.recommendations && sections.filters.recommendations.length > 0)) && (
                    <div style={{ marginTop: 8 }}>
                      <h3 style={{ margin: '0 0 8px 0', fontSize: '0.78rem', color: 'var(--text-mid)', letterSpacing: '0.05em', textTransform: 'uppercase' }}>
                        Proposed Filter Changes
                      </h3>
                      {recBlock(sections.vibration.recommendation)}
                      {sections.filters.recommendations?.map((r, i) => (
                        <div key={i} style={{ marginTop: 8 }}>{recBlock(r)}</div>
                      ))}
                    </div>
                  )}
                </>
              )}
            </div>
          )}

          {/* --- Tab Content: Actuator Saturation --- */}
          {activeTab === 'actuator' && (
            <div className="stack animate-fade">
              {sections.actuator_saturation.skipped ? (
                <div className="muted">{sections.actuator_saturation.skipped}</div>
              ) : (
                <>
                  <div className="stack" style={{ gap: 12 }}>
                    {Object.entries(sections.actuator_saturation.channels).map(([ch, c]) => (
                      <div key={ch} className="gauge">
                        <div className="spread row" style={{ fontSize: '0.8rem', marginBottom: 4 }}>
                          <span className="mono" style={{ fontWeight: 600 }}>{ch}</span>
                          <span className="mono" style={{ color: c.flagged ? 'var(--crit)' : 'var(--text-mid)' }}>
                            {c.saturated_pct}% saturated {c.longest_burst_s > 0 && `(longest burst: ${c.longest_burst_s}s)`}
                          </span>
                        </div>
                        <div className="gauge-track">
                          <div className={`gauge-fill ${c.flagged ? 'crit' : c.saturated_pct > 5 ? 'warn' : 'ok'}`} style={{ width: `${Math.min(c.saturated_pct || 1, 100)}%` }}></div>
                        </div>
                      </div>
                    ))}
                  </div>

                  {sections.actuator_saturation.advice && (
                    <div className="insight-item warning" style={{ marginTop: 8 }}>
                      <span className="insight-icon">⚠</span>
                      <div className="insight-content">{sections.actuator_saturation.advice}</div>
                    </div>
                  )}
                </>
              )}
            </div>
          )}

          {/* --- Tab Content: EKF Health --- */}
          {activeTab === 'ekf' && (
            <div className="stack animate-fade">
              <div className="stack" style={{ gap: 12 }}>
                {sections.ekf_delays.skipped ? (
                  <div className="muted">{sections.ekf_delays.skipped}</div>
                ) : (
                  <div className="proposal">
                    <div style={{ fontSize: '0.7rem', textTransform: 'uppercase', color: 'var(--text-mid)', fontWeight: 600, letterSpacing: '0.05em', marginBottom: 6 }}>
                      GPS Delay Diagnostic
                    </div>
                    <div style={{ fontSize: '0.88rem' }}>
                      Measured Delay: <strong style={{ color: 'var(--accent)' }}>{sections.ekf_delays.measured_gps_delay_ms} ms</strong> &nbsp;·&nbsp; Currently Configured: <span className="mono">{sections.ekf_delays.current_EKF2_GPS_DELAY} ms</span>
                    </div>
                    {recBlock(sections.ekf_delays.recommendation)}
                  </div>
                )}

                {sections.ekf_delays.baro && (
                  sections.ekf_delays.baro.skipped ? (
                    <div className="muted">Baro: {sections.ekf_delays.baro.skipped}</div>
                  ) : (
                    <div className="proposal">
                      <div style={{ fontSize: '0.7rem', textTransform: 'uppercase', color: 'var(--text-mid)', fontWeight: 600, letterSpacing: '0.05em', marginBottom: 6 }}>
                        Barometer Delay Diagnostic
                      </div>
                      <div style={{ fontSize: '0.88rem' }}>
                        Measured Delay: <strong style={{ color: 'var(--accent)' }}>{sections.ekf_delays.baro.measured_baro_delay_ms} ms</strong> &nbsp;·&nbsp; Currently Configured: <span className="mono">{sections.ekf_delays.baro.current_EKF2_BARO_DELAY} ms</span>
                      </div>
                      {recBlock(sections.ekf_delays.baro.recommendation)}
                    </div>
                  )
                )}

                {sections.ekf_noise.skipped ? (
                  <div className="muted">{sections.ekf_noise.skipped}</div>
                ) : (
                  <div className="proposal">
                    <div style={{ fontSize: '0.7rem', textTransform: 'uppercase', color: 'var(--text-mid)', fontWeight: 600, letterSpacing: '0.05em', marginBottom: 6 }}>
                      Hover Vibration Floor
                    </div>
                    <div style={{ fontSize: '0.88rem' }}>
                      Accel Noise Floor: <strong style={{ color: sections.ekf_noise.accel_std_m_s2 > 1.5 ? 'var(--warn)' : 'var(--ok)' }}>{sections.ekf_noise.accel_std_m_s2} m/s²</strong> &nbsp;·&nbsp; Gyro Noise Floor: <span className="mono">{sections.ekf_noise.gyro_std_rad_s} rad/s</span>
                    </div>
                    {sections.ekf_noise.recommendations?.map((r, i) => (
                      <div key={i} style={{ marginTop: 8 }}>{recBlock(r)}</div>
                    ))}
                  </div>
                )}
              </div>
            </div>
          )}

        </div>
      )}
    </div>
  );
}
