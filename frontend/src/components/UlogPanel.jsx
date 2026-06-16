import { useEffect, useRef, useState } from 'react';
import { api } from '../api.js';
import StepResponseEnvelope from './StepResponseEnvelope.jsx';
import ActuatorTimelineChart from './ActuatorTimelineChart.jsx';
import MotorBalanceChart from './MotorBalanceChart.jsx';
import SingleChannelCompareChart from './SingleChannelCompareChart.jsx';

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
  const [selectedEnvelopeAxis, setSelectedEnvelopeAxis] = useState('');
  const [modeFilter, setModeFilter] = useState('all');
  const [selectedChannel, setSelectedChannel] = useState('');

  useEffect(() => {
    api.host().then((info) => {
      if (info && info.backend_session_id) {
        const saved = sessionStorage.getItem('mint_backend_session_id');
        if (saved && saved !== info.backend_session_id) {
          // Backend has restarted, invalidate frontend session data
          sessionStorage.clear();
          setReport(null);
          sessionStorage.setItem('mint_backend_session_id', info.backend_session_id);
        } else {
          sessionStorage.setItem('mint_backend_session_id', info.backend_session_id);
          restoreReport();
        }
      } else {
        restoreReport();
      }
    }).catch((e) => {
      console.warn("Failed to verify backend session status:", e);
      restoreReport();
    });

    function restoreReport() {
      const savedHash = sessionStorage.getItem('mint_active_report_hash');
      if (savedHash) {
        setBusy(true);
        api.getReport(savedHash)
          .then((res) => {
            setReport(res);
          })
          .catch((e) => {
            console.warn("Failed to restore active report from hash:", e);
            sessionStorage.removeItem('mint_active_report_hash');
          })
          .finally(() => {
            setBusy(false);
          });
      }
    }
  }, []);

  const analyze = async (file) => {
    if (!file) return;
    setBusy(true); setError(null); setReport(null);
    sessionStorage.removeItem('mint_active_report_hash');
    try {
      const result = await api.analyzeUlog(file);
      setReport(result);
      if (result.file_hash) {
        sessionStorage.setItem('mint_active_report_hash', result.file_hash);
      }
    }
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

  const renderConfidence = (confidence) => {
    if (!confidence || confidence.score === undefined) return null;
    const score = confidence.score;
    const flags = confidence.flags || {};
    
    const explanations = {
      pre_step_motion: "Pre-step motion present – may reduce accuracy.",
      ramped_input: "Input was ramped – τ adjusted.",
      low_coherence: "Low coherence in control band – step may be noisy.",
      short_post_window: "Post-step window truncated – oscillation count uncertain.",
      derived_from_attitude: "Yaw analysis derived from attitude setpoint (lower confidence).",
    };
    
    const activeFlags = Object.entries(flags)
      .filter(([key, val]) => {
        if (key === 'osc_reliable') return !val;
        return !!val;
      })
      .map(([key]) => key);
      
    const tooltipText = activeFlags.length > 0
      ? activeFlags.map(f => f === 'osc_reliable' ? "Oscillation count uncertain." : explanations[f]).join('\n')
      : "High confidence data.";

    return (
      <div className="confidence-badge" title={tooltipText} style={{ display: 'inline-flex', alignItems: 'center', gap: 4, cursor: 'help', fontSize: '0.74rem', background: 'rgba(255,255,255,0.06)', padding: '2px 6px', borderRadius: 4 }}>
        <span className="muted">Confidence:</span>
        <strong style={{ color: score > 0.7 ? 'var(--ok)' : score > 0.4 ? 'var(--warn)' : 'var(--crit)' }}>
          {Math.round(score * 100)}%
        </strong>
        <span>ⓘ</span>
      </div>
    );
  };

  const recBlock = (rec) => rec && (
    <div className="proposal" style={{ marginTop: 8 }}>
      <div className="row spread" style={{ alignItems: 'center', marginBottom: 4 }}>
        <div className="mono">{rec.param} → <strong style={{ color: 'var(--accent)' }}>{rec.proposed_value}</strong></div>
        {renderConfidence(rec.confidence)}
      </div>
      {rec.limitations && (
        <div style={{ fontSize: '0.76rem', color: '#fbbf24', marginBottom: '6px', fontWeight: 500 }}>
          ⚠️ {rec.limitations}
        </div>
      )}
      <div className="muted" style={{ margin: '6px 0' }}>{rec.rationale}</div>
      <button className="btn primary" onClick={() => stage(rec)}>Stage as proposal</button>
    </div>
  );

  const sections = report?.sections;

  const getAxesDatasets = () => {
    if (!sections?.pid?.axes) return [];
    return Object.entries(sections.pid.axes).flatMap(([ax, s]) => {
      const isBody = ax.includes('(body)');
      const loopType = isBody ? 'body' : 'rate';
      const axisName = ax.split(' ')[0]; // roll, pitch, yaw
      if (s && (s.mc || s.fw)) {
        return [
          { label: `${ax} (MC)`, key: `${ax}_mc`, data: s.mc, loopType, mode: 'mc', axisName },
          { label: `${ax} (FW)`, key: `${ax}_fw`, data: s.fw, loopType, mode: 'fw', axisName }
        ];
      }
      return [{ label: ax, key: ax, data: s, loopType, mode: 'all', axisName }];
    });
  };

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
              🎯 PID Tuning &amp; Envelope
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
                  <div className="muted" style={{ fontSize: '0.78rem', marginBottom: 6 }}>
                    💡 Click any row in the table below to inspect its step response envelope, gain recommendations, and insights.
                  </div>

                  {(() => {
                    const datasets = getAxesDatasets();
                    const hasVtolModes = datasets.some(d => d.mode === 'mc' || d.mode === 'fw');
                    
                    const filteredDatasets = datasets.filter(ds => {
                      if (!hasVtolModes) return true;
                      if (modeFilter === 'all') return true;
                      return ds.mode === modeFilter;
                    });
                    
                    const currentDatasetKey = selectedEnvelopeAxis || (filteredDatasets[0]?.key ?? '');
                    const activeDataset = filteredDatasets.find(d => d.key === currentDatasetKey) || filteredDatasets[0];
                    const activeKey = activeDataset?.key ?? '';

                    const rateDatasets = filteredDatasets.filter(ds => ds.loopType === 'rate');
                    const bodyDatasets = filteredDatasets.filter(ds => ds.loopType === 'body');

                    const renderRow = (ds) => {
                      const isSelected = ds.key === activeKey;
                      const data = ds.data;
                      return (
                        <tr
                          key={ds.key}
                          className={`clickable-row ${isSelected ? 'selected' : ''}`}
                          onClick={() => setSelectedEnvelopeAxis(ds.key)}
                          style={{
                            cursor: 'pointer',
                            backgroundColor: isSelected ? 'rgba(79, 140, 255, 0.08)' : undefined,
                            borderLeft: isSelected ? '4px solid var(--accent)' : '4px solid transparent',
                            transition: 'all 0.15s ease',
                          }}
                        >
                          <td className="mono" style={{ fontWeight: 600, paddingLeft: isSelected ? '6px' : '10px' }}>
                            {ds.label} {isSelected && <span style={{ color: 'var(--accent)', marginLeft: 4 }}>•</span>}
                          </td>
                          {(!data || data.n_steps === 0) ? (
                            <td colSpan={5} className="muted" style={{ textAlign: 'center', fontSize: '0.8rem' }}>
                              no step maneuvers found
                              {data?.candidates > 0 && (
                                <span className="muted" style={{ fontSize: '0.74rem', display: 'block', cursor: 'help' }} title={
                                  data.rejections ? Object.entries(data.rejections)
                                    .filter(([_, val]) => val > 0)
                                    .map(([key, val]) => `${val} rejected due to ${key.replace(/_/g, ' ')}`)
                                    .join('\n') : ''
                                }>
                                  ({data.candidates} candidates detected but all rejected ⓘ)
                                </span>
                              )}
                            </td>
                          ) : (
                            <>
                              <td>
                                <span style={{ fontWeight: 'bold' }}>{data.n_steps}</span>
                                {data.candidates !== undefined && data.candidates > data.n_steps && (
                                  <span className="muted" style={{ fontSize: '0.74rem', display: 'block', cursor: 'help' }} title={
                                    data.rejections ? Object.entries(data.rejections)
                                      .filter(([_, val]) => val > 0)
                                      .map(([key, val]) => `${val} rejected due to ${key.replace(/_/g, ' ')}`)
                                      .join('\n') : ''
                                  }>
                                    ({data.candidates} cand) ⓘ
                                  </span>
                                )}
                              </td>
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
                      );
                    };

                    // Filter recommendations & notes for activeDataset
                    const isRecForDataset = (r, ds) => {
                      if (!ds) return false;
                      const baseAx = ds.key.replace(/_(mc|fw)$/, '');
                      const mode = ds.key.endsWith('_mc') ? 'mc' : ds.key.endsWith('_fw') ? 'fw' : null;
                      
                      const matchesAxis = r.rationale.toLowerCase().includes(baseAx.toLowerCase()) || 
                                          r.param.toLowerCase().includes(baseAx.split(' ')[0]);
                      
                      if (!matchesAxis) return false;

                      // Check loop type compatibility to prevent mixing rate and attitude parameters
                      const isParamRate = r.param.includes('RATE') || r.param.startsWith('FW_RR') || r.param.startsWith('FW_PR') || r.param.startsWith('FW_YR');
                      const isParamBody = r.param.endsWith('_TC') || r.param === 'MC_ROLL_P' || r.param === 'MC_PITCH_P' || r.param === 'MC_YAW_P';
                      const matchesLoop = ds.loopType === 'rate' ? isParamRate : ds.loopType === 'body' ? isParamBody : true;
                      if (!matchesLoop) return false;
                      
                      if (mode) {
                        return r.rationale.toLowerCase().includes(`${mode} mode`);
                      }
                      return true;
                    };

                    const isNoteForDataset = (n, ds) => {
                      if (!ds) return false;
                      const baseAx = ds.key.replace(/_(mc|fw)$/, '');
                      const mode = ds.key.endsWith('_mc') ? 'mc' : ds.key.endsWith('_fw') ? 'fw' : null;
                      
                      const matchesAxis = n.toLowerCase().includes(baseAx.toLowerCase());
                      if (!matchesAxis) return false;
                      if (mode) {
                        return n.toLowerCase().includes(`(${mode})`) || n.toLowerCase().includes(` (${mode.toUpperCase()})`);
                      }
                      return true;
                    };

                    const axisRecs = sections.pid.recommendations?.filter(r => isRecForDataset(r, activeDataset)) ?? [];
                    const axisNotes = sections.pid.notes?.filter(n => isNoteForDataset(n, activeDataset)) ?? [];

                    return (
                      <>
                        {hasVtolModes && (
                          <div className="row" style={{ marginBottom: 10, gap: 10, flexWrap: 'wrap' }}>
                            <span className="muted" style={{ fontSize: '0.8rem', fontWeight: 600 }}>Filter Mode:</span>
                            <div className="topbar-nav" style={{ margin: 0 }}>
                              <button
                                className={`nav-link ${modeFilter === 'mc' ? 'active' : ''}`}
                                onClick={() => setModeFilter('mc')}
                                style={{ background: 'none', border: 'none', cursor: 'pointer' }}
                              >
                                Multirotor (MC)
                              </button>
                              <button
                                className={`nav-link ${modeFilter === 'fw' ? 'active' : ''}`}
                                onClick={() => setModeFilter('fw')}
                                style={{ background: 'none', border: 'none', cursor: 'pointer' }}
                              >
                                Fixed-Wing (FW)
                              </button>
                              <button
                                className={`nav-link ${modeFilter === 'all' ? 'active' : ''}`}
                                onClick={() => setModeFilter('all')}
                                style={{ background: 'none', border: 'none', cursor: 'pointer' }}
                              >
                                Show All
                              </button>
                            </div>
                          </div>
                        )}

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
                            {/* Inner Loop: Rate Control Header */}
                            <tr style={{ background: 'rgba(255, 255, 255, 0.02)' }}>
                              <td colSpan={6} style={{ padding: '10px 12px', fontWeight: 'bold', fontSize: '0.78rem', color: 'var(--accent)', textTransform: 'uppercase', letterSpacing: '0.05em' }}>
                                🔄 Inner Loop: Rate Control (rad/s)
                              </td>
                            </tr>
                            {rateDatasets.length > 0 ? rateDatasets.map(renderRow) : (
                              <tr>
                                <td colSpan={6} className="muted" style={{ textAlign: 'center', fontSize: '0.8rem' }}>
                                  no rate datasets found
                                </td>
                              </tr>
                            )}

                            {/* Outer Loop: Body Attitude Control Header */}
                            <tr style={{ background: 'rgba(255, 255, 255, 0.02)' }}>
                              <td colSpan={6} style={{ padding: '16px 12px 10px 12px', fontWeight: 'bold', fontSize: '0.78rem', color: 'var(--accent)', textTransform: 'uppercase', letterSpacing: '0.05em' }}>
                                📐 Outer Loop: Body Attitude Control (deg)
                              </td>
                            </tr>
                            {bodyDatasets.length > 0 ? bodyDatasets.map(renderRow) : (
                              <tr>
                                <td colSpan={6} className="muted" style={{ textAlign: 'center', fontSize: '0.8rem' }}>
                                  no attitude datasets found
                                </td>
                              </tr>
                            )}
                          </tbody>
                        </table>

                        {/* Selected Axis Deep Dive Area */}
                        {activeDataset && (
                          <div className="proposal" style={{ marginTop: 16, padding: '16px', background: 'rgba(255, 255, 255, 0.02)', border: '1px solid var(--glass-border)', borderRadius: '12px' }}>
                            <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: 12, borderBottom: '1px solid var(--glass-border)', paddingBottom: 8 }}>
                              <h3 style={{ margin: 0, fontSize: '0.9rem', color: 'var(--text)' }}>
                                📊 Axis Deep Dive: <strong style={{ color: 'var(--accent)' }}>{activeDataset.label}</strong>
                              </h3>
                            </div>

                        {activeDataset.data && activeDataset.data.n_steps > 0 ? (
                          <div className="stack" style={{ gap: 14, marginBottom: 16 }}>
                            <StepResponseEnvelope
                              steps={activeDataset.data.steps}
                              tau={activeDataset.data.tau_s_median}
                              settling={activeDataset.data.settling_s_median}
                              overshoot={activeDataset.data.overshoot_max}
                              axisLabel={activeDataset.label}
                            />
                          </div>
                        ) : (
                          <div className="alert info" style={{ margin: '8px 0 16px 0' }}>
                            ℹ️ No step maneuvers found for <strong>{activeDataset.label}</strong>. Fly sharp alternating stick pulses (deflections ≥ 0.15 rad/s for rate, ≥ 5° for body) to enable step-response envelope plots.
                          </div>
                        )}

                        {/* Adjustments & Notes specific to this axis */}
                        <div style={{ display: 'grid', gridTemplateColumns: '1fr', gap: 12 }}>
                          {axisRecs.length > 0 && (
                            <div>
                              <h4 style={{ margin: '0 0 8px 0', fontSize: '0.74rem', color: 'var(--text-mid)', letterSpacing: '0.05em', textTransform: 'uppercase' }}>
                                Proposed Adjustments
                              </h4>
                              {axisRecs.map((r, i) => (
                                <div key={i} style={{ marginBottom: 8 }}>{recBlock(r)}</div>
                              ))}
                            </div>
                          )}

                          {axisNotes.length > 0 && (
                            <div>
                              <h4 style={{ margin: '0 0 6px 0', fontSize: '0.74rem', color: 'var(--text-mid)', letterSpacing: '0.05em', textTransform: 'uppercase' }}>
                                Diagnostic Insights
                              </h4>
                              <div className="stack" style={{ gap: 6 }}>
                                {axisNotes.map((n, i) => {
                                  let severity = 'info';
                                  if (n.includes('Check the actuator-saturation') || n.includes('no usable step')) {
                                    severity = 'warning';
                                  }
                                  return (
                                    <div key={i} className={`insight-item ${severity}`} style={{ padding: '8px 12px', fontSize: '0.8rem' }}>
                                      <span className="insight-icon">{severity === 'warning' ? '⚠' : 'ℹ'}</span>
                                      <div className="insight-content">{n}</div>
                                    </div>
                                  );
                                })}
                              </div>
                            </div>
                          )}
                        </div>
                      </div>
                    )}
                  </>
                );
              })()}
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
            <div className="stack animate-fade" style={{ gap: 16 }}>
              {sections.actuator_saturation.skipped ? (
                <div className="muted">{sections.actuator_saturation.skipped}</div>
              ) : (
                <>
                  {/* Timeline Chart */}
                  {sections.actuator_saturation.t && sections.actuator_saturation.t.length > 0 && (
                    <ActuatorTimelineChart
                      t={sections.actuator_saturation.t}
                      channels={sections.actuator_saturation.channels}
                      airspeed={sections.actuator_saturation.airspeed}
                    />
                  )}

                  {/* Motor Balance History Chart */}
                  {sections.actuator_saturation.motor_balance && (
                    <MotorBalanceChart balance={sections.actuator_saturation.motor_balance} />
                  )}

                  <div style={{ display: 'grid', gridTemplateColumns: '260px 1fr', gap: 14, alignItems: 'start' }}>
                    {/* Channel stats list */}
                    <div className="glass panel stack" style={{ gap: 12, padding: 14 }}>
                      <h3 style={{ margin: 0, fontSize: '0.8rem', textTransform: 'uppercase', color: 'var(--accent)', letterSpacing: '0.05em' }}>
                        Channel Outputs
                      </h3>
                      <div className="stack" style={{ gap: 8 }}>
                        {Object.entries(sections.actuator_saturation.channels).map(([ch, c]) => {
                          const isSelected = (selectedChannel || Object.keys(sections.actuator_saturation.channels)[0]) === ch;
                          return (
                            <div
                              key={ch}
                              className="stack"
                              style={{
                                gap: 4,
                                cursor: 'pointer',
                                padding: '8px 10px',
                                borderRadius: 8,
                                background: isSelected ? 'rgba(79, 140, 255, 0.08)' : 'transparent',
                                border: isSelected ? '1px solid var(--accent)' : '1px solid transparent',
                                transition: 'all 0.15s ease'
                              }}
                              onClick={() => setSelectedChannel(ch)}
                            >
                              <div className="spread row" style={{ fontSize: '0.76rem' }}>
                                <span className="mono" style={{ fontWeight: 600 }}>{ch}</span>
                                <span className="mono" style={{ color: c.flagged ? 'var(--crit)' : 'var(--text-mid)' }}>
                                  {c.saturated_pct}% sat
                                </span>
                              </div>
                              <div className="gauge-track">
                                <div className={`gauge-fill ${c.flagged ? 'crit' : c.saturated_pct > 5 ? 'warn' : 'ok'}`} style={{ width: `${Math.min(c.saturated_pct || 1, 100)}%` }}></div>
                              </div>
                            </div>
                          );
                        })}
                      </div>
                    </div>

                    {/* Insights & Comparison Chart Panel */}
                    <div className="stack" style={{ gap: 14 }}>
                      {(() => {
                        const activeKey = selectedChannel || Object.keys(sections.actuator_saturation.channels)[0];
                        const activeChData = sections.actuator_saturation.channels[activeKey];
                        if (!activeChData) return null;

                        return (
                          <>
                            {/* Command vs Actual Tracking Comparison Chart */}
                            <div className="glass panel" style={{ padding: 14 }}>
                              <SingleChannelCompareChart
                                t={sections.actuator_saturation.t}
                                command={activeChData.command_values}
                                actual={activeChData.values}
                                label={activeKey}
                                axis={activeChData.correlated_axis || ''}
                              />
                            </div>

                            <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: 14, alignItems: 'start' }}>
                              {/* Diagnostic Details */}
                              <div className="glass panel stack" style={{ gap: 10, padding: 14 }}>
                                <h4 style={{ margin: 0, fontSize: '0.78rem', textTransform: 'uppercase', color: 'var(--accent)', letterSpacing: '0.05em' }}>
                                  Diagnostics: {activeKey}
                                </h4>
                                <div className="stack" style={{ gap: 8, fontSize: '0.74rem' }}>
                                  {activeChData.note && (
                                    <div className="insight-item warning" style={{ margin: 0, padding: '8px 10px' }}>
                                      <span className="insight-icon">ℹ️</span>
                                      <div className="insight-content">{activeChData.note}</div>
                                    </div>
                                  )}
                                  
                                  {activeChData.correlated_axis ? (
                                    <div className="row spread mono" style={{ padding: '6px 8px', background: 'rgba(255,255,255,0.03)', borderRadius: 4 }}>
                                      <span>Primary Command:</span>
                                      <strong style={{ color: 'var(--text)' }}>
                                        {activeChData.correlated_axis} (r = {activeChData.correlation})
                                      </strong>
                                    </div>
                                  ) : (
                                    <div className="muted mono" style={{ padding: '6px 8px' }}>
                                      No strongly correlated command axis detected for this channel.
                                    </div>
                                  )}

                                  {activeChData.command_mismatch_duration_s > 0 && (
                                    <div className="insight-item critical" style={{ margin: 0, padding: '8px 10px' }}>
                                      <span className="insight-icon">⚠️</span>
                                      <div className="insight-content">
                                        <strong>Mismatch Saturation:</strong> Demand did not track physical output for up to {activeChData.command_mismatch_duration_s}s.
                                      </div>
                                    </div>
                                  )}
                                </div>
                              </div>

                              {/* Saturation Advice & Guide */}
                              <div className="stack" style={{ gap: 10 }}>
                                {sections.actuator_saturation.advice && (
                                  <div className="insight-item warning" style={{ margin: 0, padding: '8px 10px' }}>
                                    <span className="insight-icon">⚠</span>
                                    <div className="insight-content" style={{ fontSize: '0.78rem' }}>
                                      <strong>Saturation Advice:</strong><br />
                                      {sections.actuator_saturation.advice}
                                    </div>
                                  </div>
                                )}
                                
                                <div className="glass panel stack" style={{ gap: 8, padding: 14 }}>
                                  <h4 style={{ margin: 0, fontSize: '0.76rem', textTransform: 'uppercase', color: 'var(--text-mid)', letterSpacing: '0.05em' }}>
                                    Diagnostic Quick Guide
                                  </h4>
                                  <ul className="muted" style={{ fontSize: '0.72rem', paddingLeft: 16, margin: 0, listStyleType: 'disc' }}>
                                    <li style={{ marginBottom: 4 }}>
                                      <strong>Motors:</strong> Saturation above 95% points to underpowered airframes, excessive payload, or battery sag.
                                    </li>
                                    <li style={{ marginBottom: 4 }}>
                                      <strong>Surfaces:</strong> Deflection at cruise (&gt; 12 m/s) points to gains too high or mechanical link limitations.
                                    </li>
                                  </ul>
                                </div>
                              </div>
                            </div>
                          </>
                        );
                      })()}
                    </div>
                  </div>
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
