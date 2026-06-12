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
          <div className="muted mono">
            {report.original_filename} · {report.duration_s}s flight ·
            {' '}{(report.size_bytes / 2 ** 20).toFixed(1)} MiB · PX4 {report.px4_version}
            {report.airframe_label && <> · {report.airframe_label}</>}
          </div>

          <section>
            <h2>PID Rate Tracking</h2>
            {sections.pid.skipped
              ? <div className="muted">{sections.pid.skipped}</div>
              : <>
                  {Object.entries(sections.pid.axes ?? {}).map(([ax, s]) => (
                    <div key={ax} className="muted mono">
                      {ax}: {s.n_steps === 0
                        ? 'no step maneuvers found'
                        : <>
                            {s.n_steps} steps · τ {s.tau_s_median ?? '—'}s ·
                            settle {s.settling_s_median ?? '—'}s ·{' '}
                            <span style={{ color: s.overshoot_max > 0.25 ? 'var(--warn)' : undefined }}>
                              overshoot {(s.overshoot_max * 100).toFixed(0)}%
                            </span>{' · '}
                            <span style={{ color: s.r !== null && s.r < 0.85 ? 'var(--crit)' : undefined }}>
                              r {s.r ?? '—'}
                            </span> · ε {s.nrmse ?? '—'}
                          </>}
                    </div>
                  ))}
                  {sections.pid.notes?.map((n, i) => (
                    <div key={i} className="muted" style={{ marginTop: 4 }}>ℹ {n}</div>
                  ))}
                  {sections.pid.recommendations?.map((r) => recBlock(r))}
                </>}
          </section>

          <section>
            <h2>Vibration / FFT</h2>
            {sections.vibration.skipped
              ? <div className="muted">{sections.vibration.skipped}</div>
              : <>
                  {Object.entries(sections.vibration.axes).map(([ax, a]) => (
                    <div key={ax} className="muted">
                      gyro {ax}: {a.peaks.length
                        ? a.peaks.map((p) => `${p.freq_hz} Hz`).join(', ')
                        : 'no significant peaks'}
                    </div>
                  ))}
                  {recBlock(sections.vibration.recommendation)}
                </>}
          </section>

          <section>
            <h2>Filter Tuning</h2>
            {sections.filters.skipped
              ? <div className="muted">{sections.filters.skipped}</div>
              : <>
                  <div className="muted mono">
                    spectrum: {sections.filters.spectrum_class} · gyro HF noise{' '}
                    {sections.filters.gyro_hf_rms_rad_s ?? '—'} rad/s ·
                    filter lag {sections.filters.filter_delay_ms ?? '—'} ms ·
                    notch scheme {sections.filters.notch_naming}
                    {sections.filters.harmonics_of_hz &&
                      <> · harmonics of {sections.filters.harmonics_of_hz} Hz</>}
                  </div>
                  {sections.filters.notes?.map((n, i) => (
                    <div key={i} className="muted" style={{ marginTop: 4 }}>ℹ {n}</div>
                  ))}
                  {sections.filters.recommendations?.map((r) => recBlock(r))}
                </>}
          </section>

          <section>
            <h2>Actuator Saturation</h2>
            {sections.actuator_saturation.skipped
              ? <div className="muted">{sections.actuator_saturation.skipped}</div>
              : <>
                  {Object.entries(sections.actuator_saturation.channels).map(([ch, c]) => (
                    <div key={ch} className="muted" style={{ color: c.flagged ? 'var(--warn)' : undefined }}>
                      {ch}: {c.saturated_pct}% saturated, longest burst {c.longest_burst_s}s
                      {c.flagged && ' ⚠'}
                    </div>
                  ))}
                  {sections.actuator_saturation.advice && (
                    <div className="alert warning" style={{ marginTop: 8 }}>
                      {sections.actuator_saturation.advice}
                    </div>
                  )}
                </>}
          </section>

          <section>
            <h2>EKF Timing &amp; Noise</h2>
            {sections.ekf_delays.skipped
              ? <div className="muted">{sections.ekf_delays.skipped}</div>
              : <>
                  <div className="muted">
                    Measured GPS delay: {sections.ekf_delays.measured_gps_delay_ms} ms
                    (configured {sections.ekf_delays.current_EKF2_GPS_DELAY} ms)
                  </div>
                  {recBlock(sections.ekf_delays.recommendation)}
                </>}
            {sections.ekf_delays.baro && (sections.ekf_delays.baro.skipped
              ? <div className="muted">baro: {sections.ekf_delays.baro.skipped}</div>
              : <>
                  <div className="muted">
                    Measured baro delay: {sections.ekf_delays.baro.measured_baro_delay_ms} ms
                    (configured {sections.ekf_delays.baro.current_EKF2_BARO_DELAY} ms)
                  </div>
                  {recBlock(sections.ekf_delays.baro.recommendation)}
                </>)}
            {sections.ekf_noise.skipped
              ? <div className="muted">{sections.ekf_noise.skipped}</div>
              : <>
                  <div className="muted">
                    Hover floor — accel {sections.ekf_noise.accel_std_m_s2} m/s²,
                    gyro {sections.ekf_noise.gyro_std_rad_s} rad/s
                  </div>
                  {sections.ekf_noise.recommendations?.map((r) => recBlock(r))}
                </>}
          </section>
        </div>
      )}
    </div>
  );
}
