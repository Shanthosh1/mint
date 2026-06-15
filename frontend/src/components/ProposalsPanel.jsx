import { useEffect, useState } from 'react';
import { api } from '../api.js';
import { useTelemetryChannel } from '../hooks/useTelemetry.js';

/**
 * Human-In-The-Loop proposal queue.
 *
 * Every card shows current → proposed value, the analyzer's rationale,
 * and the safety registry's verdict. Writing requires the explicit
 * "Approve & Write" click — there is no auto-apply anywhere in the app.
 */
export default function ProposalsPanel() {
  const [proposals, setProposals] = useState([]);
  const [busyId, setBusyId] = useState(null);
  const [error, setError] = useState(null);
  const [host, setHost] = useState(null);
  const [confirmedRisks, setConfirmedRisks] = useState({});

  const refresh = () => api.proposals().then(setProposals).catch(() => {});
  useEffect(() => {
    refresh();
    api.host().then(setHost).catch(() => {});
  }, []);
  useTelemetryChannel('proposal', (d) => {
    if (d) {
      if (d.clear) {
        // Disconnect cleared all proposals on backend — reset UI instantly
        setProposals([]);
      } else {
        refresh();
      }
    }
  });

  const getAxis = (param) => {
    if (param.includes('ROLL')) return 'roll';
    if (param.includes('PITCH')) return 'pitch';
    if (param.includes('YAW')) return 'yaw';
    return null;
  };

  const hasPendingFeedbackOnAxis = (param) => {
    const axis = getAxis(param);
    if (!axis) return false;
    return proposals.some(x => 
      x.state === 'written' && 
      !x.feedback && 
      x.is_saturation_gain_reduction && 
      getAxis(x.param) === axis
    );
  };

  const approve = async (id) => {
    setBusyId(id); setError(null);
    try { await api.approveProposal(id); }
    catch (e) { setError(e.message); }
    finally { setBusyId(null); refresh(); }
  };

  const dismiss = async (id) => {
    await api.dismissProposal(id).catch(() => {});
    refresh();
  };

  const giveFeedback = async (id, outcome) => {
    setError(null);
    try { await api.proposalFeedback(id, outcome); }
    catch (e) { setError(e.message); }
    finally { refresh(); }
  };

  const revert = async (id) => {
    setBusyId(id); setError(null);
    try { await api.revertProposal(id); }
    catch (e) { setError(e.message); }
    finally { setBusyId(null); refresh(); }
  };

  const clearAll = async () => {
    await Promise.all(proposals.map((p) => api.dismissProposal(p.id).catch(() => {})));
    refresh();
  };

  // Compact "✓2 ✗1" track record for a proposal's prior history, if any.
  const historyLine = (h) => {
    if (!h || !h.total) return null;
    const dir = h.direction === 'raise' ? 'Raising' : h.direction === 'lower' ? 'Lowering' : 'Changing';
    return (
      <div className="muted" style={{ fontSize: '0.74rem', marginTop: 6 }}>
        📊 {dir} this on {h.airframe_class} before:{' '}
        <span style={{ color: 'var(--ok)' }}>✓{h.better} better</span>{' · '}
        <span style={{ color: 'var(--crit)' }}>✗{h.worse} worse</span>
        {h.no_change ? <> · ∅{h.no_change} no change</> : null}
      </div>
    );
  };

  const FEEDBACK = [
    ['better', 'better'],
    ['worse', 'worse'],
    ['no_change', 'no change'],
  ];

  const stateBadge = (p) => ({
    presented: <span className="badge"><span className="dot warn" />awaiting pilot</span>,
    written: <span className="badge"><span className="dot ok" />written ✓</span>,
    rejected: <span className="badge"><span className="dot crit" />safety rejected</span>,
    write_failed: <span className="badge"><span className="dot crit" />write failed</span>,
    approved: <span className="badge"><span className="dot warn" />writing…</span>,
    reverted: <span className="badge"><span className="dot off" />reverted ↩</span>,
  })[p.state] ?? null;

  return (
    <div className="glass panel">
      <div className="row spread" style={{ marginBottom: '14px', alignItems: 'center' }}>
        <h2 style={{ margin: 0 }}>Parameter Proposals</h2>
        {proposals.length > 0 && (
          <button className="btn" style={{ padding: '4px 8px', fontSize: '0.72rem', borderRadius: '6px' }} onClick={clearAll}>
            Clear all
          </button>
        )}
      </div>
      <div className="stack scrollable-stack">
        {proposals.length === 0 && (
          <div className="muted">
            No proposals yet. Advice appears here after live tracking analysis
            or a ULog deep dive — nothing is ever written without your approval.
          </div>
        )}
        {error && <div className="alert critical">{error}</div>}
        {proposals.map((p) => {
          if (p.state === 'diagnostic') {
            return (
              <div className="proposal card diagnostic" key={p.id} style={{ opacity: 0.65, borderLeft: '3px solid var(--accent)' }}>
                <div className="row spread" style={{ marginBottom: '6px' }}>
                  <span className="mono" style={{ fontWeight: 600, display: 'flex', alignItems: 'center', gap: '6px' }}>
                    <svg viewBox="0 0 24 24" width="14" height="14" stroke="currentColor" strokeWidth="2.5" fill="none" strokeLinecap="round" strokeLinejoin="round" style={{ color: 'var(--accent)' }}>
                      <circle cx="12" cy="12" r="10" />
                      <line x1="12" y1="16" x2="12" y2="12" />
                      <line x1="12" y1="8" x2="12.01" y2="8" />
                    </svg>
                    {p.param}
                  </span>
                  <span className="badge">analysis inactive</span>
                </div>
                <div className="muted" style={{ margin: '6px 0', fontSize: '0.82rem' }}>{p.rationale}</div>
              </div>
            );
          }

          return (
            <div className="proposal" key={p.id}>
              <div className="row spread">
                <span className="mono" style={{ fontWeight: 600 }}>{p.param}</span>
                <div style={{ display: 'flex', gap: '6px', alignItems: 'center' }}>
                  {p.confidence && <span className="badge">{p.confidence}</span>}
                  {stateBadge(p)}
                </div>
              </div>
              <div className="delta" style={{ margin: '8px 0' }}>
                {p.current_value}
                <span className="arrow">→</span>
                <strong style={{ color: 'var(--accent)' }}>{p.proposed_value}</strong>
                {p.requested_value !== p.proposed_value && (
                  <span className="muted"> (requested {p.requested_value}, clamped)</span>
                )}
              </div>
              {p.limitations && (
                <div style={{ fontSize: '0.76rem', color: '#fbbf24', marginBottom: '6px', fontWeight: 500 }}>
                  ⚠️ {p.limitations}
                </div>
              )}
              <div className="muted" style={{ marginBottom: 6 }}>{p.rationale}</div>
              <div className="muted" style={{ fontSize: '0.74rem' }}>🛡 {p.safety_note}</div>
              {p.state === 'presented' && (
                <>
                  {historyLine(p.tuning_history)}
                  {p.is_saturation_gain_reduction && host?.expert_mode && (
                    <div style={{
                      background: 'rgba(251, 191, 36, 0.15)',
                      border: '1px solid rgba(251, 191, 36, 0.4)',
                      borderRadius: '8px',
                      padding: '10px',
                      marginTop: '10px',
                      marginBottom: '10px',
                      color: '#fbbf24',
                      fontSize: '0.82rem'
                    }}>
                      <strong>EXPERT MODE ACTIVE</strong> — gain reduction during saturation may reduce control authority. Confirm you understand the risk.
                      <label style={{ display: 'flex', alignItems: 'center', gap: '8px', marginTop: '8px', cursor: 'pointer', fontWeight: 500 }}>
                        <input type="checkbox" checked={!!confirmedRisks[p.id]} onChange={(e) => {
                          setConfirmedRisks(prev => ({ ...prev, [p.id]: e.target.checked }));
                        }} />
                        I understand the risk
                      </label>
                    </div>
                  )}
                  {hasPendingFeedbackOnAxis(p.param) && (
                    <div style={{
                      background: 'rgba(248, 113, 113, 0.15)',
                      border: '1px solid rgba(248, 113, 113, 0.4)',
                      borderRadius: '8px',
                      padding: '8px 10px',
                      marginTop: '10px',
                      marginBottom: '10px',
                      color: '#f87171',
                      fontSize: '0.8rem',
                      fontWeight: 500
                    }}>
                      ⚠️ Pending feedback required on this axis before making further tuning changes.
                    </div>
                  )}
                  <div className="row" style={{ marginTop: 10 }}>
                    <button className="btn approve" style={{ flex: 1 }}
                            disabled={busyId === p.id || (p.is_saturation_gain_reduction && !confirmedRisks[p.id])}
                            onClick={() => approve(p.id)}>
                      Approve &amp; Write
                    </button>
                    <button className="btn" onClick={() => dismiss(p.id)}>Dismiss</button>
                  </div>
                </>
              )}
              {p.state === 'written' && (
                <div style={{ marginTop: 10 }}>
                  {p.feedback ? (
                    <div className="muted" style={{ fontSize: '0.74rem' }}>
                      Your verdict: <strong>{p.feedback.replace('_', ' ')}</strong> — thanks, logged for next time.
                    </div>
                  ) : (
                    <>
                      <div className="muted" style={{ fontSize: '0.74rem', marginBottom: 6, fontWeight: p.is_saturation_gain_reduction ? 'bold' : 'normal', color: p.is_saturation_gain_reduction ? 'var(--accent)' : 'inherit' }}>
                        {p.is_saturation_gain_reduction ? "Did this help?" : "How did it fly?"}
                      </div>
                      <div className="row">
                        {FEEDBACK.map(([value, label]) => (
                          <button key={value} className="btn" style={{ flex: 1 }}
                                  onClick={() => giveFeedback(p.id, value)}>
                            {label}
                          </button>
                        ))}
                        <button className="btn" style={{ flex: 1 }}
                                onClick={() => giveFeedback(p.id, 'skip')}>
                          Skip
                        </button>
                      </div>
                    </>
                  )}
                  <button className="btn" style={{ marginTop: 8, width: '100%' }}
                          disabled={busyId === p.id}
                          title={`Restore ${p.param} to its pre-write value ${p.current_value}`}
                          onClick={() => revert(p.id)}>
                    ↩ Undo (revert to {p.current_value})
                  </button>
                </div>
              )}
            </div>
          );
        })}
      </div>
    </div>
  );
}
