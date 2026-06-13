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

  const refresh = () => api.proposals().then(setProposals).catch(() => {});
  useEffect(() => { refresh(); }, []);
  useTelemetryChannel('proposal', refresh);

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
      <h2>Parameter Proposals</h2>
      <div className="stack">
        {proposals.length === 0 && (
          <div className="muted">
            No proposals yet. Advice appears here after live tracking analysis
            or a ULog deep dive — nothing is ever written without your approval.
          </div>
        )}
        {error && <div className="alert critical">{error}</div>}
        {proposals.map((p) => (
          <div className="proposal" key={p.id}>
            <div className="row spread">
              <span className="mono" style={{ fontWeight: 600 }}>{p.param}</span>
              {stateBadge(p)}
            </div>
            <div className="delta" style={{ margin: '8px 0' }}>
              {p.current_value}
              <span className="arrow">→</span>
              <strong style={{ color: 'var(--accent)' }}>{p.proposed_value}</strong>
              {p.requested_value !== p.proposed_value && (
                <span className="muted"> (requested {p.requested_value}, clamped)</span>
              )}
            </div>
            <div className="muted" style={{ marginBottom: 6 }}>{p.rationale}</div>
            <div className="muted" style={{ fontSize: '0.74rem' }}>🛡 {p.safety_note}</div>
            {p.state === 'presented' && (
              <>
                {historyLine(p.tuning_history)}
                <div className="row" style={{ marginTop: 10 }}>
                  <button className="btn approve" style={{ flex: 1 }}
                          disabled={busyId === p.id}
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
                    <div className="muted" style={{ fontSize: '0.74rem', marginBottom: 6 }}>
                      How did it fly?
                    </div>
                    <div className="row">
                      {FEEDBACK.map(([value, label]) => (
                        <button key={value} className="btn" style={{ flex: 1 }}
                                onClick={() => giveFeedback(p.id, value)}>
                          {label}
                        </button>
                      ))}
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
        ))}
      </div>
    </div>
  );
}
