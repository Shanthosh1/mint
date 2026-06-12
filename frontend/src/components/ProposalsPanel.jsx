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

  const stateBadge = (p) => ({
    presented: <span className="badge"><span className="dot warn" />awaiting pilot</span>,
    written: <span className="badge"><span className="dot ok" />written ✓</span>,
    rejected: <span className="badge"><span className="dot crit" />safety rejected</span>,
    write_failed: <span className="badge"><span className="dot crit" />write failed</span>,
    approved: <span className="badge"><span className="dot warn" />writing…</span>,
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
              <div className="row" style={{ marginTop: 10 }}>
                <button className="btn approve" style={{ flex: 1 }}
                        disabled={busyId === p.id}
                        onClick={() => approve(p.id)}>
                  Approve &amp; Write
                </button>
                <button className="btn" onClick={() => dismiss(p.id)}>Dismiss</button>
              </div>
            )}
          </div>
        ))}
      </div>
    </div>
  );
}
