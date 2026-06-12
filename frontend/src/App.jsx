import { useTelemetrySocket, useChannelState } from './hooks/useTelemetry.js';
import ConnectionPanel from './components/ConnectionPanel.jsx';
import PidPanel from './components/PidPanel.jsx';
import EkfPanel from './components/EkfPanel.jsx';
import AlertStack from './components/AlertStack.jsx';
import ProposalsPanel from './components/ProposalsPanel.jsx';
import UlogPanel from './components/UlogPanel.jsx';

const REGIME_LABELS = {
  pre_flight: { label: 'Pre-Flight', dot: 'off' },
  steady_hold: { label: 'Steady Hold', dot: 'ok' },
  dynamic_maneuver: { label: 'Dynamic Maneuver', dot: 'warn' },
};

export default function App() {
  const wsUp = useTelemetrySocket();
  const vehicle = useChannelState('connection', { connected: false });
  const regime = useChannelState('regime');
  const mode = useChannelState('flight_mode');
  const r = REGIME_LABELS[regime?.state] ?? REGIME_LABELS.pre_flight;

  return (
    <div className="app-shell">
      <header className="glass topbar">
        <div className="logo">MIN<span>T</span></div>
        <span className="muted">MAVLink Intelligent Tuning Assistant</span>
        <div style={{ flex: 1 }} />
        {mode && (
          <span className="badge" title={`custom_mode ${mode.custom_mode}`}>
            <span className={`dot ${mode.armed ? 'warn' : 'off'}`} />
            {mode.mode}{mode.armed ? ' · ARMED' : ''}
          </span>
        )}
        <span className="badge" title={`stick σ²: ${regime?.stick_variance ?? '—'}`}>
          <span className={`dot ${r.dot}`} />
          {r.label}
        </span>
        <span className="badge">
          <span className={`dot ${wsUp ? 'ok' : 'crit'}`} />
          backend {wsUp ? 'live' : 'offline'}
        </span>
        <span className="badge">
          <span className={`dot ${vehicle?.connected ? 'ok' : 'off'}`} />
          vehicle {vehicle?.connected ? 'connected' : '—'}
        </span>
      </header>

      <div className="grid">
        <div className="col">
          <ConnectionPanel />
          <EkfPanel />
        </div>
        <div className="col col-main">
          <PidPanel />
          <UlogPanel />
        </div>
        <div className="col">
          <AlertStack />
          <ProposalsPanel />
        </div>
      </div>
    </div>
  );
}
