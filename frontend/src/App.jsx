import { useState, useEffect } from 'react';
import { useTelemetrySocket, useChannelState } from './hooks/useTelemetry.js';
import ConnectionPanel from './components/ConnectionPanel.jsx';
import PidPanel from './components/PidPanel.jsx';
import EkfPanel from './components/EkfPanel.jsx';
import VibrationPanel from './components/VibrationPanel.jsx';
import ActuatorPanel from './components/ActuatorPanel.jsx';
import AlertStack from './components/AlertStack.jsx';
import ProposalsPanel from './components/ProposalsPanel.jsx';
import UlogPanel from './components/UlogPanel.jsx';

const REGIME_LABELS = {
  pre_flight: { label: 'Pre-Flight', dot: 'off' },
  steady_hold: { label: 'Steady Hold', dot: 'ok' },
  dynamic_maneuver: { label: 'Dynamic Maneuver', dot: 'warn' },
};

function getPage() {
  const h = window.location.hash.replace('#', '') || 'live';
  return h === 'postflight' ? 'postflight' : 'live';
}

export default function App() {
  const wsUp = useTelemetrySocket();
  const vehicle = useChannelState('connection', { connected: false });
  const regime = useChannelState('regime');
  const mode = useChannelState('flight_mode');
  const telemetryStale = useChannelState('telemetry_stale');
  const isStale = telemetryStale?.stale === true;
  const r = REGIME_LABELS[regime?.state] ?? REGIME_LABELS.pre_flight;

  const [page, setPage] = useState(getPage);

  useEffect(() => {
    const onHash = () => setPage(getPage());
    window.addEventListener('hashchange', onHash);
    return () => window.removeEventListener('hashchange', onHash);
  }, []);

  const nav = (target) => (e) => {
    e.preventDefault();
    window.location.hash = target;
  };

  return (
    <div className="app-shell">
      {isStale && (
        <div className="telemetry-stale-banner animate-fade">
          ⚠️ TELEMETRY IS STALE — METRICS ARE FROZEN! CHECK TELEMETRY LINK/ROUTER
        </div>
      )}
      <header className={`glass topbar ${isStale ? 'stale' : ''}`}>
        <div className="logo">MIN<span>T</span></div>

        <nav className="topbar-nav">
          <a
            href="#live"
            className={`nav-link ${page === 'live' ? 'active' : ''}`}
            onClick={nav('live')}
          >
            ⚡ Live Dashboard
          </a>
          <a
            href="#postflight"
            className={`nav-link ${page === 'postflight' ? 'active' : ''}`}
            onClick={nav('postflight')}
          >
            📊 Post-Flight Analysis
          </a>
        </nav>

        <div style={{ flex: 1 }} className="topbar-spacer" />

        <div className="topbar-status">
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
        </div>
      </header>

      {page === 'live' && (
        <div className="grid animate-fade">
          <div className="col">
            <ActuatorPanel />
          </div>
          <div className="col col-main">
            <PidPanel />
            <div className="row-panels">
              <ConnectionPanel />
              <EkfPanel />
              <VibrationPanel />
            </div>
          </div>
          <div className="col">
            <AlertStack />
            <ProposalsPanel />
          </div>
        </div>
      )}

      {page === 'postflight' && (
        <div className="postflight-page animate-fade">
          <UlogPanel />
        </div>
      )}
    </div>
  );
}
