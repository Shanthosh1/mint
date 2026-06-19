import { useEffect, useState } from 'react';
import { api } from '../api.js';
import { useChannelState } from '../hooks/useTelemetry.js';

const BAUDS = [57600, 115200, 230400, 921600];

const MODES = [
  { id: 'serial', label: 'Serial / USB' },
  { id: 'udp_listen', label: 'UDP Listen' },
  { id: 'udp_connect', label: 'UDP Connect' },
  { id: 'tcp_connect', label: 'TCP' },
];

const MODE_HINTS = {
  serial: 'USB flight controller or telemetry radio.',
  udp_listen: 'Vehicle, companion computer, or SITL sends MAVLink to this machine. ' +
              'Bind 0.0.0.0 and the port the vehicle targets.',
  udp_connect: 'Connect out to a UDP peer (e.g. an ethernet FC or companion serving MAVLink).',
  tcp_connect: 'TCP client — serial-over-ethernet bridges, ArduPilot SITL (port 5760).',
};

function ActuatorManualMapper({ onSave }) {
  const [map, setMap] = useState({
    1: 'none', 2: 'none', 3: 'none', 4: 'none',
    5: 'none', 6: 'none', 7: 'none', 8: 'none',
    9: 'none', 10: 'none', 11: 'none', 12: 'none',
    13: 'none', 14: 'none', 15: 'none', 16: 'none',
  });
  const [busy, setBusy] = useState(false);
  const [err, setErr] = useState(null);

  const apply = async () => {
    setBusy(true); setErr(null);
    try {
      const hover_motors = [];
      const thrust_motors = [];
      const control_surfaces = [];
      const tilt_servos = [];

      for (let ch = 1; ch <= 16; ch++) {
        const role = map[ch];
        const idx = ch - 1;
        if (role === 'hover') hover_motors.push(idx);
        else if (role === 'thrust') thrust_motors.push(idx);
        else if (role === 'surface') control_surfaces.push(idx);
        else if (role === 'tilt') tilt_servos.push(idx);
      }

      await api.updateActuatorMap({
        hover_motors,
        thrust_motors,
        control_surfaces,
        tilt_servos
      });
      if (onSave) onSave();
    } catch (e) {
      setErr(e.message);
    } finally {
      setBusy(false);
    }
  };

  return (
    <div className="alert warning stack" style={{ marginTop: 12, padding: 12, gap: 10, background: 'rgba(255, 165, 0, 0.08)', border: '1px solid rgba(255, 165, 0, 0.3)' }}>
      <div style={{ fontWeight: 600, fontSize: '0.82rem', display: 'flex', alignItems: 'center', gap: 6 }}>
        <span>⚠</span> Actuator Auto-Discovery Failed
      </div>
      <div className="muted" style={{ fontSize: '0.72rem', lineHeight: '1.25' }}>
        Could not detect actuator assignments automatically. Assign physical outputs manually to continue:
      </div>
      
      <div className="scrollable-stack" style={{ maxHeight: 180, overflowY: 'auto', paddingRight: 4, display: 'flex', flexDirection: 'column', gap: 4 }}>
        {Array.from({ length: 16 }, (_, i) => i + 1).map((ch) => (
          <div key={ch} className="row spread" style={{ fontSize: '0.74rem', padding: '2px 0', alignItems: 'center' }}>
            <span className="mono">Channel {ch}</span>
            <select
              value={map[ch]}
              onChange={(e) => setMap({ ...map, [ch]: e.target.value })}
              style={{ padding: '2px 6px', fontSize: '0.72rem', width: 140, background: 'rgba(0,0,0,0.2)', color: 'var(--text-main)', border: '1px solid var(--glass-border)', borderRadius: 4 }}
            >
              <option value="none">None / Unmapped</option>
              <option value="hover">Hover Motor (M)</option>
              <option value="thrust">Thrust Motor (P)</option>
              <option value="surface">Control Surface (S)</option>
              <option value="tilt">Tilt Servo (Tilt)</option>
            </select>
          </div>
        ))}
      </div>

      {err && <div className="alert critical" style={{ fontSize: '0.72rem', padding: '4px 8px', marginTop: 4 }}>{err}</div>}
      
      <button className="btn primary" disabled={busy} onClick={apply} style={{ fontSize: '0.72rem', padding: '6px 10px', marginTop: 4 }}>
        {busy ? 'Applying...' : 'Apply Actuator Mapping'}
      </button>
    </div>
  );
}


/**
 * MAVLink source picker (serial / UDP / TCP) + mavp2p router toggle +
 * vehicle connect. Surfaces OS info, permission problems, and the
 * detected airframe.
 */
export default function ConnectionPanel() {
  const [host, setHost] = useState(null);
  const [ports, setPorts] = useState([]);
  const [mode, setMode] = useState('serial');
  const [port, setPort] = useState('');                 // serial device
  const [baud, setBaud] = useState(57600);
  const [netHost, setNetHost] = useState('0.0.0.0');    // network modes
  const [netPort, setNetPort] = useState(14550);
  const [router, setRouter] = useState({ running: false });
  const [busy, setBusy] = useState(false);
  const [error, setError] = useState(null);
  const airframe = useChannelState('airframe');
  const vehicle = useChannelState('connection', { connected: false });

  const refresh = async () => {
    try {
      const [h, p, r] = await Promise.all([api.host(), api.serialPorts(), api.routerStatus()]);
      setHost(h); setPorts(p); setRouter(r);
      if (!port && p.length) setPort(p[0].device);
      setError(null);
    } catch (e) { setError(e.message); }
  };

  useEffect(() => { refresh(); }, []);

  const act = (fn) => async () => {
    setBusy(true); setError(null);
    try { await fn(); await refresh(); }
    catch (e) { setError(e.message); }
    finally { setBusy(false); }
  };

  const startRouter = () =>
    api.routerStart({
      mode,
      serial_device: mode === 'serial' ? port : null,
      baud,
      host: netHost,
      port: Number(netPort),
    });

  // PX4 SITL preset: SITL's offboard/API stream targets localhost:14540.
  // We listen there; QGC keeps its own direct 14550 stream from SITL, so
  // the backend automatically skips QGC forwarding.
  const applySitlPreset = () => {
    setMode('udp_listen');
    setNetHost('0.0.0.0');
    setNetPort(14540);
  };

  const selected = ports.find((p) => p.device === port);
  const sourceReady = mode === 'serial'
    ? Boolean(port)
    : Boolean(netHost) && netPort > 0;

  return (
    <div className="glass panel">
      <div className="row spread">
        <h2>Connection</h2>
        <button className="btn" style={{ fontSize: '0.72rem' }}
                onClick={applySitlPreset} title="UDP Listen 0.0.0.0:14540">
          PX4 SITL preset
        </button>
      </div>
      <div className="stack">
        {host && (
          <div className="row spread muted">
            <span>{host.os_label} · {host.machine}</span>
            <span className="badge">
              <span className={`dot ${host.router_binary ? 'ok' : 'crit'}`} />
              router binary
            </span>
          </div>
        )}

        <div className="row mode-tabs">
          {MODES.map((m) => (
            <button key={m.id}
              className={`btn ${mode === m.id ? 'mode-active' : ''}`}
              style={{ flex: 1, padding: '6px 4px', fontSize: '0.74rem' }}
              disabled={router.running}
              onClick={() => setMode(m.id)}>
              {m.label}
            </button>
          ))}
        </div>
        <div className="muted" style={{ fontSize: '0.74rem' }}>{MODE_HINTS[mode]}</div>

        {mode === 'serial' ? (
          <>
            <div className="row">
              <select value={port} onChange={(e) => setPort(e.target.value)}>
                {ports.length === 0 && <option value="">No serial ports found</option>}
                {ports.map((p) => (
                  <option key={p.device} value={p.device}>
                    {p.is_px4_likely ? '✈ ' : ''}{p.device} — {p.description}
                  </option>
                ))}
              </select>
              <button className="btn" onClick={refresh} title="Rescan ports">⟳</button>
            </div>

            {selected && !selected.permission_ok && (
              <div className="alert warning">
                No permission to open {selected.device}. Add your user to the
                <code className="mono"> dialout</code> group and re-login.
              </div>
            )}

            <select value={baud} onChange={(e) => setBaud(Number(e.target.value))}>
              {BAUDS.map((b) => <option key={b} value={b}>{b} baud</option>)}
            </select>
          </>
        ) : (
          <div className="row">
            <input type="text" value={netHost} style={{ flex: 2 }}
                   placeholder={mode === 'udp_listen' ? 'bind address' : 'remote host'}
                   onChange={(e) => setNetHost(e.target.value)} />
            <input type="text" value={netPort} style={{ flex: 1 }} className="mono"
                   placeholder="port" inputMode="numeric"
                   onChange={(e) => setNetPort(e.target.value.replace(/\D/g, ''))} />
          </div>
        )}

        {router.running ? (
          <button className="btn danger" disabled={busy} onClick={act(api.routerStop)}>
            Stop Router ({router.source})
          </button>
        ) : (
          <button className="btn primary" disabled={busy || !sourceReady}
                  onClick={act(startRouter)}>
            Start Router
          </button>
        )}

        {router.running && (
          <div className="muted mono">
            PID {router.pid} → {router.endpoints?.join('  ·  ')}
            {!router.qgc_forwarded &&
              '  (QGC not forwarded — source owns 14550, QGC connects directly)'}
          </div>
        )}

        <div className="row">
          <button className="btn primary" style={{ flex: 1 }}
                  disabled={busy || !router.running || vehicle?.connected}
                  onClick={act(api.vehicleConnect)}>
            {vehicle?.connected ? 'Vehicle Connected' : 'Connect Vehicle'}
          </button>
          {vehicle?.connected && (
            <button className="btn" disabled={busy} onClick={act(api.vehicleDisconnect)}>
              Disconnect
            </button>
          )}
        </div>

        {airframe && (
          <div className="badge" style={{ alignSelf: 'flex-start' }}>
            <span className="dot ok" />
            {airframe.label} ({airframe.airframe_class}) · SYS_AUTOSTART {airframe.sys_autostart}
          </div>
        )}

        {airframe?.discovery_failed && <ActuatorManualMapper onSave={refresh} />}

        {error && <div className="alert critical">{error}</div>}
      </div>
    </div>
  );
}
