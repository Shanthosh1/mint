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
  // the backend automatically skips QGC forwarding and shifts MAVSDK to
  // an alternate port.
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

        {error && <div className="alert critical">{error}</div>}
      </div>
    </div>
  );
}
