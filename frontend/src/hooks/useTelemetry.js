import { useEffect, useRef, useState, useCallback } from 'react';
import { api } from '../api.js';

/**
 * Single shared WebSocket to /ws/telemetry with auto-reconnect.
 *
 * Components subscribe per channel:
 *   useTelemetryChannel('attitude', (payload, ts) => { ... })
 *
 * The socket is module-scoped so every panel shares one connection.
 */
const listeners = new Map(); // channel -> Set<fn>
let socket = null;
let reconnectTimer = null;

let frontendConfig = null;
const configListeners = new Set();

export function loadFrontendConfig() {
  if (frontendConfig) return Promise.resolve(frontendConfig);
  return api.frontendConfig()
    .then((cfg) => {
      frontendConfig = cfg;
      configListeners.forEach((fn) => fn(cfg));
      return cfg;
    })
    .catch((err) => {
      console.error("Failed to load frontend config, using defaults:", err);
      frontendConfig = {
        reconnect_delay_ms: 1500,
        actuator_panel: { warn_threshold: 0.8, sat_threshold: 1.0 },
        live_loop_chart: { window_s: 30, zoom_factor: 0.75, min_zoom_range_s: 0.5 }
      };
      configListeners.forEach((fn) => fn(frontendConfig));
      return frontendConfig;
    });
}

export function useFrontendConfig() {
  const [config, setConfig] = useState(frontendConfig);
  useEffect(() => {
    if (frontendConfig) {
      setConfig(frontendConfig);
      return;
    }
    const fn = (cfg) => setConfig(cfg);
    configListeners.add(fn);
    loadFrontendConfig();
    return () => configListeners.delete(fn);
  }, []);
  return config;
}

function ensureSocket(setConnected) {
  if (socket && socket.readyState <= WebSocket.OPEN) return;
  const proto = location.protocol === 'https:' ? 'wss' : 'ws';
  socket = new WebSocket(`${proto}://${location.host}/ws/telemetry`);

  socket.onopen = () => setConnected?.(true);
  socket.onmessage = (e) => {
    const { ch, t, ts, d } = JSON.parse(e.data);
    // t = server monotonic (ordering only); ts = wall-clock epoch seconds.
    listeners.get(ch)?.forEach((fn) => fn(d, t, ch, ts));
    listeners.get('*')?.forEach((fn) => fn(d, t, ch, ts));

    if (ch === 'connection' && d?.connected === false) {
      listeners.forEach((set, channel) => {
        if (channel !== 'connection' && channel !== '*') {
          set.forEach((fn) => {
            try {
              fn(null, t, channel, ts);
            } catch (err) {
              console.error(`Error clearing channel ${channel}:`, err);
            }
          });
        }
      });
    }
  };
  socket.onclose = () => {
    setConnected?.(false);
    clearTimeout(reconnectTimer);
    const delay = frontendConfig?.reconnect_delay_ms ?? 1500;
    reconnectTimer = setTimeout(() => ensureSocket(setConnected), delay);
  };
}

export function useTelemetrySocket() {
  const [connected, setConnected] = useState(false);
  useEffect(() => {
    loadFrontendConfig().then(() => {
      ensureSocket(setConnected);
    });
  }, []);
  return connected;
}

export function useTelemetryChannel(channel, handler) {
  const ref = useRef(handler);
  ref.current = handler;
  useEffect(() => {
    const fn = (...args) => ref.current(...args);
    if (!listeners.has(channel)) listeners.set(channel, new Set());
    listeners.get(channel).add(fn);
    return () => listeners.get(channel)?.delete(fn);
  }, [channel]);
}

/** Convenience: keep the latest payload of a channel in state. */
export function useChannelState(channel, initial = null) {
  const [value, setValue] = useState(initial);
  useTelemetryChannel(channel, useCallback((d) => setValue(d), []));
  return value;
}
