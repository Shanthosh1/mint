import { useEffect, useRef, useState, useCallback } from 'react';

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
  };
  socket.onclose = () => {
    setConnected?.(false);
    clearTimeout(reconnectTimer);
    reconnectTimer = setTimeout(() => ensureSocket(setConnected), 1500);
  };
}

export function useTelemetrySocket() {
  const [connected, setConnected] = useState(false);
  useEffect(() => {
    ensureSocket(setConnected);
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
