import { useEffect, useRef } from 'react';
import uPlot from 'uplot';
import { useTelemetryChannel } from '../hooks/useTelemetry.js';

const WINDOW_S = 30;
const DEG = 180 / Math.PI;

const quatRollPitch = (q) => {
  const [w, x, y, z] = q;
  return {
    roll: Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
    pitch: Math.asin(Math.max(-1, Math.min(1, 2 * (w * y - z * x)))),
  };
};

/**
 * Per-loop signal extraction: which WS channels carry the setpoint and
 * the actual, and how to pull the selected axis out of each payload.
 */
export const LOOP_CONFIG = {
  rate: {
    axes: ['roll', 'pitch', 'yaw'], unit: '°/s',
    actCh: 'attitude', spCh: 'attitude_target',
    act: (d, ax) => (d[{ roll: 'rollspeed', pitch: 'pitchspeed', yaw: 'yawspeed' }[ax]] ?? null) * DEG,
    sp: (d, ax) => {
      const v = d[{ roll: 'body_roll_rate', pitch: 'body_pitch_rate', yaw: 'body_yaw_rate' }[ax]];
      return v == null ? null : v * DEG;
    },
  },
  attitude: {
    axes: ['roll', 'pitch'], unit: '°',
    actCh: 'attitude', spCh: 'attitude_target',
    act: (d, ax) => (d[ax] ?? null) * DEG,
    sp: (d, ax) => {
      if (!d.q || d.q.length !== 4) return null;
      return quatRollPitch(d.q)[ax] * DEG;
    },
  },
  velocity: {
    axes: ['vx', 'vy', 'vz'], unit: 'm/s',
    actCh: 'local_position', spCh: 'position_target',
    act: (d, ax) => d[ax] ?? null,
    sp: (d, ax) => d[ax] ?? null,
  },
  position: {
    axes: ['x', 'y', 'z'], unit: 'm',
    actCh: 'local_position', spCh: 'position_target',
    act: (d, ax) => d[ax] ?? null,
    sp: (d, ax) => d[ax] ?? null,
  },
};

/** Rolling 30 s setpoint-vs-actual chart for any cascade loop. */
export default function LiveLoopChart({ loop = 'rate', axis = 'roll' }) {
  const cfg = LOOP_CONFIG[loop];
  const el = useRef(null);
  const plot = useRef(null);
  const buf = useRef({ t: [], actual: [], target: [] });
  const raf = useRef(0);
  const targetRef = useRef(null);

  useEffect(() => {
    buf.current = { t: [], actual: [], target: [] };
    targetRef.current = null;
    const opts = {
      width: el.current.clientWidth,
      height: 220,
      series: [
        {},
        { label: `${axis}`, stroke: '#4f8cff', width: 2 },
        { label: `${axis} cmd`, stroke: '#fbbf24', width: 1.5, dash: [6, 4] },
      ],
      axes: [
        { stroke: '#5e6a85', grid: { stroke: 'rgba(255,255,255,0.06)' } },
        { stroke: '#5e6a85', grid: { stroke: 'rgba(255,255,255,0.06)' },
          values: (u, vals) => vals.map((v) => `${v}${cfg.unit}`) },
      ],
      legend: { live: true },
      cursor: { show: false },
    };
    plot.current = new uPlot(opts, [[], [], []], el.current);
    const ro = new ResizeObserver(() =>
      plot.current?.setSize({ width: el.current.clientWidth, height: 220 }));
    ro.observe(el.current);
    return () => { ro.disconnect(); plot.current?.destroy(); };
  }, [loop, axis]);

  useTelemetryChannel(cfg.spCh, (d) => {
    const v = cfg.sp(d, axis);
    if (v !== null && v !== undefined && !Number.isNaN(v)) targetRef.current = v;
  });

  useTelemetryChannel(cfg.actCh, (d, ts) => {
    const v = cfg.act(d, axis);
    if (v === null || v === undefined || Number.isNaN(v)) return;
    const b = buf.current;
    b.t.push(ts); b.actual.push(v); b.target.push(targetRef.current);
    while (b.t.length && b.t[0] < ts - WINDOW_S) {
      b.t.shift(); b.actual.shift(); b.target.shift();
    }
    cancelAnimationFrame(raf.current);
    raf.current = requestAnimationFrame(() =>
      plot.current?.setData([b.t, b.actual, b.target]));
  });

  return <div ref={el} />;
}
