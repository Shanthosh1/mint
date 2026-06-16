import { useEffect, useRef, useState, useCallback } from 'react';
import uPlot from 'uplot';
import { useTelemetryChannel, useFrontendConfig } from '../hooks/useTelemetry.js';

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

/**
 * uPlot plugin: scroll-wheel zoom on X axis + drag-to-select zoom.
 * Sets `zoomedRef.current = true` when zoomed so auto-scroll pauses.
 */
function zoomPlugin(zoomedRef, bufRef, syncZoomState, windowS, zoomFactor = 0.75, minZoomRangeS = 0.5) {
  return {
    hooks: {
      ready: [
        (u) => {
          const over = u.over;

          // --- Scroll-wheel zoom on X axis ---
          over.addEventListener('wheel', (e) => {
            e.preventDefault();
            const { left, width } = over.getBoundingClientRect();
            const cursorPct = (e.clientX - left) / width;
            const xMin = u.scales.x.min;
            const xMax = u.scales.x.max;
            const xRange = xMax - xMin;
            const dir = e.deltaY < 0 ? zoomFactor : 1 / zoomFactor;
            const newRange = Math.max(xRange * dir, minZoomRangeS);
            const center = xMin + cursorPct * xRange;
            const nMin = center - cursorPct * newRange;
            const nMax = center + (1 - cursorPct) * newRange;
            u.setScale('x', { min: nMin, max: nMax });
            zoomedRef.current = true;
            syncZoomState();
          }, { passive: false });

          // --- Double-click to reset zoom ---
          over.addEventListener('dblclick', () => {
            zoomedRef.current = false;
            if (bufRef.current) {
              const b = bufRef.current;
              const ts = b.t.length ? b.t[b.t.length - 1] : Date.now() / 1000;
              u.setData([b.t, b.actual, b.target], false);
              u.setScale('x', { min: ts - windowS, max: ts });
            }
            syncZoomState();
          });
        },
      ],
      setSelect: [
        (u) => {
          const sel = u.select;
          if (sel.width > 10) {
            const xMin = u.posToVal(sel.left, 'x');
            const xMax = u.posToVal(sel.left + sel.width, 'x');
            u.setScale('x', { min: xMin, max: xMax });
            zoomedRef.current = true;
            syncZoomState();
          }
          // Clear the selection rectangle
          u.setSelect({ left: 0, width: 0, top: 0, height: 0 }, false);
        },
      ],
    },
  };
}

/** Rolling setpoint-vs-actual chart for any cascade loop. */
export default function LiveLoopChart({ loop = 'rate', axis = 'roll' }) {
  const fConfig = useFrontendConfig();
  const windowS = fConfig?.live_loop_chart?.window_s ?? 30;
  const zoomFactor = fConfig?.live_loop_chart?.zoom_factor ?? 0.75;
  const minZoomRangeS = fConfig?.live_loop_chart?.min_zoom_range_s ?? 0.5;

  const cfg = LOOP_CONFIG[loop];
  const el = useRef(null);
  const plot = useRef(null);
  const buf = useRef({ t: [], actual: [], target: [] });
  const raf = useRef(0);
  const targetRef = useRef(null);
  const zoomedRef = useRef(false);
  const [zoomed, setZoomed] = useState(false);

  // Sync ref → state for the reset button (debounced via rAF)
  const syncZoomState = useCallback(() => setZoomed(zoomedRef.current), []);

  const resetZoom = useCallback(() => {
    zoomedRef.current = false;
    setZoomed(false);
    if (plot.current) {
      const b = buf.current;
      const ts = b.t.length ? b.t[b.t.length - 1] : Date.now() / 1000;
      plot.current.setData([b.t, b.actual, b.target], false);
      plot.current.setScale('x', { min: ts - windowS, max: ts });
    }
  }, [windowS]);

  useEffect(() => {
    buf.current = { t: [], actual: [], target: [] };
    targetRef.current = null;
    zoomedRef.current = false;
    setZoomed(false);

    const opts = {
      width: el.current.clientWidth,
      height: 220,
      plugins: [zoomPlugin(zoomedRef, buf, syncZoomState, windowS, zoomFactor, minZoomRangeS)],
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
      cursor: {
        show: true,
        drag: { x: true, y: false, setScale: false },  // drag draws select rect
      },
      select: {
        show: true,
        over: true,
      },
    };
    plot.current = new uPlot(opts, [[], [], []], el.current);
    let lastWidth = el.current?.clientWidth || 0;
    const ro = new ResizeObserver(() => {
      const w = el.current?.clientWidth || 0;
      if (w !== lastWidth && w > 0) {
        lastWidth = w;
        plot.current?.setSize({ width: w, height: 220 });
      }
    });
    ro.observe(el.current);
    return () => { ro.disconnect(); plot.current?.destroy(); };
  }, [loop, axis, zoomFactor, minZoomRangeS]);

  useTelemetryChannel(cfg.spCh, (d) => {
    if (!d) {
      targetRef.current = null;
      return;
    }
    const v = cfg.sp(d, axis);
    if (v !== null && v !== undefined && !Number.isNaN(v)) targetRef.current = v;
  });

  useTelemetryChannel(cfg.actCh, (d, ts) => {
    if (!d) {
      buf.current = { t: [], actual: [], target: [] };
      targetRef.current = null;
      zoomedRef.current = false;
      setZoomed(false);
      cancelAnimationFrame(raf.current);
      raf.current = requestAnimationFrame(() =>
        plot.current?.setData([[], [], []]));
      return;
    }
    const v = cfg.act(d, axis);
    if (v === null || v === undefined || Number.isNaN(v)) return;
    const b = buf.current;
    b.t.push(ts); b.actual.push(v); b.target.push(targetRef.current);
    const maxBufferS = 300; // Keep up to 5 minutes of historical data so zoom remains visible
    while (b.t.length && b.t[0] < ts - maxBufferS) {
      b.t.shift(); b.actual.shift(); b.target.shift();
    }
    cancelAnimationFrame(raf.current);
    raf.current = requestAnimationFrame(() => {
      const u = plot.current;
      if (!u) return;
      if (zoomedRef.current) {
        // When zoomed, keep the user's X scale — only update data and redraw
        u.setData([b.t, b.actual, b.target], false);
        u.redraw();
      } else {
        // Auto-scroll: show the latest windowS seconds
        u.setData([b.t, b.actual, b.target], false);
        u.setScale('x', { min: ts - windowS, max: ts });
      }
      syncZoomState();
    });
  });

  return (
    <div style={{ position: 'relative' }}>
      <div ref={el} />
      {zoomed && (
        <button
          className="btn zoom-reset-btn"
          onClick={resetZoom}
          title="Double-click chart or press this to reset"
        >
          ↩ Reset Zoom
        </button>
      )}
    </div>
  );
}
