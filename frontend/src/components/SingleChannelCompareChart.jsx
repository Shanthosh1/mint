import { useEffect, useRef, useState } from 'react';
import uPlot from 'uplot';

export default function SingleChannelCompareChart({ t = [], command = [], actual = [], label = '', axis = '' }) {
  const containerRef = useRef(null);
  const plotRef = useRef(null);
  const [hoverIdx, setHoverIdx] = useState(null);

  const hasCommand = command && command.length > 0;

  useEffect(() => {
    if (t.length === 0 || !containerRef.current) {
      if (plotRef.current) {
        plotRef.current.destroy();
        plotRef.current = null;
      }
      return;
    }

    // Series 0: X axis (Time)
    // Series 1 (optional): Command
    // Series 1/2: Actual (Actuator output)
    const uPlotData = hasCommand ? [t, command, actual] : [t, actual];

    const isBipolar = actual.some(v => v < -0.05);
    const yMin = isBipolar ? -1.05 : -0.05;
    const yMax = 1.05;

    const seriesConfig = [
      {
        // X axis (Time)
      }
    ];

    if (hasCommand) {
      seriesConfig.push({
        label: `Command (${axis})`,
        stroke: '#fbbf24',
        width: 1.5,
        dash: [6, 4],
        points: { show: false }
      });
    }

    seriesConfig.push({
      label: `Actual (${label})`,
      stroke: '#4f8cff',
      width: 2.0,
      points: { show: false }
    });

    const opts = {
      width: containerRef.current.clientWidth,
      height: 180,
      series: seriesConfig,
      scales: {
        x: { time: false },
        y: { range: [yMin, yMax] }
      },
      axes: [
        {
          scale: 'x',
          stroke: '#5e6a85',
          grid: { stroke: 'rgba(255,255,255,0.06)' },
          values: (u, vals) => vals.map(v => `${Number(v).toFixed(1)}s`)
        },
        {
          scale: 'y',
          stroke: '#5e6a85',
          grid: { stroke: 'rgba(255,255,255,0.06)' },
          values: (u, vals) => vals.map(v => `${Math.round(v * 100)}%`)
        }
      ],
      cursor: {
        show: true,
        drag: { x: true, y: false }
      },
      legend: {
        show: false
      },
      hooks: {
        setCursor: [
          (self) => {
            const idx = self.cursor.idx;
            if (idx !== null && idx !== undefined && idx >= 0 && idx < t.length) {
              setHoverIdx(idx);
            } else {
              setHoverIdx(null);
            }
          }
        ]
      }
    };

    const plot = new uPlot(opts, uPlotData, containerRef.current);
    plotRef.current = plot;

    let lastWidth = containerRef.current?.clientWidth || 0;
    const ro = new ResizeObserver(() => {
      if (containerRef.current) {
        const w = containerRef.current.clientWidth;
        if (w !== lastWidth && w > 0) {
          lastWidth = w;
          plot.setSize({
            width: w,
            height: 180
          });
        }
      }
    });
    ro.observe(containerRef.current);

    return () => {
      ro.disconnect();
      plot.destroy();
      plotRef.current = null;
    };
  }, [t.length, label, axis, hasCommand]);

  const renderReadout = () => {
    if (hoverIdx === null || hoverIdx === undefined || hoverIdx >= t.length) {
      return (
        <span className="muted" style={{ fontSize: '0.7rem' }}>
          💡 Hover to inspect tracking values
        </span>
      );
    }
    const timestamp = t[hoverIdx];
    const actVal = actual[hoverIdx];
    const cmdVal = hasCommand ? command[hoverIdx] : null;
    return (
      <span className="mono" style={{ fontSize: '0.74rem', color: 'var(--text-mid)' }}>
        Time: <strong style={{ color: 'var(--text)' }}>{timestamp.toFixed(2)}s</strong> | Output: <strong style={{ color: '#4f8cff' }}>{actVal != null ? `${Math.round(actVal * 100)}%` : '—'}</strong>
        {cmdVal != null && (
          <> | Cmd: <strong style={{ color: '#fbbf24' }}>{Math.round(cmdVal * 100)}%</strong></>
        )}
      </span>
    );
  };

  return (
    <div className="stack" style={{ gap: 8 }}>
      <div className="row spread" style={{ fontSize: '0.74rem', fontWeight: 600, color: 'var(--text-mid)', alignItems: 'center' }}>
        <span style={{ color: 'var(--accent)' }}>Diagnostics Timeline: {label}</span>
        <div className="row" style={{ gap: 12, alignItems: 'center' }}>
          <span className="muted" style={{ fontSize: '0.68rem', fontWeight: 400 }}>
            💡 Drag to zoom | Double-click to reset
          </span>
          {hasCommand && (
            <span style={{ display: 'inline-flex', alignItems: 'center', gap: 4 }}>
              <span style={{ display: 'inline-block', width: 10, height: 2, background: '#fbbf24', borderTop: '2px dashed #fbbf24' }} /> Cmd ({axis})
            </span>
          )}
          <span style={{ display: 'inline-flex', alignItems: 'center', gap: 4 }}>
            <span style={{ display: 'inline-block', width: 10, height: 2, background: '#4f8cff' }} /> Actual Output
          </span>
        </div>
      </div>
      
      <div ref={containerRef} style={{ width: '100%', minHeight: 180 }} />
      
      <div style={{ borderTop: '1px solid var(--glass-border)', paddingTop: 6, minHeight: 20, display: 'flex', alignItems: 'center' }}>
        {renderReadout()}
      </div>
    </div>
  );
}
