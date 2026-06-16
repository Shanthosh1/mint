import { useEffect, useRef, useState } from 'react';
import uPlot from 'uplot';

const MOTOR_COLORS = ['#38bdf8', '#8b5cf6', '#eab308', '#ec4899', '#6366f1', '#10b981', '#f97316', '#2563eb'];

export default function MotorBalanceChart({ balance = {} }) {
  const containerRef = useRef(null);
  const plotRef = useRef(null);
  const [hoverIdx, setHoverIdx] = useState(null);

  const t = balance.t || [];
  const deviations = balance.deviations || {};
  const motorEntries = Object.entries(deviations);

  const [visibleMotors, setVisibleMotors] = useState(() => {
    const initial = {};
    Object.keys(deviations).forEach(m => {
      initial[m] = true;
    });
    return initial;
  });

  const [prevDeviationsKey, setPrevDeviationsKey] = useState('');
  const deviationsKey = Object.keys(deviations).join(',');
  if (deviationsKey !== prevDeviationsKey) {
    const initial = {};
    Object.keys(deviations).forEach(m => {
      initial[m] = true;
    });
    setVisibleMotors(initial);
    setPrevDeviationsKey(deviationsKey);
  }

  useEffect(() => {
    if (t.length === 0 || motorEntries.length === 0 || !containerRef.current) {
      if (plotRef.current) {
        plotRef.current.destroy();
        plotRef.current = null;
      }
      return;
    }

    // Series 0: Time
    // Series 1..N: Motor Deviations
    // Series N+1: Upper Limit (+15%)
    // Series N+2: Lower Limit (-15%)
    const uPlotData = [t];
    const seriesConfig = [
      {
        // X axis (Time)
      }
    ];

    motorEntries.forEach(([motorLabel, values]) => {
      // Replace nulls with NaN for uPlot to break lines/handle missing data gracefully
      const cleanVals = values.map(v => v === null ? NaN : v);
      uPlotData.push(cleanVals);

      const motorNum = parseInt(motorLabel.replace(/\D/g, '')) || 1;
      const strokeColor = MOTOR_COLORS[(motorNum - 1) % MOTOR_COLORS.length];

      seriesConfig.push({
        label: motorLabel,
        stroke: strokeColor,
        width: 2.0,
        points: { show: false },
        show: visibleMotors[motorLabel] !== false
      });
    });

    // Add Upper & Lower Warn Limits (+15% / -15%)
    const limitUpper = new Array(t.length).fill(0.15);
    const limitLower = new Array(t.length).fill(-0.15);
    uPlotData.push(limitUpper);
    uPlotData.push(limitLower);

    seriesConfig.push({
      label: 'Warn Limit (+15%)',
      stroke: 'rgba(239, 68, 68, 0.45)',
      width: 1.2,
      dash: [6, 4],
      points: { show: false }
    });

    seriesConfig.push({
      label: 'Warn Limit (-15%)',
      stroke: 'rgba(239, 68, 68, 0.45)',
      width: 1.2,
      dash: [6, 4],
      points: { show: false }
    });

    const opts = {
      width: containerRef.current.clientWidth,
      height: 220,
      series: seriesConfig,
      scales: {
        x: { time: false },
        y: { range: [-0.3, 0.3] } // Fit -30% to +30% deviation
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
          values: (u, vals) => vals.map(v => `${v > 0 ? '+' : ''}${Math.round(v * 100)}%`)
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
            height: 220
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
  }, [t.length, Object.keys(deviations).length, JSON.stringify(visibleMotors)]);

  const renderInteractiveLegend = () => {
    return (
      <div className="stack" style={{ gap: 8 }}>
        {/* Header with quick toggles */}
        <div className="row spread" style={{ fontSize: '0.72rem', color: 'var(--text-mid)', borderBottom: '1px solid var(--glass-border)', paddingBottom: 6, alignItems: 'center' }}>
          <div className="row" style={{ gap: 8, alignItems: 'center' }}>
            <span>Interactive Legend (Click item to toggle line visibility)</span>
            {hoverIdx !== null && hoverIdx < t.length && (
              <span className="mono" style={{ color: 'var(--accent)', marginLeft: 8 }}>
                Time: <strong>{t[hoverIdx].toFixed(2)}s</strong>
              </span>
            )}
          </div>
          <div className="row" style={{ gap: 8 }}>
            <button
              className="btn"
              style={{ padding: '2px 8px', fontSize: '0.66rem', borderRadius: '4px' }}
              onClick={() => {
                const updated = {};
                motorEntries.forEach(([motor]) => { updated[motor] = true; });
                setVisibleMotors(updated);
              }}
            >
              Show All
            </button>
            <button
              className="btn"
              style={{ padding: '2px 8px', fontSize: '0.66rem', borderRadius: '4px' }}
              onClick={() => {
                const updated = {};
                motorEntries.forEach(([motor]) => { updated[motor] = false; });
                setVisibleMotors(updated);
              }}
            >
              Hide All
            </button>
          </div>
        </div>

        {/* Legend grid */}
        <div className="row" style={{ gap: '8px 10px', flexWrap: 'wrap', fontSize: '0.74rem' }}>
          {motorEntries.map(([motorLabel, values]) => {
            const isVisible = visibleMotors[motorLabel] !== false;
            const motorNum = parseInt(motorLabel.replace(/\D/g, '')) || 1;
            const strokeColor = MOTOR_COLORS[(motorNum - 1) % MOTOR_COLORS.length];
            const val = hoverIdx !== null && hoverIdx < t.length ? values[hoverIdx] : null;
            const formattedVal = val != null ? `${val > 0 ? '+' : ''}${Math.round(val * 100)}%` : '';

            return (
              <div
                key={motorLabel}
                onClick={() => {
                  setVisibleMotors(prev => ({
                    ...prev,
                    [motorLabel]: !isVisible
                  }));
                }}
                className="row"
                style={{
                  alignItems: 'center',
                  gap: 6,
                  cursor: 'pointer',
                  padding: '4px 8px',
                  borderRadius: 6,
                  background: isVisible ? 'rgba(255,255,255,0.03)' : 'rgba(255,255,255,0.01)',
                  border: `1px solid ${isVisible ? 'var(--glass-border)' : 'transparent'}`,
                  opacity: isVisible ? 1 : 0.45,
                  transition: 'all 0.15s ease',
                  userSelect: 'none'
                }}
              >
                <span style={{
                  display: 'inline-block',
                  width: 8,
                  height: 8,
                  borderRadius: '50%',
                  backgroundColor: strokeColor,
                  border: isVisible ? 'none' : '1px dashed var(--text-muted)'
                }} />
                <span className="mono" style={{ fontWeight: isVisible ? 600 : 400 }}>{motorLabel}</span>
                {isVisible && formattedVal && (
                  <strong className="mono" style={{ color: Math.abs(val) >= 0.15 ? 'var(--warn)' : 'var(--text)', marginLeft: 2 }}>
                    {formattedVal}
                  </strong>
                )}
              </div>
            );
          })}
        </div>
      </div>
    );
  };

  return (
    <div className="glass panel stack" style={{ padding: 14, gap: 12, background: 'rgba(0,0,0,0.1)' }}>
      <div className="row spread" style={{ alignItems: 'center' }}>
        <span style={{ fontSize: '0.76rem', textTransform: 'uppercase', color: 'var(--accent)', fontWeight: 600, letterSpacing: '0.05em' }}>
          Motor Hover Balance History (Deviations from Fleet Mean)
        </span>
        <span className="muted" style={{ fontSize: '0.68rem' }}>
          💡 Drag to zoom time | Double-click to reset
        </span>
      </div>
      <div ref={containerRef} style={{ width: '100%', minHeight: 220 }} />
      <div style={{ borderTop: '1px solid var(--glass-border)', paddingTop: 10 }}>
        {renderInteractiveLegend()}
      </div>
    </div>
  );
}
