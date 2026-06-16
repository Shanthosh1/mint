import { useEffect, useRef, useState } from 'react';
import uPlot from 'uplot';

const MOTOR_COLORS = ['#38bdf8', '#0ea5e9', '#6366f1', '#8b5cf6', '#2563eb', '#1d4ed8', '#4f46e5', '#7c3aed'];
const SURFACE_COLORS = ['#f43f5e', '#ec4899', '#f97316', '#eab308', '#d946ef', '#a855f7'];

export default function ActuatorTimelineChart({ t = [], channels = {}, airspeed = null }) {
  const containerRef = useRef(null);
  const plotRef = useRef(null);
  const [hoverIdx, setHoverIdx] = useState(null);

  const channelEntries = Object.entries(channels);

  const [visibleChannels, setVisibleChannels] = useState(() => {
    const initial = {};
    Object.keys(channels).forEach(ch => {
      initial[ch] = true;
    });
    return initial;
  });

  const [prevChannelsKey, setPrevChannelsKey] = useState('');
  const channelsKey = Object.keys(channels).join(',');
  if (channelsKey !== prevChannelsKey) {
    const initial = {};
    Object.keys(channels).forEach(ch => {
      initial[ch] = true;
    });
    setVisibleChannels(initial);
    setPrevChannelsKey(channelsKey);
  }

  // Pre-calculate colors consistently between uPlot series and legend items
  const channelColors = {};
  let motorColorIdx = 0;
  let surfColorIdx = 0;
  channelEntries.forEach(([chLabel]) => {
    const isMotor = chLabel.startsWith('M') || chLabel.startsWith('P') || chLabel.includes('thrust');
    if (isMotor) {
      channelColors[chLabel] = MOTOR_COLORS[motorColorIdx % MOTOR_COLORS.length];
      motorColorIdx++;
    } else {
      channelColors[chLabel] = SURFACE_COLORS[surfColorIdx % SURFACE_COLORS.length];
      surfColorIdx++;
    }
  });

  useEffect(() => {
    if (t.length === 0 || channelEntries.length === 0 || !containerRef.current) {
      if (plotRef.current) {
        plotRef.current.destroy();
        plotRef.current = null;
      }
      return;
    }

    // 1. Gather Series Data
    const uPlotData = [t];
    const seriesConfig = [
      {
        // X axis (Time)
      }
    ];

    channelEntries.forEach(([chLabel, chData]) => {
      uPlotData.push(chData.values || []);
      const strokeColor = channelColors[chLabel] || '#fff';

      seriesConfig.push({
        label: chLabel,
        stroke: strokeColor,
        width: 1.5,
        scale: 'y',
        points: { show: false },
        show: visibleChannels[chLabel] !== false
      });
    });

    const hasAirspeed = airspeed && airspeed.length === t.length;
    if (hasAirspeed) {
      uPlotData.push(airspeed);
      seriesConfig.push({
        label: 'Airspeed',
        stroke: '#10b981',
        width: 2.0,
        dash: [4, 4],
        scale: 'airspeed',
        points: { show: false }
      });
    }

    // Determine range for Y (actuator inputs)
    const isBipolar = channelEntries.some(([label]) => {
      const lower = label.toLowerCase();
      return lower.startsWith('s') ||
             lower.startsWith('tilt') ||
             lower.includes('elevon') ||
             lower.includes('aileron') ||
             lower.includes('elevator') ||
             lower.includes('rudder') ||
             lower.includes('v-tail') ||
             lower.includes('a-tail') ||
             lower.includes('flap') ||
             lower.includes('spoiler') ||
             lower.includes('airbrake') ||
             lower.includes('roll') ||
             lower.includes('pitch') ||
             lower.includes('yaw');
    });
    const yMin = isBipolar ? -1.05 : -0.05;
    const yMax = 1.05;

    const scalesConfig = {
      x: { time: false },
      y: { range: [yMin, yMax] }
    };

    const axesConfig = [
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
    ];

    if (hasAirspeed) {
      scalesConfig.airspeed = { range: [0, Math.max(25, ...airspeed) * 1.1] };
      axesConfig.push({
        scale: 'airspeed',
        side: 1, // Right-side axis
        stroke: '#10b981',
        grid: { show: false },
        values: (u, vals) => vals.map(v => `${Number(v).toFixed(0)} m/s`)
      });
    }

    const opts = {
      width: containerRef.current.clientWidth,
      height: 280,
      series: seriesConfig,
      scales: scalesConfig,
      axes: axesConfig,
      cursor: {
        show: true,
        drag: { x: true, y: false }
      },
      legend: {
        show: false // Hide default legend
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
            height: 280
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
  }, [t.length, Object.keys(channels).length, airspeed ? airspeed.length : 0, JSON.stringify(visibleChannels)]);

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
                {airspeed && airspeed[hoverIdx] != null && (
                  <> | Airspeed: <strong style={{ color: '#10b981' }}>{airspeed[hoverIdx].toFixed(1)} m/s</strong></>
                )}
              </span>
            )}
          </div>
          <div className="row" style={{ gap: 8 }}>
            <button
              className="btn"
              style={{ padding: '2px 8px', fontSize: '0.66rem', borderRadius: '4px' }}
              onClick={() => {
                const updated = {};
                channelEntries.forEach(([ch]) => { updated[ch] = true; });
                setVisibleChannels(updated);
              }}
            >
              Show All
            </button>
            <button
              className="btn"
              style={{ padding: '2px 8px', fontSize: '0.66rem', borderRadius: '4px' }}
              onClick={() => {
                const updated = {};
                channelEntries.forEach(([ch]) => { updated[ch] = false; });
                setVisibleChannels(updated);
              }}
            >
              Hide All
            </button>
          </div>
        </div>

        {/* Legend grid */}
        <div className="row" style={{ gap: '8px 10px', flexWrap: 'wrap', fontSize: '0.74rem' }}>
          {channelEntries.map(([chLabel, chData]) => {
            const isVisible = visibleChannels[chLabel] !== false;
            const strokeColor = channelColors[chLabel];
            const val = hoverIdx !== null && hoverIdx < t.length && chData.values ? chData.values[hoverIdx] : null;
            const formattedVal = val != null ? `${Math.round(val * 100)}%` : '';

            return (
              <div
                key={chLabel}
                onClick={() => {
                  setVisibleChannels(prev => ({
                    ...prev,
                    [chLabel]: !isVisible
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
                <span className="mono" style={{ fontWeight: isVisible ? 600 : 400 }}>{chLabel}</span>
                {isVisible && formattedVal && (
                  <strong className="mono" style={{ color: 'var(--text)', marginLeft: 2 }}>
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
          Actuator Timeline Graph
        </span>
        <div className="row" style={{ gap: 10, alignItems: 'center' }}>
          <span className="muted" style={{ fontSize: '0.68rem' }}>
            💡 Drag to zoom time | Double-click to reset
          </span>
          {airspeed && (
            <span style={{ fontSize: '0.7rem', color: '#10b981', display: 'inline-flex', alignItems: 'center', gap: 4 }}>
              <span style={{ display: 'inline-block', width: 10, height: 2, background: '#10b981', borderTop: '2px dashed #10b981' }} /> Airspeed Overlay Enabled
            </span>
          )}
        </div>
      </div>
      <div ref={containerRef} style={{ width: '100%', minHeight: 280 }} />
      <div style={{ borderTop: '1px solid var(--glass-border)', paddingTop: 10 }}>
        {renderInteractiveLegend()}
      </div>
    </div>
  );
}
