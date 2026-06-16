import { useEffect, useRef, useState } from 'react';
import uPlot from 'uplot';

// Common grid parameters
const DT = 0.01;
const T_MIN = -0.2;
const T_MAX = 1.5;

function generateCommonGrid() {
  const grid = [];
  for (let t = T_MIN; t <= T_MAX; t = parseFloat((t + DT).toFixed(2))) {
    grid.push(t);
  }
  return grid;
}

function interpolate(xGrid, yGrid, targetX) {
  if (xGrid.length === 0) return 0;
  if (targetX <= xGrid[0]) return yGrid[0];
  if (targetX >= xGrid[xGrid.length - 1]) return yGrid[yGrid.length - 1];

  for (let i = 0; i < xGrid.length - 1; i++) {
    if (targetX >= xGrid[i] && targetX <= xGrid[i + 1]) {
      const x0 = xGrid[i];
      const x1 = xGrid[i + 1];
      const y0 = yGrid[i];
      const y1 = yGrid[i + 1];
      const t = (targetX - x0) / (x1 - x0);
      return y0 + t * (y1 - y0);
    }
  }
  return yGrid[yGrid.length - 1];
}

function getPercentile(vals, p) {
  if (vals.length === 0) return 0;
  const sorted = [...vals].sort((a, b) => a - b);
  const pos = (sorted.length - 1) * p;
  const base = Math.floor(pos);
  const rest = pos - base;
  if (sorted[base + 1] !== undefined) {
    return sorted[base] + rest * (sorted[base + 1] - sorted[base]);
  }
  return sorted[base];
}

export default function StepResponseEnvelope({ steps = [], tau, settling, overshoot, axisLabel }) {
  const containerRef = useRef(null);
  const plotRef = useRef(null);
  const [hoveredStep, setHoveredStep] = useState(null);
  const [showSpread, setShowSpread] = useState(true);

  const validSteps = steps && Array.isArray(steps) ? steps.filter(s => s && s.t && s.sp && s.act) : [];

  useEffect(() => {
    setHoveredStep(null);
  }, [axisLabel, validSteps.length]);

  useEffect(() => {
    if (validSteps.length < 3 || !containerRef.current) {
      if (plotRef.current) {
        plotRef.current.destroy();
        plotRef.current = null;
      }
      return;
    }

    const commonGrid = generateCommonGrid();

    // 1. Align and normalize each step
    const normalizedSteps = validSteps.map((step) => {
      const amp = step.amplitude || 1.0;
      const sp_pre = step.sp_pre !== undefined 
        ? step.sp_pre 
        : (step.sp.slice(0, 10).reduce((a, b) => a + b, 0) / 10);
      const t0 = step.t0 !== undefined ? step.t0 : step.t[0];

      // Align time relative to t0 and normalize
      const t_aligned = step.t.map(tVal => tVal - t0);
      const sp_norm = step.sp.map(val => (val - sp_pre) / amp);
      const act_norm = step.act.map(val => (val - sp_pre) / amp);

      // Interpolate onto common grid
      const interpSp = commonGrid.map(tVal => interpolate(t_aligned, sp_norm, tVal));
      const interpAct = commonGrid.map(tVal => interpolate(t_aligned, act_norm, tVal));

      return {
        sp_norm: interpSp,
        act_norm: interpAct,
      };
    });

    // 2. Compute aggregate statistics (mean sp, mean act, P25/P75 act)
    const mean_sp = [];
    const mean_act = [];
    const p25_act = [];
    const p75_act = [];

    for (let j = 0; j < commonGrid.length; j++) {
      const spVals = normalizedSteps.map(s => s.sp_norm[j]);
      const actVals = normalizedSteps.map(s => s.act_norm[j]);

      const sumSp = spVals.reduce((a, b) => a + b, 0);
      const sumAct = actVals.reduce((a, b) => a + b, 0);

      mean_sp.push(sumSp / spVals.length);
      mean_act.push(sumAct / actVals.length);
      p25_act.push(getPercentile(actVals, 0.25));
      p75_act.push(getPercentile(actVals, 0.75));
    }

    // 3. Construct uPlot data
    // Series 0: Time
    // Series 1: Mean Setpoint
    // Series 2: Mean Actual
    // Series 3 to 3+N-1: Individual actual steps
    // Series 3+N: P25 Bound
    // Series 3+N+1: P75 Bound
    const uPlotData = [
      commonGrid,
      mean_sp,
      mean_act,
      ...normalizedSteps.map(s => s.act_norm),
      p25_act,
      p75_act,
    ];

    const numSteps = normalizedSteps.length;
    const seriesConfig = [
      {}, // x axis
      {
        label: 'Setpoint (Mean)',
        stroke: '#fbbf24',
        width: 1.5,
        dash: [6, 4],
      },
      {
        label: 'Actual (Mean)',
        stroke: '#4f8cff',
        width: 2.5,
      },
      // Individual step series (transparent or thin opacity in plot, hidden from legend)
      ...validSteps.map((_, i) => ({
        label: `Step ${i + 1}`,
        stroke: showSpread ? 'rgba(79, 140, 255, 0.15)' : 'transparent',
        width: showSpread ? 1.0 : 0,
      })),
      {
        label: 'P25 Bound',
        stroke: 'transparent',
        width: 0,
      },
      {
        label: 'P75 Bound',
        stroke: 'transparent',
        width: 0,
      },
    ];

    // Bands are used to fill the area between P25 Bound and P75 Bound
    const bandsConfig = [
      {
        series: [3 + numSteps, 3 + numSteps + 1],
        fill: showSpread ? 'rgba(79, 140, 255, 0.07)' : 'transparent',
      },
    ];

    const opts = {
      width: containerRef.current.clientWidth,
      height: 260,
      bands: bandsConfig,
      series: seriesConfig,
      cursor: {
        show: true,
        drag: { x: false, y: false },
      },
      axes: [
        {
          stroke: '#5e6a85',
          grid: { stroke: 'rgba(255,255,255,0.06)' },
        },
        {
          stroke: '#5e6a85',
          grid: { stroke: 'rgba(255,255,255,0.06)' },
          values: (u, vals) => vals.map((v) => Number(v).toFixed(1)),
        },
      ],
      legend: {
        show: false, // Hide default legend to avoid list cluttering; we build custom legend below
      },
      hooks: {
        setCursor: [
          (self) => {
            const idx = self.cursor.idx;
            if (idx === null || idx === undefined || idx < 0 || idx >= commonGrid.length) {
              return;
            }
            const yVal = self.posToVal(self.cursor.top, 'y');

            // Find the closest step to the cursor Y position at the hovered X index
            let closestIdx = -1;
            let minDist = Infinity;

            normalizedSteps.forEach((nStep, i) => {
              const val = nStep.act_norm[idx];
              const dist = Math.abs(val - yVal);
              if (dist < minDist) {
                minDist = dist;
                closestIdx = i;
              }
            });

            if (closestIdx !== -1) {
              const targetId = closestIdx + 1;
              setHoveredStep((prev) => {
                if (prev && prev.id === targetId) return prev;
                const originalStep = validSteps[closestIdx];
                return {
                  id: targetId,
                  tau_s: originalStep.tau_s,
                  overshoot: originalStep.overshoot,
                  settling_s: originalStep.settling_s,
                  amplitude: originalStep.amplitude,
                  noise_ratio: originalStep.noise_ratio,
                  ramped_input: originalStep.ramped_input,
                  confidence: originalStep.confidence,
                };
              });
            }
          },
        ],
      },
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
            height: 260,
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
  }, [axisLabel, validSteps.length, showSpread]);

  if (validSteps.length < 3) {
    return (
      <div className="alert info" style={{ marginTop: 12 }}>
        ℹ️ Not enough steps to generate envelope (n={validSteps.length}). At least 3 steps are required.
      </div>
    );
  }

  const formatSec = (v) => (v === null || v === undefined ? '—' : `${Number(v).toFixed(3)}s`);
  const formatPct = (v) => (v === null || v === undefined ? '—' : `${Math.round(v * 100)}%`);

  return (
    <div className="stack" style={{ gap: 14 }}>
      {/* Aggregate metrics header */}
      <div className="row muted mono" style={{ gap: 18, flexWrap: 'wrap', padding: '6px 12px', background: 'rgba(0,0,0,0.15)', borderRadius: 8, border: '1px solid var(--glass-border)', fontSize: '0.82rem' }}>
        <span>Axis: <strong style={{ color: 'var(--text)' }}>{axisLabel}</strong></span>
        <span title="Aggregate time constant (median)">Median τ: <strong style={{ color: 'var(--text)' }}>{formatSec(tau)}</strong></span>
        <span title="Aggregate settling time (median)">Median Settle: <strong style={{ color: 'var(--text)' }}>{formatSec(settling)}</strong></span>
        <span title="Maximum overshoot observed">Max Overshoot: <strong style={{ color: overshoot > 0.25 ? 'var(--warn)' : 'var(--text)' }}>{formatPct(overshoot)}</strong></span>
        <span>Steps: <strong style={{ color: 'var(--text)' }}>{validSteps.length}</strong></span>
      </div>

      <div style={{ display: 'grid', gridTemplateColumns: '1fr minmax(220px, 260px)', gap: 14, alignItems: 'start' }}>
        {/* Plot container */}
        <div style={{ position: 'relative', width: '100%' }}>
          {/* Custom clean legend with Spread Toggle Button */}
          <div className="row spread" style={{ marginBottom: 8, alignItems: 'center' }}>
            <div className="row" style={{ gap: 12, fontSize: '0.74rem', flexWrap: 'wrap' }}>
              <span style={{ display: 'inline-flex', alignItems: 'center', gap: 6 }}>
                <span style={{ display: 'inline-block', width: 12, height: 2, background: '#fbbf24', borderTop: '2px dashed #fbbf24' }} /> Setpoint
              </span>
              <span style={{ display: 'inline-flex', alignItems: 'center', gap: 6 }}>
                <span style={{ display: 'inline-block', width: 12, height: 3, background: '#4f8cff' }} /> Actual (Mean)
              </span>
              {showSpread && (
                <span style={{ display: 'inline-flex', alignItems: 'center', gap: 6 }}>
                  <span style={{ display: 'inline-block', width: 12, height: 8, background: 'rgba(79, 140, 255, 0.15)' }} /> Spread (P25 - P75)
                </span>
              )}
            </div>
            <button
              className="btn"
              style={{ padding: '3px 8px', fontSize: '0.7rem', borderRadius: '5px' }}
              onClick={() => setShowSpread(!showSpread)}
            >
              {showSpread ? 'Hide Individual Steps' : 'Show Individual Steps'}
            </button>
          </div>
          <div ref={containerRef} style={{ width: '100%' }} />
        </div>

        {/* Hover/metrics side panel */}
        <div className="proposal" style={{ height: '100%', minHeight: 260, display: 'flex', flexDirection: 'column', justifyContent: 'center', margin: 0, padding: 14 }}>
          {hoveredStep ? (
            <div className="stack" style={{ gap: 6, width: '100%' }}>
              <h4 style={{ margin: '0 0 6px 0', fontSize: '0.8rem', textTransform: 'uppercase', color: 'var(--accent)', letterSpacing: '0.05em' }}>
                Step {hoveredStep.id} Details
              </h4>
              <div className="row spread mono" style={{ fontSize: '0.76rem' }}>
                <span>Rise τ:</span>
                <strong>{formatSec(hoveredStep.tau_s)}</strong>
              </div>
              <div className="row spread mono" style={{ fontSize: '0.76rem' }}>
                <span>Settling:</span>
                <strong>{formatSec(hoveredStep.settling_s)}</strong>
              </div>
              <div className="row spread mono" style={{ fontSize: '0.76rem' }}>
                <span>Overshoot:</span>
                <strong style={{ color: hoveredStep.overshoot > 0.25 ? 'var(--warn)' : 'inherit' }}>
                  {formatPct(hoveredStep.overshoot)}
                </strong>
              </div>
              <div className="row spread mono" style={{ fontSize: '0.76rem' }}>
                <span>Amplitude:</span>
                <strong>{Number(hoveredStep.amplitude).toFixed(2)} rad/s</strong>
              </div>
              <div className="row spread mono" style={{ fontSize: '0.76rem' }}>
                <span>Noise Ratio:</span>
                <strong>{(hoveredStep.noise_ratio * 100).toFixed(0)}%</strong>
              </div>
              {hoveredStep.ramped_input && (
                <div style={{ fontSize: '0.72rem', color: '#fbbf24', fontWeight: 500, marginTop: 4 }}>
                  ⚠️ Ramped input (τ adjusted)
                </div>
              )}
              {hoveredStep.confidence && hoveredStep.confidence.score !== undefined && (
                <div style={{ marginTop: 6, borderTop: '1px solid var(--glass-border)', paddingTop: 6 }}>
                  <div className="row spread mono" style={{ fontSize: '0.76rem' }}>
                    <span>Step Conf:</span>
                    <strong style={{ color: hoveredStep.confidence.score > 0.7 ? 'var(--ok)' : hoveredStep.confidence.score > 0.4 ? 'var(--warn)' : 'var(--crit)' }}>
                      {Math.round(hoveredStep.confidence.score * 100)}%
                    </strong>
                  </div>
                </div>
              )}
            </div>
          ) : (
            <div className="muted" style={{ textAlign: 'center', fontSize: '0.78rem' }}>
              💡 Hover cursor over curves to inspect individual step parameters.
            </div>
          )}
        </div>
      </div>
    </div>
  );
}
