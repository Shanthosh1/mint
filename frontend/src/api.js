/**
 * Thin REST client. All endpoints are same-origin (FastAPI serves the SPA);
 * the Vite dev server proxies them during development.
 */
async function request(path, options = {}) {
  const res = await fetch(path, {
    headers: { 'Content-Type': 'application/json' },
    ...options,
  });
  if (!res.ok) {
    const body = await res.json().catch(() => ({}));
    throw new Error(body.detail || `${res.status} ${res.statusText}`);
  }
  return res.json();
}

export const api = {
  host: () => request('/api/system/host'),
  frontendConfig: () => request('/api/system/config/frontend'),
  serialPorts: () => request('/api/system/serial-ports'),
  routerStatus: () => request('/api/system/router'),
  /** target: { mode, serial_device, baud, host, port } */
  routerStart: (target) =>
    request('/api/system/router/start', {
      method: 'POST', body: JSON.stringify(target),
    }),
  routerStop: () => request('/api/system/router/stop', { method: 'POST' }),
  vehicleConnect: () => request('/api/system/vehicle/connect', { method: 'POST' }),
  vehicleDisconnect: () => request('/api/system/vehicle/disconnect', { method: 'POST' }),
  updateActuatorMap: (map) =>
    request('/api/system/actuator-map', {
      method: 'POST', body: JSON.stringify(map),
    }),

  proposals: () => request('/api/params/proposals'),
  approveProposal: (id) =>
    request(`/api/params/proposals/${id}/approve`, { method: 'POST' }),
  dismissProposal: (id) =>
    request(`/api/params/proposals/${id}`, { method: 'DELETE' }),
  revertProposal: (id) =>
    request(`/api/params/proposals/${id}/revert`, { method: 'POST' }),
  /** outcome: 'better' | 'worse' | 'no_change' */
  proposalFeedback: (id, outcome) =>
    request(`/api/params/proposals/${id}/feedback`, {
      method: 'POST', body: JSON.stringify({ outcome }),
    }),
  /** body: { param, rationale, and exactly one of target_value | scale_factor | delta } */
  createProposal: (body) =>
    request('/api/params/proposals', {
      method: 'POST', body: JSON.stringify(body),
    }),
  startTuningWindow: (axis, loop) =>
    request('/api/params/tuning-window/start', {
      method: 'POST', body: JSON.stringify({ axis, loop }),
    }),
  stopTuningWindow: () =>
    request('/api/params/tuning-window/stop', { method: 'POST' }),

  analyzeUlog: async (file, onProgress) => {
    // FormData upload — let the browser set the multipart boundary.
    const form = new FormData();
    form.append('file', file);
    const res = await fetch('/api/analyze-ulog', { method: 'POST', body: form });
    if (!res.ok) {
      const body = await res.json().catch(() => ({}));
      throw new Error(body.detail || `Upload failed (${res.status})`);
    }
    return res.json();
  },
};
