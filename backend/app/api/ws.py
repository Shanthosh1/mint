"""
WebSocket telemetry bridge.

One socket per browser tab at /ws/telemetry. High-rate channels
(attitude, pid_metrics, ekf_metrics) are decimated to WS_STREAM_HZ by
the Downsampler; event-like channels (alerts, prompts, proposals,
airframe, connection) always pass through.

Wire format (JSON):
    { "ch": "<channel>", "t": <monotonic-ts>, "d": { ...payload } }
"""
from __future__ import annotations

import json
import logging

from fastapi import APIRouter, WebSocket, WebSocketDisconnect

from ..mavlink.telemetry_hub import HUB, Downsampler

log = logging.getLogger("mint.ws")
router = APIRouter()

# Channels exempt from downsampling — every event matters.
_PASSTHROUGH = {
    "alert", "pilot_prompt", "proposal", "airframe",
    "connection", "param_written", "regime", "recommendation",
    "flight_mode", "cascade_state", "telemetry_stale",
}


@router.websocket("/ws/telemetry")
async def telemetry_socket(ws: WebSocket) -> None:
    await ws.accept()
    downsampler = Downsampler()
    log.info("WS client connected: %s", ws.client)
    try:
        # Replay current state so a (re)connecting tab paints immediately
        # instead of waiting for the next live sample on each channel.
        for event in HUB.latest_snapshot():
            await ws.send_text(json.dumps({
                "ch": event.channel,
                "t": event.ts,
                "ts": event.ts_epoch,
                "d": event.payload,
                "replay": True,
            }))
        async for event in HUB.subscribe():
            if event.channel not in _PASSTHROUGH and not downsampler.admit(event.channel):
                continue
            await ws.send_text(json.dumps({
                "ch": event.channel,
                "t": event.ts,
                "ts": event.ts_epoch,
                "d": event.payload,
            }))
    except WebSocketDisconnect:
        log.info("WS client disconnected: %s", ws.client)
    except Exception:
        log.exception("WS stream error")
