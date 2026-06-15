"""
Asyncio fan-out hub between MAVLink collection tasks and consumers.

Collectors publish typed events; subscribers (the WebSocket layer, the
live analysis engines) each get an independent bounded queue. Slow
consumers drop the *oldest* sample instead of back-pressuring telemetry
ingestion — losing a stale UI frame is always preferable to stalling
the analysis pipeline.

A separate `Downsampler` decimates high-rate streams to the UI rate so
the browser never receives more than WS_STREAM_HZ frames per channel.
"""
from __future__ import annotations

import asyncio
import time
from collections import defaultdict
from dataclasses import dataclass, field
from typing import Any, AsyncIterator

from ..core import config


@dataclass
class Event:
    channel: str                 # e.g. "attitude", "ekf", "advice", "alert"
    payload: dict[str, Any]
    ts: float = field(default_factory=time.monotonic)        # monotonic, server-side
    ts_epoch: float = field(default_factory=time.time)       # wall-clock for the UI


# Channels whose latest value represents current *state* and is worth
# replaying to a (re)connecting UI. Deliberately excludes event-like channels
# (alert, pilot_prompt, proposal, param_written) — replaying those would
# resurface notifications the pilot already saw or dismissed.
_REPLAY_CHANNELS: frozenset[str] = frozenset({
    "connection", "airframe", "regime", "flight_mode", "cascade_state",
    "ekf_metrics", "loop_metrics", "vibration_metrics", "telemetry_stale",
    "actuation",
})


class TelemetryHub:
    """Many-producer / many-consumer pub-sub with per-subscriber queues."""

    def __init__(self, queue_size: int = 256):
        self._queue_size = queue_size
        self._subscribers: dict[int, asyncio.Queue[Event]] = {}
        self._next_id = 0
        self._channel_counts: dict[str, int] = defaultdict(int)
        # Monotonic timestamp of the last publish per channel — drives
        # staleness detection so consumers can tell "no fault" apart from
        # "the data stopped arriving" (dead router, FC stopped streaming).
        self._last_seen: dict[str, float] = {}
        # Last event per *state-like* channel, kept so a freshly (re)connected
        # WebSocket can replay current state instead of showing blank panels
        # until the next sample arrives. Event-like channels (alerts, prompts)
        # are NOT retained — replaying them would re-fire stale notifications.
        self._latest: dict[str, Event] = {}

    def publish(self, channel: str, payload: dict[str, Any]) -> None:
        """Non-blocking publish; never raises on full queues."""
        event = Event(channel=channel, payload=payload)
        self._channel_counts[channel] += 1
        self._last_seen[channel] = event.ts
        if channel in _REPLAY_CHANNELS:
            # loop_metrics multiplexes rate/attitude/velocity/position over one
            # channel via a "loop" field — sub-key so all loops replay, not
            # just whichever published last.
            key = channel
            if channel == "loop_metrics":
                key = f"loop_metrics:{payload.get('loop', '')}"
            self._latest[key] = event
        for q in self._subscribers.values():
            if q.full():
                try:
                    q.get_nowait()  # evict oldest
                except asyncio.QueueEmpty:
                    pass
            q.put_nowait(event)

    async def subscribe(self) -> AsyncIterator[Event]:
        """Async iterator yielding every event until the consumer cancels."""
        sub_id = self._next_id
        self._next_id += 1
        q: asyncio.Queue[Event] = asyncio.Queue(self._queue_size)
        self._subscribers[sub_id] = q
        try:
            while True:
                yield await q.get()
        finally:
            self._subscribers.pop(sub_id, None)

    def is_stale(self, channel: str, max_age_s: float = 2.0) -> bool:
        """True if `channel` has never been seen or is older than `max_age_s`.

        Uses the same monotonic clock as Event.ts. Engines gate analysis on
        this so a dead firehose surfaces as an explicit staleness signal
        instead of frozen metrics the pilot might trust.
        """
        last = self._last_seen.get(channel)
        return last is None or (time.monotonic() - last) > max_age_s

    def last_seen(self, channel: str) -> float | None:
        """Monotonic timestamp of the last publish on `channel`, or None."""
        return self._last_seen.get(channel)

    def latest_snapshot(self) -> list[Event]:
        """Most-recent event per replayable state channel, oldest first.

        A (re)connecting WebSocket sends these before the live stream so the
        UI paints current state immediately instead of blank panels.
        """
        return sorted(self._latest.values(), key=lambda e: e.ts)

    def clear_latest(self) -> None:
        """Clear cached state snapshots. Typically called on manual disconnect."""
        self._latest.clear()
        self._last_seen.clear()

    def stats(self) -> dict:
        return {
            "subscribers": len(self._subscribers),
            "events_by_channel": dict(self._channel_counts),
        }


class Downsampler:
    """
    Per-channel rate limiter for UI-bound streams.

    Keeps the most recent sample per channel and releases it at most
    `hz` times per second. Analysis consumers bypass this entirely and
    read the hub at full rate.
    """

    def __init__(self, hz: float = config.WS_STREAM_HZ):
        self._min_interval = 1.0 / hz
        self._last_emit: dict[str, float] = {}

    def admit(self, channel: str) -> bool:
        now = time.monotonic()
        last = self._last_emit.get(channel, 0.0)
        if now - last >= self._min_interval:
            self._last_emit[channel] = now
            return True
        return False


HUB = TelemetryHub()
