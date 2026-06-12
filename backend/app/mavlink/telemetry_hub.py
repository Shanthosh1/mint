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
    ts: float = field(default_factory=time.monotonic)


class TelemetryHub:
    """Many-producer / many-consumer pub-sub with per-subscriber queues."""

    def __init__(self, queue_size: int = 256):
        self._queue_size = queue_size
        self._subscribers: dict[int, asyncio.Queue[Event]] = {}
        self._next_id = 0
        self._channel_counts: dict[str, int] = defaultdict(int)

    def publish(self, channel: str, payload: dict[str, Any]) -> None:
        """Non-blocking publish; never raises on full queues."""
        event = Event(channel=channel, payload=payload)
        self._channel_counts[channel] += 1
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
