"""
Tuning feedback memory.

Persists the pilot's verdict on each *written* parameter change — "better",
"worse", or "no change" — to an append-only JSONL file, and summarises prior
outcomes so a new proposal can be shown the relevant track record before the
pilot approves it.

Design notes:
  * Records are keyed for retrieval by (airframe_class, param, direction),
    where direction is "raise" or "lower". That is coarse enough to match
    across sessions/flights yet specific enough to be meaningful — the same
    gain moved the same way on the same airframe family is a genuine prior.
  * This is advisory ONLY. History is never fed back into the analysis
    engines or the safety registry: surfacing a prior keeps the human in the
    loop without letting past feedback entrench a bad local optimum or muddy
    the deterministic safety bounds.
  * The file is the source of truth; an in-memory index mirrors it for O(1)
    summary lookups and is rebuilt from disk on startup. A corrupt or partial
    line is skipped rather than fatal — a field log must keep working.
"""
from __future__ import annotations

import json
import logging
import time
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Literal, Optional

from ..core import config

log = logging.getLogger("mint.tuning_memory")

Outcome = Literal["better", "worse", "no_change"]
Direction = Literal["raise", "lower", "none"]
_VALID_OUTCOMES = ("better", "worse", "no_change")


def classify_direction(current: float, written: float) -> Direction:
    """Which way the parameter moved (the retrieval axis alongside param)."""
    if written > current:
        return "raise"
    if written < current:
        return "lower"
    return "none"


@dataclass
class OutcomeSummary:
    """Prior track record for one (airframe_class, param, direction) key."""
    param: str
    airframe_class: str
    direction: Direction
    better: int = 0
    worse: int = 0
    no_change: int = 0

    @property
    def total(self) -> int:
        return self.better + self.worse + self.no_change

    def to_dict(self) -> dict:
        return {
            "param": self.param,
            "airframe_class": self.airframe_class,
            "direction": self.direction,
            "better": self.better,
            "worse": self.worse,
            "no_change": self.no_change,
            "total": self.total,
        }


class TuningMemory:
    """Append-only outcome log with an in-memory summary index."""

    def __init__(self, path: Path = config.TUNING_HISTORY_PATH) -> None:
        self._path = path
        self._lock = threading.Lock()
        # (airframe_class, param, direction) -> {outcome: count}
        self._index: dict[tuple[str, str, str], dict[str, int]] = {}
        self._load()

    # ------------------------------------------------------------------ #
    def _load(self) -> None:
        """Rebuild the index from disk; tolerate a missing/partial file."""
        if not self._path.is_file():
            return
        try:
            with open(self._path, "r", encoding="utf-8") as f:
                for line in f:
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        self._index_record(json.loads(line))
                    except (json.JSONDecodeError, KeyError, TypeError):
                        # A truncated final line (interrupted write) or a
                        # hand-edited entry — skip it, keep the rest.
                        continue
        except OSError as exc:
            log.warning("Could not read tuning history %s: %s", self._path, exc)

    def _index_record(self, rec: dict) -> None:
        outcome = rec["outcome"]
        if outcome not in _VALID_OUTCOMES:
            return
        key = (rec["airframe_class"], rec["param"], rec["direction"])
        bucket = self._index.setdefault(key, {})
        bucket[outcome] = bucket.get(outcome, 0) + 1

    # ------------------------------------------------------------------ #
    def record_outcome(
        self,
        *,
        proposal_id: str,
        param: str,
        airframe_class: str,
        direction: Direction,
        current_value: float,
        written_value: float,
        outcome: Outcome,
    ) -> None:
        """Append one pilot verdict for a written change and index it."""
        if outcome not in _VALID_OUTCOMES:
            raise ValueError(
                f"outcome must be one of {_VALID_OUTCOMES}, got {outcome!r}"
            )
        rec = {
            "ts": time.time(),
            "proposal_id": proposal_id,
            "param": param,
            "airframe_class": airframe_class,
            "direction": direction,
            "current_value": current_value,
            "written_value": written_value,
            "outcome": outcome,
        }
        self._path.parent.mkdir(parents=True, exist_ok=True)
        # Append-only: a single line write is atomic enough for this size,
        # and the index is updated only after the line is safely on disk.
        with self._lock:
            with open(self._path, "a", encoding="utf-8") as f:
                f.write(json.dumps(rec) + "\n")
        self._index_record(rec)
        log.info("Recorded tuning outcome: %s %s (%s) -> %s",
                 param, direction, airframe_class, outcome)

    # ------------------------------------------------------------------ #
    def summarize(
        self, airframe_class: str, param: str, direction: Direction
    ) -> Optional[OutcomeSummary]:
        """Prior outcomes for this exact (airframe, param, direction), or None
        when there is no history to show."""
        bucket = self._index.get((airframe_class, param, direction))
        if not bucket:
            return None
        return OutcomeSummary(
            param=param,
            airframe_class=airframe_class,
            direction=direction,
            better=bucket.get("better", 0),
            worse=bucket.get("worse", 0),
            no_change=bucket.get("no_change", 0),
        )


MEMORY = TuningMemory()
