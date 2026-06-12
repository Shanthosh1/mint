"""
Deterministic safety boundary registry.

This module is the single gatekeeper between *advice* and *hardware*.
Every proposed parameter change MUST pass `validate_proposal()` before it
is even shown to the pilot, and again immediately before the approved
write is dispatched. The registry itself is a read-only JSON file baked
into the bundle — it is intentionally NOT editable from the UI.
"""
from __future__ import annotations

import json
import sys
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Optional

def _locate_registry() -> Path:
    """Find the registry JSON in dev and in the PyInstaller bundle.

    Dev: sibling of this module. Frozen: PyInstaller unpacks data files
    under sys._MEIPASS; try the module-relative path first (works when
    the spec dest matches the import path), then the bundle root.
    """
    sibling = Path(__file__).with_name("safety_registry.json")
    if sibling.is_file():
        return sibling
    if getattr(sys, "frozen", False):
        candidate = Path(getattr(sys, "_MEIPASS")) / "app" / "core" / "safety_registry.json"
        if candidate.is_file():
            return candidate
    raise FileNotFoundError(
        f"safety_registry.json not found near {sibling} — "
        f"check the PyInstaller datas mapping in mint.spec"
    )


_REGISTRY_PATH = _locate_registry()


class Verdict(str, Enum):
    OK = "ok"                       # within step delta and absolute caps
    CLAMPED = "clamped"             # advisor wanted more; value reduced to limit
    REJECTED_UNKNOWN = "rejected_unknown_param"   # param not in registry: never writable
    REJECTED_BOUNDS = "rejected_out_of_bounds"    # even clamped value is unsafe


@dataclass
class SafetyCheck:
    verdict: Verdict
    requested: float
    allowed: Optional[float]   # value that may actually be written (None if rejected)
    reason: str


class SafetyRegistry:
    """In-memory view of the hardcoded JSON safety registry."""

    def __init__(self, path: Path = _REGISTRY_PATH):
        with open(path, "r", encoding="utf-8") as f:
            raw = json.load(f)
        # "COMMON" holds airframe-agnostic parameters (sensor filters)
        # merged into every class; class-specific entries win on conflict.
        common = raw.get("COMMON", {})
        self._table: dict[str, dict] = {
            k: {**common, **v} for k, v in raw.items()
            if not k.startswith("_") and k != "COMMON"
        }

    def airframe_classes(self) -> list[str]:
        return list(self._table.keys())

    def params_for(self, airframe_class: str) -> dict[str, dict]:
        return self._table.get(airframe_class, {})

    def is_int(self, airframe_class: str, param: str) -> bool:
        """True for integer parameters (must be written via set_param_int)."""
        limits = self._table.get(airframe_class, {}).get(param, {})
        return limits.get("type") == "int"

    def validate_proposal(
        self,
        airframe_class: str,
        param: str,
        current_value: float,
        proposed_value: float,
    ) -> SafetyCheck:
        """
        Validate a proposed parameter change for a given airframe class.

        Order of enforcement:
          1. Unknown param for this class -> hard reject (whitelist model).
          2. Clamp the per-write step delta to `max_step`.
          3. Clamp the result into [abs_min, abs_max].
          4. If clamping inverted the intent (no movement possible), reject.
        """
        limits = self._table.get(airframe_class, {}).get(param)
        if limits is None:
            return SafetyCheck(
                verdict=Verdict.REJECTED_UNKNOWN,
                requested=proposed_value,
                allowed=None,
                reason=f"{param} is not whitelisted for airframe class {airframe_class}",
            )

        max_step = limits["max_step"]
        abs_min, abs_max = limits["abs_min"], limits["abs_max"]

        # Step-delta clamp.
        delta = proposed_value - current_value
        clamped_delta = max(-max_step, min(max_step, delta))
        stepped = current_value + clamped_delta

        # Absolute-cap clamp.
        final = max(abs_min, min(abs_max, stepped))

        if delta != 0 and (final - current_value) * delta <= 0:
            # Clamping removed all movement in the requested direction —
            # the current value is already pinned at a safety boundary.
            return SafetyCheck(
                verdict=Verdict.REJECTED_BOUNDS,
                requested=proposed_value,
                allowed=None,
                reason=(
                    f"{param} is already at its safety boundary "
                    f"[{abs_min}, {abs_max}]; no further change permitted"
                ),
            )

        if final != proposed_value:
            return SafetyCheck(
                verdict=Verdict.CLAMPED,
                requested=proposed_value,
                allowed=round(final, 6),
                reason=(
                    f"Requested {proposed_value} exceeded limits "
                    f"(max step ±{max_step}, range [{abs_min}, {abs_max}]); "
                    f"clamped to {round(final, 6)}"
                ),
            )

        return SafetyCheck(
            verdict=Verdict.OK,
            requested=proposed_value,
            allowed=round(final, 6),
            reason="Within per-step delta and absolute caps",
        )


# Module-level singleton — the registry is immutable, one copy is enough.
REGISTRY = SafetyRegistry()
