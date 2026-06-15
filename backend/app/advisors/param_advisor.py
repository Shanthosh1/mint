"""
Parameter proposal lifecycle — the Human-In-The-Loop enforcer.

State machine per proposal:

    DRAFT -> VALIDATED -> PRESENTED -> APPROVED -> WRITTEN
                  \\-> REJECTED (safety registry veto)

Invariants enforced here:
  * A proposal can only be created against the *currently detected*
    airframe class; no airframe, no proposals.
  * Safety validation runs at creation AND again at approval time
    (the current on-vehicle value is re-read in between, so the step
    delta is always computed against live truth).
  * `approve_and_write` is the only caller of
    ConnectionManager.write_approved_param.
"""
from __future__ import annotations

import asyncio
import threading
import time
import uuid
from dataclasses import dataclass, field, asdict
from enum import Enum
from typing import Optional

from ..core.safety_registry import REGISTRY, Verdict
from ..mavlink.connection import CONNECTION
from ..mavlink.telemetry_hub import HUB
from .tuning_memory import MEMORY, classify_direction


class ProposalState(str, Enum):
    PRESENTED = "presented"
    APPROVED = "approved"
    WRITTEN = "written"
    REJECTED = "rejected"
    FAILED = "write_failed"
    REVERTED = "reverted"        # written, then rolled back to the pre-write value


@dataclass
class Proposal:
    id: str
    param: str
    current_value: float
    proposed_value: float        # post-clamp value that would be written
    requested_value: float       # what the analyzer originally wanted
    rationale: str
    airframe_class: str
    state: ProposalState
    safety_note: str
    created_at: float = field(default_factory=time.time)
    # Prior pilot outcomes for this airframe/param/direction (advisory only,
    # surfaced to the UI). None when there is no history to show.
    tuning_history: Optional[dict] = None
    # Pilot's verdict once this was written and flown: better/worse/no_change.
    feedback: Optional[str] = None
    is_saturation_gain_reduction: bool = False
    confidence: Optional[str] = None
    limitations: Optional[str] = None
    written_at: Optional[float] = None


class ParamAdvisor:
    """Registry of pending proposals keyed by id."""

    def __init__(self) -> None:
        self._proposals: dict[str, Proposal] = {}
        self._diagnostics: dict[str, str] = {}
        self._lock = threading.Lock()
        self._timeout_task: Optional[asyncio.Task] = None

    def start(self) -> None:
        if self._timeout_task is None or self._timeout_task.done():
            self._timeout_task = asyncio.create_task(self._auto_timeout_loop(), name="advisor-timeout")

    def stop(self) -> None:
        if self._timeout_task:
            self._timeout_task.cancel()
            self._timeout_task = None

    async def _auto_timeout_loop(self) -> None:
        try:
            while True:
                await asyncio.sleep(5.0)
                now = time.time()
                to_update = []
                with self._lock:
                    for p in self._proposals.values():
                        if p.state == ProposalState.WRITTEN and p.feedback is None:
                            written_at = getattr(p, "written_at", None)
                            if written_at is not None and now - written_at > 60.0:
                                to_update.append(p)
                for p in to_update:
                    self.record_feedback(p.id, "no_feedback")
        except asyncio.CancelledError:
            pass
        except Exception:
            pass

    def set_diagnostic_card(self, axis: str, text: str) -> None:
        changed = False
        with self._lock:
            if self._diagnostics.get(axis) != text:
                self._diagnostics[axis] = text
                changed = True
        if changed:
            HUB.publish("proposal", {"refresh": True})

    def clear_diagnostic_card(self, axis: str) -> None:
        changed = False
        with self._lock:
            if axis in self._diagnostics:
                del self._diagnostics[axis]
                changed = True
        if changed:
            HUB.publish("proposal", {"refresh": True})

    def clear(self) -> None:
        with self._lock:
            self._proposals.clear()
            self._diagnostics.clear()



    # ------------------------------------------------------------------ #
    async def create_proposal(self, param: str, target_value: float,
                              rationale: str, is_saturation_gain_reduction: bool = False,
                              confidence: Optional[str] = None,
                              limitations: Optional[str] = None) -> Proposal:
        """Validate analyzer advice and stage it for pilot review."""
        airframe = CONNECTION.state.airframe
        if airframe is None:
            raise PermissionError(
                "Airframe not detected — proposals are disabled until "
                "SYS_AUTOSTART has been read from the vehicle."
            )

        current = await CONNECTION.read_param(param)

        if is_saturation_gain_reduction:
            from ..core import config
            if not config.EXPERT_MODE:
                raise PermissionError("Tuning gains is blocked during saturation. Back off rates instead.")
            if target_value >= current:
                raise ValueError("Expert mode P-gain modification during saturation must be a reduction.")
            if target_value < current * 0.85:
                target_value = round(current * 0.85, 6)

        check = REGISTRY.validate_proposal(
            airframe.airframe_class, param, current, target_value
        )
        written_value = check.allowed if check.allowed is not None else current

        # Surface the pilot's prior track record for this exact change so they
        # can weigh it before approving (advisory only — never auto-applied).
        summary = MEMORY.summarize(
            airframe.airframe_class, param,
            classify_direction(current, written_value),
        )

        prop = Proposal(
            id=uuid.uuid4().hex[:12],
            param=param,
            current_value=round(current, 6),
            proposed_value=written_value,
            requested_value=target_value,
            rationale=rationale,
            airframe_class=airframe.airframe_class,
            state=(ProposalState.REJECTED
                   if check.verdict in (Verdict.REJECTED_UNKNOWN, Verdict.REJECTED_BOUNDS)
                   else ProposalState.PRESENTED),
            safety_note=check.reason,
            tuning_history=summary.to_dict() if summary else None,
            is_saturation_gain_reduction=is_saturation_gain_reduction,
            confidence=confidence,
            limitations=limitations,
        )
        with self._lock:
            self._proposals[prop.id] = prop
        HUB.publish("proposal", asdict(prop))
        return prop

    # ------------------------------------------------------------------ #
    async def approve_and_write(self, proposal_id: str) -> Proposal:
        prop = None
        with self._lock:
            prop = self._proposals.get(proposal_id)
        if prop is None:
            raise KeyError(f"Unknown proposal {proposal_id}")
        if prop.state != ProposalState.PRESENTED:
            raise ValueError(f"Proposal {proposal_id} is {prop.state}, not approvable")

        live_current = await CONNECTION.read_param(prop.param)
        recheck = REGISTRY.validate_proposal(
            prop.airframe_class, prop.param, live_current, prop.proposed_value
        )
        if recheck.allowed is None:
            with self._lock:
                prop.state = ProposalState.REJECTED
                prop.safety_note = f"Re-validation failed at write time: {recheck.reason}"
            HUB.publish("proposal", asdict(prop))
            return prop

        with self._lock:
            prop.state = ProposalState.APPROVED
        try:
            await CONNECTION.write_approved_param(
                prop.param, recheck.allowed,
                as_int=REGISTRY.is_int(prop.airframe_class, prop.param),
            )
            with self._lock:
                prop.proposed_value = recheck.allowed
                prop.state = ProposalState.WRITTEN
                prop.written_at = time.time()
            
            # If this is a rate auto limit, notify pilot that new flight data is required to verify the fix
            if prop.param.startswith("MC_") and prop.param.endswith("RAUTO_MAX"):
                axis = prop.param.split("_")[1].replace("ROLL", "roll").replace("PITCH", "pitch").replace("YAW", "yaw").lower()
                HUB.publish("pilot_prompt", {
                    "axis": axis,
                    "text": f"New flight data required: Please perform dynamic maneuvers on the {axis} axis to verify that saturation is resolved.",
                    "kind": "step_input_request"
                })
        except Exception as exc:
            with self._lock:
                prop.state = ProposalState.FAILED
                prop.safety_note = f"Write failed: {exc}"
        HUB.publish("proposal", asdict(prop))
        return prop

    # ------------------------------------------------------------------ #
    async def revert(self, proposal_id: str) -> Proposal:
        prop = None
        with self._lock:
            prop = self._proposals.get(proposal_id)
        if prop is None:
            raise KeyError(f"Unknown proposal {proposal_id}")
        if prop.state != ProposalState.WRITTEN:
            raise ValueError(
                f"Proposal {proposal_id} is {prop.state}; only a written "
                f"change can be reverted."
            )

        live = await CONNECTION.read_param(prop.param)
        wrote = prop.proposed_value
        tol = max(1e-4, abs(wrote) * 1e-3)
        if abs(live - wrote) > tol:
            raise ValueError(
                f"{prop.param} is now {live:g}, not the {wrote:g} MINT wrote — "
                f"it was changed since (engine, pilot, or QGC). Revert blocked "
                f"so it can't clobber that newer value."
            )

        await CONNECTION.write_approved_param(
            prop.param, prop.current_value,
            as_int=REGISTRY.is_int(prop.airframe_class, prop.param),
        )
        with self._lock:
            prop.state = ProposalState.REVERTED
            prop.safety_note = (
                f"Reverted {prop.param} from {wrote:g} back to its pre-write "
                f"value {prop.current_value:g}."
            )
        HUB.publish("proposal", asdict(prop))
        return prop

    # ------------------------------------------------------------------ #
    def record_feedback(self, proposal_id: str, outcome: str) -> Proposal:
        with self._lock:
            prop = self._proposals.get(proposal_id)
            if prop is None:
                raise KeyError(f"Unknown proposal {proposal_id}")
            if prop.state != ProposalState.WRITTEN:
                raise ValueError(
                    f"Proposal {proposal_id} is {prop.state}; feedback is only "
                    f"valid once a change has been written."
                )
            if outcome not in ("skip", "no_feedback"):
                MEMORY.record_outcome(
                    proposal_id=prop.id,
                    param=prop.param,
                    airframe_class=prop.airframe_class,
                    direction=classify_direction(prop.current_value, prop.proposed_value),
                    current_value=prop.current_value,
                    written_value=prop.proposed_value,
                    outcome=outcome,
                )
            prop.feedback = outcome
            HUB.publish("proposal", asdict(prop))
            return prop

    # ------------------------------------------------------------------ #
    def dismiss(self, proposal_id: str) -> None:
        with self._lock:
            self._proposals.pop(proposal_id, None)

    def list_proposals(self) -> list[dict]:
        with self._lock:
            res = [asdict(p) for p in
                   sorted(self._proposals.values(), key=lambda p: -p.created_at)]
            for axis, msg in self._diagnostics.items():
                has_real_proposal = False
                # Substring matching on parameter names (e.g. ROLL, PITCH, YAW) is used to detect the axis.
                # This matches both rate-P gain parameters and rate limit parameters (e.g., MC_ROLLRAUTO_MAX).
                # This suppression logic prevents showing a diagnostic ("fly steps") if an active proposal already exists for that axis.
                for p in self._proposals.values():
                    if p.state in (ProposalState.PRESENTED, ProposalState.WRITTEN):
                        if axis == "roll" and "ROLL" in p.param:
                            has_real_proposal = True
                        elif axis == "pitch" and "PITCH" in p.param:
                            has_real_proposal = True
                        elif axis == "yaw" and "YAW" in p.param:
                            has_real_proposal = True
                
                if not has_real_proposal:
                    res.append({
                        "id": f"diag_{axis}",
                        "param": axis.upper(),
                        "current_value": 0.0,
                        "proposed_value": 0.0,
                        "requested_value": 0.0,
                        "rationale": msg,
                        "airframe_class": "",
                        "state": "diagnostic",
                        "safety_note": "",
                        "created_at": time.time(),
                        "tuning_history": None,
                        "feedback": None,
                        "is_saturation_gain_reduction": False,
                        "confidence": None,
                        "limitations": None,
                        "written_at": None,
                    })
            return res


ADVISOR = ParamAdvisor()
