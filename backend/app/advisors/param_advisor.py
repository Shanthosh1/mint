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


class ParamAdvisor:
    """Registry of pending proposals keyed by id."""

    def __init__(self) -> None:
        self._proposals: dict[str, Proposal] = {}

    # ------------------------------------------------------------------ #
    async def create_proposal(self, param: str, target_value: float,
                              rationale: str) -> Proposal:
        """Validate analyzer advice and stage it for pilot review."""
        airframe = CONNECTION.state.airframe
        if airframe is None:
            raise PermissionError(
                "Airframe not detected — proposals are disabled until "
                "SYS_AUTOSTART has been read from the vehicle."
            )

        current = await CONNECTION.read_param(param)
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
        )
        self._proposals[prop.id] = prop
        HUB.publish("proposal", asdict(prop))
        return prop

    # ------------------------------------------------------------------ #
    async def approve_and_write(self, proposal_id: str) -> Proposal:
        """
        Pilot pressed "Approve & Write".

        Re-validates against the live on-vehicle value before dispatching —
        if anything moved (e.g. pilot changed it in QGC meanwhile), the
        write is blocked and the proposal flips to REJECTED.

        Accepted race: there is a sub-millisecond window between this re-read
        and the write in which another GCS could change the parameter, so the
        write would commit against a just-stale validation. This is tolerated
        rather than fixed — MAVSDK exposes no compare-and-set param API, and
        the safety registry's per-write `max_step` clamp bounds the worst-case
        movement regardless, so a lost race can never produce a catastrophic
        value (only a slightly-off-from-intended one against the new baseline).
        """
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
            prop.state = ProposalState.REJECTED
            prop.safety_note = f"Re-validation failed at write time: {recheck.reason}"
            HUB.publish("proposal", asdict(prop))
            return prop

        prop.state = ProposalState.APPROVED
        try:
            await CONNECTION.write_approved_param(
                prop.param, recheck.allowed,
                as_int=REGISTRY.is_int(prop.airframe_class, prop.param),
            )
            prop.proposed_value = recheck.allowed
            prop.state = ProposalState.WRITTEN
        except Exception as exc:
            prop.state = ProposalState.FAILED
            prop.safety_note = f"Write failed: {exc}"
        HUB.publish("proposal", asdict(prop))
        return prop

    # ------------------------------------------------------------------ #
    async def revert(self, proposal_id: str) -> Proposal:
        """Undo a written change: restore the parameter's pre-write value.

        `current_value` is the live on-vehicle value captured at write time,
        so reverting it rolls back exactly this one change (not any earlier
        ones). Guarded by the same baseline check as a forward write: if the
        live value no longer matches what MINT wrote, the parameter was
        changed by someone else since (engine, pilot, QGC) and the revert is
        blocked — silently writing the old value back would clobber that
        newer change. Restoring a value that was live moments ago is itself
        safe, but the write still goes through the single approved-write path
        so the invariant ("one writer") and logging hold.
        """
        prop = self._proposals.get(proposal_id)
        if prop is None:
            raise KeyError(f"Unknown proposal {proposal_id}")
        if prop.state != ProposalState.WRITTEN:
            raise ValueError(
                f"Proposal {proposal_id} is {prop.state}; only a written "
                f"change can be reverted."
            )

        live = await CONNECTION.read_param(prop.param)
        # Float params: compare with a relative tolerance so storage rounding
        # doesn't read as tampering.
        wrote = prop.proposed_value
        tol = max(1e-6, abs(wrote) * 1e-3)
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
        prop.state = ProposalState.REVERTED
        prop.safety_note = (
            f"Reverted {prop.param} from {wrote:g} back to its pre-write "
            f"value {prop.current_value:g}."
        )
        HUB.publish("proposal", asdict(prop))
        return prop

    # ------------------------------------------------------------------ #
    def record_feedback(self, proposal_id: str, outcome: str) -> Proposal:
        """Pilot's verdict on a written change (better/worse/no_change).

        Only WRITTEN proposals can take feedback — there is no outcome to
        record for a change that was never applied. The outcome is persisted
        to the tuning memory keyed by airframe/param/direction so future
        proposals for the same change can show the track record.
        """
        prop = self._proposals.get(proposal_id)
        if prop is None:
            raise KeyError(f"Unknown proposal {proposal_id}")
        if prop.state != ProposalState.WRITTEN:
            raise ValueError(
                f"Proposal {proposal_id} is {prop.state}; feedback is only "
                f"valid once a change has been written."
            )
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
        self._proposals.pop(proposal_id, None)

    def list_proposals(self) -> list[dict]:
        return [asdict(p) for p in
                sorted(self._proposals.values(), key=lambda p: -p.created_at)]


ADVISOR = ParamAdvisor()
