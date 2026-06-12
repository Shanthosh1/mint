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


class ProposalState(str, Enum):
    PRESENTED = "presented"
    APPROVED = "approved"
    WRITTEN = "written"
    REJECTED = "rejected"
    FAILED = "write_failed"


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

        prop = Proposal(
            id=uuid.uuid4().hex[:12],
            param=param,
            current_value=round(current, 6),
            proposed_value=check.allowed if check.allowed is not None else current,
            requested_value=target_value,
            rationale=rationale,
            airframe_class=airframe.airframe_class,
            state=(ProposalState.REJECTED
                   if check.verdict in (Verdict.REJECTED_UNKNOWN, Verdict.REJECTED_BOUNDS)
                   else ProposalState.PRESENTED),
            safety_note=check.reason,
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
    def dismiss(self, proposal_id: str) -> None:
        self._proposals.pop(proposal_id, None)

    def list_proposals(self) -> list[dict]:
        return [asdict(p) for p in
                sorted(self._proposals.values(), key=lambda p: -p.created_at)]


ADVISOR = ParamAdvisor()
