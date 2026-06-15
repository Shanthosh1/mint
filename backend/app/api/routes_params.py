"""
Parameter advice + Human-In-The-Loop approval endpoints.

There is deliberately NO endpoint that writes an arbitrary param/value
pair. The only write path is approving a staged proposal that has
passed the safety registry twice.
"""
from __future__ import annotations

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field, model_validator

from ..advisors.param_advisor import ADVISOR
from ..analysis.stick_monitor import STICK_MONITOR
from ..core.safety_registry import REGISTRY
from ..mavlink.connection import CONNECTION

router = APIRouter(prefix="/api/params", tags=["params"])


class ProposalRequest(BaseModel):
    """Exactly one change form: absolute target, multiplicative scale,
    or additive delta. Relative forms are resolved against the live
    on-vehicle value at staging time (live engines emit relative advice
    because they never track current parameter values themselves)."""
    param: str = Field(..., examples=["MC_ROLLRATE_P"])
    target_value: float | None = None
    scale_factor: float | None = Field(None, gt=0, le=3.0)
    delta: float | None = None
    rationale: str = Field(default="Manually requested by pilot")
    is_saturation_gain_reduction: bool = False
    confidence: str | None = None
    limitations: str | None = None

    @model_validator(mode="after")
    def _exactly_one_form(self):
        forms = [v is not None for v in (self.target_value, self.scale_factor, self.delta)]
        if sum(forms) != 1:
            raise ValueError("Provide exactly one of target_value / scale_factor / delta")
        return self


class TuningWindowRequest(BaseModel):
    axis: str = Field(..., pattern="^(roll|pitch|yaw)$")
    loop: str = Field(..., pattern="^(rate|attitude|velocity|position)$")


class FeedbackRequest(BaseModel):
    outcome: str = Field(..., pattern="^(better|worse|no_change|skip|no_feedback)$",
                         examples=["better"])


@router.get("/safety-registry")
def safety_registry() -> dict:
    """Expose the (read-only) bounds so the UI can render limit hints."""
    return {cls: REGISTRY.params_for(cls) for cls in REGISTRY.airframe_classes()}


@router.get("/proposals")
def list_proposals() -> list[dict]:
    return ADVISOR.list_proposals()


@router.post("/proposals")
async def create_proposal(req: ProposalRequest) -> dict:
    """Stage a validated proposal for pilot review (never writes)."""
    try:
        if req.target_value is not None:
            target = req.target_value
        else:
            current = await CONNECTION.read_param(req.param)
            target = (current * req.scale_factor if req.scale_factor is not None
                      else current + req.delta)
        prop = await ADVISOR.create_proposal(
            req.param, round(target, 6), req.rationale,
            is_saturation_gain_reduction=req.is_saturation_gain_reduction,
            confidence=req.confidence,
            limitations=req.limitations
        )
    except PermissionError as exc:
        raise HTTPException(409, str(exc))
    except ConnectionError as exc:
        raise HTTPException(503, str(exc))
    except Exception as exc:
        raise HTTPException(422, f"Could not resolve {req.param}: {exc}")
    return {"id": prop.id, "state": prop.state, "proposed_value": prop.proposed_value,
            "safety_note": prop.safety_note}


@router.post("/proposals/{proposal_id}/approve")
async def approve_proposal(proposal_id: str) -> dict:
    """The explicit 'Approve & Write' action — the sole write path."""
    try:
        prop = await ADVISOR.approve_and_write(proposal_id)
    except KeyError as exc:
        raise HTTPException(404, str(exc))
    except ValueError as exc:
        raise HTTPException(409, str(exc))
    except ConnectionError as exc:
        raise HTTPException(503, str(exc))
    return {"id": prop.id, "state": prop.state, "written_value": prop.proposed_value,
            "safety_note": prop.safety_note}


@router.post("/proposals/{proposal_id}/revert")
async def revert_proposal(proposal_id: str) -> dict:
    """Undo a written change — restore the parameter's pre-write value.

    Blocked if the live value no longer matches what MINT wrote (something
    changed it since). The only other write path besides Approve & Write.
    """
    try:
        prop = await ADVISOR.revert(proposal_id)
    except KeyError as exc:
        raise HTTPException(404, str(exc))
    except ValueError as exc:
        raise HTTPException(409, str(exc))
    except ConnectionError as exc:
        raise HTTPException(503, str(exc))
    return {"id": prop.id, "state": prop.state, "reverted_to": prop.current_value,
            "safety_note": prop.safety_note}


@router.post("/proposals/{proposal_id}/feedback")
def proposal_feedback(proposal_id: str, req: FeedbackRequest) -> dict:
    """Record the pilot's verdict on a written change (better/worse/no_change).

    Persists to the tuning memory so future proposals for the same airframe/
    param/direction can show the track record. Advisory only — never alters
    the safety bounds or auto-applies anything.
    """
    try:
        prop = ADVISOR.record_feedback(proposal_id, req.outcome)
    except KeyError as exc:
        raise HTTPException(404, str(exc))
    except ValueError as exc:
        raise HTTPException(409, str(exc))
    return {"id": prop.id, "feedback": prop.feedback}


@router.delete("/proposals/{proposal_id}")
def dismiss_proposal(proposal_id: str) -> dict:
    ADVISOR.dismiss(proposal_id)
    return {"dismissed": proposal_id}


@router.get("/{name}")
async def read_param(name: str) -> dict:
    try:
        value = await CONNECTION.read_param(name)
    except ConnectionError as exc:
        raise HTTPException(503, str(exc))
    except Exception as exc:
        raise HTTPException(404, f"Parameter {name} unavailable: {exc}")
    return {"param": name, "value": value}


@router.post("/tuning-window/start")
def start_tuning_window(req: TuningWindowRequest) -> dict:
    """Open a dynamic-testing window; the stick monitor begins watching."""
    STICK_MONITOR.begin_window(req.axis, req.loop)
    return {"tuning_axis": req.axis}


@router.post("/tuning-window/stop")
def stop_tuning_window() -> dict:
    STICK_MONITOR.end_window()
    return {"tuning_axis": None}
