"""
Post-flight NPFG (Nonlinear Guidance Law) configuration analysis.
"""
from __future__ import annotations

import pandas as pd


def analyze_npfg(params: dict, airframe_class: str | None) -> dict:
    """Check NPFG configuration parameters."""
    if airframe_class not in ("FIXED_WING", "DELTA_WING", "VTOL"):
        return {"skipped": "NPFG analysis is only applicable to Fixed-Wing and VTOL vehicles."}

    # Check NPFG_EN_AUTO_TUNING (NPFG auto-tuning) parameter
    npfg_en = params.get("NPFG_EN_AUTO_TUNING")

    recommendations = []
    notes = []

    if npfg_en is not None:
        val = int(float(npfg_en))
        if val == 0:
            recommendations.append({
                "param": "NPFG_EN_AUTO_TUNING",
                "proposed_value": 1,
                "rationale": "NPFG auto-tuning is currently disabled. Enabling it (setting to 1) allows the guidance controller to dynamically adjust period/damping for optimal path tracking under varying wind conditions.",
                "confidence": {
                    "score": 1.0,
                    "flags": {}
                }
            })
    else:
        notes.append("NPFG_EN_AUTO_TUNING parameter not found in the log.")

    return {
        "recommendations": recommendations,
        "notes": notes
    }
