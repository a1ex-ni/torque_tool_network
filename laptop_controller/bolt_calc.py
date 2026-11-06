"""
bolt_calc.py: target tension and torque from bolt_profiles.json (API 6A Tables H.1 / H.2),
for the chosen bolt stress and friction factor.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Dict, List, Optional

BOLT_PROFILES_PATH = Path(__file__).resolve().parent / "bolt_profiles.json"

N_PER_LBF = 4.4482216
NM_PER_FTLBF = 1.3558179


def load_bolt_profiles() -> dict:
    with BOLT_PROFILES_PATH.open(encoding="utf-8") as f:
        return json.load(f)


def _stress_caps(flange_id: str, bp: dict) -> Dict[str, float]:
    """Stress caps from H.3.3."""
    by_id = {s["id"]: s for s in bp["stress_options"]}
    caps: Dict[str, float] = {}
    for o in bp.get("flange_overrides", []):
        if o["flange"] == flange_id:
            for sid in o["max_stress"]:
                s = by_id[sid]
                caps[s["system"]] = min(caps.get(s["system"], s["stress"]), s["stress"])
    return caps


def bolt_setup(flange: dict, bp: dict) -> dict:
    """Stress / friction choices for a flange."""
    bolt_id = flange.get("bolt", "")
    bolt = bp["bolts"].get(bolt_id)
    if bolt is None:
        return {"bolt": bolt_id, "error": f"Unknown bolt '{bolt_id}' in bolt_profiles.json", "options": []}

    if "custom" in bolt:
        c = bolt["custom"]
        return {"bolt": bolt_id, "custom": True, "description": bolt.get("description", ""),
                "target_n": c["target_n"], "max_n": c["max_n"], "options": [], "friction_factors": []}

    caps = _stress_caps(flange["id"], bp)
    over = 1 + bp["settings"]["max_over_target_pct"] / 100
    options: List[dict] = []
    for s in bp["stress_options"]:
        r = bolt["ratings"].get(s["id"])
        if r is None or s["stress"] > caps.get(s["system"], float("inf")):
            continue
        if s["system"] == "metric":
            tension_n = r["tension_kN"] * 1000
            torque_native, unit_native = r["torque_Nm"], "N·m"
            torque_nm = dict(torque_native)
            tension_native, tension_unit = r["tension_kN"], "kN"
        else:
            tension_n = r["tension_lbf"] * N_PER_LBF
            torque_native, unit_native = r["torque_ftlbf"], "lbf·ft"
            torque_nm = {f: v * NM_PER_FTLBF for f, v in torque_native.items()}
            tension_native, tension_unit = r["tension_lbf"], "lbf"
        options.append({
            "stress_id": s["id"], "label": s["label"], "note": s["note"], "system": s["system"],
            "tension_n": round(tension_n, 1), "max_n": round(tension_n * over, 1),
            "tension_native": tension_native, "tension_unit": tension_unit,
            "torque_nm": torque_nm, "torque_native": torque_native, "torque_unit": unit_native,
        })
    return {"bolt": bolt_id, "custom": False, "diameter_in": bolt["diameter_in"], "tpi": bolt["tpi"],
            "diameter_mm": bolt.get("diameter_mm"), "pitch_mm": bolt.get("pitch_mm"),
            "capped": bool(caps), "options": options,
            "friction_factors": [str(f) for f in bp["friction_factors"]]}


def resolve(flange: dict, bp: dict, stress_id: Optional[str], friction: Optional[str]) -> dict:
    """Targets for one choice. Raises ValueError."""
    setup = bolt_setup(flange, bp)
    if setup.get("error"):
        raise ValueError(setup["error"])
    if setup["custom"]:
        return {"bolt": setup["bolt"], "method": "custom", "target_n": setup["target_n"],
                "max_n": setup["max_n"], "label": "Fixed bench target"}

    opt = next((o for o in setup["options"] if o["stress_id"] == stress_id), None)
    if opt is None:
        raise ValueError("Choose an allowed bolt stress for this flange.")
    f = str(friction)
    if f not in setup["friction_factors"]:
        raise ValueError("Choose a friction factor.")
    return {
        "bolt": setup["bolt"], "method": "table", "stress_id": opt["stress_id"],
        "label": f"{opt['label']}, f = {f}", "system": opt["system"], "friction": f,
        "target_n": opt["tension_n"], "max_n": opt["max_n"],
        "tension_native": opt["tension_native"], "tension_unit": opt["tension_unit"],
        "torque_nm": opt["torque_nm"][f], "torque_native": opt["torque_native"][f],
        "torque_unit": opt["torque_unit"],
    }
