from __future__ import annotations

"""Scoring wrappers and TASK_TRACE logging for keyboard_tray_latch (T22)."""

import json
import os
from typing import Any, Mapping

from geometry import DESK_NAME
from logic import evaluate_state

_TRACE_STEP = [0]


def _trace(d_pos, tr, lat, s, ok, reason) -> None:
    path = os.environ.get("TASK_TRACE")
    if not path:
        return
    _TRACE_STEP[0] += 1
    rec = {
        "step": _TRACE_STEP[0],
        "desk_pos": [round(v, 4) for v in d_pos],
        "tray_slide": round(tr, 4),
        "latched": bool(lat),
        "score": s,
        "success": ok,
        "reason": reason,
    }
    with open(path, "a", encoding="utf-8") as f:
        f.write(json.dumps(rec) + chr(10))


def _extract(objects: Mapping[str, Any]):
    desk = objects[DESK_NAME]
    d_pos = tuple(float(v) for v in desk.pos)
    sliders = getattr(desk, "sliders", {})
    tr = abs(float(sliders.get("tray", 0.0)))
    lat = abs(float(sliders.get("latch", 0.0))) >= 0.006 or tr >= 0.278
    resting = getattr(desk, "resting", lambda: True)()
    return d_pos, tr, lat, resting


def score(objects: Mapping[str, Any]) -> float:
    d_pos, tr, lat, resting = _extract(objects)
    s, ok, reason = evaluate_state(d_pos, tr, lat, resting=resting)
    _trace(d_pos, tr, lat, s, ok, reason)
    return s


def success(objects: Mapping[str, Any]) -> bool:
    d_pos, tr, lat, resting = _extract(objects)
    _, ok, _ = evaluate_state(d_pos, tr, lat, resting=resting)
    return ok

def distance(objects) -> float:
    return max(0.0, 1.0 - score(objects)) * 0.05
