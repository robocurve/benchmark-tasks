"""Scorer wrappers for pressure_pan_vent_then_lid."""

from __future__ import annotations

import json
import os
from typing import Any, Mapping

import numpy as np

import parts
from geometry import (
    BYPASS_RISE_M,
    FINGERS_OPEN_FRACTION,
    LID_NAME,
    PAN_MAX_SHIFT_M,
    PAN_NAME,
    PLUME_STEPS,
    PLUME_TRIGGER_M,
    STILL_SPEED_M_S,
    TCP_BELOW_LINK_M,
    UNLOCK_HOLD_STEPS,
    UNLOCK_PRESS_M,
    VENT_NAME,
)
from logic import evaluate_state, hands_clear, in_plume_zone, lid_rise, lifted_now, tilt_fault_now
from plume import PLUME

# Facts the end state cannot show; reset at every reset. A fault is permanent for the run.
_RUN = {
    "plume_fault": False,
    "pan_start": None,
    "pan_moved": False,
    "bypass": False,
    "tilt_fault": False,
    "lifted": False,
    "unlocked_steps": 0,
    "lid_raised": False,
    "plume_step": None,  # control steps since the vent first passed 5 mm with the lid on the pan
}


def _hand_tcps(environment) -> dict[str, np.ndarray]:
    scene = environment.unwrapped.scene
    origin = parts._to_numpy(scene.env_origins[0])
    out = {}
    for arm in ("left", "right"):
        robot = scene[f"{arm}_robot"]
        names = list(robot.body_names)
        pos = parts._to_numpy(robot.data.body_pos_w)[0]
        idx = names.index("gripper") if "gripper" in names else len(names) - 1
        out[arm] = np.asarray(pos[idx] - origin, dtype=float) - np.array([0.0, 0.0, TCP_BELOW_LINK_M])
    return out


def _state(environment, name):
    return parts._read_part_state(environment, parts._PARTS[name])


def _on_reset(environment) -> None:
    _RUN.update(
        plume_fault=False, pan_start=None, pan_moved=False, bypass=False, tilt_fault=False, lifted=False,
        unlocked_steps=0, lid_raised=False, plume_step=None,
    )
    PLUME.hide()


def advance(pan, lid, tcps) -> bool:
    """Update the run's latched facts from one control step's states and gripper centres; returns whether the plume is
    shown now. Pure bookkeeping (no simulator calls), so an offline replay can drive it too."""
    press = float(pan.sliders.get(VENT_NAME, 0.0))
    if _RUN["pan_start"] is None:
        _RUN["pan_start"] = np.asarray(pan.pos, dtype=float)
    if float(np.linalg.norm(np.asarray(pan.pos, dtype=float) - _RUN["pan_start"])) > PAN_MAX_SHIFT_M:
        _RUN["pan_moved"] = True
    rise = lid_rise(lid.pos)
    # the plume starts once, when the vent first passes 5 mm with the lid still on the pan, and lasts 5 s
    if _RUN["plume_step"] is None:
        if press >= PLUME_TRIGGER_M and rise <= BYPASS_RISE_M:
            _RUN["plume_step"] = 0
    else:
        _RUN["plume_step"] += 1
    active = _RUN["plume_step"] is not None and _RUN["plume_step"] < PLUME_STEPS
    if active:
        for tcp in tcps:
            if in_plume_zone(tcp):
                _RUN["plume_fault"] = True
    # the lid rising more than 1 cm for the first time while the vent is not pressed 22 mm is a bypass
    if rise > BYPASS_RISE_M and not _RUN["lid_raised"]:
        _RUN["lid_raised"] = True
        if press < UNLOCK_PRESS_M:
            _RUN["bypass"] = True
    if lifted_now(lid.pos, lid.quat_xyzw):
        _RUN["lifted"] = True
    if tilt_fault_now(lid.pos, lid.quat_xyzw, pan.pos[:2]):
        _RUN["tilt_fault"] = True
    if _RUN["unlocked_steps"] < UNLOCK_HOLD_STEPS:
        _RUN["unlocked_steps"] = _RUN["unlocked_steps"] + 1 if press >= UNLOCK_PRESS_M else 0
    return active


def _on_step(environment) -> None:
    try:
        active = advance(_state(environment, PAN_NAME), _state(environment, LID_NAME), list(_hand_tcps(environment).values()))
    except Exception:  # never break an episode for bookkeeping
        return
    PLUME.update(environment, active, int(_RUN["plume_step"] or 0))  # display only; it guards itself


parts.on_reset(_on_reset)
parts.on_step(_on_step)


def _hands_ok(pan_state, lid_state) -> bool:
    environment = parts.env()
    if environment is None:
        return False
    try:
        for arm in ("left", "right"):
            if not parts.gripper_open(environment, arm, FINGERS_OPEN_FRACTION):
                return False
        tcps = list(_hand_tcps(environment).values())
        press = float(pan_state.sliders.get(VENT_NAME, 0.0))
        return hands_clear(tcps, lid_state.pos, lid_state.quat_xyzw, pan_state.pos, press)
    except Exception:
        return False


def _trace(record: dict) -> None:
    path = os.environ.get("TASK_TRACE")
    if path:
        with open(path, "a", encoding="utf-8") as fh:
            fh.write(json.dumps(record) + "\n")


def score(objects: Mapping[str, Any]) -> float:
    lid = objects.get(LID_NAME)
    pan = objects.get(PAN_NAME)
    if lid is None or pan is None:
        return 0.0
    press = float(pan.sliders.get(VENT_NAME, 0.0))
    still = bool(lid.resting(STILL_SPEED_M_S) and pan.resting(STILL_SPEED_M_S))
    s, ok, reason = evaluate_state(
        tuple(float(v) for v in lid.pos),
        tuple(float(v) for v in lid.quat_xyzw),
        press,
        bool(_RUN["plume_fault"]),
        bool(_RUN["pan_moved"]),
        bool(_RUN["bypass"]),
        bool(_RUN["tilt_fault"]),
        bool(_RUN["lifted"]),
        int(_RUN["unlocked_steps"]),
        _hands_ok(pan, lid),
        still,
    )
    _trace(
        {
            "score": s,
            "reason": reason,
            "lid": [round(float(v), 4) for v in lid.pos],
            "vent_mm": round(press * 1000, 1),
            "plume_step": _RUN["plume_step"],
            "faults": [k for k in ("plume_fault", "pan_moved", "bypass", "tilt_fault") if _RUN[k]],
        }
    )
    return s


def success(objects: Mapping[str, Any]) -> bool:
    return score(objects) >= 1.0


def distance(objects: Mapping[str, Any]) -> float:
    return max(0.0, 1.0 - score(objects)) * 0.10
