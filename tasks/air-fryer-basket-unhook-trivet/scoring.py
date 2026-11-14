"""Scorer wrappers and trace logging for air_fryer_basket_release_button_v2."""

from __future__ import annotations

import json
import math
import os
from typing import Any

import numpy as np

from display import DISPLAY
from geometry import (
    BASKET_CLEARED_DRAWER_Z,
    BASKET_LIFT_MIN_M,
    BUTTON_CLEARANCE_DISPLACEMENT_M,
    BYPASS_BAND_BUTTON_M,
    MAT_CENTRE,
    STILL_WINDOW_STEPS,
    TABLE_TOP,
    TCP_BELOW_LINK_M,
)
from logic import (
    evaluate_state,
    hot_contact,
    on_mat,
    released,
    still,
)
from mechkit import on_reset

# Latched run state across control steps
_RUN = {
    "hot_contact_ever": False,
    "cleared_drawer_ever": False,
    "unhook_lift_ever": False,
    "button_used": False,
    "band_checked": False,
    "hand_fail": False,
    "bypass_ever": False,
    "dropped_ever": False,
    "z_hist": [],
    "basket_start_z": None,
    "hist": [],
    "step": 0,
}


def _reset_run() -> None:
    _RUN["hot_contact_ever"] = False
    _RUN["cleared_drawer_ever"] = False
    _RUN["unhook_lift_ever"] = False
    _RUN["button_used"] = False
    _RUN["band_checked"] = False
    _RUN["hand_fail"] = False
    _RUN["bypass_ever"] = False
    _RUN["dropped_ever"] = False
    _RUN["z_hist"] = []
    _RUN["basket_start_z"] = None
    _RUN["hist"] = []
    _RUN["step"] = 0



def _org(st):
    """The Mech's body origin (MechState.pos is the bounding-box centre, which moves with the sliding parts)."""
    o = getattr(st, "origin", None)
    return st.pos if o is None or not np.any(np.asarray(o, dtype=float)) else o  # unset (zero) origin: fall back to pos

_TILT = {"left": None, "right": None}  # hand tilt from vertical (deg), logged with the pads
_ENV = [None]  # the simulator environment, captured at reset (score() is called without it)


@on_reset
def _on_reset_hook(environment) -> None:
    _ENV[0] = environment
    _reset_run()
    DISPLAY.reset(environment)


def _to_numpy(val: Any) -> np.ndarray:
    if hasattr(val, "detach"):
        val = val.detach()
    if hasattr(val, "cpu"):
        val = val.cpu()
    if hasattr(val, "numpy"):
        val = val.numpy()
    return np.asarray(val, dtype=np.float64)


def advance(
    basket_pos: Sequence[float],
    basket_quat: Sequence[float],
    button_disp: float,
    tcps: Sequence[Sequence[float]],
    pads: Sequence[Sequence[float]],
    drawer_slide: float = 0.0,
) -> None:
    """Update latched facts from a single step."""
    _RUN["step"] += 1
    bz = float(basket_pos[2])
    if _RUN["basket_start_z"] is None:
        _RUN["basket_start_z"] = bz

    # Check hot contact with both fingertip bodies against every hot box
    if pads and hot_contact(pads, basket_pos, basket_quat, drawer_slide):
        _RUN["hot_contact_ever"] = True

    # Cleared drawer
    if bz >= BASKET_CLEARED_DRAWER_Z:
        _RUN["cleared_drawer_ever"] = True

    # Unhook lift: button pressed >= 8 mm and lifted >= 20 mm
    if button_disp >= BUTTON_CLEARANCE_DISPLACEMENT_M and bz >= _RUN["basket_start_z"] + BASKET_LIFT_MIN_M:
        _RUN["unhook_lift_ever"] = True

    # Latch bypass: the catches only let the basket past the ledges (a lift of 8 to 14 mm) while the release button is held
    # in at least 7 mm AT THAT MOMENT. A basket that passes the ledges with the button out (shoved back, or released after a
    # tap) is a permanent zero.
    if button_disp >= BUTTON_CLEARANCE_DISPLACEMENT_M:
        _RUN["button_used"] = True
    lift = bz - _RUN["basket_start_z"]
    if lift >= 0.008 and not _RUN["band_checked"]:
        _RUN["band_checked"] = True
        if button_disp < BYPASS_BAND_BUTTON_M:
            _RUN["bypass_ever"] = True
    if 0.008 <= lift <= 0.014 and button_disp < BYPASS_BAND_BUTTON_M:
        _RUN["bypass_ever"] = True

    # Dropped: the basket fell more than 5 cm within 5 steps (0.25 s)
    _RUN["z_hist"].append(bz)
    if len(_RUN["z_hist"]) > 5:
        _RUN["z_hist"] = _RUN["z_hist"][-6:]
        if _RUN["z_hist"][0] - min(_RUN["z_hist"][1:]) > 0.05:
            _RUN["dropped_ever"] = True

    # History for stillness check
    _RUN["hist"].append(list(map(float, basket_pos)))
    if len(_RUN["hist"]) > STILL_WINDOW_STEPS * 2:
        _RUN["hist"] = _RUN["hist"][-STILL_WINDOW_STEPS:]


def _trace(record: dict) -> None:
    path = os.environ.get("TASK_TRACE")
    if not path:
        return
    # Convert any numpy types to plain python floats/ints/bools
    clean = {}
    for k, v in record.items():
        if isinstance(v, (np.floating, float)):
            clean[k] = round(float(v), 5)
        elif isinstance(v, (np.integer, int)):
            clean[k] = int(v)
        elif isinstance(v, (np.bool_, bool)):
            clean[k] = bool(v)
        elif isinstance(v, (list, tuple)):
            clean[k] = [round(float(x), 5) if isinstance(x, (float, np.floating)) else x for x in v]
        else:
            clean[k] = v
    with open(path, "a", encoding="utf-8") as fh:
        fh.write(json.dumps(clean) + "\n")



def _hand_data2(environment):
    """Both grippers' tool point (TCP) and every finger pad box centre (eight per hand).

    TCP = gripper body position + its rotation applied to the 0.134 m tool offset (the yam_arena TCP_OFFSET, as in the
    accepted tasks), so a tilted hand is placed correctly. Pads: on the gripper x axis (the closing axis) at the measured
    finger opening (joint7/joint8) plus 6 mm, for each finger four 12 mm boxes along the finger: centred at the TCP, 7 mm
    below it (lower face at the fingertip), and 3 cm and 6 cm above it (the finger root and the base of the gripper).
    A failure to read the hands is latched and scores 0 (it never silently skips the hot-surface check).
    """
    if environment is None:
        return [], []
    try:
        scene = environment.unwrapped.scene
        origin = _to_numpy(scene.env_origins[0])
        tcps, pads = [], []
        for arm in ("left", "right"):
            robot = scene[arm + "_robot"]
            names = list(robot.body_names)
            bp = _to_numpy(robot.data.body_pos_w)[0]
            idx = names.index("gripper") if "gripper" in names else len(names) - 1
            x, y, z, w = (float(v) for v in _to_numpy(robot.data.body_quat_w)[0][idx])  # (x, y, z, w)
            n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
            x, y, z, w = x / n, y / n, z / n, w / n
            rot = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
            tcp = np.asarray(bp[idx] - origin, dtype=float) + rot @ np.array([0.0, 0.0, TCP_BELOW_LINK_M])
            tcps.append(tcp)
            q = [0.02, 0.02]
            try:
                jn = list(robot.joint_names)
                jp = _to_numpy(robot.data.joint_pos)[0]
                q = [abs(float(jp[jn.index(j)])) if j in jn else 0.02 for j in ("joint7", "joint8")]
            except Exception:
                pass
            half_gap = max(q)
            # As in the accepted tasks: the fingers close along the gripper's own x axis. Four 12 mm boxes per finger (see above).
            for along in (0.0, 0.007, -0.030, -0.060):
                centre = tcp + rot @ np.array([0.0, 0.0, along])
                for side in (1.0, -1.0):
                    pads.append(centre + rot[:, 0] * side * (half_gap + 0.006))
            _TILT[arm] = round(math.degrees(math.acos(max(-1.0, min(1.0, -float(rot[2, 2]))))), 1)
        return tcps, pads
    except Exception:
        _RUN["hand_fail"] = True
        return [], []


def score(objects: dict[str, Any], environment: Any = None) -> float:
    """Score the task state."""
    basket_st = objects.get("af2_basket")
    housing_st = objects.get("af2_housing")

    if basket_st is None:
        return 0.0

    b_pos = list(_org(basket_st))
    b_quat = list(basket_st.quat_xyzw)
    button_disp = float(basket_st.joints.get("button", 0.0)) if hasattr(basket_st, "joints") else 0.0

    drawer_slide = 0.0
    if housing_st and hasattr(housing_st, "joints"):
        drawer_slide = float(housing_st.joints.get("drawer", 0.0))

    tcps, pads = _hand_data2(environment if environment is not None else _ENV[0])
    try:
        _tp = os.environ.get("TASK_TRACE")
        if _tp and pads:
            with open(_tp + ".pads.jsonl", "a", encoding="utf-8") as _fh:
                _fh.write(json.dumps({"p": [[round(float(v), 3) for v in q] for q in pads], "t": [[round(float(v), 3) for v in q] for q in tcps], "tilt": dict(_TILT)}) + "\n")
    except Exception:
        pass
    advance(b_pos, b_quat, button_disp, tcps, pads, drawer_slide)

    rel = released(list(tcps) + list(pads), b_pos, b_quat) if tcps else False   # both gripper centres and every finger pad box
    stl = still(_RUN["hist"])

    s, reason = evaluate_state(
        b_pos,
        b_quat,
        button_disp,
        _RUN["hot_contact_ever"],
        _RUN["cleared_drawer_ever"],
        _RUN["unhook_lift_ever"],
        rel,
        stl,
        _RUN["bypass_ever"],
        _RUN["dropped_ever"],
    )
    if _RUN["hand_fail"]:
        s, reason = 0.0, "hand poses could not be read (scores 0)"

    _trace({
        "step": int(_RUN["step"]),
        "score": float(s),
        "reason": reason,
        "basket_z": round(float(b_pos[2]), 4), "basket": [round(float(v), 4) for v in b_pos],
        "button_disp": round(float(button_disp), 4), "basket_tilt": round(float(__import__("logic").tilt_deg(b_quat)), 1),
        "hot_contact": bool(_RUN["hot_contact_ever"]),
        "on_mat": bool(on_mat(b_pos, b_quat)),
        "released": bool(rel),
        "still": bool(stl),
    })

    return s


def success(objects: dict[str, Any], environment: Any = None) -> bool:
    return score(objects, environment) >= 1.0


def distance(objects: dict[str, Any]) -> float:
    basket_st = objects.get("af2_basket")
    if basket_st is None:
        return 1.0
    dx = float(_org(basket_st)[0]) - MAT_CENTRE[0]
    dy = float(_org(basket_st)[1]) - MAT_CENTRE[1]
    return float(math.hypot(dx, dy))
