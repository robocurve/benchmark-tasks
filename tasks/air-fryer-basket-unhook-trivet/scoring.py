"""Scorer wrappers and trace logging for the air fryer fry-basket task."""

from __future__ import annotations

import json
import math
import os
from typing import Any

import numpy as np

from display import DISPLAY
from geometry import (
    BASKET_CLEARED_PAN_M,
    BASKET_LIFT_MIN_M,
    GUARD_OPEN_M,
    LEVER_RELEASE_RAD,
    MAT_CENTRE,
    PAN_LIFTED_M,
    PAN_START_POS,
    STILL_WINDOW_STEPS,
    UNLATCHED_LIFT_M,
)
import logic
from logic import (
    basket_rise_over_pan,
    evaluate_state,
    hot_contact,
    on_mat,
    over_pan,
    pan_home,
    released,
    still,
)
from mechkit import on_reset

FACTS = ("hot_contact_ever", "released_in_air_ever", "unlatched_lift_ever", "dropped_ever", "cleared_pan_ever",
         "unhook_lift_ever", "lever_used", "guard_opened", "hand_fail")

# Latched run state across control steps
_RUN: dict[str, Any] = {}


def _reset_run() -> None:
    _RUN.clear()
    _RUN.update({k: False for k in FACTS})
    _RUN.update({"z_hist": [], "pan_z_hist": [], "hist": [], "step": 0})


_reset_run()


def _org(st):
    """The Mech's body origin (MechState.pos is the bounding-box centre, which moves with the sliding parts)."""
    o = getattr(st, "origin", None)
    return st.pos if o is None or not np.any(np.asarray(o, dtype=float)) else o  # unset (zero) origin: fall back to pos


_TILT = {"left": None, "right": None}  # hand tilt from vertical (deg), logged with the pads
_RAW = {"left": None, "right": None}  # raw gripper quaternion (x, y, z, w as Isaac Lab 3 reports it) and body position, logged with the pads
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


def _fell(hist: list, z: float) -> bool:
    """Append z; True if it fell more than 5 cm within the last 5 steps (0.25 s)."""
    hist.append(z)
    if len(hist) > 6:
        del hist[:-6]
    return len(hist) == 6 and hist[0] - min(hist[1:]) > 0.05


def advance(
    basket_pos, basket_quat, pan_pos, pan_quat, lever_angle: float, guard_slide: float, pads,
) -> None:
    """Update the latched facts from one step."""
    _RUN["step"] += 1
    rise = basket_rise_over_pan(basket_pos, pan_pos)
    pressed = lever_angle >= LEVER_RELEASE_RAD
    pan_up = float(pan_pos[2]) - PAN_START_POS[2]

    if pads and hot_contact(pads, basket_pos, basket_quat, pan_pos, pan_quat):
        _RUN["hot_contact_ever"] = True
    if guard_slide >= GUARD_OPEN_M:
        _RUN["guard_opened"] = True
    if pressed:
        _RUN["lever_used"] = True
    # The manual's caution: the release is pressed only with both baskets resting on the heat-resistant surface. Pressing
    # it with the outer basket lifted 1 cm or more off the trivet (carried by the latched handle) drops the hot outer basket.
    if pressed and pan_up >= PAN_LIFTED_M:
        _RUN["released_in_air_ever"] = True
    # The latch holds the fry basket in the outer basket until the lever is pressed: a fry basket that rises 12 mm out of
    # the outer basket (still over it) with the lever never pressed has defeated the latch.
    # (Only while the outer basket is still on its trivet: a latched stack carried together tilts both baskets.)
    if not _RUN["lever_used"] and rise >= UNLATCHED_LIFT_M and over_pan(basket_pos, pan_pos) and pan_up < PAN_LIFTED_M:
        _RUN["unlatched_lift_ever"] = True
    if rise >= BASKET_CLEARED_PAN_M:
        _RUN["cleared_pan_ever"] = True
    if pressed and rise >= BASKET_LIFT_MIN_M:
        _RUN["unhook_lift_ever"] = True
    if _fell(_RUN["z_hist"], float(basket_pos[2])) or _fell(_RUN["pan_z_hist"], float(pan_pos[2])):
        _RUN["dropped_ever"] = True

    _RUN["hist"].append(list(map(float, basket_pos)))
    if len(_RUN["hist"]) > STILL_WINDOW_STEPS * 2:
        _RUN["hist"] = _RUN["hist"][-STILL_WINDOW_STEPS:]


def _trace(record: dict) -> None:
    path = os.environ.get("TASK_TRACE")
    if not path:
        return
    clean = {}
    for k, v in record.items():
        if isinstance(v, (np.floating, float)):
            clean[k] = round(float(v), 5)
        elif isinstance(v, (np.bool_, bool)):
            clean[k] = bool(v)
        elif isinstance(v, (np.integer, int)):
            clean[k] = int(v)
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
            _RAW[arm] = {"q": [round(x, 5), round(y, 5), round(z, 5), round(w, 5)], "bp": [round(float(v), 4) for v in (bp[idx] - origin)]}
            body_rel = np.asarray(bp[idx] - origin, dtype=float)
            q = [0.02, 0.02]
            try:
                jn = list(robot.joint_names)
                jp = _to_numpy(robot.data.joint_pos)[0]
                q = [abs(float(jp[jn.index(j)])) if j in jn else 0.02 for j in ("joint7", "joint8")]
            except Exception:
                pass
            # Ground truth for the quaternion order (logged only, never scored): the simulator's own tip_left/tip_right link
            # positions against those predicted from the gripper body pose read as (x, y, z, w) and as (w, x, y, z)
            try:
                q7, q8 = (float(jp[jn.index(j)]) for j in ("joint7", "joint8"))
                meas = [np.asarray(bp[names.index(t)] - origin, dtype=float) for t in ("tip_left", "tip_right")]
                err = {}
                for order, qq in (("xyzw", (x, y, z, w)), ("wxyz", (y, z, w, x))):
                    pred = logic.tip_link_positions(body_rel, qq, q7, q8)
                    err[order] = round(1000.0 * max(float(np.linalg.norm(p - m)) for p, m in zip(pred, meas)), 2)
                _RAW[arm]["tips"] = [[round(float(v), 4) for v in m] for m in meas]
                _RAW[arm]["q78"] = [round(q7, 5), round(q8, 5)]
                _RAW[arm]["tip_err_mm"] = err
            except Exception:
                pass
            half_gap = max(q)
            # As in the accepted tasks: the fingers close along the gripper's own x axis. Four 12 mm boxes per finger (logic.hand_pose_boxes).
            tcp, hand_pads, tilt = logic.hand_pose_boxes(body_rel, (x, y, z, w), half_gap)
            tcps.append(tcp)
            pads.extend(hand_pads)
            _TILT[arm] = round(tilt, 1)
        return tcps, pads
    except Exception:
        _RUN["hand_fail"] = True
        return [], []


def score(objects: dict[str, Any], environment: Any = None) -> float:
    """Score the task state."""
    basket_st = objects.get("af3_basket")
    pan_st = objects.get("af3_pan")
    if basket_st is None or pan_st is None:
        return 0.0

    b_pos, b_quat = list(_org(basket_st)), list(basket_st.quat_xyzw)
    p_pos, p_quat = list(_org(pan_st)), list(pan_st.quat_xyzw)
    joints = getattr(basket_st, "joints", {}) or {}
    lever = float(joints.get("lever", 0.0))
    guard = float(joints.get("guard", 0.0))

    tcps, pads = _hand_data2(environment if environment is not None else _ENV[0])
    try:
        _tp = os.environ.get("TASK_TRACE")
        if _tp and pads:
            with open(_tp + ".pads.jsonl", "a", encoding="utf-8") as _fh:
                _fh.write(json.dumps({"p": [[round(float(v), 3) for v in q] for q in pads], "t": [[round(float(v), 3) for v in q] for q in tcps], "tilt": dict(_TILT), "raw": dict(_RAW)}) + "\n")
    except Exception:
        pass
    advance(b_pos, b_quat, p_pos, p_quat, lever, guard, pads)

    rel = released(list(tcps) + list(pads), b_pos, b_quat, lever, guard) if tcps else False
    stl = still(_RUN["hist"])
    s, reason = evaluate_state(b_pos, b_quat, p_pos, p_quat, lever, _RUN, rel, stl)
    if _RUN["hand_fail"]:
        s, reason = 0.0, "hand poses could not be read (scores 0)"

    _trace({
        "step": int(_RUN["step"]),
        "score": float(s),
        "reason": reason,
        "basket": [round(float(v), 4) for v in b_pos],
        "pan": [round(float(v), 4) for v in p_pos],
        "lever": round(lever, 4),
        "guard": round(guard, 4),
        "rise": round(basket_rise_over_pan(b_pos, p_pos), 4),
        "basket_tilt": round(float(logic.tilt_deg(b_quat)), 1),
        "pan_tilt": round(float(logic.tilt_deg(p_quat)), 1),
        "hot_contact": bool(_RUN["hot_contact_ever"]),
        "on_mat": bool(on_mat(b_pos, b_quat)),
        "pan_home": bool(pan_home(p_pos, p_quat)),
        "released": bool(rel),
        "still": bool(stl),
        "facts": [k for k in FACTS if _RUN[k]],
    })
    return s


def success(objects: dict[str, Any], environment: Any = None) -> bool:
    return score(objects, environment) >= 1.0


def distance(objects: dict[str, Any]) -> float:
    basket_st = objects.get("af3_basket")
    if basket_st is None:
        return 1.0
    o = _org(basket_st)
    return float(math.hypot(float(o[0]) - MAT_CENTRE[0], float(o[1]) - MAT_CENTRE[1]))
