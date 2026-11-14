"""Scorer wrappers for receipt_printer_roll_reload_v2."""

from __future__ import annotations

import json
import math
import os
from typing import Any, Mapping

import numpy as np

from display import DISPLAY
from geometry import (
    BAY_OPEN_DEG,
    COVER_NAME,
    COVER_SHAPES,
    FINGERS_OPEN_FRACTION,
    FIXTURE_MOVE_M,
    HINGE_AT,
    PRINTER_NAME,
    PRINTER_SHAPES,
    PRINTER_XY,
    RELEASE_MIN_DIST_M,
    ROLL_CENTRE_Z,
    ROLL_CORE_WIDTH_M,
    ROLL_NAME,
    ROLL_RADIUS_M,
    ROLL_START_ORIGIN_Z,
    ROLL_TAIL,
    STAND_NAME,
    STILL_MOTION_M,
    STILL_SPEED_M_S,
    STILL_WINDOW_STEPS,
    TABLE_TOP,
    TCP_BELOW_LINK_M,
)
from logic import evaluate_state, lamp_state, lid_open_deg, rotation_from_quat
from mechkit import _axis_matrix, _to_numpy, on_reset

# Facts the end state cannot show; reset at every reset.
_RUN = {
    "fixture_start": None,   # printer and stand positions at first step
    "fixture_moved": False,
    "open_run": 0,           # control steps in a row with lid open >= 50 degrees
    "open_run_max": 0,       # longest such run so far
    "max_lift": 0.0,         # highest roll origin has been above start height
    "hist": [],              # (roll position, lid joint val) of the last STILL_WINDOW_STEPS steps
    "step": 0,
}

_EYE = np.eye(3)
_CURRENT_ENV: list[Any] = [None]
_FINGER_OPEN_POS = -0.04695
_FINGER_JOINTS = ("joint7", "joint8")
_TILT = {"left": None, "right": None}


def _hand_data2(environment):
    """Both grippers' tool point (TCP) and BOTH fingertip pad centres.

    TCP = gripper body position + its rotation applied to the 0.134 m tool offset (the yam_arena TCP_OFFSET, as in the
    accepted tasks), so a tilted hand is placed correctly. Pads: on the gripper x axis (the closing axis) at the measured
    finger opening (joint7/joint8) plus 6 mm, centred at the TCP and 7 mm lower (lower face at the fingertip): four per hand.
    """
    if environment is None:
        return [], []
    try:
        scene = environment.unwrapped.scene
        origin = _to_numpy(scene.env_origins)[0]
        tcps, pads = [], []
        for arm in ("left", "right"):
            robot = scene[arm + "_robot"]
            names = list(robot.body_names)
            bp = _to_numpy(robot.data.body_pos_w)[0]
            idx = names.index("gripper") if "gripper" in names else len(names) - 1
            x, y, z, w = (float(v) for v in _to_numpy(robot.data.body_quat_w)[0][idx])
            n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
            x, y, z, w = x / n, y / n, z / n, w / n
            rot = np.array(
                [
                    [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                    [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                    [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
                ]
            )
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
            for along in (0.0, 0.007):
                centre = tcp + rot @ np.array([0.0, 0.0, along])
                for side in (1.0, -1.0):
                    pads.append(centre + rot[:, 0] * side * (half_gap + 0.006))
            _TILT[arm] = round(math.degrees(math.acos(max(-1.0, min(1.0, -float(rot[2, 2]))))), 1)
        return tcps, pads
    except Exception:
        return [], []


def _box_distance(point: np.ndarray, centre_world: np.ndarray, rot: np.ndarray, half: np.ndarray) -> float:
    local = (point - centre_world) @ rot
    d = np.maximum(np.abs(local) - half, 0.0)
    return float(np.linalg.norm(d))


def tcp_to_printer(tcp: np.ndarray, lid_joint_val: float) -> float:
    """Distance from a gripper centre to the printer body, chute and the hinged lid (turned by lid_joint_val)."""
    base = np.array([PRINTER_XY[0], PRINTER_XY[1], TABLE_TOP])
    nearest = 9.0
    for shape in PRINTER_SHAPES:
        half = np.asarray(shape.size, dtype=float) / 2
        nearest = min(nearest, _box_distance(tcp, base + np.asarray(shape.at, dtype=float), _EYE, half))

    hinge_world = base + np.asarray(HINGE_AT, dtype=float)
    rot = _axis_matrix("y", float(lid_joint_val))
    for shape in COVER_SHAPES:
        centre_world = hinge_world + rot @ np.asarray(shape.at, dtype=float)
        half = np.asarray(shape.size, dtype=float) / 2
        nearest = min(nearest, _box_distance(tcp, centre_world, rot, half))
    return nearest


def tcp_to_roll(tcp: np.ndarray, roll_pos, roll_quat) -> float:
    """Distance from a gripper centre to the paper cylinder or the paper tail."""
    rot = rotation_from_quat(roll_quat)
    p = (np.asarray(tcp, dtype=float) - np.asarray(roll_pos, dtype=float)) @ rot            # roll frame
    radial = max(math.hypot(p[0], p[2] - ROLL_CENTRE_Z) - ROLL_RADIUS_M, 0.0)
    axial = max(abs(p[1]) - ROLL_CORE_WIDTH_M / 2, 0.0)
    cylinder = math.hypot(radial, axial)
    tail_half = np.asarray(ROLL_TAIL.size, dtype=float) / 2
    tail = _box_distance(p, np.asarray(ROLL_TAIL.at, dtype=float), _EYE, tail_half)
    return min(cylinder, tail)


def hands_clear(tcps, roll_pos, roll_quat, lid_joint_val: float) -> bool:
    """Every gripper centre is at least RELEASE_MIN_DIST_M from the printer (with its lid) and from the roll."""
    for tcp in tcps:
        if tcp_to_printer(np.asarray(tcp, dtype=float), lid_joint_val) < RELEASE_MIN_DIST_M:
            return False
        if tcp_to_roll(np.asarray(tcp, dtype=float), roll_pos, roll_quat) < RELEASE_MIN_DIST_M:
            return False
    return True


def _reset_run() -> None:
    _RUN.update(fixture_start=None, fixture_moved=False, open_run=0, open_run_max=0, max_lift=0.0, hist=[], step=0)


def _org(st):
    """The Mech's body origin (MechState.pos is the bounding-box centre, which moves when the cover hinge swings)."""
    o = getattr(st, "origin", None)
    return st.pos if o is None or not np.any(np.asarray(o, dtype=float)) else o  # unset (zero) origin: fall back to pos


def advance(printer, roll, stand) -> None:
    """Update run's latched facts from one control step's states."""
    joint_val = float(printer.joints.get(COVER_NAME, 0.0))
    if _RUN["fixture_start"] is None:
        _RUN["fixture_start"] = (np.asarray(_org(printer), dtype=float), np.asarray(_org(stand), dtype=float))
    p0, s0 = _RUN["fixture_start"]
    if (
        float(np.linalg.norm(np.asarray(_org(printer), dtype=float) - p0)) > FIXTURE_MOVE_M
        or float(np.linalg.norm(np.asarray(_org(stand), dtype=float) - s0)) > FIXTURE_MOVE_M
    ):
        _RUN["fixture_moved"] = True
    _RUN["open_run"] = _RUN["open_run"] + 1 if lid_open_deg(joint_val) >= BAY_OPEN_DEG else 0
    _RUN["open_run_max"] = max(_RUN["open_run_max"], _RUN["open_run"])
    _RUN["max_lift"] = max(_RUN["max_lift"], float(_org(roll)[2]) - (TABLE_TOP + ROLL_START_ORIGIN_Z))
    _RUN["hist"] = (_RUN["hist"] + [(np.asarray(_org(roll), dtype=float), joint_val)])[-STILL_WINDOW_STEPS:]


def window_still() -> bool:
    """The roll moved less than 1 mm and the cover angle changed less than 0.5 deg (or motion < 1 mm) over the last second."""
    hist = _RUN["hist"]
    if len(hist) < STILL_WINDOW_STEPS:
        return True
    roll_xyz = np.array([h[0] for h in hist])
    angles = np.array([h[1] for h in hist])
    # 0.001 rad ~ 0.057 deg
    return bool(np.ptp(roll_xyz, axis=0).max() <= STILL_MOTION_M and np.ptp(angles) <= 0.01)


def _on_reset(environment) -> None:
    _reset_run()
    _CURRENT_ENV[0] = environment
    try:
        base = environment.unwrapped
        scene = base.scene
        env0 = _to_numpy(scene.env_origins)[0]
        p_ent = scene[PRINTER_NAME]
        r_ent = scene[ROLL_NAME]
        p_pos = _to_numpy(p_ent.data.root_link_pose_w)[0][:3] - env0
        r_pose = _to_numpy(r_ent.data.root_link_pose_w)[0]
        r_pos, r_quat = r_pose[:3] - env0, r_pose[3:]
        j_val = float(_to_numpy(p_ent.data.joint_pos)[0][0]) if p_ent.joint_names else 0.0
        DISPLAY.reset(environment, lamp_state(r_pos, r_quat, j_val), r_pos, r_quat)
    except Exception:
        pass


on_reset(_on_reset)


def _gripper_open(environment, arm: str, threshold: float = 0.9) -> bool:
    try:
        robot = environment.unwrapped.scene[f"{arm}_robot"]
        names = list(robot.joint_names)
        positions = _to_numpy(robot.data.joint_pos)[0]
        for j in _FINGER_JOINTS:
            if j in names:
                q = float(positions[names.index(j)])
                frac = q / _FINGER_OPEN_POS
                if frac < threshold - 1e-6:
                    return False
        return True
    except Exception:
        return False


def _released(printer_state, roll_state) -> bool:
    env = _CURRENT_ENV[0]
    if env is None:
        return False
    try:
        for arm in ("left", "right"):
            if not _gripper_open(env, arm, FINGERS_OPEN_FRACTION):
                return False
        lid_val = float(printer_state.joints.get(COVER_NAME, 0.0))
        tcps, _ = _hand_data2(env)
        if not tcps:
            return False
        return hands_clear(tcps, _org(roll_state), roll_state.quat_xyzw, lid_val)
    except Exception:
        return False


def _trace(record: dict) -> None:
    path = os.environ.get("TASK_TRACE")
    if path:
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


def score(objects: Mapping[str, Any]) -> float:
    printer = objects.get(PRINTER_NAME)
    roll = objects.get(ROLL_NAME)
    stand = objects.get(STAND_NAME)
    if printer is None or roll is None:
        return 0.0
    _RUN["step"] += 1
    if stand is not None:
        advance(printer, roll, stand)

    lid_val = float(printer.joints.get(COVER_NAME, 0.0))
    still = bool(roll.resting(STILL_SPEED_M_S) and window_still())
    s, ok, reason = evaluate_state(
        tuple(float(v) for v in _org(roll)),
        tuple(float(v) for v in roll.quat_xyzw),
        lid_val,
        int(_RUN["open_run_max"]),
        float(_RUN["max_lift"]),
        bool(_RUN["fixture_moved"]),
        _released(printer, roll),
        still,
    )
    if _CURRENT_ENV[0] is not None:
        try:
            DISPLAY.update(_CURRENT_ENV[0], lamp_state(_org(roll), roll.quat_xyzw, lid_val), _org(roll), roll.quat_xyzw)
        except Exception:
            pass

    tcps, pads = _hand_data2(_CURRENT_ENV[0])
    try:
        _tp = os.environ.get("TASK_TRACE")
        if _tp and pads and _RUN["step"] % 4 == 0:
            with open(_tp + ".pads.jsonl", "a", encoding="utf-8") as _fh:
                _fh.write(
                    json.dumps(
                        {
                            "p": [[round(float(v), 3) for v in q] for q in pads],
                            "t": [[round(float(v), 3) for v in q] for q in tcps],
                            "tilt": dict(_TILT),
                        }
                    )
                    + "\n"
                )
    except Exception:
        pass

    trace_rec = {
        "step": int(_RUN["step"]),
        "score": float(s),
        "reason": str(reason),
        "roll": [round(float(v), 4) for v in _org(roll)],
        "lid_deg": round(lid_open_deg(lid_val), 1),
        "open_run_max": int(_RUN["open_run_max"]),
        "max_lift_mm": round(float(_RUN["max_lift"]) * 1000, 1),
        "fixture_moved": bool(_RUN["fixture_moved"]),
    }
    if pads and _RUN["step"] % 4 == 0:
        trace_rec["pads"] = [[round(float(c), 4) for c in pt] for pt in pads]
        trace_rec["tcps"] = [[round(float(c), 4) for c in pt] for pt in tcps]

    _trace(trace_rec)
    return float(s)


def success(objects: Mapping[str, Any]) -> bool:
    return score(objects) >= 1.0


def distance(objects: Mapping[str, Any]) -> float:
    return float(max(0.0, 1.0 - score(objects)) * 0.10)
