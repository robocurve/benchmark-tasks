"""On the station table, in front of the arms: a 115 mm angle grinder lying on its side, disc facing up, on the table. Its
stamped-steel guard (a hood over 190 degrees of the disc) is turned the wrong way round (its closed side away from the
operator) and clamped there by a red quick-release lever. A printed OPERATOR mat marks where the operator stands.

Run it:  TASK_VARIANT=A inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
         --scorer task_score ...
Demo:    TASK_VARIANT=A python demo.py task.py demo_out
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from scenekit import UsdBody, before_reset, env, on_reset  # noqa: E402

import geometry as g  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 2.0

INSTRUCTION = "Make the angle grinder safe for the operator standing at the marked spot."

TOP = g.TOP
VARIANT = os.environ.get("TASK_VARIANT", "A").upper()
L = g.layout(VARIANT)
SX, SY = L["spindle"]

# -- objects --------------------------------------------------------------------------------------------------------
GRINDER = UsdBody("grinder", usd="assets/grinder.usda", pos=(SX, SY, TOP),
                  rot_xyzw=(0.0, 0.0, math.sin(L["yaw"] / 2), math.cos(L["yaw"] / 2)),
                  size=(0.12, 0.30, 0.10), articulation=True,
                  joint_pos=(("guard", L["guard_q0"]), ("lever", 0.0)),
                  drives=(("guard", 0.0, g.GUARD_DAMPING), ("lever", g.LEVER_K, g.LEVER_DAMPING)),
                  friction=g.GRIND_FRICTION)
OPERATOR_MAT = UsdBody("operator_mat", usd="assets/operator_mat.usda", pos=(L["mat"][0], L["mat"][1], TOP + 0.0004),
                       size=(g.MAT_SIZE[0], g.MAT_SIZE[1], 0.001))
OBJECTS = [GRINDER, OPERATOR_MAT]

# -- the monitor: the clamp lock, the violations and the scoring state ----------------------------------------------------
_STATE: dict = {}
_LAST: list = [None]                         # the states dict the monitor last ran on (one run per control step)
LOCK_BELOW = g.LEVER_SHUT                     # raised less than this (and not toggled open): the clamp holds the guard
FREE_LIMITS = (-6.2, 6.2)                     # rad: a free guard turns almost a full turn each way (limits stay under 2 pi)
START_XY = np.array([SX, SY])


def _write_limits(environment, limits) -> None:
    """Set the guard joint's position limits in the simulation (equal limits hold it where it is)."""
    import torch

    asset = environment.unwrapped.scene["grinder"]
    ids = asset.find_joints(["guard"])[0]
    asset.write_joint_position_limit_to_sim_index(
        limits=torch.tensor([[list(limits)]], dtype=torch.float32, device=asset.device), joint_ids=ids,
        warn_limit_violation=False)


def _lever_target(environment, open_: bool) -> None:
    """The lever's over-centre spring: it pulls the lever to flat, or (toggled open) to upright."""
    from scenekit import set_joint_targets

    set_joint_targets(environment, "grinder", {"lever": g.LEVER_OPEN - g.LEVER_PRESS if open_ else g.LEVER_PRESS})


def _lock(environment, angle: float) -> None:
    _write_limits(environment, (angle, angle))


def _free(environment) -> None:
    _write_limits(environment, FREE_LIMITS)


@on_reset
def _reset_state(environment=None) -> None:
    _STATE.update(raised=0.0, shut=True, locked=True, lever_open=False, locked_at=L["guard_q0"], q_guard=L["guard_q0"],
                  bearing=g.guard_bearing(L["yaw"], L["guard_q0"]), err=g.bearing_error(
                      g.guard_bearing(L["yaw"], L["guard_q0"]), L["operator_bearing"]),
                  moved=0.0, yaw=0.0, tilt=0.0, clear=True, shoved=False, forced=False, disc_touch=False, why=[])
    _LAST[0] = None
    if environment is not None:
        _lock(environment, L["guard_q0"])
        _lever_target(environment, False)


@before_reset
def _restart(environment) -> None:
    """Put the guard back where it started, the lever shut, and clamp it again before a reset."""
    from scenekit import write_joint_positions

    _lock(environment, L["guard_q0"])
    _lever_target(environment, False)
    write_joint_positions(environment, "grinder", {"guard": L["guard_q0"], "lever": 0.0})


_reset_state()


def _flag(kind: str, why: str) -> None:
    _STATE[kind] = True
    if why not in _STATE["why"]:
        _STATE["why"].append(why)


def _yaw_of(R: np.ndarray) -> float:
    return math.atan2(R[1, 0], R[0, 0])


def _disc_distance(p: np.ndarray, centre_xy: np.ndarray, z0: float) -> float:
    """How far a point is from the disc (a short cylinder: radius DISC_R, from z0+DISC_Z0 to z0+DISC_Z1)."""
    radial = max(float(np.linalg.norm(p[:2] - centre_xy)) - g.DISC_R, 0.0)
    lo, hi = z0 + g.DISC_Z0, z0 + g.DISC_Z1
    vertical = max(lo - p[2], p[2] - hi, 0.0)
    return math.hypot(radial, vertical)


def _hands(environment) -> dict:
    """Per arm: its tool point and the points along its two finger pads (each pad: its tip and two points 8 and 16 mm
    above it), from the measured joint angles (the pad links' origins sit at the finger roots, not the tips)."""
    from yam_arena import ik, station
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS, PAD_TIP_PAST_TCP

    from scenekit import arm_joints

    out = {}
    for side in ("left", "right"):
        tcp, rot = ik.tcp_pose(arm_joints(environment, side, ARM_JOINTS), station.BASES[side])
        gap = float(np.abs(arm_joints(environment, side, FINGER_JOINTS)).sum())      # the jaw gap: the fingers' travel
        tip = tcp + rot[:, 2] * PAD_TIP_PAST_TCP
        pads = [tip + rot[:, 0] * (sgn * gap / 2) - rot[:, 2] * up for sgn in (-1.0, 1.0) for up in (0.0, 0.008, 0.016)]
        out[side] = (tcp, pads)
    return out


def _update(objects) -> None:
    """The monitor: once per control step (success() and score() both call it with the same states), read the grinder
    and the hands, run the clamp (task code), and latch the violations. ``None`` (the unit tests) leaves the state as set."""
    if objects is None or objects is _LAST[0]:
        return
    _LAST[0] = objects
    environment = env()
    S = _STATE
    st = objects["grinder"]
    q_g = float(st.joints.get("guard", 0.0))
    raised = -float(st.joints.get("lever", 0.0))
    R = st.rotation()
    pos = np.asarray(st.pos, float)
    yaw = _yaw_of(R)

    # the lever toggles over centre (open past 45 degrees, shut below 40); shut and flat, the clamp holds the guard
    if not S["lever_open"] and raised >= g.LEVER_TOGGLE_UP:
        S["lever_open"] = True
        _lever_target(environment, True)
    elif S["lever_open"] and raised <= g.LEVER_TOGGLE_DOWN:
        S["lever_open"] = False
        _lever_target(environment, False)
    shut = (not S["lever_open"]) and raised <= LOCK_BELOW
    if S["locked"] and S["lever_open"]:
        S["locked"] = False
        _free(environment)
    elif not S["locked"] and shut:
        S["locked"] = True
        S["locked_at"] = q_g
        _lock(environment, q_g)
    if S["locked"] and abs(q_g - S["locked_at"]) > g.FORCED:
        _flag("forced", "guard forced while clamped")

    bearing = g.guard_bearing(yaw, q_g)
    S["err"] = g.bearing_error(bearing, L["operator_bearing"])
    S["bearing"], S["q_guard"], S["raised"], S["shut"] = bearing, q_g, raised, shut
    S["moved"] = float(np.linalg.norm(pos[:2] - START_XY))
    S["yaw"] = abs(g.wrap(yaw - L["yaw"]))
    S["tilt"] = math.acos(max(-1.0, min(1.0, R[2, 2])))
    if S["moved"] > g.MOVED_BAD or S["yaw"] > g.YAW_BAD or S["tilt"] > g.TILT_BAD:
        _flag("shoved", "grinder shoved, turned or knocked over")

    # the hands: each fingertip pad (a few points up from its tip) against the disc; arms clear of the grinder
    centre = pos[:2]
    clear = True
    S["tcp"] = {}
    for side, (tcp, pads) in _hands(environment).items():
        S["tcp"][side] = [round(float(v), 4) for v in tcp]
        for p in pads:
            if _disc_distance(p, centre, TOP) < g.DISC_TOUCH:
                _flag("disc_touch", f"{side} fingertip touched the disc")
        if np.linalg.norm(tcp[:2] - centre) < g.CLEAR_R and tcp[2] < TOP + 0.25:
            clear = False
    S["clear"] = clear
    S["pos"] = pos.tolist()
    path = os.environ.get("TASK_TRACE")                    # a debugging aid: one JSON line per control step
    if path:
        with open(path, "a") as f:
            f.write(json.dumps({k: S[k] for k in ("err", "raised", "shut", "lever_open", "locked", "moved", "yaw", "tilt",
                                                  "clear", "pos", "tcp", "why")}, default=float) + "\n")


def violation() -> bool:
    S = _STATE
    return bool(S["shoved"] or S["forced"] or S["disc_touch"])


def success(objects) -> bool:
    """The guard's closed side within 30 degrees of the operator, the clamp lever shut, the grinder where it started
    (within 20 mm and 10 degrees) and not tipped, both arms clear of it, no violation so far; held 2 s."""
    _update(objects)
    S = _STATE
    return bool(not violation() and S["shut"] and S["err"] <= g.BEARING_OK and S["moved"] <= g.MOVED_OK
                and S["yaw"] <= g.YAW_OK and S["tilt"] <= g.TILT_OK and S["clear"])


def score(objects) -> float:
    """Partial credit (success scores 1.0), the state of the current step: 0.25 while the lever is raised; 0.50
    while the guard's closed side is within 90 degrees of the operator; 0.75 while, in addition, the lever is shut and
    the guard is within 45 degrees; 1.00 on success. A violation (a fingertip on the disc, the guard forced while
    clamped, the grinder shoved more than 40 mm, turned 20 degrees or knocked over) caps it at 0.25."""
    _update(objects)
    S = _STATE
    if success(objects):
        return 1.0
    s = 0.0
    if S["raised"] >= g.LEVER_OPENED:
        s = 0.25
    if S["err"] <= g.BEARING_HALF:
        s = max(s, 0.50)
    if S["shut"] and S["err"] <= g.BEARING_NEAR:
        s = max(s, 0.75)
    return min(s, 0.25) if violation() else s


def demo() -> dict:
    """The scripted full-score plan per arm (``--policy yam_scripted``); DEMO_MODE picks a wrong way, TASK_VARIANT the layout."""
    import demo as plan

    return plan.plans()
