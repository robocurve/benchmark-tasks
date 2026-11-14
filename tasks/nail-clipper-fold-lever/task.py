"""On the station bench, real size: a steel lever nail clipper (85 x 15 mm, 54 g) lying loose in a tray slot, jaws
toward the robot, the lever raised for cutting (pointing to the tail, 20 deg up, its heel down on the upper blade).
Two
mechkit objects (layout.py): the tray (fixed) and the clipper, loose, ONE articulation: the base (lower blade, tail
block and upper blade, one rigid body), the pin that swivels in it and the lever that hinges on the pin. The kit
switches self-collision off inside an articulation, so the lever's two rests on the upper blade (on its heel; turned
over onto the blade) are the stops of its hinge, placed where the lever meets the blade (layout.py). No joint joins two
free bodies (the earlier build's bond between two free assemblies did not hold on the GPU). Nothing is latched or moved
by code: the lever stays where gravity and its stops leave it.

Goal: swing the lever half a turn about the pin and flip it over so that it lies on the upper blade pointing to the
tail, heel side up, leave the clipper in the slot and move both grippers clear. See scoring.py for the tiers.

The assemblies are mechkit's (mechkit.py next to this file); the arms are read through rigstate.py; the pitched grip
of the flip comes from tiltgrip.py. None changes what the embodiment does.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score -E object_jitter=0 ...
Demo:    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py -E object_jitter=0 ...
Variant: DEMO_VARIANT=swing_only | press_flat before the demo command (see demo() below).
One arm: DEMO_IDLE_ARM=left (or right) before the demo command leaves that arm at its ready pose.
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402

import layout as L  # noqa: E402
import mechkit as mk  # noqa: E402  (importing it hooks yam_arena so the assemblies are built as articulations)
import rigstate  # noqa: E402
import tiltgrip  # noqa: E402
from offline_clipper import OfflineClipper  # noqa: E402
from scoring import CLIPPER, LEVER, PIN, TIP_FACE, Tracker  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402

rigstate.install()  # reads both arms before every control step (False offline: the checker feeds it instead)

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0  # 1.0 must hold for 1 s on top of the scorer's own 20 steps

INSTRUCTION = ("Fold the nail clipper shut for storage: its lever is raised for cutting and must end folded flat along "
               "the upper blade, pointing to the tail. Leave the clipper in the tray slot and move both grippers "
               "clear.")


# -- the objects ----------------------------------------------------------------------------------------------------
def _shape(row):
    kind, c, size, ypr, colour = row
    centre = tuple(v * L.MM for v in c)
    ypr_rad = None if ypr is None else tuple(math.radians(v) for v in ypr)
    if kind == "box":
        return mk.box(centre, tuple(v * L.MM for v in size), colour, ypr=ypr_rad)
    radius, height, axis = size
    return mk.cylinder(centre, radius * L.MM, height * L.MM, axis=axis, color=colour, ypr=ypr_rad)


def _shapes(rows):
    return [_shape(r) for r in rows]


def _p(*v_mm):
    return tuple(v * L.MM for v in v_mm)


_TRAY_C, _TRAY_OUTER, _TRAY_WALL, _TRAY_FLOOR = L.tray_outer()
TRAY = mk.assembly("tray", L.TRAY_ORIGIN,
                   mk.open_box(_p(*_TRAY_C), _p(*_TRAY_OUTER), _TRAY_WALL * L.MM, _TRAY_FLOOR * L.MM, L.TRAY_COLOUR))

PIN_LINK = mk.hinge(PIN, "Z", _p(L.PIN_X, 0.0, 8.0), _shapes(L.pin_shapes()), lower=-L.PIN_LIMIT, upper=L.PIN_LIMIT,
                    mass=L.M_PIN, damping=L.PIN_DAMPING, friction=L.PIN_FRICTION)
LEVER_LINK = mk.hinge(LEVER, "-Y", _p(L.PIN_X, 0.0, L.ZH), _shapes(L.lever_shapes()), lower=L.LEVER_LOWER,
                      upper=L.LEVER_UPPER, mass=L.M_LEVER, damping=L.LEVER_DAMPING, friction=L.LEVER_FRICTION,
                      parent=PIN)

# Offline the clipper is a small box at the tail (the head of the clipper, where no gripper goes in the demo), so the
# checker's own carry proxy leaves it alone; ``offline_states`` below moves the clipper's joints.
_BOX_CLIPPER = (np.array(_p(82.5, 0.0, 1.0)), _p(5.0, L.WIDTH, 2.0))


class _Offline(mk.Assembly):
    """The assembly exactly as mechkit builds and reads it in the simulator. Only tools/offline/check_task.py calls the
    two methods below: offline the clipper is a small box at the tail, and ``offline_state`` puts the base frame back
    on that box."""

    BOX = _BOX_CLIPPER

    def offline_box(self):
        origin, quat = self.spawn_pose()
        centre = origin + mk.quat_to_matrix(quat) @ self.BOX[0]
        return tuple(float(v) for v in centre), tuple(float(v) for v in self.BOX[1]), float(self.yaw)

    def offline_state(self, state):
        quat = np.asarray(state.quat_xyzw, dtype=float)
        origin = np.asarray(state.pos, dtype=float) - mk.quat_to_matrix(quat) @ self.BOX[0]
        vel = np.concatenate([np.asarray(state.lin_vel, dtype=float), np.asarray(state.ang_vel, dtype=float)])
        return mk.state_from_poses(self, (*origin, *quat), vel, source="offline")


class Clipper(_Offline):
    BOX = _BOX_CLIPPER


CLIPPER_SPEC = Clipper(name=CLIPPER, pos=L.ORIGIN, color=L.STEEL, mass=L.M_BASE, yaw=0.0,
                       shapes=tuple(_shapes(L.base_shapes())), joints=(PIN_LINK, LEVER_LINK), friction=L.FRICTION,
                       solver_iterations=L.SOLVER_ITERATIONS)
CLIPPER_SPEC._check()
OBJECTS = [TRAY, CLIPPER_SPEC]


# -- scoring plumbing -----------------------------------------------------------------------------------------------
_TRACKER = Tracker()
_EPISODE = {"mechkit": None}
OFFLINE = OfflineClipper()  # only stepped without a simulator (see offline_clipper.py)


@mk.on_reset
def _reset_offline() -> None:
    OFFLINE.reset()


@rigstate.on_reset
def _new_episode() -> None:
    """rigstate saw the embodiment reset: a new trial (the scorer also spots a teleport back to the start)."""
    _TRACKER.reset()
    OFFLINE.reset()


def no_simulator(objects) -> bool:
    """True only where nothing simulates the clipper: mechkit has no environment AND the state handed in was not read
    from one (mechkit marks those ``source="sim"``). Either sign of a simulator keeps its states untouched."""
    return mk.env() is None and getattr(objects.get(CLIPPER), "source", "sim") != "sim"


def offline_states(objects, arms) -> dict:
    """Without a simulator nothing moves the clipper's joints, so offline_clipper.py moves them from the arms'
    commanded motion; the scorer then sees those states."""
    OFFLINE.step(arms)
    origin, yaw = OFFLINE.pose()
    pose = (*origin, *mk.yaw_quat(yaw))
    out = dict(objects)
    out[CLIPPER] = mk.state_from_poses(CLIPPER_SPEC, pose, np.zeros(6), joints=OFFLINE.joints(), source="offline")
    return out


def _r(v, n: int = 5) -> list:
    return [round(float(x), n) for x in np.asarray(v, dtype=float).ravel()]


def key_poses(objects) -> dict:
    """The dense pose trace of the key parts, every control step: the clipper's base (origin, quaternion), the two
    joints, the lever's link pose, its tip (blade-side face, world) and the hinge's world point. Never raises."""
    try:
        sc = objects[CLIPPER]
        r_l, p_l = sc.link_pose(LEVER)
        return {"origin": _r(sc.origin), "quat_xyzw": _r(sc.quat_xyzw), "pin_rad": round(float(sc.joints[PIN]), 5),
                "lever_rad": round(float(sc.joints[LEVER]), 5), "lever_link": _r(p_l),
                "lever_quat": _r(mk.matrix_to_quat(np.asarray(r_l, dtype=float))),
                "tip": _r(sc.link_to_world(LEVER, TIP_FACE)),
                "hinge": _r(sc.link_to_world(LEVER, np.array([L.PIN_X, 0.0, L.ZH]) * L.MM))}
    except Exception as exc:  # noqa: BLE001 - a debugging aid only
        return {"error": str(exc)[:200]}


def _trace(row: dict) -> None:
    """With TASK_TRACE=<file>, append one JSON line per control step: what the scorer saw and decided."""
    path = os.environ.get("TASK_TRACE")
    if path:
        try:
            with open(path, "a") as f:
                f.write(json.dumps(row, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o)) + "\n")
        except OSError:  # a debugging aid only: never let it stop a trial
            pass


def success(objects) -> bool:
    """Folded, in the slot, both grippers clear and everything still for the scorer's 20 steps; held 1 s. This runs
    every control step, so it also feeds the tracker (and, without a simulator, the offline kinematics)."""
    if _EPISODE["mechkit"] != mk.episode():  # mechkit counts resets too: a second, independent reset signal
        if _EPISODE["mechkit"] is not None:
            _TRACKER.reset()
            OFFLINE.reset()
        _EPISODE["mechkit"] = mk.episode()
    arms = rigstate.arms()
    offline = no_simulator(objects)
    if offline:
        objects = offline_states(objects, arms)
    result = _TRACKER.update(objects, arms)
    row = {"episode": rigstate.episode(), "step": _TRACKER.steps, "score": result.score, "success": result.success,
           "fallback_resets": _TRACKER.resets, **result.details, "pose": key_poses(objects)}
    if arms is not None:
        try:
            row.update(left_tcp=arms["left"].tcp, right_tcp=arms["right"].tcp,
                       openings={a: round(float(arms[a].opening), 3) for a in ("left", "right")})
        except (AttributeError, KeyError, TypeError, ValueError):
            row["rigstate_error"] = "unreadable snapshot"
    else:
        row["rigstate_error"] = rigstate.error()
    if offline:
        row["offline"] = {"psi_deg": round(math.degrees(OFFLINE.psi), 2),
                          "eps_deg": round(math.degrees(OFFLINE.eps), 2)}
    _trace(row)
    return result.success


def score(objects) -> float:
    """1.0 folded flat, in the slot, both grippers clear and still; 0.5 the lever to the tail within 10 deg in plan with
    its heel side away from the lower blade; 0.25 the lever out past the jaws within 30 deg of the blade's line; 0
    otherwise. The score of the latest scored step."""
    return _TRACKER.last.score


# -- the scripted demo ----------------------------------------------------------------------------------------------
# Open loop: timed waypoints from the known start layout, nothing read from the simulator. TCP points (the pad tips
# are 13 mm below). The right arm only; the left stays parked. The gripper stays closed (tip 16 x 27 mm) throughout.
# Clipper-frame mm (layout.py) become world metres by adding the slot floor's origin.
# A1 swing: straight down, the pads closing along world x. The tips (17 mm over the slot floor) start beside the
#   lever's long edge, 45 mm from the pin, and go round the jaw end on that circle at about 25 mm/s, pushing the
#   lever's edge round half a turn; the last move presses the pads' 16 mm side face up to the lever's edge (it squares
#   the lever to the blade), then backs off 5 mm.
# A2 flip: the gripper pitched 35 deg about the clipper's y (tiltgrip.py carries the full rotation of each waypoint) so
#   the outer face of one pad faces the tail and upward, the fingertips pointing to the tail and down. Phase 1: the
#   lever's tip rides the pad face 15 mm above the fingertip edge until the lever is at 55 deg; phase 2: the fingertip
#   edge rides the lever's heel-side face 42 mm from the hinge, over the top and down to the tail, drawing in to 33 mm.
#   Past 145 deg of lever the pitch eases to 180 deg less the lever's angle (layout.flip_pitch): the pad's tip face
#   then lies parallel to the lever's face, so the pad's inner tip corner never dips into the lever or the tray.
# Retreat: straight up 60 mm, then away; hold 1.5 s.
GRIP = 0.0  # closed
YAW = math.pi / 2  # pads closing along world x
SWING_SPEED = 25.0  # mm/s along the circle
HOVER = 60.0  # mm over the working height
HOLD_S = 1.5
FLIP_SEG_S = 0.4  # per 10 deg of lever: about 25 deg/s, the fingertip edge at about 18 mm/s
VARIANTS = ("swing_only", "press_flat")
PRESS_X = 40.0  # press_flat: the pads this far along the lever from the hinge
PRESS_PAST = 1.8  # and down this far past the lever's top under the pads' tail-side edge (it is on its stop)
PRESS_TIP_Z = (L.ZH + (PRESS_X * math.cos(L.TH0) + L.PAD_C) * math.tan(L.TH0) + (L.LEV_T / 2) / math.cos(L.TH0)
               - PRESS_PAST)  # 27.4 mm over the slot floor


def ry(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def grip_yaw(pitch_deg: float):
    """A waypoint's ``yaw`` value: the straight-down grip, or pitched about world y (the approach axis toward +x and
    down, pad faces normal to it in the xz plane)."""
    if pitch_deg == 0.0:
        return YAW
    rot = ry(-math.radians(pitch_deg)) @ tiltgrip.rotation(YAW, 0.0)
    return tiltgrip.TiltedYaw(YAW, rot)


def _w(duration, tcp_mm, label, pitch_deg: float = 0.0) -> Waypoint:
    pos = tuple(float(L.ORIGIN[i] + tcp_mm[i] * L.MM) for i in range(3))
    return Waypoint(float(duration), pos, GRIP, label, grip_yaw(pitch_deg))


def _down(xy, tip_z):
    """TCP (mm) of a straight-down closed gripper with its pad tips at height ``tip_z`` over the slot floor."""
    return (float(xy[0]), float(xy[1]), tip_z + L.TIP_PAST_TCP)


def swing_plan() -> list[Waypoint]:
    circle = L.swing_circle()
    first, last = circle[0], circle[-1]
    away = -L.SWING_SIDE * L.BACKOFF
    plan = [
        _w(2.5, _down(first, L.SWING_TIP_Z + HOVER), "over the lever's long edge, 45 mm from the pin"),
        _w(1.2, _down(first, L.SWING_TIP_Z), "down beside the lever's edge, tips 17 mm over the slot floor"),
        _w(0.3, _down(first, L.SWING_TIP_Z), "settle"),
    ]
    for a, b in zip(circle, circle[1:]):
        dur = max(0.2, float(np.linalg.norm(b - a)) / SWING_SPEED)
        plan.append(_w(dur, _down(b, L.SWING_TIP_Z), "round the jaw end on the circle about the pin"))
    end = (last[0], last[1] + away)
    plan.append(_w(0.5, _down(last, L.SWING_TIP_Z), "square the lever against the 16 mm side face"))
    plan.append(_w(0.5, _down(end, L.SWING_TIP_Z), "back off 5 mm"))
    return plan


def flip_plan() -> list[Waypoint]:
    pitch = L.PITCH_DEG
    edges = L.flip_edges(pitch)
    n = L.n_vec(pitch)
    e0 = np.array([edges[0][0], 0.0, edges[0][1]])
    near = e0 - 20.0 * n  # 20 mm off the lever's tip, on the pad's own side
    high = L.tcp_for_edge(near, pitch) + np.array([0.0, 0.0, 70.0])
    plan = [
        _w(0.8, _hold_at_end(), "straight up from the lever"),
        _w(1.5, high, "over to the lever's tip side, pitching the gripper 35 deg", pitch),
        _w(0.8, L.tcp_for_edge(near, pitch), "down, the pad face 20 mm off the tip", pitch),
        _w(1.0, L.tcp_for_edge(e0, pitch), "in until the pad face meets the lever's tip", pitch),
    ]
    for x, z, label, p in edges[1:]:
        plan.append(_w(FLIP_SEG_S, L.tcp_for_edge((x, 0.0, z), p), f"flip: {label}", p))
    return plan


def _hold_at_end():
    last = L.swing_circle()[-1]
    return _down((last[0], last[1] - L.SWING_SIDE * L.BACKOFF), L.SWING_TIP_Z + HOVER)


def retreat_plan(pitch_deg: float, from_tcp, back: float) -> list[Waypoint]:
    """Straight up 60 mm, then up and away from the clipper (toward the robot) by ``back`` mm; hold."""
    up = np.asarray(from_tcp, dtype=float) + np.array([0.0, 0.0, HOVER])
    away = up + np.array([-back, 0.0, 40.0])
    return [
        _w(0.8, up, "straight up, clear of the clipper", pitch_deg),
        _w(1.2, away, "away from the clipper", pitch_deg),
        _w(HOLD_S, away, "hold", pitch_deg),
    ]


def press_plan() -> list[Waypoint]:
    x = L.PIN_X + PRESS_X * math.cos(L.TH0)
    return [
        _w(3.0, _down((x, 0.0), 60.0), "over the lever, 40 mm from the hinge"),
        _w(1.5, _down((x, 0.0), 30.0), "down toward the lever"),
        _w(3.0, _down((x, 0.0), PRESS_TIP_Z), "press the lever down onto its heel"),
        _w(8.0, _down((x, 0.0), PRESS_TIP_Z), "hold it there to the end"),
    ]


def plans(variant: str = "") -> dict:
    """The right arm's plan for the demo or a counterfactual variant (the left stays parked)."""
    if variant not in ("", *VARIANTS):
        raise ValueError(f"DEMO_VARIANT={variant!r} is not one of {', '.join(VARIANTS)}")
    if variant == "press_flat":
        return {"right": press_plan()}
    plan = swing_plan()
    if variant == "swing_only":
        last = plan[-1]
        tcp = [(last.pos[i] - L.ORIGIN[i]) / L.MM for i in range(3)]
        return {"right": plan + retreat_plan(0.0, tcp, 50.0)}
    plan += flip_plan()
    last = plan[-1]
    tcp = [(last.pos[i] - L.ORIGIN[i]) / L.MM for i in range(3)]
    return {"right": plan + retreat_plan(L.flip_edges()[-1][3], tcp, 50.0)}


def demo():
    """The right arm. DEMO_HOLD_ONLY=1: no plan (both arms hold the ready pose, a settling check).
    DEMO_IDLE_ARM=left|right: that arm's plan is dropped (the one-arm checks).
    DEMO_VARIANT (counterfactual runs for the README, never the submitted demo):
      swing_only   the swing (A1), then the retreat: the lever ends out past the jaws, not folded (0.25)
      press_flat   straight down on the lever 40 mm from the hinge, 1.8 mm past its top, held to the end (0)"""
    if os.environ.get("DEMO_HOLD_ONLY") == "1":
        return {}
    plan = plans(os.environ.get("DEMO_VARIANT", ""))
    plan.pop(os.environ.get("DEMO_IDLE_ARM", ""), None)
    return plan
