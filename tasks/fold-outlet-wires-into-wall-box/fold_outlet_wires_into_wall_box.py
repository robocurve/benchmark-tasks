"""A short piece of off-white wall stands on the table in front of the robot, a little to its right. Set into it,
flush with its face, is an empty blue single-gang electrical box with a threaded screw boss at its top and bottom. An
ivory wall outlet on a steel mounting strap hangs out of the box on its wires, about 11 cm in front of the wall: a
black wire to its right side, a white wire to its left side and a bare copper wire to the left side, lower down. The
wires are stiff solid copper, about 20 cm long, still kinked where they were folded, and come out of the top back of the
box. The outlet's two mounting screws sit in the ends of its strap, one above its face and one below. A red-handled
ratcheting screwdriver lies on a grey rest on the table to the robot's left, pointing away from the robot.
"""

# yam_arena hands the docstring above to the policy as task documentation, so it describes only what is on the table.
#
# The numbers (sizes, the wires' start shape, the scoring limits and the scripted demo's moves) are in outlet_logic.py;
# copper.py is how a 12 AWG copper conductor bends; labkit.py builds the outlet and its wires as one articulation.
# screws.py holds the two mounting screws, the screwdriver and the rule that stands in for the thread.
# The scripted demo is closed loop (it reads where the wires and the outlet are as it goes): scripted_demo.py runs it.
# Development switches, left at their defaults for the task:
#   OUTLET_STYLE=d6|alternate   joint style (two-axis joints, or alternating revolute joints)
#   OUTLET_ARMATURE=5e-5        per-joint armature (kg m^2): larger is steadier, and slower to move
#   OUTLET_STIFFNESS_SCALE=1.0  multiplies copper's joint stiffness
#   OUTLET_HARD=1.0             scales copper's work hardening (0: none)

from __future__ import annotations

import json
import math
import os
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))  # labkit and outlet_logic sit next to this file

import numpy as np  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

import copper  # noqa: E402
import outlet_logic as L  # noqa: E402
import screw_moves  # noqa: E402
import screws as S  # noqa: E402
from labkit import (  # noqa: E402
    PHYSICS_ERRORS, Built, Harness, HarnessPart, HarnessWire, Part, Pose, Prop, forward_grip, on_physics, on_reset,
    on_step, place, push, push_link, ready_pose, state,
)

SCENE = "station"
assert abs(TABLE_TOP[SCENE] - L.TOP) < 1e-9, "outlet_logic.TOP must match the station's table top"
CONTROL_HZ = L.CONTROL_HZ
SUCCESS_HOLD_S = 1.0
INSTRUCTION = ("Put the outlet back in its wall box with every wire inside the box, then drive its two mounting screws "
               "home with the screwdriver and let go.")
PROBE = os.environ.get("OUTLET_PROBE", "straight")
LOG = bool(os.environ.get("OUTLET_LOG"))  # print the measurements behind the score twice a second

DRYWALL, BLUE, IVORY, STEEL = (0.90, 0.90, 0.87), (0.15, 0.33, 0.72), (0.93, 0.91, 0.84), (0.66, 0.67, 0.70)
HOLE, BLUE_SHADE = (0.04, 0.04, 0.05), (0.08, 0.18, 0.42)  # a hole seen from outside; the lead-in round a boss's hole

# -- the wall, with a hole for the box -------------------------------------------------------------------------------------
_y, _z = L.OPEN_C
_hy, _hz = L.BOX_W / 2 + L.BOX_WALL, L.BOX_H / 2 + L.BOX_WALL  # the hole fits the box's outside
_wx = L.WALL_X + L.WALL_T / 2
_lo, _hi = L.TOP, L.TOP + L.WALL_H


def _wall(name, y0, y1, z0, z1) -> Box:
    return Box(name, pos=(_wx, (y0 + y1) / 2, (z0 + z1) / 2), size=(L.WALL_T, y1 - y0, z1 - z0), color=DRYWALL)


WALL = [
    _wall("wall_right", L.WALL_Y[0], _y - _hy, _lo, _hi),
    _wall("wall_left", _y + _hy, L.WALL_Y[1], _lo, _hi),
    _wall("wall_below", _y - _hy, _y + _hy, _lo, _z - _hz),
    _wall("wall_above", _y - _hy, _y + _hy, _z + _hz, _hi),
]

# -- the box: five walls and two mounting bosses, its rim flush with the wall face ----------------------------------------
_bx = L.WALL_X + L.BOX_D / 2
_t = L.BOX_WALL
BOX = [
    Box("box_back", pos=(L.WALL_X + L.BOX_D + _t / 2, _y, _z), size=(_t, 2 * _hy, 2 * _hz), color=BLUE),
    Box("box_top", pos=(_bx, _y, _z + L.BOX_H / 2 + _t / 2), size=(L.BOX_D, 2 * _hy, _t), color=BLUE),
    Box("box_bottom", pos=(_bx, _y, _z - L.BOX_H / 2 - _t / 2), size=(L.BOX_D, 2 * _hy, _t), color=BLUE),
    Box("box_right", pos=(_bx, _y - L.BOX_W / 2 - _t / 2, _z), size=(L.BOX_D, _t, L.BOX_H), color=BLUE),
    Box("box_left", pos=(_bx, _y + L.BOX_W / 2 + _t / 2, _z), size=(L.BOX_D, _t, L.BOX_H), color=BLUE),
    Box("boss_top", pos=(_bx, _y, _z + L.BOX_H / 2 - L.BOSS_P / 2), size=(L.BOX_D, L.BOSS_W, L.BOSS_P), color=BLUE),
    Box("boss_bottom", pos=(_bx, _y, _z - L.BOX_H / 2 + L.BOSS_P / 2), size=(L.BOX_D, L.BOSS_W, L.BOSS_P),
        color=BLUE),
]
# The bosses' screw holes, drawn on their faces at outlet_logic.boss_holes() (screws.boss_marks). Looks only: a fixed
# prop with no collider of its own and no colliding part, so the bosses above are the same solid boxes to the wires.
_hole_c = (L.WALL_X, _y, _z)
BOSS_HOLES = Prop(
    "boss_holes", pos=_hole_c, size=(2 * S.MARK, S.BOSS_LEAD_D, L.SCREW_SPACING + S.BOSS_LEAD_D), color=BLUE,
    parts=tuple(Part(name, mesh, at=tuple(float(a - c) for a, c in zip(at, _hole_c, strict=True)),
                     color=BLUE_SHADE if name.startswith("lead") else HOLE,
                     roughness=0.6 if name.startswith("lead") else 0.9)
                for name, mesh, at in S.boss_marks()))

# -- the outlet on its wires (labkit Harness) ------------------------------------------------------------------------------
_ear_z = L.STRAP_MID_H / 2 + L.EAR_H / 2
PARTS = (
    HarnessPart("strap", (L.STRAP_T, L.BODY[1], L.STRAP_MID_H), (-L.STRAP_T / 2, 0.0, 0.0), STEEL),
    HarnessPart("ear_top", (L.STRAP_T, L.STRAP_W, L.EAR_H), (-L.STRAP_T / 2, 0.0, _ear_z), STEEL),
    HarnessPart("ear_bottom", (L.STRAP_T, L.STRAP_W, L.EAR_H), (-L.STRAP_T / 2, 0.0, -_ear_z), STEEL),
    HarnessPart("body", L.BODY, (L.BODY[0] / 2, 0.0, 0.0), IVORY),
    HarnessPart("face", L.FACE, (-L.STRAP_T - L.FACE[0] / 2, 0.0, 0.0), IVORY),
)
# The strap's two screw slots, drawn where screws.slot_points has them (screws.slot_marks). Looks only: the strap
# above stays the same solid plate to the wires, the wall and the screws' heads (labkit's Harness.looks never collide).
SLOTS = tuple(Part(name, mesh, at=at, color=HOLE, roughness=0.9) for name, mesh, at in S.slot_marks())


def _link_mass(w: L.WireRun) -> float:
    copper = np.pi * (L.D12 / 2) ** 2 * L.SEGMENT * L.COPPER_DENSITY
    insulation = np.pi * (w.radius**2 - (L.D12 / 2) ** 2) * L.SEGMENT * 1400.0  # PVC/nylon
    return float(copper + max(0.0, insulation))


_f = lambda key, default: float(os.environ.get(key, default))  # noqa: E731


def _torque_cap() -> float | None:
    v = os.environ.get("OUTLET_TORQUE_CAP", "").strip()
    return None if not v else L.yield_moment() if v == "yield" else float(v)


GROUND = os.environ.get("OUTLET_GROUND", "side15")  # the candidate task's build (outlet_logic.BUILDS)


HARD = _f("OUTLET_HARD", 1.0)  # scales copper's work hardening (1: handbook annealed copper; 0: none)


def build_harness(ground: str = GROUND) -> Harness:
    kp, flow = copper.flow_table(HARD)
    scale = _f("OUTLET_STIFFNESS_SCALE", 1.0)
    return Harness(
        "outlet", pos=tuple(L.ANCHOR), size=(0.01, 0.01, 0.01), color=IVORY,
        wires=tuple(HarnessWire(w.name, tuple(np.asarray(w.root) - L.ANCHOR), w.terminal, L.segments(w), w.radius,
                                w.color, _link_mass(w),
                                points=tuple(tuple(float(v) for v in p) for p in L.start_polyline(w) - L.ANCHOR))
                    for w in L.wires(ground)),  # pre-bent: each wire's relaxed S (outlet_logic.start_polyline)
        parts=PARTS, looks=SLOTS, device_at=tuple(L.DEVICE_START - L.ANCHOR), device_mass=L.DEVICE_MASS,
        stiffness=L.joint_stiffness(L.SEGMENT) * scale,
        damping=0.03, armature=_f("OUTLET_ARMATURE", 5e-5),
        yield_angle=L.yield_angle(L.SEGMENT) * _f("OUTLET_YIELD_SCALE", 1.0),
        max_bend=L.MAX_BEND, style=os.environ.get("OUTLET_STYLE", "d6"), torque_cap=_torque_cap(),
        # Round 13, the copper rule (copper.py): a half link clamped at each end, a joint per SEGMENT of wire, yield at
        # copper's flow moment (59 N mm at first, about 100 N mm once fully plastic, more as it work-hardens), and the
        # drive torque capped at that moment each control step. Rounds 1-12 yielded at 59 N mm for ever, and a careful
        # push then seated the outlet with no wire handling.
        segment=L.SEGMENT, flow_kp=tuple(float(v) for v in kp), flow_m=tuple(float(v) * scale for v in flow),
        rigid_root=True, end_filter=L.END_FILTER,
        hardened0=tuple(float(v) for w in L.wires(ground) for v in L.start_hardening(w)),  # the old folds start stiff
    )


HARNESS = build_harness()

# -- the two mounting screws, the screwdriver and its rest (screws.py) -----------------------------------------------------
# ZINC: clear zinc plate on steel, a cool mid grey. ZINC_METAL: mostly metal, so it shines but keeps some of its colour.
ZINC, RED, BLACK, TOOL_STEEL, REST_GREY = (0.62, 0.64, 0.68), (0.74, 0.10, 0.08), (0.07, 0.07, 0.07), (0.72, 0.73, 0.76), \
    (0.36, 0.37, 0.40)
ZINC_METAL = 0.7
_sm, _dm = S.screw_meshes(), S.driver_meshes()
_START_SCREWS = S.Screws()


def _screw(i: int) -> Built:
    """A mounting screw as a driven body: its head collides (it is what presses the strap to the wall), its shank is
    only drawn. The head has no friction: a head that nothing can stop, gripping the strap while it pulls it flat, twists
    the whole outlet round (the other slot swung 2.5 mm off its hole, turning head or not). What keeps the strap on its
    holes is each started screw's shank in its slot, which screws.pin_forces stands in for."""
    pos, axis, _ = _START_SCREWS.pose(L.DEVICE_START, (0.0, 0.0, 0.0, 1.0), i)
    return Built(
        S.NAMES[i], pos=tuple(float(v) for v in pos - axis * S.HEAD_H / 2), size=S.SCREW_CORE, color=ZINC,
        mass=S.SCREW_MASS, driven=True, friction=(0.0, 0.0), friction_mix="min",
        parts=(Part("head", _sm["head"], color=ZINC, collides=True, roughness=0.3, metallic=ZINC_METAL),
               Part("shank", _sm["shank"], color=ZINC, roughness=0.35, metallic=ZINC_METAL),
               Part("slot", _sm["slot"], color=BLACK), Part("cross", _sm["cross"], color=BLACK)))


SCREWS = [_screw(0), _screw(1)]
# The screwdriver and the screws don't collide: a rigid point on a rigid domed head skates off it (first Isaac run of
# the bit, 19 Nov: 3.4 mm off the axis before the recess rule could hold it). The thread rule is their whole contact:
# the head stops the bit and the recess centres it (screws.py).
SCREWDRIVER = Built(
    "screwdriver", pos=tuple(float(v) for v in S.driver_start()), size=S.DRIVER_CORE, color=RED, mass=S.DRIVER_MASS,
    apart_from=S.NAMES,
    parts=(Part("grip", _dm["grip"], color=RED, collides=True, roughness=0.6),
           Part("collar", _dm["collar"], color=BLACK, collides=True, roughness=0.4),
           Part("cap", _dm["cap"], color=BLACK, collides=True, roughness=0.4),
           Part("shaft", _dm["shaft"], color=TOOL_STEEL, collides=True, roughness=0.25),
           Part("tip", _dm["tip"], color=TOOL_STEEL, collides=True, roughness=0.25)))
REST = [Box(name, pos=tuple(float(v) for v in at), size=tuple(float(v) for v in size), color=REST_GREY)
        for name, size, at in S.rest_posts()]

OBJECTS = [*WALL, *BOX, BOSS_HOLES, HARNESS, *SCREWS, SCREWDRIVER, *REST]

# The thread: screws.py's rule, run before every physics step. It puts each screw where it belongs (riding in the strap
# until it is started, then in the box, one turn deeper per forward turn of an engaged screwdriver) and holds the
# engaged bit on the screw's axis, as the recess does. Nothing else moves an object.
THREAD = S.Screws()
_THREAD: dict = {"device": None, "error": None, "tip": None}


@on_physics
def _drive_screws(environment) -> None:
    try:
        from labkit import _np

        base = environment.unwrapped
        outlet = base.scene["outlet"]
        if _THREAD["device"] is None:
            _THREAD["device"] = list(outlet.body_names).index("device")
        d = _THREAD["device"]
        origin = _np(base.scene.env_origins)[0][:3]
        dev_pos = _np(outlet.data.body_pos_w)[0][d] - origin
        dev_quat = _np(outlet.data.body_quat_w)[0][d]
        driver = base.scene["screwdriver"]
        pose = _np(driver.data.root_link_pose_w)[0]
        vel = _np(driver.data.root_vel_w)[0]
        poses, force, tip = THREAD.step(dev_pos, dev_quat, pose[:3] - origin, L._rot(pose[3:7]), vel[:3], vel[3:6])
        _THREAD["tip"] = tip
        for name, (pos, quat) in zip(S.NAMES, poses, strict=True):
            place(environment, name, pos, quat)
        if force.any():
            push(environment, "screwdriver", force, tip)
        for pin, at in S.pin_forces(dev_pos, dev_quat, THREAD.started, THREAD.pin):  # a started screw's shank holds its slot
            push_link(environment, "outlet", d, pin, at)  # one call each, as tuned (see screws.K_PIN)
    except Exception as err:  # noqa: BLE001 - a broken thread must show in the log, never stop a trial
        _THREAD["error"] = f"{type(err).__name__}: {err}"


# -- scoring ---------------------------------------------------------------------------------------------------------------
# score() sees object states only, so an on_step hook records each step how close the nearest gripper point (TCP and
# pad tips, both arms) comes to the outlet's outline: credit needs the outlet let go, 3 cm clear (outlet_logic.CLEAR).
SEEN = {"hand_clear": float("inf"), "clear_s": 0.0, "anchor": None, "error": None}


@on_reset
def _new_trial() -> None:
    SEEN["hand_clear"], SEEN["clear_s"], SEEN["anchor"], SEEN["error"] = float("inf"), 0.0, None, None
    THREAD.reset()
    _THREAD["error"], _THREAD["tip"] = None, None


def _gripper_points(environment) -> list:
    from labkit import _np
    from yam_arena.runtime import pad_tips_world, tcp_world

    base = environment.unwrapped
    origin = _np(base.scene.env_origins)[0]
    out = []
    for key in ("left_robot", "right_robot"):
        robot = base.scene[key]
        out += [np.asarray(tcp_world(robot), float) - origin,
                *np.atleast_2d(np.asarray(pad_tips_world(robot), float) - origin)]
    return out


@on_step
def _step(environment) -> None:
    if environment is None:
        return
    try:
        s = state(environment, "outlet")
        pts = _gripper_points(environment)
        if _THREAD["tip"] is not None:  # the screwdriver's tip counts as a hand: left on a screw, it has not let go
            pts = [*pts, np.asarray(_THREAD["tip"], float)]
        # clear of the outlet AND of every wire: a gripper pressing the wires into the box has not let go
        SEEN["hand_clear"] = min(L.hand_clearance(pts, s.pos, s.quat_xyzw), L.wire_clearance(pts, s.polylines))
        # how long it has been let go and at rest, without a break: no strap corner 1 mm from where it was (L.rest_count)
        SEEN["clear_s"], SEEN["anchor"] = L.rest_count(SEEN["clear_s"], SEEN["anchor"], L.strap_corners(s.pos, s.quat_xyzw),
                                                       SEEN["hand_clear"], 1.0 / CONTROL_HZ)
    except Exception as err:  # noqa: BLE001 - never credit an outlet whose grippers couldn't be checked
        SEEN["hand_clear"], SEEN["clear_s"], SEEN["anchor"], SEEN["error"] = 0.0, 0.0, None, f"{type(err).__name__}: {err}"
    if LOG:  # OUTLET_LOG=1: the measurements behind the score, twice a second
        try:
            from labkit import trial_time

            n = int(round(trial_time() * CONTROL_HZ))
            if n % 10 == 0:
                s = state(environment, "outlet")
                front = {k: round(1000 * float(L.WALL_X - np.min(np.asarray(v)[:-1, 0])), 1) for k, v in s.polylines.items()}
                print("OUTLET " + json.dumps({
                    "t": round(n / CONTROL_HZ, 2), "device_mm": [round(1000 * float(v), 1) for v in s.pos],
                    "strap_gap_mm": [round(1000 * float(v), 1) for v in L.strap_gaps(s.pos, s.quat_xyzw)],
                    "tab_off_mm": [round(1000 * float(v), 1) for v in L.tab_offsets(s.pos, s.quat_xyzw)],
                    "loose_wire_front_mm": front, "wires_outside": L.wires_outside(s.polylines),
                    "hand_clear_mm": round(1000 * min(float(SEEN["hand_clear"]), 9.999), 1), "clear_s": round(SEEN["clear_s"], 2),
                    "speed_mm_s": round(1000 * float(np.linalg.norm(s.lin_vel)), 1),
                    "screw_turns": [round(float(v), 2) for v in THREAD.turns], "screw_started": list(THREAD.started),
                    "bit_on_screw": list(THREAD.engaged),
                    "screw_head_mm": [round(1000 * float(L.WALL_X - state(environment, n).pos[0] - S.HEAD_H / 2), 1)
                                      for n in S.NAMES],
                    "score": score({"outlet": s, **{n: state(environment, n) for n in S.NAMES}}),
                    "demo_error": _DEMO.get("error") if _DEMO else None, "check_error": SEEN["error"],
                    "thread_error": _THREAD["error"], "physics_errors": dict(PHYSICS_ERRORS) or None,
                    "screw_moves": _SCREWS["log"], "screw_moves_error": _SCREWS["error"]}), flush=True)
        except Exception as err:  # noqa: BLE001 - logging must never stop a trial
            print(f"OUTLET log error: {type(err).__name__}: {err}", flush=True)


def score(objects) -> float:
    """0 unless the outlet stands upright (within 15 degrees), every strap corner is within 15 mm of the wall, every
    wire is inside the box (none more than 2 mm in front of the wall face, each wire's end at its terminal aside; none
    looped over the box's edge or trapped under the strap), and it has been let go: both grippers and the
    screwdriver's tip at least 3 cm from the outlet and from every wire, and the outlet at rest (no corner of its strap
    moved more than 1 mm), for the last 2 s. Then: 0.25 if a strap slot is more than 3 mm off its boss hole; 0.5 with both slots on their holes
    and no screw driven home; 0.75 with one screw home; 1.0 with both. A screw is home when its head is down on the
    strap with that end of the strap on the wall (the head's underside within 1 mm of the strap's thickness off the
    wall)."""
    s = objects.get("outlet")
    if s is None:
        return 0.0
    seated = L.task_score(s.pos, s.quat_xyzw, s.polylines, SEEN["hand_clear"], 0.0, SEEN["clear_s"])  # at rest: in clear_s
    heads = [objects.get(n) for n in S.NAMES]
    homes = S.screws_home([h.pos for h in heads]) if all(h is not None for h in heads) else 0
    return S.task_score(seated, homes)


def success(objects) -> bool:
    return score(objects) >= 1.0


# -- the probe's scripted moves ----------------------------------------------------------------------------------------------
def _waypoint(arm: str, step):
    seconds, pos, fingers, grip, label = step
    # "grip": part-closed on the outlet's body (outlet_logic.GRIP_FINGER), so the squeeze stays under the fingers'
    # 50 N limit and they hold it firmly sideways (round 3: fully closed, both fingers sat at the limit).
    finger = {"open": FINGER_OPEN, "closed": FINGER_CLOSED, "grip": L.GRIP_FINGER, "grip9": L.GRIP_FIRM}[fingers]
    if grip == "ready":
        w = ready_pose(arm, duration=seconds, label=label)
        w.finger = finger
        return w
    rot = forward_grip(0.0)
    if grip.startswith("pitch:"):  # a forward grip tipped about world y (outlet_logic.roty)
        rot = L.roty(float(grip.split(":", 1)[1])) @ rot
    elif grip.startswith("yaw:"):  # a forward grip turned about the vertical (the left hand's poke)
        rot = forward_grip(float(grip.split(":", 1)[1]))
    elif grip.startswith("crest:"):  # round 12: the left hand's press, fingertips forward and 30 degrees down
        rot = L.crest_rotation(float(grip.split(":", 1)[1]))
    return Pose(seconds, pos, finger, label, rot=rot)


def plan_waypoints(plan: dict) -> dict:
    """A plan in outlet_logic's step format (probe_plan, trial_plan) as yam_arena waypoints."""
    return {arm: [_waypoint(arm, s) for s in steps] for arm, steps in plan.items()}


# -- the scripted demo -----------------------------------------------------------------------------------------------------
# Closed loop: it reads where the wires and the outlet are as it goes (a scripted demo may read the scene; it never moves
# an object or edits the simulation). The benchmark's yam_scripted policy plays demo()'s waypoints open loop, so the loop
# is closed here: demo() hands the policy waypoint objects, and after every control step _demo_control() works out
# where each hand should be on the next step and writes that into the two waypoints the policy is interpolating between.
# The moves and the control rules are the ones scripted_demo.py ran to build the demo (outlet_logic's trial DEMO_TRIAL).
DEMO_TRIAL = "j-run"
_DEMO: dict = {}


def demo():
    """The scripted demo's plan, per arm, for the yam_scripted policy (see _demo_control)."""
    import copy

    from yam_arena.demo import ArmPlan
    from yam_arena.station import BASES

    tr = L.trial(DEMO_TRIAL)
    steps = L.trial_plan(tr)
    way = plan_waypoints(steps)
    wires_s = L.plan_seconds(steps)
    # The screw part. The left hand's targets are set step by step (_screw_control). The right hand ends the wire
    # part in front of the outlet, where the screwdriver has to pass, so it goes straight back to its ready pose.
    way["left"] = [*way["left"], ready_pose("left", duration=SCREWS_S, label="drive the screws")]
    way["right"] = [*way["right"], ready_pose("right", duration=RIGHT_CLEAR_S, label="back to ready"),
                    ready_pose("right", duration=SCREWS_S - RIGHT_CLEAR_S, label="wait")]
    SCREW_MOVES.reset()
    _SCREWS.update(t0=wires_s, error=None, log=None)
    _DEMO.clear()
    _DEMO.update(
        tr=tr, steps=steps, way=way,
        plan={arm: ArmPlan(copy.deepcopy(ws), BASES[arm]) for arm, ws in way.items()},  # the plan as written
        g={"twist": 0.0, "x": None, "frozen_s": 0.0, "started": False}, corr=np.zeros(3), corr_l=np.zeros(3),
        face_snap=None, error=None)
    return way


SCREWS_S = 100.0  # the longest the screw part may take (s)
RIGHT_CLEAR_S = 2.5  # the right hand is back at its ready pose this long into the screw part (s)
SCREW_MOVES = screw_moves.ScrewMoves()
_SCREWS: dict = {"t0": None, "error": None, "log": None}


def _right_to_ready(t: float) -> None:
    """The right hand through the screw part, one control step: back to its ready pose, as the plan was written. It
    has to be set step by step like the left hand's target: the wire part's last step writes the right hand's place
    in front of the outlet into the waypoint that follows it, so left to the plan it would stay in the screwdriver's
    way."""
    pos, rot, finger, _ = _DEMO["plan"]["right"].target(t)
    _play("right", t, pos, rot, finger)


def _screw_control(environment, t: float) -> None:
    """The demo's screw part, one control step: send the right hand back to its ready pose, then read the left hand,
    the screwdriver and the screws, and set the left hand's next target (screw_moves.py)."""
    _right_to_ready(t)
    from labkit import _np, rolled
    from yam_arena.runtime import tcp_world

    base = environment.unwrapped
    origin = _np(base.scene.env_origins)[0][:3]
    tcp = np.asarray(tcp_world(base.scene["left_robot"]), float).reshape(-1)[:3] - origin
    drv = state(environment, "screwdriver")
    rot = L._rot(drv.quat_xyzw)
    tip = np.asarray(drv.pos, float) + rot @ S.DRIVER_TIP
    out = state(environment, "outlet")
    tops = []
    for i in range(2):
        under, axis, _ = THREAD.pose(out.pos, out.quat_xyzw, i)
        tops.append(under - axis * S.HEAD_H)
    pos, roll, finger, label = SCREW_MOVES.step(tcp, drv.pos, tip, tops, list(THREAD.engaged),
                                                [THREAD.home(0), THREAD.home(1)])
    _play("left", t, pos, rolled(forward_grip(0.0), roll), finger)
    i = SCREW_MOVES.screw
    _SCREWS["log"] = {"stage": label, "roll": round(float(roll), 1), "tcp_err_mm": [round(1000 * float(v), 1) for v in pos - tcp],
                      "tip_to_head_mm": [round(1000 * float(v), 1) for v in tip - tops[i]],
                      "tool_off_deg": round(math.degrees(math.acos(max(-1.0, min(1.0, float(rot[0, 0]))))), 1),
                      "handle_mm": [round(1000 * float(v), 1) for v in drv.pos]}


def _play(arm: str, t: float, pos, rot, finger) -> None:
    """Make the policy's plan for ``arm`` give exactly (pos, rot, finger) at time ``t``: both waypoints it is
    interpolating between are set to that target. (During a plan's first step there is no earlier waypoint, and
    nothing is corrected then.)"""
    acc, ws = 0.0, _DEMO["way"][arm]
    for i, w in enumerate(ws):
        if t < acc + w.duration:
            if i > 0:
                for v in (ws[i - 1], w):
                    v.pos, v.rot, v.finger = tuple(float(c) for c in pos), np.asarray(rot, float), float(finger)
            return
        acc += w.duration
    if ws:  # past the end: the plan holds its last waypoint
        ws[-1].pos, ws[-1].rot, ws[-1].finger = tuple(float(c) for c in pos), np.asarray(rot, float), float(finger)


@on_step
def _demo_control(environment) -> None:
    if environment is None or not _DEMO:
        return
    d = _DEMO
    if _SCREWS["t0"] is not None:
        try:
            from labkit import trial_time as _now

            t_next = (int(round(_now() * CONTROL_HZ)) + 1) / CONTROL_HZ
            if t_next > _SCREWS["t0"]:
                _screw_control(environment, t_next)
                return
        except Exception as err:  # noqa: BLE001
            _SCREWS["error"] = f"{type(err).__name__}: {err}"
            return
    try:
        import dataclasses

        from labkit import _np, rolled, trial_time
        from yam_arena.runtime import tcp_world

        tr, g, corr, corr_l, steps_plan = d["tr"], d["g"], d["corr"], d["corr_l"], d["steps"]
        step = int(round(trial_time() * CONTROL_HZ)) + 1  # the control step the policy is about to command
        t = step / CONTROL_HZ
        right, left = d["plan"]["right"], d["plan"]["left"]
        fast = getattr(tr, "steer", "slow") == "fast"
        every = L.FAST_EVERY if fast else 10
        # -- the right hand: keep the held outlet centred on the opening, and hold the push back until it is
        planned = right.target(t)
        label = planned[3]
        st = state(environment, "outlet")
        dev = np.asarray(st.pos, float)
        roll = L.roll_deg(st.quat_xyzw)
        if getattr(tr, "steer", "slow") != "none" and step % every == 0:
            if label in L.AIM_LABELS:
                gain, cy, cz = (L.FAST_GAIN, L.FAST_Y, L.FAST_Z) if fast else (L.AIM_GAIN, L.AIM_Y, L.AIM_Z)
                corr[1] = np.clip(corr[1] + gain * (L.OPEN_C[0] - dev[1]), -cy, cy)
                corr[2] = np.clip(corr[2] + gain * (L.OPEN_C[1] - dev[2]), *cz)
                if fast:
                    g["twist"] = float(np.clip(g["twist"] - L.TWIST_GAIN * roll, -L.TWIST_CAP, L.TWIST_CAP))
            elif label in L.FACE_LABELS:
                if d["face_snap"] is None:
                    d["face_snap"] = dev.copy()
                corr[1] = np.clip(d["face_snap"][1] - L.OPEN_C[0], -L.AIM_Y, L.AIM_Y)
                corr[2] = np.clip(d["face_snap"][2] - L.OPEN_C[1], *L.AIM_Z)
        if getattr(tr, "gate", False):  # every step: the push's x, held back while the outlet is off
            plan_x = float(planned[0][0])
            if label in L.GATED_LABELS:
                g["started"] = True
            if g["started"] and label in L.AIM_LABELS:
                off = max(abs(dev[1] - L.OPEN_C[0]), abs(dev[2] - L.OPEN_C[1]))
                aligned = off <= L.GATE_MM / 1000 and abs(roll) <= L.GATE_DEG
                if g["x"] is None:
                    g["x"] = plan_x
                if label in L.GATED_LABELS and not aligned and g["frozen_s"] < L.GATE_MAX_S:
                    g["frozen_s"] += 1.0 / CONTROL_HZ
                else:  # catch up with the plan, no faster than the gate's speed
                    g["x"] = min(plan_x, g["x"] + L.GATE_SPEED / CONTROL_HZ) if plan_x > g["x"] else plan_x
            elif g["started"]:
                g["x"] = None  # the gripped push is over: the plan's own x again
        # -- the left hand: aim each wire move at where that wire's loop is, just before the move starts
        aimed = g.get("aimed", 0)
        if getattr(tr, "live", False) and tr.crest and aimed < len(tr.crest) and t >= (
                steps_plan["left"][0][0] + L.crest_start(aimed) - (0.3 if aimed == 0 else 0.05)):
            tops = L.loop_tops(st.polylines, tuple(set(tr.crest)))
            aims = g.setdefault("aims", [None] * len(tr.crest))
            for k_ in range(aimed, len(tr.crest)):
                aims[k_] = tops[tr.crest[k_]]
            whole = L.trial_plan(dataclasses.replace(tr, pieces=0)) if getattr(tr, "pieces", 0) else steps_plan
            left_new = [whole["left"][0]] + L.live_plan(tr, aims)
            rest_s = L.plan_seconds(whole) - sum(s_[0] for s_ in left_new)
            if rest_s > 1e-9:
                left_new.append((rest_s, None, "open", "ready", "wait"))
            new_way = plan_waypoints({"left": left_new})["left"]
            # Only the wire part is re-aimed: what the plan holds after it (the screw part's waypoint) stays as it is.
            if [w.duration for w in new_way] != [w.duration for w in d["way"]["left"][:len(new_way)]]:
                raise ValueError("the re-aimed left plan's step times differ from the plan the policy holds")
            left.waypoints = [*new_way, *left.waypoints[len(new_way):]]
            g["aimed"] = aimed + 1
        # -- the left hand: follow the wire sideways during a sweep; a lean stops once the loop is near the centre
        if getattr(tr, "track", False):
            lab_l = str(left.target(t)[3])
            if lab_l.startswith(("sweep the ", "lean the ")):
                base_ = environment.unwrapped
                tcp_l = np.asarray(tcp_world(base_.scene["left_robot"]), float) - _np(base_.scene.env_origins)[0]
                tip_l = tcp_l + L.PAD_TIP * (L.crest_rotation(0.0) @ np.array([0.0, 0.0, 1.0]))
                wire_l = np.asarray(st.polylines[lab_l.split()[2]], float)
                if lab_l.startswith("sweep the "):  # keep the fingertips on the wire's own y
                    near = wire_l[np.hypot(wire_l[:, 0] - tip_l[0], wire_l[:, 2] - tip_l[2]) < 0.025]
                    if len(near):
                        corr_l[1] = float(np.clip(corr_l[1] + 0.3 * (float(np.mean(near[:, 1])) - tip_l[1]), -0.03, 0.03))
                elif g.get("lean_hold") is None:
                    top_l = L.loop_tops({"w": wire_l}, ("w",))["w"]
                    if top_l is None or abs(float(top_l[1]) - float(L.OPEN_C[0])) <= L.LEAN_TO + 0.002:
                        g["lean_hold"] = float(tcp_l[1])
            elif lab_l != "hold it there":
                corr_l[:] = 0.0
                g["lean_hold"] = None
        # -- hand the policy the two targets for the step it is about to command
        r_pos, r_rot, r_finger, _ = right.target(t)
        r_pos = np.asarray(r_pos, float) + corr
        if g["x"] is not None:
            r_pos[0] = g["x"]
        if g["twist"]:
            r_rot = rolled(np.asarray(r_rot, float), g["twist"])
        _play("right", t, r_pos, r_rot, r_finger)
        l_pos, l_rot, l_finger, _ = left.target(t)
        l_pos = np.asarray(l_pos, float) + (corr_l if getattr(tr, "track", False) else 0.0)
        if g.get("lean_hold") is not None:
            l_pos[1] = g["lean_hold"]
        _play("left", t, l_pos, l_rot, l_finger)
    except Exception as err:  # noqa: BLE001 - a demo fault must not take the trial down with it; score() will show it
        d["error"] = f"{type(err).__name__}: {err}"


def probe_demo():
    """An early open-loop development variant (OUTLET_PROBE: hang, straight, bend, ...), which scripted_demo.py's
    single-variant mode plays. It is NOT the task's demo and does not reach full score: the demo is closed loop
    (demo() above, with _demo_control)."""
    return {arm: [_waypoint(arm, s) for s in steps] for arm, steps in L.probe_plan(PROBE).items()}
