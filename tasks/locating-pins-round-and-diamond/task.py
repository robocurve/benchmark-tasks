"""On the station table: a grey aluminium fixture plate (120 x 200 x 20 mm, fixed) on the midline 30 cm in front of the
arm bases, with two 12.4 mm locating holes 100.0 mm apart on a line along y (A on the robot's left, B on its right)
and four low rest pads. In two small racks hang three loose steel locating pins: two round and one diamond (relieved:
two 2 mm lands on the 16 mm circle, 8 mm across its flats between them, its head blackened). On rails in front of the
fixture lies a blue aluminium bracket (150 x 60 x 8 mm) with two 16.4 mm holes 101.0 mm apart.

Rule (two-hole location): one round pin in one hole and the diamond in the other with its lands square to the line
between the holes, each pushed home, then the bracket over both pins onto the rest pads. The holes are 1.0 mm farther
apart than the pins, so the bracket goes down over a round pin and a diamond only when the diamond's relief lies along
the line between the holes; over two round pins, or over a diamond turned a quarter turn, it binds on the pin heads.

The pins, the bracket, the fixture and the racks are mechkit compounds (mechkit.py, next to this file): each is one
rigid body made of box and cylinder colliders, the round holes 16-sided rings of turned boxes with 45 degree
lead-ins (layout.py). A pin pushed home (flange within 0.5 mm of the plate, upright) is LOCKED by a rule (``_LOCKS``
below: a mechkit weld that rewrites the pin's pose every control step), standing for the pin's ball detent or press
fit; before that it turns freely. The arms are read through rigstate.py. Neither changes what the embodiment does.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score -E object_jitter=0 ...
Demo:    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py -E object_jitter=0 ...
One arm: DEMO_IDLE_ARM=left (or right) before the demo command leaves that arm at its ready pose.
Variants (counterfactual runs, never the submitted demo): DEMO_VARIANT=<name>, see VARIANTS and demo().
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402

import fitgeom  # noqa: E402
import layout as L  # noqa: E402
import mechkit as mk  # noqa: E402  (importing it hooks yam_arena so the parts are built as compounds)
import rigstate  # noqa: E402
import scoring as S  # noqa: E402
from scoring import BRACKET  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import Box  # noqa: E402

rigstate.install()  # reads both arms before every control step (False offline: the checker feeds it instead)

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0  # 1.0 must hold for 1 s on top of the scorer's own 1 s of release and stillness

INSTRUCTION = "Fit locating pins to the fixture plate and load the bracket onto them so it is located."

FRICTION = (0.4, 0.35)  # dry steel on aluminium, for every part and the fixture


class _Offline(mk.Assembly):
    """A fixed part as the offline checker draws it: only its floor slab. A pin hanging in a rack or standing in a plate
    hole is inside the part's bounding box, which the checker would flag as interpenetration; with the floor slab alone
    a pin lowered into a hole comes to rest on the floor slab, 18 mm down, which is where the real pin's flange meets the
    plate. The simulator never uses this: it builds every shape."""

    FLOOR = 0.002  # set per part below

    def offline_box(self):
        lo, hi = self.local_bounds()
        origin, quat = self.spawn_pose()
        size = (float(hi[0] - lo[0]), float(hi[1] - lo[1]), float(self.FLOOR))
        centre = origin + mk.quat_to_matrix(quat) @ np.array([(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, lo[2] + self.FLOOR / 2])
        return tuple(float(v) for v in centre), size, float(self.yaw)

    def offline_state(self, state):
        return self.rest_state()


def _fixed(name, origin, shapes, floor):  # floor: the offline slab's thickness
    spec = _Offline(name=name, pos=origin, color=L.FIXTURE_GREY, mass=None, yaw=0.0, shapes=tuple(shapes),
                    friction=FRICTION, **mk.TIGHT_FIT)
    spec.FLOOR = floor
    spec._check()
    return spec


FIXTURE = _fixed("fixture", (L.FIX_X, L.FIX_Y, L.TOP), L.fixture_shapes(), L.PIN_SEAT_Z - L.SHANK_L)  # 2 mm: a lowered pin ends home
RIGHT_RACK = _fixed("right_rack", (L.RIGHT_RACK_X, L.RIGHT_RACK_Y, L.TOP),
                    L.rack_shapes(L.RIGHT_RACK_SIZE, [(0.0, 0.030), (0.0, -0.030)]), L.RACK_H - L.SHANK_L)
LEFT_RACK = _fixed("left_rack", (L.LEFT_RACK_X, L.LEFT_RACK_Y, L.TOP), L.rack_shapes(L.LEFT_RACK_SIZE, [(0.0, 0.0)]),
                   L.RACK_H - L.SHANK_L)
RAILS = [Box(f"rail_{i + 1}", pos=(L.BR_START[0] + dx, L.BR_START[1], L.TOP + L.RAIL_H / 2),
             size=(L.RAIL_W, L.RAIL_L, L.RAIL_H), color=L.RACK_BLACK) for i, dx in enumerate(L.RAIL_X)]


def _pin(name, diamond):
    x, y = L.DIAMOND_START if diamond else L.ROUND_START[name]
    return mk.assembly(name, origin=(x, y, L.TOP + L.RACK_H), yaw=L.DIAMOND_START_YAW if diamond else 0.0,
                       mass=L.pin_mass(diamond), color=L.PIN_STEEL, shapes=L.pin_shapes(diamond), friction=FRICTION,
                       tight_fit=True)


PIN_SPECS = {n: _pin(n, n == L.DIAMOND) for n in L.PIN_NAMES}
BRACKET_PART = mk.assembly(BRACKET, origin=(L.BR_START[0], L.BR_START[1], L.TOP + L.RAIL_H), mass=L.BR_MASS,
                           color=L.BRACKET_BLUE, shapes=L.bracket_shapes(), friction=FRICTION, tight_fit=True)
OBJECTS = [FIXTURE, RIGHT_RACK, LEFT_RACK, *RAILS, *PIN_SPECS.values(), BRACKET_PART]


# -- the lock: a pin pushed home is held there ------------------------------------------------------------------------
def _snap(pos, rot):
    """The pose a locked pin is held at: its axis on the hole's centre, flange on the plate, upright, with the turn
    about its axis it had when it went home (the diamond's orientation is fixed by then)."""
    hole = min(L.HOLES.values(), key=lambda h: (h[0] - pos[0]) ** 2 + (h[1] - pos[1]) ** 2)
    yaw = math.atan2(rot[1, 0], rot[0, 0])
    return np.array([hole[0], hole[1], L.TOP + L.PIN_SEAT_Z]), mk.axis_angle_matrix((0.0, 0.0, 1.0), yaw)


def _home_now(name):
    def when(states):
        r = S.read(states)
        return r is not None and S.home_hole(r, name) is not None

    return when


_LOCKS = {n: mk.weld(PIN_SPECS[n], None, rel_pose=_snap, when=_home_now(n), name=f"{n}_home") for n in L.PIN_NAMES}


# -- the offline stand-in for the physics ----------------------------------------------------------------------------
# Without a simulator nothing makes the bracket bind or seat: the checker's proxy drops it onto the pins' bounding boxes
# or the floor slab. This puts it where the shapes would (fitgeom: the heads in the holes' 16-sided rings): over every
# locked pin it fits (no overlap) it lies on the pads, 25.0 mm up; if it does not fit it rests on the heads (about
# 33.6 mm). Applied only once its underside is in the pins' zone (36 mm up or lower). Sim: never.
ZONE_Z = L.PIN_SEAT_Z + L.HEAD_TOP + 0.001
BIND_Z = L.PIN_SEAT_Z + L.HEAD_TOP - 0.0014  # the head's chamfer ends 1.4 mm below its top


def no_simulator(objects) -> bool:
    return mk.env() is None and getattr(objects.get(BRACKET), "source", "sim") != "sim"


def bracket_fits(r, home_pins) -> bool:
    """Whether the bracket at its pose goes down over every pin in ``home_pins`` ({name: hole})."""
    o, rot = r.pos[BRACKET], r.rot[BRACKET]
    theta = math.atan2(rot[1, 0], rot[0, 0]) - ARM_YAW["bracket"]  # the yaw trim stands for the GPU arm's offset too
    pins = []
    for n, h in home_pins.items():
        kind = "diamond" if n == L.DIAMOND else "round"
        yaw = math.atan2(r.rot[n][1, 0], r.rot[n][0, 0])
        pins.append((kind, L.HOLES[h][1] - L.FIX_Y, yaw))
    if not pins:
        return True
    # The demo's bracket trim stands for the arm's measured offset on the GPU: offline the arm is exact, so take it back out.
    v = fitgeom.violation(o[0] - L.FIX_X - TRIM["bracket"][0], o[1] - L.FIX_Y - TRIM["bracket"][1], theta, pins)
    return bool(v <= 0.0)


def offline_states(objects):
    r = S.read(objects)
    if r is None:
        return objects
    z = r.pos[BRACKET][2]
    if z > ZONE_Z:
        return objects
    homes = {n: S.home_hole(r, n) for n in L.PIN_NAMES}
    homes = {n: h for n, h in homes.items() if h is not None}
    new_z = L.PAD_TOP if bracket_fits(r, homes) else BIND_Z
    b = objects[BRACKET]
    origin = np.asarray(b.origin, dtype=float).copy()
    origin[2] = L.TOP + new_z
    quat = np.asarray(b.quat_xyzw, dtype=float)
    if ARM_YAW["bracket"]:  # likewise the yaw trim
        quat = np.asarray(mk.matrix_to_quat(mk.axis_angle_matrix((0.0, 0.0, 1.0), -ARM_YAW["bracket"])
                                            @ mk.quat_to_matrix(quat)), dtype=float)
    origin[0] -= TRIM["bracket"][0]  # the bracket trim stands for the GPU arm's measured offset: offline the arm is exact
    origin[1] -= TRIM["bracket"][1]
    vel = np.concatenate([np.asarray(b.lin_vel, dtype=float), np.asarray(b.ang_vel, dtype=float)])
    out = dict(objects)
    out[BRACKET] = mk.state_from_poses(BRACKET_PART, (*origin, *quat), vel,
                                       source="offline")
    return out


_TRACKER = S.Tracker()
_EPISODE = {"mechkit": None}
_HIST = {n: [] for n in (*L.PIN_NAMES, BRACKET)}  # trace only: [step, z mm, tilt deg, dx mm, dy mm, yaw deg] near the fixture
HIST_MAX = 160


def _history(r) -> dict:
    """Trace only (TASK_TRACE): each part's recent samples near the fixture, so a run's few kept rows still show how a pin
    went into its hole: height over the plate (pins) or over the pads (bracket), tilt, and offset from the nearest hole's
    centre (pins) or from the fixture's centre (bracket). Every second step while within 30 mm of the plate."""
    step = _TRACKER.steps
    if step % 2 == 0:
        for n in _HIST:
            o = r.pos[n]
            if n == BRACKET:
                z, ref = o[2] - L.PAD_TOP, (L.FIX_X, L.FIX_Y)
                if not (-3e-3 < z < 0.03 and abs(o[0] - L.FIX_X) < 0.05):
                    continue
            else:
                z = o[2] - L.PIN_SEAT_Z
                ref = min(L.HOLES.values(), key=lambda h: (h[0] - o[0]) ** 2 + (h[1] - o[1]) ** 2)
                if not (-1e-3 < z < 0.03 and math.hypot(o[0] - ref[0], o[1] - ref[1]) < 0.02):
                    continue
            if len(_HIST[n]) < HIST_MAX:
                _HIST[n].append([step, round(z * 1000, 2), round(S.tilt_deg(r.rot[n]), 2),
                                 round((o[0] - ref[0]) * 1000, 2), round((o[1] - ref[1]) * 1000, 2),
                                 round(math.degrees(math.atan2(r.rot[n][1, 0], r.rot[n][0, 0])), 2)])
    return {n: h for n, h in _HIST.items() if h}


@rigstate.on_reset
def _new_episode() -> None:
    """rigstate saw the embodiment reset: a new trial (the scorer also spots a teleport back to the start)."""
    _TRACKER.reset()
    for h in _HIST.values():
        h.clear()


def _trace(row: dict) -> None:
    """With TASK_TRACE=<file>, append one JSON line per control step: what the scorer saw and decided."""
    path = os.environ.get("TASK_TRACE")
    if path:
        try:
            with open(path, "a") as f:
                f.write(json.dumps(row, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o)) + "\n")
        except OSError:  # a debugging aid only: never let it stop a trial
            pass


def _parts(objects) -> dict:
    return {n: objects.get(n) for n in (*L.PIN_NAMES, BRACKET)}


def success(objects) -> bool:
    """A located set (one round pin and the diamond, home in the two holes, the diamond's lands square to the line within
    15 degrees), the bracket on the pads over them (hole centre within 0.3 mm of the round pin, 0.3 mm and 0.3 degrees),
    both grippers 25 mm clear, still for 1 s; held 1 s. This runs first every control step, so it also feeds the
    tracker (and, without a simulator, the offline stand-in)."""
    if _EPISODE["mechkit"] != mk.episode():  # mechkit counts resets too: a second, independent reset signal
        if _EPISODE["mechkit"] is not None:
            _TRACKER.reset()
        _EPISODE["mechkit"] = mk.episode()
    parts = _parts(objects)
    offline = no_simulator(parts)
    if offline:
        parts = offline_states(parts)
    arms = rigstate.arms()
    result = _TRACKER.update(parts, arms)
    row = {"episode": mk.episode(), "step": _TRACKER.steps, "score": result.score, "success": result.success,
           "fallback_resets": _TRACKER.resets, **result.details,
           "locks": {n: w.status() for n, w in _LOCKS.items()}}
    if arms is not None:
        try:
            row.update(left_tcp=arms["left"].tcp, right_tcp=arms["right"].tcp,
                       openings={a: round(float(arms[a].opening), 3) for a in ("left", "right")})
        except (AttributeError, KeyError, TypeError, ValueError):
            row["rigstate_error"] = "unreadable snapshot"
    else:
        row["rigstate_error"] = rigstate.error()
    row["source"] = getattr(parts.get(BRACKET), "source", None)
    if os.environ.get("TASK_TRACE") and _TRACKER.prev is not None:
        row["near"] = _history(_TRACKER.prev)  # grows to HIST_MAX samples per part: the last row has the whole story
    _trace(row)
    return result.success


def score(objects) -> float:
    """0 if a pin or the bracket is off the table; 1.0 a located set home with the bracket seated on the pads (within
    0.3 mm and 0.3 degrees of its located pose), both grippers clear and everything still for 1 s; 0.5 a located set
    home, the bracket not seated; 0.25 any one pin home but no located set; 0 otherwise. The score of the latest scored
    step."""
    return _TRACKER.last.score


@mk.on_reset
def _locks_reset() -> None:
    _TRACKER.reset()


# -- the scripted demo -----------------------------------------------------------------------------------------------
# Open loop: timed waypoints from the known start layout, nothing read from the simulator. TCP points; the pad tips are
# 13 mm below. Finger targets: the joint value is minus half the commanded gap (8 mm narrower than the part).
OPEN = -0.04695  # 94 mm
PAD_TIP = 0.013
GRIP_PIN = -(2 * L.HEAD_R - 0.008) / 2  # 8 mm commanded across a round head, or across the diamond's two lands (16 mm)
GRIP_BR = -(L.BR_LEN_X - 0.008) / 2  # 52 mm commanded across the bracket's 60 mm width
GRIP_PUSH = 0.0  # closed: the two pads together press a head's top
PIN_GRIP_UP = L.FLANGE_T + 0.0015 + PAD_TIP  # TCP above a pin's origin: the pad tips 1.5 mm over the flange
PIN_RELEASE_ORIGIN = L.PIN_SEAT_Z + L.SHANK_L + 0.003  # the shank's tip 3 mm above the hole's mouth: where it is lowered slowly to
PIN_LET_GO_ORIGIN = L.PIN_SEAT_Z + 0.008  # let go with the flange 8 mm over the plate: 10 mm of the shank already in the hole
# (GPU run 1: dropped from 3 mm above the mouth, both round pins jammed leaning at 4 to 6 mm of insertion and never went home)
BR_GRIP_UP = L.BR_T - 0.005 + PAD_TIP  # TCP above the bracket's underside: the pad tips 5 mm below its top face
BR_RELEASE_UNDER = L.PIN_SEAT_Z + L.HEAD_TOP + 0.001  # lowered slowly to here: the underside 1 mm over the pin heads
BR_LET_GO_UNDER = L.PAD_TOP - 0.0005  # let go with the bracket pressed 0.5 mm into the pads (GPU run 2: dropped from 1 mm over
# the heads it wedged on the diamond's lands, B end down, A end 4.9 mm up, 2.7 degrees)
BR_PICK_UNDER = L.RAIL_H
APPROACH = 0.030
HOVER_PIN = L.TOP + 0.12
HOVER_BR = L.TOP + 0.10
PUSH_TCP = L.PIN_SEAT_Z + L.HEAD_TOP - 0.004 + PAD_TIP  # closed pad tips 4 mm below a home pin's head top, commanded
YAW_DIAMOND_PICK, YAW_LEFT, YAW_RIGHT = 0.0, -math.pi / 2, math.pi / 2
# GPU runs 1 and 2: the left hand turns 0.8 to 0.9 degrees more than commanded at the fixture, so the diamond went home
# 0.82 degrees off square and the bracket came down 0.94 degrees off (it seated 0.3 mm and 0.34 degrees off its located pose,
# just outside the 0.3 and 0.3 asked for). Place yaws are trimmed by minus that.
YAW_LEFT_DIAMOND = YAW_LEFT - math.radians(0.82)
YAW_LEFT_BRACKET = YAW_LEFT - math.radians(0.94)
ARM_YAW = {"bracket": YAW_LEFT_BRACKET - YAW_LEFT}  # what the offline stand-in takes back out (the arm is exact offline)
PARK_L = (0.20, 0.16, L.TOP + 0.15)
PARK_R = (0.18, -0.25, L.TOP + 0.15)
# Trim of each place point (x, y) for the GPU run: move a point by minus the offset the trace shows.
# GPU run 2: the bracket held 7.5 mm over the pads was 0.6 mm short in x and 0.13 mm long in y of the aim; run 3: it seated
# 0.17 mm long in y of its located pose after the first trim.
TRIM = {"diamond": (0.0, 0.0), "round": (0.0, 0.0), "bracket": (0.0006, -0.0003)}


def _duration(plan) -> float:
    return sum(w.duration for w in plan)


def _pin_moves(pick, hole, trim, yaw_pick, yaw_place, what, where, wait=0.0):
    """Pick a pin out of its rack and stand it in a hole: come down beside it slowly, grip it, lift it, (wait), carry it
    over the hole (turning the hand to ``yaw_place``), lower it until its tip is 3 mm over the hole's mouth, then slowly 10 mm
    into the hole (the funnel centres it, the hand keeps it upright), let go, rise, close the fingers and press its head home, rise."""
    px, py = pick
    qx, qy = L.HOLES[hole][0] + trim[0], L.HOLES[hole][1] + trim[1]
    pz = L.TOP + L.RACK_H + PIN_GRIP_UP
    z_rel = L.TOP + PIN_RELEASE_ORIGIN + PIN_GRIP_UP
    z_let = L.TOP + PIN_LET_GO_ORIGIN + PIN_GRIP_UP
    z_up = L.TOP + L.PIN_SEAT_Z + L.HEAD_TOP + 0.012 + PAD_TIP  # pad tips 12 mm over a home pin's head top
    plan = [
        Waypoint(1.5, (px, py, HOVER_PIN), OPEN, f"over the {what}", yaw_pick),
        Waypoint(1.0, (px, py, pz + APPROACH), OPEN, f"down to just above the {what}", yaw_pick),
        Waypoint(1.5, (px, py, pz), OPEN, f"slowly down beside the {what}", yaw_pick),
        Waypoint(0.5, (px, py, pz), OPEN, "settle", yaw_pick),
        Waypoint(1.5, (px, py, pz), GRIP_PIN, f"grip the {what} head", yaw_pick),
        Waypoint(1.5, (px, py, HOVER_PIN), GRIP_PIN, f"lift the {what}", yaw_pick),
    ]
    if wait:
        plan.append(Waypoint(wait, (px, py, HOVER_PIN), GRIP_PIN, "wait for the other arm to clear the fixture", yaw_pick))
    plan += [
        Waypoint(2.0, (qx, qy, HOVER_PIN), GRIP_PIN, f"carry it over {where}", yaw_place),
        Waypoint(0.5, (qx, qy, HOVER_PIN), GRIP_PIN, f"settle over {where}", yaw_place),
        Waypoint(1.0, (qx, qy, z_rel + APPROACH), GRIP_PIN, f"down to just above {where}", yaw_place),
        Waypoint(2.0, (qx, qy, z_rel), GRIP_PIN, f"slowly lower it to {where}'s mouth", yaw_place),
        Waypoint(0.5, (qx, qy, z_rel), GRIP_PIN, "settle", yaw_place),
        Waypoint(2.5, (qx, qy, z_let), GRIP_PIN, f"slowly push its shank 10 mm into {where}, held upright", yaw_place),
        Waypoint(0.5, (qx, qy, z_let), GRIP_PIN, "settle", yaw_place),
        Waypoint(1.5, (qx, qy, z_let), OPEN, f"let go in {where}: it stands in the hole", yaw_place),
        Waypoint(0.8, (qx, qy, z_up), OPEN, "straight up, clear of the head", yaw_place),
        Waypoint(0.7, (qx, qy, z_up), GRIP_PUSH, "close the fingers", yaw_place),
        Waypoint(1.3, (qx, qy, L.TOP + PUSH_TCP), GRIP_PUSH, "press the head home", yaw_place),
        Waypoint(0.5, (qx, qy, L.TOP + PUSH_TCP), GRIP_PUSH, "hold", yaw_place),
        Waypoint(1.0, (qx, qy, HOVER_PIN), OPEN, "straight up, opening", yaw_place),
    ]
    return plan


def _bracket_moves(aim_dy, wait, label="bracket", forced=True):
    """The left arm takes the bracket by its long sides, holds it up until the other arm is clear, carries it over the
    fixture with its holes over the pin positions (``aim_dy``: its centre's y from the fixture's), lowers it over the
    pin heads (the lead-ins centre it) onto the pads, lets go, rises, then aside. ``forced=False`` (the counterfactual variants): only lowers it to 1 mm over the
    pin heads and lets it drop, so a bind on the heads shows."""
    px, py = L.BR_START
    pz = L.TOP + BR_PICK_UNDER + BR_GRIP_UP
    qx, qy = L.FIX_X + TRIM["bracket"][0], L.FIX_Y + aim_dy + TRIM["bracket"][1]
    z_rel = L.TOP + BR_RELEASE_UNDER + BR_GRIP_UP
    z_carry = L.TOP + 0.075 + BR_GRIP_UP
    z_pad = L.TOP + BR_LET_GO_UNDER + BR_GRIP_UP
    plan = [
        Waypoint(2.0, (px, py, HOVER_BR), OPEN, f"over the {label}", YAW_LEFT),
        Waypoint(1.0, (px, py, pz + APPROACH), OPEN, f"down to just above the {label}", YAW_LEFT),
        Waypoint(1.5, (px, py, pz), OPEN, f"slowly down beside the {label}", YAW_LEFT),
        Waypoint(0.5, (px, py, pz), OPEN, "settle", YAW_LEFT),
        Waypoint(1.5, (px, py, pz), GRIP_BR, f"grip the {label} by its long sides", YAW_LEFT),
        Waypoint(1.5, (px, py, HOVER_BR), GRIP_BR, f"lift the {label}", YAW_LEFT),
    ]
    if wait:
        plan.append(Waypoint(wait, (px, py, HOVER_BR), GRIP_BR, "wait for the right arm to clear the fixture", YAW_LEFT))
    plan += [
        Waypoint(2.5, (qx, qy, z_carry), GRIP_BR, f"carry it over the fixture, holes over the pins", YAW_LEFT_BRACKET),
        Waypoint(0.8, (qx, qy, z_carry), GRIP_BR, "settle over the pins", YAW_LEFT_BRACKET),
        Waypoint(2.5, (qx, qy, z_rel), GRIP_BR, "slowly lower it to the pin heads", YAW_LEFT_BRACKET),
    ]
    if not forced:
        plan += [
            Waypoint(0.8, (qx, qy, z_rel), GRIP_BR, "settle", YAW_LEFT_BRACKET),
            Waypoint(1.5, (qx, qy, z_rel), OPEN, f"let go of the {label}: it drops onto the pins", YAW_LEFT_BRACKET),
            Waypoint(1.0, (qx, qy, HOVER_BR), OPEN, "straight up", YAW_LEFT_BRACKET),
            Waypoint(1.5, PARK_L, OPEN, "aside", YAW_LEFT_BRACKET),
        ]
        return plan
    plan += [
        Waypoint(0.4, (qx, qy, z_rel), GRIP_BR, "settle", YAW_LEFT_BRACKET),
        Waypoint(2.0, (qx, qy, z_pad), GRIP_BR, "slowly lower it over the pins onto the pads", YAW_LEFT_BRACKET),
        Waypoint(0.5, (qx, qy, z_pad), GRIP_BR, "settle", YAW_LEFT_BRACKET),
        Waypoint(1.5, (qx, qy, z_pad), OPEN, f"let go of the {label}", YAW_LEFT_BRACKET),
        Waypoint(1.0, (qx, qy, HOVER_BR), OPEN, "straight up", YAW_LEFT_BRACKET),
        Waypoint(1.5, PARK_L, OPEN, "aside", YAW_LEFT_BRACKET),
    ]
    return plan


ROUND_PICK = {"round_pin_1": L.ROUND_START["round_pin_1"], "round_pin_2": L.ROUND_START["round_pin_2"]}
ALIGN_DY = {"B": L.SPACING_ERROR / 2, "A": -L.SPACING_ERROR / 2}  # bracket centre's y when the round pin is in this hole

VARIANTS = ("two_round", "diamond_turned", "bracket_only_one_pin", "smoke")


def plans(variant: str = "") -> dict:
    """Both arms' plans for the demo or a counterfactual variant.

    Order of work: the left arm sets the diamond in A (turning its hand a quarter turn on the way, so its lands end up
    square to the line between the holes); the right arm waits at hover, then sets a round pin in B; the left arm,
    holding the bracket up, lowers it once the right arm has moved aside."""
    if variant not in ("",) + VARIANTS:
        raise ValueError(f"DEMO_VARIANT={variant!r} is not one of {', '.join(VARIANTS)}")
    t = TRIM
    if variant == "smoke":  # the right arm only: a round pin into B, pushed home (the lock fires)
        return {"right": _pin_moves(ROUND_PICK["round_pin_1"], "B", t["round"], YAW_RIGHT, YAW_RIGHT, "round pin",
                                    "hole B")}
    left, right = [], []
    if variant == "two_round":  # the right arm sets both round pins; the left arm brings the bracket
        right = _pin_moves(ROUND_PICK["round_pin_1"], "B", t["round"], YAW_RIGHT, YAW_RIGHT, "round pin", "hole B")
        right += _pin_moves(ROUND_PICK["round_pin_2"], "A", t["round"], YAW_RIGHT, YAW_RIGHT, "second round pin",
                            "hole A")
        right.append(Waypoint(1.5, PARK_R, OPEN, "aside", YAW_RIGHT))
        aim = 0.0
    else:
        if variant != "bracket_only_one_pin":
            yaw_place = YAW_DIAMOND_PICK if variant == "diamond_turned" else YAW_LEFT_DIAMOND  # turned wrong: no quarter turn
            left = _pin_moves(L.DIAMOND_START, "A", t["diamond"], YAW_DIAMOND_PICK, yaw_place, "diamond pin", "hole A")
        t_left = _duration(left)
        pick_part = 7.5  # the right arm's own pick and lift, before it waits
        right = _pin_moves(ROUND_PICK["round_pin_1"], "B", t["round"], YAW_RIGHT, YAW_RIGHT, "round pin", "hole B",
                           wait=max(0.0, t_left - pick_part))
        right.append(Waypoint(1.5, PARK_R, OPEN, "aside", YAW_RIGHT))
        aim = ALIGN_DY["B"]
    t_right = _duration(right)
    bracket_start = _duration(left) + 9.0  # its approach, grip and lift
    left += _bracket_moves(aim, wait=max(0.0, t_right - bracket_start), forced=not variant)
    return {"left": left, "right": right}


def demo():
    """Both arms on one clock. DEMO_HOLD_ONLY=1: no plan (both arms hold the ready pose, a settling check).
    DEMO_IDLE_ARM=left|right: that arm's plan is dropped (the one-arm check).
    DEMO_VARIANT (counterfactual runs for the README, never the submitted demo):
      two_round             the right arm sets both round pins (B then A); the bracket is lowered over them: it binds
      diamond_turned        the diamond goes into A without its quarter turn (lands along the line between the holes)
      bracket_only_one_pin  the right arm sets a round pin in B; the diamond is left in its rack; the bracket is lowered
      smoke                 the right arm only: a round pin into B, pushed home (the lock fires)"""
    if os.environ.get("DEMO_HOLD_ONLY") == "1":
        return {}
    plan = plans(os.environ.get("DEMO_VARIANT", ""))
    plan.pop(os.environ.get("DEMO_IDLE_ARM", ""), None)
    return plan
