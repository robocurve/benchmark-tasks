"""Lock a castle nut with a cotter pin: push the pin through the nut's slot and the stud's cross-hole, then spread its
two prongs apart round the nut with a flat-head screwdriver.

On the station table a steering arm (a steel plate on a block) carries an upright 3/8-24 stud with a slotted hex nut on
it, one slot lined up over the stud's cross-hole and running along x (front to back). A new 1 in steel cotter pin (its
wire as an ASME B18.8.1 1/16 in pin's, its prongs 0.7 mm apart) lies flat in a groove along the top of an aluminium
block in front of the right arm, prongs towards the robot, its eye sticking out past the block's far end; a precision
flat-head screwdriver (a thin wedge-shaped blade, 0.4 mm at its edge) stands in a rack in front of the left arm.
Mechanics finish a castle nut on a steering linkage this way: a new cotter pin through the slot and the hole, its
prongs spread so it can't come out (Suzuki RA410 service manual: "bend new split pin"; Meritor: tighten to the next
slot, never back the nut off).

What makes it hard: the pin (2.2 mm across) has to go in square through a 2.8 mm hole hidden behind the slot, and its
two prongs, 0.75 mm thick each, stand only 0.7 mm apart: a parallel gripper can't take hold of one, nor squeeze a prong
hard enough to bend it. The screwdriver's thin blade goes down between the prongs and levers each one round, then
pushes it on round against the nut from beyond its tip; meanwhile the pin's head has to be held home. Each prong takes about 6 N, 8 mm from its root, to bend (cotterkit.py).

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
Demo:    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py -E object_jitter=0 ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from scenekit import Shape, UsdBody, on_reset, on_step  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

import cotterkit  # noqa: E402
import touch  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0

INSTRUCTION = "Lock the castle nut with the cotter pin: push it through the nut's slot and the stud's hole, then spread its prongs apart."

MM = 0.001
TOP = TABLE_TOP[SCENE]

# -- sizes, as work/cotter/make_cotter.py builds them ------------------------------------------------------------
NUT_AF, NUT_H = 14.29 * MM, 8.33 * MM  # 3/8 in slotted hex nut, flats facing +-x
SLOT_W, SLOT_D = 3.0 * MM, 3.2 * MM
STUD_R = 9.53 * MM / 2
HOLE = 2.8 * MM
ARM_TOP = 0.040
NUT_TOP = ARM_TOP + NUT_H
SLOT_FLOOR = NUT_TOP - SLOT_D
HOLE_Z = SLOT_FLOOR + HOLE / 2  # above the table
HOME_X = NUT_AF / 2  # the prongs' roots (the cotter's origin) with its head against the near flat
PRONG_T, PRONG_W = 0.75 * MM, 1.5 * MM  # each prong's thickness (y, across the split) and width (z)
PRONG_GAP = 0.7 * MM  # the prongs stand this far apart all along (a screwdriver's edge goes between them)
PRONG_YC = PRONG_GAP / 2 + PRONG_T / 2  # each prong's centre line from the pin's axis
PIN_W = PRONG_GAP + 2 * PRONG_T  # the pin's width: 2.2 mm
INSIDE = NUT_AF  # the shank from under the head to the prongs' roots
EYE_D = 5.0 * MM  # the eye lies flat: this long (x) and wide (y), PRONG_W thick
PRONG_L = 25.4 * MM - INSIDE  # the +y (short) prong beyond its root; the -y one is 1 mm longer

# -- the scene --------------------------------------------------------------------------------------------------
NUT_XY = (0.300, 0.0)
NX, NY = NUT_XY
PIN_DIR = -1.0  # the pin goes in pointing along -x (in from the far side, prongs towards the robot): a hand holding
# the screwdriver nearly flat along the prongs then points it forwards (+x), which the left arm reaches; pointing it
# back at the robot (-x) it can't
TURNED = (0.0, 0.0, 1.0, 0.0)  # the fixture and the pin, turned half round about z from their files (both along +x)
FIXTURE = UsdBody("fixture", usd="assets/fixture.usda", pos=(NX, NY, TOP), rot_xyzw=TURNED, size=(0.08, 0.04, 0.05),
                  mass=None, material=False)
COTTER_XY = (0.300, -0.120)  # its origin (the prongs' roots); it lies flat, pointing along -x, head towards +x

# the new pin waits in a groove along the top of a 20 x 20 x 60 mm aluminium block, its eye sticking out past the
# block's end, so a hand can close round the eye from above
BLOCK_H, BLOCK_W, BLOCK_L = 0.020, 0.020, 0.060
GROOVE_W, GROOVE_D = 3.0 * MM, 1.0 * MM
BLOCK_END = COTTER_XY[0] - PIN_DIR * float(os.environ.get("BLOCK_OUT", "0.006"))  # the block's far (+x) end: the eye and 8.3 mm of shank stick out past
# it (the pin's centre of mass, 2.9 mm behind the prongs' roots, stays 3 mm in from the end)
ALU = (0.75, 0.76, 0.78)
_bx = BLOCK_END + PIN_DIR * BLOCK_L / 2
_rail = (BLOCK_W - GROOVE_W) / 2
BLOCK = [
    Shape("block_base", pos=(_bx, COTTER_XY[1], TOP + (BLOCK_H - GROOVE_D) / 2), size=(BLOCK_L, BLOCK_W, BLOCK_H - GROOVE_D),
          color=ALU),
    *(Shape(f"block_rail{i}", pos=(_bx, COTTER_XY[1] + s * (GROOVE_W + _rail) / 2, TOP + BLOCK_H - GROOVE_D / 2),
            size=(BLOCK_L, _rail, GROOVE_D), color=ALU) for i, s in enumerate((-1, 1))),
]
COTTER_START = (COTTER_XY[0], COTTER_XY[1], TOP + BLOCK_H - GROOVE_D + PRONG_W / 2 + 0.0002)  # in the groove
COTTER = UsdBody("cotter", usd="assets/cotter.usda", pos=COTTER_START, rot_xyzw=TURNED, size=(0.03, 0.005, 0.002),
                 articulation=True, material=False, solver_iters=32, joint_stiffness=cotterkit.K,
                 joint_damping=cotterkit.DAMPING, joint_pos=tuple((j, 0.0) for j in cotterkit.JOINTS))

# the screwdriver (assets/screwdriver.usda, work/cotter/make_cotter.py: a slim precision flat-head, 120 mm overall: a
# 40 mm blade, as thin as its 2.5 x 0.4 mm edge for 8 mm, then tapering to 4 x 3 mm, then an 80 mm handle; origin at the edge's middle, z
# along it), standing tip down in a rack (a 30 mm block with a 4 mm square hole) in front of the left arm, its edge
# along x
TOOL_MASS = 0.03
RACK_FLOOR, RACK_H, RACK_HOLE, RACK_W = 0.003, 0.025, 0.004, 0.030
RACK_XY = (0.220, 0.180)
TOOL_START = (RACK_XY[0], RACK_XY[1], TOP + RACK_FLOOR + 0.0003)
GREY = (0.45, 0.45, 0.47)
_side = (RACK_W - RACK_HOLE) / 2
RACK = [
    Shape("rack_floor", pos=(*RACK_XY, TOP + RACK_FLOOR / 2), size=(RACK_W, RACK_W, RACK_FLOOR), color=GREY),
    *(Shape(f"rack_wall{i}", pos=(RACK_XY[0] + sx * (RACK_HOLE + _side) / 2, RACK_XY[1] + sy * (RACK_HOLE + _side) / 2,
                                  TOP + RACK_FLOOR + RACK_H / 2),
            size=(_side if sx else RACK_W, _side if sy else RACK_HOLE, RACK_H), color=GREY)
      for i, (sx, sy) in enumerate(((-1, 0), (1, 0), (0, -1), (0, 1)))),
]
TOOL = UsdBody("screwdriver", usd="assets/screwdriver.usda", pos=TOOL_START, size=(0.018, 0.018, 0.12),
               mass=TOOL_MASS, material=False, torsional_patch=0.009)

OBJECTS = [FIXTURE, *BLOCK, COTTER, TOOL, *RACK]

# -- scoring ----------------------------------------------------------------------------------------------------
# the prongs' spread, from an overhead photo: each prong's angle outwards from the pin's axis, and the angle between them
LOCKED = math.radians(90.0)  # the pin counts as locked with its prongs at least this far apart...
EACH = math.radians(30.0)  # ... each of them bent at least this far out (both spread, not one)
PART = math.radians(45.0)  # partly spread: at least this far apart
HOME_GAP = 1.0 * MM  # the head counts as home within this of the nut's near flat (a 1 mm feeler gauge won't go in)
AXIS_TOL = 1.5 * MM  # the pin's axis within this of the hole's (it is in the hole: it can't be further)
TOOL_CLEAR = 0.020  # at the end the screwdriver's tip at least this far from the prongs' roots
_STATE: dict = {"plastic": None, "free": True, "t": 0.0, "last": -1.0}


def root_rel(c) -> np.ndarray:
    """The cotter's origin (its prongs' roots) relative to the stud's axis at the cross-hole's height: along the way
    the pin goes in (PIN_DIR * x), y, z."""
    r = np.asarray(c.pos) - np.array([NX, NY, TOP + HOLE_Z])
    return np.array([PIN_DIR * r[0], r[1], r[2]])


def through(c) -> bool:
    """The pin passes through the stud's cross-hole: pointing the way it goes in (-x), on the hole's axis, its shank
    spanning the stud (prong roots beyond the stud's robot side, the head on the far side)."""
    r = root_rel(c)
    along = c.rotation()[:, 0]
    return bool(PIN_DIR * along[0] > math.cos(math.radians(15)) and abs(r[1]) < AXIS_TOL and abs(r[2]) < AXIS_TOL
                and STUD_R < r[0] < INSIDE - STUD_R)


def spread(c) -> tuple[float, float]:
    """Each prong's bend outwards (rad): +y prong, -y prong."""
    return tuple(cotterkit.outward(j, c.joints.get(j, 0.0)) for j in cotterkit.JOINTS)


def score(objects) -> float:
    """0: the pin is not through the stud's cross-hole.
    0.25: through the hole; its prongs less than 45 degrees apart.
    0.5: the prongs at least 45 degrees apart, but less than 90, or one of them bent out less than 30 degrees.
    0.75: the prongs at least 90 degrees apart, each bent out at least 30, but the head more than 1 mm from the nut,
    or a gripper still on the pin, or the screwdriver's tip within 20 mm of the prongs' roots.
    1.0: spread so, the head within 1 mm of the nut, the robot and the screwdriver off the pin."""
    c = objects["cotter"]
    if not through(c):
        return 0.0
    p, n = spread(c)
    if p + n < PART:
        return 0.25
    if p + n < LOCKED or min(p, n) < EACH:
        return 0.5
    home = HOME_X - root_rel(c)[0] < HOME_GAP  # the eye can't go past home: it is wider than the slot
    tool = objects.get("screwdriver")
    clear = tool is None or float(np.linalg.norm(np.asarray(tool.pos) - np.asarray(c.pos))) > TOOL_CLEAR
    return 1.0 if home and clear and _STATE["free"] else 0.75


def success(objects) -> bool:
    return score(objects) == 1.0


@on_reset
def _reset(environment) -> None:
    cotter = environment.unwrapped.scene["cotter"]
    if _STATE["plastic"] is None or _STATE["plastic"].cotter is not cotter:
        _STATE["plastic"] = cotterkit.Plastic(cotter)
    else:
        _STATE["plastic"].reset()
    touch.reset()
    _STATE.update(free=True, t=0.0, last=-1.0)


def _aim_live(environment) -> None:
    """The scripted demo only: move the screwdriver's next pass at the prongs (cotterkit.LIVE, set by demo()) with the
    pin, from where it should sit to where it sits, until the blade is down in the gap; and while it holds still over
    the gap and goes down, take out what's left sideways between where its tip is and where it should be (its sway in
    the hand and the arm's own error, a few tenths of a mm either way, and up to 0.8 mm on the way down: the gap leaves
    0.15 mm a side)."""
    passes = cotterkit.LIVE.get("passes")
    if not passes or all(_STATE["t"] >= p["t_freeze"] for p in passes):
        return
    from yam_arena.scene import object_states

    s = object_states(environment, ["cotter", "screwdriver"])
    c = s["cotter"]
    if not through(c):
        return
    tool_tip = np.asarray(s["screwdriver"].pos, dtype=float)
    ax = c.rotation()[:, 0]
    turn = math.atan2(ax[1], ax[0]) - math.atan2(0.0, PIN_DIR)  # the pin's yaw from pointing along PIN_DIR
    turn = (turn + math.pi) % (2 * math.pi) - math.pi
    rx, ry = (float(v) for v in np.asarray(c.pos)[:2])
    x0, y0 = cotterkit.LIVE["root0"]
    tool_at, back = cotterkit.LIVE["tool_at"], cotterkit.LIVE["back"]
    cs, sn = math.cos(turn), math.sin(turn)
    for p in passes:
        if _STATE["t"] >= p["t_freeze"]:
            continue

        def moved(tip):
            dx, dy = tip[0] - x0, tip[1] - y0
            return np.array([rx + cs * dx - sn * dy, ry + sn * dx + cs * dy, tip[2]])

        corr = p.setdefault("corr", np.zeros(3))
        if _STATE["t"] >= p["t_freeze"] - p["down"] - p["settle"]:  # holding still over the gap and going down into
            # it: close in on the tip's sideways error (its height keeps TIP_DOWN's allowance for the sag)
            err = moved(p["aims"][2][1]) - tool_tip
            err[2] = 0.0
            corr += AIM_GAIN * err
            np.clip(corr, -AIM_MAX, AIM_MAX, out=corr)
        for w, tip, tilt, heading in p["aims"]:
            h = heading + turn
            w.pos = tool_at(*(moved(tip) + corr), tilt, h)
            w.yaw = (h - back, tilt, h)
        p["seen"] = (rx - x0, ry - y0, turn)
        if os.environ.get("COTTER_LOG") and _STATE["t"] + environment.unwrapped.step_dt >= p["t_freeze"]:
            err = (moved(p["aims"][2][1]) - tool_tip) * 1000
            print(f"LOG aim: pin root off by {(rx - x0) * 1000:+.2f} {(ry - y0) * 1000:+.2f} mm, turned "
                  f"{math.degrees(turn):+.2f} deg; tip correction {np.round(corr * 1000, 2)} mm, tip still off "
                  f"{np.round(err, 2)} mm", flush=True)


@on_step
def _watch(environment) -> None:
    """Released: no robot link touches the pin's body or either prong (PhysX contacts, touch.py)."""
    names = ["cotter", "cotter/prong_p", "cotter/prong_n"]
    _STATE["free"] = not any(touch.touching(environment, names).values())
    _STATE["t"] += environment.unwrapped.step_dt
    _aim_live(environment)
    if os.environ.get("COTTER_LOG") and _STATE["t"] - _STATE["last"] >= 0.5:
        _STATE["last"] = _STATE["t"]
        from yam_arena.scene import object_states

        s = object_states(environment, ["cotter", "screwdriver"])
        c = s["cotter"]
        r = root_rel(c) * 1000
        p, n = (math.degrees(a) for a in spread(c))
        tool = (np.asarray(s["screwdriver"].pos) - np.array([NX, NY, TOP])) * 1000
        ax = c.rotation()[:, 0] * np.array([PIN_DIR, 1.0, 1.0])  # yaw 0: pointing the way it goes in
        pitch, yaw = math.degrees(math.asin(max(-1.0, min(1.0, ax[2])))), math.degrees(math.atan2(ax[1], ax[0]))
        print(f"LOG t={_STATE['t']:.1f} root(in) {r[0]:+.1f} {r[1]:+.1f} {r[2]:+.1f} mm  pitch {pitch:+.1f} yaw {yaw:+.1f}  "
              f"through={through(c)}  prongs p(-y) {p:.0f} / n(+y) {n:.0f} deg (apart {p + n:.0f})  free={_STATE['free']}  "
              f"tool tip {tool[0]:+.1f} {tool[1]:+.1f} {tool[2]:+.1f} mm  score={score(s)}", flush=True)


# -- the scripted demo: both arms at once. The right hand takes the pin in from the far side and keeps its head pressed
# home; the left, on the robot's side of the nut, picks the screwdriver up upright, lays it nearly flat pointing forwards
# (+x), puts its blade down between the prongs and levers each one round about its root ---------------------------------
TOOL_GRIP = 42.0 * MM  # the pads' tips this far up the tool from its tip, on the handle just above the blade
TOOL_CLOSE = -(9.0 * MM - 8.0 * MM)  # finger joint on the 18 mm handle: 8 mm of squeeze each side (about 40 N: levering
# a prong round, the blade must not turn in the hand)
TOOL_LIFT = 0.045
EYE_CLOSE = -(EYE_D / 2 - 1.5 * MM)  # finger joint on the 5 mm eye: 1.5 mm of squeeze each side
PAD_BELOW = 3.0 * MM  # holding the eye (it sticks out past the block's end), the pads' tips this far below it
FINGER_HALF = 7.75 * MM  # a finger is 15.5 mm wide (along x, yaw 0) over its last 15 mm (yam_arena's tip_left hulls)
NUT_CLEAR = 0.8 * MM  # holding the eye, the fingers stop this far short of the nut's flat; then the closed fingertips
# push the head home from behind
PIN_LEVEL = math.radians(float(os.environ.get("PIN_LEVEL_DEG", "1.5")))  # (see demo)
PUSH_BELOW = 4.0 * MM  # pushing the head home, the closed fingertips this far below the pin's axis: their lowest,
# nearly upright part meets the eye's lower half (at 1.75 mm the fingers' slanted front met its upper half, rode up over
# it and pressed the head down; the steel plate's top is 6.5 mm below the axis)
DROP_CLEAR = 10.0 * MM  # the closed fingertips come down this far behind the released pin's eye (at 5 mm they
# still landed on the eye: it lags 2.6 mm in the hand, and the fingers are wider than 15.5 mm a little up)
GRIP_BACK = float(os.environ.get("GRIP_BACK", "-0.001"))  # grip the eye this far behind its centre: 1 mm in front of
# it, the pads also take the shank's end and the pin stays level as it comes out of the groove (2 mm further out it
# pivots 9 degrees, prongs down, the moment it leaves the groove)
LIFT_T = float(os.environ.get("LIFT_T", "1.2"))  # s: lifting the pin out of the groove
APPROACH = 0.040  # line the pin up with the slot with its roots this far behind the nut's flat (its tips then 13 mm
# short of the nut: the arm can overshoot a few mm coming in, and the tips must not land on the nut)
AIM_Z = float(os.environ.get("AIM_Z", "0.0003"))  # aim the held pin this far up (it can sag 0.6 mm in the hand)...
AIM_Y = float(os.environ.get("AIM_Y", "-0.0004"))  # ... and this far sideways (it sat 0.4 mm to +y once lifted)
TOOL_Y = float(os.environ.get("TOOL_Y", "0.0006"))  # aim the screwdriver's tip this far sideways (m): going straight
# down into the gap by the prongs' roots, the held screwdriver's tip sits 0.6 mm to -y of where it is aimed (run 33),
# and the gap is 0.7 mm
FLAT = math.radians(float(os.environ.get("FLAT_DEG", "65")))  # the screwdriver this far from upright (25 degrees
# down from flat), its handle towards the robot (-x) and its tip forwards and down: at 40-80 degrees the left arm
# reaches all round the nut (90, quite flat, is near a singular wrist); at 65 it levers a prong 85 degrees round one way
# (+y) and 25 the other (offline IK checks). Steeper (runs 42-45: 55-40 degrees), its tip came down less surely into the
# gap
BACK = math.pi  # the handle's heading, sliding in: towards -x
TIP_DOWN = float(os.environ.get("TIP_DOWN", "-0.0007"))  # the blade's tip (the middle of its edge) aimed this far
# below the prongs' middle: above it, as the held screwdriver sags 0.9 mm (run 32), so it ends 0.2 mm below; its edge,
# 2.5 mm tall, then spans the prongs (1.5 mm), and where it does the blade is 0.4 mm thick (the gap is 0.7)
INSERT_STOP = 2.5 * MM  # the blade goes down into the gap with its tip this far out from the nut's flat (the prongs'
# roots; its edge leans 25 degrees, its top towards the nut)
DESCEND_FROM = 4.0 * MM  # ... from this far above, lined up over the gap
LEVER_P = math.radians(float(os.environ.get("LEVER_P", "85")))  # lever the +y prong round this far (it lags ~16 deg)
LEVER_N = math.radians(float(os.environ.get("LEVER_N", "25")))  # ... the -y one only this far (the arm's reach, with
# the hand turned to keep the blade's edge upright)
LEVER_STEP = math.radians(7.5)  # in steps this big (the tip stays within 0.2 mm of the pivot between them)
SECOND_Y = 0.6 * MM  # going in again for the -y prong (the +y one is open), the blade this far to +y of the axis
FOLD_C = 4.5 * MM  # folding a prong on round against the nut, the blade's tip meets it this far out (sideways) from
# its centre line: wherever it lies between 25 and 75 degrees, the tip starts outside it...
FOLD_C_LOW = 2.5 * MM  # ... and, for the -y prong (levered part way), first this far out: from 15 degrees on, it meets
# the prong and pushes it to about 50; then FOLD_C pushes it past 60
FOLD_STOP = 1.5 * MM  # ... and pushes until it is this far short of the nut's flat
SETTLE = 2.5  # s: hold the screwdriver still over the gap before going down (its sway dies down, its aim is trimmed)
AIM_GAIN = 0.08  # per control step, the share of the tip's remaining error the live aim takes out while it holds still
AIM_MAX = 0.003  # m: the live aim's trim stays within this


def tip_rotation(tilt: float, heading: float) -> np.ndarray:
    """The rotation that leans a straight-down tool ``tilt`` from upright, its top (the handle) towards ``heading``."""
    from yam_arena.urdf import axis_angle_matrix

    h = np.array([math.cos(heading), math.sin(heading), 0.0])
    axis = np.cross(np.array([0.0, 0.0, -1.0]), -math.sin(tilt) * h + np.array([0.0, 0.0, -math.cos(tilt)]))
    n = np.linalg.norm(axis)
    return np.eye(3) if n < 1e-12 else axis_angle_matrix(axis / n, tilt)


def _install_tilt() -> None:
    """Let a Waypoint's ``yaw`` be a triple (yaw, tilt, heading): the straight-down grip turned ``yaw`` about the
    vertical, then tipped ``tilt`` so that the hand (the tool's handle) leans towards ``heading`` (0 = +x, 90 deg = +y).
    yam_arena's ArmPlan builds each waypoint's rotation with demo.grip_rotation(w.yaw) and solves its IK for the full
    rotation; this wraps that function, in this process only, without changing yam_arena."""
    from yam_arena import demo as _demo

    if getattr(_demo.grip_rotation, "_tilt", False):
        return
    plain = _demo.grip_rotation

    def grip_rotation(yaw=0.0):
        if isinstance(yaw, tuple):
            turn, tilt, heading = yaw
            return tip_rotation(tilt, heading) @ plain(turn)
        return plain(yaw)

    grip_rotation._tilt = True
    _demo.grip_rotation = grip_rotation


def demo():
    """Right arm: pick the pin by its eye where it sticks out of the block, carry it round to the nut's far side, slide
    it along the slot through the stud's hole, let go just short of home and push the head home with the closed
    fingertips; keep pressing it there until the left arm is done, then move clear. Left arm, at the same time: take
    the screwdriver out of its rack upright and lay it down to 25 degrees, tip forwards; put the blade down into the
    gap between the prongs, lever the +y prong round about its root and push it on round against the nut's flat from
    beyond its tip; go in again by the -y prong, lever it round the other way and push it on round the same way; stand
    the screwdriver back in its rack and move clear."""
    from yam_arena.constants import FINGER_OPEN as O
    from yam_arena.constants import PAD_TIP_PAST_TCP as TIP
    from yam_arena.demo import Waypoint as W

    _install_tilt()
    hover = TOP + 0.150
    line_z = TOP + HOLE_Z  # the pin's centre line in the hole

    def X(u, x0=NX):
        """World x of a point ``u`` along the way the pin goes in, from x0 (the stud's axis)."""
        return x0 + PIN_DIR * u

    grip_up = TIP - PRONG_W / 2 - PAD_BELOW  # the TCP above the pin's centre line while holding the eye
    push_up = TIP - PUSH_BELOW  # ... while pushing its head with the closed fingertips
    cx, cy, cz = COTTER_START
    eye = -INSIDE - EYE_D / 2 - GRIP_BACK  # where the fingers hold the eye, from the cotter's origin, along its axis
    behind = HOME_X - INSIDE - EYE_D - FINGER_HALF  # (along) the TCP with the closed fingers' front face at the eye's back,
    # the pin home
    stop = -HOME_X - NUT_CLEAR - FINGER_HALF  # (along) the TCP when the fingers holding the eye are just short of the nut
    in_line = -HOME_X - APPROACH + eye + INSIDE  # (along) the TCP with the pin lined up, APPROACH short
    # let go, the pin is short of home by where the hand held it (and it may lag a few mm more in the hand): the closed
    # fingers come down DROP_CLEAR behind where its eye's back is then, not behind where it will be once home
    released = stop - eye  # (along) the prongs' roots as the hand lets go
    drop = released - INSIDE - EYE_D - DROP_CLEAR - FINGER_HALF  # (along) the TCP coming down behind the eye
    park_r, park_l = (0.20, -0.30, hover), (0.20, 0.30, hover)
    R_YAW = 0.0  # the right hand's turn about the vertical (its wrist can't turn it half round)
    # holding the pin, the hand tipped PIN_LEVEL, its top towards -x (the prongs' way), so the prongs dip: the pin,
    # held near its head, sits 1-2 degrees prongs-up in the pads, and through the 2.8 mm hole (its shank is 2.66 mm
    # corner to corner) it jams past about 1 degree
    H = (0.0, PIN_LEVEL, math.pi) if PIN_LEVEL else R_YAW
    root = X(HOME_X)  # the prongs' roots (world x), on the slot's line, at the nut's robot-side flat

    def dur(plan):
        return sum(w.duration for w in plan)

    # -- right: the pin in from the far side, then hold its head home
    right = [
        W(1.5, (X(eye, cx), cy, hover), O, "over the pin's eye", R_YAW),
        W(1.5, (X(eye, cx), cy, cz + grip_up), O, "down round the eye, past the block's end", R_YAW),
        W(0.8, (X(eye, cx), cy, cz + grip_up), EYE_CLOSE, "grip", R_YAW),
        W(LIFT_T, (X(eye, cx), cy, cz + grip_up + 0.015), EYE_CLOSE, "lift it out of the groove", H),
        W(2.5, (X(in_line), NY + AIM_Y, line_z + AIM_Z + grip_up + 0.010), EYE_CLOSE, "round to the far side, in line with the slot", H),
        W(1.0, (X(in_line), NY + AIM_Y, line_z + AIM_Z + grip_up), EYE_CLOSE, "down to the hole's height", H),
        W(4.0, (X(stop), NY + AIM_Y, line_z + AIM_Z + grip_up), EYE_CLOSE, "slide it through the slot and the hole", H),
        W(0.6, (X(stop), NY + AIM_Y, line_z + AIM_Z + grip_up), -0.012, "let go", H),
        W(0.8, (X(stop - 0.004), NY, line_z + grip_up + 0.012), -0.012, "up and back", R_YAW),
        W(1.5, (X(drop), NY, line_z + push_up + 0.012), 0.0, "behind the eye, fingers closed", R_YAW),
        W(0.8, (X(drop), NY, line_z + push_up), 0.0, "down", R_YAW),
        W(3.0, (X(behind + 0.0008), NY, line_z + push_up), 0.0, "push the head home", R_YAW),
    ]
    t_pressing = dur(right)

    # -- left: the screwdriver, laid nearly flat
    D = TOOL_GRIP + TIP  # the TCP from the tool's tip, along the tool

    def tool_at(x, y, z, tilt, heading):
        """The TCP holding the screwdriver tilted ``tilt`` (handle towards ``heading``) with its tip at world (x, y, z)."""
        r = D * math.sin(tilt)
        return (x + r * math.cos(heading), y + TOOL_Y + r * math.sin(heading), z + D * math.cos(tilt))

    made = {}  # id(waypoint) -> (tip, tilt, heading): what each screwdriver waypoint aims at

    def tw(dt, tip, label, heading=BACK, tilt=FLAT):
        # the hand turned with the heading: the blade's edge stays upright and its thin side faces the prongs (turned
        # only by the tilt, a levering blade would lie flat, its 4 mm edge sideways into the nut)
        w = W(dt, tool_at(*tip, tilt, heading), TOOL_CLOSE, label, (heading - BACK, tilt, heading))
        made[id(w)] = (tip, tilt, heading)
        return w

    T = TOOL_CLOSE
    tx, ty, tz = TOOL_START
    g = (tx, ty, tz + D)
    up_g = (g[0], g[1], g[2] + TOOL_LIFT)
    t_z = line_z - TIP_DOWN  # the blade's tip at the prongs' height
    out = root - (PRONG_L + 2.0 * MM) - 0.006  # (world x) the tip just beyond the prongs' tips, lined up
    deep = root - INSERT_STOP  # ... and where it goes down between them
    left = [
        W(1.5, (g[0], g[1], g[2] + 0.05), O, "over the screwdriver", 0.0),
        W(1.5, g, O, "down round its handle", 0.0),
        W(1.2, g, T, "grip", 0.0),
        W(1.5, up_g, T, "lift it out of the rack", 0.0),
        tw(2.5, (out - 0.030, NY + 0.060, t_z + 0.030), "lay it nearly flat, tip forwards, out of the way"),
    ]
    # wait out of the right arm's way until it is pressing the pin's head home
    left.append(W(max(t_pressing - dur(left), 0.5), left[-1].pos, T, "wait for the pin to be home", left[-1].yaw))

    def go_in(y, name):
        """Over the gap between the prongs, by their roots, then straight down into it."""
        return [
            tw(2.0, (deep, y, t_z + 0.010), f"over the gap, {name}"),
            tw(1.0, (deep, y, t_z + DESCEND_FROM), "lower, just above the prongs"),
            tw(SETTLE, (deep, y, t_z + DESCEND_FROM), "hold still"),
            tw(2.0, (deep, y, t_z), "straight down into the gap"),
        ]

    def lever(side, to, y, name):
        """Swing the handle round about the blade's tip (by the prongs' roots): the blade levers one prong round,
        then comes back in line and goes straight up."""
        n = max(1, round(to / LEVER_STEP))
        up = t_z + 0.010
        # out of the gap first, still turned, then back in line in the same small steps, the tip held over the pivot
        # (turned back in one step, the tip swings through the nut: the rotation and the hand's path interpolate apart)
        return [tw(0.6, (deep, y, t_z), f"lever the {name} prong round ({math.degrees(to * k / n):.0f} deg)",
                   heading=BACK - side * to * k / n) for k in range(1, n + 1)] + [
            tw(1.2, (deep, y, up), "straight up out of the gap", heading=BACK - side * to),
        ] + [tw(0.5, (deep, y, up), "back in line, over the gap", heading=BACK - side * to * k / n)
             for k in range(n - 1, -1, -1)]

    def fold(side, name, c=FOLD_C):
        """From beyond the opened prong's tip, push it on round towards the nut's flat with the blade's tip."""
        fy = NY + side * (PRONG_YC + c)
        return [
            tw(1.2, (out, fy, t_z + 0.008), f"beyond the {name} prong's tip"),
            tw(0.8, (out, fy, t_z), "down"),
            tw(2.5, (root - FOLD_STOP, fy, t_z), f"push the {name} prong on round against the nut"),
            tw(0.6, (root - FOLD_STOP, fy, t_z), "hold"),
            tw(0.8, (root - FOLD_STOP - 0.004, fy, t_z + 0.012), "back off and up"),
        ]

    # each pass at the prongs aims where the pin actually sits (it shifts a little in the slot and the hole as the right
    # hand takes hold of it): until the blade starts down into the gap, the on_step hook moves the pass's waypoints with
    # the pin (the 0.7 mm gap leaves no room for its 0.5 mm of play)
    passes = []
    for steps in ((go_in(NY, "between the prongs"), lever(1.0, LEVER_P, NY, "+y"), fold(1.0, "+y")),
                  (go_in(NY + SECOND_Y, "by the -y prong"), lever(-1.0, LEVER_N, NY + SECOND_Y, "-y"),
                   fold(-1.0, "-y", FOLD_C_LOW), fold(-1.0, "-y"))):
        ws = [w for part in steps for w in part]
        t_down = dur(left) + dur(steps[0][:3])  # the blade starts down into the gap...
        t_in = t_down + steps[0][3].duration  # ... and is in it
        passes.append({"t_freeze": t_in, "down": steps[0][3].duration, "settle": SETTLE,
                       "aims": [(w,) + made[id(w)] for w in ws]})
        left += ws
    cotterkit.LIVE.update(passes=passes, tool_at=tool_at, root0=(root, NY), back=BACK)
    left += [
        tw(1.5, (out - 0.030, NY + 0.060, t_z + 0.030), "away from the nut"),
        W(2.5, up_g, T, "upright again, over the rack", 0.0),
        W(1.5, g, T, "stand it in the rack", 0.0),
        W(0.8, g, O, "let go", 0.0),
        W(1.2, (g[0], g[1], g[2] + 0.05), O, "up", 0.0),
    ]
    left.append(W(1.5, park_l, O, "clear", 0.0))
    t_done = dur(left) - 7.5  # the screwdriver is away from the nut
    right += [
        # (pinching the head instead, runs 36-40, the fingers can't reach low enough round the eye by the nut to grip
        # it, and the levered pin turned in them)
        W(max(t_done - dur(right), 0.5), (X(behind + 0.0008), NY, line_z + push_up), 0.0, "keep the head pressed home", R_YAW),
        W(0.8, (X(behind - 0.006), NY, line_z + push_up + 0.012), 0.0, "off the head", R_YAW),
        W(1.5, park_r, O, "clear", 0.0),
    ]
    t = max(dur(right), dur(left)) + 3.0
    right.append(W(t - dur(right), park_r, O, "wait", 0.0))
    left.append(W(t - dur(left), park_l, O, "wait", 0.0))
    return {"left": left, "right": right}


def self_test(ObjectState=None):
    """The scoring rules and the plastic rule on hand-built states."""
    from scenekit import BodyState

    def pin(dx=HOME_X, dy=0.0, dz=0.0, yaw=0.0, p=0.0, n=0.0):
        pos = np.array([NX + PIN_DIR * dx, NY + dy, TOP + HOLE_Z + dz])
        q = np.array([0.0, 0.0, math.sin((math.pi + yaw) / 2), math.cos((math.pi + yaw) / 2)])
        return BodyState(pos=pos, quat_xyzw=q, lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=pos, high=pos,
                         joints={"prong_p_joint": p, "prong_n_joint": -n})

    def tool(dist):
        pos = np.array([NX + PIN_DIR * (HOME_X + dist), NY, TOP + HOLE_Z])
        return BodyState(pos=pos, quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=pos, high=pos)

    d = math.radians
    far = tool(0.10)
    cases = {
        "on the block": ({"cotter": pin(dx=-0.07, dz=-0.02), "screwdriver": far}, True, 0.0),
        "in the slot short of the stud": ({"cotter": pin(dx=STUD_R - 1 * MM), "screwdriver": far}, True, 0.0),
        "through, prongs straight": ({"cotter": pin(), "screwdriver": far}, True, 0.25),
        "through, prongs opened 20 degrees": ({"cotter": pin(p=d(20), n=d(20)), "screwdriver": far}, True, 0.25),
        "through, one prong spread": ({"cotter": pin(p=d(75)), "screwdriver": far}, True, 0.5),
        "one prong bent 90, the other straight": ({"cotter": pin(p=d(90)), "screwdriver": far}, True, 0.5),
        "spread 85 degrees apart": ({"cotter": pin(p=d(50), n=d(35)), "screwdriver": far}, True, 0.5),
        "spread 99 degrees apart (64 and 35)": ({"cotter": pin(p=d(64), n=d(35)), "screwdriver": far}, True, 1.0),
        "spread 95 degrees apart, one only 25": ({"cotter": pin(p=d(70), n=d(25)), "screwdriver": far}, True, 0.5),
        "through, both spread, released": ({"cotter": pin(p=d(75), n=d(70)), "screwdriver": far}, True, 1.0),
        "both spread, still held": ({"cotter": pin(p=d(75), n=d(70)), "screwdriver": far}, False, 0.75),
        "both spread, head 1.5 mm out": ({"cotter": pin(dx=HOME_X - 1.5 * MM, p=d(75), n=d(70)), "screwdriver": far}, True, 0.75),
        "both spread, screwdriver left between them": ({"cotter": pin(p=d(75), n=d(70)), "screwdriver": tool(0.008)}, True, 0.75),
        "bent into each other": ({"cotter": pin(p=-d(70), n=-d(70)), "screwdriver": far}, True, 0.25),
        "turned across the slot": ({"cotter": pin(yaw=d(30), p=d(75), n=d(70)), "screwdriver": far}, True, 0.0),
        "pulled out after bending": ({"cotter": pin(dx=INSIDE + 1 * MM, p=d(75), n=d(70)), "screwdriver": far}, True, 0.0),
    }
    for name, (objs, free, want) in cases.items():
        _STATE["free"] = free
        got = score(objs)
        assert abs(got - want) < 1e-9, (name, got, want)
    _STATE["free"] = True
    # the plastic rule: below yield a prong springs back; past it, it keeps the bend less the springback; it never
    # sets inwards past INWARD
    j = "prong_p_joint"
    s = cotterkit.plastic_step(0.0, cotterkit.SPRINGBACK * 0.9, j)
    assert s == 0.0, s
    for a in np.linspace(0.0, d(80), 200):
        s = cotterkit.plastic_step(s, a, j)
    assert abs(s - (d(80) - cotterkit.SPRINGBACK)) < 1e-9, s
    s = 0.0
    for a in np.linspace(0.0, -d(40), 200):
        s = cotterkit.plastic_step(s, a, j)
    assert abs(s + cotterkit.INWARD) < 1e-9, s
    # sizes against each other: the pin fits the hole, the slot and the groove; the prongs reach past the nut; the
    # force is within the arm's push
    assert PIN_W < SLOT_W and PRONG_W < HOLE and PIN_W < HOLE and PIN_W < GROOVE_W
    assert PRONG_L * math.tan(math.radians(15)) > FOLD_C_LOW  # a prong at 15 degrees reaches the low fold's line
    assert cotterkit.MY / (8 * MM) < 20.0, cotterkit.MY / (8 * MM)
    return len(cases) + 3


if __name__ == "__main__":
    # Offline checks, no GPU: the scoring rules, then each arm's plan through the kinematics at 20 Hz.
    from yam_arena import ik, station
    from yam_arena.demo import ArmPlan
    from yam_arena.task_file import load

    print(f"self-test: {self_test()} cases pass")
    task = load(__file__)
    for arm, waypoints in task.demo().items():
        plan = ArmPlan(list(waypoints), station.BASES[arm])
        q = np.array([plan.action7((k + 1) / 20)[0] for k in range(round(plan.duration * 20) + 5)])
        assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), arm
        print(f"{arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / {plan.worst_ik_deg:.3f} deg")
