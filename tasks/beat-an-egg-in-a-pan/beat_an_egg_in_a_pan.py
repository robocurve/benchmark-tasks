"""On the station table, a black glass hob with a grey burner ring. On the ring stands a dark non-stick pan (inside 20 cm
across and 4.5 cm deep) whose black handle points left, toward the left arm. In the pan lies an egg made of 101 beads
(10 mm) in one layer: 37 yellow beads, the yolk, fill a disc in the middle, and 64 white beads, the white, fill the
ring around it. To the right of the pan, a balloon whisk (14 cm long) stands upright, wires down, in a white whisk
holder. The pan, the beads and the whisk are loose; the hob, the ring and the holder are fixed.

Task: hold the pan by its handle and beat the egg with the whisk until the yolk is broken up through the white: no patch
of yellow beads (beads whose centres are less than 1.7 cm apart, linked into one patch) may hold more than 12 of the
37. Moving the egg about whole leaves the yolk in one patch. Keep the beads in the pan and the pan flat on the burner
ring throughout, and touch the egg only with the whisk: no gripper finger may go into the pan below its rim.

Run it:  inspect-robots run --embodiment yam_arena -E task=beat_an_egg_in_a_pan.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import Body, BoxShape, CylShape, SphereShape, as_numpy, on_reset, on_step, rotation, segment  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- layout (world frame: x forward from the arm bases, y left, z up; metres) ----------------------------------------
PAN_XY = (0.24, -0.10)  # centre of the burner ring and of the pan at the start
HOB_SIZE = (0.28, 0.28, 0.006)
RING_RADIUS, RING_H = 0.125, 0.001
PAN_Z = TOP + HOB_SIZE[2] + RING_H  # the pan's underside

R_IN, WALL_T, WALL_H, FLOOR_T = 0.100, 0.006, 0.045, 0.005  # pan: inside radius, wall thickness and height, floor
R_OUT = R_IN + WALL_T
HANDLE_LEN, HANDLE_W, HANDLE_T, HANDLE_Z = 0.14, 0.024, 0.016, 0.040  # handle along +y, centre height above underside
HANDLE_GRIP_Y = R_OUT + 0.035  # where the left gripper holds it, from the pan's centre (close in: a short lever)

BEAD_R = 0.005  # 10 mm beads
BEAD_MASS = 0.0008
PITCH = 0.0155  # the egg's beads sit on a square grid of this pitch centred on the pan's centre
YOLK_R = 0.050  # grid spots this close to the centre hold yolk (yellow) beads
WHITE_FROM = 0.060  # and those from here out to BEAD_MAX white ones
BEAD_MAX = R_IN - BEAD_R - 0.0025

WHISK_XY = (0.26, -0.29)
BULB_R, BULB_H, BULB_TOP_R = 0.025, 0.070, 0.007  # balloon: widest radius, height, radius where the wires meet
FERRULE_R, FERRULE_H = 0.009, 0.012
WHISK_HANDLE = (0.022, 0.016, 0.060)  # x, y (the thin side, which the fingers squeeze), z
WHISK_LEN = BULB_H + FERRULE_H + WHISK_HANDLE[2]
WIRE = 0.003
HOLDER_R_IN, HOLDER_T, HOLDER_H = BULB_R + 0.0015, 0.005, 0.045  # the holder: a short tube just wider than the bulb

YOLK, WHITE = (0.98, 0.70, 0.08), (0.96, 0.96, 0.94)
PAN_DARK, PAN_WALL, BLACK, STEEL, CROCK = (
    (0.13, 0.13, 0.14), (0.22, 0.22, 0.24), (0.06, 0.06, 0.06), (0.70, 0.71, 0.73), (0.90, 0.90, 0.88))

# -- scoring --------------------------------------------------------------------------------------------------------
# The yolk is broken up when its beads no longer hang together: two yellow beads belong to one patch when their centres
# are less than LINK apart (the yolk's grid pitch is 1.55 cm, so the whole yolk starts as one patch of 37), and the
# score is set by the largest patch. Well-mixed beads (placed at random over the pan) give a largest patch of 5 (median)
# and 12 or fewer 99% of the time; packed into half the pan, 9 (median) and 21 (99%).
LINK = 0.017
BROKEN_MAX = 12  # full score: no patch bigger than a third of the yolk
CRACKED_MAX = 24  # half score: no patch bigger than two thirds of it
PAN_MOVE_MAX = 0.03  # the pan's centre may end at most this far from the ring's centre
PAN_TILT_MAX = math.radians(5)  # at any time: the pan stays flat on the burner (no tipping the egg round)
PAN_LIFT_MAX = 0.01  # at any time: the pan's centre stays within 1 cm of the burner ring
SPILL_MAX = 3  # more beads than this outside the pan scores 0
SUCCESS_HOLD_S = 1.0


def _pan() -> Body:
    shapes = [CylShape((0.0, 0.0, FLOOR_T / 2), R_OUT, FLOOR_T, PAN_DARK)]
    n = 24
    chord = 2 * R_OUT * math.tan(math.pi / n) + 0.001
    for i in range(n):
        a = 2 * math.pi * i / n
        r = R_IN + WALL_T / 2
        shapes.append(BoxShape((r * math.cos(a), r * math.sin(a), FLOOR_T + WALL_H / 2), (WALL_T, chord, WALL_H), PAN_WALL,
                               yaw=a))
    shapes.append(BoxShape((0.0, R_OUT + HANDLE_LEN / 2, HANDLE_Z), (HANDLE_W, HANDLE_LEN, HANDLE_T), BLACK))
    return Body.build("pan", pos=(*PAN_XY, PAN_Z), mass=0.9, shapes=shapes, friction=(0.5, 0.4), color=PAN_DARK)


def _egg_spots() -> tuple[list, list]:
    """Pan-frame (x, y) of the yolk and white beads: the grid spots within YOLK_R, and from WHITE_FROM to BEAD_MAX."""
    spots = [(i * PITCH, j * PITCH) for i in range(-6, 7) for j in range(-6, 7)]
    yolk = [p for p in spots if math.hypot(*p) <= YOLK_R + 1e-9]
    white = [p for p in spots if WHITE_FROM - 1e-9 <= math.hypot(*p) <= BEAD_MAX]
    return yolk, white


def _beads() -> list[Body]:
    yolk, white = _egg_spots()
    out = []
    for name, colour, spots in (("yolk", YOLK, yolk), ("white", WHITE, white)):
        for i, (x, y) in enumerate(spots):
            pos = (PAN_XY[0] + x, PAN_XY[1] + y, PAN_Z + FLOOR_T + BEAD_R + 0.0005)
            out.append(Body.build(f"{name}_{i:02d}", pos=pos, mass=BEAD_MASS, color=colour,
                                  shapes=[SphereShape((0.0, 0.0, 0.0), BEAD_R, colour)], damping=(0.3, 2.0),
                                  friction=(0.25, 0.2)))
    return out


def _whisk_profile() -> list[tuple[float, float]]:
    """(radius, height) along one wire, from the tip up: a quarter circle out to BULB_R, then a straight taper in."""
    pts = [(BULB_R * math.sin(math.radians(a)), BULB_R * (1 - math.cos(math.radians(a)))) for a in range(0, 91, 18)]
    for k in range(1, 5):
        f = k / 4
        pts.append((BULB_R + (BULB_TOP_R - BULB_R) * f, BULB_R + (BULB_H - BULB_R) * f))
    return pts


def _whisk() -> Body:
    """Body origin at the tip of the wires. Four wire loops (eight wires) in planes 45 degrees apart, a steel ferrule
    and a handle whose thin side faces y."""
    prof = _whisk_profile()
    shapes = []
    for k in range(8):
        a = math.pi * k / 4
        u = (math.cos(a), math.sin(a))
        for (r1, z1), (r2, z2) in zip(prof, prof[1:]):
            shapes.append(segment((r1 * u[0], r1 * u[1], z1), (r2 * u[0], r2 * u[1], z2), WIRE, STEEL))
    shapes.append(CylShape((0.0, 0.0, BULB_H + FERRULE_H / 2), FERRULE_R, FERRULE_H, STEEL))
    shapes.append(BoxShape((0.0, 0.0, BULB_H + FERRULE_H + WHISK_HANDLE[2] / 2), WHISK_HANDLE, BLACK))
    return Body.build("whisk", pos=(*WHISK_XY, TOP + 0.0005), mass=0.06, shapes=shapes, friction=(0.9, 0.8), color=STEEL)


def _holder() -> list[Box]:
    """A fixed short tube (12 flat segments) around the whisk's bulb."""
    n, (cx, cy) = 12, WHISK_XY
    r = HOLDER_R_IN + HOLDER_T / 2
    chord = 2 * (HOLDER_R_IN + HOLDER_T) * math.tan(math.pi / n) + 0.001
    return [Box(f"holder_{k:02d}", pos=(cx + r * math.cos(2 * math.pi * k / n), cy + r * math.sin(2 * math.pi * k / n),
                                        TOP + HOLDER_H / 2),
                size=(HOLDER_T, chord, HOLDER_H), yaw=2 * math.pi * k / n, color=CROCK) for k in range(n)]


PAN = _pan()
BEADS = _beads()
WHISK = _whisk()
OBJECTS = [
    Box("hob", pos=(*PAN_XY, TOP + HOB_SIZE[2] / 2), size=HOB_SIZE, color=(0.05, 0.05, 0.06)),
    Cylinder("burner_ring", pos=(*PAN_XY, TOP + HOB_SIZE[2] + RING_H / 2), radius=RING_RADIUS, height=RING_H,
             color=(0.32, 0.32, 0.34)),
    PAN,
    *BEADS,
    WHISK,
    *_holder(),
]
BEAD_NAMES = [b.name for b in BEADS]


# -- scoring --------------------------------------------------------------------------------------------------------


_EPISODE = {"max_tilt": 0.0, "max_lift": 0.0, "finger_in_egg": False, "lowest_tip": 1.0}  # over the episode


@on_reset
def _new_episode() -> None:
    _EPISODE.update(max_tilt=0.0, max_lift=0.0, finger_in_egg=False, lowest_tip=1.0)


# The egg may be touched only with the whisk: a gripper whose pad tips go into the pan below its rim is in the egg.
# The pad tips' midpoint, in the gripper link's frame: the TCP (between the pads) plus the 13 mm to their tips.
TIP_IN_GRIPPER = np.array([0.0, 0.0, TCP_OFFSET[2] + PAD_TIP_PAST_TCP])
_GRIPPER_BODY: dict = {}


@on_step
def _watch_fingers(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    pan = as_numpy(base.scene["pan"].data.root_link_pose_w)[0]
    pan_rot, pan_pos = rotation(pan[3:7]), pan[:3] - origin
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        if arm not in _GRIPPER_BODY:
            names = list(robot.body_names)
            _GRIPPER_BODY[arm] = next(i for i, n in enumerate(names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][_GRIPPER_BODY[arm]]
        tip = g[:3] - origin + rotation(g[3:7]) @ TIP_IN_GRIPPER
        p = pan_rot.T @ (tip - pan_pos)
        if math.hypot(p[0], p[1]) < R_IN:
            _EPISODE["lowest_tip"] = min(_EPISODE["lowest_tip"], float(p[2]))
            if p[2] < FLOOR_T + WALL_H:
                _EPISODE["finger_in_egg"] = True


def _largest_patch(points) -> int:
    """Size of the largest group of points linked by distances under LINK (single linkage)."""
    n = len(points)
    if n == 0:
        return 0
    parent = list(range(n))

    def root(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    pts = np.asarray(points, float)
    for i in range(n):
        near = np.nonzero(np.hypot(*(pts[i + 1:] - pts[i]).T) < LINK)[0] + i + 1
        for j in near:
            parent[root(int(j))] = root(i)
    return int(np.bincount([root(i) for i in range(n)]).max())


def _tally(objects) -> dict:
    """In the pan's frame (by bead centre): the largest patch of yellow beads, the beads outside the pan; the pan's
    offset from the ring's centre and its tilt; and what has happened over the episode (worst tilt and lift, fingers)."""
    pan = objects["pan"]  # pos: the pan's origin, the centre of its underside
    rot = rotation(pan.quat_xyzw)
    yolk, outside = [], 0
    for name in BEAD_NAMES:
        p = rot.T @ (np.asarray(objects[name].pos, float) - pan.pos)
        if math.hypot(p[0], p[1]) > R_IN or not (0.0 < p[2] < FLOOR_T + WALL_H):
            outside += 1
        elif name.startswith("yolk"):
            yolk.append(p[:2])
    tilt = float(math.acos(max(-1.0, min(1.0, rot[2, 2]))))
    _EPISODE["max_tilt"] = max(_EPISODE["max_tilt"], tilt)
    _EPISODE["max_lift"] = max(_EPISODE["max_lift"], float(pan.pos[2] - PAN_Z))
    return {
        "patch": _largest_patch(yolk),
        "outside": outside,
        "pan_off": float(math.hypot(pan.pos[0] - PAN_XY[0], pan.pos[1] - PAN_XY[1])),
        "pan_tilt": tilt,
        "max_tilt": _EPISODE["max_tilt"],
        "max_lift": _EPISODE["max_lift"],
        "finger_in_egg": _EPISODE["finger_in_egg"],
        "lowest_tip": _EPISODE["lowest_tip"],
    }


def _points(t) -> float:
    if (t["pan_off"] > PAN_MOVE_MAX or t["outside"] > SPILL_MAX or t["max_tilt"] >= PAN_TILT_MAX
            or t["max_lift"] > PAN_LIFT_MAX or t["finger_in_egg"]):
        return 0.0
    if t["patch"] <= BROKEN_MAX:
        return 1.0
    return 0.5 if t["patch"] <= CRACKED_MAX else 0.0


_DEBUG = {"on": bool(os.environ.get("EGG_DEBUG")), "calls": 0}


def score(objects) -> float:
    """0 if at any time the pan tilts 5 degrees or more or lifts more than 1 cm off the burner ring, or a gripper's
    fingers go into the pan below its rim (the egg may be touched only with the whisk), or if at the end the pan's
    centre is more than 3 cm from the ring's centre or more than three beads are outside it. Otherwise by the largest
    patch of yellow beads: 12 or fewer 1.0, 13 to 24 0.5, more 0."""
    t = _tally(objects)
    if _DEBUG["on"]:  # EGG_DEBUG=1: one line a second (at 20 Hz) on stderr
        _DEBUG["calls"] += 1
        if _DEBUG["calls"] % 20 == 0:
            print(f"[egg] t={_DEBUG['calls'] / 20:5.1f}s yolk_patch={t['patch']} out={t['outside']} "
                  f"pan_off={t['pan_off'] * 100:.1f}cm "
                  f"tilt={math.degrees(t['pan_tilt']):.1f}deg max_tilt={math.degrees(t['max_tilt']):.1f}deg "
                  f"max_lift={t['max_lift'] * 100:.1f}cm finger_in_egg={t['finger_in_egg']} "
                  f"lowest_tip_over_pan={t['lowest_tip'] * 100:.1f}cm score={_points(t)}", file=sys.stderr, flush=True)
    return _points(t)


def success(objects) -> bool:
    """Full score with the pan, the whisk and every bead still."""
    if _points(_tally(objects)) < 1.0:
        return False
    return all(objects[n].resting() for n in ("pan", "whisk", *BEAD_NAMES))


# -- scripted demo --------------------------------------------------------------------------------------------------

CONTROL_DT = 0.05  # one waypoint per control step on the circles, so the whisk moves at an even speed
PAD = 0.013  # the pad tips reach this far past the TCP
GRIP_FROM_TOP = 0.040  # the right gripper's pad tips close this far below the top of the whisk's handle
TIP_GAP = 0.003  # the whisk's tip this far above the pan's floor while beating
LEFT_YAW = -math.pi / 2  # the left gripper closes along x across the pan's handle; -90 keeps joint6 in range
WHISK_SPEED = 0.10  # m/s, the whisk's top speed along its loops
# The whisk's axis (pan frame, m) traces loops whose centre itself travels round the pan, as a cook whisks: small
# fast loops that sweep the whole pan, so they dig into the beads wherever they have bunched up. Each pattern is
# (big radius, loop radius, loops per trip round, trips round; negative trips go clockwise seen from above). The bulb
# reaches 2.5 cm past the axis; big + loop radius stays within 6 cm, so the bulb keeps 1.5 cm off the wall, wider than
# a bead, and beads roll past it there instead of being squeezed against the wall.
PATTERNS = (
    (0.000, 0.020, 1, 2),  # break the yolk: two small circles round the middle
    (0.035, 0.025, 6, 1),
    (0.035, 0.025, -6, -1),
    (0.020, 0.040, 4, 1),
    (0.035, 0.025, 6, -1),
    (0.020, 0.040, -4, 1),
)
START = (0.0078, 0.0078)  # where the whisk goes down: between four beads of the grid
# The finish, so the beating does not stop wherever a loop happens to leave the beads: one slow circle that drags the
# outer ring round and spreads the broken yolk along it; then the whisk lifts straight out.
RING_PASS = (0.055, 1, 0.08)  # radius, turns, speed (m/s)


def _loop_point(big, small, loops, theta):
    return (big * math.cos(theta) + small * math.cos(loops * theta), big * math.sin(theta) + small * math.sin(loops * theta))


def _w(t, xyz, finger, label, yaw=0.0) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in xyz), finger, label, yaw)


def _beat_path() -> list[tuple[float, float, float, float]]:
    """(duration, x, y, lift) in the pan's frame for the whisk's axis, lift being the height above beating height (0
    throughout here): each pattern is entered along a straight line from wherever the last one ended, then followed with
    one point per control step at up to WHISK_SPEED; then the finish."""
    path, here = [], START

    def line_to(x, y, speed, lift=0.0, min_s=0.4):
        nonlocal here
        d = math.hypot(x - here[0], y - here[1])
        path.append((max(min_s, d / speed), x, y, lift))
        here = (x, y)

    for big, small, loops, trips in PATTERNS:
        entry = _loop_point(big, small, loops, 0.0)
        if math.hypot(entry[0] - here[0], entry[1] - here[1]) > 1e-6:
            line_to(*entry, 0.6 * WHISK_SPEED)
        rate = WHISK_SPEED / (big + abs(small * loops))  # rad/s of theta at the top speed
        span = 2 * math.pi * trips
        steps = max(8, int(round(abs(span) / rate / CONTROL_DT)))
        for k in range(1, steps + 1):
            path.append((CONTROL_DT, *_loop_point(big, small, loops, span * k / steps), 0.0))
        here = path[-1][1:3]
    # finish 1: drag the outer ring round once
    r, turns, speed = RING_PASS
    a0 = math.atan2(here[1], here[0])
    line_to(r * math.cos(a0), r * math.sin(a0), 0.6 * WHISK_SPEED)
    steps = int(round(2 * math.pi * r * abs(turns) / speed / CONTROL_DT))
    for k in range(1, steps + 1):
        a = a0 + math.copysign(2 * math.pi * abs(turns), turns) * k / steps
        path.append((CONTROL_DT, r * math.cos(a), r * math.sin(a), 0.0))
    return path


BEAT = _beat_path()
BEAT_S = sum(p[0] for p in BEAT)


def _right() -> list[Waypoint]:
    """Right arm: take the whisk out of its holder, beat the egg in loops, finish with a slow circle round the outer
    ring, lift the whisk straight out and stand it back in the holder."""
    wx, wy = WHISK_XY
    grip_z = TOP + WHISK_LEN - GRIP_FROM_TOP + PAD
    below = WHISK_LEN - GRIP_FROM_TOP + PAD  # the whisk's tip below the TCP
    lift_z = max(TOP + HOLDER_H, PAN_Z + FLOOR_T + WALL_H) + 0.008 + below  # higher puts joint4 at its limit
    beat_z = PAN_Z + FLOOR_T + TIP_GAP + below
    cx, cy = PAN_XY
    wps = [
        _w(2.0, (wx, wy, grip_z + 0.04), FINGER_OPEN, "to the whisk"),
        _w(1.5, (wx, wy, grip_z), FINGER_OPEN, "down"),
        _w(0.8, (wx, wy, grip_z), FINGER_CLOSED, "grip the whisk"),
        _w(1.5, (wx, wy, lift_z), FINGER_CLOSED, "lift it out"),
        _w(2.0, (cx + START[0], cy + START[1], lift_z), FINGER_CLOSED, "over the pan"),
        _w(1.5, (cx + START[0], cy + START[1], beat_z), FINGER_CLOSED, "into the egg"),
    ]
    wps += [_w(dt, (cx + x, cy + y, beat_z + lift), FINGER_CLOSED, "beat") for dt, x, y, lift in BEAT]
    last = wps[-1].pos
    wps += [
        _w(1.5, (last[0], last[1], lift_z), FINGER_CLOSED, "lift straight out of the pan"),
        _w(2.0, (wx, wy, lift_z), FINGER_CLOSED, "back to the holder"),
        _w(1.5, (wx, wy, grip_z + 0.004), FINGER_CLOSED, "into the holder"),
        _w(0.8, (wx, wy, grip_z + 0.004), FINGER_OPEN, "let go"),
        _w(1.5, (wx, wy, grip_z + 0.05), FINGER_OPEN, "up"),
    ]
    return wps


def _left() -> list[Waypoint]:
    """Left arm: grip the pan's handle across its width before the whisk goes in, let go after it comes out."""
    x, y = PAN_XY[0], PAN_XY[1] + HANDLE_GRIP_Y
    z = PAN_Z + HANDLE_Z + 0.005
    hover = z + 0.08
    hold = (2.0 + 1.5 + 0.8 + 1.5 + 2.0 + 1.5 + BEAT_S + 1.5) - (2.0 + 1.5 + 0.8) + 1.0  # until the whisk is out
    return [
        _w(2.0, (x, y, hover), FINGER_OPEN, "to the handle", LEFT_YAW),
        _w(1.5, (x, y, z), FINGER_OPEN, "down to the handle", LEFT_YAW),
        _w(0.8, (x, y, z), FINGER_CLOSED, "grip the handle", LEFT_YAW),
        _w(hold, (x, y, z), FINGER_CLOSED, "hold the pan", LEFT_YAW),
        _w(0.8, (x, y, z), FINGER_OPEN, "let go", LEFT_YAW),
        _w(1.5, (x, y, hover), FINGER_OPEN, "up", LEFT_YAW),
    ]


def demo():
    return {"left": _left(), "right": _right()}
