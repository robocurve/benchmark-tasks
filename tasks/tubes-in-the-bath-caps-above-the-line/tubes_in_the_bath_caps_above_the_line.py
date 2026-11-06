"""On the station table: a steel-grey water bath (fixed, 16 x 20 cm outside, walls 7 cm tall) stands 17 to 33 cm in
front of the arms, centred between them. Inside it, a small flat blue ledge, the water line, runs along each side wall,
12 x 1.2 cm with its centre 6 cm above the table and its top face at 6.2.
A dark grey submerged rack (fixed, 6 x 16 cm, top 3.5 cm above the table) fills the back half of the bath; the bath
floor, 1 cm above the table, is open in front of it. Four loose orange sample tubes, 3 cm across, stand on the bench in
a row at x 12: tall tubes (8 cm, 60 g) at y +15 and -15, short tubes (5 cm, 45 g) at y +6 and -6. The simulation shows
no water.

A tall tube's top lands in the band (7 to 10 cm above the table) only from the bath floor, and a short tube's only from
the rack.

Requires -E object_jitter=0: the demo's grasps and the setup are for the listed positions.

Run it:  inspect-robots run --embodiment yam_arena -E task=tubes_in_the_bath_caps_above_the_line.py \\
           -E object_jitter=0 --instruction "Stand all four sample tubes upright in the water bath. Each tube's top must
           end 1 to 4 cm above the blue water line: lower floods the sample, higher leaves it out of the heat."
           --scorer task_score ...
"""

import os
import sys
from dataclasses import replace

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
from belimits import Rect, bottom, inside, is_airborne, overlaps, tilt_deg, top  # noqa: E402

from yam_arena.demo import Waypoint, pick_place_waypoints  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
# success() only sees the objects, and a tube lowered to its release height already counts as placed, so a 1 s hold
# would end the episode with the hand still around the last tube. Holding success for 6 s keeps the run going while
# the gripper opens, rises and parks, so the release is seen.
SUCCESS_HOLD_S = 6.0

STEEL, RACK_GREY, BLUE = (0.60, 0.62, 0.65), (0.24, 0.25, 0.28), (0.05, 0.25, 0.90)
ORANGE = (0.95, 0.40, 0.05)  # deep orange: stands out from the oak table

# The bath: a 1 cm floor and 1 cm walls around an interior of x 0.18..0.32, |y| <= 0.09. Wall tops at TOP + 0.07.
FLOOR_H, WALL_T, WALL_TOP = 0.01, 0.01, TOP + 0.07
FLOOR_Z = TOP + FLOOR_H
WALL_H = WALL_TOP - FLOOR_Z
WALL_Z = FLOOR_Z + WALL_H / 2
INTERIOR = Rect((0.18, 0.32), (-0.09, 0.09))
BATH = [
    Box("bath_floor", pos=(0.25, 0.0, TOP + FLOOR_H / 2), size=(0.16, 0.20, FLOOR_H), color=STEEL),
    Box("bath_wall_near", pos=(0.175, 0.0, WALL_Z), size=(WALL_T, 0.20, WALL_H), color=STEEL),
    Box("bath_wall_far", pos=(0.325, 0.0, WALL_Z), size=(WALL_T, 0.20, WALL_H), color=STEEL),
    Box("bath_wall_left", pos=(0.25, 0.095, WALL_Z), size=(0.14, WALL_T, WALL_H), color=STEEL),
    Box("bath_wall_right", pos=(0.25, -0.095, WALL_Z), size=(0.14, WALL_T, WALL_H), color=STEEL),
]

# The water line: a blue ledge on the inside of each side wall, flat so the top camera sees it, its centre at the line
# and its top face 2 mm higher. It leaves |y| <= 0.078 open, so a standing tube's centre stays within |y| 0.063. The band
# is 1 to 4 cm above the line's centre.
WATER_LINE = TOP + 0.06
BAND = (WATER_LINE + 0.01, WATER_LINE + 0.04)
LINES = [
    Box(f"water_line_{side}", pos=(0.25, sign * 0.084, WATER_LINE), size=(0.12, 0.012, 0.004), color=BLUE)
    for side, sign in (("left", 1), ("right", -1))
]

# The submerged rack: a solid block on the bath floor, in the back half, top at TOP + 0.035.
RACK_TOP = TOP + 0.035
RACK_AREA = Rect((0.255, 0.315), (-0.08, 0.08))
RACK = Box("rack", pos=(0.285, 0.0, TOP + 0.0225), size=(0.06, 0.16, 0.025), color=RACK_GREY)

# The tubes: 3 cm across, sand-weighted so they sink on either support and when lying down: a tall tube holds 56.5 cm3
# and weighs 60 g (density 1.06), a short one holds 35.3 cm3 and weighs 45 g (density 1.27). A tall tube tops out at
# 0.09 on the floor and 0.115 on the rack; a short tube at 0.06 on the floor and 0.085 on the rack.
R, TALL_H, SHORT_H = 0.015, 0.08, 0.05
BENCH_X = 0.12


def _tube(name, y, h, mass):
    return Cylinder(name, pos=(BENCH_X, y, TOP + h / 2), radius=R, height=h, mass=mass, color=ORANGE)


TALL_A = _tube("tall_a", 0.15, TALL_H, 0.06)
SHORT_A = _tube("short_a", 0.06, SHORT_H, 0.045)
SHORT_B = _tube("short_b", -0.06, SHORT_H, 0.045)
TALL_B = _tube("tall_b", -0.15, TALL_H, 0.06)
TUBES = (TALL_A, SHORT_A, SHORT_B, TALL_B)

OBJECTS = [*BATH, *LINES, RACK, *TUBES]

MAX_TILT = 10.0  # degrees
OFF_TABLE = TOP - 0.05  # a tube whose bottom is below this has left the table


def _in_bath(s, spec) -> bool:
    """Any part of the tube's outline, seen from above, is inside the bath's walls, at any height."""
    return overlaps(s, spec, INTERIOR)


def _over_rack(s) -> bool:
    x, y = float(s.pos[0]), float(s.pos[1])
    return RACK_AREA.x0 <= x <= RACK_AREA.x1 and RACK_AREA.y0 <= y <= RACK_AREA.y1


def _support(s) -> float:
    """The rack's top over the rack, else the bath floor. The rack's height does not count over the open floor, so a
    short tube hanging in the band over the floor is caught."""
    return RACK_TOP if _over_rack(s) else FLOOR_Z


def _held_up(s, spec) -> bool:
    """More than 1 cm above the support under its centre."""
    return is_airborne(s, spec, [_support(s)], clearance=0.01)


def _in_band(s, spec) -> bool:
    """The top is in the band once the tube has dropped onto its support: the simulator cannot open the grippers, so a
    short tube held up to 1 cm over the floor with its top in the band is judged as the flooded tube it would become."""
    return BAND[0] <= top(s, spec) - max(0.0, bottom(s, spec) - _support(s)) <= BAND[1]


def _breaks_rule(s, spec) -> bool:
    """A tube in the bath whose top is outside the band (flooded, lying down, on the wrong support, stacked), or one
    hanging over the bath above its support: the simulator cannot open the grippers, so it never counts as placed."""
    return _in_bath(s, spec) and (not _in_band(s, spec) or _held_up(s, spec))


def _placed(s, spec) -> bool:
    """Wholly inside the walls, standing on its support, upright and still, top in the band."""
    return bool(
        inside(s, spec, INTERIOR)
        and not _held_up(s, spec)
        and tilt_deg(s) <= MAX_TILT
        and _in_band(s, spec)
        and s.resting()
    )


def score(objects):
    """0 if any tube in the bath ends with its top outside the band or held above its support, or any tube left the
    table. Otherwise 0.25 per tube placed."""
    tubes = [(objects[t.name], t) for t in TUBES]
    if any(_breaks_rule(s, t) or bottom(s, t) < OFF_TABLE for s, t in tubes):
        return 0.0
    return 0.25 * sum(_placed(s, t) for s, t in tubes)


def success(objects):
    """All four tubes placed and still."""
    return score(objects) == 1.0


# Checked offline with the URDF; the scorer itself only sees the objects. No arm part may enter the water, or pass below
# the wall tops over the walls or the ring around them. The ring starts at x 0.145, 2.5 cm in front of the near wall, so
# the bench grasps stay 5 mm outside it (a finger reaches x 0.14), and runs 4 cm beyond the other walls. These regions stop at the water and the walls rather
# than at TOP + 0.25, and the ring leaves the interior open, because the job is to reach into the bath from above: the
# hand has to be over it and between the walls, just never in the water. The water region runs up to the risen surface:
# with all four tubes in, the water rises from the line to about 6.5 cm, level with the pad tips at each set-down, so the
# finger pads may reach it; damp pads are acceptable.
RING_X = (0.145, 0.37)
RING_Y = 0.14
ARM_KEEP_OUT = [
    {"name": "the water", "x": (INTERIOR.x0, INTERIOR.x1), "y": (INTERIOR.y0, INTERIOR.y1), "z_top": WATER_LINE + 0.005},
    {"name": "the near wall and 2.5 cm before it", "x": (RING_X[0], INTERIOR.x0), "y": (-RING_Y, RING_Y), "z_top": WALL_TOP},
    {"name": "the far wall and 4 cm past it", "x": (INTERIOR.x1, RING_X[1]), "y": (-RING_Y, RING_Y), "z_top": WALL_TOP},
    {"name": "the left wall and 4 cm past it", "x": (INTERIOR.x0, INTERIOR.x1), "y": (INTERIOR.y1, RING_Y), "z_top": WALL_TOP},
    {"name": "the right wall and 4 cm past it", "x": (INTERIOR.x0, INTERIOR.x1), "y": (-RING_Y, INTERIOR.y0), "z_top": WALL_TOP},
]

# The demo. Open-loop plans have to allow for the arm's lag: the joint drives trail their commands by 0.06 s (shoulder)
# to 0.15 s (wrist), so at 0.2 m/s straight down the fingertips run about 14 mm to the side of the plan, and a pad that
# strikes a tube before the grip closes knocks it over. Light tubes also tip rather than slide when pushed high up, so a
# fast close or a hard squeeze tips or squeezes out what a slow one holds. Hence: fingers open to 8 cm at the bench (2.5
# cm each side of a tube) and only 5 cm over the bath; slow descents, set-downs and first centimetres of retreat; a pause
# to settle before each close and each release; a two-stage close, quick to 2.5 mm from the tube and then slow, to a target
# that holds the tube at about 30 N a side, below the 50 N cap where the two finger drives stop centring it; and one arm
# at a time, the other parked, because the hands' link4s ride ahead of them and come within 2 cm of each other when both
# hands work over the bench.
# Grip along y. A tall tube is gripped 1.5 cm below its top and a short one 1 cm below, so both are set down with the
# TCP at TOP + 0.078: 3 mm above rest (TOP + 0.075), over the walls, with the pad tips above the water line.
PICK_OPEN = -0.04
GRIP_OPEN = -0.025
PRE_CLOSE = -0.0175  # the pads 2.5 mm from the tube, before the slow part of the close
GRIP_HOLD = -0.009  # 6 mm past the tube's surface: about 30 N a side, under the drives' 50 N cap, so the tube stays centred
TRAVEL = TOP + 0.155  # a tall tube's underside clears the walls by 2 cm
CLEAR = 0.003
PADS_CLEAR = TOP + 0.113  # TCP height at which the pad tips are 1 cm above a tall tube's top
FLOOR_XY = {"left": (0.2175, 0.04), "right": (0.2175, -0.04)}  # centred between the near wall and the rack
RACK_XY = {"left": (0.285, 0.04), "right": (0.285, -0.04)}
PARK = (0.12, 0.25, TOP + 0.16)
PEAK_SPEED = {"descend": 0.10, "lift": 0.10, "carry": 0.15, "lower": 0.10, "retreat": 0.05}  # m/s, TCP; 0.20 elsewhere
MAX_SPEED = 0.20  # m/s: the fastest any TCP goes, under the 0.25 m/s the envelope declares
SETTLE_S = 0.4  # s: the hand stops before the fingers move
AT_LEAST = {"pre-close": 0.6, "close": 1.2, "release": 1.0}  # s, whatever the distance


def _timed(waypoints, start):
    """Pace each move by its length: a smoothstep peaks at 1.5 times its average speed."""
    out, prev = [], np.asarray(start, float)
    for w in waypoints:
        pos = np.asarray(w.pos, float)
        need = 1.5 * float(np.linalg.norm(pos - prev)) / PEAK_SPEED.get(w.label, MAX_SPEED)
        out.append(w if w.label == "settle" else replace(w, duration=max(need, AT_LEAST.get(w.label, 0.5))))
        prev = pos
    return out


def _move(tube, below_top, spot, start):
    """One tube to one spot: fingers wide at the bench, 5 cm over the bath, a settle before each close and release, and
    a slow start to the retreat while the pads are beside the tube."""
    grasp = (*tube.pos[:2], tube.pos[2] + tube.height / 2 - below_top)
    place = (*spot, TOP + 0.075 + CLEAR)
    plan = []
    for w in pick_place_waypoints(grasp, place, travel_z=TRAVEL):
        if w.label in ("to the object", "descend"):
            w = replace(w, finger=PICK_OPEN)
        elif w.label in ("release", "retreat"):
            w = replace(w, finger=GRIP_OPEN)
        else:
            w = replace(w, finger=GRIP_HOLD)
        if w.label == "retreat":
            plan += [replace(w, pos=(*spot, PADS_CLEAR)), replace(w, label="clear the bath")]
            continue
        if w.label in ("close", "release"):
            plan.append(replace(plan[-1], duration=SETTLE_S, label="settle"))
        if w.label == "close":
            plan.append(replace(w, finger=PRE_CLOSE, label="pre-close"))
        plan.append(w)
    return _timed(plan, start)


def _arm(side, sign, wait=0.0):
    """One arm's two tubes, short then tall, then back to its park. ``wait`` holds it at the park first."""
    short, tall = (SHORT_A, TALL_A) if side == "left" else (SHORT_B, TALL_B)
    park = (PARK[0], sign * PARK[1], PARK[2])
    start = (0.323, sign * 0.305, 1.008)
    plan = []
    if wait:
        plan += _timed([Waypoint(0, park, PICK_OPEN, "park")], start) + [Waypoint(wait, park, PICK_OPEN, "settle")]
        start = park
    plan += _move(short, 0.01, RACK_XY[side], start)
    plan += _move(tall, 0.015, FLOOR_XY[side], plan[-1].pos)
    return plan + _timed([Waypoint(0, park, GRIP_OPEN, "park")], plan[-1].pos)


def demo():
    """One arm at a time. The right arm parks beside the bench while the left sets its short tube on the rack and its
    tall tube on the floor and parks too; then the right arm does the same on its side. The fingers go no lower than
    just above the water line."""
    left = _arm("left", 1)
    right_park = _timed([Waypoint(0, (PARK[0], -PARK[1], PARK[2]), PICK_OPEN, "park")], (0.323, -0.305, 1.008))[0]
    return {"left": left, "right": _arm("right", -1, wait=sum(w.duration for w in left) - right_park.duration + 0.3)}
