"""On the station table, between the arms, a round black stove (a fixed block 16 cm across and 5 cm tall). On it stands a
steel stockpot (inside 12 cm across and 13 cm deep, 1 kg) with two flat upright lug handles, one sticking out to the
left and one to the right, each 5 cm long, 3 cm tall and 1.2 cm thick, near the rim. In the pot is the soup: 90 orange
beads (12 mm) in two layers on its floor. In front of the stove stands a white serving bowl (inside 16 cm across and
8 cm deep), fixed to the table. The pot and the beads are loose; the stove and the bowl are fixed.

Task: lift the pot off the stove by both handles, pour all of the soup into the bowl without spilling, and set the pot
back on the stove. The pot is too heavy to hold by one handle: it swings down about that handle.

Run it:  inspect-robots run --embodiment yam_arena -E task=pour_a_pot_with_both_hands.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import Body, BoxShape, CylShape, SphereShape, allow_matrix_grips, on_reset, rotation  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- layout (world frame: x forward from the arm bases, y left, z up; metres) ----------------------------------------
STOVE_XY, STOVE_R, STOVE_H = (0.17, 0.0), 0.080, 0.050
STOVE_TOP = TOP + STOVE_H

R_IN, WALL_T, WALL_H, FLOOR_T = 0.060, 0.006, 0.130, 0.006  # pot: inside radius, wall, wall height, floor
R_OUT = R_IN + WALL_T
POT_H = FLOOR_T + WALL_H
LUG_T, LUG_LEN, LUG_H, LUG_Z = 0.012, 0.050, 0.030, 0.115  # lug thickness (x), length (y), height, centre height
POT_MASS = 1.0

BEAD_R, BEAD_MASS, N_BEADS = 0.006, 0.002, 90

BOWL_XY, BOWL_R_IN, BOWL_T, BOWL_H, BOWL_FLOOR = (0.34, 0.0), 0.080, 0.006, 0.085, 0.004

STEEL, DARK, SOUP, WHITE = (0.70, 0.71, 0.73), (0.08, 0.08, 0.09), (0.93, 0.45, 0.10), (0.93, 0.93, 0.91)

# -- scoring --------------------------------------------------------------------------------------------------------
FULL_IN, FULL_SPILL = 77, 4  # 85 % in the bowl, at most 4 spilled
HALF_IN, HALF_SPILL = 45, 9  # 50 %, at most 9 (10 %)
SOME_IN = 9  # 10 %
BACK_OFF, BACK_GAP, UPRIGHT = 0.03, 0.01, math.radians(10)  # back on the stove: centre, underside gap, tilt
TIPPED = math.radians(30)  # tilted this far the pot has been tipped
LOW = 0.06  # tipping it while any part of it is this close to the table (on the stove, say) scores 0
SUCCESS_HOLD_S = 1.0


def _pot() -> Body:
    shapes = [CylShape((0.0, 0.0, FLOOR_T / 2), R_OUT, FLOOR_T, STEEL)]
    n = 24
    chord = 2 * R_OUT * math.tan(math.pi / n) + 0.001
    for i in range(n):
        a = 2 * math.pi * i / n
        r = R_IN + WALL_T / 2
        shapes.append(BoxShape((r * math.cos(a), r * math.sin(a), FLOOR_T + WALL_H / 2), (WALL_T, chord, WALL_H), STEEL,
                               yaw=a))
    for side in (1, -1):
        shapes.append(BoxShape((0.0, side * (R_OUT + LUG_LEN / 2 - 0.001), LUG_Z), (LUG_T, LUG_LEN, LUG_H), DARK))
    return Body.build("pot", pos=(*STOVE_XY, STOVE_TOP + 0.0005), mass=POT_MASS, shapes=shapes, friction=(0.6, 0.5),
                      color=STEEL)


def _bead_spots() -> list[tuple[float, float, float]]:
    """Pot-frame centres: hexagonal layers on the floor, the second in the hollows of the first, N_BEADS in all."""
    d = 2 * BEAD_R + 0.0006
    rmax = R_IN - BEAD_R - 0.002
    spots = []
    for layer in range(3):
        z = FLOOR_T + BEAD_R + 0.0005 + layer * d * math.sqrt(2 / 3)
        ox, oy = (d / 2, d / (2 * math.sqrt(3))) if layer % 2 else (0.0, 0.0)
        pts = []
        for j in range(-8, 9):
            for i in range(-8, 9):
                x, y = ox + (i + 0.5 * (j % 2)) * d, oy + j * d * math.sqrt(3) / 2
                if math.hypot(x, y) <= rmax:
                    pts.append((x, y, z))
        pts.sort(key=lambda p: (round(math.hypot(p[0], p[1]), 6), p[0], p[1]))
        spots += pts
        if len(spots) >= N_BEADS:
            break
    return spots[:N_BEADS]


def _beads() -> list[Body]:
    px, py = STOVE_XY
    z0 = STOVE_TOP + 0.0005
    return [Body.build(f"soup_{i:02d}", pos=(px + x, py + y, z0 + z), mass=BEAD_MASS, color=SOUP, friction=(0.3, 0.25),
                       shapes=[SphereShape((0.0, 0.0, 0.0), BEAD_R, SOUP)], damping=(0.05, 0.3))
            for i, (x, y, z) in enumerate(_bead_spots())]


def _bowl() -> list:
    """A fixed bowl: a floor disc and 20 flat wall segments."""
    (cx, cy), n = BOWL_XY, 20
    r = BOWL_R_IN + BOWL_T / 2
    chord = 2 * (BOWL_R_IN + BOWL_T) * math.tan(math.pi / n) + 0.001
    parts = [Cylinder("bowl_floor", pos=(cx, cy, TOP + BOWL_FLOOR / 2), radius=BOWL_R_IN + BOWL_T, height=BOWL_FLOOR,
                      color=WHITE)]
    for k in range(n):
        a = 2 * math.pi * k / n
        parts.append(Box(f"bowl_{k:02d}", pos=(cx + r * math.cos(a), cy + r * math.sin(a), TOP + BOWL_H / 2),
                         size=(BOWL_T, chord, BOWL_H), yaw=a, color=WHITE))
    return parts


POT = _pot()
BEADS = _beads()
OBJECTS = [
    Cylinder("stove", pos=(*STOVE_XY, TOP + STOVE_H / 2), radius=STOVE_R, height=STOVE_H, color=DARK),
    POT,
    *BEADS,
    *_bowl(),
]
BEAD_NAMES = [b.name for b in BEADS]


# -- scoring --------------------------------------------------------------------------------------------------------

_EPISODE = {"tipped_low": False}


@on_reset
def _new_episode() -> None:
    _EPISODE.update(tipped_low=False)


def _tally(objects) -> dict:
    """Beads in the bowl, in the pot and spilled (by bead centre), and how the pot stands."""
    pot = objects["pot"]  # pos: the centre of the pot's underside
    rot = rotation(pot.quat_xyzw)
    in_bowl = in_pot = 0
    for name in BEAD_NAMES:
        b = np.asarray(objects[name].pos, float)
        p = rot.T @ (b - pot.pos)
        if math.hypot(p[0], p[1]) < R_IN and 0.0 < p[2] < POT_H:
            in_pot += 1
        elif math.hypot(b[0] - BOWL_XY[0], b[1] - BOWL_XY[1]) < BOWL_R_IN and b[2] < TOP + BOWL_H:
            in_bowl += 1
    tilt = float(math.acos(max(-1.0, min(1.0, rot[2, 2]))))
    side = R_OUT * math.hypot(rot[2, 0], rot[2, 1])  # how far a rim dips below its centre
    lowest = float(min(pot.pos[2], pot.pos[2] + POT_H * rot[2, 2]) - side)
    if tilt >= TIPPED and lowest < TOP + LOW:
        _EPISODE["tipped_low"] = True
    back = (math.hypot(pot.pos[0] - STOVE_XY[0], pot.pos[1] - STOVE_XY[1]) <= BACK_OFF
            and abs(pot.pos[2] - STOVE_TOP) <= BACK_GAP and tilt < UPRIGHT)
    return {"in_bowl": in_bowl, "in_pot": in_pot, "spilled": N_BEADS - in_bowl - in_pot, "tilt": tilt,
            "back": back, "tipped_low": _EPISODE["tipped_low"]}


def _points(t) -> float:
    if t["tipped_low"] or t["tilt"] >= TIPPED:
        return 0.0
    if t["in_bowl"] >= FULL_IN and t["spilled"] <= FULL_SPILL and t["back"]:
        return 1.0
    if t["in_bowl"] >= HALF_IN and t["spilled"] <= HALF_SPILL:
        return 0.5
    return 0.25 if t["in_bowl"] >= SOME_IN else 0.0


_DEBUG = {"on": bool(os.environ.get("POT_DEBUG")), "calls": 0}


def score(objects) -> float:
    """0 if the pot was ever tipped 30 degrees or more with any part of it within 6 cm of the table (tipped over on the
    stove rather than lifted), or ends tilted 30 degrees or more. Otherwise 1.0 with at least 77 of the 90 beads in the
    bowl, at most 4 spilled and the pot back upright on the stove; 0.5 with at least 45 in the bowl and at most 9
    spilled; 0.25 with at least 9 in the bowl; else 0."""
    t = _tally(objects)
    if _DEBUG["on"]:  # POT_DEBUG=1: one line a second (at 20 Hz) on stderr
        _DEBUG["calls"] += 1
        if _DEBUG["calls"] % 20 == 0:
            print(f"[pot] t={_DEBUG['calls'] / 20:5.1f}s bowl={t['in_bowl']} pot={t['in_pot']} spilled={t['spilled']} "
                  f"tilt={math.degrees(t['tilt']):.0f}deg back={t['back']} tipped_low={t['tipped_low']} "
                  f"score={_points(t)}", file=sys.stderr, flush=True)
    return _points(t)


def success(objects) -> bool:
    """Full score with the pot and every bead still."""
    if _points(_tally(objects)) < 1.0:
        return False
    return all(objects[n].resting() for n in ("pot", *BEAD_NAMES))


# -- scripted demo --------------------------------------------------------------------------------------------------
# Each arm reaches its lug from its own side, level, its fingers closing front and back across the lug. Pouring turns
# both grips about the line through the two lugs (world y), which the wrists do by rolling, so the TCPs stay put while
# the pot tips. The pot (and the two grips with it) follows one shared path of (seconds, pot x, lug height, tip).


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


SIDE_GRIP = {  # the TCP rotation of a level grip reaching in from the arm's own side, fingers closing along x
    "left": _rx(-math.pi / 2) @ grip_rotation(-math.pi / 2),  # pointing -y
    "right": _rx(math.pi / 2) @ grip_rotation(math.pi / 2),  # pointing +y
}
GRIP_Y = R_OUT + 0.005 + PAD_TIP_PAST_TCP  # TCP's distance from the pot's axis: pad tips 5 mm off the wall
# A level side grip only reaches within about 12 cm of the centre line, so the grips do not slide in from outside:
# each turns level with its fingers open on either side of where its lug will be, and comes straight down round it
# (the lug is 1.2 cm thick, the open fingers 9.4 cm apart).
LUG_WORLD_Z = STOVE_TOP + LUG_Z
ABOVE = 0.04  # the grips turn level this far above the lugs
CARRY_Z = TOP + 0.22  # lug height while carrying and pouring (the pot's underside 10.5 cm up, clear of the bowl's rim)
POUR_X = 0.32  # pot axis while pouring: the lip ends up just behind the bowl's centre
POUR_TILT = math.radians(110)
LEAD_S, LEAD_STEPS = 3.0, 15

# After the lead-in, (seconds, pot x, lug height, tip angle, fingers, label) for both grips together
PATH = [
    (1.2, STOVE_XY[0], LUG_WORLD_Z, 0.0, "open", "down round the lugs"),
    (0.8, STOVE_XY[0], LUG_WORLD_Z, 0.0, "closed", "grip the lugs"),
    (1.5, STOVE_XY[0], CARRY_Z, 0.0, "closed", "lift together"),
    (2.0, POUR_X, CARRY_Z, 0.0, "closed", "carry to the bowl"),
    (2.0, POUR_X, CARRY_Z, math.radians(60), "closed", "tip"),
    (2.5, POUR_X, CARRY_Z, POUR_TILT, "closed", "pour"),
    (4.0, POUR_X, CARRY_Z, POUR_TILT, "closed", "let it drain"),
    (2.5, POUR_X, CARRY_Z, 0.0, "closed", "tip back"),
    (2.0, STOVE_XY[0], CARRY_Z, 0.0, "closed", "carry back"),
    (1.5, STOVE_XY[0], LUG_WORLD_Z + 0.003, 0.0, "closed", "set down on the stove"),
    (0.8, STOVE_XY[0], LUG_WORLD_Z + 0.003, 0.0, "open", "let go"),
    (1.2, STOVE_XY[0], LUG_WORLD_Z + ABOVE, 0.0, "open", "up off the lugs"),
]


def _good_ik(arm: str, pos, rot) -> np.ndarray:
    """A joint solution for this TCP pose with the most room to every joint limit (multi-start IK)."""
    import itertools

    from yam_arena import ik, station

    best = None
    for seed in itertools.product((-0.5, 0.0, 0.5), (0.8, 1.3), (0.4, 1.0), (-0.8, 0.0, 0.8), (-1.4, 0.0, 1.4), (0.0,)):
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.array(seed), base_pos=station.BASES[arm],
                             iters=200)
        if pe < 0.001 and re < math.radians(0.5):
            room = float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())
            if best is None or room > best[1]:
                best = (q, room)
    if best is None:
        raise ValueError(f"no IK solution for the {arm} grip at {pos}")
    return best[0]


def _arm(arm: str) -> list[Waypoint]:
    """The lead-in, as TCP waypoints along a straight line in joint space from the ready pose to a level grip above the
    lug (so the IK, warm-started step by step, stays on that solution's branch), then PATH."""
    from yam_arena import ik, station
    from yam_arena.constants import READY

    side = 1.0 if arm == "left" else -1.0
    y = side * GRIP_Y
    above = (STOVE_XY[0], y, LUG_WORLD_Z + ABOVE)
    q0, q1 = np.array(READY, float), _good_ik(arm, above, SIDE_GRIP[arm])
    out = []
    for k in range(1, LEAD_STEPS + 1):
        pos, rot = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(LEAD_S / LEAD_STEPS, tuple(float(v) for v in pos), FINGER_OPEN, "turn level over the lug", rot))
    for dt, x, z, tip, fingers, label in PATH:
        out.append(Waypoint(dt, (float(x), float(y), float(z)), FINGER_OPEN if fingers == "open" else FINGER_CLOSED, label,
                            _ry(tip) @ SIDE_GRIP[arm]))
    return out


def demo():
    return {"left": _arm("left"), "right": _arm("right")}
