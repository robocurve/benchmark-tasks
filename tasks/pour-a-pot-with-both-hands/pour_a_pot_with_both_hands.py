"""On the station table, between the arms, a portable single hob (black glass top 20 cm square, 4.5 cm tall, a burner
ring and a knob; switched off). On it stands a full two-handled stainless pot: inside 18 cm across and 12 cm deep (3 L),
1.2 kg empty, with a rolled rim and two flat cast-steel loop handles, one sticking out to the left and one to the right.
In it is the soup, 1.8 L: in a real trial 1,800 ml of room-temperature water tinted orange; here 120 orange beads (24 mm,
15 g each), each standing for 15 ml of it. The full pot weighs 3.0 kg. In front of the hob stands a white soup tureen
(inside 20 cm across and 8.7 cm deep, 2.7 L), fixed to the table. The pot and the beads are loose; the hob and the
tureen are fixed.

Task: pour all of the soup into the tureen without spilling and set the pot back on the hob. The full pot is too heavy
to hold by one handle: it swings down about that handle. Scoring is by volume transferred and spilled: a real trial
weighs the tureen and the pot before and after (1 g = 1 ml); here each bead counts as 15 ml.

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
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- layout (world frame: x forward from the arm bases, y left, z up; metres) ----------------------------------------
HOB_XY, HOB_HALF, HOB_H = (0.18, 0.0), 0.100, 0.045  # portable hob: 20 cm square, 4.5 cm tall
HOB_TOP = TOP + HOB_H

R_IN, WALL_T, WALL_H, FLOOR_T = 0.090, 0.004, 0.120, 0.006  # pot: inside radius, wall, wall height, floor
R_OUT = R_IN + WALL_T
POT_H = FLOOR_T + WALL_H
POT_MASS = 1.2  # empty, handles included
# Side handles: a horizontal U of 16 mm cast steel lying flat on each side, its two posts 6.4 cm apart (centres) and its
# outer bar, running front to back, 3.5 cm out from the wall (outer face), leaving a 1.9 cm gap inside.
BAR_R, POST_X, BAR_Y, HANDLE_Z = 0.008, 0.032, R_OUT + 0.027, 0.100  # rod radius, post offset, bar axis, height

BEAD_R, BEAD_MASS, N_BEADS = 0.012, 0.015, 120  # 120 x 15 g = 1.8 kg of soup
ML_PER_BEAD = 15  # each bead stands for 15 ml of the real trial's 1,800 ml of tinted water
SOUP_KG = N_BEADS * BEAD_MASS

TUREEN_XY, TUREEN_R_IN, TUREEN_T, TUREEN_FLOOR, TUREEN_H = (0.40, 0.0), 0.100, 0.006, 0.008, 0.095

STEEL, HANDLE_STEEL, DARK, GLASS = (0.60, 0.61, 0.64), (0.44, 0.45, 0.47), (0.10, 0.10, 0.11), (0.03, 0.03, 0.035)
SOUP, WHITE, RING = (0.93, 0.45, 0.10), (0.94, 0.94, 0.92), (0.45, 0.45, 0.47)

# -- scoring --------------------------------------------------------------------------------------------------------
FULL_IN, FULL_SPILL = 102, 5  # at least 1,530 ml (85 %) in the tureen, at most 75 ml spilled
HALF_IN, HALF_SPILL = 60, 12  # at least 900 ml (50 %), at most 180 ml spilled
SOME_IN = 12  # at least 180 ml (10 %)
BACK_OFF, BACK_GAP, UPRIGHT = 0.03, 0.01, math.radians(10)  # back on the hob: centre, underside gap, tilt
TIPPED = math.radians(30)  # tilted this far the pot has been tipped
LOW = 0.06  # tipping it while its body (base or rim) is this close to the table (on the hob, say) scores 0
SUCCESS_HOLD_S = 1.0


def _ring_boxes(r_mid, thick, z0, h, n, color, collide=True):
    chord = 2 * (r_mid + thick / 2) * math.tan(math.pi / n) + 0.001
    return [BoxShape((r_mid * math.cos(a), r_mid * math.sin(a), z0 + h / 2), (thick, chord, h), color, yaw=a,
                     collide=collide) for a in (2 * math.pi * i / n for i in range(n))]


def _pot() -> Body:
    shapes = [CylShape((0.0, 0.0, FLOOR_T / 2), R_OUT, FLOOR_T, STEEL)]
    shapes += _ring_boxes(R_IN + WALL_T / 2, WALL_T, FLOOR_T, WALL_H, 32, STEEL)
    shapes += _ring_boxes(R_OUT + 0.002, 0.004, POT_H - 0.007, 0.007, 32, STEEL)  # rolled rim
    root_y = math.sqrt(R_OUT ** 2 - POST_X ** 2) - 0.003  # the posts start just inside the wall
    for side in (1, -1):
        post_len = BAR_Y - root_y
        for px in (POST_X, -POST_X):
            shapes.append(CylShape((px, side * (root_y + post_len / 2), HANDLE_Z), BAR_R, post_len, HANDLE_STEEL,
                                   axis="Y"))
            shapes.append(BoxShape((px, side * (root_y + 0.003), HANDLE_Z), (0.024, 0.003, 0.028), HANDLE_STEEL,
                                   collide=False))  # riveted plate
        shapes.append(CylShape((0.0, side * BAR_Y, HANDLE_Z), BAR_R, 2 * (POST_X + BAR_R), HANDLE_STEEL, axis="X"))
    return Body.build("pot", pos=(*HOB_XY, HOB_TOP + 0.0005), mass=POT_MASS, shapes=shapes, friction=(0.8, 0.7),
                      color=STEEL)


def _bead_spots() -> list[tuple[float, float, float]]:
    """Pot-frame centres: hexagonal layers on the floor, each in the hollows of the one below, N_BEADS in all."""
    d = 2 * BEAD_R + 0.0006
    rmax = R_IN - BEAD_R - 0.002
    spots = []
    for layer in range(12):
        z = FLOOR_T + BEAD_R + 0.0005 + layer * d * math.sqrt(2 / 3)
        ox, oy = (d / 2, d / (2 * math.sqrt(3))) if layer % 2 else (0.0, 0.0)
        pts = []
        for j in range(-10, 11):
            for i in range(-10, 11):
                x, y = ox + (i + 0.5 * (j % 2)) * d, oy + j * d * math.sqrt(3) / 2
                if math.hypot(x, y) <= rmax:
                    pts.append((x, y, z))
        pts.sort(key=lambda p: (round(math.hypot(p[0], p[1]), 6), p[0], p[1]))
        spots += pts
        if len(spots) >= N_BEADS:
            break
    return spots[:N_BEADS]


def _beads() -> list[Body]:
    px, py = HOB_XY
    z0 = HOB_TOP + 0.0005
    return [Body.build(f"soup_{i:03d}", pos=(px + x, py + y, z0 + z), mass=BEAD_MASS, color=SOUP, friction=(0.3, 0.25),
                       shapes=[SphereShape((0.0, 0.0, 0.0), BEAD_R, SOUP)], damping=(0.05, 0.3))
            for i, (x, y, z) in enumerate(_bead_spots())]


def _hob() -> Body:
    """A portable single hob, fixed: dark body, black glass top, a grey burner ring, a knob on its front."""
    shapes = [BoxShape((0.0, 0.0, (HOB_H - 0.004) / 2), (2 * HOB_HALF, 2 * HOB_HALF, HOB_H - 0.004), DARK),
              BoxShape((0.0, 0.0, HOB_H - 0.002), (2 * HOB_HALF - 0.004, 2 * HOB_HALF - 0.004, 0.004), GLASS),
              CylShape((0.0, 0.0, HOB_H + 0.0003), 0.088, 0.0006, RING, collide=False),
              CylShape((0.0, 0.0, HOB_H + 0.0006), 0.082, 0.0006, GLASS, collide=False),
              CylShape((-HOB_HALF - 0.008, -0.06, 0.022), 0.015, 0.016, RING, collide=False, axis="X"),
              BoxShape((-HOB_HALF - 0.0165, -0.06, 0.030), (0.002, 0.004, 0.012), WHITE, collide=False)]
    return Body.build("hob", pos=(*HOB_XY, TOP), mass=2.0, shapes=shapes, kinematic=True, color=DARK)


def _tureen() -> Body:
    """A white soup tureen, fixed: a floor, 32 wall segments, a rolled rim and two small side lugs."""
    wall_h = TUREEN_H - TUREEN_FLOOR
    shapes = [CylShape((0.0, 0.0, TUREEN_FLOOR / 2), TUREEN_R_IN + TUREEN_T, TUREEN_FLOOR, WHITE),
              CylShape((0.0, 0.0, 0.002), 0.080, 0.004, (0.85, 0.85, 0.83), collide=False)]  # foot ring
    shapes += _ring_boxes(TUREEN_R_IN + TUREEN_T / 2, TUREEN_T, TUREEN_FLOOR, wall_h, 32, WHITE)
    shapes += _ring_boxes(TUREEN_R_IN + TUREEN_T + 0.002, 0.004, TUREEN_H - 0.008, 0.008, 32, WHITE)
    for side in (1, -1):
        shapes.append(BoxShape((0.0, side * (TUREEN_R_IN + TUREEN_T + 0.012), TUREEN_H - 0.018), (0.045, 0.022, 0.012),
                               WHITE, collide=False))
    return Body.build("tureen", pos=(*TUREEN_XY, TOP), mass=1.5, shapes=shapes, kinematic=True, color=WHITE)


POT = _pot()
BEADS = _beads()
OBJECTS = [_hob(), POT, *BEADS, _tureen()]
BEAD_NAMES = [b.name for b in BEADS]


# -- scoring --------------------------------------------------------------------------------------------------------

_EPISODE = {"tipped_low": False}


@on_reset
def _new_episode() -> None:
    _EPISODE.update(tipped_low=False)


def _tally(objects) -> dict:
    """Beads in the tureen, in the pot and spilled (by bead centre), and how the pot stands."""
    pot = objects["pot"]  # pos: the centre of the pot's underside
    rot = rotation(pot.quat_xyzw)
    in_tureen = in_pot = 0
    for name in BEAD_NAMES:
        b = np.asarray(objects[name].pos, float)
        p = rot.T @ (b - pot.pos)
        if math.hypot(p[0], p[1]) < R_IN and 0.0 < p[2] < POT_H:
            in_pot += 1
        elif math.hypot(b[0] - TUREEN_XY[0], b[1] - TUREEN_XY[1]) < TUREEN_R_IN and b[2] < TOP + TUREEN_H:
            in_tureen += 1
    tilt = float(math.acos(max(-1.0, min(1.0, rot[2, 2]))))
    side = R_OUT * math.hypot(rot[2, 0], rot[2, 1])  # how far a rim dips below its centre
    lowest = float(min(pot.pos[2], pot.pos[2] + POT_H * rot[2, 2]) - side)
    if tilt >= TIPPED and lowest < TOP + LOW:
        _EPISODE["tipped_low"] = True
    back = (math.hypot(pot.pos[0] - HOB_XY[0], pot.pos[1] - HOB_XY[1]) <= BACK_OFF
            and abs(pot.pos[2] - HOB_TOP) <= BACK_GAP and tilt < UPRIGHT)
    return {"in_tureen": in_tureen, "in_pot": in_pot, "spilled": N_BEADS - in_tureen - in_pot, "tilt": tilt,
            "back": back, "tipped_low": _EPISODE["tipped_low"], "pot": pot.pos}


def _points(t) -> float:
    if t["tipped_low"] or t["tilt"] >= TIPPED:
        return 0.0
    if t["in_tureen"] >= FULL_IN and t["spilled"] <= FULL_SPILL and t["back"]:
        return 1.0
    if t["in_tureen"] >= HALF_IN and t["spilled"] <= HALF_SPILL:
        return 0.5
    return 0.25 if t["in_tureen"] >= SOME_IN else 0.0


_DEBUG = {"on": bool(os.environ.get("POT_DEBUG")), "calls": 0}


def score(objects) -> float:
    """0 if the pot was ever tipped 30 degrees or more with the lowest point of its body (base or rim) within 6 cm
    of the table (tipped over on the hob rather than lifted), or ends tilted 30 degrees or more. Otherwise 1.0 with
    at least 102 of the 120 beads (1,530 of 1,800 ml) in the tureen, at most 5 (75 ml) spilled and the pot back upright
    on the hob; 0.5 with at least 60 (900 ml) in the tureen and at most 12 (180 ml) spilled; 0.25 with at least 12
    (180 ml) in the tureen; else 0. Each bead stands for 15 ml; spilled means outside both the pot and the tureen."""
    t = _tally(objects)
    if _DEBUG["on"]:  # POT_DEBUG=1: two lines a second (at 20 Hz) on stderr
        _DEBUG["calls"] += 1
        if _DEBUG["calls"] % 10 == 0:
            print(f"[pot] t={_DEBUG['calls'] / 20:5.1f}s tureen={t['in_tureen']} pot={t['in_pot']} "
                  f"spilled={t['spilled']} (ml: {t['in_tureen'] * ML_PER_BEAD}/{t['in_pot'] * ML_PER_BEAD}/"
                  f"{t['spilled'] * ML_PER_BEAD}) tilt={math.degrees(t['tilt']):.0f}deg pos=({t['pot'][0]:.3f},"
                  f"{t['pot'][1]:.3f},{t['pot'][2] - TOP:.3f}) back={t['back']} tipped_low={t['tipped_low']} "
                  f"score={_points(t)}", file=sys.stderr, flush=True)
    return _points(t)


def success(objects) -> bool:
    """Full score with the pot and every bead still."""
    if _points(_tally(objects)) < 1.0:
        return False
    return all(objects[n].resting() for n in ("pot", *BEAD_NAMES))


# -- scripted demo --------------------------------------------------------------------------------------------------
# Each arm turns level in front of its handle, pointing at the pot with its fingers one above and one below the
# handle's plane, slides back so the handle's outer bar comes between them (the fingertips inside the U) and closes on
# the bar, so the lower finger carries the pot's weight. Pouring turns both grips about the line through the two
# handles (world y), which the wrists do by rolling, so the TCPs stay put while the pot tips. The pot (and the two grips
# with it) follows one shared path of (seconds, grip x, handle height, tip).


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


ROLL = {"left": math.pi / 2, "right": -math.pi / 2}  # about each approach axis: fingers above and below the handle
SIDE_GRIP = {  # level grips reaching in from each arm's own side, fingers closing along z
    "left": _rx(-math.pi / 2) @ grip_rotation(-math.pi / 2) @ _rz(ROLL["left"]),  # pointing -y
    "right": _rx(math.pi / 2) @ grip_rotation(math.pi / 2) @ _rz(ROLL["right"]),  # pointing +y
}
GRIP_Y = BAR_Y  # TCP's distance from the pot's axis: on the outer bar, the fingertips inside the U
FINGER_PRE = -0.030  # pads 6 cm apart, either side of the handle's plane
FRONT_X = HOB_XY[0] + 0.08  # where the grips start and end: in front of the handles (the U ends 4 cm in front)
BAR_WORLD_Z = HOB_TOP + HANDLE_Z
ABOVE = 0.08  # the grips turn level this far above the handles' plane
CARRY_Z = TOP + 0.23  # handle height wanted while carrying (the pot's underside 13 cm up, clear of the rim)
# The arms sag under the full 3 kg pot (stock gains: 4-5 cm at the handles), so the waypoints aim higher: 4 cm while
# lifting and carrying, 2 cm while pouring (the lip then dips just inside the tureen's rim), 3 cm with the pot empty.
LIFT_Z, POUR_Z, EMPTY_Z = CARRY_Z + 0.04, CARRY_Z + 0.02, CARRY_Z + 0.03
POUR_X = TUREEN_XY[0] - 0.015  # pot axis while pouring: the lip ends up just behind the tureen's centre
POUR_TILT = math.radians(110)
LEAD_S, LEAD_STEPS = 3.0, 15

# After the lead-in, (seconds, grip x, handle height, tip angle, fingers, label) for both grips together
PATH = [
    (1.2, FRONT_X, BAR_WORLD_Z, 0.0, "pre", "down in front of the handles"),
    (1.5, HOB_XY[0], BAR_WORLD_Z, 0.0, "pre", "slide back round the handle bars"),
    (1.0, HOB_XY[0], BAR_WORLD_Z, 0.0, "closed", "grip the bars"),
    (2.0, HOB_XY[0], LIFT_Z, 0.0, "closed", "lift together"),
    (2.5, POUR_X, LIFT_Z, 0.0, "closed", "carry to the tureen"),
    (2.0, POUR_X, POUR_Z, math.radians(60), "closed", "tip"),
    (2.5, POUR_X, POUR_Z, POUR_TILT, "closed", "pour"),
    (4.0, POUR_X, POUR_Z, POUR_TILT, "closed", "let it drain"),
    (2.5, POUR_X, EMPTY_Z, 0.0, "closed", "tip back"),
    (2.5, HOB_XY[0], EMPTY_Z, 0.0, "closed", "carry back"),
    (2.0, HOB_XY[0], BAR_WORLD_Z + 0.002, 0.0, "closed", "set down on the hob"),
    (0.8, HOB_XY[0], BAR_WORLD_Z + 0.002, 0.0, "pre", "let go"),
    (1.5, FRONT_X, BAR_WORLD_Z + 0.002, 0.0, "pre", "slide out forward"),
    (1.5, FRONT_X, BAR_WORLD_Z + ABOVE, 0.0, "open", "up and clear"),
]
FINGERS = {"open": FINGER_OPEN, "pre": FINGER_PRE, "closed": FINGER_CLOSED}


def _good_ik(arm: str, pos, rot) -> np.ndarray:
    """A joint solution for this TCP pose with the most room to every joint limit (multi-start IK)."""
    import itertools

    from yam_arena import ik, station

    best = None
    for seed in itertools.product((-0.5, 0.0, 0.5), (0.8, 1.3), (0.4, 1.0), (-0.8, 0.0, 0.8), (-1.4, 0.0, 1.4),
                                  (-1.5, 0.0, 1.5)):
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.array(seed),
                             base_pos=station.BASES[arm], iters=200)
        if pe < 0.001 and re < math.radians(0.5):
            room = float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())
            if best is None or room > best[1]:
                best = (q, room)
    if best is None:
        raise ValueError(f"no IK solution for the {arm} grip at {pos}")
    return best[0]


def _arm(arm: str) -> list[Waypoint]:
    """The lead-in, as TCP waypoints along a straight line in joint space from the ready pose to a level grip above the
    start in front of the handle (so the IK, warm-started step by step, stays on that solution's branch), then PATH."""
    from yam_arena import ik, station
    from yam_arena.constants import READY

    side = 1.0 if arm == "left" else -1.0
    y = HOB_XY[1] + side * GRIP_Y
    above = (FRONT_X, y, BAR_WORLD_Z + ABOVE)
    q0, q1 = np.array(READY, float), _good_ik(arm, above, SIDE_GRIP[arm])
    out = []
    for k in range(1, LEAD_STEPS + 1):
        pos, rot = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(LEAD_S / LEAD_STEPS, tuple(float(v) for v in pos), FINGER_PRE,
                            "turn level in front of the handle", rot))
    for dt, x, z, tip, fingers, label in PATH:
        out.append(Waypoint(dt, (float(x), float(y), float(z)), FINGERS[fingers], label, _ry(tip) @ SIDE_GRIP[arm]))
    return out


def demo():
    return {"left": _arm("left"), "right": _arm("right")}
