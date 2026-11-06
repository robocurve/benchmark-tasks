"""On the station table, a paper-towel roll stands on its end, loose, ahead of the arms and a little to the left. Two
sheets are unrolled from its front and stand on edge along the table to the right: the inner one still on the roll, the
outer one joined to it along a perforation. To the right, a blue mat on the table holds six yellow cubes (the crumbs).

Task: tear the outer sheet off with both hands, holding the roll (or the inner sheet) with one hand while the other pulls
the outer sheet away (pulled with one hand alone, the strip only drags the light roll along), then wipe every cube off
the blue mat with the torn sheet.

Run it:  inspect-robots run --embodiment yam_arena -E task=tear_a_paper_towel_and_wipe_the_table.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, hinge, on_reset, on_step, rotation  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the roll (origin at the centre of its underside) and the sheets (origin at the sheet's centre) -----------------
ROLL_XY = (0.30, 0.15)
ROLL_R, ROLL_H, ROLL_MASS = 0.040, 0.14, 0.12
CORE_R = 0.020
SHEET_L, SHEET_H, SHEET_T, SHEET_MASS = 0.12, 0.14, 0.004, 0.005  # along the strip, tall, thick
SHEET_LIFT = 0.0005  # the sheets stand on their lower edges on the table
SHEET_X = ROLL_XY[0] - ROLL_R - SHEET_T / 2  # the strip's plane: tangent to the roll's front
PERF_Y = ROLL_XY[1] - SHEET_L  # the perforation between the inner sheet (a) and the outer one (b)
TEAR_FORCE = 3.0  # N: the perforation breaks under this pull

# -- the mat and the cubes ----------------------------------------------------------------------------------------
MAT_XY, MAT_HALF = (0.23, -0.20), 0.07  # a 14 cm square
CUBE = 0.02
CUBE_MASS = 0.005
CUBE_XY = [(x, y) for x in (0.19, 0.23) for y in (-0.24, -0.20, -0.16)]

WHITE, CORE_C, PERF_C = (0.96, 0.96, 0.94), (0.62, 0.47, 0.32), (0.72, 0.72, 0.72)
MAT_C, CUBE_C = (0.20, 0.40, 0.85), (0.95, 0.80, 0.10)

# -- scoring ------------------------------------------------------------------------------------------------------
TORN = 0.01  # the two sheets' perforation edges this far apart: torn
NEAR_SHEET = 0.05  # a cube counts as wiped off if, when its centre leaves the mat, the torn sheet is within this ...
CLEAR_OF_HANDS = 0.06  # ... and neither gripper's fingertip centre (TCP) is within this of it
SUCCESS_HOLD_S = 1.0


def _roll() -> Body:
    shapes = [CylShape((0.0, 0.0, ROLL_H / 2), ROLL_R, ROLL_H, WHITE),
              CylShape((0.0, 0.0, ROLL_H + 0.0005), CORE_R, 0.001, CORE_C, collide=False)]
    return Body.build("roll", pos=(*ROLL_XY, TOP + 0.0005), mass=ROLL_MASS, shapes=shapes, friction=(0.5, 0.4),
                      color=WHITE)


def _sheet(name: str, y_centre: float, perforated_edge: float | None) -> Body:
    shapes = [BoxShape((0.0, 0.0, 0.0), (SHEET_T, SHEET_L, SHEET_H), WHITE)]
    if perforated_edge is not None:  # a faint line along the perforation, on both faces
        shapes.append(BoxShape((0.0, perforated_edge * (SHEET_L / 2 - 0.001), 0.0), (SHEET_T + 0.0006, 0.0015, SHEET_H),
                               PERF_C, collide=False))
    return Body.build(name, pos=(SHEET_X, y_centre, TOP + SHEET_LIFT + SHEET_H / 2), mass=SHEET_MASS, shapes=shapes,
                      friction=(1.0, 0.9), color=WHITE)


ROLL = _roll()
SHEET_A = _sheet("sheet_a", ROLL_XY[1] - SHEET_L / 2, -1.0)
SHEET_B = _sheet("sheet_b", ROLL_XY[1] - 1.5 * SHEET_L, +1.0)
hinge("roll_to_a", "roll", "sheet_a", (SHEET_X, ROLL_XY[1], TOP + SHEET_LIFT + SHEET_H / 2), "Z", limits_deg=(-80.0, 5.0))
hinge("perforation", "sheet_a", "sheet_b", (SHEET_X, PERF_Y, TOP + SHEET_LIFT + SHEET_H / 2), "Z",
      limits_deg=(-90.0, 90.0), break_force=TEAR_FORCE)  # parts under a pull, not under bending

CUBES = [Body.build(f"cube_{i}", pos=(x, y, TOP + 0.001 + CUBE / 2 + 0.0005), mass=CUBE_MASS, color=CUBE_C,
                    shapes=[BoxShape((0.0, 0.0, 0.0), (CUBE, CUBE, CUBE), CUBE_C)], friction=(0.4, 0.3))
         for i, (x, y) in enumerate(CUBE_XY)]
CUBE_NAMES = [c.name for c in CUBES]
OBJECTS = [Box("mat", pos=(*MAT_XY, TOP + 0.0005), size=(2 * MAT_HALF, 2 * MAT_HALF, 0.001), color=MAT_C),
           ROLL, SHEET_A, SHEET_B, *CUBES]


# -- scoring ------------------------------------------------------------------------------------------------------
# on_step keeps, for each cube, whether it is on the mat and whether its last way off the mat was a wipe with the
# sheet; score() reads that and whether the perforation has parted.

_EPISODE = {"on_mat": [True] * len(CUBES), "wiped": [False] * len(CUBES)}
_DEBUG = {"on": bool(os.environ.get("TEAR_DEBUG")), "calls": 0}


@on_reset
def _new_episode() -> None:
    _EPISODE.update(on_mat=[True] * len(CUBES), wiped=[False] * len(CUBES))


def _on_mat(p) -> bool:
    return abs(p[0] - MAT_XY[0]) < MAT_HALF and abs(p[1] - MAT_XY[1]) < MAT_HALF and p[2] < TOP + 0.05


def _to_sheet(p, sheet_pos, sheet_rot) -> float:
    """Distance (m) from point p to the sheet's slab."""
    local = sheet_rot.T @ (np.asarray(p, float) - sheet_pos)
    half = np.array([SHEET_T, SHEET_L, SHEET_H]) / 2
    return float(np.linalg.norm(local - np.clip(local, -half, half)))


def _fingertips(base, origin) -> list[np.ndarray]:
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        out.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
    return out


@on_step
def _watch_cubes(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    b = as_numpy(base.scene["sheet_b"].data.root_link_pose_w)[0]
    b_pos, b_rot = b[:3] - origin, rotation(b[3:7])
    tips = _fingertips(base, origin)
    for i, name in enumerate(CUBE_NAMES):
        p = as_numpy(base.scene[name].data.root_link_pose_w)[0][:3] - origin
        now = _on_mat(p)
        if _EPISODE["on_mat"][i] and not now:
            _EPISODE["wiped"][i] = (_to_sheet(p, b_pos, b_rot) <= NEAR_SHEET
                                    and min(np.linalg.norm(p - t) for t in tips) >= CLEAR_OF_HANDS)
        elif now:
            _EPISODE["wiped"][i] = False
        _EPISODE["on_mat"][i] = now


def _torn(objects) -> bool:
    a, b = objects["sheet_a"], objects["sheet_b"]
    edge_a = np.asarray(a.pos) + rotation(a.quat_xyzw) @ np.array([0.0, -SHEET_L / 2, 0.0])
    edge_b = np.asarray(b.pos) + rotation(b.quat_xyzw) @ np.array([0.0, SHEET_L / 2, 0.0])
    return float(np.linalg.norm(edge_a - edge_b)) > TORN


def _tally(objects) -> dict:
    off = [not _on_mat(objects[n].pos) and w for n, w in zip(CUBE_NAMES, _EPISODE["wiped"])]
    return {"torn": _torn(objects), "wiped": int(sum(off))}


def _points(t) -> float:
    if not t["torn"]:
        return 0.0
    if t["wiped"] == len(CUBES):
        return 1.0
    if t["wiped"] >= len(CUBES) // 2:
        return 0.5
    return 0.25


def score(objects) -> float:
    """0 unless the outer sheet has been torn off at its perforation. Otherwise 1.0 with all 6 cubes wiped off the blue
    mat by the torn sheet, 0.5 with at least 3, else 0.25. A cube counts as wiped off when it is off the mat now and,
    when its centre last left the mat, the torn sheet was within 5 cm of it and neither gripper's TCP within 6 cm."""
    t = _tally(objects)
    if _DEBUG["on"]:  # TEAR_DEBUG=1: one line a second (at 20 Hz) on stderr
        _DEBUG["calls"] += 1
        if _DEBUG["calls"] % 20 == 0:
            on_mat = sum(_on_mat(objects[n].pos) for n in CUBE_NAMES)
            print(f"[tear] t={_DEBUG['calls'] / 20:5.1f}s torn={t['torn']} on_mat={on_mat} wiped={t['wiped']} "
                  f"roll={np.round(np.asarray(objects['roll'].pos) * 100, 1).tolist()} score={_points(t)}",
                  file=sys.stderr, flush=True)
    return _points(t)


def success(objects) -> bool:
    """Full score with every cube still."""
    if _points(_tally(objects)) < 1.0:
        return False
    return all(objects[n].resting() for n in CUBE_NAMES)


# -- scripted demo --------------------------------------------------------------------------------------------------
# Both hands come straight down: the left one takes the roll by its top, fingers closing left and right across it; the
# right one pinches the outer sheet's upper edge 5 cm past the perforation, fingers closing front and back. The right
# hand pulls straight away along the strip until the perforation parts, lifts the torn sheet clear of the cubes, sets
# its lower edge down in front of the mat and pushes it forward across the mat. (The hands stay 17 cm apart: closer,
# the two wrists collide.)

PINCH = 0.004  # the TCP this far below the sheets' upper edge: pinched near the tips, where the jaws are apart
SHEET_TOP = TOP + SHEET_LIFT + SHEET_H
GRIP_Z = SHEET_TOP - PINCH
DOWN = {"left": grip_rotation(0.0), "right": grip_rotation(math.pi / 2)}  # straight down; fingers closing along y, x
ABOVE = 0.03
ROLL_GRIP = np.array([ROLL_XY[0], ROLL_XY[1], TOP + ROLL_H - 0.012])
B_PAST = 0.05  # the right hand pinches the outer sheet this far past the perforation
B_GRIP = np.array([SHEET_X, PERF_Y - B_PAST, GRIP_Z])
PULL = 0.05  # how far the right hand pulls along the strip
CARRY = 0.035  # lift the torn sheet this much to clear the cubes
WIPE_Y = MAT_XY[1] + (SHEET_L - 2 * B_PAST) / 2  # the sheet (5 cm to the grip's left, 7 to its right) then covers the cubes
WIPE_FROM, WIPE_TO = MAT_XY[0] - MAT_HALF - 0.03, MAT_XY[0] + MAT_HALF + 0.04
LEAD_S, LEAD_STEPS = 3.0, 15
WIPE_GAP = 0.005  # the torn sheet's lower edge this far above the table while wiping


def _good_ik(arm: str, pos, rot) -> np.ndarray:
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


def _lead_in(arm: str, pos, rot, label: str) -> list[Waypoint]:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    q0, q1 = np.array(READY, float), _good_ik(arm, pos, rot)
    out = []
    for k in range(1, LEAD_STEPS + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(LEAD_S / LEAD_STEPS, tuple(float(v) for v in p), FINGER_OPEN, label, r))
    return out


def _w(t, pos, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in pos), finger, label, rot)


GRAB = [1.2, 0.8]  # down round the edge, close


def _left() -> list[Waypoint]:
    rot = DOWN["left"]
    out = _lead_in("left", ROLL_GRIP + [0.0, 0.0, ABOVE], rot, "over the roll")
    out += [_w(GRAB[0], ROLL_GRIP, FINGER_OPEN, "down round its top", rot),
            _w(GRAB[1], ROLL_GRIP, FINGER_CLOSED, "grip it: hold the roll", rot),
            _w(16.0, ROLL_GRIP, FINGER_CLOSED, "keep holding", rot)]
    return out


def _right() -> list[Waypoint]:
    torn = B_GRIP + [0.0, -PULL, 0.0]
    start = np.array([WIPE_FROM, WIPE_Y, GRIP_Z - SHEET_LIFT + WIPE_GAP])
    rot = DOWN["right"]
    out = _lead_in("right", B_GRIP + [0.0, 0.0, ABOVE], rot, "over the outer sheet")
    out += [_w(GRAB[0], B_GRIP, FINGER_OPEN, "down round its upper edge", rot),
            _w(GRAB[1], B_GRIP, FINGER_CLOSED, "pinch it", rot),
            _w(0.4, B_GRIP, FINGER_CLOSED, "settle", rot),
            _w(1.2, torn, FINGER_CLOSED, "pull along the strip: tear at the perforation", rot),
            _w(0.8, torn + [0.0, 0.0, CARRY], FINGER_CLOSED, "lift the torn sheet", rot),
            _w(2.0, start + [0.0, 0.0, CARRY], FINGER_CLOSED, "carry it in front of the mat", rot),
            _w(1.0, start, FINGER_CLOSED, "lower edge down on the table", rot),
            _w(3.5, np.array([WIPE_TO, WIPE_Y, start[2]]), FINGER_CLOSED, "wipe forward across the mat", rot),
            _w(3.0, np.array([WIPE_TO, WIPE_Y, start[2]]), FINGER_CLOSED, "hold still", rot)]
    return out


def demo():
    return {"left": _left(), "right": _right()}
