"""On the station table lies a pharmacy counting tray. Ten counted tablets wait in the trough along its right side,
which opens at the back into a short chute; six more lie on the tray's main floor, kept out of the trough by a low
divider. Beside the tray stands an empty square amber vial with a wide mouth.

Task: pour the ten counted tablets, and only those, into the vial. As pharmacy staff are taught, one hand holds the vial
up to the tray's chute (off the counter, so no tablet bounces out) while the other lifts the tray by its grip fin and
tips it back, so the counted tablets slide down the trough and out of the chute; the extra ones stay behind on the
tray's floor. Too many or too few in the vial is a miscount.

Run it:  inspect-robots run --embodiment yam_arena -E task=pour_ten_counted_tablets_into_a_held_vial.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, on_reset, on_step, rotation  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the counting tray; its frame: origin at the middle of its floor's underside, x forward, y left --------------------
HALF_L, HALF_W = 0.045, 0.040  # the floor, 9 x 8 cm
FLOOR_T, RIM_H, WALL_T = 0.003, 0.015, 0.003  # rims 1.5 cm above the table, 3 mm thick
DIV_H = 0.011  # the divider between the main floor and the trough: 8 mm above the floor
TROUGH = (-HALF_W + WALL_T, -0.017)  # the trough's inside, across (y): 2 cm wide
TROUGH_Y = sum(TROUGH) / 2
DIV_Y = TROUGH[1] + WALL_T / 2
CHUTE_L = 0.035  # the chute, out of the back of the trough
TIP = np.array([-HALF_L - CHUTE_L, TROUGH_Y, FLOOR_T])  # the chute's lip
# the grip fin standing on the front rim, pointing forward: x from/to, thickness, z from/to
FIN = ((-0.003 + HALF_L, 0.095), 0.012, (0.012, 0.048))
GRIP = np.array([0.075, 0.0, 0.034])  # the demo's left TCP on the fin: 3 cm ahead of the rim, fingertips above it
TRAY_MASS = 0.06
TX, TY = 0.315, 0.04

# -- the vial: wide-mouthed amber, open; its frame: origin at the middle of its underside --------------------------
VIAL_R, VIAL_T, VIAL_H, VIAL_FLOOR = 0.025, 0.002, 0.045, 0.003
VIAL_MASS = 0.02
VX0, VY0 = 0.178, -0.065

# -- the tablets: film-coated discs, slick --------------------------------------------------------------------------
TAB_R, TAB_H, TAB_MASS, TAB_FRICTION = 0.004, 0.0035, 0.0003, 0.15
COUNTED = 10
TROUGH_SPOTS = [(x, y) for x in (0.030, 0.021, 0.012, 0.003, -0.006) for y in (TROUGH_Y + 0.0045, TROUGH_Y - 0.0045)]
EXTRA_SPOTS = [(x, y) for x in (0.020, -0.010) for y in (0.002, 0.014, 0.026)]

BLUE, AMBER, WHITE = (0.16, 0.42, 0.72), (0.62, 0.32, 0.06), (0.96, 0.96, 0.94)

# -- scoring ----------------------------------------------------------------------------------------------------------
HELD_UP = 0.02  # the vial's underside at least this far off the table when the first tablet goes in
SUCCESS_HOLD_S = 1.0


def _tray() -> Body:
    z_rim = RIM_H / 2
    back_y0 = TROUGH[1]  # the back rim stops at the trough, which opens into the chute
    chute_x, chute_len = -HALF_L - CHUTE_L / 2 + 0.0005, CHUTE_L + 0.001
    shapes = [
        BoxShape((0.0, 0.0, FLOOR_T / 2), (2 * HALF_L, 2 * HALF_W, FLOOR_T), BLUE),
        BoxShape((0.0, HALF_W - WALL_T / 2, z_rim), (2 * HALF_L, WALL_T, RIM_H), BLUE),  # left rim
        BoxShape((0.0, -HALF_W + WALL_T / 2, z_rim), (2 * HALF_L, WALL_T, RIM_H), BLUE),  # right rim
        BoxShape((HALF_L - WALL_T / 2, 0.0, z_rim), (WALL_T, 2 * HALF_W, RIM_H), BLUE),  # front rim
        BoxShape((-HALF_L + WALL_T / 2, (back_y0 + HALF_W) / 2, z_rim), (WALL_T, HALF_W - back_y0, RIM_H), BLUE),
        BoxShape((0.0, DIV_Y, DIV_H / 2), (2 * HALF_L, WALL_T, DIV_H), BLUE),  # divider
        BoxShape((chute_x, TROUGH_Y, FLOOR_T / 2), (chute_len, TROUGH[1] - TROUGH[0] + 2 * WALL_T, FLOOR_T), BLUE),
        BoxShape((chute_x, -HALF_W + WALL_T / 2, DIV_H / 2), (chute_len, WALL_T, DIV_H), BLUE),  # the chute's walls
        BoxShape((chute_x, DIV_Y, DIV_H / 2), (chute_len, WALL_T, DIV_H), BLUE),
        BoxShape(((FIN[0][0] + FIN[0][1]) / 2, 0.0, (FIN[2][0] + FIN[2][1]) / 2),
                 (FIN[0][1] - FIN[0][0], FIN[1], FIN[2][1] - FIN[2][0]), BLUE),  # grip fin
    ]
    return Body.build("tray", pos=(TX, TY, TOP + 0.0005), mass=TRAY_MASS, shapes=shapes, friction=(1.0, 0.9),
                      color=BLUE, com=(0.0, 0.0, 0.005))


def _vial() -> Body:
    """A square amber bottle: flat sides for the fingers (a round one squirts out of the grip)."""
    c = VIAL_R - VIAL_T / 2
    walls = [BoxShape((c, 0.0, (VIAL_FLOOR + VIAL_H) / 2), (VIAL_T, 2 * VIAL_R, VIAL_H - VIAL_FLOOR), AMBER),
             BoxShape((-c, 0.0, (VIAL_FLOOR + VIAL_H) / 2), (VIAL_T, 2 * VIAL_R, VIAL_H - VIAL_FLOOR), AMBER),
             BoxShape((0.0, c, (VIAL_FLOOR + VIAL_H) / 2), (2 * VIAL_R, VIAL_T, VIAL_H - VIAL_FLOOR), AMBER),
             BoxShape((0.0, -c, (VIAL_FLOOR + VIAL_H) / 2), (2 * VIAL_R, VIAL_T, VIAL_H - VIAL_FLOOR), AMBER)]
    shapes = [BoxShape((0.0, 0.0, VIAL_FLOOR / 2), (2 * VIAL_R, 2 * VIAL_R, VIAL_FLOOR), AMBER), *walls]
    return Body.build("vial", pos=(VX0, VY0, TOP + 0.0005), mass=VIAL_MASS, shapes=shapes, friction=(1.0, 0.9),
                      color=AMBER, com=(0.0, 0.0, 0.015))


def _tablets() -> list[Body]:
    out = []
    for i, (x, y) in enumerate(TROUGH_SPOTS + EXTRA_SPOTS):
        name = f"tablet_{i:02d}" if i < COUNTED else f"extra_{i - COUNTED:02d}"
        out.append(Body.build(name, pos=(TX + x, TY + y, TOP + 0.0005 + FLOOR_T + TAB_H / 2 + 0.0004), mass=TAB_MASS,
                              shapes=[CylShape((0.0, 0.0, 0.0), TAB_R, TAB_H, WHITE)], color=WHITE,
                              friction=(TAB_FRICTION, TAB_FRICTION), slick=True, damping=(0.05, 0.3)))
    return out


TRAY, VIAL = _tray(), _vial()
TABLETS = _tablets()
TABLET_NAMES = [t.name for t in TABLETS]
OBJECTS = [TRAY, VIAL, *TABLETS]


# -- followed after every control step: when the first tablet goes into the vial, and where the vial was then ----------

def _fresh() -> dict:
    return {"first_in": False, "held_up": False, "underside_then": None}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("TAB_DEBUG")), "steps": 0}


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0


def _where(t_pos, tray_pos, tray_rot, vial_pos, vial_rot) -> str:
    v = vial_rot.T @ (np.asarray(t_pos, float) - vial_pos)
    if max(abs(v[0]), abs(v[1])) < VIAL_R - VIAL_T and -0.002 < v[2] < VIAL_H + 0.004:
        return "vial"
    w = tray_rot.T @ (np.asarray(t_pos, float) - tray_pos)
    if -HALF_L - CHUTE_L <= w[0] <= HALF_L and -HALF_W <= w[1] <= HALF_W and -0.004 < w[2] < 0.03:
        return "tray"
    return "out"


def _counts(t_positions, tray_pos, tray_rot, vial_pos, vial_rot) -> dict:
    """Where the tablets are (in TABLET_NAMES order: the 10 counted ones first, then the extras)."""
    out = {"vial": 0, "tray": 0, "out": 0, "extras_in": 0}
    for i, p in enumerate(t_positions):
        w = _where(p, tray_pos, tray_rot, vial_pos, vial_rot)
        out[w] += 1
        if w == "vial" and i >= COUNTED:
            out["extras_in"] += 1
    return out


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])

    def pose(name):
        p = as_numpy(base.scene[name].data.root_link_pose_w)[0]
        return p[:3] - origin, rotation(p[3:7])

    tray_pos, tray_rot = pose("tray")
    vial_pos, vial_rot = pose("vial")
    tabs = [pose(n)[0] for n in TABLET_NAMES]
    c = _counts(tabs, tray_pos, tray_rot, vial_pos, vial_rot)
    if c["vial"] > 0 and not _EP["first_in"]:
        _EP["first_in"] = True
        _EP["underside_then"] = float(vial_pos[2] - TOP)
        _EP["held_up"] = _EP["underside_then"] >= HELD_UP
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            tip = tray_pos + tray_rot @ TIP
            tilt = math.degrees(math.atan2(tray_rot[2, 0], tray_rot[0, 0]))  # tipped back: positive
            print(f"[tab] t={_DEBUG['steps'] / 20:5.1f}s tray={np.round(tray_pos * 1000).astype(int).tolist()} tilt={tilt:5.1f} "
                  f"vial={np.round(vial_pos * 1000).astype(int).tolist()} tip-vial={np.round((tip - vial_pos) * 1000).astype(int).tolist()} "
                  f"counts={c} first_in={_EP['first_in']} underside_then={_EP['underside_then']} "
                  f"score={_points(c, _EP['held_up'])}", file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def _points(c: dict, held_up: bool) -> float:
    if c["vial"] == COUNTED and c["extras_in"] == 0 and c["out"] == 0 and held_up:
        return 1.0
    if COUNTED - 1 <= c["vial"] <= COUNTED + 1:
        return 0.5
    return 0.25 if c["vial"] >= 1 else 0.0


def score(objects) -> float:
    """1.0 with the 10 counted tablets in the vial and none of the 6 extras, none spilt, and the vial held up off the
    table (its underside 2 cm up) when the first one went in; 0.5 with 9 to 11 tablets in the vial; 0.25 with at least
    1; else 0.0."""
    tray, vial = objects["tray"], objects["vial"]
    c = _counts([objects[n].pos for n in TABLET_NAMES], np.asarray(tray.pos, float), rotation(tray.quat_xyzw),
                np.asarray(vial.pos, float), rotation(vial.quat_xyzw))
    return _points(c, _EP["held_up"])


def success(objects) -> bool:
    """Full score with the tray, the vial and every tablet still."""
    return score(objects) >= 1.0 and all(objects[n].resting() for n in ("tray", "vial", *TABLET_NAMES))


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The right hand comes straight down on the vial, fingers closing on its left and right, and holds it up where the
# chute's lip will be. The left hand comes straight down on the tray's grip fin, grips it and lifts the tray, then tips
# it back a little (6 degrees: the tablets stay put) and brings the chute in, between the right hand's fingers, until
# its lip is over the vial. Then it tips the tray on back to 22 degrees about the lip: the ten tablets in the trough
# slide out into the vial, the extras slide only to the back of the tray's floor. Then it tips the tray level again and
# draws it away.

GRIP_L = grip_rotation(0.0)  # straight down, fingers closing along y (across the fin)
GRIP_R = grip_rotation(0.0)  # straight down, fingers closing along y (left and right of the bottle: the chute comes in
# between them from the front)
VIAL_IN_HAND = 0.015  # the right TCP this far below the vial's rim
POUR_TILT, CARRY_TILT = math.radians(22.0), math.radians(6.0)
VIAL_AT = np.array([0.178, 0.013])  # where the held vial's middle should be
VIAL_UP = 0.034  # aim the held vial's underside this high (the arm gives about 6 mm under it)
VIAL_AIM = np.array([-0.008, 0.024])  # the held vial's middle from the right TCP (measured: the 5 cm grip does not
# centre the bottle, and the arm comes up short)
# The loaded arm tips the tray past the planned 22 degrees and lowers it, which puts the chute's lip about 1.7 cm
# further forward and 1 cm lower than planned: aim it that much back and up, and 1 cm more toward the vial's middle.
LIP_OVER = VIAL_AT + np.array([-0.027, 0.004])
LIP_Z = TOP + 0.091
LEAD_S, LEAD_STEPS = 3.0, 15


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


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


def _lead_in(arm: str, pos, rot, label: str, finger) -> list[Waypoint]:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    q0, q1 = np.array(READY, float), _good_ik(arm, pos, rot)
    out = []
    for k in range(1, LEAD_STEPS + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(LEAD_S / LEAD_STEPS, tuple(float(v) for v in p), finger, label, r))
    return out


def _w(t, pos, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in pos), finger, label, rot)


def _tray_grip(lip, tilt: float) -> tuple[np.ndarray, np.ndarray]:
    """The left TCP pose that puts the chute's lip at ``lip`` (world) with the tray tipped back ``tilt``."""
    r = _ry(-tilt)
    return np.asarray(lip, float) + r @ (GRIP - TIP), r @ GRIP_L


VIAL_HELD = np.array([VIAL_AT[0] - VIAL_AIM[0], VIAL_AT[1] - VIAL_AIM[1], TOP + VIAL_UP + VIAL_H - VIAL_IN_HAND])
IN_PLACE_AT = LEAD_S + 1.0 + 0.8 + 1.0 + 1.5


def _right() -> list[Waypoint]:
    pick = np.array([VX0, VY0, TOP + 0.0005 + VIAL_H - VIAL_IN_HAND])
    out = _lead_in("right", pick + [0.0, 0.0, 0.04], GRIP_R, "over the vial", FINGER_OPEN)
    out += [_w(1.0, pick, FINGER_OPEN, "down round it", GRIP_R),
            _w(0.8, pick, FINGER_CLOSED, "grip it", GRIP_R),
            _w(1.0, pick + [0.0, 0.0, VIAL_UP + 0.01], FINGER_CLOSED, "lift it", GRIP_R),
            _w(1.5, VIAL_HELD, FINGER_CLOSED, "hold it up where the chute will pour", GRIP_R)]
    return out


def _left() -> list[Waypoint]:
    fin = np.array([TX, TY, TOP + 0.0005]) + GRIP
    out = _lead_in("left", fin + [0.0, 0.0, 0.04], GRIP_L, "over the tray's grip fin", FINGER_OPEN)
    out += [_w(1.0, fin, FINGER_OPEN, "down round it", GRIP_L),
            _w(0.8, fin, FINGER_CLOSED, "grip it", GRIP_L)]
    out.append(_w(1.0, fin + [0.0, 0.0, 0.03], FINGER_CLOSED, "lift the tray off the table", GRIP_L))
    lip = np.array([LIP_OVER[0], LIP_OVER[1], LIP_Z])
    ahead = lip + [0.06, 0.0, 0.005]
    held, held_r = _tray_grip(ahead, CARRY_TILT)
    out.append(_w(1.5, held, FINGER_CLOSED, "lift it on, tipped back a little (the tablets stay put)", held_r))
    t = sum(w.duration for w in out)
    out.append(_w(max(IN_PLACE_AT - t, 0.2) + 0.3, held, FINGER_CLOSED, "wait for the vial to be held up", held_r))
    p, r = _tray_grip(lip + [0.0, 0.0, 0.002], CARRY_TILT)
    out.append(_w(2.0, p, FINGER_CLOSED, "bring the chute's lip over the vial", r))
    for k in range(1, 5):
        a = CARRY_TILT + (POUR_TILT - CARRY_TILT) * k / 4
        p, r = _tray_grip(lip, a)
        out.append(_w(0.5, p, FINGER_CLOSED, "tip it back: the counted tablets slide out", r))
    out.append(_w(3.0, p, FINGER_CLOSED, "let them all slide out", r))
    p, r = _tray_grip(lip + [0.0, 0.0, 0.002], CARRY_TILT)
    out.append(_w(1.5, p, FINGER_CLOSED, "tip it forward again", r))
    p, r = _tray_grip(ahead, CARRY_TILT)
    out.append(_w(1.5, p, FINGER_CLOSED, "draw the chute away", r))
    out.append(_w(3.0, p, FINGER_CLOSED, "hold still: the trial ends once the full score has held 1 s", r))
    return out


def demo():
    left, right = _left(), _right()
    t_l, t_r = sum(w.duration for w in left), sum(w.duration for w in right)
    right.append(_w(max(t_l - t_r, 0.5), right[-1].pos, FINGER_CLOSED, "hold the vial still", GRIP_R))
    return {"left": left, "right": right}
