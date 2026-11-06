"""On the station table, a wooden cutting board lies in front of the arms with twelve diced vegetables on it; a grip
bar runs across its left end. To its right stands a wide pot, fixed on the table, its rim 7 cm up; nearer the arms, a
small bench scraper stands blade down in a fixed rack.

Task: scrape the vegetables off the board into the pot. The wet pieces stick to the board (they do not slide off even
with the board on edge), and the pot's rim is above the board: one hand has to hold the board up with its end over the
pot while the other scrapes the pieces along it and off its end.

Run it:  inspect-robots run --embodiment yam_arena -E task=scrape_chopped_vegetables_off_a_board_into_a_pot.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, on_reset, on_step, plate, rotation  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
G = 9.81

# -- the board. Its frame: origin at the middle of its top face, x across it, y along it (the grip bar at +y) ---------
BOARD_L, BOARD_W, BOARD_T = 0.30, 0.14, 0.010
BAR_W, BAR_H = 0.025, 0.025  # the grip bar across the board's left end, on top of it
BOARD_MASS = 0.25
BOARD_X = 0.31
BOARD_Y0 = 0.059  # where its middle lies on the table: its right end 2 cm from the pot
BAR_Y = BOARD_L / 2 - BAR_W / 2  # the bar's middle, along the board

# -- the vegetables: 12 mm dice, 2 g each, in a 3 x 4 patch on the board ----------------------------------------------
PIECE, PIECE_MASS = 0.012, 0.002
PIECE_XS = (-0.022, 0.0, 0.022)
PIECE_YS = (-0.032, -0.052, -0.072, -0.092)  # along the board (its right end at -0.15)
STICK = 3.0  # a piece lying on the board's face is pulled onto it with this many times its weight (wet pieces stick)
VEG_COLORS = ((0.95, 0.50, 0.10), (0.45, 0.72, 0.25), (0.93, 0.90, 0.80))  # carrot, celery, onion

# -- the pot (fixed): origin at the middle of its underside -----------------------------------------------------------
POT_X, POT_Y = 0.31, -0.20
R_IN, WALL_T, POT_H, FLOOR_T = 0.085, 0.004, 0.070, 0.006
R_OUT = R_IN + WALL_T
RIM_Z = TOP + POT_H

# -- the bench scraper: a steel blade with a wooden bar along its top; origin at the middle of the blade's edge -------
SCRAPER_W, BLADE_T, BLADE_H, GRIP_S = 0.10, 0.002, 0.040, 0.022
SCRAPER_MASS, SCRAPER_COM = 0.09, (0.0, 0.0, 0.030)
RACK_X, RACK_Y, SLOT, RACK_BASE = 0.20, -0.33, 0.006, 0.006

WOOD, BAR_C, STEEL, DARK, RACK_C = (0.80, 0.64, 0.42), (0.55, 0.38, 0.22), (0.70, 0.71, 0.74), (0.20, 0.20, 0.22), \
    (0.25, 0.25, 0.27)

# -- scoring ----------------------------------------------------------------------------------------------------------
N = len(PIECE_XS) * len(PIECE_YS)
FULL_IN, FULL_SPILT = 10, 1  # 1.0: at least 10 of the 12 in the pot, at most 1 anywhere but the pot or the board
HALF_IN, HALF_SPILT = 6, 2
SOME_IN = 3
SUCCESS_HOLD_S = 1.0


def _wall(n: int, r_in: float, t: float, z0: float, z1: float, color) -> list:
    """A round wall of n panels, each as wide as the outer chord (neighbours overlap: no slots)."""
    r_mid, r_out = r_in + t / 2, r_in + t
    out = []
    for k in range(n):
        a = 2 * math.pi * k / n
        u = (math.cos(a), math.sin(a), 0.0)
        out.append(plate((r_mid * u[0], r_mid * u[1], z0), (r_mid * u[0], r_mid * u[1], z1),
                         2 * r_out * math.tan(math.pi / n) + 0.001, t, u, color))
    return out


def _pot() -> Body:
    shapes = [CylShape((0.0, 0.0, FLOOR_T / 2), R_IN + WALL_T / 2, FLOOR_T, DARK)]
    shapes += _wall(32, R_IN, WALL_T, 0.0, POT_H, DARK)
    shapes += [BoxShape((s * (R_OUT + 0.012), 0.0, POT_H - 0.018), (0.024, 0.050, 0.008), DARK) for s in (-1.0, 1.0)]
    return Body.build("pot", pos=(POT_X, POT_Y, TOP), mass=2.0, shapes=shapes, kinematic=True, friction=(0.5, 0.4),
                      color=DARK)


def _board() -> Body:
    shapes = [BoxShape((0.0, 0.0, -BOARD_T / 2), (BOARD_W, BOARD_L, BOARD_T), WOOD),
              BoxShape((0.0, BAR_Y, BAR_H / 2), (BOARD_W, BAR_W, BAR_H), BAR_C)]
    return Body.build("board", pos=(BOARD_X, BOARD_Y0, TOP + BOARD_T + 0.0005), mass=BOARD_MASS, shapes=shapes,
                      friction=(0.8, 0.7), color=WOOD)


def _pieces() -> list[Body]:
    out = []
    k = 0
    for y in PIECE_YS:
        for x in PIECE_XS:
            c = VEG_COLORS[k % len(VEG_COLORS)]
            out.append(Body.build(f"veg_{k:02d}", pos=(BOARD_X + x, BOARD_Y0 + y, TOP + BOARD_T + PIECE / 2 + 0.001),
                                  mass=PIECE_MASS, shapes=[BoxShape((0.0, 0.0, 0.0), (PIECE, PIECE, PIECE), c)],
                                  friction=(1.0, 0.9), color=c, yaw=0.3 * ((k * 7) % 5 - 2)))
            k += 1
    return out


def _scraper() -> Body:
    shapes = [BoxShape((0.0, 0.0, BLADE_H / 2), (SCRAPER_W, BLADE_T, BLADE_H), STEEL),
              BoxShape((0.0, 0.0, BLADE_H + GRIP_S / 2), (SCRAPER_W, GRIP_S, GRIP_S), BAR_C)]
    return Body.build("scraper", pos=(RACK_X, RACK_Y, TOP + RACK_BASE + 0.0005), mass=SCRAPER_MASS, shapes=shapes,
                      friction=(0.4, 0.3), color=STEEL, com=SCRAPER_COM)


def _rack() -> Body:
    shapes = [BoxShape((0.0, 0.0, RACK_BASE / 2), (0.13, 0.05, RACK_BASE), RACK_C)]
    shapes += [BoxShape((0.0, s * (SLOT / 2 + 0.0075), RACK_BASE + 0.0125), (0.13, 0.015, 0.025), RACK_C)
               for s in (-1.0, 1.0)]
    return Body.build("rack", pos=(RACK_X, RACK_Y, TOP), mass=1.0, shapes=shapes, kinematic=True, friction=(0.3, 0.3),
                      color=RACK_C)


POT = _pot()
RACK = _rack()
BOARD = _board()
PIECES = _pieces()
SCRAPER = _scraper()
OBJECTS = [POT, RACK, BOARD, *PIECES, SCRAPER]
PIECE_NAMES = [p.name for p in PIECES]


# -- the wet pieces stick to the board: after every control step, each piece lying on the board's face gets a pull onto
# it of STICK times its weight (in the piece's own frame, held until the next step) ------------------------------------

def _on_board(board_pos, board_rot, p) -> bool:
    local = board_rot.T @ (np.asarray(p, float) - np.asarray(board_pos, float))
    return (abs(local[0]) < BOARD_W / 2 and -BOARD_L / 2 < local[1] < BOARD_L / 2 - BAR_W
            and 0.0 < local[2] < PIECE * 0.75 + 0.002)


_EP = {"tcps": None}
_DEBUG = {"on": bool(os.environ.get("SCRAPE_DEBUG")), "steps": 0}


@on_reset
def _new_episode() -> None:
    _EP["tcps"] = None
    _DEBUG["steps"] = 0


def _tcps(base, origin) -> list[np.ndarray]:
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        out.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
    return out


@on_step
def _stick(env) -> None:
    import torch

    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    b = as_numpy(base.scene["board"].data.root_link_pose_w)[0]
    bpos, brot = b[:3] - origin, rotation(b[3:7])
    pull = -STICK * PIECE_MASS * G * brot[:, 2]  # world frame: into the board's face
    stuck = 0
    for n in PIECE_NAMES:
        asset = base.scene[n]
        p = as_numpy(asset.data.root_link_pose_w)[0]
        f = rotation(p[3:7]).T @ pull if _on_board(bpos, brot, p[:3] - origin) else np.zeros(3)
        stuck += bool(f.any())
        t = torch.tensor(f, dtype=torch.float32, device=asset.device).view(1, 1, 3)
        asset.permanent_wrench_composer.set_forces_and_torques(forces=t, torques=torch.zeros_like(t), is_global=False)
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            _EP["tcps"] = _tcps(base, origin)
            s = as_numpy(base.scene["scraper"].data.root_link_pose_w)[0]
            srot = rotation(s[3:7])
            local = brot.T @ (s[:3] - origin - bpos)
            states = {n: as_numpy(base.scene[n].data.root_link_pose_w)[0][:3] - origin for n in PIECE_NAMES}
            t = _tally_from(bpos, brot, states)
            tl, tr = _EP["tcps"]
            bar = bpos + brot @ np.array([0.0, BAR_Y, BAR_H / 2])
            print(f"[scrape] t={_DEBUG['steps'] / 20:5.1f}s board={np.round(bpos * 1000).astype(int).tolist()} "
                  f"tilt={math.degrees(math.acos(min(1.0, brot[2, 2]))):4.1f} stuck={stuck} in_pot={t['in_pot']} "
                  f"on_board={t['on_board']} spilt={t['spilt']} scraper_in_board={np.round(local * 1000).astype(int).tolist()} "
                  f"scraper_tilt={math.degrees(math.acos(min(1.0, srot[2, 2]))):4.1f} "
                  f"left TCP-bar={np.round((tl - bar) * 1000).astype(int).tolist()} "
                  f"score={_points(t)}", file=sys.stderr, flush=True)


# -- scoring ----------------------------------------------------------------------------------------------------------

def _in_pot(p) -> bool:
    return math.hypot(p[0] - POT_X, p[1] - POT_Y) < R_IN and TOP + FLOOR_T - 0.002 < p[2] < RIM_Z


def _tally_from(board_pos, board_rot, positions) -> dict:
    in_pot = sum(_in_pot(positions[n]) for n in PIECE_NAMES)
    on_board = sum(_on_board(board_pos, board_rot, positions[n]) for n in PIECE_NAMES)
    return {"in_pot": int(in_pot), "on_board": int(on_board), "spilt": int(N - in_pot - on_board)}


def _tally(objects) -> dict:
    b = objects["board"]
    return _tally_from(np.asarray(b.pos, float), rotation(b.quat_xyzw), {n: objects[n].pos for n in PIECE_NAMES})


def _points(t) -> float:
    if t["in_pot"] >= FULL_IN and t["spilt"] <= FULL_SPILT:
        return 1.0
    if t["in_pot"] >= HALF_IN and t["spilt"] <= HALF_SPILT:
        return 0.5
    return 0.25 if t["in_pot"] >= SOME_IN else 0.0


def score(objects) -> float:
    """1.0 with at least 10 of the 12 pieces in the pot and at most 1 spilt (anywhere but the pot or the board's face);
    0.5 with at least 6 in the pot and at most 2 spilt; 0.25 with at least 3 in the pot; else 0.0."""
    return _points(_tally(objects))


def success(objects) -> bool:
    """Full score with every piece still."""
    return _points(_tally(objects)) >= 1.0 and all(objects[n].resting() for n in PIECE_NAMES)


# -- scripted demo ------------------------------------------------------------------------------------------------------
# The left hand comes straight down on the grip bar, fingers closing across it, lifts the board, carries it right and
# sets it down with its right end 4.5 cm over the pot, resting on the rim, and holds it there. The right hand takes the
# scraper out of its rack by its wooden bar, sets the blade on the board just right of the left hand (the wrists need
# 15 cm between them) and pushes it along the board and off its end, sweeping the pieces into the pot; then once more.

HELD_Y = POT_Y + R_OUT - 0.045 + BOARD_L / 2  # the board's middle once its right end is 4.5 cm past the near rim
BOARD_Z = RIM_Z + BOARD_T  # its top face resting on the rim, level
IN_HAND = 0.003  # each TCP this far above the middle of the bar it holds
PRESS_RIM = 0.002  # the left hand aims the board this far below resting on the rim
PRESS_BLADE = 0.003  # the right hand aims the blade's edge this far into the board
START_Y = -0.014  # the blade set down here along the board (the left TCP is at BAR_Y = +0.1375)
END_Y = -BOARD_L / 2 - 0.012  # and pushed to here, past the board's end
LIFT = 0.025  # the board's underside lifted this far over the rim to carry it across
LEAD_S, LEAD_STEPS = 3.0, 15
ABOVE = 0.05


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


GRIP_L = GRIP_R = grip_rotation(0.0)  # straight down, fingers closing along y


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


def _bar_tcp(board_y: float, board_top: float) -> np.ndarray:
    """The left TCP on the grip bar with the board's middle at (BOARD_X, board_y), its top face at board_top."""
    return np.array([BOARD_X, board_y + BAR_Y, board_top + BAR_H / 2 + IN_HAND])


def _blade_tcp(along: float, edge_z: float) -> np.ndarray:
    """The right TCP with the blade's edge at ``along`` on the held board, at height edge_z."""
    return np.array([BOARD_X, HELD_Y + along, edge_z + BLADE_H + GRIP_S / 2 + IN_HAND])


PICK = [1.0, 0.8, 0.3]
CARRY = [1.5, 1.5, 1.0]  # lift, across, down onto the rim
HELD_AT = LEAD_S + sum(PICK) + sum(CARRY)


def _left() -> list[Waypoint]:
    rest = _bar_tcp(BOARD_Y0, TOP + BOARD_T)
    lifted = _bar_tcp(BOARD_Y0, BOARD_Z + LIFT)
    over = _bar_tcp(HELD_Y, BOARD_Z + LIFT)
    held = _bar_tcp(HELD_Y, BOARD_Z - PRESS_RIM)
    out = _lead_in("left", rest + [0.0, 0.0, ABOVE], GRIP_L, "over the board's grip bar")
    out += [_w(PICK[0], rest, FINGER_OPEN, "down round the bar", GRIP_L),
            _w(PICK[1], rest, FINGER_CLOSED, "grip it", GRIP_L),
            _w(PICK[2], rest, FINGER_CLOSED, "settle", GRIP_L),
            _w(CARRY[0], lifted, FINGER_CLOSED, "lift the board over the rim's height", GRIP_L),
            _w(CARRY[1], over, FINGER_CLOSED, "carry it right, its end over the pot", GRIP_L),
            _w(CARRY[2], held, FINGER_CLOSED, "set its end on the rim", GRIP_L),
            _w(30.0, held, FINGER_CLOSED, "hold the board while the pieces are scraped off", GRIP_L)]
    return out


def _right() -> list[Waypoint]:
    rot = GRIP_R
    pick = np.array([RACK_X, RACK_Y, TOP + RACK_BASE + BLADE_H + GRIP_S / 2 + IN_HAND])
    edge = BOARD_Z - PRESS_BLADE
    start, end = _blade_tcp(START_Y, edge), _blade_tcp(END_Y, edge)
    up = np.array([0.0, 0.0, 0.025])  # higher is out of reach over the start
    out = _lead_in("right", pick + [0.0, 0.0, ABOVE], rot, "over the scraper")
    out += [_w(PICK[0], pick, FINGER_OPEN, "down round its bar", rot),
            _w(PICK[1], pick, FINGER_CLOSED, "grip it", rot),
            _w(PICK[2], pick, FINGER_CLOSED, "settle", rot),
            _w(1.2, pick + [0.0, 0.0, 0.07], FINGER_CLOSED, "lift it out of the rack", rot)]
    t = LEAD_S + sum(PICK) + 1.2
    out.append(_w(max(HELD_AT - t, 0.3) + 0.3, pick + [0.0, 0.0, 0.07], FINGER_CLOSED, "wait for the board", rot))
    for k in range(2):
        out += [_w(2.0 if k == 0 else 1.5, start + up, FINGER_CLOSED, "over the board, left of the pieces", rot),
                _w(0.8, start, FINGER_CLOSED, "blade down on the board", rot),
                _w(3.0, end, FINGER_CLOSED, "push the pieces along the board and off its end into the pot", rot),
                _w(0.8, end + up, FINGER_CLOSED, "lift the blade", rot)]
    out.append(_w(30.0, end + up, FINGER_CLOSED, "hold", rot))
    return out


def demo():
    return {"left": _left(), "right": _right()}
