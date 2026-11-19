"""On the station table, between the arms: an end-grain cutting board with twenty diced carrot pieces on it, a quarter
sheet pan butted against the board's right edge, and a bench scraper standing in a utensil holder on the far right.
Nothing else is on the table."""

# Transferring chopped vegetables from the cutting board to a sheet pan with a bench scraper is a core, many-times-a-
# shift prep operation for restaurant cooks (about 1.4 million US workers, BLS OEWS May 2024, 35-2014) and food
# preparation workers (35-2021). By hand it is two-handed: one hand holds the board still, the other drives the
# scraper across it and pushes the pieces off its edge into the pan.
#
# Physical difficulty (bimanual): the board slides on the worktop (wood on steel, friction 0.25) when the scraper is
# dragged across it pressed down, so one arm must press the board down and hold it while the other drags a 15 cm bench
# scraper through twenty scattered pieces and off the board's edge, with the blade's edge kept down on the board so no
# piece slips under it; pieces pushed off before the board's edge, or round the blade's ends, land on the worktop.
# Full-size objects: a 30 x 23 x 3.8 cm end-grain board, a quarter sheet pan and a 15 cm bench scraper.
#
# World frame: x forward (away from the arms), y left, z up, metres. The arm bases stand at x 0, y +-0.305.

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from labkit import part  # noqa: E402
from taskbase import Trial, fake_state  # noqa: E402

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = ("Scrape all the diced carrot off the cutting board into the sheet pan with the bench scraper, holding "
               "the board still, then put the scraper back in its holder and let go of the board.")

# -- the scene -------------------------------------------------------------------------------------------------------

MAPLE = (0.80, 0.64, 0.42)
ALU = (0.78, 0.79, 0.80)
STEEL = (0.70, 0.72, 0.75)
DARK = (0.30, 0.31, 0.33)
CARROT = (0.93, 0.50, 0.12)

BOARD = (0.23, 0.30, 0.038)  # forward x across x thick
BOARD_MASS = 1.6
BOARD_FRICTION = 0.25
BOARD_XY = (0.27, 0.06)  # centre; its right edge at y = -0.09
BOARD_EDGE_Y = BOARD_XY[1] - BOARD[1] / 2

PAN_OUT = (0.33, 0.23, 0.025)  # quarter sheet pan, outside: 33 cm forward x 23 cm, 2.5 cm rim
PAN_WALL = 0.002
PAN_MASS = 0.4
PAN_XY = (0.27, BOARD_EDGE_Y - 0.004 - PAN_OUT[1] / 2)  # butted against the board's right edge

N_PIECES = 20
PIECE = 0.010  # 1 cm dice, 1 g
PIECE_MASS = 0.001

BLADE_L, BLADE_H, BLADE_T = 0.15, 0.075, 0.003  # bench scraper blade
GRIP_W, GRIP_H = 0.022, 0.022  # its handle along the top edge
SCRAPER_MASS = 0.2
HOLDER_XY = (0.22, -0.385)  # utensil holder: a slotted block on the table
HOLDER_H = 0.04


def _piece_xy():
    rng = np.random.default_rng(5)
    pts = []
    while len(pts) < N_PIECES:
        p = (round(float(rng.uniform(0.215, 0.325)), 3), round(float(rng.uniform(-0.075, 0.005)), 3))
        if all(math.hypot(p[0] - q[0], p[1] - q[1]) > 0.017 for q in pts):
            pts.append(p)
    return pts


PIECE_XY = _piece_xy()
PIECE_NAMES = [f"carrot_{i:02d}" for i in range(N_PIECES)]
BOARD_TOP = TOP + BOARD[2]


def _pan_boxes():
    L, W, H = PAN_OUT
    t = PAN_WALL
    return [((0, 0, t / 2), (L, W, t), ALU),
            ((-L / 2 + t / 2, 0, H / 2), (t, W, H), ALU), ((L / 2 - t / 2, 0, H / 2), (t, W, H), ALU),
            ((0, -W / 2 + t / 2, H / 2), (L - 2 * t, t, H), ALU), ((0, W / 2 - t / 2, H / 2), (L - 2 * t, t, H), ALU)]


BOARD_PART = part("cutting_board", pos=(BOARD_XY[0], BOARD_XY[1], TOP), mass=BOARD_MASS, color=MAPLE,
                  friction=BOARD_FRICTION, boxes=[((0, 0, BOARD[2] / 2), BOARD, MAPLE)])
PAN = part("sheet_pan", pos=(PAN_XY[0], PAN_XY[1], TOP), mass=PAN_MASS, color=ALU, friction=0.3, boxes=_pan_boxes())
SCRAPER = part("bench_scraper", pos=(HOLDER_XY[0], HOLDER_XY[1], TOP + 0.004), mass=SCRAPER_MASS, color=STEEL,
               friction=0.5, boxes=[((0, 0, BLADE_H / 2), (BLADE_L, BLADE_T, BLADE_H), STEEL),
                                    ((0, 0, BLADE_H + GRIP_H / 2), (BLADE_L, GRIP_W, GRIP_H), MAPLE)])
SLOT = 0.006  # slot width at the bottom; the top 2 cm flares to FLARE across, guiding the blade in
FLARE = 0.03


def _holder_boxes():
    L = BLADE_L + 0.03
    wall_h = HOLDER_H - 0.02
    out = [((0, 0, 0.002), (L, 0.05, 0.004), DARK),
           ((0, -(SLOT / 2 + 0.0045), wall_h / 2), (L, 0.009, wall_h), DARK),
           ((0, SLOT / 2 + 0.0045, wall_h / 2), (L, 0.009, wall_h), DARK)]
    run, rise = (FLARE - SLOT) / 2, 0.02
    slope = math.atan2(rise, run)
    for s in (-1, 1):  # the two guide plates of the flare, sloping up and out from the slot's lips
        c = (0, s * (SLOT / 2 + run / 2 + 0.002), wall_h + rise / 2)
        out.append((c, (math.hypot(run, rise), L, 0.004), DARK, (math.pi / 2, -s * slope)))
    return out


HOLDER = part("utensil_holder", pos=(HOLDER_XY[0], HOLDER_XY[1], TOP), fixed=True, color=DARK, boxes=_holder_boxes())
PIECES = [part(n, pos=(x, y, BOARD_TOP + PIECE / 2), mass=PIECE_MASS, color=CARROT, friction=0.5,
               boxes=[((0, 0, 0), (PIECE, PIECE, PIECE), CARROT)]) for n, (x, y) in zip(PIECE_NAMES, PIECE_XY)]
OBJECTS = [BOARD_PART, PAN, SCRAPER, HOLDER, *PIECES]

# -- scoring ---------------------------------------------------------------------------------------------------------
#
# A piece is in the pan when its centre, in the pan's own frame, lies inside the pan's walls and no higher than 1 cm
# above the rim. Score at the end of the trial: 0.6 x (pieces in the pan) / 20 (0 to 0.6). 1.0 when at least SUCCESS_N
# pieces are in the pan, the bench scraper is back in its holder (its bottom within 2.5 cm of the slot's middle along
# the slot and 1 cm across it, below the holder's top, standing within 15 degrees of upright), the board, the pan, the scraper and every piece have stopped
# moving (under 5 mm over the last 0.3 s), and no gripper tool point is within 5 cm of the scraper's handle or of the
# board's top surface, held for 1 s. Completed less than 1 s before the run limit: the partial score stands.

SUCCESS_N = 18
ABOVE_RIM = 0.01
HANDS_OFF = 0.05


def _rot(q):
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def in_pan(pan, s) -> bool:
    x, y, z = _rot(pan.quat_xyzw).T @ (np.asarray(s.pos, float) - np.asarray(pan.pos, float))
    L, W, H = PAN_OUT
    return abs(x) <= L / 2 - PAN_WALL and abs(y) <= W / 2 - PAN_WALL and PAN_WALL - 0.002 <= z <= H + ABOVE_RIM


HOME_ALONG, HOME_ACROSS, HOME_TILT = 0.025, 0.010, 15.0


def scraper_home(s) -> bool:
    x, y, z = (float(v) for v in s.pos)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(_rot(s.quat_xyzw)[2, 2])))))
    return (abs(x - HOLDER_XY[0]) <= HOME_ALONG and abs(y - HOLDER_XY[1]) <= HOME_ACROSS and z < TOP + HOLDER_H
            and tilt < HOME_TILT)


def board_points(b):
    """Points over the board's top surface: a 5 x 5 grid, for the hands-off check."""
    R = _rot(b.quat_xyzw)
    return [np.asarray(b.pos, float) + R @ np.array((u * BOARD[0] / 2, v * BOARD[1] / 2, BOARD[2]))
            for u in np.linspace(-1, 1, 5) for v in np.linspace(-1, 1, 5)]


def evaluate(objects, trial):
    pan, board, scraper = objects["sheet_pan"], objects["cutting_board"], objects["bench_scraper"]
    n_in = sum(in_pan(pan, objects[n]) for n in PIECE_NAMES)
    home = scraper_home(scraper)
    trial.info.update(in_pan=n_in, scraper_home=home)
    grip = np.asarray(scraper.pos, float) + _rot(scraper.quat_xyzw) @ np.array((0, 0, BLADE_H + GRIP_H / 2))
    done = (n_in >= SUCCESS_N and home
            and all(trial.still(n) for n in ["cutting_board", "sheet_pan", "bench_scraper", *PIECE_NAMES])
            and trial.clear_of([grip, *board_points(board)], HANDS_OFF))
    return 0.6 * n_in / N_PIECES, done


TRIAL = Trial(evaluate, trace_env="BOARD_TRACE")
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"


def success(objects) -> bool:
    return TRIAL.success(objects)


def score(objects) -> float:
    return TRIAL.score(objects)


def distance(objects) -> float:
    pan = objects["sheet_pan"]
    c = np.asarray(pan.pos, float)[:2]
    return float(sum(0.0 if in_pan(pan, objects[n]) else np.linalg.norm(np.asarray(objects[n].pos)[:2] - c)
                     for n in PIECE_NAMES))


# -- the demonstration -----------------------------------------------------------------------------------------------
#
# Scripted, open loop. The left arm presses its closed fingers down on the board's left half and holds it there
# throughout. The right arm lifts the bench scraper out of its holder by its handle, sets the blade's edge on the board
# left of the pieces and drags it to the right across the board and 2.5 cm past its edge over the pan, twice; then it
# puts the scraper back in its holder and lets go, and the left arm lifts off the board.

YAW_L = -math.pi / 2  # left pads close along x
YAW_R = 0.0  # right pads close along y: across the scraper's handle
PRESS = 0.003
HOLD_XY = (BOARD_XY[0], BOARD_XY[1] + 0.09)  # where the left fingers press: the board's left half
GRIP_Z = BLADE_H + GRIP_H / 2  # tool point above the blade's edge when holding the handle
SCRAPE_Y0 = max(y for _, y in PIECE_XY) + PIECE / 2 + BLADE_T / 2 + 0.008  # blade start: just left of the pieces
SCRAPE_Y1 = BOARD_EDGE_Y - 0.025  # blade end: 2.5 cm past the board's edge, over the pan


def _blade(y, z_edge):
    return (BOARD_XY[0], y, z_edge + GRIP_Z)


def _scrape(label):
    return [Waypoint(1.2, _blade(SCRAPE_Y0, BOARD_TOP + 0.03), FINGER_CLOSED, f"blade over the board ({label})", yaw=YAW_R),
            Waypoint(0.6, _blade(SCRAPE_Y0, BOARD_TOP - PRESS), FINGER_CLOSED, "edge down on the board", yaw=YAW_R),
            Waypoint(2.4, _blade(BOARD_EDGE_Y, BOARD_TOP - PRESS), FINGER_CLOSED, "drag the pieces to the edge", yaw=YAW_R),
            Waypoint(0.6, _blade(SCRAPE_Y1, BOARD_TOP - PRESS), FINGER_CLOSED, "push them off into the pan", yaw=YAW_R),
            Waypoint(0.5, _blade(SCRAPE_Y1, BOARD_TOP + 0.04), FINGER_CLOSED, "lift the scraper", yaw=YAW_R)]


def demo():
    hp = np.array((HOLD_XY[0], HOLD_XY[1], BOARD_TOP))
    left = [Waypoint(1.4, tuple(hp + (0, 0, 0.05)), FINGER_CLOSED, "over the board's left half", yaw=YAW_L),
            Waypoint(0.8, tuple(hp), FINGER_CLOSED, "fingers on the board", yaw=YAW_L),
            Waypoint(0.4, tuple(hp - (0, 0, PRESS)), FINGER_CLOSED, "press the board down", yaw=YAW_L)]
    hz = TOP + 0.004 + GRIP_Z
    hx, hy = HOLDER_XY
    up = BOARD_TOP + 0.04 + GRIP_Z
    right = [Waypoint(2.6, (hx, hy, hz + 0.05), FINGER_OPEN * 0.5, "over the scraper's handle", yaw=YAW_R),
             Waypoint(0.8, (hx, hy, hz), FINGER_OPEN * 0.5, "around the handle", yaw=YAW_R),
             Waypoint(0.6, (hx, hy, hz), FINGER_CLOSED, "grip the scraper", yaw=YAW_R),
             Waypoint(1.0, (hx, hy, up), FINGER_CLOSED, "lift it out of the holder", yaw=YAW_R)]
    right += _scrape("first pass") + _scrape("second pass")
    right += [Waypoint(1.4, (hx, hy, up), FINGER_CLOSED, "carry it back over the holder", yaw=YAW_R),
              Waypoint(1.0, (hx, hy, hz + 0.002), FINGER_CLOSED, "into the holder", yaw=YAW_R),
              Waypoint(0.5, (hx, hy, hz + 0.002), FINGER_OPEN * 0.5, "let go of the scraper", yaw=YAW_R),
              Waypoint(0.7, (hx, hy, hz + 0.06), FINGER_OPEN * 0.5, "lift clear", yaw=YAW_R),
              Waypoint(1.0, (0.12, -0.27, TOP + 0.16), FINGER_OPEN * 0.5, "clear", yaw=YAW_R)]
    t_l, t_r = sum(w.duration for w in left), sum(w.duration for w in right)
    left += [Waypoint(t_r - t_l, tuple(hp - (0, 0, PRESS)), FINGER_CLOSED, "hold the board", yaw=YAW_L),
             Waypoint(0.7, tuple(hp + (0, 0, 0.07)), FINGER_CLOSED, "lift off the board", yaw=YAW_L),
             Waypoint(1.0, (0.12, 0.27, TOP + 0.16), FINGER_CLOSED, "clear", yaw=YAW_L),
             Waypoint(3.0, (0.12, 0.27, TOP + 0.16), FINGER_CLOSED, "wait", yaw=YAW_L)]
    right += [Waypoint(sum(w.duration for w in left) - sum(w.duration for w in right), (0.12, -0.27, TOP + 0.16),
                       FINGER_OPEN * 0.5, "wait", yaw=YAW_R)]
    return {"left": left, "right": right}


# -- offline cases (check_task.py) ----------------------------------------------------------------------------------

def CASES():
    half = (PIECE / 2,) * 3

    def st(n_in, home=True, on_table=0):
        d = {"cutting_board": fake_state((BOARD_XY[0], BOARD_XY[1], TOP), tuple(np.array(BOARD) / 2)),
             "sheet_pan": fake_state((PAN_XY[0], PAN_XY[1], TOP), tuple(np.array(PAN_OUT) / 2))}
        d["bench_scraper"] = fake_state((HOLDER_XY[0], HOLDER_XY[1], TOP + 0.004) if home else (0.27, 0.0, BOARD_TOP),
                                        (BLADE_L / 2, GRIP_W / 2, 0.048))
        for i, n in enumerate(PIECE_NAMES):
            if i < n_in:
                p = (PAN_XY[0] - 0.08 + 0.04 * (i % 5), PAN_XY[1] - 0.06 + 0.03 * (i // 5), TOP + PAN_WALL + PIECE / 2)
            elif i < n_in + on_table:
                p = (0.40, -0.06, TOP + PIECE / 2)
            else:
                p = (PIECE_XY[i][0], PIECE_XY[i][1], BOARD_TOP + PIECE / 2)
            d[n] = fake_state(p, half)
        return d
    far = lambda: [np.array((0.1, 0.3, 1.0)), np.array((0.1, -0.3, 1.0))]
    holding = lambda: [np.array((HOLD_XY[0], HOLD_XY[1], BOARD_TOP)), np.array((0.1, -0.3, 1.0))]
    return [
        ("all pieces on the board", [st(0)] * 30, far, 0.0, False),
        ("10 in the pan", [st(10)] * 30, far, 0.3, False),
        ("all 20 in, scraper home, hands off, held 1 s", [st(20)] * 30, far, 1.0, True),
        ("18 in, 2 on the worktop", [st(18, on_table=2)] * 30, far, 1.0, True),
        ("17 in", [st(17)] * 30, far, 0.51, False),
        ("all in, scraper left on the board", [st(20, home=False)] * 30, far, 0.6, False),
        ("all in, still holding the board", [st(20)] * 30, holding, 0.6, False),
        ("all in, 0.75 s before the run limit", [st(20)] * 15, far, 0.6, True),
    ]
