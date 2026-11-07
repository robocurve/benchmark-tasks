"""On the station table lies a long pine board. A tape measure's hook is caught over a nail at the board's right end, its
case lying on the board against it, the tape wound in. A marker pen stands in a holder beside the board.

Task: mark the board 300 mm from its end. One hand pulls the tape's case out along the board, well past 300 mm, and
holds it there: its spring winds the tape straight back in as soon as it is let go. The other takes the pen and dabs a
mark on the board right beside the tape's 300 mm graduation (a red line across the tape), then lifts the pen.

Run it:  inspect-robots run --embodiment yam_arena -E task=measure_and_mark_300_mm_with_a_spring_return_tape.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, on_reset, on_step, plate, slider  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the board (fixed), along y; the tape's zero at its right end (the hook's outer face) ----------------------------
BOARD_X, BOARD_T = (0.25, 0.33), 0.018
Y0 = -0.16  # the board's right end: the tape's zero
BOARD_Y = (Y0, 0.39)
BT = TOP + BOARD_T  # its top

# -- the tape: the blade lies along the board's near part; the case slides along it on a spring that winds it in --------
TAPE_X, TAPE_W = 0.27, 0.019  # the blade's middle line, and its width
TAPE_FAR = TAPE_X + TAPE_W / 2  # the blade's far edge (away from the arms): the mark goes on the board beyond it
CASE = (0.030, 0.070, 0.060)  # x, y (along the tape), z
RIDGE = (0.016, 0.040, 0.035)  # a grip ridge on its top, toward its back
RIDGE_Y = 0.050  # the ridge's middle, back from the mouth
CASE_MASS = 0.15
CASE_Y0 = Y0 + 0.004  # the case's mouth (its origin, on the board's top) with the tape wound in
OUT_MAX = 0.47
SPRING_K, SPRING_TO = 4.0, -0.5  # N/m toward this far past wound in: about 2 N wound in, 3.9 N pulled out 47 cm
SEG_L, N_SEG = 0.03, 17  # the blade shown in 3 cm pieces, each shown once the case's mouth is past its start

# -- the pen: a felt tip under a square barrel; it stands in a holder -------------------------------------------------
TIP_R, TIP_L = 0.0025, 0.012
BARREL_W, BARREL_L = 0.016, 0.078  # a flat-sided barrel: it sits square in the fingers
PEN_MASS = 0.012
HOLDER_AT = (0.20, -0.04)
HOLDER_R, HOLDER_H = 0.0125, 0.024

# -- the ink: one dot each time the tip comes down on the board ---------------------------------------------------------
N_DOTS = 6
DOT_R = 0.0022
TOUCH = 0.0015  # the tip within this of the board: it marks
LIFT = 0.004  # ... once it has been at least this far up since its last mark
HIDDEN = np.array([0.30, 0.20, TOP - 0.03])  # where unused dots and blade pieces wait, under the table

WOOD, YELLOW, BLACK, RED, CASE_C, BRASS, BLUE, INK, DARK = (0.55, 0.36, 0.20), (0.96, 0.82, 0.15), \
    (0.05, 0.05, 0.05), (0.85, 0.10, 0.08), (0.20, 0.22, 0.25), (0.78, 0.64, 0.30), (0.15, 0.30, 0.75), \
    (0.06, 0.06, 0.10), (0.20, 0.21, 0.23)

# -- scoring ----------------------------------------------------------------------------------------------------------
MARK_AT = 0.300
WITHIN, NEAR = 0.004, 0.010
BESIDE = (TAPE_FAR, TAPE_FAR + 0.025)  # a mark's middle, across: on the board just beyond the tape's far edge
OUT_PAST = 0.010  # a mark is measured if the tape is out at least this far past it when it is made
SUCCESS_HOLD_S = 1.0


def _board() -> Body:
    cx, cy = (BOARD_X[0] + BOARD_X[1]) / 2, (BOARD_Y[0] + BOARD_Y[1]) / 2
    shapes = [BoxShape((cx, cy, BOARD_T / 2), (BOARD_X[1] - BOARD_X[0], BOARD_Y[1] - BOARD_Y[0], BOARD_T), WOOD),
              # the hook, caught over a nail at the end (shown only)
              BoxShape((TAPE_X, Y0 + 0.002, BOARD_T + 0.0008), (TAPE_W + 0.002, 0.004, 0.0016), BRASS, collide=False),
              BoxShape((TAPE_X, Y0 - 0.0008, BOARD_T - 0.004), (TAPE_W + 0.002, 0.0016, 0.010), BRASS, collide=False),
              CylShape((TAPE_X, Y0 - 0.003, BOARD_T - 0.006), 0.002, 0.004, DARK, collide=False, axis="Y")]
    return Body.build("board", pos=(0.0, 0.0, TOP), mass=2.0, shapes=shapes, kinematic=True, friction=(0.6, 0.5),
                      color=WOOD)


def _case() -> Body:
    """The tape's case; origin at its mouth, on the board's top, on the blade's line."""
    shapes = [BoxShape((0.0, CASE[1] / 2, CASE[2] / 2), CASE, CASE_C),
              BoxShape((0.0, RIDGE_Y, CASE[2] + RIDGE[2] / 2), RIDGE, YELLOW),
              BoxShape((0.0, 0.0005, 0.001), (TAPE_W, 0.001, 0.002), BRASS, collide=False)]  # the blade's tip at the mouth
    return Body.build("case", pos=(TAPE_X, CASE_Y0, BT), mass=CASE_MASS, shapes=shapes, friction=(1.2, 1.1),
                      color=CASE_C, com=(0.0, CASE[1] / 2, CASE[2] / 2))


def _segment(k: int) -> Body:
    """A 3 cm piece of the blade, from 3k cm past the zero, with its graduations (every cm; longer every 5 cm; a red
    line across at 300 mm)."""
    shapes = [BoxShape((0.0, 0.0, 0.0), (TAPE_W, SEG_L, 0.0003), YELLOW, collide=False)]
    for i in range(3):
        cm = 3 * k + i
        y = -SEG_L / 2 + 0.01 * i
        if cm == 0:
            continue
        if cm * 10 == round(MARK_AT * 1000):
            shapes.append(BoxShape((0.0, y, 0.0003), (TAPE_W, 0.0015, 0.0003), RED, collide=False))
        else:
            ln = 0.010 if cm % 5 == 0 else 0.005
            shapes.append(BoxShape((-TAPE_W / 2 + ln / 2, y, 0.0003), (ln, 0.0007, 0.0003), BLACK, collide=False))
    return Body.build(f"tape_{k:02d}", pos=tuple(HIDDEN.tolist()), mass=0.001, kinematic=True, color=YELLOW,
                      shapes=shapes)


def _holder() -> Body:
    shapes = [CylShape((0.0, 0.0, 0.002), HOLDER_R + 0.004, 0.004, DARK)]
    n = 10
    for k in range(n):
        a = 2 * math.pi * k / n
        u = (math.cos(a), math.sin(a), 0.0)
        r = HOLDER_R + 0.001
        shapes.append(plate((r * u[0], r * u[1], 0.004), (r * u[0], r * u[1], HOLDER_H),
                            2 * r * math.tan(math.pi / n) + 0.001, 0.002, u, DARK))
    return Body.build("holder", pos=(HOLDER_AT[0], HOLDER_AT[1], TOP), mass=0.1, shapes=shapes, kinematic=True,
                      friction=(0.3, 0.25), color=DARK)


def _pen() -> Body:
    """Origin at the tip's point; the pen stands upright."""
    shapes = [CylShape((0.0, 0.0, TIP_L / 2), TIP_R, TIP_L, INK),
              BoxShape((0.0, 0.0, TIP_L + BARREL_L / 2), (BARREL_W, BARREL_W, BARREL_L), BLUE)]
    return Body.build("pen", pos=(HOLDER_AT[0], HOLDER_AT[1], TOP + 0.0045), mass=PEN_MASS, shapes=shapes,
                      friction=(1.4, 1.3), color=BLUE, com=(0.0, 0.0, TIP_L + BARREL_L / 2))


def _dots() -> list[Body]:
    return [Body.build(f"ink_{k}", pos=tuple(HIDDEN.tolist()), mass=0.001, kinematic=True, color=INK,
                       shapes=[CylShape((0.0, 0.0, 0.0), DOT_R, 0.0002, INK, collide=False)]) for k in range(N_DOTS)]


SEGMENTS = [_segment(k) for k in range(N_SEG)]
DOTS = _dots()
OBJECTS = [_board(), _case(), _holder(), _pen(), *SEGMENTS, *DOTS]
slider("tape", "board", "case", anchor=(TAPE_X, CASE_Y0, BT), axis="Y", limits=(0.0, OUT_MAX), stiffness=SPRING_K,
       damping=3.0, target=SPRING_TO, max_force=20.0)


# -- followed after every control step: the blade shown as far as it is out, the ink -----------------------------------

def _fresh() -> dict:
    return {"out": 0.0, "max_out": 0.0, "shown": 0, "marks": [], "lifted": True, "tip": None}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("TAPE_DEBUG")), "steps": 0}


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0


def _place(asset, origin, at) -> None:
    import torch

    p = np.asarray(at, float) + origin
    asset.write_root_pose_to_sim(torch.tensor([[*p.tolist(), 0.0, 0.0, 0.0, 1.0]], dtype=torch.float32,
                                              device=asset.device))


def _segment_at(k: int) -> np.ndarray:
    return np.array([TAPE_X, Y0 + SEG_L * (k + 0.5), BT + 0.00016])


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    c = as_numpy(base.scene["case"].data.root_link_pose_w)[0]
    out = float(c[1] - origin[1] - Y0)  # the tape out from its zero to the case's mouth
    _EP["out"] = out
    _EP["max_out"] = max(_EP["max_out"], out)
    shown = min(N_SEG, max(0, math.ceil((out - 0.002) / SEG_L)))
    if shown != _EP["shown"]:  # show the pieces now out (the rest of the last one is inside the case)
        for k in range(min(shown, _EP["shown"]), max(shown, _EP["shown"])):
            _place(base.scene[SEGMENTS[k].name], origin, _segment_at(k) if k < shown else HIDDEN)
        _EP["shown"] = shown
    p = as_numpy(base.scene["pen"].data.root_link_pose_w)[0]
    tip = p[:3] - origin
    _EP["tip"] = tip
    on_board = BOARD_X[0] < tip[0] < BOARD_X[1] and BOARD_Y[0] < tip[1] < BOARD_Y[1]
    height = float(tip[2] - BT)
    if height >= LIFT:
        _EP["lifted"] = True
    if on_board and height < TOUCH and _EP["lifted"] and len(_EP["marks"]) < N_DOTS:
        _EP["lifted"] = False
        _place(base.scene[DOTS[len(_EP["marks"])].name], origin, [tip[0], tip[1], BT + 0.0001])
        d = float(tip[1] - Y0)
        _EP["marks"].append({"d": d, "x": float(tip[0]), "measured": out >= d + OUT_PAST})
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            st = _status()
            print(f"[tape] t={_DEBUG['steps'] / 20:5.1f}s out={out * 1000:6.1f}mm max_out={_EP['max_out'] * 1000:6.1f}mm "
                  f"tip={np.round(tip * 1000).astype(int).tolist()} tip_d={(tip[1] - Y0) * 1000:6.1f}mm "
                  f"tip_up={height * 1000:5.1f}mm marks={[(round(m['d'] * 1000, 1), round(m['x'] * 1000, 1), m['measured']) for m in _EP['marks']]} "
                  f"good={st['good']} near={st['near']} strays={st['strays']} score={_points(st)}",
                  file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def _status() -> dict:
    marks = _EP["marks"]

    def beside(m):
        return BESIDE[0] <= m["x"] <= BESIDE[1]

    good = [m for m in marks if m["measured"] and beside(m) and abs(m["d"] - MARK_AT) <= WITHIN]
    near = [m for m in marks if m["measured"] and beside(m) and abs(m["d"] - MARK_AT) <= NEAR]
    strays = [m for m in marks if not (m["measured"] and beside(m) and abs(m["d"] - MARK_AT) <= NEAR)]
    tip = _EP["tip"]
    lifted = tip is None or float(tip[2] - BT) >= LIFT
    return {"good": len(good), "near": len(near), "strays": len(strays), "lifted": lifted,
            "measured": _EP["max_out"] >= MARK_AT + OUT_PAST}


def _points(st) -> float:
    if st["good"] and not st["strays"] and st["lifted"]:
        return 1.0
    if st["near"]:
        return 0.5
    return 0.25 if st["measured"] else 0.0


def score(objects) -> float:
    """1.0 with a mark on the board beside the tape within 4 mm of 300 mm from the board's end, made while the tape was
    out past it, no other marks, and the pen lifted off; 0.5 with such a mark within 1 cm of 300 mm (or a good one with
    stray marks, or the pen still down); 0.25 with the tape pulled out past 300 mm at some point; else 0.0."""
    return _points(_status())


def success(objects) -> bool:
    """Full score with the pen and the tape's case still: the trial ends once it has held 1 s."""
    return score(objects) >= 1.0 and objects["pen"].resting() and objects["case"].resting()


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand comes down on the case's grip ridge (leaning 20 degrees, to reach across), fingers closing across it,
# pulls the case straight along the board to 40 cm out, and holds it there. The right hand comes straight down on the
# pen's barrel, takes it out of its holder, brings it (leaning 20 degrees) over the board just beyond the tape's far edge
# at the 300 mm line, dabs its tip down on the board and lifts it. Then the left lets the case go: the tape winds back in.

def _rx(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


TILT = math.radians(20.0)  # each hand leans its wrist back toward its own side: it reaches further across
GRIP_L = _rx(-TILT) @ grip_rotation(-math.pi / 2)  # down, leaning 20 degrees; fingers closing along x, across the ridge
GRIP_R = grip_rotation(0.0)  # straight down, fingers closing along y, across the barrel (to take the pen)
DAB_R = _rx(TILT) @ GRIP_R  # the same, leaning 20 degrees: the pen leans with it
RIDGE_IN_HAND = 0.006  # the left TCP this far below the ridge's top
PULL_TO = 0.40  # the case's mouth this far out (the left hand 15 cm past the mark)
PULL_EXTRA = 0.02  # aimed this much further (the arm gives against the spring)
PEN_IN_HAND = 0.02  # the right TCP this far below the barrel's top
PEN_TCP = TIP_L + BARREL_L - PEN_IN_HAND  # the right TCP above the tip's point
TIP_DOWN = 0.002  # aim the tip this far into the board (the arm stops short)
MARK_X = TAPE_FAR + 0.010
PEN_LEAN = np.array([0.026, -0.003, 0.0])  # the pen tips over this far in the fingers as it is lifted out (measured)
LEAD_S, LEAD_STEPS = 3.0, 15


def _good_ik(arm: str, pos, rot) -> np.ndarray:
    import itertools

    from yam_arena import ik, station

    best = None
    for seed in itertools.product((-0.5, 0.0, 0.5), (0.8, 1.3), (0.4, 1.0), (-0.8, 0.0, 0.8), (-1.4, 0.0, 1.4), (0.0,)):
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.array(seed),
                             base_pos=station.BASES[arm], iters=200)
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


def _ridge(out: float) -> np.ndarray:
    return np.array([TAPE_X, Y0 + out + RIDGE_Y, BT + CASE[2] + RIDGE[2] - RIDGE_IN_HAND])


def _left(release_at: float, total: float) -> tuple[list[Waypoint], float]:
    start = _ridge(CASE_Y0 - Y0)
    out = _lead_in("left", start + [0.0, 0.0, 0.04], GRIP_L, "over the tape's case", FINGER_OPEN)
    out += [_w(1.0, start, FINGER_OPEN, "down round its grip ridge", GRIP_L),
            _w(0.6, start, FINGER_CLOSED, "grip it", GRIP_L)]
    for k in range(1, 6):
        out.append(_w(0.8, _ridge(CASE_Y0 - Y0 + (PULL_TO + PULL_EXTRA - CASE_Y0 + Y0) * k / 5), FINGER_CLOSED,
                      "pull the tape out along the board", GRIP_L))
    held_at = sum(w.duration for w in out)
    end = _ridge(PULL_TO + PULL_EXTRA)
    out += [_w(max(release_at - held_at, 0.3), end, FINGER_CLOSED, "hold it out while the mark is made", GRIP_L),
            _w(0.5, end, FINGER_OPEN, "let go: the tape winds back in", GRIP_L),
            _w(1.0, end + [0.0, 0.0, 0.05], FINGER_OPEN, "rise", GRIP_L)]
    t = sum(w.duration for w in out)
    out.append(_w(max(total - t, 0.5), end + [0.0, 0.0, 0.05], FINGER_OPEN,
                  "hold still: the trial ends once the full score has held 1 s", GRIP_L))
    return out, held_at


def _right(start: float) -> list[Waypoint]:
    pick = np.array([HOLDER_AT[0], HOLDER_AT[1], TOP + 0.0045 + PEN_TCP])
    tip = np.array([MARK_X, Y0 + MARK_AT, BT - TIP_DOWN])
    dab = tip - PEN_LEAN - _rx(TILT) @ np.array([0.0, 0.0, -PEN_TCP])  # the right TCP up the leaning pen from its tip
    out = _lead_in("right", pick + [0.0, 0.0, 0.04], GRIP_R, "over the pen", FINGER_OPEN)
    out += [_w(1.0, pick, FINGER_OPEN, "down round its barrel", GRIP_R),
            _w(0.8, pick, FINGER_CLOSED, "take it", GRIP_R),
            _w(1.5, pick + [0.0, 0.0, 0.06], FINGER_CLOSED, "lift it out of its holder", GRIP_R)]
    t = sum(w.duration for w in out)
    out += [_w(max(start - t, 0.3) + 1.5, dab + [0.0, 0.0, 0.03], FINGER_CLOSED,
               "over the board beside the tape's 300 mm line", DAB_R),
            _w(1.0, dab, FINGER_CLOSED, "dab the tip down on the board: a mark", DAB_R),
            _w(0.4, dab, FINGER_CLOSED, "dab the tip down on the board: a mark", DAB_R),
            _w(1.0, dab + [0.0, 0.0, 0.04], FINGER_CLOSED, "lift the pen off", DAB_R)]
    return out


def demo():
    _, held_at = _left(0.0, 0.0)
    right = _right(held_at)
    t_r = sum(w.duration for w in right)
    total = t_r + 4.0
    right.append(_w(4.0, right[-1].pos, FINGER_CLOSED, "hold still", DAB_R))
    left, _ = _left(t_r + 0.3, total)
    return {"left": left, "right": right}
