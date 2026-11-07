"""On the station table lies a sheet of paper with a thick ruler across it, left to right. A felt-tip marker stands in
a holder beside the paper.

Task: draw a straight line along the ruler's near edge. One hand presses the ruler down on the paper; the other takes
the marker and draws with its tip riding against the ruler's edge, at least 18 cm, then lifts it off the paper and puts
it down beside the paper. Then both hands slide the ruler away from the line, showing it, and go back to where they
started. The ruler is light: pushed against by the pen and not held, it slides away and the line goes astray.

Run it:  inspect-robots run --embodiment yam_arena -E task=draw_a_line_along_a_held_ruler.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import (Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, on_reset, on_step, plate,  # noqa: E402
                     rotation)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the paper (fixed) and the ruler (loose), along y ------------------------------------------------------------------
PAPER = (0.21, 0.42, 0.0004)  # x, y, thick (an A3 sheet)
PAPER_AT = (0.30, -0.03)
PAPER_TOP = TOP + PAPER[2]
RULER = (0.03, 0.40, 0.006)  # wide (x), long (y), thick
RULER_AT = (0.30, -0.03)
RULER_MASS = 0.025
EDGE_X = RULER_AT[0] - RULER[0] / 2  # the ruler's near edge (toward the arms)

# -- the marker: a felt tip under a thicker barrel; it stands in a holder ----------------------------------------------
TIP_R, TIP_L = 0.0025, 0.012
BARREL_R, BARREL_L = 0.007, 0.078
PEN_MASS = 0.012
HOLDER_AT = (0.20, -0.12)
HOLDER_R, HOLDER_H = 0.009, 0.02

# -- the ink: dots laid down where the tip touches the paper ----------------------------------------------------------
N_DOTS = 64
DOT_STEP = 0.0035  # a new dot each time the tip has moved this far on the paper
DOT_R = 0.0022  # overlapping: a continuous line
TOUCH = 0.0015  # the tip within this of the paper: it draws
HIDDEN = (PAPER_AT[0] + 0.09, PAPER_AT[1] + 0.13)  # where the unused dots wait, under the paper

# -- scoring ----------------------------------------------------------------------------------------------------------
LINE_X = EDGE_X - TIP_R  # where the tip's middle runs, riding the ruler's edge where the ruler lies
ALONG = 0.005  # every dot within this of that line
RIDES = 0.0007  # the tip touching the ruler's edge: within this of it
RIDE_SHARE = 0.8  # for at least this share of the dots
LONG = 0.18
CLEAR = 0.03  # the ruler slid away: its near edge at least this far beyond the line
HOME_WITHIN = 0.05  # each TCP this close to where it started: the arms are back home
LIFTED = 0.005
WHITE, WOOD, INK, DARK, BLUE = (0.95, 0.95, 0.93), (0.85, 0.72, 0.45), (0.05, 0.10, 0.45), (0.20, 0.21, 0.23), \
    (0.12, 0.25, 0.65)
SUCCESS_HOLD_S = 1.0


def _paper() -> Body:
    return Body.build("paper", pos=(PAPER_AT[0], PAPER_AT[1], TOP), mass=0.01, kinematic=True, color=WHITE,
                      shapes=[BoxShape((0.0, 0.0, PAPER[2] / 2), PAPER, WHITE)], friction=(0.4, 0.35))


def _ruler() -> Body:
    shapes = [BoxShape((0.0, 0.0, RULER[2] / 2), RULER, WOOD)]
    for k in range(41):  # the centimetre marks (show only)
        length = 0.008 if k % 5 == 0 else 0.004
        shapes.append(BoxShape((-RULER[0] / 2 + length / 2, -RULER[1] / 2 + k * 0.01, RULER[2] + 0.0001),
                               (length, 0.0006, 0.0002), DARK, collide=False))
    return Body.build("ruler", pos=(RULER_AT[0], RULER_AT[1], PAPER_TOP + 0.0002), mass=RULER_MASS, shapes=shapes,
                      friction=(0.12, 0.1), color=WOOD)  # smooth: it slides on the paper unless pressed down


def _holder() -> Body:
    shapes = [CylShape((0.0, 0.0, 0.002), HOLDER_R + 0.004, 0.004, DARK)]
    n = 10
    for k in range(n):
        a = 2 * math.pi * k / n
        u = (math.cos(a), math.sin(a), 0.0)
        r = HOLDER_R + 0.001
        shapes.append(plate((r * u[0], r * u[1], 0.004), (r * u[0], r * u[1], HOLDER_H), 2 * r * math.tan(math.pi / n) + 0.001,
                            0.002, u, DARK))
    return Body.build("holder", pos=(HOLDER_AT[0], HOLDER_AT[1], TOP), mass=0.1, shapes=shapes, kinematic=True,
                      friction=(0.3, 0.25), color=DARK)


def _pen() -> Body:
    """Origin at the tip's point; the pen stands upright."""
    shapes = [CylShape((0.0, 0.0, TIP_L / 2), TIP_R, TIP_L, INK),
              CylShape((0.0, 0.0, TIP_L + BARREL_L / 2), BARREL_R, BARREL_L, BLUE)]
    return Body.build("pen", pos=(HOLDER_AT[0], HOLDER_AT[1], TOP + 0.0045), mass=PEN_MASS, shapes=shapes,
                      friction=(1.0, 0.9), color=BLUE, com=(0.0, 0.0, TIP_L + BARREL_L / 2),
                      damping=(0.5, 3.0))  # laid down, it does not roll away


def _dots() -> list[Body]:
    return [Body.build(f"ink_{k:02d}", pos=(HIDDEN[0], HIDDEN[1], TOP + 0.0001), mass=0.001, kinematic=True, color=INK,
                       shapes=[CylShape((0.0, 0.0, 0.0), DOT_R, 0.0002, INK, collide=False)]) for k in range(N_DOTS)]


PAPER_B, RULER_B, HOLDER, PEN = _paper(), _ruler(), _holder(), _pen()
DOTS = _dots()
DOT_NAMES = [d.name for d in DOTS]
OBJECTS = [PAPER_B, RULER_B, HOLDER, PEN, *DOTS]


# -- followed after every control step: the ink -------------------------------------------------------------------------

def _fresh() -> dict:
    return {"dots": [], "rides": [], "tip": None}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("PEN_DEBUG")), "steps": 0}


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0


def _place(asset, origin, x: float, y: float) -> None:
    import torch

    pose = torch.tensor([[x + origin[0], y + origin[1], PAPER_TOP + 0.0002 + origin[2], 0.0, 0.0, 0.0, 1.0]],
                        dtype=torch.float32, device=asset.device)
    asset.write_root_pose_to_sim(pose)


def _tcps(base, origin) -> list[np.ndarray]:
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        out.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
    return out


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    p = as_numpy(base.scene["pen"].data.root_link_pose_w)[0]
    r = as_numpy(base.scene["ruler"].data.root_link_pose_w)[0]
    tip, r_pos, r_rot = p[:3] - origin, r[:3] - origin, rotation(r[3:7])
    _EP["tip"] = tip
    _EP["tcps"] = _tcps(base, origin)
    on_paper = (tip[2] - PAPER_TOP < TOUCH and abs(tip[0] - PAPER_AT[0]) < PAPER[0] / 2
                and abs(tip[1] - PAPER_AT[1]) < PAPER[1] / 2)
    _EP["ruler_edge"] = float(r_pos[0] - RULER[0] / 2)
    if on_paper and len(_EP["dots"]) < N_DOTS:
        last = _EP["dots"][-1] if _EP["dots"] else None
        if last is None or math.hypot(tip[0] - last[0], tip[1] - last[1]) >= DOT_STEP:
            _place(base.scene[DOT_NAMES[len(_EP["dots"])]], origin, float(tip[0]), float(tip[1]))
            v = r_rot.T @ (tip - r_pos)  # the tip against the ruler's near edge (the ruler as it lies now)
            _EP["rides"].append(abs(-RULER[0] / 2 - v[0]) - TIP_R <= RIDES)
            _EP["dots"].append((float(tip[0]), float(tip[1])))
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            st = _status()
            print(f"[pen] t={_DEBUG['steps'] / 20:5.1f}s tip={np.round(tip * 1000).astype(int).tolist()} "
                  f"ruler={np.round(r_pos * 1000).astype(int).tolist()} dots={len(_EP['dots'])} length={st['length'] * 1000:.0f}mm "
                  f"worst_off={st['worst'] * 1000:.1f}mm rides={st['ride_share']:.2f} lifted={st['lifted']} "
                  f"pen_mid={np.round((tip + rotation(p[3:7]) @ np.array([0.0, 0.0, TIP_L + BARREL_L / 2])) * 1000).astype(int).tolist()} "
                  f"shown={st['shown']} score={_points(st)}",
                  file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def _status() -> dict:
    d = _EP["dots"]
    ys = [y for _, y in d]
    length = (max(ys) - min(ys)) if d else 0.0
    worst = max((abs(x - LINE_X) for x, _ in d), default=0.0)
    share = (sum(_EP["rides"]) / len(_EP["rides"])) if _EP["rides"] else 0.0
    tip = _EP["tip"]
    lifted = tip is not None and (tip[2] - PAPER_TOP >= LIFTED or abs(tip[0] - PAPER_AT[0]) > PAPER[0] / 2
                                  or abs(tip[1] - PAPER_AT[1]) > PAPER[1] / 2)  # off the paper, or off its edge
    shown = _EP.get("ruler_edge", EDGE_X) >= LINE_X + TIP_R + CLEAR
    return {"length": length, "worst": worst, "ride_share": share, "lifted": lifted, "shown": shown}


def _points(st) -> float:
    straight = st["worst"] <= ALONG and st["ride_share"] >= RIDE_SHARE
    if st["length"] >= LONG and straight and st["lifted"] and st["shown"]:
        return 1.0
    if st["length"] >= LONG or (straight and st["length"] >= LONG / 2):
        return 0.5
    return 0.25 if st["length"] >= 0.03 else 0.0


def score(objects) -> float:
    """1.0 for a line at least 18 cm long along the ruler's near edge where the ruler lay (every bit within 5 mm of
    it), drawn with the tip riding against the edge (for at least 80% of it), the pen lifted off the paper, and the
    ruler slid away from the line (its near edge at least 3 cm beyond it); 0.5 for a line at least 18 cm long, or one
    half as long that is along the edge; 0.25 for a line at least 3 cm long; else 0.0. Full score also needs the
    marker laid down beside the paper (resting on the table, its middle at most 3 cm in from the paper's edge, no
    gripper within 3 cm of it) and both arms back where they started; without them it is 0.5."""
    pts = _points(_status())
    if pts >= 1.0 and not (_laid_down(objects) and _arms_home()):
        return 0.5  # drawn and shown, but the marker not laid down or the arms not back
    return pts


def _laid_down(objects) -> bool:
    """The marker laid down beside the paper: resting on the table (its lower end within 1 cm of it, lying or standing),
    its middle (seen from above) off the paper or at most 3 cm in from its edge, and no TCP within 3 cm of it."""
    pen = objects["pen"]
    tip, rot = np.asarray(pen.pos, float), rotation(pen.quat_xyzw)
    mid = tip + rot @ np.array([0.0, 0.0, TIP_L + BARREL_L / 2])
    end = tip + rot @ np.array([0.0, 0.0, TIP_L + BARREL_L])
    down = min(float(tip[2]), float(end[2])) < TOP + 0.01
    inside = min(PAPER[0] / 2 - abs(mid[0] - PAPER_AT[0]), PAPER[1] / 2 - abs(mid[1] - PAPER_AT[1]))
    off = inside <= 0.03  # beside the paper: its middle off it, or at most 3 cm in from its edge
    free = _EP["tcps"] is not None and all(float(np.linalg.norm(t - mid)) >= 0.03 for t in _EP["tcps"])
    return bool(down and off and free)


_HOME: dict = {}


def _arms_home() -> bool:
    """Both arms back where they started (each TCP within HOME_WITHIN of its start)."""
    from yam_arena import ik, station
    from yam_arena.constants import READY

    tcps = _EP.get("tcps")
    if tcps is None:
        return False
    if "tcps" not in _HOME:
        _HOME["tcps"] = [np.asarray(ik.tcp_pose(np.array(READY, float), station.BASES[a])[0], float)
                         for a in ("left", "right")]
    return all(float(np.linalg.norm(t - h)) < HOME_WITHIN for t, h in zip(tcps, _HOME["tcps"]))


def success(objects) -> bool:
    """Full score with the pen and the ruler still: the trial ends there."""
    return score(objects) >= 1.0 and objects["pen"].resting() and objects["ruler"].resting()


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand comes straight down on the ruler's left end, fingers closed, and presses it onto the paper. The right
# hand comes straight down on the marker, fingers closing across its barrel, lifts it out of its holder and carries it
# to the right end of the ruler; it sets the tip on the paper against the ruler's near edge and draws along the edge
# to the left, 20 cm, pressing the tip lightly against the edge; then it lifts the marker off the paper and lays it
# down on the table beside the paper. Then both hands, fingers closed, press on the ruler and slide it 6 cm away from
# the line, showing it, and both arms go back to where they started.

GRIP_L = grip_rotation(0.0)  # straight down, fingers closed: it presses the ruler
GRIP_R = grip_rotation(0.0)  # straight down, fingers closing along y, across the barrel
PRESS = 0.012  # the left aims its fingertips this far into the ruler (the arm gives: it presses it down)
PRESS_Y = RULER_AT[1] + RULER[1] / 2 - 0.02  # where it presses, near the ruler's left end
PEN_IN_HAND = 0.02  # the right TCP this far below the barrel's top
TIP_DOWN = 0.002  # aim the tip this far into the paper (the arm gives: it presses lightly)
TIP_IN = 0.001  # ... and this far into the ruler's edge, so it rides against it
HOVER = 0.0045  # the tip this far above the paper (below the ruler's top) as it is brought against the ruler's side
STROKE = (RULER_AT[1] - RULER[1] / 2 + 0.01, RULER_AT[1] - RULER[1] / 2 + 0.21)  # from y, to y: 20 cm
PUT_DOWN = (0.165, -0.20)  # where the right lays the marker down, off the paper
PUSH_Y = -0.12  # where the right presses the ruler to slide it away
PUSH = 0.065  # both hands slide the ruler this far away from the line
LEAD_S, LEAD_STEPS = 3.0, 15


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


PEN_TCP = TIP_L + BARREL_L - PEN_IN_HAND  # the right TCP above the tip's point


def _go_home(arm: str, pos, rot, finger) -> list[Waypoint]:
    """From a pose back to the arm's start, straight through joint space (the lead-in, reversed)."""
    from yam_arena import ik, station
    from yam_arena.constants import READY

    q0, q1 = _good_ik(arm, pos, rot), np.array(READY, float)
    out = []
    for k in range(1, LEAD_STEPS + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(LEAD_S / LEAD_STEPS, tuple(float(v) for v in p), finger, "back to where it started", r))
    return out


PUSH_S = 2.5


def _on_ruler(y: float, press: float) -> np.ndarray:
    return np.array([RULER_AT[0], y, PAPER_TOP + RULER[2] + 0.013 - press])


def _left(push_at: float, total: float) -> tuple[list[Waypoint], float]:
    press = _on_ruler(PRESS_Y, PRESS)
    out = _lead_in("left", press + [0.0, 0.0, 0.04 + PRESS], GRIP_L, "over the ruler's left end", FINGER_CLOSED)
    out += [_w(1.0, press + [0.0, 0.0, PRESS], FINGER_CLOSED, "down onto it", GRIP_L),
            _w(0.8, press, FINGER_CLOSED, "press it onto the paper", GRIP_L)]
    held_at = sum(w.duration for w in out)
    pushed = press + [PUSH, 0.0, 0.0]
    out += [_w(max(push_at - held_at, 0.3), press, FINGER_CLOSED, "hold it while the line is drawn", GRIP_L),
            _w(PUSH_S, pushed, FINGER_CLOSED, "slide the ruler away from the line", GRIP_L),
            _w(1.0, pushed + [0.0, 0.0, 0.05], FINGER_CLOSED, "rise", GRIP_L)]
    out += _go_home("left", pushed + [0.0, 0.0, 0.05], GRIP_L, FINGER_CLOSED)
    t = sum(w.duration for w in out)
    out.append(_w(max(total - t, 0.5), out[-1].pos, FINGER_CLOSED,
                  "hold still: the trial ends once the full score has held 1 s with both arms home", out[-1].yaw))
    return out, held_at


def _right(start: float) -> tuple[list[Waypoint], float]:
    pick = np.array([HOLDER_AT[0], HOLDER_AT[1], TOP + 0.0045 + PEN_TCP])
    x = LINE_X + TIP_IN
    a = np.array([x, STROKE[0], PAPER_TOP + PEN_TCP - TIP_DOWN])
    b = np.array([x, STROKE[1], PAPER_TOP + PEN_TCP - TIP_DOWN])
    out = _lead_in("right", pick + [0.0, 0.0, 0.04], GRIP_R, "over the marker", FINGER_OPEN)
    out += [_w(1.0, pick, FINGER_OPEN, "down round its barrel", GRIP_R),
            _w(0.8, pick, FINGER_CLOSED, "take it", GRIP_R),
            _w(1.0, pick + [0.0, 0.0, 0.03], FINGER_CLOSED, "lift it out of its holder", GRIP_R)]
    t = sum(w.duration for w in out)
    near = a + [-0.004, 0.0, HOVER]  # the tip held just above the paper short of the edge, then moved against the
    # ruler's side and only then set down (the marker leans in the fingers by a few mm, differently each time)
    down = np.array([*PUT_DOWN, TOP + 0.02 + PEN_TCP])
    out += [_w(max(start - t, 0.3) + 1.5, near + [0.0, 0.0, 0.03], FINGER_CLOSED, "over the ruler's right end", GRIP_R),
            _w(1.0, near, FINGER_CLOSED, "the tip just above the paper, short of the ruler's near edge", GRIP_R),
            _w(0.8, a + [0.0, 0.0, HOVER], FINGER_CLOSED, "against the ruler's side", GRIP_R),
            _w(0.3, a + [0.0, 0.0, HOVER], FINGER_CLOSED, "settle there", GRIP_R),
            _w(0.5, a, FINGER_CLOSED, "set the tip on the paper against the edge", GRIP_R),
            _w(6.0, b, FINGER_CLOSED, "draw along the edge, 20 cm", GRIP_R),
            _w(1.0, b + [-0.01, 0.0, 0.03], FINGER_CLOSED, "lift it off the paper", GRIP_R),
            _w(1.5, down + [0.0, 0.0, 0.02], FINGER_CLOSED, "over the table beside the paper", GRIP_R),
            _w(0.6, down, FINGER_CLOSED, "lower it", GRIP_R),
            _w(0.5, down, FINGER_OPEN, "let go: it is down on the table", GRIP_R),
            _w(0.8, down + [0.0, 0.0, 0.04], FINGER_OPEN, "rise", GRIP_R)]
    push = _on_ruler(PUSH_Y, PRESS)
    out += [_w(1.5, push + [0.0, 0.0, 0.04 + PRESS], FINGER_CLOSED, "over the ruler's right part", GRIP_R),
            _w(0.8, push + [0.0, 0.0, PRESS], FINGER_CLOSED, "down onto it", GRIP_R),
            _w(0.6, push, FINGER_CLOSED, "press on it", GRIP_R)]
    push_at = sum(w.duration for w in out)
    pushed = push + [PUSH, 0.0, 0.0]
    out += [_w(PUSH_S, pushed, FINGER_CLOSED, "slide the ruler away from the line", GRIP_R),
            _w(1.0, pushed + [0.0, 0.0, 0.05], FINGER_CLOSED, "rise", GRIP_R)]
    out += _go_home("right", pushed + [0.0, 0.0, 0.05], GRIP_R, FINGER_CLOSED)
    return out, push_at


def demo():
    _, held_at = _left(0.0, 0.0)
    right, push_at = _right(held_at)
    total = sum(w.duration for w in right) + 3.0
    right.append(_w(3.0, right[-1].pos, FINGER_CLOSED, "wait at the start", right[-1].yaw))
    left, _ = _left(push_at, total)
    return {"left": left, "right": right}
