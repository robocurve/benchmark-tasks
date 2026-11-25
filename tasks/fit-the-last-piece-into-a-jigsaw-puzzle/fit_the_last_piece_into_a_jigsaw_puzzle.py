"""Fit the last piece into a jigsaw puzzle.

A wooden jigsaw puzzle (laser-cut from 1/4 in plywood, 30 x 30 cm) lies finished on the table except for one gap, the
second piece in from its right-hand edge. The last piece lies face up beside it, turned a quarter turn from the way it
fits. The piece is 6.35 mm thick, 3 cm square with three knobs and one hole, so it fits its gap only one way round: its
knobs go into the sockets of the pieces round the gap, and the knob of the piece in front of it (nearer the robot) goes
into its hole.

The robot has to pinch the flat piece off the table by its edges, turn it to match the gap, bring it down over the gap
squarely enough that every knob drops into its socket (0.5 mm of play round the edges), and press it flat.

Run it:  inspect-robots run --embodiment yam_arena -E task=fit_the_last_piece_into_a_jigsaw_puzzle.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import bodykit  # noqa: E402,F401
from bodykit import Body, BoxShape, allow_matrix_grips, as_numpy, on_reset, on_step, rotation  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402,F401
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the piece's shape (its frame: origin at the middle of its underside; +y "north", +x "east") -------------------------
A = 0.015  # half the body's side (a 3 cm body)
T = 0.00635  # thickness: 1/4 in plywood
NECK_W, NECK_L = 0.003, 0.003  # a knob's neck: half width, length out from the edge
HEAD_C, HEAD_R = 0.0068, 0.0042  # the knob's round head: its centre's distance out from the edge, radius
KNOBS = ("N", "E", "S")  # knobs left, away and right (+y, +x, -y); the hole faces the robot (-x)
CL = float(os.environ.get("JIG_CL", "0.0005"))  # the play between the piece and its gap, all round
ROW = 0.0005  # the shapes are built as strips this wide
PIECE_MASS = 0.0040  # 1/4 in plywood, about 600 kg/m3
# -- where things are --------------------------------------------------------------------------------------------------
GAP = np.array([0.340, 0.000, TOP])  # the gap's middle, on the table
PIECE0 = np.array([0.335, -0.095, TOP])  # the loose piece's middle at the start
PIECE0_YAW = float(os.environ.get("JIG_YAW0", "-90"))  # degrees: turned this much from the way it fits
SLAB = 0.075  # the modelled part of the finished puzzle reaches this far from the gap (the rest is drawn)
SLAB_R = 0.045  # ... except to the right, where the puzzle's edge is 1.5 pieces from the gap's middle
# -- the rules ---------------------------------------------------------------------------------------------------------
SEAT_XY, SEAT_YAW, SEAT_Z, SEAT_TILT = 0.0015, 4.0, 0.0008, 3.0
HANDS_CLEAR = 0.03
SUCCESS_HOLD_S = 1.0

PIC, PIC2, EDGE = (0.93, 0.62, 0.25), (0.30, 0.47, 0.70), (0.80, 0.78, 0.72)  # the piece's picture, the puzzle's


# -- the shapes, as unions of intervals row by row ----------------------------------------------------------------------

def _rot(side: str, x: float, y: float) -> tuple[float, float]:
    """A point given for the north side, turned to ``side``."""
    return {"N": (x, y), "E": (y, -x), "S": (-x, -y), "W": (-y, x)}[side]


def _knob_rows(side: str, y: float, grow: float) -> list:
    """x intervals at row ``y`` of a knob on ``side``, grown by ``grow`` (negative shrinks)."""
    out = []
    if side in ("N", "S"):
        sgn = 1.0 if side == "N" else -1.0
        yy = sgn * y  # in the north frame
        if A - 1e-9 <= yy <= A + HEAD_C:
            out.append((-NECK_W - grow, NECK_W + grow))
        dy = yy - (A + HEAD_C)
        r = HEAD_R + grow
        if abs(dy) < r:
            h = math.sqrt(r * r - dy * dy)
            out.append((-h, h))
        return out
    # east or west: the knob runs along x at this row
    sgn = 1.0 if side == "E" else -1.0
    if abs(y) <= NECK_W + grow:
        out.append(tuple(sorted((sgn * A, sgn * (A + HEAD_C)))))
    if abs(y) < HEAD_R + grow:
        h = math.sqrt((HEAD_R + grow) ** 2 - y * y)
        c = sgn * (A + HEAD_C)
        out.append((c - h, c + h))
    return out


def _hole_rows(side: str, y: float, grow: float, ext: float = 0.0) -> list:
    """x intervals at row ``y`` of a hole cut into the body on ``side`` (the hole a knob of size ``grow`` fits); its
    neck reaches ``ext`` out past the edge (to join the neighbouring piece across the play)."""
    # a hole is the shape of a knob, mirrored into the body: its neck from the edge in to its head's centre
    out = []
    sgn = 1.0 if side == "E" else -1.0  # only east and west holes are used here
    if abs(y) <= NECK_W + grow:
        out.append(tuple(sorted((sgn * (A + ext), sgn * (A - HEAD_C)))))
    if abs(y) < HEAD_R + grow:
        h = math.sqrt((HEAD_R + grow) ** 2 - y * y)
        c = sgn * (A - HEAD_C)
        out.append((c - h, c + h))
    return out


def _union(iv: list) -> list:
    iv = sorted((a, b) for a, b in iv if b > a)
    out = []
    for a, b in iv:
        if out and a <= out[-1][1] + 1e-9:
            out[-1] = (out[-1][0], max(out[-1][1], b))
        else:
            out.append((a, b))
    return out


def _minus(iv: list, cut: list) -> list:
    out = []
    for a, b in _union(iv):
        segs = [(a, b)]
        for c, d in _union(cut):
            nxt = []
            for s, e in segs:
                if d <= s or c >= e:
                    nxt.append((s, e))
                else:
                    if c > s:
                        nxt.append((s, c))
                    if d < e:
                        nxt.append((d, e))
            segs = nxt
        out += segs
    return [(s, e) for s, e in out if e - s > 1e-4]


def piece_rows(y: float, grow: float = 0.0) -> list:
    """x intervals of the piece (grown by ``grow`` all round) at row ``y``."""
    iv = [(-A - grow, A + grow)] if abs(y) <= A + grow else []
    for side in KNOBS:
        iv += _knob_rows(side, y, grow)
    hole = _hole_rows("W", y, -grow, ext=(grow + 0.001) if grow > 0 else 0.0) if abs(y) <= A + grow else []
    return _minus(_union(iv), hole)


def _strips(rows_fn, lo: float, hi: float, z0: float, z1: float, color, frame=lambda x, y: (x, y)) -> list:
    out = []
    n = int(round((hi - lo) / ROW))
    for i in range(n):
        y = lo + (i + 0.5) * ROW
        for a, b in rows_fn(y):
            cx, cy = frame((a + b) / 2, y)
            out.append(BoxShape((cx, cy, (z0 + z1) / 2), (b - a, ROW + 0.00005, z1 - z0), color))
    return out


def _piece() -> Body:
    sh = _strips(lambda y: piece_rows(y), -A - NECK_L - HEAD_C - HEAD_R, A + NECK_L + HEAD_C + HEAD_R, 0.0, T, PIC)
    return Body.build("piece", pos=tuple(PIECE0.tolist()), mass=PIECE_MASS, shapes=sh, color=PIC, yaw=math.radians(PIECE0_YAW),
                      friction=(0.6, 0.5), solver_iterations=32, contact_offset=0.0006)


def _slab() -> Body:
    """The finished puzzle round the gap (kinematic): everything within SLAB of the gap but the gap itself."""
    reach = A + HEAD_C + HEAD_R + 0.004

    def slab_rows(y):
        return _minus([(-reach, reach)], piece_rows(y, CL))

    sh = _strips(slab_rows, -reach, reach, 0.0, T, PIC2)
    for lo, hi in (((-SLAB, -SLAB_R), (SLAB, -reach)), ((-SLAB, reach), (SLAB, SLAB)), ((-SLAB, -reach), (-reach, reach)),
                   ((reach, -reach), (SLAB, reach))):
        sh.append(BoxShape(((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, T / 2), (hi[0] - lo[0], hi[1] - lo[1], T), PIC2))
    # the rest of the finished puzzle (30 x 30 cm in all), drawn only, round the modelled part
    x0, x1, y1 = -0.135, 0.165, 0.255  # 10 x 10 pieces; the gap is 2nd from the right edge, 5th row from the robot
    for lo, hi in (((x0, -SLAB_R), (-SLAB, y1)), ((SLAB, -SLAB_R), (x1, y1)), ((-SLAB, SLAB), (SLAB, y1))):
        sh.append(BoxShape(((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, T / 2 - 0.0002), (hi[0] - lo[0], hi[1] - lo[1], T - 0.0004),
                           PIC2, collide=False))
    return Body.build("puzzle", pos=tuple(GAP.tolist()), mass=0.5, kinematic=True, shapes=sh, color=PIC2,
                      friction=(0.6, 0.5), contact_offset=0.0006)


OBJECTS = [_slab(), _piece()]


# -- followed after every control step ---------------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"s": {}, "steps": 0, "last": None}


_EP = _fresh()


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())


def _tcps(base, origin):
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        out.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
    return out


def measure(pp, rp, tcps) -> dict:
    off = float(np.hypot(*(pp - GAP)[:2]))
    yaw = math.degrees(math.atan2(float(rp[1, 0]), float(rp[0, 0])))
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(rp[2, 2])))))
    dz = float(pp[2] - GAP[2])
    seated = off <= SEAT_XY and abs(yaw) <= SEAT_YAW and abs(dz) <= SEAT_Z and tilt <= SEAT_TILT
    in_gap = off <= 0.004 and abs(yaw) <= 10.0 and dz <= 0.003
    over = off <= 0.006 and abs(yaw) <= 10.0 and dz <= 0.006
    clear = all(float(np.linalg.norm(t - pp)) >= HANDS_CLEAR for t in tcps)
    return {"off": off, "yaw": yaw, "tilt": tilt, "dz": dz, "seated": bool(seated), "in_gap": bool(in_gap),
            "over": bool(over), "hands_clear": bool(clear)}


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    p = as_numpy(base.scene["piece"].data.root_link_pose_w)[0]
    pp, rp = p[:3] - origin, rotation(p[3:7])
    tcps = _tcps(base, origin)
    s = measure(pp, rp, tcps)
    last = _EP.get("last")
    s["still"] = last is not None and float(np.linalg.norm(pp - last)) < 0.0002
    _EP["last"] = pp
    _EP["s"] = s
    _EP["steps"] += 1
    if os.environ.get("JIG_DEBUG") and _EP["steps"] % int(os.environ.get("JIG_DEBUG_N", "20")) == 0:
        print(f"[jig] step {_EP['steps']} piece {np.round((pp - GAP) * 1000, 2)} yaw {s['yaw']:.1f} tilt {s['tilt']:.1f} "
              f"seated {s['seated']} in_gap {s['in_gap']} over {s['over']} clear {s['hands_clear']} "
              f"R {np.round((tcps[1] - GAP) * 1000, 1)}", flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------------------

def score(objects) -> float:
    """1.0: the piece seated in its gap (centred within 1.5 mm, turned within 4 degrees, flat within 3 degrees and down
    within 0.8 mm of the table, so level with the pieces round it) and the hands 3 cm or more from it; 0.75: seated, a
    hand still on it; 0.5: in the gap the right way round (within 4 mm and 10 degrees) but not down flat (a knob riding
    on a neighbour); 0.25: over the gap the right way round but resting on the puzzle; 0 otherwise."""
    s = _EP["s"]
    if not s:
        return 0.0
    if s["seated"]:
        return 1.0 if s["hands_clear"] else 0.75
    if s["in_gap"]:
        return 0.5
    return 0.25 if s["over"] else 0.0


def success(objects) -> bool:
    return score(objects) >= 1.0 and _EP["s"].get("still", False)


# -- scripted demo ------------------------------------------------------------------------------------------------------------
# The right hand, pointing down, pinches the piece by two straight stretches of its edges, just off the table, lifts it,
# turns it a quarter turn, carries it over the gap, lowers it to just above the puzzle, sets it down into the gap with a
# small circling wiggle so the knobs find their sockets, lets go and presses it flat with the closed fingertips.

LEAD_S, LEAD_STEPS = 3.0, 15


def _frame(approach, close) -> np.ndarray:
    z = np.asarray(approach, float) / np.linalg.norm(approach)
    x = np.asarray(close, float) - np.dot(close, z) * z
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def _rz(deg: float) -> np.ndarray:
    a = math.radians(deg)
    return np.array([[math.cos(a), -math.sin(a), 0.0], [math.sin(a), math.cos(a), 0.0], [0.0, 0.0, 1.0]])


def _good_ik(arm: str, pos, rot, near=None) -> np.ndarray:
    import itertools

    from yam_arena import ik, station

    sols = []
    for seed in itertools.product((-0.5, 0.0, 0.5), (1.3, 1.9), (1.0, 1.7), (-1.0, 0.0), (0.0,), (-1.0, 0.0, 1.0)):
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.array(seed),
                             base_pos=station.BASES[arm], iters=150)
        if pe < 0.001 and re < math.radians(0.5):
            sols.append((q, float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())))
    if not sols:
        raise ValueError(f"no IK solution for the {arm} hand at {np.round(pos, 4)}")
    if near is not None:
        roomy = [s for s in sols if s[1] > 0.15] or sols
        return min(roomy, key=lambda s: float(np.linalg.norm(s[0] - near)))[0]
    return max(sols, key=lambda s: s[1])[0]


def _joint_path(arm: str, q0, q1, label: str, finger, secs=LEAD_S, steps=LEAD_STEPS) -> list:
    from yam_arena import ik, station

    out = []
    for k in range(1, steps + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / steps, station.BASES[arm])
        out.append(Waypoint(secs / steps, tuple(float(v) for v in p), finger, label, r))
    return out


def demo():
    from yam_arena import ik, station
    from yam_arena.constants import READY

    plan = {"left": [], "right": []}
    cur = {}
    for arm in ("left", "right"):
        p, r = ik.tcp_pose(np.array(READY, float), station.BASES[arm])
        cur[arm] = (np.asarray(p, float), FINGER_OPEN, r)

    def go(arm, t, pos, finger, label, rot=None, n=1):
        p0, _, r0 = cur[arm]
        rot = r0 if rot is None else rot
        p1 = np.asarray(pos, float)
        for j in range(1, n + 1):
            plan[arm].append(Waypoint(t / n, tuple(float(v) for v in p0 + (p1 - p0) * j / n), finger, label, rot))
        cur[arm] = (p1, finger, rot)

    def wait(arm, t, label="wait"):
        p, f, r = cur[arm]
        if t > 0:
            plan[arm].append(Waypoint(t, tuple(float(v) for v in p), f, label, r))

    def dur(arm):
        return sum(w.duration for w in plan[arm])

    def lead(arm, pos, rot, finger, label, secs=LEAD_S):
        p0, _, r0 = cur[arm]
        q0 = np.array(READY, float) if not plan[arm] else _good_ik(arm, p0, r0, near=np.array(READY, float))
        plan[arm] += _joint_path(arm, q0, _good_ik(arm, pos, rot, near=q0), label, finger, secs)
        cur[arm] = (np.asarray(pos, float), finger, rot)

    def home(arm, label="back to where it started"):
        p, f, r = cur[arm]
        plan[arm] += _joint_path(arm, _good_ik(arm, p, r, near=np.array(READY, float)), np.array(READY, float), label, f)
        cur[arm] = (np.asarray(plan[arm][-1].pos, float), f, plan[arm][-1].yaw)

    up = np.array([0.0, 0.0, 1.0])
    env = os.environ.get
    # the fingers close along the piece's x, across its east and west edges, on the straight stretch by its south
    # corners (clear of the east knob, whose head the wide fingertips would otherwise land on)
    pinch_y = float(env("JIG_PINCH_Y", "-0.0185"))
    tcp_z = TOP + float(env("JIG_TCP_Z", "0.0110"))  # the fingertips on the table (they stop the hand here)
    open_ = float(env("JIG_OPEN", "-0.032"))
    drop_z = TOP + float(env("JIG_DROP_Z", "0.0200"))  # the piece's underside just above the puzzle's top
    fix = np.array([float(env("JIG_DX", "0")), float(env("JIG_DY", "0")), 0.0])  # aim correction at the gap
    r_piece = _frame((0.0, 0.0, -1.0), (1.0, 0.0, 0.0))

    def hand(centre, yaw_deg, z):  # the TCP and rotation for the piece's middle at ``centre``, turned ``yaw_deg``
        rz = _rz(yaw_deg)
        p = np.asarray(centre, float) + rz @ np.array([0.0, pinch_y, 0.0])
        return np.array([p[0], p[1], z]), rz @ r_piece

    a = "right"
    p, r = hand(PIECE0, PIECE0_YAW, tcp_z)
    lead(a, p + up * 0.05, r, open_, "above the piece, fingers either side of its edges")
    go(a, 1.2, p, open_, "fingertips down to the table beside its edges", n=2)
    go(a, 0.6, p, FINGER_CLOSED, "pinch the piece by its edges")
    go(a, 0.8, p + up * 0.020, FINGER_CLOSED, "lift it")
    for k in range(1, 5):  # turn it the quarter turn it is off, on the way over
        yaw = PIECE0_YAW * (1 - k / 4)
        c = PIECE0 + (GAP - PIECE0) * (k / 4) + fix * (k / 4)
        p, r = hand(c, yaw, tcp_z + 0.020)
        go(a, 0.7, p, FINGER_CLOSED, "turn it to match the gap and carry it over", rot=r)
    # where the piece ends up in the hand: its middle relative to the commanded TCP, in the hand's turned frame (mm), and
    # its turn relative to the hand (degrees), as measured in the simulator (the pinch shifts it a little)
    ho = np.array([float(env("JIG_HOX", "-1.40")), float(env("JIG_HOY", "25.1")), 0.0]) * 0.001
    ho_yaw = float(env("JIG_HOYAW", "3.4"))

    def place(centre, z):  # the TCP and rotation that put the piece's middle at ``centre``, turned to fit
        rz = _rz(-ho_yaw)
        p = np.asarray(centre, float) - rz @ ho
        return np.array([p[0], p[1], z]), rz @ r_piece

    p, r = place(GAP + fix, drop_z + 0.004)
    go(a, 1.2, p, FINGER_CLOSED, "lower it to just above the puzzle", rot=r, n=2)
    p, r = place(GAP + fix, drop_z)
    go(a, 1.0, p, FINGER_CLOSED, "hold it squarely over the gap, its underside level with the puzzle's top", rot=r, n=2)
    wait(a, 0.5, "settle")
    go(a, 0.5, p, open_, "let go: it drops into the gap")
    go(a, 0.6, p + up * 0.012, open_, "fingers up")
    go(a, 0.6, p + up * 0.012, FINGER_CLOSED, "close the fingers")
    press = np.array([GAP[0], GAP[1], TOP + float(env("JIG_PRESS_Z", "0.0185"))])
    go(a, 0.8, press + up * 0.004, FINGER_CLOSED, "over the middle of the piece", rot=r)
    w = float(env("JIG_WIGGLE", "0.0006"))
    for k in range(8):  # press it flat, circling a little so a knob riding on a neighbour drops into its socket
        ang = 2 * math.pi * k / 8
        go(a, 0.25, press + np.array([w * math.cos(ang), w * math.sin(ang), 0.0]), FINGER_CLOSED,
           "press it flat with the closed fingertips", rot=r)
    wait(a, 0.3, "hold")
    go(a, 0.8, press + up * 0.06, FINGER_CLOSED, "hand up")
    home(a)
    end = max(dur("left"), dur("right")) + 2.0
    for arm in ("left", "right"):
        wait(arm, end - dur(arm), "hold still")
    return plan
