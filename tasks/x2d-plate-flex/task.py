"""A 3D printer's flexible build plate lies on a plate stand on the bench, just taken off the printer, with a freshly
printed robot part (a long arm link) stuck across its middle. A parts bin stands beside it.

Task: get the part off the plate and into the bin, and leave the plate flat on its stand. A print sticks to a textured
PEI plate until the plate is flexed: holding the plate by its two side edges, bend both edges down, so the plate curves
away under the part's ends and the part pops free; then take the part off and put it in the bin. Bending the other way
(edges up) squeezes the part and frees nothing; bending too sharply kinks the spring-steel plate for good.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --instruction "..." --scorer task_score ...

The plate is five rigid strips joined along x by four sprung hinges, so it curves in a gentle arc; the part is held to
the middle strip by a fixed joint (a Glue from bodykit.py) that this file switches off once the plate under both of the
part's ends has turned far enough the convex way, the modelling used by the accepted ice-cube tray task (TK-YTH6M4),
whose bodykit.py this folder reuses unchanged.
"""

from __future__ import annotations

import itertools
import math
import os
import pathlib
import sys
from collections import deque

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import bodykit  # noqa: E402
from bodykit import (Body, BoxShape, allow_matrix_grips, as_numpy, glue, hinge, on_reset, on_step, rotation,  # noqa: E402
                     set_glue_on)
from yam_arena.constants import FINGER_CLOSED, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the plate stand: an aluminium block the plate lies on, its edges overhanging all round ---------------------------
PX = 0.330  # the plate's (and the stand's) centre, x; y = 0
STAND = (0.200, 0.170, 0.180)  # narrower than the plate across y, so the lower fingers fit under its side edges
ST = TOP + STAND[2]  # the stand's top

# -- the build plate: 257 mm square spring steel with a textured PEI face, in five strips along x ---------------------
HALF = 0.1285  # half the plate's side
T = 0.002  # thickness in the sim (0.5 mm real), so the fingers can hold its edge
PZ = ST + T / 2  # its middle plane, lying on the stand
N_STRIPS = 5
PITCH = 2 * HALF / N_STRIPS  # each strip's share of the plate's width (y), 51.4 mm
SEAM = 0.0002  # the gap between strips
STRIP_W = PITCH - SEAM
MID = N_STRIPS // 2  # the middle strip's index; strip k is at y = (k - MID) * PITCH
PLATE_MASS = 0.23
BEND_K = (0.04, 0.08)  # N m per degree at the inner and outer hinge each side: in series they make the plate's stiffness
#                (the inner one softer, so a bend held at the edges spreads in from them; together about a
#                quarter of real 0.5 mm spring steel, so the fingers' grip holds the bend)
LIMIT_DEG = 20.0
KINK_DEG = 20.0  # any one hinge bent past this (a bend radius under about 15 cm): the plate kinks for good (score 0)

# -- the print: a printed robot arm link, PLA, lying across the middle of the plate -----------------------------------
PRINT = (0.024, 0.170, 0.018)  # x, y (its ends past both bend lines each side, at |y| 85 mm), height
PRINT_MASS = 0.040
PRINT_Z = ST + T + PRINT[2] / 2
PART_X = PX - 0.100  # printed near the plate's front edge (the arms reach down onto it only close in)
END_K = 2  # the strip under each of the part's ends: the edge strips MID +- 2
FREE_DEG = 6.0  # the strips under both of the part's ends turned this far from the middle strip, edges down: it frees

# -- the parts bin: polypropylene, open top ---------------------------------------------------------------------------
BIN = np.array([0.270, -0.245])
BIN_IN = (0.090, 0.190, 0.080)  # inside: x, y, depth
BIN_W = 0.003

STEEL, PEI, MARK, ORANGE, ALU, BIN_C = ((0.55, 0.56, 0.58), (0.62, 0.55, 0.42), (0.30, 0.27, 0.22), (0.95, 0.45, 0.10),
                                        (0.70, 0.72, 0.74), (0.20, 0.35, 0.60))

# -- scoring ----------------------------------------------------------------------------------------------------------
BACK_XY, BACK_Z, FLAT_DEG = 0.010, 0.003, 3.0  # each strip back on the stand, flat
CLEAR = 0.08  # every fingertip this far from the plate and the print
DROP_Z = 0.020  # a strip's centre this far below the stand's top: the plate fell
STILL_MM, STILL_STEPS = 1.0, 20  # the last second (20 steps at 20 Hz): everything moved under 1 mm


def _stand() -> Body:
    return Body.build("plate_stand", pos=(PX, 0.0, TOP), mass=1.0, kinematic=True, color=ALU, friction=(0.5, 0.4),
                      shapes=[BoxShape((0.0, 0.0, STAND[2] / 2), STAND, ALU)])


def _bin() -> Body:
    lx, ly, h = BIN_IN
    w = BIN_W
    shapes = [BoxShape((0.0, 0.0, w / 2), (lx + 2 * w, ly + 2 * w, w), BIN_C)]
    for s in (-1.0, 1.0):
        shapes.append(BoxShape((s * (lx / 2 + w / 2), 0.0, w + h / 2), (w, ly + 2 * w, h), BIN_C))
        shapes.append(BoxShape((0.0, s * (ly / 2 + w / 2), w + h / 2), (lx, w, h), BIN_C))
    return Body.build("parts_bin", pos=(float(BIN[0]), float(BIN[1]), TOP), mass=0.3, kinematic=True, color=BIN_C,
                      friction=(0.5, 0.4), shapes=shapes)


def _strip(k: int) -> Body:
    """Strip k of the plate, origin at its centre (mid-plane). The PEI face runs to the strip's long edges, so the
    seams barely show; it stops 3 mm short of the plate's own edges, like the real plate's steel border."""
    y = (k - MID) * PITCH
    lo, hi = y - STRIP_W / 2, y + STRIP_W / 2
    plo, phi = max(lo, -HALF + 0.003), min(hi, HALF - 0.003)
    shapes = [BoxShape((0.0, 0.0, 0.0), (2 * HALF, STRIP_W, T), STEEL),
              BoxShape((0.0, (plo + phi) / 2 - y, T / 2 + 0.0002), (2 * HALF - 0.006, phi - plo, 0.0004), PEI,
                       collide=False)]
    if k == N_STRIPS - 1:  # the plate's printed corner mark, at the corner nearest the left arm's base
        shapes.append(BoxShape((-HALF + 0.012, HALF - 0.012 - y, T / 2 + 0.0005), (0.010, 0.010, 0.0004), MARK,
                               collide=False))
    return Body.build(STRIPS[k], pos=(PX, y, PZ), mass=PLATE_MASS / N_STRIPS, shapes=shapes, color=STEEL,
                      friction=(1.0, 0.9), com=(0.0, 0.0, 0.0))


def _print() -> Body:
    shapes = [BoxShape((0.0, 0.0, 0.0), PRINT, ORANGE)]
    for yy in (-0.065, 0.065):  # the link's two bearing bosses, printed in
        shapes.append(BoxShape((0.0, yy, PRINT[2] / 2 + 0.0003), (0.014, 0.014, 0.0006), (0.6, 0.25, 0.05),
                               collide=False))
    return Body.build("print", pos=(PART_X, 0.0, PRINT_Z), mass=PRINT_MASS, shapes=shapes, color=ORANGE,
                      friction=(0.6, 0.5), com=(0.0, 0.0, 0.0))


STRIPS = tuple(f"plate_{k}" for k in range(N_STRIPS))  # plate_0 at -y (right arm's side) .. plate_8 at +y
_ORDER = [MID] + [MID + s * d for d in range(1, MID + 1) for s in (1, -1)]  # middle first: every hinge's inner
OBJECTS = [_stand(), _bin(), *(_strip(k) for k in _ORDER), _print()]  # strip comes before its outer one
HINGES = []  # (name, inner strip, outer strip, side)
for _d in range(1, MID + 1):
    for _s in (1, -1):
        _inner, _outer = MID + _s * (_d - 1), MID + _s * _d
        _n = f"bend_{'l' if _s > 0 else 'r'}{_d}"
        hinge(_n, STRIPS[_inner], STRIPS[_outer], anchor=(PX, _s * (_d - 0.5) * PITCH, PZ), axis="X",
              limits_deg=(-LIMIT_DEG, LIMIT_DEG), spring=BEND_K[_d - 1], damping=0.01, target_deg=0.0)
        HINGES.append((_n, _inner, _outer, _s))
glue("bond", STRIPS[MID], "print", anchor=(PART_X, 0.0, ST + T))


def _author_glues_colliding(stage, prim_path: str, spec, _author=bodykit._author_glues) -> None:
    """bodykit's glue, with the two bodies still colliding: a USD joint turns contact between its bodies off by default,
    so once the bond let go the print fell through the plate. The print touches the plate, it doesn't overlap it."""
    from pxr import UsdPhysics

    _author(stage, prim_path, spec)
    for name, path in bodykit._GLUE_PATHS.items():
        if path.startswith(prim_path + "/"):
            UsdPhysics.Joint(stage.GetPrimAtPath(path)).CreateCollisionEnabledAttr(True)


bodykit._author_glues = _author_glues_colliding
START = {o.name: np.array(o.pos, float) for o in OBJECTS}


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"tcps": None, "s": {}, "free": False, "kinked": False, "fell": False, "dropped": False, "max_bend": 0.0,
            "hist": deque(maxlen=STILL_STEPS + 1)}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("X2D_DEBUG")), "steps": 0}


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0


def _tcps(base, origin) -> list[np.ndarray]:
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        out.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
    return out


def _bend(r_mid, r_out) -> float:
    """An outer strip's turn about x against the middle strip (deg)."""
    rel = r_mid.T @ r_out
    return math.degrees(math.atan2(float(rel[2, 1]), float(rel[1, 1])))


def _tilt(r) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, float(r[2, 2])))))


def _in_bin(p) -> bool:
    return (abs(float(p[0]) - BIN[0]) <= BIN_IN[0] / 2 and abs(float(p[1]) - BIN[1]) <= BIN_IN[1] / 2
            and float(p[2]) < TOP + BIN_W + BIN_IN[2])


def _box_dist(p, c, r, half) -> float:
    """Distance from point p to a box (centre c, rotation r, half sizes)."""
    q = r.T @ (np.asarray(p, float) - c)
    return float(np.linalg.norm(np.maximum(np.abs(q) - half, 0.0)))


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    st = {}
    for n in (*STRIPS, "print"):
        a = as_numpy(base.scene[n].data.root_link_pose_w)[0]
        st[n] = (a[:3] - origin, rotation(a[3:7]))
    tcps = _tcps(base, origin)
    _EP["tcps"] = tcps
    bends = [_bend(st[STRIPS[i]][1], st[STRIPS[o]][1]) for _, i, o, _ in HINGES]
    worst = max(abs(b) for b in bends)
    _EP["max_bend"] = max(_EP["max_bend"], worst)
    if worst > KINK_DEG:
        _EP["kinked"] = True
    r_mid = st[STRIPS[MID]][1]
    # how far the plate under each of the part's ends has turned away from it (edges down, +), from the middle strip
    ends = [-s * _bend(r_mid, st[STRIPS[MID + s * END_K]][1]) for s in (1, -1)]
    if not _EP["free"] and min(ends) >= FREE_DEG:  # curved away under both ends at once: the part pops free
        _EP["free"] = True
        set_glue_on("bond", False)
    if any(float(st[n][0][2]) < ST - DROP_Z for n in STRIPS):
        _EP["fell"] = True
    pp = st["print"][0]
    if float(pp[2]) < TOP + 0.030 and not _in_bin(pp):  # down on the bench, outside the bin (falling into it is fine)
        _EP["dropped"] = True
    _EP["hist"].append(np.concatenate([st[n][0] for n in (*STRIPS, "print")]))
    hist = np.array(_EP["hist"])
    still = len(hist) > STILL_STEPS and float(np.abs(hist - hist[-1]).reshape(len(hist), -1, 3).max(axis=0)
                                              .max(axis=0).max()) * 1000 < STILL_MM
    back = all(float(np.linalg.norm(st[n][0][:2] - START[n][:2])) <= BACK_XY
               and abs(float(st[n][0][2]) - START[n][2]) <= BACK_Z for n in STRIPS)
    flat = worst <= FLAT_DEG and _tilt(r_mid) <= FLAT_DEG
    shapes = [(st[n][0], st[n][1], np.array([HALF, STRIP_W / 2, T / 2])) for n in STRIPS]
    shapes.append((pp, st["print"][1], np.array(PRINT) / 2))
    clear = all(_box_dist(t, c, r, h) >= CLEAR for t in tcps for c, r, h in shapes)
    _EP["s"] = {"in_bin": _in_bin(pp), "back": back and flat, "clear": clear, "still": still}
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            print(f"[x2d] t={_DEBUG['steps'] / 20:5.1f}s ends L/R={ends[0]:5.1f}/{ends[1]:5.1f} "
                  f"hinges={[round(b, 1) for b in bends]} max={_EP['max_bend']:4.1f} "
                  f"free={_EP['free']} kinked={_EP['kinked']} fell={_EP['fell']} dropped={_EP['dropped']} "
                  f"print={np.round((pp - [PART_X, 0, ST]) * 1000).astype(int).tolist()} {_EP['s']} "
                  f"L={np.round(tcps[0] * 1000).astype(int).tolist()} R={np.round(tcps[1] * 1000).astype(int).tolist()} "
                  f"score={_points()}", file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def _points() -> float:
    e, s = _EP, _EP["s"]
    if e["kinked"] or e["fell"] or e["dropped"] or not s.get("still"):
        return 0.0
    if not e["free"]:
        return 0.0
    if not s["in_bin"]:
        return 0.25
    if not s["back"]:
        return 0.5
    return 1.0 if s["clear"] else 0.75


def score(objects) -> float:
    """0 if, at any time, the plate bent more than 20 degrees across any one of its 4 bend lines (kinked), any part of
    the plate fell 2 cm below the stand's top, or the part came down on the bench outside the bin; 0 unless the plate
    and the part moved under 1 mm in the last second. Otherwise 0.25 once the part has been freed (the plate under both
    of its ends turned 6 degrees or more from the plate's middle at once, edges down); 0.5 with the part in the bin;
    0.75 with the plate also back flat on its stand (every strip within 10 mm of where it started and 3 mm of its
    height, every bend line and the middle flat within 3 degrees); 1.0 with every fingertip also 8 cm or more from the
    plate and the part."""
    return _points()


def success(objects) -> bool:
    return score(objects) >= 1.0


# -- scripted demo ----------------------------------------------------------------------------------------------------
# Both hands come in level from the sides, pointing at the plate's middle, fingers closing across its edge (one above,
# one below). They lift it 4 cm off the stand, bend both sides down 15 degrees about the hinge lines so the plate bows,
# hold, flatten it, set it back on the stand, let go and back off. The right hand then takes the print from above and
# drops it in the bin; both go back to where they started. Open loop: it reads nothing from the simulation.


def _rx(deg: float) -> np.ndarray:
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _ry(deg: float) -> np.ndarray:
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _rz(deg: float) -> np.ndarray:
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


# Each hand points in across its side edge, turned 30 degrees forward (a square side grip is out of reach), fingers
# closing up and down on it: the pads' length, mostly along y, holds the bend, which turns the edge about x. (A grip
# pointing along the edge let the plate roll in the fingers.)
YAW = 45.0
GRIPS = {"left": _rz(YAW) @ _rx(-90.0) @ grip_rotation(0.0), "right": _rz(-YAW) @ _rx(90.0) @ grip_rotation(0.0)}
SIDE = {"left": 1.0, "right": -1.0}
OUTWARD = {a: np.array([-math.sin(math.radians(YAW)), s * math.cos(math.radians(YAW)), 0.0]) for a, s in SIDE.items()}
G_IN = 0.015  # each TCP this far in from the plate's side edge
G_DX = -0.040  # and this far along x from the plate's centre
OPEN_EDGE = -0.010  # round the 2 mm edge
OPEN_PRINT = -0.022
PICK_Y = 0.055  # the right hand takes the part this far to its own side of the part's middle
BOW = -15.0  # each edge strip turned this far, edges down: the plate curves away under the part's ends
LIFT = 0.060  # high enough for the wrists to turn the edges down
BACK = 0.060  # backed off the side edge along the fingers: fingertips clear of it
LEAD_S, LEAD_STEPS = 3.0, 15


def _good_ik(arm: str, pos, rot) -> np.ndarray:
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


def _joint_path(arm: str, q0, q1, label: str, finger) -> list[Waypoint]:
    from yam_arena import ik, station

    out = []
    for k in range(1, LEAD_STEPS + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(LEAD_S / LEAD_STEPS, tuple(float(v) for v in p), finger, label, r))
    return out


def _w(t, pos, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in pos), finger, label, rot)


def _bent(arm: str, grip, deg: float):
    """The grip point and rotation with this hand's side of the plate curved so its edge strip has turned ``deg``
    (+: edge up), each of the side's bend lines taking the share its spring allows under the same moment."""
    s, p = SIDE[arm], np.array(grip, float)
    soft = [1.0 / k for k in BEND_K]
    for d in range(MID, 0, -1):  # the outermost bend line first: each turn carries the lines outside it
        h = np.array([PX, s * (d - 0.5) * PITCH, grip[2]])
        p = h + _rx(s * deg * soft[d - 1] / sum(soft)) @ (p - h)
    return p, _rx(s * deg) @ GRIPS[arm]


def _plate_moves(arm: str) -> list[Waypoint]:
    from yam_arena.constants import READY

    s, g = SIDE[arm], GRIPS[arm]
    grip = np.array([PX + G_DX, s * (HALF - G_IN), PZ])
    out_pt = grip + BACK * OUTWARD[arm]
    pre = out_pt + [0.0, 0.0, 0.070]  # above the approach point: the turn from the start pose happens clear of the plate
    out = _joint_path(arm, np.array(READY, float), _good_ik(arm, pre, g), "turned level, wide of the plate",
                      OPEN_EDGE)
    up = grip + [0.0, 0.0, LIFT]
    out += [_w(1.2, out_pt, OPEN_EDGE, "down beside the plate's edge", g),
            _w(1.5, grip, OPEN_EDGE, "in round the plate's edge", g),
            _w(0.6, grip, FINGER_CLOSED, "take hold of the edge", g),
            _w(1.0, up, FINGER_CLOSED, "lift the plate off its stand", g)]
    for k in range(1, 6):
        p, r = _bent(arm, up, BOW * k / 5)
        out.append(_w(0.3, p, FINGER_CLOSED, "bend the edge down: the plate curves away under the part", r))
    p, r = _bent(arm, up, BOW)
    out.append(_w(1.0, p, FINGER_CLOSED, "hold the bow", r))
    for k in range(1, 6):
        p, r = _bent(arm, up, BOW * (1 - k / 5))
        out.append(_w(0.3, p, FINGER_CLOSED, "flatten it", r))
    out += [_w(1.2, grip + [0.0, 0.0, 0.001], FINGER_CLOSED, "set it back on the stand", g),
            _w(0.6, grip + [0.0, 0.0, 0.001], OPEN_EDGE, "let go", g),
            _w(1.2, out_pt, OPEN_EDGE, "back off the edge", g),
            _w(0.8, out_pt + [0.0, 0.0, 0.08], OPEN_EDGE, "up", g)]
    return out


def demo():
    from yam_arena.constants import READY

    left = _plate_moves("left")
    right = _plate_moves("right")
    gl, gr = GRIPS["left"], GRIPS["right"]
    left += _joint_path("left", _good_ik("left", np.array(left[-1].pos), gl), np.array(READY, float),
                        "back to where it started", OPEN_EDGE)
    down = grip_rotation(math.pi / 2)  # straight down, fingers closing across x (the part's 24 mm)
    at = np.array([PART_X, -PICK_Y, ST + T + PRINT[2] * 0.55])  # on the right arm's side of the part's middle
    lift = at + [0.0, 0.0, 0.015]  # straight-down grips this high reach only close in
    above_bin = np.array([BIN[0], BIN[1] - PICK_Y, lift[2]])  # the part's middle over the bin's
    # The turn to the straight-down grip happens high above everything (a joint-space move bulges).
    high = np.array(right[-1].pos) + [0.0, 0.0, 0.08]
    beside = np.array([PART_X, -0.200, lift[2]])  # out beside the plate, over the bin side: the turn happens here
    over = np.array([PART_X, -PICK_Y, lift[2]])
    right += [_w(0.8, high, OPEN_PRINT, "up, clear of everything", gr)]
    right += _joint_path("right", _good_ik("right", high, gr), _good_ik("right", beside, down),
                         "turned to point down, out beside the plate", OPEN_PRINT)
    right += [_w(1.5, over, OPEN_PRINT, "in over the part's end, fingers either side of it", down)]
    right += [_w(1.5, at, OPEN_PRINT, "down round the part", down),
              _w(0.6, at, FINGER_CLOSED, "take hold of it", down),
              _w(1.2, lift, FINGER_CLOSED, "lift it off the plate", down),
              _w(2.0, above_bin, FINGER_CLOSED, "over the bin", down),
              _w(0.6, above_bin, OPEN_PRINT, "let it drop into the bin", down),
              _w(1.0, above_bin + [-0.06, 0.0, 0.0], OPEN_PRINT, "back, clear of the bin", down)]
    right += _joint_path("right", _good_ik("right", above_bin + [-0.06, 0.0, 0.0], down), np.array(READY, float),
                         "back to where it started", OPEN_PRINT)
    total = max(sum(w.duration for w in left), sum(w.duration for w in right)) + 3.0
    for side in (left, right):
        t = sum(w.duration for w in side)
        side.append(Waypoint(max(total - t, 0.5), side[-1].pos, side[-1].finger,
                             "hold still: the trial ends once the full score has held 1 s", side[-1].yaw))
    return {"left": left, "right": right}
