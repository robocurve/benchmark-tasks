"""On the station table, a square sandwich loaf lies on a fixed cutting board, along the left-right line; a bread knife
stands blade down in a fixed knife rest to the right.

Task: cut one slice, 1 to 2 cm thick, off the loaf's right end with the bread knife, holding the loaf with the other
hand, and keep that hand at least 3 cm from the blade all the while (a claw grip, fingertips well back from the cut).
Sawing drags the loaf along with the blade: an unheld loaf slides back and forth with the knife and hardly gets cut.

Run it:  inspect-robots run --embodiment yam_arena -E task=slice_a_loaf_holding_it_with_a_claw_grip.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import (Body, BoxShape, allow_matrix_grips, as_numpy, glue, on_reset, on_step, rotation,  # noqa: E402
                     set_glue_on)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the cutting board (fixed) ----------------------------------------------------------------------------------------
BOARD_X, BOARD_Y, BOARD_L, BOARD_W, BOARD_T = 0.30, 0.06, 0.36, 0.18, 0.015
BOARD_TOP = TOP + BOARD_T

# -- the loaf: a square sandwich loaf along y. Its right end is six 5 mm slabs glued together and to the rest of the
# loaf; a cut through a glue line (finished by sawing) frees the slice beyond it -------------------------------------
LX = 0.295  # its middle, along x
Y_R = -0.040  # its right end
LOAF_W, LOAF_H = 0.090, 0.080
SLAB, N_SLABS = 0.005, 6
REST_L = 0.170  # the rest of the loaf, one body
LOAF_DENSITY = 250.0  # kg/m3 (bread)
CRUST, CRUMB = (0.72, 0.47, 0.22), (0.96, 0.90, 0.74)
THIN, THICK = (2, 4)  # a slice 1 to 2 cm thick: a cut at glue line 2, 3 or 4 (glue line k is k*5 mm from the end)

# -- the knife: a bread knife. Its frame: origin at the blade's spine where it meets the handle; the blade along +x,
# its edge 3.5 cm below the spine; the handle along -x ---------------------------------------------------------------
BLADE_L, BLADE_H, BLADE_T = 0.200, 0.035, 0.0015
HANDLE_L, HANDLE_W, HANDLE_H = 0.110, 0.022, 0.024
HANDLE_Z = -0.010  # the handle's middle, below the spine
KNIFE_MASS, KNIFE_COM = 0.10, (0.020, 0.0, -0.015)  # a light blade: balanced near the handle
REST_X, REST_Y = 0.22, -0.20  # where the blade meets the handle, in the knife rest
REST_BASE, REST_SLOT, REST_WALL_H = 0.004, 0.006, 0.030
STEEL, HANDLE_C, BOARD_C, REST_C = (0.78, 0.79, 0.81), (0.18, 0.12, 0.08), (0.80, 0.66, 0.46), (0.22, 0.22, 0.24)

# -- cutting ----------------------------------------------------------------------------------------------------------
RATE = 0.20  # m of depth cut per m of blade sawn along the cut (relative to the loaf)
ON_LINE = 0.0025  # the blade's plane within this of a glue line is cutting along it
KERF = 0.006  # once a cut is started, the blade follows it while its plane stays within this of it
DRAG_K, DRAG_MAX = 150.0, 5.0  # the blade drags the loaf along: N per m/s of sawing, at most N (it slides at ~2)
TIP_FORCE, TIP_STEPS = 0.3, 10  # a slice once cut free is nudged at its top, away from the loaf, for 0.5 s: it falls
RES_K, RES_MAX = 1500.0, 25.0  # uncut bread under the edge pushes the knife back up: N/m, at most N (applied at the
# handle, where it is held, so that it does not tip the knife in the hand)

# -- scoring ----------------------------------------------------------------------------------------------------------
SAFE = 0.03  # both hands' TCPs at least this far from the blade (not its handle), always
SUCCESS_HOLD_S = 1.0


def _slab(i: int) -> Body:
    """Slab i (0 at the loaf's right end). Its frame: origin at the middle of its underside."""
    shapes = [BoxShape((0.0, 0.0, LOAF_H / 2), (LOAF_W, SLAB, LOAF_H), CRUST)]
    for s in (-1.0, 1.0):  # crumb on both cut faces, inside a 3 mm crust
        shapes.append(BoxShape((0.0, s * (SLAB / 2 + 0.0001), LOAF_H / 2 - 0.0005), (LOAF_W - 0.006, 0.0002, LOAF_H - 0.007),
                               CRUMB, collide=False))
    y = Y_R + (i + 0.5) * SLAB
    return Body.build(f"slab_{i}", pos=(LX, y, BOARD_TOP + 0.0005), mass=LOAF_W * SLAB * LOAF_H * LOAF_DENSITY,
                      shapes=shapes, friction=(0.7, 0.6), color=CRUST)


def _rest_of_loaf() -> Body:
    """The rest of the loaf. Its frame: origin at the middle of its underside's right edge (glue line N_SLABS)."""
    shapes = [BoxShape((0.0, REST_L / 2, LOAF_H / 2), (LOAF_W, REST_L, LOAF_H), CRUST),
              BoxShape((0.0, -0.0001, LOAF_H / 2 - 0.0005), (LOAF_W - 0.006, 0.0002, LOAF_H - 0.007), CRUMB, collide=False)]
    return Body.build("loaf", pos=(LX, Y_R + N_SLABS * SLAB, BOARD_TOP + 0.0005),
                      mass=LOAF_W * REST_L * LOAF_H * LOAF_DENSITY, shapes=shapes, friction=(0.7, 0.6), color=CRUST)


def _board() -> Body:
    return Body.build("board", pos=(BOARD_X, BOARD_Y, TOP), mass=1.0, kinematic=True, friction=(0.6, 0.5),
                      shapes=[BoxShape((0.0, 0.0, BOARD_T / 2), (BOARD_W, BOARD_L, BOARD_T), BOARD_C)], color=BOARD_C)


def _knife() -> Body:
    shapes = [BoxShape((BLADE_L / 2, 0.0, -BLADE_H / 2), (BLADE_L, BLADE_T, BLADE_H), STEEL),
              BoxShape((-HANDLE_L / 2, 0.0, HANDLE_Z), (HANDLE_L, HANDLE_W, HANDLE_H), HANDLE_C)]
    names = [f"slab_{i}" for i in range(N_SLABS)] + ["loaf"]
    return Body.build("knife", pos=(REST_X, REST_Y, TOP + REST_BASE + BLADE_H + 0.0005), mass=KNIFE_MASS, shapes=shapes,
                      friction=(1.0, 0.9), color=STEEL, com=KNIFE_COM, ignore=tuple(names))


def _rest() -> Body:
    shapes = [BoxShape((0.0, 0.0, REST_BASE / 2), (0.17, 0.05, REST_BASE), REST_C)]
    shapes += [BoxShape((0.0, s * (REST_SLOT / 2 + 0.005), REST_BASE + REST_WALL_H / 2), (0.17, 0.010, REST_WALL_H), REST_C)
               for s in (-1.0, 1.0)]
    return Body.build("knife_rest", pos=(REST_X + 0.09, REST_Y, TOP), mass=1.0, shapes=shapes, kinematic=True,
                      friction=(0.4, 0.3), color=REST_C)


BOARD, REST = _board(), _rest()
LOAF = _rest_of_loaf()
SLABS = [_slab(i) for i in range(N_SLABS)]
KNIFE = _knife()
OBJECTS = [BOARD, REST, LOAF, *SLABS[::-1], KNIFE]  # each slab after the one it is glued to
SLAB_NAMES = [s.name for s in SLABS]
# glue line k (1..6) between slab k-1 and slab k (slab 6 is the rest of the loaf), k*5 mm from the loaf's right end
GLUE_NAMES = []
for k in range(1, N_SLABS + 1):
    left = "loaf" if k == N_SLABS else f"slab_{k}"
    glue(f"crumb_{k}", left, f"slab_{k - 1}", anchor=(LX, Y_R + k * SLAB, BOARD_TOP + 0.0005 + LOAF_H / 2))
    GLUE_NAMES.append(f"crumb_{k}")


# -- followed after every control step: the cut along each glue line, the drag on the loaf, the holding hand ----------

def _fresh() -> dict:
    return {"cut": [0.0] * (N_SLABS + 1), "through": [False] * (N_SLABS + 1), "rel_x": None, "unsafe": False,
            "tcps": None, "line": None, "tip": 0}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("SLICE_DEBUG")), "steps": 0}


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


def _blade_distance(p, k_pos, k_rot) -> float:
    """How far point p is from the blade (a rectangle in the knife's x-z plane)."""
    v = k_rot.T @ (np.asarray(p, float) - np.asarray(k_pos, float))
    dx = max(0.0, -v[0], v[0] - BLADE_L)
    dz = max(0.0, -BLADE_H - v[2], v[2])
    return float(math.sqrt(dx * dx + v[1] * v[1] + dz * dz))


def _set_force(asset, f_world, rot, at=None) -> None:
    """Apply a force (world frame) to a body until the next step, at ``at`` (in its own frame) or its origin."""
    import torch

    f = rot.T @ np.asarray(f_world, float)
    t = torch.tensor(f, dtype=torch.float32, device=asset.device).view(1, 1, 3)
    pos = None if at is None else torch.tensor(np.asarray(at, float), dtype=torch.float32, device=asset.device).view(1, 1, 3)
    asset.permanent_wrench_composer.set_forces_and_torques(forces=t, torques=torch.zeros_like(t), positions=pos,
                                                           is_global=False)


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    lo = as_numpy(base.scene["loaf"].data.root_link_pose_w)[0]
    kn = as_numpy(base.scene["knife"].data.root_link_pose_w)[0]
    l_pos, l_rot = lo[:3] - origin, rotation(lo[3:7])
    k_pos, k_rot = kn[:3] - origin, rotation(kn[3:7])
    _EP["tcps"] = _tcps(base, origin)
    if min(_blade_distance(t, k_pos, k_rot) for t in _EP["tcps"]) < SAFE:  # a hand too near the blade (not the
        _EP["unsafe"] = True  # handle: a hand holding the knife by its handle's middle is 5.5 cm from the blade)
    # the blade in the loaf's frame (origin at glue line 6, the underside's middle)
    sp = l_rot.T @ (k_pos - l_pos)  # the knife's origin (spine, at the handle)
    kx = l_rot.T @ (k_rot @ np.array([1.0, 0.0, 0.0]))
    ky = l_rot.T @ (k_rot @ np.array([0.0, 1.0, 0.0]))
    y_line = sp[1]  # the blade's plane, along the loaf (0 at glue line 6, negative toward the right end)
    k = int(round(N_SLABS + y_line / SLAB))
    kz = l_rot.T @ (k_rot @ np.array([0.0, 0.0, 1.0]))
    s0 = float(np.clip(-sp[0] / kx[0], 0.0, BLADE_L)) if abs(kx[0]) > 1e-6 else 0.0  # along the blade, over the middle
    edge_z = float((sp + s0 * kx - BLADE_H * kz)[2])  # the edge's height there
    x0, x1 = sorted((sp[0], sp[0] + BLADE_L * kx[0]))
    in_bread = (abs(ky[1]) > math.cos(math.radians(15)) and x1 > -LOAF_W / 2 and x0 < LOAF_W / 2
                and edge_z < LOAF_H and -N_SLABS * SLAB - 0.005 < y_line < REST_L)
    loaf_asset, knife_asset = base.scene["loaf"], base.scene["knife"]
    drag = np.zeros(3)
    push = np.zeros(3)
    if in_bread:
        rel_x = sp[0]
        if _EP["rel_x"] is not None:
            saw = abs(rel_x - _EP["rel_x"])
            v_rel = (rel_x - _EP["rel_x"]) * 20.0  # m/s along the blade, against the loaf
            drag = l_rot @ np.array([float(np.clip(DRAG_K * v_rel, -DRAG_MAX, DRAG_MAX)), 0.0, 0.0])
            depth = LOAF_H - edge_z  # how far the edge is below the loaf's top
            started = [j for j in range(1, N_SLABS + 1) if 0.002 < _EP["cut"][j] and not _EP["through"][j]
                       and abs(y_line - (j - N_SLABS) * SLAB) <= KERF]
            if started:  # in a cut already started: the blade follows its kerf
                k = max(started, key=lambda j: _EP["cut"][j])
            on_line = bool(started) or (1 <= k <= N_SLABS and abs(y_line - (k - N_SLABS) * SLAB) <= ON_LINE)
            cut = _EP["cut"][k] if on_line else 0.0
            if on_line and not _EP["through"][k]:
                cut = min(cut + RATE * saw, max(cut, depth))
                _EP["cut"][k] = cut
                if cut >= LOAF_H - 0.002:
                    _EP["through"][k] = True
                    set_glue_on(GLUE_NAMES[k - 1], False)
                    _EP["tip"] = TIP_STEPS
                    cut = LOAF_H
            if depth > cut + 0.002:  # uncut bread under the edge holds the knife up
                push = l_rot @ np.array([0.0, 0.0, min(RES_K * (depth - cut - 0.002), RES_MAX)])
        _EP["rel_x"] = rel_x
        _EP["line"] = k
    else:
        _EP["rel_x"] = None
    _set_force(loaf_asset, drag, l_rot)
    s0 = as_numpy(base.scene["slab_0"].data.root_link_pose_w)[0]
    tip = np.zeros(3)
    if _EP["tip"] > 0:  # a freshly cut slice tips away from the loaf
        _EP["tip"] -= 1
        tip = l_rot @ np.array([0.0, -TIP_FORCE, 0.0])
    _set_force(base.scene["slab_0"], tip, rotation(s0[3:7]), at=(0.0, 0.0, LOAF_H))
    _set_force(knife_asset, push, k_rot, at=(-HANDLE_L / 2, 0.0, HANDLE_Z))  # at the hand: it holds the knife up
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            tl, tr = _EP["tcps"]
            print(f"[slice] t={_DEBUG['steps'] / 20:5.1f}s loaf={np.round(l_pos * 1000).astype(int).tolist()} "
                  f"knife_in_loaf={np.round(sp * 1000).astype(int).tolist()} line={k} in_bread={in_bread} "
                  f"cut(mm)={[round(c * 1000) for c in _EP['cut'][1:]]} through={[i for i, t in enumerate(_EP['through']) if t]} "
                  f"drag={drag[0]:5.1f}N push={push[2]:5.1f}N left_to_blade={_blade_distance(tl, k_pos, k_rot) * 100:4.1f}cm "
                  f"right_to_blade={_blade_distance(tr, k_pos, k_rot) * 100:4.1f}cm "
                  f"unsafe={_EP['unsafe']} right TCP-handle={np.round((tr - (k_pos + k_rot @ np.array([-HANDLE_L / 2, 0.0, HANDLE_Z]))) * 1000).astype(int).tolist()} "
                  f"score={_points()}", file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def _points() -> float:
    if _EP["unsafe"]:
        return 0.0
    right = range(THIN, THICK + 1)
    if any(_EP["through"][k] for k in right):
        return 1.0
    if any(_EP["through"][1:]) or any(_EP["cut"][k] >= LOAF_H / 2 for k in right):
        return 0.5
    return 0.25 if max(_EP["cut"]) >= 0.01 else 0.0


def score(objects) -> float:
    """0.0 if either hand ever came within 3 cm of the blade. Otherwise 1.0 with a slice 1 to 2 cm thick cut clean
    off; 0.5 with a slice of another thickness cut off, or a cut 1 to 2 cm from the end at least half way down; 0.25
    with any cut at least 1 cm deep; else 0.0."""
    return _points()


def success(objects) -> bool:
    """Full score with the loaf still."""
    return _points() >= 1.0 and objects["loaf"].resting()


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand comes straight down on the loaf's top, 11 cm from the cut, fingers closed, and presses it onto the
# board. The right hand comes straight down on the knife's handle, grips it and lifts the knife out of the rest; it
# sets the blade on the loaf's top across it, 1.5 cm from the right end, and saws: back and forth along the blade, lowering it as the
# cut deepens, until the edge is through to the board. Then it lifts the knife out.


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


GRIP = grip_rotation(0.0)  # straight down, fingers closing along y (the left hand, pressing)
KNIFE_GRIP = GRIP  # straight down on the handle, fingers closing across it
HANDLE_IN_HAND = 0.003  # the right TCP this far above the handle's middle
CUT_AT = 3  # glue line 3: a 1.5 cm slice
R_FIX_Y = -0.003  # the knife ends up ~3 mm left of where the right hand aims it: aim right
PRESS_Y = Y_R + 0.13  # the left hand presses here
PRESS = 0.016  # aim the fingertips this far into the loaf's top (the arm gives: it presses the loaf down)
SAW = 0.035  # strokes this far either side of the blade's middle over the loaf
STROKE_S = 0.6
LEAD_S, LEAD_STEPS = 3.0, 15
PICK = [1.0, 0.8, 0.3]


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


def _handle_tcp(spine_x: float, spine_y: float, edge_z: float) -> np.ndarray:
    """The right TCP on the handle with the knife's origin at (spine_x, spine_y) and its edge at height edge_z."""
    return np.array([spine_x - HANDLE_L / 2, spine_y, edge_z + BLADE_H + HANDLE_Z + HANDLE_IN_HAND])


def _right() -> tuple[list[Waypoint], float]:
    pick = _handle_tcp(REST_X, REST_Y, TOP + REST_BASE + 0.0005)
    out = _lead_in("right", pick + [0.0, 0.0, 0.05], KNIFE_GRIP, "over the knife's handle", FINGER_OPEN)
    out += [_w(PICK[0], pick, FINGER_OPEN, "down round the handle", KNIFE_GRIP),
            _w(PICK[1], pick, FINGER_CLOSED, "grip it", KNIFE_GRIP),
            _w(PICK[2], pick, FINGER_CLOSED, "settle", KNIFE_GRIP),
            _w(1.2, pick + [0.0, 0.0, 0.09], FINGER_CLOSED, "lift it out of the rest", KNIFE_GRIP)]
    y = Y_R + CUT_AT * SLAB + R_FIX_Y
    x_mid = LX - BLADE_L / 2  # the blade's middle over the loaf's middle
    top = BOARD_TOP + LOAF_H
    out += [_w(2.0, _handle_tcp(x_mid, y, top + 0.02), FINGER_CLOSED, "over the loaf, 1.5 cm from its end", KNIFE_GRIP),
            _w(0.8, _handle_tcp(x_mid, y, top), FINGER_CLOSED, "edge on the crust", KNIFE_GRIP)]
    start = sum(w.duration for w in out)
    depth, sign = 0.0, 1.0
    while depth < LOAF_H + 0.004:
        depth = min(depth + RATE * 2 * SAW * 0.6, LOAF_H + 0.004)  # a stroke cuts about this much deeper
        out.append(_w(STROKE_S, _handle_tcp(x_mid + sign * SAW, y, top - depth - 0.010), FINGER_CLOSED,
                      "saw: along the blade, lowering it as the cut deepens", KNIFE_GRIP))
        sign = -sign
    out += [_w(STROKE_S, _handle_tcp(x_mid + sign * SAW, y, BOARD_TOP - 0.002), FINGER_CLOSED, "one more stroke, on the board",
               KNIFE_GRIP),
            _w(1.2, _handle_tcp(x_mid, y, top + 0.03), FINGER_CLOSED, "lift the knife out", KNIFE_GRIP),
            _w(3.0, _handle_tcp(x_mid, y, top + 0.03), FINGER_CLOSED, "hold it", KNIFE_GRIP)]
    return out, start


def _left(total: float) -> list[Waypoint]:
    press = np.array([LX, PRESS_Y, BOARD_TOP + LOAF_H + 0.013 - PRESS])
    out = _lead_in("left", press + [0.0, 0.0, 0.05], GRIP, "over the loaf, well left of the cut", FINGER_CLOSED)
    out += [_w(PICK[0], press, FINGER_CLOSED, "fingertips down on the loaf", GRIP)]
    t = LEAD_S + PICK[0]
    out.append(_w(max(total - t, 0.5), press, FINGER_CLOSED, "press it onto the board, fingers clear of the blade", GRIP))
    return out


def demo():
    right, _ = _right()
    total = sum(w.duration for w in right)
    return {"left": _left(total), "right": right}
