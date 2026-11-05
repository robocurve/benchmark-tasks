"""E-waste strip-out bench on the station table. A dark-brown wooden upright board (240 mm long, 50 mm tall, 12 mm thick)
stands on the midline between the two arms (faces towards the arms), from x 18 cm to x 42 cm, screwed down. At x 30 cm,
low in the board, is a 14 x 14 mm square hole (bottom 8 mm above the table). A grey 6 mm cord (30 cm) passes through
the hole straight across the board: on the left-arm side 18 cm of it lies on the table and ends in a black moulded
plug (40 x 24 x 18 mm, far too big for the hole); on the right-arm side 12 cm of it lies on the table and ends in a
bare end (a small 10 x 8 x 8 mm copper-coloured crimp). On the right-arm side a blue open bin (inside 18 x 16 cm,
40 mm tall walls) is screwed down, its near wall 15.5 cm from the board.

The cord has to come out of the board and go into the bin. Pulling the bare end towards the bin jams the plug in
the hole, so the cord must be withdrawn by its plug (left arm), handed over the board to the right arm, and dropped
in the bin. The plug and the bin are on opposite sides of the board.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score \\
           --instruction "<INSTRUCTION below>"
Test-only demo variants (env var CORD_CHEAT): nothing | bare (pull the bare end into the bin) | hold (full demo, but
the right arm keeps holding the cord over the bin) | onearm (left arm only: withdraw, carry the plug over the board to y -10 cm, drop).
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import cablekit  # noqa: E402,F401  (hooks yam_arena so the Cable below is simulated)
import numpy as np  # noqa: E402
from cablekit import Plug, cable  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = ("Take the grey cord out of the hole in the wooden board, drop it into the blue bin and let go of it.")

WOOD = (0.36, 0.22, 0.12)  # dark-stained wood: contrasts with the light table
BLUE = (0.12, 0.33, 0.70)
CORD_GREY = (0.55, 0.56, 0.58)
PLUG_BLACK = (0.06, 0.06, 0.07)
COPPER = (0.80, 0.45, 0.20)

# -- board with a square hole (board plane y = 0; plug side = left arm, y > 0; bin side = right arm, y < 0) --------
BOARD_X0, BOARD_X1 = 0.18, 0.42
BOARD_T = 0.012
BOARD_H = 0.050
HOLE_X = 0.30
HOLE_W = 0.014
HOLE_Z0 = TOP + 0.008
HOLE_Z1 = HOLE_Z0 + HOLE_W
HOLE_ZC = (HOLE_Z0 + HOLE_Z1) / 2


def _bpart(name, x0, x1, z0, z1):
    return Box(name, pos=((x0 + x1) / 2, 0.0, (z0 + z1) / 2), size=(x1 - x0, BOARD_T, z1 - z0), color=WOOD)


BOARD = [
    _bpart("board_left", BOARD_X0, HOLE_X - HOLE_W / 2, TOP, TOP + BOARD_H),
    _bpart("board_right", HOLE_X + HOLE_W / 2, BOARD_X1, TOP, TOP + BOARD_H),
    _bpart("board_below", HOLE_X - HOLE_W / 2, HOLE_X + HOLE_W / 2, TOP, HOLE_Z0),
    _bpart("board_above", HOLE_X - HOLE_W / 2, HOLE_X + HOLE_W / 2, HOLE_Z1, TOP + BOARD_H),
]

# -- cord: bare end (crimp) on the bin side, plug on the other side ------------------------------------------------
R = 0.003  # 6 mm cord
CORD_LEN = 0.30  # 15 segments of 2 cm
BARE_LEN = 0.12  # cord on the bin side of the board's mid-plane at the start
PL, PW, PH = 0.040, 0.024, 0.018
PLUG = Plug(size=(PL, PW, PH), mass=0.018, color=PLUG_BLACK)
CRIMP = Plug(size=(0.010, 0.008, 0.008), mass=0.002, color=COPPER)
CORD = cable("cord", start=(HOLE_X, -BARE_LEN, HOLE_ZC), yaw=math.pi / 2, length=CORD_LEN, radius=R,
             cable_mass=0.03, start_plug=CRIMP, end_plug=PLUG, color=CORD_GREY, bend_stiffness=0.01,
             bend_damping=0.001)

# -- bin on the right-arm side ---------------------------------------------------------------------------------------
BIN_X0, BIN_X1 = 0.24, 0.42  # inside
BIN_Y0, BIN_Y1 = -0.32, -0.16  # inside (y1 = near wall, towards the board)
BIN_H = 0.040
BIN_W = 0.005  # wall thickness
_bx, _by = (BIN_X0 + BIN_X1) / 2, (BIN_Y0 + BIN_Y1) / 2
_ox, _oy = BIN_X1 - BIN_X0 + 2 * BIN_W, BIN_Y1 - BIN_Y0 + 2 * BIN_W
BIN = [
    Box("bin_near", pos=(_bx, BIN_Y1 + BIN_W / 2, TOP + BIN_H / 2), size=(_ox, BIN_W, BIN_H), color=BLUE),
    Box("bin_far", pos=(_bx, BIN_Y0 - BIN_W / 2, TOP + BIN_H / 2), size=(_ox, BIN_W, BIN_H), color=BLUE),
    Box("bin_front", pos=(BIN_X0 - BIN_W / 2, _by, TOP + BIN_H / 2), size=(BIN_W, BIN_Y1 - BIN_Y0, BIN_H), color=BLUE),
    Box("bin_back", pos=(BIN_X1 + BIN_W / 2, _by, TOP + BIN_H / 2), size=(BIN_W, BIN_Y1 - BIN_Y0, BIN_H), color=BLUE),
]

OBJECTS = [*BOARD, *BIN, CORD]
TWIN_EXTRA: list = []  # everything is in OBJECTS (plug and crimp are moulded/crimped onto the cord)

# -- scoring ----------------------------------------------------------------------------------------------------------
RELEASE_DIST = 0.03  # every gripper TCP at least 3 cm from every cord point and the plug = let go
APERTURE_TOL = 0.003


def _dense(objects, step: float = 0.005) -> np.ndarray:
    """Cord centre line (crimp centre .. plug tip) resampled every ``step`` m."""
    c = objects["cord"]
    poly = c.polyline()
    pts = [c.start_plug] if c.start_plug is not None else []
    pts += list(poly)
    if c.end_plug is not None:
        pts += [c.end_plug, c.end_tip + c.end_dir * PL]
    poly = np.asarray(pts, dtype=float)
    seg = np.linalg.norm(np.diff(poly, axis=0), axis=1)
    s = np.r_[0.0, np.cumsum(seg)]
    t = np.arange(0.0, s[-1] + 1e-9, step)
    return np.stack([np.interp(t, s, poly[:, k]) for k in range(3)], axis=1)


def _finite(objects) -> bool:
    c = objects["cord"]
    return bool(np.isfinite(c.points).all() and c.end_plug is not None and np.isfinite(c.end_plug).all())


def through_hole(objects) -> bool:
    """True if the cord's centre line crosses the board's mid-plane (y = 0) inside the hole (with 3 mm tolerance),
    i.e. the cord still passes through the board."""
    p = _dense(objects, 0.002)
    for a, b in zip(p[:-1], p[1:]):
        if (a[1] > 0) != (b[1] > 0):
            t = a[1] / (a[1] - b[1])
            q = a + t * (b - a)
            if abs(q[0] - HOLE_X) <= HOLE_W / 2 + APERTURE_TOL and q[2] <= HOLE_Z1 + APERTURE_TOL:
                return True
    return False


def _inside_bin(p: np.ndarray) -> np.ndarray:
    return ((p[:, 0] >= BIN_X0) & (p[:, 0] <= BIN_X1) & (p[:, 1] >= BIN_Y0) & (p[:, 1] <= BIN_Y1)
            & (p[:, 2] <= TOP + BIN_H))


def in_bin_fraction(objects) -> float:
    """Fraction of the cord's length (crimp centre to plug tip) inside the bin: within its inner walls, below its rim."""
    return float(_inside_bin(_dense(objects)).mean())


_ENV: dict = {"env": None}


def _install_env_hook() -> None:
    """Keep a handle on the sim env so score() can check that the grippers have let go."""
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls._task_step, "_cordhole_hooked", False):
        return
    base = cls._task_step

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        return base(self, *a, **k)

    _task_step._cordhole_hooked = True
    cls._task_step = _task_step


_install_env_hook()


def grippers() -> list[np.ndarray]:
    """World TCPs (between the finger pads) of the left and right gripper; [] offline."""
    env = _ENV["env"]
    if env is None:
        return []
    from yam_arena.runtime import tcp_world

    base = env.unwrapped
    origin = np.asarray(getattr(base.scene.env_origins, "torch", base.scene.env_origins)[0].detach().cpu().numpy(),
                        dtype=np.float64)
    return [np.asarray(tcp_world(base.scene[k]), dtype=np.float64) - origin for k in ("left_robot", "right_robot")]


def released(objects) -> bool:
    pts = _dense(objects)
    return all(float(np.linalg.norm(pts - g, axis=1).min()) >= RELEASE_DIST for g in grippers())


def score(objects) -> float:
    """0 if the cord still passes through the hole, a gripper is still holding it (TCP within 3 cm of the cord or
    plug) or the state is invalid. Otherwise 0.3 + 0.7 x (fraction of the cord's length inside the bin); 1.0 when the
    whole cord, plug and crimp included, lies inside the bin's walls and below its rim."""
    if not _finite(objects) or through_hole(objects) or not released(objects):
        return 0.0
    f = in_bin_fraction(objects)
    return 1.0 if f >= 1.0 else 0.3 + 0.7 * f


def success(objects) -> bool:
    return score(objects) >= 1.0 and objects["cord"].resting(max_speed=0.05)


def distance(objects) -> float:
    """1 - fraction in the bin once the cord is out of the hole; 1 + plug distance from its start otherwise."""
    if not _finite(objects):
        return 2.0
    if through_hole(objects):
        return 1.0 + 1.0 / (1.0 + 10 * float(np.linalg.norm(objects["cord"].end_plug[:2] - _PLUG0[:2])))
    return 1.0 - in_bin_fraction(objects)


_PLUG0 = CORD.point_at(CORD_LEN + PL / 2)


def progress(objects) -> dict:
    """Stage diagnostic (not the score)."""
    if not _finite(objects):
        return {"plug_moved": 0.0, "out_of_hole": 0.0, "on_bin_side": 0.0, "in_bin": 0.0, "released": 0.0}
    c = objects["cord"]
    pts = _dense(objects)
    return {"plug_moved": float(np.linalg.norm(c.end_plug[:2] - _PLUG0[:2]) > 0.03),
            "out_of_hole": float(not through_hole(objects)),
            "on_bin_side": float((pts[:, 1] < -BOARD_T / 2).mean()),
            "in_bin": in_bin_fraction(objects), "released": float(released(objects))}


VARIATION = {"cable:cord": {"dx": 0.0, "dy": 0.02, "yaw": 0.0}}  # along the hole axis only (it threads the hole)

# -- closed-loop scripted demo (cabledemo.py) --------------------------------------------------------------------------


def _grip_yaw(heading: float) -> float:
    return (heading + math.pi / 2) % math.pi - math.pi / 2


def _cord_point(objects, a: float) -> tuple[np.ndarray, float]:
    """Live cord point at arclength ``a`` (m) from the crimp end and the cord heading there."""
    c = objects["cord"]
    poly = c.polyline()
    s = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(poly, axis=0), axis=1))]
    p = np.array([np.interp(a, s, poly[:, k]) for k in range(3)])
    i = int(np.clip(np.searchsorted(s, a), 1, len(poly) - 1))
    d = poly[i] - poly[i - 1]
    return p, math.atan2(d[1], d[0])


def demo_script(ctx):
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    L, Rt = ctx.left, ctx.right
    hi = TOP + 0.08
    PLUG_OPEN = -0.022  # pads 44 mm apart round the 24 mm plug
    CORD_OPEN = -0.014  # pads 28 mm apart round the 6 mm cord
    cheat = os.environ.get("CORD_CHEAT", "")

    def log(tag):
        st = ctx.states()
        c = st["cord"]
        print(f"[demo] {tag}: plug={np.round(c.end_plug, 3)} crimp={np.round(c.start_plug, 3)} "
              f"through={through_hole(st)} inbin={in_bin_fraction(st):.2f} rel={released(st)} "
              f"score={score(st):.3f} prog={ {k: round(v, 2) for k, v in progress(st).items()} }", flush=True)

    log("start")
    if cheat == "nothing":
        yield from L.wait(5.0)
        log("end")
        return
    if cheat == "bare":
        # Right arm takes the cord 3 cm from the crimp and pulls it into the bin: the plug jams in the hole.
        st = ctx.states()
        p, h = _cord_point(st, 0.03)
        gy = _grip_yaw(h)
        yield from Rt.move((p[0], p[1], hi), yaw=gy, finger=CORD_OPEN, duration=1.5)
        yield from Rt.move((p[0], p[1], max(p[2], TOP + R) + 0.010), yaw=gy, duration=1.0)
        yield from Rt.grip(FINGER_CLOSED)
        for y in (-0.18, -0.24, -0.28):
            yield from Rt.move((HOLE_X + 0.03, y, TOP + 0.06), yaw=gy, duration=1.5)
            log(f"bare pull to {y}")
        yield from Rt.grip(FINGER_OPEN, duration=0.5)
        yield from Rt.move((0.25, -0.25, TOP + 0.15), yaw=0.0, duration=1.0)
        yield from Rt.wait(3.0)
        log("end")
        return

    # 1) Left arm takes the plug at its middle (wrist camera looking back at the board) and draws the cord out of the
    #    hole along the hole axis until the crimp is clear of the board.
    st = ctx.states()
    c = st["cord"]
    pp = c.end_plug
    gy = -math.pi / 2
    yield from L.move((pp[0], pp[1], hi), yaw=gy, finger=PLUG_OPEN, duration=1.5)
    yield from L.move((pp[0], pp[1], pp[2] + 0.008), yaw=gy, duration=1.0)
    yield from L.grip(FINGER_CLOSED)
    pull_z = TOP + 0.030
    y = pp[1]
    yield from L.move((HOLE_X, y, pull_z), yaw=gy, duration=0.6)
    log("plug gripped")
    for k in range(6):
        st = ctx.states()
        tip = st["cord"].start_plug
        if tip[1] > BOARD_T / 2 + 0.03:
            break
        step = float(np.clip(BOARD_T / 2 + 0.04 - tip[1], 0.02, 0.10))
        y = min(y + step, 0.40)
        yield from L.move((HOLE_X, y, pull_z), yaw=gy, duration=max(0.8, step * 15))
        log(f"pull {k}")
    if cheat == "onearm":
        # Left arm alone: carry the plug as far towards the bin as it reaches and let go.
        yield from L.move((HOLE_X, y, hi), yaw=0.0, duration=1.0)
        yield from L.move((0.26, -0.10, hi), yaw=0.0, duration=2.5)
        yield from L.grip(FINGER_OPEN, duration=0.5)
        yield from L.move((0.25, 0.20, hi), yaw=0.0, duration=1.5)
        yield from L.wait(3.0)
        log("end")
        return
    # 2) Lift the plug over the board (turned so it lies along the board) and set it down on the bin side.
    yield from L.move((HOLE_X, y, hi), yaw=0.0, duration=1.0)
    set_y = -0.05
    yield from L.move((HOLE_X, set_y, hi), yaw=0.0, duration=2.0)
    yield from L.move((HOLE_X, set_y, TOP + PH / 2 + 0.010), yaw=0.0, duration=1.0)
    log("plug set down")
    yield from L.grip(PLUG_OPEN, duration=0.5)
    yield from L.move((HOLE_X, set_y, hi + 0.04), yaw=0.0, duration=0.6)
    yield from L.move((0.25, 0.22, hi), yaw=0.0, duration=1.2)
    log("left away")
    # 3) Right arm takes the plug, carries it over the bin and lowers the cord into it.
    st = ctx.states()
    c = st["cord"]
    pp = c.end_plug
    h = math.atan2(c.end_dir[1], c.end_dir[0])
    gy = _grip_yaw(h)
    yield from Rt.move((pp[0], pp[1], hi), yaw=gy, finger=PLUG_OPEN, duration=1.5)
    yield from Rt.move((pp[0], pp[1], pp[2] + 0.008), yaw=gy, duration=1.0)
    yield from Rt.grip(FINGER_CLOSED)
    log("right has plug")
    yield from Rt.move((pp[0], pp[1], TOP + 0.16), yaw=gy, duration=1.0)
    yield from Rt.move((0.37, -0.27, TOP + 0.16), yaw=0.0, duration=2.0)
    log("over bin")
    yield from Rt.move((0.38, -0.28, TOP + 0.05), yaw=0.0, duration=1.5)
    log("lowered")
    if cheat == "hold":
        yield from Rt.wait(4.0)
        log("end (held)")
        return
    yield from Rt.grip(PLUG_OPEN, duration=0.5)
    yield from Rt.move((0.38, -0.28, TOP + 0.12), yaw=0.0, duration=0.8)
    log("plug in bin")
    # 4) The crimp end still trails over the bin's near wall: pick the cord up near the crimp and lay it in the bin.
    for k in range(3):
        st = ctx.states()
        if in_bin_fraction(st) >= 1.0:
            break
        pts = _dense(st)
        out = np.flatnonzero(~_inside_bin(pts))
        a = float(np.clip(out.mean() * 0.005, 0.03, CORD_LEN - 0.03))  # middle of the part outside the bin
        p, h = _cord_point(st, a)
        gy = _grip_yaw(h)
        yield from Rt.move((p[0], p[1], hi), yaw=gy, finger=CORD_OPEN, duration=1.2)
        yield from Rt.move((p[0], p[1], max(p[2], TOP + R) + 0.010), yaw=gy, duration=0.8)
        yield from Rt.grip(FINGER_CLOSED)
        yield from Rt.move((p[0], p[1], TOP + 0.10), yaw=gy, duration=0.8)
        yield from Rt.move((0.29, -0.22, TOP + 0.10), yaw=gy, duration=1.2)
        yield from Rt.move((0.29, -0.22, TOP + 0.03), yaw=gy, duration=0.6)
        yield from Rt.grip(CORD_OPEN, duration=0.4)
        yield from Rt.move((0.29, -0.22, TOP + 0.12), yaw=gy, duration=0.6)
        yield from Rt.wait(0.5)
        log(f"tail {k}")
    yield from Rt.move((0.20, -0.25, TOP + 0.15), yaw=0.0, duration=0.8)
    yield from Rt.wait(1.0)
    log("released")
    yield from Rt.wait(2.0)
    log("end")
