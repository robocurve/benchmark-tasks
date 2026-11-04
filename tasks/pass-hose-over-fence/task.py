"""On the station table a white garden fence board (2 cm thick, 12 cm tall) stands on the midline between the two arms
(its faces look towards the arms, y = 0) and runs the whole length of the table, from the edge behind the arm bases
to the far edge, so there is no way round it or under it. Left of the fence (left-arm side) the table is covered by
a green grass mat, right of it by a brown soil mat. On the grass, a grey tap block (40 x 40 x 45 mm) near the fence
holds the fixed end (12 cm in front of the arm bases, 8 cm left of the fence) of a green 9 mm garden hose, 55 cm long, that lies
straight on the grass diagonally away from the robot and to the left (55 deg) and ends in an orange 50 x 22 x 22 mm
nozzle. On the soil, out of the left arm's reach, stands an open grey trough (inner 12 x 8 cm, walls 4 cm tall)
whose inner area spans x 24-36 cm and y -19 to -27 cm. The nozzle must be lifted over the fence by the left arm,
taken on the far side by the right arm and laid in the trough, with the hose left draped over the fence top, and
both grippers must let go.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score \\
           --instruction "<INSTRUCTION below>"
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

INSTRUCTION = ("Pass the green garden hose over the white fence: lift its orange nozzle over the fence with the left "
               "arm, take it on the far side with the right arm and lay it in the grey trough, leaving the hose "
               "draped over the top of the fence. Then let go of the hose.")

# -- ground mats (left = grass, right = soil) and the fence along the midline -----------------------------------
TABLE_X0, TABLE_X1 = -0.19, 0.91  # station table extent along x
MAT_T = 0.002
GROUND = TOP + MAT_T
FENCE_T = 0.020  # thickness (y)
FENCE_H = 0.120
FENCE_TOP = TOP + FENCE_H
MAT_X0, MAT_X1 = 0.05, 0.85
MAT_Y = 0.58  # mats reach this far out from the fence

GRASS = (0.40, 0.55, 0.26)
SOIL = (0.42, 0.29, 0.17)
WHITE = (0.93, 0.93, 0.89)
GREY = (0.58, 0.60, 0.62)

FENCE = Box("fence", pos=((TABLE_X0 + TABLE_X1) / 2, 0.0, TOP + FENCE_H / 2), size=(TABLE_X1 - TABLE_X0, FENCE_T,
            FENCE_H), color=WHITE)
_my0 = FENCE_T / 2 + 0.001
GRASS_MAT = Box("grass_mat", pos=((MAT_X0 + MAT_X1) / 2, (_my0 + MAT_Y) / 2, TOP + MAT_T / 2),
                size=(MAT_X1 - MAT_X0, MAT_Y - _my0, MAT_T), color=GRASS)
SOIL_MAT = Box("soil_mat", pos=((MAT_X0 + MAT_X1) / 2, -(_my0 + MAT_Y) / 2, TOP + MAT_T / 2),
               size=(MAT_X1 - MAT_X0, MAT_Y - _my0, MAT_T), color=SOIL)

# -- trough on the soil side (fixed walls; the soil mat is its floor) ---------------------------------------------
TR_X0, TR_X1 = 0.24, 0.36  # inner area
TR_Y0, TR_Y1 = -0.27, -0.19
TR_WALL, TR_H = 0.010, 0.040
TR_TOP = GROUND + TR_H
TR_C = ((TR_X0 + TR_X1) / 2, (TR_Y0 + TR_Y1) / 2)
TROUGH = [
    Box("trough_near", pos=(TR_X0 - TR_WALL / 2, TR_C[1], GROUND + TR_H / 2),
        size=(TR_WALL, TR_Y1 - TR_Y0 + 2 * TR_WALL, TR_H), color=GREY),
    Box("trough_far", pos=(TR_X1 + TR_WALL / 2, TR_C[1], GROUND + TR_H / 2),
        size=(TR_WALL, TR_Y1 - TR_Y0 + 2 * TR_WALL, TR_H), color=GREY),
    Box("trough_inner", pos=(TR_C[0], TR_Y1 + TR_WALL / 2, GROUND + TR_H / 2),
        size=(TR_X1 - TR_X0, TR_WALL, TR_H), color=GREY),
    Box("trough_outer", pos=(TR_C[0], TR_Y0 - TR_WALL / 2, GROUND + TR_H / 2),
        size=(TR_X1 - TR_X0, TR_WALL, TR_H), color=GREY),
]

# -- hose: fixed at the tap, orange nozzle on its free end ----------------------------------------------------------
R = 0.0045  # 9 mm hose
HOSE_LEN = 0.55
NOZZLE = Plug(size=(0.050, 0.022, 0.022), mass=0.020, color=(0.95, 0.55, 0.05))
HOSE_YAW = math.radians(55.0)
_DIR = np.array([math.cos(HOSE_YAW), math.sin(HOSE_YAW)])
HOSE_START = (0.12, 0.08, GROUND + NOZZLE.size[2] / 2 + 0.0005)  # raised so the nozzle does not start in the mat
TAP_SIZE = (0.040, 0.040, 0.045)
_tc = np.array(HOSE_START[:2]) - _DIR * (TAP_SIZE[0] / 2 + 0.006)  # hose leaves 6 mm in front of the tap face
TAP = Box("tap", pos=(float(_tc[0]), float(_tc[1]), GROUND + TAP_SIZE[2] / 2), size=TAP_SIZE, yaw=HOSE_YAW,
          color=(0.45, 0.47, 0.50))
HOSE = cable("hose", start=HOSE_START, yaw=HOSE_YAW, length=HOSE_LEN, radius=R, anchor_start=True,
             end_plug=NOZZLE, cable_mass=0.06, color=(0.05, 0.62, 0.15), bend_stiffness=0.01, bend_damping=0.001)

OBJECTS = [FENCE, GRASS_MAT, SOIL_MAT, *TROUGH, TAP, HOSE]

# -- grippers (score requires both to have let go) ----------------------------------------------------------------
RELEASE_DIST = 0.035  # each gripper TCP (between the finger pads) at least this far from the hose and the nozzle
_ENV: dict = {"env": None}


def _install_env_hook() -> None:
    """Keep a handle on the sim env so score() can check that the grippers have let go."""
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls._task_step, "_hose_hooked", False):
        return
    base = cls._task_step

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        return base(self, *a, **k)

    _task_step._hose_hooked = True
    cls._task_step = _task_step


_install_env_hook()


def grippers() -> list[np.ndarray]:
    """World positions of both grippers' TCP (between the finger pads, 13 mm above the pad tips), [] if unknown."""
    env = _ENV["env"]
    if env is None:
        return []
    base = env.unwrapped
    origin = base.scene.env_origins[0]
    out = []
    for key in ("left_robot", "right_robot"):
        art = base.scene[key]
        names = list(art.body_names)
        pos = (getattr(art.data.body_pos_w, "torch", art.data.body_pos_w)[0] - origin).detach().cpu().numpy()
        g = pos[names.index("gripper")].astype(np.float64)
        mid = (pos[names.index("tip_left")] + pos[names.index("tip_right")]).astype(np.float64) / 2
        ax = (mid - g) / (np.linalg.norm(mid - g) + 1e-12)
        out.append(g + ax * 0.134)
    return out


# -- geometry helpers -------------------------------------------------------------------------------------------


def _dense(poly: np.ndarray, step: float = 0.003) -> np.ndarray:
    out = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        k = max(1, int(np.linalg.norm(b - a) / step))
        out.extend(a + (b - a) * (j / k) for j in range(1, k + 1))
    return np.array(out)


def _line(objects) -> np.ndarray:
    """Hose centre line from the tap to the nozzle's front face."""
    c = objects["hose"]
    poly = c.polyline()
    if c.end_plug is not None:
        poly = np.vstack([poly, c.end_tip + c.end_dir * NOZZLE.size[0]])
    return poly


def crossings(objects) -> list[np.ndarray]:
    """Every point where the hose (or nozzle) centre line crosses the fence's mid-plane y = 0."""
    poly = _line(objects)
    out = []
    for i in range(len(poly) - 1):
        a, b = poly[i, 1], poly[i + 1, 1]
        if (a > 0) != (b > 0):
            t = a / (a - b)
            out.append(poly[i] + t * (poly[i + 1] - poly[i]))
    return out


def released(objects) -> bool:
    pts = _dense(_line(objects))
    return all(float(np.linalg.norm(pts - g, axis=1).min()) >= RELEASE_DIST for g in grippers())


def in_trough(objects) -> bool:
    """Nozzle centre inside the trough's inner walls (x 24-36 cm, y -19 to -27 cm) and below the wall tops."""
    p = objects["hose"].end_plug
    return bool(p is not None and TR_X0 <= p[0] <= TR_X1 and TR_Y0 <= p[1] <= TR_Y1 and p[2] <= TR_TOP)


ON_TOP_BELOW, ON_TOP_ABOVE = 0.02, 0.03


def on_fence_top(objects) -> bool:
    """The hose crosses the fence line exactly once, resting on the top: centre line between 2 cm below and 3 cm above
    the fence top where it crosses."""
    xs = crossings(objects)
    return len(xs) == 1 and FENCE_TOP - ON_TOP_BELOW <= xs[0][2] <= FENCE_TOP + ON_TOP_ABOVE \
        and TABLE_X0 <= xs[0][0] <= TABLE_X1


def score(objects) -> float:
    """0 unless both grippers are >= 3.5 cm from the hose and nozzle and the nozzle centre is inside the trough (inner
    walls, below their tops). Then 0.7, + 0.3 if the hose crosses the fence line exactly once, resting on its top."""
    if not released(objects) or not in_trough(objects):
        return 0.0
    return 1.0 if on_fence_top(objects) else 0.7


def success(objects) -> bool:
    return score(objects) >= 1.0 and objects["hose"].resting(max_speed=0.02)


def distance(objects) -> float:
    """Nozzle centre to the trough centre (m)."""
    p = objects["hose"].end_plug
    return float(np.linalg.norm(p[:2] - np.array(TR_C))) if p is not None else 1.0


# -- closed-loop scripted demo (cabledemo.py) --------------------------------------------------------------------


def _grip_yaw(heading: float) -> float:
    return (heading + math.pi / 2) % math.pi - math.pi / 2


def demo_script(ctx):
    """Left arm takes the nozzle, turns it to point across the fence, lifts it over the fence top and lowers it on the
    far side onto the soil; the right arm picks it up (at its live position), lays it in the trough and both arms
    move clear.  HOSE_CHEAT=nothing | leftonly (left arm swings the nozzle over towards the trough and lets go,
    no hand-off) | hold (full demo but the right arm keeps holding the nozzle in the trough) | around (try to pass
    the nozzle round the fence's far end) -- all must score 0."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    L, Rt = ctx.left, ctx.right
    cheat = os.environ.get("HOSE_CHEAT", "")

    def log(tag):
        st = ctx.states()
        c = st["hose"]
        xs = [np.round(p, 3).tolist() for p in crossings(st)]
        print(f"[demo t={ctx.t:5.1f}] {tag}: nozzle={np.round(c.end_plug, 3)} cross={xs} trough={in_trough(st)} "
              f"rel={released(st)} score={score(st):.2f}", flush=True)

    log("start")
    if cheat == "nothing":
        yield from L.wait(4.0)
        log("end")
        return

    # 1) Left arm grips the nozzle 15 mm in from its hose end.
    st = ctx.states()
    c = st["hose"]
    g = c.end_tip + c.end_dir * 0.015
    gy = _grip_yaw(math.atan2(c.end_dir[1], c.end_dir[0]))
    yield from L.move((g[0], g[1], TOP + 0.06), yaw=gy, finger=FINGER_OPEN, duration=1.5)
    yield from L.move((g[0], g[1], c.end_plug[2] + 0.010), yaw=gy, duration=1.0)
    yield from L.grip(FINGER_CLOSED)
    yield from L.move((g[0], g[1], TOP + 0.06), yaw=gy, duration=0.8)
    log("left has nozzle")
    # 2) Turn the nozzle to point across the fence (-y) while bringing it in, then lift it over the fence top.
    YL = -math.pi / 2
    hi = FENCE_TOP + 0.05  # TCP height crossing the fence: nozzle bottom ~2.5 cm above its top
    yield from L.move((0.30, 0.25, hi), yaw=0.0, duration=1.5)
    yield from L.move((0.23, 0.07, hi), yaw=YL, duration=1.5)
    if cheat == "around":
        # carry the nozzle low along the fence to the far end of the left arm's reach and offer it to the right
        # arm across the fence; the fence runs to the table's far edge, so there is no way round.
        yield from L.move((0.45, 0.05, GROUND + 0.04), yaw=YL, duration=2.0)
        yield from Rt.move((0.48, -0.05, GROUND + 0.04), yaw=math.pi / 2, finger=FINGER_OPEN, duration=2.0)
        yield from L.move((0.60, 0.05, GROUND + 0.04), yaw=YL, duration=1.5)
        yield from Rt.move((0.60, -0.05, GROUND + 0.04), yaw=math.pi / 2, duration=1.5)
        log("at the fence end of reach")
        yield from L.grip(FINGER_OPEN)
        yield from ctx.both(L.move((0.30, 0.25, hi), yaw=YL, duration=1.2),
                            Rt.move((0.30, -0.30, hi), yaw=math.pi / 2, duration=1.2))
        yield from L.wait(1.5)
        log("end")
        return
    yield from L.move((0.23, -0.03, hi), yaw=YL, duration=1.2)
    log("over the fence")
    if cheat == "leftonly":
        # swing out towards the trough as far as the arm goes and open the fingers during the swing (a throw)
        yield from L.move((0.26, -0.12, GROUND + 0.07), yaw=YL, finger=FINGER_OPEN, duration=0.5)
        yield from L.move((0.24, -0.04, hi), yaw=YL, duration=0.8)
        yield from L.move((0.20, 0.25, hi), yaw=YL, duration=1.5)
        yield from L.wait(2.0)
        log("end")
        return
    # 3) Lower it on the far side and put it down on the soil, then the left arm backs off over the fence.
    yield from L.move((0.24, -0.06, GROUND + 0.06), yaw=YL, duration=1.2)
    yield from L.grip(FINGER_OPEN, duration=0.5)
    yield from L.move((0.24, -0.06, GROUND + 0.10), yaw=YL, duration=0.6)
    yield from L.move((0.23, -0.04, hi), yaw=YL, duration=0.6)
    yield from L.move((0.22, 0.20, hi), yaw=YL, duration=1.2)
    log("left let go")
    # 4) Right arm picks the nozzle up where it lies and lays it in the trough.
    st = ctx.states()
    c = st["hose"]
    p = c.end_plug
    ry = _grip_yaw(math.atan2(c.end_dir[1], c.end_dir[0]))
    if ry < -1.2:
        ry = math.pi / 2  # same grasp, IK-friendly for the right arm
    carry = TR_TOP + 0.05
    yield from ctx.both(Rt.move((p[0], p[1], carry), yaw=ry, finger=FINGER_OPEN, duration=1.5),
                        L.move((0.15, 0.30, hi), yaw=0.0, duration=1.5))
    p = ctx.states()["hose"].end_plug
    yield from Rt.move((p[0], p[1], p[2] + 0.010), yaw=ry, duration=1.0)
    yield from Rt.grip(FINGER_CLOSED)
    yield from Rt.move((p[0], p[1], carry), yaw=ry, duration=0.8)
    log("right has nozzle")
    yield from Rt.move((TR_C[0], TR_C[1], carry), yaw=math.pi / 2, duration=1.5)
    yield from Rt.move((TR_C[0], TR_C[1], GROUND + NOZZLE.size[2] / 2 + 0.016), yaw=math.pi / 2, duration=1.0)
    log("in trough (held)")
    if cheat == "hold":
        yield from Rt.wait(3.0)
        log("end (held)")
        return
    yield from Rt.grip(FINGER_OPEN, duration=0.5)
    yield from Rt.move((TR_C[0], TR_C[1], carry + 0.05), yaw=math.pi / 2, duration=0.8)
    yield from Rt.move((0.20, -0.32, carry + 0.05), yaw=math.pi / 2, duration=1.0)
    log("released")
    yield from Rt.wait(1.5)
    log("end")
