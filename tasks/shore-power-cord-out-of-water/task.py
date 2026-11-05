"""On the station table: a raised fixed dock (light-brown deck, 24 x 64 cm, 6 cm high) fills the far part of the
table; in front of it, at table level, a blue band stands for the water. A boat's hull (white block, 15 x 22 cm, its
deck level with the dock) lies in the water in front of the right arm, 3 cm off the dock edge. Along the dock's front
face, 1 cm in front of it, floats a narrow grey fender board (7 cm wide, its top 3.5 cm above the water, 2.5 cm below
the deck). A thick yellow shore-power cord (9 mm, 52 cm, rigid dark-grey plug on its free end) leaves the hull's inlet
(on the hull's left side, at deck height) and lies, dry, along the fender board, plug on the board in front of the
left arm. On the dock, in front of the left arm, stands a grey shore-power pedestal whose socket (an opening at deck
level facing right, towards the hull) takes the plug. The robot must lay the whole cord on the dock (off the fender
board, where it could slip into the water) and push the plug fully into the socket, then let go: no part of the cord
beyond the first 12 cm from the inlet (marked by a black band) may be off the dock. Plugging in with the cord still
hanging off the dock scores 0.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score \\
           --instruction "<INSTRUCTION below>"
Test-only demo variants (env var SHORE_DEMO): correct (default) | nothing | plug_only | dangle | hold | dock_only.
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

INSTRUCTION = ("Connect the moored boat to shore power. Lift the yellow cord off the grey fender board and lay the "
               "whole cord on the dock, clear of the water and the fender board, then push its plug fully into the "
               "grey pedestal's socket and let go.")

WATER_C = (0.16, 0.42, 0.78)
DECK_C = (0.62, 0.45, 0.28)
HULL_C = (0.93, 0.93, 0.91)
GREY = (0.45, 0.47, 0.50)
DARK = (0.12, 0.12, 0.13)
YELLOW = (0.95, 0.78, 0.08)

# -- dock (raised deck), water, hull ------------------------------------------------------------------------------
DOCK_H = 0.06
DOCK_TOP = TOP + DOCK_H
DOCK_X0, DOCK_X1 = 0.30, 0.54  # front edge (towards the robot) and back edge
DOCK_Y0, DOCK_Y1 = -0.24, 0.40
DOCK = Box("dock", pos=((DOCK_X0 + DOCK_X1) / 2, (DOCK_Y0 + DOCK_Y1) / 2, TOP + DOCK_H / 2),
           size=(DOCK_X1 - DOCK_X0, DOCK_Y1 - DOCK_Y0, DOCK_H), color=DECK_C)
WATER_X0 = 0.04
WATER = Box("water", pos=((WATER_X0 + DOCK_X0) / 2, 0.0, TOP + 0.0005), size=(DOCK_X0 - WATER_X0, 0.84, 0.001),
            color=WATER_C)
HULL_X0, HULL_X1 = 0.12, 0.27
HULL_Y0, HULL_Y1 = -0.44, -0.22
HULL = Box("hull", pos=((HULL_X0 + HULL_X1) / 2, (HULL_Y0 + HULL_Y1) / 2, TOP + 0.001 + DOCK_H / 2),
           size=(HULL_X1 - HULL_X0, HULL_Y1 - HULL_Y0, DOCK_H - 0.002), color=HULL_C)

# -- cord: leaves the hull's inlet (on its +y side) at deck height, heading +y along the dock edge ----------------
R = 0.0045  # 9 mm cord
CORD_LEN = 0.52
INLET_X = 0.255
INLET_Z = DOCK_TOP + R + 0.002
INLET_SIZE = (0.03, 0.012, 0.03)
INLET = Box("inlet", pos=(INLET_X, HULL_Y1 + INLET_SIZE[1] / 2, INLET_Z), size=INLET_SIZE, color=DARK)
PL, PW, PH = 0.045, 0.020, 0.020  # plug body
PLUG = Plug(size=(PL, PW, PH), mass=0.02, color=DARK)
CORD_START = (INLET_X, HULL_Y1 + INLET_SIZE[1] + 0.006, INLET_Z)  # cord end 6 mm off the inlet face
CORD = cable("cord", start=CORD_START, yaw=math.pi / 2, length=CORD_LEN, radius=R, anchor_start=True,
             end_plug=PLUG, cable_mass=0.06, color=YELLOW, bend_stiffness=0.01, bend_damping=0.001)

# -- shore-power pedestal on the dock: body + socket sleeve opening towards -y (the hull) ----------------------------
PORT_X = 0.40
FACE_Y = 0.24  # socket opening
CAV_W, CAV_H, CAV_D = PW + 0.005, PH + 0.005, 0.030  # 25 (x) x 25 (z) mm opening, 30 mm deep
SLV_T = 0.005
SLV_W, SLV_H, SLV_LEN = CAV_W + 2 * SLV_T, CAV_H + SLV_T, 0.050
PED_SIZE = (0.09, 0.09, 0.12)
PEDESTAL = [
    Box("pedestal", pos=(PORT_X, FACE_Y + SLV_LEN + PED_SIZE[1] / 2, DOCK_TOP + PED_SIZE[2] / 2), size=PED_SIZE,
        color=GREY),
    Box("socket_back", pos=(PORT_X, FACE_Y + CAV_D + (SLV_LEN - CAV_D) / 2, DOCK_TOP + SLV_H / 2),
        size=(SLV_W, SLV_LEN - CAV_D, SLV_H), color=DARK),
    Box("socket_roof", pos=(PORT_X, FACE_Y + CAV_D / 2, DOCK_TOP + (CAV_H + SLV_H) / 2),
        size=(SLV_W, CAV_D, SLV_H - CAV_H), color=DARK),
    Box("socket_wall_f", pos=(PORT_X + (CAV_W + SLV_T) / 2, FACE_Y + CAV_D / 2, DOCK_TOP + CAV_H / 2),
        size=(SLV_T, CAV_D, CAV_H), color=DARK),
    Box("socket_wall_n", pos=(PORT_X - (CAV_W + SLV_T) / 2, FACE_Y + CAV_D / 2, DOCK_TOP + CAV_H / 2),
        size=(SLV_T, CAV_D, CAV_H), color=DARK),
]

# -- fender board: a dry, low board floating along the dock face; the cord starts lying on it ---------------------
FENDER_X0, FENDER_X1 = 0.22, 0.29
FENDER_Y0, FENDER_Y1 = -0.205, 0.40
FENDER_TOP = TOP + 0.035
FENDER = Box("fender_board", pos=((FENDER_X0 + FENDER_X1) / 2, (FENDER_Y0 + FENDER_Y1) / 2, (TOP + 0.001 + FENDER_TOP) / 2),
             size=(FENDER_X1 - FENDER_X0, FENDER_Y1 - FENDER_Y0, FENDER_TOP - TOP - 0.001), color=(0.55, 0.56, 0.58))

OBJECTS = [WATER, DOCK, HULL, FENDER, INLET, CORD, *PEDESTAL]

# -- black band on the cord 12-14 cm from the inlet (end of the exempt hull-to-dock run). Visual only. --------------
EXEMPT = 0.12  # m of cord from the inlet that may cross the gap between hull and dock
BAND_SEG = int(round(EXEMPT / CORD.seg_len))


def _install_band() -> None:
    orig = cablekit._spawn_cable
    if getattr(orig, "_shore_band", False):
        return

    def spawn(prim_path, cfg, *a, **k):
        prim = orig(prim_path, cfg, *a, **k)
        try:
            import isaaclab.sim as sim_utils
            from isaaclab.sim.utils import bind_visual_material
            from isaaclab.sim.utils.stage import get_current_stage
            from yam_arena.station import linear_rgb

            if cfg.cable_name == CORD.name:
                look = f"{prim_path}/Looks/band"
                mat = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(DARK), roughness=0.5)
                mat.func(look, mat)
                bind_visual_material(f"{prim_path}/seg_{BAND_SEG}/geom", look, stage=get_current_stage())
        except Exception as e:  # pragma: no cover - never break the scene for a colour
            print(f"[shore task] band colour not applied: {e}", flush=True)
        return prim

    spawn._shore_band = True
    cablekit._spawn_cable = spawn


_install_band()

# -- start-of-episode settling + env handle --------------------------------------------------------------------------
# The kit spawns the cord straight at deck height above the fender board; after every reset the scene is stepped (arms at
# READY) for SETTLE_EXTRA s so each trial starts with the cord at rest on the fender board.
SETTLE_EXTRA = 3.0
_ENV: dict = {"env": None}


def _install_hooks() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
        return
    cls = ie.YamArenaEmbodiment
    if not getattr(cls._task_step, "_shore_hooked", False):
        base_step = cls._task_step

        def _task_step(self, *a, **k):
            _ENV["env"] = self._env
            return base_step(self, *a, **k)

        _task_step._shore_hooked = True
        cls._task_step = _task_step
    if not getattr(cls.reset, "_shore_hooked", False):
        base_reset = cls.reset

        def reset(self, *a, **k):
            base_reset(self, *a, **k)
            _ENV["env"] = self._env
            b = self._base
            for _ in range(max(1, round(SETTLE_EXTRA / b.step_dt))):
                self._env.step(self._ready)
            for _ in range(4):
                b.sim.render()
            self._t = 0.0
            return self._observation()

        reset._shore_hooked = True
        cls.reset = reset


_install_hooks()

# -- scoring -----------------------------------------------------------------------------------------------------
SEATED_MIN = 0.015  # plug tip at least 15 mm inside the socket
RELEASE_PLUG = 0.045  # both TCPs at least this far from the plug centre ...
RELEASE_CORD = 0.03  # ... and from every point of the cord


def _dense(poly: np.ndarray, step: float = 0.004) -> tuple[np.ndarray, np.ndarray]:
    """Points every ~4 mm along the polyline and their arclength from the inlet end."""
    out, s = [poly[0]], [0.0]
    acc = 0.0
    for a, b in zip(poly[:-1], poly[1:]):
        d = float(np.linalg.norm(b - a))
        k = max(1, int(d / step))
        for j in range(1, k + 1):
            out.append(a + (b - a) * (j / k))
            s.append(acc + d * j / k)
        acc += d
    return np.array(out), np.array(s)


def _finite(objects) -> bool:
    c = objects[CORD.name]
    return bool(np.isfinite(c.points).all() and c.end_plug is not None and np.isfinite(c.end_plug).all())


def _on_deck(p: np.ndarray) -> np.ndarray:
    """Top view: centre inside the dock outline; and not below the deck (1 cm tolerance)."""
    p = np.atleast_2d(p)
    return ((p[:, 0] >= DOCK_X0) & (p[:, 0] <= DOCK_X1) & (p[:, 1] >= DOCK_Y0) & (p[:, 1] <= DOCK_Y1)
            & (p[:, 2] >= DOCK_TOP - 0.01))


def off_dock(objects) -> float:
    """Length (m) of cord beyond the exempt 12 cm, plus the plug, that is not over the dock (hanging over the edge,
    on the fender board, over the water, on the hull...)."""
    if not _finite(objects):
        return 1.0
    c = objects[CORD.name]
    pts, s = _dense(c.polyline())
    m = s >= EXEMPT
    bad = float((~_on_deck(pts[m])).sum()) * 0.004
    if not _on_deck(c.end_plug)[0]:
        bad += PL
    return bad


def cord_on_dock(objects) -> bool:
    return off_dock(objects) == 0.0


def depth(objects) -> float:
    """How far (m) the plug tip is inside the socket; -1 if it is not lined up with the opening."""
    if not _finite(objects):
        return -1.0
    c = objects[CORD.name]
    p = np.asarray(c.end_plug, dtype=float)
    a = p - np.asarray(c.points[-1], dtype=float)
    a /= np.linalg.norm(a) + 1e-12
    if abs(p[0] - PORT_X) > 0.008 or p[2] > DOCK_TOP + CAV_H + 0.005 or a[1] < 0.9:
        return -1.0
    return float(p[1] + PL / 2 - FACE_Y)


def plug_seated(objects) -> bool:
    return depth(objects) >= SEATED_MIN


def grippers() -> list[np.ndarray]:
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


def released(objects) -> bool:
    if not _finite(objects):
        return False
    c = objects[CORD.name]
    pts, _ = _dense(c.polyline())
    for g in grippers():
        if np.linalg.norm(c.end_plug - g) < RELEASE_PLUG or np.linalg.norm(pts - g, axis=1).min() < RELEASE_CORD:
            return False
    return True


def score(objects) -> float:
    """0 if a gripper still holds/touches the cord or plug, or if any of the cord beyond 12 cm from the inlet, or the
    plug, is not over the dock (top view: over the edge, on the fender board or hull, over water). Otherwise 0.5 for the
    cord lying on the dock, plus 0.5 if the plug tip is >= 15 mm inside the socket, lined up with it."""
    if not released(objects) or not cord_on_dock(objects):
        return 0.0
    return 0.5 + (0.5 if plug_seated(objects) else 0.0)


def success(objects) -> bool:
    return bool(score(objects) >= 1.0 and objects[CORD.name].resting(0.02, 1.0))


def distance(objects) -> float:
    """Cord length off the dock plus how far the plug still is from 15 mm deep (0.1 if not lined up)."""
    d = depth(objects)
    return float(off_dock(objects) + (max(0.0, SEATED_MIN - d) if d > -1 else 0.1))


# -- closed-loop scripted demo (cabledemo.py) ----------------------------------------------------------------------
# Right arm: takes the cord off the fender board 20 cm from the inlet (the left arm can't reach there), lifts it over
# the dock edge and holds it on the deck 10 cm behind the edge. Left arm: picks the plug up off the board, carries it
# over the deck to the socket and pushes it in, then lets go. Right arm then lays its part down and lets go.

FINGER_PINCH = -0.020  # plug pinch (pads ~44 mm apart)
FINGER_CORD = -0.014  # narrow opening for the cord next to the dock wall
GRASP_BACK = 0.010
SEAT_TO = 0.026
HOLD_S = float(os.environ.get("SHORE_HOLD_S", "0.20"))
HOLD_XY = (float(os.environ.get("SHORE_HOLD_X", "0.40")), float(os.environ.get("SHORE_HOLD_Y", "-0.07")))


def _point_at(c, s: float):
    poly = c.polyline()
    seg = np.linalg.norm(np.diff(poly, axis=0), axis=1)
    acc = np.r_[0.0, np.cumsum(seg)]
    i = int(np.clip(np.searchsorted(acc, s) - 1, 0, len(seg) - 1))
    t = (s - acc[i]) / max(seg[i], 1e-9)
    return poly[i] + t * (poly[i + 1] - poly[i]), poly[i + 1] - poly[i]


def _grip_yaw(heading: float) -> float:
    return (heading + math.pi / 2) % math.pi - math.pi / 2


def _log(ctx, tag=""):
    st = ctx.states()
    c = st[CORD.name]
    print(f"[t={ctx.t:5.2f}] {tag}: off={off_dock(st):.3f} depth={depth(st) * 1000:.1f}mm rel={released(st)} "
          f"score={score(st)} plug={np.round(c.end_plug, 3).tolist()} "
          f"minz={c.points[:, 2].min() - TOP:.3f} tcp={[np.round(g, 3).tolist() for g in grippers()]}", flush=True)


def _dbg(ctx, tag):
    if os.environ.get("SHORE_DBG"):
        _log(ctx, tag)
        c = ctx.states()[CORD.name]
        ax = c.end_plug - c.points[-1]
        print(f"   plug heading={math.degrees(math.atan2(ax[1], ax[0])):.1f} deg", flush=True)
        print(f"   cmd L={np.round(ctx.left.tcp(), 4).tolist()} R={np.round(ctx.right.tcp(), 4).tolist()}", flush=True)


def _right_lift(Rt, ctx, hold):
    """Take the cord at HOLD_S, lift it over the edge and press it onto the deck at HOLD_XY (keeps holding)."""
    from yam_arena.constants import FINGER_CLOSED

    p, h = _point_at(ctx.states()[CORD.name], HOLD_S)
    gy = _grip_yaw(math.atan2(h[1], h[0]))
    if gy < 0:  # the right arm reaches this spot better with the wrist turned the other way
        gy = min(gy + math.pi, math.pi / 2)
    yield from Rt.move((p[0], p[1], TOP + 0.08), yaw=gy, finger=FINGER_CORD, duration=1.5)
    p, h = _point_at(ctx.states()[CORD.name], HOLD_S)
    yield from Rt.move((p[0], p[1], p[2] + 0.010), yaw=gy, duration=1.0)
    yield from Rt.grip(FINGER_CLOSED)
    yield from Rt.move((p[0], p[1], DOCK_TOP + 0.05), yaw=gy, duration=1.0)
    yield from Rt.move((hold[0], hold[1], DOCK_TOP + 0.05), yaw=gy, duration=1.5)
    yield from Rt.move((hold[0], hold[1], DOCK_TOP + R + 0.010), yaw=gy, duration=1.0)
    return gy


def _left_plug(L, ctx, insert=True):
    from yam_arena.constants import FINGER_CLOSED

    yaw = math.pi / 2  # wrist camera (offset along +yaw) faces the socket while the plug goes in along +y
    c = ctx.states()[CORD.name]
    p = np.asarray(c.end_plug)
    a = p - c.points[-1]
    a /= np.linalg.norm(a)
    g = p - a * (PL / 2 - GRASP_BACK)
    gz = p[2] + PH / 2 + 0.003
    yield from L.move((g[0], g[1], TOP + 0.08), yaw=yaw, finger=FINGER_PINCH, duration=1.5)
    yield from L.move((g[0], g[1], gz), yaw=yaw, duration=1.0)
    yield from L.grip(FINGER_CLOSED, 0.6)
    yield from L.move((g[0], g[1], DOCK_TOP + 0.05), yaw=yaw, duration=1.0)
    _dbg(ctx, "plug lifted")
    # carry over the deck, plug tip 1.5 cm short of the socket opening, lined up with it
    zz = DOCK_TOP + PH + 0.003
    gy0 = FACE_Y - 0.015 - (PL - GRASP_BACK)
    yield from L.move((PORT_X, gy0, DOCK_TOP + 0.05), yaw=yaw, duration=1.8)
    yield from L.move((PORT_X, gy0, zz), yaw=yaw, duration=0.8)
    yield from L.wait(0.3)
    _dbg(ctx, "staged")
    if not insert:
        return gy0, yaw
    dx, dz, dyaw = 0.0, 0.0, 0.0
    z_goal = DOCK_TOP + PH / 2 + 0.002  # plug centre just above the deck (the opening is 25 mm tall)
    for _ in range(5):  # line the plug up with the socket: sideways, height and heading (it sits differently in
        c = ctx.states()[CORD.name]  # the grip each time; closed loop on the plug)
        pp = np.asarray(c.end_plug)
        ax = pp - np.asarray(c.points[-1])
        eyaw = math.pi / 2 - math.atan2(ax[1], ax[0])
        ex, ez = PORT_X - pp[0], z_goal - pp[2]
        if abs(ex) < 0.0007 and abs(ez) < 0.0015 and abs(eyaw) < 0.02:
            break
        dyaw = float(np.clip(dyaw + eyaw, -0.4, 0.4))
        dx = float(np.clip(dx + ex, -0.02, 0.02))
        dz = float(np.clip(dz + ez, -0.012, 0.006))
        yield from L.move((PORT_X + dx, gy0, zz + dz), yaw=yaw + dyaw, duration=0.5)
        yield from L.wait(0.2)
    zz += dz
    yaw += dyaw
    _dbg(ctx, f"aligned dx={dx:.4f} dz={dz:.4f} dyaw={dyaw:.3f}")
    gy = gy0 + SEAT_TO + 0.015
    yield from L.move((PORT_X + dx, gy, zz), yaw=yaw, duration=2.0)
    _dbg(ctx, "pushed")
    for _ in range(2):
        d = depth(ctx.states())
        if d >= SEAT_TO - 0.002:
            break
        gy += SEAT_TO - max(d, 0.0)
        yield from L.move((PORT_X + dx, gy, zz), yaw=yaw, duration=1.0)
    yield from L.wait(0.3)
    return gy, yaw


def _left_release(L, gy_yaw):
    gy, yaw = gy_yaw
    yield from L.grip(FINGER_PINCH, 0.6)
    yield from L.move((PORT_X, gy - 0.005, DOCK_TOP + 0.06), yaw=yaw, duration=1.0)
    yield from L.move((0.18, 0.25, TOP + 0.14), yaw=0.0, duration=1.2)


def _right_release(Rt, hold, gy):
    yield from Rt.grip(FINGER_CORD, 0.6)
    yield from Rt.move((hold[0], hold[1], DOCK_TOP + 0.06), yaw=gy, duration=0.8)
    yield from Rt.move((0.18, -0.25, TOP + 0.14), yaw=0.0, duration=1.2)


def demo_script(ctx):
    mode = os.environ.get("SHORE_DEMO", "correct")
    L, Rt = ctx.left, ctx.right
    _log(ctx, "start")
    if mode == "nothing":  # shortcut: do nothing
        for _ in range(4):
            yield from L.wait(1.0)
            _log(ctx)
        return
    if mode == "plug_only":  # shortcut: plug in with the left arm only, the cord left hanging off the dock
        gl = yield from _left_plug(L, ctx)
        _log(ctx, "plugged")
        yield from _left_release(L, gl)
        for _ in range(3):
            yield from L.wait(1.0)
            _log(ctx, "end")
        return
    hold = HOLD_XY
    gr = yield from _right_lift(Rt, ctx, hold)
    _log(ctx, "cord on deck (held)")
    if mode == "dock_only":  # partial: cord laid on the dock, plug left lying on the deck in front of the socket
        gl = yield from _left_plug(L, ctx, insert=False)
    else:
        gl = yield from _left_plug(L, ctx)
    _log(ctx, "plugged (held)")
    if mode == "hold":  # shortcut: everything done, but both arms keep holding
        for _ in range(3):
            yield from L.wait(1.0)
            _log(ctx, "end (held)")
        return
    yield from _left_release(L, gl)
    if mode == "dangle":  # shortcut: plugged in, then the slack is put back over the dock edge onto the fender board
        yield from Rt.move((hold[0], hold[1], DOCK_TOP + 0.05), yaw=gr, duration=1.0)
        yield from Rt.move((0.255, -0.06, DOCK_TOP + 0.05), yaw=gr, duration=1.5)
        yield from Rt.move((0.255, -0.06, FENDER_TOP + R + 0.012), yaw=gr, duration=1.0)
        _log(ctx, "slack back over the edge")
        hold = (0.255, -0.06)
    yield from _right_release(Rt, hold, gr)
    _log(ctx, "released")
    for _ in range(4):
        yield from L.wait(1.0)
        _log(ctx, "settling")
