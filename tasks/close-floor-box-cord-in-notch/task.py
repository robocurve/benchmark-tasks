"""Office fit-out mock-up on the station table: a floor box (a recessed power box set into the floor) built as an open
white well, 14 x 12 cm inside and 4.5 cm deep with 1 cm walls, fixed to the table in front of the robot. The top of its
far wall has a cord notch (15 mm wide, 25 mm deep) 2 cm left of the wall's centre. Inside, a power socket faces UP
(vertical insertion, as in real floor boxes): a dark grey receptacle with a 36 x 26 mm opening, 22 mm deep, with a
10 mm cord slot in its far side. A free grey cover plate (16 x 14 cm, 6 mm thick, 100 g with its 50 x 14 x 20 mm lift
handle on top) rests on the rim and closes the box. On the right, a 6 mm black power cord (42 cm) leaves a fixed grey
service box beyond the well and lies straight on the table towards the robot; its free end carries a right-angle
plug (black block 30 x 20 x 36 mm tall, cord leaving one side at mid-height) standing on the table. The robot must
open the box, plug the cord into the socket, lead the cord out through the notch, close the cover flat and let go. A
cover closed over a cord lying across the rim (the cord's natural path towards the service box) rocks on the cord:
a pinched cord.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score \\
           --instruction "<INSTRUCTION below>"
Test-only demo variants: FLOORBOX_DEMO=correct (default, 1.0) | tier04 (0.4) | tier06 (0.6) | nothing | hold | lazy |
pinch (each 0).
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

INSTRUCTION = ("Plug the black power cord into the socket inside the floor box: lift the cover off, plug in, lead the "
               "cord out through the notch in the box's far wall, then put the cover back flat on the box and let go.")

WHITE = (0.93, 0.93, 0.92)
COVER_GREY = (0.62, 0.64, 0.66)
SOCKET_GREY = (0.30, 0.31, 0.33)
BLACK = (0.05, 0.05, 0.05)

# -- floor-box well (fixed) ----------------------------------------------------------------------------------------
WC = np.array([0.25, 0.0])  # well centre (x, y)
IX, IY = 0.14, 0.12  # inside size
WT, WH = 0.010, 0.045  # wall thickness, wall height (well depth)
RIM = TOP + WH
X_IN0, X_IN1 = WC[0] - IX / 2, WC[0] + IX / 2  # 0.21 .. 0.35
Y_IN0, Y_IN1 = WC[1] - IY / 2, WC[1] + IY / 2  # -0.06 .. 0.06
NOTCH_Y, NOTCH_W, NOTCH_D = 0.02, 0.015, 0.025  # cord notch in the far wall (centre y, width, depth from the rim)

# -- up-facing socket (fixed) --------------------------------------------------------------------------------------
PL, PW, PH = 0.030, 0.020, 0.036  # plug body: length along the cord (x), width, height
SX, SY = 0.27, 0.0  # socket opening centre
CL, CW, CD = PL + 0.006, PW + 0.006, 0.022  # opening 36 (x) x 26 (y) mm, 22 mm deep (floor = table)
ST = 0.006  # socket wall thickness
SLOT_W = 0.010  # cord slot in the socket's +x wall
SOCK_TOP = TOP + CD


def _well() -> list[Box]:
    zc = TOP + WH / 2
    oy = IY + 2 * WT
    out = [
        Box("well_wall_near", pos=(X_IN0 - WT / 2, WC[1], zc), size=(WT, oy, WH), color=WHITE),
        Box("well_wall_left", pos=(WC[0], Y_IN1 + WT / 2, zc), size=(IX, WT, WH), color=WHITE),
        Box("well_wall_right", pos=(WC[0], Y_IN0 - WT / 2, zc), size=(IX, WT, WH), color=WHITE),
    ]
    # far wall in three pieces: right of the notch, the notch sill, left of the notch
    y_lo, y_hi = WC[1] - oy / 2, WC[1] + oy / 2
    n0, n1 = NOTCH_Y - NOTCH_W / 2, NOTCH_Y + NOTCH_W / 2
    xf = X_IN1 + WT / 2
    out += [
        Box("well_wall_far_r", pos=(xf, (y_lo + n0) / 2, zc), size=(WT, n0 - y_lo, WH), color=WHITE),
        Box("well_wall_far_l", pos=(xf, (n1 + y_hi) / 2, zc), size=(WT, y_hi - n1, WH), color=WHITE),
        Box("well_notch_sill", pos=(xf, NOTCH_Y, TOP + (WH - NOTCH_D) / 2), size=(WT, NOTCH_W, WH - NOTCH_D),
            color=WHITE),
    ]
    return out


def _socket() -> list[Box]:
    zc = TOP + CD / 2
    oy = CW + 2 * ST
    side = (oy - SLOT_W) / 2
    xp = SX + CL / 2 + ST / 2
    return [
        Box("socket_wall_near", pos=(SX - CL / 2 - ST / 2, SY, zc), size=(ST, oy, CD), color=SOCKET_GREY),
        Box("socket_wall_left", pos=(SX, SY + CW / 2 + ST / 2, zc), size=(CL, ST, CD), color=SOCKET_GREY),
        Box("socket_wall_right", pos=(SX, SY - CW / 2 - ST / 2, zc), size=(CL, ST, CD), color=SOCKET_GREY),
        Box("socket_wall_far_r", pos=(xp, SY - SLOT_W / 2 - side / 2, zc), size=(ST, side, CD), color=SOCKET_GREY),
        Box("socket_wall_far_l", pos=(xp, SY + SLOT_W / 2 + side / 2, zc), size=(ST, side, CD), color=SOCKET_GREY),
    ]


# -- cover (free) with its lift handle welded on top --------------------------------------------------------------
COV_X, COV_Y, COV_T = 0.16, 0.14, 0.006
COVER_MASS, HANDLE_MASS = 0.09, 0.01
HDL = (0.05, 0.014, 0.020)
COVER_Z0 = RIM + COV_T / 2  # cover centre height when seated
COVER = Box("floor_box_cover", pos=(float(WC[0]), float(WC[1]), COVER_Z0), size=(COV_X, COV_Y, COV_T),
            mass=COVER_MASS, color=COVER_GREY)
HANDLE_DZ = COV_T / 2 + HDL[2] / 2
HANDLE = Box("cover_handle", pos=(float(WC[0]), float(WC[1]), COVER_Z0 + HANDLE_DZ), size=HDL, mass=HANDLE_MASS,
             color=(0.40, 0.42, 0.45))

# -- service box and power cord ------------------------------------------------------------------------------------
R = 0.003  # 6 mm cord
CORD_LEN = 0.42
PLUG_Z = TOP + PH / 2 + 0.0005  # cord axis height at spawn (the plug stands on the table)
ANCHOR = (0.554, -0.17)
SERVICE = Box("service_box", pos=(0.59, ANCHOR[1], TOP + 0.02), size=(0.06, 0.07, 0.04), color=(0.45, 0.46, 0.48))
CORD = cable("cord", start=(ANCHOR[0], ANCHOR[1], PLUG_Z), yaw=math.pi, length=CORD_LEN, radius=R,
             anchor_start=True, end_plug=Plug(size=(PL, PW, PH), mass=0.02, color=(0.08, 0.08, 0.09)), color=BLACK,
             bend_stiffness=0.01, bend_damping=0.001)

OBJECTS = [*_well(), *_socket(), COVER, HANDLE, SERVICE, CORD]


# -- cover/handle weld ---------------------------------------------------------------------------------------------
# yam_arena objects are single boxes, so the handle is a second free box tied to the cover by a fixed joint, authored
# right after the cable (the only kit spawn hook). The joint touches no cable body and is kept out of the articulation.


def _install_handle_weld() -> None:
    orig = cablekit._spawn_cable
    if getattr(orig, "_handle_weld", False):
        return

    def spawn(prim_path, cfg, *a, **k):
        prim = orig(prim_path, cfg, *a, **k)
        if getattr(cfg, "cable_name", "") != CORD.name:
            return prim
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Gf, Sdf, UsdPhysics

        stage = get_current_stage()
        parent = str(prim_path).rsplit("/", 1)[0]
        j = UsdPhysics.FixedJoint.Define(stage, f"{prim_path}/handle_weld")
        j.CreateBody0Rel().SetTargets([Sdf.Path(f"{parent}/{COVER.name}")])
        j.CreateBody1Rel().SetTargets([Sdf.Path(f"{parent}/{HANDLE.name}")])
        j.CreateLocalPos0Attr(Gf.Vec3f(0.0, 0.0, float(HANDLE_DZ)))
        j.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        j.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
        j.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        j.CreateExcludeFromArticulationAttr(True)
        return prim

    spawn._handle_weld = True
    cablekit._spawn_cable = spawn


_install_handle_weld()

# -- scoring -------------------------------------------------------------------------------------------------------
SEAT_MIN = 0.015  # plug bottom at least 15 mm below the socket's top edge
LIFT_MAX = 0.003  # every corner of the cover's underside within 3 mm of the rim
CENTRE_TOL = 0.010  # cover centre within 1 cm (horizontally) of the well centre
RELEASE_DIST = 0.035  # both grippers' TCP at least 3.5 cm from the plug, every cord point, the cover and its handle
BOX_X0, BOX_X1 = X_IN0 - WT, X_IN1 + WT  # box outer footprint (x 0.17 .. 0.33, y -0.07 .. 0.07)
BOX_Y0, BOX_Y1 = Y_IN0 - WT, Y_IN1 + WT

_ENV: dict = {"env": None}


def _install_hooks() -> None:
    """Keep a handle on the env so score() can see the grippers."""
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # noqa: BLE001  (no embodiment: offline use)
        return
    cls = ie.YamArenaEmbodiment
    if not getattr(cls._task_step, "_floorbox_hooked", False):
        base = cls._task_step

        def _task_step(self, *a, **k):
            _ENV["env"] = self._env
            return base(self, *a, **k)

        _task_step._floorbox_hooked = True
        cls._task_step = _task_step


_install_hooks()


def _rot(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def cover_corner_lift(objects) -> float:
    """Largest |height of a cover underside corner above the rim| (m)."""
    c = objects[COVER.name]
    if not np.isfinite(c.pos).all():
        return 1.0
    Rm = _rot(c.quat_xyzw)
    lifts = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            p = c.pos + Rm @ np.array([sx * COV_X / 2, sy * COV_Y / 2, -COV_T / 2])
            lifts.append(abs(p[2] - RIM))
    return float(max(lifts))


def cover_tilt_deg(objects) -> float:
    Rm = _rot(objects[COVER.name].quat_xyzw)
    return math.degrees(math.acos(max(-1.0, min(1.0, Rm[2, 2]))))


def cover_shift(objects) -> float:
    return float(np.linalg.norm(objects[COVER.name].pos[:2] - WC))


def cover_closed(objects) -> bool:
    return cover_corner_lift(objects) <= LIFT_MAX and cover_shift(objects) <= CENTRE_TOL


def cover_off_box(objects) -> bool:
    """No part of the cover or its handle is above the box's outer footprint (the box left open, cover set aside)."""
    c, h = objects[COVER.name], objects[HANDLE.name]
    lo = np.minimum(c.low, h.low)
    hi = np.maximum(c.high, h.high)
    if not (np.isfinite(lo).all() and np.isfinite(hi).all()):
        return False
    return bool(hi[0] < BOX_X0 or lo[0] > BOX_X1 or hi[1] < BOX_Y0 or lo[1] > BOX_Y1)


def plug_depth(objects) -> float:
    """How far the plug's bottom is below the socket's top edge (m); -1 if the plug is not in the socket opening."""
    p = objects[CORD.name].end_plug
    if p is None or not np.isfinite(p).all():
        return -1.0
    if abs(p[0] - SX) > (CL - PL) / 2 + 0.002 or abs(p[1] - SY) > (CW - PW) / 2 + 0.002:
        return -1.0
    return float(SOCK_TOP - (p[2] - PH / 2))


def cord_in_notch(objects) -> bool:
    """Some part of the cord lies inside the notch opening (between its sides, through the wall, below the rim)."""
    pts = _dense(objects[CORD.name].polyline())
    x0, x1 = X_IN1 - 0.003, X_IN1 + WT + 0.003
    ok = ((pts[:, 0] >= x0) & (pts[:, 0] <= x1) & (np.abs(pts[:, 1] - NOTCH_Y) <= NOTCH_W / 2 + 0.002)
          & (pts[:, 2] <= RIM))
    return bool(ok.any())


def _dense(poly: np.ndarray, step: float = 0.004) -> np.ndarray:
    if not np.isfinite(poly).all():
        return np.asarray(poly)
    out = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        n = max(1, int(np.ceil(np.linalg.norm(b - a) / step)))
        out += [a + (b - a) * (k / n) for k in range(1, n + 1)]
    return np.asarray(out)


def grippers() -> list[np.ndarray]:
    """World positions of both grippers' TCP (13 mm above the finger pad tips); [] if unknown (offline)."""
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


def _box_dist(p, st) -> float:
    d = np.maximum(np.maximum(st.low - p, p - st.high), 0.0)
    return float(np.linalg.norm(d))


def released(objects) -> bool:
    cord = objects[CORD.name]
    pts = _dense(cord.polyline())
    plug = cord.end_plug
    for g in grippers():
        if float(np.min(np.linalg.norm(pts - g, axis=1))) < RELEASE_DIST:
            return False
        if plug is not None and float(np.linalg.norm(np.asarray(plug) - g)) < RELEASE_DIST:
            return False
        if min(_box_dist(g, objects[COVER.name]), _box_dist(g, objects[HANDLE.name])) < RELEASE_DIST:
            return False
    return True


def score(objects) -> float:
    """Gates (score 0): a gripper within 3.5 cm of the cord, plug, cover or handle; the plug less than 15 mm deep in
    the socket; the cover anywhere over the box without being closed flat (every underside corner within 3 mm of the
    rim, centre within 1 cm of the box centre) with the cord through the notch, i.e. a pinched or askew cover.
    Ladder: 0.4 plug seated, cover set aside off the box, cord not through the notch; 0.6 plug seated, cord through
    the notch, cover set aside off the box; 1.0 plug seated, cord through the notch, cover closed flat."""
    cord = objects[CORD.name]
    if not (np.isfinite(cord.polyline()).all() and np.isfinite(objects[COVER.name].pos).all()):
        return 0.0  # simulation blew up: no credit
    if not released(objects) or plug_depth(objects) < SEAT_MIN:
        return 0.0
    notch = cord_in_notch(objects)
    if cover_off_box(objects):
        return 0.6 if notch else 0.4  # box left open: unfinished, but nothing pinched
    if cover_closed(objects) and notch:
        return 1.0
    return 0.0  # cover on the box but rocking on the cord, askew or shifted: pinched cord / box not closed


def success(objects) -> bool:
    return bool(score(objects) == 1.0 and objects[COVER.name].resting() and
                np.linalg.norm(objects[CORD.name].lin_vel) < 0.02)


def distance(objects) -> float:
    """How far (m) the plug still is from 15 mm deep in the socket."""
    p = objects[CORD.name].end_plug
    if p is None:
        return 1.0
    return float(np.linalg.norm(np.asarray(p) - np.array([SX, SY, SOCK_TOP - SEAT_MIN + PH / 2])))


# -- closed-loop scripted demo (cabledemo.py) ----------------------------------------------------------------------
# Left arm: pinch the cover's handle, lift the cover straight up and hold it out of the way on the left.
# Right arm (in parallel): pick the plug up by its top, wait above the box until it is open, lower the plug into the
# socket in line with the opening (closed loop on the plug's measured position), let go; then pick the cord up outside
# the box and lay it into the notch from outside, so the cord runs plug -> slot -> notch -> service box; move clear.
# Left arm: bring the cover back over the well, lower it until it is just above the rim, open, move clear.

FINGER_PINCH = -0.020  # pads ~44 mm apart
FINGER_CORD = -0.014  # pads ~28 mm apart: round a 6 mm cord lying on the table
HI = TOP + 0.13


def _crossing(st):
    """Where the cord crosses the far wall's mid-plane (y, z above the table), or None."""
    poly = st[CORD.name].polyline()
    if not np.isfinite(poly).all():
        return "NaN"
    pts = _dense(poly, 0.002)
    xf = X_IN1 + WT / 2
    i = int(np.argmin(np.abs(pts[:, 0] - xf)))
    return np.round(pts[i, [1, 2]] - [0.0, TOP], 4) if abs(pts[i, 0] - xf) < 0.004 else None


def _log(ctx, tag=""):
    st = ctx.states()
    print(f"    crossing(y, z above table)={_crossing(st)}", flush=True)
    c = st[COVER.name]
    print(f"[t={ctx.t:5.2f}] {tag} depth={plug_depth(st) * 1000:.1f}mm plug={np.round(st[CORD.name].end_plug, 4)} "
          f"notch={cord_in_notch(st)} cover lift={cover_corner_lift(st) * 1000:.1f}mm tilt={cover_tilt_deg(st):.2f} "
          f"shift={cover_shift(st) * 1000:.1f}mm z={c.pos[2] - TOP:.4f} off_box={cover_off_box(st)} rel={released(st)} "
          f"score={score(st)}",
          flush=True)


class _Flags:
    open = False
    plugged = False


def _lift_cover(ctx, flags, hold):
    from yam_arena.constants import FINGER_CLOSED

    L = ctx.left
    h = ctx.states()[HANDLE.name].pos
    g = np.array([h[0], h[1], h[2] + HDL[2] / 2 + 0.003])
    yield from L.move((g[0], g[1], g[2] + 0.04), yaw=0.0, finger=FINGER_PINCH, duration=1.5)
    yield from L.move(g, yaw=0.0, duration=1.0)
    yield from L.grip(FINGER_CLOSED, 0.6)
    yield from L.move((g[0], g[1], HI + 0.01), yaw=0.0, duration=1.2)
    yield from L.move(hold, yaw=0.0, duration=1.5)
    flags.open = True
    _log(ctx, "cover lifted")
    return g


def _close_cover(ctx, hold, let_go=True):
    """Bring the cover back over the well (closed loop on its measured offset in the grip), set it down, let go."""
    L = ctx.left
    above = np.array([WC[0], WC[1], HI + 0.01])
    yield from L.move(above, yaw=0.0, duration=1.5)
    yield from L.wait(0.3)
    c = ctx.states()[COVER.name].pos
    tcp = above.copy()
    tcp[:2] += WC - c[:2]
    dz = c[2] - COVER_Z0  # cover height above its seat at ``above``
    yield from L.move(tcp, yaw=0.0, duration=0.6)
    yield from L.move((tcp[0], tcp[1], tcp[2] - dz + 0.004), yaw=0.0, duration=2.0)
    yield from L.wait(0.3)
    if not let_go:  # shortcut "hold": keep pinching the handle to the end
        _log(ctx, "cover closed, still held")
        return
    yield from L.grip(FINGER_PINCH, 0.6)
    yield from L.move((tcp[0], tcp[1], HI + 0.02), yaw=0.0, duration=1.0)
    _log(ctx, "cover closed")
    del hold


def _pick_plug(ctx, wait_pos):
    from yam_arena.constants import FINGER_CLOSED

    Rt = ctx.right
    p = ctx.states()[CORD.name].end_plug
    g = np.array([p[0], p[1], p[2] + PH / 2 + 0.003])
    yield from Rt.move((g[0], g[1], g[2] + 0.05), yaw=0.0, finger=FINGER_PINCH, duration=1.5)
    yield from Rt.move(g, yaw=0.0, duration=1.0)
    yield from Rt.grip(FINGER_CLOSED, 0.6)
    yield from Rt.move((g[0], g[1], HI), yaw=0.0, duration=1.2)
    yield from Rt.move(wait_pos, yaw=0.0, duration=1.2)


def _insert_plug(ctx):
    """Plug hanging in the grip above the socket: centre it on the opening, lower it to the bottom, let go."""
    Rt = ctx.right
    tcp = np.array([SX, SY, HI])
    yield from Rt.move(tcp, yaw=0.0, duration=1.5)
    yield from Rt.wait(0.4)
    for z_bot in (SOCK_TOP + 0.020, SOCK_TOP + 0.004):
        p = ctx.states()[CORD.name].end_plug
        tcp[:2] += np.array([SX, SY]) - p[:2]
        off = tcp[2] - (p[2] - PH / 2)  # TCP height above the plug's bottom
        tcp[2] = z_bot + off
        yield from Rt.move(tcp, yaw=0.0, duration=1.2)
        yield from Rt.wait(0.3)
    p = ctx.states()[CORD.name].end_plug
    tcp[:2] += np.array([SX, SY]) - p[:2]
    off = tcp[2] - (p[2] - PH / 2)
    yield from Rt.move((tcp[0], tcp[1], TOP + off - 0.002), yaw=0.0, duration=1.5)
    yield from Rt.wait(0.3)
    _log(ctx, "plug pushed")
    yield from Rt.grip(FINGER_PINCH, 0.6)
    yield from Rt.move((tcp[0], tcp[1], HI), yaw=0.0, duration=1.2)


def _press_plug(ctx):
    """Closed fingers straight down on the plug's top: push it to the bottom of the socket, then lift away."""
    from yam_arena.constants import FINGER_CLOSED

    Rt = ctx.right
    p = ctx.states()[CORD.name].end_plug
    top = p[2] + PH / 2
    yield from Rt.move((p[0], p[1], top + 0.05), yaw=0.0, finger=FINGER_CLOSED, duration=1.2)
    yield from Rt.move((p[0], p[1], TOP + PH + 0.013 - 0.003), yaw=0.0, duration=1.2)
    yield from Rt.wait(0.3)
    _log(ctx, "plug pressed")
    yield from Rt.move((p[0], p[1], top + 0.06), yaw=0.0, duration=1.0)


def _grip_yaw(d) -> float:
    a = math.atan2(d[1], d[0])
    while a > math.pi / 2:
        a -= math.pi
    while a < -math.pi / 2:
        a += math.pi
    return a


def _outside_box(p, margin) -> bool:
    return bool(p[0] > X_IN1 + WT + margin or p[0] < X_IN0 - WT - margin or p[1] > Y_IN1 + WT + margin
                or p[1] < Y_IN0 - WT - margin)


def _lay_cord(ctx, target, *, s_from_plug=0.10, yaw_out=math.pi / 2, carry_z=TOP + 0.065, set_z=TOP + 0.033):
    """Pick the cord up just outside the box about ``s_from_plug`` from the plug, carry it low to ``target`` (x, y)
    right outside a wall and set it down there, so the cord between the socket and the fingers crosses that wall at
    ``target``'s x (for the real task: in the notch); let go and lift away."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    Rt = ctx.right
    poly = ctx.states()[CORD.name].polyline()[::-1]  # from the plug end
    s = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(poly, axis=0), axis=1))]
    k = int(np.searchsorted(s, s_from_plug))
    k = min(max(k, 1), len(poly) - 2)
    while k < len(poly) - 2 and not _outside_box(poly[k], 0.012):  # grasp on the table, clear of the walls
        k += 1
    p = poly[k]
    if target[0] is None:  # set the cord down just far enough out that the stretch from the socket is straight
        target = (min(0.405, SX + CL / 2 + ST + s[k] - 0.027), target[1])
    yaw = _grip_yaw(poly[k + 1] - poly[k - 1])
    print(f"[demo] cord grasp at s={s[k]:.3f} p={np.round(p, 4)} yaw={yaw:.2f} -> {np.round(target, 3)}", flush=True)
    yield from Rt.move((p[0], p[1], HI), yaw=yaw, finger=FINGER_CORD, duration=1.2)
    yield from Rt.move((p[0], p[1], p[2] + 0.010), yaw=yaw, duration=1.2)
    yield from Rt.grip(FINGER_CLOSED, 0.5)
    yield from Rt.move((p[0], p[1], carry_z), yaw=yaw, duration=1.0)
    yield from Rt.move((target[0], target[1], carry_z), yaw=yaw_out, duration=1.8)
    yield from Rt.move((target[0], target[1], set_z), yaw=yaw_out, duration=1.2)
    yield from Rt.wait(0.3)
    _log(ctx, "cord set")
    yield from Rt.grip(FINGER_OPEN, 0.5)
    yield from Rt.move((target[0], target[1], TOP + 0.10), yaw=yaw_out, duration=1.0)
    _log(ctx, "cord laid")


def demo_script(ctx):
    from yam_arena.constants import FINGER_CLOSED

    # test only: correct (1.0) | tier04 (0.4) | tier06 (0.6) | nothing | hold | lazy | pinch (shortcuts, 0)
    mode = os.environ.get("FLOORBOX_DEMO", "correct")
    _log(ctx, "start")
    if mode == "nothing":
        for _ in range(6):
            yield from ctx.left.wait(1.0)
            _log(ctx)
        return
    flags = _Flags()
    hold = np.array([0.26, 0.22, HI + 0.01])
    wait_pos = np.array([0.20, -0.12, HI])

    def right():
        yield from _pick_plug(ctx, wait_pos)
        while not flags.open:
            yield
        yield from _insert_plug(ctx)
        if mode == "pinch":  # cheat: cord led straight out over the far wall's rim, cover closed on it
            yield from _lay_cord(ctx, (X_IN1 + WT + 0.015, -0.03), yaw_out=0.0)
        elif mode in ("lazy", "tier04"):  # plug in without leading the cord anywhere (it crosses the far rim)
            pass
        else:
            yield from _lay_cord(ctx, (None, NOTCH_Y + 0.016), yaw_out=0.0, set_z=TOP + 0.013)
        yield from _press_plug(ctx)
        yield from ctx.right.move((0.18, -0.24, HI + 0.02), yaw=0.0, finger=FINGER_CLOSED, duration=1.2)
        flags.plugged = True

    def left():
        yield from _lift_cover(ctx, flags, hold)
        while not flags.plugged:
            yield
        if mode in ("tier04", "tier06"):  # partial: plug in (and route), but leave the cover set aside on the table
            yield from ctx.left.move((hold[0], hold[1], TOP + 0.074 + 0.004), yaw=0.0, duration=2.0)
            yield from ctx.left.grip(FINGER_PINCH, 0.6)
            yield from ctx.left.move((hold[0], hold[1], HI + 0.02), yaw=0.0, duration=1.0)
        elif mode == "hold":  # cheat: everything right, but the cover's handle is still pinched at the end
            yield from _close_cover(ctx, hold, let_go=False)
            return
        else:
            yield from _close_cover(ctx, hold)
        yield from ctx.left.move((0.18, 0.24, HI + 0.02), yaw=0.0, finger=FINGER_CLOSED, duration=1.2)

    yield from ctx.both(left(), right())
    yield from ctx.left.wait(1.0)
    _log(ctx, "end")
