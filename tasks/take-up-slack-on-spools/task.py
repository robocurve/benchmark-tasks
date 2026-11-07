"""On the station table a patch cord's excess length has to be stored on two slack spools before it is plugged in.

Inside a taped rectangle (yellow tape, inner 25 x 33.5 cm) on the left half of the table stand two light-grey slack
spools (60 mm hub, 4 cm tall, with a 76 mm flange on top), 12 cm apart along the robot's left-right axis, 31 cm in
front of the arm bases. A grey device block on the rectangle's left edge (nearer its near end) holds the fixed end of a blue 6 mm
patch cord (84 cm), which lies straight on the table towards the robot's right, across the whole table, ending in a
blue plug (45 x 16 x 14 mm) far out on the right. A dark-grey socket block stands just outside the rectangle's near-right corner; its
port (20 x 16.5 mm, 26 mm deep) opens towards the robot's left (+y). The cord is far longer than the direct path from
the device to the port: the robot must wind the excess once round the pair of spools (the cord lying against both
hubs), push the plug into the port and let go, with the whole cord inside the tape. Neither arm can do it alone:
only the right arm reaches the plug at the start, only the left arm reaches round the spools and the port.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score \\
           --instruction "<INSTRUCTION below>"
Test-only demo variants (env var SLACK_CHEAT): nothing | direct | pile | noplug | hold.
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
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = ("The blue patch cord is too long. Take up its extra length by winding it once around the two grey "
               "slack spools, so the cord lies against both spools, then push its plug fully into the port of the "
               "dark socket. Keep the whole cord inside the yellow tape rectangle, then let go.")

# -- slack spools ------------------------------------------------------------------------------------------------
SPOOLS = {"s1": np.array([0.31, 0.22]), "s2": np.array([0.31, 0.10])}  # s1 = left (+y) spool, s2 = right
HUB_R, HUB_H = 0.030, 0.040  # 60 mm hub, 4 cm tall
FL_R, FL_T = 0.038, 0.006  # 76 mm flange on top keeps the turns on the hub
SPOOL_GREY = (0.80, 0.81, 0.83)
SPOOL_OBJS = []
for _k, _c in SPOOLS.items():
    SPOOL_OBJS += [
        Cylinder(f"spool_{_k}_hub", pos=(float(_c[0]), float(_c[1]), TOP + HUB_H / 2), radius=HUB_R, height=HUB_H,
                 color=SPOOL_GREY),
        Cylinder(f"spool_{_k}_flange", pos=(float(_c[0]), float(_c[1]), TOP + HUB_H + FL_T / 2), radius=FL_R,
                 height=FL_T, color=SPOOL_GREY),
    ]

# -- socket: port opening faces +y (towards the robot's left); the plug goes in moving -y -------------------------
PL, PW, PH = 0.045, 0.016, 0.014  # plug: length along the cord, width, height
CAV_W, CAV_H, CAV_D = PW + 0.004, PH + 0.0025, 0.026  # port opening (x, z) and depth (y)
PORT_X, FACE_Y = 0.150, 0.0  # port axis x, socket front face y
SOCK_W, SOCK_D, SOCK_H = 0.060, 0.050, 0.035
DARK = (0.16, 0.17, 0.18)
_side = (SOCK_W - CAV_W) / 2
SOCKET = [
    Box("socket_back", pos=(PORT_X, FACE_Y - CAV_D - (SOCK_D - CAV_D) / 2, TOP + SOCK_H / 2),
        size=(SOCK_W, SOCK_D - CAV_D, SOCK_H), color=DARK),
    Box("socket_roof", pos=(PORT_X, FACE_Y - CAV_D / 2, TOP + (CAV_H + SOCK_H) / 2), size=(CAV_W, CAV_D, SOCK_H - CAV_H),
        color=DARK),
    Box("socket_wall_n", pos=(PORT_X - CAV_W / 2 - _side / 2, FACE_Y - CAV_D / 2, TOP + SOCK_H / 2),
        size=(_side, CAV_D, SOCK_H), color=DARK),
    Box("socket_wall_f", pos=(PORT_X + CAV_W / 2 + _side / 2, FACE_Y - CAV_D / 2, TOP + SOCK_H / 2),
        size=(_side, CAV_D, SOCK_H), color=DARK),
]

# -- device block with the cord's fixed end, and the cord ----------------------------------------------------------
R = 0.003  # 6 mm patch cord
CORD_X, CORD_Y0 = 0.22, 0.326  # the cord leaves the device's right (-y) face here, 6 mm in front of it
CORD_LEN = 0.84
DEV_SIZE = (0.060, 0.050, 0.040)
DEVICE = Box("device", pos=(CORD_X, CORD_Y0 + 0.006 + DEV_SIZE[1] / 2, TOP + DEV_SIZE[2] / 2), size=DEV_SIZE,
             color=(0.45, 0.47, 0.50))
BLUE = (0.10, 0.30, 0.85)
PLUG = Plug(size=(PL, PW, PH), mass=0.015, color=(0.07, 0.22, 0.70))
CORD = cable("cord", start=(CORD_X, CORD_Y0, TOP + PH / 2 + 0.0005), yaw=-math.pi / 2, length=CORD_LEN, radius=R,
             anchor_start=True, end_plug=PLUG, color=BLUE, bend_stiffness=0.01, bend_damping=0.001)

# -- tape rectangle (inner edges); 1 cm yellow tape, 1 mm thick, kept out of the port opening ------------------------
RX0, RX1, RY0, RY1 = 0.12, 0.37, 0.0, 0.335
TW, TT = 0.010, 0.001
YELLOW = (0.95, 0.80, 0.10)
_sx0, _sx1 = PORT_X - SOCK_W / 2, PORT_X + SOCK_W / 2
TAPE = [
    Box("tape_far", pos=(RX1 + TW / 2, (RY0 + RY1) / 2, TOP + TT / 2), size=(TW, RY1 - RY0 + 2 * TW, TT), color=YELLOW),
    Box("tape_near", pos=(RX0 - TW / 2, (RY0 + RY1) / 2, TOP + TT / 2), size=(TW, RY1 - RY0 + 2 * TW, TT),
        color=YELLOW),
    Box("tape_left", pos=((RX0 + RX1) / 2, RY1 + TW / 2, TOP + TT / 2), size=(RX1 - RX0, TW, TT), color=YELLOW),
    Box("tape_right_a", pos=((_sx1 + RX1) / 2, RY0 - TW / 2, TOP + TT / 2), size=(RX1 - _sx1, TW, TT), color=YELLOW),
]

OBJECTS = [*SPOOL_OBJS, *SOCKET, DEVICE, CORD, *TAPE]

# -- grippers (score requires both to have let go) ----------------------------------------------------------------
RELEASE_DIST = 0.03
_ENV: dict = {"env": None}


def _install_env_hook() -> None:
    """Keep a handle on the sim env so score() can check that the grippers have let go."""
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls._task_step, "_slack_hooked", False):
        return
    base = cls._task_step

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        return base(self, *a, **k)

    _task_step._slack_hooked = True
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


# -- scoring geometry ------------------------------------------------------------------------------------------------
SEATED_MIN = 0.015  # full credit: plug tip >= 15 mm inside the port
ENTER = 0.005  # half credit: tip 5-15 mm inside
IN_RECT_MIN = 0.95  # fraction of cord length inside the tape rectangle
CONTACT_GAP = 0.015  # cord counts as lying against a hub when its surface is within 15 mm of the hub surface
COVER_MIN = 150.0  # deg of each hub's circumference the cord must lie against
LOW_Z = TOP + HUB_H  # ... below the flange


def _dense(poly: np.ndarray, step: float = 0.003) -> np.ndarray:
    out = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        k = max(1, int(np.linalg.norm(b - a) / step))
        out.extend(a + (b - a) * (j / k) for j in range(1, k + 1))
    return np.array(out)


def _ok(objects) -> bool:
    c = objects["cord"]
    return bool(np.isfinite(c.points).all() and c.end_plug is not None and np.isfinite(c.end_plug).all())


def plug_tip(objects) -> np.ndarray:
    c = objects["cord"]
    a = np.asarray(c.end_plug, dtype=float) - np.asarray(c.points[-1], dtype=float)
    a /= np.linalg.norm(a) + 1e-12
    return np.asarray(c.end_plug, dtype=float) + a * PL / 2


def seat_depth(objects) -> float:
    """How far (m) the plug tip is inside the port (past the socket face, within the opening); -1 if not in it."""
    t = plug_tip(objects)
    if abs(t[0] - PORT_X) > CAV_W / 2 + 0.002 or t[2] > TOP + CAV_H + 0.003:
        return -1.0
    return float(FACE_Y - t[1])


def in_rect_fraction(objects) -> float:
    """Fraction of the cord's length (plug excluded) whose centre line lies inside the tape's inner edges."""
    p = _dense(objects["cord"].polyline(), 0.002)
    ins = (p[:, 0] >= RX0) & (p[:, 0] <= RX1) & (p[:, 1] >= RY0) & (p[:, 1] <= RY1)
    return float(ins.mean())


def coverage(objects, k: str) -> float:
    """Degrees of spool ``k``'s hub circumference the cord lies against (top view): union of the angles swept,
    about the hub axis, between consecutive cord points both within 15 mm of the hub surface and below the flange."""
    p = _dense(objects["cord"].polyline(), 0.002)
    c = SPOOLS[k]
    gap = np.linalg.norm(p[:, :2] - c, axis=1) - HUB_R - R
    near = (gap <= CONTACT_GAP) & (p[:, 2] <= LOW_Z)
    ang = np.degrees(np.arctan2(p[:, 1] - c[1], p[:, 0] - c[0]))
    hit = np.zeros(360, dtype=bool)
    for i in range(len(p) - 1):
        if near[i] and near[i + 1]:
            a0 = ang[i]
            d = (ang[i + 1] - a0 + 180.0) % 360.0 - 180.0
            n = max(1, int(abs(d)) + 1)
            for t in np.linspace(0.0, 1.0, n + 1):
                hit[int(math.floor((a0 + t * d) % 360.0)) % 360] = True
    return float(hit.sum())


def released(objects) -> bool:
    c = objects["cord"]
    pts = np.vstack([_dense(c.polyline()), c.end_plug[None, :], plug_tip(objects)[None, :]])
    return all(float(np.linalg.norm(pts - g, axis=1).min()) >= RELEASE_DIST for g in grippers())


def wound(objects) -> bool:
    return all(coverage(objects, k) >= COVER_MIN for k in SPOOLS)


def score(objects) -> float:
    """0 unless: both grippers >= 3 cm from the cord and plug; >= 95 % of the cord inside the tape rectangle; the
    cord lies against >= 150 deg of EACH spool hub (within 15 mm, below the flange). Then 1.0 if the plug tip is
    >= 15 mm inside the port, 0.5 if 5-15 mm, else 0."""
    if not _ok(objects):
        return 0.0
    if not released(objects) or in_rect_fraction(objects) < IN_RECT_MIN or not wound(objects):
        return 0.0
    d = seat_depth(objects)
    if d >= SEATED_MIN:
        return 1.0
    if d >= ENTER:
        return 0.5
    return 0.0


def success(objects) -> bool:
    return score(objects) >= 1.0 and objects["cord"].resting(max_speed=0.02)


def distance(objects) -> float:
    """Missing hub coverage (fraction of the 2 x 150 deg needed) plus the plug tip's distance (m) to 15 mm deep."""
    if not _ok(objects):
        return 2.0
    miss = sum(max(0.0, COVER_MIN - coverage(objects, k)) for k in SPOOLS) / (2 * COVER_MIN)
    t = plug_tip(objects)
    return miss + float(np.linalg.norm(t - np.array([PORT_X, FACE_Y - SEATED_MIN, TOP + PH / 2])))


def progress(objects) -> dict:
    """Stage diagnostic for analysis (not the score)."""
    if not _ok(objects):
        return {"valid": 0}
    c = objects["cord"]
    g = grippers()
    d = seat_depth(objects)
    return {
        "plug_grasped": int(any(float(np.linalg.norm(x - c.end_plug)) < 0.03 for x in g)),
        "spool_s2_covered": round(min(1.0, coverage(objects, "s2") / COVER_MIN), 2),
        "spool_s1_covered": round(min(1.0, coverage(objects, "s1") / COVER_MIN), 2),
        "plug_entered": int(d >= ENTER),
        "plug_seated": int(d >= SEATED_MIN),
        "cord_in_rect": int(in_rect_fraction(objects) >= IN_RECT_MIN),
        "released": int(released(objects)),
    }


# Seeded reset variation for a later runner (the demo runs on the nominal layout and follows the live cord).
VARIATION = {"cable:cord": {"dx": 0.01, "dy": 0.0, "yaw": 0.03}}


# -- closed-loop scripted demo (cabledemo.py) --------------------------------------------------------------------
FINGER_PINCH = -0.020  # finger opening for the plug pinch
GZ = TOP + PH + 0.003  # TCP height when pinching a plug on the table
CARRY_Z = GZ + 0.006  # carry the plug just above the table (higher, it swings about the pinch axis)
HI = TOP + 0.08
OFF = 0.075  # carry radius round a spool axis (gripper clear of the flange)
HAND = (0.345, 0.035)  # plug centre at the hand-over spot (right of the right spool)
SEAT_TO = 0.020  # target tip depth (port is 26 mm deep)
GRASP_BACK = 0.010  # push grip: pinch this far in front of the plug's cord end
PARK = {"left": (0.10, 0.45, TOP + 0.15), "right": (0.15, -0.30, TOP + 0.15)}


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def _plug_pose(ctx):
    c = ctx.states()["cord"]
    p = np.asarray(c.end_plug, dtype=float)
    a = p - np.asarray(c.points[-1], dtype=float)
    a /= np.linalg.norm(a) + 1e-12
    return p, a


def _log(ctx, tag=""):
    st = ctx.states()
    if not _ok(st):
        print(f"[t={ctx.t:5.1f}] {tag}: NaN", flush=True)
        return
    p = st["cord"].end_plug
    print(f"[t={ctx.t:5.1f}] {tag}: plug={np.round(p, 3).tolist()} cov=({coverage(st, 's1'):.0f},"
          f"{coverage(st, 's2'):.0f}) rect={in_rect_fraction(st):.2f} depth={seat_depth(st) * 1000:.1f}mm "
          f"rel={released(st)} score={score(st):.2f}", flush=True)
    if os.environ.get("SLACK_DEBUG"):
        pc = np.round(st["cord"].points * 100).astype(int)
        print("   cord cm:", " ".join(f"{x},{y}" for x, y, _ in pc), flush=True)


def _pinch(arm, ctx, yaw, back=0.0, lift=CARRY_Z - GZ, dur=1.5):
    """Pinch the plug ``back`` m behind its centre with grip ``yaw`` and lift it ``lift``."""
    from yam_arena.constants import FINGER_CLOSED

    p, a = _plug_pose(ctx)
    g = p - a * back
    yield from arm.move((g[0], g[1], HI), yaw=yaw, finger=FINGER_PINCH, duration=dur)
    p, a = _plug_pose(ctx)
    g = p - a * back
    yield from arm.move((g[0], g[1], GZ), yaw=yaw, duration=1.0)
    yield from arm.grip(FINGER_CLOSED, 0.6)
    if lift:
        yield from arm.move((g[0], g[1], GZ + lift), yaw=yaw, duration=0.5)


def _path(arm, pts, dur=0.5):
    for x, y, yw in pts:
        yield from arm.move((x, y, CARRY_Z), yaw=yw, duration=dur)


def _set_down(arm, x, y, yaw):
    yield from arm.move((x, y, GZ), yaw=yaw, duration=0.6)
    yield from arm.grip(FINGER_PINCH, 0.5)
    yield from arm.move((x, y, HI), yaw=yaw, duration=0.6)


def _insert(arm, ctx, hold=False):
    """Plug lies flat in front of the port: pinch it near its cord end (grip yaw -90 deg: wrist camera looks
    along -y into the port), square it up, line its tip up with the port axis and push it SEAT_TO deep."""
    tgt = -math.pi / 2
    yaw = tgt
    yield from _pinch(arm, ctx, yaw, back=PL / 2 - GRASP_BACK, lift=0.0)
    gx, gy = (float(v) for v in arm.tcp()[:2])
    for _ in range(3):  # square up (plug axis along -y)
        _, a = _plug_pose(ctx)
        err = _wrap(math.atan2(a[1], a[0]) - tgt)
        if abs(err) < 0.015:
            break
        yaw = float(np.clip(yaw - err, -math.pi / 2, math.pi / 2))
        yield from arm.move((gx, gy, GZ), yaw=yaw, duration=0.5)
        yield from arm.wait(0.2)
    for _ in range(3):  # line the tip up with the port axis
        err = PORT_X - plug_tip(ctx.states())[0]
        if abs(err) < 0.0007:
            break
        gx += float(np.clip(err, -0.01, 0.01))
        yield from arm.move((gx, gy, GZ), yaw=yaw, duration=0.5)
        yield from arm.wait(0.2)
    _log(ctx, "aligned")
    for i in range(3):  # push; top up if the plug slipped back in the fingers
        st = ctx.states()
        if seat_depth(st) >= SEAT_TO - 0.002:
            break
        gy -= plug_tip(st)[1] - (FACE_Y - SEAT_TO)
        yield from arm.move((gx, gy, GZ), yaw=yaw, duration=2.0 if i == 0 else 1.0)
        yield from arm.wait(0.2)
    _log(ctx, "pushed")
    if hold:
        return
    yield from arm.grip(FINGER_PINCH, 0.6)
    yield from arm.move((gx, gy + 0.004, HI), yaw=yaw, duration=1.0)
    yield from arm.move(PARK[arm.name], yaw=0.0, duration=1.2)


def _arc(c, a0, a1, yaw_fn, r=OFF, step=20.0):
    n = max(1, int(round(abs(a1 - a0) / step)))
    return [(c[0] + r * math.cos(math.radians(a)), c[1] + r * math.sin(math.radians(a)), yaw_fn(a))
            for a in np.linspace(a0, a1, n + 1)]


def demo_script(ctx):
    """Right arm takes the plug from the far right and carries it up the right of the right spool (the slack
    trails behind it in a U that will hook round that spool) to a hand-over spot; the left arm takes it on up the
    far side of both spools, round the left spool, back down the near side and to the port (turning the plug so it
    leads); it sets it down in front of the port, re-pinches it near its cord end with the wrist camera facing the
    port, squares it up and pushes it in; the cord is drawn snug round both spools; it lets go."""
    L, Rt = ctx.left, ctx.right
    cheat = os.environ.get("SLACK_CHEAT", "")
    _log(ctx, "start")
    if cheat == "nothing":
        yield from L.wait(4.0)
        _log(ctx, "end")
        return

    # 1) right arm: plug from the far right up to the hand-over spot, turning it to point +y
    p, a = _plug_pose(ctx)
    yield from _pinch(Rt, ctx, -math.pi / 2)
    _log(ctx, "right has plug")
    yield from _path(Rt, [(0.30, -0.47, -1.2), (0.37, -0.36, -0.6), (0.38, -0.20, 0.3), (0.37, -0.05, 1.0),
                          (HAND[0], HAND[1] - 0.01, math.radians(80))], dur=1.0)
    yield from _set_down(Rt, HAND[0], HAND[1], math.radians(80))
    yield from Rt.move(PARK["right"], yaw=0.0, duration=1.2)
    _log(ctx, "at hand-over")

    if cheat in ("direct", "pile"):
        # take the plug straight to the port without winding (direct: slack left lying where it is; pile: first
        # bunch the slack into the rectangle in front of the spools)
        p, a = _plug_pose(ctx)
        yield from _pinch(L, ctx, -math.pi / 2)
        if cheat == "pile":
            yield from _path(L, [(0.30, 0.04, -1.2), (0.24, 0.06, -0.6), (0.20, 0.14, 0.4), (0.25, 0.20, 1.2),
                                 (0.20, 0.25, 1.0), (0.16, 0.18, 1.0), (0.16, 0.10, 1.0)], dur=1.0)
        else:
            yield from _path(L, [(0.30, 0.06, 1.0), (0.22, 0.08, 1.0)], dur=1.0)
    else:
        # 2) left arm: up the far side, round the left spool (turning the plug to lead), down the near side
        yield from _pinch(L, ctx, -math.pi / 2)
        _log(ctx, "left has plug")
        s1, s2 = SPOOLS["s1"], SPOOLS["s2"]
        far_x = s1[0] + OFF
        yield from _path(L, [(far_x, s2[1] - 0.03, -math.pi / 2), (far_x, s2[1] + 0.04, -math.pi / 2),
                             (far_x, s1[1], -math.pi / 2)], dur=1.0)
        _log(ctx, "far side")
        yield from _path(L, _arc(s1, 0.0, 180.0, lambda d: math.radians(min(-90.0 + d, 75.0)))[1:], dur=0.5)
        _log(ctx, "round s1")
        near_x = s1[0] - OFF
        yield from _path(L, [(near_x, s1[1] - 0.05, math.radians(75)), (near_x, s2[1], math.radians(60)),
                             (0.22, 0.07, math.radians(60))], dur=1.0)
        _log(ctx, "near side")
    if cheat == "noplug":  # wound, but the plug is only laid down short of the port
        yield from _set_down(L, PORT_X, FACE_Y + 0.06 + PL / 2, math.radians(60))
        yield from L.move(PARK["left"], yaw=0.0, duration=1.2)
        yield from L.wait(2.0)
        _log(ctx, "end")
        return
    # 3) set the plug down in front of the port, then insert
    yield from _path(L, [(PORT_X + 0.01, FACE_Y + 0.012 + PL / 2 + 0.02, math.radians(60))], dur=1.0)
    yield from _set_down(L, PORT_X, FACE_Y + 0.012 + PL / 2, math.radians(60))
    _log(ctx, "set down at port")
    yield from _insert(L, ctx, hold=(cheat == "hold"))
    _log(ctx, "inserted")
    yield from L.wait(2.0)
    _log(ctx, "end")
