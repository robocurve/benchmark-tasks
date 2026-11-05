"""On the station table, a white 3D-printed PLA wiring trough runs left-right in front of the robot: 30 cm long
inside, 28 mm wide inside and 60 mm deep, with 6 mm walls, open at the top. Six small cross ribs (8 mm tall, 8 mm
wide, 44 mm apart) stand on the trough floor and hold wires 8 mm off it. A loose yellow wire offcut (6 mm, 24 cm,
PVC) lies straight on the ribs, centred along the trough, its axis 11 mm from the near inner wall. The gripper
fingers widen from 8 mm at their tips to 36 mm 5 cm above them: closed, they stop just above the wire and cannot
grasp it; opened, they jam higher up. On the right a black PLA tool holder holds an upright hook pick: a 10 x 10 x 100 mm PLA shank with a 14 x 6 x 4 mm foot at its bottom end. On the left is an open
grey tray (20 x 22 cm inside). The robot must hook the wire with the pick, lift it out of the trough and lay the
whole wire in the tray, then let go.

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
from cablekit import cable  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = ("A loose wire offcut has fallen into the deep wiring trough, where the grippers do not fit. Use the "
               "hook pick to lift the wire out of the trough, then lay the whole wire in the grey tray. Leave nothing in the "
               "grippers.")

# -- trough (fixed, runs along y; x = forward) -------------------------------------------------------------------------
TR_X0, TR_X1 = 0.276, 0.304  # inner faces (28 mm)
TR_Y0, TR_Y1 = -0.150, 0.150  # inner end faces (30 cm)
TR_H = 0.060  # wall height = depth
TR_T = 0.006  # wall thickness
RIM_Z = TOP + TR_H
_WHITE = (0.92, 0.92, 0.90)
TROUGH = [
    Box("trough_near_wall", pos=(TR_X0 - TR_T / 2, 0.0, TOP + TR_H / 2), size=(TR_T, TR_Y1 - TR_Y0 + 2 * TR_T, TR_H),
        color=_WHITE),
    Box("trough_far_wall", pos=(TR_X1 + TR_T / 2, 0.0, TOP + TR_H / 2), size=(TR_T, TR_Y1 - TR_Y0 + 2 * TR_T, TR_H),
        color=_WHITE),
    Box("trough_right_end", pos=((TR_X0 + TR_X1) / 2, TR_Y0 - TR_T / 2, TOP + TR_H / 2),
        size=(TR_X1 - TR_X0, TR_T, TR_H), color=_WHITE),
    Box("trough_left_end", pos=((TR_X0 + TR_X1) / 2, TR_Y1 + TR_T / 2, TOP + TR_H / 2),
        size=(TR_X1 - TR_X0, TR_T, TR_H), color=_WHITE),
]
RIB_H, RIB_W, RIB_PITCH = 0.008, 0.008, 0.044
RIB_Y = [round(-0.019 + RIB_PITCH * k, 4) for k in range(-2, 4)]  # -0.107 .. 0.113
RIBS = [Box(f"rib_{i}", pos=((TR_X0 + TR_X1) / 2, y, TOP + RIB_H / 2), size=(TR_X1 - TR_X0, RIB_W, RIB_H),
            color=_WHITE) for i, y in enumerate(RIB_Y)]

# -- the loose wire --------------------------------------------------------------------------------------------------------
R = 0.003  # 6 mm wire
WIRE_LEN = 0.24
WIRE_X = TR_X0 + 0.011  # centre line 11 mm from the near inner face
WIRE = cable("wire", start=(WIRE_X, -WIRE_LEN / 2, TOP + RIB_H + R + 0.0005), yaw=math.pi / 2, length=WIRE_LEN,
             radius=R, anchor_start=False, color=(0.95, 0.80, 0.05), bend_stiffness=0.01, bend_damping=0.001)

# -- hook pick (free): shank + foot, welded (see _install_pick_weld) -----------------------------------------------------
SHANK = (0.010, 0.010, 0.100)
FOOT = (0.006, 0.014, 0.004)  # x, y, z when the foot points +y (as in the holder)
PICK_XY = (0.25, -0.22)
SHANK_BOX = Box("pick", pos=(PICK_XY[0], PICK_XY[1], TOP + SHANK[2] / 2 + 0.0005), size=SHANK, mass=0.026,
                color=(0.10, 0.35, 0.85))
FOOT_OFF = SHANK[1] / 2 + FOOT[1] / 2  # foot centre from shank axis (along the foot direction)
FOOT_BOX = Box("pick_foot", pos=(PICK_XY[0], PICK_XY[1] + FOOT_OFF, TOP + FOOT[2] / 2 + 0.0005), size=FOOT,
               mass=0.003, color=(0.10, 0.35, 0.85))
FOOT_REACH = SHANK[1] / 2 + FOOT[1]  # 19 mm from the shank axis to the foot tip
_BLACK = (0.12, 0.12, 0.13)
HOLD_IN = (0.016, 0.032)  # holder pocket inside (x, y)
HOLD_H, HOLD_T = 0.030, 0.006
_hc = (PICK_XY[0], PICK_XY[1] + (FOOT_REACH - SHANK[1] / 2) / 2)  # pocket centre
HOLDER = [
    Box("holder_near", pos=(_hc[0] - HOLD_IN[0] / 2 - HOLD_T / 2, _hc[1], TOP + HOLD_H / 2),
        size=(HOLD_T, HOLD_IN[1] + 2 * HOLD_T, HOLD_H), color=_BLACK),
    Box("holder_far", pos=(_hc[0] + HOLD_IN[0] / 2 + HOLD_T / 2, _hc[1], TOP + HOLD_H / 2),
        size=(HOLD_T, HOLD_IN[1] + 2 * HOLD_T, HOLD_H), color=_BLACK),
    Box("holder_right", pos=(_hc[0], _hc[1] - HOLD_IN[1] / 2 - HOLD_T / 2, TOP + HOLD_H / 2),
        size=(HOLD_IN[0], HOLD_T, HOLD_H), color=_BLACK),
    Box("holder_left", pos=(_hc[0], _hc[1] + HOLD_IN[1] / 2 + HOLD_T / 2, TOP + HOLD_H / 2),
        size=(HOLD_IN[0], HOLD_T, HOLD_H), color=_BLACK),
]

# -- tray (fixed, left) ----------------------------------------------------------------------------------------------------
TRAY_C = (0.27, 0.285)
TRAY_IN = (0.20, 0.22)
TRAY_H, TRAY_T = 0.015, 0.006
_GREY = (0.45, 0.47, 0.50)
TRAY = [
    Box("tray_floor", pos=(TRAY_C[0], TRAY_C[1], TOP + 0.001), size=(TRAY_IN[0], TRAY_IN[1], 0.002), color=_GREY),
    Box("tray_near", pos=(TRAY_C[0] - TRAY_IN[0] / 2 - TRAY_T / 2, TRAY_C[1], TOP + TRAY_H / 2),
        size=(TRAY_T, TRAY_IN[1] + 2 * TRAY_T, TRAY_H), color=_GREY),
    Box("tray_far", pos=(TRAY_C[0] + TRAY_IN[0] / 2 + TRAY_T / 2, TRAY_C[1], TOP + TRAY_H / 2),
        size=(TRAY_T, TRAY_IN[1] + 2 * TRAY_T, TRAY_H), color=_GREY),
    Box("tray_right", pos=(TRAY_C[0], TRAY_C[1] - TRAY_IN[1] / 2 - TRAY_T / 2, TOP + TRAY_H / 2),
        size=(TRAY_IN[0], TRAY_T, TRAY_H), color=_GREY),
    Box("tray_left", pos=(TRAY_C[0], TRAY_C[1] + TRAY_IN[1] / 2 + TRAY_T / 2, TOP + TRAY_H / 2),
        size=(TRAY_IN[0], TRAY_T, TRAY_H), color=_GREY),
]

OBJECTS = [*TROUGH, *RIBS, WIRE, SHANK_BOX, FOOT_BOX, *HOLDER, *TRAY]

TWIN_EXTRA = [
    {"name": "pick", "kind": "body", "size_mm": [10, 33, 100], "colour": "blue",
     "pose": [PICK_XY[0], PICK_XY[1], 0.05],
     "note": "one printed PLA part: 10x10x100 mm shank with a 14x6x4 mm foot at the bottom (L shape), ~30 g"},
]


# -- the pick weld: foot fixed to the shank ----------------------------------------------------------------------------
def _install_pick_weld() -> None:
    orig = cablekit._spawn_cable
    if getattr(orig, "_pick_weld", False):
        return

    def spawn(prim_path, cfg, *a, **k):
        prim = orig(prim_path, cfg, *a, **k)
        if getattr(cfg, "cable_name", "") != WIRE.name:
            return prim
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Gf, Sdf, UsdPhysics

        stage = get_current_stage()
        root = str(prim_path).rsplit("/", 1)[0]
        j = UsdPhysics.FixedJoint.Define(stage, f"{prim_path}/pick_weld")
        j.CreateBody0Rel().SetTargets([Sdf.Path(f"{root}/{SHANK_BOX.name}")])
        j.CreateBody1Rel().SetTargets([Sdf.Path(f"{root}/{FOOT_BOX.name}")])
        off = np.asarray(FOOT_BOX.pos) - np.asarray(SHANK_BOX.pos)
        j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, off)))
        j.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        j.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
        j.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        j.CreateExcludeFromArticulationAttr(True)
        return prim

    spawn._pick_weld = True
    cablekit._spawn_cable = spawn


_install_pick_weld()

# -- grippers (for the release check) -----------------------------------------------------------------------------------
_ENV: dict = {"env": None}


def _install_hooks() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls._task_step, "_trough_hooked", False):
        return
    base = cls._task_step

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        return base(self, *a, **k)

    _task_step._trough_hooked = True
    cls._task_step = _task_step


_install_hooks()


def tcps() -> list[np.ndarray]:
    env = _ENV["env"]
    if env is None:
        return []
    from yam_arena.runtime import tcp_world

    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy()
    return [np.asarray(tcp_world(base.scene[k]), dtype=float) - origin for k in ("left_robot", "right_robot")]


# -- scoring -------------------------------------------------------------------------------------------------------------
ON_RIM = 0.010  # a wire point less than this above the rim, over the trough outline, is still 'in the trough'
RELEASE_DIST = 0.03  # a TCP closer than this to the wire or the pick shank = still holding it
TRAY_TOP_Z = TOP + TRAY_H + 0.01


def _dense(poly: np.ndarray, step: float = 0.004) -> np.ndarray:
    out = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        k = max(1, int(np.linalg.norm(b - a) / step))
        out.extend(a + (b - a) * (j / k) for j in range(1, k + 1))
    return np.array(out)


def _wire_pts(objects) -> np.ndarray:
    return _dense(objects["wire"].polyline())


def _in_trough(p: np.ndarray) -> np.ndarray:
    """Points in or on the trough: within its outer outline (walls included), below 1 cm above the rim (a wire
    lying across a wall top counts as still in the trough)."""
    return ((p[:, 0] > TR_X0 - TR_T) & (p[:, 0] < TR_X1 + TR_T) & (p[:, 1] > TR_Y0 - TR_T) & (p[:, 1] < TR_Y1 + TR_T)
            & (p[:, 2] < RIM_Z + ON_RIM))


def _in_tray(p: np.ndarray) -> np.ndarray:
    return ((np.abs(p[:, 0] - TRAY_C[0]) <= TRAY_IN[0] / 2) & (np.abs(p[:, 1] - TRAY_C[1]) <= TRAY_IN[1] / 2)
            & (p[:, 2] < TRAY_TOP_Z))


def frac_in_trough(objects) -> float:
    return float(_in_trough(_wire_pts(objects)).mean())


def frac_in_tray(objects) -> float:
    return float(_in_tray(_wire_pts(objects)).mean())


def _shank_axis(objects) -> np.ndarray:
    from scipy.spatial.transform import Rotation

    s = objects["pick"]
    up = Rotation.from_quat(s.quat_xyzw).apply([0.0, 0.0, 1.0])
    return np.array([s.pos + up * SHANK[2] * t for t in np.linspace(-0.5, 0.5, 11)])


def released(objects) -> bool:
    """Neither TCP within 3 cm of the wire or of the pick shank's axis."""
    g = tcps()
    if not g:
        return True
    pts = np.vstack([_wire_pts(objects), _shank_axis(objects)])
    return all(float(np.linalg.norm(pts - t, axis=1).min()) >= RELEASE_DIST for t in g)


def score(objects) -> float:
    """0 while a gripper still holds (is within 3 cm of) the wire or the pick. Otherwise:
    whole wire out of the trough: 0.5 + 0.5 x fraction of the wire length inside the tray (1.0 = all in the tray);
    at least half of the wire length out of the trough: 0.25; else 0."""
    if not released(objects):
        return 0.0
    pts = _wire_pts(objects)
    f_trough = float(_in_trough(pts).mean())
    if f_trough <= 0.0:
        return round(0.5 + 0.5 * float(_in_tray(pts).mean()), 4)
    if f_trough <= 0.5:
        return 0.25
    return 0.0


def success(objects) -> bool:
    return score(objects) >= 1.0


def distance(objects) -> float:
    """Fraction of the wire not yet in the tray."""
    return 1.0 - frac_in_tray(objects)


def progress(objects) -> dict:
    pick = objects["pick"]
    pts = _wire_pts(objects)
    return {"pick_lifted": float(pick.pos[2] > TOP + SHANK[2] / 2 + 0.02),
            "wire_raised": float(pts[:, 2].max() > RIM_Z),
            "wire_out_of_trough": round(1.0 - float(_in_trough(pts).mean()), 3),
            "wire_in_tray": round(float(_in_tray(pts).mean()), 3),
            "released": float(released(objects))}


VARIATION = {"cable:wire": {"dx": 0.002, "dy": 0.015, "yaw": 0.0},
             "obj:pick": {"dx": 0.0, "dy": 0.0, "yaw": 0.0}}


# -- closed-loop scripted demo ---------------------------------------------------------------------------------------------
def _pick_state(st):
    """Shank centre, shank-top z, foot direction (unit xy) of the pick."""
    s, f = st["pick"], st["pick_foot"]
    d = f.pos[:2] - s.pos[:2]
    d = d / (np.linalg.norm(d) + 1e-12)
    return s.pos, s.pos[2] + SHANK[2] / 2, d


def _servo_pick(ctx, arm, cmd, target, z, yaw, dur=1.2, iters=3):
    """Move so the pick's shank axis reaches ``target`` (xy): command, settle, re-measure, correct."""
    cmd = np.asarray(cmd, dtype=float).copy()
    yield from arm.move((cmd[0], cmd[1], z), yaw=yaw, duration=dur)
    for _ in range(iters):
        yield from arm.wait(0.4)
        err = np.asarray(target, dtype=float) - ctx.states()["pick"].pos[:2]
        if np.linalg.norm(err) < 0.0008:
            break
        cmd += err
        yield from arm.move((cmd[0], cmd[1], z), yaw=yaw, duration=0.5)
    return cmd


def demo_script(ctx):
    """Right arm takes the pick from its holder, lowers it into a rib gap beside the wire with the foot pointing
    along the trough, turns it 90 deg so the foot swings under the wire, lifts the wire's middle above the rim, carries
    it over the near wall and lays the loop on the table in front of the trough, then pulls the pick back out and
    puts it down. Left arm picks up the loop and carries the wire (its ends slide up over the wall) into the tray."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    Rt, L = ctx.right, ctx.left
    cheat = os.environ.get("TROUGH_CHEAT", "")  # test only: "none" | "fingers" | "hold"

    def log(tag):
        st = ctx.states()
        sp, top, d = _pick_state(st)
        w = st["wire"].points
        print(f"[demo t={ctx.t:5.1f}] {tag}: score={score(st):.3f} trough={frac_in_trough(st):.2f} "
              f"tray={frac_in_tray(st):.2f} rel={released(st)} pick={np.round(sp, 3)} foot_dir={np.round(d, 2)} "
              f"wire_zmax={w[:, 2].max() - TOP:.3f} wire_xmin={w[:, 0].min():.3f} tcps={[np.round(t, 3) for t in tcps()]}",
              flush=True)

    log("start")
    if cheat == "none":
        yield from Rt.wait(4.0)
        log("end")
        return
    if cheat == "fingers":  # try to fish the wire out with the bare closed fingers
        yield from Rt.grip(FINGER_CLOSED)
        x0 = (TR_X0 + TR_X1) / 2
        for yaw in (math.pi / 2, 0.0):
            yield from Rt.move((x0, -0.01, RIM_Z + 0.06), yaw=yaw, duration=1.5)
            yield from Rt.move((x0, -0.01, TOP + 0.015), yaw=yaw, duration=1.5)  # as deep as it goes
            log(f"poked yaw={yaw:.2f}")
            yield from Rt.move((x0 - 0.01, -0.01, TOP + 0.03), yaw=yaw, duration=0.8)
            yield from Rt.move((x0, -0.01, RIM_Z + 0.08), yaw=yaw, duration=1.0)
        st = ctx.states()
        w = st["wire"].points
        k = int(np.argmax(w[:, 2]))
        p = w[k]
        yield from L.move((p[0] - 0.08, max(p[1], 0.0), RIM_Z + 0.08), yaw=0.0, finger=FINGER_OPEN, duration=1.5)
        yield from L.move((p[0], max(p[1], 0.0), RIM_Z + 0.03), yaw=0.0, duration=1.0)
        yield from L.grip(FINGER_CLOSED)
        yield from L.move((TRAY_C[0], TRAY_C[1], TOP + 0.10), yaw=0.0, duration=2.0)
        yield from L.grip(FINGER_OPEN)
        yield from L.move((0.20, 0.30, TOP + 0.12), yaw=0.0, duration=1.0)
        yield from Rt.move((0.20, -0.30, TOP + 0.12), yaw=0.0, duration=1.0)
        yield from Rt.wait(1.0)
        log("end")
        return

    hi = TOP + 0.165
    # 1) take the pick out of its holder (fingers close along y on the shank's top end)
    sp, top, d = _pick_state(ctx.states())
    grip_z = top - 0.012
    yield from Rt.move((sp[0], sp[1], hi), yaw=0.0, finger=FINGER_OPEN, duration=1.5)
    yield from Rt.move((sp[0], sp[1], grip_z), yaw=0.0, duration=1.2)
    yield from Rt.grip(FINGER_CLOSED, duration=0.8)
    yield from Rt.move((sp[0], sp[1], hi), yaw=0.0, duration=1.2)
    log("pick lifted")

    # 2) over the rib gap: shank against the far wall, foot pointing along the trough (+y), beside the wire
    ys = RIB_Y[2] + RIB_W / 2 + 0.010  # shank axis 10 mm into the gap after the rib at y -1.9 cm
    xs = TR_X1 - 0.0015 - SHANK[0] / 2
    st = ctx.states()
    sp, top, d = _pick_state(st)
    tcp = Rt.tcp()
    bot_below_tcp = tcp[2] - (top - SHANK[2])
    cmd = np.array([xs, ys]) + (tcp[:2] - sp[:2])
    cmd = yield from _servo_pick(ctx, Rt, cmd, (xs, ys), hi, 0.0)
    log("over gap")
    cmd = yield from _servo_pick(ctx, Rt, cmd, (xs, ys), TOP + 0.0015 + bot_below_tcp + 0.025, 0.0, dur=1.5)
    yield from Rt.move((cmd[0], cmd[1], TOP + 0.0015 + bot_below_tcp), yaw=0.0, duration=1.0)
    off = np.array([xs, ys]) - cmd  # pick minus command
    log("pick down")

    # 3) turn the foot under the wire (+y -> -x)
    for k in range(1, 10):
        yield from Rt.move((xs - off[0], ys - off[1], TOP + 0.0015 + bot_below_tcp), yaw=k * math.pi / 18,
                           duration=0.3)
    yield from Rt.wait(0.3)
    log("foot under wire")

    # 4) lift the wire's middle above the rim, slowly
    yaw = math.pi / 2
    z_up = RIM_Z + 0.012 + bot_below_tcp
    yield from Rt.move((xs - off[0], ys - off[1], z_up), yaw=yaw, duration=3.0)
    log("lifted")

    # 5) carry over the near wall (foot tip first) and lay the loop on the table in front of the trough
    st = ctx.states()
    sp, _, _ = _pick_state(st)
    tcp = Rt.tcp()
    off = sp[:2] - tcp[:2]
    x_dep = 0.205
    yield from Rt.move((x_dep - off[0], ys - off[1], z_up), yaw=yaw, duration=2.5)
    yield from Rt.move((x_dep - off[0], ys - off[1], TOP + 0.002 + bot_below_tcp), yaw=yaw, duration=1.5)
    yield from Rt.wait(0.3)
    log("loop laid")

    # 6) pull the pick back out from under the wire, lift it and put it down on the right
    yield from Rt.move((x_dep + 0.035 - off[0], ys - off[1], TOP + 0.002 + bot_below_tcp), yaw=yaw, duration=1.2)
    yield from Rt.move((x_dep + 0.035 - off[0], ys - off[1], hi), yaw=yaw, duration=1.0)
    yield from Rt.move((0.20, -0.16, hi), yaw=yaw, duration=1.2)
    yield from Rt.move((0.20, -0.16, TOP + bot_below_tcp + 0.004), yaw=yaw, duration=1.2)
    yield from Rt.grip(FINGER_OPEN)
    yield from Rt.move((0.20, -0.16, hi), yaw=yaw, duration=0.8)
    yield from Rt.move((0.15, -0.30, hi), yaw=0.0, duration=1.0)
    log("pick put down")

    # 7) left arm: take the wire near its middle where it lies on the table in front of the trough
    st = ctx.states()
    w = st["wire"].points
    ok = (w[:, 0] < TR_X0 - TR_T - 0.04) & (w[:, 2] < TOP + 0.02) & (w[:, 1] > -0.01)
    idx = np.nonzero(ok)[0]
    mid = (len(w) - 1) / 2
    k = int(idx[np.argmin(np.abs(idx - mid))]) if len(idx) else int(np.argmin(w[:, 0]))
    p = w[k]
    t = w[min(k + 1, len(w) - 1)] - w[max(k - 1, 0)]
    h = math.atan2(t[1], t[0])
    gy = (h + math.pi / 2) % math.pi - math.pi / 2  # close across the wire
    if gy > 0.0:
        gy = 0.0 if gy <= math.pi / 4 else -math.pi / 2  # left wrist range is [-pi/2, 0] (yaw ~ yaw + pi)
    print(f"[demo] left grasp at wire point {k}/{len(w)} {np.round(p, 3)} yaw {math.degrees(gy):.0f}", flush=True)
    f_part = -0.014
    yield from L.move((p[0], p[1], TOP + 0.08), yaw=gy, finger=f_part, duration=1.5)
    yield from L.move((p[0], p[1], p[2] + 0.010), yaw=gy, duration=1.0)
    yield from L.grip(FINGER_CLOSED)
    log("loop grasped")
    yield from L.move((p[0], p[1], TOP + 0.17), yaw=gy, duration=2.0)
    yield from L.move((min(p[0], 0.20), TRAY_C[1] - 0.04, TOP + 0.17), yaw=gy, duration=2.0)
    # lower until the hanging ends touch the tray's near half, then lay the doubled wire out along the tray's long
    # side (+y); the wrist turns so the wire leaves the fingers along y
    lay = -math.pi / 2
    yield from L.move((TRAY_C[0], TRAY_C[1] - 0.07, TOP + 0.17), yaw=lay, duration=1.0)
    yield from L.wait(1.0)  # let the hanging wire stop swinging
    yield from L.move((TRAY_C[0], TRAY_C[1] - 0.07, TOP + 0.13), yaw=lay, duration=1.0)
    yield from L.move((TRAY_C[0], TRAY_C[1] - 0.01, TOP + 0.06), yaw=lay, duration=1.5)
    yield from L.move((TRAY_C[0], TRAY_C[1] + 0.05, TOP + 0.02), yaw=lay, duration=1.2)
    log("over tray")
    if cheat == "hold":
        yield from L.wait(3.0)
        log("end (held)")
        return
    yield from L.grip(FINGER_OPEN)
    yield from L.move((TRAY_C[0], TRAY_C[1] + 0.05, TOP + 0.12), yaw=lay, duration=1.0)
    yield from L.move((0.15, 0.30, TOP + 0.12), yaw=0.0, duration=1.0)
    yield from L.wait(1.5)
    log("end")
