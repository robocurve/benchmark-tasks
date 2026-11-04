"""Tabletop mock of an EV charging point. A white charger post (box, 9 cm deep, 20 cm wide, 22 cm tall) stands at
the back of the table, centred between the arms. Its thick black charging cable (12 mm diameter, 50 cm long, 75 g)
leaves a grey cable boot at the foot of the post's front-left corner and lies straight in front of the post towards
the right arm, right across a parking bay taped on the table (yellow tape outline, 20 x 30 cm, to the right of the
post). On the cable's free end is a large black plug body (60 x 35 x 30 mm, 30 g) lying just beyond the bay. Two
fixed light-grey parts are on the post's front face: on the right, an open-top holster slot for the plug (floor 9 cm
above the table; the plug lies in it nose against the post, its cable leaving over a low lip at the front); on the
left, a cable hook (a peg sticking out 6 cm towards the robot, its top 14 cm above the table, with a 15 mm lip at its
tip). The plug must end in the holster and the cable must hang over the hook, with no part of the cable or plug in
the bay, and both grippers away from them.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score \\
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

INSTRUCTION = ("Put the charger plug in its holster and hang the cable over the hook on the charger post, so that "
               "no part of the cable or plug lies in the taped parking bay, then let go.")

WHITE = (0.90, 0.90, 0.88)
LIGHT = (0.70, 0.72, 0.75)
DARK = (0.06, 0.06, 0.06)
YELLOW = (0.95, 0.78, 0.05)

# -- charger post ----------------------------------------------------------------------------------------------------
FACE_X = 0.29  # front face of the post (towards the robot)
POST_D, POST_W, POST_H = 0.09, 0.20, 0.22
POST = Box("charger_post", pos=(FACE_X + POST_D / 2, 0.0, TOP + POST_H / 2), size=(POST_D, POST_W, POST_H), color=WHITE)
# cable boot: the cable leaves the post through a block at the foot of its front-left corner, 5.5 cm out from the face
CORD_X = FACE_X - 0.055
OUTLET = Box("cable_outlet", pos=(CORD_X + 0.0125, 0.095, TOP + 0.016), size=(0.085, 0.020, 0.032), color=LIGHT)

# -- cable hook: a peg out of the front face (along -x) with a lip at its tip; the cable drapes over it along y -----
HOOK_Y = 0.06
PEG_LEN, PEG_W, PEG_T = 0.060, 0.016, 0.014
PEG_TOP = TOP + 0.14
LIP_T, LIP_H = 0.012, 0.015
PEG_X0 = FACE_X - PEG_LEN  # tip end of the peg
HOOK = [
    Box("hook_peg", pos=(FACE_X - PEG_LEN / 2, HOOK_Y, PEG_TOP - PEG_T / 2), size=(PEG_LEN, PEG_W, PEG_T), color=LIGHT),
    Box("hook_lip", pos=(PEG_X0 + LIP_T / 2, HOOK_Y, PEG_TOP + LIP_H / 2), size=(LIP_T, PEG_W, LIP_H), color=LIGHT),
]

# -- holster: open-top slot on the front face; the plug lies in it along x, nose against the post face, cable out
#    of the open front end (towards the robot)
PL, PW, PH = 0.060, 0.035, 0.030  # plug length (along the cable), width, height
HOL_FLOOR = TOP + 0.09  # top of the slot floor
HOL_CY = -0.055  # slot centre (y)
HOL_IN_W = 0.043  # inner width (y) between the side walls
HOL_LEN = 0.074  # slot length (x), from the post face towards the robot (incl. the front lip)
HOL_LIP_T, HOL_LIP_H = 0.004, 0.008  # low lip across the open front end; the cable leaves over it
HOL_WALL_T, HOL_WALL_H, HOL_FLOOR_T = 0.005, 0.010, 0.008
HOL_X0 = FACE_X - HOL_LEN  # open end
_hw = HOL_IN_W + 2 * HOL_WALL_T
HOLSTER = [
    Box("holster_floor", pos=(FACE_X - HOL_LEN / 2, HOL_CY, HOL_FLOOR - HOL_FLOOR_T / 2),
        size=(HOL_LEN, _hw, HOL_FLOOR_T), color=LIGHT),
    *[Box(f"holster_wall_{n}", pos=(FACE_X - HOL_LEN / 2, HOL_CY + sg * (HOL_IN_W + HOL_WALL_T) / 2,
                                    HOL_FLOOR + HOL_WALL_H / 2),
          size=(HOL_LEN, HOL_WALL_T, HOL_WALL_H), color=LIGHT) for n, sg in (("l", 1), ("r", -1))],
    Box("holster_lip", pos=(HOL_X0 + HOL_LIP_T / 2, HOL_CY, HOL_FLOOR + HOL_LIP_H / 2),
        size=(HOL_LIP_T, HOL_IN_W, HOL_LIP_H), color=LIGHT),
]
PLUG_HOME_X = FACE_X - PL / 2 - 0.002  # plug centre when pushed in against the post face

# -- parking bay: yellow tape outline (15 mm tape, 1 mm thick); bay = everything inside the outer tape edge ---------
BAY_X0, BAY_X1 = 0.18, 0.38
BAY_Y0, BAY_Y1 = -0.42, -0.12
TAPE_W, TAPE_T = 0.015, 0.001
BAY = [
    Box("bay_tape_near", pos=(BAY_X0 + TAPE_W / 2, (BAY_Y0 + BAY_Y1) / 2, TOP + TAPE_T / 2),
        size=(TAPE_W, BAY_Y1 - BAY_Y0, TAPE_T), color=YELLOW),
    Box("bay_tape_far", pos=(BAY_X1 - TAPE_W / 2, (BAY_Y0 + BAY_Y1) / 2, TOP + TAPE_T / 2),
        size=(TAPE_W, BAY_Y1 - BAY_Y0, TAPE_T), color=YELLOW),
    Box("bay_tape_left", pos=((BAY_X0 + BAY_X1) / 2, BAY_Y1 - TAPE_W / 2, TOP + TAPE_T / 2),
        size=(BAY_X1 - BAY_X0 - 2 * TAPE_W, TAPE_W, TAPE_T), color=YELLOW),
    Box("bay_tape_right", pos=((BAY_X0 + BAY_X1) / 2, BAY_Y0 + TAPE_W / 2, TOP + TAPE_T / 2),
        size=(BAY_X1 - BAY_X0 - 2 * TAPE_W, TAPE_W, TAPE_T), color=YELLOW),
]

# -- charging cable: anchored at the outlet, straight along -y in front of the post, plug on its free end ----------
R = 0.006  # 12 mm cable
CORD_LEN = 0.50
CORD_Y0 = 0.085 - 0.012  # 6 mm clear of the outlet block's side
PLUG = Plug(size=(PL, PW, PH), mass=0.030, color=DARK)
CORD = cable("charge_cable", start=(CORD_X, CORD_Y0, TOP + PH / 2 + 0.0005), yaw=-math.pi / 2, length=CORD_LEN,
             radius=R, anchor_start=True, end_plug=PLUG, cable_mass=0.075, color=DARK,
             bend_stiffness=0.015, bend_damping=0.0015)

OBJECTS = [POST, OUTLET, *HOOK, *HOLSTER, *BAY, CORD]

# -- scoring ---------------------------------------------------------------------------------------------------------
RELEASE_CABLE = 0.04  # each gripper TCP at least this far from the cable centreline ...
RELEASE_PLUG = 0.06  # ... and from the plug centre (3 cm past the plug's half length)


def _dense(poly: np.ndarray, step: float = 0.003) -> np.ndarray:
    out = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        k = max(1, int(np.linalg.norm(b - a) / step))
        out.extend(a + (b - a) * (j / k) for j in range(1, k + 1))
    return np.array(out)


def _plug_axis(objects) -> np.ndarray:
    """Points along the plug's long axis (from the cable end to its nose)."""
    c = objects["charge_cable"]
    a = c.end_tip
    return np.array([a + c.end_dir * PL * t for t in np.linspace(0.0, 1.0, 9)])


def in_bay(objects) -> bool:
    """Any part of the cable or plug over the area inside the bay's outer tape edge (seen from above)."""
    pts = _dense(objects["charge_cable"].polyline())

    def inside(p, m):
        return (p[:, 0] >= BAY_X0 - m) & (p[:, 0] <= BAY_X1 + m) & (p[:, 1] >= BAY_Y0 - m) & (p[:, 1] <= BAY_Y1 + m)

    return bool(inside(pts, R).any() or inside(_plug_axis(objects), PW / 2).any())


def holstered(objects) -> bool:
    """Plug lying in the holster slot: centre between the side walls and between the front lip and the post
    face (3 mm tolerance), resting on the slot floor."""
    p = objects["charge_cable"].end_plug
    if p is None:
        return False
    ok_y = abs(p[1] - HOL_CY) <= (HOL_IN_W - PW) / 2 + 0.002
    ok_x = HOL_X0 + HOL_LIP_T + PL / 2 - 0.003 <= p[0] <= FACE_X - PL / 2 + 0.003
    ok_z = HOL_FLOOR + PH / 2 - 0.004 <= p[2] <= HOL_FLOOR + PH / 2 + 0.006
    return bool(ok_x and ok_y and ok_z)


def hooked(objects) -> bool:
    """Some part of the cable rests on the hook peg's top, between the lip and the post face."""
    pts = _dense(objects["charge_cable"].polyline())
    ok = (pts[:, 0] >= PEG_X0 + LIP_T) & (pts[:, 0] <= FACE_X) & (np.abs(pts[:, 1] - HOOK_Y) <= PEG_W / 2 + R) \
        & (pts[:, 2] >= PEG_TOP) & (pts[:, 2] <= PEG_TOP + R + 0.008)
    return bool(ok.any())


_ENV: dict = {"env": None}


def _install_env_hook() -> None:
    """Keep a handle on the sim env so score() can check that the grippers have let go."""
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls._task_step, "_hang_hooked", False):
        return
    base = cls._task_step

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        return base(self, *a, **k)

    _task_step._hang_hooked = True
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


def released(objects) -> bool:
    c = objects["charge_cable"]
    pts = _dense(c.polyline())
    for g in grippers():
        if float(np.linalg.norm(pts - g, axis=1).min()) < RELEASE_CABLE:
            return False
        if c.end_plug is not None and float(np.linalg.norm(c.end_plug - g)) < RELEASE_PLUG:
            return False
    return True


def score(objects) -> float:
    """0 if any part of the cable or plug is in the bay, or a gripper is still at the cable/plug. Otherwise
    0.6 if the plug lies in the holster + 0.4 if the cable rests on the hook."""
    if in_bay(objects) or not released(objects):
        return 0.0
    return round(0.6 * holstered(objects) + 0.4 * hooked(objects), 3)


def success(objects) -> bool:
    return score(objects) >= 1.0 and objects["charge_cable"].resting(max_speed=0.02)


def distance(objects) -> float:
    """Plug centre to its holstered position (m), + 0.1 if the cable is not on the hook."""
    p = objects["charge_cable"].end_plug
    if p is None:
        return 1.0
    d = float(np.linalg.norm(p - np.array([PLUG_HOME_X, HOL_CY, HOL_FLOOR + PH / 2])))
    return d + (0.0 if hooked(objects) else 0.1)


# -- closed-loop scripted demo (cabledemo.py) --------------------------------------------------------------------


def _cord_at(objects, a: float) -> np.ndarray:
    """Live cable point at arclength ``a`` from the outlet."""
    poly = objects["charge_cable"].polyline()
    s = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(poly, axis=0), axis=1))]
    return np.array([np.interp(a, s, poly[:, k]) for k in range(3)])


HOOK_S = 0.22  # the left arm takes the cable this far from the outlet and drapes that point over the hook
F_CABLE = -0.016  # fingers ~32 mm apart: enough for the 12 mm cable, clear of the post
F_PLUG = -0.028  # ~56 mm apart around the 35 mm plug


def demo_script(ctx):
    """Left arm: take the cable 20 cm from the outlet (in front of the post, by the bay), lift it over the hook's lip
    and lay it on the peg, let go. Right arm: take the plug beyond the bay near its cable end, lift it, turn it to
    point at the post and carry it (pulling the rest of the cable out of the bay; it now hangs from the hook) to the
    holster, lower it in, let go. Both arms move clear."""
    from yam_arena.constants import FINGER_CLOSED

    L, Rt = ctx.left, ctx.right
    cheat = os.environ.get("EVHOLSTER_CHEAT", "")  # test only: "none" | "holster" | "drag" | "hold"
    yl = -math.pi / 2  # fingers close along x, across the cable (which runs along y)
    over = 0.010

    def log(tag):
        st = ctx.states()
        c = st["charge_cable"]
        print(f"[demo t={ctx.t:5.1f}] {tag}: bay={in_bay(st)} hol={holstered(st)} hook={hooked(st)} "
              f"rel={released(st)} score={score(st):.2f} plug={np.round(c.end_plug, 3)}", flush=True)
        if os.environ.get("EVHOLSTER_DEBUG"):
            pts = c.points.copy()
            pts[:, 2] -= TOP
            print("   pts(z rel top):", np.round(pts[::2], 3).tolist(), "grippers:",
                  [np.round(g - [0, 0, TOP], 3).tolist() for g in grippers()], flush=True)

    log("start")
    if cheat == "none":
        yield from L.wait(3.0)
        log("end")
        return

    def left_hook():
        # stay well in front of the post (the holster hangs at x > 22 cm, 8-10 cm above the table)
        yield from L.move((0.15, 0.08, TOP + 0.08), yaw=yl, finger=F_CABLE, duration=1.2)
        p = _cord_at(ctx.states(), HOOK_S)
        yield from L.move((0.15, p[1], TOP + 0.06), yaw=yl, duration=1.2)
        yield from L.move((p[0], p[1], TOP + 0.06), yaw=yl, duration=0.8)
        p = _cord_at(ctx.states(), HOOK_S)
        yield from L.move((p[0], p[1], p[2] + over), yaw=yl, duration=0.8)
        yield from L.wait(0.3)
        log(f"left at cable, cmd {np.round(p + [0, 0, over - TOP], 3)}")
        yield from L.grip(FINGER_CLOSED)
        hx = (PEG_X0 + LIP_T + FACE_X) / 2 - 0.004
        yield from L.move((p[0], p[1], TOP + 0.06), yaw=yl, duration=0.6)
        yield from L.move((0.15, p[1], TOP + 0.06), yaw=yl, duration=0.8)
        yield from L.move((0.15, HOOK_Y - 0.02, TOP + 0.10), yaw=yl, duration=1.0)
        yield from L.move((PEG_X0 - 0.03, HOOK_Y - 0.01, PEG_TOP + 0.045), yaw=yl, duration=1.0)
        yield from L.move((hx, HOOK_Y, PEG_TOP + 0.045), yaw=yl, duration=0.8)
        log("over hook")
        yield from L.move((hx, HOOK_Y, PEG_TOP + R + over + 0.004), yaw=yl, duration=0.8)
        log("on hook, holding")
        yield from L.grip(F_CABLE)
        yield from L.move((hx, HOOK_Y, PEG_TOP + 0.05), yaw=yl, duration=0.6)
        yield from L.move((0.14, 0.18, TOP + 0.15), yaw=yl, duration=1.0)

    def right_plug(dest: str):
        st = ctx.states()
        c = st["charge_cable"]
        pp = c.end_plug - c.end_dir * 0.012  # grip the plug a little towards its cable end
        yield from Rt.move((pp[0], pp[1], TOP + 0.07), yaw=yl, finger=F_PLUG, duration=1.5)
        c = ctx.states()["charge_cable"]
        pp = c.end_plug - c.end_dir * 0.012
        yield from Rt.move((pp[0], pp[1], pp[2] + 0.004), yaw=yl, duration=0.8)
        yield from Rt.grip(FINGER_CLOSED)
        if dest == "drag":  # pull plug and cable towards the robot, out of the bay, and leave them on the table
            yield from Rt.move((0.12, pp[1] + 0.02, pp[2] + 0.012), yaw=yl, duration=2.0)
            yield from Rt.move((0.12, -0.08, pp[2] + 0.012), yaw=yl, duration=2.0)
            yield from Rt.move((0.11, 0.01, pp[2] + 0.012), yaw=yl, duration=1.5)
            yield from Rt.grip(F_PLUG)
            yield from Rt.move((0.12, -0.20, TOP + 0.12), yaw=yl, duration=1.0)
            return
        yield from Rt.move((pp[0], pp[1], TOP + 0.13), yaw=yl, duration=0.8)
        yield from Rt.move((0.20, -0.22, TOP + 0.15), yaw=-0.8, duration=1.5)
        log("plug carried")
        # the plug (nose had pointed -y) now points +x, at the post. Closed loop: correct for where the plug actually
        # sits in the fingers, so its centre lands 2 mm short of the post face, centred between the walls.
        yield from Rt.move((PLUG_HOME_X - 0.05, HOL_CY, TOP + 0.15), yaw=0.0, duration=1.2)
        yield from Rt.wait(0.3)
        g = grippers()
        off = ctx.states()["charge_cable"].end_plug - (g[1] if g else Rt.tcp())
        gx, gy = PLUG_HOME_X - off[0], HOL_CY - off[1]
        yield from Rt.move((gx, gy, TOP + 0.15), yaw=0.0, duration=0.8)
        yield from Rt.move((gx, gy, HOL_FLOOR + PH / 2 + 0.007 - off[2]), yaw=0.0, duration=1.0)
        log("plug in holster, holding")
        if dest == "hold":
            yield from Rt.wait(3.0)
            return
        yield from Rt.grip(F_PLUG)
        yield from Rt.move((gx, gy, TOP + 0.16), yaw=0.0, duration=0.8)
        yield from Rt.move((0.15, -0.25, TOP + 0.15), yaw=0.0, duration=1.0)

    if cheat == "holster":  # plug into the holster, cable left across the bay
        yield from right_plug("holster")
    elif cheat == "drag":
        yield from right_plug("drag")
    else:
        yield from left_hook()
        log("hooked")
        yield from right_plug("hold" if cheat == "hold" else "holster")
    log("placed")
    yield from L.wait(2.0)
    log("end")
