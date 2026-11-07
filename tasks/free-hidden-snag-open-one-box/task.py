"""Building-wiring mock-up on the station table. Three identical closed grey junction boxes (10 x 10 x 5 cm, open
bottom on the table, 5 mm walls) stand in a row along y, centres at x 0.34: LEFT (y +0.33), MIDDLE (+0.12) and RIGHT
(-0.09), seen from the robot. Each has a light grey lid (10 x 10 x 0.5 cm) hinged along its far top edge; a 3 cm tab
at the lid's front-left corner carries a dark 16 mm knob on its own pin. A lid swung past upright rests open on its
hinge stop (100 deg). One black 6 mm cable (72 cm) runs through all three boxes: each box has a 22 x 18 mm hole at
the foot of both side walls, 5 mm in front of the box's centre line. The cable's free tail end lies inside the LEFT
box. Between the boxes the cable lies in slack bows (4 cm towards the robot); it leaves the RIGHT box in a third bow
to its plug, which lies on the table in front of the right box. A blue open tray stands at the front right.

Hidden snag: inside the MIDDLE box the cable runs between two 16 mm pins (9 mm gap), and a cable tie left on the
cable (a rigid white head, 12 x 13 x 12 mm) sits on the tail side of the pins. Pulling the plug draws the cable until
the tie head jams against the pins: the bow between the right and middle boxes straightens, while the bow between
the middle and left boxes stays. That is the only outside cue to where the cable is caught. The robot must open only
the middle box's lid, lift the tie over the pins, close the lid, and draw the whole cable out of the boxes into the
tray. Opening the left or right lid more than 10 degrees at any time scores 0 for the run.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score \\
           --instruction "<INSTRUCTION below>"
Test-only demo variants: SNAG_CHEAT=<none|hold|yank|onearm_left|wrong_right|all_lids|tier30|tier60>.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import cablekit  # noqa: E402,F401  (hooks yam_arena so the Cable below is simulated)
import cableshape  # noqa: E402
import numpy as np  # noqa: E402
from cablekit import Plug, cable  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder, Sphere  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = ("The black cable is caught inside one of the three junction boxes. Free it by opening only that "
               "box, close its lid again, then pull the whole cable out of the boxes into the blue tray and let go.")

BOX_COL = (0.50, 0.51, 0.53)
LID_COL = (0.78, 0.79, 0.80)
KNOB_COL = (0.15, 0.15, 0.16)
PIN_COL = (0.30, 0.30, 0.32)
TRAY_COL = (0.15, 0.35, 0.80)
CABLE_COL = (0.04, 0.04, 0.04)
TIE_COL = (0.95, 0.95, 0.92)

# -- junction boxes (fixed walls; open bottom on the table) --------------------------------------------------------
XB = 0.34  # box centre x
BX, BY = 0.10, 0.10  # outer size: 10 cm deep (x), 10 cm wide (y)
WT = 0.005  # wall thickness
BH = 0.050  # wall height
HX = 0.335  # hole centre x (5 mm in front of the box centre line): the cable line
HW, HH = 0.022, 0.018  # hole width (x) and height (from the table)
BOX_Y = {"left": 0.33, "middle": 0.12, "right": -0.09}
KEYS = ("left", "middle", "right")
SNAG = "middle"


def _box(k: str) -> list[Box]:
    yc = BOX_Y[k]
    zc = TOP + BH / 2
    x0, x1 = XB - BX / 2, XB + BX / 2
    h0, h1 = HX - HW / 2, HX + HW / 2
    out = [
        Box(f"{k}_box_near", pos=(x0 + WT / 2, yc, zc), size=(WT, BY, BH), color=BOX_COL),
        Box(f"{k}_box_far", pos=(x1 - WT / 2, yc, zc), size=(WT, BY, BH), color=BOX_COL),
    ]
    for side, ys in (("l", yc + BY / 2 - WT / 2), ("r", yc - BY / 2 + WT / 2)):
        out += [
            Box(f"{k}_box_{side}_a", pos=((x0 + WT + h0) / 2, ys, zc), size=(h0 - x0 - WT, WT, BH), color=BOX_COL),
            Box(f"{k}_box_{side}_b", pos=((h1 + x1 - WT) / 2, ys, zc), size=(x1 - WT - h1, WT, BH), color=BOX_COL),
            Box(f"{k}_box_{side}_top", pos=(HX, ys, TOP + (HH + BH) / 2), size=(HW, WT, BH - HH), color=BOX_COL),
        ]
    return out


# -- lids (free, hinged to the world along the far top edge), knob on its own pin -----------------------------------
LID_T = 0.005
LID_Z0 = TOP + BH + LID_T / 2 + 0.0003
HINGE_X = XB + BX / 2
KNOB_R = 0.008
LEV = (0.006, 0.030, LID_T)  # lever: a 3 cm tab sticking out of each lid's front corner towards +y (beside the lid)
LEV_X = XB - BX / 2 + LEV[0] / 2
HINGE_MAX = 100.0  # deg: past upright, so an opened lid stays open
LIDS = {k: Box(f"{k}_lid", pos=(XB, BOX_Y[k], LID_Z0), size=(BX, BY, LID_T), mass=0.040, color=LID_COL)
        for k in KEYS}
LEVERS = {k: Box(f"{k}_lever", pos=(LEV_X, BOX_Y[k] + BY / 2 + LEV[1] / 2, LID_Z0), size=LEV, mass=0.004,
                 color=LID_COL) for k in KEYS}
KNOB_P0 = {k: np.array([LEV_X, BOX_Y[k] + BY / 2 + LEV[1] - 0.003, LID_Z0 + 0.004]) for k in KEYS}
KNOBS = {k: Sphere(f"{k}_knob", pos=tuple(KNOB_P0[k].tolist()), radius=KNOB_R, mass=0.004, color=KNOB_COL)
         for k in KEYS}

# -- the catch in the middle box: two pins either side of the cable line --------------------------------------------
PIN_R, PIN_H = 0.004, 0.016
PIN_GAP = 0.009  # clear gap between the pins (6 mm cable passes, the 13 mm tie head does not)
FORK_Y = BOX_Y[SNAG] - 0.015  # 1.5 cm right of the box centre, so the tie has room on its tail side
PIN_DX = PIN_GAP / 2 + PIN_R
PINS = [Cylinder(f"pin_{s}", pos=(HX + sg * PIN_DX, FORK_Y, TOP + PIN_H / 2), radius=PIN_R, height=PIN_H,
                 color=PIN_COL) for s, sg in (("far", 1), ("near", -1))]

# -- tray (fixed, open top) ----------------------------------------------------------------------------------------
TRAY_C = (0.19, -0.34)
TRAY_IX, TRAY_IY = 0.16, 0.18  # inner size
TRAY_WT, TRAY_H, TRAY_FLOOR = 0.005, 0.020, 0.003


def _tray() -> list[Box]:
    x, y = TRAY_C
    zc = TOP + TRAY_H / 2
    ox, oy = TRAY_IX + 2 * TRAY_WT, TRAY_IY + 2 * TRAY_WT
    return [
        Box("tray_floor", pos=(x, y, TOP + TRAY_FLOOR / 2), size=(ox, oy, TRAY_FLOOR), color=TRAY_COL),
        Box("tray_wall_near", pos=(x - TRAY_IX / 2 - TRAY_WT / 2, y, zc), size=(TRAY_WT, oy, TRAY_H), color=TRAY_COL),
        Box("tray_wall_far", pos=(x + TRAY_IX / 2 + TRAY_WT / 2, y, zc), size=(TRAY_WT, oy, TRAY_H), color=TRAY_COL),
        Box("tray_wall_l", pos=(x, y + TRAY_IY / 2 + TRAY_WT / 2, zc), size=(TRAY_IX, TRAY_WT, TRAY_H), color=TRAY_COL),
        Box("tray_wall_r", pos=(x, y - TRAY_IY / 2 - TRAY_WT / 2, zc), size=(TRAY_IX, TRAY_WT, TRAY_H), color=TRAY_COL),
    ]


# -- the cable: tail end (start) in the left box, plug (end) in front of the right box -------------------------------
R = 0.003
SEG = 0.02
BOW_A = 0.040  # slack bow between the boxes, towards the robot
BOW_P = 1.2  # bow profile sin(pi u)^p: eases the bend where the cable leaves a hole
PLUG = Plug(size=(0.035, 0.016, 0.012), mass=0.015, color=(0.10, 0.10, 0.11))
TIE = (0.012, 0.013, 0.012)  # tie head: along the cable, across, height (bottom on the table)
TIE_Y0 = FORK_Y + PIN_R + TIE[0] / 2 + 0.006  # tie head centre at the start: just on the tail side of the pins
TAIL_Y_NOM = BOX_Y["left"] + 0.030


def _bow(y0: float, y1: float, a: float, n: int = 40) -> np.ndarray:
    """A slack bow from (HX, y0) to (HX, y1) bulging ``a`` towards -x; zero slope at both ends."""
    u = np.linspace(0.0, 1.0, n)
    s = np.sin(np.pi * u) ** BOW_P  # 0 at both ends, 1 in the middle
    return np.c_[HX - a * s, y0 + (y1 - y0) * u]


def _path(tail_y: float) -> np.ndarray:
    yl, ym, yr = BOX_Y["left"], BOX_Y["middle"], BOX_Y["right"]
    h = BY / 2 + 0.004  # bows start/end 4 mm outside the walls
    pts = [np.array([[HX, tail_y]])]
    pts.append(np.array([[HX, yl - h]]))
    pts.append(_bow(yl - h, ym + h, BOW_A))
    pts.append(np.array([[HX, ym - h]]))
    pts.append(_bow(ym - h, yr + h, BOW_A))
    pts.append(np.array([[HX, yr - h]]))
    pts.append(_bow(yr - h, yr - h - 0.13, BOW_A))
    p = np.vstack(pts)
    keep = np.r_[True, np.linalg.norm(np.diff(p, axis=0), axis=1) > 1e-9]
    return p[keep]


def _arclen_to_y(path: np.ndarray, y: float) -> float:
    s = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
    i = int(np.argmax(path[:, 1] <= y))
    return float(s[i - 1] + (s[i] - s[i - 1]) * (path[i - 1, 1] - y) / (path[i - 1, 1] - path[i, 1]))


# Put the tie head exactly on a segment centre: shift the tail so that its arclength is (k + 0.5) * SEG.
_s_nom = _arclen_to_y(_path(TAIL_Y_NOM), TIE_Y0)
TIE_SEG = int(round(_s_nom / SEG - 0.5))
TAIL_Y = TAIL_Y_NOM + (TIE_SEG + 0.5) * SEG - _s_nom
PATH = _path(TAIL_Y)
_PATH_LEN = float(np.linalg.norm(np.diff(PATH, axis=0), axis=1).sum())
N_SEG = int(round(_PATH_LEN / SEG))
CORD_LEN = N_SEG * SEG
CORD = cable("cable", start=(HX, TAIL_Y, TOP + R + 0.0005), yaw=-math.pi / 2, length=CORD_LEN, radius=R,
             end_plug=PLUG, cable_mass=0.035, color=CABLE_COL, bend_stiffness=0.003, bend_damping=0.0005)

OBJECTS = [*(b for k in KEYS for b in _box(k)), *LIDS.values(), *LEVERS.values(), *KNOBS.values(), *PINS, *_tray(), CORD]

TWIN_EXTRA = [
    *({"name": f"{k}_lid_hinge", "kind": "joint", "real": "brass butt hinge along the box's far top edge",
       "note": "lid turns 0-100 deg about y; past upright it rests open on the stop"} for k in KEYS),
    *({"name": f"{k}_lever_weld", "kind": "joint", "real": "lever tab glued/screwed to the lid",
       "note": "3 cm tab at the lid's front corner, sticking out beside the box"} for k in KEYS),
    *({"name": f"{k}_knob_pin", "kind": "joint", "real": "knob on a loose M4 shoulder screw",
       "note": "the knob turns freely about y, so a firm pinch does not lock the hinge"} for k in KEYS),
    {"name": "tie_head", "kind": "body", "size_mm": [12, 13, 12], "colour": "white",
     "pose": [HX, float(TIE_Y0), 0.006], "note": "rigid cable-tie head clamped on the cable (printed block)"},
]


# -- spawn hook: lid hinges, knob pins and the tie head collider -------------------------------------------------------

def _install_spawn() -> None:
    orig = cablekit._spawn_cable
    if getattr(orig, "_snag_hook", False):
        return

    def spawn(prim_path, cfg, *a, **k):
        prim = orig(prim_path, cfg, *a, **k)
        if getattr(cfg, "cable_name", "") != CORD.name:
            return prim
        import isaaclab.sim as sim_utils
        from isaaclab.sim.utils import bind_physics_material, bind_visual_material
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics

        from yam_arena.station import linear_rgb

        stage = get_current_stage()
        parent = str(prim_path).rsplit("/", 1)[0]
        w = UsdGeom.Xformable(stage.GetPrimAtPath(parent)).ComputeLocalToWorldTransform(0).ExtractTranslation()
        for kk in KEYS:
            lid, knob = f"{parent}/{LIDS[kk].name}", f"{parent}/{KNOBS[kk].name}"
            h = UsdPhysics.RevoluteJoint.Define(stage, f"{prim_path}/{kk}_lid_hinge")
            h.CreateBody1Rel().SetTargets([Sdf.Path(lid)])
            h.CreateAxisAttr("Y")
            h.CreateLocalPos0Attr(Gf.Vec3f(float(w[0] + HINGE_X), float(w[1] + BOX_Y[kk]), float(w[2] + LID_Z0)))
            h.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
            h.CreateLocalPos1Attr(Gf.Vec3f(float(BX / 2), 0.0, 0.0))
            h.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
            h.CreateLowerLimitAttr(-2.0)
            h.CreateUpperLimitAttr(HINGE_MAX)
            h.CreateExcludeFromArticulationAttr(True)
            lev = f"{parent}/{LEVERS[kk].name}"
            for jn, b0, b1, p0, pin in (("lever_weld", lid, lev, np.array(LEVERS[kk].pos) - np.array(LIDS[kk].pos), False),
                                        ("knob_pin", lev, knob, KNOB_P0[kk] - np.array(LEVERS[kk].pos), True)):
                if pin:  # the knob turns freely on a pin parallel to the hinge, so a firm pinch doesn't lock the lid
                    j = UsdPhysics.RevoluteJoint.Define(stage, f"{prim_path}/{kk}_{jn}")
                    j.CreateAxisAttr("Y")
                else:
                    j = UsdPhysics.FixedJoint.Define(stage, f"{prim_path}/{kk}_{jn}")
                j.CreateBody0Rel().SetTargets([Sdf.Path(b0)])
                j.CreateBody1Rel().SetTargets([Sdf.Path(b1)])
                j.CreateLocalPos0Attr(Gf.Vec3f(*(float(v) for v in p0)))
                j.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
                j.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
                j.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
                j.CreateExcludeFromArticulationAttr(True)
        # tie head: an extra box collider on one cable segment (moves with it; the segment's mass is unchanged)
        look = f"{prim_path}/Looks/tie"
        mat = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(TIE_COL), roughness=0.5)
        mat.func(look, mat)
        g = UsdGeom.Cube.Define(stage, f"{prim_path}/seg_{TIE_SEG}/tie_head")
        g.CreateSizeAttr(1.0)
        g.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, float(TIE[2] / 2 - R + 0.0004)))
        g.AddScaleOp().Set(Gf.Vec3f(*(float(v) for v in TIE)))
        UsdPhysics.CollisionAPI.Apply(g.GetPrim())
        cp = PhysxSchema.PhysxCollisionAPI.Apply(g.GetPrim())
        cp.CreateContactOffsetAttr(0.002)
        cp.CreateRestOffsetAttr(0.0)
        bind_physics_material(str(g.GetPath()), f"{prim_path}/Looks/cable_physics", stage=stage)
        bind_visual_material(str(g.GetPath()), look, stage=stage)
        return prim

    spawn._snag_hook = True
    cablekit._spawn_cable = spawn


_install_spawn()


def _shape(emb):
    cableshape.preshape(emb, CORD, PATH, hold="set", settle_s=0)
    cableshape.settle(emb, 2.0)
    _reset_episode()


cableshape.on_reset(_shape)

# -- env handle (gripper release check) --------------------------------------------------------------------------------
_ENV: dict = {"env": None}


def _install_env_hook() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover  (offline import)
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls._task_step, "_snag_hooked", False):
        return
    base = cls._task_step

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        return base(self, *a, **k)

    _task_step._snag_hooked = True
    cls._task_step = _task_step


_install_env_hook()


def grippers() -> list[np.ndarray]:
    env = _ENV["env"]
    if env is None:
        return []
    from yam_arena.runtime import tcp_world

    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
    return [np.asarray(tcp_world(base.scene[k]), dtype=np.float64) - origin for k in ("left_robot", "right_robot")]


# -- geometry ---------------------------------------------------------------------------------------------------------


def _rot(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def lid_deg(objects, k) -> float:
    """Lid opening angle about its hinge (deg, 0 = shut, ~100 = resting open on the stop)."""
    st = objects[LIDS[k].name]
    if not np.isfinite(st.quat_xyzw).all():
        return 180.0
    front = _rot(st.quat_xyzw) @ np.array([-1.0, 0.0, 0.0])
    return float(math.degrees(math.atan2(front[2], -front[0])))


def _dense(poly: np.ndarray, step: float = 0.004) -> np.ndarray:
    if not np.isfinite(poly).all():
        return np.asarray(poly)
    out = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        n = max(1, int(np.ceil(np.linalg.norm(b - a) / step)))
        out += [a + (b - a) * (i / n) for i in range(1, n + 1)]
    return np.asarray(out)


def cable_pts(objects) -> np.ndarray:
    c = objects[CORD.name]
    pts = _dense(c.polyline())
    return np.vstack([pts, np.asarray(c.end_plug)[None]]) if c.end_plug is not None else pts


def tie_pos(objects) -> np.ndarray:
    return np.asarray(objects[CORD.name].points[TIE_SEG])


def _in_box(p, k, margin=0.0) -> np.ndarray:
    p = np.atleast_2d(p)
    yc = BOX_Y[k]
    return ((np.abs(p[:, 0] - XB) <= BX / 2 + margin) & (np.abs(p[:, 1] - yc) <= BY / 2 + margin))


def in_any_box(objects) -> bool:
    pts = cable_pts(objects)
    return bool(any(_in_box(pts, k).any() for k in KEYS))


def tie_out(objects) -> bool:
    """The tie head is past the catch and out of every box, on the plug side of the right box."""
    t = tie_pos(objects)
    return bool(t[1] < BOX_Y["right"] - BY / 2 and not any(_in_box(t, k)[0] for k in KEYS))


TRAY_RIM_Z = TOP + TRAY_H + 0.010  # cable resting in the tray: every point below 1 cm above the rim


def in_tray(objects) -> bool:
    pts = cable_pts(objects)
    x, y = TRAY_C
    return bool((np.abs(pts[:, 0] - x) <= TRAY_IX / 2).all() and (np.abs(pts[:, 1] - y) <= TRAY_IY / 2).all()
                and (pts[:, 2] <= TRAY_RIM_Z).all())


WRONG_OPEN_DEG = 10.0  # a wrong lid opened more than this at any time: 0 for the run
CLOSED_DEG = 5.0  # the opened lid counts as closed again within 5 deg of shut
RELEASE_DIST = 0.03  # both TCPs >= 3 cm from the cable, plug, tie, every lid knob and lid

_STATE = {"violated": False, "why": "", "max_deg": {k: 0.0 for k in KEYS}}


def _reset_episode():
    _STATE["violated"] = False
    _STATE["why"] = ""
    _STATE["max_deg"] = {k: 0.0 for k in KEYS}


def _finite(objects) -> bool:
    return bool(np.isfinite(objects[CORD.name].polyline()).all()
                and all(np.isfinite(objects[LIDS[k].name].pos).all() for k in KEYS))


def _track(objects) -> None:
    if not _finite(objects):
        _STATE["violated"], _STATE["why"] = True, _STATE["why"] or "nan"
        return
    for k in KEYS:
        d = lid_deg(objects, k)
        _STATE["max_deg"][k] = max(_STATE["max_deg"][k], d)
        if k != SNAG and d > WRONG_OPEN_DEG and not _STATE["violated"]:
            _STATE["violated"], _STATE["why"] = True, f"{k} lid opened {d:.0f} deg"


def _aabb_dist(st, g) -> float:
    d = np.maximum(np.maximum(st.low - g, g - st.high), 0.0)
    return float(np.linalg.norm(d))


def released(objects) -> bool:
    pts = np.vstack([cable_pts(objects), tie_pos(objects)[None]])
    for g in grippers():
        if float(np.linalg.norm(pts - g, axis=1).min()) < RELEASE_DIST:
            return False
        for k in KEYS:
            if np.linalg.norm(np.asarray(objects[KNOBS[k].name].pos) - g) < RELEASE_DIST:
                return False
            if min(_aabb_dist(objects[LIDS[k].name], g), _aabb_dist(objects[LEVERS[k].name], g)) < RELEASE_DIST:
                return False
    return True


def score(objects) -> float:
    """0 if: at any time the left or right lid opened more than 10 deg (latched; opening every lid included); NaN;
    a gripper TCP within 3 cm of the cable, plug, tie, any lid or knob at the end; the tie head still in a box or
    not yet past the right box. Otherwise 0.3 (tie freed and drawn out past the right box), 0.6 (no part of the
    cable or plug above any box's footprint), 1.0 (also every cable point and the plug inside the tray's inner
    footprint, resting in it, and the middle lid closed within 5 deg)."""
    _track(objects)
    if _STATE["violated"] or not _finite(objects) or not released(objects):
        return 0.0
    if not tie_out(objects):
        return 0.0
    if in_any_box(objects):
        return 0.3
    if in_tray(objects) and lid_deg(objects, SNAG) <= CLOSED_DEG:
        return 1.0
    return 0.6


def success(objects) -> bool:
    return bool(score(objects) >= 1.0 and objects[CORD.name].resting(max_speed=0.02))


def distance(objects) -> float:
    if _STATE["violated"] or not _finite(objects):
        return 1.0
    pts = cable_pts(objects)
    return float(np.linalg.norm(pts[:, :2] - np.array(TRAY_C), axis=1).max())


def progress(objects) -> dict:
    return {
        "wrong_lids_shut": int(not _STATE["violated"]),
        "snag_lid_opened": int(_STATE["max_deg"][SNAG] > WRONG_OPEN_DEG),
        "tie_past_pins": int(bool(tie_pos(objects)[1] < FORK_Y - PIN_R)),
        "tie_out": int(tie_out(objects)),
        "cable_out_of_boxes": int(not in_any_box(objects)),
        "in_tray": int(in_tray(objects)),
        "snag_lid_closed": int(lid_deg(objects, SNAG) <= CLOSED_DEG),
        "released": int(released(objects)),
    }


VARIATION = {
    "cable:cable": {"dx": 0.0, "dy": 0.0, "yaw": 0.0},  # threaded through fixed holes: no free offset
    "obj:left_lid": {"dx": 0.0, "dy": 0.0, "yaw": 0.0},  # hinged lids: no offset
    "obj:middle_lid": {"dx": 0.0, "dy": 0.0, "yaw": 0.0},
    "obj:right_lid": {"dx": 0.0, "dy": 0.0, "yaw": 0.0},
}

# -- closed-loop scripted demo (cabledemo.py) -------------------------------------------------------------------------
# Right arm: take the plug and pull it straight out from the right box in 1 cm steps until it stops following (the
# tie has jammed). The bow between the right and middle boxes is now straight, the left one is not: the snag is in
# the middle box. Ease back 5 cm of slack. Left arm: swing the middle lid open by its knob (past upright, it rests on
# its stop), slide the cable 3 cm towards the tail so the tie head is clear of the pins, pick the tie head up, carry it
# over the pins to the plug side and set it down; swing the lid shut. Right arm: lay the plug in the tray, then draw
# the cable out of the right box's hole stroke by stroke into the tray, and let go.

FINGER_PINCH = -0.020
FINGER_CORD = -0.014
LYAW = -math.pi / 2  # left wrist camera looks -y (into the middle box from the left arm's side)
HI = TOP + 0.10


def _yaw(h: float) -> float:
    return (h + math.pi / 2) % math.pi - math.pi / 2


def _gap_dev(objects, y_hi: float, y_lo: float) -> float:
    """Largest sideways deviation (m) of the cable from the straight hole-to-hole line between y_hi and y_lo."""
    pts = objects[CORD.name].points
    m = (pts[:, 1] < y_hi) & (pts[:, 1] > y_lo)
    return float(np.abs(pts[m, 0] - HX).max()) if m.any() else 0.0


def gap_bows(objects) -> tuple[float, float]:
    h = BY / 2
    return (_gap_dev(objects, BOX_Y["left"] - h, BOX_Y["middle"] + h),
            _gap_dev(objects, BOX_Y["middle"] - h, BOX_Y["right"] + h))


def demo_script(ctx):
    from yam_arena.constants import FINGER_CLOSED

    cheat = os.environ.get("SNAG_CHEAT", "")
    L, Rt = ctx.left, ctx.right

    def log(tag):
        st = ctx.states()
        bl, br = gap_bows(st)
        print(f"[demo t={ctx.t:5.1f}] {tag}: score={score(st)} lids="
              f"{ {k: round(lid_deg(st, k), 1) for k in KEYS} } bows L/R={bl * 1000:.0f}/{br * 1000:.0f} mm "
              f"tie={np.round(tie_pos(st), 4)} plug={np.round(st[CORD.name].end_plug, 3)} "
              f"viol={_STATE['violated']}:{_STATE['why']} prog={progress(st)}", flush=True)

    log("start")
    if cheat == "none":
        for _ in range(4):
            yield from L.wait(1.0)
        log("end")
        return

    grab = {}

    # ---- plug handling (right arm) ----------------------------------------------------------------------------
    def plug_pose():
        c = ctx.states()[CORD.name]
        p = np.asarray(c.end_plug)
        d = c.points[-1] - c.points[-2]
        return p, math.atan2(d[1], d[0])

    def take_plug():
        p, h = plug_pose()
        y = _yaw(h + math.pi)  # wrist camera looks back along the cable towards the boxes
        yield from Rt.move((p[0], p[1], HI), yaw=y, finger=FINGER_PINCH, duration=1.5)
        yield from Rt.move((p[0], p[1], p[2] + 0.010), yaw=y, duration=1.0)
        yield from Rt.grip(FINGER_CLOSED, 0.5)
        grab["ryaw"] = y
        grab["roff"] = np.array([p[0], p[1], p[2] + 0.010]) - plug_pose()[0]

    def pull_until_stall(direction, max_pull=0.10, step=0.01):
        u = np.asarray(direction, float) / np.linalg.norm(direction)
        tcp = Rt.tcp().copy() if hasattr(Rt, "tcp") else None
        p0, _ = plug_pose()
        target = p0 + grab["roff"]
        target[2] = TOP + 0.025
        yield from Rt.move(target, yaw=grab["ryaw"], duration=0.6)
        pulled = 0.0
        while pulled < max_pull:
            before, _ = plug_pose()
            target = target + u * step
            yield from Rt.move(target, yaw=grab["ryaw"], duration=0.4)
            yield from Rt.wait(0.2)
            after, _ = plug_pose()
            moved = float(np.dot(after - before, u))
            pulled += step
            if moved < 0.4 * step:  # the plug stopped following: the cable is held
                log(f"stall after {pulled * 100:.0f} cm (last step moved {moved * 1000:.1f} mm)")
                break
        del tcp
        grab["rt"] = target

    def ease(back=0.05):
        p, _ = plug_pose()
        hole = np.array([HX, BOX_Y["right"] - BY / 2, p[2]])
        u = hole - p
        u[2] = 0
        u /= np.linalg.norm(u)
        t = grab["rt"] + u * back
        yield from Rt.move(t, yaw=grab["ryaw"], duration=1.0)
        grab["rt"] = t

    def lay_plug_in_tray():
        x, y = TRAY_C
        dest = np.array([x - 0.03, y - 0.05, TOP + 0.03])
        yield from Rt.move((grab["rt"][0], grab["rt"][1], TOP + 0.05), yaw=grab["ryaw"], duration=0.6)
        yield from Rt.move((dest[0], dest[1], TOP + 0.05), yaw=grab["ryaw"], duration=1.5)
        yield from Rt.move(dest, yaw=grab["ryaw"], duration=0.6)
        yield from Rt.grip(FINGER_PINCH, 0.4)
        yield from Rt.move((dest[0], dest[1], HI), yaw=grab["ryaw"], duration=0.8)

    # ---- lid handling (one arm, closed loop on the hinge) -------------------------------------------------------
    def knob_step(k, delta_deg):
        st = ctx.states()
        lid = st[LIDS[k].name]
        hinge = np.asarray(lid.pos) + _rot(lid.quat_xyzw) @ np.array([BX / 2, 0.0, 0.0])
        v = np.asarray(st[KNOBS[k].name].pos) - hinge
        t = math.radians(delta_deg)  # opening = positive turn about +y: the front edge rises and swings back
        x, z = v[0] * math.cos(t) + v[2] * math.sin(t), -v[0] * math.sin(t) + v[2] * math.cos(t)
        return hinge + np.array([x, v[1], z])

    def swing(arm, k, target_deg, yaw, step=8.0, dur=0.3):
        last = None
        for _ in range(40):
            now = lid_deg(ctx.states(), k)
            d = target_deg - now
            if abs(d) < 2.0 or (last is not None and abs(now - last) < 0.3):
                break
            last = now
            yield from arm.move(knob_step(k, float(np.clip(d, -step, step))) + grab[f"koff_{k}"], yaw=yaw, duration=dur)
            print(f"[swing {k}] {lid_deg(ctx.states(), k):5.1f} deg", flush=True)

    def open_lid(arm, k, yaw, deg=96.0):
        kp = ctx.states()[KNOBS[k].name].pos
        yield from arm.move((kp[0], kp[1], kp[2] + 0.05), yaw=yaw, finger=FINGER_PINCH, duration=1.5)
        yield from arm.move((kp[0], kp[1], kp[2] + 0.006), yaw=yaw, duration=0.8)
        yield from arm.grip(FINGER_CLOSED, 0.5)
        grab[f"koff_{k}"] = np.array([kp[0], kp[1], kp[2] + 0.006]) - ctx.states()[KNOBS[k].name].pos
        yield from swing(arm, k, deg, yaw)
        yield from arm.wait(0.3)
        yield from arm.grip(FINGER_PINCH, 0.4)
        p = arm.tcp()
        yield from arm.move((p[0], p[1], p[2] + 0.03), yaw=yaw, duration=0.6)
        yield from arm.move((p[0] - 0.06, p[1], p[2] + 0.03), yaw=yaw, duration=0.8)
        yield from arm.wait(0.5)
        log(f"{k} lid opened")

    def close_lid(arm, k, yaw, let_go=True):
        kp = ctx.states()[KNOBS[k].name].pos
        yield from arm.move((kp[0] - 0.04, kp[1], kp[2] + 0.02), yaw=yaw, finger=FINGER_PINCH, duration=1.2)
        yield from arm.move((kp[0], kp[1], kp[2] + 0.006), yaw=yaw, duration=0.8)
        yield from arm.grip(FINGER_CLOSED, 0.5)
        grab[f"koff_{k}"] = np.array([kp[0], kp[1], kp[2] + 0.006]) - ctx.states()[KNOBS[k].name].pos
        yield from swing(arm, k, 8.0, yaw)
        if not let_go:
            return
        yield from arm.grip(FINGER_PINCH, 0.4)
        p = arm.tcp()
        yield from arm.move((p[0] - 0.03, p[1], p[2] + 0.05), yaw=yaw, duration=0.8)
        yield from arm.wait(0.6)
        log(f"{k} lid closed")

    # ---- freeing the tie (left arm) ---------------------------------------------------------------------------
    def free_tie():
        """Pinch the tie head itself (pads clear of the pins), lift it clear over the pins, carry it to the plug side
        of the pins and set it down there."""
        t = tie_pos(ctx.states())
        gy = max(t[1], FORK_Y + PIN_R + 0.017)  # pads (about 8 mm each side) clear of the pins
        g = np.array([t[0], gy, TOP + TIE[2] / 2 + 0.010])
        yield from L.move((g[0], g[1], HI), yaw=LYAW, finger=FINGER_CORD, duration=1.2)
        yield from L.move(g, yaw=LYAW, duration=0.8)
        print(f"[free] grasp cmd {np.round(g, 4)} tcp {np.round(L.tcp(), 4)} real {np.round(grippers()[0], 4)} tie {np.round(t, 4)}", flush=True)
        yield from L.grip(FINGER_CLOSED, 0.4)
        print(f"[free] closed real {np.round(grippers()[0], 4)}", flush=True)
        lift = TOP + PIN_H + TIE[2] + 0.018
        yield from L.move((g[0], g[1], lift), yaw=LYAW, duration=0.8)
        yield from L.wait(0.3)
        print(f"[free] lifted tcp {np.round(L.tcp(), 4)} real {np.round(grippers()[0], 4)}", flush=True)
        log("tie lifted")
        dest_y = FORK_Y - PIN_R - 0.020
        yield from L.move((HX, dest_y, lift), yaw=LYAW, duration=1.2)
        yield from L.move((HX, dest_y, g[2] + 0.002), yaw=LYAW, duration=0.8)
        yield from L.grip(FINGER_CORD, 0.3)
        yield from L.move((HX, dest_y, HI), yaw=LYAW, duration=0.6)
        log("tie over the pins")

    # ---- extraction strokes (right arm) ---------------------------------------------------------------------
    def stroke(max_strokes=6, until_tie_out=False):
        x, y = TRAY_C
        hole_y = BOX_Y["right"] - BY / 2
        for i in range(max_strokes):
            st = ctx.states()
            if until_tie_out and tie_out(st):
                break
            pts = st[CORD.name].points
            # first segment (from the tail) that is out of the right box, 2.5 cm past its hole
            ok = np.where(pts[:, 1] < hole_y - 0.025)[0]
            if not in_any_box(st) or len(ok) == 0:
                break
            j = int(ok[0])
            p = pts[j]
            d = pts[min(j + 1, len(pts) - 1)] - pts[max(j - 1, 0)]
            yw = _yaw(math.atan2(d[1], d[0]))
            yield from Rt.move((p[0], p[1], HI), yaw=yw, finger=FINGER_CORD, duration=0.9)
            yield from Rt.move((p[0], p[1], p[2] + 0.010), yaw=yw, duration=0.6)
            yield from Rt.grip(FINGER_CLOSED, 0.4)
            dest = np.array([x + 0.02 - 0.02 * i, y + 0.03 - 0.02 * (i % 3), TOP + 0.03])
            yield from Rt.move((p[0], p[1], TOP + 0.03), yaw=yw, duration=0.4)
            yield from Rt.move(dest, yaw=yw, duration=1.4)
            yield from Rt.move((dest[0], dest[1], TOP + 0.015), yaw=yw, duration=0.4)
            yield from Rt.grip(FINGER_CORD, 0.3)
            yield from Rt.move((dest[0], dest[1], HI), yaw=yw, duration=0.6)
            log(f"stroke {i}")

    def tidy(max_moves=4):
        """Pick any cable stretch lying outside the tray and lay it inside."""
        x, y = TRAY_C
        for i in range(max_moves):
            st = ctx.states()
            pts = st[CORD.name].points
            out = np.where((np.abs(pts[:, 0] - x) > TRAY_IX / 2 - 0.01) | (np.abs(pts[:, 1] - y) > TRAY_IY / 2 - 0.01))[0]
            if len(out) == 0:
                break
            j = int(out[np.argmax(np.linalg.norm(pts[out, :2] - np.array(TRAY_C), axis=1))])
            p = pts[j]
            d = pts[min(j + 1, len(pts) - 1)] - pts[max(j - 1, 0)]
            yw = _yaw(math.atan2(d[1], d[0]))
            v = np.array(TRAY_C) - p[:2]
            v = v / (np.linalg.norm(v) + 1e-9)
            dest = p[:2] + v * min(np.linalg.norm(np.array(TRAY_C) - p[:2]), 0.08)
            yield from Rt.move((p[0], p[1], HI), yaw=yw, finger=FINGER_CORD, duration=0.9)
            yield from Rt.move((p[0], p[1], p[2] + 0.010), yaw=yw, duration=0.6)
            yield from Rt.grip(FINGER_CLOSED, 0.4)
            yield from Rt.move((p[0], p[1], TOP + 0.035), yaw=yw, duration=0.4)
            yield from Rt.move((dest[0], dest[1], TOP + 0.035), yaw=yw, duration=1.0)
            yield from Rt.move((dest[0], dest[1], TOP + 0.015), yaw=yw, duration=0.4)
            yield from Rt.grip(FINGER_CORD, 0.3)
            yield from Rt.move((dest[0], dest[1], HI), yaw=yw, duration=0.6)
            log(f"tidy {i}")

    def park(arm, side):
        yield from arm.move((0.18, side * 0.26, TOP + 0.16), yaw=0.0, finger=FINGER_CLOSED, duration=1.0)

    # ---- shortcut variants (test only) ----------------------------------------------------------------------
    if cheat == "yank":  # right arm alone: pull the plug hard and far, then carry it to the tray
        yield from take_plug()
        yield from pull_until_stall((0.0, -1.0, 0.0), max_pull=0.0)
        t = grab["rt"]
        for k in range(8):
            t = t + np.array([-0.01, -0.012, 0.0])
            yield from Rt.move(t, yaw=grab["ryaw"], duration=0.15)
        log("yanked")
        grab["rt"] = t
        yield from lay_plug_in_tray()
        yield from park(Rt, -1)
        yield from L.wait(1.0)
        log("end")
        return
    if cheat in ("wrong_right", "all_lids"):  # guess: open the box next to the plug (and, for all_lids, every box)
        yield from open_lid(Rt, "right", math.pi / 2)
        yield from close_lid(Rt, "right", math.pi / 2)
        yield from park(Rt, -1)
        if cheat == "all_lids":
            yield from open_lid(L, "left", LYAW)
            yield from close_lid(L, "left", LYAW)
    if cheat == "onearm_left":  # left arm only: open, free the tie, close; nobody pulls the cable out
        yield from open_lid(L, SNAG, LYAW)
        yield from free_tie()
        yield from close_lid(L, SNAG, LYAW)
        yield from park(L, 1)
        yield from L.wait(1.0)
        log("end")
        return

    # ---- the real job ---------------------------------------------------------------------------------------
    yield from take_plug()
    yield from pull_until_stall((0.0, -1.0, 0.0))
    log("pull test")
    yield from open_lid(L, SNAG, LYAW)  # the right arm keeps the cable taut meanwhile
    yield from ease(0.04)
    log("slack given")
    yield from free_tie()
    # take up the slack at once, so the freed tie is drawn away from the pins before anything else
    p, _ = plug_pose()
    t = grab["rt"] + np.array([0.0, -0.05, 0.0])
    yield from Rt.move(t, yaw=grab["ryaw"], duration=1.0)
    grab["rt"] = t
    log("slack taken up")
    if cheat != "tier60":
        yield from close_lid(L, SNAG, LYAW, let_go=(cheat != "hold"))
    if cheat != "hold":
        yield from park(L, 1)
    yield from lay_plug_in_tray()
    log("plug in tray")
    if cheat == "tier30":  # partial: draw cable only until the tie is out past the right box, rest left in the boxes
        yield from stroke(until_tie_out=True)
        yield from park(Rt, -1)
    else:
        yield from stroke()
        if cheat != "tier60":
            yield from tidy()
        if cheat == "hold":  # everything done, but the right gripper closes on the cable again at the end
            st = ctx.states()
            p = st[CORD.name].points[0]
            yield from Rt.move((p[0], p[1], HI), yaw=0.0, finger=FINGER_CORD, duration=0.8)
            yield from Rt.move((p[0], p[1], p[2] + 0.010), yaw=0.0, duration=0.6)
            yield from Rt.grip(FINGER_CLOSED, 0.4)
        else:
            yield from park(Rt, -1)
    for _ in range(3):
        yield from L.wait(0.5)
    log("end")
