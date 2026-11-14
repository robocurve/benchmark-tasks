"""Jump-start a flat car battery from a donor battery with a pair of jump leads and four sprung clamps.

On the station table stand two full-size 12 V car batteries (BCI group 26R cases, 208 x 173 mm, 197 mm tall
to the top of the posts), long side across the table, posts on the edge nearest the robot. The LEFT one is flat
(the "dead" battery), the RIGHT one is the donor. Each has two tapered lead posts on its grey lid: the + post
(17.5 mm across at the top, 19.0 mm at the foot, 18 mm tall) at the end towards the robot's right, the - post
(15.9 mm / 17.4 mm) at the other end. A flipped-open red (+) or black (-) terminal cover lies just behind each
post. Farther left, at least 15 cm from the flat battery, the flat car's engine ground point: an unpainted steel
stud (12 mm across, 25 mm tall) on a steel bracket bolted to a dark grey engine block.

Two jump leads lie straight across the table in front of the batteries: a red lead (62 cm, nearer the robot) and
a black lead (92 cm, behind it). Each lead ends in a sprung booster clamp at both ends (four clamps). A clamp is two
copper-plated steel halves on a pivot pin with a torsion spring (153 mm long, 19 mm strip, 38 mm maximum jaw
spread, vinyl handle covers in the lead's colour). It is shut at rest; squeezing the two handles together opens the
jaws. Each clamp lies flat on the table, jaws pointing away from the robot.

A clamp is ATTACHED to a post when nothing holds it and its own spring keeps the jaws closed on the post: the post
stands between the two jaws inside their length, the jaws are propped open by it (3 deg or more, short of the
37 deg stop), the jaws overlap the post's height, the clamp is level within 20 deg, and no gripper is within 4 cm.
The safe jump-start order (HSE INDG139) is: (1) red to the flat battery's +, (2) the other red end to the donor's
+, (3) black to the donor's -, (4) the other black end to the engine ground stud, never to the flat battery's -.
The two red connections may be made in either order (both are on the positive side, so neither order can close the
circuit); both must be in place before the black lead goes on the donor's -, and that before the engine stud.
Scoring: see score().
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import cablekit  # noqa: E402,F401
from cablekit import Plug, cable  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

RED = (0.80, 0.06, 0.05)
BLACK = (0.06, 0.06, 0.06)
CASE = (0.13, 0.14, 0.16)
LID = (0.55, 0.57, 0.60)
COPPER = (0.78, 0.45, 0.22)
STEEL = (0.62, 0.62, 0.64)
BARE = (0.74, 0.75, 0.77)
LEAD_GREY = (0.45, 0.46, 0.48)
ENGINE = (0.28, 0.29, 0.31)
WHITE = (0.92, 0.92, 0.90)

# -- batteries: BCI group 26R, 208 L x 173 W x 197 H overall (to the top of the posts) -------------------------------
BAT_L, BAT_W, BAT_H = 0.208, 0.173, 0.197
POST_H = 0.018
LID_TOP = BAT_H - POST_H  # lid surface 179 mm above the table
LID_T = 0.015
POST_INSET = 0.028  # post axis from the front edge and from the end of the case
BATS = {"dead": (0.42, 0.20), "donor": (0.42, -0.20)}  # case centres (x, y); long side along y
POS_R = (0.0095, 0.00875)  # + post radius at the foot, at the top (SAE: 17.5 mm across at the top)
NEG_R = (0.0087, 0.00795)  # - post (SAE: 15.9 mm across at the top)


def _post_xy(bat: str, sign: str):
    cx, cy = BATS[bat]
    x = cx - BAT_W / 2 + POST_INSET
    inner = cy - math.copysign(BAT_L / 2 - POST_INSET, cy)  # post at the end nearer the table's midline
    outer = cy + math.copysign(BAT_L / 2 - POST_INSET, cy)
    return (x, inner if sign == "pos" else outer)  # both + posts at the inner ends


# -- engine ground point: steel stud on a bracket on the engine block ------------------------------------------------
STUD_XY = (0.42, 0.46)
ENG_SIZE = (0.10, 0.10, 0.050)
BRACKET_T = 0.008
STUD_FOOT = ENG_SIZE[2] + BRACKET_T  # 58 mm above the table
STUD_R, STUD_H = 0.006, 0.025

# name -> axis xy, foot z, top z (above the table), radius at foot, radius at top
POSTS = {
    "dead_pos": (_post_xy("dead", "pos"), LID_TOP, BAT_H, *POS_R),
    "dead_neg": (_post_xy("dead", "neg"), LID_TOP, BAT_H, *NEG_R),
    "donor_pos": (_post_xy("donor", "pos"), LID_TOP, BAT_H, *POS_R),
    "donor_neg": (_post_xy("donor", "neg"), LID_TOP, BAT_H, *NEG_R),
    "ground": (STUD_XY, STUD_FOOT, STUD_FOOT + STUD_H, STUD_R, STUD_R),
}
STEPS = [("red", "dead_pos"), ("red", "donor_pos"), ("black", "donor_neg"), ("black", "ground")]
ALLOWED = {"red": {"dead_pos", "donor_pos"}, "black": {"donor_neg", "ground"}}


def _battery(tag: str) -> list:
    cx, cy = BATS[tag]
    out = [Box(f"{tag}_case", pos=(cx, cy, TOP + (LID_TOP - LID_T) / 2), size=(BAT_W, BAT_L, LID_TOP - LID_T),
               color=CASE),
           Box(f"{tag}_lid", pos=(cx, cy, TOP + LID_TOP - LID_T / 2), size=(BAT_W, BAT_L, LID_T), color=LID)]
    for sign, col in (("pos", RED), ("neg", BLACK)):
        x, y = _post_xy(tag, sign)
        # flipped-open terminal cover lying behind the post
        out.append(Box(f"{tag}_{sign}_cover", pos=(x + 0.050, y, TOP + LID_TOP + 0.005), size=(0.030, 0.034, 0.010),
                       color=col))
        # moulded polarity mark on the lid, 4 cm in front of the post (robot side)
        out.append(Box(f"{tag}_{sign}_mark_h", pos=(x + 0.030 - 0.062, y, TOP + LID_TOP + 0.0005),
                       size=(0.004, 0.016, 0.001), color=col if sign == "pos" else BLACK))
        if sign == "pos":
            out.append(Box(f"{tag}_{sign}_mark_v", pos=(x + 0.030 - 0.062, y, TOP + LID_TOP + 0.0005),
                           size=(0.016, 0.004, 0.001), color=col))
    return out


ENGINE_OBJS = [
    Box("engine_block", pos=(STUD_XY[0] + 0.02, STUD_XY[1] + 0.01, TOP + ENG_SIZE[2] / 2), size=ENG_SIZE, color=ENGINE),
    Box("ground_bracket", pos=(STUD_XY[0], STUD_XY[1], TOP + ENG_SIZE[2] + BRACKET_T / 2), size=(0.040, 0.050, BRACKET_T),
        color=BARE),
    Cylinder("ground_stud", pos=(STUD_XY[0], STUD_XY[1], TOP + STUD_FOOT + STUD_H / 2), radius=STUD_R, height=STUD_H,
             color=BARE),
]

# -- clamp geometry (half frame: origin on the pivot pin, +x towards the jaw tips, z up; half A is the +y half) -----
# Sized from a 200 A battery clip datasheet: overall 153 mm, 19 mm wide strip, 38 mm maximum jaw spread.
JH = 0.019  # strip width (vertical when the clamp lies flat)
JAW_T = 0.004
JAW_X0, JAW_X1 = 0.006, 0.060  # jaw from 6 mm to 60 mm in front of the pin
TOOTH = 0.003
H_START = (-0.002, 0.006)  # handle bar centreline, pin end
H_END = (-0.093, 0.036)  # handle end (153 mm overall with the 60 mm jaw)
H_T = 0.008
GRIP_X = -0.055  # where a gripper pinches the handles
OPEN_MAX_DEG = 37.0  # relative opening at the stop: 2 x 18.5 deg -> 38 mm at the jaw tips
SPRING_K = 1.24  # N m/rad on the relative angle
SPRING_PRELOAD = 0.90  # N m with the jaws shut
HALF_MASS = 0.05


def _bar(a, b):
    a, b = np.asarray(a, float), np.asarray(b, float)
    d = b - a
    return (a + b) / 2, float(np.linalg.norm(d)), math.atan2(d[1], d[0])


def half_parts(s: float):
    """(name, centre xyz, size xyz, yaw, look) for one half; s = +1 (A) or -1 (B)."""
    hc, hl, hy = _bar((H_START[0], s * H_START[1]), (H_END[0], s * H_END[1]))
    return [
        ("jaw", ((JAW_X0 + JAW_X1) / 2, s * JAW_T / 2, 0.0), (JAW_X1 - JAW_X0, JAW_T, JH), 0.0, "copper"),
        ("tooth", (JAW_X1 - 0.003, -s * TOOTH / 2, 0.0), (0.006, TOOTH, JH), 0.0, "copper"),
        ("web", (0.0, s * 0.004, 0.0), (0.014, 0.008, JH), 0.0, "copper"),
        ("handle", (hc[0], hc[1], 0.0), (hl, H_T, JH), hy, "vinyl"),
    ]


# metal sample points of one half (half frame): along the jaw centreline, three heights
_JAW_PTS = {s: np.array([(x, s * JAW_T / 2, z) for x in np.linspace(JAW_X0, JAW_X1, 7)
                         for z in (-JH / 2 + 0.002, 0.0, JH / 2 - 0.002)]) for s in (1.0, -1.0)}
# body sample points (for gripper release distance)
_BODY_PTS = {s: np.array([(0.0, 0.0, 0.0), (GRIP_X, s * 0.024, 0.0), (H_END[0], s * H_END[1], 0.0),
                          (-0.03, s * 0.016, 0.0), ((JAW_X0 + JAW_X1) / 2, s * JAW_T / 2, 0.0)]) for s in (1.0, -1.0)}


class Clamp:
    def __init__(self, name: str, colour: str, pivot, weld_half: str):
        self.name, self.colour, self.pivot, self.weld = name, colour, np.asarray(pivot, float), weld_half
        col = STEEL
        self.a = Box(f"{name}_a", pos=tuple(float(v) for v in self.pivot), size=(0.012, 0.012, JH), color=col,
                     mass=HALF_MASS)
        self.b = Box(f"{name}_b", pos=tuple(float(v) for v in self.pivot), size=(0.012, 0.012, JH), color=col,
                     mass=HALF_MASS)

    def world(self, p):
        return self.pivot + np.array([p[0], p[1], p[2] if len(p) > 2 else 0.0])

    @property
    def grip(self):
        return self.world((GRIP_X, 0.0, 0.0))


R_LEAD = 0.005  # 10 mm over the insulation (6 AWG booster cable)
LEAD_KG_PER_M = 0.12
CLAMP_Z = TOP + JH / 2 + 0.0005
FERRULE_GAP = 0.022  # cable tip to the clamp's handle end (ferrule 20 mm + 2 mm)


def _lead(colour: str, x: float, length: float):
    """A lead lying straight along -y at x, centred on y = 0, a clamp at each end (jaws pointing +x)."""
    y0 = length / 2
    he_left = np.array([x, y0 + FERRULE_GAP])  # handle end of the left clamp's B half (the -y half)
    he_right = np.array([x, -y0 - FERRULE_GAP])  # handle end of the right clamp's A half (the +y half)
    left = Clamp(f"{colour}_l", colour, (he_left[0] - H_END[0], he_left[1] + H_END[1], CLAMP_Z), "b")
    right = Clamp(f"{colour}_r", colour, (he_right[0] - H_END[0], he_right[1] - H_END[1], CLAMP_Z), "a")
    col = RED if colour == "red" else BLACK
    ferrule = Plug(size=(0.020, 0.012, 0.012), mass=0.01, color=col)
    seg = 0.03
    spec = cable(f"{colour}_lead", start=(x, y0, TOP + R_LEAD + 0.0005), yaw=-math.pi / 2, length=length,
                 radius=R_LEAD, segment=seg, cable_mass=LEAD_KG_PER_M * length, start_plug=ferrule, end_plug=ferrule,
                 color=col, bend_stiffness=0.01, bend_damping=0.001)
    return spec, left, right


RED_LEAD, RED_L, RED_R = _lead("red", 0.22, 0.62)
BLACK_LEAD, BLACK_L, BLACK_R = _lead("black", 0.13, 0.92)
LEADS = {"red": RED_LEAD, "black": BLACK_LEAD}
CLAMPS = [RED_L, RED_R, BLACK_L, BLACK_R]


def _install_spawn() -> None:
    orig = cablekit._spawn_cable
    if getattr(orig, "_jl_hook", False):
        return

    def spawn(prim_path, cfg, *a, **k):
        prim = orig(prim_path, cfg, *a, **k)
        if getattr(cfg, "cable_name", "") != BLACK_LEAD.name:  # the last object: every other prim exists now
            return prim
        import isaaclab.sim as sim_utils
        from isaaclab.sim.utils import bind_physics_material, bind_visual_material
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics

        from yam_arena.station import linear_rgb

        stage = get_current_stage()
        root = str(prim_path).rsplit("/", 1)[0]
        looks = f"{root}/JLooks"
        mats = {}
        for key, col, rough, met in (("copper", COPPER, 0.35, 0.6), ("red", RED, 0.5, 0.0), ("black", BLACK, 0.5, 0.0),
                                     ("lead", LEAD_GREY, 0.45, 0.3)):
            p = f"{looks}/{key}"
            m = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(col), roughness=rough, metallic=met)
            m.func(p, m)
            mats[key] = p
        jaw_phys = f"{looks}/jaw_phys"
        pm = sim_utils.RigidBodyMaterialCfg(static_friction=0.8, dynamic_friction=0.7)
        pm.func(jaw_phys, pm)
        vinyl_phys = f"{looks}/vinyl_phys"
        pv = sim_utils.RigidBodyMaterialCfg(static_friction=1.2, dynamic_friction=1.0)
        pv.func(vinyl_phys, pv)

        def cube(path, c, s, yaw, look, phys=None, patch=0.0):
            g = UsdGeom.Cube.Define(stage, path)
            g.CreateSizeAttr(1.0)
            g.AddTranslateOp().Set(Gf.Vec3d(*map(float, c)))
            g.AddOrientOp().Set(Gf.Quatf(math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)))
            g.AddScaleOp().Set(Gf.Vec3f(*map(float, s)))
            UsdPhysics.CollisionAPI.Apply(g.GetPrim())
            cp = PhysxSchema.PhysxCollisionAPI.Apply(g.GetPrim())
            cp.CreateContactOffsetAttr(0.002)
            cp.CreateRestOffsetAttr(0.0)
            if patch:
                cp.CreateTorsionalPatchRadiusAttr(patch)
                cp.CreateMinTorsionalPatchRadiusAttr(patch / 2)
            if phys:
                bind_physics_material(str(g.GetPath()), phys, stage=stage)
            bind_visual_material(str(g.GetPath()), look, stage=stage)

        def xf(path):
            return UsdGeom.Xformable(stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(0)

        def filt(p0, p1):
            UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(p0)).CreateFilteredPairsRel().AddTarget(Sdf.Path(p1))

        for c in CLAMPS:
            vinyl = mats["red"] if c.colour == "red" else mats["black"]
            pa, pb = f"{root}/{c.a.name}", f"{root}/{c.b.name}"
            for s, p in ((1.0, pa), (-1.0, pb)):
                for name, ctr, size, yaw, look in half_parts(s):
                    cube(f"{p}/{name}", ctr, size, yaw, mats["copper"] if look == "copper" else vinyl,
                         jaw_phys if look == "copper" else vinyl_phys, patch=0.0 if look == "copper" else 0.008)
            filt(pa, pb)
            j = UsdPhysics.RevoluteJoint.Define(stage, f"{pa}/pivot_pin")
            j.CreateBody0Rel().SetTargets([Sdf.Path(pa)])
            j.CreateBody1Rel().SetTargets([Sdf.Path(pb)])
            j.CreateLocalPos0Attr(Gf.Vec3f(0, 0, 0))
            j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
            j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
            j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
            j.CreateAxisAttr("Z")
            j.CreateLowerLimitAttr(-OPEN_MAX_DEG)
            j.CreateUpperLimitAttr(0.0)
            j.CreateExcludeFromArticulationAttr(True)
            drv = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
            drv.CreateTypeAttr("force")
            drv.CreateStiffnessAttr(SPRING_K * math.pi / 180)  # USD angular gains are per degree
            drv.CreateDampingAttr(0.004 * math.pi / 180)
            drv.CreateTargetPositionAttr(math.degrees(SPRING_PRELOAD / SPRING_K))
        # weld each lead's ferrules to its clamps, at their authored relative pose
        for colour, spec in LEADS.items():
            cp = f"{root}/{spec.name}"
            lc, rc = (RED_L, RED_R) if colour == "red" else (BLACK_L, BLACK_R)
            for end, cl in (("start", lc), ("end", rc)):
                plug = f"{cp}/{end}_plug"
                half = f"{root}/{(cl.b if cl.weld == 'b' else cl.a).name}"
                rel = xf(plug) * xf(half).GetInverse()
                t, q = rel.ExtractTranslation(), rel.ExtractRotationQuat()
                w = UsdPhysics.FixedJoint.Define(stage, f"{cp}/{end}_ferrule_weld")
                w.CreateBody0Rel().SetTargets([Sdf.Path(half)])
                w.CreateBody1Rel().SetTargets([Sdf.Path(plug)])
                w.CreateLocalPos0Attr(Gf.Vec3f(*map(float, t)))
                w.CreateLocalRot0Attr(Gf.Quatf(float(q.GetReal()), *map(float, q.GetImaginary())))
                w.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
                w.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
                w.CreateExcludeFromArticulationAttr(True)
                filt(half, plug)
                other = f"{root}/{(cl.a if cl.weld == 'b' else cl.b).name}"
                filt(other, plug)
        # tapered lead posts (static convex hulls) on the lids
        for name, ((x, y), foot, top, r0, r1) in POSTS.items():
            if name == "ground":
                continue
            bat = f"{root}/{name.split('_')[0]}_lid"
            cx, cy = BATS[name.split("_")[0]]
            pts = []
            for z, r in ((0.0, r0), (top - foot, r1)):
                for k in range(24):
                    a_ = 2 * math.pi * k / 24
                    pts.append(Gf.Vec3f(r * math.cos(a_), r * math.sin(a_), z))
            mesh = UsdGeom.Mesh.Define(stage, f"{bat}/post_{name.split('_')[1]}")
            mesh.CreatePointsAttr(pts)
            idx = list(range(23, -1, -1)) + list(range(24, 48))
            for k in range(24):
                k1 = (k + 1) % 24
                idx += [k, k1, 24 + k1, 24 + k]
            mesh.CreateFaceVertexCountsAttr([24, 24] + [4] * 24)
            mesh.CreateFaceVertexIndicesAttr(idx)
            mesh.AddTranslateOp().Set(Gf.Vec3d(x - cx, y - cy, LID_T / 2))
            UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
            UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("convexHull")
            bind_visual_material(str(mesh.GetPath()), mats["lead"], stage=stage)
            bind_physics_material(str(mesh.GetPath()), jaw_phys, stage=stage)
        print("[jl] 4 sprung clamps, 2 leads welded, 4 tapered posts authored", flush=True)
        return prim

    spawn._jl_hook = True
    cablekit._spawn_cable = spawn


_install_spawn()

OBJECTS = [*_battery("dead"), *_battery("donor"), *ENGINE_OBJS,
           *(h for c in CLAMPS for h in (c.a, c.b)), RED_LEAD, BLACK_LEAD]

TWIN_EXTRA = [
    {"name": "booster_clamp", "kind": "body", "size_mm": [153, 80, 19], "colour": "copper jaws, red/black vinyl handles",
     "note": "x4; 200 A battery clip (153 mm, 38 mm max jaw spread); two halves on a pin with a torsion spring"},
    {"name": "clamp_pivot", "kind": "joint", "real": "riveted pivot pin + torsion spring",
     "note": "0.9 N m preload shut, 1.24 N m/rad, 37 deg stop (~26 N handle squeeze at full open)"},
    {"name": "battery_posts", "kind": "body", "size_mm": [19, 19, 18], "colour": "lead grey",
     "note": "SAE tapered posts: + 17.5 mm top / 19.0 foot, - 15.9 / 17.4; on a real 26R battery"},
    {"name": "ferrule", "kind": "joint", "real": "crimped cable lug inside the handle", "note": "lead fixed to one half"},
]

# -- live state ------------------------------------------------------------------------------------------------------
_ENV = {"env": None}
_EP: dict = {}


def _reset_episode() -> None:
    _EP.clear()
    _EP.update(violation="", events=[], prev=set(), done=[False] * 4, t=0, nan=False)


_reset_episode()


def _install_hooks():
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:
        return
    cls = ie.YamArenaEmbodiment
    if not getattr(cls._task_step, "_jl_hooked", False):
        base = cls._task_step

        def _task_step(self, *a, **k):
            _ENV["env"] = self._env
            return base(self, *a, **k)

        _task_step._jl_hooked = True
        cls._task_step = _task_step
    if getattr(cls.reset, "_task_reset_hooks", None) is None:
        base_reset = cls.reset

        def reset(self, *a, **k):
            for fn in reset._task_reset_hooks:
                fn()
            return base_reset(self, *a, **k)

        reset._task_reset_hooks = []
        cls.reset = reset
    if _reset_episode not in cls.reset._task_reset_hooks:
        cls.reset._task_reset_hooks.append(_reset_episode)


_install_hooks()


def _rot(q):
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _body(name):
    env = _ENV["env"]
    if env is None:
        return None
    base = env.unwrapped
    o = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
    ro = base.scene.rigid_objects[name].data
    p = ro.root_pose_w.torch[0].detach().cpu().numpy().astype(np.float64)
    v = ro.root_lin_vel_w.torch[0].detach().cpu().numpy().astype(np.float64)
    if not (np.isfinite(p).all() and np.isfinite(v).all()):
        return None
    return p[:3] - o, _rot(p[3:7]), v


def grippers():
    env = _ENV["env"]
    if env is None:
        return None
    from yam_arena.runtime import tcp_world
    base = env.unwrapped
    o = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
    return [np.asarray(tcp_world(base.scene[k]), dtype=np.float64) - o for k in ("left_robot", "right_robot")]


def clamp_state(c: Clamp):
    """Live clamp: pivot, mid-frame rotation, opening (deg, + = open), jaw metal points, body points, speed."""
    a, b = _body(c.a.name), _body(c.b.name)
    if a is None or b is None:
        return None
    rel = a[1].T @ b[1]
    opening = -math.degrees(math.atan2(rel[1, 0], rel[0, 0]))
    ha = math.radians(opening) / 2
    cz, sz = math.cos(-ha), math.sin(-ha)  # mid frame = A turned back by half the opening
    Rm = a[1] @ np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]])
    piv = (a[0] + b[0]) / 2
    jaw = np.vstack([a[0] + _JAW_PTS[1.0] @ a[1].T, b[0] + _JAW_PTS[-1.0] @ b[1].T])
    body = np.vstack([a[0] + _BODY_PTS[1.0] @ a[1].T, b[0] + _BODY_PTS[-1.0] @ b[1].T])
    return dict(pivot=piv, R=Rm, opening=opening, jaw=jaw, body=body,
                speed=max(float(np.linalg.norm(a[2])), float(np.linalg.norm(b[2]))))


def _post_gap(pts: np.ndarray, name: str) -> float:
    """Smallest distance from points to the post's solid (truncated cone), minus half a jaw thickness."""
    (x, y), foot, top, r0, r1 = POSTS[name]
    z = pts[:, 2] - TOP
    zc = np.clip(z, foot, top)
    r = r0 + (r1 - r0) * (zc - foot) / (top - foot)
    dr = np.maximum(np.hypot(pts[:, 0] - x, pts[:, 1] - y) - r, 0.0)
    dz = np.abs(z - zc)
    return float(np.hypot(dr, dz).min()) - JAW_T / 2


TOUCH = 0.003  # metal-to-metal "touch" distance
RELEASE = 0.04  # no gripper TCP within 4 cm of a clamp's body points
LEVEL_COS = math.cos(math.radians(20))


def attached_to(c: Clamp, s: dict, grips) -> str | None:
    """Name of the post this clamp's spring holds it on, or None."""
    if s is None or grips is None:
        return None
    if any(float(np.linalg.norm(s["body"] - g, axis=1).min()) < RELEASE for g in grips):
        return None
    if not (3.0 <= s["opening"] <= OPEN_MAX_DEG - 1.0) or s["R"][2, 2] < LEVEL_COS:
        return None
    for name, ((x, y), foot, top, _, _) in POSTS.items():
        d = s["R"].T @ (np.array([x, y, TOP + (foot + top) / 2]) - s["pivot"])
        cz = s["pivot"][2] - TOP
        if (JAW_X0 + 0.004 <= d[0] <= JAW_X1 - 0.002 and abs(d[1]) <= 0.006
                and cz - JH / 2 <= top - 0.005 and cz + JH / 2 >= foot + 0.005):
            return name
    return None


def _track() -> None:
    """Called every control step via score(): attachment events, order, touch rules (all latched)."""
    grips = grippers()
    states = {c.name: clamp_state(c) for c in CLAMPS}
    if grips is None:
        return
    if any(s is None for s in states.values()):
        _EP["nan"] = True
        return
    att = {}
    for c in CLAMPS:
        p = attached_to(c, states[c.name], grips)
        if p is not None:
            att[c.name] = (c.colour, p)
    sat = [any(v == st for v in att.values()) for st in STEPS]
    v = _EP["violation"]
    # wrong terminal for the colour (red on a - post or the ground; black on a + post or the flat battery's -)
    for c in CLAMPS:
        for name in POSTS:
            if name not in ALLOWED[c.colour] and _post_gap(states[c.name]["jaw"], name) <= TOUCH:
                v = v or f"{c.name} jaw touched {name}"
    # metal of a red clamp touching metal of a black clamp while any clamp is attached
    if att:
        for r in (RED_L, RED_R):
            for b in (BLACK_L, BLACK_R):
                dd = np.linalg.norm(states[r.name]["jaw"][:, None, :] - states[b.name]["jaw"][None, :, :], axis=2)
                if float(dd.min()) <= JAW_T + TOUCH:
                    v = v or f"{r.name} touched {b.name} while live"
    # each connection must be made with exactly the earlier ones in place; the two red (+) connections, steps 1
    # and 2, may come in either order, but both before either black one
    for k, st in enumerate(STEPS):
        was = _EP["prev_sat"][k] if "prev_sat" in _EP else False
        if sat[k] and not was:
            if k < 2:
                ok = not any(sat[2:])
            else:
                ok = all(sat[:k]) and not any(sat[k + 1:])
            _EP["events"].append((round(_EP["t"] / 20.0, 2), k + 1, ok))
            if not ok:
                v = v or f"step {k + 1} {st} connected out of order"
            _EP["done"][k] = True
    _EP["prev_sat"] = sat
    _EP["att"] = att
    # settled: every pivot moved under 2 cm over the last 20 control steps (1 s at 20 Hz). Contact jitter makes the
    # instantaneous body velocity of a spring-loaded clamp noisy, so the pose drift over a second is used instead.
    hist = _EP.setdefault("piv_hist", [])
    hist.append(np.concatenate([states[c.name]["pivot"] for c in CLAMPS]))
    del hist[:-21]
    if len(hist) == 21:
        _EP["speed"] = max(float(np.linalg.norm(hist[-1][i:i + 3] - hist[0][i:i + 3])) for i in range(0, len(hist[0]), 3))
    else:
        _EP["speed"] = 1.0
    _EP["violation"] = v
    _EP["t"] += 1


def score(objects) -> float:
    _track()
    if _EP["violation"] or _EP["nan"] or "prev_sat" not in _EP:
        return 0.0
    sat = _EP["prev_sat"]
    n = int(sat[0]) + int(sat[1])  # red connections, either order
    if n == 2:
        for k in (2, 3):
            if not sat[k]:
                break
            n += 1
    return 0.25 * n


def success(objects) -> bool:
    return score(objects) >= 1.0 and _EP.get("speed", 1.0) < 0.02


def distance(objects) -> float:
    return 1.0 - score(objects)


def progress(objects) -> dict:
    sat = _EP.get("prev_sat", [False] * 4)
    return {"red_dead_pos": int(sat[0]), "red_donor_pos": int(sat[1]), "black_donor_neg": int(sat[2]),
            "black_ground": int(sat[3]), "no_violation": int(not _EP.get("violation"))}


VARIATION = {"cable:red_lead": {"dx": 0.0, "dy": 0.0, "yaw": 0.0}, "cable:black_lead": {"dx": 0.0, "dy": 0.0, "yaw": 0.0}}
