"""Nail station on the station table. A life-size silicone nail-practice hand (a right hand, palm down on a folded
towel, fingers spread and pointing towards the robot, thumb on the robot's right, held by its table clamp) lies in
front of the right arm. A 13.5 mL bottle of red nail polish stands open-cap-down... no: closed, its cap-brush
seated in the neck, in a rubber bottle holder taped to the table near the robot.

The task: paint all five nails red, keep polish off the skin, put the brush back in the bottle and park both arms.
The brush is the bottle's cap: a 16 mm cap with a 3 mm stem and a flat 6 mm wide, 10 mm long bristle tuft (one rigid
body, all of it colliding). Dipping the tuft into the polish (below the polish level inside the bottle) loads it with
about one nail's worth (160 mm2); every newly coated mm2 uses that up. A nail cell is coated only while the physics engine
reports the loaded tuft pressing on that nail (contact force over 0.02 N), round the points of its lowest edge that
lie on the plate (within 0.4 mm: the bristles splay); once the brush has been dipped, the same edge
within 0.6 mm of skin more than 1 mm outside a nail's edge means polish on the skin (score 0).

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score --instruction "<INSTRUCTION>"
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import os
import sys

import numpy as np
from yam_arena.task_file import TABLE_TOP, Box, Cylinder, Sphere

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = ("Paint all five fingernails of the practice hand with the red polish, keeping it off the skin, "
               "then put the brush back in the bottle.")

SKIN = (0.90, 0.72, 0.60)
NAIL_COL = (0.96, 0.86, 0.84)  # bare nail: pale pink
POLISH = (0.80, 0.03, 0.08)  # red lacquer
TOWEL_COL = (0.95, 0.95, 0.93)
CAP_COL = (0.06, 0.06, 0.06)
GLASS_RED = (0.70, 0.05, 0.10)  # the bottle's body seen with red polish inside
HOLDER_COL = (0.25, 0.25, 0.27)
STEM_COL = (0.85, 0.85, 0.85)

# -- folded towel under the hand ---------------------------------------------------------------------------------------
TOWEL_T = 0.006
TW = TOP + TOWEL_T
TOWEL = Box("towel", pos=(0.355, -0.150, TOP + TOWEL_T / 2), size=(0.26, 0.20, TOWEL_T), color=TOWEL_COL)

# -- the practice hand (fixed, held by its clamp): right hand, palm down, fingers towards -x, thumb towards -y ----------
KNUCKLE_X = 0.335  # knuckle line (finger bases)
PALM_X0, PALM_X1 = KNUCKLE_X, 0.425  # palm from knuckles to wrist crease (9 cm)
PALM_Y0, PALM_Y1 = -0.188, -0.102  # 8.6 cm across
PALM_T = 0.022
PALM = Box("hand_palm", pos=((PALM_X0 + PALM_X1) / 2, (PALM_Y0 + PALM_Y1) / 2, TW + PALM_T / 2),
           size=(PALM_X1 - PALM_X0, PALM_Y1 - PALM_Y0, PALM_T), color=SKIN)
WRIST_R, WRIST_L = 0.022, 0.120  # wrist stub: the hand is 28.4 cm overall (middle fingertip to wrist end)
WRIST = Cylinder("hand_wrist", pos=(PALM_X1 + WRIST_L / 2 - 0.004, (PALM_Y0 + PALM_Y1) / 2, TW + WRIST_R),
                 radius=WRIST_R, height=WRIST_L, color=SKIN)

# fingers: (name, base x, base y, heading of the finger (rad, world yaw it points towards), radius, length
#           knuckle->tip, nail length, nail width). Adult female sizes (nail widths 8.5-12.5 mm).
FINGERS = [
    ("thumb", 0.405, -0.184, math.pi + 0.62, 0.0095, 0.066, 0.0115, 0.0125),
    ("index", KNUCKLE_X + 0.004, -0.172, math.pi, 0.0080, 0.074, 0.0110, 0.0105),
    ("middle", KNUCKLE_X + 0.004, -0.151, math.pi, 0.0083, 0.082, 0.0115, 0.0110),
    ("ring", KNUCKLE_X + 0.004, -0.131, math.pi, 0.0080, 0.077, 0.0110, 0.0105),
    ("little", KNUCKLE_X + 0.004, -0.112, math.pi + 0.08, 0.0068, 0.062, 0.0090, 0.0085),
]
NAIL_T = 0.003  # nail plate box: embedded in the finger, its top NAIL_PROUD above the finger's top line
NAIL_PROUD = 0.0012
FREE_EDGE_IN = 0.6  # the nail's free edge sits this many finger radii back from the fingertip's end


def _yq(yaw):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def _qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def _rotmat_xyzw(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


Q_Y90 = (0.0, math.sin(math.pi / 4), 0.0, math.cos(math.pi / 4))  # cylinder axis z -> x
ROT: dict = {"hand_wrist": Q_Y90}


class Finger:
    """Geometry of one finger: a horizontal cylinder from its base to a rounded tip, the nail on top near the tip."""

    def __init__(self, name, bx, by, heading, r, length, nl, nw):
        self.name, self.r, self.length, self.nl, self.nw = name, r, length, nl, nw
        self.d = np.array([math.cos(heading), math.sin(heading)])  # base -> tip (unit, xy)
        self.n = np.array([-self.d[1], self.d[0]])  # across the finger
        self.base = np.array([bx, by])
        self.heading = heading
        self.zc = TW + r  # axis height (finger lies on the towel)
        self.tip_c = self.base + (length - r) * self.d  # centre of the tip sphere
        self.cyl_len = length - r
        self.cyl_c = self.base + self.cyl_len / 2 * self.d
        free = self.tip_c + (r - FREE_EDGE_IN * r) * self.d  # nail free edge, along the finger
        self.nail_c = free - nl / 2 * self.d
        self.nail_top = self.zc + r + NAIL_PROUD
        self.nail_yaw = heading

    def nail_local(self, xy):
        """(along, across) of a world xy relative to the nail centre (along = towards the fingertip)."""
        v = np.asarray(xy, float)[..., :2] - self.nail_c
        return v @ self.d, v @ self.n

    def outside_nail(self, xy) -> float:
        """How far (m) a world xy lies outside the nail's outline (0 inside)."""
        a, c = self.nail_local(xy)
        return float(np.hypot(max(abs(a) - self.nl / 2, 0.0), max(abs(c) - self.nw / 2, 0.0)))

    def skin_z(self, xy):
        """Height of the finger's skin surface above world xy, or None if xy is not over this finger."""
        v = np.asarray(xy, float)[:2] - self.base
        a, c = float(v @ self.d), float(v @ self.n)
        best = None
        if 0.0 <= a <= self.cyl_len and abs(c) < self.r:
            best = self.zc + math.sqrt(self.r ** 2 - c ** 2)
        w = np.asarray(xy, float)[:2] - self.tip_c
        rr = float(w @ w)
        if rr < self.r ** 2:
            z = self.zc + math.sqrt(self.r ** 2 - rr)
            best = z if best is None else max(best, z)
        return best


HAND = [Finger(*f) for f in FINGERS]
FINGER_BY_NAME = {f.name: f for f in HAND}


def _hand_parts() -> list:
    out = [PALM, WRIST]
    for f in HAND:
        cyl = Cylinder(f"finger_{f.name}", pos=(float(f.cyl_c[0]), float(f.cyl_c[1]), f.zc), radius=f.r,
                       height=f.cyl_len, color=SKIN)
        ROT[cyl.name] = _qmul(_yq(f.heading), Q_Y90)
        out.append(cyl)
        out.append(Sphere(f"fingertip_{f.name}", pos=(float(f.tip_c[0]), float(f.tip_c[1]), f.zc), radius=f.r,
                          color=SKIN))
        out.append(Box(f"nail_{f.name}", pos=(float(f.nail_c[0]), float(f.nail_c[1]), f.nail_top - NAIL_T / 2),
                       size=(f.nl, f.nw, NAIL_T), yaw=f.heading, color=NAIL_COL))
    return out


# -- bottle (fixed, in a holder taped to the table): 29 x 29 mm glass body, 50 mm tall, short neck --------------------
BOT_C = np.array([0.215, -0.255])
BOT_W, BOT_H = 0.029, 0.050
BORE = 0.0085  # square bore down the bottle's centre (the brush's way into the polish)
NECK_W, NECK_H = 0.014, 0.007
NECK_TOP = TOP + BOT_H + NECK_H
FLOOR_T = 0.003
POLISH_Z = TOP + 0.036  # polish level inside the bottle
HOLDER_H = 0.012


def _bottle_parts() -> list:
    out = []
    zc = TOP + FLOOR_T + (BOT_H - FLOOR_T) / 2
    hz = BOT_H - FLOOR_T
    out.append(Box("bottle_floor", pos=(float(BOT_C[0]), float(BOT_C[1]), TOP + FLOOR_T / 2), size=(BOT_W, BOT_W, FLOOR_T),
                   color=GLASS_RED))
    side = (BOT_W - BORE) / 2
    for nm, dx, dy, sx, sy in (("n", -1, 0, side, BOT_W), ("f", 1, 0, side, BOT_W),
                               ("l", 0, 1, BORE, side), ("r", 0, -1, BORE, side)):
        off = (BORE + side) / 2
        out.append(Box(f"bottle_wall_{nm}", pos=(float(BOT_C[0] + dx * off), float(BOT_C[1] + dy * off), zc),
                       size=(sx, sy, hz), color=GLASS_RED))
    nside = (NECK_W - BORE) / 2
    nz = TOP + BOT_H + NECK_H / 2
    for nm, dx, dy, sx, sy in (("n", -1, 0, nside, NECK_W), ("f", 1, 0, nside, NECK_W),
                               ("l", 0, 1, BORE, nside), ("r", 0, -1, BORE, nside)):
        off = (BORE + nside) / 2
        out.append(Box(f"bottle_neck_{nm}", pos=(float(BOT_C[0] + dx * off), float(BOT_C[1] + dy * off), nz),
                       size=(sx, sy, NECK_H), color=(0.92, 0.92, 0.92)))
    # rubber bottle holder ring round the base (scenery, taped down)
    hw = BOT_W + 0.016
    for nm, dx, dy, sx, sy in (("n", -1, 0, 0.008, hw), ("f", 1, 0, 0.008, hw),
                               ("l", 0, 1, BOT_W, 0.008), ("r", 0, -1, BOT_W, 0.008)):
        off = (BOT_W + 0.008) / 2
        out.append(Box(f"holder_{nm}", pos=(float(BOT_C[0] + dx * off), float(BOT_C[1] + dy * off), TOP + HOLDER_H / 2),
                       size=(sx, sy, HOLDER_H), color=HOLDER_COL))
    return out


# -- brush (free): the bottle's cap with its stem and bristle tuft, one rigid body --------------------------------------
CAP_R, CAP_H = 0.008, 0.036
STEM_R, STEM_L = 0.0015, 0.042
TUFT_W, TUFT_T, TUFT_L = 0.006, 0.002, 0.010  # bristle tuft: width, thickness, length
BRUSH_MASS = 0.008
CAP_Z0 = NECK_TOP + CAP_H / 2 + 0.0003  # cap seated on the neck
STEM_LOCAL = -CAP_H / 2 - STEM_L / 2
TUFT_LOCAL = -CAP_H / 2 - STEM_L - TUFT_L / 2


class BrushCap(Cylinder):
    """The brush cap; its spawner adds the stem and tuft colliders, and the polish decals on the nails."""


BRUSH = BrushCap("nail_brush", pos=(float(BOT_C[0]), float(BOT_C[1]), CAP_Z0), radius=CAP_R, height=CAP_H,
                 mass=BRUSH_MASS, color=CAP_COL)


def _pyfloats(o):
    import dataclasses

    return dataclasses.replace(o, pos=tuple(float(v) for v in o.pos))


OBJECTS = [_pyfloats(o) for o in (TOWEL, *_hand_parts(), *_bottle_parts(), BRUSH)]
BRUSH = OBJECTS[-1]

TWIN_EXTRA = [
    {"name": "brush_stem_tuft", "kind": "body", "size_mm": [6, 2, 52], "colour": "white",
     "note": "the cap's 3 mm stem and 6 x 2 x 10 mm bristle tuft (part of the cap body)"},
    {"name": "hand_clamp", "kind": "joint", "real": "the practice hand's table clamp", "note": "hand fixed"},
]

# -- scoring constants -------------------------------------------------------------------------------------------------
CELL = 0.0005  # coverage grid on each nail (0.5 mm)
COAT_FORCE = 0.02  # N: the physics engine must report the tuft pressing on a nail (contact) for it to coat ...
COAT_SPLAY = 0.0004  # ... and then the lowest edge's points within this of the plate are the bristles touching it
OFFLINE_GAP = 0.0001  # offline checks (no physics engine): contact = the lowest edge within this of the plate
COAT_R = 0.0012  # ... over this radius round each sampled point of that edge (bristles splay)
SKIN_GAP = 0.0006  # the tuft's lowest edge this close to skin = polish on the skin
SKIN_MARGIN = 0.001  # ... if more than this outside every nail outline
LOAD_MM2 = 160.0  # one dip coats about this much nail (mm2)
COVER_FULL, COVER_HALF = 0.90, 0.50
NAIL_FULL, NAIL_HALF, PARK_PTS = 0.18, 0.09, 0.10
CLEAR = 0.10  # parked grippers this far from the brush, bottle and hand
STILL = 0.02
PARK_HOLD = 0.5
VIS_CELL = 0.002  # decal squares on the nails (shown when 60% of their fine cells are coated)


def _grid(f: Finger):
    na, nc = int(round(f.nl / CELL)), int(round(f.nw / CELL))
    a = (np.arange(na) + 0.5) * CELL - f.nl / 2
    c = (np.arange(nc) + 0.5) * CELL - f.nw / 2
    return a, c


GRIDS = {f.name: _grid(f) for f in HAND}


def _vis_grid(f: Finger):
    na, nc = max(1, int(round(f.nl / VIS_CELL))), max(1, int(round(f.nw / VIS_CELL)))
    return na, nc, f.nl / na, f.nw / nc


def _decal_points(f: Finger, shown: set):
    """Mesh points (nail-local) for a nail's decal squares: opened out where shown, collapsed elsewhere."""
    from pxr import Gf, Vt

    na, nc, da, dc = _vis_grid(f)
    z = NAIL_T / 2 + 0.0002
    pts = []
    for i in range(na):
        for j in range(nc):
            cx, cy = -f.nl / 2 + (i + 0.5) * da, -f.nw / 2 + (j + 0.5) * dc
            hx, hy = (da * 0.52, dc * 0.52) if (i, j) in shown else (0.0, 0.0)
            pts += [Gf.Vec3f(cx - hx, cy - hy, z), Gf.Vec3f(cx + hx, cy - hy, z), Gf.Vec3f(cx + hx, cy + hy, z),
                    Gf.Vec3f(cx - hx, cy + hy, z)]
    return Vt.Vec3fArray(pts)


# -- Isaac side: custom spawner for the brush ---------------------------------------------------------------------------
_P: dict = {}


def _author_extras(stage, root: str, brush_path: str) -> None:
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_visual_material
    from pxr import Gf, UsdGeom, UsdPhysics
    from yam_arena.station import linear_rgb

    def look(name, col):
        path = f"{root}/Looks_{name}"
        c = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(col), roughness=0.25)
        c.func(path, c)
        return path

    m_stem, m_tuft, m_pol = look("stem", STEM_COL), look("tuft", POLISH), look("polish", POLISH)
    from pxr import PhysxSchema

    PhysxSchema.PhysxContactReportAPI.Apply(stage.GetPrimAtPath(brush_path)).CreateThresholdAttr(0.0)
    try:
        import carb

        carb.settings.get_settings().set_bool("/physics/disableContactProcessing", False)
    except Exception:  # noqa: BLE001
        pass
    # stem and tuft: colliders of the brush body
    s = UsdGeom.Cylinder.Define(stage, f"{brush_path}/stem")
    s.CreateRadiusAttr(STEM_R)
    s.CreateHeightAttr(STEM_L)
    s.CreateAxisAttr("Z")
    UsdGeom.Xformable(s).AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, STEM_LOCAL))
    UsdPhysics.CollisionAPI.Apply(s.GetPrim())
    bind_visual_material(f"{brush_path}/stem", m_stem, stage=stage)
    t = UsdGeom.Cube.Define(stage, f"{brush_path}/tuft")
    t.CreateSizeAttr(1.0)
    x = UsdGeom.Xformable(t)
    x.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, TUFT_LOCAL))
    x.AddScaleOp().Set(Gf.Vec3f(TUFT_T, TUFT_W, TUFT_L))  # thin along local x, wide along local y
    UsdPhysics.CollisionAPI.Apply(t.GetPrim())
    bind_visual_material(f"{brush_path}/tuft", m_tuft, stage=stage)
    # polish decals on every nail: one mesh per nail with a quad per decal square; a square not yet coated is
    # collapsed to a point (zero area), a coated one is opened out (mesh points are rewritten as the coat grows)
    _P["decals"] = {}
    for f in HAND:
        na, nc, da, dc = _vis_grid(f)
        p = f"{root}/nail_{f.name}/polish"
        m = UsdGeom.Mesh.Define(stage, p)
        m.CreateFaceVertexCountsAttr([4] * (na * nc))
        m.CreateFaceVertexIndicesAttr(list(range(4 * na * nc)))
        m.CreateDoubleSidedAttr(True)
        m.CreateSubdivisionSchemeAttr("none")
        m.CreatePointsAttr(_decal_points(f, set()))
        bind_visual_material(p, m_pol, stage=stage)
        _P["decals"][f.name] = p
    if os.environ.get("NAIL_DEBUG"):
        print(f"[nails] extras authored under {root}", flush=True)


def _spawn_brush(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    from isaaclab.sim.spawners.shapes import spawn_cylinder
    from isaaclab.sim.utils.stage import get_current_stage

    prim = spawn_cylinder(prim_path, cfg, translation, orientation, **kwargs)
    _author_extras(get_current_stage(), str(prim_path).rsplit("/", 1)[0], str(prim_path))
    return prim


def _brush_object(spec: BrushCap):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.schemas import MassPropertiesCfg, PhysxRigidBodyPropertiesCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose
    from yam_arena import scene as ysc
    from yam_arena.station import linear_rgb

    _register_module()

    @configclass
    class BrushCfg(sim_utils.CylinderCfg):
        func: object = clone(_spawn_brush)

    cfg = BrushCfg(radius=spec.radius, height=spec.height, axis="Z",
                   rigid_props=PhysxRigidBodyPropertiesCfg(solver_position_iteration_count=16),
                   mass_props=MassPropertiesCfg(mass=spec.mass), collision_props=ysc._contact_props(),
                   physics_material=ysc._grippy(),
                   visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(spec.color), roughness=0.3))
    return Object(name=spec.name, object_type=ObjectType.RIGID, spawner_cfg=cfg,
                  initial_pose=Pose(position_xyz=tuple(spec.pos), rotation_xyzw=(0.0, 0.0, 0.0, 1.0)))


def _register_module():
    if __name__ not in sys.modules:
        import types

        mod = types.ModuleType(__name__)
        mod.__dict__.update(globals())
        sys.modules[__name__] = mod


def _patch_scene(scene) -> None:
    if getattr(scene, "_nails_patched", False):
        return
    orig = scene.task_object

    def task_object(spec):
        if isinstance(spec, BrushCap):
            return _brush_object(spec)
        q = ROT.get(spec.name)
        if q is None:
            return orig(spec)
        yaw_q = scene._yaw_xyzw
        scene._yaw_xyzw = lambda _yaw: q
        try:
            return orig(spec)
        finally:
            scene._yaw_xyzw = yaw_q

    scene.task_object = task_object
    scene._nails_patched = True


class _Finder(importlib.abc.MetaPathFinder):
    _nails = True

    def find_spec(self, fullname, path, target=None):
        if fullname != "yam_arena.scene":
            return None
        sys.meta_path.remove(self)
        try:
            spec = importlib.util.find_spec(fullname)
        finally:
            sys.meta_path.insert(0, self)
        if spec is None or spec.loader is None:
            return None
        orig_exec = spec.loader.exec_module

        def exec_module(module):
            orig_exec(module)
            _patch_scene(module)

        spec.loader.exec_module = exec_module
        return spec


def _install() -> None:
    if "yam_arena.scene" in sys.modules:
        _patch_scene(sys.modules["yam_arena.scene"])
    elif not any(getattr(f, "_nails", False) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


_install()

# -- episode state -------------------------------------------------------------------------------------------------------
_ENV: dict = {"env": None}


class _Episode:
    def __init__(self):
        self.coat = {f.name: np.zeros((len(GRIDS[f.name][0]), len(GRIDS[f.name][1])), bool) for f in HAND}
        self.shown = {f.name: set() for f in HAND}
        self.load = 0.0  # mm2 of polish left on the brush
        self.dipped = False
        self.skin = False
        self.why = ""
        self.brush_pos = np.array([BOT_C[0], BOT_C[1], CAP_Z0])
        self.brush_R = np.eye(3)
        self.prev_pts = None
        self.parked_s = 0.0
        self.t = 0.0
        self.last = None


_EP = _Episode()


def _hide_all() -> None:
    if not _P.get("decals"):  # nothing spawned yet (and never import pxr before the app is up)
        return
    try:
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import UsdGeom

        st = get_current_stage()
        for f in HAND:
            pr = st.GetPrimAtPath(_P["decals"][f.name])
            if pr and pr.IsValid():
                UsdGeom.Mesh(pr).GetPointsAttr().Set(_decal_points(f, set()))
    except Exception:  # noqa: BLE001
        pass


def reset_episode() -> None:
    global _EP
    _EP = _Episode()
    _hide_all()


def _brush_pose_live():
    env = _ENV["env"]
    if env is None:
        return None
    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
    p = base.arena_world.get_pose_w(BRUSH.name)[0].detach().cpu().numpy().astype(np.float64)
    return p[:3] - origin, _rotmat_xyzw(p[3:7])


_CV: dict = {"view": None, "err": None, "names": None}


def _contact_view():
    """PhysX contact view: the brush body against each nail plate (built on first use)."""
    if _CV["view"] is not None or _CV["err"] is not None:
        return _CV["view"]
    try:
        from isaaclab.sim.utils.stage import get_current_stage
        from isaaclab_physx.physics.physx_manager import PhysxManager
        from pxr import UsdPhysics

        sv = PhysxManager.get_physics_sim_view()
        stage = get_current_stage()
        brush, nails = None, {}
        for prim in stage.Traverse():
            p = prim.GetPath().pathString
            if "/env_0/" not in p:
                continue
            if p.endswith(f"/{BRUSH.name}") and prim.HasAPI(UsdPhysics.RigidBodyAPI):
                brush = p
            for f in HAND:
                if f"/nail_{f.name}/" in p + "/" and prim.HasAPI(UsdPhysics.CollisionAPI) and f.name not in nails:
                    q = prim
                    while q and q.IsValid() and not q.HasAPI(UsdPhysics.RigidBodyAPI):
                        q = q.GetParent()
                    nails[f.name] = q.GetPath().pathString if q and q.IsValid() and q.HasAPI(UsdPhysics.RigidBodyAPI) else p
        if brush is None or len(nails) < len(HAND):
            raise RuntimeError(f"contact view: brush={brush} nails={sorted(nails)}")
        names = [f.name for f in HAND]
        _CV["view"] = sv.create_rigid_contact_view([brush], filter_patterns=[[nails[n] for n in names]],
                                                   max_contact_data_count=64)
        _CV["names"] = names
        print(f"[nails] contact view: {brush} x {len(names)} nails", flush=True)
    except Exception as e:  # noqa: BLE001
        _CV["err"] = repr(e)
        print(f"[nails] contact view failed: {e!r}", flush=True)
    return _CV["view"]


def _touching_live():
    """Names of the nails the brush presses on with more than COAT_FORCE (empty if the view is unavailable)."""
    v = _contact_view()
    if v is None:
        return set()
    from isaaclab_physx.physics.physx_manager import PhysxManager

    fm = v.get_contact_force_matrix(dt=PhysxManager.get_physics_dt())
    if hasattr(fm, "detach"):
        fm = fm.detach().cpu().numpy()
    else:  # a warp array (GPU physics)
        fm = np.asarray(fm.numpy() if hasattr(fm, "numpy") else fm)
    fm = fm.reshape(len(_CV["names"]), 3)
    return {n for n, fv in zip(_CV["names"], fm) if float(np.linalg.norm(fv)) > COAT_FORCE}


def _install_hooks() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # noqa: BLE001  (offline)
        return
    cls = ie.YamArenaEmbodiment
    if not getattr(cls.reset, "_nails_hooked", False):
        orig = cls.reset

        def reset(self, *a, **k):
            reset_episode()
            return orig(self, *a, **k)

        reset._nails_hooked = True
        cls.reset = reset
    if not getattr(cls._task_step, "_nails_hooked", False):
        base = cls._task_step

        def _task_step(self, *a, **k):
            _ENV["env"] = self._env
            out = base(self, *a, **k)
            try:
                pose = _brush_pose_live()
                if pose is not None:
                    step(pose[0], pose[1], float(getattr(getattr(self._env, "unwrapped", self._env), "step_dt", 0.05)),
                         touching=_touching_live())
            except Exception as e:  # noqa: BLE001
                print(f"[nails] step hook failed: {e!r}", flush=True)
            return out

        _task_step._nails_hooked = True
        cls._task_step = _task_step


_install_hooks()


# -- brush geometry ----------------------------------------------------------------------------------------------------
def tuft_corners(pos, R) -> np.ndarray:
    """The 8 corners of the bristle tuft in world coordinates."""
    c = np.asarray(pos, float) + R @ np.array([0.0, 0.0, TUFT_LOCAL])
    h = np.array([TUFT_T, TUFT_W, TUFT_L]) / 2
    return np.array([c + R @ (h * np.array(s)) for s in
                     [(a, b, d) for a in (-1, 1) for b in (-1, 1) for d in (-1, 1)]])


def tuft_edge(pos, R, n: int = 7) -> np.ndarray:
    """Sample points along the tuft's lowest edge (the two lowest corners that lie across the tuft's width)."""
    c = np.asarray(pos, float) + R @ np.array([0.0, 0.0, TUFT_LOCAL])
    h = np.array([TUFT_T, TUFT_W, TUFT_L]) / 2
    best = None
    for sa in (-1, 1):
        for sd in (-1, 1):
            m = c + R @ np.array([sa * h[0], 0.0, sd * h[2]])  # mid-point of an edge running across the width
            if best is None or m[2] < best[2]:
                best = m
    w = R @ np.array([0.0, 1.0, 0.0]) * h[1]
    return np.array([best + s * w for s in np.linspace(-1, 1, n)])


def _skin_z(xy):
    """Height of any skin surface (palm, wrist, fingers, fingertips) under world xy, or None."""
    best = None
    for f in HAND:
        z = f.skin_z(xy)
        if z is not None:
            best = z if best is None else max(best, z)
    if PALM_X0 <= xy[0] <= PALM_X1 and PALM_Y0 <= xy[1] <= PALM_Y1:
        best = max(best or 0.0, TW + PALM_T)
    wy = (PALM_Y0 + PALM_Y1) / 2
    if PALM_X1 - 0.004 <= xy[0] <= PALM_X1 + WRIST_L and abs(xy[1] - wy) < WRIST_R:
        best = max(best or 0.0, TW + WRIST_R + math.sqrt(WRIST_R ** 2 - (xy[1] - wy) ** 2))
    return best


def in_bottle(p) -> bool:
    return bool(abs(p[0] - BOT_C[0]) <= BORE / 2 and abs(p[1] - BOT_C[1]) <= BORE / 2 and p[2] < NECK_TOP)


def _coat_at(pt, touching) -> float:
    """Coat nail cells round one edge point on the nails the tuft is pressed on; returns the newly coated area (mm2)."""
    new = 0.0
    for f in HAND:
        if f.name not in touching or pt[2] - f.nail_top > COAT_SPLAY or pt[2] < f.nail_top - 0.004:
            continue
        a0, c0 = f.nail_local(pt)
        if abs(a0) > f.nl / 2 + COAT_R or abs(c0) > f.nw / 2 + COAT_R:
            continue
        ga, gc = GRIDS[f.name]
        ia = np.where(np.abs(ga - a0) <= COAT_R)[0]
        ic = np.where(np.abs(gc - c0) <= COAT_R)[0]
        if not len(ia) or not len(ic):
            continue
        da = (ga[ia][:, None] - a0) ** 2 + (gc[ic][None, :] - c0) ** 2 <= COAT_R ** 2
        sub = _EP.coat[f.name][np.ix_(ia, ic)]
        fresh = da & ~sub
        k = int(fresh.sum())
        if k:
            cap = int(_EP.load / (CELL * CELL * 1e6))
            if cap <= 0:
                return new
            if k > cap:  # brush runs dry part-way: coat only what is left
                idx = np.argwhere(fresh)[:cap]
                fresh = np.zeros_like(fresh)
                fresh[idx[:, 0], idx[:, 1]] = True
                k = cap
            sub |= fresh
            _EP.coat[f.name][np.ix_(ia, ic)] = sub
            _EP.load -= k * CELL * CELL * 1e6
            new += k * CELL * CELL * 1e6
    return new


def _on_skin(pt) -> bool:
    if any(f.outside_nail(pt) <= SKIN_MARGIN for f in HAND):
        return False
    z = _skin_z(pt)
    return z is not None and pt[2] - z <= SKIN_GAP


def step(pos, R, dt: float = 0.05, pts=None, touching=None) -> None:
    """Advance the episode by one control step with the brush at (pos, R). ``touching``: names of the nails the
    physics engine reports the brush pressing on this step (None offline: then the lowest edge must be within
    OFFLINE_GAP of a nail's plate)."""
    pos = np.asarray(pos, float)
    if not (np.isfinite(pos).all() and np.isfinite(R).all()):
        _EP.skin, _EP.why = True, "brush state not finite"
        return
    prev = _EP.brush_pos
    _EP.brush_pos, _EP.brush_R = pos, np.asarray(R, float)
    _EP.t += dt
    # interpolate between the last pose and this one so fast strokes don't skip cells
    nsub = int(min(10, max(1, math.ceil(float(np.linalg.norm(pos - prev)) / 0.0008))))
    for s in np.linspace(1.0 / nsub, 1.0, nsub):
        p = prev + s * (pos - prev)
        edge = tuft_edge(p, R)
        if any(in_bottle(q) and q[2] < POLISH_Z for q in edge):
            _EP.load, _EP.dipped = LOAD_MM2, True
        if touching is None:
            touch = {f.name for f in HAND for q in edge
                     if abs(q[2] - f.nail_top) <= OFFLINE_GAP and f.outside_nail(q) <= 0.0}
        else:
            touch = touching
        if _EP.dipped:
            for q in edge:
                if not _EP.skin and _on_skin(q):
                    _EP.skin, _EP.why = True, f"polish on the skin at {np.round(q, 4)}"
                if _EP.load > 0 and touch:
                    _coat_at(q, touch)
    _show()
    if _ENV["env"] is not None or pts is not None:
        pts = gripper_points() if pts is None else pts
        still = False
        if _EP.prev_pts is not None and len(pts) == len(_EP.prev_pts) and dt > 0:
            v = max(float(np.linalg.norm(a - b)) for a, b in zip(pts, _EP.prev_pts)) / dt
            still = v < STILL
        _EP.prev_pts = pts
        _EP.parked_s = _EP.parked_s + dt if (still and released(pts)) else 0.0


def _show() -> None:
    if not _P.get("decals"):
        return
    try:
        changed = []
        for f in HAND:
            na, nc, da, dc = _vis_grid(f)
            cov = _EP.coat[f.name]
            ga, gc = GRIDS[f.name]
            new = False
            for i in range(na):
                ma = (ga >= -f.nl / 2 + i * da - 1e-9) & (ga < -f.nl / 2 + (i + 1) * da + 1e-9)
                for j in range(nc):
                    if (i, j) in _EP.shown[f.name]:
                        continue
                    mc = (gc >= -f.nw / 2 + j * dc - 1e-9) & (gc < -f.nw / 2 + (j + 1) * dc + 1e-9)
                    blk = cov[np.ix_(ma, mc)]
                    if blk.size and blk.mean() >= 0.6:
                        _EP.shown[f.name].add((i, j))
                        new = True
            if new:
                changed.append(f)
        if not changed:
            return
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import UsdGeom

        st = get_current_stage()
        for f in changed:
            pr = st.GetPrimAtPath(_P["decals"][f.name])
            if pr and pr.IsValid():
                UsdGeom.Mesh(pr).GetPointsAttr().Set(_decal_points(f, _EP.shown[f.name]))
    except Exception as e:  # noqa: BLE001
        print(f"[nails] decal update failed: {e!r}", flush=True)


# -- grippers ------------------------------------------------------------------------------------------------------------
def _robots():
    env = _ENV["env"]
    if env is None:
        return None, None
    base = env.unwrapped
    return base, base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)


def gripper_points() -> list[np.ndarray]:
    base, origin = _robots()
    if base is None:
        return []
    from yam_arena.runtime import link_poses, pad_tips_world, tcp_world

    pts = []
    for k in ("left_robot", "right_robot"):
        rob = base.scene[k]
        t = np.asarray(tcp_world(rob), dtype=np.float64) - origin
        tip = np.asarray(pad_tips_world(rob), dtype=np.float64) - origin
        pts += [tip, t]
        lp = link_poses(rob)
        for n in ("tip_left", "tip_right"):
            if n in lp:
                pts.append(np.asarray(lp[n][:3, 3], dtype=np.float64) - origin)
    return pts


def _aabb_dist(p, lo, hi) -> float:
    return float(np.linalg.norm(np.maximum(np.maximum(np.asarray(lo) - p, p - np.asarray(hi)), 0.0)))


HAND_LO = np.array([0.25, PALM_Y0 - 0.04, TW])
HAND_HI = np.array([PALM_X1 + WRIST_L, PALM_Y1 + 0.01, TW + PALM_T + 0.01])
BOT_LO = np.array([BOT_C[0] - BOT_W / 2, BOT_C[1] - BOT_W / 2, TOP])
BOT_HI = np.array([BOT_C[0] + BOT_W / 2, BOT_C[1] + BOT_W / 2, NECK_TOP])


def released(pts=None) -> bool:
    """Every gripper point at least 10 cm from the brush, the bottle and the hand."""
    pts = gripper_points() if pts is None else pts
    if not pts:
        return False
    bc = tuft_corners(_EP.brush_pos, _EP.brush_R)
    cap = _EP.brush_pos
    for p in pts:
        if not np.isfinite(p).all():
            return False
        if _aabb_dist(p, HAND_LO, HAND_HI) < CLEAR or _aabb_dist(p, BOT_LO, BOT_HI) < CLEAR:
            return False
        if min(float(np.linalg.norm(p - cap)), float(np.linalg.norm(p - bc, axis=1).min())) < CLEAR:
            return False
    return True


def parked() -> bool:
    return bool(_EP.parked_s >= PARK_HOLD - 1e-6)


# -- readouts and score ------------------------------------------------------------------------------------------------
def coverage() -> dict:
    return {f.name: float(_EP.coat[f.name].mean()) for f in HAND}


def brush_home() -> bool:
    """The cap sits on the bottle's neck: its axis within 10 deg of vertical, its centre within 5 mm of the bottle's
    axis (the stem then runs down the 8.5 mm bore) and its underside within 3 mm of the neck top."""
    p, R = _EP.brush_pos, _EP.brush_R
    up = float(R[2, 2])
    under = float(p[2]) - CAP_H / 2
    return bool(up >= math.cos(math.radians(10)) and float(np.linalg.norm(p[:2] - BOT_C)) <= 0.005
                and abs(under - NECK_TOP) <= 0.003)


def nail_points() -> float:
    s = 0.0
    for c in coverage().values():
        if c >= COVER_FULL:
            s += NAIL_FULL
        elif c >= COVER_HALF:
            s += NAIL_HALF
    return s


def gated() -> bool:
    return bool(_EP.skin)


def score(objects=None) -> float:
    """0 if polish ever touched the skin (more than 1 mm outside a nail). Else 0.18 per nail
    coated over 90% (0.09 if 50-90%), plus 0.10 if at least one nail scored, the brush is back on the bottle and both
    grippers are parked (10 cm clear of brush, bottle and hand, under 2 cm/s, for 0.5 s)."""
    if gated():
        return 0.0
    s = nail_points()
    if s > 0 and brush_home() and parked():
        s += PARK_PTS
    return float(round(min(1.0, s), 4))


def success(objects=None) -> bool:
    return bool(score(objects) >= 1.0 - 1e-6)


def distance(objects=None) -> float:
    if gated():
        return 1.0
    return float(1.0 - min(1.0, sum(coverage().values()) / len(HAND)))


def progress(objects=None) -> dict:
    out = {f"coat_{k}": round(v, 3) for k, v in coverage().items()}
    out.update({"dipped": float(_EP.dipped), "skin": float(_EP.skin), "brush_home": float(brush_home()),
                "parked": float(parked())})
    return out


VARIATION: dict = {}
