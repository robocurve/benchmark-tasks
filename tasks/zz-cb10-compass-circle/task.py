"""Draw a 50 mm-radius circle with a school compass round a marked centre, holding the paper still.

Scene (metres, table frame: x forward from the arm-base midpoint, y left, z up from the table top).
A cork board (26 x 32 cm, 3 mm) lies centred at (0.30, -0.04). On it lies a loose A4 sheet (210 x 297 mm, 80 g/m2,
5 g), long side along y, centred on the board, with a printed cross (two 12 x 1.5 mm bars) at sheet-local (-0.03, -0.07),
i.e. table (0.27, -0.11). A school compass preset to a 50 mm spread stands legs-down in a desk-tidy cup (inside
74 x 60 mm, 40 mm deep) centred at (0.12, -0.25), legs along x, needle leg toward -x. The compass has a needle leg (with
the head and stem) and a pencil leg joined at a stiff friction hinge, and a knurled grip sleeve that turns freely on
the stem.

To draw, one hand holds the sheet flat (pencil drag would otherwise spin it round the needle), the other carries the
compass by its sleeve, sets the needle on the cross, presses it in, and moves the sleeve round the needle so the
pencil leg sweeps a full turn, then lifts the compass out and sets it down on the table off the board.

Run:
  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1500 \\
    --instruction "Draw a circle around the cross with the compass, then set the compass down on the table off the board."
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

import grip_limited as gl  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
INSTRUCTION = "Draw a circle around the cross with the compass, then set the compass down on the table off the board."
RUN_LIMIT_STEPS = 1500  # 75 s at 20 Hz

# -- colours ----------------------------------------------------------------------------------------------------------
CORK = (0.70, 0.55, 0.38)
PAPER_COL = (0.96, 0.96, 0.94)
INK = (0.12, 0.12, 0.14)
RED = (0.80, 0.12, 0.10)
CHROME = (0.75, 0.76, 0.78)
DARKBLUE = (0.10, 0.16, 0.45)
GRAPH = (0.25, 0.25, 0.27)
CUP_COL = (0.20, 0.45, 0.70)

# -- geometry (table frame, z above table) ----------------------------------------------------------------------------
MAT_X, MAT_Y, MAT_T = 0.30, -0.04, 0.003
MAT_SX, MAT_SY = 0.26, 0.32
PAPER_SX, PAPER_SY, PAPER_T = 0.210, 0.297, 0.0005   # A4, 0.5 mm collider (real sheet ~0.1 mm)
PAPER_M = 0.005                                        # 80 g/m2 x 0.0624 m2
PAPER_X, PAPER_Y = 0.30, -0.04
PAPER_TOP = MAT_T + PAPER_T
CROSS_LOCAL = np.array([-0.03, -0.07])                 # cross on the sheet, sheet-local
CROSS = np.array([PAPER_X, PAPER_Y]) + CROSS_LOCAL
HOLD_LOCAL = np.array([0.0, 0.10])                     # where the left hand presses the sheet (demo only)

RADIUS = 0.050                 # preset spread = circle radius
LEG_L = 0.115                  # hinge axis to tip
ALPHA = math.asin(RADIUS / 2 / LEG_L)   # each leg's tilt from the stem axis when both tips stand level
TIP_DZ = LEG_L * math.cos(ALPHA)        # hinge height above the tips
LEG_W, LEG_D = 0.0045, 0.006
KNUCKLE = (0.010, 0.008, 0.010)
STEM_R, STEM_Z0, STEM_Z1 = 0.0028, 0.005, 0.036       # stem above the hinge (needle-leg body frame)
SLEEVE_R, SLEEVE_H = 0.0045, 0.024                    # knurled grip sleeve, free to turn on the stem
SLEEVE_DZ = 0.022                                     # sleeve centre above the hinge
LEG_M, PENCIL_M, SLEEVE_M = float(os.environ.get('CMP_LEGM', '0.022')), 0.010, float(os.environ.get('CMP_SLM', '0.012'))
HINGE_K = 10.0                 # N m / rad: the tight hinge holds its spread against side loads
HINGE_C = 0.05                 # N m s / rad
LEG_ANG_DAMP = float(os.environ.get("CMP_LAD", "20.0"))   # 1/s angular damping of the two legs
SLEEVE_C = float(os.environ.get("CMP_SLC", "0"))   # N m s / rad: no drive; the sleeve turns on a plain free bearing

# cup (desk tidy) the compass stands in, legs down
CUP_X, CUP_Y = 0.12, -0.25
CUP_IX, CUP_IY, CUP_H, CUP_WALL = 0.074, 0.060, 0.040, 0.003
CUP_FLOOR = 0.003
REST_H = CUP_FLOOR + TIP_DZ                          # hinge height when standing in the cup

# pierce / draw model
PIERCE_N = 0.6                 # needle pressed on the sheet with >= 0.6 N pricks it and pins the sheet there
PIN_OUT = 0.002                # the needle leaves its prick hole once its point is 2 mm above the sheet
LEAD_MIN_N = 0.15              # the lead marks the sheet only while it presses on it with >= 0.15 N
LEAD_GAP = 0.0006              # lead tip within 0.6 mm of the sheet surface counts as touching
STROKE_MAX = 0.006             # a lead kept down between two control steps draws the stroke between them (<= 6 mm)
NBINS = 360                    # 1 deg bins round the cross
SEG_DEG = 3                    # drawn line shown in 3 deg segments
LINE_W = 0.002                 # drawn line rendered 2 mm wide so the camera shows it (scoring uses the points)

# scoring
PAPER_TURN_MAX_DEG = 5.0
PAPER_SLIDE_MAX = 0.005
ARC_LOW_DEG = 180
ARC_MID_DEG = 340
RADIUS_TOL = 0.002
CENTRE_TOL = 0.002
GAP_MAX = 0.003                # largest gap in a closed circle (3 mm)
CLEAR = 0.10
OFF_BOARD = 0.01               # compass set down >= 1 cm outside the board's outline
OFF_SHEET_Z = 0.05             # a gripper over the sheet must be >= 5 cm above it
STILL = 0.01
STILL_S = 1.0
STILL_N = int(round(STILL_S * 20))

# -- objects -----------------------------------------------------------------------------------------------------------
MAT = Box("cork_board", pos=(MAT_X, MAT_Y, TOP + MAT_T / 2), size=(MAT_SX, MAT_SY, MAT_T), color=CORK)
PAPER = Box("paper", pos=(PAPER_X, PAPER_Y, TOP + MAT_T + PAPER_T / 2 + 0.0002), size=(PAPER_SX, PAPER_SY, PAPER_T),
            mass=PAPER_M, color=PAPER_COL)
_H0 = (CUP_X, CUP_Y, TOP + REST_H)
NEEDLE_LEG = Box("compass", pos=_H0, size=KNUCKLE, mass=LEG_M, color=CHROME)
PENCIL_LEG = Box("pencil_leg", pos=_H0, size=(KNUCKLE[0] * 0.8, KNUCKLE[1] * 1.3, KNUCKLE[2] * 0.8), mass=PENCIL_M,
                 color=CHROME)
SLEEVE = Cylinder("grip_sleeve", pos=(CUP_X, CUP_Y, TOP + REST_H + SLEEVE_DZ), radius=SLEEVE_R, height=SLEEVE_H,
                  mass=SLEEVE_M, color=DARKBLUE)
CUP_WALLS = [
    Box("cup_wall_px", pos=(CUP_X + CUP_IX / 2 + CUP_WALL / 2, CUP_Y, TOP + CUP_H / 2),
        size=(CUP_WALL, CUP_IY + 2 * CUP_WALL, CUP_H), color=CUP_COL),
    Box("cup_wall_nx", pos=(CUP_X - CUP_IX / 2 - CUP_WALL / 2, CUP_Y, TOP + CUP_H / 2),
        size=(CUP_WALL, CUP_IY + 2 * CUP_WALL, CUP_H), color=CUP_COL),
    Box("cup_wall_py", pos=(CUP_X, CUP_Y + CUP_IY / 2 + CUP_WALL / 2, TOP + CUP_H / 2),
        size=(CUP_IX, CUP_WALL, CUP_H), color=CUP_COL),
    Box("cup_wall_ny", pos=(CUP_X, CUP_Y - CUP_IY / 2 - CUP_WALL / 2, TOP + CUP_H / 2),
        size=(CUP_IX, CUP_WALL, CUP_H), color=CUP_COL),
    Box("cup_floor", pos=(CUP_X, CUP_Y, TOP + CUP_FLOOR / 2), size=(CUP_IX, CUP_IY, CUP_FLOOR), color=CUP_COL),
]
OBJECTS = [MAT, *CUP_WALLS, PAPER, NEEDLE_LEG, PENCIL_LEG, SLEEVE]

TWIN_EXTRA = [
    {"name": "hinge", "kind": "joint", "real": "compass head friction hinge",
     "note": f"revolute, holds its set spread with {HINGE_K} N m/rad"},
    {"name": "sleeve", "kind": "joint", "real": "knurled grip sleeve turning freely on the stem",
     "note": "revolute about the stem axis"},
    {"name": "needle_pin", "kind": "joint", "real": "needle point pricked through the sheet",
     "note": f"pins the point sideways once the needle presses >= {PIERCE_N} N; lifts out freely"},
    {"name": "grasp", "kind": "joint", "real": "finger pads on the grip sleeve",
     "note": "force-limited compliant grip that engages only on two-pad contact (grip_limited)"},
]

_P: dict = {}

GRIP = gl.LimitedGrip(lambda: [p for p in (_P.get("grip_sleeve"), _P.get("compass"), _P.get("pencil_leg")) if p],
                      label="compass", name="pad_grip", joint_root=lambda: _P.get("root"),
                      k_lin=float(os.environ.get("CMP_KLIN", "8000")), c_lin=float(os.environ.get("CMP_CLIN", "80")),
                      k_rot=float(os.environ.get("CMP_KROT", "20")), c_rot=float(os.environ.get("CMP_CROT", "0.2")))


def _rotm(axis, ang):
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    c, s = math.cos(ang), math.sin(ang)
    x, y, z = a
    return np.array([[c + x * x * (1 - c), x * y * (1 - c) - z * s, x * z * (1 - c) + y * s],
                     [y * x * (1 - c) + z * s, c + y * y * (1 - c), y * z * (1 - c) - x * s],
                     [z * x * (1 - c) - y * s, z * y * (1 - c) + x * s, c + z * z * (1 - c)]])


def needle_tip_local():
    return np.array([-LEG_L * math.sin(ALPHA), 0.0, -LEG_L * math.cos(ALPHA)])


def lead_tip_local():
    return np.array([LEG_L * math.sin(ALPHA), 0.0, -LEG_L * math.cos(ALPHA)])


# -- USD authoring -----------------------------------------------------------------------------------------------------
def _author(root: str) -> None:
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics

    st = get_current_stage()
    _P["root"] = root
    for o in OBJECTS:
        _P[o.name] = f"{root}/{o.name}"

    def cube(parent, name, ctr, size, color, collide=True, rot_y=0.0, rot_z=0.0, off=0.0003):
        g = UsdGeom.Cube.Define(st, f"{parent}/{name}")
        g.CreateSizeAttr(1.0)
        g.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        x = UsdGeom.Xformable(g)
        x.AddTranslateOp().Set(Gf.Vec3d(*map(float, ctr)))
        if rot_z:
            x.AddRotateZOp().Set(float(math.degrees(rot_z)))
        if rot_y:
            x.AddRotateYOp().Set(float(math.degrees(rot_y)))
        x.AddScaleOp().Set(Gf.Vec3f(*map(float, size)))
        if collide:
            UsdPhysics.CollisionAPI.Apply(g.GetPrim())
            pc = PhysxSchema.PhysxCollisionAPI.Apply(g.GetPrim())
            pc.CreateContactOffsetAttr(off)
            pc.CreateRestOffsetAttr(0.0)
        return g

    def cyl(parent, name, ctr, r, h, color, collide=True):
        g = UsdGeom.Cylinder.Define(st, f"{parent}/{name}")
        g.CreateRadiusAttr(r)
        g.CreateHeightAttr(h)
        g.CreateAxisAttr("Z")
        g.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        UsdGeom.Xformable(g).AddTranslateOp().Set(Gf.Vec3d(*map(float, ctr)))
        if collide:
            UsdPhysics.CollisionAPI.Apply(g.GetPrim())
        return g

    nl, pl, sl, pp = _P["compass"], _P["pencil_leg"], _P["grip_sleeve"], _P["paper"]
    # needle leg: shaft tilted -ALPHA about y (tip at -x), steel needle point, head block, stem
    mid = 0.5 * needle_tip_local()
    cube(nl, "shaft", mid * 0.92, (LEG_W, LEG_D, LEG_L * 0.92), CHROME, rot_y=-ALPHA)
    tip = needle_tip_local()
    cube(nl, "needle", tip + np.array([0.0006 * math.sin(ALPHA), 0, 0.004]), (0.0012, 0.0012, 0.008), GRAPH,
         rot_y=-ALPHA, off=0.0002)
    cube(nl, "head", (0, 0, 0.003), (0.016, 0.010, 0.008), CHROME)
    cyl(nl, "stem", (0, 0, (STEM_Z0 + STEM_Z1) / 2), STEM_R, STEM_Z1 - STEM_Z0, CHROME, collide=False)
    # pencil leg: shaft tilted +ALPHA, lead holder and graphite lead
    mid = 0.5 * lead_tip_local()
    cube(pl, "shaft", mid * 0.85, (LEG_W, LEG_D, LEG_L * 0.85), CHROME, rot_y=ALPHA)
    lt = lead_tip_local()
    cube(pl, "holder", lt * 0.88, (0.007, 0.007, 0.012), RED, rot_y=ALPHA)
    cube(pl, "lead", lt + np.array([-0.0005 * math.sin(ALPHA), 0, 0.003]), (0.0012, 0.0012, 0.006), GRAPH,
         rot_y=ALPHA, off=0.0002)
    # knurl rings on the sleeve (visual)
    for k in range(4):
        cyl(sl, f"knurl{k}", (0, 0, -0.009 + k * 0.006), SLEEVE_R + 0.0004, 0.0015, CHROME, collide=False)
    # printed cross and pre-made line segments (hidden) on the sheet, sheet-local frame
    zt = PAPER_T / 2 + 0.00006
    cube(pp, "cross_a", (CROSS_LOCAL[0], CROSS_LOCAL[1], zt), (0.012, 0.0015, 0.0001), INK, collide=False)
    cube(pp, "cross_b", (CROSS_LOCAL[0], CROSS_LOCAL[1], zt), (0.0015, 0.012, 0.0001), INK, collide=False)
    _P["segs"] = []
    for k in range(NBINS // SEG_DEG):
        g = cube(pp, f"line_{k}", (CROSS_LOCAL[0], CROSS_LOCAL[1], zt), (0.0028, 0.0008, 0.0001), INK,
                 collide=False)
        UsdGeom.Imageable(g).MakeInvisible()
        x = UsdGeom.Xformable(g)
        ops = x.GetOrderedXformOps()
        _P["segs"].append((g, ops))
    # thin contact offsets on the sheet and board
    for p in (pp, _P["cork_board"]):
        prim = st.GetPrimAtPath(p)
        for q in [prim, *prim.GetChildren()]:
            if q.HasAPI(UsdPhysics.CollisionAPI):
                pc = PhysxSchema.PhysxCollisionAPI.Apply(q)
                pc.CreateContactOffsetAttr(0.0004)
                pc.CreateRestOffsetAttr(0.0)
    for p in (nl, pl, sl, pp):
        prim = st.GetPrimAtPath(p)
        PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)   # report light touches (default 1 N)
        r = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        r.CreateSolverPositionIterationCountAttr(32)
        r.CreateSolverVelocityIterationCountAttr(8)
        r.CreateMaxDepenetrationVelocityAttr(0.3)
        if p in (nl, pl):
            r.CreateAngularDampingAttr(LEG_ANG_DAMP)   # finger friction on the knurl: no free pendulum swing

    w = UsdGeom.Xformable(st.GetPrimAtPath(root)).ComputeLocalToWorldTransform(0).ExtractTranslation()
    W = np.array([w[0], w[1], w[2]], dtype=float)

    def joint(kind, name, b0, b1, p_world, axis):
        j = kind.Define(st, f"{root}/{name}")
        j.CreateBody0Rel().SetTargets([Sdf.Path(b0)])
        j.CreateBody1Rel().SetTargets([Sdf.Path(b1)])
        j.CreateLocalPos0Attr(Gf.Vec3f(0.0, 0.0, 0.0))
        j.CreateLocalPos1Attr(Gf.Vec3f(*map(float, p_world)))
        j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateAxisAttr(axis)
        j.CreateExcludeFromArticulationAttr(True)
        j.CreateCollisionEnabledAttr(False)
        return j

    # hinge: needle leg (body0, frame at the hinge) -> pencil leg (body1 origin = hinge)
    j = joint(UsdPhysics.RevoluteJoint, "compass_hinge", nl, pl, (0, 0, 0), "Y")
    j.CreateLowerLimitAttr(-25.0)
    j.CreateUpperLimitAttr(25.0)
    d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
    d.CreateTypeAttr("force")
    d.CreateStiffnessAttr(HINGE_K * math.pi / 180.0)
    d.CreateDampingAttr(HINGE_C * math.pi / 180.0)
    d.CreateTargetPositionAttr(0.0)
    # free-turning sleeve on the stem: needle leg frame at (0,0,SLEEVE_DZ) -> sleeve origin
    j = UsdPhysics.RevoluteJoint.Define(st, f"{root}/sleeve_bearing")
    j.CreateBody0Rel().SetTargets([Sdf.Path(nl)])
    j.CreateBody1Rel().SetTargets([Sdf.Path(sl)])
    j.CreateLocalPos0Attr(Gf.Vec3f(0.0, 0.0, float(SLEEVE_DZ)))
    j.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
    j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
    j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    j.CreateAxisAttr("Z")
    j.CreateExcludeFromArticulationAttr(True)
    j.CreateCollisionEnabledAttr(False)
    if SLEEVE_C > 0:
        d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
        d.CreateTypeAttr("force")
        d.CreateStiffnessAttr(0.0)
        d.CreateDampingAttr(SLEEVE_C * math.pi / 180.0)
        d.CreateTargetVelocityAttr(0.0)
    _P["W"] = W
    print(f"[compass] authored root={root} alpha={math.degrees(ALPHA):.2f} deg", flush=True)


def _patch_scene(sc):
    if getattr(sc.task_object, "_compass_patched", False):
        return
    original = sc.task_object

    def task_object(spec):
        obj = original(spec)
        if getattr(spec, "name", None) == SLEEVE.name:
            from isaaclab.sim.utils import clone

            cfg = obj.spawner_cfg
            base = getattr(cfg.func, "__wrapped__", cfg.func)
            cfg.func = clone(lambda p, c, *a, **k: (base(p, c, *a, **k), _author(str(p).rsplit("/", 1)[0]))[0])
            obj.object_cfg = obj._init_object_cfg()
        return obj

    task_object._compass_patched = True
    sc.task_object = task_object


class _Finder(importlib.abc.MetaPathFinder):
    _compass = True

    def find_spec(self, fullname, path, target=None):
        if fullname != "yam_arena.scene":
            return None
        sys.meta_path.remove(self)
        try:
            spec = importlib.util.find_spec(fullname)
        finally:
            sys.meta_path.insert(0, self)
        if spec and spec.loader:
            execute = spec.loader.exec_module
            spec.loader.exec_module = lambda m: (execute(m), _patch_scene(m))
        return spec


if "yam_arena.scene" in sys.modules:
    _patch_scene(sys.modules["yam_arena.scene"])
elif not any(getattr(f, "_compass", False) for f in sys.meta_path):
    sys.meta_path.insert(0, _Finder())


# -- episode state ---------------------------------------------------------------------------------------------------
_ENV: dict = {"env": None}


class _Episode:
    def __init__(self):
        self.step = 0
        self.pinned = False
        self.pin_n = 0
        self.pin_local = None          # needle point on the sheet (sheet-local xy) while pinned
        self.bins = [None] * NBINS     # radius (m) of the mark in each 1 deg bin about the cross
        self.pts: list = []            # sheet-local mark points
        self.paper_moved = False
        self.paper_turn = 0.0
        self.paper_slide = 0.0
        self.hist: list = []
        self.chist: list = []          # compass points, last STILL_N + 2 control steps
        self.lead_n = 0.0
        self.needle_n = 0.0
        self.shown = set()
        self.last_mark = None          # sheet-local lead point at the previous control step, if it was marking


_EP = _Episode()
_VIEWS: dict = {}


def _stage():
    from isaaclab.sim.utils.stage import get_current_stage

    return get_current_stage()


def reset_episode() -> None:
    global _EP
    _EP = _Episode()
    GRIP.reset()
    try:
        st = _stage()
        for k in list(_P.get("pins", [])):
            p = st.GetPrimAtPath(k)
            if p and p.IsValid():
                p.GetAttribute("physics:jointEnabled").Set(False)
        for g, _ops in _P.get("segs", []):
            from pxr import UsdGeom

            UsdGeom.Imageable(g).MakeInvisible()
    except Exception:  # noqa: BLE001
        pass


def _pose(name):
    env = _ENV.get("env")
    if env is None:
        return None
    try:
        base = env.unwrapped
        pose = base.arena_world.get_pose_w(name)[0].detach().cpu().numpy().astype(np.float64)
        origin = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
        if not np.isfinite(pose).all():
            return None
        x, y, z, w = pose[3:7]
        R = _rotm_q(w, x, y, z)
        return pose[:3] - origin - np.array([0, 0, TOP]), R
    except Exception:  # noqa: BLE001
        return None


def _rotm_q(w, x, y, z):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _contact_forces() -> tuple[float, float]:
    """PhysX contact force [N] of the sheet against (needle leg, pencil leg). The sheet is the sensor body."""
    try:
        sv = GRIP._sim_view()
        key = (id(sv), "paper_vs_legs")
        if key not in _VIEWS:
            gl.enable_contacts()
            _VIEWS[key] = sv.create_rigid_contact_view([_P[PAPER.name]],
                                                        filter_patterns=[[_P[NEEDLE_LEG.name], _P[PENCIL_LEG.name]]],
                                                        max_contact_data_count=256)
        from yam_arena.constants import PHYSICS_DT

        m = _VIEWS[key].get_contact_force_matrix(dt=PHYSICS_DT)
        m = gl._np(m).reshape(2, 3)   # warp array -> numpy
        return float(np.linalg.norm(m[0])), float(np.linalg.norm(m[1]))
    except Exception as e:  # noqa: BLE001
        _P["cerr"] = _P.get("cerr", 0) + 1
        if _P["cerr"] <= 3:
            print(f"[compass] contact view failed (step {_EP.step}): {e!r}", flush=True)
        _VIEWS.pop((id(GRIP._sim_view()), "paper_vs_legs"), None)
        return 0.0, 0.0


def paper_pose():
    return _pose(PAPER.name)


def to_paper(p_table):
    pp = paper_pose()
    if pp is None:
        return None
    pos, R = pp
    return R.T @ (np.asarray(p_table, float) - pos)


def needle_tip():
    c = _pose(NEEDLE_LEG.name)
    return None if c is None else c[0] + c[1] @ needle_tip_local()


def lead_tip():
    c = _pose(PENCIL_LEG.name)
    return None if c is None else c[0] + c[1] @ lead_tip_local()


def spread():
    a, b = needle_tip(), lead_tip()
    return float("nan") if a is None or b is None else float(np.linalg.norm(a - b))


def _pin(on: bool, tip_table=None):
    from pxr import Gf, Sdf, UsdPhysics

    st = _stage()
    if not on:
        for k in _P.get("pins", []):
            p = st.GetPrimAtPath(k)
            if p and p.IsValid():
                p.GetAttribute("physics:jointEnabled").Set(False)
        _EP.pinned = False
        return
    _EP.pin_n += 1
    path = f"{_P['root']}/needle_pin_{_EP.pin_n}"
    j = UsdPhysics.Joint.Define(st, path)
    j.CreateBody0Rel().SetTargets([Sdf.Path(_P[NEEDLE_LEG.name])])
    j.CreateBody1Rel().SetTargets([Sdf.Path(_P[PAPER.name])])
    j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, needle_tip_local())))
    loc = to_paper(tip_table)
    loc = np.array([loc[0], loc[1], PAPER_T / 2])
    j.CreateLocalPos1Attr(Gf.Vec3f(*map(float, loc)))
    j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
    j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    j.CreateExcludeFromArticulationAttr(True)
    j.CreateCollisionEnabledAttr(False)
    prim = j.GetPrim()
    for ax in ("transX", "transY"):          # pinned sideways in the prick hole
        lim = UsdPhysics.LimitAPI.Apply(prim, ax)
        lim.CreateLowAttr(1.0)
        lim.CreateHighAttr(-1.0)
    # transZ and all rotations stay free: the point lifts straight out; the sheet's surface (contact) stops it going down
    _P.setdefault("pins", []).append(path)
    _P["pin_path"] = path
    _EP.pinned = True
    _EP.pin_local = loc[:2].copy()
    print(f"[compass] needle pricked the sheet at {np.round(loc[:2] * 1000, 1)} mm (step {_EP.step})", flush=True)


def _pin_alive() -> bool:
    """The point stays in its prick hole until it is lifted PIN_OUT above the sheet's resting surface (table frame)."""
    tip = needle_tip()
    if tip is None or _EP.pin_local is None:
        return False
    return bool(tip[2] - PAPER_TOP < PIN_OUT)


def _show_seg(k: int, p_local):
    g, _ops = _P["segs"][k]
    from pxr import Gf, UsdGeom

    ops = UsdGeom.Xformable(g).GetOrderedXformOps()
    ang = math.atan2(p_local[1] - CROSS_LOCAL[1], p_local[0] - CROSS_LOCAL[0])
    for op in ops:
        n = op.GetOpName()
        if "translate" in n:
            op.Set(Gf.Vec3d(float(p_local[0]), float(p_local[1]), PAPER_T / 2 + 0.00006))
    x = UsdGeom.Xformable(g)
    if not any("rotateZ" in op.GetOpName() for op in ops):
        x.ClearXformOpOrder()
        x.AddTranslateOp().Set(Gf.Vec3d(float(p_local[0]), float(p_local[1]), PAPER_T / 2 + 0.00006))
        x.AddRotateZOp().Set(float(math.degrees(ang) + 90.0))
        x.AddScaleOp().Set(Gf.Vec3f(0.0032, LINE_W, 0.0001))
    UsdGeom.Imageable(g).MakeVisible()


def _mark(q) -> None:
    d = q - CROSS_LOCAL
    b = int(math.degrees(math.atan2(d[1], d[0])) % 360) % NBINS
    _EP.bins[b] = float(np.hypot(*d))
    _EP.pts.append(np.array(q, dtype=float))
    k = b // SEG_DEG
    if k not in _EP.shown and "segs" in _P:
        _EP.shown.add(k)
        _show_seg(k, q)


def _step(env) -> None:
    _EP.step += 1
    pp = paper_pose()
    if pp is None:
        return
    pos, R = pp
    turn = abs(math.degrees(math.atan2(R[1, 0], R[0, 0])))
    slide = float(np.hypot(pos[0] - PAPER_X, pos[1] - PAPER_Y))
    _EP.paper_turn, _EP.paper_slide = max(_EP.paper_turn, turn), max(_EP.paper_slide, slide)
    if turn > PAPER_TURN_MAX_DEG or slide > PAPER_SLIDE_MAX:
        _EP.paper_moved = True
    tip = needle_tip()
    _EP.needle_n, _EP.lead_n = _contact_forces()
    if _EP.pinned and not _pin_alive():
        _pin(False)
        print(f"[compass] needle out of the sheet (step {_EP.step})", flush=True)
    if not _EP.pinned and tip is not None and _EP.needle_n >= PIERCE_N:
        loc = to_paper(tip)
        if abs(loc[0]) < PAPER_SX / 2 and abs(loc[1]) < PAPER_SY / 2 and loc[2] < PAPER_T / 2 + 0.0015:
            _pin(True, tip)
    lt = lead_tip()
    marked = False
    if _EP.pinned and lt is not None and _EP.lead_n >= LEAD_MIN_N:
        loc = to_paper(lt)
        if loc[2] < PAPER_T / 2 + LEAD_GAP:
            marked = True
            p1 = loc[:2].copy()
            p0 = _EP.last_mark
            # the lead stayed down since the last control step: it drew the stroke in between
            n = 1 if p0 is None or np.hypot(*(p1 - p0)) > STROKE_MAX else max(1, int(np.hypot(*(p1 - p0)) / 0.0003))
            for j in range(1, n + 1):
                q = p1 if p0 is None else p0 + (p1 - p0) * j / n
                _mark(q)
            _EP.last_mark = p1
    if not marked:
        _EP.last_mark = None
    gp = gripper_points()
    if gp:
        _EP.hist.append(np.array(gp[:4]))
        _EP.hist = _EP.hist[-(STILL_N + 2):]
    cp = compass_points()
    if len(cp) == 5:
        _EP.chist.append(np.array(cp))
        _EP.chist = _EP.chist[-(STILL_N + 2):]


def _install_hooks() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # noqa: BLE001
        return
    cls = ie.YamArenaEmbodiment
    if not getattr(cls.reset, "_compass_hooked", False):
        orig = cls.reset

        def reset(self, *a, **k):
            reset_episode()
            return orig(self, *a, **k)

        reset._compass_hooked = True
        cls.reset = reset
    if not getattr(cls._task_step, "_compass_hooked", False):
        base = cls._task_step

        def _task_step(self, *a, **k):
            _ENV["env"] = self._env
            out = base(self, *a, **k)
            try:
                GRIP.update(self._env)
                _step(self._env)
            except Exception as e:  # noqa: BLE001
                print(f"[compass] step failed: {e!r}", flush=True)
            return out

        _task_step._compass_hooked = True
        cls._task_step = _task_step


_install_hooks()


# -- grippers ----------------------------------------------------------------------------------------------------------
def gripper_points() -> list:
    env = _ENV["env"]
    if env is None:
        return []
    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
    from yam_arena.runtime import link_poses, pad_tips_world, tcp_world

    pts = []
    for k in ("left_robot", "right_robot"):
        rob = base.scene[k]
        pts.append(np.asarray(tcp_world(rob), dtype=np.float64) - origin - np.array([0, 0, TOP]))
        pts.append(np.asarray(pad_tips_world(rob), dtype=np.float64) - origin - np.array([0, 0, TOP]))
    for k in ("left_robot", "right_robot"):
        lp = link_poses(base.scene[k])
        for n in ("tip_left", "tip_right"):
            if n in lp:
                pts.append(np.asarray(lp[n][:3, 3], dtype=np.float64) - origin - np.array([0, 0, TOP]))
    return pts


def compass_points():
    out = []
    for n in (NEEDLE_LEG.name, PENCIL_LEG.name, SLEEVE.name):
        c = _pose(n)
        if c is not None:
            out.append(c[0])
    for p in (needle_tip(), lead_tip()):
        if p is not None:
            out.append(p)
    return out


def set_down() -> bool:
    """The compass rests on the table off the board: every compass point (both tips, both legs, the sleeve) lies
    >= 1 cm outside the board's outline, the needle is out of the sheet, and the compass moved <= 1 cm in the last 1 s."""
    cps = compass_points()
    if len(cps) != 5 or _EP.pinned:
        return False
    for p in cps:
        if abs(p[0] - MAT_X) < MAT_SX / 2 + OFF_BOARD and abs(p[1] - MAT_Y) < MAT_SY / 2 + OFF_BOARD:
            return False
    h = _EP.chist
    if len(h) < STILL_N + 1:
        return False
    return bool(np.max(np.linalg.norm(h[-1] - h[-1 - STILL_N], axis=1)) <= STILL)


def parked() -> bool:
    """Both grips released, every gripper point >= 10 cm from the compass and >= 5 cm above the sheet's top,
    TCPs and pad tips moved <= 1 cm over the last 1 s."""
    if GRIP.holders() != 0:
        return False
    pts = gripper_points()
    cps = compass_points()
    if not pts or not cps:
        return False
    for p in pts:
        if min(float(np.linalg.norm(p - c)) for c in cps) < CLEAR:
            return False
        if p[2] < PAPER_TOP + OFF_SHEET_Z and to_paper(p) is not None:
            loc = to_paper(p)
            if abs(loc[0]) < PAPER_SX / 2 + 0.01 and abs(loc[1]) < PAPER_SY / 2 + 0.01:
                return False
    h = _EP.hist
    if len(h) < STILL_N + 1:
        return False
    return bool(np.max(np.linalg.norm(h[-1] - h[-1 - STILL_N], axis=1)) <= STILL)


# -- circle measures (sheet-local, about the printed cross) ------------------------------------------------------------
def arc_deg() -> int:
    return int(sum(b is not None for b in _EP.bins))


def largest_gap_mm() -> float:
    """Longest run of undrawn 1 deg bins, as arc length at the circle radius (mm)."""
    drawn = [b is not None for b in _EP.bins]
    if not any(drawn):
        return 1e9
    run = best = 0
    for v in drawn + drawn:
        run = 0 if v else run + 1
        best = max(best, min(run, NBINS))
    r = np.mean([b for b in _EP.bins if b is not None])
    return float(best * math.pi / 180.0 * r * 1000.0)


def circle_fit():
    """Least-squares circle through the drawn points: (centre offset from the cross [m], mean radius [m])."""
    if len(_EP.pts) < 10:
        return None
    P = np.array(_EP.pts)
    A = np.c_[2 * P[:, 0], 2 * P[:, 1], np.ones(len(P))]
    bb = (P ** 2).sum(axis=1)
    cx, cy, c = np.linalg.lstsq(A, bb, rcond=None)[0]
    r = math.sqrt(max(c + cx * cx + cy * cy, 0.0))
    return float(np.hypot(cx - CROSS_LOCAL[0], cy - CROSS_LOCAL[1])), r


def radius_ok() -> bool:
    # every recorded mark (not only the latest per 1 deg bin), so an early stray mark can't be overwritten
    rs = [float(np.hypot(*(p - CROSS_LOCAL))) for p in _EP.pts]
    return bool(rs) and all(abs(r - RADIUS) <= RADIUS_TOL for r in rs)


def centred() -> bool:
    f = circle_fit()
    return f is not None and f[0] <= CENTRE_TOL


# -- scoring ---------------------------------------------------------------------------------------------------------
def violation() -> str:
    if _EP.paper_moved:
        return "the sheet turned or slid"
    return ""


def score(objects=None) -> float:
    """0 if the sheet turned > 5 deg or slid > 5 mm at any time. Marks count only where the lead pressed the sheet
    (>= 0.15 N) while the needle was pricked in it. 0.3: an arc of >= 180 deg round the cross; 0.6: >= 340 deg with
    every mark at 50 +- 2 mm from the cross and the fitted centre within 2 mm of it; 1.0: also no gap > 3 mm (closed),
    the compass set down >= 1 cm outside the board, needle out, still 1 s, both grippers open, >= 10 cm from the
    compass, >= 5 cm above the sheet if over it, and still for 1 s."""
    if violation():
        return 0.0
    a = arc_deg()
    if a < ARC_LOW_DEG:
        return 0.0
    if not (a >= ARC_MID_DEG and radius_ok() and centred()):
        return 0.3
    if not (largest_gap_mm() <= GAP_MAX * 1000 and set_down() and parked()):
        return 0.6
    return 1.0


def success(objects=None) -> bool:
    return bool(score(objects) == 1.0)


def distance(objects=None) -> float:
    return 1.0 - arc_deg() / NBINS


def progress(objects=None) -> dict:
    f = circle_fit()
    return {"arc": arc_deg(), "gap_mm": round(largest_gap_mm(), 1) if arc_deg() else None,
            "fit": None if f is None else [round(f[0] * 1000, 2), round(f[1] * 1000, 2)],
            "pinned": _EP.pinned, "needle_n": round(_EP.needle_n, 2), "lead_n": round(_EP.lead_n, 2),
            "spread_mm": round(spread() * 1000, 2), "paper_turn": round(_EP.paper_turn, 2),
            "paper_slide_mm": round(_EP.paper_slide * 1000, 2), "set_down": set_down(), "parked": parked(),
            "grip": GRIP.holders(), "violation": violation(), "tips_z": _tips_z()}


def _tips_z():
    try:
        a, b, c = needle_tip(), lead_tip(), _pose(SLEEVE.name)[0]
        return [round(float(a[2] - PAPER_TOP) * 1000, 1), round(float(b[2] - PAPER_TOP) * 1000, 1),
                [round(float(v) * 1000, 1) for v in c]]
    except Exception:  # noqa: BLE001
        return None


VARIATION: dict = {}

