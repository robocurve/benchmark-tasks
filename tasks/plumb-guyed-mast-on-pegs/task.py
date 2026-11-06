"""A small guyed mast on the station table, straight in front of the robot. A white mast (2 x 2 x 22 cm) stands on a
hinged foot 30 cm out; the hinge lets it tip only sideways (towards either arm) and has a stop at 30 deg each way. At
the start it leans 30 deg towards the robot's right and rests on that stop. A yellow guy collar slides on the top of
the mast on a spring. Two blue 6 mm guy lines are tied to the collar, one on each side: the left guy is 29.6 cm long,
the right one 26.1 cm. Each ends in a rigid white ring eye (44 x 44 mm, 32 x 32 mm hole) with a grip bail over its
hole. Both lines lie loose on the table.

On each side, along the line through the mast foot (x 30 cm), stand three hooked ground pegs (black posts 8 x 8 x
30 mm, each with a 9 mm nose at the top pointing away from the mast): left pegs at y 20, 24 and 28 cm, right pegs at
y -23, -27 and -31 cm. A ring dropped over a peg is pulled in against the post by its guy and held under the nose.
Only one peg per side stands the mast plumb: left y 28 (the outer one), right y -23 (the inner one). Every other peg
leaves its guy slack or too short, and the mast stands out of plumb.

The robot must stand the mast up plumb with its two guys alone (never touching the mast or the collar), put each
guy's ring on the peg that holds the mast plumb, and let go.

Why two arms: each ring and its peg row can be reached by one arm only (reach-forced). A single guy pegged on its own
pulls the mast past plumb onto the far stop, so the mast stands only when both lines are pegged against each other.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score \\
           --instruction "<INSTRUCTION below>"
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import cablekit  # noqa: E402,F401  (hooks yam_arena so the Cables below are simulated)
import cableshape  # noqa: E402
import numpy as np  # noqa: E402
from cablekit import Plug, cable  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = ("Stand the white mast up plumb using only its two blue guy lines: put each guy's ring eye on the "
               "ground peg on its own side that holds the mast plumb, then let go of both rings.")

WHITE = (0.92, 0.92, 0.90)
YELLOW = (0.95, 0.78, 0.10)
GREY = (0.45, 0.45, 0.47)
BLACK = (0.08, 0.08, 0.09)
ROPE = (0.10, 0.25, 0.70)
RING_C = (0.96, 0.96, 0.96)

# -- mast, foot, collar (m, world frame: x forward, y left, z up) -----------------------------------------------------
XM = 0.30  # mast plane: the hinge axis runs along x through (XM, 0)
FOOT_H = 0.02
HZ = TOP + FOOT_H  # hinge height
MAST_L, MAST_W, MAST_M = 0.22, 0.02, 0.030
STOP_DEG = 30.0  # hinge stop each way
LEAN0_DEG = -STOP_DEG  # start: leaning 30 deg towards -y (the robot's right), on the stop
COLLAR_S, COLLAR_M = (0.034, 0.034, 0.020), 0.010
COLLAR_H = 0.21  # collar centre above the hinge with the spring at rest (against its top stop)
COLLAR_K = float(os.environ.get("MAST_COLLAR_K", "24.0"))  # N/m spring under the collar
COLLAR_TRAVEL = 0.06
HINGE_DAMP = 0.002  # N m s / rad
ATT_Y = COLLAR_S[1] / 2  # guys tied to the collar's +y / -y faces

FOOT = Box("mast_foot", pos=(XM, 0.0, TOP + FOOT_H / 2), size=(0.05, 0.05, FOOT_H), color=GREY)
MAST = Box("mast", pos=(XM, 0.0, HZ + MAST_L / 2), size=(MAST_W, MAST_W, MAST_L), mass=MAST_M, color=WHITE)
COLLAR = Box("guy_collar", pos=(XM, 0.0, HZ + COLLAR_H), size=COLLAR_S, mass=COLLAR_M, color=YELLOW)

# -- hooked pegs ---------------------------------------------------------------------------------------------------------
POST, POST_H = 0.008, 0.030
NOSE_L, NOSE_T = 0.009, 0.006  # nose: sticks out NOSE_L from the post's outer face, NOSE_T thick, flush with the top
NOSE_Z0 = POST_H - NOSE_T  # nose underside height above the table
PEG_Y = {"left": (0.20, 0.24, 0.28), "right": (-0.23, -0.27, -0.31)}
OK_PEG = {"left": 2, "right": 0}  # left: the outer peg; right: the inner peg
SIDES = {"left": 1.0, "right": -1.0}


def _pegs() -> list:
    out = []
    for side, sgn in SIDES.items():
        for i, y in enumerate(PEG_Y[side]):
            out.append(Box(f"peg_{side}_{i + 1}", pos=(XM, y, TOP + POST_H / 2), size=(POST, POST, POST_H), color=BLACK))
            out.append(Box(f"peg_{side}_{i + 1}_nose", pos=(XM, y + sgn * (POST / 2 + NOSE_L / 2), TOP + NOSE_Z0 + NOSE_T / 2),
                           size=(POST, NOSE_L, NOSE_T), color=BLACK))
    return out


PEGS = _pegs()

# -- guys and ring eyes ----------------------------------------------------------------------------------------------------
R = 0.003
SEG = 0.02
# ring eye (plug-local: x along the line away from the mast, y across, z up; the plug box is the ring's inner bar)
BAR = 0.006
RING_T = 0.005
RING_W = 0.044  # outer size along and across
HOLE = RING_W - 2 * BAR  # 32 mm square hole
RING_PLUG = Plug(size=(BAR, RING_W, RING_T), mass=0.008, color=RING_C)
BAIL_H = 0.050  # grip bail: two legs on the side bars and a cross bar over the hole centre, 50 mm above the ring
BAIL_LEG = (0.006, 0.004, BAIL_H)  # leg: 6 (along) x 4 (across) mm, centred on the side bar
BAIL_BAR = (0.008, RING_W, 0.006)  # bar across the top: 8 (along) x 44 (across) x 6 (high) mm
BAIL_UP = float(os.environ.get("MAST_BAIL_UP", "1.0"))  # plug-local z side of the bail (set from the start roll)
HOLE_C = BAR / 2 + HOLE / 2  # hole centre from the plug centre, along plug x
LINE_LEN = {"left": float(os.environ.get("MAST_LEN_L", "0.296")), "right": float(os.environ.get("MAST_LEN_R", "0.261"))}


def _guy(side: str):
    sgn = SIDES[side]
    return cable(f"{side}_guy", start=(XM, sgn * (ATT_Y + 0.0005), HZ + COLLAR_H), yaw=sgn * math.pi / 2,
                 length=LINE_LEN[side], radius=R, segment=SEG, cable_mass=0.015, bend_stiffness=0.004,
                 bend_damping=0.0005, friction=0.8, end_plug=RING_PLUG, color=ROPE)


GUYS = {side: _guy(side) for side in SIDES}

OBJECTS = [FOOT, MAST, COLLAR, *PEGS, GUYS["left"], GUYS["right"]]

TWIN_EXTRA = [
    {"name": "mast_hinge", "kind": "joint", "real": "print-in-place or M4-bolt hinge on the foot, axis along x, "
     "stops at +-30 deg; a drop of grease for slight damping", "note": "the mast tips only sideways"},
    {"name": "collar_spring", "kind": "joint", "real": "collar sliding on the mast top over a 24 N/m compression "
     "spring, 6 cm travel, top stop at 21 cm above the hinge", "note": "stands in for guy stretch / tensioner"},
    {"name": "guy_ties", "kind": "joint", "real": "each guy knotted through a hole in the collar's side face"},
    *({"name": f"ring_eye_{s}", "kind": "body", "size_mm": [44, 44, 5], "colour": "white",
       "note": "square ring, 32 mm hole, grip bail 50 mm tall over the hole; glued to the guy end"}
      for s in SIDES),
]

VARIATION = {"cable:left_guy": {"dx": 0.0, "dy": 0.0, "yaw": 0.0}, "cable:right_guy": {"dx": 0.0, "dy": 0.0, "yaw": 0.0},
             "obj:mast": {"dx": 0.0, "dy": 0.0, "yaw": 0.0}}  # guys tied to the hinged mast: fixed layout

# -- spawn hook: hinge, collar slide + spring, guy ties, ring colliders -------------------------------------------------
_PATHS: dict = {}


def _install_spawn() -> None:
    orig = cablekit._spawn_cable
    if getattr(orig, "_mast_task", False):
        return

    def spawn(prim_path, cfg, *a, **k):
        prim = orig(prim_path, cfg, *a, **k)
        name = getattr(cfg, "cable_name", "")
        if name not in (GUYS["left"].name, GUYS["right"].name):
            return prim
        import isaaclab.sim as sim_utils
        from isaaclab.sim.utils import bind_physics_material, bind_visual_material
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics

        stage = get_current_stage()
        root = str(prim_path).rsplit("/", 1)[0]
        side = "left" if name == GUYS["left"].name else "right"
        sgn = SIDES[side]
        spec = GUYS[side]
        collar, mast, foot = f"{root}/{COLLAR.name}", f"{root}/{MAST.name}", f"{root}/{FOOT.name}"
        q0 = Gf.Quatf(1, 0, 0, 0)
        # tie: the guy's start tip on the collar face (ball joint, translations locked)
        j = UsdPhysics.Joint.Define(stage, f"{prim_path}/collar_tie")
        j.CreateBody0Rel().SetTargets([Sdf.Path(collar)])
        j.CreateBody1Rel().SetTargets([Sdf.Path(f"{prim_path}/seg_0")])
        j.CreateLocalPos0Attr(Gf.Vec3f(0.0, float(sgn * (ATT_Y + 0.0005)), 0.0))
        j.CreateLocalRot0Attr(q0)
        j.CreateLocalPos1Attr(Gf.Vec3f(-spec.seg_len / 2, 0.0, 0.0))
        j.CreateLocalRot1Attr(q0)
        for dof in ("transX", "transY", "transZ"):
            lim = UsdPhysics.LimitAPI.Apply(j.GetPrim(), dof)
            lim.CreateLowAttr(1.0)
            lim.CreateHighAttr(-1.0)
        j.CreateExcludeFromArticulationAttr(True)
        for body in (collar, mast):
            fp = UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(body))
            for i in range(3):
                fp.GetFilteredPairsRel().AddTarget(Sdf.Path(f"{prim_path}/seg_{i}"))
        # ring eye: side bars, outer bar and grip bail as extra colliders on the plug body (the plug box = inner bar)
        plug = f"{prim_path}/end_plug"
        look = f"{prim_path}/Looks/end_plug"
        parts = [("side_a", (HOLE_C, (RING_W - BAR) / 2, 0.0), (HOLE, BAR, RING_T)),
                 ("side_b", (HOLE_C, -(RING_W - BAR) / 2, 0.0), (HOLE, BAR, RING_T)),
                 ("outer", (BAR + HOLE, 0.0, 0.0), (BAR, RING_W, RING_T)),
                 ("bail_a", (HOLE_C, (RING_W - BAR) / 2, BAIL_UP * BAIL_H / 2), BAIL_LEG),
                 ("bail_b", (HOLE_C, -(RING_W - BAR) / 2, BAIL_UP * BAIL_H / 2), BAIL_LEG),
                 ("bail_bar", (HOLE_C, 0.0, BAIL_UP * (BAIL_H + BAIL_BAR[2] / 2 - 0.002)), BAIL_BAR)]
        for nm, pos, size in parts:
            g = UsdGeom.Cube.Define(stage, f"{plug}/{nm}")
            g.CreateSizeAttr(1.0)
            g.AddTranslateOp().Set(Gf.Vec3d(*map(float, pos)))
            g.AddScaleOp().Set(Gf.Vec3f(*map(float, size)))
            UsdPhysics.CollisionAPI.Apply(g.GetPrim())
            cp = PhysxSchema.PhysxCollisionAPI.Apply(g.GetPrim())
            cp.CreateContactOffsetAttr(0.002)
            cp.CreateRestOffsetAttr(0.0)
            bind_physics_material(str(g.GetPath()), f"{prim_path}/Looks/cable_physics", stage=stage)
            bind_visual_material(str(g.GetPath()), look, stage=stage)
        if side != "right":
            return prim
        # ---- once (the right guy is spawned last): hinge, collar slide ----
        h = UsdPhysics.RevoluteJoint.Define(stage, f"{root}/mast_hinge")
        h.CreateBody1Rel().SetTargets([Sdf.Path(mast)])
        h.CreateAxisAttr("X")
        w = UsdGeom.Xformable(stage.GetPrimAtPath(root)).ComputeLocalToWorldTransform(0).ExtractTranslation()
        h.CreateLocalPos0Attr(Gf.Vec3f(float(w[0] + XM), float(w[1]), float(w[2] + HZ)))
        h.CreateLocalRot0Attr(q0)
        h.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, float(-MAST_L / 2)))
        h.CreateLocalRot1Attr(q0)
        h.CreateLowerLimitAttr(-STOP_DEG)
        h.CreateUpperLimitAttr(STOP_DEG)
        h.CreateExcludeFromArticulationAttr(True)
        d = UsdPhysics.DriveAPI.Apply(h.GetPrim(), "angular")
        d.CreateTypeAttr("force")
        d.CreateStiffnessAttr(0.0)
        d.CreateDampingAttr(HINGE_DAMP * math.pi / 180)
        d.CreateTargetVelocityAttr(0.0)
        s = UsdPhysics.PrismaticJoint.Define(stage, f"{root}/collar_slide")
        s.CreateBody0Rel().SetTargets([Sdf.Path(mast)])
        s.CreateBody1Rel().SetTargets([Sdf.Path(collar)])
        s.CreateAxisAttr("Z")
        s.CreateLocalPos0Attr(Gf.Vec3f(0.0, 0.0, float(COLLAR_H - MAST_L / 2)))
        s.CreateLocalRot0Attr(q0)
        s.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
        s.CreateLocalRot1Attr(q0)
        s.CreateLowerLimitAttr(-COLLAR_TRAVEL)
        s.CreateUpperLimitAttr(0.0)
        s.CreateExcludeFromArticulationAttr(True)
        d = UsdPhysics.DriveAPI.Apply(s.GetPrim(), "linear")
        d.CreateTypeAttr("force")
        d.CreateStiffnessAttr(COLLAR_K)
        d.CreateDampingAttr(0.3)
        d.CreateTargetPositionAttr(0.0)
        for a_, b_ in ((mast, collar), (mast, foot)):
            UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(a_)).GetFilteredPairsRel().AddTarget(Sdf.Path(b_))
        _PATHS["root"] = root
        return prim

    spawn._mast_task = True
    cablekit._spawn_cable = spawn


_install_spawn()


# -- start layout: mast on its stop leaning 30 deg right, guys laid loose on the table -----------------------------------
def _quat_x(deg: float) -> np.ndarray:
    a = math.radians(deg) / 2
    return np.array([math.sin(a), 0.0, 0.0, math.cos(a)])  # xyzw


def mast_axis(lean_deg: float) -> tuple[np.ndarray, np.ndarray]:
    """Mast up axis and its y axis (world) at a lean (deg, + towards +y)."""
    a = math.radians(lean_deg)
    return np.array([0.0, math.sin(a), math.cos(a)]), np.array([0.0, math.cos(a), -math.sin(a)])


def attach_point(side: str, lean_deg: float, drop: float = 0.0) -> np.ndarray:
    u, v = mast_axis(lean_deg)
    return np.array([XM, 0.0, HZ]) + (COLLAR_H - drop) * u + SIDES[side] * (ATT_Y + 0.0005) * v


def start_paths() -> dict:
    """Waypoints of each guy's starting path (from its tie on the collar)."""
    zt = TOP + R + 0.0015
    pl = attach_point("left", LEAN0_DEG)
    pr = attach_point("right", LEAN0_DEG)
    return {
        # left guy: from the collar (above the leaning mast) straight down to the table left of the foot, then along +y
        "left": np.array([pl, (0.265, 0.045, zt), (0.262, 0.20, zt)]),
        # right guy: hangs from the collar down beside the mast, then along the table towards the robot and right
        "right": np.array([pr, pr + np.array([-0.03, -0.01, -0.10]), (0.245, -0.15, zt), (0.17, -0.19, zt)]),
    }


def _shape(emb):
    import torch

    env, base = emb._env, emb._env.unwrapped
    _ST["env"] = env
    origin = base.scene.env_origins[0]
    dev = base.device
    u, _ = mast_axis(LEAN0_DEG)
    hinge = np.array([XM, 0.0, HZ])
    for obj, dist in ((MAST.name, MAST_L / 2), (COLLAR.name, COLLAR_H)):
        o = base.scene[obj]
        pose = torch.zeros((1, 7), dtype=torch.float32, device=dev)
        pose[0, :3] = origin + torch.tensor(hinge + dist * u, dtype=torch.float32, device=dev)
        # hinge angle about +x: positive leans the mast towards -y
        pose[0, 3:7] = torch.tensor(_quat_x(-LEAN0_DEG), dtype=torch.float32, device=dev)
        ids = torch.tensor([0], device=dev)
        o.write_root_pose_to_sim_index(root_pose=pose, env_ids=ids)
        o.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), device=dev), env_ids=ids)
    paths = start_paths()
    for side, spec in GUYS.items():
        cableshape.preshape(emb, spec, paths[side], hold="ramp", settle_s=0, lift=0.0)
    _ST["settling"] = True
    cableshape.settle(emb, 2.5)
    _ST["settling"] = False
    _reset_episode()


cableshape.on_reset(_shape)

# -- episode state, contact monitor (robot links x mast + collar), hooks ------------------------------------------------
_ST: dict = {"env": None, "t": 0.0, "settling": False, "nan": False, "t_rel": None, "max_lean_rel": 0.0,
             "views": None, "touch": False, "polls": 0, "err": None}
TOUCH_SEP = 0.001
_ROBOT_PRIMS = ("/LeftArm/", "/RightArm/")


def _reset_episode() -> None:
    _ST.update(t=0.0, nan=False, t_rel=None, max_lean_rel=0.0, touch=False, polls=0)


def _np(x) -> np.ndarray:
    if hasattr(x, "detach"):
        return x.detach().cpu().numpy()
    return np.asarray(x.numpy() if hasattr(x, "numpy") else x)


def _body_path(stage, name: str) -> str | None:
    from pxr import UsdPhysics

    for prim in stage.Traverse():
        p = prim.GetPath().pathString
        if "/env_0/" in p and p.rsplit("/", 1)[-1] == name and prim.HasAPI(UsdPhysics.RigidBodyAPI):
            return p
    return None


def _make_views(mgr) -> None:
    from isaaclab.sim.utils.stage import get_current_stage

    sv = mgr.get_physics_sim_view()
    if sv is None:
        return
    stage = get_current_stage()
    robot = [p.GetPath().pathString for p in stage.Traverse()
             if "/env_0/" in p.GetPath().pathString and any(r in p.GetPath().pathString for r in _ROBOT_PRIMS)
             and "PhysxContactReportAPI" in p.GetAppliedSchemas()]
    targets = [_body_path(stage, n) for n in (MAST.name, COLLAR.name)]
    if not robot or None in targets:
        raise RuntimeError(f"mast contact monitor: robot {len(robot)}, targets {targets}")
    v = sv.create_rigid_contact_view(robot, filter_patterns=[targets] * len(robot),
                                     max_contact_data_count=64 * len(robot) * len(targets))
    _ST["views"] = (v, len(robot), len(targets))
    print(f"[mast] contact monitor: {len(robot)} robot links x mast + collar", flush=True)


def _poll_contacts(mgr) -> None:
    if _ST["settling"]:
        return
    try:
        if _ST["views"] is None:
            _make_views(mgr)
        if _ST["views"] is None:
            return
        v, ns, nf = _ST["views"]
        _, _, _, sep, counts, starts = v.get_contact_data(dt=mgr.get_physics_dt())
        counts = _np(counts).reshape(ns, nf)
        _ST["polls"] += 1
        if not counts.any():
            return
        starts = _np(starts).reshape(ns, nf)
        sep = _np(sep).reshape(-1)
        for i, j in zip(*np.nonzero(counts)):
            s = int(starts[i, j])
            if float(sep[s:s + int(counts[i, j])].min()) <= TOUCH_SEP and not _ST["touch"]:
                _ST["touch"] = True
                print(f"[mast] robot touched the {'mast' if j == 0 else 'collar'} (poll {_ST['polls']})", flush=True)
    except Exception as e:  # pragma: no cover
        if _ST["err"] is None:
            _ST["err"] = repr(e)
            print(f"[mast] contact monitor error: {e!r}", flush=True)


_PHYSX_MGR = "isaaclab_physx.physics.physx_manager"


def _patch_manager(mod) -> None:
    mgr = mod.PhysxManager
    if getattr(mgr.step, "_mast_hooked", False):
        return
    base_cfg = mgr._configure_physics.__func__
    base_step = mgr.step.__func__

    def _configure_physics(cls) -> None:
        base_cfg(cls)
        try:
            import carb

            carb.settings.get_settings().set_bool("/physics/disableContactProcessing", False)
        except Exception:  # pragma: no cover
            pass

    def step(cls) -> None:
        base_step(cls)
        _poll_contacts(cls)

    step._mast_hooked = True
    mgr._configure_physics = classmethod(_configure_physics)
    mgr.step = classmethod(step)


class _MgrFinder(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path, target=None):
        if fullname != _PHYSX_MGR:
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
            _patch_manager(module)

        spec.loader.exec_module = exec_module
        return spec


def _install_hooks() -> None:
    if _PHYSX_MGR in sys.modules:
        _patch_manager(sys.modules[_PHYSX_MGR])
    elif not any(isinstance(f, _MgrFinder) for f in sys.meta_path):
        sys.meta_path.insert(0, _MgrFinder())
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
        return
    cls = ie.YamArenaEmbodiment
    if not getattr(cls.step, "_mast_hooked", False):
        base_step = cls.step

        def step(self, *a, **k):
            _ST["env"] = self._env
            out = base_step(self, *a, **k)
            _ST["t"] = float(self._t)
            return out

        step._mast_hooked = True
        cls.step = step
    if not getattr(cls.reset, "_mast_hooked", False):
        base_reset = cls.reset

        def reset(self, *a, **k):
            _ST["views"] = None
            _ST["env"] = self._env
            out = base_reset(self, *a, **k)
            _ST["env"] = self._env
            _reset_episode()
            return out

        reset._mast_hooked = True
        cls.reset = reset


_install_hooks()


def grippers() -> list[np.ndarray]:
    env = _ST["env"]
    if env is None:
        return []
    from yam_arena.runtime import tcp_world

    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
    return [np.asarray(tcp_world(base.scene[k]), dtype=np.float64) - origin for k in ("left_robot", "right_robot")]


# -- geometry -------------------------------------------------------------------------------------------------------------
def _rot(q) -> np.ndarray:
    x, y, z, w = (float(t) for t in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def lean_deg(objects) -> float:
    """The mast's lean from plumb (deg, + towards the robot's left)."""
    u = _rot(objects[MAST.name].quat_xyzw)[:, 2]
    return math.degrees(math.atan2(u[1], u[2]))


def collar_drop(objects) -> float:
    """How far (m) the collar is pulled down its spring from the top stop."""
    m, c = objects[MAST.name], objects[COLLAR.name]
    u = _rot(m.quat_xyzw)[:, 2]
    top = np.asarray(m.pos, dtype=float) + (COLLAR_H - MAST_L / 2) * u
    return float(np.dot(top - np.asarray(c.pos, dtype=float), u))


def ring_pose(side: str):
    """(centre, rotation) of a guy's ring plug body (the ring's inner bar), env frame, or None offline."""
    env = _ST["env"]
    if env is None:
        return None
    base = env.unwrapped
    art = base.scene[GUYS[side].name]
    names = list(art.body_names)
    i = names.index("end_plug")
    t = lambda a: getattr(a, "torch", a)  # noqa: E731
    origin = _np(t(base.scene.env_origins)[0]).astype(np.float64)
    p = _np(t(art.data.body_pos_w)[0, i]).astype(np.float64) - origin
    q = _np(t(art.data.body_quat_w)[0, i]).astype(np.float64)  # xyzw
    return p, _rot(q)


def ring_points(side: str) -> np.ndarray:
    """Hole centre, bail centre and the four bar centres of a ring (world), for release checks."""
    rp = ring_pose(side)
    if rp is None:
        return np.zeros((0, 3))
    c, Rm = rp
    loc = [(HOLE_C, 0, 0), (HOLE_C, 0, BAIL_UP * (BAIL_H + 0.001)), (0, 0, 0), (BAR + HOLE, 0, 0),
           (HOLE_C, (RING_W - BAR) / 2, 0), (HOLE_C, -(RING_W - BAR) / 2, 0)]
    return np.array([c + Rm @ np.array(v, dtype=float) for v in loc])


def bail_point(side: str) -> np.ndarray:
    return ring_points(side)[1]


def hooked_peg(side: str) -> int | None:
    """Index of the peg (on this ring's own side) whose post passes through the ring's hole, or None. The post's
    axis must cross the ring's plane inside the hole (2 mm in from its edges), below the nose."""
    rp = ring_pose(side)
    if rp is None:
        return None
    c, Rm = rp
    n = Rm[:, 2]
    if abs(n[2]) < 0.15:  # ring standing on edge: the post cannot pass through its hole
        return None
    for i, y in enumerate(PEG_Y[side]):
        z = c[2] + (n[0] * (c[0] - XM) + n[1] * (c[1] - y)) / n[2]  # where the post's axis meets the ring plane
        if not (TOP - 0.003 <= z <= TOP + NOSE_Z0):
            continue
        loc = Rm.T @ (np.array([XM, y, z]) - c)
        if BAR / 2 + 0.002 <= loc[0] <= BAR / 2 + HOLE - 0.002 and abs(loc[1]) <= HOLE / 2 - 0.002:
            return i
    return None


def _dense(poly: np.ndarray, step: float = 0.004) -> np.ndarray:
    out = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        k = min(200, max(1, int(np.linalg.norm(b - a) / step)))
        out.extend(a + (b - a) * (j / k) for j in range(1, k + 1))
    return np.array(out)


RELEASE = 0.03  # let go: both TCPs at least 3 cm from every point of both guys and both rings


def released(objects) -> bool:
    g = grippers()
    if not g:
        return False
    pts = np.vstack([_dense(objects[s.name].polyline()) for s in GUYS.values()] + [ring_points(k) for k in SIDES])
    return all(float(np.linalg.norm(pts - p, axis=1).min()) >= RELEASE for p in g)


def _finite(objects) -> bool:
    m = objects[MAST.name]
    return bool(np.isfinite(m.pos).all() and np.isfinite(m.quat_xyzw).all()
                and all(np.isfinite(objects[s.name].points).all() for s in GUYS.values()))


# -- scoring --------------------------------------------------------------------------------------------------------------
PLUMB_FULL = 3.0  # deg
PLUMB_HALF = 8.0  # deg
HOLD_S = 2.0  # the mast must stay within the band for 2 s after both rings are let go


def _track(objects) -> None:
    """Latched until the next trial: a robot link touched the mast or the collar (contact monitor, every physics
    substep); a non-finite state. Also: the time since both grippers last let go of both guys and rings, and the
    largest lean since then."""
    if not _finite(objects):
        _ST["nan"] = True
        return
    if released(objects):
        if _ST["t_rel"] is None:
            _ST["t_rel"] = _ST["t"]
            _ST["max_lean_rel"] = 0.0
        _ST["max_lean_rel"] = max(_ST["max_lean_rel"], abs(lean_deg(objects)))
    else:
        _ST["t_rel"] = None


def score(objects) -> float:
    """0 if: a robot link ever touched the mast or the guy collar; either ring is not on a peg on its own side; a
    gripper is within 3 cm of either guy or ring at the end, or let go less than 2 s ago; the mast leaned more than 8
    deg from plumb at any time in those 2 s. Otherwise 1.0 if it stayed within 3 deg of plumb, else 0.5."""
    if not _finite(objects):
        _ST["nan"] = True
        return 0.0
    _track(objects)
    if _ST["nan"] or _ST["touch"]:
        return 0.0
    if any(hooked_peg(s) is None for s in SIDES):
        return 0.0
    if _ST["t_rel"] is None or _ST["t"] - _ST["t_rel"] < HOLD_S:
        return 0.0
    lean = max(_ST["max_lean_rel"], abs(lean_deg(objects)))
    if lean <= PLUMB_FULL:
        return 1.0
    return 0.5 if lean <= PLUMB_HALF else 0.0


def success(objects) -> bool:
    return bool(score(objects) >= 1.0 and objects[MAST.name].resting(0.01, 0.2))


def distance(objects) -> float:
    if not _finite(objects):
        return 1.0
    _track(objects)
    if _ST["touch"]:
        return 1.0
    return float(abs(lean_deg(objects)) / 30.0 + sum(0.0 if hooked_peg(s) is not None else 0.1 for s in SIDES))


def progress(objects) -> dict:
    if not _finite(objects):
        return {"mast_up_10deg": 0, "left_on_peg": 0, "right_on_peg": 0, "on_plumb_pegs": 0, "released": 0,
                "plumb_3deg": 0}
    _track(objects)
    hp = {s: hooked_peg(s) for s in SIDES}
    return {"mast_up_10deg": int(abs(lean_deg(objects)) <= 10.0), "left_on_peg": int(hp["left"] is not None),
            "right_on_peg": int(hp["right"] is not None),
            "on_plumb_pegs": int(all(hp[s] == OK_PEG[s] for s in SIDES)), "released": int(released(objects)),
            "plumb_3deg": int(abs(lean_deg(objects)) <= PLUMB_FULL)}


# -- closed-loop scripted demo (cabledemo.py) ---------------------------------------------------------------------------
GRIP_DZ = BAIL_H + 0.011  # TCP above the ring's centre plane when pinching the bail bar
GRIP_Z = TOP + RING_T / 2 + GRIP_DZ  # TCP height pinching the bail of a ring lying flat on the table
CARRY_Z = TOP + 0.075 + GRIP_DZ
HOLD_Z = TOP + 0.045 + GRIP_DZ
ARMI = {"left": 0, "right": 1}


def _log(ctx, tag=""):
    st = ctx.states()
    if not _finite(st):
        print(f"[t={_ST['t']:5.2f}] {tag}: NON-FINITE", flush=True)
        return
    _track(st)
    rings = {s: np.round(ring_points(s)[0] - [0, 0, TOP], 3).tolist() + [round(float(ring_pose(s)[1][2, 2]), 2)]
             for s in SIDES}
    print(f"[t={_ST['t']:5.2f}] {tag}: lean={lean_deg(st):+.2f} drop={collar_drop(st) * 1000:.1f}mm "
          f"hooked={[hooked_peg(s) for s in SIDES]} holes={rings} rel={released(st)} touch={_ST['touch']} "
          f"maxlean_rel={_ST['max_lean_rel']:.2f} score={score(st):.2f}", flush=True)


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def _grip_yaw(side: str) -> tuple[float, float]:
    """(yaw to pinch the bail now, yaw that turns the ring to point straight away from the mast). The fingers close
    across the bail bar (along the ring's own axis)."""
    c, Rm = ring_pose(side)
    h = math.atan2(Rm[1, 0], Rm[0, 0])  # ring heading (plug x)
    yg = _wrap(h - math.pi / 2)
    if yg > math.pi / 2:
        yg -= math.pi
    elif yg < -math.pi / 2:
        yg += math.pi
    yf = yg + _wrap(SIDES[side] * math.pi / 2 - h)
    return yg, yf


def _take(arm, ctx, side):
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    yg, yf = _grip_yaw(side)
    p = bail_point(side)
    yield from arm.move((p[0], p[1], CARRY_Z), yaw=yg, finger=FINGER_OPEN, duration=1.2)
    p = bail_point(side)
    yield from arm.move((p[0], p[1], GRIP_Z), yaw=yg, duration=0.8)
    yield from arm.grip(FINGER_CLOSED, 0.5)
    _TAKE[side] = (yg, yf)


_TAKE: dict = {}


def _hole_offset(side: str, yaw: float) -> np.ndarray:
    """Hole centre minus TCP (xy) now, rotated into the gripper's yaw frame."""
    g = grippers()[ARMI[side]]
    d = ring_points(side)[0][:2] - g[:2]
    c, s = math.cos(-yaw), math.sin(-yaw)
    return np.array([c * d[0] - s * d[1], s * d[0] + c * d[1]])


def _tcp_for_hole(side: str, hole_xy, yaw_now: float, yaw_new: float) -> np.ndarray:
    off = _hole_offset(side, yaw_now)
    c, s = math.cos(yaw_new), math.sin(yaw_new)
    return np.asarray(hole_xy, dtype=float) - np.array([c * off[0] - s * off[1], s * off[0] + c * off[1]])


def hole_target(side: str, peg: int) -> np.ndarray:
    """Where to drop a ring's hole centre over a peg: centred on the post + nose footprint."""
    sgn = SIDES[side]
    return np.array([XM, PEG_Y[side][peg] + sgn * (NOSE_L / 2 + 0.003)])


def _carry(arm, ctx, side, peg, z, duration=1.5, steps=1):
    _, yf = _TAKE[side]
    for k in range(steps):
        g = _tcp_for_hole(side, hole_target(side, peg), arm._yaw, yf)
        yield from arm.move((g[0], g[1], z), yaw=yf, duration=duration / steps)


def _haul(arm, ctx, side, peg):
    """Carry the ring out over its peg in steps (the mast comes up as the line draws it), closed loop on the hole."""
    _, yf = _TAKE[side]
    for k in range(10):
        g = _tcp_for_hole(side, hole_target(side, peg), arm._yaw, yf)
        t = grippers()[ARMI[side]]
        d = g - t[:2]
        n = float(np.linalg.norm(d))
        if n < 0.002:
            break
        step = d if n < 0.03 else d / n * 0.03
        yield from arm.move((t[0] + step[0], t[1] + step[1], CARRY_Z), yaw=yf, duration=0.5)
    _log(ctx, f"{side} hauled over peg {peg}")


def _drop(arm, ctx, side, peg):
    """Lower the ring over the peg to the table, closed loop on the hole, then let go."""
    from yam_arena.constants import FINGER_OPEN

    _, yf = _TAKE[side]
    for z in (TOP + 0.045 + GRIP_DZ, TOP + 0.022 + GRIP_DZ, GRIP_Z + 0.001):
        g = _tcp_for_hole(side, hole_target(side, peg), arm._yaw, yf)
        yield from arm.move((g[0], g[1], z), yaw=yf, duration=0.8)
    _log(ctx, f"{side} ring down at peg {peg}")
    yield from arm.grip(FINGER_OPEN, 0.5)
    t = grippers()[ARMI[side]]
    yield from arm.move((t[0], t[1], TOP + 0.14), yaw=yf, duration=0.8)


def _park(ctx):
    yield from ctx.both(ctx.left.move((0.17, 0.27, TOP + 0.16), yaw=0.0, duration=1.2),
                        ctx.right.move((0.17, -0.27, TOP + 0.16), yaw=0.0, duration=1.2))


def demo_script(ctx):
    """MAST_DEMO=correct (default). Shortcut variants (all must score 0): none (wait) | hold (haul the mast plumb and
    hold both rings over their pegs without pegging) | holdend (correct, but keep pinching both rings) | onearm (the
    right arm pegs its ring; the left ring is left on the table) | onearm_left (the left arm alone hauls the mast up
    and pegs its ring; the right ring is left on the table) | mirror (both rings on the inner pegs) | middle (both
    on the middle pegs) | push (a gripper pushes the mast upright, then the rings are pegged correctly). Tier check:
    tier05 (left ring correct, right ring on its middle peg: the mast stands 3-8 deg out). pair: any peg pair from MAST_PEGS (the outcome table)."""
    mode = os.environ.get("MAST_DEMO", "correct")
    L, Rt = ctx.left, ctx.right
    peg = dict(OK_PEG)
    if mode == "mirror":
        peg = {"left": 0, "right": 0}
    elif mode == "middle":
        peg = {"left": 1, "right": 1}
    elif mode == "tier05":
        peg = {"left": 2, "right": 1}
    elif mode == "pair":  # any peg pair, MAST_PEGS="<left index>,<right index>" (0 = the peg nearest the mast)
        i, j = (int(v) for v in os.environ.get("MAST_PEGS", "2,0").split(","))
        peg = {"left": i, "right": j}
    _log(ctx, f"start mode={mode} pegs={peg} len={LINE_LEN}")
    if mode == "none":
        for _ in range(8):
            yield from L.wait(1.0)
            _log(ctx, "waiting")
        return
    if mode == "push":
        yield from Rt.move((XM - 0.002, -0.17, TOP + 0.13), yaw=0.0, duration=1.5)
        yield from Rt.move((XM - 0.002, -0.06, TOP + 0.17), yaw=0.0, duration=2.0)
        _log(ctx, "pushed")
    if mode == "onearm":
        yield from _take(Rt, ctx, "right")
        yield from _carry(Rt, ctx, "right", peg["right"], CARRY_Z, 1.5, 2)
        _log(ctx, "right taken")
        yield from _drop(Rt, ctx, "right", peg["right"])
        yield from _park(ctx)
        for _ in range(5):
            yield from L.wait(1.0)
            _log(ctx, "end")
        return
    if mode == "onearm_left":
        yield from _take(L, ctx, "left")
        yield from _haul(L, ctx, "left", peg["left"])
        yield from _carry(L, ctx, "left", peg["left"], HOLD_Z, 0.8)
        yield from _drop(L, ctx, "left", peg["left"])
        yield from _park(ctx)
        for _ in range(5):
            yield from L.wait(1.0)
            _log(ctx, "end")
        return
    yield from ctx.both(_take(L, ctx, "left"), _take(Rt, ctx, "right"))
    _log(ctx, "both taken")
    # right ring held over its peg as the catch; left hauls the mast up and over to it
    yield from _carry(Rt, ctx, "right", peg["right"], HOLD_Z + 0.03, 1.6, 2)
    _log(ctx, "right holding")
    yield from _haul(L, ctx, "left", peg["left"])
    yield from ctx.both(_carry(L, ctx, "left", peg["left"], HOLD_Z, 0.8), _carry(Rt, ctx, "right", peg["right"], HOLD_Z, 0.8))
    _log(ctx, "both over pegs")
    if mode == "hold":
        for _ in range(5):
            yield from L.wait(1.0)
            _log(ctx, "holding")
        return
    if mode == "holdend":
        from yam_arena.constants import FINGER_OPEN  # noqa: F401

        for side, arm in (("left", L), ("right", Rt)):
            _, yf = _TAKE[side]
            for z in (TOP + 0.045 + GRIP_DZ, TOP + 0.022 + GRIP_DZ, GRIP_Z + 0.001):
                g = _tcp_for_hole(side, hole_target(side, peg[side]), arm._yaw, yf)
                yield from arm.move((g[0], g[1], z), yaw=yf, duration=0.8)
        for _ in range(5):
            yield from L.wait(1.0)
            _log(ctx, "holding down")
        return
    yield from _drop(L, ctx, "left", peg["left"])
    _log(ctx, "left pegged")
    yield from _drop(Rt, ctx, "right", peg["right"])
    _log(ctx, "right pegged")
    yield from _park(ctx)
    for _ in range(6):
        yield from L.wait(0.5)
        _log(ctx, "end")
