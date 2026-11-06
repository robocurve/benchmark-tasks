"""A model navigation lock on the station table. A grey lock wall (5 cm thick, 10.5 cm tall, 64 cm long, with a
rounded 2.5 cm coping on top) runs across the table 32-37 cm in front of the robot. Beyond it, in the lock chamber, an
orange model boat (8 x 50 x 2.5 cm) lies with its fenders against the wall. Two 6 mm lines run from cleats near the
boat's ends (bow on the robot's left at y +23 cm, stern on its right at y -23 cm) up over the coping, down the wall
face and along the table towards the robot. Each line carries four identical white toggles 5 cm apart. Outboard of
each line stands a slotted cleat: two black posts 6 cm tall with a 12 mm slot between them; a toggle cannot pass the
slot.

Three seconds after the start the lock fills: the boat rises 11 cm in 12 s (a scripted lift carriage). The boat is
free to move away from the wall and to swing (yaw), and from the start of the fill a steady 0.08 N push (the filling
water) drives it away from the wall unless its lines hold it. As the boat rises each line slackens by about 7 cm, so
both lines must be taken in continuously during the same 12 s to keep the boat at the wall. When the boat stops
rising, each line is made fast by laying it into its slotted cleat with the toggle that leaves the line just taut on
the robot's side of the slot, and both arms let go. Each line and its cleat can be reached by one arm only.

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
from cablekit import cable  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = ("The lock is filling and the orange boat will rise. Using only its two lines, keep the boat against "
               "the lock wall the whole time it rises. When it stops rising, make each line fast in its slotted "
               "cleat, taut, with the boat against the wall, and let go of both lines.")

# -- layout (m, world frame: x forward, y left, z up) ------------------------------------------------------------------
CONCRETE = (0.62, 0.62, 0.60)
ORANGE = (0.95, 0.45, 0.10)
ROPE = (0.10, 0.22, 0.55)
TOGGLE_C = (0.95, 0.95, 0.92)
POST_C = (0.08, 0.08, 0.09)
WATER_C = (0.25, 0.48, 0.70)

WALL_X0, WALL_X1, WALL_HY, WALL_H = 0.32, 0.37, 0.32, 0.105
COPE_R = 0.025
COPE_MU = float(os.environ.get("LOCK_COPE_MU", "0.8"))  # rough concrete coping
COPE_X, COPE_Z = (WALL_X0 + WALL_X1) / 2, TOP + WALL_H  # coping (half-round on the wall top): axis along y
WALL = Box("lock_wall", pos=(COPE_X, 0.0, TOP + WALL_H / 2), size=(WALL_X1 - WALL_X0, 2 * WALL_HY, WALL_H),
           color=CONCRETE)

FENDER_X = 0.385  # the boat's wall-side edge when it rests on its fenders (1.5 cm from the wall face)
BOAT_L, BOAT_W, BOAT_H, BOAT_Z0 = 0.08, 0.50, 0.025, 0.005  # x, y, z size; bottom 5 mm above the chamber floor
BOAT_C0 = (FENDER_X + BOAT_L / 2, 0.0, TOP + BOAT_Z0 + BOAT_H / 2)
BOAT = Box("boat", pos=BOAT_C0, size=(BOAT_L, BOAT_W, BOAT_H), mass=0.15, color=ORANGE)
DECK_Z = TOP + BOAT_Z0 + BOAT_H

RISE, T_RISE0, T_RISE = 0.11, 3.0, 12.0  # the boat rises 11 cm, from t 3 s to t 15 s (control time)
T_WATCH = T_RISE0 + T_RISE + 1.0  # the "keep it at the wall" rule is watched until 1 s after the rise ends
X_TRAVEL = 0.10  # the boat can drift up to 10 cm off the wall
PUSH_F, PUSH_DAMP = float(os.environ.get("LOCK_PUSH", "0.08")), 2.0  # outward push (N) from the rise on; drag N s/m
FENDER_K, FENDER_D = 400.0, 8.0  # fender stop: 400 N/m spring, 8 N s/m
CAR_MASS = 20.0  # the lift carriage: heavy, gravity-free, velocity-driven along its vertical slide

R = 0.003
CLEAT_X, CLEAT_Y = FENDER_X + 0.010, 0.23
LINE_Z0 = DECK_Z + R + 0.0005
SEG = 0.02

# slotted cleats (make-fast points), inboard of each line
FORK_X = float(os.environ.get("LOCK_FORK_X", "0.180"))
FORK_Y = 0.275  # outboard of each line
POST, POST_H, SLOT = 0.012, 0.060, 0.012
TOG_L, TOG_W, TOG_H = 0.010, 0.028, 0.008


def _posts(tag: str, sgn: float) -> list:
    y = sgn * FORK_Y
    off = SLOT / 2 + POST / 2
    return [Box(f"cleat_post_{tag}_in", pos=(FORK_X, y - off, TOP + POST_H / 2), size=(POST, POST, POST_H), color=POST_C),
            Box(f"cleat_post_{tag}_out", pos=(FORK_X, y + off, TOP + POST_H / 2), size=(POST, POST, POST_H), color=POST_C)]


POSTS = _posts("bow", 1.0) + _posts("stern", -1.0)

# -- the lines' starting path: cleat -> up over the coping -> down the wall face -> along the table towards the robot ---
RC = COPE_R + R + 0.003  # centreline radius over the coping (3 mm clear; 2 cm chords cut 1.6 mm inside)
RB = 0.03  # bend radius at the wall's foot
Z_TAB = TOP + R + 0.0015


def _tangent_up(px: float, pz: float) -> tuple[float, float]:
    """Angle (rad, from +x about the coping axis) of the tangent point from (px, pz), and the tangent length."""
    dx, dz = px - COPE_X, pz - COPE_Z
    d = math.hypot(dx, dz)
    return math.atan2(dz, dx) + math.acos(RC / d), math.sqrt(d * d - RC * RC)


def start_path_xz() -> np.ndarray:
    a, _ = _tangent_up(CLEAT_X, LINE_Z0)
    pts = [(CLEAT_X, LINE_Z0)]
    pts += [(COPE_X + RC * math.cos(th), COPE_Z + RC * math.sin(th)) for th in np.linspace(a, math.pi, 48)]
    xv = COPE_X - RC
    pts.append((xv, Z_TAB + RB))
    pts += [(xv - RB + RB * math.cos(th), Z_TAB + RB - RB * math.sin(th)) for th in np.linspace(0, math.pi / 2, 24)[1:]]
    pts.append((xv - RB - 0.70, Z_TAB))
    return np.array(pts)


def _path_s(p: np.ndarray) -> np.ndarray:
    return np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))]


def end_length(x_end: float, dy: float, z_end: float) -> float:
    """Line length from the cleat to the point (x_end, dy to the side, z_end), boat risen and on its fenders, line taut
    over the coping."""
    a1, t1 = _tangent_up(CLEAT_X, LINE_Z0 + RISE)
    dx, dz = x_end - COPE_X, z_end - COPE_Z
    d = math.hypot(dx, dz)
    a2 = (math.atan2(dz, dx) - math.acos(RC / d)) % (2 * math.pi)
    return t1 + RC * (a2 - a1) + math.hypot(math.sqrt(d * d - RC * RC), dy)


# toggles: the correct one lies just on the robot side of the slot when the risen boat is on its fenders
S_CAL = 0.0009  # measured: a made-fast line is 0.9 mm shorter than the ideal tangent geometry
TOG_Z_END = 0.027  # toggle height against the posts the calibration aims at (it can sit 0-5 cm up: +-1.2 cm of line)
TOG_S_OK = round(end_length(FORK_X - POST / 2 - TOG_L / 2 - 0.002, CLEAT_Y - FORK_Y, TOP + TOG_Z_END) - S_CAL, 4)
TOG_SPACING = 0.05
TOGGLE_S = [TOG_S_OK - TOG_SPACING, TOG_S_OK, TOG_S_OK + TOG_SPACING, TOG_S_OK + 2 * TOG_SPACING]
OK_TOGGLE = 1  # index into TOGGLE_S (the second toggle from the boat)
LINE_LEN = math.ceil((TOGGLE_S[-1] + 0.05) / SEG) * SEG
N_SEG = int(round(LINE_LEN / SEG))
TOG_SEG = [int(s // SEG) for s in TOGGLE_S]
TOG_DX = [s - (i + 0.5) * SEG for s, i in zip(TOGGLE_S, TOG_SEG)]  # offset along the segment from its centre


def _line(name: str, sgn: float):
    return cable(name, start=(CLEAT_X, sgn * CLEAT_Y, LINE_Z0), yaw=0.0, length=LINE_LEN, radius=R, segment=SEG,
                 cable_mass=0.025, bend_stiffness=0.004, bend_damping=0.0005, friction=0.8, color=ROPE)


BOW = _line("bow_line", 1.0)
STERN = _line("stern_line", -1.0)
LINES = {"bow": (BOW, 1.0), "stern": (STERN, -1.0)}

OBJECTS = [WALL, BOAT, *POSTS, BOW, STERN]

TWIN_EXTRA = [
    {"name": "coping", "kind": "body", "size_mm": [50, 640, 25], "colour": "grey",
     "pose": [COPE_X, 0.0, WALL_H], "note": "half-round 25 mm radius on the wall top (PVC pipe half, painted)"},
    {"name": "lift_carriage", "kind": "joint", "real": "vertical linear slide driven by a lead-screw stepper",
     "note": "raises the boat 110 mm in 12 s; the boat sits on a cross slide (x, 0-100 mm) with a yaw bearing"},
    {"name": "outward_push", "kind": "joint", "real": "~0.1 N constant-force spring (or 10 g weight on a pulley)",
     "note": "pushes the boat away from the wall; the fender stop is the cross slide's end stop 15 mm from the wall"},
    {"name": "cleat_swivel", "kind": "joint", "real": "line tied to a small deck cleat", "note": "one per line"},
    *({"name": f"toggle_{k}_{i}", "kind": "body", "size_mm": [10, 28, 8], "colour": "white",
       "note": f"toggle on the {k} line, {round(TOGGLE_S[i] * 1000)} mm from the cleat"} for k in LINES for i in range(4)),
    {"name": "water", "kind": "decal", "size_mm": [250, 640, 1], "colour": "blue", "note": "rises with the boat"},
]

VARIATION = {"cable:bow_line": {"dx": 0.0, "dy": 0.0, "yaw": 0.0}, "cable:stern_line": {"dx": 0.0, "dy": 0.0, "yaw": 0.0},
             "obj:boat": {"dx": 0.0, "dy": 0.0, "yaw": 0.0}}  # lines tied to the boat and laid over the wall: fixed


# -- spawn hook: coping, lift carriage + float joint, cleat swivels, toggles ------------------------------------------
_PATHS: dict = {}


def _install_spawn() -> None:
    orig = cablekit._spawn_cable
    if getattr(orig, "_lock_task", False):
        return

    def spawn(prim_path, cfg, *a, **k):
        prim = orig(prim_path, cfg, *a, **k)
        name = getattr(cfg, "cable_name", "")
        if name not in (BOW.name, STERN.name):
            return prim
        import isaaclab.sim as sim_utils
        from isaaclab.sim.utils import bind_physics_material, bind_visual_material
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics
        from yam_arena.station import linear_rgb

        stage = get_current_stage()
        root = str(prim_path).rsplit("/", 1)[0]
        boat = f"{root}/{BOAT.name}"
        spec = BOW if name == BOW.name else STERN
        sgn = 1.0 if spec is BOW else -1.0

        def look(path, rgb):
            m = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(rgb), roughness=0.6)
            m.func(path, m)
            return path

        # cleat swivel: the line's start tip is tied to the deck (ball joint, translations locked)
        j = UsdPhysics.Joint.Define(stage, f"{prim_path}/cleat_swivel")
        j.CreateBody0Rel().SetTargets([Sdf.Path(boat)])
        j.CreateBody1Rel().SetTargets([Sdf.Path(f"{prim_path}/seg_0")])
        j.CreateLocalPos0Attr(Gf.Vec3f(float(CLEAT_X - BOAT_C0[0]), float(sgn * CLEAT_Y), float(LINE_Z0 - BOAT_C0[2])))
        j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLocalPos1Attr(Gf.Vec3f(-spec.seg_len / 2, 0.0, 0.0))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        for dof in ("transX", "transY", "transZ"):
            lim = UsdPhysics.LimitAPI.Apply(j.GetPrim(), dof)
            lim.CreateLowAttr(1.0)
            lim.CreateHighAttr(-1.0)
        j.CreateExcludeFromArticulationAttr(True)
        fp = UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(boat))
        for i in range(2):
            fp.GetFilteredPairsRel().AddTarget(Sdf.Path(f"{prim_path}/seg_{i}"))
        # cleat (visual only)
        cl = UsdGeom.Cube.Define(stage, f"{boat}/cleat_{'bow' if sgn > 0 else 'stern'}")
        cl.CreateSizeAttr(1.0)
        cl.AddTranslateOp().Set(Gf.Vec3d(float(CLEAT_X - BOAT_C0[0]), float(sgn * CLEAT_Y), BOAT_H / 2 + 0.003))
        cl.AddScaleOp().Set(Gf.Vec3f(0.010, 0.030, 0.006))
        bind_visual_material(str(cl.GetPath()), look(f"{prim_path}/Looks/cleat", POST_C), stage=stage)
        # toggles: extra colliders on their segments (part of the segment body)
        tl = look(f"{prim_path}/Looks/toggle", TOGGLE_C)
        for i, (si, dx) in enumerate(zip(TOG_SEG, TOG_DX)):
            g = UsdGeom.Cube.Define(stage, f"{prim_path}/seg_{si}/toggle_{i}")
            g.CreateSizeAttr(1.0)
            g.AddTranslateOp().Set(Gf.Vec3d(float(dx), 0.0, 0.001))
            g.AddScaleOp().Set(Gf.Vec3f(TOG_L, TOG_W, TOG_H))
            UsdPhysics.CollisionAPI.Apply(g.GetPrim())
            cp = PhysxSchema.PhysxCollisionAPI.Apply(g.GetPrim())
            cp.CreateContactOffsetAttr(0.002)
            cp.CreateRestOffsetAttr(0.0)
            bind_physics_material(str(g.GetPath()), f"{prim_path}/Looks/cable_physics", stage=stage)
            bind_visual_material(str(g.GetPath()), tl, stage=stage)
        if spec is not STERN:
            return prim
        # ---- once (the stern line is spawned last): coping, carriage, float joint, water ----
        cmat = f"{root}/lock_looks/coping_physics"
        pm = sim_utils.RigidBodyMaterialCfg(static_friction=COPE_MU, dynamic_friction=COPE_MU)
        pm.func(cmat, pm)
        PhysxSchema.PhysxMaterialAPI.Apply(stage.GetPrimAtPath(cmat)).CreateFrictionCombineModeAttr().Set("min")
        cx = UsdGeom.Xform.Define(stage, f"{root}/lock_coping")
        cx.AddTranslateOp().Set(Gf.Vec3d(COPE_X, 0.0, COPE_Z))
        cy = UsdGeom.Cylinder.Define(stage, f"{root}/lock_coping/geom")
        cy.CreateAxisAttr("Y")
        cy.CreateRadiusAttr(COPE_R)
        cy.CreateHeightAttr(2 * WALL_HY)
        UsdPhysics.CollisionAPI.Apply(cy.GetPrim())
        PhysxSchema.PhysxCollisionAPI.Apply(cy.GetPrim()).CreateContactOffsetAttr(0.002)
        bind_physics_material(str(cy.GetPath()), cmat, stage=stage)
        bind_visual_material(str(cy.GetPath()), look(f"{root}/lock_looks/concrete", CONCRETE), stage=stage)

        car = UsdGeom.Xform.Define(stage, f"{root}/lock_carriage")
        car.AddTranslateOp().Set(Gf.Vec3d(*map(float, BOAT_C0)))
        UsdPhysics.RigidBodyAPI.Apply(car.GetPrim())
        UsdPhysics.MassAPI.Apply(car.GetPrim()).CreateMassAttr(CAR_MASS)
        crb = PhysxSchema.PhysxRigidBodyAPI.Apply(car.GetPrim())
        crb.CreateDisableGravityAttr(True)
        crb.CreateSolverPositionIterationCountAttr(32)
        wpos = UsdGeom.Xformable(car.GetPrim()).ComputeLocalToWorldTransform(0).ExtractTranslation()
        lift = UsdPhysics.PrismaticJoint.Define(stage, f"{root}/lock_carriage/lift_slide")  # body0 empty = world
        lift.CreateBody1Rel().SetTargets([Sdf.Path(f"{root}/lock_carriage")])
        lift.CreateAxisAttr("Z")
        lift.CreateLocalPos0Attr(Gf.Vec3f(float(wpos[0]), float(wpos[1]), float(wpos[2])))
        lift.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        lift.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
        lift.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        lift.CreateLowerLimitAttr(0.0)
        lift.CreateUpperLimitAttr(RISE + 0.003)
        lift.CreateExcludeFromArticulationAttr(True)
        wt = UsdGeom.Cube.Define(stage, f"{root}/lock_carriage/water")  # visual only (no collider)
        wt.CreateSizeAttr(1.0)
        w_x0, w_x1, w_top = WALL_X1 + 0.001, 0.62, BOAT_Z0 + 0.012
        wt.AddTranslateOp().Set(Gf.Vec3d((w_x0 + w_x1) / 2 - BOAT_C0[0], 0.0, TOP + (w_top - 0.13) / 2 - BOAT_C0[2]))
        wt.AddScaleOp().Set(Gf.Vec3f(w_x1 - w_x0, 2 * WALL_HY, 0.13 + w_top))  # 25 x 64 cm
        bind_visual_material(str(wt.GetPath()), look(f"{root}/lock_looks/water", WATER_C), stage=stage)

        fj = UsdPhysics.Joint.Define(stage, f"{root}/lock_carriage/float_joint")
        fj.CreateBody0Rel().SetTargets([Sdf.Path(f"{root}/lock_carriage")])
        fj.CreateBody1Rel().SetTargets([Sdf.Path(boat)])
        for attr in (fj.CreateLocalPos0Attr, fj.CreateLocalPos1Attr):
            attr(Gf.Vec3f(0.0, 0.0, 0.0))
        for attr in (fj.CreateLocalRot0Attr, fj.CreateLocalRot1Attr):
            attr(Gf.Quatf(1, 0, 0, 0))
        jp = fj.GetPrim()
        sl = PhysxSchema.PhysxLimitAPI.Apply(jp, "transX")  # soft stops: the fenders give a little under a hard pull
        sl.CreateStiffnessAttr(FENDER_K)
        sl.CreateDampingAttr(FENDER_D)
        for dof, lo, hi in (("transY", 1.0, -1.0), ("transZ", 1.0, -1.0), ("rotX", 1.0, -1.0), ("rotY", 1.0, -1.0),
                            ("transX", 0.0, X_TRAVEL), ("rotZ", -25.0, 25.0)):
            lim = UsdPhysics.LimitAPI.Apply(jp, dof)
            lim.CreateLowAttr(lo)
            lim.CreateHighAttr(hi)
        d = UsdPhysics.DriveAPI.Apply(jp, "transX")  # water drag only; the outward push is applied per substep
        d.CreateTypeAttr("force")
        d.CreateStiffnessAttr(0.0)
        d.CreateDampingAttr(PUSH_DAMP)
        d.CreateTargetPositionAttr(0.0)
        d = UsdPhysics.DriveAPI.Apply(jp, "rotZ")
        d.CreateTypeAttr("force")
        d.CreateStiffnessAttr(0.004 * math.pi / 180)
        d.CreateDampingAttr(0.01 * math.pi / 180)
        d.CreateTargetPositionAttr(0.0)
        fj.CreateExcludeFromArticulationAttr(True)
        _PATHS["carriage"] = f"{root}/lock_carriage"
        _PATHS["boat"] = boat
        return prim

    spawn._lock_task = True
    cablekit._spawn_cable = spawn


_install_spawn()


def _shape(emb):
    xz = start_path_xz()
    for spec, sgn in LINES.values():
        pts = np.c_[xz[:, 0], np.full(len(xz), sgn * CLEAT_Y), xz[:, 1]]
        cableshape.preshape(emb, spec, pts, hold="ramp", settle_s=0, lift=0.0)
    cableshape.settle(emb, 2.0)


cableshape.on_reset(_shape)

# -- episode state, carriage driver, hooks ---------------------------------------------------------------------------
_ST: dict = {"env": None, "t": 0.0, "t0": 0.0, "n": 0, "active": False, "view": None, "tf0": None, "idx": None,
             "drifted": False, "max_drift": 0.0, "hand": False, "nan": False, "err": None}


def _reset_episode() -> None:
    _ST.update(t=0.0, t0=0.0, n=0, active=False, drifted=False, max_drift=0.0, hand=False, nan=False)


def lift_z(t: float) -> float:
    return RISE * min(1.0, max(0.0, (t - T_RISE0) / T_RISE))


def _drive_carriage(mgr) -> None:
    """Every physics substep: drive the carriage along its vertical slide to lift_z(t) (velocity write with position
    correction), and push the boat away from the wall with PUSH_F from the start of the rise."""
    try:
        import warp as wp

        if _ST["view"] is None:
            path, boat = _PATHS.get("carriage"), _PATHS.get("boat")
            sv = mgr.get_physics_sim_view()
            if path is None or sv is None:
                return
            view, bview = sv.create_rigid_body_view(path), sv.create_rigid_body_view(boat)
            raw = view.get_transforms()
            _ST["dev"] = str(getattr(raw, "device", "cuda:0"))
            _ST["z0"] = float(np.asarray(raw.numpy()).reshape(-1)[2])
            _ST["idx"] = wp.array(np.array([0], dtype=np.int32), dtype=wp.int32, device=_ST["dev"])
            _ST["view"], _ST["bview"] = view, bview
            print(f"[lock] carriage view on {path} z0={_ST['z0']:.4f}; boat view {boat}", flush=True)
        dt = mgr.get_physics_dt()
        t = _ST["t0"] + _ST["n"] * dt if _ST["active"] else 0.0
        _ST["n"] += 1
        z = float(np.asarray(_ST["view"].get_transforms().numpy()).reshape(-1)[2]) - _ST["z0"]
        zt = lift_z(t + dt)
        v = (lift_z(t + dt) - lift_z(t)) / dt + 0.5 * (zt - z) / dt
        v = float(np.clip(v, -0.3, 0.3))
        _ST["car_z"] = z
        _ST["view"].set_velocities(wp.array(np.array([[0, 0, v, 0, 0, 0]], np.float32), dtype=wp.float32,
                                            device=_ST["dev"]), _ST["idx"])
        f = PUSH_F * min(1.0, max(0.0, (t - T_RISE0) / 0.5)) if _ST["active"] else 0.0
        if f > 0:
            _ST["bview"].apply_forces_and_torques_at_position(
                wp.array(np.array([[f, 0, 0]], np.float32), dtype=wp.float32, device=_ST["dev"]), None, None,
                _ST["idx"], True)
    except Exception as e:  # pragma: no cover
        if _ST["err"] is None:
            _ST["err"] = repr(e)
            print(f"[lock] carriage error: {e!r}", flush=True)


_PHYSX_MGR = "isaaclab_physx.physics.physx_manager"


def _patch_manager(mod) -> None:
    mgr = mod.PhysxManager
    if getattr(mgr.step, "_lock_hooked", False):
        return
    base_step = mgr.step.__func__

    def step(cls) -> None:
        base_step(cls)
        _drive_carriage(cls)

    step._lock_hooked = True
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
    if not getattr(cls.step, "_lock_hooked", False):
        base_step = cls.step

        def step(self, *a, **k):
            _ST["env"] = self._env
            _ST["t0"], _ST["n"], _ST["active"] = float(self._t), 0, True
            out = base_step(self, *a, **k)
            _ST["t"] = float(self._t)
            return out

        step._lock_hooked = True
        cls.step = step
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


def grippers() -> list[np.ndarray]:
    env = _ST["env"]
    if env is None:
        return []
    from yam_arena.runtime import tcp_world

    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
    return [np.asarray(tcp_world(base.scene[k]), dtype=np.float64) - origin for k in ("left_robot", "right_robot")]


# -- geometry and scoring ----------------------------------------------------------------------------------------------
DRIFT_MAX = 0.03  # during the rise: each boat end within 3 cm of its fender position, at every control step
FAST_TOL = 0.015  # at the end: a made-fast end within 1.5 cm of its fender position
HAND_DIST = 0.04  # a gripper TCP within 4 cm of the boat at any time = the boat was handled
RELEASE = 0.035  # both TCPs at least 3.5 cm from every point of both lines and every toggle


def _rot(q) -> np.ndarray:
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _finite(objects) -> bool:
    b = objects["boat"]
    ok = np.isfinite(b.pos).all() and np.isfinite(b.quat_xyzw).all()
    return bool(ok and all(np.isfinite(objects[s.name].points).all() for s, _ in LINES.values()))


def end_drift(objects) -> dict:
    """How far (m) each end of the boat's wall-side edge is from its fender position (positive = away from the wall)."""
    b = objects["boat"]
    Rm, c = _rot(b.quat_xyzw), np.asarray(b.pos, dtype=float)
    return {k: float((c + Rm @ np.array([-BOAT_L / 2, sgn * BOAT_W / 2, 0.0]))[0] - FENDER_X)
            for k, (_, sgn) in LINES.items()}


def _boat_dist(objects, p) -> float:
    b = objects["boat"]
    loc = _rot(b.quat_xyzw).T @ (np.asarray(p, dtype=float) - np.asarray(b.pos, dtype=float))
    half = np.array([BOAT_L, BOAT_W, BOAT_H]) / 2
    return float(np.linalg.norm(np.maximum(np.abs(loc) - half, 0.0)))


def toggles(objects, key) -> np.ndarray:
    """Centres (4, 3) of the four toggles on a line, from the live segment poses."""
    p = objects[LINES[key][0].name].points
    out = []
    for si, dx in zip(TOG_SEG, TOG_DX):
        a, b = p[max(si - 1, 0)], p[min(si + 1, len(p) - 1)]
        d = (b - a) / (np.linalg.norm(b - a) + 1e-12)
        out.append(p[si] + d * dx)
    return np.array(out)


def _dense(poly: np.ndarray, step: float = 0.003) -> np.ndarray:
    if not np.isfinite(poly).all():
        return poly
    out = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        k = min(200, max(1, int(np.linalg.norm(b - a) / step)))
        out.extend(a + (b - a) * (j / k) for j in range(1, k + 1))
    return np.array(out)


def seated_toggle(objects, key) -> int | None:
    """Index of a toggle seated on the robot side of this line's slotted cleat with the line through the slot, or None.
    Seated: the toggle centre is 0-2.5 cm in front of the posts' robot-side face, within 14 mm of the slot's centre
    line sideways and below the post tops; and the line's centre runs through the slot (within its 12 mm width, below
    the post tops) somewhere between the posts."""
    sgn = LINES[key][1]
    fy = sgn * FORK_Y
    face = FORK_X - POST / 2
    poly = _dense(objects[LINES[key][0].name].polyline())
    inslot = (np.abs(poly[:, 0] - FORK_X) <= POST / 2) & (np.abs(poly[:, 1] - fy) <= SLOT / 2) & (poly[:, 2] <= TOP + POST_H)
    if not inslot.any():
        return None
    for i, t in enumerate(toggles(objects, key)):
        if face - 0.025 <= t[0] <= face and abs(t[1] - fy) <= 0.014 and t[2] <= TOP + POST_H:
            return i
    return None


def made_fast(objects, key) -> bool:
    """Made fast taut: the seated toggle is the one that leaves the line taut with the boat on its fenders (a later
    toggle leaves 5 cm of slack; an earlier one cannot reach the posts), and that boat end is within 1.5 cm of them."""
    return seated_toggle(objects, key) == OK_TOGGLE and abs(end_drift(objects)[key]) <= FAST_TOL


def released(objects) -> bool:
    pts = [_dense(objects[s.name].polyline()) for s, _ in LINES.values()] + [toggles(objects, k) for k in LINES]
    pts = np.vstack(pts)
    return all(float(np.linalg.norm(pts - g, axis=1).min()) >= RELEASE for g in grippers())


def _track(objects) -> None:
    """Latched until the next trial: a boat end more than 3 cm off its fender position at any control step until
    1 s after the rise ends; a gripper within 4 cm of the boat at any time; a non-finite state."""
    if not _finite(objects):
        _ST["nan"] = True
        return
    if _ST["t"] <= T_WATCH:
        d = max(end_drift(objects).values())
        _ST["max_drift"] = max(_ST["max_drift"], d)
        if d > DRIFT_MAX:
            _ST["drifted"] = True
    if not _ST["hand"] and any(_boat_dist(objects, g) < HAND_DIST for g in grippers()):
        _ST["hand"] = True


def score(objects) -> float:
    """0 if: a boat end was ever more than 3 cm off the wall (its fender position) before 1 s after the rise ended; a
    gripper came within 4 cm of the boat; the rise has not finished; a gripper is within 3.5 cm of a line or toggle at
    the end. Otherwise 0.5 per line made fast: its second toggle (the one that leaves the line taut) seated in front of
    that line's slotted cleat with the line through the slot, and that end of the boat within 1.5 cm of its fender
    position."""
    if not _finite(objects):
        _ST["nan"] = True
        return 0.0
    _track(objects)
    if _ST["nan"] or _ST["drifted"] or _ST["hand"] or _ST["t"] < T_WATCH:
        return 0.0
    if not released(objects):
        return 0.0
    return 0.5 * sum(made_fast(objects, k) for k in LINES)


def success(objects) -> bool:
    return bool(score(objects) >= 1.0 and objects["boat"].resting(0.01, 0.5)
                and all(objects[s.name].resting(0.02) for s, _ in LINES.values()))


def distance(objects) -> float:
    if not _finite(objects):
        return 1.0
    _track(objects)
    if _ST["drifted"] or _ST["hand"]:
        return 1.0
    return float(sum(max(0.0, v) for v in end_drift(objects).values())
                 + sum(0.0 if seated_toggle(objects, k) is not None else 0.05 for k in LINES))


def progress(objects) -> dict:
    if not _finite(objects):
        return {"kept_at_wall": 0, "rise_done": 0, "bow_fast": 0, "stern_fast": 0, "released": 0}
    _track(objects)
    return {"kept_at_wall": int(not _ST["drifted"] and not _ST["hand"]), "rise_done": int(_ST["t"] >= T_WATCH),
            "bow_fast": int(made_fast(objects, "bow")), "stern_fast": int(made_fast(objects, "stern")),
            "released": int(released(objects))}


# -- closed-loop scripted demo (cabledemo.py) --------------------------------------------------------------------------
GRIP_Z_OVER = 0.010
HI = TOP + 0.06
YAW = {"bow": 0.4, "stern": -0.4}  # fingers nearly across the line; wrist cameras look towards the wall and the slot
ARMI = {"bow": 0, "stern": 1}


def _log(ctx, tag=""):
    st = ctx.states()
    if not _finite(st):
        print(f"[t={_ST['t']:5.2f}] {tag}: NON-FINITE", flush=True)
        return
    _track(st)
    dr = end_drift(st)
    tg = {k: np.round(toggles(st, k)[OK_TOGGLE] - [0, 0, TOP], 3).tolist() for k in LINES}
    sl = {}
    for k in LINES:
        q = _dense(st[LINES[k][0].name].polyline())
        m = np.abs(q[:, 0] - FORK_X) <= POST / 2
        sl[k] = np.round([np.abs(q[m, 1] - LINES[k][1] * FORK_Y).min(), (q[m, 2] - TOP).min()], 3).tolist() if m.any() else None
    print(f"[t={_ST['t']:5.2f}] {tag}: car_z={_ST.get('car_z', 0):.3f} boat_z={st['boat'].pos[2] - BOAT_C0[2]:.3f} drift bow={dr['bow'] * 1000:.1f} "
          f"stern={dr['stern'] * 1000:.1f}mm (max {_ST['max_drift'] * 1000:.1f}) tog={tg} slot={sl} "
          f"seat={[seated_toggle(st, k) for k in LINES]} rel={released(st)} hand={_ST['hand']} score={score(st):.2f}",
          flush=True)


def _line_pt(st, key, s):
    """Live point of a line at arclength s from the cleat (segment centres interpolated)."""
    p = st[LINES[key][0].name].points
    f = s / SEG - 0.5
    i = int(min(max(math.floor(f), 0), len(p) - 2))
    return p[i] + (p[i + 1] - p[i]) * (f - i)


def _take(arm, ctx, key, s):
    from yam_arena.constants import FINGER_CLOSED

    p = _line_pt(ctx.states(), key, s)
    yield from arm.move((p[0], p[1], HI), yaw=YAW[key], finger=-0.016, duration=1.0)
    p = _line_pt(ctx.states(), key, s)
    yield from arm.move((p[0], p[1], p[2] + GRIP_Z_OVER), yaw=YAW[key], duration=0.7)
    yield from arm.grip(FINGER_CLOSED, 0.4)


def _tcp(key):
    g = grippers()
    return g[ARMI[key]] if g else None


GRIP_S = TOG_S_OK + 1.5 * TOG_SPACING  # tending pinch: midway between the 3rd and 4th toggles (on the table at the start)


def _tend_step(arm, key, drift, dt=0.25, gain=1.2, ff=0.0025):
    """One tending step: draw the line in towards the robot by what the drift needs, never pay out."""
    t = _tcp(key)
    pull = min(0.02, max(0.0, gain * (drift + 0.002) + ff))
    g = (t[0] - pull, t[1], t[2])
    yield from arm.move(g, yaw=YAW[key], duration=dt)


def _line_y_at(st, key, x):
    """y of the line's centre where it crosses x (wall side of the toggles), from the dense polyline."""
    poly = _dense(st[LINES[key][0].name].polyline())
    i = int(np.argmin(np.abs(poly[:, 0] - x) + 10.0 * (poly[:, 2] > TOP + 0.08)))
    return float(poly[i, 1])


def _make_fast(arm, ctx, key, toggle=OK_TOGGLE):
    """Lift the held line clear of the posts, carry it over the slot so the chosen toggle hangs just in front of the
    posts and the line just over the slot (closed loop on the live line), lower it into the slot."""
    sgn = LINES[key][1]
    tx = FORK_X - POST / 2 - TOG_L / 2 - 0.006
    zc = TOP + 0.095
    t = _tcp(key)
    yield from arm.move((t[0], t[1], zc), yaw=YAW[key], duration=0.6)
    g = np.array(t[:2], dtype=float)
    for k in range(6):
        st = ctx.states()
        tog = toggles(st, key)[toggle]
        err = np.array([tx - tog[0], sgn * FORK_Y - _line_y_at(st, key, FORK_X)])
        if k > 0 and np.abs(err).max() < 0.0015:
            break
        g = g + err * np.array([1.0, 1.2])
        yield from arm.move((g[0], g[1], zc), yaw=YAW[key], duration=1.0 if k == 0 else 0.5)
    _log(ctx, f"{key} over slot err={np.round(err * 1000, 1).tolist()}mm")
    for z in (TOP + 0.06, TOP + R + GRIP_Z_OVER + 0.002):
        st = ctx.states()
        tog = toggles(st, key)[toggle]
        err = np.array([tx - tog[0], sgn * FORK_Y - _line_y_at(st, key, FORK_X)])
        g = g + np.clip(err, -0.004, 0.004) * np.array([1.0, 1.2])
        yield from arm.move((g[0], g[1], z), yaw=YAW[key], duration=0.8)


def _let_go(arm, key):
    yield from arm.grip(-0.016, 0.4)
    t = _tcp(key)
    yield from arm.move((t[0], t[1], HI + 0.04), yaw=YAW[key], duration=0.7)


def _park(ctx):
    yield from ctx.both(ctx.left.move((0.18, 0.26, TOP + 0.15), yaw=0.0, duration=1.0),
                        ctx.right.move((0.18, -0.26, TOP + 0.15), yaw=0.0, duration=1.0))


def _tend(ctx, keys, until):
    """Both (or the listed) arms draw their lines in, closed loop on the boat's end drift, until time `until`."""
    arms = {"bow": ctx.left, "stern": ctx.right}
    pre = [arms[key].move((_tcp(key)[0] - 0.045, _tcp(key)[1], _tcp(key)[2]), yaw=YAW[key], duration=0.8) for key in keys]
    yield from (ctx.both(*pre) if len(pre) == 2 else pre[0])  # take the sag out of the line before the rise
    k = 0
    while _ST["t"] < until:
        dr = end_drift(ctx.states())
        gens = [_tend_step(arms[key], key, dr[key]) for key in keys]
        if len(gens) == 2:
            yield from ctx.both(*gens)
        elif gens:
            yield from gens[0]
        else:
            yield from ctx.left.wait(0.25)
        k += 1
        if k % 8 == 0:
            _log(ctx, "tending")


def demo_script(ctx):
    """LOCK_DEMO=correct (default). Shortcut variants (all must score 0): none (wait) | hold (take both lines and
    hold them still) | onearm (left arm tends and makes fast the bow line; stern line untouched) | pullend (wait for
    the rise, then take in both lines and make them fast) | fastfirst (make both lines fast before the rise, at the
    toggle that reaches the slot then) | wrongloop (tend correctly, make fast at the next toggle) | holdend (correct,
    but keep pinching after the lines are laid in the slots). Tier check: tier05 (tend both, make only the bow line
    fast, let go of the stern line)."""
    mode = os.environ.get("LOCK_DEMO", "correct")
    L, Rt = ctx.left, ctx.right
    _log(ctx, f"start mode={mode} fork_x={FORK_X} toggles_s={np.round(TOGGLE_S, 4).tolist()} len={LINE_LEN}")
    if mode == "none":
        while _ST["t"] < T_WATCH + 2.0:
            yield from L.wait(1.0)
            _log(ctx, "waiting")
        return
    if mode == "pullend":
        while _ST["t"] < T_RISE0 + T_RISE:
            yield from L.wait(0.5)
        _log(ctx, "rise over")
    if mode == "fastfirst":
        st = ctx.states()
        # the toggle closest to the slot right now (the boat is still low)
        k = int(np.argmin([abs(t[0] - FORK_X) for t in toggles(st, "bow")]))
        _log(ctx, f"fastfirst toggle {k}")
        yield from ctx.both(_take(L, ctx, "bow", TOGGLE_S[k] + 0.025), _take(Rt, ctx, "stern", TOGGLE_S[k] + 0.025))
        yield from ctx.both(_make_fast(L, ctx, "bow", k), _make_fast(Rt, ctx, "stern", k))
        yield from ctx.both(_let_go(L, "bow"), _let_go(Rt, "stern"))
        yield from _park(ctx)
        while _ST["t"] < T_WATCH + 3.0:
            yield from L.wait(1.0)
            _log(ctx, "after")
        return
    if mode == "onearm":
        yield from _take(L, ctx, "bow", GRIP_S)
        _log(ctx, "bow taken")
        yield from _tend(ctx, ["bow"], T_WATCH - 0.5)
    else:
        yield from ctx.both(_take(L, ctx, "bow", GRIP_S), _take(Rt, ctx, "stern", GRIP_S))
        _log(ctx, "both taken")
        if mode == "hold":
            while _ST["t"] < T_WATCH + 1.0:
                yield from L.wait(1.0)
                _log(ctx, "holding")
            yield from ctx.both(_let_go(L, "bow"), _let_go(Rt, "stern"))
            yield from _park(ctx)
            yield from L.wait(2.0)
            _log(ctx, "end")
            return
        yield from _tend(ctx, ["bow", "stern"], T_WATCH - 0.5)
    _log(ctx, "rise over, tended")
    tog = OK_TOGGLE + 1 if mode == "wrongloop" else OK_TOGGLE
    if mode == "onearm":
        yield from _make_fast(L, ctx, "bow", tog)
        yield from _let_go(L, "bow")
    elif mode == "tier05":
        yield from _make_fast(L, ctx, "bow", tog)
        yield from ctx.both(_let_go(L, "bow"), _let_go(Rt, "stern"))
    else:
        yield from ctx.both(_make_fast(L, ctx, "bow", tog), _make_fast(Rt, ctx, "stern", tog))
        _log(ctx, "laid in slots")
        if mode == "holdend":
            yield from L.wait(3.0)
            _log(ctx, "end (still holding)")
            return
        yield from ctx.both(_let_go(L, "bow"), _let_go(Rt, "stern"))
    _log(ctx, "let go")
    yield from _park(ctx)
    for _ in range(6):
        yield from L.wait(0.5)
    _log(ctx, "end")
