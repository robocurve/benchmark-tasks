"""rackkit: the 12U four-post rack, the installed 1U server, the closed sliding rail in its cradle for yam_arena (Isaac
Lab-Arena / PhysX), plus the runtime hooks.

Authored as one custom-spawned assembly (yam_arena task-file primitives are single boxes/spheres):
* the rack (posts with EIA-310 square-hole flanges, outer legs with printed U numbers, base and top frames, far-side
  posts), the installed 1U server (with its near-side rail) and the rail's foam cradle as kinematic rigid bodies (so PhysX contact views can
  report forces on them);
* the rail as two free rigid bodies: the outer member (rear end bracket with two square pegs) and the inner member
  (front end bracket with two square pegs) joined by a prismatic joint (hard limits 0..80 mm of extension; the two
  members do not collide with each other).
Geometry: ``../src/layout.py``; latch rule and scorer: ``../src/scoring.py`` (pure functions).

Mechanism emulation (computed forces, applied every physics step from the measured member poses; drives on maximal
joints are inert in this PhysX build, so none is used):
* slide drag on the telescoping joint: -clip(150 N s/m x v_rel, +-4 N) between the members;
* each end's spring catch (scoring.Latch): a 10 N push-back over the bracket's last 2.5 mm of travel to the flange while
  its pegs are in the holes; latched when the face reaches the flange; afterwards a retention spring keeps the end
  from backing out. Its latch flag (orange tab out = open, green tab flush = latched) shows the state.
Nothing here writes a rail pose or velocity during a trial; ``Runtime.restore_authored`` puts the rail back in its
cradle (and moves the rack, server and cradle to the setup's poses) only on an environment reset. The scripted demo
READS poses through ``bench_state()``.
"""

from __future__ import annotations

import math
import os
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
SRC = HERE.parent / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

import layout as L  # noqa: E402
from scoring import Latch, measure, Scorer  # noqa: E402
from yam_arena.task_file import Box  # noqa: E402

PHYSICS_DT = float(os.environ.get("RACK_PHYSICS_DT", str(1 / 240)))
CONTACT_OFFSET = float(os.environ.get("RACK_CONTACT_OFFSET", ".0001"))  # below the 0.25 mm peg play a side
SETUP_BASE = int(os.environ.get("RACK_SETUP", "0"))  # named setup of the first unseeded trial; later trials cycle
# Trials reset WITH a seed (every inspect-robots rollout passes one) draw the start state from the seed
# (layout.setup_spec); RACK_SETUP set in the environment forces the named setups instead.
FORCE_NAMED = "RACK_SETUP" in os.environ
DEBUG = bool(os.environ.get("RACK_DEBUG"))

STEEL_BLACK, ZINC, ZINC_DARK, FOAM, PANEL_COL, PORT, WHITE = (
    (0.10, 0.10, 0.11),
    (0.74, 0.76, 0.78),
    (0.55, 0.57, 0.60),
    (0.20, 0.22, 0.25),
    (0.20, 0.42, 0.80),  # installed server lid: blue-painted steel (high contrast against the black rack and the bench)
    (0.03, 0.03, 0.03),
    (0.95, 0.95, 0.93),
)
TAB_OPEN, TAB_LATCHED, LABEL_FRONT, LABEL_REAR = (
    (0.95, 0.45, 0.05),
    (0.15, 0.75, 0.25),
    (0.15, 0.35, 0.85),
    (0.85, 0.20, 0.20),
)
KINEMATIC = ("rack", "server", "cradle")
DYNAMIC = ("outer", "inner")
BODIES = DYNAMIC + KINEMATIC


class Rack(Box):
    """The bench, as one task-file object (subclasses Box only to pass yam_arena's checks)."""


def _rot(q_xyzw):
    x, y, z, w = q_xyzw
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def _quat(r):
    from scipy.spatial.transform import Rotation

    q = Rotation.from_matrix(np.asarray(r, float)).as_quat()  # xyzw
    return q / np.linalg.norm(q)


def authored(setup):
    """Authored (reset) pose of every body for a setup: {name: (pos, rot)} in the env frame."""
    spec = L.setup_spec(setup)
    po, ro = L.rail_rest_pose(spec)
    pi, ri = L.inner_from_outer(po, ro, 0.0)
    rack = L.rack_origin(spec)
    server = np.array([rack[0], rack[1], L.u_bottom(spec["server_u"])])
    return {
        "outer": (po, ro),
        "inner": (pi, ri),
        "rack": (rack, np.eye(3)),
        "server": (server, np.eye(3)),
        "cradle": (po, ro),
    }


_SPEC: dict = {}


def _spawn_bench(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics
    from scipy.spatial import ConvexHull
    from yam_arena.station import linear_rgb

    stage = get_current_stage()
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    ox, oy, oz, ow = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(ow, ox, oy, oz))
    looks, mats = {}, {}

    def look(color):
        if color not in looks:
            path = f"{prim_path}/Looks/c{len(looks)}"
            vis = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=0.45)
            vis.func(path, vis)
            looks[color] = path
        return looks[color]

    def material(friction):
        if friction not in mats:
            path = f"{prim_path}/Looks/phys{len(mats)}"
            pm = sim_utils.RigidBodyMaterialCfg(static_friction=friction, dynamic_friction=friction)
            pm.func(path, pm)
            mats[friction] = path
        return mats[friction]

    def xform(path, pos, rot):
        x = UsdGeom.Xform.Define(stage, path)
        x.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*map(float, pos)))
        q = _quat(rot)
        x.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(float(q[3]), *map(float, q[:3])))
        return x

    def rigid(path, mass=None, kinematic=False, com=None):
        prim = stage.GetPrimAtPath(path)
        rb = UsdPhysics.RigidBodyAPI.Apply(prim)
        if kinematic:
            rb.CreateKinematicEnabledAttr(True)
        m = UsdPhysics.MassAPI.Apply(prim)
        m.CreateMassAttr(float(mass if mass is not None else 1.0))
        if com is not None:
            m.CreateCenterOfMassAttr(Gf.Vec3f(*map(float, com)))
        px = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        px.CreateMaxDepenetrationVelocityAttr(0.2)
        px.CreateSolverPositionIterationCountAttr(32)
        px.CreateSolverVelocityIterationCountAttr(4)
        px.CreateLinearDampingAttr(0.0)
        px.CreateAngularDampingAttr(0.05)
        px.CreateSleepThresholdAttr(0.0)  # never sleeps: a sleeping body reports no contacts
        px.CreateStabilizationThresholdAttr(0.0)
        PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)

    count = {"n": 0}

    def hull(path, v, color, friction=L.HARD_FRICTION, collide=True):
        v = np.asarray(v, float)
        h = ConvexHull(v)
        m = UsdGeom.Mesh.Define(stage, path)
        m.CreatePointsAttr([Gf.Vec3f(*map(float, p)) for p in v])
        m.CreateFaceVertexCountsAttr([3] * len(h.simplices))
        faces = []
        c = v.mean(axis=0)
        for s in h.simplices:
            a, b, d = v[s[0]], v[s[1]], v[s[2]]
            n = np.cross(b - a, d - a)
            faces.append(s if n @ (a - c) > 0 else s[::-1])
        m.CreateFaceVertexIndicesAttr([int(j) for f in faces for j in f])
        m.CreateSubdivisionSchemeAttr("none")
        m.CreateExtentAttr([Gf.Vec3f(*map(float, v.min(axis=0))), Gf.Vec3f(*map(float, v.max(axis=0)))])
        prim = m.GetPrim()
        if collide:
            UsdPhysics.CollisionAPI.Apply(prim)
            cp = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            cp.CreateContactOffsetAttr(CONTACT_OFFSET)
            cp.CreateRestOffsetAttr(0.0)
            UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("convexHull")
            hc = PhysxSchema.PhysxConvexHullCollisionAPI.Apply(prim)
            hc.CreateHullVertexLimitAttr(64)
            hc.CreateMinThicknessAttr(0.0001)
            bind_physics_material(str(prim.GetPath()), material(friction), stage=stage)
            count["n"] += 1
        bind_visual_material(str(prim.GetPath()), look(color), stage=stage)
        return m

    def box(path, x0, x1, y0, y1, z0, z1, color, friction=L.HARD_FRICTION, collide=True):
        return hull(path, L.box_corners(x0, x1, y0, y1, z0, z1), color, friction, collide)

    def stroke(path, p0, p1, color, width=0.0022, depth=0.0004):
        """Visual-only bar between two points on a plane facing -x (for printed marks on the posts' legs)."""
        p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
        y0, y1 = sorted((p0[1], p1[1]))
        z0, z1 = sorted((p0[2], p1[2]))
        x = p0[0]
        return box(path, x - depth, x, y0 - width / 2, y1 + width / 2, z0 - width / 2, z1 + width / 2, color,
                   collide=False)

    A = authored(SETUP_BASE)
    # ---------------------------------------------------------------- rack (kinematic), frame at the rack origin
    xform(f"{prim_path}/rack", *A["rack"])
    rigid(f"{prim_path}/rack", 20.0, kinematic=True)
    rk = f"{prim_path}/rack"
    zt = L.u_bottom(L.N_U + 1) + 0.010
    for sign, tag in ((1, "rear"), (-1, "front")):
        for i, v in enumerate(L.flange_pieces(sign)):
            hull(f"{rk}/{tag}_flange_{i:02d}", v, STEEL_BLACK)
        # outer leg (y-z plane) on the robot side of the flange, running outward
        y0, y1 = sorted((sign * L.Y_F, sign * (L.Y_F + L.LEG_W)))
        box(f"{rk}/{tag}_leg", -L.FLANGE_HALF_W - 0.002, -L.FLANGE_HALF_W, y0, y1, L.TOP, zt, STEEL_BLACK)
        # back leg (closes the post section)
        box(f"{rk}/{tag}_back", -L.FLANGE_HALF_W - 0.002, L.FLANGE_HALF_W, sign * (L.Y_F + L.LEG_W) - 0.001,
            sign * (L.Y_F + L.LEG_W) + 0.001, L.TOP, zt, STEEL_BLACK)
        # printed U numbers and U boundary ticks on the leg's robot-facing side
        xf = -L.FLANGE_HALF_W - 0.002
        yc = sign * (L.Y_F + L.LEG_W / 2)
        for u in range(1, L.N_U + 2):
            zb = L.u_bottom(u)
            stroke(f"{rk}/{tag}_tick_{u:02d}", (xf, sign * L.Y_F + sign * 0.002, zb),
                   (xf, sign * L.Y_F + sign * 0.012, zb), WHITE)
            if u <= L.N_U:
                for k, ((a0, b0), (a1, b1)) in enumerate(L.digit_strokes(str(u), 0.022)):
                    stroke(f"{rk}/{tag}_u{u:02d}_{k:02d}", (xf, yc + a0 * sign * -1, L.u_center(u) + b0),
                           (xf, yc + a1 * sign * -1, L.u_center(u) + b1), WHITE)
        # far-side post (plain), base rail and top rail along x
        xs = L.HOLE_SPAN_X
        box(f"{rk}/{tag}_far_post", xs - L.FLANGE_HALF_W, xs + L.FLANGE_HALF_W + 0.002, y0, y1, L.TOP, zt,
            STEEL_BLACK)
        box(f"{rk}/{tag}_base_x", -L.FLANGE_HALF_W, xs + L.FLANGE_HALF_W, y0, y1, L.TOP, L.U1_BOTTOM - 0.001,
            STEEL_BLACK)
        box(f"{rk}/{tag}_top_x", -L.FLANGE_HALF_W, xs + L.FLANGE_HALF_W, y0, y1, zt - 0.025, zt, STEEL_BLACK)
    for side, x0 in (("near", -L.FLANGE_HALF_W - 0.025), ("far", L.HOLE_SPAN_X - 0.005)):
        ya, yb = -(L.Y_F + L.LEG_W), L.Y_F + L.LEG_W
        box(f"{rk}/{side}_base_y", x0, x0 + 0.025, ya, yb, L.TOP, L.U1_BOTTOM - 0.001, STEEL_BLACK)
        box(f"{rk}/{side}_top_y", x0, x0 + 0.025, ya, yb, zt - 0.025, zt, STEEL_BLACK)
    # FRONT / REAR labels at the posts' feet
    box(f"{rk}/label_front", -0.030, -0.019, -(L.Y_F + 0.040), -(L.Y_F + 0.005), L.TOP + 0.001, L.TOP + 0.020,
        LABEL_FRONT, collide=False)
    box(f"{rk}/label_rear", -0.030, -0.019, L.Y_F + 0.005, L.Y_F + 0.040, L.TOP + 0.001, L.TOP + 0.020, LABEL_REAR,
        collide=False)
    # ---------------------------------------------------------------- installed 1U server (kinematic) on its own rails,
    # frame: hole column x, rack depth centre y, bottom of its U
    xform(f"{prim_path}/server", *A["server"])
    rigid(f"{prim_path}/server", 12.0, kinematic=True)
    sv = f"{prim_path}/server"
    z0, z1 = 0.0004, 0.0004 + L.SERVER_H
    yf = -L.Y_F - L.FLANGE_T  # front face of the front flange
    box(f"{sv}/lid", L.SERVER_X[0], L.SERVER_X[1], -L.Y_F, -L.Y_F + L.SERVER_DEPTH, z0, z1, PANEL_COL)
    box(f"{sv}/bezel", L.SERVER_X[0], L.SERVER_X[1], yf - 0.030, yf, z0, z1, STEEL_BLACK)
    box(f"{sv}/ears", -L.FLANGE_HALF_W, L.HOLE_SPAN_X + L.FLANGE_HALF_W, yf - L.SERVER_EAR_T, yf, z0, z1, STEEL_BLACK)
    zc = L.U / 2
    # its near-side rail, latched at both ends (same rail as the new one, extended to the post spacing)
    box(f"{sv}/rail_web", L.OUTER_X[0], L.OUTER_X[1], -L.Y_F + L.BRACKET_T, L.Y_F - L.BRACKET_T, zc - L.OUTER_Z,
        zc + L.OUTER_Z, ZINC, friction=L.RAIL_FRICTION)
    for tag, y0, y1 in (("rear", L.Y_F - L.BRACKET_T, L.Y_F), ("front", -L.Y_F, -L.Y_F + L.BRACKET_T)):
        box(f"{sv}/rail_{tag}_bracket", L.BRACKET_X[0], L.BRACKET_X[1], y0, y1, zc - L.BRACKET_Z, zc + L.BRACKET_Z,
            ZINC_DARK, friction=L.RAIL_FRICTION)
        ty = (y0 - 0.009, y0) if tag == "rear" else (y1, y1 + 0.009)
        box(f"{sv}/rail_{tag}_tab", L.BRACKET_X[0], L.BRACKET_X[0] + 0.006, ty[0], ty[1], zc + 0.006, zc + 0.019,
            TAB_LATCHED, collide=False)
    # ---------------------------------------------------------------- cradle (kinematic), frame = rail O at rest
    xform(f"{prim_path}/cradle", *A["cradle"])
    rigid(f"{prim_path}/cradle", 0.3, kinematic=True)
    cr = f"{prim_path}/cradle"
    zs = -L.OUTER_Z  # slot floor (outer member bottom)
    zb = L.TOP - L.CRADLE_SLOT_Z - L.OUTER_Z + 0.0005  # cradle bottom (just above the bench), O-frame z
    xm = (L.OUTER_X[0] + L.OUTER_X[1]) / 2
    bx, by, bz = L.CRADLE_BLOCK
    w = L.CRADLE_SLOT_W / 2
    for i, yc in enumerate(L.CRADLE_AT):
        box(f"{cr}/base_{i}", xm - bx / 2, xm + bx / 2, yc - by / 2, yc + by / 2, zb, zs, FOAM)
        box(f"{cr}/wall_a_{i}", xm - bx / 2, xm - w, yc - by / 2, yc + by / 2, zs, zb + bz, FOAM)
        box(f"{cr}/wall_b_{i}", xm + w, xm + bx / 2, yc - by / 2, yc + by / 2, zs, zb + bz, FOAM)
    # ---------------------------------------------------------------- rail: outer member (rear end)
    xform(f"{prim_path}/outer", *A["outer"])
    rigid(f"{prim_path}/outer", L.OUTER_MASS, com=(0.017, -L.OUTER_LEN / 2, 0.0))
    ou = f"{prim_path}/outer"
    box(f"{ou}/web", L.OUTER_X[0], L.OUTER_X[1], -L.OUTER_LEN, -L.BRACKET_T + 0.0005, -L.OUTER_Z, L.OUTER_Z, ZINC,
        friction=L.RAIL_FRICTION)
    box(f"{ou}/bracket", L.BRACKET_X[0], L.BRACKET_X[1], -L.BRACKET_T, 0.0, -L.BRACKET_Z, L.BRACKET_Z, ZINC_DARK,
        friction=L.RAIL_FRICTION)
    for k, dz in enumerate((-L.PEG_DZ, L.PEG_DZ)):
        hull(f"{ou}/peg_{k}", L.peg_points(1.0, dz), ZINC_DARK, friction=L.RAIL_FRICTION)
    box(f"{ou}/tab_open", L.BRACKET_X[0] - 0.012, L.BRACKET_X[0], -0.012, -L.BRACKET_T, 0.006, 0.019, TAB_OPEN,
        collide=False)
    box(f"{ou}/tab_latched", L.BRACKET_X[0], L.BRACKET_X[0] + 0.006, -0.012, -L.BRACKET_T, 0.006, 0.019,
        TAB_LATCHED, collide=False)
    box(f"{ou}/label", L.OUTER_X[0] - 0.0004, L.OUTER_X[0], -0.120, -0.070, -0.010, 0.010, LABEL_REAR,
        collide=False)
    # ---------------------------------------------------------------- rail: inner member (front end)
    xform(f"{prim_path}/inner", *A["inner"])
    rigid(f"{prim_path}/inner", L.INNER_MASS, com=(0.017, L.INNER_LEN / 2, 0.0))
    iu = f"{prim_path}/inner"
    box(f"{iu}/web", L.INNER_X[0], L.INNER_X[1], L.BRACKET_T - 0.0005, L.INNER_LEN, -L.INNER_Z, L.INNER_Z, ZINC,
        friction=L.RAIL_FRICTION)
    box(f"{iu}/bracket", L.BRACKET_X[0], L.BRACKET_X[1], 0.0, L.BRACKET_T, -L.BRACKET_Z, L.BRACKET_Z, ZINC_DARK,
        friction=L.RAIL_FRICTION)
    for k, dz in enumerate((-L.PEG_DZ, L.PEG_DZ)):
        hull(f"{iu}/peg_{k}", L.peg_points(-1.0, dz), ZINC_DARK, friction=L.RAIL_FRICTION)
    box(f"{iu}/tab_open", L.BRACKET_X[0] - 0.012, L.BRACKET_X[0], L.BRACKET_T, 0.012, 0.006, 0.019, TAB_OPEN,
        collide=False)
    box(f"{iu}/tab_latched", L.BRACKET_X[0], L.BRACKET_X[0] + 0.006, L.BRACKET_T, 0.012, 0.006, 0.019,
        TAB_LATCHED, collide=False)
    box(f"{iu}/label", L.INNER_X[0] - 0.0004, L.INNER_X[0], 0.040, 0.090, -0.008, 0.008, LABEL_FRONT,
        collide=False)
    for t in ("outer", "inner"):
        UsdGeom.Imageable(stage.GetPrimAtPath(f"{prim_path}/{t}/tab_latched")).MakeInvisible()
    # telescoping joint: inner slides along the rail axis (O-frame y), position = -extension
    pj = UsdPhysics.PrismaticJoint.Define(stage, f"{iu}/slide")
    pj.CreateAxisAttr("Y")
    pj.CreateBody0Rel().SetTargets([Sdf.Path(ou)])
    pj.CreateBody1Rel().SetTargets([Sdf.Path(iu)])
    pj.CreateLocalPos0Attr(Gf.Vec3f(0.0, -float(L.L0), 0.0))
    pj.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    pj.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
    pj.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    pj.CreateLowerLimitAttr(-float(L.E_MAX))
    pj.CreateUpperLimitAttr(0.0)
    UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(ou)).CreateFilteredPairsRel().AddTarget(iu)
    _SPEC["paths"] = {n: f"{prim_path}/{n}" for n in BODIES}
    _SPEC["colliders"] = count["n"]
    return root.GetPrim()


def _arena_object(spec):
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    @configclass
    class BenchSpawnerCfg(SpawnerCfg):
        func: object = clone(_spawn_bench)

    return Object(
        name=spec.name,
        object_type=ObjectType.BASE,
        spawner_cfg=BenchSpawnerCfg(),
        initial_pose=Pose(position_xyz=(0.0, 0.0, 0.0), rotation_xyzw=(0.0, 0.0, 0.0, 1.0)),
    )


# ---------------------------------------------------------------------------------------------- runtime state
STATE: dict = dict(score=0.0, level=0.0, steps=0, detail={}, nonfinite=0, trial=-1, setup=None)


def finger_tips(environment):
    """Pad-tip centres of all four fingers (env frame)."""
    from yam_arena.constants import PAD_TIP_PAST_TCP, TCP_OFFSET

    base = environment.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(float)
    pts = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        pose = robot.data.body_link_pose_w[0].detach().cpu().numpy().astype(float)[
            list(robot.body_names).index("gripper")
        ]
        jp = robot.data.joint_pos[0].detach().cpu().numpy().astype(float)
        jn = list(robot.joint_names)
        half = max(abs(jp[jn.index(j)]) for j in ("joint7", "joint8"))
        r = _rot(pose[3:7])
        tip = pose[:3] + r @ (np.array(TCP_OFFSET) + [0, 0, PAD_TIP_PAST_TCP]) - origin
        pts += [tip + r[:, 0] * s * (half + 0.0032) for s in (1.0, -1.0)]
    return pts


class Runtime:
    """Computed slide drag and latch forces every physics step; scoring every control step."""

    def __init__(self, environment):
        import carb
        import omni.physics.tensors as tensors
        import torch
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import UsdUtils

        carb.settings.get_settings().set_bool("/physics/disableContactProcessing", False)
        self.base = environment.unwrapped
        self.env = environment
        self.torch = torch
        self.stage = get_current_stage()
        stage_id = UsdUtils.StageCache.Get().GetId(self.stage).ToLongInt()
        self.simview = tensors.create_simulation_view("torch", stage_id=stage_id)
        self.simview.set_subspace_roots("/")
        root = "/World/envs/env_0/rack"
        self.root = root
        P = lambda n: f"{root}/{n}"  # noqa: E731
        self.view = self.simview.create_rigid_body_view([P(n) for n in BODIES])
        self.dyn = self.simview.create_rigid_body_view([P(n) for n in DYNAMIC])
        robot_links = []
        for arm in ("left_robot", "right_robot"):
            lp = self.base.scene[arm].root_view.link_paths
            robot_links += list(lp[0] if lp and isinstance(lp[0], (list, tuple)) else lp)
        self.robot_links = robot_links
        self.cv_grip = self.simview.create_rigid_contact_view(
            [P("outer"), P("inner")], filter_patterns=[robot_links, robot_links], max_contact_data_count=4096
        )
        self.cv_rack = self.simview.create_rigid_contact_view(
            [P("outer"), P("inner")], filter_patterns=[[P("rack"), P("server")], [P("rack"), P("server")]],
            max_contact_data_count=4096,
        )
        self.cv_robot = self.simview.create_rigid_contact_view(
            [P("rack"), P("server")], filter_patterns=[robot_links, robot_links], max_contact_data_count=4096
        )
        self.origin = self.base.scene.env_origins[0].detach().cpu().numpy().astype(float)
        self.forces = torch.zeros((2, 3), dtype=torch.float32)
        self.force_idx = torch.tensor([0, 1], dtype=torch.int32)
        self.extra = {}  # development probes only: {'outer'|'inner': force vector (N, world)}
        self.setup = SETUP_BASE
        self.spec = L.setup_spec(self.setup)
        self.trial_active = False
        self._init_state()

    def _init_state(self):
        self.latch = Latch()
        self.scorer = Scorer()
        self.steps = 0
        self.t = 0.0
        self.hook_s = 0.0
        self.nonfinite = 0
        self.grip_peak = {"outer": 0.0, "inner": 0.0}
        self.rack_peak = {"outer": 0.0, "inner": 0.0}
        self.robot_rack_peak = 0.0
        self.robot_rack_link = None
        self.latch_force = {"rear": 0.0, "front": 0.0}
        self.slide_force = 0.0
        self.last_contacts = None
        self._tabs_shown = {"outer": False, "inner": False}
        self._set_tab("outer", False)
        self._set_tab("inner", False)
        STATE.update(score=0.0, level=0.0, steps=0, detail={}, nonfinite=0, setup=self.setup)

    def _set_tab(self, member, latched):
        from pxr import UsdGeom

        o = UsdGeom.Imageable(self.stage.GetPrimAtPath(f"{self.root}/{member}/tab_open"))
        c = UsdGeom.Imageable(self.stage.GetPrimAtPath(f"{self.root}/{member}/tab_latched"))
        if latched:
            o.MakeInvisible()
            c.MakeVisible()
        else:
            c.MakeInvisible()
            o.MakeVisible()
        self._tabs_shown[member] = latched

    def begin_trial(self, trial):
        self._init_state()
        self.trial_active = True
        STATE.update(trial=trial, setup=self.setup, start=self.spec)

    # ------------------------------------------------------------------ reading
    def read(self):
        tf = self.view.get_transforms().detach().cpu().numpy().astype(float)
        return {n: (tf[i, :3] - self.origin, _rot(tf[i, 3:7])) for i, n in enumerate(BODIES)}

    def velocities(self):
        v = self.dyn.get_velocities().detach().cpu().numpy().astype(float)
        return {"outer": (v[0, :3], v[0, 3:6]), "inner": (v[1, :3], v[1, 3:6])}

    # ------------------------------------------------------------------ physics hook
    def physics(self, dt):
        t0 = time.perf_counter()
        self.t += dt
        tf = self.dyn.get_transforms().detach().cpu().numpy().astype(float)
        v = self.dyn.get_velocities().detach().cpu().numpy().astype(float)
        po, ro = tf[0, :3] - self.origin, _rot(tf[0, 3:7])
        pi, ri = tf[1, :3] - self.origin, _rot(tf[1, 3:7])
        f = np.zeros((2, 3))
        # telescoping slide drag
        ax = ro[:, 1]
        v_rel = float((v[1, :3] - v[0, :3]) @ ax)
        fs = -float(np.clip(L.SLIDE_VISC * v_rel, -L.SLIDE_FRICTION_N, L.SLIDE_FRICTION_N))
        f[1] += fs * ax
        f[0] -= fs * ax
        self.slide_force = fs
        if self.trial_active:
            for k, (end, p, r) in enumerate((("rear", po, ro), ("front", pi, ri))):
                st = L.end_state(end, p, r, self.spec)
                n = L.outward(end)
                was = self.latch.latched[end]
                fl = self.latch.step(end, st, float(v[k, :3] @ n), self.t)
                f[k] += fl * n
                self.latch_force[end] = fl
                if self.latch.latched[end] and not was:
                    self._set_tab("outer" if end == "rear" else "inner", True)
            gm = self.cv_grip.get_contact_force_matrix(dt=dt).detach().cpu().numpy()
            for k, mbr in enumerate(DYNAMIC):
                g = float(np.linalg.norm(gm[k], axis=-1).max()) if gm.size else 0.0
                self.grip_peak[mbr] = max(self.grip_peak[mbr], g)
            rm = self.cv_rack.get_contact_force_matrix(dt=dt).detach().cpu().numpy()
            for k, mbr in enumerate(DYNAMIC):
                g = float(np.linalg.norm(rm[k], axis=-1).max()) if rm.size else 0.0
                self.rack_peak[mbr] = max(self.rack_peak[mbr], g)
            rb = self.cv_robot.get_contact_force_matrix(dt=dt).detach().cpu().numpy()
            if rb.size:
                nr = np.linalg.norm(rb, axis=-1).max(axis=0)
                j = int(np.argmax(nr))
                if nr[j] > self.robot_rack_peak:
                    self.robot_rack_peak = float(nr[j])
                    self.robot_rack_link = self.robot_links[j].split("/")[-2:] if nr[j] > 0.5 else None
                robot_rack_now = round(float(nr.max()), 3)
            else:
                robot_rack_now = 0.0
            self.last_contacts = dict(
                robot_rack_n=robot_rack_now,
                grip_n=[round(float(np.linalg.norm(gm[k], axis=-1).max()), 3) if gm.size else 0.0 for k in (0, 1)],
                rack_n=[round(float(np.linalg.norm(rm[k], axis=-1).max()), 3) if rm.size else 0.0 for k in (0, 1)],
            )
        for name, fv in self.extra.items():
            f[DYNAMIC.index(name)] += np.asarray(fv, float)
        self.forces.copy_(self.torch.as_tensor(f, dtype=self.torch.float32))
        self.dyn.apply_forces(self.forces, self.force_idx, True)
        self.hook_s += time.perf_counter() - t0

    # ------------------------------------------------------------------ reset
    def restore_authored(self, setup, override=None):
        """Environment reset only (never during a trial): rack, server and cradle to the setup's poses, the closed rail
        at rest in its cradle (``override``: development probes only, {body: (pos, rot)})."""
        torch = self.torch
        self.setup = setup
        self.spec = L.setup_spec(setup)
        self.trial_active = False
        a = authored(setup)
        if override:
            a.update(override)
        tf = self.view.get_transforms().clone()
        for i, n in enumerate(BODIES):
            pos, rot = a[n]
            tf[i, :3] = torch.tensor(np.asarray(pos, float) + self.origin, dtype=tf.dtype)
            tf[i, 3:7] = torch.tensor(_quat(rot), dtype=tf.dtype)
        idx = torch.tensor(list(range(len(BODIES))), dtype=torch.int32, device=tf.device)
        self.view.set_transforms(tf, idx)
        dyn = torch.tensor([BODIES.index(n) for n in DYNAMIC], dtype=torch.int32, device=tf.device)
        self.view.set_velocities(torch.zeros_like(self.view.get_velocities()), dyn)  # kinematic bodies: no velocity
        self._init_state()

    # ------------------------------------------------------------------ scoring
    def control(self, environment):
        if not self.trial_active:
            return
        self.steps += 1
        poses = self.read()
        po, ro = poses["outer"]
        pi, ri = poses["inner"]
        vel = self.velocities()
        if not all(np.all(np.isfinite(x)) for x in (po, ro, pi, ri)):
            self.nonfinite += 1
            STATE["nonfinite"] = self.nonfinite
            return
        tips = finger_tips(environment)
        m = measure(self.spec, po, ro, pi, ri, self.latch, tips)
        lin = max(float(np.linalg.norm(vel["outer"][0])), float(np.linalg.norm(vel["inner"][0])))
        level = self.scorer.step(m, lin)
        STATE.update(
            score=level,
            level=level,
            steps=self.steps,
            nonfinite=self.nonfinite,
            detail=dict(
                self.scorer.last,
                latch_events=list(self.latch.events),
                latch_force_n={k: round(v, 2) for k, v in self.latch_force.items()},
                slide_force_n=round(self.slide_force, 3),
                grip_peak_n={k: round(v, 2) for k, v in self.grip_peak.items()},
                rack_peak_n={k: round(v, 2) for k, v in self.rack_peak.items()},
                robot_rack_peak_n=round(self.robot_rack_peak, 2),
                robot_rack_link=self.robot_rack_link,
                outer_mm=np.round(po * 1000, 3).tolist(),
                inner_mm=np.round(pi * 1000, 3).tolist(),
                setup=self.setup,
                contacts=self.last_contacts,
            ),
        )


_RT: dict = {"rt": None, "sim": None, "trials": 0, "override": None}


def runtime():
    return _RT["rt"]


def bench_state():
    """Current poses of every body for the scripted demo (read-only), plus the start spec."""
    rt = _RT["rt"]
    if rt is None:
        return None
    out = rt.read()
    out["spec"] = rt.spec
    return out


def _patch_scene(scene) -> None:
    if getattr(scene, "_rack_patched", False):
        return
    orig_task_object, orig_states = scene.task_object, scene.object_states

    def task_object(spec):
        if isinstance(spec, Rack):
            return _arena_object(spec)
        return orig_task_object(spec)

    def object_states(environment, names):
        from yam_arena.task_file import ObjectState

        plain = [n for n in names if n != "rack"]
        out = orig_states(environment, plain) if plain else {}
        if "rack" in names:
            rt = _RT["rt"]
            p, r = rt.read()["outer"] if rt else authored(SETUP_BASE)["outer"]
            z = np.zeros(3)
            out["rack"] = ObjectState(
                pos=np.asarray(p),
                quat_xyzw=_quat(r),
                lin_vel=z,
                ang_vel=z,
                low=p - 0.2,
                high=p + 0.2,
            )
        return {n: out[n] for n in names}

    orig_build = scene.build_task_environment

    def build(embodiment, task, control_hz=None):
        environment = orig_build(embodiment, task, control_hz)
        if any(isinstance(o, Rack) for o in task.objects):
            previous = environment.env_cfg_callback

            def configure(cfg):
                from yam_arena.constants import CONTROL_HZ

                cfg = previous(cfg)
                cfg.sim.dt = PHYSICS_DT
                cfg.decimation = round(1.0 / (PHYSICS_DT * (control_hz or CONTROL_HZ)))
                cfg.sim.render_interval = cfg.decimation
                return cfg

            environment.env_cfg_callback = configure
        return environment

    scene.build_task_environment = build
    scene.task_object = task_object
    scene.object_states = object_states
    scene._rack_patched = True


def _has_rack(emb):
    return any(
        isinstance(o, Rack)
        for o in (emb.task.objects if emb.task is not None else ())
    )


def _patch_embodiment(ie) -> None:
    cls = ie.YamArenaEmbodiment
    if getattr(cls, "_rack_patched", False):
        return
    base_task_step, base_reset, base_ensure = cls._task_step, cls.reset, cls._ensure_sim

    def _ensure_sim(self, *a, **k):
        env_before = self._env
        out = base_ensure(self, *a, **k)
        if self._env is not None and self._env is not env_before and _has_rack(self):
            _wrap(self._env)
            _RT["rt"] = Runtime(self._env)
        return out

    def _task_step(self, *a, **k):
        if _RT["rt"] is not None:
            _RT["rt"].control(self._env)
        return base_task_step(self, *a, **k)

    def reset(self, *a, **k):
        rt = _RT["rt"]
        seed = k.get("seed")
        if seed is not None and not FORCE_NAMED:
            setup = ("seed", int(seed))
        else:
            setup = (SETUP_BASE + _RT["trials"]) % len(L.SETUPS)
        if rt is not None:
            rt.restore_authored(setup, _RT["override"])
        obs = base_reset(self, *a, **k)
        rt = _RT["rt"]
        if rt is not None:
            if rt.setup != setup:  # first reset of a fresh env
                rt.restore_authored(setup, _RT["override"])
                obs = base_reset(self, *a, **k)
            # the reset's settling steps can leave the telescoping member drawn out after a trial in which the rail was
            # extended or latched: put the rail back closed in its cradle once more, after the settle
            rt.restore_authored(setup, _RT["override"])
            rt.begin_trial(_RT["trials"])
            _RT["trials"] += 1
        return obs

    cls._ensure_sim, cls._task_step, cls.reset = _ensure_sim, _task_step, reset
    cls._rack_patched = True


def _wrap(environment) -> None:
    base = environment.unwrapped
    sim = base.sim
    if _RT["sim"] is sim:
        return
    orig = sim.step

    def step(*a, **k):
        out = orig(*a, **k)
        if _RT["rt"] is not None:
            _RT["rt"].physics(base.physics_dt)
        return out

    sim.step = step
    _RT["sim"] = sim


def install() -> None:
    import importlib.abc
    import importlib.util

    class _Finder(importlib.abc.MetaPathFinder):
        targets = {
            "yam_arena.scene": _patch_scene,
            "yam_arena.inspect_embodiment": _patch_embodiment,
        }

        def find_spec(self, fullname, path, target=None):
            if fullname not in self.targets:
                return None
            sys.meta_path.remove(self)
            try:
                spec = importlib.util.find_spec(fullname)
            finally:
                sys.meta_path.insert(0, self)
            if spec is None or spec.loader is None:
                return None
            loader, patch = spec.loader, self.targets[fullname]
            orig_exec = loader.exec_module

            def exec_module(module):
                orig_exec(module)
                patch(module)

            loader.exec_module = exec_module
            return spec

    for name, patch in _Finder.targets.items():
        if name in sys.modules:
            patch(sys.modules[name])
    if not any(
        type(f).__name__ == "_Finder"
        and getattr(f, "targets", None)
        and "yam_arena.scene" in f.targets
        for f in sys.meta_path
    ):
        sys.meta_path.insert(0, _Finder())


install()
