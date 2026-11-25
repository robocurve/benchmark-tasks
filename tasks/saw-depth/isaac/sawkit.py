"""sawkit: the circular saw, the boards and their support blocks for yam_arena (Isaac Lab-Arena / PhysX), plus the
runtime hooks.

Authored as one custom-spawned assembly (yam_arena task-file primitives are single boxes/spheres):
* six boards (one per stock thickness) as kinematic bodies: the trial's board sits on the two support blocks, the
  other five are parked out of sight under the floor (moved only on an environment reset);
* the two support blocks (2x3 offcuts on edge), kinematic;
* the saw as one floating articulation of three links: the shoe (root link, with the depth bracket), the upper
  assembly (motor, gearcase, upper guard, 24-tooth blade, rear D-handle, front knob) on a revolute joint at the depth
  pivot (hard limits 0 .. 2-9/16 in of depth), and the depth lever on a revolute joint at its clamp bolt (hard limits
  0 = locked .. 60 deg = released).
Geometry: ``../src/layout.py``; scorer: ``../src/scoring.py`` (pure functions).

Mechanisms (articulation joint drives act in this PhysX build; the values are set through the tensor API):
* depth clamp: while the lever is within 6 deg of locked, the pivot's drive is a stiff spring anchored where the clamp
  closed, capped at the clamp's holding torque (25 N m); past the cap it slips and re-anchors (stick-slip friction).
  Released: a velocity damper capped at 1.0 N m (the loose bracket). Nothing else holds the upper assembly up.
* lever cam: a computed torque of 0.45 N m pushes the lever toward the nearer end of its travel (over-centre at
  25 deg), so it toggles only when pushed with about 10 N at the tab, and stays where it snapped.
Nothing here writes a pose, a joint state or a score during a trial; ``Runtime.restore_authored`` runs only on an
environment reset. The scripted demo READS poses through ``saw_state()``.
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
from scoring import measure, Scorer  # noqa: E402
from yam_arena.task_file import Box  # noqa: E402

PHYSICS_DT = float(os.environ.get("SAW_PHYSICS_DT", str(1 / 480)))
CONTACT_OFFSET = 0.002
SETUP_BASE = int(os.environ.get("SAW_SETUP", "0"))
FORCE_NAMED = "SAW_SETUP" in os.environ
PROBE = os.environ.get("SAW_PROBE", "")  # development probes only (dev/); empty in every trial
DEBUG = bool(os.environ.get("SAW_DEBUG"))
DEG = math.pi / 180
PARK = np.array([0.0, 0.0, -2.0])  # unused boards (under the floor, out of every camera)

COLORS = dict(
    yellow=(0.95, 0.72, 0.05),
    black=(0.08, 0.08, 0.09),
    rubber=(0.12, 0.12, 0.13),
    alu=(0.70, 0.72, 0.74),
    steel=(0.55, 0.57, 0.60),
    red=(0.80, 0.12, 0.10),
    wood=(0.82, 0.66, 0.44),
    wood2=(0.70, 0.55, 0.36),
    white=(0.96, 0.96, 0.94),
    ply=(0.86, 0.74, 0.55),
    hardboard=(0.36, 0.24, 0.15),
    lvl=(0.88, 0.70, 0.42),
)
BOARD_COLORS = ("hardboard", "ply", "ply", "wood", "wood", "lvl")
LINKS = ("shoe", "upper", "lever")


class SawBench(Box):
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
    """Authored (reset) state for a setup: link poses, joint angles and board placements."""
    spec = L.setup_spec(setup)
    sp, sr = L.shoe_pose(spec)
    theta = L.theta_of_depth(spec["depth0"])
    up, ur = L.upper_pose(sp, sr, theta)
    kp, kr = L.lever_pose(up, ur, 0.0)
    boards = {i: (np.zeros(3) if i == spec["board"] else PARK.copy()) for i in range(len(L.BOARDS))}
    return dict(spec=spec, shoe=(sp, sr), upper=(up, ur), lever=(kp, kr), theta=theta, phi=0.0, boards=boards)


_SPEC: dict = {}
J_ROT = np.column_stack([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])  # joint frame: X = -v (saw frame)


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
            vis = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(COLORS[color]), roughness=0.5)
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

    def body(path, mass, kinematic=False, inertia=None, com=None, link=False):
        prim = stage.GetPrimAtPath(path)
        rb = UsdPhysics.RigidBodyAPI.Apply(prim)
        if kinematic:
            rb.CreateKinematicEnabledAttr(True)
        m = UsdPhysics.MassAPI.Apply(prim)
        m.CreateMassAttr(float(mass))
        if inertia is not None:
            m.CreateDiagonalInertiaAttr(Gf.Vec3f(*map(float, inertia)))
        if com is not None:
            m.CreateCenterOfMassAttr(Gf.Vec3f(*map(float, com)))
        px = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        px.CreateMaxDepenetrationVelocityAttr(0.2)
        px.CreateLinearDampingAttr(0.0)
        px.CreateAngularDampingAttr(0.05)
        if not link:
            px.CreateSolverPositionIterationCountAttr(32)
            px.CreateSolverVelocityIterationCountAttr(4)
            px.CreateSleepThresholdAttr(0.0)
            px.CreateStabilizationThresholdAttr(0.0)
        PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)

    count = {"n": 0}

    def hull(path, v, color, friction):
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
        UsdPhysics.CollisionAPI.Apply(prim)
        cp = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        cp.CreateContactOffsetAttr(CONTACT_OFFSET)
        cp.CreateRestOffsetAttr(0.0)
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("convexHull")
        hc = PhysxSchema.PhysxConvexHullCollisionAPI.Apply(prim)
        hc.CreateHullVertexLimitAttr(255)
        hc.CreateMinThicknessAttr(0.0001)
        bind_physics_material(str(prim.GetPath()), material(friction), stage=stage)
        bind_visual_material(str(prim.GetPath()), look(color), stage=stage)
        count["n"] += 1
        return m

    def visual_mesh(path, v, faces, color):
        m = UsdGeom.Mesh.Define(stage, path)
        m.CreatePointsAttr([Gf.Vec3f(*map(float, p)) for p in v])
        m.CreateFaceVertexCountsAttr([len(f) for f in faces])
        m.CreateFaceVertexIndicesAttr([int(j) for f in faces for j in f])
        m.CreateSubdivisionSchemeAttr("none")
        m.CreateDoubleSidedAttr(True)
        bind_visual_material(str(m.GetPrim().GetPath()), look(color), stage=stage)

    def visual_box(path, center, size, color):
        cube = UsdGeom.Cube.Define(stage, path)
        cube.CreateSizeAttr(1.0)
        cube.AddTranslateOp().Set(Gf.Vec3d(*map(float, center)))
        cube.AddScaleOp().Set(Gf.Vec3f(*map(float, size)))
        bind_visual_material(str(cube.GetPrim().GetPath()), look(color), stage=stage)

    a0 = authored(SETUP_BASE)
    # ------------------------------------------------------------------ stock: boards (kinematic) and blocks
    for i, (_, t) in enumerate(L.BOARDS):
        p = f"{prim_path}/board{i}"
        xform(p, a0["boards"][i], np.eye(3))
        body(p, 5.0, kinematic=True)
        for nm, v, col, fr in L.stock_pieces(t)[:1]:
            hull(f"{p}/{nm}", v, BOARD_COLORS[i], fr)
        # end-grain stripe on the near edge face so the board's thickness reads in the cameras
        c = L.board_center(t)
        visual_box(f"{p}/edge", (L.BOARD_X0 - 0.0004, c[1], c[2]), (0.0008, L.BOARD_L, t * 0.999), "wood2")
    p = f"{prim_path}/blocks"
    xform(p, np.zeros(3), np.eye(3))
    body(p, 2.0, kinematic=True)
    for nm, v, col, fr in L.stock_pieces(L.BOARDS[0][1])[1:]:
        hull(f"{p}/{nm}", v, col, fr)
    # ------------------------------------------------------------------ the saw (floating articulation)
    saw = UsdGeom.Xform.Define(stage, f"{prim_path}/saw")
    UsdPhysics.ArticulationRootAPI.Apply(saw.GetPrim())
    pa = PhysxSchema.PhysxArticulationAPI.Apply(saw.GetPrim())
    pa.CreateSolverPositionIterationCountAttr(32)
    pa.CreateSolverVelocityIterationCountAttr(4)
    pa.CreateEnabledSelfCollisionsAttr(False)
    pa.CreateSleepThresholdAttr(0.0)
    pa.CreateStabilizationThresholdAttr(0.0)
    paths = {n: f"{prim_path}/saw/{n}" for n in LINKS}
    sp, sr = a0["shoe"]
    up, ur = L.upper_pose(sp, sr, 0.0)  # joints authored at zero; reset sets the start angle
    kp, kr = L.lever_pose(up, ur, 0.0)
    xform(paths["shoe"], sp, sr)
    body(paths["shoe"], L.M_SHOE, inertia=(0.0042, 0.0050, 0.0090), com=(0.0, 0.040, 0.008), link=True)
    for nm, v, col, fr in L.shoe_pieces():
        hull(f"{paths['shoe']}/{nm}", v, col, fr)
    for k, (pt, d_in) in enumerate(L.depth_ticks_s()):
        big = abs(d_in - round(d_in)) < 1e-6
        visual_box(f"{paths['shoe']}/tick_{k}", (pt[0] - 0.013, L.QUAD_V[0] - 0.0004, pt[2]),
                   (0.010 if big else 0.006, 0.0008, 0.0012 if big else 0.0008), "white")
    xform(paths["upper"], up, ur)
    body(paths["upper"], L.M_UPPER, inertia=L.upper_inertia(), com=L.upper_com_u(), link=True)
    for nm, v, col, fr in L.upper_pieces():
        if nm == "blade":
            hull(f"{paths['upper']}/{nm}", v, col, fr)
            UsdGeom.Imageable(stage.GetPrimAtPath(f"{paths['upper']}/{nm}")).MakeInvisible()
        else:
            hull(f"{paths['upper']}/{nm}", v, col, fr)
    # blade (visual): toothed plate, both faces + rim, around the arbor in the u-w plane
    poly = L.blade_polygon(teeth=True)
    c = L.ARBOR0 - L.PIVOT
    n = len(poly)
    verts, faces = [], []
    for v in (-L.BLADE_T / 2, L.BLADE_T / 2):
        verts.append([c[0], v, c[2]])
        verts += [[c[0] + p[0], v, c[2] + p[1]] for p in poly]
    for side in (0, 1):
        o = side * (n + 1)
        for i in range(n):
            a, b = o + 1 + i, o + 1 + (i + 1) % n
            faces.append([o, a, b] if side else [o, b, a])
    for i in range(n):
        a, b = 1 + i, 1 + (i + 1) % n
        faces.append([a, b, b + n + 1, a + n + 1])
    visual_mesh(f"{paths['upper']}/blade_vis", verts, faces, "steel")
    visual_box(f"{paths['upper']}/arbor_bolt", c + [0, -0.003, 0], (0.014, 0.004, 0.014), "black")
    xform(paths["lever"], kp, kr)
    body(paths["lever"], L.M_LEVER, inertia=(1.5e-5, 4.0e-5, 4.0e-5), com=(0.025, -0.008, 0.0), link=True)
    for nm, v, col, fr in L.lever_pieces():
        hull(f"{paths['lever']}/{nm}", v, col, fr)
    q = _quat(J_ROT)
    qj = Gf.Quatf(float(q[3]), *map(float, q[:3]))

    def joint(name, b0, b1, pos0, lo_deg, hi_deg, stiff, damp, maxf):
        j = UsdPhysics.RevoluteJoint.Define(stage, f"{prim_path}/saw/{name}")
        j.CreateAxisAttr("X")
        j.CreateBody0Rel().SetTargets([Sdf.Path(paths[b0])])
        j.CreateBody1Rel().SetTargets([Sdf.Path(paths[b1])])
        j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, pos0)))
        j.CreateLocalRot0Attr(qj)
        j.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
        j.CreateLocalRot1Attr(qj)
        j.CreateLowerLimitAttr(float(lo_deg))
        j.CreateUpperLimitAttr(float(hi_deg))
        d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
        d.CreateTypeAttr("force")
        d.CreateStiffnessAttr(float(stiff * DEG))
        d.CreateDampingAttr(float(damp * DEG))
        d.CreateTargetPositionAttr(0.0)
        d.CreateTargetVelocityAttr(0.0)
        d.CreateMaxForceAttr(float(maxf))
        PhysxSchema.PhysxJointAPI.Apply(j.GetPrim()).CreateArmatureAttr(0.0)
        return j

    joint("pivot", "shoe", "upper", L.PIVOT, 0.0, math.degrees(L.THETA_MAX), L.CLAMP_K, L.CLAMP_C, L.CLAMP_NM)
    joint("lever_bolt", "upper", "lever", L.BOLT0 - L.PIVOT, 0.0, math.degrees(L.LEVER_PHI_MAX), 0.0,
          L.LEVER_DAMP, 1.0)
    _SPEC["paths"] = paths
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


def finger_arms(environment):
    """Per arm: (two pad-tip points (env frame), squeeze force N, [joint7, joint8] finger forces N). Finger force
    = the gripper drive law on the finger's position error (as the drive applies it, capped at its effort)."""
    from yam_arena.constants import GRIPPER, PAD_TIP_PAST_TCP, TCP_OFFSET

    base = environment.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(float)
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        pose = robot.data.body_link_pose_w[0].detach().cpu().numpy().astype(float)[
            list(robot.body_names).index("gripper")]
        jp = robot.data.joint_pos[0].detach().cpu().numpy().astype(float)
        jt = robot.data.joint_pos_target[0].detach().cpu().numpy().astype(float)
        jn = list(robot.joint_names)
        ids = [jn.index(j) for j in ("joint7", "joint8")]
        half = max(abs(jp[i]) for i in ids)
        fingers = [min(GRIPPER["effort"], GRIPPER["stiffness"] * max(0.0, abs(jp[i]) - abs(jt[i]))) for i in ids]
        r = _rot(pose[3:7])
        tip = pose[:3] + r @ (np.array(TCP_OFFSET) + [0, 0, PAD_TIP_PAST_TCP]) - origin
        pts = [tip + r[:, 0] * s * (half + 0.0032) for s in (1.0, -1.0)]
        out.append((pts, float(max(fingers)), [float(f) for f in fingers]))
    return out


class Runtime:
    """Runs the depth clamp and the lever cam every physics step, scores every control step."""

    def __init__(self, environment):
        import omni.physics.tensors as tensors
        import torch
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import UsdUtils

        self.base = environment.unwrapped
        self.env = environment
        self.torch = torch
        stage = get_current_stage()
        stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
        self.simview = tensors.create_simulation_view("torch", stage_id=stage_id)
        self.simview.set_subspace_roots("/")
        root = "/World/envs/env_0/saw_bench"
        self.root = root
        self.art = self.simview.create_articulation_view(f"{root}/saw")
        self.link_names = [n.rsplit("/", 1)[-1] for n in self.art.shared_metatype.link_names]
        self.dof_names = list(self.art.shared_metatype.dof_names)
        self.li = {n: self.link_names.index(n) for n in LINKS}
        self.di = {"pivot": self.dof_names.index("pivot"), "lever": self.dof_names.index("lever_bolt")}
        self.kview = self.simview.create_rigid_body_view([f"{root}/board{i}" for i in range(len(L.BOARDS))])
        self.origin = self.base.scene.env_origins[0].detach().cpu().numpy().astype(float)
        dev = self.art.get_dof_positions().device
        self.dev = dev
        self.idx0 = torch.tensor([0], dtype=torch.int32, device=dev)
        self.ndof = len(self.dof_names)
        print("sawkit: links", self.link_names, "dofs", self.dof_names, flush=True)
        self.setup = SETUP_BASE
        self.spec = L.setup_spec(self.setup)
        self.trial_active = False
        self.probe = PROBE
        self.clamp_mode = None
        self.anchor = None
        self.clamp_on = False
        self._init_state()

    def _init_state(self):
        self.scorer = Scorer()
        self.steps = 0
        self.t = 0.0
        self.nonfinite = 0
        self.slips = 0
        self.clamp_events = []
        self.grip_peak = [0.0, 0.0]
        self.phi_max = 0.0
        self.theta_range = [9.0, -9.0]
        self.hook_s = 0.0
        self.extra = {}
        STATE.update(score=0.0, level=0.0, steps=0, detail={}, nonfinite=0, setup=self.setup)

    def begin_trial(self, trial):
        self._init_state()
        self.trial_active = True
        STATE.update(trial=trial, setup=self.setup, start=self.spec)

    # ------------------------------------------------------------------ reading
    def dofs(self):
        q = self.art.get_dof_positions().detach().cpu().numpy().astype(float)[0]
        return float(q[self.di["pivot"]]), float(q[self.di["lever"]])

    def read(self):
        tf = self.art.get_link_transforms().detach().cpu().numpy().astype(float)[0]
        if not np.all(np.isfinite(tf)):
            self.nonfinite += 1
            tf = np.nan_to_num(tf)
        return {n: (tf[i, :3] - self.origin, _rot(tf[i, 3:7])) for n, i in self.li.items()}

    def speeds(self):
        v = self.art.get_link_velocities().detach().cpu().numpy().astype(float)[0]
        return float(np.linalg.norm(v[:, :3], axis=1).max()), math.degrees(float(np.linalg.norm(v[:, 3:6], axis=1).max()))

    # ------------------------------------------------------------------ drives
    def _set_pivot_drive(self, stiff, damp, maxf, target=None):
        torch = self.torch
        k = self.art.get_dof_stiffnesses().clone()
        c = self.art.get_dof_dampings().clone()
        m = self.art.get_dof_max_forces().clone()
        i = self.di["pivot"]
        k[0, i], c[0, i], m[0, i] = stiff, damp, maxf
        cpu = torch.tensor([0], dtype=torch.int32)
        self.art.set_dof_stiffnesses(k.cpu(), cpu)
        self.art.set_dof_dampings(c.cpu(), cpu)
        self.art.set_dof_max_forces(m.cpu(), cpu)
        if target is not None:
            self._set_target(target)

    def _set_target(self, target):
        tg = self.art.get_dof_position_targets().clone()
        tg[0, self.di["pivot"]] = float(target)
        self.art.set_dof_position_targets(tg, self.idx0)

    def clamp(self, theta, phi):
        engaged = phi < L.LEVER_PHI_ENGAGE
        if engaged and self.clamp_mode != "locked":
            self.anchor = theta
            self._set_pivot_drive(L.CLAMP_K, L.CLAMP_C, L.CLAMP_NM, target=theta)
            self.clamp_mode = "locked"
            self.clamp_events.append(dict(ev="lock", t=round(self.t, 3), theta_deg=round(math.degrees(theta), 3)))
        elif not engaged and self.clamp_mode != "loose":
            self._set_pivot_drive(0.0, L.LOOSE_C, L.LOOSE_NM, target=theta)
            self.clamp_mode = "loose"
            self.anchor = None
            self.clamp_events.append(dict(ev="release", t=round(self.t, 3), theta_deg=round(math.degrees(theta), 3)))
        elif engaged and self.anchor is not None:
            # slip: past the holding torque the clamp re-anchors where the bracket now is
            tq = L.CLAMP_K * (self.anchor - theta)
            if abs(tq) > L.CLAMP_NM * 1.02:
                self.anchor = theta + math.copysign(L.CLAMP_NM / L.CLAMP_K, tq)
                self._set_target(self.anchor)
                self.slips += 1
        self.clamp_on = engaged
        return engaged

    def lever_torque(self, phi):
        """Cam: toward locked below the over-centre point, toward released above it (N m, +phi = releasing)."""
        return L.LEVER_TOGGLE_NM * math.tanh((phi - L.LEVER_PHI_MID) / math.radians(3.0))

    def physics(self, dt):
        t0 = time.perf_counter()
        self.t += dt
        theta, phi = self.dofs()
        self.clamp(theta, phi)
        torch = self.torch
        f = torch.zeros((1, self.ndof), dtype=torch.float32, device=self.dev)
        f[0, self.di["lever"]] = float(self.lever_torque(phi))
        self.art.set_dof_actuation_forces(f, self.idx0)
        self.phi_max = max(self.phi_max, phi)
        self.theta_range = [min(self.theta_range[0], theta), max(self.theta_range[1], theta)]
        if self.probe:
            import probe_forces

            probe_forces.step(self, theta, phi)
        self.hook_s += time.perf_counter() - t0

    def apply_link_forces(self, forces):
        """Development probes only: {link: (force world N, point world)}."""
        torch = self.torch
        nl = len(self.link_names)
        fd = torch.zeros((1, nl, 3), dtype=torch.float32, device=self.dev)
        td = torch.zeros((1, nl, 3), dtype=torch.float32, device=self.dev)
        pd = torch.zeros((1, nl, 3), dtype=torch.float32, device=self.dev)
        for n, (fw, at) in forces.items():
            i = self.li[n]
            fd[0, i] = torch.tensor(np.asarray(fw, float), dtype=torch.float32)
            pd[0, i] = torch.tensor(np.asarray(at, float) + self.origin, dtype=torch.float32)
        self.art.apply_forces_and_torques_at_position(fd, td, pd, self.idx0, True)

    # ------------------------------------------------------------------ reset
    def restore_authored(self, setup):
        """Environment reset only (never during a trial): the trial's board on the blocks, the saw on it at the start
        depth, lever locked, at rest."""
        torch = self.torch
        self.setup = setup
        self.trial_active = False
        a = authored(setup)
        self.spec = a["spec"]
        ktf = self.kview.get_transforms().clone()
        for i in range(len(L.BOARDS)):
            ktf[i, :3] = torch.tensor(a["boards"][i] + self.origin, dtype=ktf.dtype)
            ktf[i, 3:7] = torch.tensor([0.0, 0.0, 0.0, 1.0], dtype=ktf.dtype)
        self.kview.set_transforms(ktf, torch.arange(len(L.BOARDS), dtype=torch.int32, device=ktf.device))
        sp, sr = a[self.link_names[0]]  # the articulation's root link (PhysX picks it; reported at start-up)
        rt = self.art.get_root_transforms().clone()
        rt[0, :3] = torch.tensor(sp + self.origin, dtype=rt.dtype)
        rt[0, 3:7] = torch.tensor(_quat(sr), dtype=rt.dtype)
        self.art.set_root_transforms(rt, self.idx0)
        self.art.set_root_velocities(torch.zeros_like(self.art.get_root_velocities()), self.idx0)
        dp = self.art.get_dof_positions().clone()
        dp[0, self.di["pivot"]] = a["theta"]
        dp[0, self.di["lever"]] = 0.0
        self.art.set_dof_positions(dp, self.idx0)
        self.art.set_dof_velocities(torch.zeros_like(self.art.get_dof_velocities()), self.idx0)
        self.clamp_mode = None
        self.clamp(a["theta"], 0.0)
        self.restored = True
        self._init_state()

    # ------------------------------------------------------------------ scoring
    def control(self, environment):
        if not self.trial_active:
            return
        self.steps += 1
        poses = self.read()
        theta, phi = self.dofs()
        arms = finger_arms(environment)
        tips = [p for a in arms for p in a[0]]
        for i, a in enumerate(arms):
            self.grip_peak[i] = max(self.grip_peak[i], a[1])
        m = measure(poses, self.speeds(), theta, phi, self.clamp_on, self.spec["t"], tips)
        level = self.scorer.step(m)
        STATE.update(
            score=level,
            level=level,
            steps=self.steps,
            nonfinite=self.nonfinite,
            detail=dict(
                self.scorer.last,
                clamp=self.clamp_mode,
                anchor_deg=None if self.anchor is None else round(math.degrees(self.anchor), 4),
                slips=self.slips,
                clamp_events=self.clamp_events[-12:],
                grip_n=[round(a[1], 2) for a in arms],
                grip_peak_n=[round(g, 2) for g in self.grip_peak],
                phi_max_deg=round(math.degrees(self.phi_max), 2),
                board=self.spec["board"],
                t_board_mm=round(self.spec["t"] * 1e3, 1),
                setup=self.setup,
                hook_wall_s=round(self.hook_s, 2),
            ),
        )


_RT: dict = {"rt": None, "sim": None, "trials": 0}


def runtime():
    return _RT["rt"]


def saw_state():
    """Current link poses {'shoe', 'upper', 'lever'}, plus 'theta' / 'phi' (rad) and 'spec', for the scripted demo
    (read-only)."""
    rt = _RT["rt"]
    if rt is None:
        return None
    out = rt.read()
    out["theta"], out["phi"] = rt.dofs()
    out["spec"] = rt.spec
    return out


def _patch_scene(scene) -> None:
    if getattr(scene, "_saw_patched", False):
        return
    orig_task_object, orig_states = scene.task_object, scene.object_states

    def task_object(spec):
        if isinstance(spec, SawBench):
            return _arena_object(spec)
        return orig_task_object(spec)

    def object_states(environment, names):
        from yam_arena.task_file import ObjectState

        plain = [n for n in names if n != "saw_bench"]
        out = orig_states(environment, plain) if plain else {}
        if "saw_bench" in names:
            rt = _RT["rt"]
            p, r = rt.read()["upper"] if rt else authored(SETUP_BASE)["upper"]
            z = np.zeros(3)
            out["saw_bench"] = ObjectState(pos=np.asarray(p), quat_xyzw=_quat(r), lin_vel=z, ang_vel=z,
                                           low=p - 0.2, high=p + 0.2)
        return {n: out[n] for n in names}

    orig_build = scene.build_task_environment

    def build(embodiment, task, control_hz=None):
        environment = orig_build(embodiment, task, control_hz)
        if any(isinstance(o, SawBench) for o in task.objects):
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
    scene._saw_patched = True


def _has_bench(emb):
    return any(isinstance(o, SawBench) for o in (emb.task.objects if emb.task is not None else ()))


def _patch_embodiment(ie) -> None:
    cls = ie.YamArenaEmbodiment
    if getattr(cls, "_saw_patched", False):
        return
    base_task_step, base_reset, base_ensure = cls._task_step, cls.reset, cls._ensure_sim

    def _ensure_sim(self, *a, **k):
        env_before = self._env
        out = base_ensure(self, *a, **k)
        if self._env is not None and self._env is not env_before and _has_bench(self):
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
            rt.restore_authored(setup)
        obs = base_reset(self, *a, **k)
        rt = _RT["rt"]
        if rt is not None:
            if rt.setup != setup or not getattr(rt, "restored", False):  # first reset of a fresh env
                rt.restore_authored(setup)
                obs = base_reset(self, *a, **k)
            rt.begin_trial(_RT["trials"])
            _RT["trials"] += 1
        return obs

    cls._ensure_sim, cls._task_step, cls.reset = _ensure_sim, _task_step, reset
    cls._saw_patched = True


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
        targets = {"yam_arena.scene": _patch_scene, "yam_arena.inspect_embodiment": _patch_embodiment}

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
    if not any(type(f).__name__ == "_Finder" and getattr(f, "targets", None) and "yam_arena.scene" in f.targets
               for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


install()
