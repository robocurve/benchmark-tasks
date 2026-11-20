"""jointkit: the SO-101 joint-1 bench for yam_arena (Isaac Lab-Arena / PhysX), plus its runtime hooks.

Authored as one custom-spawned assembly (yam_arena task-file primitives are single boxes/spheres):
* the partly built arm (official SO-101 base, base motor holder and STS3215 servo case) as ONE static triangle-mesh
  collider: the base is clamped to the table;
* the servo's two horns: the top (output) horn a kinematic body fixed at its start angle (the unpowered gear train's
  backdrive, about 0.6 N m, is over ten times what the bracket can put on it), the bottom (idler) horn a rigid body on
  a revolute joint about the servo axis with a torque-capped zero-velocity drive as its dry friction (nearly free);
* the shoulder bracket (official rotation_pitch_so101_v1) as a free rigid body whose collider is the official mesh
  cut into convex slab pieces (``../src/make_assets.py``).
Geometry, poses and parameters: ``../src/layout.py``; scorer: ``../src/scoring.py`` (pure functions).

Runtime hooks (installed on import; nothing in yam_arena is edited; same pattern as earlier task kits): after every
control step the scorer reads the bracket pose and velocity, the top horn's pose and the finger pad tips. The scripted
demo READS poses through ``bench_state()``. Nothing here writes a body pose, a joint state or a score during a trial;
``Runtime.restore_authored`` puts the bracket and the bottom horn back at their authored poses only on an environment
reset (the top horn never moves).
"""
from __future__ import annotations

import math
import os
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
SRC = HERE.parent / 'src'
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from yam_arena.task_file import Box  # noqa: E402
import layout as L  # noqa: E402
from scoring import Scorer, measure, boxes_distance  # noqa: E402

YELLOW, BLACK, HORN, GREY = (1.0, .82, .12), (.09, .09, .1), (.32, .33, .35), (.6, .6, .62)
PHYSICS_DT = float(os.environ.get('SO101_PHYSICS_DT', str(1 / 960)))
CONTACT_OFFSET = float(os.environ.get('SO101_CONTACT_OFFSET', '.0001'))  # < the 0.25-0.35 mm slot clearances (see task.json fidelity)
HULL_MIN_THICKNESS = float(os.environ.get('SO101_HULL_MIN_THICKNESS', '.0002'))
BODIES = ('horn_top', 'horn_bottom', 'bracket')
# Horn joints: 'maximal' (default) = the top horn fixed (kinematic), the bottom horn a rigid body on a revolute joint
# with a torque-capped velocity drive as dry friction; 'articulation' = both horns as links of a fixed-base articulation with PhysX Coulomb joint friction, kept
# selectable only for the comparison probes: in this engine build its contacts with the bracket chatter (about
# 100 mm/s instantaneous velocity at a constant pose), which locks the bracket in its slots (probe_mouth.py)
HORN_JOINTS = os.environ.get('SO101_HORN_JOINTS', 'maximal')
# maximal horns: damping of the zero-velocity drive (N m s/rad; capped at the friction torque). Below the cap it is a
# damper: creep = torque / damping, 0.006 deg/s at 0.54 N m with 5000 (50 crept 0.6 deg/s: probe_horn)
HORN_DRIVE_DAMPING = float(os.environ.get('SO101_HORN_DAMPING', '5000'))


class Joint1(Box):
    """The bench, as one task-file object (subclasses Box only to pass yam_arena's checks)."""


def _rot(q_xyzw):
    x, y, z, w = q_xyzw
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _quat(r):
    """Rotation matrix -> quaternion XYZW."""
    r = np.asarray(r, float)
    t = np.trace(r)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        w, x, y, z = .25 * s, (r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2
        w, x, y, z = (r[2, 1] - r[1, 2]) / s, .25 * s, (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2
        w, x, y, z = (r[0, 2] - r[2, 0]) / s, (r[0, 1] + r[1, 0]) / s, .25 * s, (r[1, 2] + r[2, 1]) / s
    else:
        s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2
        w, x, y, z = (r[1, 0] - r[0, 1]) / s, (r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s, .25 * s
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def authored():
    """Authored (reset) pose of every dynamic body: {name: (pos, rot)} in the env frame."""
    return {'horn_top': (L.O_S.copy(), L.horn_rot(L.HORN_TOP_START_DEG)),
            'horn_bottom': (L.O_S.copy(), L.horn_rot(L.HORN_BOTTOM_START_DEG)),
            'bracket': (L.BRACKET_START.copy(), L.bracket_start_rot())}


_SPEC: dict = {}


def _spawn_bench(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics
    from yam_arena.station import linear_rgb

    asset = L.load_asset()
    _SPEC.update(asset=asset)
    stage = get_current_stage()
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    ox, oy, oz, ow = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(ow, ox, oy, oz))
    root_world = np.array(root.ComputeLocalToWorldTransform(0).ExtractTranslation(), float)
    _SPEC['root_world'] = root_world
    looks, mats = {}, {}

    def look(color):
        if color not in looks:
            path = f'{prim_path}/Looks/c{len(looks)}'
            vis = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=.5)
            vis.func(path, vis)
            looks[color] = path
        return looks[color]

    def material(friction):
        if friction not in mats:
            path = f'{prim_path}/Looks/phys{len(mats)}'
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

    def collide(prim, friction, color=None):
        UsdPhysics.CollisionAPI.Apply(prim)
        cp = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        cp.CreateContactOffsetAttr(CONTACT_OFFSET)
        cp.CreateRestOffsetAttr(0.)
        bind_physics_material(str(prim.GetPath()), material(friction), stage=stage)
        if color is not None:
            bind_visual_material(str(prim.GetPath()), look(color), stage=stage)

    def rigid(path, mass):
        prim = stage.GetPrimAtPath(path)
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(float(mass))
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateMaxDepenetrationVelocityAttr(.5)
        rb.CreateSolverPositionIterationCountAttr(32)
        rb.CreateSolverVelocityIterationCountAttr(4)
        rb.CreateLinearDampingAttr(0.)
        rb.CreateAngularDampingAttr(.5)

    def mesh(path, v, f, color, collision=None, friction=.5, normals=None):
        if collision is None and os.environ.get('SO101_DEBUG_NOVIS'):
            return None
        m = UsdGeom.Mesh.Define(stage, path)
        m.CreatePointsAttr([Gf.Vec3f(*map(float, p)) for p in v])
        m.CreateFaceVertexCountsAttr([len(t) for t in f])
        m.CreateFaceVertexIndicesAttr([int(j) for t in f for j in t])
        m.CreateSubdivisionSchemeAttr('none')
        lo, hi = np.min(v, axis=0), np.max(v, axis=0)
        m.CreateExtentAttr([Gf.Vec3f(*map(float, lo)), Gf.Vec3f(*map(float, hi))])
        if normals is not None:
            m.CreateNormalsAttr([Gf.Vec3f(*map(float, n)) for n in normals])
            m.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
        if collision is None:
            bind_visual_material(str(m.GetPrim().GetPath()), look(color), stage=stage)
        else:
            collide(m.GetPrim(), friction, color)
            UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr(collision)
            if collision == 'convexHull':
                hull = PhysxSchema.PhysxConvexHullCollisionAPI.Apply(m.GetPrim())
                hull.CreateMinThicknessAttr(HULL_MIN_THICKNESS)
                hull.CreateHullVertexLimitAttr(255)  # keep every authored vertex (horn discs have 192)
            UsdGeom.Imageable(m.GetPrim()).MakeInvisible()
        return m

    def cylinder(path, r, z0, z1, color, friction):
        """Round pin as an explicit 48-gon convex prism (its vertices on the true radius)."""
        a = np.linspace(0, 2 * np.pi, 48, endpoint=False)
        v = np.r_[np.c_[r * np.cos(a), r * np.sin(a), np.full(48, z0)], np.c_[r * np.cos(a), r * np.sin(a), np.full(48, z1)]]
        mesh(path, v, _Hull(v).simplices, color, collision='convexHull', friction=friction)

    vis = asset['vis']
    # --- clamped arm base: one static collider (official meshes, horn spaces cut out) + visuals
    base = f'{prim_path}/arm_base'
    xform(base, L.O_S, L.R_S)
    sv, sf = asset['static']
    if 'static' not in set(filter(None, os.environ.get('SO101_DEBUG_DROP', '').split(','))):
        mesh(f'{base}/collider', sv, sf, GREY, collision='none', friction=.5)
    for k, col in (('base', YELLOW), ('base_motor_holder', YELLOW), ('servo_case', BLACK)):
        mesh(f'{base}/vis_{k}', *vis[k][:2], col, normals=vis[k][2])
    poses = authored()
    # --- horns
    for name, col in (('horn_top', HORN), ('horn_bottom', HORN)):
        path = f'{prim_path}/{name}'
        xform(path, *poses[name])
        rigid(path, L.HORN_MASS)
        if name == 'horn_top' and HORN_JOINTS != 'articulation':
            # top horn fixed at its start angle: the unpowered gearbox's backdrive (about 0.6 N m) is over ten times
            # what the bracket can put on it through friction; the jointed horn turned by contact noise (artifact)
            UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath(path)).CreateKinematicEnabledAttr(True)
        mesh(f'{path}/vis', *vis[name][:2], col, normals=vis[name][2])
    from scipy.spatial import ConvexHull as _Hull

    def disc(path, r, z0, z1, chamfer_at):
        """Horn disc as a convex 64-gon prism with a 0.4 mm x 45 deg chamfer on its outer face's rim (moulded /
        machined horns have a rounded rim; this is the only lead-in modelled)."""
        a = np.linspace(0, 2 * np.pi, 64, endpoint=False)
        ring = lambda rr, z: np.c_[rr * np.cos(a), rr * np.sin(a), np.full(64, z)]  # noqa: E731
        ch = L.HORN_CHAMFER
        if chamfer_at == 'top':
            v = np.r_[ring(r, z0), ring(r, z1 - ch), ring(r - ch, z1)]
        else:
            v = np.r_[ring(r - ch, z0), ring(r, z0 + ch), ring(r, z1)]
        mesh(path, v, _Hull(v).simplices, HORN, collision='convexHull', friction=.3)

    disc(f'{prim_path}/horn_top/disc', L.HORN_R, *L.HORN_TOP_Z, 'top')
    drop = set(filter(None, os.environ.get('SO101_DEBUG_DROP', '').split(',')))  # development probes only
    if 'head' not in drop:
        cylinder(f'{prim_path}/horn_top/screw_head', L.SCREW_HEAD[0], L.SCREW_HEAD[1], L.SCREW_HEAD[2], GREY, .3)
    disc(f'{prim_path}/horn_bottom/disc', L.HORN_R, *L.HORN_BOTTOM_Z, 'bottom')
    if 'boss' not in drop:
        # the idler boss is part of the servo case (the bottom horn turns on it): a static collider of the clamped
        # base, its top 0.1 mm short of the bottom horn's face, and never collided with the horn (the horn link is
        # held by its joint: a static contact on it would over-constrain it)
        cylinder(f'{base}/boss', L.BOSS[0], L.BOSS[1], L.BOSS[2] - .0001, HORN, .3)
        UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(f'{base}/boss')).CreateFilteredPairsRel().AddTarget(
            f'{prim_path}/horn_bottom')
    q0 = _quat(L.R_S)
    if HORN_JOINTS == 'articulation':
        # comparison only (see HORN_JOINTS): the horns as the two links of a fixed-base articulation (mount body
        # fixed to the world at the servo frame) with PhysX Coulomb joint friction (static / dynamic friction effort)
        mount = f'{prim_path}/servo_mount'
        xform(mount, L.O_S, L.R_S)
        rigid(mount, .05)
        UsdPhysics.MassAPI(stage.GetPrimAtPath(mount)).CreateDiagonalInertiaAttr(Gf.Vec3f(1e-5, 1e-5, 1e-5))
        fix = UsdPhysics.FixedJoint.Define(stage, f'{mount}/fix')
        fix.CreateBody1Rel().SetTargets([Sdf.Path(mount)])  # body0 empty = world (clamped base)
        fix.CreateLocalPos0Attr(Gf.Vec3f(*map(float, L.O_S + root_world)))
        fix.CreateLocalRot0Attr(Gf.Quatf(float(q0[3]), *map(float, q0[:3])))
        fix.CreateLocalPos1Attr(Gf.Vec3f(0., 0., 0.))
        fix.CreateLocalRot1Attr(Gf.Quatf(1., 0., 0., 0.))
        UsdPhysics.ArticulationRootAPI.Apply(fix.GetPrim())
        art = PhysxSchema.PhysxArticulationAPI.Apply(fix.GetPrim())
        art.CreateEnabledSelfCollisionsAttr(False)
        art.CreateSolverPositionIterationCountAttr(32)
        art.CreateSolverVelocityIterationCountAttr(4)
        _SPEC['articulation'] = str(fix.GetPrim().GetPath())
    for name, friction_nm, start in (('horn_top', L.HORN_TOP_FRICTION_NM, L.HORN_TOP_START_DEG),
                                     ('horn_bottom', L.HORN_BOTTOM_FRICTION_NM, L.HORN_BOTTOM_START_DEG)):
        if name == 'horn_top' and HORN_JOINTS != 'articulation':
            continue
        j = UsdPhysics.RevoluteJoint.Define(stage, f'{prim_path}/{name}/axis')
        j.CreateAxisAttr('Z')
        j.CreateBody1Rel().SetTargets([Sdf.Path(f'{prim_path}/{name}')])
        q1 = _quat(L.rz(-start))
        j.CreateLocalPos1Attr(Gf.Vec3f(0., 0., 0.))
        j.CreateLocalRot1Attr(Gf.Quatf(float(q1[3]), *map(float, q1[:3])))
        if HORN_JOINTS == 'articulation':
            j.CreateBody0Rel().SetTargets([Sdf.Path(mount)])
            j.CreateLocalPos0Attr(Gf.Vec3f(0., 0., 0.))
            j.CreateLocalRot0Attr(Gf.Quatf(1., 0., 0., 0.))
            prim = j.GetPrim()
            prim.AddAppliedSchema('PhysxJointAxisAPI:angular')
            for attr, value in (('staticFrictionEffort', friction_nm), ('dynamicFrictionEffort', friction_nm),
                                ('viscousFrictionCoefficient', 0.)):
                prim.CreateAttribute(f'physxJointAxis:angular:{attr}', Sdf.ValueTypeNames.Float).Set(float(value))
        else:
            # default: joint to the world with a stiff zero-velocity drive capped at the friction torque (dry
            # friction up to creep: isaac/probe_horn.py checks it holds below the cap, turns above it and stops)
            j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, L.O_S + root_world)))
            j.CreateLocalRot0Attr(Gf.Quatf(float(q0[3]), *map(float, q0[:3])))
            d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), 'angular')
            d.CreateTypeAttr('force')
            d.CreateStiffnessAttr(0.)
            d.CreateDampingAttr(float(HORN_DRIVE_DAMPING * math.pi / 180))  # USD angular drives: N m s/deg
            d.CreateMaxForceAttr(float(friction_nm))
            d.CreateTargetVelocityAttr(0.)
    # --- shoulder bracket
    path = f'{prim_path}/bracket'
    xform(path, *poses['bracket'])
    rigid(path, L.BRACKET_MASS)
    mesh(f'{path}/vis', *vis['bracket'][:2], YELLOW, normals=vis['bracket'][2])
    if os.environ.get('SO101_CONTACT_REPORT'):  # development only (probe_fit.py): list the bracket's contacts
        PhysxSchema.PhysxContactReportAPI.Apply(stage.GetPrimAtPath(path)).CreateThresholdAttr(0.)
    drop_pieces = set()  # development probes only: SO101_DROP_PIECES="a-b,c" leaves these bracket pieces out
    for tok in filter(None, os.environ.get('SO101_DROP_PIECES', '').split(',')):
        a, _, b = tok.partition('-')
        drop_pieces.update(range(int(a), int(b or a) + 1))
    for i, p in enumerate(asset['pieces']):
        if 'lip' in drop and abs(p[:, 2].min() + .0012) < 1e-6 and abs(p[:, 2].max()) < 1e-6:
            continue
        if i in drop_pieces:
            continue
        from scipy.spatial import ConvexHull
        hull = ConvexHull(p)
        mesh(f'{path}/piece_{i:03d}', p, hull.simplices, YELLOW, collision='convexHull', friction=L.BRACKET_FRICTION)
    _SPEC['paths'] = {n: f'{prim_path}/{n}' for n in BODIES}
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

    return Object(name=spec.name, object_type=ObjectType.BASE, spawner_cfg=BenchSpawnerCfg(),
                  initial_pose=Pose(position_xyz=(0., 0., 0.), rotation_xyzw=(0., 0., 0., 1.)))


# ---------------------------------------------------------------------------------------------- runtime state
STATE: dict = dict(score=0., level=0., steps=0, detail={}, nonfinite=0)


def finger_tips(environment):
    """Pad-tip centres of all four fingers (env frame)."""
    from yam_arena.constants import PAD_TIP_PAST_TCP, TCP_OFFSET
    base = environment.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(float)
    pts = []
    for arm in ('left_robot', 'right_robot'):
        robot = base.scene[arm]
        pose = robot.data.body_link_pose_w[0].detach().cpu().numpy().astype(float)[list(robot.body_names).index('gripper')]
        jp = robot.data.joint_pos[0].detach().cpu().numpy().astype(float)
        jn = list(robot.joint_names)
        half = max(abs(jp[jn.index(j)]) for j in ('joint7', 'joint8'))
        r = _rot(pose[3:7])
        tip = pose[:3] + r @ (np.array(TCP_OFFSET) + [0, 0, PAD_TIP_PAST_TCP]) - origin
        pts += [tip + r[:, 0] * s * (half + .0032) for s in (1., -1.)]
    return pts


class Runtime:
    """Reads the three dynamic bodies through a PhysX tensor view and runs the scorer once per control step."""

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
        self.simview = tensors.create_simulation_view('torch', stage_id=stage_id)
        self.simview.set_subspace_roots('/')
        root = '/World/envs/env_0/joint1'  # Arena: '{ENV_REGEX_NS}/' + name, one env
        self.view = self.simview.create_rigid_body_view([f'{root}/{n}' for n in BODIES])
        self.index = {n: i for i, n in enumerate(BODIES)}
        self.art = self.simview.create_articulation_view(_SPEC['articulation']) if _SPEC.get('articulation') else None
        self.origin = self.base.scene.env_origins[0].detach().cpu().numpy().astype(float)
        self.boxes = [(p.min(axis=0), p.max(axis=0)) for p in _SPEC['asset']['pieces']]
        self.reset()

    def reset(self):
        self.scorer = Scorer()
        self.steps = 0
        STATE.update(score=0., level=0., steps=0, detail={}, nonfinite=0)

    def read(self):
        tf = self.view.get_transforms().detach().cpu().numpy().astype(float)
        return {n: (tf[i, :3] - self.origin, _rot(tf[i, 3:7])) for n, i in self.index.items()}

    def velocities(self):
        v = self.view.get_velocities().detach().cpu().numpy().astype(float)
        return {n: (v[i, :3], v[i, 3:6]) for n, i in self.index.items()}

    def restore_authored(self):
        """Environment reset only (never during a trial): horns and bracket back at their authored poses, at rest."""
        torch = self.torch
        tf = self.view.get_transforms().clone()
        # the top horn is kinematic (fixed) and never moves
        names = ['bracket'] if self.art is not None else ['horn_bottom', 'bracket']  # articulation links: through their joints
        for n in names:
            pos, rot = authored()[n]
            i = self.index[n]
            tf[i, :3] = torch.tensor(np.asarray(pos, float) + self.origin, dtype=tf.dtype)
            tf[i, 3:7] = torch.tensor(_quat(rot), dtype=tf.dtype)
        idx = torch.tensor([self.index[n] for n in names], dtype=torch.int32, device=tf.device)
        self.view.set_transforms(tf, idx)
        self.view.set_velocities(torch.zeros_like(self.view.get_velocities()), idx)
        if self.art is not None:  # joint angle 0 = authored horn angle (see the joints' localRot1)
            aidx = torch.arange(self.art.count, dtype=torch.int32, device=tf.device)
            self.art.set_dof_positions(torch.zeros_like(self.art.get_dof_positions()), aidx)
            self.art.set_dof_velocities(torch.zeros_like(self.art.get_dof_velocities()), aidx)

    def control(self, environment):
        """Every control step: measure, score, publish (score() reads STATE)."""
        self.steps += 1
        poses, vel = self.read(), self.velocities()
        finite = all(np.all(np.isfinite(p)) and np.all(np.isfinite(r)) for p, r in poses.values())
        if not finite:
            STATE['nonfinite'] += 1
            return
        pb, rb = poses['bracket']
        m = measure(pb, rb, poses['horn_top'][1], L.O_S, L.R_S)
        tips = finger_tips(environment)
        clear = min(boxes_distance(t, self.boxes, pb, rb) for t in tips)
        lin = float(np.linalg.norm(vel['bracket'][0]))
        ang = math.degrees(float(np.linalg.norm(vel['bracket'][1])))
        level = self.scorer.step(m, clear, lin, ang)
        STATE.update(score=level, level=level, steps=self.steps,
                     detail=dict(self.scorer.last, horn_bottom_deg=round(float(
                         math.degrees(math.atan2(*(L.R_S.T @ poses['horn_bottom'][1][:, 0])[[1, 0]]))), 3),
                         bracket_pos_mm=np.round(pb * 1000, 3).tolist()))


_RT: dict = {'rt': None}


def runtime():
    return _RT['rt']


def bench_state():
    """Current poses {body: (pos, rot)} for the scripted demo (read-only)."""
    return _RT['rt'].read() if _RT['rt'] else None


def _patch_scene(scene) -> None:
    if getattr(scene, '_joint1_patched', False):
        return
    orig_task_object, orig_states = scene.task_object, scene.object_states

    def task_object(spec):
        if isinstance(spec, Joint1):
            return _arena_object(spec)
        return orig_task_object(spec)

    def object_states(environment, names):
        from yam_arena.task_file import ObjectState
        plain = [n for n in names if n != 'joint1']
        out = orig_states(environment, plain) if plain else {}
        if 'joint1' in names:
            rt = _RT['rt']
            p, r = rt.read()['bracket'] if rt else authored()['bracket']
            z = np.zeros(3)
            out['joint1'] = ObjectState(pos=np.asarray(p), quat_xyzw=_quat(r), lin_vel=z, ang_vel=z, low=p - .05,
                                        high=p + .05)
        return {n: out[n] for n in names}

    orig_build = scene.build_task_environment

    def build(embodiment, task, control_hz=None):
        environment = orig_build(embodiment, task, control_hz)
        if any(isinstance(o, Joint1) for o in task.objects):
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
    scene._joint1_patched = True


def _patch_embodiment(ie) -> None:
    cls = ie.YamArenaEmbodiment
    if getattr(cls, '_joint1_patched', False):
        return
    base_task_step, base_reset = cls._task_step, cls.reset

    def _task_step(self, *a, **k):
        if _RT['rt'] is not None:
            _RT['rt'].control(self._env)
        return base_task_step(self, *a, **k)

    def reset(self, *a, **k):
        previous, _RT['rt'] = _RT['rt'], None  # no hooks while yam_arena resets and settles the arms
        if previous is not None:  # in-process reset: bench back to its authored poses first
            previous.restore_authored()
        obs = base_reset(self, *a, **k)
        if any(isinstance(o, Joint1) for o in (self.task.objects if self.task is not None else ())):
            _RT['rt'] = Runtime(self._env)
        return obs

    cls._task_step, cls.reset = _task_step, reset
    cls._joint1_patched = True


def install() -> None:
    import importlib.abc
    import importlib.util

    class _Finder(importlib.abc.MetaPathFinder):
        targets = {'yam_arena.scene': _patch_scene, 'yam_arena.inspect_embodiment': _patch_embodiment}

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
    if not any(type(f).__name__ == '_Finder' and getattr(f, 'targets', None) and 'yam_arena.scene' in f.targets
               for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


install()
