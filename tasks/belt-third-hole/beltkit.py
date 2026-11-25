"""beltkit: a leather belt with a single-prong frame buckle and a fixed keeper, as one articulation for yam_arena.

The strap is a chain of 25 mm rigid links (geom.N_LINKS) joined by sprung D6 joints (flat bend, twist, edge bend);
links 0-1 are the fold round the buckle's hinge bar (two layers), link 1 carries the keeper, the five hole links
have a real oval hole (four boxes round an opening), the last link is pointed. The buckle frame hangs on link 0 by a
revolute joint about its hinge bar; the prong hangs on the frame by a revolute joint about the same bar. Nothing is
glued, latched or switched: the prong holds only by passing through a hole.

Zero joint angles are the authored base start layout (geom.threaded_layout(0)); the reset writes each start layout as
joint angles relative to it. The springs' rest shape is the waist's curve round the ring and straight beyond.

Import this module from the task file before OBJECTS is built; ``Belt`` goes in OBJECTS like any object.
``read(environment, spec)`` returns a BeltState with every body's world pose.
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass, field

import numpy as np

import geom as G
import partkit
from yam_arena.task_file import Box, ObjectState

LEATHER = (0.36, 0.20, 0.10)
METAL = (0.80, 0.80, 0.76)

# joint springs (estimates, see README): leather EI ~ 4e-3 N m^2 -> k = EI / pitch per joint
FLAT_K = 0.004 / G.PITCH        # N m/rad
TWIST_K = 0.10
EDGE_K = 2.0
FLAT_LIM, TWIST_LIM, EDGE_LIM = 100.0, 25.0, 5.0      # deg from straight
DAMP_S = 0.05                   # joint damping = DAMP_S * stiffness (s)
ARMATURE = 5e-6
FRAME_LIM = (-15.0, 200.0)      # frame angle about its hinge bar (deg, 0 = in link 0's plane, + = front bar up)
PRONG_LIM = (-30.0, 200.0)      # prong angle about the bar relative to link 0 (0 = in link 0's plane, + = tip up)
SLOT_W = 0.006                  # the prong's slot in the fold (round 2)
PRONG_BACK_STOP = 1.0           # deg past its turned-back rest on the fold (the fold's top stops it; round 2)
HINGE_DAMP = 2e-5
PRONG_FRICTION = 0.0015         # N m: the prong stays where it is put (its own weight gives 0.0005 N m; ESTIMATE)
FRAME_FRICTION = float(os.environ.get("BELT_FRAME_FRICTION", "0.02"))   # N m: a new belt's frame turns stiffly in its
                                # leather fold and stays where it is put (ESTIMATE; about 1 N at the front bar)
HOLE_SHIFT = float(os.environ.get("BELT_HOLE_SHIFT", "0.0012"))   # fastened probe layout: centre hole 3 on the prong


def _rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _quat(R):
    m = np.asarray(R, float)
    K = np.array(((m[0, 0] - m[1, 1] - m[2, 2], m[0, 1] + m[1, 0], m[0, 2] + m[2, 0], m[2, 1] - m[1, 2]),
                  (m[0, 1] + m[1, 0], m[1, 1] - m[0, 0] - m[2, 2], m[1, 2] + m[2, 1], m[0, 2] - m[2, 0]),
                  (m[0, 2] + m[2, 0], m[1, 2] + m[2, 1], m[2, 2] - m[0, 0] - m[1, 1], m[1, 0] - m[0, 1]),
                  (m[2, 1] - m[1, 2], m[0, 2] - m[2, 0], m[1, 0] - m[0, 1], np.trace(m)))) / 3
    q = np.linalg.eigh(K)[1][:, -1]
    return q if q[3] >= 0 else -q


def quat_to_matrix(q):
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array(((1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
                     (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
                     (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y))))


# ------------------------------------------------------------------------------------------- shapes (body frames)
H = G.PITCH / 2


def link_boxes(i):
    """Collision boxes (centre, size) of link i in its frame (x along the strap, y across, z outward)."""
    t = G.T
    if i == 0:      # the fold; the first 12 mm wrap the bar: only the top layer either side of the prong's slot
        top = G.FOLD_TOP
        x0 = -H + 0.012
        out = [((((x0 + H) / 2), 0.0, (top - t / 2) / 2), (H - x0, G.W, top + t / 2))]
        # round 2: the leather over the bar, with a 6 mm slot for the prong's root (a full-width gap let the tongue
        # sink onto the bar and a hole hook the prong's root)
        z0 = G.BAR_Z + G.BAR_R + 0.0003
        half = (G.W / 2 - SLOT_W / 2) / 2
        for sgn in (1, -1):
            out.append(((-H + 0.006, sgn * (SLOT_W / 2 + half), (z0 + top) / 2), (0.012, 2 * half, top - z0)))
        return out
    if 0 < i < G.KEEPER_LINK:
        top = G.FOLD_TOP
        return [((0.0, 0.0, (top - t / 2) / 2), (G.PITCH, G.W, top + t / 2))]
    if i == G.KEEPER_LINK:      # end of the fold, with the keeper round it
        top = G.FOLD_TOP
        out = [((0.0, 0.0, (top - t / 2) / 2), (G.PITCH, G.W, top + t / 2))]
        kz0 = top + G.KEEPER_GAP
        ky = G.W / 2 + 0.001 + G.KEEPER_T / 2
        out.append(((0.0, 0.0, kz0 + G.KEEPER_T / 2), (G.KEEPER_LEN, 2 * ky + G.KEEPER_T, G.KEEPER_T)))
        for s in (1, -1):
            out.append(((0.0, s * ky, (kz0 - t / 2) / 2), (G.KEEPER_LEN, G.KEEPER_T, kz0 + t / 2)))
        return out
    if i in G.HOLE_LINK.values():
        hx, hy = G.HOLE_Y / 2, G.HOLE_X / 2      # along, across
        side = (G.W / 2 - hy)
        return [((0.0, (hy + side / 2), 0.0), (G.PITCH, side, t)),
                ((0.0, -(hy + side / 2), 0.0), (G.PITCH, side, t)),
                (((hx + H) / 2, 0.0, 0.0), (H - hx, 2 * hy + 0.0002, t)),
                ((-(hx + H) / 2, 0.0, 0.0), (H - hx, 2 * hy + 0.0002, t))]
    if i == G.N_LINKS - 1:  # pointed tip
        return [((-0.004, 0.0, 0.0), (G.PITCH - 0.008, G.W, t)), ((H - 0.004, 0.0, 0.0), (0.008, 0.016, t))]
    return [((0.0, 0.0, 0.0), (G.PITCH, G.W, t))]


def link_visual_extra(i):
    """Visual-only boxes (the leather wrapped round the hinge bar)."""
    if i == 0:
        top = G.FOLD_TOP
        return [((-H + 0.006 - 0.002, 0.0, (top - G.T / 2) / 2), (0.016, G.W, top + G.T / 2))]
    return []


def frame_shapes():
    """Frame (origin at the hinge bar centre; x toward the front bar at -FRAME_L): cylinders along y, boxes."""
    L, r, w = G.FRAME_L, G.BAR_R, G.FRAME_IN_X + 2 * G.BAR_R
    cyl = [((0.0, 0.0, 0.0), r, w + 2 * r), ((-L, 0.0, 0.0), r, w + 2 * r)]
    box = [((-L / 2, s * (G.FRAME_IN_X / 2 + r), 0.0), (L, 2 * r, 2 * r)) for s in (1, -1)]
    return cyl, box


def prong_segments():
    """Prong centre-line points (frame coordinates at prong angle 0): pivot, end of the bent root, tip."""
    a = G.PRONG_RISE
    p0 = np.zeros(3)
    p1 = np.array([-G.PRONG_ROOT * math.cos(a), 0.0, G.PRONG_ROOT * math.sin(a)])
    p2 = np.array([-G.PRONG_L, 0.0, G.BAR_R + G.PRONG_R + 0.0003])
    return p0, p1, p2


# ------------------------------------------------------------------------------------------------- layouts
def _surface_height(p_yz):
    """Height of a point above the trousers' surface (radial, approximately)."""
    y, z = p_yz[0] - G.YC, p_yz[1] - G.ZC
    r = math.hypot(y, z)
    th = math.atan2(y, z)
    ys, zs = G.A1 * math.sin(th), G.B1 * math.cos(th)
    return r - math.hypot(ys, zs)


def layout(kind: str, idx: int = 0):
    """Body poses {name: (R, p)} for the start layout ``kind`` ('threaded', index ``idx``; 'fastened' is a probe
    layout with the belt fastened at hole 3)."""
    if kind == "threaded":
        joints, _bow = G.threaded_layout(idx)
        under = G.T + 0.0015 + 0.0005   # the front bar rests on the tongue
    else:
        joints, _g = G.fastened_layout(hole_shift=HOLE_SHIFT)
        under = G.T + _g + 0.0025
        under = G.T + _g + 0.0025   # ... on the tongue (raised where the tongue ramps up)
    links = G.link_poses_from_joints(joints)
    out = {f"link_{i:02d}": (R, p) for i, (R, p) in enumerate(links)}
    R0, p0 = links[0]
    bar = p0 + R0 @ np.array([-H, 0.0, G.BAR_Z])
    # frame angle: front bar resting (bar radius + 0.3 mm) on what is under it
    best = None
    for k in range(-300, 301):
        phi = math.radians(k * 0.1)
        Rf = R0 @ _rot_y(phi)
        fb = bar + Rf @ np.array([-G.FRAME_L, 0.0, 0.0])
        h = _surface_height(fb[1:]) - under - G.BAR_R - 0.0003
        if kind == "threaded":      # round 2: the front bar rests on the tongue link under it
            h = None
            for i in range(3, G.N_LINKS):
                Rl, pl = out[f"link_{i:02d}"]
                l = Rl.T @ (fb - pl)
                if abs(l[0]) <= H and l[2] > -0.002:
                    hh = l[2] - G.T / 2 - G.BAR_R - 0.0003
                    h = hh if h is None else min(h, hh)
            if h is None:
                continue
        if best is None or abs(h) < abs(best[1]):
            best = (phi, h)
    phi = best[0]
    Rf = R0 @ _rot_y(phi)
    out["frame"] = (Rf, bar)
    out["prong"] = (Rf, bar.copy())
    if kind == "threaded":
        # the prong turned back over the fold, lying on link 0's top (under the tongue)
        a_, b_, c_ = prong_segments()
        best = None
        for k in range(1500, 2300):
            ang = math.radians(k * 0.1)
            Rp = R0 @ _rot_y(ang)
            pts = [bar + Rp @ v for v in (b_, c_)]
            # height of the main segment's end above link 0's top surface (link 0 frame)
            hgt = min(float((R0.T @ (q - p0))[2]) for q in pts[1:]) - G.FOLD_TOP - G.PRONG_R
            R1_, p1_ = out["link_01"]          # round 2: also clear link 1's top (the fold curves round the waist)
            for t_ in np.linspace(0.3, 1.0, 15):
                q = pts[0] + t_ * (pts[1] - pts[0])
                l1 = R1_.T @ (q - p1_)
                if abs(l1[0]) <= H:
                    hgt = min(hgt, float(l1[2]) - G.FOLD_TOP - G.PRONG_R)
            if best is None or abs(hgt - 0.0004) < abs(best[1] - 0.0004):
                best = (ang, hgt)
        out["prong"] = (R0 @ _rot_y(best[0]), bar.copy())
    return out, math.degrees(phi)


# --------------------------------------------------------------------------------------------------- spec
@dataclass(frozen=True)
class Belt(Box):
    """The belt (an articulation). ``pos`` is only a nominal centre for the harness's checks."""
    _is_beltkit = True
    layout_kind: str = "open"

    @property
    def free(self):
        return False

    def body_names(self):
        return [f"link_{i:02d}" for i in range(G.N_LINKS)] + ["frame", "prong"]


@dataclass(frozen=True)
class BeltState(ObjectState):
    poses: dict = field(default_factory=dict)       # body -> (p (3,), R (3,3)), env frame
    speeds: dict = field(default_factory=dict)

    def R(self, b):
        return self.poses[b][1]

    def p(self, b):
        return self.poses[b][0]


_SPECS: dict = {}
_CFG = None
_SPAWN = None


def _make_object(spec):
    global _CFG, _SPAWN
    import isaaclab.sim as sim
    from isaaclab.managers import EventTermCfg, SceneEntityCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    if _CFG is None:
        @configclass
        class BeltSpawnCfg(sim.SpawnerCfg):
            kit_name: str = ""
        _CFG = BeltSpawnCfg
        _SPAWN = clone(_spawn)

    class BeltObject(Object):
        def _build_reset_event(self):
            return EventTermCfg(func=_reset, mode="reset",
                                params={"asset_cfg": SceneEntityCfg(self.name)})

    _SPECS[spec.name] = spec
    return BeltObject(name=spec.name, object_type=ObjectType.ARTICULATION,
                      spawner_cfg=_CFG(func=_SPAWN, kit_name=spec.name),
                      initial_pose=Pose(position_xyz=(0.0, 0.0, 0.0), rotation_xyzw=(0.0, 0.0, 0.0, 1.0)))


def start_index() -> int:
    """Which start layout this reset uses: BELT_START in the environment, else the harness's reset seed modulo the
    number of layouts, else 0."""
    n = len(G.START_LAYOUTS)
    v = os.environ.get("BELT_START")
    if v not in (None, ""):
        return int(v) % n
    seed = getattr(partkit, "RESET_SEED", None)
    return int(seed) % n if seed is not None else 0


def _angle_y(Rrel):
    return math.atan2(Rrel[0, 2], Rrel[0, 0])


def _reset(env, env_ids, asset_cfg):
    """Root body and joints to the chosen start layout (joint angles are deltas against the authored base layout,
    whose zero angles are the layout authored at spawn), velocities 0, spring targets set."""
    import torch
    spec = _SPECS[asset_cfg.name]
    art = env.scene[asset_cfg.name]
    if env_ids is None:
        env_ids = torch.arange(env.num_envs, device=env.device)
    idx = start_index() if spec.layout_kind == "threaded" else 0
    base_poses, _ = layout(spec.layout_kind, 0)
    poses, _ = layout(spec.layout_kind, idx)
    root = list(art.data.body_names)[0]
    R, p = poses[root]
    q = _quat(R)
    origin = env.scene.env_origins[env_ids]
    rp = torch.zeros((len(env_ids), 7), dtype=origin.dtype, device=origin.device)
    rp[:, :3] = origin + torch.tensor(p, dtype=origin.dtype, device=origin.device)
    rp[:, 3:7] = torch.tensor(q, dtype=origin.dtype, device=origin.device)
    art.write_root_pose_to_sim_index(root_pose=rp, env_ids=env_ids)
    art.write_root_velocity_to_sim_index(root_velocity=torch.zeros((len(env_ids), 6), dtype=origin.dtype,
                                                                    device=origin.device), env_ids=env_ids)
    names = list(art.joint_names)
    qj = torch.zeros_like(art.data.default_joint_pos.torch[env_ids])
    tgt = torch.zeros_like(qj)
    bends0, bends = bend_angles(base_poses), bend_angles(poses)
    ring_end = ring_end_link(poses)
    for i in range(1, G.N_LINKS):
        n = f"bj_{i:02d}:1"
        if n not in names:
            continue
        dq = math.radians(bends[i - 1] - bends0[i - 1])
        qj[:, names.index(n)] = dq
        # spring targets: the part round the waist keeps the waist's curve (a worn belt has taken it); the free end
        # past the front-right loop rests straight
        tgt[:, names.index(n)] = dq if i <= ring_end else -math.radians(bends0[i - 1])
    R00, Rf0, Rp0 = base_poses["link_00"][0], base_poses["frame"][0], base_poses["prong"][0]
    R0, Rf, Rp = poses["link_00"][0], poses["frame"][0], poses["prong"][0]
    qf, qp = _angle_y(R0.T @ Rf @ Rf0.T @ R00), _angle_y(R0.T @ Rp @ Rp0.T @ R00)
    if spec.layout_kind == "threaded":
        qp = min(qp, math.radians(PRONG_BACK_STOP))
    qj[:, names.index("frame_hinge")] = qf
    qj[:, names.index("prong_hinge")] = qp
    art.write_joint_position_to_sim_index(position=qj, env_ids=env_ids)
    art.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(qj), env_ids=env_ids)
    art.set_joint_position_target_index(target=tgt, env_ids=env_ids)
    dq_max = max(abs(bends[i] - bends0[i]) for i in range(G.N_LINKS - 1))
    print(f"BELT_RESET layout {idx} (seed {getattr(partkit, 'RESET_SEED', None)}) root={root} joints={len(names)} "
          f"max joint delta {dq_max:.1f} deg frame {math.degrees(qf):.1f} prong {math.degrees(qp):.2f}", flush=True)


# ------------------------------------------------------------------------------------------------ USD authoring
def _xf(stage, path, p, R):
    from pxr import Gf, UsdGeom
    x = UsdGeom.Xform.Define(stage, path)
    x.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*map(float, p)))
    qx, qy, qz, qw = _quat(R)
    x.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(float(qw), Gf.Vec3d(float(qx), float(qy), float(qz))))
    return x


def _body(stage, path, p, R, mass, com, inertia):
    from pxr import Gf, PhysxSchema, UsdPhysics
    prim = _xf(stage, path, p, R).GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(prim)
    m = UsdPhysics.MassAPI.Apply(prim)
    m.CreateMassAttr().Set(float(mass))
    m.CreateCenterOfMassAttr().Set(Gf.Vec3f(*map(float, com)))
    m.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(*map(float, inertia)))
    rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    rb.CreateMaxDepenetrationVelocityAttr().Set(0.5)
    rb.CreateLinearDampingAttr().Set(0.3)
    rb.CreateAngularDampingAttr().Set(1.0)
    rb.CreateSolverPositionIterationCountAttr().Set(48)
    rb.CreateSolverVelocityIterationCountAttr().Set(8)
    return prim


def _box_inertia(m, boxes):
    lo = np.min([np.array(c) - np.array(s) / 2 for c, s in boxes], 0)
    hi = np.max([np.array(c) + np.array(s) / 2 for c, s in boxes], 0)
    a, b, c = hi - lo
    vols = [float(np.prod(s)) for _, s in boxes]
    com = np.average([c_ for c_, _ in boxes], axis=0, weights=vols)
    return com, (m * (b * b + c * c) / 12, m * (a * a + c * c) / 12, m * (a * a + b * b) / 12)


def _geom(stage, path, kind, c, dims, mat, look, collide=True, R_local=None):
    import isaaclab.sim as sim
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics
    if kind == "box":
        g = UsdGeom.Cube.Define(stage, path)
        g.CreateSizeAttr(1.0)
        g.AddTranslateOp().Set(Gf.Vec3d(*map(float, c)))
        g.AddScaleOp().Set(Gf.Vec3f(*map(float, dims)))
    elif kind == "cyl":                       # dims = (radius, length), along y
        g = UsdGeom.Cylinder.Define(stage, path)
        g.CreateRadiusAttr(float(dims[0]))
        g.CreateHeightAttr(float(dims[1]))
        g.CreateAxisAttr("Y")
        g.AddTranslateOp().Set(Gf.Vec3d(*map(float, c)))
    else:                                     # capsule from a to b, dims = (radius, a, b)
        r, a, b = dims
        a, b = np.asarray(a, float), np.asarray(b, float)
        d = b - a
        L = float(np.linalg.norm(d))
        g = UsdGeom.Capsule.Define(stage, path)
        g.CreateAxisAttr("X")
        g.CreateRadiusAttr(float(r))
        g.CreateHeightAttr(max(L, 1e-5))
        g.AddTranslateOp().Set(Gf.Vec3d(*map(float, (a + b) / 2)))
        x = d / max(L, 1e-9)
        ref = np.array((0., 0., 1.)) if abs(x[2]) < .9 else np.array((0., 1., 0.))
        y = np.cross(ref, x)
        y /= np.linalg.norm(y)
        z = np.cross(x, y)
        qx, qy, qz, qw = _quat(np.c_[x, y, z])
        g.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(float(qw), Gf.Vec3d(float(qx), float(qy), float(qz))))
    prim = g.GetPrim()
    if collide:
        UsdPhysics.CollisionAPI.Apply(prim)
        col = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        col.CreateContactOffsetAttr().Set(0.001)
        col.CreateRestOffsetAttr().Set(0.0)
        sim.bind_physics_material(path, mat, stage=stage)
    sim.bind_visual_material(path, look, stage=stage)


def _materials(stage, root):
    import isaaclab.sim as sim
    from yam_arena.station import linear_rgb
    mats = {}
    for name, (fs, fd) in {"leather": (0.6, 0.5), "metal": (0.4, 0.35)}.items():
        p = f"{root}/mat_{name}"
        m = sim.RigidBodyMaterialCfg(static_friction=fs, dynamic_friction=fd, restitution=0.0)
        m.func(p, m)
        mats[name] = p
    looks = {}
    for name, col in {"leather": LEATHER, "metal": METAL}.items():
        p = f"{root}/look_{name}"
        lk = sim.PreviewSurfaceCfg(diffuse_color=linear_rgb(col), roughness=0.55 if name == "leather" else 0.3,
                                   metallic=0.0 if name == "leather" else 0.6)
        lk.func(p, lk)
        looks[name] = p
    return mats, looks


def _limit(prim, axis, lo, hi):
    from pxr import UsdPhysics
    lim = UsdPhysics.LimitAPI.Apply(prim, axis)
    lim.CreateLowAttr().Set(float(lo))
    lim.CreateHighAttr().Set(float(hi))


def _drive(prim, axis, k_per_rad, d_per_rad, target_deg=0.0):
    from pxr import UsdPhysics
    d = UsdPhysics.DriveAPI.Apply(prim, axis)
    d.CreateTypeAttr().Set("force")
    d.CreateStiffnessAttr().Set(float(k_per_rad) * math.pi / 180)
    d.CreateDampingAttr().Set(float(d_per_rad) * math.pi / 180)
    d.CreateTargetPositionAttr().Set(float(target_deg))
    d.CreateTargetVelocityAttr().Set(0.0)
    d.CreateMaxForceAttr().Set(1.0e6)


def _friction(prim, effort):
    """Coulomb friction (static and dynamic effort, N m) on a revolute joint's axis."""
    from pxr import Sdf
    prim.ApplyAPI("PhysxJointAxisAPI", "angular")
    for eff, v in (("staticFrictionEffort", effort), ("dynamicFrictionEffort", 0.8 * effort)):
        name = f"physxJointAxis:angular:{eff}"
        a = prim.GetAttribute(name) or prim.CreateAttribute(name, Sdf.ValueTypeNames.Float)
        a.Set(float(v))


def _frames(j, b0, b1, p0, R0, p1, R1):
    from pxr import Gf
    j.CreateBody0Rel().SetTargets([b0])
    j.CreateBody1Rel().SetTargets([b1])
    j.CreateLocalPos0Attr().Set(Gf.Vec3f(*map(float, p0)))
    j.CreateLocalPos1Attr().Set(Gf.Vec3f(*map(float, p1)))
    for attr, R in ((j.CreateLocalRot0Attr(), R0), (j.CreateLocalRot1Attr(), R1)):
        qx, qy, qz, qw = _quat(R)
        attr.Set(Gf.Quatf(float(qw), Gf.Vec3f(float(qx), float(qy), float(qz))))
    j.CreateCollisionEnabledAttr().Set(False)


def ring_end_link(poses):
    """Index of the last link that lies round the waist (the one through the front-right loop, plus one)."""
    pl, _ = G.ellipse_pt(G.LOOPS["front_right"])
    d = [float(np.linalg.norm(poses[f"link_{i:02d}"][1][1:] - pl)) for i in range(G.N_LINKS)]
    return int(np.argmin(d)) + 1


def bend_angles(poses):
    """Layout bend of each belt joint about the link y axis (deg): R_rel = R_{i-1}^T R_i = Ry(a)."""
    out = []
    for i in range(1, G.N_LINKS):
        Rr = poses[f"link_{i - 1:02d}"][0].T @ poses[f"link_{i:02d}"][0]
        out.append(math.degrees(math.atan2(Rr[0, 2], Rr[0, 0])))
    return out


def _spawn(prim_path, cfg, *args, **kwargs):
    import isaaclab.sim as sim
    from pxr import PhysxSchema, UsdGeom, UsdPhysics
    spec = _SPECS[cfg.kit_name]
    stage = kwargs.get("stage") or sim.get_current_stage()
    root = UsdGeom.Xform.Define(stage, prim_path)
    UsdPhysics.ArticulationRootAPI.Apply(root.GetPrim())
    art = PhysxSchema.PhysxArticulationAPI.Apply(root.GetPrim())
    art.CreateEnabledSelfCollisionsAttr().Set(True)
    art.CreateSolverPositionIterationCountAttr().Set(48)
    art.CreateSolverVelocityIterationCountAttr().Set(8)
    UsdGeom.Xform.Define(stage, f"{prim_path}/joints")
    mats, looks = _materials(stage, prim_path)
    poses, phi0 = layout(spec.layout_kind, 0)          # the authored base layout (zero joint angles)
    I3 = np.eye(3)
    for i in range(G.N_LINKS):
        name = f"link_{i:02d}"
        R, p = poses[name]
        boxes = link_boxes(i)
        m = G.LINK_MASS * (2 if i <= G.KEEPER_LINK else 1)
        com, inertia = _box_inertia(m, boxes)
        path = f"{prim_path}/{name}"
        _body(stage, path, p, R, m, com, inertia)
        for k, (c, s) in enumerate(boxes):
            _geom(stage, f"{path}/b{k}", "box", c, s, mats["leather"], looks["leather"])
        for k, (c, s) in enumerate(link_visual_extra(i)):
            _geom(stage, f"{path}/v{k}", "box", c, s, mats["leather"], looks["leather"], collide=False)
    # frame
    Rf, pf = poses["frame"]
    cyl, box = frame_shapes()
    fpath = f"{prim_path}/frame"
    allb = [(c, (2 * r, l, 2 * r)) for c, r, l in cyl] + box
    com, inertia = _box_inertia(G.FRAME_MASS, allb)
    _body(stage, fpath, pf, Rf, G.FRAME_MASS, com, inertia)
    for k, (c, r, l) in enumerate(cyl):      # the hinge bar (k = 0) sits inside the fold: drawn, no collider
        _geom(stage, f"{fpath}/c{k}", "cyl", c, (r, l), mats["metal"], looks["metal"], collide=k > 0)
    for k, (c, s) in enumerate(box):
        _geom(stage, f"{fpath}/b{k}", "box", c, s, mats["metal"], looks["metal"])
    # prong
    Rp, pp = poses["prong"]
    a, b, c = prong_segments()
    ppath = f"{prim_path}/prong"
    com, inertia = _box_inertia(G.PRONG_MASS, [(((a + c) / 2).tolist(), (G.PRONG_L, 2 * G.PRONG_R, 0.008))])
    _body(stage, ppath, pp, Rp, G.PRONG_MASS, com, inertia)
    _geom(stage, f"{ppath}/s0", "cap", None, (G.PRONG_R, a, b), mats["metal"], looks["metal"])
    _geom(stage, f"{ppath}/s1", "cap", None, (G.PRONG_R, b, c), mats["metal"], looks["metal"])
    # belt joints
    bends = bend_angles(poses)
    for i in range(1, G.N_LINKS):
        R0, _ = poses[f"link_{i - 1:02d}"]
        R1, _ = poses[f"link_{i:02d}"]
        j = UsdPhysics.Joint.Define(stage, f"{prim_path}/joints/bj_{i:02d}")
        _frames(j, f"{prim_path}/link_{i - 1:02d}", f"{prim_path}/link_{i:02d}", (H, 0, 0), I3, (-H, 0, 0), R1.T @ R0)
        PhysxSchema.PhysxJointAPI.Apply(j.GetPrim()).CreateArmatureAttr().Set(ARMATURE)
        for ax in ("transX", "transY", "transZ"):
            _limit(j.GetPrim(), ax, 1.0, -1.0)
        a = bends[i - 1]
        for ax, lim, k, tgt in (("rotX", TWIST_LIM, TWIST_K, 0.0), ("rotY", FLAT_LIM, FLAT_K, -a),
                                ("rotZ", EDGE_LIM, EDGE_K, 0.0)):
            _limit(j.GetPrim(), ax, tgt - lim, tgt + lim)
            _drive(j.GetPrim(), ax, k, DAMP_S * k, tgt)
    # frame hinge on link 0, prong hinge on the frame (same bar)
    R0, _ = poses["link_00"]
    jf = UsdPhysics.RevoluteJoint.Define(stage, f"{prim_path}/joints/frame_hinge")
    _frames(jf, f"{prim_path}/link_00", fpath, (-H, 0.0, G.BAR_Z), I3, (0, 0, 0), Rf.T @ R0)
    jf.CreateAxisAttr().Set("Y")
    jf.CreateLowerLimitAttr().Set(FRAME_LIM[0] - phi0)
    jf.CreateUpperLimitAttr().Set(FRAME_LIM[1] - phi0)
    _drive(jf.GetPrim(), "angular", 0.0, HINGE_DAMP)
    PhysxSchema.PhysxJointAPI.Apply(jf.GetPrim()).CreateArmatureAttr().Set(2e-6)
    _friction(jf.GetPrim(), FRAME_FRICTION)
    # the prong hangs on link 0 (same bar axis), so that it collides with the frame's front and side bars
    jp = UsdPhysics.RevoluteJoint.Define(stage, f"{prim_path}/joints/prong_hinge")
    _frames(jp, f"{prim_path}/link_00", ppath, (-H, 0.0, G.BAR_Z), I3, (0, 0, 0), Rp.T @ R0)
    jp.CreateAxisAttr().Set("Y")
    if spec.layout_kind == "threaded":
        # round 2: the prong is authored turned back, lying on the fold. The fold (link 0, the prong's own hinge
        # body, so no contact between them) stops it there: it may not turn further back into the leather.
        Rrel = R0.T @ Rp
        p_abs = math.degrees(math.atan2(Rrel[0, 2], Rrel[0, 0])) % 360
        jp.CreateLowerLimitAttr().Set(PRONG_LIM[0] - p_abs)
        jp.CreateUpperLimitAttr().Set(PRONG_BACK_STOP)
        print(f"BELT_PRONG turned back at {p_abs:.1f} deg; stop +{PRONG_BACK_STOP} deg", flush=True)
    else:
        jp.CreateLowerLimitAttr().Set(PRONG_LIM[0] - phi0)
        jp.CreateUpperLimitAttr().Set(PRONG_LIM[1] - phi0)
    _drive(jp.GetPrim(), "angular", 0.0, HINGE_DAMP)
    PhysxSchema.PhysxJointAPI.Apply(jp.GetPrim()).CreateArmatureAttr().Set(2e-6)
    _friction(jp.GetPrim(), PRONG_FRICTION)
    print(f"BELT_SPAWN {spec.layout_kind} links={G.N_LINKS} frame_phi0={phi0:.1f}deg", flush=True)
    return root.GetPrim()


# ---------------------------------------------------------------------------------------------------- reading
def _arr(v):
    v = getattr(v, "torch", v)
    return v.detach().cpu().numpy().astype(float) if hasattr(v, "detach") else np.asarray(v, float)


def read(environment, spec):
    base = environment.unwrapped
    art = base.scene.articulations[spec.name]
    d = art.data
    poses = _arr(d.body_link_pose_w)[0]
    vel = _arr(d.body_link_vel_w)[0]
    origin = _arr(base.scene.env_origins)[0]
    bi = {n: i for i, n in enumerate(d.body_names)}
    out, speeds = {}, {}
    for n in spec.body_names():
        i = bi[n]
        q = poses[i, 3:]
        R = quat_to_matrix(q) if np.all(np.isfinite(q)) and np.linalg.norm(q) > 1e-8 else np.full((3, 3), np.nan)
        out[n] = (poses[i, :3] - origin, R)
        speeds[n] = float(np.linalg.norm(vel[i, :3]))
    pts = np.array([p for p, _ in out.values()])
    lo, hi = np.nanmin(pts, 0), np.nanmax(pts, 0)
    f = bi["frame"]
    return BeltState(pos=(lo + hi) / 2, quat_xyzw=np.array((0.0, 0.0, 0.0, 1.0)), lin_vel=vel[f, :3],
                     ang_vel=vel[f, 3:], low=lo, high=hi, poses=out, speeds=speeds)


def belt_state_from(poses_R_p):
    """BeltState from {body: (R, p)} (offline tests)."""
    out = {k: (np.asarray(p, float), np.asarray(R, float)) for k, (R, p) in poses_R_p.items()}
    pts = np.array([p for p, _ in out.values()])
    return BeltState(pos=pts.mean(0), quat_xyzw=np.array((0.0, 0.0, 0.0, 1.0)), lin_vel=np.zeros(3),
                     ang_vel=np.zeros(3), low=pts.min(0), high=pts.max(0), poses=out,
                     speeds={k: 0.0 for k in out})


def articulation(environment, name="belt"):
    return environment.unwrapped.scene.articulations[name]


partkit.EXTRA_KINDS.append((lambda s: getattr(s, "_is_beltkit", False), _make_object, read))
