"""Diaper change on a newborn care training doll (station table).

A 21 in (53 cm) vinyl newborn care doll lies face up on a changing mat, head toward the robot's left (+y), with a
used (wet) diaper whose tabs are already undone, folded in half under its bottom. A clean newborn diaper lies open,
inner side up, on the mat at the doll's feet, its back half (with the two hook tabs) toward the doll; the doll's
lower legs rest on it. A small open bin stands on the table at the robot's front left.

The robot must lift the doll's bottom by its ankles, pull the used diaper out and put it in the bin, slide the clean
diaper's back half under the bottom, lower the legs, bring the front up between the legs onto the belly, fasten both
tabs on the front's landing zone, and let go.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score --instruction "<INSTRUCTION>"
"""

import math
import os
import pathlib
import sys
from dataclasses import dataclass

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, ObjectState  # noqa: E402

import diaper as dp  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
INSTRUCTION = "Change the baby doll's wet diaper for the clean one, used diaper in the bin, then let go."

# -- layout (m) -----------------------------------------------------------------------------------------------------
MAT_T = 0.015
MT = TOP + MAT_T  # mat top
DX, Y0 = 0.27, 0.03  # doll body axis at x = DX; doll-local s = y - Y0 (0 at the crotch), head toward +s
MAT = Box("changing_mat", pos=(0.29, 0.01, TOP + MAT_T / 2), size=(0.34, 0.80, MAT_T), color=(0.16, 0.36, 0.48))

BIN_C = np.array([0.25, 0.50])
BIN_R, BIN_H, BIN_W = 0.09, 0.07, 0.006  # outer radius, height, wall
BIN_COL = (0.92, 0.92, 0.95)


def _bin_parts():
    parts = [Box("bin_floor", pos=(float(BIN_C[0]), float(BIN_C[1]), TOP + 0.003), size=(0.12, 0.12, 0.006),
                 color=BIN_COL)]
    n = 12
    for k in range(n):
        a = 2 * math.pi * k / n
        c = BIN_C + (BIN_R - BIN_W / 2) * np.array([math.cos(a), math.sin(a)])
        side = 2 * BIN_R * math.tan(math.pi / n) + 0.002
        parts.append(Box(f"bin_wall{k:02d}", pos=(float(c[0]), float(c[1]), TOP + BIN_H / 2),
                         size=(BIN_W, side, BIN_H), yaw=a, color=BIN_COL))
    return parts


# -- doll geometry (doll-local: x across, s along the body, z up from the mat top) ---------------------------------
SKIN = (0.93, 0.76, 0.64)
LUMBAR = np.array([0.0, 0.135, 0.005])  # pelvis pivots on the lower back
HIP = np.array([0.0, 0.03, 0.035])
ANKLE = np.array([0.0, -0.105, 0.017])
M_PELVIS, M_LEGS, M_FEET = 0.45, 0.30, 0.06  # kg (torso and head carry the rest of the 2.3 kg doll, fixed)
LUMBAR_LIM = (-0.45, 0.0)  # rad; negative raises the bottom
HIP_LIM = (-0.15, 0.05)  # stiff vinyl hips: legs flex up to 9 deg, then lifting the ankles rolls the pelvis up
ANKLE_LIM = (-1.2, 1.2)

# shapes: (body, kind, params, rgb) ; kinds: rbox(centre, half, r) / cap(a, b, r) / sph(c, r)
SHAPES = [
    ("torso", "sph", (np.array([0.0, 0.305, 0.056]), 0.056)),
    ("torso", "rbox", (np.array([0.0, 0.205, 0.045]), np.array([0.040, 0.060, 0.020]), 0.025)),
    ("torso", "cap", (np.array([0.075, 0.25, 0.020]), np.array([0.082, 0.15, 0.018]), 0.017)),
    ("torso", "cap", (np.array([-0.075, 0.25, 0.020]), np.array([-0.082, 0.15, 0.018]), 0.017)),
    ("pelvis", "rbox", (np.array([0.0, 0.078, 0.040]), np.array([0.030, 0.034, 0.015]), 0.025)),
    ("legs", "cap", (np.array([0.030, 0.035, 0.021]), np.array([0.034, -0.045, 0.021]), 0.021)),
    ("legs", "cap", (np.array([-0.030, 0.035, 0.021]), np.array([-0.034, -0.045, 0.021]), 0.021)),
    ("legs", "cap", (np.array([0.034, -0.045, 0.019]), np.array([0.024, -0.100, 0.017]), 0.017)),
    ("legs", "cap", (np.array([-0.034, -0.045, 0.019]), np.array([-0.024, -0.100, 0.017]), 0.017)),
    ("feet", "cap", (np.array([0.023, -0.108, 0.016]), np.array([0.023, -0.150, 0.026]), 0.015)),
    ("feet", "cap", (np.array([-0.023, -0.108, 0.016]), np.array([-0.023, -0.150, 0.026]), 0.015)),
]
FEET_GRIP = np.array([0.0, -0.13, 0.02])  # where a hand holds both ankles (doll-local, rest pose)
NAVEL = np.array([0.0, 0.115, 0.080])


def to_world(p):
    p = np.asarray(p, float)
    return p + np.array([DX, Y0, MT])


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def body_frames(q):
    """Doll-local rotation and translation of each body for joint angles q = (lumbar, hip, ankle)."""
    a1, a2, a3 = q
    rp, rl, rf = _rx(a1), _rx(a1 + a2), _rx(a1 + a2 + a3)
    tp = LUMBAR - rp @ LUMBAR
    hip_w = LUMBAR + rp @ (HIP - LUMBAR)
    tl = hip_w - rl @ HIP
    ank_w = hip_w + rl @ (ANKLE - HIP)
    tf = ank_w - rf @ ANKLE
    return {"torso": (np.eye(3), np.zeros(3)), "pelvis": (rp, tp), "legs": (rl, tl), "feet": (rf, tf)}


@dataclass(frozen=True)
class Doll(Box):
    @property
    def free(self) -> bool:
        return False


@dataclass(frozen=True)
class DollState(ObjectState):
    q: np.ndarray = None  # lumbar, hip, ankle (rad)
    body_pos: dict = None  # world positions of the body origins


DOLL = Doll("doll", pos=tuple(to_world([0, 0.1, 0.05])), size=(0.18, 0.5, 0.1), color=SKIN)

OBJECTS = [MAT, *_bin_parts(), DOLL]

TWIN_EXTRA = [
    {"name": "doll", "kind": "body", "real": "Nasco 21 in vinyl newborn care doll (SB26943)",
     "note": "torso stays on the mat; pelvis, legs and feet hinge"},
]


# -- doll articulation ------------------------------------------------------------------------------------------
def _register_module():
    if __name__ not in sys.modules:
        import types

        mod = types.ModuleType(__name__)
        mod.__dict__.update(globals())
        sys.modules[__name__] = mod


def _quat_from_y(d):
    """wxyz rotating +y onto unit d."""
    d = d / np.linalg.norm(d)
    y = np.array([0.0, 1.0, 0.0])
    c = float(y @ d)
    ax = np.cross(y, d)
    s = np.linalg.norm(ax)
    if s < 1e-9:
        return (1.0, 0.0, 0.0, 0.0) if c > 0 else (0.0, 1.0, 0.0, 0.0)
    ax /= s
    ang = math.atan2(s, c)
    return (math.cos(ang / 2), *(ax * math.sin(ang / 2)))


def _spawn_doll(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics
    from yam_arena.station import linear_rgb

    stage = get_current_stage()
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    root.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(1, 0, 0, 0))
    vis = f"{prim_path}/Looks/skin"
    look = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(SKIN), roughness=0.55)
    look.func(vis, look)
    phys = f"{prim_path}/Looks/vinyl"
    pm = sim_utils.RigidBodyMaterialCfg(static_friction=0.9, dynamic_friction=0.8)
    pm.func(phys, pm)
    # the spawn translation is the doll origin (mat top under the crotch line)
    masses = {"torso": 1.5, "pelvis": M_PELVIS, "legs": M_LEGS, "feet": M_FEET}
    for b in ("torso", "pelvis", "legs", "feet"):
        xf = UsdGeom.Xform.Define(stage, f"{prim_path}/{b}")
        xf.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(0, 0, 0))
        xf.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(1, 0, 0, 0))
        prim = xf.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(masses[b])
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateMaxDepenetrationVelocityAttr(0.3)
        rb.CreateLinearDampingAttr(0.2)
        rb.CreateAngularDampingAttr(1.0)
    for k, (b, kind, prm) in enumerate(SHAPES):
        path = f"{prim_path}/{b}/g{k:02d}"
        if kind == "sph":
            c, r = prm
            g = UsdGeom.Sphere.Define(stage, path)
            g.CreateRadiusAttr(float(r))
            g.AddTranslateOp().Set(Gf.Vec3f(*map(float, c)))
        elif kind == "cap":
            a, bb, r = prm
            g = UsdGeom.Capsule.Define(stage, path)
            g.CreateAxisAttr("Y")
            g.CreateRadiusAttr(float(r))
            g.CreateHeightAttr(float(np.linalg.norm(bb - a)))
            g.AddTranslateOp().Set(Gf.Vec3f(*map(float, (a + bb) / 2)))
            g.AddOrientOp().Set(Gf.Quatf(*map(float, _quat_from_y(bb - a))))
        else:
            c, half, r = prm
            g = UsdGeom.Cube.Define(stage, path)
            g.CreateSizeAttr(1.0)
            g.AddTranslateOp().Set(Gf.Vec3f(*map(float, c)))
            g.AddScaleOp().Set(Gf.Vec3f(*map(float, 2 * (half + r))))
        p = g.GetPrim()
        UsdPhysics.CollisionAPI.Apply(p)
        cp = PhysxSchema.PhysxCollisionAPI.Apply(p)
        cp.CreateContactOffsetAttr(0.002)
        cp.CreateRestOffsetAttr(0.0)
        bind_physics_material(str(p.GetPath()), phys, stage=stage)
        bind_visual_material(str(p.GetPath()), vis, stage=stage)
    fj = UsdPhysics.FixedJoint.Define(stage, f"{prim_path}/torso/to_world")
    fj.CreateBody1Rel().SetTargets([Sdf.Path(f"{prim_path}/torso")])
    w = np.asarray(translation or (0, 0, 0), float)
    fj.CreateLocalPos0Attr(Gf.Vec3f(*map(float, w)))
    fj.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
    fj.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    fj.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    for name, parent, child, at, lim in (("lumbar", "torso", "pelvis", LUMBAR, LUMBAR_LIM),
                                         ("hip", "pelvis", "legs", HIP, HIP_LIM),
                                         ("ankle", "legs", "feet", ANKLE, ANKLE_LIM)):
        j = UsdPhysics.RevoluteJoint.Define(stage, f"{prim_path}/{child}/{name}")
        j.CreateBody0Rel().SetTargets([Sdf.Path(f"{prim_path}/{parent}")])
        j.CreateBody1Rel().SetTargets([Sdf.Path(f"{prim_path}/{child}")])
        j.CreateAxisAttr("X")
        j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, at)))
        j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLocalPos1Attr(Gf.Vec3f(*map(float, at)))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLowerLimitAttr(math.degrees(lim[0]))
        j.CreateUpperLimitAttr(math.degrees(lim[1]))
    rp = root.GetPrim()
    UsdPhysics.ArticulationRootAPI.Apply(rp)
    art = PhysxSchema.PhysxArticulationAPI.Apply(rp)
    art.CreateEnabledSelfCollisionsAttr(False)
    art.CreateSolverPositionIterationCountAttr(32)
    art.CreateSolverVelocityIterationCountAttr(4)
    return rp


def _doll_object(spec):
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    _register_module()

    @configclass
    class DollSpawnerCfg(SpawnerCfg):
        func: object = clone(_spawn_doll)

    class DollObject(Object):
        def _generate_articulation_cfg(self):
            cfg = ArticulationCfg(
                prim_path=self.prim_path, spawn=self.spawner_cfg,
                actuators={
                    "lumbar": ImplicitActuatorCfg(joint_names_expr=["lumbar"], stiffness=0.0, damping=0.05,
                                                  friction=0.02),
                    "hip": ImplicitActuatorCfg(joint_names_expr=["hip"], stiffness=0.0, damping=0.03, friction=0.03),
                    "ankle": ImplicitActuatorCfg(joint_names_expr=["ankle"], stiffness=0.0, damping=0.005,
                                                 friction=0.002),
                })
            return self._add_initial_pose_to_cfg(cfg)

    return DollObject(name=spec.name, object_type=ObjectType.ARTICULATION, spawner_cfg=DollSpawnerCfg(),
                      initial_pose=Pose(position_xyz=tuple(map(float, to_world([0, 0, 0.0005]))),
                                        rotation_xyzw=(0.0, 0.0, 0.0, 1.0)))


def _doll_state(env) -> DollState:
    base = env.unwrapped
    art = base.scene[DOLL.name]
    origin = base.scene.env_origins[0]
    t = lambda x: getattr(x, "torch", x)  # noqa: E731
    to_np = lambda a: a.detach().cpu().numpy().astype(np.float64)  # noqa: E731
    jn = list(art.joint_names)
    jp = to_np(t(art.data.joint_pos)[0])
    q = np.array([jp[jn.index(n)] for n in ("lumbar", "hip", "ankle")])
    bp = to_np(t(art.data.body_pos_w)[0] - origin)
    pos = {n: bp[i] for i, n in enumerate(art.body_names)}
    pts = np.array(list(pos.values()))
    return DollState(pos=pos.get("pelvis", pts.mean(0)), quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=np.zeros(3),
                     ang_vel=np.zeros(3), low=pts.min(0) - 0.05, high=pts.max(0) + 0.05, q=q, body_pos=pos)


def _patch_scene(scene) -> None:
    if getattr(scene, "_doll_patched", False):
        return
    orig_task_object, orig_states = scene.task_object, scene.object_states

    def task_object(spec):
        if isinstance(spec, Doll):
            return _doll_object(spec)
        return orig_task_object(spec)

    def object_states(env, names):
        plain = [n for n in names if n != DOLL.name]
        out = orig_states(env, plain) if plain else {}
        if DOLL.name in names:
            out[DOLL.name] = _doll_state(env)
        return {n: out[n] for n in names}

    scene.task_object = task_object
    scene.object_states = object_states
    scene._doll_patched = True


# -- collision world for the sheets ------------------------------------------------------------------------------
def _sd_rbox(p, c, half, r):
    d = p - c
    q = np.abs(d) - half
    out = np.maximum(q, 0.0)
    lo = np.linalg.norm(out, axis=1)
    inside = np.max(q, axis=1)
    sd = np.where(lo > 0, lo, inside) - r
    n = np.where(lo[:, None] > 0, np.sign(d) * out / np.maximum(lo, 1e-12)[:, None], 0.0)
    ax = np.argmax(q, axis=1)
    m = lo <= 0
    if m.any():
        nn = np.zeros((m.sum(), 3))
        nn[np.arange(m.sum()), ax[m]] = np.sign(d[m, ax[m]])
        n[m] = nn
    return sd, n


def _sd_cap(p, a, b, r):
    ab = b - a
    t = np.clip(((p - a) @ ab) / max(ab @ ab, 1e-12), 0, 1)
    c = a + t[:, None] * ab
    d = p - c
    ln = np.linalg.norm(d, axis=1)
    return ln - r, d / np.maximum(ln, 1e-12)[:, None]


class World:
    """Analytic colliders for the sheets: mat, table, doll shapes (posed from its joint angles), bin, finger pads."""

    def __init__(self):
        self.q = np.zeros(3)
        self.fingers = []  # list of (a, b, r) capsules, world
        self._pose()

    def _pose(self):
        fr = body_frames(self.q)
        self.parts = []
        for b, kind, prm in SHAPES:
            R, tr = fr[b]
            off = np.array([DX, Y0, MT])
            if kind == "sph":
                self.parts.append((b, kind, (R @ prm[0] + tr + off, prm[1]), None))
            elif kind == "cap":
                self.parts.append((b, kind, (R @ prm[0] + tr + off, R @ prm[1] + tr + off, prm[2]), None))
            else:
                self.parts.append((b, kind, (R @ prm[0] + tr + off, prm[1], prm[2]), R))
        self._caps_cache = self._caps()

    def set_state(self, q, fingers):
        self.q = np.asarray(q, float)
        self.fingers = fingers
        self._pose()

    def doll_sd(self, p):
        return _doll_sd(self, p)

    def floor(self, x):
        on_mat = (np.abs(x[:, 0] - MAT.pos[0]) < MAT.size[0] / 2) & (np.abs(x[:, 1] - MAT.pos[1]) < MAT.size[1] / 2)
        return np.where(on_mat, MT, TOP)

    def _caps(self):
        A, B, Rr, trap = [], [], [], []
        for b, kind, prm, R in self.parts:
            if kind == "sph":
                A.append(prm[0]); B.append(prm[0]); Rr.append(prm[1])
            elif kind == "cap":
                A.append(prm[0]); B.append(prm[1]); Rr.append(prm[2])
            else:
                continue
            trap.append(b in ("pelvis", "torso"))
        return np.array(A), np.array(B), np.array(Rr), np.array(trap)

    def collide(self, x, prev, thick, pin_ids):
        h = thick / 2
        free = np.ones(len(x), bool)
        free[pin_ids] = False
        contact = np.zeros(len(x), bool)
        locked = np.zeros(len(x), bool)
        floor = self.floor(x)
        fl = floor + h
        low = x[:, 2] < fl
        x[low & free, 2] = fl[low & free]
        contact |= low
        on_floor = x[:, 2] <= floor + thick + 0.002
        # capsules and spheres, all at once: each node is pushed out of its deepest one
        A, B, Rr, trap = self._caps_cache
        ab = B - A
        ap = x[:, None, :] - A[None]
        t = np.clip((ap * ab[None]).sum(2) / np.maximum((ab * ab).sum(1), 1e-12)[None], 0, 1)
        d = ap - t[..., None] * ab[None]
        ln = np.linalg.norm(d, axis=2)
        sd = ln - Rr[None]
        k = np.argmin(sd, axis=1)
        idx = np.arange(len(x))
        sdk = sd[idx, k]
        hit = (sdk < h) & free
        if hit.any():
            under = hit & on_floor
            locked |= under & trap[k]
            proj = hit & ~under
            nk = d[idx, k] / np.maximum(ln[idx, k], 1e-12)[:, None]
            x[proj] += nk[proj] * (h - sdk[proj])[:, None]
            contact |= hit
        for b, kind, prm, R in self.parts:
            if kind != "rbox":
                continue
            pl = (x - prm[0]) @ R
            near = np.all(np.abs(pl) < prm[1] + prm[2] + h, axis=1) & free
            if not near.any():
                continue
            sdb, nl = _sd_rbox(pl[near], np.zeros(3), prm[1], prm[2])
            ids = np.flatnonzero(near)
            hb = sdb < h
            ids, sdb, nl = ids[hb], sdb[hb], nl[hb]
            if not len(ids):
                continue
            und = on_floor[ids]
            if b in ("pelvis", "torso"):
                locked[ids[und]] = True
            pr = ~und
            x[ids[pr]] += (nl[pr] @ R.T) * (h - sdb[pr])[:, None]
            contact[ids] = True
        if locked.any():
            x[locked, :2] = prev[locked, :2]
        # bin: open cylinder shell + floor
        d2 = x[:, :2] - BIN_C
        rho = np.linalg.norm(d2, axis=1)
        if (rho < BIN_R + 0.02).any():
            below_rim = x[:, 2] < TOP + BIN_H
            inner, outer = BIN_R - BIN_W - h, BIN_R + h
            band = below_rim & (rho > inner) & (rho < outer) & free
            if band.any():
                to_in = (rho - inner) < (outer - rho)
                tgt = np.where(to_in, inner, outer)
                x[band, :2] = BIN_C + d2[band] * (tgt[band] / np.maximum(rho[band], 1e-9))[:, None]
                contact |= band
            inb = (rho < BIN_R) & (x[:, 2] < TOP + 0.006 + h)
            x[inb & free, 2] = TOP + 0.006 + h
        for a, bb, r in self.fingers:
            sdf, n = _sd_cap(x, a, bb, r)
            hf = (sdf < h) & free
            x[hf] += n[hf] * (h - sdf[hf])[:, None]
        return x, contact, locked


# -- episode: sheets stepped from measured gripper state ---------------------------------------------------------
def _place_clean(uv):
    u, v = uv
    return np.array([DX + v, Y0 - u, MT + 0.002])


def _place_wet(uv):
    u, v = uv
    return np.array([DX + v, Y0 + 0.10 - u, MT + 0.004])


class Episode:
    def __init__(self):
        self.world = World()
        self.clean = dp.clean_diaper(_place_clean)
        self.wet = dp.wet_bundle(_place_wet)
        self.sheets = [self.wet, self.clean]
        self.step_n = 0
        self.tcp = np.zeros((2, 3)) + [[0.32, 0.3, 1.0], [0.32, -0.3, 1.0]]
        self.rot = np.tile(np.eye(3), (2, 1, 1))
        self.aperture = np.array([0.09, 0.09])
        self.meshes = {}
        self.ok = True
        self.done_since = None  # control step at which the full end state was first met


EP = Episode()
_ENV: dict = {"env": None}


def _robot_state(env):
    from yam_arena.runtime import link_poses

    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
    tcp, rot, ap = [], [], []
    from yam_arena.constants import TCP_OFFSET

    for k in ("left_robot", "right_robot"):
        r = base.scene[k]
        g = link_poses(r)["gripper"]
        tcp.append(g[:3, :3] @ np.array(TCP_OFFSET) + g[:3, 3] - origin)
        rot.append(g[:3, :3])
        jn = list(r.joint_names)
        jp = r.data.joint_pos.torch[0].cpu().numpy()
        ap.append(abs(float(jp[jn.index("joint7")])) * 2)
    return np.array(tcp), np.array(rot), np.array(ap)


def _fingers(tcp, rot, ap):
    caps = []
    for k in range(2):
        R = rot[k]
        for s in (-1, 1):
            c = tcp[k] + R[:, 0] * s * (ap[k] / 2 + 0.006)
            caps.append((c - R[:, 2] * 0.045, c.copy(), 0.007))
    return caps


def _step_sheets(env):
    tcp, rot, ap = _robot_state(env)
    EP.tcp, EP.rot, EP.aperture = tcp, rot, ap
    ds = _doll_state(env)
    q = ds.q
    EP.world.set_state(q, _fingers(tcp, rot, ap))
    if os.environ.get("DIAPER_DEBUG") and EP.step_n % 20 == 0:
        fk = {b: np.round(to_world(t), 3).tolist() for b, (R, t) in body_frames(q).items()}
        print(f"[doll] fk={fk} sim={ {b: np.round(p, 3).tolist() for b, p in ds.body_pos.items()} }", flush=True)
    for sh in EP.sheets:  # the used diaper first: a jaw closing where both overlap takes the upper layer only
        sh.blocked = {a for o in EP.sheets if o is not sh for a in o.pins}
        sh.step(tcp, rot, ap, EP.world, substeps=12)
    EP.step_n += 1
    if os.environ.get("DIAPER_DEBUG") and EP.step_n % 100 == 0:
        print(f"[eval] step {EP.step_n} {evaluate({DOLL.name: ds})}", flush=True)
        if os.environ.get("DIAPER_DUMP"):
            os.makedirs(os.environ["DIAPER_DUMP"], exist_ok=True)
            np.savez(f"{os.environ['DIAPER_DUMP']}/s{EP.step_n:05d}.npz", wet=EP.wet.x, clean=EP.clean.x, q=q,
                     tcp=tcp, ap=ap, stuck=np.array(list(EP.clean.stuck), int))
    if os.environ.get("DIAPER_DEBUG") and EP.step_n % 5 == 0:
        c = EP.clean
        td = {int(n): round(float(c._closest_on_faces(c.x[n], c.lz_faces)[2]) * 1000, 1) for n in c.pinned_nodes() if c.tab[n]}
        if td:
            print(f"[tabd] step {EP.step_n} dist_mm={td} press={dict(c.press)}", flush=True)
    if os.environ.get("DIAPER_DEBUG") and EP.step_n % 20 == 0:
        c = EP.clean
        print(f"[diaper] step {EP.step_n} q={np.round(q, 3)} pins={ {a: [n for n, _ in l] for a, l in c.pins.items()} }"
              f" wetpins={ {a: [n for n, _ in l] for a, l in EP.wet.pins.items()} } stuck={list(c.stuck)}"
              f" strain={c.peak_strain:.2f}/{EP.wet.peak_strain:.2f} ap={np.round(ap, 3)} tcp={np.round(tcp, 3).tolist()}",
              flush=True)


def _render(env):
    try:
        import omni.usd
        from pxr import Gf, Sdf, UsdGeom, Vt
    except Exception:  # noqa: BLE001
        return
    stage = omni.usd.get_context().get_stage()
    origin = env.unwrapped.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
    for sh in EP.sheets:
        nrm = sh.normals()
        for side, sgn, col in (("in", -1.0, sh.color_in), ("out", 1.0, sh.color_out)):
            key = f"{sh.name}_{side}"
            m = EP.meshes.get(key)
            faces = sh.faces if side == "out" else sh.faces[:, ::-1]
            if m is None or not m.GetPrim().IsValid():
                m = UsdGeom.Mesh.Define(stage, f"/World/diaper_{key}")
                m.CreateSubdivisionSchemeAttr("none")
                m.CreateDoubleSidedAttr(False)
                m.CreateFaceVertexCountsAttr([3] * len(faces))
                m.CreateFaceVertexIndicesAttr(faces.ravel().tolist())
                pv = m.CreateDisplayColorPrimvar(UsdGeom.Tokens.vertex)
                pv.Set(Vt.Vec3fArray.FromNumpy(np.ascontiguousarray(col, dtype=np.float32)))
                EP.meshes[key] = m
            pts = sh.x + nrm * sgn * (sh.thick / 2 + 0.0008) + origin
            m.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(np.ascontiguousarray(pts, dtype=np.float32)))


def _install_hooks() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # noqa: BLE001
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls, "_diaper_hooked", False):
        return
    orig_reset, orig_step = cls.reset, cls._task_step

    def reset(self, *a, **k):
        global EP
        EP = Episode()
        obs = orig_reset(self, *a, **k)
        if getattr(self.task.module, "INSTRUCTION", None) != INSTRUCTION:
            return obs
        _ENV["env"] = self._env
        EP.world.set_state(_doll_state(self._env).q, [])
        _render(self._env)
        for _ in range(3):
            self._base.sim.render()
        return self._observation()

    def _task_step(self, *a, **k):
        if getattr(self.task.module, "INSTRUCTION", None) == INSTRUCTION:
            _ENV["env"] = self._env
            _step_sheets(self._env)
            _render(self._env)
        return orig_step(self, *a, **k)

    cls.reset, cls._task_step, cls._diaper_hooked = reset, _task_step, True


def _install() -> None:
    import importlib.abc
    import importlib.util

    if "yam_arena.scene" in sys.modules:
        _patch_scene(sys.modules["yam_arena.scene"])
    else:
        class _Finder(importlib.abc.MetaPathFinder):
            _doll = True

            def find_spec(self, fullname, path, target=None):
                if fullname != "yam_arena.scene":
                    return None
                sys.meta_path.remove(self)
                try:
                    spec = importlib.util.find_spec(fullname)
                finally:
                    sys.meta_path.insert(0, self)
                orig_exec = spec.loader.exec_module

                def exec_module(module):
                    orig_exec(module)
                    _patch_scene(module)

                spec.loader.exec_module = exec_module
                return spec

        if not any(getattr(f, "_doll", False) for f in sys.meta_path):
            sys.meta_path.insert(0, _Finder())
    _install_hooks()


_install()


# -- scoring --------------------------------------------------------------------------------------------------
SUCCESS_HOLD_S = 3.0  # the run ends once the full end state has held this long (hands clear and still)
CLEAR = 0.05  # grippers (TCP and pad tips) this far from every diaper node and the doll's surface
IN_BIN_FRAC = 0.8
ON_DOLL = 0.015  # a used-diaper node this close to the doll counts as under / on it
NAVEL_TOL = 0.04
LEVEL_TOL = 0.03


def _doll_sd(world, p):
    """Signed distance (m) from points p (N,3) to the doll's surface."""
    A, B, Rr, _ = world._caps_cache
    ab = B - A
    ap = p[:, None, :] - A[None]
    t = np.clip((ap * ab[None]).sum(2) / np.maximum((ab * ab).sum(1), 1e-12)[None], 0, 1)
    sd = np.linalg.norm(ap - t[..., None] * ab[None], axis=2) - Rr[None]
    best = sd.min(1)
    for b, kind, prm, R in world.parts:
        if kind == "rbox":
            sdb, _ = _sd_rbox((p - prm[0]) @ R, np.zeros(3), prm[1], prm[2])
            best = np.minimum(best, sdb)
    return best


def _node(sh, u, v):
    return int(np.argmin(np.abs(sh.uv[:, 0] - u) + np.abs(sh.uv[:, 1] - v)))


def evaluate(objects) -> dict:
    """Physical end-state checks behind score()."""
    doll = objects[DOLL.name]
    q = np.asarray(doll.q, float)
    world = World()
    world.set_state(q, [])
    wet, cl = EP.wet, EP.clean
    out = {"finite": bool(np.isfinite(q).all() and np.isfinite(wet.x).all() and np.isfinite(cl.x).all())}
    if not out["finite"] or wet.torn or cl.torn:
        out["finite"] = False
        return out
    tips = np.vstack([EP.tcp, EP.tcp + EP.rot[:, :, 2] * 0.013])
    nodes = np.vstack([wet.x, cl.x])
    dn = np.linalg.norm(tips[:, None] - nodes[None], axis=2).min()
    out["released"] = bool(not wet.pins and not cl.pins and dn >= CLEAR and _doll_sd(world, tips).min() >= CLEAR)
    out["wet_on_doll"] = bool((_doll_sd(world, wet.x) < ON_DOLL).any())
    d = np.linalg.norm(wet.x[:, :2] - BIN_C, axis=1)
    inb = (d < BIN_R) & (wet.x[:, 2] < TOP + BIN_H + 0.01)
    out["wet_in_bin"] = bool(inb.mean() >= IN_BIN_FRAC)
    out["wet_bin_frac"] = round(float(inb.mean()), 2)
    # clean diaper, in doll-local coordinates of the (lowered) pelvis
    rp, tp = body_frames(q)["pelvis"]
    loc = (cl.x - np.array([DX, Y0, MT]) - tp) @ rp  # pelvis-local = rest-pose doll-local
    back = (cl.uv[:, 0] >= 0.03) & (cl.uv[:, 0] <= 0.08) & (np.abs(cl.uv[:, 1]) <= 0.03)
    pb = loc[back]
    under = bool(np.all((np.abs(pb[:, 0]) < 0.055) & (pb[:, 1] > 0.015) & (pb[:, 1] < 0.14) & (pb[:, 2] < 0.012)))
    edge = loc[_node(cl, 0.0, 0.0)]
    out["back_under"] = under and bool(0.08 <= edge[1] <= 0.17) and abs(float(q[0])) < 0.1
    fw = loc[_node(cl, dp.LENGTH, 0.0)]
    crotch = loc[_node(cl, 0.18, 0.0)]
    out["front_up"] = bool(fw[2] > 0.06 and abs(fw[0]) <= 0.04 and abs(fw[1] - NAVEL[1]) <= NAVEL_TOL
                           and abs(crotch[0]) < 0.03)
    tabs = {}
    for n in cl.stuck:
        tabs.setdefault(int(cl.tab[n]), []).append(loc[n])
    out["tabs"] = len(tabs)
    out["level"] = bool(len(tabs) == 2 and abs(np.mean(tabs[1], 0)[1] - np.mean(tabs[2], 0)[1]) <= LEVEL_TOL)
    return out


def score(objects) -> float:
    e = evaluate(objects)
    if not e.get("finite") or not e["released"] or e["wet_on_doll"] or not e["wet_in_bin"]:
        return 0.0
    if not e["back_under"]:
        return 0.25
    if not (e["front_up"] and e["tabs"] >= 1):
        return 0.5
    if not (e["tabs"] == 2 and e["level"]):
        return 0.75
    return 1.0


def success(objects) -> bool:
    if score(objects) < 1.0:
        EP.done_since = None
        return False
    if EP.done_since is None:
        EP.done_since = EP.step_n
    return EP.step_n - EP.done_since >= SUCCESS_HOLD_S * 20  # 20 Hz control


def progress(objects) -> dict:
    e = evaluate(objects)
    return {k: v for k, v in e.items()}
