"""wirekit: insulated solid wires that a wire stripper can cut and strip, for a yam_arena task file.

A :class:`Pigtail` is spawned as a static (``BASE``) Arena asset holding its own rigid bodies:

- ``core``: the wire. Its colliders are the jacket (radius ``r_ins``) from the far end to where the strippable zone starts,
  and the bare copper (``r_cu``) along the zone, out to the stripped end. Local +z runs from the far end to that end.
- ``ring_00`` ... ``ring_NN``: the jacket over the zone, ``pitch`` long each, each held to the core by its own fixed joint
  (``core_joint_NN``). Ring-to-ring joints (``ring_joint_NN``, ring NN to the next) are authored switched off. Rings and
  core do not collide with each other (the rings sit round the copper).

Cutting and stripping (checked every physics step, before PhysX steps):

1. The stripper's jaws within ``SHUT_TOL_DEG`` of shut, with the wire's axis through one of its holes, nearly square to the
   tool's plane: the blade has bitten. The rings it cuts into (those within ``BLADE_REACH`` of the tool's plane) stop
   colliding, so the jaws can close the rest of the way, as far as the blade's hole lets them. The hole's gauge decides:
   the wire's own (``gauge``) cuts the jacket through; a smaller hole also bites the copper (``nicked``); a larger hole
   leaves the jacket uncut through, and nothing comes off.
2. After a clean cut the rings beyond it join one another into a slug, held on the copper only by a slide along it with a
   spring of ``strip_n`` per ``strip_mm``. The ring just beyond the blade bears on it while the wire is pulled out; once
   the slug has been stretched ``strip_mm`` along the copper (a pull of ``strip_n``) it lets go: the slug stays in the
   jaws and the copper slides out past the blade. When the jaws open, the slug drops out.

Pure Python at import; Isaac Sim is imported lazily. Importing hooks ``yam_arena.scene`` and ``yam_arena.inspect_embodiment``.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import os
import sys
from dataclasses import dataclass, field

import numpy as np
from yam_arena.task_file import Box, ObjectState

Color = tuple[float, float, float]
COPPER = (0.80, 0.50, 0.32)
SHUT_TOL_DEG = 3.0  # the jaws within this of shut count as biting
RELEASE_DEG = 2.0  # and this much further open lets go
FULLY_SHUT_DEG = 0.7  # the jaws this close to shut on a wire outside every hole's notch have cut into it
LATCH_OFF, LATCH_TILT_DEG = 0.0008, 8.0  # the tip within this of the hole's axis, the wire within this of square, to latch
DROP_SLUG = bool(os.environ.get("WIRE_DROP_SLUG"))  # off: the cut slug stays in the opened jaws, as it often does
CLEAR_OF_BLADE = 0.010  # a cut-into or dropped ring this far from the stripper's plane gets its collider back
SEPARATED = 0.0008  # the cut ring this far off its place on the core: the slug has come away
BLADE_REACH = 0.0012  # rings with any part within this of the tool's plane are cut into (blade 1 mm thick, plus margin)
CORE_INERTIA = (7e-6, 7e-6, 4e-8)  # kg m^2: across a 150 mm wire as it is; about its own axis raised well above a thin wire's, which the solver needs
RING_MASS, RING_INERTIA = 2e-4, (8e-9, 8e-9, 8e-9)  # a jacket ring: heavier than its 0.01 g, so the solver holds it against the blade
SQUARE_DEG = 30.0  # the wire's axis within this of the tool's normal
DEBUG = bool(os.environ.get("WIRE_DEBUG"))


@dataclass(frozen=True)
class Pigtail(Box):
    """``pos`` is the wire's centre at the start and ``quat_xyzw`` its rotation (local +z from far end to stripped end)."""

    quat_xyzw: tuple = (0.0, 0.0, 0.0, 1.0)
    length: float = 0.150
    r_ins: float = 0.001346  # 14 AWG solid THHN/THWN-2: 0.106 in (2.69 mm) over the jacket (Southwire spec 10003)
    r_cu: float = 0.000815  # 1.63 mm solid copper
    zone: float = 0.050  # strippable length, from the stripped end
    pitch: float = 0.002
    mass: float | None = None  # a fixed asset; the bodies carry ``wire_mass``
    wire_mass: float = 0.0035  # 150 mm at about 23 g/m
    jacket: Color = (0.03, 0.03, 0.03)
    gauge: int = 14
    strip_n: float = 10.0  # N: the pull that takes a cut 14 AWG THHN slug off ...
    strip_mm: float = 0.001  # ... once the cut jacket has stretched this far along the copper (its spring: strip_n / strip_mm)
    strip_hold: int = 3
    friction: float = 0.9
    stripper: str = "stripper"  # the scene name of the mechkit stripper
    holes: tuple = ()  # push-in holes it can latch in: (name, mouth centre (env frame), unit axis into the hole)
    latch_depth: float = 0.012  # copper this far in engages the push-in clamp ...
    bottom_depth: float = 0.0175  # ... and can go no further than the hole's bottom
    latch_step: float = 0.0005  # the clamp holds the wire within this of the deepest it has been pushed

    @property
    def latch_depths(self) -> tuple:
        n = int(round((self.bottom_depth - self.latch_depth) / self.latch_step))
        return tuple(self.latch_depth + i * self.latch_step for i in range(n + 1))

    @property
    def n_rings(self) -> int:
        return int(round(self.zone / self.pitch))

    @property
    def zone_start(self) -> float:
        """Local z where the zone starts (the core's frame: its centre at 0)."""
        return self.length / 2 - self.zone

    def ring_z(self, k: int) -> float:
        return self.zone_start + (k + 0.5) * self.pitch


@dataclass
class Strip:
    """What has happened to one wire."""

    bites: list = field(default_factory=list)  # (t, gauge hole, local z of the cut)
    nicked: bool = False
    crushed: bool = False
    biting: tuple | None = None  # (gauge hole, local z) while the blade is in the jacket
    armed: int | None = None  # the joint index at a clean cut, waiting for the pull
    run: int = 0
    stripped_at: float | None = None  # local z of the cut that came off
    t_strip: float | None = None
    off_rings: tuple = ()  # the rings the blade is in now (colliders off)
    latched: str | None = None  # the push-in hole whose clamp holds the wire
    t_latch: float | None = None
    latch_i: int = 0  # which of its ratchet's slides holds it
    pending_on: tuple = ()  # cut-into rings whose colliders come back once they are clear of the stripper
    slug_dropped: bool = False

    @property
    def length(self) -> float | None:
        return None


@dataclass(frozen=True)
class PigtailState(ObjectState):
    """``pos`` is the core's centre (env frame). ``end`` the stripped end's centre, ``axis`` local +z in the world."""

    end: np.ndarray = None
    axis: np.ndarray = None
    bare: float = 0.0  # stripped length (m): 0 until a slug has come off
    nicked: bool = False
    slug_clear: bool = False  # every ring of the slug is off the copper
    bites: tuple = ()
    ring_pos: np.ndarray = None
    latched: str | None = None  # the hole whose clamp holds it
    depth: float = 0.0  # how far the copper's tip is in that hole (m)
    crushed: bool = False


def quat_to_matrix(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def plane_hit(core_p, axis, tool_p, tool_R):
    """Where the wire's axis meets the tool's plane: (point in tool frame, local z along the wire from the core's centre, angle to the normal in degrees), or None."""
    n = tool_R[:, 2]
    denom = float(np.dot(axis, n))
    if abs(denom) < 1e-6:
        return None
    s = float(np.dot(tool_p - core_p, n)) / denom
    hit = core_p + s * axis
    return tool_R.T @ (hit - tool_p), s, math.degrees(math.acos(min(1.0, abs(denom))))


def hole_at(local_xy, holes: dict, half: float):
    """The gauge whose hole (centres on tool y 0 at x) contains a tool-frame point, within ``half`` either way."""
    x, y = local_xy
    if abs(y) > half:
        return None
    for awg, (hx, _) in holes.items():
        if abs(x - hx) <= half:
            return awg
    return None


# -- spawning --------------------------------------------------------------------------------------------------

_SPECS: dict[str, Pigtail] = {}
_PATHS: dict[str, str] = {}


def _spawn(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics
    from yam_arena.station import linear_rgb

    s = _SPECS[cfg.wire_name]
    _PATHS.setdefault(s.name, prim_path)
    stage = get_current_stage()
    dbl = UsdGeom.XformOp.PrecisionDouble
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=dbl).Set(Gf.Vec3d(0.0, 0.0, 0.0))
    material = f"{prim_path}/Looks/physics"
    mat = sim_utils.RigidBodyMaterialCfg(static_friction=s.friction, dynamic_friction=s.friction)
    mat.func(material, mat)
    looks = {}
    for key, color, metal in (("jacket", s.jacket, 0.0), ("copper", COPPER, 0.9)):
        looks[key] = f"{prim_path}/Looks/{key}"
        vis = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=0.4 if metal else 0.6, metallic=metal)
        vis.func(looks[key], vis)

    R = quat_to_matrix(s.quat_xyzw)
    x, y, z, w = s.quat_xyzw
    q = Gf.Quatd(float(w), float(x), float(y), float(z))
    centre = np.asarray(s.pos, float)

    def body(name, local_z, mass, inertia=None):
        path = f"{prim_path}/{name}"
        xf = UsdGeom.Xform.Define(stage, path)
        xf.AddTranslateOp(precision=dbl).Set(Gf.Vec3d(*(centre + R[:, 2] * local_z)))
        xf.AddOrientOp(precision=dbl).Set(q)
        prim = xf.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        mapi = UsdPhysics.MassAPI.Apply(prim)
        mapi.CreateMassAttr(mass)
        if inertia is not None:
            mapi.CreateDiagonalInertiaAttr(Gf.Vec3f(*inertia))
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateMaxDepenetrationVelocityAttr(0.5)
        rb.CreateSolverPositionIterationCountAttr(32)
        rb.CreateSolverVelocityIterationCountAttr(4)
        rb.CreateLinearDampingAttr(0.1)
        rb.CreateAngularDampingAttr(0.5)
        return path

    def cylinder(path, name, radius, z0, z1, look, collide=True, visible=True):
        c = UsdGeom.Cylinder.Define(stage, f"{path}/{name}")
        c.CreateRadiusAttr(radius)
        c.CreateHeightAttr(z1 - z0)
        c.CreateAxisAttr("Z")
        c.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, (z0 + z1) / 2))
        if collide:
            UsdPhysics.CollisionAPI.Apply(c.GetPrim())
            col = PhysxSchema.PhysxCollisionAPI.Apply(c.GetPrim())
            col.CreateContactOffsetAttr(0.0008)
            col.CreateRestOffsetAttr(0.0)
            bind_physics_material(str(c.GetPath()), material, stage=stage)
        if look:
            bind_visual_material(str(c.GetPath()), looks[look], stage=stage)
        if not visible:
            c.CreateVisibilityAttr("invisible")
        return c.GetPrim()

    def joint(name, b0, b1, z0, z1, enabled=True, break_n=None):
        j = UsdPhysics.FixedJoint.Define(stage, f"{prim_path}/{name}")
        j.CreateBody0Rel().SetTargets([Sdf.Path(b0)])
        j.CreateBody1Rel().SetTargets([Sdf.Path(b1)])
        j.CreateLocalPos0Attr(Gf.Vec3f(0.0, 0.0, float(z0)))
        j.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, float(z1)))
        j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateExcludeFromArticulationAttr(True)
        j.CreateJointEnabledAttr(enabled)
        if break_n is not None:
            j.CreateBreakForceAttr(float(break_n))

    n = s.n_rings
    ring_mass = RING_MASS
    core_mass = s.wire_mass  # the copper and the jacket outside the zone; the rings add their own
    half = s.length / 2
    core = body("core", 0.0, core_mass, CORE_INERTIA)
    cylinder(core, "jacket", s.r_ins, -half, s.zone_start, "jacket")
    cylinder(core, "copper", s.r_cu, s.zone_start, half, "copper")
    rings = []
    for k in range(n):
        rz = s.ring_z(k)
        path = body(f"ring_{k:02d}", rz, ring_mass, RING_INERTIA)
        prim = cylinder(path, "jacket", s.r_ins, -s.pitch / 2 + 0.00002, s.pitch / 2 - 0.00002, "jacket")
        PhysxSchema.PhysxContactReportAPI.Apply(stage.GetPrimAtPath(path)).CreateThresholdAttr(0.0)
        rings.append(path)
        joint(f"core_joint_{k:02d}", core, path, rz, 0.0)
        sj = UsdPhysics.PrismaticJoint.Define(stage, f"{prim_path}/core_slide_{k:02d}")  # the cut jacket's grip on the copper: a stiff spring along it
        sj.CreateBody0Rel().SetTargets([Sdf.Path(core)])
        sj.CreateBody1Rel().SetTargets([Sdf.Path(path)])
        sj.CreateLocalPos0Attr(Gf.Vec3f(0.0, 0.0, float(rz)))
        sj.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
        sj.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        sj.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        sj.CreateAxisAttr("Z")
        sj.CreateLowerLimitAttr(-0.004)
        sj.CreateUpperLimitAttr(0.004)
        sj.CreateExcludeFromArticulationAttr(True)
        sj.CreateJointEnabledAttr(False)
        drive = UsdPhysics.DriveAPI.Apply(sj.GetPrim(), "linear")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(float(s.strip_n / s.strip_mm))
        drive.CreateDampingAttr(float(s.strip_n / s.strip_mm) * 0.02)
        drive.CreateTargetPositionAttr(0.0)
        drive.CreateMaxForceAttr(1e6)
        if k > 0:
            joint(f"ring_joint_{k - 1:02d}", rings[k - 1], path, s.pitch / 2, -s.pitch / 2, enabled=False)
    for name, mouth, axis_in in s.holes:  # a push-in clamp per hole: a ratchet of slides along the hole, each holding the wire in to its depth
        a = np.asarray(axis_in, float) / np.linalg.norm(axis_in)
        qa = np.r_[1.0 + a[2], np.cross((0.0, 0.0, 1.0), a)]  # w, x, y, z: turns +z onto the hole's axis
        qa = qa / np.linalg.norm(qa) if np.linalg.norm(qa) > 1e-9 else np.array([0.0, 1.0, 0.0, 0.0])
        for i, depth in enumerate(s.latch_depths):
            j = UsdPhysics.PrismaticJoint.Define(stage, f"{prim_path}/latch_{name}_{i:02d}")
            j.CreateBody1Rel().SetTargets([Sdf.Path(core)])
            j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, mouth)))
            j.CreateLocalRot0Attr(Gf.Quatf(*map(float, qa)))
            j.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, float(half)))
            j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
            j.CreateAxisAttr("Z")
            j.CreateLowerLimitAttr(float(depth))
            j.CreateUpperLimitAttr(float(s.bottom_depth))
            j.CreateExcludeFromArticulationAttr(True)
            j.CreateJointEnabledAttr(False)
    if not os.environ.get("WIRE_NO_FILTER"):  # rings never touch the core: they sit round its copper, joined to it or (a slug) sliding off it
        pairs = UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(core))
        pairs.CreateFilteredPairsRel().SetTargets([Sdf.Path(p) for p in rings])
    return root.GetPrim()


def _arena_object(spec: Pigtail):
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    @configclass
    class _Cfg(SpawnerCfg):
        func: object = clone(_spawn)
        wire_name: str = ""

    return Object(name=spec.name, object_type=ObjectType.BASE, spawner_cfg=_Cfg(wire_name=spec.name),
                  initial_pose=Pose(position_xyz=(0.0, 0.0, 0.0), rotation_xyzw=(0.0, 0.0, 0.0, 1.0)))


# -- runtime ---------------------------------------------------------------------------------------------------


def _arr(x) -> np.ndarray:
    x = getattr(x, "torch", x)
    if hasattr(x, "detach"):
        return x.detach().cpu().numpy().astype(np.float64)
    return np.asarray(x.numpy() if hasattr(x, "numpy") else x, dtype=np.float64)


class _Runtime:
    """PhysX views and the strip rule for one wire (env 0)."""

    def __init__(self, base, spec: Pigtail):
        from isaaclab.sim.utils.stage import get_current_stage
        from isaaclab_physx.physics import PhysxManager
        from pxr import UsdPhysics

        import stripper as S

        self.S = S
        self.spec, self.base = spec, base
        root = _PATHS[spec.name]
        self.stage = get_current_stage()
        n = spec.n_rings
        self.names = ["core"] + [f"ring_{k:02d}" for k in range(n)]
        view = PhysxManager.get_physics_sim_view()
        self.bodies = view.create_rigid_body_view([f"{root}/{nm}" for nm in self.names])
        self.has_tool = spec.stripper in base.scene.keys()
        self.contact = None
        if self.has_tool:
            tool = base.scene[spec.stripper]
            tool_paths = tool.root_physx_view.link_paths
            tool_paths = list(tool_paths[0] if tool_paths and isinstance(tool_paths[0], (list, tuple)) else tool_paths)
            if DEBUG:
                print(f"[wirekit] dbg stripper link paths {tool_paths}", flush=True)
            self.tool_paths = tool_paths
            self.contact = view.create_rigid_contact_view([f"{root}/ring_{k:02d}" for k in range(n)], filter_patterns=[tool_paths] * n,
                                                          max_contact_data_count=512)
        self.core_joints = [UsdPhysics.Joint(self.stage.GetPrimAtPath(f"{root}/core_joint_{k:02d}")) for k in range(n)]
        self.ring_joints = [UsdPhysics.Joint(self.stage.GetPrimAtPath(f"{root}/ring_joint_{k:02d}")) for k in range(n - 1)]
        self.break_joints = [UsdPhysics.Joint(self.stage.GetPrimAtPath(f"{root}/core_slide_{k:02d}")) for k in range(n)]
        self.ring_cols = [UsdPhysics.CollisionAPI(self.stage.GetPrimAtPath(f"{root}/ring_{k:02d}/jacket")) for k in range(n)]
        self.latches = {h[0]: [UsdPhysics.Joint(self.stage.GetPrimAtPath(f"{root}/latch_{h[0]}_{i:02d}")) for i in range(len(spec.latch_depths))]
                        for h in spec.holes}
        self.start = _arr(self.bodies.get_transforms()).copy()
        self.origin = _arr(base.scene.env_origins)[0]
        self.dt = float(base.physics_dt)
        self.t = 0.0
        self.strip = Strip()

    def _tool(self):
        """The stripper's lower half (its frame) pose and the moving half's joint (deg)."""
        import mechkit as m

        st = m.state(self.base, self.spec.stripper)
        return st.pos, st.rotation(), st.deg("upper")

    def _set_off(self, rings: tuple) -> None:
        old = set(self.strip.off_rings)
        for k in set(rings) - old:
            self.ring_cols[k].GetCollisionEnabledAttr().Set(False)
        for k in old - set(rings):
            self.ring_cols[k].GetCollisionEnabledAttr().Set(True)
        self.strip.off_rings = tuple(sorted(rings))

    def _slug(self, b: int, on: bool) -> None:
        """A clean cut at ring boundary b: the rings beyond it join one another and hang on the core by ring b's breakable
        joint alone, which PhysX breaks at the strip force (authored at spawn: PhysX does not take a break force changed
        while it runs). `on` False puts every ring back on its own unbreakable joint."""
        s = self.spec
        for k in range(b, s.n_rings - 1):
            self.ring_joints[k].GetJointEnabledAttr().Set(on)
        for k in range(b, s.n_rings):
            self.core_joints[k].GetJointEnabledAttr().Set(not on)
        self.break_joints[b].GetJointEnabledAttr().Set(on)

    def _latch_check(self, end, axis) -> None:
        """The copper's tip in a push-in hole, past the latch depth and square in it: the clamp takes it. Once taken, it holds
        the wire at the deepest it has been pushed (to within latch_step)."""
        s, st = self.spec, self.strip
        if not s.holes:
            return
        if st.latched is not None:
            name, mouth, axis_in = next(h for h in s.holes if h[0] == st.latched)
            a = np.asarray(axis_in, float) / np.linalg.norm(axis_in)
            depth = float((end - np.asarray(mouth, float)) @ a)
            i = max(k for k, d in enumerate(s.latch_depths) if d <= depth + 1e-9) if depth >= s.latch_depth else 0
            if i > st.latch_i:
                self.latches[name][i].GetJointEnabledAttr().Set(True)
                self.latches[name][st.latch_i].GetJointEnabledAttr().Set(False)
                st.latch_i = i
            return
        for name, mouth, axis_in in s.holes:
            a = np.asarray(axis_in, float) / np.linalg.norm(axis_in)
            d = end - np.asarray(mouth, float)
            depth = float(d @ a)
            off = float(np.linalg.norm(d - depth * a))
            tilt = math.degrees(math.acos(min(1.0, float(axis @ a))))
            if depth >= s.latch_depth and off <= LATCH_OFF and tilt <= LATCH_TILT_DEG:
                self.latches[name][0].GetJointEnabledAttr().Set(True)
                st.latched, st.t_latch, st.latch_i = name, self.t, 0
                print(f"[wirekit] {s.name}: latched in push-in hole {name} at {depth * 1000:.1f} mm, t={self.t:.2f}s", flush=True)
                return

    def physics_step(self) -> None:
        s, S, st = self.spec, self.S, self.strip
        self.t += self.dt
        if s.holes:
            tr0 = _arr(self.bodies.get_transforms())[0]
            ax = quat_to_matrix(tr0[3:7])[:, 2]
            self._latch_check(tr0[:3] - self.origin + ax * s.length / 2, ax)
        if not self.has_tool:
            return
        if st.pending_on:
            tool_p, tool_R, _ = self._tool()
            rings = _arr(self.bodies.get_transforms())[1:, :3] - self.origin
            back = []
            for k in st.pending_on:
                if abs(float((rings[k] - tool_p) @ tool_R[:, 2])) > CLEAR_OF_BLADE:
                    self.ring_cols[k].GetCollisionEnabledAttr().Set(True)
                else:
                    back.append(k)
            st.pending_on = tuple(back)
        tr = _arr(self.bodies.get_transforms())
        core_p = tr[0, :3] - self.origin
        Rw = quat_to_matrix(tr[0, 3:7])
        axis = Rw[:, 2]
        tool_p, tool_R, jdeg = self._tool()
        opened = math.radians(jdeg - S.SHUT_J)  # the moving half's angle from shut

        def gap_at(x):  # how far the moving jaw's edge still is from shut, at tool x
            return abs(x) * math.sin(max(0.0, opened))

        shut = opened <= math.radians(SHUT_TOL_DEG)
        hit = plane_hit(core_p, axis, tool_p, tool_R)
        if st.biting is not None:  # the blade is in the jacket: it stays where it bit until the jaws open
            if opened > math.radians(SHUT_TOL_DEG + RELEASE_DEG):
                drop = ()
                if DROP_SLUG and st.stripped_at is not None and not st.slug_dropped:  # the cut slug drops out of the opened jaws
                    b = int(round((st.stripped_at - s.zone_start) / s.pitch))
                    drop = tuple(range(b, s.n_rings))
                    for k in drop:
                        self.ring_cols[k].GetCollisionEnabledAttr().Set(False)
                    st.slug_dropped = True
                st.pending_on = tuple(sorted(set(st.pending_on) | set(st.off_rings) | set(drop)))  # back on once clear of the blade
                st.off_rings = ()
                if st.armed is not None:
                    self._slug(st.armed, False)
                st.biting, st.armed, st.run = None, None, 0
                return
            if st.armed is None:
                return
            b = st.armed
            ring = _arr(self.bodies.get_transforms())[1 + b]
            rel = Rw.T @ (ring[:3] - tr[0, :3])
            moved = float(rel[2] - s.ring_z(b))  # the slug pulled towards the wire's end, on its spring
            if DEBUG and int(self.t / self.dt) % 12 == 0:
                print(f"[wirekit] dbg armed b={b} slug stretched {moved * 1000:.2f} mm ({moved / s.strip_mm * s.strip_n:.1f} N)", flush=True)
            if moved >= s.strip_mm:
                st.stripped_at = s.zone_start + b * s.pitch
                st.t_strip, st.armed = self.t, None
                self.break_joints[b].GetJointEnabledAttr().Set(False)  # the jacket lets go of the copper
                print(f"[wirekit] {s.name}: stripped {(s.length / 2 - st.stripped_at) * 1000:.1f} mm at t={self.t:.2f}s", flush=True)
            return
        if hit is None or hit[2] > SQUARE_DEG or not -s.length / 2 < hit[1] < s.length / 2:
            return
        hx, hy = hit[0][:2]
        awg = hole_at((hx, hy), S.HOLES, S.NOTCH_FLAT + S.NOTCH_RAMP)  # anywhere in a hole's notch, its ramps included
        if awg is not None:
            # the blades meet the jacket when the moving jaw is within twice the jacket's depth past the blade's hole (plus 0.5 mm)
            if gap_at(S.HOLES[awg][0]) > 2 * (s.r_ins - S.blade_r(S.HOLES[awg][1])) + 0.0005:
                return
        elif not shut:
            return
        if awg is None:
            if opened <= math.radians(FULLY_SHUT_DEG) and S.JAW_X0 < hx < S.JAW_TIP and abs(hy) < S.JAW_W and not st.crushed:
                st.crushed = st.nicked = True  # the jaws' edges met on the wire between holes: cut into
                print(f"[wirekit] {s.name}: jaws shut on the wire between holes, at tool x {hx * 1000:.1f} mm: wire damaged", flush=True)
            return
        z = hit[1]
        if z <= s.zone_start - s.pitch:
            if not st.crushed:
                st.crushed = st.nicked = True
                print(f"[wirekit] {s.name}: jaws shut {(s.length / 2 - z) * 1000:.0f} mm from the end, beyond the strippable zone: wire damaged", flush=True)
            return
        st.biting = (awg, z)
        st.bites.append((round(self.t, 3), awg, round(z, 4)))
        print(f"[wirekit] {s.name}: jaws bite in the {awg} AWG hole, {(s.length / 2 - z) * 1000:.1f} mm from the end, t={self.t:.2f}s", flush=True)
        if awg > s.gauge:
            st.nicked = True
            print(f"[wirekit] {s.name}: the {awg} AWG hole is smaller than the {s.gauge} AWG copper: nicked", flush=True)
        self._set_off(tuple(k for k in range(s.n_rings) if abs(s.ring_z(k) - z) <= s.pitch / 2 + BLADE_REACH))
        if st.stripped_at is None and awg == s.gauge and not st.nicked:
            st.armed = max(0, min(s.n_rings - 1, int(round((z - s.zone_start) / s.pitch))))  # a clean cut, at the nearest ring boundary
            self._slug(st.armed, True)

    def reset(self) -> None:
        import torch

        for j in self.core_joints:
            j.GetJointEnabledAttr().Set(True)
        for j in self.break_joints:
            j.GetJointEnabledAttr().Set(False)
        for js in self.latches.values():
            for j in js:
                j.GetJointEnabledAttr().Set(False)
        for j in self.ring_joints:
            j.GetJointEnabledAttr().Set(False)
        for c in self.ring_cols:
            c.GetCollisionEnabledAttr().Set(True)
        idx = np.arange(len(self.names), dtype=np.int32)
        cur = self.bodies.get_transforms()
        if hasattr(cur, "detach"):
            dev = cur.device
            self.bodies.set_transforms(torch.tensor(self.start, dtype=torch.float32, device=dev), torch.tensor(idx, device=dev))
            self.bodies.set_velocities(torch.zeros((len(idx), 6), dtype=torch.float32, device=dev), torch.tensor(idx, device=dev))
        else:
            import warp as wp

            dev = getattr(cur, "device", None)
            self.bodies.set_transforms(wp.array(self.start.astype(np.float32), device=dev), wp.array(idx, dtype=wp.int32, device=dev))
            self.bodies.set_velocities(wp.zeros((len(idx), 6), dtype=wp.float32, device=dev), wp.array(idx, dtype=wp.int32, device=dev))
        self.strip = Strip()
        self.t = 0.0

    def state(self) -> PigtailState:
        s = self.spec
        tr = _arr(self.bodies.get_transforms())
        vel = _arr(self.bodies.get_velocities())
        p = tr[:, :3] - self.origin
        R = quat_to_matrix(tr[0, 3:7])
        axis = R[:, 2]
        end = p[0] + axis * s.length / 2
        bare, clear = 0.0, False
        if self.strip.stripped_at is not None:
            bare = s.length / 2 - self.strip.stripped_at
            b = int(round((self.strip.stripped_at - s.zone_start) / s.pitch))
            clear = True
            for k in range(b, s.n_rings):
                rel = R.T @ (p[1 + k] - p[0])
                if np.hypot(rel[0], rel[1]) < s.r_ins + 0.002 and rel[2] < s.length / 2 + s.pitch:
                    clear = False
        depth = 0.0
        if self.strip.latched is not None:
            name, mouth, axis_in = next(h for h in s.holes if h[0] == self.strip.latched)
            a = np.asarray(axis_in, float) / np.linalg.norm(axis_in)
            depth = float((end - np.asarray(mouth, float)) @ a)
        lo, hi = p.min(0), p.max(0)
        return PigtailState(pos=p[0], quat_xyzw=tr[0, 3:7], lin_vel=vel[0, :3], ang_vel=vel[0, 3:], low=lo, high=hi, end=end, axis=axis,
                            bare=bare, nicked=self.strip.nicked, slug_clear=clear, bites=tuple(self.strip.bites), ring_pos=p[1:],
                            latched=self.strip.latched, depth=depth, crushed=self.strip.crushed)


_RT: dict[str, _Runtime] = {}


def _runtime(base, name: str) -> _Runtime:
    if name not in _RT:
        try:
            _RT[name] = _Runtime(base, _SPECS[name])
        except Exception:
            import traceback

            traceback.print_exc()
            raise
    return _RT[name]


# -- hooks -----------------------------------------------------------------------------------------------------


def _patch_scene(scene) -> None:
    if getattr(scene, "_wirekit", False):
        return
    task_object, object_states = scene.task_object, scene.object_states

    def _task_object(spec):
        if isinstance(spec, Pigtail):
            _SPECS[spec.name] = spec
            return _arena_object(spec)
        return task_object(spec)

    def _object_states(environment, names):
        plain = [n for n in names if n not in _SPECS]
        out = object_states(environment, plain) if plain else {}
        out.update({n: _runtime(environment.unwrapped, n).state() for n in names if n in _SPECS})
        return {n: out[n] for n in names}

    scene.task_object, scene.object_states, scene._wirekit = _task_object, _object_states, True


def _hook_physics(environment) -> None:
    base = environment.unwrapped
    isc = base.scene
    if getattr(isc, "_wirekit", False):
        return
    write = isc.write_data_to_sim

    def write_data_to_sim(*a, **k):
        for name in _SPECS:
            _runtime(base, name).physics_step()
        return write(*a, **k)

    isc.write_data_to_sim, isc._wirekit = write_data_to_sim, True


def _patch_embodiment(module) -> None:
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_wirekit", False):
        return
    ensure_sim, reset = cls._ensure_sim, cls.reset

    def _ensure_sim(self, *a, **k):
        out = ensure_sim(self, *a, **k)
        if self._env is not None:
            _hook_physics(self._env)
        return out

    def _reset(self, *a, **k):
        if self._env is not None:
            for name in _SPECS:
                _runtime(self._base, name).reset()
        obs = reset(self, *a, **k)
        for name in _SPECS:
            rt = _runtime(self._base, name)
            rt.strip, rt.t = Strip(), 0.0  # the settle steps are not part of the trial
        return obs

    cls._ensure_sim, cls.reset, cls._wirekit = _ensure_sim, _reset, True


_TARGETS = {"yam_arena.scene": _patch_scene, "yam_arena.inspect_embodiment": _patch_embodiment}


class _Finder(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path, target=None):
        if fullname not in _TARGETS:
            return None
        sys.meta_path.remove(self)
        try:
            spec = importlib.util.find_spec(fullname)
        finally:
            sys.meta_path.insert(0, self)
        if spec is None or spec.loader is None:
            return None
        exec_module = spec.loader.exec_module

        def _exec(module):
            exec_module(module)
            _TARGETS[fullname](module)

        spec.loader.exec_module = _exec
        return spec


def install() -> None:
    for name, patch in _TARGETS.items():
        if name in sys.modules:
            patch(sys.modules[name])
    if not any(isinstance(f, _Finder) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


install()
