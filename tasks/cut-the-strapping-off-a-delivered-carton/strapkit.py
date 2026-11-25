"""A strapped carton and a concealed-blade safety cutter for yam_arena task files.

One rig holds everything that moves:

- ``carton``: one rigid box (the delivered case with its contents), free to slide on the boards it stands on.
- ``strap{s}_{k}``: each strap is a closed loop of thin rigid links round the carton's girth, joined end to end by
  ball joints. One joint of the loop, under the carton, is a slide with a spring instead: the spring is stretched at
  the start, so the loop is in tension from the first step, and it lets the strap give when something is pushed under
  it. Nothing is glued to the carton: the strap stays on by its tension and friction.
- ``cutter`` (handle, upper jaw, sole and a wedge-shaped toe) and ``blade``: two rigid bodies joined by a fixed
  joint. The blade is a small body across the cutting channel, 4 mm high, between the toe and the upper jaw; it is
  out of reach of anything thicker than the channel.

Cutting is the one thing rigid bodies cannot do, so it is a rule on a measured force. Every physics step the kit
reads PhysX's contact force between the blade body and each strap link, from the step just simulated. When that
force on one link has been at ``cut_n`` or more for ``cut_hold`` steps in a row, the strap joint nearest the blade is
switched off and the strap parts there. A strap can only reach the blade by riding up the toe into the channel.
For ``watch_steps`` after a cut the kit records how far each of the two cut ends travels.

The same contact readout reports the largest force with which the cutter has pressed on the carton.

``object_states`` returns a :class:`StrapState` for the rig. Importing the module hooks ``yam_arena.scene``,
``yam_arena.inspect_embodiment`` and ``yam_arena.demo`` (a Waypoint's ``yaw`` may be a 3x3 rotation matrix).
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
TCP_OFFSET = 0.134  # yam_arena.constants.TCP_OFFSET: tool point on the gripper link's z axis
ARMS = ("left_robot", "right_robot")


def rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def matrix_to_wxyz(R) -> np.ndarray:
    R = np.asarray(R, float)
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        q = [0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s]
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = math.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k]) * 2
        v = [0.0, 0.0, 0.0]
        v[i], v[j], v[k] = 0.25 * s, (R[j, i] + R[i, j]) / s, (R[k, i] + R[i, k]) / s
        q = [(R[k, j] - R[j, k]) / s, *v]
    q = np.asarray(q, float)
    return q / np.linalg.norm(q)


def quat_to_matrix(q) -> np.ndarray:
    """Rotation matrix of a quaternion given as x, y, z, w."""
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


@dataclass(frozen=True)
class StrapRig(Box):
    """``pos`` is the carton's centre and ``size`` its x, y, z edge lengths (env frame: x forward, y left, z up, m)."""

    carton_mass: float = 2.5
    carton_color: Color = (0.72, 0.58, 0.40)
    fixed_boxes: tuple = ()  # (centre, size, colour) boxes that do not move, env frame
    strap_x: tuple = ()  # env x of each strap's middle line; the loop runs round the carton's y-z girth
    strap_w: float = 0.012
    strap_t: float = 0.001
    n_top: int = 9  # links across the top (and across the bottom)
    n_side: int = 3  # links down each side
    link_mass: float = 0.002
    link_inertia: float = 1e-6
    strap_colors: tuple = ((0.10, 0.10, 0.11),)
    spring_k: float = 5000.0  # N/m, the strap's stretch stiffness (in the one slide under the carton)
    spring_c: float = 20.0  # N s/m
    slack: float = 0.0  # m the loop takes up before it is taut (links settle onto the carton's edges); added to the stretch
    tension: float = 30.0  # N at the start
    tool_pos: tuple = (0.0, 0.0, 0.0)  # tool frame origin (the toe's tip, sole side, mid thickness), env frame
    tool_R: tuple = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))  # start rotation matrix (columns: tool axes)
    tool_boxes: tuple = ()  # (centre, size, colour), tool frame: x toward the head, z from sole to spine
    toe_points: tuple = ()  # the toe: a convex wedge, tool frame
    toe_color: Color = (0.85, 0.20, 0.12)
    tool_mass: float = 0.046
    blade_centre: tuple = (0.0, 0.0, 0.0)  # tool frame
    blade_size: tuple = (0.002, 0.011, 0.0036)
    blade_mass: float = 0.002
    blade_color: Color = (0.78, 0.80, 0.83)
    mu_carton: float = 0.4
    mu_strap: float = 0.3
    mu_tool: float = 0.6
    cut_n: float = 5.0  # N on the blade from one strap link ...
    cut_hold: int = 24  # ... for this many physics steps in a row (0.1 s at 240 Hz)
    watch_steps: int = 120  # physics steps after a cut over which the two cut ends are followed (0.5 s)
    dig_n: float = 30.0  # N between cutter and carton ...
    dig_hold: int = 24  # ... for this many physics steps in a row marks the carton

    def _check(self) -> None:
        super()._check()
        if self.mass is not None:
            raise ValueError(f"{self.name}: a StrapRig is a fixed asset (mass None); its bodies carry their own")

    # -- strap geometry ----------------------------------------------------------------------------------------
    @property
    def n_link(self) -> int:
        return 2 * (self.n_top + self.n_side)

    @property
    def half(self) -> tuple[float, float]:
        """Half-width and half-height of the strap's middle line (half a strap thickness outside the carton)."""
        return self.size[1] / 2 + self.strap_t / 2, self.size[2] / 2 + self.strap_t / 2

    def links(self, s: int) -> list:
        """Strap ``s``: (centre (env), rotation matrix, length) of every link, in order round the loop. Link axes:
        x along env x, z along the strap's run, y pointing out of the carton. The run goes over the top toward -y,
        down the right side, back under the carton toward +y, and up the left side."""
        a, b = self.half
        cx, cy, cz = self.strap_x[s], self.pos[1], self.pos[2]
        out = []
        runs = [(self.n_top, 2 * a, (cy + a, cz + b), 0.0), (self.n_side, 2 * b, (cy - a, cz + b), 1.0),
                (self.n_top, 2 * a, (cy - a, cz - b), 2.0), (self.n_side, 2 * b, (cy + a, cz - b), 3.0)]
        for n, length, (y0, z0), quarter in runs:
            ang = math.pi / 2 * quarter  # the run direction turns about env x by a quarter turn per corner
            d = np.array([0.0, -math.cos(ang), -math.sin(ang)])
            R = np.column_stack([(1.0, 0.0, 0.0), np.cross(d, (1.0, 0.0, 0.0)), d])
            for k in range(n):
                c = np.array([cx, y0, z0]) + d * (k + 0.5) * length / n
                out.append((c, R, length / n))
        return out

    @property
    def slide_joint(self) -> int:
        """The joint that is a sprung slide: between the two middle links under the carton (joint j joins link
        j - 1 to link j)."""
        return self.n_top + self.n_side + self.n_top // 2 + 1

    def top_joints(self) -> range:
        """Joints that lie on the carton's top (between two top links)."""
        return range(1, self.n_top)


@dataclass(frozen=True)
class Cut:
    strap: int
    joint: int
    point: np.ndarray  # env frame
    t: float  # s since the reset
    force: float  # N on the blade when it parted
    tcp_m: float  # distance from the cut to the tool point of the gripper not holding the cutter
    held_n: float = 0.0  # N: the least force with which that gripper pressed on this strap over the cut's 0.1 s
    held_m: float = 9.0  # m from the cut to the strap link it pressed hardest on
    end_move: tuple = (0.0, 0.0)  # how far each cut end has travelled since the cut (largest so far), m
    end_speed: tuple = (0.0, 0.0)  # fastest each cut end has moved, m/s
    watched: bool = False  # the watch window is over


@dataclass(frozen=True)
class StrapState(ObjectState):
    """The rig after a control step. ``pos`` is the carton; every position is in the env frame."""

    carton_pos: np.ndarray = None
    carton_R: np.ndarray = None
    carton_vel: np.ndarray = None
    tool_pos: np.ndarray = None  # the cutter body's origin (the toe's tip)
    tool_R: np.ndarray = None
    tool_vel: np.ndarray = None
    link_pos: tuple = ()  # per strap: (n_link, 3)
    link_speed: float = 0.0  # fastest strap link now, m/s
    tcp: dict = field(default_factory=dict)  # arm -> tool point
    cuts: tuple = ()
    peak_blade_n: float = 0.0
    peak_carton_n: float = 0.0  # largest force between the cutter and the carton so far
    dug: bool = False  # that force stayed at ``dig_n`` or more for ``dig_hold`` physics steps
    press_n: dict = field(default_factory=dict)  # arm -> force on each strap now (N)
    tension: tuple = ()  # per strap, N, from the slide's stretch


class Cutter:
    """The cut rule and the watch on the cut ends, fed one physics step at a time (NumPy only, so tests drive it)."""

    def __init__(self, spec: StrapRig):
        self.spec = spec
        self.reset()

    def reset(self) -> None:
        s = self.spec
        self.run = np.zeros((len(s.strap_x), s.n_link), int)
        self.cut = np.zeros((len(s.strap_x), s.n_link), bool)
        self.cuts: list[Cut] = []
        self._watch: list = []  # [cut index, steps left, start positions (2, 3), last positions (2, 3)]
        self.peak = 0.0
        self.peak_carton = 0.0
        self.dig_run = 0
        self.dug = False
        self.press_log: list = []  # the last ``cut_hold`` steps: {arm: (force per strap, hardest-pressed point per strap)}

    def carton(self, force: float) -> None:
        """One physics step of the cutter's force on the carton."""
        self.peak_carton = max(self.peak_carton, float(force))
        self.dig_run = self.dig_run + 1 if force >= self.spec.dig_n else 0
        self.dug = self.dug or self.dig_run >= self.spec.dig_hold

    def pressed(self, press: dict) -> None:
        """One physics step of the grippers' forces on the straps: {arm: (N per strap, point per strap)}."""
        self.press_log.append(press)
        del self.press_log[: -self.spec.cut_hold]

    def held(self, arm: str, strap: int, point) -> tuple:
        """(least force over the logged steps, distance from ``point`` to where it pressed last) for one arm."""
        if len(self.press_log) < self.spec.cut_hold:
            return 0.0, 9.0
        least = min(float(p[arm][0][strap]) for p in self.press_log)
        return least, float(np.linalg.norm(np.asarray(self.press_log[-1][arm][1][strap], float) - np.asarray(point, float)))

    def step(self, blade_n) -> list:
        """``blade_n``: (straps, links) force on the blade from each link (N). Returns the (strap, link) pairs whose
        press has lasted ``cut_hold`` steps; the caller picks the joint and calls :meth:`part`."""
        blade_n = np.asarray(blade_n, float).reshape(self.run.shape)
        self.peak = max(self.peak, float(blade_n.max(initial=0.0)))
        self.run = np.where(blade_n >= self.spec.cut_n, self.run + 1, 0)
        hit = [(int(a), int(b)) for a, b in zip(*np.nonzero(self.run >= self.spec.cut_hold))]
        for a, b in hit:
            self.run[a, b] = 0
        return hit

    def part(self, strap: int, joint: int, point, t: float, force: float, tcp_m: float, ends, held=(0.0, 9.0)) -> None:
        self.cut[strap, joint] = True
        self.cuts.append(Cut(strap, joint, np.asarray(point, float), t, force, tcp_m, float(held[0]), float(held[1])))
        ends = np.asarray(ends, float)
        self._watch.append([len(self.cuts) - 1, self.spec.watch_steps, ends.copy(), ends.copy()])

    def watch(self, link_pos, dt: float) -> None:
        """Follow the two links beside each fresh cut: ``link_pos`` is (straps, links, 3)."""
        link_pos = np.asarray(link_pos, float)
        for w in self._watch:
            i, left, start, last = w
            if left <= 0:
                continue
            c = self.cuts[i]
            now = np.array([link_pos[c.strap, (c.joint - 1) % self.spec.n_link], link_pos[c.strap, c.joint]])
            move = np.linalg.norm(now - start, axis=1)
            speed = np.linalg.norm(now - last, axis=1) / dt
            w[1], w[3] = left - 1, now
            self.cuts[i] = Cut(c.strap, c.joint, c.point, c.t, c.force, c.tcp_m, c.held_n, c.held_m,
                               tuple(float(max(a, b)) for a, b in zip(c.end_move, move)),
                               tuple(float(max(a, b)) for a, b in zip(c.end_speed, speed)), w[1] <= 0)


# -- spawning (Isaac Sim only) ----------------------------------------------------------------------------------

_SPECS: dict[str, StrapRig] = {}
_PATHS: dict[str, str] = {}  # rig name -> its root prim (the first spawned: env 0)


def hull_faces(points, eps: float = 1e-7) -> list:
    """Faces of the convex hull of a few points, each a list of indices wound counter-clockwise from outside."""
    p = np.asarray(points, float)
    n, centre, planes = len(p), p.mean(0), {}
    for i in range(n - 2):
        for j in range(i + 1, n - 1):
            for k in range(j + 1, n):
                nrm = np.cross(p[j] - p[i], p[k] - p[i])
                size = float(np.linalg.norm(nrm))
                if size < 1e-12:
                    continue
                nrm = nrm / size
                d = (p - p[i]) @ nrm
                if d.max() > eps and d.min() < -eps:
                    continue
                if d.max() > eps:
                    nrm = -nrm
                key = tuple(np.round(np.r_[nrm, nrm @ p[i]], 5))
                planes.setdefault(key, (nrm, set()))[1].update((i, j, k))
    faces = []
    for nrm, idx in planes.values():
        idx = sorted(idx)
        c = p[idx].mean(0)
        u = p[idx[0]] - c
        u /= np.linalg.norm(u)
        v = np.cross(nrm, u)
        order = [m for _, m in sorted(zip([math.atan2((p[m] - c) @ v, (p[m] - c) @ u) for m in idx], idx))]
        faces.append(order[::-1] if nrm @ (c - centre) < 0 else order)
    return faces


def _spawn(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics, Vt
    from yam_arena.station import linear_rgb

    s = _SPECS[cfg.rig_name]
    _PATHS.setdefault(s.name, prim_path)
    stage = get_current_stage()
    dbl = UsdGeom.XformOp.PrecisionDouble
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=dbl).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    mats = {}
    for key, mu in (("carton", s.mu_carton), ("strap", s.mu_strap), ("tool", s.mu_tool)):
        mats[key] = f"{prim_path}/Looks/phys_{key}"
        m = sim_utils.RigidBodyMaterialCfg(static_friction=mu, dynamic_friction=mu)
        m.func(mats[key], m)
    looks: dict = {}

    def look(color) -> str:
        if color not in looks:
            looks[color] = f"{prim_path}/Looks/c{len(looks)}"
            vis = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=0.6)
            vis.func(looks[color], vis)
        return looks[color]

    def collider(prim, mat, offset=0.002, patch=0.0) -> None:
        UsdPhysics.CollisionAPI.Apply(prim)
        col = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        col.CreateContactOffsetAttr(offset)
        col.CreateRestOffsetAttr(0.0)
        if patch:
            col.CreateTorsionalPatchRadiusAttr(float(patch))
            col.CreateMinTorsionalPatchRadiusAttr(float(patch))
        bind_physics_material(str(prim.GetPath()), mats[mat], stage=stage)

    def boxes(path, specs, mat, offset=0.002, patch=0.0) -> None:
        for i, (centre, size, color) in enumerate(specs):
            cube = UsdGeom.Cube.Define(stage, f"{path}/box_{i}")
            cube.CreateSizeAttr(1.0)
            cube.AddTranslateOp().Set(Gf.Vec3d(*centre))
            cube.AddScaleOp().Set(Gf.Vec3f(*size))
            collider(cube.GetPrim(), mat, offset, patch)
            bind_visual_material(str(cube.GetPath()), look(tuple(color)), stage=stage)

    def body(name, pos, mass, inertia=None, kinematic=False, damping=(0.0, 0.05), q=(1.0, 0.0, 0.0, 0.0)) -> str:
        path = f"{prim_path}/{name}"
        xf = UsdGeom.Xform.Define(stage, path)
        xf.AddTranslateOp(precision=dbl).Set(Gf.Vec3d(*map(float, pos)))
        xf.AddOrientOp(precision=dbl).Set(Gf.Quatd(*map(float, q)))
        prim = xf.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim).CreateKinematicEnabledAttr(kinematic)
        m = UsdPhysics.MassAPI.Apply(prim)
        m.CreateMassAttr(mass)
        if inertia is not None:
            m.CreateDiagonalInertiaAttr(Gf.Vec3f(inertia, inertia, inertia))
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateMaxDepenetrationVelocityAttr(1.0)
        rb.CreateSolverPositionIterationCountAttr(int(os.environ.get("STRAP_ITERS", 128)))
        rb.CreateSolverVelocityIterationCountAttr(4)
        rb.CreateLinearDampingAttr(damping[0])
        rb.CreateAngularDampingAttr(damping[1])
        rb.CreateSleepThresholdAttr(0.0)
        return path

    def joint(name, cls, b0, b1, p0, p1, rot0=(1.0, 0.0, 0.0, 0.0)):
        j = cls.Define(stage, f"{prim_path}/{name}")
        j.CreateBody0Rel().SetTargets([Sdf.Path(b0)])
        j.CreateBody1Rel().SetTargets([Sdf.Path(b1)])
        j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, p0)))
        j.CreateLocalPos1Attr(Gf.Vec3f(*map(float, p1)))
        j.CreateLocalRot0Attr(Gf.Quatf(*map(float, rot0)))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateExcludeFromArticulationAttr(True)
        j.CreateJointEnabledAttr(True)
        return j

    # what does not move
    if s.fixed_boxes:
        fixed = body("fixed", (0.0, 0.0, 0.0), 1.0, kinematic=True)
        boxes(fixed, s.fixed_boxes, "carton", 0.003)
    # the carton
    carton = body("carton", s.pos, s.carton_mass, damping=(0.0, 0.05))
    boxes(carton, [((0.0, 0.0, 0.0), s.size, s.carton_color)], "carton", 0.002)
    # the straps
    for n in range(len(s.strap_x)):
        links = s.links(n)
        paths = []
        for k, (c, R, length) in enumerate(links):
            seg = body(f"strap{n}_{k:02d}", c, s.link_mass, s.link_inertia, damping=(0.5, 1.0), q=matrix_to_wxyz(R))
            boxes(seg, [((0.0, 0.0, 0.0), (s.strap_w, s.strap_t, length), s.strap_colors[k % len(s.strap_colors)])],
                  "strap", 0.0015)
            PhysxSchema.PhysxContactReportAPI.Apply(stage.GetPrimAtPath(seg)).CreateThresholdAttr(0.0)
            paths.append(seg)
        for k in range(len(links)):
            prev, (_, Rp, lp), (_, Rn, ln) = paths[k - 1], links[k - 1], links[k]
            rel = matrix_to_wxyz(Rp.T @ Rn)
            if k == s.slide_joint:
                j = joint(f"strap{n}_joint_{k:02d}", UsdPhysics.PrismaticJoint, prev, paths[k], (0.0, 0.0, lp / 2),
                          (0.0, 0.0, -ln / 2), rel)
                j.CreateAxisAttr("Z")
                j.CreateLowerLimitAttr(-0.04)
                j.CreateUpperLimitAttr(0.10)
                drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "linear")
                drive.CreateTypeAttr("force")
                drive.CreateStiffnessAttr(s.spring_k)
                drive.CreateDampingAttr(s.spring_c)
                drive.CreateTargetPositionAttr(-(s.tension / s.spring_k + s.slack))
            else:
                joint(f"strap{n}_joint_{k:02d}", UsdPhysics.SphericalJoint, prev, paths[k], (0.0, 0.0, lp / 2),
                      (0.0, 0.0, -ln / 2), rel)
    # the cutter
    R0 = np.asarray(s.tool_R, float)  # columns are the tool axes in the env frame
    q0 = matrix_to_wxyz(R0)
    tool = body("cutter", s.tool_pos, s.tool_mass, q=q0)
    boxes(tool, s.tool_boxes, "tool", 0.0015, patch=0.006)
    if s.toe_points:
        pts = np.asarray(s.toe_points, float)
        faces = hull_faces(pts)
        mesh = UsdGeom.Mesh.Define(stage, f"{tool}/toe")
        mesh.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*(float(v) for v in p)) for p in pts]))
        mesh.CreateFaceVertexCountsAttr(Vt.IntArray([len(f) for f in faces]))
        mesh.CreateFaceVertexIndicesAttr(Vt.IntArray([int(i) for f in faces for i in f]))
        mesh.CreateSubdivisionSchemeAttr("none")
        mesh.CreateDoubleSidedAttr(True)
        UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("convexHull")
        collider(mesh.GetPrim(), "tool", 0.001)
        bind_visual_material(str(mesh.GetPath()), look(tuple(s.toe_color)), stage=stage)
    blade = body("blade", tuple(np.asarray(s.tool_pos, float) + R0 @ np.asarray(s.blade_centre, float)), s.blade_mass, q=q0)
    boxes(blade, [((0.0, 0.0, 0.0), s.blade_size, s.blade_color)], "tool", 0.001)
    for path in (blade, tool):
        report = PhysxSchema.PhysxContactReportAPI.Apply(stage.GetPrimAtPath(path))
        report.CreateThresholdAttr(0.0)
    joint("blade_fixed", UsdPhysics.FixedJoint, tool, blade, s.blade_centre, (0.0, 0.0, 0.0))
    return root.GetPrim()


def _arena_object(spec: StrapRig):
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    @configclass
    class _Cfg(SpawnerCfg):
        func: object = clone(_spawn)
        rig_name: str = ""

    return Object(name=spec.name, object_type=ObjectType.BASE, spawner_cfg=_Cfg(rig_name=spec.name),
                  initial_pose=Pose(position_xyz=(0.0, 0.0, 0.0), rotation_xyzw=(0.0, 0.0, 0.0, 1.0)))


# -- runtime ----------------------------------------------------------------------------------------------------


def _arr(x) -> np.ndarray:
    x = getattr(x, "torch", x)
    if hasattr(x, "detach"):
        return x.detach().cpu().numpy().astype(np.float64)
    return np.asarray(x.numpy() if hasattr(x, "numpy") else x, dtype=np.float64)


class _Runtime:
    """PhysX views and the cut rule for one rig (env 0)."""

    def __init__(self, base, spec: StrapRig):
        from isaaclab.sim.utils.stage import get_current_stage
        from isaaclab_physx.physics import PhysxManager

        self.spec, self.base = spec, base
        root = _PATHS[spec.name]
        self.stage = get_current_stage()
        self.ns, self.nl = len(spec.strap_x), spec.n_link
        self.link_names = [f"strap{n}_{k:02d}" for n in range(self.ns) for k in range(self.nl)]
        self.names = self.link_names + ["carton", "cutter", "blade"]
        view = PhysxManager.get_physics_sim_view()
        self.bodies = view.create_rigid_body_view([f"{root}/{n}" for n in self.names])
        self.contact = view.create_rigid_contact_view([f"{root}/blade"],
                                                      filter_patterns=[[f"{root}/{n}" for n in self.link_names]],
                                                      max_contact_data_count=512)
        self.contact_carton = view.create_rigid_contact_view([f"{root}/cutter"], filter_patterns=[[f"{root}/carton"]],
                                                             max_contact_data_count=64)
        self.links_robot = []  # (arm, prim path) of every robot rigid body
        for arm in ARMS:
            paths = base.scene[arm].root_physx_view.link_paths
            for p in (paths[0] if paths and isinstance(paths[0], (list, tuple)) else paths):
                self.links_robot.append((arm, p))
        robot = [p for _, p in self.links_robot]
        self.contact_hand = view.create_rigid_contact_view([f"{root}/{n}" for n in self.link_names],
                                                           filter_patterns=[robot] * len(self.link_names),
                                                           max_contact_data_count=1024)
        self.joints = [[self.stage.GetPrimAtPath(f"{root}/strap{n}_joint_{j:02d}") for j in range(self.nl)]
                       for n in range(self.ns)]
        self.cutter = Cutter(spec)
        self.dt = float(base.physics_dt)
        self.t = 0.0
        self.start = _arr(self.bodies.get_transforms()).copy()
        self.origin = _arr(base.scene.env_origins)[0]
        self.debug = bool(os.environ.get("STRAP_DEBUG"))
        self.step_peak = self.step_carton = 0.0

    def tcps(self) -> dict[str, np.ndarray]:
        out = {}
        for arm in ARMS:
            asset = self.base.scene[arm]
            i = asset.find_bodies("gripper")[0][0]
            pose = _arr(asset.data.body_link_pose_w)[0, i]
            out[arm] = pose[:3] - self.origin + quat_to_matrix(pose[3:7])[:, 2] * TCP_OFFSET
        return out

    def _links(self, tr) -> np.ndarray:
        return (tr[: self.ns * self.nl, :3] - self.origin).reshape(self.ns, self.nl, 3)

    def physics_step(self) -> None:
        """One physics step: read the last step's blade contacts, apply the cut rule, follow fresh cut ends."""
        from pxr import UsdPhysics

        s = self.spec
        self.t += self.dt
        fm = _arr(self.contact.get_contact_force_matrix(dt=self.dt)).reshape(-1, 3)
        blade_n = np.linalg.norm(fm[: self.ns * self.nl], axis=1).reshape(self.ns, self.nl)
        fc = float(np.linalg.norm(_arr(self.contact_carton.get_contact_force_matrix(dt=self.dt)).reshape(-1, 3)[0]))
        self.cutter.carton(fc)
        self.step_peak, self.step_carton = max(self.step_peak, float(blade_n.max())), max(self.step_carton, fc)
        tr = _arr(self.bodies.get_transforms())
        links = self._links(tr)
        fh = np.linalg.norm(_arr(self.contact_hand.get_contact_force_matrix(dt=self.dt)).reshape(self.ns, self.nl, len(self.links_robot), 3), axis=3)
        press = {}
        for arm in ARMS:
            cols = [i for i, (a, _) in enumerate(self.links_robot) if a == arm]
            per_link = fh[:, :, cols].sum(axis=2)  # (straps, links)
            press[arm] = (per_link.sum(axis=1), [links[n, int(np.argmax(per_link[n]))].copy() for n in range(self.ns)])
        self.cutter.pressed(press)
        self.press_now = {arm: np.round(press[arm][0], 2) for arm in ARMS}
        for n, k in self.cutter.step(blade_n):
            blade = tr[self.ns * self.nl + 2][:3] - self.origin
            seg = tr[n * self.nl + k]
            axis = quat_to_matrix(seg[3:7])[:, 2] * s.links(n)[k][2] / 2
            ends = {k: seg[:3] - self.origin - axis, (k + 1) % self.nl: seg[:3] - self.origin + axis}
            j = min(ends, key=lambda q: np.linalg.norm(ends[q] - blade))
            if self.cutter.cut[n, j] or j == s.slide_joint:
                j = next(q for q in ends if q != j)
            if self.cutter.cut[n, j] or j == s.slide_joint:
                continue
            tcp = self.tcps()
            tool = tr[self.ns * self.nl + 1][:3] - self.origin
            other = max(ARMS, key=lambda a: np.linalg.norm(tcp[a] - tool))
            UsdPhysics.Joint(self.joints[n][j]).GetJointEnabledAttr().Set(False)
            pair = [links[n, (j - 1) % self.nl], links[n, j]]
            held = self.cutter.held(other, n, ends[j])
            self.cutter.part(n, j, ends[j], self.t, float(blade_n[n, k]), float(np.linalg.norm(tcp[other] - ends[j])), pair, held)
            print(f"[strapkit] strap {n} cut at joint {j}, {np.round(ends[j], 3)}, t={self.t:.2f}s, {blade_n[n, k]:.1f} N on "
                  f"the blade; the {other} gripper pressed on it with at least {held[0]:.2f} N, {held[1] * 100:.1f} cm from the cut",
                  flush=True)
        self.cutter.watch(links, self.dt)

    def reset(self) -> None:
        """Put every body back where it started, at rest, and rejoin the straps."""
        import torch
        from pxr import UsdPhysics

        for row in self.joints:
            for prim in row:
                UsdPhysics.Joint(prim).GetJointEnabledAttr().Set(True)
        idx = np.arange(len(self.names), dtype=np.int32)
        cur = self.bodies.get_transforms()
        if hasattr(cur, "detach"):
            dev = cur.device
            self.bodies.set_transforms(torch.tensor(self.start, dtype=torch.float32, device=dev),
                                       torch.tensor(idx, device=dev))
            self.bodies.set_velocities(torch.zeros((len(idx), 6), dtype=torch.float32, device=dev),
                                       torch.tensor(idx, device=dev))
        else:
            import warp as wp

            dev = getattr(cur, "device", None)
            self.bodies.set_transforms(wp.array(self.start.astype(np.float32), device=dev),
                                       wp.array(idx, dtype=wp.int32, device=dev))
            self.bodies.set_velocities(wp.zeros((len(idx), 6), dtype=wp.float32, device=dev),
                                       wp.array(idx, dtype=wp.int32, device=dev))
        self.cutter.reset()
        self.t = 0.0

    def state(self) -> StrapState:
        s = self.spec
        tr = _arr(self.bodies.get_transforms())
        vel = _arr(self.bodies.get_velocities())
        links = self._links(tr)
        i0 = self.ns * self.nl
        carton, tool = tr[i0], tr[i0 + 1]
        cp, tp = carton[:3] - self.origin, tool[:3] - self.origin
        tension = []
        for n in range(self.ns):
            j = s.slide_joint
            gap = float(np.linalg.norm(links[n, j] - links[n, j - 1])) - (s.links(n)[j][2] + s.links(n)[j - 1][2]) / 2
            tension.append(max(0.0, s.tension + s.spring_k * (gap + s.slack)))
        speeds = np.linalg.norm(vel[:i0, :3], axis=1)
        speed = float(speeds.max())
        if self.debug:
            t = self.tcps()
            fast = int(np.argmax(speeds))
            gaps = [float(np.linalg.norm(links[n, s.slide_joint] - links[n, s.slide_joint - 1])) - (s.links(n)[s.slide_joint][2] + s.links(n)[s.slide_joint - 1][2]) / 2 for n in range(self.ns)]
            print(f"[strapdbg] fastest link strap {fast // self.nl} link {fast % self.nl} at {np.round(links[fast // self.nl, fast % self.nl], 4)} "
                  f"slide {np.round(np.array(gaps) * 1000, 2)} mm top z {np.round((links[0, :s.n_top, 2] - s.pos[2] - s.size[2] / 2) * 1000, 2)} mm "
                  f"top x {np.round((links[0, :s.n_top, 0] - s.strap_x[0]) * 1000, 1)}", flush=True)
            print(f"[strapkit] t={self.t:5.2f} tool {np.round(tp, 3)} blade peak {self.step_peak:5.2f} N carton "
                  f"{self.step_carton:5.2f} N (max {self.cutter.peak_carton:5.2f}) tension {np.round(tension, 1)} "
                  f"carton {np.round(cp, 3)} tcpL {np.round(t['left_robot'], 3)} tcpR {np.round(t['right_robot'], 3)} "
                  f"link speed {speed:.2f} press {getattr(self, 'press_now', {})} dug {self.cutter.dug} "
                  f"cuts {[(c.strap, c.joint, round(c.held_n, 2), round(c.held_m, 3), np.round(c.end_move, 3).tolist()) for c in self.cutter.cuts]}",
                  flush=True)
        self.step_peak = self.step_carton = 0.0
        p = tr[:, :3] - self.origin
        return StrapState(pos=cp, quat_xyzw=carton[3:7], lin_vel=vel[i0, :3], ang_vel=vel[i0, 3:], low=p.min(0),
                          high=p.max(0), carton_pos=cp, carton_R=quat_to_matrix(carton[3:7]), carton_vel=vel[i0],
                          tool_pos=tp, tool_R=quat_to_matrix(tool[3:7]), tool_vel=vel[i0 + 1],
                          link_pos=tuple(links[n].copy() for n in range(self.ns)), link_speed=speed, tcp=self.tcps(),
                          cuts=tuple(self.cutter.cuts), peak_blade_n=self.cutter.peak,
                          peak_carton_n=self.cutter.peak_carton, dug=self.cutter.dug,
                          press_n={a: tuple(map(float, v)) for a, v in getattr(self, "press_now", {}).items()},
                          tension=tuple(tension))


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


# -- hooks ------------------------------------------------------------------------------------------------------

_RESET: list = []


def on_reset(fn):
    _RESET.append(fn)
    return fn


def _patch_scene(scene) -> None:
    if getattr(scene, "_strapkit", False):
        return
    task_object, object_states = scene.task_object, scene.object_states

    def _task_object(spec):
        if isinstance(spec, StrapRig):
            _SPECS[spec.name] = spec
            return _arena_object(spec)
        return task_object(spec)

    def _object_states(environment, names):
        plain = [n for n in names if n not in _SPECS]
        out = object_states(environment, plain) if plain else {}
        out.update({n: _runtime(environment.unwrapped, n).state() for n in names if n in _SPECS})
        return {n: out[n] for n in names}

    scene.task_object, scene.object_states, scene._strapkit = _task_object, _object_states, True


def _hook_physics(environment) -> None:
    base = environment.unwrapped
    isc = base.scene
    if getattr(isc, "_strapkit", False):
        return
    write = isc.write_data_to_sim

    def write_data_to_sim(*a, **k):
        for name in _SPECS:
            _runtime(base, name).physics_step()
        return write(*a, **k)

    isc.write_data_to_sim, isc._strapkit = write_data_to_sim, True


def _patch_embodiment(module) -> None:
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_strapkit", False):
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
            _runtime(self._base, name).cutter.reset()  # the settle steps are not part of the trial
            _runtime(self._base, name).t = 0.0
        for fn in _RESET:
            fn(self._env)
        return obs

    cls._ensure_sim, cls.reset, cls._strapkit = _ensure_sim, _reset, True


def _patch_demo(demo) -> None:
    if getattr(demo, "_strapkit", False):
        return
    grip_rotation = demo.grip_rotation

    def _grip_rotation(yaw=0.0):
        return np.asarray(yaw, float) if np.ndim(yaw) == 2 else grip_rotation(yaw)

    demo.grip_rotation, demo._strapkit = _grip_rotation, True


_TARGETS = {"yam_arena.scene": _patch_scene, "yam_arena.inspect_embodiment": _patch_embodiment,
            "yam_arena.demo": _patch_demo}


class _Finder(importlib.abc.MetaPathFinder):
    """Patches each target module right after it is first imported."""

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
