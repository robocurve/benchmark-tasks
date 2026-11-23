"""kitchenkit: helpers for yam_arena kitchen task files.

Importing this module is pure Python (numpy and yam_arena.task_file only), as a task file must be. The
Isaac-side code runs later, when yam_arena loads its scene and embodiment modules:

- ``Part``: a rigid object built from several boxes and cylinders (a pot, a bowl, a ladle). It goes in
  ``OBJECTS`` like any ``Box``; kitchenkit gives ``yam_arena.scene.task_object`` a spawner for it.
- ``ring`` / ``vessel``: round containers made of a floor disk and a ring of wall boxes.
- ``STATE``: robot readings captured before each task step (finger pad points per arm, and each Part's root
  pose), so a task's ``score()`` can tell which loose pieces a gripper touched.
- ``watch_contact(a, b)``: also record the contact force between Parts ``a`` and ``b`` each step
  (``STATE.contacts[(a, b)]``, newtons: the mean over the step's physics substeps), read from PhysX's contact
  reports.

Nothing in yam_arena is edited: kitchenkit wraps two functions when their modules are imported.
"""

from __future__ import annotations

import importlib.abc
import importlib.machinery
import math
import sys
from dataclasses import dataclass, field

import numpy as np
from yam_arena import task_file

# -- geometry ----------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class Geom:
    """One collision-and-visual primitive of a Part, in the Part's own frame (m)."""

    kind: str  # "box", "cyl" (an upright cylinder in the geom's frame) or "sph"
    center: tuple[float, float, float]
    size: tuple  # box: (x, y, z) edge lengths; cyl: (radius, height); sph: (radius,)
    quat_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    color: tuple[float, float, float] | None = None
    contact: tuple[float, float] | None = None  # (contact offset, rest offset) for this geom; None: the Part's
    torsion: float | None = None  # contact patch radius (m) for torsional friction (PhysX); None: none
    friction: tuple[float, float] | None = None  # (static, dynamic) for this geom; None: the Part's material
    collide: bool = True  # False: visual only (no collision shape), e.g. flames or markings
    roughness: float | None = None  # visual finish, 0 (glossy, e.g. water) to 1 (matte); None: 0.45


@dataclass(frozen=True)
class Part(task_file.Box):
    """A rigid body made of ``geoms`` (fixed in place when ``mass`` is None). ``pos`` is the Part frame's origin
    in the world; ``size`` is filled in with the geoms' extent."""

    geoms: tuple[Geom, ...] = ()
    friction: tuple[float, float] | None = None  # (static, dynamic); None keeps yam_arena's grippy default
    rigid: tuple = ()  # extra PhysxRigidBodyPropertiesCfg fields, as (name, value) pairs
    contact: tuple[float, float] | None = None  # (contact_offset, rest_offset) m; None keeps yam_arena's (0.005, 0)

    def _check(self) -> None:
        super()._check()
        if not self.geoms:
            raise ValueError(f"{self.name}: a Part needs at least one geom")
        for g in self.geoms:
            if g.kind not in ("box", "cyl", "sph"):
                raise ValueError(f"{self.name}: geom kind must be 'box', 'cyl' or 'sph', got {g.kind!r}")


_PARTS: dict[str, Part] = {}


def quat_z(angle: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(angle / 2), math.cos(angle / 2))


def quat_axis(axis, angle: float) -> tuple[float, float, float, float]:
    a = np.asarray(axis, dtype=float)
    a = a / np.linalg.norm(a)
    s = math.sin(angle / 2)
    return (float(a[0] * s), float(a[1] * s), float(a[2] * s), math.cos(angle / 2))


def quat_mul(q1, q2) -> tuple[float, float, float, float]:
    """Hamilton product of two xyzw quaternions (q1 * q2: q2 applied first, in q1's frame)."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def rot_matrix(q_xyzw) -> np.ndarray:
    x, y, z, w = q_xyzw
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def ring(radius_in: float, thickness: float, height: float, z0: float, n: int = 16, color=None) -> list[Geom]:
    """``n`` wall boxes standing on z0 around a circle of inner radius ``radius_in`` (part frame)."""
    r_mid = radius_in + thickness / 2
    # each box spans its sector's chord at the outer face, so neighbouring boxes overlap and leave no gap
    width = 2 * (radius_in + thickness) * math.tan(math.pi / n) + 0.001
    out = []
    for i in range(n):
        a = 2 * math.pi * i / n
        out.append(
            Geom("box", (r_mid * math.cos(a), r_mid * math.sin(a), z0 + height / 2), (thickness, width, height),
                 quat_z(a), color)
        )
    return out


def disk(radius: float, thickness: float, z0: float, k: int = 9, color=None) -> list[Geom]:
    """A flat round plate of radius ``radius`` from ``k`` parallel slats, each cut to its chord, so the plate stays
    inside the circle (boxes only: PhysX on the GPU handles box colliders on moving bodies robustly)."""
    w = 2 * radius / k
    out = []
    for i in range(k):
        y = -radius + w * (i + 0.5)
        half = math.sqrt(max(radius**2 - y**2, 0.0))  # chord at the slat's centre line; corners stick out <= w/2
        out.append(Geom("box", (0.0, y, z0 + thickness / 2), (2 * half, w + 0.0005, thickness), color=color))
    return out


def vessel(radius_in: float, wall: float, height: float, floor: float, n: int = 16, color=None) -> list[Geom]:
    """An open round container whose floor's underside is at z = 0: a slatted floor plate of the inner radius, and
    a ring of walls running the full height so the floor's edge is sealed."""
    return [*disk(radius_in, floor, 0.0, k=11, color=color), *ring(radius_in, wall, height, 0.0, n, color)]


def part(name: str, pos, geoms, *, mass: float | None = None, color=(0.7, 0.7, 0.7), yaw: float = 0.0,
         friction=None, rigid: dict | None = None, contact=None) -> Part:
    """A Part, registered so the spawner can find its geoms by name."""
    lo, hi = np.full(3, np.inf), np.full(3, -np.inf)
    for g in geoms:
        half = (np.array([g.size[0], g.size[0], g.size[1] / 2]) if g.kind == "cyl" else
                np.full(3, g.size[0]) if g.kind == "sph" else np.asarray(g.size) / 2)
        r = np.abs(rot_matrix(g.quat_xyzw))
        ext = r @ half
        lo, hi = np.minimum(lo, np.asarray(g.center) - ext), np.maximum(hi, np.asarray(g.center) + ext)
    p = Part(name=name, pos=tuple(float(v) for v in pos), color=tuple(color), mass=mass, yaw=yaw,
             size=tuple(float(v) for v in hi - lo), geoms=tuple(geoms), friction=friction,
             rigid=tuple(sorted((rigid or {}).items())), contact=contact)
    _PARTS[name] = p
    return p


@dataclass(frozen=True)
class Pile:
    """Many identical loose spheres (chickpeas, beads), spawned together with the Part ``anchor`` and kept out of
    the task's object list: yam_arena reads every task object separately (about six GPU reads each, every step),
    which for a hundred-odd spheres costs seconds per step. A pile is read in one batched call per step instead
    (``STATE.piles[name]``: (n, 3) centres, env-local m) and put back where it started after every reset."""

    name: str
    anchor: str
    positions: tuple  # sphere centres, env-local (m)
    radius: float
    mass: float
    color: tuple = (0.8, 0.7, 0.5)
    friction: tuple | None = None
    rigid: tuple = ()
    contact: tuple | None = None
    radii: tuple | None = None  # per-sphere radii (m); None: all ``radius``
    masses: tuple | None = None  # per-sphere masses (kg); None: all ``mass``


_PILES: dict[str, Pile] = {}


def pile(name: str, anchor: str, positions, radius: float, mass: float, *, color=(0.8, 0.7, 0.5), friction=None,
         rigid: dict | None = None, contact=None, radii=None, masses=None) -> Pile:
    """A Pile, registered so the anchor Part's spawner creates it. ``radii``/``masses`` (one per sphere) override
    ``radius``/``mass``."""
    p = Pile(name=name, anchor=anchor, positions=tuple(tuple(float(v) for v in q) for q in positions),
             radius=float(radius), mass=float(mass), color=tuple(color), friction=friction,
             rigid=tuple(sorted((rigid or {}).items())), contact=contact,
             radii=None if radii is None else tuple(float(r) for r in radii),
             masses=None if masses is None else tuple(float(m) for m in masses))
    _PILES[name] = p
    return p


def _spawn_pile(root: str, spec: Pile, stage) -> None:
    from isaaclab.sim import schemas
    from isaaclab.sim.spawners.materials.physics_materials import spawn_physics_material
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material, create_prim
    from yam_arena.scene import linear_rgb, sim_utils
    from yam_arena import scene

    create_prim(root, "Xform", stage=stage)
    phys = f"{root}/physics_material"
    material = (scene._grippy() if spec.friction is None else
                sim_utils.RigidBodyMaterialCfg(static_friction=spec.friction[0], dynamic_friction=spec.friction[1]))
    spawn_physics_material(phys, material, stage=stage)
    look_path = f"{root}/look"
    look = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(spec.color), roughness=0.45)
    look.func(look_path, look)
    collision = (scene._contact_props() if spec.contact is None else
                 scene.PhysxCollisionPropertiesCfg(contact_offset=spec.contact[0], rest_offset=spec.contact[1]))
    rigid = scene.PhysxRigidBodyPropertiesCfg(**dict(spec.rigid))
    for i, p in enumerate(spec.positions):
        path = f"{root}/{spec.name}_{i:03d}"
        r = spec.radius if spec.radii is None else spec.radii[i]
        mass = sim_utils.MassPropertiesCfg(mass=spec.mass if spec.masses is None else spec.masses[i])
        create_prim(path, "Sphere", translation=p, attributes={"radius": r}, stage=stage)
        schemas.define_collision_properties(path, collision, stage=stage)
        schemas.define_rigid_body_properties(path, rigid, stage=stage)
        schemas.define_mass_properties(path, mass, stage=stage)
        bind_visual_material(path, look_path, stage=stage)
        bind_physics_material(path, phys, stage=stage)


_VIEWS: dict = {}  # pile name -> (sim view, rigid body view, index of each sphere in the view)


def _pile_view(env, name: str):
    from isaaclab_physx.physics import PhysxManager

    sim_view = PhysxManager.get_physics_sim_view()
    cached = _VIEWS.get(name)
    if cached is not None and cached[0] is sim_view:
        return cached[1], cached[2]
    root = f"{env.unwrapped.scene.env_prim_paths[0]}/{name}"
    view = sim_view.create_rigid_body_view(f"{root}/{name}_*")
    paths = list(view.prim_paths)
    order = np.array([paths.index(f"{root}/{name}_{i:03d}") for i in range(len(_PILES[name].positions))])
    _VIEWS[name] = (sim_view, view, order)
    return view, order


def reset_piles(env) -> None:
    """Put every pile's spheres back where they started, at rest."""
    import warp as wp

    base = env.unwrapped
    origin = _np(base.scene.env_origins)[0]
    for name, spec in _PILES.items():
        view, order = _pile_view(env, name)
        n = len(spec.positions)
        tf = np.zeros((n, 7), dtype=np.float32)
        tf[order, :3] = np.asarray(spec.positions) + origin
        tf[:, 6] = 1.0
        dev = str(base.device)
        idx = wp.array(np.arange(n, dtype=np.int32), dtype=wp.int32, device=dev)
        view.set_transforms(wp.array(tf, dtype=wp.float32, device=dev), indices=idx)
        view.set_velocities(wp.zeros((n, 6), dtype=wp.float32, device=dev), indices=idx)


def read_piles(env) -> dict:
    base = env.unwrapped
    origin = _np(base.scene.env_origins)[0]
    out = {}
    for name in _PILES:
        view, order = _pile_view(env, name)
        tf = view.get_transforms()
        tf = tf.numpy() if hasattr(tf, "numpy") else _np(tf)
        out[name] = tf.reshape(-1, 7)[order, :3].astype(np.float64) - origin
    return out


# -- per-step robot readings ----------------------------------------------------------------------------------

ARMS = ("left_robot", "right_robot")
FINGER_LINKS = ("tip_left", "tip_right")


@dataclass
class _State:
    step: int = -1
    fingers: dict = field(default_factory=dict)  # arm -> (k, 3) points on the finger pads, world frame (m)
    poses: dict = field(default_factory=dict)  # part name -> (pos (3,), quat_xyzw (4,)) of the Part's root frame
    piles: dict = field(default_factory=dict)  # pile name -> (n, 3) sphere centres
    contacts: dict = field(default_factory=dict)  # (part a, part b) -> contact force between them (N)
    links: dict = field(default_factory=dict)  # arm -> (n, 3) origins of the arm's links, world frame (m), if tracked
    error: str | None = None

    def clear(self) -> None:
        self.step, self.fingers, self.poses, self.piles, self.contacts, self.error = -1, {}, {}, {}, {}, None
        self.links = {}


STATE = _State()
RESET_CALLBACKS: list = []  # called with no arguments when an episode resets
_TRACK_LINKS = [False]


def track_links() -> None:
    """Record the origins of every link of both arms in ``STATE.links`` each step (for keep-out rules)."""
    _TRACK_LINKS[0] = True


def _np(t) -> np.ndarray:
    t = getattr(t, "torch", t)
    return t.detach().cpu().numpy().astype(np.float64)


# Finger pads in the TCP frame (x: closing axis, z: approach, toward the tips). Measured in the simulator: each
# pad's inner face sits at x = joint7 (left finger) and x = -joint8 (right finger), both 0 when closed and
# -0.047 m open; pads run from about 3 cm behind the TCP to PAD_TIP_PAST_TCP (13 mm) past it.
PAD_Y = (-0.010, 0.0, 0.010)
PAD_Z = (-0.030, -0.015, 0.0, 0.013)
PAD_OUT = 0.004  # sample points sit this far behind each pad's inner face, inside the finger


def finger_points(q6, j7: float, j8: float, base) -> np.ndarray:
    """Points on both finger pads (world frame, env-local m) for arm joints ``q6`` and finger joints."""
    from yam_arena import ik

    tcp, r = ik.tcp_pose(np.asarray(q6, dtype=float), base)
    pts = []
    for x in (j7 - PAD_OUT, -j8 + PAD_OUT):
        for y in PAD_Y:
            for z in PAD_Z:
                pts.append(tcp + r @ np.array([x, y, z]))
    return np.array(pts)


_CONTACT_PAIRS: list[tuple[str, str]] = []


def watch_contact(a: str, b: str) -> None:
    """Record the contact force between Parts ``a`` and ``b`` (both with a mass) in ``STATE.contacts`` each step."""
    if (a, b) not in _CONTACT_PAIRS:
        _CONTACT_PAIRS.append((a, b))


def _contact_view(env, a: str, b: str):
    from isaaclab_physx.physics import PhysxManager

    sim_view = PhysxManager.get_physics_sim_view()
    key = f"contact:{a}:{b}"
    cached = _VIEWS.get(key)
    if cached is not None and cached[0] is sim_view:
        return cached[1]
    root = env.unwrapped.scene.env_prim_paths[0]
    view = sim_view.create_rigid_contact_view([f"{root}/{a}"], filter_patterns=[[f"{root}/{b}"]],
                                              max_contact_data_count=64)
    _VIEWS[key] = (sim_view, view, None)
    return view


_ACC: dict = {}  # pair -> [sum of forces, number of substeps] since the last capture


def _hook_substeps(env) -> None:
    """Read the watched contact forces after every physics substep (the simulator's step), so a control step's
    force is the mean over its substeps rather than one sample."""
    sim = env.unwrapped.sim
    if getattr(sim, "_kitchenkit_contacts", False):
        return
    step = sim.step

    def step_(*a, **k):
        out = step(*a, **k)
        try:
            for pair, f in read_contacts(env).items():
                acc = _ACC.setdefault(pair, [0.0, 0])
                acc[0] += f
                acc[1] += 1
        except Exception as exc:  # noqa: BLE001 - keep stepping; capture() falls back to one sample
            if STATE.error is None:
                STATE.error = repr(exc)
                print(f"kitchenkit: contact read failed: {exc!r}", flush=True)
        return out

    sim.step, sim._kitchenkit_contacts = step_, True


def mean_contacts(env) -> dict:
    """Mean contact force per watched pair over the substeps since the last call (one sample if there were none)."""
    _hook_substeps(env)
    out, last = {}, None
    for pair in _CONTACT_PAIRS:
        acc = _ACC.get(pair)
        if acc and acc[1] > 0:
            out[pair] = acc[0] / acc[1]
            STATE.contact_substeps = acc[1]
        else:
            last = last or read_contacts(env)
            out[pair] = last[pair]
    _ACC.clear()
    return out


def read_contacts(env) -> dict:
    """Contact force (N) between each watched pair at the last physics substep."""
    base = env.unwrapped
    dt = float(getattr(base, "physics_dt", 1.0 / 240.0))
    out = {}
    for a, b in _CONTACT_PAIRS:
        f = _contact_view(env, a, b).get_contact_force_matrix(dt=dt)
        f = f.numpy() if hasattr(f, "numpy") else _np(f)
        out[(a, b)] = float(np.linalg.norm(np.asarray(f, dtype=np.float64).reshape(-1, 3).sum(axis=0)))
    return out


def capture(env) -> None:
    """Record finger pad points and Part root poses for the step about to be scored."""
    from yam_arena import station

    base = env.unwrapped
    origin = _np(base.scene.env_origins)[0]
    fingers, links = {}, {}
    for arm, side in zip(ARMS, ("left", "right")):
        robot = base.scene[arm]
        jn = list(robot.joint_names)
        jp = _np(robot.data.joint_pos)[0]
        q6 = [jp[jn.index(f"joint{i}")] for i in range(1, 7)]
        fingers[arm] = finger_points(q6, jp[jn.index("joint7")], jp[jn.index("joint8")], station.BASES[side])
        if _TRACK_LINKS[0]:
            bp = getattr(robot.data, "body_link_pos_w", None)
            bp = robot.data.body_pos_w if bp is None else bp
            links[arm] = _np(bp)[0] - origin
    poses = {}
    rigid = base.scene.rigid_objects
    for name in _PARTS:
        if name in rigid:
            st = _np(rigid[name].data.root_link_pose_w)[0]
            poses[name] = (st[:3] - origin, st[3:7])
    STATE.fingers, STATE.poses, STATE.links = fingers, poses, links
    STATE.piles = read_piles(env) if _PILES else {}
    STATE.contacts = mean_contacts(env) if _CONTACT_PAIRS else {}
    STATE.step += 1


# -- Isaac-side hooks (run when yam_arena's modules are imported) ------------------------------------------------


def _spawn_part(prim_path, cfg, translation=None, orientation=None, **kwargs):
    from isaaclab.sim import schemas
    from isaaclab.sim.spawners.materials.physics_materials import spawn_physics_material
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material, create_prim, get_current_stage
    from yam_arena.scene import linear_rgb, sim_utils

    stage = get_current_stage()
    spec = _PARTS[prim_path.rstrip("/").split("/")[-1]]
    create_prim(prim_path, "Xform", translation=translation, orientation=orientation, stage=stage)
    phys = f"{prim_path}/geometry/physics_material"
    spawn_physics_material(phys, cfg.physics_material, stage=stage)
    looks: dict = {}
    for i, g in enumerate(spec.geoms):
        path = f"{prim_path}/geometry/g{i:02d}"
        if g.kind == "box":
            create_prim(path, "Cube", translation=g.center, orientation=g.quat_xyzw, scale=g.size,
                        attributes={"size": 1.0}, stage=stage)
        elif g.kind == "cyl":
            create_prim(path, "Cylinder", translation=g.center, orientation=g.quat_xyzw,
                        attributes={"radius": float(g.size[0]), "height": float(g.size[1]), "axis": "Z"}, stage=stage)
        else:
            create_prim(path, "Sphere", translation=g.center, attributes={"radius": float(g.size[0])}, stage=stage)
        props = cfg.collision_props
        if g.contact is not None or g.torsion is not None:
            co = g.contact or (props.contact_offset, props.rest_offset)
            extra = {} if g.torsion is None else dict(torsional_patch_radius=g.torsion,
                                                      min_torsional_patch_radius=0.8 * g.torsion)
            props = type(props)(contact_offset=co[0], rest_offset=co[1], **extra)
        if g.collide:
            schemas.define_collision_properties(path, props, stage=stage)
        color = tuple(g.color or spec.color)
        rough = 0.45 if g.roughness is None else float(g.roughness)
        if (color, rough) not in looks:
            looks[(color, rough)] = f"{prim_path}/geometry/look{len(looks)}"
            look = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=rough)
            look.func(looks[(color, rough)], look)
        bind_visual_material(path, looks[(color, rough)], stage=stage)
        if g.friction is not None:
            own = f"{prim_path}/geometry/physics_material_g{i:02d}"
            spawn_physics_material(own, sim_utils.RigidBodyMaterialCfg(static_friction=g.friction[0],
                                                                       dynamic_friction=g.friction[1]), stage=stage)
            bind_physics_material(path, own, stage=stage)
        else:
            bind_physics_material(path, phys, stage=stage)
    if getattr(cfg, "mass_props", None) is not None:
        schemas.define_mass_properties(prim_path, cfg.mass_props, stage=stage)
    if getattr(cfg, "rigid_props", None) is not None:
        schemas.define_rigid_body_properties(prim_path, cfg.rigid_props, stage=stage)
    if any(spec.name in pair for pair in _CONTACT_PAIRS):
        schemas.activate_contact_sensors(prim_path, threshold=0.0, stage=stage)
    for pl in _PILES.values():
        if pl.anchor == spec.name:
            _spawn_pile(f"{prim_path.rstrip('/').rsplit('/', 1)[0]}/{pl.name}", pl, stage)
    return stage.GetPrimAtPath(prim_path)


def _patch_scene(scene) -> None:
    if getattr(scene, "_kitchenkit", False):
        return
    from isaaclab.sim.utils import clone

    original = scene.task_object
    spawn = clone(_spawn_part)

    def task_object(spec):
        if not isinstance(spec, Part):
            return original(spec)
        contact = (scene._contact_props() if spec.contact is None else
                   scene.PhysxCollisionPropertiesCfg(contact_offset=spec.contact[0], rest_offset=spec.contact[1]))
        if spec.free:
            body = dict(rigid_props=scene.PhysxRigidBodyPropertiesCfg(**dict(spec.rigid)),
                        mass_props=scene.sim_utils.MassPropertiesCfg(mass=spec.mass), collision_props=contact)
        else:
            body = dict(collision_props=scene.PhysxCollisionPropertiesCfg() if spec.contact is None else contact)
        material = (scene._grippy() if spec.friction is None else
                    scene.sim_utils.RigidBodyMaterialCfg(static_friction=spec.friction[0],
                                                         dynamic_friction=spec.friction[1]))
        cfg = scene.sim_utils.CuboidCfg(size=tuple(spec.size), func=spawn, physics_material=material, **body)
        return scene.Object(
            name=spec.name,
            object_type=scene.ObjectType.RIGID if spec.free else scene.ObjectType.BASE,
            spawner_cfg=cfg,
            initial_pose=scene.Pose(position_xyz=tuple(spec.pos), rotation_xyzw=scene._yaw_xyzw(spec.yaw)),
        )

    scene.task_object = task_object
    scene._kitchenkit = True


def _patch_embodiment(mod) -> None:
    cls = mod.YamArenaEmbodiment
    if getattr(cls, "_kitchenkit", False):
        return
    task_step, reset = cls._task_step, cls.reset

    def _task_step(self):
        try:
            capture(self._env)
        except Exception as exc:  # noqa: BLE001 - keep the episode running, report once
            if STATE.error is None:
                STATE.error = repr(exc)
                print(f"kitchenkit: capture failed: {exc!r}", flush=True)
        return task_step(self)

    def install(env) -> None:
        if getattr(env, "_kitchenkit_piles", False):
            return
        env_reset = env.reset

        def env_reset_(*a, **k):
            out = env_reset(*a, **k)
            reset_piles(env)
            return out

        env.reset, env._kitchenkit_piles = env_reset_, True

    def reset_(self, *args, **kwargs):
        STATE.clear()
        for cb in RESET_CALLBACKS:
            cb()
        if _PILES:
            if getattr(self, "_env", None) is not None:
                install(self._env)
            else:  # first reset: the simulator starts inside reset(); hook its env as soon as it exists
                ensure = self._ensure_sim

                def ensure_(*a, **k):
                    out = ensure(*a, **k)
                    install(self._env)
                    return out

                self._ensure_sim = ensure_
        try:
            return reset(self, *args, **kwargs)
        finally:
            self.__dict__.pop("_ensure_sim", None)

    cls._task_step, cls.reset, cls._kitchenkit = _task_step, reset_, True


_HOOKS = {"yam_arena.scene": _patch_scene, "yam_arena.inspect_embodiment": _patch_embodiment}


class _AfterImport(importlib.abc.MetaPathFinder):
    def find_spec(self, name, path, target=None):
        if name not in _HOOKS:
            return None
        spec = importlib.machinery.PathFinder.find_spec(name, path)
        if spec is None or spec.loader is None:
            return None
        loader, hook = spec.loader, _HOOKS[name]

        class _Loader(importlib.abc.Loader):
            def create_module(self, s):
                return loader.create_module(s)

            def exec_module(self, module):
                loader.exec_module(module)
                hook(module)

        spec.loader = _Loader()
        return spec


def install() -> None:
    for name, hook in _HOOKS.items():
        if name in sys.modules:
            hook(sys.modules[name])
    if not any(isinstance(f, _AfterImport) for f in sys.meta_path):
        sys.meta_path.insert(0, _AfterImport())


install()


# -- scoring helpers (pure numpy, safe to use in task files) -----------------------------------------------------


def box_dist(pts, center, size, pose) -> np.ndarray:
    """Distance from each point (n, 3) to a box (center and size in the frame of ``pose`` = (pos, R))."""
    p, R = pose
    loc = (np.asarray(pts) - p) @ R - np.asarray(center)
    return np.linalg.norm(np.maximum(np.abs(loc) - np.asarray(size) / 2, 0.0), axis=1)


GRIP_DIST = 0.006  # m: a pad point of each finger this close to the box (pad points sit 4 mm inside the finger)
GRIP_SPAN = 0.050  # m: and the two fingers' pads no more than this far apart (closed on it, not straddling it)


def holds(arm: str, center, size, pose, dist: float = GRIP_DIST, span: float = GRIP_SPAN) -> bool:
    """Is ``arm``'s gripper closed on the box (center, size in the frame of ``pose``): a pad point of each finger
    within ``dist`` of it, the box's centre between the fingers, and the pads no more than ``span`` apart?"""
    pts = STATE.fingers.get(arm)
    if pts is None:
        return False
    n = len(pts) // 2
    d1 = float(box_dist(pts[:n], center, size, pose).min())
    d2 = float(box_dist(pts[n:], center, size, pose).min())
    p, R = pose
    loc = (pts - p) @ R - np.asarray(center)
    c1, c2 = loc[:n].mean(axis=0), loc[n:].mean(axis=0)
    gap = float(np.linalg.norm(c1 - c2))
    u = (c1 - c2) / max(gap, 1e-9)
    return d1 < dist and d2 < dist and float(c1 @ u) > 0.0 > float(c2 @ u) and gap <= span


def near(arm: str, center, size, pose, dist: float) -> bool:
    """Is any of ``arm``'s finger-pad points within ``dist`` of the box?"""
    pts = STATE.fingers.get(arm)
    return pts is not None and float(box_dist(pts, center, size, pose).min()) < dist


def touched_by_any(center, size, pose, dist: float = 0.008) -> bool:
    return any(near(arm, center, size, pose, dist) for arm in STATE.fingers)


def tilt_deg(R) -> float:
    """Angle between a part's z axis and the world's vertical (deg)."""
    return math.degrees(math.acos(float(np.clip(np.asarray(R)[2, 2], -1.0, 1.0))))


def pose_of(name: str, default_pos=(0.0, 0.0, 0.0)):
    """(pos, R) of a Part from the last capture, or its default with no rotation."""
    pos, q = STATE.poses.get(name, (np.asarray(default_pos, float), np.array([0.0, 0.0, 0.0, 1.0])))
    return np.asarray(pos, float), rot_matrix(q)


class FallWatch:
    """Flags a drop: the part falls ``fall`` m within ``steps`` control steps while no gripper is near it."""

    def __init__(self, fall: float = 0.05, steps: int = 10):
        self.fall, self.steps, self.z = fall, steps, []

    def reset(self) -> None:
        self.z = []

    def update(self, z: float, handled: bool) -> bool:
        self.z.append(float(z))
        del self.z[:-self.steps]
        return (not handled) and max(self.z) - self.z[-1] > self.fall


class StillWatch:
    """True once every tracked position has moved less than ``tol`` m over the last ``steps`` steps."""

    def __init__(self, tol: float = 0.002, steps: int = 20):
        self.tol, self.steps, self.buf = tol, steps, []

    def reset(self) -> None:
        self.buf = []

    def update(self, *positions) -> bool:
        self.buf.append(np.concatenate([np.asarray(p, float).ravel() for p in positions]))
        del self.buf[:-self.steps]
        return len(self.buf) == self.steps and float(np.ptp(np.array(self.buf), axis=0).max()) < self.tol
