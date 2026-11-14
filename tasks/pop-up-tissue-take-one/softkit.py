"""softkit v0.1: real deformable bodies in a yam_arena task file (shipped beside the task file, like mechkit.py).

A task file lists a soft part in OBJECTS like any other object:

    import softkit as sk
    GAUZE = sk.Soft("gauze", pos=(0.25, 0.10, TOP + 0.004), size=(0.30, 0.03, 0.001), kind="cloth", friction=1.0)
    FOAM = sk.Soft("foam", pos=(0.28, 0.10, TOP + 0.011), size=(0.086, 0.066, 0.022), kind="block",
                   youngs=3.0e4, poisson=0.2, density=80.0, refine=10.0)
    PEACH = sk.Soft("peach", pos=(0.20, 0.15, TOP + 0.033), size=(0.066, 0.066, 0.066), kind="ball", youngs=3e5)

and score() reads its mesh nodes: ``sk.nodes(objects["foam"])`` is an (N, 3) array of node positions (world frame,
metres); ``sk.motion(state, name)`` is the largest node speed measured from node displacement between control steps
(use it for stillness: the solver's node velocity readout jitters at several cm/s at rest). Nothing else about the body is hidden: every readout is a point you
could track on a video.

What it is, so you can reason about the physics:
- ``kind="cloth"``: a PhysX surface deformable (a triangle mesh, ``size`` = length x width x thickness). Defaults
  are the settings of Isaac Lab's PhysX cloth-lift example environment (rest offset 2 mm, contact offset 10 mm,
  density 1000, 1 mm thick, E 1 MPa, bend 1e6); the Arena library's default cloth does not simulate on this rig.
- ``kind="block"`` / ``kind="ball"``: a PhysX FEM volume deformable (a tetrahedral mesh made by the TetWild
  tetrahedralizer from a box / sphere surface; ``size`` = edge lengths / diameter).
- Mesh size: ``edge`` (m) or ``refine`` (Isaac Lab's ``edge_refinement``: longest edge = bounding-box diagonal /
  refine). Volumes are tetrahedralized at that edge (86 x 66 x 22 mm block: refine 6 -> 43 nodes, 10 -> 117,
  14 -> 398; the library's 4.0 gives ~21). Cloth is spawned as a regular grid of right triangles with that spacing
  (Isaac Lab's own rectangle refinement halves every triangle in both directions, so a long strip gets thousands
  of sliver nodes). Keep meshes small: they cost sim time.
- Soft parts have no ``mass`` (their mass comes from ``density``), so yam_arena never treats them as rigid
  bodies or jitters them. Arena's own deformable reset puts every node back at the start of each episode.

How it hooks in (in memory, nothing installed): it chains onto mechkit's patch of ``yam_arena.scene`` so that a
``Soft`` becomes an Isaac Lab-Arena ``DeformableObject`` (PhysX deformable schema) and ``object_states`` reads its
nodes through ``arena_world.get_nodal_positions_w / get_nodal_velocities_w``. Offline (no Isaac Sim) the same
names work on hand-built states: ``sk.rest_nodes(spec)`` gives an approximate node set for the start pose and
``sk.fake(spec, nodes)`` wraps nodes as a state, for self-tests.

Assets: none. Every mesh is generated in code at spawn time (box, sphere and rectangle primitives of Isaac Lab).
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402  (episode hook, per-episode memory, the grippers' TCPs)

from yam_arena.task_file import Box, ObjectState  # noqa: E402

__version__ = "0.2"
KINDS = ("cloth", "block", "ball")


@dataclass(frozen=True)
class Soft(Box):
    """A deformable part. ``size``: cloth (length x, width y, thickness), block (x, y, z), ball (d, d, d)."""

    kind: str = "cloth"
    density: float = 1000.0  # kg/m^3
    youngs: float = 1.0e6  # Pa
    poisson: float = 0.25
    friction: float = 1.0  # static = dynamic, combined with the touching body's friction by PhysX (average)
    damping: float = 0.1  # elasticity damping
    refine: float = 8.0  # volumes: longest mesh edge = bounding-box diagonal / refine
    edge: float | None = None  # target mesh edge (m); cloth: the grid spacing (default diagonal / refine)
    bend: float = 1.0e6  # cloth only: bend stiffness
    bend_damping: float = 0.1  # cloth only
    self_collision: bool = False  # cloth: stacked folds take up thickness instead of passing through each other
    rest_offset: float | None = None  # m; default 2 mm for cloth, 0 for volumes
    contact_offset: float | None = None  # m; default 10 mm for cloth, 3 mm for volumes
    iterations: int = 16  # solver position iterations
    profile: tuple | None = None  # cloth only: a creased sheet. Its cross-section as (y, z) points in the part frame
    # (relative to pos, before yaw); the sheet runs along that polyline and spans size[0] along x. The spawn shape
    # is also the rest shape, so creases and bends in it spring back like folded paper.

    def _check(self) -> None:
        super()._check()
        if self.mass is not None:
            raise ValueError(f"{self.name}: a Soft part takes its mass from density; leave mass=None")
        if self.kind not in KINDS:
            raise ValueError(f"{self.name}.kind must be one of {KINDS}")
        if self.refine < 1.0:
            raise ValueError(f"{self.name}.refine must be >= 1")

    @property
    def offsets(self) -> tuple[float, float]:
        cloth = self.kind == "cloth"
        rest = (0.002 if cloth else 0.0) if self.rest_offset is None else self.rest_offset
        contact = (0.010 if cloth else 0.003) if self.contact_offset is None else self.contact_offset
        return rest, contact

    @property
    def volume(self) -> float:
        sx, sy, sz = self.size
        if self.kind == "cloth":
            return sx * sy * sz
        return sx * sy * sz if self.kind == "block" else math.pi / 6 * sx ** 3

    @property
    def kg(self) -> float:
        """The part's mass from its density and size."""
        return self.density * self.volume


@dataclass(frozen=True)
class SoftState(ObjectState):
    """A soft part after a control step: the usual bounding-box fields plus every mesh node."""

    nodes: np.ndarray = None  # (N, 3) world positions, m
    node_vel: np.ndarray = None  # (N, 3) m/s

    def resting(self, max_speed: float = 0.01, max_spin: float = 0.5) -> bool:
        """Still: every node under ``max_speed`` m/s."""
        return max_node_speed(self) < max_speed


# -- readouts (offline-safe) ----------------------------------------------------------------------------------------


def nodes(state) -> np.ndarray:
    """(N, 3) node positions of a soft part (world frame, m)."""
    return np.asarray(state.nodes, dtype=float)


def speeds(state) -> np.ndarray:
    """(N,) node speeds (m/s); zeros when unknown (offline states)."""
    v = getattr(state, "node_vel", None)
    return np.zeros(len(state.nodes)) if v is None else np.linalg.norm(np.asarray(v, dtype=float), axis=1)


def max_node_speed(state) -> float:
    s = speeds(state)
    return float(s.max()) if len(s) else 0.0


CONTROL_DT = 0.05  # s per control step (20 Hz)


def motion(state, name: str) -> float:
    """Max node speed (m/s) from how far the nodes moved since the previous control step. Use this for stillness in
    sim: PhysX FEM node *velocity* readouts jitter at 5-25 cm/s even when the body is at rest (positions stay put to
    0.1 mm). Repeated calls within one control step (score() and success()) return the same value; the first call of
    an episode returns inf. Offline (no sim) it falls back to the state's node velocities."""
    if mk._EP.get("env") is None:
        return max_node_speed(state)
    step, p = mk._EP.get("step", 0), nodes(state)
    m = mk.MEMORY.setdefault("softkit_motion", {})
    rec = m.get(name)
    if rec is None:
        m[name] = (step, p.copy(), math.inf)
    elif step != rec[0]:
        d = float(np.max(np.linalg.norm(p - rec[1], axis=1))) if len(p) == len(rec[1]) else math.inf
        m[name] = (step, p.copy(), d / (CONTROL_DT * max(1, step - rec[0])))
    return m[name][2]


def clear_of_tcps(state, margin: float = 0.03, tcps=None) -> bool:
    """Neither gripper TCP within ``margin`` of any node (True offline when no TCPs are given)."""
    tc = mk.tcps() if tcps is None else tcps
    p = nodes(state)
    return not any(float(np.min(np.linalg.norm(p - np.asarray(t), axis=1))) < margin for t in tc)


def inside_box(points, lo, hi) -> np.ndarray:
    """Boolean mask: which points lie inside the axis-aligned box lo..hi."""
    p = np.asarray(points, dtype=float)
    return np.all((p >= np.asarray(lo)) & (p <= np.asarray(hi)), axis=1)


# -- offline stand-ins ----------------------------------------------------------------------------------------------


def edge_length(spec: Soft) -> float:
    """The longest mesh edge the spawner aims for (m)."""
    if spec.edge is not None:
        return float(spec.edge)
    sx, sy, sz = spec.size
    diag = math.sqrt(sx * sx + sy * sy + (0.0 if spec.kind == "cloth" else sz * sz))
    return diag / spec.refine


def grid(length: float, width: float, h: float):
    """A flat rectangle (centred, in the xy plane) as a grid of right triangles with spacing <= h: (verts, faces)."""
    nx, ny = max(1, math.ceil(length / h - 1e-9)), max(1, math.ceil(width / h - 1e-9))
    xs, ys = np.linspace(-length / 2, length / 2, nx + 1), np.linspace(-width / 2, width / 2, ny + 1)
    verts = np.array([(x, y, 0.0) for x in xs for y in ys])
    faces = []
    for i in range(nx):
        for j in range(ny):
            a, b, c, d = i * (ny + 1) + j, (i + 1) * (ny + 1) + j, (i + 1) * (ny + 1) + j + 1, i * (ny + 1) + j + 1
            faces += [(a, b, c), (a, c, d)] if (i + j) % 2 == 0 else [(a, b, d), (b, c, d)]
    return verts, np.array(faces)


def profile_grid(width: float, path, h: float):
    """A creased sheet: the polyline ``path`` [(y, z), ...] resampled every <= h along its length, swept along x
    over ``width``; a grid of right triangles. Returns (verts, faces)."""
    pts = np.asarray(path, dtype=float)
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    n = max(1, math.ceil(s[-1] / h - 1e-9))
    t = np.linspace(0.0, s[-1], n + 1)
    yz = np.c_[np.interp(t, s, pts[:, 0]), np.interp(t, s, pts[:, 1])]
    ny = max(1, math.ceil(width / h - 1e-9))
    xs = np.linspace(-width / 2, width / 2, ny + 1)
    verts = np.array([(x, y, z) for (y, z) in yz for x in xs])
    faces = []
    for i in range(n):
        for j in range(ny):
            a, b, c, d = i * (ny + 1) + j, (i + 1) * (ny + 1) + j, (i + 1) * (ny + 1) + j + 1, i * (ny + 1) + j + 1
            faces += [(a, b, c), (a, c, d)] if (i + j) % 2 == 0 else [(a, b, d), (b, c, d)]
    return verts, np.array(faces)


def path_length(path) -> float:
    p = np.asarray(path, dtype=float)
    return float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum())


def cloth_mesh(spec: Soft):
    """(verts, faces) of a cloth part in its own frame: a flat grid, or the creased ``profile`` sheet."""
    if spec.profile:
        return profile_grid(float(spec.size[0]), spec.profile, edge_length(spec))
    return grid(float(spec.size[0]), float(spec.size[1]), edge_length(spec))


def spawn_origin(spec: Soft) -> np.ndarray:
    """Where the part's prim is placed: ``pos``, except for a creased cloth, whose mesh is spawned about its node
    centroid (Isaac Lab-Arena's deformable reset puts the node centroid at the initial pose)."""
    p = np.asarray(spec.pos, dtype=float)
    if spec.kind != "cloth" or not spec.profile:
        return p
    c, s = math.cos(spec.yaw), math.sin(spec.yaw)
    m = cloth_mesh(spec)[0].mean(axis=0)
    return p + np.array([c * m[0] - s * m[1], s * m[0] + c * m[1], m[2]])


def rest_nodes(spec: Soft) -> np.ndarray:
    """An approximate node set for the part's start pose (a lattice at the target edge length; offline only, the sim
    mesh comes from the tetrahedralizer / surface subdivision and differs in detail)."""
    h = edge_length(spec)
    sx, sy, sz = spec.size
    c, s = math.cos(spec.yaw), math.sin(spec.yaw)
    rot = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])

    def ax(n):
        k = max(2, int(math.ceil(n / h)) + 1)
        return np.linspace(-n / 2, n / 2, k)

    if spec.kind == "cloth":
        g = cloth_mesh(spec)[0]
    elif spec.kind == "block":
        g = np.array([(x, y, z) for x in ax(sx) for y in ax(sy) for z in ax(sz)])
    else:
        r = sx / 2
        g = np.array([(x, y, z) for x in ax(sx) for y in ax(sx) for z in ax(sx) if x * x + y * y + z * z <= r * r])
        n = max(12, int(4 * math.pi * r * r / (h * h)))
        k = np.arange(n) + 0.5
        phi, th = np.arccos(1 - 2 * k / n), math.pi * (1 + 5 ** 0.5) * k
        g = np.vstack([g, r * np.c_[np.cos(th) * np.sin(phi), np.sin(th) * np.sin(phi), np.cos(phi)]])
    return g @ rot.T + np.asarray(spec.pos)


def fake(spec_or_name, pts, vel=None) -> SoftState:
    """A SoftState from hand-built node positions (offline self-tests)."""
    p = np.asarray(pts, dtype=float)
    v = np.zeros_like(p) if vel is None else np.asarray(vel, dtype=float)
    return SoftState(pos=p.mean(0), quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]), lin_vel=v.mean(0), ang_vel=np.zeros(3),
                     low=p.min(0), high=p.max(0), nodes=p, node_vel=v)


# -- yam_arena hook (in memory) -------------------------------------------------------------------------------------

SOFTS: dict[str, Soft] = {}  # every Soft the task file built, by name (filled as specs are spawned or registered)
NODE_COUNT: dict[str, int] = {}  # sim: how many nodes each soft part got


def register(*specs: Soft) -> None:
    """Optional: declare the task's soft parts up front (spawning them registers them too)."""
    for s in specs:
        SOFTS[s.name] = s


def _spawner_cfg(mod, spec: Soft):
    import isaaclab.sim as sim_utils
    from isaaclab_physx.sim.schemas import PhysxCollisionCfg, PhysxDeformableBodyPropertiesCfg
    from isaaclab_physx.sim.spawners.materials import (
        PhysxDeformableBodyMaterialCfg,
        PhysxSurfaceDeformableBodyMaterialCfg,
    )

    rest, contact = spec.offsets
    sx, sy, sz = (float(v) for v in spec.size)
    diag = math.sqrt(sx * sx + sy * sy + sz * sz)
    props = PhysxDeformableBodyPropertiesCfg(solver_position_iteration_count=int(spec.iterations),
                                             linear_damping=0.0,
                                             self_collision=True if spec.self_collision else None)
    common = dict(
        edge_refinement=1.0 if spec.kind == "cloth" else max(1.0, diag / edge_length(spec)),
        deformable_props=props,
        collision_props=[PhysxCollisionCfg(rest_offset=rest, contact_offset=contact)],
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=mod.linear_rgb(spec.color), roughness=0.8),
    )
    if spec.kind == "cloth":
        mat = PhysxSurfaceDeformableBodyMaterialCfg(
            density=spec.density, surface_thickness=float(spec.size[2]), poissons_ratio=spec.poisson,
            youngs_modulus=spec.youngs, surface_bend_stiffness=spec.bend, elasticity_damping=spec.damping,
            bend_damping=spec.bend_damping, static_friction=spec.friction, dynamic_friction=spec.friction)
        return sim_utils.MeshRectangleCfg(func=_cloth_spawner(), size=(sx, sy), physics_material=mat, **common)
    mat = PhysxDeformableBodyMaterialCfg(density=spec.density, youngs_modulus=spec.youngs,
                                         poissons_ratio=spec.poisson, static_friction=spec.friction,
                                         dynamic_friction=spec.friction, elasticity_damping=spec.damping)
    if spec.kind == "block":
        return sim_utils.MeshCuboidCfg(size=tuple(float(v) for v in spec.size), physics_material=mat, **common)
    return sim_utils.MeshSphereCfg(radius=float(spec.size[0]) / 2, physics_material=mat, **common)


_CLOTH_FN = []


def _cloth_spawner():
    """A spawner (decorated like Isaac Lab's own) for a cloth rectangle meshed as a regular grid (see grid())."""
    if _CLOTH_FN:
        return _CLOTH_FN[0]
    import trimesh
    from isaaclab.sim.spawners.meshes import meshes as M
    from isaaclab.sim.utils import clone, get_current_stage

    @clone
    def spawn(prim_path, cfg, translation=None, orientation=None, **kwargs):
        spec = SOFTS[prim_path.rsplit("/", 1)[-1]]
        verts, faces = cloth_mesh(spec)
        verts = verts - verts.mean(axis=0)  # node centroid at the prim origin (Arena resets nodes about it)
        mesh = trimesh.Trimesh(vertices=verts, faces=faces, process=False)
        stage = get_current_stage()
        M._spawn_mesh_geom_from_mesh(prim_path, cfg, mesh, translation, orientation, None, stage=stage)
        return stage.GetPrimAtPath(prim_path)

    _CLOTH_FN.append(spawn)
    return spawn


def _patch_scene(mod) -> None:
    if getattr(mod, "_softkit", False):
        return
    mod._softkit = True
    obj0, states0 = mod.task_object, mod.object_states

    def task_object(spec):
        if not isinstance(spec, Soft):
            return obj0(spec)
        from isaaclab_arena.assets.deformable_object import DeformableObject

        SOFTS[spec.name] = spec
        q = (0.0, 0.0, math.sin(spec.yaw / 2), math.cos(spec.yaw / 2))
        return DeformableObject(name=spec.name, spawner_cfg=_spawner_cfg(mod, spec),
                                initial_pose=mod.Pose(position_xyz=tuple(float(v) for v in spawn_origin(spec)),
                                                      rotation_xyzw=q))

    def object_states(env, names):
        soft = [n for n in names if n in SOFTS]
        out = states0(env, [n for n in names if n not in SOFTS])
        if soft:
            world = env.unwrapped.arena_world
            origin = env.unwrapped.scene.env_origins[0]
            for n in soft:
                p = (world.get_nodal_positions_w(n)[0] - origin).detach().cpu().numpy().astype(float)
                v = world.get_nodal_velocities_w(n)[0].detach().cpu().numpy().astype(float)
                if n not in NODE_COUNT:
                    NODE_COUNT[n] = len(p)
                    print(f"[softkit] {n}: {len(p)} nodes", flush=True)
                out[n] = SoftState(pos=p.mean(0), quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]), lin_vel=v.mean(0),
                                   ang_vel=np.zeros(3), low=p.min(0), high=p.max(0), nodes=p, node_vel=v)
        return {n: out[n] for n in names}

    mod.task_object, mod.object_states = task_object, object_states


def _install() -> None:
    """Run _patch_scene right after yam_arena.scene is imported, after mechkit's own patch of it."""
    if "yam_arena.scene" in sys.modules:
        _patch_scene(sys.modules["yam_arena.scene"])
        return
    for f in sys.meta_path:
        if isinstance(f, mk._AfterImport) and f.name == "yam_arena.scene":
            if getattr(f.patch, "_softkit", False):
                return
            first = f.patch

            def both(module, first=first):
                first(module)
                _patch_scene(module)

            both._softkit = True
            f.patch = both
            return
    mk._when_imported("yam_arena.scene", _patch_scene)


_install()
