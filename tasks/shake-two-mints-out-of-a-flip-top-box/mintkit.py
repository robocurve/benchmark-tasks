"""Parts for the mint-box task: a flip-top mint box, its sprung lid flap, loose mints, and a dish.

What a task file can list in ``OBJECTS`` (all are Box subclasses so the loader accepts them):

``MintBox``   The box: a thin-walled plastic shell 47 x 17 x 73 mm with a dispensing opening in one corner of its top
              end. Free. Its ``pos`` is the centre of the shell; its own x runs across its width toward the side the
              opening is on, its y through its thickness, its z along its length toward the lid end.
``Flap``      The flip-top flap that covers the opening, a separate free body joined to its ``box`` by a hinge across
              the box's thickness. A torque law on the hinge makes it snap: it is held shut, and past a small angle it
              springs open to its stop. Its ``pos`` is the middle of the hinge line; closed, its axes are the box's.
``Mint``      One mint: a rounded oblong (a capsule), free. Its ``pos`` is its centre; its x is its long axis.

The dish is five fixed boards (plain ``Box`` objects): see ``dish_boards``.

Every shape is built from numbers in NumPy when the task file is read, so the same numbers serve the scene, the
scoring and the offline checks. Isaac Sim is reached only when the scene is built.

The structure follows the kits used by tasks already in this repository (Box subclasses so the loader accepts the
parts, custom spawners, hooks installed through ``sys.meta_path``, a passive joint authored afresh at every reset).
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import sys
from dataclasses import dataclass, field

import numpy as np
from yam_arena.task_file import Box, ObjectState

# -- the box (m) ---------------------------------------------------------------------------------------------------------
W, T, H = 0.047, 0.017, 0.073  # width, thickness, length of the 1 oz pack (a retailer's listing: 1.85 x 0.67 x 2.87 in)
WALL = 0.001                   # wall thickness (an estimate)
TOP_T = 0.002                  # the plate across the top end that the opening is cut in (an estimate)
FLAP_T = 0.0015                # the flap lies on that plate, flush with the top end (an estimate)
OPEN_W, OPEN_T = 0.012, 0.010  # the opening: along the width, and across the thickness (estimates)
FLAP_L = 0.020                 # the flap from its hinge to the outside of the box's narrow side (an estimate)
LIP = 0.0025                   # the flap's thumb lip overhangs the narrow side by this much (an estimate)
FLAP_W = T - 2 * WALL          # the flap's width, across the thickness
HINGE = (W / 2 - FLAP_L, 0.0, H / 2)   # the hinge line's middle, in the box's frame
LID_MAX = 110.0                # degrees the flap opens to (an estimate)
BOX_MASS = 0.009               # kg, the empty box without its flap (an estimate)
BOX_INERTIA = 0.060            # kg the box's body is given, so that the contact solver holds it between the fingers;
#                                a constant upward force carries the difference, so it still weighs BOX_MASS
FLAP_MASS = 0.002              # kg: heavier than a real flap (about 0.5 g) so that the contact solver holds it
# The snap, as a torque on the hinge (N m, + opens). Below OVER degrees the flap is held shut; above it, pushed open.
T_SHUT = 0.020                 # 0.9 N at the lip, 22 mm from the hinge (an estimate)
T_OPEN = 0.005                 # (an estimate)
OVER, RAMP = 12.0, 4.0         # the changeover angle and the half-width of the ramp between the two (estimates)
LID_DAMP = 0.001               # N m s/rad of hinge damping (an estimate)
SHUT_AT = 3.0                  # the lid counts as shut within this many degrees of closed

# -- a mint ----------------------------------------------------------------------------------------------------------------
MINT_D, MINT_L = 0.0065, 0.0115  # across and along (estimates; with the labelled 0.5 g they give sugar's 1.6 g/cm3)
MINT_MASS = 0.0005               # kg: "Serving Size 1 mint (0.5g)" on the maker's page
MINT_FRICTION = (0.35, 0.30)     # a sugar shell on plastic and glazed stoneware (an estimate)
MINT_BOUNCE = 0.2                # (an estimate)

# -- the dish ------------------------------------------------------------------------------------------------------------
DISH_IN = 0.110                # inside, square
DISH_WALL = 0.005
DISH_FLOOR = 0.005
DISH_H = 0.025                 # outside height

CLEAR = (0.80, 0.86, 0.90)
LID_WHITE = (0.96, 0.96, 0.94)
MINT_WHITE = (0.97, 0.97, 0.95)
DISH_BLUE = (0.16, 0.24, 0.40)
PLASTIC = (0.40, 0.35)         # the box's plastic against a mint and the lid's flap (an estimate)
RUBBER = (0.90, 0.80)          # the box's outside against a gripper's rubber pad (an estimate)
PAD_PATCH = 0.008             # radius of the patch over which a rubber pad meets the box's side (an estimate)
PALM_BACK = 0.065
FINGER_THICK = 0.012


# -- geometry (pure) -------------------------------------------------------------------------------------------------
def _rotation(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _quat(rot) -> tuple:
    m = np.asarray(rot, float)
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        q = ((m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, 0.25 * s)
    else:
        i = int(np.argmax([m[0, 0], m[1, 1], m[2, 2]]))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = math.sqrt(max(1e-12, 1.0 + m[i, i] - m[j, j] - m[k, k])) * 2
        q = [0.0, 0.0, 0.0, (m[k, j] - m[j, k]) / s]
        q[i], q[j], q[k] = 0.25 * s, (m[j, i] + m[i, j]) / s, (m[k, i] + m[i, k]) / s
    return tuple(float(v) for v in q)


def _rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _ry(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def place(pos, quat, local) -> np.ndarray:
    """A point given in a part's own frame, in the world."""
    return np.asarray(pos, float) + _rotation(quat) @ np.asarray(local, float)


def to_local(point, pos, quat) -> np.ndarray:
    return _rotation(quat).T @ (np.asarray(point, float) - np.asarray(pos, float))


def _obb(points, centre, rot, size) -> np.ndarray:
    """Signed distance from each point to a box of ``size`` at ``centre`` with its edges along the columns of ``rot``."""
    q = np.abs((np.asarray(points, float).reshape(-1, 3) - centre) @ rot) - np.asarray(size, float) / 2
    return np.linalg.norm(np.maximum(q, 0.0), axis=1) + np.minimum(q.max(axis=1), 0.0)


LYING = np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])   # columns: the box's x, y, z in the world


def lying_rot(yaw: float = 0.0) -> np.ndarray:
    """The box lying on a big face: its thickness up, and at ``yaw`` 0 its lid end toward the robot (-x) and the side
    its opening is on toward the robot's right (-y)."""
    return _rz(yaw) @ LYING


def opening() -> np.ndarray:
    """The middle of the dispensing opening, in the box's frame."""
    return np.array([W / 2 - WALL - OPEN_W / 2, 0.0, H / 2 - FLAP_T - TOP_T / 2])


def box_boards() -> list:
    """The box's shell as boards in its own frame: (tag, centre, size)."""
    it, top_z = T - 2 * WALL, H / 2 - FLAP_T - TOP_T / 2
    x2 = W / 2 - WALL
    x1 = x2 - OPEN_W
    x0 = -W / 2 + WALL
    ledge = (it - OPEN_T) / 2
    hx = HINGE[0]
    return [
        ("face_lo", (0.0, -(T - WALL) / 2, 0.0), (W, WALL, H)),
        ("face_hi", (0.0, (T - WALL) / 2, 0.0), (W, WALL, H)),
        ("side_a", (-(W - WALL) / 2, 0.0, 0.0), (WALL, it, H)),
        ("side_b", ((W - WALL) / 2, 0.0, -FLAP_T / 2), (WALL, it, H - FLAP_T)),
        ("bottom", (0.0, 0.0, -(H - WALL) / 2), (W - 2 * WALL, it, WALL)),
        ("top_main", ((x0 + x1) / 2, 0.0, top_z), (x1 - x0, it, TOP_T)),
        ("top_ledge_lo", ((x1 + x2) / 2, -(OPEN_T + ledge) / 2, top_z), (OPEN_W, ledge, TOP_T)),
        ("top_ledge_hi", ((x1 + x2) / 2, (OPEN_T + ledge) / 2, top_z), (OPEN_W, ledge, TOP_T)),
        ("top_cover", ((x0 + hx) / 2, 0.0, (H - FLAP_T) / 2), (hx - x0, it, FLAP_T)),
    ]


def flap_boards() -> list:
    """The flap in its own frame (origin on the hinge line, x toward the lip, its outer face at z = 0)."""
    length = FLAP_L + LIP
    return [("plate", (length / 2, 0.0, -FLAP_T / 2), (length, FLAP_W, FLAP_T))]


def flap_pose(box_pos, box_quat, angle_deg: float = 0.0) -> tuple:
    """Where the flap is when it stands ``angle_deg`` open: its position and quaternion in the world."""
    rot = _rotation(box_quat) @ _ry(-math.radians(angle_deg))
    return place(box_pos, box_quat, HINGE), _quat(rot)


def lid_angle(box_quat, flap_quat) -> float:
    """How far the flap stands open (degrees; 0 shut, + open)."""
    rel = _rotation(box_quat).T @ _rotation(flap_quat)
    return math.degrees(math.atan2(rel[2, 0], rel[0, 0]))


def lid_torque(angle_deg: float) -> float:
    """The snap: the torque on the hinge at a flap angle (N m; + opens, - shuts)."""
    s = float(np.clip((angle_deg - (OVER - RAMP)) / (2 * RAMP), 0.0, 1.0))
    return -T_SHUT + s * (T_SHUT + T_OPEN)


def in_box(point, box_pos, box_quat, margin: float = 0.0) -> bool:
    """A point is inside the box's outline."""
    p = np.abs(to_local(point, box_pos, box_quat))
    return bool(p[0] <= W / 2 + margin and p[1] <= T / 2 + margin and p[2] <= H / 2 + margin)


def box_distance(points, box_pos, box_quat) -> float:
    """The least distance from any of ``points`` (world) to the box's outline."""
    return float(_obb(points, np.asarray(box_pos, float), _rotation(box_quat), (W, T, H)).min())


def mint_grid(count: int) -> list:
    """Where ``count`` mints lie at the start, in the box's frame: one layer on the lower big face, long axes along
    the box, filled row by row from the bottom end. Each is (position, rotation whose first column is the long axis)."""
    cols, pitch_x, pitch_z = 6, 0.0072, 0.0122
    rot = np.array([[0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]])       # the mint's x along the box's z
    y = -T / 2 + WALL + MINT_D / 2 + 0.0004
    z0 = -H / 2 + WALL + MINT_L / 2 + 0.0012
    out = []
    for k in range(count):
        row, col = divmod(k, cols)
        out.append((np.array([(col - (cols - 1) / 2) * pitch_x, y, z0 + row * pitch_z]), rot))
    return out


def dish_boards(centre_xy) -> list:
    """The dish as five boards: (tag, centre above the table, size)."""
    cx, cy = (float(v) for v in centre_xy)
    half, wall = DISH_IN / 2, DISH_WALL
    out = [("floor", np.array([cx, cy, DISH_FLOOR / 2]), (DISH_IN + 2 * wall, DISH_IN + 2 * wall, DISH_FLOOR))]
    rim_h = DISH_H - DISH_FLOOR
    z = DISH_FLOOR + rim_h / 2
    out.append(("near", np.array([cx - half - wall / 2, cy, z]), (wall, DISH_IN + 2 * wall, rim_h)))
    out.append(("far", np.array([cx + half + wall / 2, cy, z]), (wall, DISH_IN + 2 * wall, rim_h)))
    out.append(("right", np.array([cx, cy - half - wall / 2, z]), (DISH_IN, wall, rim_h)))
    out.append(("left", np.array([cx, cy + half + wall / 2, z]), (DISH_IN, wall, rim_h)))
    return out


def in_dish(point, centre_xy, top: float = 0.0) -> bool:
    """A point (world) is in the dish: over its floor, inside its rim, and below the rim's top."""
    p = np.asarray(point, float)
    return bool(abs(p[0] - centre_xy[0]) <= DISH_IN / 2 and abs(p[1] - centre_xy[1]) <= DISH_IN / 2
                and 0.0 <= p[2] - top <= DISH_H)


def over_dish(point, centre_xy, margin: float = 0.0) -> bool:
    """A point's footprint is on the dish (rim included)."""
    half = DISH_IN / 2 + DISH_WALL + margin
    return bool(abs(point[0] - centre_xy[0]) <= half and abs(point[1] - centre_xy[1]) <= half)


# -- the parts -----------------------------------------------------------------------------------------------------------

class _Part(Box):
    def _check(self) -> None:
        if not self.name.isidentifier():
            raise ValueError(f"object name {self.name!r} must be a Python identifier")


@dataclass(frozen=True)
class MintBox(_Part):
    quat: tuple = (0.0, 0.0, 0.0, 1.0)


@dataclass(frozen=True)
class Flap(_Part):
    quat: tuple = (0.0, 0.0, 0.0, 1.0)
    box: str = ""


@dataclass(frozen=True)
class Mint(_Part):
    quat: tuple = (0.0, 0.0, 0.0, 1.0)


@dataclass(frozen=True)
class KitState(ObjectState):
    """A kit object as the simulator has it; ``pos`` is its own origin (see each class)."""

    joints: dict = field(default_factory=dict)


# -- authoring helpers -----------------------------------------------------------------------------------------------

class _Author:
    def __init__(self, prim_path, translation, orientation):
        import isaaclab.sim as sim_utils
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Gf, UsdGeom

        self.sim_utils, self.Gf, self.UsdGeom = sim_utils, Gf, UsdGeom
        self.stage = get_current_stage()
        self.path = prim_path
        double = UsdGeom.XformOp.PrecisionDouble
        root = UsdGeom.Xform.Define(self.stage, prim_path)
        root.AddTranslateOp(precision=double).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
        x, y, z, w = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
        root.AddOrientOp(precision=double).Set(Gf.Quatd(w, x, y, z))
        self.root = root.GetPrim()
        self.looks, self.mats = {}, {}

    def look(self, color, rough=0.6, metal=0.0, opacity=1.0) -> str:
        from yam_arena.station import linear_rgb

        key = (tuple(color), rough, metal, opacity)
        if key not in self.looks:
            path = f"{self.path}/Looks/v{len(self.looks)}"
            kw = {"opacity": opacity} if opacity < 1.0 else {}
            vis = self.sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=rough, metallic=metal, **kw)
            vis.func(path, vis)
            self.looks[key] = path
        return self.looks[key]

    def mat(self, key, sf, df, bounce=0.0) -> str:
        if key not in self.mats:
            path = f"{self.path}/Looks/phys_{key}"
            try:
                pm = self.sim_utils.RigidBodyMaterialCfg(static_friction=sf, dynamic_friction=df, restitution=bounce,
                                                         friction_combine_mode="average", restitution_combine_mode="average")
            except TypeError:
                pm = self.sim_utils.RigidBodyMaterialCfg(static_friction=sf, dynamic_friction=df, restitution=bounce)
            pm.func(path, pm)
            self.mats[key] = path
        return self.mats[key]

    def body(self, path, mass, lin_damp=0.05, ang_damp=0.1, iterations=None, depen=0.3):
        from pxr import PhysxSchema, UsdPhysics

        prim = self.UsdGeom.Xform.Define(self.stage, path).GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(float(mass))
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateMaxDepenetrationVelocityAttr(float(depen))
        rb.CreateLinearDampingAttr(lin_damp)
        rb.CreateAngularDampingAttr(ang_damp)
        rb.CreateSleepThresholdAttr(0.0)
        if iterations:
            rb.CreateSolverPositionIterationCountAttr(int(iterations))
            rb.CreateSolverVelocityIterationCountAttr(4)
        return prim

    def _collide(self, prim, mat, offset, patch=0.0):
        from isaaclab.sim.utils import bind_physics_material
        from pxr import PhysxSchema, UsdPhysics

        UsdPhysics.CollisionAPI.Apply(prim)
        cp = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        cp.CreateContactOffsetAttr(offset)
        cp.CreateRestOffsetAttr(0.0)
        if patch:                                    # a rubber pad meets the part over a patch, which resists twisting
            cp.CreateTorsionalPatchRadiusAttr(float(patch))
            cp.CreateMinTorsionalPatchRadiusAttr(float(patch))
        bind_physics_material(str(prim.GetPath()), mat, stage=self.stage)

    def bar(self, path, centre, size, look, mat=None, offset=0.0008, patch=0.0):
        """A box of ``size`` at ``centre``: a box collider (thin plates cannot be cooked as convex hulls for the GPU
        pipeline, and then collide with nothing)."""
        from isaaclab.sim.utils import bind_visual_material

        cube = self.UsdGeom.Cube.Define(self.stage, path)
        cube.CreateSizeAttr(1.0)
        cube.AddTranslateOp().Set(self.Gf.Vec3d(*(float(v) for v in centre)))
        cube.AddScaleOp().Set(self.Gf.Vec3f(*(float(v) for v in size)))
        prim = cube.GetPrim()
        if look:
            bind_visual_material(str(prim.GetPath()), look, stage=self.stage)
        if mat:
            self._collide(prim, mat, offset, patch)
        return prim

    def pill(self, path, length, radius, look, mat=None, offset=0.0008):
        """A capsule along x, ``length`` overall."""
        from isaaclab.sim.utils import bind_visual_material

        cap = self.UsdGeom.Capsule.Define(self.stage, path)
        cap.CreateAxisAttr("X")
        cap.CreateHeightAttr(float(length - 2 * radius))
        cap.CreateRadiusAttr(float(radius))
        prim = cap.GetPrim()
        if look:
            bind_visual_material(str(prim.GetPath()), look, stage=self.stage)
        if mat:
            self._collide(prim, mat, offset)
        return prim


# -- spawning ----------------------------------------------------------------------------------------------------------

_SPECS: dict = {}
_PATHS: dict = {}                      # where each kit object's prim was spawned
BOX_OPACITY = 0.35


def _start(cfg, prim_path, translation, orientation, mass=None, **kw):
    spec = _SPECS[cfg.kit_name]
    _PATHS[spec.name] = prim_path
    a = _Author(prim_path, translation, orientation)
    a.body(prim_path, float(spec.mass if mass is None else mass), **kw)
    return spec, a


def _spawn_box(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    spec, a = _start(cfg, prim_path, translation, orientation, mass=BOX_INERTIA, iterations=64)
    m = a.mat("shell", *PLASTIC)
    clear, white = a.look(CLEAR, rough=0.25, opacity=BOX_OPACITY), a.look(LID_WHITE, rough=0.5)
    for tag, centre, size in box_boards():
        a.bar(f"{prim_path}/{tag}", centre, size, white if tag.startswith("top") else clear, m)
    # Everything but a mint meets the box as a solid block of its outline: 1 mm plates alone let a squeezing
    # fingertip through. The mints do not collide with the block, only with the plates round them.
    core = a.bar(f"{prim_path}/core", (0.0, 0.0, 0.0), (W, T, H), None, a.mat("grip", *RUBBER), patch=PAD_PATCH)
    a.UsdGeom.Imageable(core).CreatePurposeAttr("guide")
    _unblock(prim_path)
    return a.root


def _unblock(box_path: str) -> None:
    """Switch off collisions between the box's solid block and every mint."""
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Sdf, UsdPhysics

    core = get_current_stage().GetPrimAtPath(f"{box_path}/core")
    if not (core and core.IsValid()):
        return
    parent = box_path.rsplit("/", 1)[0]
    targets = [Sdf.Path(_PATHS.get(s.name, f"{parent}/{s.name}")) for s in _SPECS.values() if isinstance(s, Mint)]
    UsdPhysics.FilteredPairsAPI.Apply(core).CreateFilteredPairsRel().SetTargets(targets)


def _spawn_flap(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    spec, a = _start(cfg, prim_path, translation, orientation, iterations=32, ang_damp=0.05)
    m = a.mat("flap", *PLASTIC)
    for tag, centre, size in flap_boards():
        a.bar(f"{prim_path}/{tag}", centre, size, a.look(LID_WHITE, rough=0.5), m)
    return a.root


def _spawn_mint(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    spec, a = _start(cfg, prim_path, translation, orientation, iterations=16, lin_damp=0.02, ang_damp=0.05, depen=0.5)
    a.pill(f"{prim_path}/pill", MINT_L, MINT_D / 2, a.look(MINT_WHITE, rough=0.35), a.mat("mint", *MINT_FRICTION, bounce=MINT_BOUNCE),
           offset=0.001)
    return a.root


def _rigid_object(spec, spawn):
    from isaaclab.sim.spawners.spawner_cfg import RigidObjectSpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    @configclass
    class KitSpawnerCfg(RigidObjectSpawnerCfg):
        func: object = clone(spawn)
        kit_name: str = ""

    _SPECS[spec.name] = spec
    _PATHS.pop(spec.name, None)
    quat = tuple(float(v) for v in getattr(spec, "quat", (0.0, 0.0, 0.0, 1.0)))
    return Object(name=spec.name, object_type=ObjectType.RIGID, spawner_cfg=KitSpawnerCfg(kit_name=spec.name),
                  initial_pose=Pose(position_xyz=tuple(float(v) for v in spec.pos), rotation_xyzw=quat))


_SPAWNERS = {MintBox: _spawn_box, Flap: _spawn_flap, Mint: _spawn_mint}
_BOUNDS: dict = {}


def _points_of(spec) -> np.ndarray:
    if isinstance(spec, MintBox):
        return np.array([[-W / 2, -T / 2, -H / 2], [W / 2, T / 2, H / 2]])
    if isinstance(spec, Flap):
        return np.array([[0.0, -FLAP_W / 2, -FLAP_T], [FLAP_L + LIP, FLAP_W / 2, 0.0]])
    return np.array([[-MINT_L / 2, -MINT_D / 2, -MINT_D / 2], [MINT_L / 2, MINT_D / 2, MINT_D / 2]])


# -- the hinge -----------------------------------------------------------------------------------------------------------

def _flaps() -> list:
    return [s for s in _SPECS.values() if isinstance(s, Flap) and s.box in _SPECS and s.name in _PATHS and s.box in _PATHS]


def _joint_path(flap) -> str:
    return f"{_PATHS[flap.name]}/hinge"


def _author(flap) -> None:
    """Join a flap to its box with a hinge across the box's thickness: 0 (shut) to LID_MAX degrees open, damped."""
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, UsdPhysics

    stage = get_current_stage()
    j = UsdPhysics.RevoluteJoint.Define(stage, _joint_path(flap))
    j.CreateJointEnabledAttr().Set(False)
    j.CreateBody0Rel().SetTargets([_PATHS[flap.box]])
    j.CreateBody1Rel().SetTargets([_PATHS[flap.name]])
    j.CreateLocalPos0Attr().Set(Gf.Vec3f(*(float(v) for v in HINGE)))
    j.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))
    j.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    j.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
    j.CreateExcludeFromArticulationAttr().Set(True)
    j.CreateCollisionEnabledAttr().Set(False)
    j.CreateAxisAttr().Set("Y")
    j.CreateLowerLimitAttr().Set(-float(LID_MAX))          # opening turns the flap about the box's -y
    j.CreateUpperLimitAttr().Set(0.0)
    drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
    drive.CreateTypeAttr().Set("force")
    drive.CreateStiffnessAttr().Set(0.0)
    drive.CreateDampingAttr().Set(float(LID_DAMP * math.pi / 180.0))      # USD angular drives are per degree
    drive.CreateTargetVelocityAttr().Set(0.0)
    j.CreateJointEnabledAttr().Set(True)       # last write, after the frames


def _disarm() -> None:
    """Switch every hinge off (before a reset puts the parts back)."""
    from isaaclab.sim.utils.stage import get_current_stage

    stage = get_current_stage()
    for flap in _flaps():
        prim = stage.GetPrimAtPath(_joint_path(flap))
        if prim and prim.IsValid():
            prim.GetAttribute("physics:jointEnabled").Set(False)


def _arm(environment) -> None:
    """Join the parts as the reset left them."""
    for flap in _flaps():
        _author(flap)
    for spec in _SPECS.values():
        if isinstance(spec, MintBox) and spec.name in _PATHS:
            _unblock(_PATHS[spec.name])
    if not _ENV.get("told"):
        _ENV["told"] = True
        print(f"[mintkit] paths: { {k: v for k, v in list(_PATHS.items())[:3]} }", file=sys.stderr, flush=True)


# -- run time ----------------------------------------------------------------------------------------------------------

def _t(x):
    return getattr(x, "torch", x)


def _np(x) -> np.ndarray:
    if hasattr(x, "numpy") and not hasattr(x, "detach"):
        return np.asarray(x.numpy(), dtype=np.float64)
    return _t(x).detach().cpu().numpy().astype(np.float64)


def _origin(environment) -> np.ndarray:
    return _np(environment.unwrapped.scene.env_origins[0])


def _pose(environment, name: str) -> tuple:
    pose = _np(environment.unwrapped.scene[name].data.root_link_pose_w)[0]
    return pose[:3] - _origin(environment), pose[3:7]


def _wrench(environment, name: str, torque_world, lift: float = 0.0) -> None:
    """Hold a torque (N m) and an upward force (N) on a body until the next call."""
    import torch

    asset = environment.unwrapped.scene[name]
    t = torch.tensor([[[float(v) for v in torque_world]]], dtype=torch.float32, device=asset.device)
    f = torch.tensor([[[0.0, 0.0, float(lift)]]], dtype=torch.float32, device=asset.device)
    asset.permanent_wrench_composer.set_forces_and_torques_index(forces=f, torques=t, is_global=True)


def _snap(environment) -> None:
    """Apply the lid's snap for the coming control step: the torque law on the flap, and its reaction on the box,
    together with the force that carries the box's extra mass."""
    for flap in _flaps():
        try:
            (_, bq), (_, fq) = _pose(environment, flap.box), _pose(environment, flap.name)
        except Exception:                            # noqa: BLE001
            continue
        tau = lid_torque(lid_angle(bq, fq))
        axis = -_rotation(bq)[:, 1]                   # opening turns the flap about the box's -y
        _wrench(environment, flap.name, tau * axis)
        _wrench(environment, flap.box, -tau * axis, lift=(BOX_INERTIA - BOX_MASS) * 9.81)


def write_pose(environment, name: str, pos, quat_xyzw=(0.0, 0.0, 0.0, 1.0)) -> None:
    """Put a loose object at ``pos`` (relative to the env origin), ``quat_xyzw``, at rest."""
    import torch

    asset = environment.unwrapped.scene[name]
    dev = _t(asset.data.root_link_pose_w).device
    pose = torch.tensor([[*(np.asarray(pos, float) + _origin(environment)), *quat_xyzw]], dtype=torch.float32, device=dev)
    vel = torch.zeros((1, 6), dtype=torch.float32, device=dev)
    for stem, kw in (("write_root_pose_to_sim", {"root_pose": pose}), ("write_root_velocity_to_sim", {"root_velocity": vel})):
        fn = getattr(asset, f"{stem}_index", None)
        if fn is not None:
            fn(env_ids=[0], **kw)
        else:
            getattr(asset, stem)(*kw.values())


def _local_bounds(spec) -> tuple:
    key = type(spec)
    if key not in _BOUNDS:
        pts = _points_of(spec)
        _BOUNDS[key] = (pts.min(0), pts.max(0))
    return _BOUNDS[key]


def _kit_state(environment, spec) -> KitState:
    asset = environment.unwrapped.scene[spec.name]
    pose = _np(asset.data.root_link_pose_w)[0]
    pos, quat = pose[:3] - _origin(environment), pose[3:7]
    vel = _np(asset.data.root_vel_w)[0] if hasattr(asset.data, "root_vel_w") else np.zeros(6)
    lo, hi = _local_bounds(spec)
    corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    pts = corners @ _rotation(quat).T + pos
    return KitState(pos=pos, quat_xyzw=quat, lin_vel=vel[:3], ang_vel=vel[3:6], low=pts.min(0), high=pts.max(0))


def finger_points(environment) -> list:
    """The grasp point of each gripper, left then right (world, relative to the env origin)."""
    from yam_arena.constants import TCP_OFFSET

    base = environment.unwrapped
    origin = _origin(environment)
    pts = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        pose = _np(robot.data.body_link_pose_w)[0][list(robot.body_names).index("gripper")]
        pts.append(pose[:3] + _rotation(pose[3:7]) @ np.array(TCP_OFFSET, float) - origin)
    return pts


def finger_frames(environment) -> list:
    """Each gripper's own rotation as the robot has it, left then right: columns are its closing axis, the axis
    across its fingers, and the axis its fingers point along."""
    base = environment.unwrapped
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        pose = _np(robot.data.body_link_pose_w)[0][list(robot.body_names).index("gripper")]
        out.append(_rotation(pose[3:7]))
    return out


def gripper_points(environment) -> list:
    """Points on each gripper that could touch something held between or beside its fingers, left gripper then right
    (world, relative to the env origin): the pad tips, the tool point and the palm, at the fingers' current opening."""
    from yam_arena.constants import PAD_TIP_PAST_TCP, TCP_OFFSET

    base = environment.unwrapped
    origin = _origin(environment)
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        names = list(robot.body_names)
        pose = _np(robot.data.body_link_pose_w)[0][names.index("gripper")]
        rot = _rotation(pose[3:7])
        tcp = pose[:3] + rot @ np.array(TCP_OFFSET, float) - origin
        q, jn = _np(robot.data.joint_pos)[0], list(robot.joint_names)
        half = sum(abs(float(q[jn.index(j)])) for j in ("joint7", "joint8")) / 2 + FINGER_THICK
        pts = [tcp, tcp - rot[:, 2] * PALM_BACK]
        for s in (-1.0, 1.0):
            for along in (PAD_TIP_PAST_TCP, 0.0, -0.03, -PALM_BACK):
                pts.append(tcp + rot[:, 0] * s * half + rot[:, 2] * along)
        out.append(np.array(pts))
    return out


# -- hooks ---------------------------------------------------------------------------------------------------------------

_STEP: list = []
_RESET: list = []
_ENV: dict = {"env": None}


def on_step(fn):
    _STEP.append(fn)
    return fn


def on_reset(fn):
    _RESET.append(fn)
    return fn


def env():
    return _ENV["env"]


def _patch_scene(scene) -> None:
    if getattr(scene, "_mintkit_patched", False):
        return
    stock_object, stock_states = scene.task_object, scene.object_states

    def task_object(spec):
        for kind, spawn in _SPAWNERS.items():
            if isinstance(spec, kind):
                return _rigid_object(spec, spawn)
        return stock_object(spec)

    def object_states(environment, names):
        mine = {n for n in names if isinstance(_SPECS.get(n), tuple(_SPAWNERS))}
        plain = [n for n in names if n not in mine]
        out = stock_states(environment, plain) if plain else {}
        for n in mine:
            out[n] = _kit_state(environment, _SPECS[n])
        return {n: out[n] for n in names}

    scene.task_object = task_object
    scene.object_states = object_states
    scene._mintkit_patched = True


def _patch_embodiment(module) -> None:
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_mintkit_patched", False):
        return
    stock_step, stock_reset = cls._task_step, cls.reset

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        for fn in _STEP:
            fn(self._env)
        _snap(self._env)
        return stock_step(self, *a, **k)

    def reset(self, *a, **k):
        # The hinge has to be whole before the reset's settling steps and first picture: switch it off, let the
        # environment put the parts back, then author it afresh from where the parts are.
        base, saved, had = None, None, False
        try:
            self._ensure_sim(None)
            base = self._env.unwrapped
            env_reset = base.reset
            saved, had = vars(base).get("reset"), "reset" in vars(base)

            def reset_then_arm(*aa, **kk):
                _disarm()
                result = env_reset(*aa, **kk)
                _arm(self._env)
                _snap(self._env)
                return result

            base.reset = reset_then_arm
        except Exception as err:                     # noqa: BLE001
            print(f"[mintkit] hinge not re-armed around the reset: {err!r}", file=sys.stderr, flush=True)
            base = None
        try:
            obs = stock_reset(self, *a, **k)
        finally:
            if base is not None:
                if had:
                    base.reset = saved
                else:
                    del base.reset
        _ENV["env"] = self._env
        for fn in _RESET:
            fn(self._env)
        return obs

    cls._task_step, cls.reset = _task_step, reset
    cls._mintkit_patched = True


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
        stock_exec = loader.exec_module

        def exec_module(module):
            stock_exec(module)
            patch(module)

        loader.exec_module = exec_module
        return spec


def install() -> None:
    """Hook yam_arena (now for modules already imported, otherwise when they are). Idempotent."""
    for name, patch in _Finder.targets.items():
        if name in sys.modules:
            patch(sys.modules[name])
    if not any(isinstance(f, _Finder) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


install()
