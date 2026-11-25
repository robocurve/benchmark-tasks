"""boxkit: a corrugated pizza box blank as rigid panels on crease hinges, for yam_arena task files.

    import pathlib, sys
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
    from boxkit import Blank, on_reset, on_step

    BLANK = Blank("blank", pos=(0.36, 0.0, 0.7871), mass=0.118)
    OBJECTS = [BLANK]

The blank is one die-cut sheet of B-flute board, lying flat with the box's inside face up. In its own frame the base's
centre is the origin, x runs from the box's front to its back, y to the box's left, z up; the board's mid-plane is z = 0.

    front of the box (-x)                                                         back (+x)
    feet | inner leaf | cap | front wall | base (with two slots) | back wall | lid | lid's front flap
                                 tab  -  side wall (each side)              lid's side flap (each side)

Every panel is one rigid body (a few box colliders of the board's thickness) and every crease is a revolute joint of one
articulation whose root is the base. A crease has a spring toward flat and a holding friction, so a folded panel springs
part of the way back and rests there: the scores round the base and the lid's hinge rest about 78 degrees folded; the
short perforated creases (tabs, the roll-over's cap and inner leaf, the lid's flaps) stay about where they are put.

The front wall is a roll-over: wall, a narrow cap, and an inner leaf with two feet. Folded, the leaf stands behind the
two tabs with its feet through the base's slots, the cap lies across the top, and each tab sits in the pocket between
wall and leaf, base and cap. Nothing here is a rule: the lock is contact between the panels.

``success()`` and ``score()`` receive a :class:`BlankState`: the base's pose, every crease's fold angle (degrees, 0 flat,
positive toward the box's inside) and every panel's pose. ``on_step`` and ``on_reset`` register callbacks ``fn(env)``.

Pure Python at import time; Isaac Sim is reached only when the scene is built. The hooks follow the kits used by tasks
already in this repository (a Box subclass so the loader accepts the part, a custom spawner, hooks through
``sys.meta_path``).
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import sys
from dataclasses import dataclass, field

import numpy as np
from yam_arena.task_file import Box, ObjectState

__all__ = ["Blank", "BlankState", "PANELS", "env", "finger_points", "install", "on_reset", "on_step", "fk", "panel_points"]

IN = 0.0254
T = 0.125 * IN                # B-flute board, 1/8 in (the kraft version's listing: "spanning 1/8\" in thickness")
S = 10.0 * IN                 # the box's base, 10 in square (listing: "Length 10 Inches", "Width 10 Inches")
H = 2.0 * IN                  # wall height (listing: "Height 2 Inches")
XF, XB, YS = -S / 2, S / 2, S / 2          # the base's front, back and side creases
E = T / 2 + 0.0005            # boards stop this far short of a crease
# -- sizes below are estimates: no page gives the blank's die line ---------------------------------------------------
TAB_IN = T + 0.0005           # the tabs' crease, in from the front crease: a folded tab lies just inside the front wall
TAB_L = 0.060                 # a tab's length
TAB_Z = (0.0035, 0.0475)      # a tab's lower and upper edge above the base's mid-plane when its wall stands
CAP = 2 * T + 0.0017          # the roll-over's cap, crease to crease: wall, tab and leaf side by side under it
LEAF = H - 0.004              # the inner leaf's height
FOOT = 0.0065                 # each foot's length below the leaf
FOOT_Y = (0.057, 0.083)       # the feet, out from the centre line
SLOT_X = (0.0046, 0.0120)     # the base's two slots, in from the front crease
SLOT_Y = (0.050, 0.090)
LEAF_W = YS - 0.0035          # the leaf's half width (it stands between the side walls)
WALL_BACK = T + 0.0006        # a side wall's rear end, short of the back crease (the back wall stands outside it)
LID_W = YS - T / 2 - 0.0006 - T / 2      # the lid's half width, to its side flaps' creases (the flaps go inside the walls)
LID_L = S - 0.0170             # the lid stops 17 mm short of the front crease: its front flap goes down behind the roll-over
                               # with room to spare, wherever within a few degrees of upright the back wall stands
SIDE_FLAP = (0.030, 0.045, H - 0.0024)   # a side flap: starts this far from the hinge, ends this far short of the front, its height
FRONT_FLAP = (0.110, 0.040)   # the front flap's half width and height
RHO = 0.55                    # kg/m2, B-flute board
MIN_MASS = 0.003              # kg: the lightest panels (cap, tabs) are given this much (a solver aid; real: 1 to 1.5 g)

# name: (parent, axis, pivot (x, y), sign: fold = sign * joint angle, crease class, fold limits in degrees)
PANELS = {
    "base": (None, None, None, 1.0, None, None),
    "wall_l": ("base", "X", (0.0, YS), 1.0, "main", (-60.0, 150.0)),
    "wall_r": ("base", "X", (0.0, -YS), -1.0, "main", (-60.0, 150.0)),
    "tab_l": ("wall_l", "Y", (XF + TAB_IN, YS + H / 2), 1.0, "tab", (-10.0, 150.0)),
    "tab_r": ("wall_r", "Y", (XF + TAB_IN, -YS - H / 2), 1.0, "tab", (-10.0, 150.0)),
    "front": ("base", "Y", (XF, 0.0), 1.0, "main", (-60.0, 130.0)),
    "cap": ("front", "Y", (XF - H, 0.0), 1.0, "roll", (-5.0, 100.0)),
    "leaf": ("cap", "Y", (XF - H - CAP, 0.0), 1.0, "leaf", (-5.0, 130.0)),
    "back": ("base", "Y", (XB, 0.0), -1.0, "main", (-60.0, 130.0)),
    "lid": ("back", "Y", (XB + H, 0.0), -1.0, "main", (-150.0, 130.0)),
    "flap_l": ("lid", "X", (XB + H + LID_L / 2, LID_W), 1.0, "flap", (-20.0, 150.0)),
    "flap_r": ("lid", "X", (XB + H + LID_L / 2, -LID_W), -1.0, "flap", (-20.0, 150.0)),
    "flap_f": ("lid", "Y", (XB + H + LID_L, 0.0), -1.0, "flap", (-20.0, 150.0)),
}
ORDER = tuple(PANELS)
CREASES = tuple(f"crease_{n}" for n in ORDER[1:])
# spring toward flat (N m/rad), holding friction at rest and moving (N m), damping (N m s/rad): estimates
CREASE = {"main": (0.0083, 0.0113, 0.0098, 0.0012),
          "tab": (0.0008, 0.0040, 0.0035, 0.0004),
          "roll": (0.0012, 0.0060, 0.0054, 0.0004),
          "leaf": (0.0006, 0.0028, 0.0025, 0.0003),
          "flap": (0.0020, 0.0080, 0.0070, 0.0006)}
ARMATURE = 2.0e-5             # kg m2 added at every crease (a solver aid)
# A score takes a set. Folding a wall just to upright leaves its score springing back toward flat; folding it on past
# upright ("pre-breaking" the score) leaves the score resting part folded: its rest angle becomes the amount it has been
# folded past upright (an estimate of the amount; the effect itself is what carton makers pre-break creases for).
SET_FROM, SET_MAX = 90.0, 90.0
SET_CREASES = ("wall_l", "wall_r", "front", "back")


def _rect(x0, x1, y0, y1):
    return ((0.5 * (x0 + x1), 0.5 * (y0 + y1), 0.0), (abs(x1 - x0), abs(y1 - y0), T))


def boxes() -> dict:
    """Each panel's colliders as (centre, size) in the blank's frame, blank flat."""
    b = {}
    b["base"] = [_rect(XF + SLOT_X[1], XB - E, -YS + E, YS - E),
                 _rect(XF + E, XF + SLOT_X[0], -YS + E, YS - E),
                 _rect(XF + SLOT_X[0], XF + SLOT_X[1], -SLOT_Y[0], SLOT_Y[0]),
                 _rect(XF + SLOT_X[0], XF + SLOT_X[1], SLOT_Y[1], YS - E),
                 _rect(XF + SLOT_X[0], XF + SLOT_X[1], -YS + E, -SLOT_Y[1])]
    for s, n in ((1.0, "l"), (-1.0, "r")):
        y0, y1 = sorted((s * (YS + E), s * (YS + H)))
        b[f"wall_{n}"] = [_rect(XF + TAB_IN + E, XB - WALL_BACK, y0, y1)]
        y0, y1 = sorted((s * (YS + TAB_Z[0]), s * (YS + TAB_Z[1])))
        b[f"tab_{n}"] = [_rect(XF + TAB_IN - TAB_L, XF + TAB_IN - E, y0, y1)]
        y0, y1 = sorted((s * (LID_W + E), s * (LID_W + SIDE_FLAP[2])))
        b[f"flap_{n}"] = [_rect(XB + H + SIDE_FLAP[0], XB + H + LID_L - SIDE_FLAP[1], y0, y1)]
    b["front"] = [_rect(XF - H + E, XF - E, -YS + 0.0005, YS - 0.0005)]
    b["cap"] = [_rect(XF - H - CAP + 0.0012, XF - H - 0.0012, -YS + T / 2 + 0.0008, YS - T / 2 - 0.0008)]
    x_leaf = XF - H - CAP
    b["leaf"] = [_rect(x_leaf - LEAF, x_leaf - E, -LEAF_W, LEAF_W)] + \
                [_rect(x_leaf - LEAF - FOOT, x_leaf - LEAF, s * FOOT_Y[0], s * FOOT_Y[1]) for s in (1.0, -1.0)]
    b["back"] = [_rect(XB + E, XB + H - E, -YS + 0.0005, YS - 0.0005)]
    b["lid"] = [_rect(XB + H + E, XB + H + LID_L - E, -LID_W + E, LID_W - E)]
    b["flap_f"] = [_rect(XB + H + LID_L + E, XB + H + LID_L + FRONT_FLAP[1], -FRONT_FLAP[0], FRONT_FLAP[0])]
    return b


BOXES = boxes()
ORIGIN = {n: np.array(BOXES[n][0][0]) for n in ORDER}          # each body's origin: its first board's centre, blank flat
ORIGIN["base"] = np.zeros(3)


def masses() -> dict:
    return {n: max(MIN_MASS, RHO * sum(sz[0] * sz[1] for _, sz in BOXES[n])) for n in ORDER}


def _axis_rot(axis: str, a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    if axis == "X":
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], float)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], float)


def fk(folds: dict | None = None) -> dict:
    """Each panel's (rotation, origin) in the blank's frame for the given fold angles (degrees; missing ones are 0)."""
    folds = folds or {}
    out = {"base": (np.eye(3), np.zeros(3))}
    for n in ORDER[1:]:
        parent, axis, pivot, sign, _, _ = PANELS[n]
        Rp, pp = out[parent]
        piv = np.array([pivot[0], pivot[1], 0.0])
        R = _axis_rot(axis, sign * math.radians(float(folds.get(n, 0.0))))
        Rn = Rp @ R
        out[n] = (Rn, pp + Rp @ (piv - ORIGIN[parent]) + Rn @ (ORIGIN[n] - piv))
    return out


def panel_points(name: str, pose, local) -> np.ndarray:
    """Points given in the blank's flat frame, as they ride on panel ``name`` whose pose is (rotation, origin)."""
    R, p = pose
    return (np.atleast_2d(np.asarray(local, float)) - ORIGIN[name]) @ np.asarray(R, float).T + np.asarray(p, float)


def corners(name: str) -> np.ndarray:
    """The eight corners of each of a panel's boards, in the blank's flat frame."""
    pts = []
    for c, sz in BOXES[name]:
        h = np.array(sz) / 2
        pts += [np.array(c) + h * np.array([sx, sy, sz_]) for sx in (-1, 1) for sy in (-1, 1) for sz_ in (-1, 1)]
    return np.array(pts)


def box_distance(point, name: str, pose) -> float:
    """Distance from a point (in the frame the pose is given in) to a panel's boards."""
    R, p = pose
    local = np.asarray(R, float).T @ (np.asarray(point, float) - np.asarray(p, float)) + ORIGIN[name]
    best = 9.0
    for c, sz in BOXES[name]:
        d = np.abs(local - np.array(c)) - np.array(sz) / 2
        best = min(best, float(np.linalg.norm(np.maximum(d, 0.0))))
    return best


@dataclass(frozen=True)
class Blank(Box):
    """The blank, flat, inside face up. ``pos`` is the base's centre on the board's mid-plane; ``yaw`` turns it (0: the
    box's front toward the robot). ``start`` gives starting fold angles in degrees by panel name (tests and probes)."""

    static_friction: float = 0.45
    dynamic_friction: float = 0.40
    combine: str = "average"
    crease_color: tuple = (0.42, 0.34, 0.24)
    contact_offset: float = 0.001
    start: tuple = ()
    meta: dict = field(default_factory=dict, compare=False, hash=False)

    def _check(self) -> None:
        if not self.name.isidentifier():
            raise ValueError(f"object name {self.name!r} must be a Python identifier")
        if self.mass is None or self.mass <= 0:
            raise ValueError(f"{self.name}: a Blank must be loose (give it a mass)")


@dataclass(frozen=True)
class BlankState(ObjectState):
    """The blank as the simulator has it. ``pos``/``quat_xyzw`` are the base's pose. ``folds`` maps each panel to its
    crease's fold angle in degrees; ``speeds`` to the crease's speed in degrees a second; ``panels`` maps each panel to
    (rotation matrix, origin) in the world (relative to the env origin)."""

    folds: dict = field(default_factory=dict)
    speeds: dict = field(default_factory=dict)
    panels: dict = field(default_factory=dict)


def _rotation(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _yaw_xyzw(yaw: float) -> tuple:
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def _material(sim_utils, spec):
    try:
        return sim_utils.RigidBodyMaterialCfg(static_friction=spec.static_friction, dynamic_friction=spec.dynamic_friction,
                                              friction_combine_mode=spec.combine)
    except TypeError:
        return sim_utils.RigidBodyMaterialCfg(static_friction=spec.static_friction, dynamic_friction=spec.dynamic_friction)


# -- spawning ---------------------------------------------------------------------------------------------------------

_BLANKS: dict[str, Blank] = {}


def _spawn_blank(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    """Author the blank under ``prim_path`` (pxr only): thirteen panel bodies and twelve crease joints."""
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics

    from yam_arena.station import linear_rgb

    spec: Blank = _BLANKS[cfg.blank_name]
    stage = get_current_stage()
    double = UsdGeom.XformOp.PrecisionDouble
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=double).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    x, y, z, w = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root.AddOrientOp(precision=double).Set(Gf.Quatd(w, x, y, z))

    phys = f"{prim_path}/Looks/physics"
    pm = _material(sim_utils, spec)
    pm.func(phys, pm)
    looks = {}
    for key, color in (("board", spec.color), ("crease", spec.crease_color)):
        path = f"{prim_path}/Looks/{key}"
        vis = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=0.9)
        vis.func(path, vis)
        looks[key] = path

    mass = masses()
    paths = {}
    for name in ORDER:
        path = f"{prim_path}/{name}"
        xform = UsdGeom.Xform.Define(stage, path)
        xform.AddTranslateOp(precision=double).Set(Gf.Vec3d(*ORIGIN[name]))
        xform.AddOrientOp(precision=double).Set(Gf.Quatd(1, 0, 0, 0))
        prim = xform.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(float(mass[name]))
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateMaxDepenetrationVelocityAttr(0.5)
        rb.CreateLinearDampingAttr(0.05)
        rb.CreateAngularDampingAttr(0.3)
        rb.CreateSleepThresholdAttr(0.0)
        for i, (centre, size) in enumerate(BOXES[name]):
            cube = UsdGeom.Cube.Define(stage, f"{path}/board_{i}")
            cube.CreateSizeAttr(1.0)
            cube.AddTranslateOp(precision=double).Set(Gf.Vec3d(*(np.array(centre) - ORIGIN[name])))
            cube.AddScaleOp().Set(Gf.Vec3f(*size))
            UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
            cp = PhysxSchema.PhysxCollisionAPI.Apply(cube.GetPrim())
            cp.CreateContactOffsetAttr(spec.contact_offset)
            cp.CreateRestOffsetAttr(0.0)
            bind_physics_material(str(cube.GetPath()), phys, stage=stage)
            bind_visual_material(str(cube.GetPath()), looks["board"], stage=stage)
        paths[name] = path

    # crease lines, drawn on the parent's inside face along each crease (no collider)
    for name in ORDER[1:]:
        parent, axis, pivot, sign, kind, lim = PANELS[name]
        (c, sz) = BOXES[name][0]
        length = sz[0] if axis == "X" else sz[1]
        line = UsdGeom.Cube.Define(stage, f"{paths[parent]}/line_{name}")
        line.CreateSizeAttr(1.0)
        along = c[0] if axis == "X" else c[1]
        centre = np.array([along, pivot[1], T / 2 + 0.0002]) if axis == "X" else np.array([pivot[0], along, T / 2 + 0.0002])
        line.AddTranslateOp(precision=double).Set(Gf.Vec3d(*(centre - ORIGIN[parent])))
        line.AddScaleOp().Set(Gf.Vec3f(*((length, 0.0015, 0.0002) if axis == "X" else (0.0015, length, 0.0002))))
        bind_visual_material(str(line.GetPath()), looks["crease"], stage=stage)

    start = dict(spec.start)
    for name in ORDER[1:]:
        parent, axis, pivot, sign, kind, lim = PANELS[name]
        piv = np.array([pivot[0], pivot[1], 0.0])
        j = UsdPhysics.RevoluteJoint.Define(stage, f"{paths[name]}/crease_{name}")
        j.CreateAxisAttr(axis)
        j.CreateBody0Rel().SetTargets([Sdf.Path(paths[parent])])
        j.CreateBody1Rel().SetTargets([Sdf.Path(paths[name])])
        j.CreateLocalPos0Attr(Gf.Vec3f(*(piv - ORIGIN[parent])))
        j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLocalPos1Attr(Gf.Vec3f(*(piv - ORIGIN[name])))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        lo, hi = sorted((sign * lim[0], sign * lim[1]))
        j.CreateLowerLimitAttr(float(lo))
        j.CreateUpperLimitAttr(float(hi))
        k, fs, fd, damp = CREASE[kind]
        drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(math.radians(k))            # USD angular drives are per degree
        drive.CreateDampingAttr(math.radians(damp))
        drive.CreateTargetPositionAttr(0.0)
        pj = PhysxSchema.PhysxJointAPI.Apply(j.GetPrim())
        pj.CreateArmatureAttr(ARMATURE)
        if name in start:
            st = PhysxSchema.JointStateAPI.Apply(j.GetPrim(), "angular")
            st.CreatePositionAttr(float(sign * start[name]))
            st.CreateVelocityAttr(0.0)
    base = stage.GetPrimAtPath(paths["base"])
    UsdPhysics.ArticulationRootAPI.Apply(base)
    art = PhysxSchema.PhysxArticulationAPI.Apply(base)
    art.CreateEnabledSelfCollisionsAttr(True)
    art.CreateSolverPositionIterationCountAttr(48)
    art.CreateSolverVelocityIterationCountAttr(4)
    art.CreateSleepThresholdAttr(0.0)
    return root.GetPrim()


def _body_object(spec: Blank):
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    @configclass
    class BlankSpawnerCfg(SpawnerCfg):
        func: object = clone(_spawn_blank)
        blank_name: str = ""

    class KitObject(Object):
        def _generate_articulation_cfg(self):
            actuators = {}
            for kind, (k, fs, fd, damp) in CREASE.items():
                names = [f"crease_{n}" for n in ORDER[1:] if PANELS[n][4] == kind]
                kw = dict(joint_names_expr=names, stiffness=k, damping=damp, armature=ARMATURE)
                try:
                    actuators[kind] = ImplicitActuatorCfg(friction=fs, dynamic_friction=fd, **kw)
                except TypeError:
                    actuators[kind] = ImplicitActuatorCfg(friction=fs, **kw)
            init = ArticulationCfg.InitialStateCfg(joint_pos={f"crease_{n}": math.radians(PANELS[n][3] * v)
                                                              for n, v in dict(spec.start).items()} or {".*": 0.0})
            cfg = ArticulationCfg(prim_path=self.prim_path, spawn=self.spawner_cfg, actuators=actuators, init_state=init)
            return self._add_initial_pose_to_cfg(cfg)

    return KitObject(name=spec.name, object_type=ObjectType.ARTICULATION,
                     spawner_cfg=BlankSpawnerCfg(blank_name=spec.name),
                     initial_pose=Pose(position_xyz=tuple(float(v) for v in spec.pos), rotation_xyzw=_yaw_xyzw(spec.yaw)))


def _t(x):
    return getattr(x, "torch", x)


def _np(x) -> np.ndarray:
    return _t(x).detach().cpu().numpy().astype(np.float64)


def _blank_state(environment, spec: Blank) -> BlankState:
    base = environment.unwrapped
    asset = base.scene[spec.name]
    origin = _np(base.scene.env_origins[0])
    poses = _np(asset.data.body_link_pose_w)[0]
    names = list(asset.body_names)
    panels, pts = {}, []
    for n in ORDER:
        k = names.index(n)
        R, p = _rotation(poses[k][3:7]), poses[k][:3] - origin
        panels[n] = (R, p)
        pts.append(panel_points(n, (R, p), corners(n)))
    pts = np.concatenate(pts)
    q, qd = _np(asset.data.joint_pos)[0], _np(asset.data.joint_vel)[0]
    jn = list(asset.joint_names)
    folds = {n: math.degrees(PANELS[n][3] * float(q[jn.index(f"crease_{n}")])) for n in ORDER[1:]}
    speeds = {n: abs(math.degrees(float(qd[jn.index(f"crease_{n}")]))) for n in ORDER[1:]}
    k = names.index("base")
    vel = _np(asset.data.body_link_vel_w)[0][k] if hasattr(asset.data, "body_link_vel_w") else np.zeros(6)
    return BlankState(pos=panels["base"][1], quat_xyzw=poses[k][3:7], lin_vel=vel[:3], ang_vel=vel[3:6], low=pts.min(0),
                      high=pts.max(0), folds=folds, speeds=speeds, panels=panels)


def finger_points(environment) -> list:
    """The grasp point of each gripper (relative to the env origin), left then right."""
    from yam_arena.constants import TCP_OFFSET

    base = environment.unwrapped
    origin = _np(base.scene.env_origins[0])
    pts = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        pose = _np(robot.data.body_link_pose_w)[0][list(robot.body_names).index("gripper")]
        pts.append(pose[:3] + _rotation(pose[3:7]) @ np.array(TCP_OFFSET, float) - origin)
    return pts


def hand_rotation(environment, arm: str) -> np.ndarray:
    robot = environment.unwrapped.scene[f"{arm}_robot"]
    return _rotation(_np(robot.data.body_link_pose_w)[0][list(robot.body_names).index("gripper")][3:7])


def set_neutral(environment, name: str, neutral: dict) -> None:
    """Set creases' neutral (spring-rest) fold angles, degrees by panel name, in the running simulation."""
    import torch

    asset = environment.unwrapped.scene[name]
    jn = list(asset.joint_names)
    ids = [jn.index(f"crease_{n}") for n in neutral]
    vals = torch.tensor([[math.radians(PANELS[n][3] * float(v)) for n, v in neutral.items()]], dtype=torch.float32,
                        device=_t(asset.data.joint_pos).device)
    asset.set_joint_position_target(vals, joint_ids=ids)


_SETS: dict = {}
_PRE: dict = dict(__import__("ast").literal_eval(__import__("os").environ.get("BOX_PRE") or "{}"))   # probes only: scores that start pre-broken


def crease_sets(name: str) -> dict:
    """Each pre-breakable score's rest angle now (degrees)."""
    return dict(_SETS.get(name, {}))


def crease_set(most_fold_deg: float) -> float:
    """The material rule (an estimate): the rest angle, in degrees, of a main score that has been folded at most
    ``most_fold_deg``. Nothing until the fold passes upright, then one degree of set for each degree past it."""
    return float(min(SET_MAX, max(0.0, most_fold_deg - SET_FROM)))


def _crease_set(environment) -> None:
    """The material rule, every control step: a main score's rest angle follows how far past upright it has been folded."""
    for name in _BLANKS:
        asset = environment.unwrapped.scene[name]
        q = _np(asset.data.joint_pos)[0]
        jn = list(asset.joint_names)
        sets = _SETS.setdefault(name, {n: float(_PRE.get(n, 0.0)) for n in SET_CREASES})
        for n in SET_CREASES:
            fold = math.degrees(PANELS[n][3] * float(q[jn.index(f"crease_{n}")]))
            sets[n] = max(sets[n], crease_set(fold))
        set_neutral(environment, name, sets)


# -- hooking yam_arena ------------------------------------------------------------------------------------------------

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
    if getattr(scene, "_boxkit_patched", False):
        return
    stock_object, stock_states = scene.task_object, scene.object_states

    def task_object(spec):
        if isinstance(spec, Blank):
            _BLANKS[spec.name] = spec
            return _body_object(spec)
        return stock_object(spec)

    def object_states(environment, names):
        plain = [n for n in names if n not in _BLANKS]
        out = stock_states(environment, plain) if plain else {}
        for n in names:
            if n in _BLANKS:
                out[n] = _blank_state(environment, _BLANKS[n])
        return {n: out[n] for n in names}

    scene.task_object = task_object
    scene.object_states = object_states
    scene._boxkit_patched = True


def _patch_embodiment(module) -> None:
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_boxkit_patched", False):
        return
    stock_step, stock_reset = cls._task_step, cls.reset

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        _crease_set(self._env)
        for fn in _STEP:
            fn(self._env)
        return stock_step(self, *a, **k)

    def reset(self, *a, **k):
        obs = stock_reset(self, *a, **k)
        _ENV["env"] = self._env
        _SETS.clear()
        for fn in _RESET:
            fn(self._env)
        return obs

    cls._task_step, cls.reset = _task_step, reset
    cls._boxkit_patched = True


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
