"""Shut an overpacked hard-shell carry-on with a jumper sleeve hanging out: tuck it in, press the lid down, latch both.

On the table in front of the arms lies an 18 in aluminium-frame hard-shell case (46 x 34 x 21 cm, two lever latches) on its back,
long side along y, hinge along the far side, front face towards the robot (x 20 cm), taped down. It is packed too
full: the two packing cubes bulge, and with a folded chunky jumper on top the load stands about 7 cm proud of the rim at
the jumper's front edge, so the let-down lid rests on it about 17 deg open (10 cm gap at the front). One sleeve of the
jumper hangs out of the front of the fold, across the cubes, over the front rim and down the front face, its cuff
lying on the table. Each latch is a rigid lever hinged on the base below the seam with a hook at its top that passes
over a strike on the lid's front edge only when the lid is shut; within 15 deg of shut a spring snaps it home.

Sim model: case shell, cubes and latches rigid; the load (cubes' bulge + jumper) is a rigid block turning on the lid's
hinge axis with a torsion spring (about 7 N net push-back at the lid's front edge when shut). The sleeve is a reduced
position-based cloth sheet (sleeve.py) stepped every control step from the measured gripper state; it collides with
the shell, the cubes, the jumper, the lid, the latches, the fingers and the hands, and is drawn as a mesh.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score --instruction "<INSTRUCTION>"
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import os
import sys

import numpy as np
from yam_arena.task_file import TABLE_TOP, Box

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sleeve as sleeve_mod  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = "Close the suitcase with nothing caught in it, latch it shut, then let go."

SHELL = (0.62, 0.64, 0.66)  # aluminium
FRAME = (0.30, 0.31, 0.33)
CUBE_COL = (0.10, 0.36, 0.42)  # teal nylon packing cubes
KNIT = (0.66, 0.10, 0.12)  # red knit jumper
CHROME = (0.80, 0.81, 0.83)
RUBBER = (0.06, 0.06, 0.06)

# -- case (fixed base): x 0.20..0.60, y -0.275..0.275, seam 11.5 cm up ------------------------------------------------
BX0, BX1 = 0.20, 0.54
BY = 0.230
DEPTH = BX1 - BX0
WALL = 0.012
SEAM = TOP + 0.105
LID_H = 0.105  # lid half: 10.5 cm; lid top 21 cm above the table when shut
IX0, IX1, IY = BX0 + WALL, BX1 - WALL, BY - WALL  # inner faces
HINGE = np.array([BX1, 0.0, SEAM])  # back top edge, axis along y
LID_MASS = 0.9
CUBE_TOP = SEAM - 0.045  # cube tops round the jumper (front shelf, back, sides): their edges sag below the rim
# load: the jumper on the cubes' bulge, a rigid block on the hinge axis (turns with the lid)
JX0, JX1, JY = 0.31, 0.515, 0.170
CX0 = JX0 - 0.022  # front cube strip ends here: the jumper's lower front edge swings forward to x 29.2 cm
J_H = 0.095  # shut: its top level with the seam (the lid half is packed full behind its divider)
J_MASS = 0.3
LOAD_DEG = 16.9  # rest: top 7 cm proud at its front edge
LOAD_K = 6.0  # N m / rad
LOAD_PRE = 0.417  # rad of preload beyond the rest stop
LID_DEG0 = LOAD_DEG + 0.2  # lid spawned resting on the load

# -- latches (as on a frame case: rigid lever + hook on a hinge on the base, strike on the lid) ----------------------
LATCH_Y = (0.11, -0.11)  # left (+y), right
LIP_D, LIP_H, LIP_W = 0.006, 0.006, 0.050
LIP_Z0 = SEAM + 0.004
LIP_TOP = LIP_Z0 + LIP_H
LAT_T, LAT_W = 0.006, 0.040
PIVOT_X = BX0 - LIP_D - 0.001 - LAT_T / 2
HOOK_D, HOOK_H = 0.0055, 0.006
HOOK_Z0 = LIP_TOP + 0.001
LAT_TOP = HOOK_Z0 + HOOK_H
PIVOT_Z = SEAM - 0.035
LAT_L = LAT_TOP - PIVOT_Z
LAT_MASS, HOOK_MASS = 0.025, 0.006
OPEN_DEG = 30.0
SNAP_DEG = 15.0
SNAP_K = 0.02
SNAP_TARGET = 10.0
LATCH_DEG = 3.0

# -- sleeve ---------------------------------------------------------------------------------------------------------
SLEEVE_Y0 = -0.060
SLEEVE_NC = 8  # 8.75 cm wide
NOLOAD = bool(os.environ.get("SUIT_NOLOAD"))  # scorer check: the jumper taken out and left on the table


def _rot_y(p, ang, about):
    """Rotate world point p about the y axis through `about` by ang (rad); + lifts points in front of `about`."""
    d = np.asarray(p, float) - about
    c, s = math.cos(ang), math.sin(ang)
    return about + np.array([d[0] * c + d[2] * s, d[1], -d[0] * s + d[2] * c])


def _rot_pt(dx: float, dz: float, open_deg: float) -> tuple[float, float]:
    a = math.radians(open_deg)
    return dx * math.cos(a) - dz * math.sin(a), dx * math.sin(a) + dz * math.cos(a)


# objects ------------------------------------------------------------------------------------------------------------
def _case_parts() -> list[Box]:
    xc = (BX0 + BX1) / 2
    h = SEAM - TOP
    z = TOP + h / 2
    return [
        Box("shell_front", pos=(BX0 + WALL / 2, 0.0, z), size=(WALL, 2 * BY, h), color=SHELL),
        Box("shell_back", pos=(BX1 - WALL / 2, 0.0, z), size=(WALL, 2 * BY, h), color=SHELL),
        Box("shell_end_l", pos=(xc, BY - WALL / 2, z), size=(DEPTH - 2 * WALL, WALL, h), color=SHELL),
        Box("shell_end_r", pos=(xc, -BY + WALL / 2, z), size=(DEPTH - 2 * WALL, WALL, h), color=SHELL),
    ]


def _cube_parts() -> list[Box]:
    h = CUBE_TOP - TOP
    z = TOP + h / 2
    yw = 2 * IY
    side = IY - JY - 0.003
    return [
        Box("cubes_front", pos=((IX0 + CX0) / 2, 0.0, z), size=(CX0 - IX0, yw, h), color=CUBE_COL),
        Box("cubes_back", pos=((JX1 + 0.003 + IX1) / 2, 0.0, z), size=(IX1 - JX1 - 0.003, yw, h), color=CUBE_COL),
        Box("cubes_l", pos=((JX0 + JX1) / 2, IY - side / 2, z), size=(JX1 - JX0 + 0.006, side, h), color=CUBE_COL),
        Box("cubes_r", pos=((JX0 + JX1) / 2, -IY + side / 2, z), size=(JX1 - JX0 + 0.006, side, h), color=CUBE_COL),
    ]


PITCH: dict = {}  # spawn rotation about +y (rad) of bodies that do not spawn level

_jc = _rot_y(((JX0 + JX1) / 2, 0.0, SEAM - J_H / 2), math.radians(LOAD_DEG), HINGE)
JUMPER = Box("jumper", pos=tuple(float(v) for v in _jc), size=(JX1 - JX0, 2 * JY, J_H), mass=J_MASS, color=KNIT)
PITCH["jumper"] = math.radians(LOAD_DEG)
if NOLOAD:
    JUMPER = Box("jumper", pos=(0.36, 0.42, TOP + 0.04), size=(0.28, 0.20, 0.08), mass=J_MASS, color=KNIT)
    PITCH.pop("jumper")

_lc = _rot_y((BX0 + DEPTH / 2, 0.0, SEAM + LID_H / 2), math.radians(LID_DEG0), HINGE)
LID = Box("lid", pos=tuple(float(v) for v in _lc), size=(DEPTH, 2 * BY, LID_H), mass=LID_MASS, color=SHELL)
PITCH["lid"] = math.radians(LID_DEG0)

LATCHES, HOOKS, LIPS = [], [], []
for _y, _n in zip(LATCH_Y, ("l", "r")):
    _px, _pz = _rot_pt(0.0, LAT_L / 2, OPEN_DEG)
    LATCHES.append(Box(f"latch_{_n}", pos=(PIVOT_X + _px, _y, PIVOT_Z + _pz), size=(LAT_T, LAT_W, LAT_L),
                       mass=LAT_MASS, color=CHROME))
    _hx, _hz = _rot_pt(LAT_T / 2 + HOOK_D / 2, LAT_L - HOOK_H / 2, OPEN_DEG)
    HOOKS.append(Box(f"hook_{_n}", pos=(PIVOT_X + _hx, _y, PIVOT_Z + _hz), size=(HOOK_D, LAT_W, HOOK_H),
                     mass=HOOK_MASS, color=CHROME))
    PITCH[f"latch_{_n}"] = PITCH[f"hook_{_n}"] = -math.radians(OPEN_DEG)
    _sp = _rot_y((BX0 - LIP_D / 2, _y, LIP_Z0 + LIP_H / 2), math.radians(LID_DEG0), HINGE)
    LIPS.append(Box(f"strike_{_n}", pos=tuple(float(v) for v in _sp), size=(LIP_D, LIP_W, LIP_H), mass=0.01, color=CHROME))
    PITCH[f"strike_{_n}"] = math.radians(LID_DEG0)


class WheelBlock(Box):
    """The wheel housing at the case's -y end (fixed); its spawner also authors all the case's joints."""


WHEELS = WheelBlock("wheel_housing", pos=(BX1 - 0.04, -BY - 0.012, TOP + 0.03), size=(0.05, 0.024, 0.05),
                    color=RUBBER)

OBJECTS = [*_case_parts(), *_cube_parts(), JUMPER, LID, *LATCHES, *HOOKS, *LIPS, WHEELS]

TWIN_EXTRA = [
    {"name": "lid_hinge", "kind": "joint", "real": "the case's piano hinge", "note": "lid 0-85 deg"},
    {"name": "load_spring", "kind": "joint", "real": "overfilled cubes + folded jumper",
     "note": "rigid block on the hinge axis, torsion spring: ~7 N net at the lid's front edge when shut"},
    {"name": "latches", "kind": "joint", "real": "the case's two lever latches",
     "note": "colliding lever+hook on a hinge (open stop 30 deg); spring snaps it shut within 15 deg"},
    {"name": "sleeve", "kind": "body", "real": "the jumper's sleeve (knit, about 9 x 37 cm flat)",
     "size_mm": [370, 88], "colour": "red", "pose": [0.20, -0.016, 0.06]},
]

# -- Isaac side: joints authored when the wheel housing (the last object) is spawned -------------------------------
_P: dict = {}


def _author_joints(stage, parent: str) -> None:
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics

    w = UsdGeom.Xformable(stage.GetPrimAtPath(parent)).ComputeLocalToWorldTransform(0).ExtractTranslation()
    W = np.array([w[0], w[1], w[2]], dtype=float)
    q0 = Gf.Quatf(1.0, 0.0, 0.0, 0.0)

    def v(p):
        return Gf.Vec3f(*(float(x) for x in p))

    def path(o):
        return Sdf.Path(f"{parent}/{o.name}")

    def local(o, p_world):
        d = np.asarray(p_world, float) - np.array(o.pos)
        b = PITCH.get(o.name, 0.0)
        return (d[0] * math.cos(b) - d[2] * math.sin(b), d[1], d[0] * math.sin(b) + d[2] * math.cos(b))

    def joint(kind, name, b0, b1, p_world, axis="Y", lo=None, hi=None):
        j = kind.Define(stage, f"{parent}/wheel_housing/{name}")
        p_world = np.asarray(p_world, float)
        if b0 is not None:
            j.CreateBody0Rel().SetTargets([path(b0)])
            j.CreateLocalPos0Attr(v(local(b0, p_world)))
        else:
            j.CreateLocalPos0Attr(v(W + p_world))
        j.CreateBody1Rel().SetTargets([path(b1)])
        j.CreateLocalPos1Attr(v(local(b1, p_world)))
        j.CreateLocalRot0Attr(q0)
        j.CreateLocalRot1Attr(q0)
        if kind is not UsdPhysics.FixedJoint:
            j.CreateAxisAttr(axis)
            if lo is not None:
                j.CreateLowerLimitAttr(lo)
                j.CreateUpperLimitAttr(hi)
        j.CreateExcludeFromArticulationAttr(True)
        return j

    def drive(j, kind, k, c, target=0.0):
        d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), kind)
        d.CreateTypeAttr("force")
        d.CreateStiffnessAttr(k)
        d.CreateDampingAttr(c)
        d.CreateTargetPositionAttr(target)
        return d

    deg = math.pi / 180.0
    # joint angle = body pitch (local frames identity): + opens
    j = joint(UsdPhysics.RevoluteJoint, "hinge_lid", None, LID, HINGE, lo=0.0, hi=85.0)
    drive(j, "angular", 0.0, 0.02 * deg)
    if not NOLOAD:
        j = joint(UsdPhysics.RevoluteJoint, "hinge_load", None, JUMPER, HINGE, lo=-0.5, hi=LOAD_DEG)
        drive(j, "angular", LOAD_K * deg, 0.4 * deg, LOAD_DEG + math.degrees(LOAD_PRE))
    for lat, hook, lip, y in zip(LATCHES, HOOKS, LIPS, LATCH_Y):
        j = joint(UsdPhysics.RevoluteJoint, f"hinge_{lat.name}", None, lat, (PIVOT_X, y, PIVOT_Z), lo=-OPEN_DEG,
                  hi=0.3)
        drive(j, "angular", 0.0, 0.0005)
        _P[f"hinge_{lat.name}"] = str(j.GetPath())
        joint(UsdPhysics.FixedJoint, f"weld_{hook.name}", lat, hook, hook.pos)
        joint(UsdPhysics.FixedJoint, f"weld_{lip.name}", LID, lip, lip.pos)
    _P["parent"] = parent
    _P["origin"] = W
    CLOTH.reset()
    _render(stage)
    if os.environ.get("SUIT_DEBUG"):
        print(f"[suitcase] joints authored under {parent}; env origin {W}", flush=True)


def _spawn_wheels(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    from isaaclab.sim.spawners.shapes import spawn_cuboid
    from isaaclab.sim.utils.stage import get_current_stage

    prim = spawn_cuboid(prim_path, cfg, translation, orientation, **kwargs)
    _author_joints(get_current_stage(), str(prim_path).rsplit("/", 1)[0])
    return prim


def _wheel_object(spec: WheelBlock):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.schemas import CollisionPropertiesCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose
    from yam_arena.constants import GRIPPY_FRICTION
    from yam_arena.station import linear_rgb

    _register_module()

    @configclass
    class WheelCfg(sim_utils.CuboidCfg):
        func: object = clone(_spawn_wheels)

    cfg = WheelCfg(size=tuple(spec.size), collision_props=CollisionPropertiesCfg(),
                   physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=GRIPPY_FRICTION[0],
                                                                   dynamic_friction=GRIPPY_FRICTION[1]),
                   visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(spec.color), roughness=0.4))
    return Object(name=spec.name, object_type=ObjectType.BASE, spawner_cfg=cfg,
                  initial_pose=Pose(position_xyz=tuple(spec.pos), rotation_xyzw=(0.0, 0.0, 0.0, 1.0)))


def _register_module():
    if __name__ not in sys.modules:
        import types

        mod = types.ModuleType(__name__)
        mod.__dict__.update(globals())
        sys.modules[__name__] = mod


def _patch_scene(scene) -> None:
    if getattr(scene, "_suitcase_patched", False):
        return
    orig = scene.task_object

    def task_object(spec):
        if isinstance(spec, WheelBlock):
            return _wheel_object(spec)
        pitch = PITCH.get(spec.name)
        if pitch is None:
            return orig(spec)
        yaw_q = scene._yaw_xyzw
        scene._yaw_xyzw = lambda _yaw: (0.0, math.sin(pitch / 2), 0.0, math.cos(pitch / 2))
        try:
            return orig(spec)
        finally:
            scene._yaw_xyzw = yaw_q

    scene.task_object = task_object
    scene._suitcase_patched = True


class _Finder(importlib.abc.MetaPathFinder):
    _suitcase = True

    def find_spec(self, fullname, path, target=None):
        if fullname != "yam_arena.scene":
            return None
        sys.meta_path.remove(self)
        try:
            spec = importlib.util.find_spec(fullname)
        finally:
            sys.meta_path.insert(0, self)
        if spec is None or spec.loader is None:
            return None
        orig_exec = spec.loader.exec_module

        def exec_module(module):
            orig_exec(module)
            _patch_scene(module)

        spec.loader.exec_module = exec_module
        return spec


def _install() -> None:
    if "yam_arena.scene" in sys.modules:
        _patch_scene(sys.modules["yam_arena.scene"])
    elif not any(getattr(f, "_suitcase", False) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


_install()


# -- geometry readouts -------------------------------------------------------------------------------------------------
def _finite(a) -> bool:
    return a is not None and bool(np.isfinite(np.asarray(a, dtype=float)).all())


def _pitch(q_xyzw) -> float:
    """Rotation about world +y (rad) of a body that only turns about y (quat in x, y, z, w order)."""
    x, y, z, w = (float(c) for c in q_xyzw)
    return 2.0 * math.atan2(y, w)


def _Ry(b: float) -> np.ndarray:
    c, s = math.cos(b), math.sin(b)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def lid_angle(objects) -> float:
    o = objects[LID.name]
    return _pitch(o.quat_xyzw) if _finite(o.quat_xyzw) else float("nan")


def lid_gap(objects) -> float:
    a = lid_angle(objects)
    return DEPTH * math.sin(a) if math.isfinite(a) else float("nan")


def latch_open(objects, i: int) -> float:
    o = objects[LATCHES[i].name]
    return -math.degrees(_pitch(o.quat_xyzw)) if _finite(o.quat_xyzw) else float("nan")


SHUT_GAP = 0.002


def shut(objects) -> bool:
    g = lid_gap(objects)
    return bool(math.isfinite(g) and g <= SHUT_GAP)


def latched(objects, i: int) -> bool:
    a = latch_open(objects, i)
    return bool(math.isfinite(a) and a <= LATCH_DEG and shut(objects))


def load_in(objects) -> bool:
    """The jumper (the load) lies in the case on the cubes."""
    o = objects[JUMPER.name]
    if not _finite(o.pos):
        return False
    return bool(IX0 < o.pos[0] < IX1 and abs(o.pos[1]) < IY and o.pos[2] < SEAM + 0.06)


# -- the sleeve (cloth surrogate) ------------------------------------------------------------------------------------
def _box_obb(o, size, pitch=None):
    b = _pitch(o.quat_xyzw) if pitch is None else pitch
    return sleeve_mod.obb(o.pos, _Ry(b), np.asarray(size, float) / 2)


def _static_obbs():
    out = []
    for b in [*_case_parts(), *_cube_parts(), WHEELS]:
        out.append(sleeve_mod.obb(b.pos, np.eye(3), np.asarray(b.size) / 2))
    return out


SLEEVE_LEN = 0.37


def _sleeve_path():
    """Sleeve centre line at the start: out of the jumper's front face, across the cubes, over the front rim, down
    the front face and out across the table to the cuff (cut to SLEEVE_LEN)."""
    m = sleeve_mod.MARGIN + 0.001
    jf = _rot_y((JX0, 0, SEAM - 0.020), math.radians(LOAD_DEG), HINGE)
    jb = _rot_y((JX0, 0, SEAM - 0.045), math.radians(LOAD_DEG), HINGE)
    p = np.array([
        [jf[0] - m, 0, jf[2]], [jb[0] - m, 0, jb[2]],
        [CX0 - 0.004, 0, CUBE_TOP + m], [IX0 + 0.004, 0, CUBE_TOP + m],
        [IX0 - 0.002, 0, SEAM + m], [BX0 - m, 0, SEAM + m], [BX0 - m - 0.004, 0, TOP + m + 0.004],
        [BX0 - 0.30, 0, TOP + m],
    ])
    seg = np.linalg.norm(np.diff(p, axis=0), axis=1)
    s = np.cumsum(seg)
    k = int(np.searchsorted(s, SLEEVE_LEN))
    rem = SLEEVE_LEN - (s[k - 1] if k else 0.0)
    end = p[k] + (p[k + 1] - p[k]) * rem / seg[k]
    return np.vstack([p[:k + 1], end])


class _Cloth:
    def __init__(self):
        self.sl = None
        self.mesh = None
        self.count = 0
        self.root_body = None

    def reset(self):
        self.sl = sleeve_mod.Sleeve(_sleeve_path(), SLEEVE_Y0, SLEEVE_NC)
        jp, jR = self._jumper_pose0()
        self.sl.attach_root(jp, jR)
        self.count = 0
        # settle the sheet on the start layout (no grippers near it)
        lid = sleeve_mod.obb(LID.pos, _Ry(PITCH["lid"]), np.asarray(LID.size) / 2)
        boxes = _static_obbs() + [sleeve_mod.obb(jp, jR, np.asarray(JUMPER.size) / 2), lid]
        far = np.array([[0.0, 0.3, 2.0], [0.0, -0.3, 2.0]])
        for _ in range(60):
            self.sl.step(far, np.array([np.eye(3)] * 2), np.array([0.04, 0.04]), boxes, jp, jR, floor=TOP)
        self.sl.v[:] = 0.0

    @staticmethod
    def _jumper_pose0():
        if NOLOAD:  # sleeve sewn to nothing that moves: fix it to the cubes
            return np.array(JUMPER.pos) * 0 + np.array([JX0, 0.0, SEAM]), np.eye(3)
        return np.array(JUMPER.pos), _Ry(PITCH["jumper"])


CLOTH = _Cloth()


def arm_boxes(t, R, offs=(0.01, -0.01)):
    """Collision boxes of one gripper for the sleeve: two fingers (9 cm long, ending 13 mm past the TCP, at the
    measured lateral offsets of the finger links) and the hand body behind them."""
    ax, cl = R[:, 2], R[:, 0]
    tip = t + 0.013 * ax
    out = []
    for k, off in enumerate(offs):
        s = math.copysign(max(abs(off), 0.008), off if off != 0 else (1 if k == 0 else -1))
        out.append(sleeve_mod.obb(tip + s * cl - 0.045 * ax, R, (0.005, 0.011, 0.045)))
    out.append(sleeve_mod.obb(t - 0.105 * ax, R, (0.045, 0.030, 0.030)))
    return out


def _gripper_state():
    """Measured TCP, rotation and finger opening of both arms (env frame), and finger/hand collision boxes."""
    env = _ENV["env"]
    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
    from yam_arena.runtime import link_poses, tcp_world

    tcp, rot, opening, boxes = [], [], [], []
    for k in ("left_robot", "right_robot"):
        rob = base.scene[k]
        lp = link_poses(rob)
        R = lp["gripper"][:3, :3]
        t = np.asarray(tcp_world(rob), dtype=np.float64) - origin
        jn = list(rob.joint_names)
        jp = rob.data.joint_pos.torch[0].detach().cpu().numpy()
        op = abs(float(jp[jn.index("joint7")]))
        tcp.append(t)
        rot.append(R)
        opening.append(op)
        offs = []
        for n in ("tip_left", "tip_right"):
            offs.append(float(np.dot(lp[n][:3, 3] - origin - t, R[:, 0])) if n in lp else 0.0)
        boxes += arm_boxes(t, R, offs)
    return np.array(tcp), np.array(rot), np.array(opening), boxes


def _cloth_step(objects) -> None:
    if CLOTH.sl is None:
        CLOTH.reset()
    if objects[LID.name].pos is None or not _finite(objects[LID.name].pos):
        return
    tcp, rot, opening, gboxes = _gripper_state()
    boxes = list(_static_obbs())
    if NOLOAD:
        jp, jR = CLOTH._jumper_pose0()
    else:
        jo = objects[JUMPER.name]
        jp, jR = np.asarray(jo.pos), _Ry(_pitch(jo.quat_xyzw))
        boxes.append(sleeve_mod.obb(jp, jR, np.asarray(JUMPER.size) / 2))
    lo = objects[LID.name]
    boxes.append(sleeve_mod.obb(lo.pos, _Ry(_pitch(lo.quat_xyzw)), np.asarray(LID.size) / 2))
    for b in [*LATCHES, *HOOKS]:
        boxes.append(_box_obb(objects[b.name], b.size))
    boxes += gboxes
    CLOTH.sl.step(tcp, rot, opening, boxes, jp, jR, floor=TOP)
    CLOTH.count += 1
    if os.environ.get("SUIT_DEBUG") and CLOTH.count % 5 == 0:
        for a in range(2):
            if opening[a] < sleeve_mod.CLOSED and a not in CLOTH.sl.pins:
                loc = (CLOTH.sl.x[CLOTH.sl.edge] - tcp[a]) @ rot[a]
                k = int(np.argmin(np.linalg.norm(loc, axis=1)))
                print(f"[suitcase] arm {a} closed, no pin: tcp={np.round(tcp[a], 3)} nearest edge loc={np.round(loc[k], 3)}"
                      f" R[:,2]={np.round(rot[a][:, 2], 2)} R[:,0]={np.round(rot[a][:, 0], 2)}", flush=True)
    if os.environ.get("SUIT_DEBUG") and CLOTH.count % 40 == 0:
        print(f"[suitcase] cloth step {CLOTH.count}: open={np.round(opening, 4)} tcp_r={np.round(tcp[1], 3)} "
              f"minx={CLOTH.sl.x[:, 0].min():.3f} pins={ {a: len(p.node) for a, p in CLOTH.sl.pins.items()} } "
              f"nboxes={len(boxes)}", flush=True)
    _render()


def _render(st=None) -> None:
    from pxr import Gf, Sdf, UsdGeom, UsdShade, Vt

    st = st or _stage()
    if CLOTH.mesh is None or not CLOTH.mesh.GetPrim().IsValid():
        m = UsdGeom.Mesh.Define(st, "/World/JumperSleeve")
        m.CreateDoubleSidedAttr(True)
        m.CreateSubdivisionSchemeAttr("none")
        mat = UsdShade.Material.Define(st, "/World/JumperSleeveMat")
        sh = UsdShade.Shader.Define(st, "/World/JumperSleeveMat/Shader")
        sh.CreateIdAttr("UsdPreviewSurface")
        from yam_arena.station import linear_rgb

        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*linear_rgb(KNIT)))
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.9)
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(mat)
        m.GetFaceVertexCountsAttr().Set([3] * len(CLOTH.sl.faces))
        m.GetFaceVertexIndicesAttr().Set(CLOTH.sl.faces.ravel().tolist())
        CLOTH.mesh = m
    origin = _P.get("origin", np.zeros(3))
    CLOTH.mesh.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy((CLOTH.sl.x + origin).astype(np.float32)))


def sleeve_nodes():
    return None if CLOTH.sl is None else CLOTH.sl.x.copy()


def sleeve_inside(objects) -> bool:
    """Every point of the sleeve lies inside the case: within the inner faces of the shell walls (4 mm in from the
    front wall's inner face) and below the lid's underside; nothing on the rim, in the seam or outside."""
    x = sleeve_nodes()
    if x is None or CLOTH.sl.bad or not np.isfinite(x).all():
        return False
    a = lid_angle(objects)
    if not math.isfinite(a):
        return False
    inner = (x[:, 0] > IX0 + 0.0015) & (x[:, 0] < IX1) & (np.abs(x[:, 1]) < IY)
    # height above the lid's underside plane (through the hinge axis, turned by the lid angle)
    d = x - HINGE
    above = d[:, 0] * math.sin(a) + d[:, 2] * math.cos(a)
    return bool(np.all(inner & (above < 0.004)))


# -- episode: latch snap + cloth run every control step -------------------------------------------------------------
_ENV: dict = {"env": None}


class _Episode:
    def __init__(self):
        self.snap = [None, None]


_EP = _Episode()


def _stage():
    from isaaclab.sim.utils.stage import get_current_stage

    return get_current_stage()


def _reset_joints() -> None:
    if not _P:
        return
    try:
        st = _stage()
    except Exception:  # noqa: BLE001
        return
    if st is None:
        return
    for lat in LATCHES:
        d = st.GetPrimAtPath(_P.get(f"hinge_{lat.name}", ""))
        if d and d.IsValid():
            d.GetAttribute("drive:angular:physics:stiffness").Set(0.0)


def reset_episode() -> None:
    global _EP
    _EP = _Episode()
    _reset_joints()
    if _P:  # scene built: start the sleeve again from its laid-out start, shown in the reset frame
        CLOTH.reset()
        try:
            _render()
        except Exception as e:  # noqa: BLE001
            print(f"[suitcase] render at reset failed: {e!r}", flush=True)


def _install_hooks() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # noqa: BLE001
        return
    cls = ie.YamArenaEmbodiment
    if not getattr(cls.reset, "_suitcase_hooked", False):
        orig = cls.reset

        def reset(self, *a, **k):
            reset_episode()
            return orig(self, *a, **k)

        reset._suitcase_hooked = True
        cls.reset = reset
    if not getattr(cls._task_step, "_suitcase_hooked", False):
        base = cls._task_step

        def _task_step(self, *a, **k):
            _ENV["env"] = self._env
            out = base(self, *a, **k)
            try:
                from yam_arena.scene import object_states

                names = [LID.name, JUMPER.name, *(o.name for o in LATCHES), *(o.name for o in HOOKS)]
                st = object_states(self._env, names)
                _latch_step(st)
                _cloth_step(st)
            except Exception as e:  # noqa: BLE001
                import traceback

                print(f"[suitcase] step failed: {e!r}", flush=True)
                traceback.print_exc()
            return out

        _task_step._suitcase_hooked = True
        cls._task_step = _task_step


_install_hooks()


def _set_snap(i: int, on: bool) -> None:
    if _EP.snap[i] == on:
        return
    _EP.snap[i] = on
    d = _stage().GetPrimAtPath(_P[f"hinge_{LATCHES[i].name}"])
    d.GetAttribute("drive:angular:physics:stiffness").Set(SNAP_K if on else 0.0)
    d.GetAttribute("drive:angular:physics:targetPosition").Set(SNAP_TARGET)


def _latch_step(objects) -> None:
    for i in range(2):
        a = latch_open(objects, i)
        if math.isfinite(a):
            _set_snap(i, a < SNAP_DEG)


# -- grippers ----------------------------------------------------------------------------------------------------------
def gripper_points() -> list[np.ndarray]:
    env = _ENV["env"]
    if env is None:
        return []
    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
    from yam_arena.runtime import link_poses, pad_tips_world, tcp_world

    pts = []
    for k in ("left_robot", "right_robot"):
        rob = base.scene[k]
        pts += [np.asarray(pad_tips_world(rob), dtype=np.float64) - origin,
                np.asarray(tcp_world(rob), dtype=np.float64) - origin]
        lp = link_poses(rob)
        for n in ("tip_left", "tip_right"):
            if n in lp:
                pts.append(np.asarray(lp[n][:3, 3], dtype=np.float64) - origin)
    return pts


CLEAR = 0.05
CASE_LO = np.array([PIVOT_X - LAT_T / 2 - LAT_L * math.sin(math.radians(OPEN_DEG)), -BY, TOP])
CASE_HI = np.array([BX1, BY, SEAM + LID_H])


def released(objects) -> bool:
    pts = gripper_points()
    if not pts:
        return False
    for p in pts:
        if not _finite(p):
            return False
        if float(np.linalg.norm(np.maximum(np.maximum(CASE_LO - p, p - CASE_HI), 0.0))) < CLEAR:
            return False
    return True


def score(objects) -> float:
    """0 if the jumper is not in the case, any of the sleeve is outside the case or in the seam, or a gripper is within
    5 cm of the case. Else 1.0 both latched (latch shut, hook over the strike, lid within 2 mm of shut); 0.5 one
    latched; 0.25 none latched."""
    if not load_in(objects) or not sleeve_inside(objects) or not released(objects):
        return 0.0
    n = sum(latched(objects, i) for i in range(2))
    return {2: 1.0, 1: 0.5}.get(n, 0.25)


def success(objects) -> bool:
    return bool(score(objects) == 1.0)


def distance(objects) -> float:
    g = lid_gap(objects)
    if not math.isfinite(g):
        return 1.0
    return float(max(0.0, g) + 0.01 * (2 - sum(latched(objects, i) for i in range(2)))
                 + (0.0 if sleeve_inside(objects) else 0.05))


def progress(objects) -> dict:
    x = sleeve_nodes()
    return {"sleeve_in": float(sleeve_inside(objects)),
            "sleeve_min_x": float(x[:, 0].min()) if x is not None else float("nan"),
            "latched_l": float(latched(objects, 0)), "latched_r": float(latched(objects, 1)),
            "gap_mm": lid_gap(objects) * 1000, "latch_l_deg": latch_open(objects, 0),
            "latch_r_deg": latch_open(objects, 1), "load_in": float(load_in(objects)),
            "released": float(released(objects))}


VARIATION: dict = {}
