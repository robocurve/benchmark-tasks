"""Self-serve bakery case on a counter (Cal-Mil 921 class: clear acrylic, 18-1/2 x 14 x 10 in, two tiers, a front
door that lifts up on a top hinge and shuts by gravity).

The case stands on the counter in front of the arms, front face 40 cm out, taped down. Its front door (44 x 21 cm
acrylic, 3 mm) hangs from a hinge along the case's top front edge, with a small pull on its bottom edge left of centre.
It opens upward to a stop at 88 deg (it never stays open by itself) and swings shut under its own weight; let go when
open, it falls and bangs against the frame, so it is lowered by hand until it rests shut. The lower tray holds six glazed ring donuts in two rows of three, 4 cm apart.

Flat pastry tongs (9-1/4 in, sprung open, tips 12.5 cm apart) stand open on their edges on a small tong rest to the
right of the case; an open single-donut bakery box sits beyond it.

The act: hold the door up with one hand, take one donut with the tongs in the other (blades into the 4 cm gaps on
either side of it), put it in the box, lower the door until it rests shut, put the tongs back on their rest.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score --instruction "<INSTRUCTION>"
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import os
import sys

import numpy as np
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = ("Put one donut from the case into the box using the tongs, then lower the case door shut by hand and put "
               "the tongs back on their rest.")

ACRYLIC = (0.80, 0.88, 0.92)
FRAME = (0.15, 0.15, 0.16)
TRAY_COL = (0.92, 0.92, 0.90)
STEEL = (0.74, 0.75, 0.77)
DOUGH = (0.78, 0.52, 0.24)
GLAZE = (0.93, 0.80, 0.56)
HOLE_COL = (0.30, 0.18, 0.08)
BOX_COL = (0.97, 0.97, 0.95)
MAT_COL = (0.20, 0.35, 0.55)
WOOD = (0.62, 0.45, 0.28)
EDGE_COL = (0.55, 0.57, 0.60)

# -- case (fixed): front face x 0.40, 47 wide (y), 35.6 deep (x), 25.4 tall -----------------------------------------
X0 = 0.40
CD, CW, CH = 0.356, 0.470, 0.254  # 14 deep x 18-1/2 wide x 10 tall (in)
X1 = X0 + CD
YW = CW / 2
WALL = 0.006
BASE_H = 0.020  # base panel; the lower tray sits on it
TRAY_T = 0.005
TRAY_TOP = TOP + BASE_H + TRAY_T
TRAY_X0, TRAY_X1 = X0 + 0.012, X0 + 0.012 + 0.330  # 13 x 18 in tray, long side across
UPPER_Z = TOP + 0.135  # upper tier tray (empty), set back
UPPER_X0 = X0 + 0.10

# -- door: hinged along the top front edge, hangs shut; opens upward (+ about world y) ---------------------------
HINGE = np.array([X0 - 0.003, 0.0, TOP + 0.242])
DOOR_L = 0.212  # hinge line to the bottom edge (bottom edge 3 cm above the counter)
DOOR_T = 0.003
DOOR_W = 0.440
DOOR_MASS = 0.33  # 3 mm acrylic 44 x 21 cm, pull and hinge leaf included
DOOR_MAX = 88.0  # top stop (it can never stay open by itself)
DOOR_DAMP = 0.0004  # N m s / deg: light hinge friction
PULL_D, PULL_W, PULL_H = 0.025, 0.012, 0.020  # pull tab on the bottom edge: 2.5 cm out, 1.2 wide, 2 tall
PULL_Y = 0.15  # left of centre
PULL_Z = -DOOR_L + 0.012  # pull centre below the hinge line
SHUT_DEG = 2.0
LETGO_DEG = 10.0  # deg: the door must never be left without a hand on it while open more than this (it would fall)
HAND_ON = 0.015  # m: a gripper point this close to the door panel or its pull = a hand on the door

# -- donuts: 9.5 cm across, 3.5 cm tall, 55 g; two rows of three, 4 cm apart --------------------------------------
DN_R, DN_H, DN_MASS = 0.0475, 0.035, 0.055
DN_PITCH = 2 * DN_R + 0.040
ROW_X = (TRAY_X0 + 0.060, TRAY_X0 + 0.060 + DN_PITCH)
COL_Y = tuple((i - 1) * DN_PITCH for i in range(3))  # -0.135, 0, 0.135
SPOTS = [(x, y) for x in ROW_X for y in COL_Y]
DONUTS = [Cylinder(f"donut_{i}", pos=(x, y, TRAY_TOP + DN_H / 2 + 0.0005), radius=DN_R, height=DN_H, mass=DN_MASS,
                   color=DOUGH) for i, (x, y) in enumerate(SPOTS)]
SPOT_TOL = 0.015

# -- tongs: two sprung stainless arms hinged at the back; flat paddles at the tips ---------------------------------
T_L = 0.235  # 9-1/4 in, hinge pin to tips
T_THK = 0.003  # arm thickness (y when pointing +x)
STRIP_H = 0.016  # arm (strip) height
PAD_L, PAD_H = 0.065, 0.032  # flat paddle at each tip
T_OPEN_TIPS = 0.125  # tip centre-lines apart when sprung fully open
T_HALF = math.asin(T_OPEN_TIPS / 2 / T_L)  # half opening angle (rad), 15.4 deg
STRIP_MASS, PAD_MASS = 0.042, 0.012
T_SPRING_K = 0.020  # N m / deg (torsion spring), preloaded ...
T_SPRING_SET = -30.0  # ... against the open stop: 0.6 N m there (about 5 N squeezed 12 cm from the pin)
T_CLOSE_MAX = 2 * math.degrees(T_HALF)  # tips together
# tong rest: a board (28 x 14 x 0.6 cm) with two wooden bars across it that the open tongs stand on, on their edges
MAT_C = (0.300, -0.340)
MAT_SIZE = (0.280, 0.140, 0.006)
T_HEAD = -math.pi / 4  # the rest (and the tongs on it) point forward-right, away from the case's door
_U = np.array([math.cos(T_HEAD), math.sin(T_HEAD)])
T_HINGE0 = tuple(np.array(MAT_C) - 0.115 * _U)  # hinge pin over the rest's back end, tips toward its front end
BAR_H, BAR_W = 0.050, 0.015  # bars: 5 cm tall, 1.5 cm wide (x)
BAR_D = (0.035, 0.195)  # bar centres: this far from the hinge pin along the tongs
BAR_LEN = (0.050, 0.150)  # bar lengths across (y)
MAT_TOP = TOP + MAT_SIZE[2] + BAR_H  # the tongs stand on the bars' tops


def _arm_specs(prefix: str, sgn: int, hinge_xy, z0: float, half: float):
    """Strip and paddle of one tongs arm (sgn +1: the +y arm), bottoms at z0, opened by `half` (rad)."""
    yaw = T_HEAD + sgn * half
    u = np.array([math.cos(yaw), math.sin(yaw)])
    sc = np.asarray(hinge_xy) + u * (T_L / 2)
    pc = np.asarray(hinge_xy) + u * (T_L - PAD_L / 2)
    strip = Box(f"{prefix}_strip", pos=(float(sc[0]), float(sc[1]), z0 + STRIP_H / 2), size=(T_L, T_THK, STRIP_H),
                mass=STRIP_MASS, color=STEEL, yaw=float(yaw))
    pad = Box(f"{prefix}_pad", pos=(float(pc[0]), float(pc[1]), z0 + PAD_H / 2), size=(PAD_L, T_THK, PAD_H),
              mass=PAD_MASS, color=STEEL, yaw=float(yaw))
    return strip, pad


TONG_A, PAD_A = _arm_specs("tong_a", +1, T_HINGE0, MAT_TOP + 0.0005, T_HALF)
TONG_B, PAD_B = _arm_specs("tong_b", -1, T_HINGE0, MAT_TOP + 0.0005, T_HALF)
TONGS = [TONG_A, PAD_A, TONG_B, PAD_B]

# -- donut box: open single-donut bakery box (white board), 15.5 cm square inside, 6.5 cm walls, lid standing open behind -----------
BOX_C = (0.500, -0.520)
BOX_IN = 0.155
BOX_H = 0.065
BOX_FLOOR = 0.003
BOX_WALL = 0.003


def _case_parts() -> list:
    xc = (X0 + X1) / 2
    top_z = TOP + CH
    return [
        Box("case_base", pos=(xc, 0.0, TOP + BASE_H / 2), size=(CD, CW, BASE_H), color=FRAME),
        Box("lower_tray", pos=((TRAY_X0 + TRAY_X1) / 2, 0.0, TOP + BASE_H + TRAY_T / 2),
            size=(TRAY_X1 - TRAY_X0, CW - 2 * WALL - 0.004, TRAY_T), color=TRAY_COL),
        Box("upper_tray", pos=((UPPER_X0 + X1 - WALL) / 2, 0.0, UPPER_Z), size=(X1 - WALL - UPPER_X0, CW - 2 * WALL - 0.004,
                                                                               TRAY_T), color=TRAY_COL),
        Box("case_side_l", pos=(xc, YW - WALL / 2, TOP + CH / 2), size=(CD, WALL, CH), color=ACRYLIC),
        Box("case_side_r", pos=(xc, -YW + WALL / 2, TOP + CH / 2), size=(CD, WALL, CH), color=ACRYLIC),
        Box("case_back", pos=(X1 - WALL / 2, 0.0, TOP + CH / 2), size=(WALL, CW - 2 * WALL, CH), color=ACRYLIC),
        Box("case_top", pos=(xc, 0.0, top_z - WALL / 2), size=(CD, CW, WALL), color=ACRYLIC),
        Box("case_header", pos=(X0 + 0.005, 0.0, top_z - 0.012), size=(0.010, CW - 2 * WALL, 0.012), color=FRAME),
    ]


def _box_parts() -> list:
    bx, by = BOX_C
    o = BOX_IN / 2 + BOX_WALL / 2
    h = BOX_H
    return [
        Box("dbox_floor", pos=(bx, by, TOP + BOX_FLOOR / 2), size=(BOX_IN + 2 * BOX_WALL, BOX_IN + 2 * BOX_WALL, BOX_FLOOR),
            color=BOX_COL),
        Box("dbox_front", pos=(bx - o, by, TOP + h / 2), size=(BOX_WALL, BOX_IN, h), color=BOX_COL),
        Box("dbox_back", pos=(bx + o, by, TOP + h / 2), size=(BOX_WALL, BOX_IN, h), color=BOX_COL),
        Box("dbox_left", pos=(bx, by + o, TOP + h / 2), size=(BOX_IN + 2 * BOX_WALL, BOX_WALL, h), color=BOX_COL),
        Box("dbox_right", pos=(bx, by - o, TOP + h / 2), size=(BOX_IN + 2 * BOX_WALL, BOX_WALL, h), color=BOX_COL),
        # lid hinged at the back top edge, standing open (leaning back 15 deg is cosmetic: modelled upright)
        Box("dbox_lid", pos=(bx + o + 0.004, by, TOP + h + BOX_IN / 2), size=(BOX_WALL, BOX_IN + 2 * BOX_WALL, BOX_IN),
            color=BOX_COL),
    ]


DOOR = Box("case_door", pos=(float(HINGE[0]), 0.0, float(HINGE[2] - DOOR_L / 2)), size=(DOOR_T, DOOR_W, DOOR_L),
           mass=DOOR_MASS, color=ACRYLIC)
DOOR_PULL = Box("door_pull", pos=(float(HINGE[0] - DOOR_T / 2 - PULL_D / 2), PULL_Y, float(HINGE[2] + PULL_Z)),
                size=(PULL_D, PULL_W, PULL_H), mass=0.01, color=FRAME)
MAT = Box("tong_mat", pos=(MAT_C[0], MAT_C[1], TOP + MAT_SIZE[2] / 2), size=MAT_SIZE, color=MAT_COL, yaw=T_HEAD)
BARS = [Box(f"rest_bar{i + 1}", pos=(T_HINGE0[0] + d * _U[0], T_HINGE0[1] + d * _U[1], TOP + MAT_SIZE[2] + BAR_H / 2),
            size=(BAR_W, ln, BAR_H), yaw=T_HEAD,
            color=WOOD) for i, (d, ln) in enumerate(zip(BAR_D, BAR_LEN))]


class CaseHeader(Box):
    """The case's top header (fixed); its spawner also authors the joints and the donut / tongs looks."""


_parts = _case_parts()
HEADER = CaseHeader(_parts[-1].name, pos=_parts[-1].pos, size=_parts[-1].size, color=_parts[-1].color)
OBJECTS = [*_parts[:-1], *_box_parts(), MAT, *BARS, DOOR, DOOR_PULL, *DONUTS, *TONGS, HEADER]
ACRYLIC_NAMES = {"case_side_l", "case_side_r", "case_back", "case_top", "upper_tray", DOOR.name}  # clear: drawn as edges

TWIN_EXTRA = [
    {"name": "door_hinge", "kind": "joint", "real": "the case's front serving door hinge (top edge)",
     "note": "revolute 0-88 deg about the top front edge; gravity shuts it; light friction"},
    {"name": "tongs_hinge", "kind": "joint", "real": "flat pastry tongs' spring end",
     "note": "revolute between the two arms, torsion spring preloaded 0.6 N m at the open stop (tips 12.5 cm)"},
    {"name": "donut_looks", "kind": "decal", "size_mm": [22, 22], "colour": "dark brown",
     "pose": "hole look on each donut's top (collider is a solid disc)"},
]

# -- Isaac side -------------------------------------------------------------------------------------------------------
_P: dict = {}
_OBJ = {o.name: o for o in OBJECTS}


def _yaw_q(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def _rotz(yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def _author_joints(stage, parent: str) -> None:
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics

    w = UsdGeom.Xformable(stage.GetPrimAtPath(parent)).ComputeLocalToWorldTransform(0).ExtractTranslation()
    W = np.array([w[0], w[1], w[2]], dtype=float)

    def v(p):
        return Gf.Vec3f(*(float(x) for x in p))

    def path(o):
        return Sdf.Path(f"{parent}/{o.name}")

    def local(o, p_world):
        return tuple(_rotz(o.yaw).T @ (np.asarray(p_world, float) - np.array(o.pos)))

    def qf(yaw):
        return Gf.Quatf(float(math.cos(yaw / 2)), 0.0, 0.0, float(math.sin(yaw / 2)))

    def joint(kind, name, b0, b1, p_world, axis="Y", lo=None, hi=None):
        j = kind.Define(stage, f"{parent}/{HEADER.name}/{name}")
        if b0 is not None:
            j.CreateBody0Rel().SetTargets([path(b0)])
            j.CreateLocalPos0Attr(v(local(b0, p_world)))
            j.CreateLocalRot0Attr(qf(-b0.yaw))  # joint frames along the world axes at the spawn
        else:
            j.CreateLocalPos0Attr(v(W + np.asarray(p_world, float)))
            j.CreateLocalRot0Attr(qf(0.0))
        j.CreateBody1Rel().SetTargets([path(b1)])
        j.CreateLocalPos1Attr(v(local(b1, p_world)))
        j.CreateLocalRot1Attr(qf(-b1.yaw))
        if kind is not UsdPhysics.FixedJoint:
            j.CreateAxisAttr(axis)
            if lo is not None:
                j.CreateLowerLimitAttr(lo)
                j.CreateUpperLimitAttr(hi)
        j.CreateExcludeFromArticulationAttr(True)
        _P[name] = str(j.GetPath())
        return j

    def drive(j, kind, k, c, target=0.0):
        d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), kind)
        d.CreateTypeAttr("force")
        d.CreateStiffnessAttr(k)
        d.CreateDampingAttr(c)
        d.CreateTargetPositionAttr(target)
        return d

    # door: + about world y opens it (bottom edge swings out towards the robot and up)
    j = joint(UsdPhysics.RevoluteJoint, "hinge_door", None, DOOR, HINGE, axis="Y", lo=0.0, hi=DOOR_MAX)
    drive(j, "angular", 0.0, DOOR_DAMP)
    # the pull is part of the door (fixed to it, as on the real case)
    joint(UsdPhysics.FixedJoint, "weld_pull", DOOR, DOOR_PULL, DOOR_PULL.pos)
    # tongs: arm b turns about z relative to arm a at the hinge pin; 0 = sprung fully open, + closes
    hz = MAT_TOP + 0.0005 + STRIP_H / 2
    pin = (T_HINGE0[0], T_HINGE0[1], hz)
    # body0 = arm b, body1 = arm a; both joint frames along world x at the spawn (0 = sprung fully open). Arm a turning
    # -z relative to arm b closes the tips, so the travel is [-T_CLOSE_MAX, 0] and the spring's set point lies past 0.
    j = joint(UsdPhysics.RevoluteJoint, "hinge_tongs", TONG_B, TONG_A, pin, axis="Z", lo=-T_CLOSE_MAX, hi=0.0)
    drive(j, "angular", T_SPRING_K, 0.0002, -T_SPRING_SET)
    joint(UsdPhysics.FixedJoint, "weld_pad_a", TONG_A, PAD_A, PAD_A.pos)
    joint(UsdPhysics.FixedJoint, "weld_pad_b", TONG_B, PAD_B, PAD_B.pos)

    def look(prim, name, centre, size, color, cyl=False):
        if cyl:
            g = UsdGeom.Cylinder.Define(stage, f"{prim}/{name}")
            g.CreateRadiusAttr(float(size[0]))
            g.CreateHeightAttr(float(size[1]))
            g.CreateAxisAttr("Z")
        else:
            g = UsdGeom.Cube.Define(stage, f"{prim}/{name}")
            g.CreateSizeAttr(1.0)
        g.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        x = UsdGeom.Xformable(g)
        x.AddTranslateOp().Set(Gf.Vec3d(*map(float, centre)))
        if not cyl:
            x.AddScaleOp().Set(Gf.Vec3f(*map(float, size)))

    # clear acrylic: the panels' own meshes are hidden (they still collide); their edges are drawn instead
    from pxr import Usd

    for n in ACRYLIC_NAMES:
        root = stage.GetPrimAtPath(f"{parent}/{n}")
        for pr in Usd.PrimRange(root):
            if pr.IsA(UsdGeom.Mesh) or pr.IsA(UsdGeom.Cube) or pr.IsA(UsdGeom.Gprim):
                UsdGeom.Imageable(pr).MakeInvisible()
    hp = f"{parent}/{HEADER.name}"
    hc = np.array(HEADER.pos)
    e = 0.008
    zt = TOP + CH
    xs, ys, zs = (X0, X1), (-YW, YW), (TOP, zt)
    k = 0
    for y in ys:
        for z in zs:
            look(hp, f"edge{k}", (((X0 + X1) / 2) - hc[0], y - hc[1], z - hc[2]), (CD, e, e), EDGE_COL); k += 1
    for x in xs:
        for z in zs:
            look(hp, f"edge{k}", (x - hc[0], -hc[1], z - hc[2]), (e, CW, e), EDGE_COL); k += 1
    for x in xs:
        for y in ys:
            look(hp, f"edge{k}", (x - hc[0], y - hc[1], TOP + CH / 2 - hc[2]), (e, e, CH), EDGE_COL); k += 1
    # upper tray outline
    ux0, ux1 = UPPER_X0, X1 - WALL
    for y in (-YW + 0.01, YW - 0.01):
        look(hp, f"edge{k}", ((ux0 + ux1) / 2 - hc[0], y - hc[1], UPPER_Z - hc[2]), (ux1 - ux0, 0.006, 0.006), EDGE_COL); k += 1
    look(hp, f"edge{k}", (ux0 - hc[0], -hc[1], UPPER_Z - hc[2]), (0.006, CW - 0.02, 0.006), EDGE_COL)
    # door: a frame round its edges (in the door's own frame; it turns with it)
    dp = f"{parent}/{DOOR.name}"
    fw = 0.012
    look(dp, "frame_t", (0, 0, DOOR_L / 2 - fw / 2), (DOOR_T + 0.002, DOOR_W, fw), EDGE_COL)
    look(dp, "frame_b", (0, 0, -DOOR_L / 2 + fw / 2), (DOOR_T + 0.002, DOOR_W, fw), EDGE_COL)
    for sgn in (-1, 1):
        look(dp, f"frame_{'lr'[sgn > 0]}", (0, sgn * (DOOR_W / 2 - fw / 2), 0), (DOOR_T + 0.002, fw, DOOR_L), EDGE_COL)
    for d in DONUTS:
        look(f"{parent}/{d.name}", "glaze", (0, 0, DN_H / 2 + 0.0005), (DN_R - 0.004, 0.001), GLAZE, cyl=True)
        look(f"{parent}/{d.name}", "hole", (0, 0, DN_H / 2 + 0.0012), (0.011, 0.001), HOLE_COL, cyl=True)
    # spring end of the tongs (on arm a, near the pin)
    look(f"{parent}/{TONG_A.name}", "spring_end", (-T_L / 2 + 0.012, -0.004, 0.0), (0.030, 0.010, STRIP_H + 0.002),
         STEEL)
    if os.environ.get("DONUT_DEBUG"):
        print(f"[donut] joints authored under {parent}; env origin {W}", flush=True)


def _spawn_header(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    from isaaclab.sim.spawners.shapes import spawn_cuboid
    from isaaclab.sim.utils.stage import get_current_stage

    prim = spawn_cuboid(prim_path, cfg, translation, orientation, **kwargs)
    _author_joints(get_current_stage(), str(prim_path).rsplit("/", 1)[0])
    return prim


def _header_object(spec: CaseHeader):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.schemas import CollisionPropertiesCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose
    from yam_arena.station import linear_rgb

    _register_module()

    @configclass
    class HeaderCfg(sim_utils.CuboidCfg):
        func: object = clone(_spawn_header)

    cfg = HeaderCfg(size=tuple(spec.size), collision_props=CollisionPropertiesCfg(),
                    visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(spec.color), roughness=0.4))
    return Object(name=spec.name, object_type=ObjectType.BASE, spawner_cfg=cfg,
                  initial_pose=Pose(position_xyz=tuple(spec.pos), rotation_xyzw=(0.0, 0.0, 0.0, 1.0)))


def _register_module():
    if __name__ not in sys.modules:  # configclass needs this module registered (task files load unregistered)
        import types

        mod = types.ModuleType(__name__)
        mod.__dict__.update(globals())
        sys.modules[__name__] = mod


def _patch_scene(scene) -> None:
    if getattr(scene, "_donut_patched", False):
        return
    orig = scene.task_object

    def task_object(spec):
        if isinstance(spec, CaseHeader):
            return _header_object(spec)
        return orig(spec)

    scene.task_object = task_object
    scene._donut_patched = True


class _Finder(importlib.abc.MetaPathFinder):
    _donut = True

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
    elif not any(getattr(f, "_donut", False) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


_install()

# -- episode: latched events, updated every control step ---------------------------------------------------------------
_ENV: dict = {"env": None}
WATCH = [DOOR.name, *(d.name for d in DONUTS), *(t.name for t in TONGS)]


class _Episode:
    def __init__(self):
        self.touched = False  # a gripper touched a donut
        self.dropped = False  # a donut touched the counter or left the tray other than into the box
        self.let_go = False  # the door was left without a hand on it while open more than LETGO_DEG
        self.lifted = False  # a donut was lifted clear of the tray pinched between the tongs' paddles
        self.unpinched = False  # a donut was off the tray, not over the box, without both paddles on it (latched)
        self.pinched = set()  # donuts that have been lifted clear of the tray pinched between both paddles
        self.prev_door = 0.0
        self.max_door = 0.0
        self.why = ""
        self.last = None
        self.grip_hist = []  # gripper points at the last PARK_STEPS + 1 control steps


_EP = _Episode()


def reset_episode() -> None:
    global _EP
    _EP = _Episode()


def _install_hooks() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # noqa: BLE001  (offline)
        return
    cls = ie.YamArenaEmbodiment
    if not getattr(cls.reset, "_donut_hooked", False):
        orig = cls.reset

        def reset(self, *a, **k):
            reset_episode()
            return orig(self, *a, **k)

        reset._donut_hooked = True
        cls.reset = reset
    if not getattr(cls._task_step, "_donut_hooked", False):
        base = cls._task_step

        def _task_step(self, *a, **k):
            _ENV["env"] = self._env
            out = base(self, *a, **k)
            try:
                from yam_arena.scene import object_states

                st = object_states(self._env, WATCH)
                dt = float(getattr(getattr(self._env, "unwrapped", self._env), "step_dt", 0.05))
                _update(st, dt)
                gp = gripper_points()
                if gp:
                    _EP.grip_hist = (_EP.grip_hist + [np.array(gp, dtype=np.float64)])[-(PARK_STEPS + 1):]
            except Exception as e:  # noqa: BLE001
                print(f"[donut] step hook failed: {e!r}", flush=True)
            return out

        _task_step._donut_hooked = True
        cls._task_step = _task_step


_install_hooks()


# -- geometry readouts -------------------------------------------------------------------------------------------------
def _finite(a) -> bool:
    return a is not None and bool(np.isfinite(np.asarray(a, dtype=float)).all())


def door_angle(objects) -> float:
    """Door opening (deg): 0 hanging shut, + open (rotation about world +y)."""
    o = objects[DOOR.name]
    if not _finite(o.quat_xyzw):
        return float("nan")
    x, y, z, w = (float(c) for c in o.quat_xyzw)
    return math.degrees(2.0 * math.atan2(y, w))


def door_shut(objects) -> bool:
    a = door_angle(objects)
    return bool(math.isfinite(a) and a <= SHUT_DEG)


def _xy(o) -> np.ndarray:
    return np.asarray(o.pos, float)[:2]


def over_box(o) -> bool:
    """Donut centre over the box's inside (released over it, it drops in)."""
    if not _finite(o.pos):
        return False
    return bool(np.all(np.abs(_xy(o) - np.asarray(BOX_C)) <= BOX_IN / 2))


def in_box(o) -> bool:
    """Donut in the box: its centre over the box's inside and its underside below the box's walls' top."""
    if not (_finite(o.pos) and _finite(o.low)):
        return False
    d = np.abs(_xy(o) - np.asarray(BOX_C))
    return bool(np.all(d <= BOX_IN / 2) and o.low[2] <= TOP + BOX_H - 0.01)


def on_spot(o, i: int) -> bool:
    """Donut i still on the tray within SPOT_TOL of its spot, lying flat on it."""
    if not (_finite(o.pos) and _finite(o.low)):
        return False
    return bool(np.linalg.norm(_xy(o) - np.asarray(SPOTS[i])) <= SPOT_TOL and abs(o.low[2] - TRAY_TOP) <= 0.006)


def on_tray(o) -> bool:
    if not (_finite(o.pos) and _finite(o.low)):
        return False
    x, y = _xy(o)
    return bool(TRAY_X0 <= x <= TRAY_X1 and abs(y) <= YW - WALL and o.low[2] <= TRAY_TOP + 0.006)


def _in_case_or_box_xy(o) -> bool:
    x, y = _xy(o)
    in_case = X0 <= x <= X1 and abs(y) <= YW
    in_bx = bool(np.all(np.abs(np.array([x, y]) - np.asarray(BOX_C)) <= BOX_IN / 2 + BOX_WALL))
    return bool(in_case or in_bx)


def tongs_on_mat(objects) -> bool:
    """The tongs are back on their rest: the middle of each arm and of each paddle lies over the rest's board, and each
    part's underside is at the bars' top height (8 mm below to 10 mm above)."""
    for t in TONGS:
        o = objects[t.name]
        if not (_finite(o.low) and _finite(o.high) and _finite(o.pos)):
            return False
        loc = _rotz(T_HEAD).T @ (np.asarray(o.pos, float) - np.array([MAT_C[0], MAT_C[1], 0.0]))
        if abs(loc[0]) > MAT_SIZE[0] / 2 or abs(loc[1]) > MAT_SIZE[1] / 2:
            return False
        if not (MAT_TOP - 0.008 <= o.low[2] <= MAT_TOP + 0.01):
            return False
    return True


# -- grippers ----------------------------------------------------------------------------------------------------------
def _robots():
    env = _ENV["env"]
    if env is None:
        return None, None
    base = env.unwrapped
    return base, base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)


def gripper_points() -> list[np.ndarray]:
    base, origin = _robots()
    if base is None:
        return []
    from yam_arena.runtime import link_poses, pad_tips_world, tcp_world

    pts = []
    for k in ("left_robot", "right_robot"):
        rob = base.scene[k]
        t = np.asarray(tcp_world(rob), dtype=np.float64) - origin
        tip = np.asarray(pad_tips_world(rob), dtype=np.float64) - origin
        pts += [tip, t, (tip + t) / 2]
        lp = link_poses(rob)
        for n in ("tip_left", "tip_right"):
            if n in lp:
                pts.append(np.asarray(lp[n][:3, 3], dtype=np.float64) - origin)
    return pts


def _box_dist(p, lo, hi) -> float:
    return float(np.linalg.norm(np.maximum(np.maximum(np.asarray(lo) - p, p - np.asarray(hi)), 0.0)))


TOUCH = 0.006  # a gripper point this close to a donut's bounding box counts as a touch


def _touching_donut(objects, pts) -> bool:
    for d in DONUTS:
        o = objects[d.name]
        if not (_finite(o.low) and _finite(o.high)):
            continue
        for p in pts:
            if _finite(p) and _box_dist(p, o.low, o.high) < TOUCH:
                return True
    return False


def _update(objects, dt: float = 0.05) -> None:
    if objects is _EP.last:
        return
    _EP.last = objects
    pts = gripper_points()
    if not _EP.touched and pts and _touching_donut(objects, pts):
        _EP.touched, _EP.why = True, "a gripper touched a donut"
    for i, d in enumerate(DONUTS):
        o = objects[d.name]
        if not (_finite(o.pos) and _finite(o.low)):
            if not _EP.dropped:
                _EP.dropped, _EP.why = True, f"{d.name} state lost"
            continue
        if not _EP.dropped and o.low[2] < TOP + 0.004 and not _in_case_or_box_xy(o):
            _EP.dropped, _EP.why = True, f"{d.name} dropped on the counter"
        if o.low[2] > TRAY_TOP + 0.010 and not in_box(o):  # off the tray, not yet in the box
            if pinched_by_tongs(objects, o):
                _EP.pinched.add(i)
                if not _EP.touched:
                    _EP.lifted = True
            elif not over_box(o) and not _EP.unpinched:  # carried or knocked up without both paddles on it
                _EP.unpinched, _EP.why = True, f"{d.name} off the tray without being pinched in the tongs"
    a = door_angle(objects)
    if math.isfinite(a):
        if not _EP.let_go and a > LETGO_DEG and pts and hand_to_door(pts, a) > HAND_ON:
            _EP.let_go, _EP.why = True, f"door let go {a:.0f} deg open"
        _EP.prev_door = a
        _EP.max_door = max(_EP.max_door, a)


def _rot_y(deg: float) -> np.ndarray:
    a = math.radians(deg)
    c, s_ = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s_], [0.0, 1.0, 0.0], [-s_, 0.0, c]])


# the door panel and its pull in the hinge frame (door shut): (low, high) corners
_DOOR_BOXES = [(np.array([-DOOR_T / 2, -DOOR_W / 2, -DOOR_L]), np.array([DOOR_T / 2, DOOR_W / 2, 0.0])),
               (np.array([-DOOR_T / 2 - PULL_D, PULL_Y - PULL_W / 2, PULL_Z - PULL_H / 2]),
                np.array([-DOOR_T / 2, PULL_Y + PULL_W / 2, PULL_Z + PULL_H / 2]))]


def hand_to_door(pts, deg: float) -> float:
    """Distance (m) from the nearest gripper point to the door panel or its pull, the door opened by deg."""
    r = _rot_y(deg)
    best = math.inf
    for p in pts:
        if not _finite(p):
            continue
        q = r.T @ (np.asarray(p, float) - np.asarray(HINGE, float))
        best = min(best, *(_box_dist(q, lo, hi) for lo, hi in _DOOR_BOXES))
    return best


PARK = 0.10  # parked: every gripper point at least 10 cm from the tongs, the case and the box ...
PARK_SPEED = 0.02  # ... moving under 2 cm/s ...
PARK_STEPS = 10  # ... over the last 0.5 s (10 control steps at 20 Hz)
CONTROL_HZ = 20.0
_CASE_LO = np.array([X0 - DOOR_L - 0.02, -YW, TOP])  # case with its door at any angle
_CASE_HI = np.array([X1, YW, TOP + CH])


def _zones(objects):
    """Boxes the parked grippers keep clear of: the case (shut door and pull included), the box (walls and the open
    lid standing behind) and each tong part."""
    zones = [(np.array([HINGE[0] - DOOR_T - PULL_D, -YW, TOP]), _CASE_HI),
             (np.array([BOX_C[0] - BOX_IN / 2 - BOX_WALL, BOX_C[1] - BOX_IN / 2 - BOX_WALL, TOP]),
              np.array([BOX_C[0] + BOX_IN / 2 + BOX_WALL, BOX_C[1] + BOX_IN / 2 + BOX_WALL, TOP + BOX_H + BOX_IN]))]
    for t in TONGS:
        o = objects[t.name]
        if not (_finite(o.low) and _finite(o.high)):
            return None
        zones.append((np.asarray(o.low), np.asarray(o.high)))
    return zones


def parked(objects) -> bool:
    """Both arms parked: over the last 0.5 s every gripper point stayed 10 cm or more from the case, the box and the
    tongs and moved under 2 cm/s (so the tongs have been let go)."""
    hist = _EP.grip_hist
    zones = _zones(objects)
    if len(hist) < PARK_STEPS + 1 or zones is None:
        return False
    for k, frame in enumerate(hist):
        if not np.all(np.isfinite(frame)):
            return False
        for p in frame:
            for lo, hi in zones:
                if _box_dist(p, lo, hi) < PARK:
                    return False
        if k and float(np.linalg.norm(frame - hist[k - 1], axis=1).max()) * CONTROL_HZ >= PARK_SPEED:
            return False
    return True


def gated(objects) -> bool:
    _update(objects)
    return bool(_EP.touched or _EP.dropped or _EP.let_go or _EP.unpinched)


def end_ok(objects) -> bool:
    """Every donut either in the box or on the tray."""
    for d in DONUTS:
        o = objects[d.name]
        if not (in_box(o) or on_tray(o)):
            return False
    return True


def boxed(objects) -> list[int]:
    """Donuts in the box (however they got there)."""
    return [i for i, d in enumerate(DONUTS) if in_box(objects[d.name])]


PINCH = 0.006  # m: each paddle's bounding box this close to the donut's = the donut is between the paddles


def pinched_by_tongs(objects, o) -> bool:
    """Both tong paddles at the donut (bounding boxes within PINCH) and on opposite sides of its centre (in plan): held
    between them, not scooped on one."""
    pa, pb = objects[PAD_A.name], objects[PAD_B.name]
    if not (_finite(pa.pos) and _finite(pb.pos)):
        return False
    c = np.asarray(o.pos, float)[:2]
    if float(np.dot(np.asarray(pa.pos, float)[:2] - c, np.asarray(pb.pos, float)[:2] - c)) >= 0.0:
        return False
    for pad in (PAD_A, PAD_B):
        q = objects[pad.name]
        if not (_finite(q.low) and _finite(q.high)):
            return False
        gap = np.maximum(np.maximum(np.asarray(q.low) - np.asarray(o.high), np.asarray(o.low) - np.asarray(q.high)), 0.0)
        if float(np.linalg.norm(gap)) > PINCH:
            return False
    return True


def score(objects) -> float:
    """0 if a gripper ever touched a donut, a donut touched the counter or ends anywhere but the tray or the box, or
    a donut was ever off the tray (1 cm up), not in or over the box, without both tong paddles on it, or
    the door was ever left without a hand on it (no gripper point within 1.5 cm of the door or its pull) while open more
    than 10 deg. Else 1.0 exactly one donut in the box, the other five
    within 1.5 cm of their spots, door shut (within 2 deg), tongs on their rest and both arms parked (every gripper
    point 10 cm or more from the tongs, case and box, moving under 2 cm/s, for the last 0.5 s), the boxed donut having been lifted pinched between both tong paddles; 0.5 a donut carried
    that way is in the box; 0.25 a donut was lifted clear of the tray (1 cm) pinched between both paddles; else 0."""
    if gated(objects) or not end_ok(objects):
        return 0.0
    b = boxed(objects)
    good = [i for i in b if i in _EP.pinched]  # boxed donuts that were carried pinched in the tongs
    if len(b) == 1 and good and all(on_spot(objects[d.name], i) for i, d in enumerate(DONUTS) if i != b[0]) \
            and door_shut(objects) and tongs_on_mat(objects) and parked(objects):
        return 1.0
    if good:
        return 0.5
    if _EP.lifted:
        return 0.25
    return 0.0


def success(objects) -> bool:
    return bool(score(objects) == 1.0)


def distance(objects) -> float:
    if gated(objects):
        return 1.0
    b = boxed(objects)
    a = door_angle(objects)
    return float(0.5 * (len(b) != 1) + 0.3 * (0 if not math.isfinite(a) else min(a, 90) / 90)
                 + 0.2 * (not tongs_on_mat(objects)))


def progress(objects) -> dict:
    _update(objects)
    return {"door_max_deg": _EP.max_door, "lifted": float(_EP.lifted), "boxed": float(len(boxed(objects))),
            "door_deg": door_angle(objects), "door_shut": float(door_shut(objects)),
            "tongs_on_mat": float(tongs_on_mat(objects)), "parked": float(parked(objects)),
            "touched": float(_EP.touched), "dropped": float(_EP.dropped), "let_go": float(_EP.let_go), "unpinched": float(_EP.unpinched)}


VARIATION: dict = {"obj:tongs": {"dx": 0.01, "dy": 0.01}}
