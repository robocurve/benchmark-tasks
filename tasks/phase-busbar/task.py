"""On the station table: a mounting plate stands upright across the far side of the table, facing the robot, with a
DIN rail on it. Four single-pole 16 A circuit breakers with screwless (spring) busbar terminals sit side by side on
the rail, 17.5 mm wide each. Their busbar terminals are on their undersides: a slot in each breaker's bottom face,
12 mm behind its shoulder face. A single-pole insulated fork-type phase busbar, cut to four modules (78 mm long with
its end caps), lies on its back on the table in front of the right arm, its four copper forks pointing away from the
robot.

Goal: the busbar connects all four breakers: each of its forks pushed fully up into its breaker's busbar terminal,
so that the busbar's top edge rests against the breakers' undersides, and the busbar is let go.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import os
import sys
from pathlib import Path

import numpy as np

from yam_arena.task_file import TABLE_TOP, Asset

SCENE = "station"
TOP = TABLE_TOP[SCENE]
HERE = Path(__file__).resolve().parent
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- geometry (m, world frame: x away from the arms, y left, z up) -------------------------------------------------
#
# Estimated from manufacturers' published data (17.5 mm module, DIN 43880 size-1 breaker, 10 mm2 fork busbar) and
# product photos; see README "Fidelity".

N_POLES = 4
MODULE = 0.0175  # breaker width and busbar pitch
ROW_Y = 0.0  # middle of the row
SLOT_X = 0.28  # the busbar slots' centre plane (where each fork's tines sit when home)
BREAKER_BOTTOM = TOP + 0.13  # breakers' undersides

# Breaker housing (DIN 43880 size 1, estimated): 85 mm tall, 17.2 mm wide (0.3 mm between neighbours), 58 mm from
# the rail to the shoulder face, with a 45 mm tall front window standing 10 mm proud of the shoulder.
BREAKER_H = 0.085
BREAKER_W = 0.0172
SHOULDER_X = SLOT_X - 0.012  # shoulder face, 12 mm in front of the slot
RAIL_FACE_X = SHOULDER_X + 0.058  # where the breakers sit on the rail
WINDOW = (0.010, 0.045)  # front window: depth proud of the shoulder, height
RAIL = (0.0075, 0.035)  # TS35 x 7.5 rail: depth, height
RAIL_LEN = 0.20
PLATE = (0.003, 0.24, 0.30)  # mounting plate: thickness, width, height; standing on the table
PLATE_CENTER = (RAIL_FACE_X + RAIL[0] + PLATE[0] / 2, ROW_Y, TOP + PLATE[2] / 2)
RAIL_Z = BREAKER_BOTTOM + BREAKER_H / 2

# Busbar terminal, per breaker: a slot in the underside, 10 mm wide (y), 14 mm deep, with 1.5 mm lead-ins. Its rear
# wall is the fixed contact. In front of it a push-in clamping spring: a leaf hinged at the slot's mouth, leaning up
# and back toward the wall at 20 deg, its tip resting 0.2 mm off the wall. A tine pushed up swings it forward and is
# pressed against the wall; pulled down, it is held, as push-in terminals hold a conductor until opened (here by the
# tine notch, TINE_NOTCH).
TINE_T = 0.0012  # tine thickness (x)
SLOT_W = 0.010
SLOT_DEPTH = 0.014
MOUTH_CHAMFER = 0.0015  # 45 deg lead-ins on the slot's side walls and on the contact wall's lower edge
WALL_X = SLOT_X + TINE_T / 2  # the fixed contact face
LEAF_GAP = 0.0002  # the leaf's tip rests this far in front of the wall: a tine swings it 1 mm forward
LEAF_ANGLE = 20.0  # deg from vertical, leaning toward the wall
LEAF_LEN, LEAF_T, LEAF_W = 0.010, 0.0008, 0.0092  # hinge to tip, thickness, width
LEAF_PIVOT_Z = BREAKER_BOTTOM + 0.001
LEAF_SWING = 25.0  # deg it can swing forward
LEAF_MASS = 0.01  # heavier than the real spring, so the solver holds its hinge stop under load
CLAMP_STIFFNESS = 0.06  # N m/rad about the hinge: ~0.8 N at the tip on a tine
CLAMP_PRELOAD = 2.0  # deg past the rest stop the spring would reach without it
CLAMP_DAMPING = 0.0005  # N m s/rad
CONTACT_FRICTION = 0.5  # tine on the clamp leaf and the contact wall (tinned copper on tinned copper alloy)
PLASTIC_FRICTION = 0.3  # busbar on the terminal's plastic (side walls, lead-ins)

# Busbar (single pole, 10 mm2, fork type, cut to four modules): insulated body 78 mm long (end caps included),
# 6 mm thick (x), 10 mm tall (z); one fork per module, two tines 2.4 mm wide with a 1.2 mm gap, 1.2 mm thick,
# standing 12 mm above the body. Installed frame: origin at the body's centre, forks up (+z), length along y,
# tines in the body's mid plane (x = 0).
BUSBAR_ASSET = "phase_busbar_4m"
BUSBAR_USD = HERE / "assets" / "busbar_4m.usd"  # built by tools/make_busbar_usd.py
END_CAP = 0.003  # each end cap reaches this far past the last module
BODY = (0.006, 4 * MODULE + 2 * END_CAP, 0.010)
TINE_W, TINE_GAP, TINE_LEN = 0.0024, 0.0012, 0.012
# Simulation only: the clamping spring's one-way (self-locking) grip is modelled as a 0.8 mm deep notch in each
# tine's front face, 0.6 to 2.6 mm below its tip, that the spring's tip drops into once the fork is home.
TINE_NOTCH = (0.0008, 0.0006, 0.0026)  # depth, top and bottom (below the tip)
TINE_ROOT = 0.002  # tines run this far down into the body
BUSBAR_MASS = 0.016
FORK_Y = tuple((i - (N_POLES - 1) / 2) * MODULE for i in range(N_POLES))  # in the installed frame
BODY_COLOR = (0.50, 0.53, 0.57)
COPPER = (0.78, 0.50, 0.30)

# Start: lying on its back (installed -x down), forks pointing away from the robot (+x), in front of the right arm.
# The USD holds the lying pose; LYING maps the installed frame to it (+90 deg about y: forks +z -> +x).
START_BODY = (0.20, -0.13, TOP + BODY[0] / 2)


def _rot_y(deg: float) -> np.ndarray:
    a = math.radians(deg)
    return np.array([[math.cos(a), 0.0, math.sin(a)], [0.0, 1.0, 0.0], [-math.sin(a), 0.0, math.cos(a)]])


LYING = _rot_y(90.0)
LYING_BBOX_CENTER_X = TINE_LEN / 2  # lying, the forks stick out 12 mm past the body, so the box centre is 6 mm out
START_XY = (START_BODY[0] + LYING_BBOX_CENTER_X, START_BODY[1])  # bounding-box centre, as Asset places it


def _register_busbar_asset() -> None:
    """Make the busbar USD known to Arena's asset registry, where yam_arena's Asset looks models up."""
    from isaaclab_arena.assets.registries import AssetRegistry

    class PhaseBusbar4M:
        name = BUSBAR_ASSET
        tags = ["object"]
        usd_path = str(BUSBAR_USD)

    if not AssetRegistry().is_registered(BUSBAR_ASSET, ensure_loaded=False):
        AssetRegistry().register(PhaseBusbar4M, BUSBAR_ASSET)


_register_busbar_asset()

OBJECTS = [Asset("busbar", asset=BUSBAR_ASSET, xy=START_XY, extent=BODY[1], mass=BUSBAR_MASS)]


# -- the panel: plate, rail, breakers and their sprung busbar terminals ---------------------------------------------
#
# A task file's objects are plain boxes, cylinders, spheres or library models, and the terminals need moving parts.
# So this file adds the panel to the scene itself: when yam_arena.scene is imported (after Isaac Sim starts), its
# task_room() is wrapped to append one more scene object, "panel", whose spawner authors the plate and everything
# on it. Nothing else in yam_arena changes.

GREY_PLATE = (0.80, 0.81, 0.80)
RAIL_GREY = (0.62, 0.64, 0.66)
HOUSING = (0.93, 0.93, 0.91)
LEVER = (0.20, 0.21, 0.23)
CONTACT_GREY = (0.70, 0.66, 0.55)


def breaker_y(i: int) -> float:
    return ROW_Y + FORK_Y[i]


def panel_boxes() -> list[tuple]:
    """Every static box of the panel except the plate: (name, size, world centre, colour, rotation), rotation None or
    (axis "x" | "y", degrees)."""
    boxes = [("rail", (RAIL[0], RAIL_LEN, RAIL[1]), (RAIL_FACE_X + RAIL[0] / 2, ROW_Y, RAIL_Z), RAIL_GREY, None)]
    front_x = SHOULDER_X
    leaf_front_limit = leaf_pivot(0)[0] - LEAF_T - 0.0025  # the pocket's front face
    z0, z1 = BREAKER_BOTTOM, BREAKER_BOTTOM + SLOT_DEPTH
    c, t = MOUTH_CHAMFER, 0.0008  # lead-in size, and the thickness of the thin boxes that make its faces
    hyp = c * math.sqrt(2.0)
    pocket_x = (WALL_X + leaf_front_limit) / 2
    pocket_len = WALL_X - leaf_front_limit
    side = (BREAKER_W - SLOT_W) / 2
    for i in range(N_POLES):
        y = breaker_y(i)
        for s, tag in ((1, "l"), (-1, "r")):
            face = y + s * SLOT_W / 2  # the slot's side face
            mid = np.array([face + s * c / 2, z0 + c / 2]) + np.array([s, 1.0]) / math.sqrt(2.0) * t / 2
            boxes += [
                (f"q{i}_side_{tag}", (pocket_len, side, SLOT_DEPTH - c),
                 (pocket_x, face + s * side / 2, (z0 + c + z1) / 2), HOUSING, None),
                (f"q{i}_side_{tag}_low", (pocket_len, side - c, c),
                 (pocket_x, face + s * (c + (side - c) / 2), z0 + c / 2), HOUSING, None),
                (f"q{i}_lead_{tag}", (pocket_len, t, hyp), (pocket_x, mid[0], mid[1]), HOUSING,
                 ("x", 45.0 if s > 0 else 135.0)),
            ]
        mid = np.array([WALL_X + c / 2, z0 + c / 2]) + np.array([1.0, 1.0]) / math.sqrt(2.0) * t / 2
        boxes += [
            (f"q{i}_wall", (RAIL_FACE_X - WALL_X, BREAKER_W, SLOT_DEPTH - c),
             ((WALL_X + RAIL_FACE_X) / 2, y, (z0 + c + z1) / 2), CONTACT_GREY, None),
            (f"q{i}_wall_low", (RAIL_FACE_X - WALL_X - c, BREAKER_W, c),
             ((WALL_X + c + RAIL_FACE_X) / 2, y, z0 + c / 2), HOUSING, None),
            (f"q{i}_lead_wall", (t, SLOT_W, hyp), (mid[0], y, mid[1]), CONTACT_GREY, ("y", -45.0)),
        ]
        boxes += [
            # above the terminal
            (f"q{i}_body", (RAIL_FACE_X - front_x, BREAKER_W, BREAKER_H - SLOT_DEPTH),
             ((front_x + RAIL_FACE_X) / 2, y, (z1 + BREAKER_BOTTOM + BREAKER_H) / 2), HOUSING, None),
            (f"q{i}_window", (WINDOW[0], BREAKER_W, WINDOW[1]),
             (front_x - WINDOW[0] / 2, y, RAIL_Z), HOUSING, None),
            (f"q{i}_lever", (0.012, 0.007, 0.010), (front_x - WINDOW[0] - 0.006, y, RAIL_Z + 0.008), LEVER, None),
            # the front block, in front of the clamp leaf's pocket
            (f"q{i}_front", (leaf_front_limit - front_x, BREAKER_W, SLOT_DEPTH),
             ((front_x + leaf_front_limit) / 2, y, (z0 + z1) / 2), HOUSING, None),
        ]
    return boxes


def leaf_pivot(i: int) -> tuple[float, float, float]:
    """World position of breaker i's clamp leaf hinge (the leaf body's origin)."""
    a = math.radians(LEAF_ANGLE)
    return (WALL_X - LEAF_GAP - LEAF_LEN * math.sin(a), breaker_y(i), LEAF_PIVOT_Z)


def leaf_box() -> tuple[tuple, tuple]:
    """(size, centre relative to the hinge) of the leaf plate, which is turned LEAF_ANGLE about y: its wall-side face
    runs from the hinge to the tip."""
    a = math.radians(LEAF_ANGLE)
    along = np.array([math.sin(a), 0.0, math.cos(a)])
    out = np.array([math.cos(a), 0.0, -math.sin(a)])  # toward the wall
    centre = along * LEAF_LEN / 2 - out * LEAF_T / 2
    return (LEAF_T, LEAF_W, LEAF_LEN), tuple(float(c) for c in centre)


def _spawn_panel(prim_path: str, cfg, *args, **kwargs):
    """The plate (cfg's cuboid), then the rail, the breakers and the clamp leaves on their sprung joints."""
    import isaaclab.sim as sim_utils
    from isaaclab.sim.spawners.shapes import spawn_cuboid
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics, UsdShade

    prim = spawn_cuboid.__wrapped__(prim_path, cfg, *args, **kwargs)
    stage = prim.GetStage()
    origin = np.array(PLATE_CENTER)
    looks: dict[tuple, str] = {}

    def look(color) -> str:
        if color not in looks:
            path = f"{prim_path}/looks/m{len(looks)}"
            surface = sim_utils.PreviewSurfaceCfg(diffuse_color=_linear(color), roughness=0.5)
            surface.func(path, surface)
            looks[color] = path
        return looks[color]

    def physics_material(path, friction):
        UsdShade.Material.Define(stage, path)
        mat = UsdPhysics.MaterialAPI.Apply(stage.GetPrimAtPath(path))
        mat.CreateStaticFrictionAttr(friction)
        mat.CreateDynamicFrictionAttr(friction)
        PhysxSchema.PhysxMaterialAPI.Apply(stage.GetPrimAtPath(path)).CreateFrictionCombineModeAttr("min")
        return path

    contact = physics_material(f"{prim_path}/looks/contact", CONTACT_FRICTION)
    plastic = physics_material(f"{prim_path}/looks/plastic", PLASTIC_FRICTION)

    def cube(path, size, centre, color, rot=None, physics_material=None):
        geom = UsdGeom.Cube.Define(stage, path)
        geom.CreateSizeAttr(1.0)
        geom.AddTranslateOp().Set(Gf.Vec3d(*map(float, centre)))
        if rot:
            axis, deg = rot
            a = math.radians(deg) / 2
            q = (math.sin(a), 0.0, 0.0) if axis == "x" else (0.0, math.sin(a), 0.0)
            geom.AddOrientOp().Set(Gf.Quatf(math.cos(a), *q))
        geom.AddScaleOp().Set(Gf.Vec3f(*map(float, size)))
        UsdPhysics.CollisionAPI.Apply(geom.GetPrim())
        sim_utils.bind_visual_material(path, look(color), stage=stage)
        if physics_material:
            UsdShade.MaterialBindingAPI.Apply(geom.GetPrim()).Bind(
                UsdShade.Material(stage.GetPrimAtPath(physics_material)),
                UsdShade.Tokens.weakerThanDescendants,
                "physics",
            )
        return geom

    for name, size, centre, color, rot in panel_boxes():
        kind = name.split("_")[1] if name.startswith("q") else ""
        material = contact if kind == "wall" or name.endswith("lead_wall") else plastic if kind in ("side", "lead", "front") else None
        cube(f"{prim_path}/parts/{name}", size, np.array(centre) - origin, color, rot, physics_material=material)

    # Joint anchor: a small static collider inside the plate, unscaled, so the joints' frames are plain offsets.
    anchor = UsdGeom.Cube.Define(stage, f"{prim_path}/parts/anchor")
    anchor.CreateSizeAttr(0.002)
    UsdPhysics.CollisionAPI.Apply(anchor.GetPrim())
    UsdGeom.Imageable(anchor).MakeInvisible()

    size, centre = leaf_box()
    per_deg = math.pi / 180.0  # USD angular drives take N m per degree
    for i in range(N_POLES):
        pivot = np.array(leaf_pivot(i)) - origin
        leaf_path = f"{prim_path}/leaves/q{i}"
        leaf = UsdGeom.Xform.Define(stage, leaf_path)
        leaf.AddTranslateOp().Set(Gf.Vec3d(*map(float, pivot)))
        UsdPhysics.RigidBodyAPI.Apply(leaf.GetPrim())
        UsdPhysics.MassAPI.Apply(leaf.GetPrim()).CreateMassAttr(LEAF_MASS)
        body = PhysxSchema.PhysxRigidBodyAPI.Apply(leaf.GetPrim())
        body.CreateSolverPositionIterationCountAttr(32)
        body.CreateSolverVelocityIterationCountAttr(4)
        cube(f"{leaf_path}/plate", size, centre, CONTACT_GREY, ("y", LEAF_ANGLE), physics_material=contact)
        joint = UsdPhysics.RevoluteJoint.Define(stage, f"{prim_path}/joints/q{i}")
        joint.CreateBody0Rel().SetTargets([anchor.GetPath()])
        joint.CreateBody1Rel().SetTargets([leaf.GetPath()])
        joint.CreateLocalPos0Attr(Gf.Vec3f(*map(float, pivot)))
        joint.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
        joint.CreateAxisAttr("Y")  # positive turns the tip toward the wall
        joint.CreateLowerLimitAttr(-LEAF_SWING)  # swung forward by a tine
        joint.CreateUpperLimitAttr(0.0)  # the rest stop
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(CLAMP_STIFFNESS * per_deg)
        drive.CreateDampingAttr(CLAMP_DAMPING * per_deg)
        drive.CreateTargetPositionAttr(CLAMP_PRELOAD)
        # The leaf touches only tines: it would otherwise rub on its own pocket.
        filtered = UsdPhysics.FilteredPairsAPI.Apply(leaf.GetPrim())
        filtered.CreateFilteredPairsRel().SetTargets([f"{prim_path}/parts", f"{prim_path}/geometry"])
    return prim


def _linear(srgb):
    return tuple(c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4 for c in srgb)


def _panel_object():
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import clone
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose
    from isaaclab_physx.sim.schemas import PhysxCollisionPropertiesCfg

    return Object(
        name="panel",
        object_type=ObjectType.BASE,
        spawner_cfg=sim_utils.CuboidCfg(
            func=clone(_spawn_panel),
            size=PLATE,
            collision_props=PhysxCollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=_linear(GREY_PLATE), roughness=0.6),
        ),
        initial_pose=Pose(position_xyz=PLATE_CENTER),
    )


def _patch_scene(scene) -> None:
    if getattr(scene, "_phase_busbar_panel", False):
        return
    room = scene.task_room

    def task_room(*args, **kwargs):
        return [*room(*args, **kwargs), _panel_object()]

    scene.task_room = task_room
    scene._phase_busbar_panel = True
    _wrap_object_states(scene)


class _SceneHook(importlib.abc.MetaPathFinder):
    """Patches yam_arena.scene as soon as it is imported (it imports Isaac Sim, so not before the app starts)."""

    def find_spec(self, name, path, target=None):
        if name != "yam_arena.scene":
            return None
        sys.meta_path.remove(self)
        spec = importlib.util.find_spec(name)
        run = spec.loader.exec_module

        def exec_module(module):
            run(module)
            _patch_scene(module)

        spec.loader.exec_module = exec_module
        return spec


if "yam_arena.scene" in sys.modules:
    _patch_scene(sys.modules["yam_arena.scene"])
elif not any(isinstance(f, _SceneHook) for f in sys.meta_path):
    sys.meta_path.insert(0, _SceneHook())


# -- scoring --------------------------------------------------------------------------------------------------------
#
# Besides the busbar's bounding box, scoring reads the busbar's pose, both grippers' tool centre points, and runs the
# tug test. yam_arena hands a task file only its objects' states, so object_states() is wrapped (on import, above) to
# add them under "busbar_pose", "left_gripper" and "right_gripper".

HOME_GAP = 0.001  # a fork is home: the body's top edge within 1 mm of the breakers' underside at that fork...
IN_SLOT = (0.003, 0.004)  # ... with the fork within 3 mm (x) and 4 mm (y) of its slot's centre line (one module off
#                            is 17.5 mm)
RELEASED = 0.05  # m: neither gripper's centre point within 5 cm of the busbar
TUG_N, TUG_RAMP_S, TUG_S = 5.0, 1.0, 2.0  # the tug: pulled straight down, raised to 5 N over 1 s and held to 2 s...
TUG_MAX_MOVE = 0.001  # ...it passes if the busbar moves less than 1 mm and its forks are still home
RELEASE_WAIT_S = 3.0  # the tug starts once the forks are home and both grippers have stayed clear this long
POINTS_PER_FORK = 0.2
GRIPPERS = ("left_gripper", "right_gripper")

_state = {"released_steps": 0, "tug": None, "tug_passed": False, "tug_failed": False, "steps": 0}
_TRACE = bool(os.environ.get("BUSBAR_TRACE"))  # BUSBAR_TRACE=1 prints the scoring state every 10 steps


class GripperState:
    def __init__(self, tcp, gap):
        self.tcp = np.asarray(tcp, dtype=float)
        self.gap = float(gap)


class Pose:
    def __init__(self, pos, rot):
        self.pos = np.asarray(pos, dtype=float)
        self.rot = np.asarray(rot, dtype=float)  # installed frame -> world


def _quat_matrix(q_xyzw) -> np.ndarray:
    x, y, z, w = q_xyzw
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def forks(pose: Pose) -> list[np.ndarray]:
    """World position of each fork's root on the body's top edge (installed frame (0, y_i, +BODY_z/2))."""
    return [pose.pos + pose.rot @ np.array([0.0, y, BODY[2] / 2]) for y in FORK_Y]


def home(pose: Pose) -> list[bool]:
    out = []
    for i, root in enumerate(forks(pose)):
        up = pose.rot[:, 2]
        out.append(
            bool(
                up[2] > math.cos(math.radians(10))
                and abs(root[2] - BREAKER_BOTTOM) <= HOME_GAP
                and abs(root[0] - SLOT_X) <= IN_SLOT[0]
                and abs(root[1] - breaker_y(i)) <= IN_SLOT[1]
            )
        )
    return out


def _released(objects) -> bool:
    p = objects["busbar_pose"].pos
    grippers = [objects.get(g) for g in GRIPPERS]
    return all(g is not None and np.linalg.norm(g.tcp - p) >= RELEASED for g in grippers)


def _wrap_object_states(scene) -> None:
    """Each step's states also get the busbar's pose and both grippers; the tug test runs here, since it needs the
    simulator: once all four forks are home and the grippers have stayed clear for RELEASE_WAIT_S, a downward force
    on the busbar is raised to TUG_N and held to TUG_S, then removed. The test runs once per episode."""
    from yam_arena.constants import FINGER_JOINTS, TCP_OFFSET

    original = scene.object_states

    def object_states(env, names):
        import torch

        out = original(env, names)
        base = env.unwrapped
        origin = base.scene.env_origins[0].detach().cpu().numpy()
        bar = base.scene["busbar"]
        pos = bar.data.root_pos_w.torch[0].detach().cpu().numpy() - origin
        quat = bar.data.root_quat_w.torch[0].detach().cpu().numpy()
        pose = Pose(pos, _quat_matrix(quat) @ LYING)
        out["busbar_pose"] = pose
        for key, robot in base.scene.articulations.items():
            if "gripper" not in robot.body_names:
                continue
            body = robot.body_names.index("gripper")
            gpos = robot.data.body_pos_w.torch[0, body].detach().cpu().numpy() - origin
            gquat = robot.data.body_quat_w.torch[0, body].detach().cpu().numpy()
            fingers = [robot.joint_names.index(j) for j in FINGER_JOINTS]
            q = robot.data.joint_pos.torch[0, fingers].detach().cpu().numpy()
            side = "left" if float(robot.data.root_pos_w.torch[0, 1]) > float(origin[1]) else "right"
            out[f"{side}_gripper"] = GripperState(gpos + _quat_matrix(gquat) @ np.array(TCP_OFFSET), float(np.abs(q).sum()))

        _reset_if_fresh(out)
        if _TRACE:
            _state["leaves"] = _leaf_offsets(origin)
        all_home = all(home(pose))
        _state["released_steps"] = _state["released_steps"] + 1 if all_home and _released(out) else 0
        dt = base.step_dt
        tug = _state["tug"]
        if tug is None and not (_state["tug_passed"] or _state["tug_failed"]):
            if _state["released_steps"] * dt >= RELEASE_WAIT_S:
                _state["tug"] = {"t": 0.0, "start": pose.pos.copy(), "moved": 0.0}
                _set_tug(bar, 0.0, torch)
        elif tug is not None:
            tug["t"] += dt
            moved = float(np.linalg.norm(pose.pos - tug["start"]))
            if _TRACE and moved > TUG_MAX_MOVE and tug["moved"] <= TUG_MAX_MOVE:
                print(f"TUG slipped at {_tug_force(tug['t']):.2f} N", flush=True)
            tug["moved"] = max(tug["moved"], moved)
            _set_tug(bar, _tug_force(tug["t"]), torch)
            if tug["t"] >= TUG_S:
                _set_tug(bar, 0.0, torch)
                _state["tug"] = None
                passed = tug["moved"] < TUG_MAX_MOVE and all_home
                _state["tug_passed"], _state["tug_failed"] = passed, not passed
                if _TRACE:
                    print(f"TUG moved {tug['moved'] * 1000:.2f} mm -> {'pass' if passed else 'fail'}", flush=True)
        return out

    object_states._phase_busbar = True
    scene.object_states = object_states


def _leaf_offsets(origin) -> list:
    """Debug (BUSBAR_TRACE): each clamp leaf's swing (deg, negative = forward), read from PhysX."""
    from isaaclab_physx.physics import PhysxManager

    view = _state.get("leaf_view")
    if view is None:
        view = PhysxManager.get_physics_sim_view().create_rigid_body_view("/World/envs/env_*/panel/leaves/*")
        _state["leaf_view"] = view
    poses = view.get_transforms()
    poses = poses.numpy() if hasattr(poses, "numpy") else np.asarray(poses)
    poses = np.asarray(poses).reshape(-1, 7)
    return [round(math.degrees(2 * math.atan2(p[4], p[6])), 1) for p in poses]  # swing about y (deg)


def _tug_force(t: float) -> float:
    """The pull at time t into the tug: raised evenly to TUG_N over TUG_RAMP_S, then held."""
    return TUG_N * min(1.0, t / TUG_RAMP_S)


def _set_tug(bar, newtons: float, torch) -> None:
    device = bar.data.root_pos_w.torch.device
    force = torch.tensor([[[0.0, 0.0, -newtons]]], dtype=torch.float32, device=device)
    bar.permanent_wrench_composer.set_forces_and_torques_index(
        forces=force, torques=torch.zeros_like(force), body_ids=[0], env_ids=torch.tensor([0], device=device),
        is_global=True,
    )


def _reset_if_fresh(objects) -> None:
    """A new episode: the busbar lies untouched where it starts."""
    p = objects["busbar_pose"].pos
    if np.linalg.norm(p[:2] - np.array(START_BODY[:2])) < 0.002 and abs(p[2] - START_BODY[2]) < 0.002:
        _state.update(released_steps=0, tug=None, tug_passed=False, tug_failed=False)


def success(objects):
    """All four forks home, the busbar let go, and it held the tug."""
    pose = objects["busbar_pose"]
    seated = home(pose)
    ok = bool(all(seated) and _released(objects) and _state["tug_passed"])
    if _TRACE:
        _state["steps"] += 1
        if _state["steps"] % (1 if os.environ.get("BUSBAR_TRACE") == "2" else 10) == 0:  # BUSBAR_TRACE=2: every step
            g = {s: objects.get(f"{s}_gripper") for s in ("left", "right")}
            roots = forks(pose)
            print(
                f"TRACE {_state['steps']} body={np.round(pose.pos, 4)} up={np.round(pose.rot[:, 2], 3)} "
                f"gaps_mm={[round((r[2] - BREAKER_BOTTOM) * 1000, 1) for r in roots]} "
                f"dx_mm={[round((r[0] - SLOT_X) * 1000, 1) for r in roots]} "
                f"dy_mm={[round((r[1] - breaker_y(i)) * 1000, 1) for i, r in enumerate(roots)]} home={seated} "
                f"leaves={_state.get('leaves')} released={_released(objects)} tug={_state['tug_passed']}/{_state['tug_failed']} ok={ok} | "
                + " ".join(f"{s}:{np.round(x.tcp, 4)} gap={x.gap * 1000:.1f}" for s, x in g.items() if x),
                flush=True,
            )
    return ok


def score(objects):
    """Partial credit (success scores 1.0): 0.2 per fork home, counted once the busbar is let go."""
    if not _released(objects):
        return 0.0
    n = sum(home(objects["busbar_pose"]))
    if n == N_POLES and _state["tug_passed"]:
        return 1.0
    return POINTS_PER_FORK * n


# -- scripted demo --------------------------------------------------------------------------------------------------


class Orientation(float):
    """A waypoint's ``yaw`` that carries a full TCP rotation matrix (yam_arena's waypoints only turn a downward grip
    about the vertical). Its float value is unused."""

    def __new__(cls, matrix):
        obj = super().__new__(cls, 0.0)
        obj.matrix = np.asarray(matrix, dtype=float)
        return obj


def _enable_full_orientations(demo_module) -> None:
    """Let yam_arena's ArmPlan follow an :class:`Orientation`. Only the scripted demo calls this."""
    original = demo_module.grip_rotation
    if getattr(original, "_full_orientations", False):
        return

    def grip_rotation(yaw: float = 0.0):
        return yaw.matrix if isinstance(yaw, Orientation) else original(yaw)

    grip_rotation._full_orientations = True
    demo_module.grip_rotation = grip_rotation


GRIP_DEPTH = 0.012  # m: the body's centre sits this far past the TCP along the approach axis...
GRIP_HIGH = 0.006  # m: ...and this far above it once turned: gripped off-centre, so the fingers stay clear longer
GRIP_ROLL = 0.015  # rad: measured; picked off the table, the busbar sits this far rolled in the grip (its -y end
#                    high), so the hand is rolled back by as much and all four forks meet their terminals together
SQUEEZE_GAP = BODY[1] - 0.010  # m: finger target 10 mm inside the busbar's length: a limited squeeze (~25 N per finger)
START_DEPTH = -0.016  # m: held by its end caps, the busbar is commanded 16 mm *past* home: the wrist's position
#                        gain (10 N m/rad, as on the real arm) turns that error into push force
TIP_REACH = 0.009  # m: closed, the fingers' last 8 mm (5..13 mm past the TCP) push with their top edge...
TIP_TOP = 0.0077  # m: ...which is 7.7 mm above the TCP
PUSH_PAST = 0.035  # m: commanded past home: the compliant wrist turns the error into push force


def demo():
    """Right arm: grips the busbar by its two end caps (gripper pointing down), lifts it and turns it forks-up with
    the gripper pointing forward, and pushes its forks up into the four terminals, until the clamps hold it (about
    1 mm short of home, the fingertips would meet the outer breakers). Then it lets go, closes its fingers, and
    presses the busbar fully home from below with the top edge of its fingertips, at the busbar's middle, as an
    electrician presses a busbar home with a thumb. Finally it lowers and backs out."""
    import yam_arena.demo as yd
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
    from yam_arena.demo import Waypoint
    from yam_arena.urdf import axis_angle_matrix

    _enable_full_orientations(yd)
    down = yd.grip_rotation(0.0)  # pointing down, fingers closing along y
    g_down = Orientation(down)
    g_fwd = Orientation(axis_angle_matrix(np.array([0.0, 1.0, 0.0]), math.radians(-90)) @ down)  # pointing +x
    g_level = Orientation(axis_angle_matrix(np.array([1.0, 0.0, 0.0]), GRIP_ROLL) @ g_fwd.matrix)

    squeeze = -SQUEEZE_GAP / 2  # finger joint target
    bx, by, bz = START_BODY
    grasp = (bx - GRIP_HIGH, by, bz + GRIP_DEPTH)  # lying, the installed "up" is +x
    held_home = np.array([SLOT_X - GRIP_DEPTH, ROW_Y, BREAKER_BOTTOM - BODY[2] / 2 - GRIP_HIGH])  # TCP, busbar home
    below = held_home - np.array([0.0, 0.0, 0.03])
    started = held_home - np.array([0.0, 0.0, START_DEPTH])
    # Pressing from below: the fingertips' top edge under the body's bottom face, at the busbar's middle.
    press_home = np.array([SLOT_X - TIP_REACH, ROW_Y, BREAKER_BOTTOM - BODY[2] - TIP_TOP])
    press_below = press_home - np.array([0.0, 0.0, 0.015])
    right = [
        Waypoint(2.0, (bx, by, TOP + 0.10), FINGER_OPEN, "above the busbar", g_down),
        Waypoint(1.5, grasp, FINGER_OPEN, "descend", g_down),
        Waypoint(0.8, grasp, squeeze, "grip the end caps", g_down),
        Waypoint(1.2, (bx, by, TOP + 0.08), squeeze, "lift", g_down),
        Waypoint(3.0, tuple(below - np.array([0.03, 0.0, 0.0])), squeeze, "turn forks up, under the row", g_level),
        Waypoint(1.5, tuple(below), squeeze, "line up under the slots", g_level),
        Waypoint(2.0, tuple(started), squeeze, "start the forks into the terminals", g_level),
        Waypoint(0.8, tuple(started), FINGER_OPEN, "let go", g_level),
        Waypoint(1.2, tuple(started - np.array([0.04, 0.0, 0.0])), FINGER_OPEN, "back out", g_fwd),
        Waypoint(1.0, tuple(press_below - np.array([0.04, 0.0, 0.0])), FINGER_CLOSED, "close, go below", g_fwd),
        Waypoint(1.2, tuple(press_below), FINGER_CLOSED, "under the middle of the busbar", g_fwd),
        Waypoint(2.0, tuple(press_home + np.array([0.0, 0.0, PUSH_PAST])), FINGER_CLOSED, "press it home", g_fwd),
        Waypoint(1.0, tuple(press_home + np.array([0.0, 0.0, PUSH_PAST])), FINGER_CLOSED, "hold", g_fwd),
        Waypoint(1.0, tuple(press_below - np.array([0.0, 0.0, 0.02])), FINGER_CLOSED, "lower", g_fwd),
        Waypoint(1.5, tuple(press_below + np.array([-0.07, 0.04, -0.03])), FINGER_CLOSED, "clear", g_fwd),
    ]
    return {"right": right}
