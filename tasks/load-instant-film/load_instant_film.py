"""On the station table, 38 cm in front of the arms' bases and centred between them, an instant camera (an instax mini
12: 10.4 cm wide, 6.7 cm deep, 12.2 cm tall, mint green) stands upright on its base. Its back, with the hinged back
cover, faces the right arm; its front, with the lens and the shutter button, faces the left arm.

- On the back, at the top towards the arms, is the back cover lock: a small grey tab that slides down. Pressing it
  down unlocks the back cover, which springs a little ajar; the cover is hinged along its far edge (away from the arms)
  and swings open towards the right arm. Under it is the empty film chamber, with a yellow mark at its top far corner.
- On the front: the lens in its ring (the ring turns towards the arms, clockwise as seen from the front, to switch the
  camera ON), and the round shutter button above and nearer the arms than the lens. The film ejection slot is on the
  camera's top, at the front.

On the table in front of the right arm lies a new instax mini film pack (9.2 x 6.1 x 2.0 cm, black), its back up: the
back has two rectangular holes, which must never be pressed, and a yellow mark at one corner.

Load the film and make the camera ready to shoot: open the back cover, put the pack in with its yellow mark at the
camera's yellow mark (the pack's back with the holes facing out), close the back cover until it locks, turn the lens
ring to ON, and press the shutter button once, which ejects the black film cover from the slot on top.
"""

import math
import os
import pathlib
import sys
from dataclasses import dataclass, replace

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
import mechanism as mech  # noqa: E402
from mechanism import Joint, Link, Shape, mechanism  # noqa: E402
from yam_arena import demo as yam_demo  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402
from yam_arena.urdf import axis_angle_matrix  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the camera ---------------------------------------------------------------------------------------------------------
# instax mini 12: 104 x 66.6 x 122 mm (W x D x H), 306 g without batteries, strap and film (Fujifilm's specifications
# page). The chamber, cover, lock, lens ring, shutter button and slot sizes and places are estimates from the owner's
# manual's drawings, until a camera is measured. Heights are above the table; x, y are world (x forward, y left).
# The camera's back faces -y (the right arm), its front +y (the left arm); "far" is +x, away from the arms.
CX = 0.38  # the camera's middle (x); centred on y 0
W, D, H = 0.104, 0.0666, 0.122
X0, X1 = CX - W / 2, CX + W / 2
YB, YF = -D / 2, D / 2  # back and front faces
# The film chamber: a recess in the back, 3 mm wider than the pack on each side [estimate], 0.5 mm deeper than it.
# WHY 3 mm: with 1 mm, a pack held 2.6 degrees off upright stopped at the mouth (official sim, 2026-11-22).
PACK = (0.092, 0.061, 0.020)  # the film pack: length, width, thickness (m)
CH_W, CH_H, CH_D = PACK[1] + 0.006, PACK[0] + 0.006, PACK[2] + 0.0005
CH_X, CH_Z = CX, 0.060  # the chamber's middle
CH_WALL = YB + CH_D  # the chamber's floor (the face the pack's front rests on), y
CH_Z0, CH_Z1 = CH_Z - CH_H / 2, CH_Z + CH_H / 2
# The back cover: a 3 mm plate over the back, hinged along its far edge (x X1), opening about the vertical towards -y.
COVER_T = 0.003
COVER_Z0, COVER_Z1 = 0.006, 0.108
HINGE = (X1, YB - COVER_T / 2, (COVER_Z0 + COVER_Z1) / 2)
COVER_KG = 0.030
COVER_OPEN_MAX = math.radians(95.0)  # its stop [estimate]; the guide warns against opening it further
POP_DEG = 20.0  # unlocked, the cover springs this far ajar [estimate]
COVER_K = 20.0  # N m/rad: the lock's hold (target 0); unlocked, the target follows the cover
FORCED_DEG = 5.0  # opened this far while locked: the lock forced
LATCH_DEG = 2.0  # pushed shut within this, with the lock tab back up, it locks
# The back cover lock: a tab on the back, at the top towards the arms, sprung up; pressed 2 mm down, it unlocks.
LOCK_X = (CX - 0.040, CX - 0.028)
LOCK_Y = (YB - 0.012, YB)
LOCK_Z = (0.112, 0.119)
LOCK_TRAVEL = 0.003
UNLOCKED = 0.002
# The lens ring, on the front, turns clockwise as seen from the front (negative about +y) to ON.
LENS_X, LENS_Z = CX + 0.006, 0.048
RING_R, RING_Y = 0.031, (YF + 0.004, YF + 0.026)
ON_DEG = 30.0  # turned this far, the camera is on
RING_MAX = math.radians(80.0)
# The shutter button, on the front, above the lens and nearer the arms.
SHUTTER_XZ = (CX - 0.034, 0.092)
SHUTTER_R, SHUTTER_PROUD, SHUTTER_TRAVEL = 0.009, 0.004, 0.0025
PRESSED = 0.0015
# The black film cover, inside the camera until it is ejected up out of the slot on top.
COVER_SHEET = (0.054, 0.0012, 0.086)
SHEET_Y = YF - 0.012
EJECT = 0.045  # how far it comes out of the slot

# -- the stand: the camera leans forward on it, so its back (and the film chamber's mouth) faces partly up -------------
# WHY: standing upright, a pack let go part-way into the chamber slid straight back out every time (official sim,
# 2026-11-23, six variants); tilted, its weight holds it in, as when a person holds the camera back up to load it.
# Everything above is in the camera's own "upright frame": world axes and heights as if it stood on its base on the
# table. The stand tilts that frame by TILT_DEG about the x axis (top towards +y, the lens towards the table) and lifts
# it so the lens ring's lowest point is RING_CLEAR above the table.
TILT_DEG = float(os.environ.get("FILM_TILT", "30"))
RING_CLEAR = 0.025
ALPHA = -math.radians(TILT_DEG)  # about +x: the back's normal (-y) turns towards +z
R_TILT = axis_angle_matrix(np.eye(3)[0], ALPHA)


def _origin() -> np.ndarray:
    """Where the upright frame's origin goes (x, y, and height above the table): the camera's middle at y 0, the lens
    ring's lowest point RING_CLEAR up."""
    middle = R_TILT @ np.array([CX, 0.0, H / 2])
    lowest = min((R_TILT @ np.array([LENS_X, y, LENS_Z + RING_R * math.cos(t)]))[2]
                 for y in RING_Y for t in np.linspace(0, 2 * math.pi, 73))
    lowest = min(lowest, (R_TILT @ np.array([0.0, YF, 0.0]))[2])
    return np.array([0.0, -middle[1], RING_CLEAR - lowest])


ORIGIN = _origin()
UP = np.array([0.0, 0.0, 1.0])


def to_world(u) -> np.ndarray:
    """A point in the upright frame (heights include TOP) to the world."""
    return ORIGIN + np.array([0.0, 0.0, TOP]) + R_TILT @ (np.asarray(u, float) - TOP * UP)


def to_upright(w) -> np.ndarray:
    """A world point to the camera's upright frame (heights include TOP)."""
    return R_TILT.T @ (np.asarray(w, float) - ORIGIN - TOP * UP) + TOP * UP

MINT, DARK, GREY, BLACK = (0.55, 0.85, 0.75), (0.10, 0.10, 0.11), (0.45, 0.47, 0.50), (0.04, 0.04, 0.04)
YELLOW, SILVER = (1.0, 0.82, 0.0), (0.75, 0.76, 0.78)
MATERIALS = {"plastic": (0.6, 0.5)}


def _box(x0, x1, y0, y1, z0, z1, color, material="plastic") -> Shape:
    return Shape("box", ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), (x1 - x0, y1 - y0, z1 - z0), color,
                 material=material)


def _cyl_y(x, z, y0, y1, r, color) -> Shape:
    return Shape("cylinder", (x, (y0 + y1) / 2, z), (r, y1 - y0), color, axis="Y", material="plastic")


def _camera():
    cx0, cx1 = CH_X - CH_W / 2, CH_X + CH_W / 2
    base = Link("body", 0.31, (
        _box(X0, X1, CH_WALL, YF, 0.0, H, MINT),  # the front of the body, behind the chamber
        _box(X0, cx0, YB, CH_WALL, 0.0, H, MINT),  # the rear frame round the chamber: near side
        _box(cx1, X1, YB, CH_WALL, 0.0, H, MINT),  # far side
        _box(cx0, cx1, YB, CH_WALL, 0.0, CH_Z0, MINT),  # below
        _box(cx0, cx1, YB, CH_WALL, CH_Z1, H, MINT),  # above
        _box(cx0, cx1, CH_WALL - 0.0004, CH_WALL, CH_Z0, CH_Z1, DARK),  # the chamber floor, dark
        _box(cx1 - 0.008, cx1, CH_WALL - 0.0008, CH_WALL - 0.0004, CH_Z1 - 0.008, CH_Z1, YELLOW),  # yellow mark
        _cyl_y(LENS_X, LENS_Z, YF, YF + 0.004, 0.027, SILVER),  # the lens barrel's base
        _box(CX - 0.030, CX + 0.030, YF - 0.016, YF - 0.008, H, H + 0.0004, BLACK),  # the ejection slot, drawn
        _box(CX + 0.014, CX + 0.040, YF, YF + 0.002, 0.096, 0.114, DARK),  # viewfinder window
        _box(CX - 0.044, CX - 0.022, YF, YF + 0.002, 0.107, 0.117, SILVER),  # flash
    ))
    cover = Link("back_cover", COVER_KG, (_box(X0, X1, YB - COVER_T, YB, COVER_Z0, COVER_Z1, MINT),
                                         _box(X0 + 0.004, X0 + 0.012, YB - COVER_T - 0.0005, YB - COVER_T,
                                              0.030, 0.084, GREY)))  # a grip ridge at its free edge
    lock = Link("lock", 0.003, (_box(*LOCK_X, *LOCK_Y, *LOCK_Z, GREY),))
    ring = Link("lens_ring", 0.020, (_cyl_y(LENS_X, LENS_Z, *RING_Y, RING_R, SILVER),
                                     _cyl_y(LENS_X, LENS_Z, RING_Y[1], RING_Y[1] + 0.002, 0.018, BLACK),
                                     _box(LENS_X - 0.002, LENS_X + 0.002, RING_Y[0], RING_Y[1],
                                          LENS_Z + RING_R - 0.001, LENS_Z + RING_R + 0.0005, BLACK)))  # index mark
    sx, sz = SHUTTER_XZ
    shutter = Link("shutter", 0.003, (_cyl_y(sx, sz, YF, YF + SHUTTER_PROUD, SHUTTER_R, SILVER),))
    sheet = Link("film_cover", 0.002, (_box(CX - COVER_SHEET[0] / 2, CX + COVER_SHEET[0] / 2,
                                            SHEET_Y - COVER_SHEET[1] / 2, SHEET_Y + COVER_SHEET[1] / 2,
                                            H - 0.001 - COVER_SHEET[2], H - 0.001, BLACK),))
    joints = [
        Joint("back_cover", "revolute", "back_cover", HINGE, "Z", 0.0, COVER_OPEN_MAX, stiffness=COVER_K, damping=0.05,
              target=0.0),
        Joint("lock", "prismatic", "lock", (sum(LOCK_X) / 2, sum(LOCK_Y) / 2, LOCK_Z[1]), "Z", -LOCK_TRAVEL, 0.0,
              stiffness=300.0, damping=1.0, target=0.0),
        Joint("lens_ring", "revolute", "lens_ring", (LENS_X, sum(RING_Y) / 2, LENS_Z), "Y", -RING_MAX, 0.0,
              damping=0.02, friction=0.05),
        Joint("shutter", "prismatic", "shutter", (sx, YF + SHUTTER_PROUD, sz), "Y", -SHUTTER_TRAVEL, 0.0,
              stiffness=300.0, damping=2.0, target=0.0),
        Joint("film_cover", "prismatic", "film_cover", (CX, SHEET_Y, H), "Z", 0.0, EJECT + 0.005, stiffness=60.0,
              damping=3.0, target=0.0),
    ]
    # The stand: a grey block under the camera's base, as tilted as the camera; its lower part is below the table.
    stand = _box(X0, X1, YB, YF, -0.08, 0.0, GREY)
    base = replace(base, shapes=(*base.shapes, stand))
    # The mechanism's frame is turned a quarter turn about z from the upright frame (its x is the upright y), so that
    # its pitch (about the world y axis, before a yaw of +90 degrees) tilts the camera about the world x axis:
    # Rz(90) Ry(p) Rz(-90) = Rx(-p).
    links = tuple(_local_link(link) for link in (base, cover, lock, ring, shutter, sheet))
    joints = [replace(j, anchor=_local_point(j.anchor), axis=_LOCAL_AXIS[j.axis]) for j in joints]
    return mechanism("camera", tuple((ORIGIN + TOP * UP).tolist()), links[0], links[1:], joints,
                     {**MATERIALS, "default": (0.6, 0.5)}, yaw=math.pi / 2, pitch=-ALPHA)


# Upright frame to the mechanism's frame: (x, y, z) -> (y, -x, z). The upright y axis becomes the mechanism's +x, so
# a joint along or about it keeps its sign; no joint is along the upright x.
_LOCAL_AXIS = {"Y": "X", "Z": "Z"}


def _local_point(p) -> tuple:
    x, y, z = p
    return (float(y), float(-x), float(z))


def _local_link(link: Link) -> Link:
    shapes = []
    for s in link.shapes:
        size = (s.size[1], s.size[0], s.size[2]) if s.kind == "box" else s.size
        shapes.append(replace(s, center=_local_point(s.center), size=size, axis={"X": "Y", "Y": "X", "Z": "Z"}[s.axis]))
    return replace(link, shapes=tuple(shapes))


# -- the film pack ------------------------------------------------------------------------------------------------------
# instax mini film pack: 92 x 61 x 20 mm [the size Wikipedia gives; to be measured]; about 45 g [estimate]. Its frame:
# x is its length (the yellow-marked end at +x), y its width, z its thickness, the back (with the two rectangular holes)
# on +z. The yellow mark is at the corner (+x, -y) of the back. Loaded correctly, its +x end is up and its back faces
# out of the chamber (towards -y), which puts the yellow mark at the chamber's top far corner, by the camera's.
PACK_KG = 0.045
HOLES = (((0.0, 0.016), (0.020, 0.010)), ((0.0, -0.016), (0.020, 0.010)))  # (centre x, y), (size x, y) on the back
YELLOW_AT = (PACK[0] / 2 - 0.006, -PACK[1] / 2 + 0.006)
PACK_START = (0.24, -0.18, TOP + PACK[2] / 2 + 0.0005)


def _carry_turn() -> np.ndarray:
    """How the right hand's carry turns the pack: from its pick (pointing down, fingers closing along x) to the
    chamber (pointing +y, fingers still closing along x): a quarter turn about the line between the fingers."""
    pick = yam_demo.grip_rotation(math.pi / 2)
    return axis_angle_matrix(np.eye(3)[0], math.pi / 2) @ pick @ pick.T


def _start_yaw() -> float:
    """The pack lies back up, its length pointing away from the right arm (along y), turned so that the carry brings
    its yellow end up; FILM_PACK_TURNED=1 turns it half round (a probe: the same moves then load it yellow end down)."""
    turn = _carry_turn()
    for yaw in (math.pi / 2, -math.pi / 2):
        r = turn @ axis_angle_matrix(np.eye(3)[2], yaw)
        if r[2, 0] > 0.99 and -r[1, 2] > 0.99:
            return yaw + (math.pi if os.environ.get("FILM_PACK_TURNED") == "1" else 0.0)
    raise RuntimeError("no start yaw brings the pack's yellow end up")


@dataclass(frozen=True)
class FilmPack(Box):
    """The film pack: an ordinary loose box whose spawner also draws its two holes and yellow mark on its back."""

    film_pack: bool = True


CAMERA = _camera()
PACK_YAW = _start_yaw()
OBJECTS = [CAMERA, FilmPack("film_pack", PACK_START, color=(0.06, 0.06, 0.07), mass=PACK_KG, size=PACK, yaw=PACK_YAW)]


# -- the simulator side: the pack's marks, the cover's lock, the shutter, and the film cover ----------------------------
def _pack_spawner():
    from isaaclab.sim.spawners.shapes import spawn_cuboid
    from isaaclab.sim.utils import bind_visual_material, clone
    import isaaclab.sim as sim_utils
    from pxr import Gf, UsdGeom

    @clone
    def spawn_pack(prim_path, cfg, translation=None, orientation=None, **kwargs):
        prim = spawn_cuboid.__wrapped__(prim_path, cfg, translation, orientation, **kwargs)
        stage = prim.GetStage()
        marks = [("hole", (c[0], c[1]), s, (0.30, 0.30, 0.32)) for c, s in HOLES]
        marks.append(("yellow", YELLOW_AT, (0.012, 0.012), YELLOW))
        for i, (kind, (x, y), (sx, sy), color) in enumerate(marks):
            look = f"{prim_path}/{kind}_look_{i}"
            surface = sim_utils.PreviewSurfaceCfg(diffuse_color=color, roughness=0.6)
            surface.func(look, surface)
            mark = UsdGeom.Cube.Define(stage, f"{prim_path}/{kind}_{i}")  # drawn only: no collider, no mass
            mark.CreateSizeAttr(1.0)
            mark.AddTranslateOp().Set(Gf.Vec3d(x, y, PACK[2] / 2 + 0.0001))
            mark.AddScaleOp().Set(Gf.Vec3f(sx, sy, 0.0002))
            bind_visual_material(str(mark.GetPrim().GetPath()), look, stage=stage)
        return prim

    return spawn_pack


@dataclass(frozen=True)
class Pads:
    """One gripper after a control step (world frame, m): the span along its pointing axis from the pads' back to
    their tips, and points across the end of each of its two fingertips."""

    back: np.ndarray
    tips: np.ndarray
    fingertips: tuple = ()


PAD_BACK = 0.025  # the pads reach this far behind the TCP
GRIPPERS = ("left_gripper", "right_gripper")
FINGERTIP_ENDS = {  # as in copy-id-card: the end of each fingertip in its link's frame (m)
    "tip_left": [(x, -0.04454, -0.09016) for x in (0.01799, 0.02399, 0.02999)],
    "tip_right": [(x, 0.04454, -0.09016) for x in (-0.02999, -0.02399, -0.01799)],
}


def _gripper_states(env) -> dict:
    from yam_arena import runtime

    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy()
    out = {}
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        poses = runtime.link_poses(robot)
        axis = poses["gripper"][:3, 2]
        tcp = runtime.tcp_world(robot) - origin
        ends = tuple(poses[link][:3, :3] @ np.array(p) + poses[link][:3, 3] - origin
                     for link, points in FINGERTIP_ENDS.items() for p in points)
        out[f"{side}_gripper"] = Pads(back=tcp - PAD_BACK * axis, tips=tcp + PAD_TIP_PAST_TCP * axis, fingertips=ends)
    return out


@dataclass(frozen=True)
class FilmRecord:
    """This trial so far: ``locked`` whether the back cover is locked shut now; ``ejected`` whether a shutter press
    has ejected the film cover; ``presses`` how many shutter presses; ``forced`` whether the cover was ever pulled
    open past its lock."""

    locked: bool = True
    ejected: bool = False
    presses: int = 0
    forced: bool = False


_REC = {"locked": True, "popping": False, "ejected": False, "presses": 0, "down": False, "forced": False}


def _clear_record() -> None:
    _REC.update(locked=True, popping=False, ejected=False, presses=0, down=False, forced=False)


mech.on_reset(_clear_record)


def lock_step(rec: dict, cover: float, lock: float) -> float:
    """The back cover's lock, once a control step: returns the cover spring's target (rad).

    Locked: held shut (target 0) until the tab is pressed 2 mm down; then it springs ajar to POP_DEG. Unlocked: the
    spring follows the cover wherever it is put, so it stays there; pushed back within LATCH_DEG with the tab up, it
    locks again."""
    if rec["locked"]:
        if -lock >= UNLOCKED:
            rec.update(locked=False, popping=True)
            return math.radians(POP_DEG)
        if cover > math.radians(FORCED_DEG):
            rec["forced"] = True  # pulled open past the lock: in a real camera, a broken lock or fogged film
        return 0.0
    if rec["popping"] and cover >= math.radians(POP_DEG - 3.0):
        rec["popping"] = False
    if rec["popping"]:
        return math.radians(POP_DEG)
    if cover <= math.radians(LATCH_DEG) and -lock < UNLOCKED / 2:
        rec["locked"] = True
        return 0.0
    return cover


def couple(environment) -> None:
    """Before each control step: the cover's lock, and the film cover out of the slot once ejected."""
    j = mech.joint_positions(environment, CAMERA)
    target = lock_step(_REC, float(j["back_cover"]), float(j["lock"]))
    mech.set_targets(environment, CAMERA, {"back_cover": target, "film_cover": EJECT if _REC["ejected"] else 0.0})


mech.on_step(couple)


def _record(states) -> FilmRecord:
    """Look at the shutter after a step: a press that has just gone down ejects the film cover if the camera is
    ready (loaded right, locked shut, switched on)."""
    camera, pack = states.get("camera"), states.get("film_pack")
    if camera is not None and pack is not None:
        down = -float(camera.joints.get("shutter", 0.0)) >= PRESSED
        if down and not _REC["down"]:
            _REC["presses"] += 1
            if ready(states):
                _REC["ejected"] = True
        _REC["down"] = down
    return FilmRecord(_REC["locked"], _REC["ejected"], _REC["presses"], _REC["forced"])


def _patch_scene(scene) -> None:
    if getattr(scene, "_film_patched", False):
        return
    stock_object, stock_states = scene.task_object, scene.object_states

    def task_object(spec):
        obj = stock_object(spec)
        if getattr(spec, "film_pack", False):
            obj.spawner_cfg.func = _pack_spawner()
        return obj

    def object_states(env, names):
        states = stock_states(env, names)
        states = {**states, **_gripper_states(env)}
        return {**states, "film_record": _record(states)}

    scene.task_object, scene.object_states = task_object, object_states
    scene._film_patched = True


# WHY through mechanism's own import hook rather than a second one: two hooks that each look the module up through
# the other finders called each other without end (copy-id-card, official sim, 2026-11-20).
_mech_scene_patch = mech._PATCHES["yam_arena.scene"]


def _scene_patches(scene) -> None:
    _mech_scene_patch(scene)
    _patch_scene(scene)


mech._PATCHES["yam_arena.scene"] = _scene_patches
if "yam_arena.scene" in sys.modules:
    _scene_patches(sys.modules["yam_arena.scene"])


# -- scoring: a function of the object states (the pack, the camera's joints, the grippers, the record) -----------------
TURN_DEG = 10.0  # the pack's axes within this of where they belong
CENTRE_MM = 3.5  # the pack's middle within this of the chamber's (x and z): its 3 mm play, and 0.5 mm
SEATED_MM = 1.5  # its back within this of where a fully seated pack's back is
CLEAR = 0.02  # hands off: no point along either gripper's pads or fingertips within 2 cm of the camera
COMPLETE_SCORE = 0.9  # score() for a finished load; 1.0 comes only from success() holding 1 s, which ends the trial


def _rot(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _upright_rot(pack) -> np.ndarray:
    """The pack's rotation in the camera's upright frame."""
    return R_TILT.T @ _rot(pack.quat_xyzw)


def right_way(pack) -> bool:
    """Its yellow-marked end up and its back (the holes) facing out of the chamber: the yellow marks together."""
    r, c = _upright_rot(pack), math.cos(math.radians(TURN_DEG))
    return float(r[2, 0]) >= c and float(-r[1, 2]) >= c


def in_chamber(pack) -> bool:
    """Its middle inside the chamber's opening (x, z) and at least half its thickness past the chamber's mouth."""
    p = to_upright(pack.pos)
    return (abs(p[0] - CH_X) <= CH_W / 2 and abs(p[2] - TOP - CH_Z) <= CH_H / 2
            and CH_WALL - CH_D <= p[1] <= CH_WALL)


def seated(pack) -> bool:
    """In the chamber the right way, standing on its floor, centred across it, and pushed fully home (its back flush
    with the chamber's mouth)."""
    p = to_upright(pack.pos)
    back = p[1] - PACK[2] / 2  # the right way, its back faces -y
    home = CH_WALL - PACK[2]
    rest_z = TOP + CH_Z0 + PACK[0] / 2  # standing on the chamber's floor
    return (right_way(pack) and abs(p[0] - CH_X) * 1000 <= CENTRE_MM and abs(p[2] - rest_z) * 1000 <= CENTRE_MM
            and abs(back - home) * 1000 <= SEATED_MM)


def cover_locked(objects) -> bool:
    record = objects.get("film_record") or FilmRecord()
    return record.locked and float(objects["camera"].joints["back_cover"]) <= math.radians(LATCH_DEG)


def switched_on(objects) -> bool:
    return -float(objects["camera"].joints["lens_ring"]) >= math.radians(ON_DEG)


def ready(objects) -> bool:
    """What a shutter press now would do: eject the film cover from a camera loaded right, locked shut and on."""
    return seated(objects["film_pack"]) and cover_locked(objects) and switched_on(objects)


def _near_camera(pads: Pads) -> bool:
    lo = np.array([X0, YB - COVER_T, TOP]) - CLEAR
    hi = np.array([X1, RING_Y[1] + 0.002, TOP + H + EJECT]) + CLEAR
    along = [pads.back + s * (pads.tips - pads.back) for s in np.linspace(0.0, 1.0, 11)]
    points = (*along, *(np.asarray(f, float) for f in pads.fingertips))
    return any(bool(np.all(q >= lo) and np.all(q <= hi)) for q in (to_upright(w) for w in points))


def hands_off(objects) -> bool:
    pads = [objects.get(g) for g in GRIPPERS]
    return all(p is not None for p in pads) and not any(_near_camera(p) for p in pads)


def grade(objects) -> float:
    """1.0: the film cover ejected by a shutter press with the pack seated the right way, the back cover locked shut
    and the camera on; still so, both hands 2 cm clear of the camera, the pack still. 0.75: the pack seated the right
    way, the cover locked, the ring at ON (not yet ejected, or a hand near). 0.5: seated the right way and the cover
    locked. 0.25: seated the right way. 0: anything else, including a pack in the chamber the wrong way round and the
    back cover ever pulled open past its lock."""
    pack = objects["film_pack"]
    record = objects.get("film_record") or FilmRecord()
    if record.forced or not seated(pack):
        return 0.0
    if not cover_locked(objects):
        return 0.25
    if record.ejected:
        return 1.0 if hands_off(objects) and pack.resting() else 0.75
    return 0.75 if switched_on(objects) else 0.5


def _debug(objects, g) -> None:
    """With FILM_DEBUG=path set, one line per control step."""
    path = os.environ.get("FILM_DEBUG")
    if not path:
        return
    pack = objects["film_pack"]
    r = _upright_rot(pack)
    j = objects["camera"].joints
    rec = objects.get("film_record") or FilmRecord()

    def tips(name):
        pads = objects.get(name)
        return "?" if pads is None else "(%.0f,%.0f,%.0f)" % tuple(to_upright(pads.tips) * 1000)

    p = to_upright(pack.pos) * 1000
    with open(path, "a") as f:
        f.write(f"grade={g} pack=({p[0]:.1f},{p[1]:.1f},{p[2] - TOP * 1000:.1f}) up={r[2, 0]:.3f} out={-r[1, 2]:.3f} "
                f"in_chamber={in_chamber(pack)} seated={seated(pack)} cover_deg={math.degrees(j['back_cover']):.1f} "
                f"lock_mm={j['lock'] * 1000:.2f} ring_deg={math.degrees(j['lens_ring']):.1f} "
                f"shutter_mm={j['shutter'] * 1000:.2f} sheet_mm={j['film_cover'] * 1000:.1f} locked={rec.locked} "
                f"presses={rec.presses} ejected={rec.ejected} "
                f"hands_off={hands_off(objects)} L={tips('left_gripper')} R={tips('right_gripper')}\n")


def score(objects) -> float:
    g = grade(objects)
    _debug(objects, g)
    return min(g, COMPLETE_SCORE)


def success(objects) -> bool:
    return grade(objects) == 1.0


# -- the scripted demo --------------------------------------------------------------------------------------------------
def _allow_matrix_grips() -> None:
    """Let a Waypoint's ``yaw`` be a full 3x3 TCP rotation instead of an angle (yam_arena.demo builds each waypoint's
    rotation with grip_rotation(yaw), which this wraps)."""
    if getattr(yam_demo, "_matrix_grips", False):
        return
    plain = yam_demo.grip_rotation

    def grip_rotation(yaw=0.0):
        return np.asarray(yaw, float) if np.ndim(yaw) == 2 else plain(yaw)

    yam_demo.grip_rotation = grip_rotation
    yam_demo._matrix_grips = True


_allow_matrix_grips()

X_AXIS, Y_AXIS, Z_AXIS = np.eye(3)
DOWN = yam_demo.grip_rotation(0.0)  # pointing down, fingers closing along y
DOWN_X = yam_demo.grip_rotation(math.pi / 2)  # pointing down, fingers closing along x (WHY +90: at -90 the right
# arm's wrist roll is past its limit over the lock, offline 2026-11-22)
_INTO_BACK = axis_angle_matrix(X_AXIS, math.pi / 2) @ DOWN  # the right hand pointing +y, at the camera's back
_INTO_FRONT = axis_angle_matrix(X_AXIS, -math.pi / 2) @ DOWN  # the left hand pointing -y, at the camera's front
BACK_FLAT = _INTO_BACK  # fingers closing up and down
FRONT_FLAT = _INTO_FRONT
# The pack is carried with the fingers closing along x on its long sides; the wrist's roll sign is the arm's reachable
# one (offline IK, 2026-11-22: the right arm pointing +y reaches the camera's back with roll -90, not +90).
BACK_PINCH = axis_angle_matrix(Y_AXIS, -math.pi / 2) @ _INTO_BACK


def _pt(*v) -> tuple:
    return tuple(float(a) for a in v)


def down_tcp(x, y, tips_z) -> tuple:
    return _pt(x, y, tips_z + PAD_TIP_PAST_TCP)


def into_back_tcp(x, tips_y, z) -> tuple:
    return _pt(x, tips_y - PAD_TIP_PAST_TCP, z)


def into_front_tcp(x, tips_y, z) -> tuple:
    return _pt(x, tips_y + PAD_TIP_PAST_TCP, z)


HINGE_W = np.array([HINGE[0], HINGE[1], TOP + HINGE[2]])


def along_cover(deg: float) -> np.ndarray:
    """Pointing down, turned so that the closed hand's two fingertips (48 mm apart) lie along the cover opened
    ``deg``: both on one face of it. WHY: across it, one fingertip landed on the cover's edge or its far side."""
    return yam_demo.grip_rotation(math.radians(deg))


def on_cover(deg: float, along: float, off: float, z: float) -> tuple:
    """A point ``along`` the cover from its hinge and ``off`` its face (+ inside, towards the camera; - outside) with
    the cover opened ``deg``, at height ``z`` above the table."""
    a = math.radians(deg)
    d = np.array([-math.cos(a), -math.sin(a), 0.0])  # from the hinge towards the free edge
    n = np.array([-math.sin(a), math.cos(a), 0.0])  # the cover's inner normal
    p = HINGE_W + along * d + off * n
    return _pt(p[0], p[1], TOP + z)


# Moves at the camera are planned as if it stood upright, then carried by its tilt. The hand's rotation follows a
# share of the tilt (FILM_K_*: 1, all of it, a rigid move with the camera; 0, none, the hand as if the camera were
# upright; below 0, leaning the other way). The shares are where the arms reach (offline IK, 2026-11-23, 30 degrees:
# the hand tilted fully with the camera, the lock, the cover and the front were 28-90 mm off target; with these, every
# step is within 0.1 mm).
POINTING = DOWN.T @ np.array([0.0, 0.0, -1.0])  # the TCP's pointing axis in the hand's own frame
K_LOCK = float(os.environ.get("FILM_K_LOCK", "-0.5"))
K_BACK = float(os.environ.get("FILM_K_BACK", "1"))  # the pack into the chamber, and the cover pushed shut
K_FRONT = float(os.environ.get("FILM_K_FRONT", "0"))  # the lens ring and the shutter button


def at_camera(duration: float, tcp_u, finger: float, label: str, rot_u: np.ndarray, k: float,
              turn: np.ndarray | None = None) -> Waypoint:
    """A waypoint planned in the camera's upright frame (the TCP at ``tcp_u``, the hand turned ``rot_u``): the
    fingertips' point goes with the camera's tilt, the hand turns ``k`` of it (then ``turn``, a world rotation, if
    given), and the TCP sits behind the fingertips along its pointing axis."""
    tips = to_world(np.asarray(tcp_u, float) + PAD_TIP_PAST_TCP * (rot_u @ POINTING))
    r = axis_angle_matrix(X_AXIS, ALPHA * k) @ rot_u
    if turn is not None:
        r = turn @ r
    return Waypoint(duration, _pt(*(tips - PAD_TIP_PAST_TCP * (r @ POINTING))), finger, label, r)


# The pack's start yaw is chosen so that the right hand's pick (pointing down, fingers across the pack) and carry
# (pointing +y with BACK_PINCH) bring its yellow end up and its back out; see test_load_instant_film.
PICK_GRIP = DOWN_X  # fingers across the pack's width
PICK_TIPS_UNDER_TOP = float(os.environ.get("FILM_GRIP", "0.004"))  # the fingertips this far below the pack's top:
# they hold its back 4 mm, so they stay outside the chamber's mouth (it has no room for fingers beside the pack)
CARRY_DZ = float(os.environ.get("FILM_CARRY_DZ", "0.0"))  # the carry's height shifted, for margin runs
PICK_TCP = down_tcp(PACK_START[0], PACK_START[1], PACK_START[2] + PACK[2] / 2 - PICK_TIPS_UNDER_TOP)
PARK_RIGHT, PARK_LEFT = _pt(0.16, -0.33, TOP + 0.14), _pt(0.16, 0.33, TOP + 0.14)
LOW_TRAVEL_Z = TOP + 0.125  # far out (x 0.44), the arm pointing down reaches only this high; clear of the cover (11 cm)
TRAVEL_Z = TOP + 0.15  # travel height for the right hand pointing down (WHY: at 20 cm the arm can't reach over the camera)
LOCK_PRESS = float(os.environ.get("FILM_LOCK_PRESS", "0.0032"))  # how far below the tab's top the fingertips are aimed
COVER_OPEN_DEG = float(os.environ.get("FILM_COVER_OPEN", "95"))  # to its stop (WHY: left at 79, the pack met it)
COVER_PUSH_Z = 0.060  # the fingertips' height when pushing the cover round
SWEEP_TO_DEG = 35.0  # swept shut round its hinge to here, then pushed straight shut
SHUT_PAST = 0.0005
SHUT_FROM = float(os.environ.get("FILM_SHUT_FROM", "0.075"))  # the straight push starts this far behind the back
TURN_AT = float(os.environ.get("FILM_TURN_AT", "0.15"))  # where the hand turns to point at the cover (y, m, negated)
OPEN_R, CLOSE_R = 0.070, 0.075  # how far from the hinge the hand pushes the cover (WHY: nearer the free edge, the
# hand's fingers come down onto the lock tab)
GAP_OFF, OPEN_OFF, CLOSE_OFF = 0.008, 0.0035, -0.004  # off the cover's middle plane: coming down into the gap,
# pushing it open from inside, pushing it shut from outside (the fingers overlap the 3 mm cover, so they push it)
OPEN_LEAN = float(os.environ.get("FILM_OPEN_LEAN", "65"))  # degrees: pushing the cover open, the hand leans this far
# in the cover's own plane, its fingertips towards the hinge (WHY: tilted with the camera, the hand points back at the
# right arm, out of its reach; leaned out of the cover's plane instead, its back met the cover; official sim and
# offline IK, 2026-11-23)
K_SHUT = float(os.environ.get("FILM_K_SHUT", "-1"))  # pushing it shut from outside, the hand leans out (see K_LOCK)
K_EDGE = float(os.environ.get("FILM_K_EDGE", "-1.5"))  # coming round the opened cover's free edge
GAP_Z = TOP + float(os.environ.get("FILM_GAP_Z", "0.13"))  # (upright frame) above the gap behind the ajar cover
UP_Z = TOP + float(os.environ.get("FILM_UP_Z", "0.10"))  # (upright frame) up from the opened cover
EDGE_Z = TOP + float(os.environ.get("FILM_EDGE_Z", "0.10"))  # (upright frame) coming round to the cover's free edge
TIPS_OUT = float(os.environ.get("FILM_TIPS_OUT", "0.002"))  # carried in until the fingertips are this far outside the
# chamber's mouth: the pack is then PACK[2] - TIPS_OUT - PICK_TIPS_UNDER_TOP (14 mm) in
JUST_OPEN = float(os.environ.get("FILM_JUST_OPEN", "-0.033"))  # finger target (m): 66 mm apart, 2.5 mm past each side
# WHY only just open, and slowly: opened fully, the fingers swept back along the pack's sides and threw it out
# (official sim, 2026-11-23)
PUSH_PAST = float(os.environ.get("FILM_PUSH_PAST", "0.0015"))  # pushed this far past flush, so it goes fully home


def leaned(deg: float, rot: np.ndarray, lean: float) -> np.ndarray:
    """``rot`` turned ``lean`` degrees about the normal of the cover opened ``deg``: still in the cover's plane, its
    pointing tipped towards the hinge."""
    a = math.radians(deg)
    n, towards = np.array([-math.sin(a), math.cos(a), 0.0]), np.array([math.cos(a), math.sin(a), 0.0])
    r = axis_angle_matrix(n, math.radians(lean))
    if (r @ rot @ POINTING) @ towards < 0:
        r = axis_angle_matrix(n, -math.radians(lean))
    return r @ rot


def _right():
    lx, ly = sum(LOCK_X) / 2, LOCK_Y[0] + 0.005
    over_lock = down_tcp(lx, ly, TOP + LOCK_Z[1] + 0.02)
    on_lock = down_tcp(lx, ly, TOP + LOCK_Z[1] - LOCK_PRESS)
    pop, stop = POP_DEG, math.degrees(COVER_OPEN_MAX)
    tips_z = COVER_PUSH_Z - 0.013  # the fingertips' height above the table
    tcp_z = tips_z + PAD_TIP_PAST_TCP

    def lock(d, p, label):
        return at_camera(d, p, FINGER_CLOSED, label, DOWN_X, K_LOCK)

    def opening(d, deg, along, off, z, label):
        """Inside the cover opened ``deg``, the fingertips ``along`` it from the hinge, ``off`` its middle plane, at
        height ``z``; the hand along the cover, leaned in its plane."""
        rot = leaned(deg, along_cover(deg), OPEN_LEAN)
        tips = np.array(on_cover(deg, along, off, z))
        return at_camera(d, tips - PAD_TIP_PAST_TCP * (rot @ POINTING), FINGER_CLOSED, label, rot, 1.0)

    def shutting(d, p, label, rot, k=K_SHUT):
        return at_camera(d, p, FINGER_CLOSED, label, rot, k)

    def back(d, p, finger, label, rot):
        return at_camera(d, p, finger, label, rot, K_BACK)

    sweep = [opening(0.25, d, OPEN_R, OPEN_OFF, tips_z, f"cover open {d:g} degrees")
             for d in np.arange(pop + 5.0, stop + 0.1, 5.0)]
    close = [shutting(0.25, on_cover(d, CLOSE_R, CLOSE_OFF, tcp_z), f"cover shut {d:g} degrees", along_cover(d))
             for d in np.arange(stop - 5.0, SWEEP_TO_DEG - 0.1, -5.0)]
    shut_in = into_back_tcp(CX, YB - COVER_T + SHUT_PAST, TOP + CH_Z)  # fingertips 0.5 mm into the shut cover
    # WHY 7.5 cm out: at 6 cm the start was inside the half-shut cover's swing and the hand pushed it open again
    shut_out = into_back_tcp(CX, YB - SHUT_FROM, TOP + CH_Z)
    open_end = on_cover(stop, OPEN_R, GAP_OFF, tcp_z)
    edge = on_cover(stop, W, 0.0, tcp_z)  # the open cover's free edge, at its stop
    out_n = -np.array([-math.sin(COVER_OPEN_MAX), math.cos(COVER_OPEN_MAX)])  # the open cover's outer normal (x, y)
    along = np.array([-math.cos(COVER_OPEN_MAX), -math.sin(COVER_OPEN_MAX)])
    b = np.array(edge[:2]) + 0.035 * along + 0.010 * out_n
    beyond = _pt(b[0], b[1], edge[2])  # past its free edge (the hand's two fingertips lie along the cover), outside
    # The pack, turned to the chamber: its middle on the chamber's line, carried part-way in, let go, then pushed home
    # by the closed hand, its two fingertips (48 mm apart, across the pack) on its back.
    insert_tcp = into_back_tcp(CH_X, YB - TIPS_OUT, TOP + CH_Z + CARRY_DZ)
    approach = _pt(insert_tcp[0], insert_tcp[1] - 0.05, insert_tcp[2])
    rest_z = TOP + CH_Z0 + PACK[0] / 2  # the pack's middle once it stands on the chamber's floor
    push_out = into_back_tcp(CH_X, YB - 0.03, rest_z)
    push_in = into_back_tcp(CH_X, CH_WALL - PACK[2] + PUSH_PAST, rest_z)
    tilted = axis_angle_matrix(X_AXIS, math.pi / 4) @ PICK_GRIP
    turning_down = axis_angle_matrix(X_AXIS, math.pi / 4) @ DOWN
    return [
        # WHY a lead-in: straight from the ready pose, the arm's IK took a branch 22 cm off target (offline, 2026-11-22).
        Waypoint(1.5, _pt(0.28, -0.14, TRAVEL_Z), FINGER_CLOSED, "up, towards the camera's back", DOWN),
        lock(1.2, over_lock, "over the back cover lock"),
        lock(0.8, on_lock, "press the lock down"),
        lock(0.6, on_lock, "hold: the cover springs ajar"),
        lock(0.6, over_lock, "let the lock up"),
        # In behind the ajar cover, then push it open round its hinge to its stop, past upright, where its weight
        # holds it.
        opening(1.0, pop, OPEN_R, GAP_OFF, GAP_Z - TOP, "above the gap"),
        opening(1.0, pop, OPEN_R, GAP_OFF, tips_z, "down into the gap behind the cover"),
        *sweep,
        # WHY out towards the arms first: the open cover's inside faces them; straight up, the arm can't reach.
        shutting(0.6, _pt(open_end[0] - 0.04, open_end[1], open_end[2]), "out from the cover", DOWN),
        shutting(0.8, _pt(open_end[0] - 0.04, open_end[1], UP_Z), "up", DOWN),
        # The pack: pinched across its width, near its back only, then turned to the chamber.
        Waypoint(1.5, _pt(PICK_TCP[0], PICK_TCP[1], TRAVEL_Z), FINGER_OPEN, "over the pack", PICK_GRIP),
        Waypoint(1.0, PICK_TCP, FINGER_OPEN, "down astride it", PICK_GRIP),
        Waypoint(1.5, PICK_TCP, FINGER_CLOSED, "pinch it by its long sides", PICK_GRIP),
        Waypoint(0.5, PICK_TCP, FINGER_CLOSED, "hold", PICK_GRIP),
        Waypoint(0.8, _pt(PICK_TCP[0], PICK_TCP[1], TOP + 0.12), FINGER_CLOSED, "lift it", PICK_GRIP),
        Waypoint(1.2, _pt(0.32, -0.14, TOP + 0.12), FINGER_CLOSED, "towards the camera, tipping it", tilted),
        back(1.2, _pt(approach[0], approach[1], approach[2] + 0.03), FINGER_CLOSED, "upright, above the line",
             BACK_PINCH),
        back(1.2, approach, FINGER_CLOSED, "behind the chamber, yellow end up, back out", BACK_PINCH),
        back(1.5, insert_tcp, FINGER_CLOSED, "part-way into the chamber", BACK_PINCH),
        back(2.0, insert_tcp, JUST_OPEN, "let go, opening the fingers just past its sides", BACK_PINCH),
        back(0.8, approach, JUST_OPEN, "back out", BACK_PINCH),
        back(1.0, push_out, FINGER_CLOSED, "closed, behind the pack's middle", BACK_PINCH),
        back(1.2, push_in, FINGER_CLOSED, "push it home", BACK_PINCH),
        back(0.4, push_in, FINGER_CLOSED, "hold", BACK_PINCH),
        back(0.8, approach, FINGER_CLOSED, "back out again", BACK_PINCH),
        # The cover's outside faces away from the arms: round its free edge, low, and in along its outside.
        # WHY turned back through 45 degrees, as on the way in: turned straight to pointing down, the arm's IK lost its
        # branch by 90 degrees (offline, 2026-11-22).
        back(0.8, _pt(approach[0], -0.12, TOP + 0.08), FINGER_CLOSED, "back from the camera", BACK_FLAT),
        shutting(1.0, _pt(0.33, -0.15, TOP + 0.10), "turning down", turning_down),
        shutting(1.0, _pt(0.37, beyond[1] - 0.01, EDGE_Z), "pointing down", DOWN),
        shutting(1.2, beyond, "round the cover's free edge", along_cover(stop), K_EDGE),
        shutting(1.0, on_cover(stop, CLOSE_R, CLOSE_OFF - 0.006, tcp_z), "in along its outside", along_cover(stop),
                 K_EDGE),
        *close,
        # WHY the last part straight on, pointing at it: swept round nearer shut, the fingers met the lock tab and
        # stopped at 26 degrees (official sim, 2026-11-22).
        shutting(0.6, _pt(*on_cover(SWEEP_TO_DEG, CLOSE_R, -0.03, 0.0)[:2], TOP + 0.10), "up off it",
                 along_cover(SWEEP_TO_DEG)),
        shutting(1.0, _pt(0.40, -TURN_AT, TOP + 0.10), "turning to point at it, outside its swing", turning_down),
        back(1.0, shut_out, FINGER_CLOSED, "in front of it", BACK_FLAT),
        back(1.2, shut_in, FINGER_CLOSED, "push it shut: it locks", BACK_FLAT),
        back(0.6, shut_in, FINGER_CLOSED, "hold it shut", BACK_FLAT),
        back(0.8, shut_out, FINGER_CLOSED, "back off", BACK_FLAT),
        Waypoint(1.5, PARK_RIGHT, FINGER_OPEN, "clear", DOWN),
        Waypoint(20.0, PARK_RIGHT, FINGER_OPEN, "stay", DOWN),
    ]


RING_TURN_DEG = float(os.environ.get("FILM_RING_TURN", "70"))  # WHY 70: turned 40, the ring slipped in the level grip and stopped at 23.7 (official sim, 2026-11-23)
RING_TIPS_Y = YF + 0.006  # the fingertips 6 mm off the body's front, astride the ring
SHUTTER_PRESS = 0.0022  # the fingertips aimed this far into the button's face


def _ring_turned(deg: float) -> np.ndarray:
    """A world rotation: ``deg`` about the hand's pointing axis at the lens, clockwise as seen from the front. WHY not
    about the ring's own tilted axis: turned so past 20 degrees, the arm was 13-34 mm off target (offline IK,
    2026-11-23); the grip's two contacts, on the ring's top and bottom, still move along its rim."""
    return axis_angle_matrix(axis_angle_matrix(X_AXIS, ALPHA * K_FRONT) @ -Y_AXIS, math.radians(deg))


def _left(start_at: float):
    def front(d, p, finger, label, rot=FRONT_FLAT, deg=0.0):
        return at_camera(d, p, finger, label, rot, K_FRONT, _ring_turned(deg) if deg else None)

    ring = into_front_tcp(LENS_X, RING_TIPS_Y, TOP + LENS_Z)
    ring_out = _pt(ring[0], ring[1] + 0.05, ring[2])
    turns = [front(0.5, ring, FINGER_CLOSED, f"turn the ring {d:g} degrees", deg=d)
             for d in np.arange(10.0, RING_TURN_DEG + 0.1, 10.0)]
    sx, sz = SHUTTER_XZ
    face = YF + SHUTTER_PROUD
    shutter_out = into_front_tcp(sx, face + 0.03, TOP + sz)
    shutter_in = into_front_tcp(sx, face - SHUTTER_PRESS, TOP + sz)
    return [
        Waypoint(2.0, _pt(0.30, 0.20, TRAVEL_Z), FINGER_OPEN, "up, out of the way", DOWN),
        Waypoint(max(0.5, start_at - 2.0), _pt(0.30, 0.20, TRAVEL_Z), FINGER_OPEN, "wait for the right hand", DOWN),
        front(1.5, ring_out, FINGER_OPEN, "in front of the lens", FRONT_FLAT),
        front(1.0, ring, FINGER_OPEN, "astride the lens ring", FRONT_FLAT),
        front(1.5, ring, FINGER_CLOSED, "grip the ring", FRONT_FLAT),
        *turns,
        front(0.8, ring, FINGER_OPEN, "let go: the camera is on", deg=RING_TURN_DEG),
        front(0.8, ring_out, FINGER_OPEN, "back", deg=RING_TURN_DEG),
        front(1.0, shutter_out, FINGER_CLOSED, "in front of the shutter button", FRONT_FLAT),
        front(0.8, shutter_in, FINGER_CLOSED, "press it", FRONT_FLAT),
        front(0.5, shutter_in, FINGER_CLOSED, "hold", FRONT_FLAT),
        front(0.6, shutter_out, FINGER_CLOSED, "let it up: the film cover comes out", FRONT_FLAT),
        Waypoint(1.5, PARK_LEFT, FINGER_OPEN, "clear", DOWN),
        Waypoint(8.0, PARK_LEFT, FINGER_OPEN, "stay", DOWN),
    ]


def _ends_of(plan, label: str) -> float:
    """When the step labelled ``label`` ends (s from the start)."""
    t = 0.0
    for w in plan:
        t += w.duration
        if w.label == label:
            return t
    raise KeyError(label)


def demo():
    """The right hand opens the back cover, loads the pack and shuts the cover; then the left hand turns the ring to
    ON and presses the shutter button. Probes: FILM_PACK_TURNED=1 (the pack laid the other way round; the same moves
    load it yellow end down), FILM_DEMO=noring (the ring left OFF before the shutter press)."""
    right = _right()
    start = _ends_of(right, "clear") + 0.2
    left = _left(start)
    variant = os.environ.get("FILM_DEMO", "")
    if variant == "noring":
        left = [w for w in left if "ring" not in w.label]
    return {"left": left, "right": right}
