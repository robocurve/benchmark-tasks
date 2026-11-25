"""On the station table, 22 cm in front of the arms' bases and centred between them, stands a home all-in-one printer
(42.5 cm wide, 30.4 cm deep, 15.4 cm tall), its front towards the arms. Its flat top is a hinged scanner lid, light
grey, that lifts at its front edge (the edge overhangs the printer's front by 1.5 cm) and turns up about a hinge at
the back. Under the lid is the scanner glass (dark), set 1.5 mm below a raised frame whose front and right edges are
the guides; an orange mark on the frame shows the front-right corner, where originals go. To the left of the lid is
the control panel with two buttons: Black Copy (nearer the lid) and Color Copy.

On top of the closed lid, near its front right, lies a plastic ID card (8.6 x 5.4 cm), face up (the side with the dark
photo), sticking 2.5 cm out over the lid's front edge.

Make one black-and-white copy of the card's face: take the card off the lid, lift the lid, lay the card face down on
the glass with its edges against the front and right guides at the marked corner, close the lid, and press Black Copy.
"""

import math
import os
import pathlib
import sys
from dataclasses import dataclass

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

# -- the printer --------------------------------------------------------------------------------------------------------
# HP DeskJet 2800e series all-in-one: body 425 x 304 x 154 mm, 3.42 kg (HP's spec pages for the 2810e and 2742e, the
# same body). Scan area 21.6 x 29.7 cm from the glass (HP DeskJet user guide, Specifications). The lid's size and
# hinge, the glass's depth below the frame and the buttons' places are estimates until a printer is measured.
# Heights below are above the table; x, y are world (x forward from the arm bases, y left).
BODY_X0, BODY_D, BODY_W = 0.22, 0.304, 0.425
BODY_X1 = BODY_X0 + BODY_D
FRAME_Z = 0.140  # top of the scanner frame
GLASS_DROP = 0.0015  # the glass sits this far below the frame, so the frame's edges are guides
GLASS_Z = FRAME_Z - GLASS_DROP
GX0, GY0 = 0.245, -0.180  # the glass's front-right corner (front guide at x GX0, right guide at y GY0)
GLASS_X, GLASS_Y = 0.216, 0.297  # the scan area
GX1, GY1 = GX0 + GLASS_X, GY0 + GLASS_Y
LID_X0 = 0.205  # the lid's front edge, 1.5 cm proud of the body's front face: where a hand lifts it
LID_Y0, LID_Y1 = -BODY_W / 2, 0.140  # the lid covers the glass and frame; the control panel is to its left
LID_T = 0.014
LID_TOP = FRAME_Z + LID_T
LID_KG = 0.45  # [estimate]
HINGE = (0.517, 0.0, FRAME_Z + LID_T / 2)
LID_OPEN_MAX = math.radians(95.0)
PANEL_Z = FRAME_Z + 0.006
BUTTON = (0.012, 0.012, 0.004)
BUTTON_PROUD = 0.002
BUTTON_TRAVEL = 0.0025
BLACK_XY, COLOR_XY = (0.245, 0.165), (0.245, 0.190)
PRESSED = 0.0015  # a button counts as pressed once it is this far down

WHITE, DARK, GLASS_COLOR, PANEL_COLOR = (0.88, 0.88, 0.86), (0.32, 0.32, 0.34), (0.10, 0.14, 0.18), (0.22, 0.22, 0.24)
ORANGE, BLACK_KEY, CYAN_KEY = (1.0, 0.55, 0.0), (0.05, 0.05, 0.05), (0.0, 0.65, 0.85)
MATERIALS = {"plastic": (0.6, 0.5), "glass": (0.3, 0.25)}


def _box(x0, x1, y0, y1, z0, z1, color, material="plastic") -> Shape:
    return Shape("box", ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), (x1 - x0, y1 - y0, z1 - z0), color,
                 material=material)


def _printer():
    body_top = GLASS_Z - 0.001  # the glass is a 1 mm plate on the body
    yw = BODY_W / 2
    base = Link("body", 3.4, (
        _box(BODY_X0, BODY_X1, -yw, yw, 0.0, body_top, WHITE),
        _box(GX0, GX1, GY0, GY1, body_top, GLASS_Z, GLASS_COLOR, "glass"),
        _box(BODY_X0, GX0, -yw, LID_Y1, body_top, FRAME_Z, DARK),  # front guide
        _box(GX1, BODY_X1, -yw, LID_Y1, body_top, FRAME_Z, DARK),
        _box(GX0, GX1, -yw, GY0, body_top, FRAME_Z, DARK),  # right guide
        _box(GX0, GX1, GY1, LID_Y1, body_top, FRAME_Z, DARK),
        _box(BODY_X0, BODY_X1, LID_Y1, yw, body_top, PANEL_Z, PANEL_COLOR),  # control panel
        _box(GX0 - 0.009, GX0 - 0.001, GY0 + 0.001, GY0 + 0.009, FRAME_Z, FRAME_Z + 0.0004, ORANGE),  # corner mark
    ))
    lid = Link("lid", LID_KG, (_box(LID_X0, BODY_X1, LID_Y0, LID_Y1, FRAME_Z, LID_TOP, WHITE),))
    buttons, joints = [], [
        Joint("lid", "revolute", "lid", HINGE, "Y", 0.0, LID_OPEN_MAX, damping=0.2),
    ]
    for key, (x, y), color in (("black_copy", BLACK_XY, BLACK_KEY), ("color_copy", COLOR_XY, CYAN_KEY)):
        top = PANEL_Z + BUTTON_PROUD
        buttons.append(Link(key, 0.005, (Shape("box", (x, y, top - BUTTON[2] / 2), BUTTON, color),)))
        joints.append(Joint(key, "prismatic", key, (x, y, top), "Z", -BUTTON_TRAVEL, 0.0, stiffness=400.0,
                            damping=2.0, target=0.0))
    return mechanism("printer", (0.0, 0.0, TOP), base, (lid, *buttons), joints,
                     {**MATERIALS, "default": (0.8, 0.7)})


# -- the card -----------------------------------------------------------------------------------------------------------
# ID-1 (ISO/IEC 7810): 85.60 x 53.98 x 0.76 mm; about 5 g [estimate]. It lies with its long side left-right.
CARD = (0.05398, 0.0856, 0.00076)  # x, y, z
CARD_KG = 0.005
CARD_OVERHANG = 0.025
CARD_XY = (LID_X0 - CARD_OVERHANG + CARD[0] / 2, -0.110)
CARD_START = (CARD_XY[0], CARD_XY[1], TOP + LID_TOP + CARD[2] / 2 + 0.0003)
CARD_BLUE = (0.45, 0.70, 0.95)
PHOTO = (0.022, 0.018)  # the dark photo on the card's face (x, y), drawn only
PHOTO_AT = (0.0, 0.025)


@dataclass(frozen=True)
class Card(Box):
    """The ID card: an ordinary loose box whose spawner also draws a photo on its face (+z side)."""

    face_photo: bool = True


PRINTER = _printer()
OBJECTS = [
    PRINTER,
    Card("card", CARD_START, color=CARD_BLUE, mass=CARD_KG, size=CARD),
]


# -- the simulator side: the card's face, the glass's friction, the grippers, and a record of every Copy press ---------
def _card_spawner():
    from isaaclab.sim.spawners.shapes import spawn_cuboid
    from isaaclab.sim.utils import bind_visual_material, clone
    import isaaclab.sim as sim_utils
    from pxr import Gf, UsdGeom

    @clone
    def spawn_card(prim_path, cfg, translation=None, orientation=None, **kwargs):
        prim = spawn_cuboid.__wrapped__(prim_path, cfg, translation, orientation, **kwargs)
        stage = prim.GetStage()
        look = f"{prim_path}/photo_look"
        surface = sim_utils.PreviewSurfaceCfg(diffuse_color=(0.02, 0.02, 0.03), roughness=0.6)
        surface.func(look, surface)
        photo = UsdGeom.Cube.Define(stage, f"{prim_path}/photo")  # drawn only: no collider, no mass
        photo.CreateSizeAttr(1.0)
        photo.AddTranslateOp().Set(Gf.Vec3d(PHOTO_AT[0], PHOTO_AT[1], CARD[2] / 2 + 0.0001))
        photo.AddScaleOp().Set(Gf.Vec3f(PHOTO[0], PHOTO[1], 0.0002))
        bind_visual_material(str(photo.GetPrim().GetPath()), look, stage=stage)
        return prim

    return spawn_card


def _install_glass_friction() -> None:
    """Glass against a plastic card is slippery: the glass's friction combines with "min", so the card slides on it at
    the glass's 0.3 / 0.25 while a fingertip pressing on the card still drags it (the task-file format gives every
    loose object 1.2 / 1.0, which with PhysX's default averaging would make the card grip the glass at 0.75)."""
    if getattr(mech, "_glass_min", False):
        return
    plain = mech._spawn

    def spawn(prim_path, cfg, translation=None, orientation=None, **kwargs):
        prim = plain(prim_path, cfg, translation, orientation, **kwargs)
        from pxr import PhysxSchema

        material = prim.GetStage().GetPrimAtPath(f"{prim_path}/Looks/physics_glass")
        if material.IsValid():
            api = PhysxSchema.PhysxMaterialAPI.Apply(material)
            api.CreateFrictionCombineModeAttr().Set("min")
        return prim

    mech._spawn = spawn
    mech._glass_min = True


_install_glass_friction()


@dataclass(frozen=True)
class Pads:
    """One gripper after a control step (world frame, m): the span along its pointing axis from the pads' back to
    their tips, and points across the end of each of its two fingertips."""

    back: np.ndarray
    tips: np.ndarray
    fingertips: tuple = ()


PAD_BACK = 0.025  # the pads reach this far behind the TCP
GRIPPERS = ("left_gripper", "right_gripper")
FINGERTIP_ENDS = {  # as in break-chocolate-row: the end of each fingertip in its link's frame (m)
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
class CopyRecord:
    """This trial's Copy presses. ``presses`` counts every press of either button and ``color`` the Color Copy presses.
    ``good`` means the first press was Black Copy, made with the lid shut and the card face down, square at the corner:
    the one copy asked for. ``max_lid`` is how far the lid has been opened (rad)."""

    presses: int = 0
    good: bool = False
    max_lid: float = 0.0
    color: int = 0

    def asked_for_copy(self) -> bool:
        """Exactly one copy made, and it is the black-and-white copy of the card's face asked for."""
        return self.presses == 1 and self.good

    def wrong_copy(self) -> bool:
        """Any copy other than that one: a Color Copy press, a Black Copy press with the card not in place or the lid
        open, or a second copy of any kind."""
        return self.presses > 0 and not self.asked_for_copy()


_REC = {"presses": 0, "good": False, "max_lid": 0.0, "color": 0, "down": {"black_copy": False, "color_copy": False}}


def _clear_record() -> None:
    _REC.update(presses=0, good=False, max_lid=0.0, color=0, down={"black_copy": False, "color_copy": False})


mech.on_reset(_clear_record)


def _record(states) -> CopyRecord:
    """Look at the buttons after a step: a button that has just gone down is a press. Only a first press, of Black
    Copy, with the copy good, makes the record good; every press is counted."""
    printer = states.get("printer")
    if printer is not None and "card" in states:
        joints = printer.joints
        _REC["max_lid"] = max(_REC["max_lid"], float(joints.get("lid", 0.0)))
        for key in ("black_copy", "color_copy"):
            down = float(joints.get(key, 0.0)) <= -PRESSED
            if down and not _REC["down"][key]:
                _REC["presses"] += 1
                if key == "color_copy":
                    _REC["color"] += 1
                elif _REC["presses"] == 1 and copy_is_good(states):
                    _REC["good"] = True
            _REC["down"][key] = down
    return CopyRecord(_REC["presses"], _REC["good"], _REC["max_lid"], _REC["color"])


def _patch_scene(scene) -> None:
    if getattr(scene, "_copy_card_patched", False):
        return
    stock_object, stock_states = scene.task_object, scene.object_states

    def task_object(spec):
        obj = stock_object(spec)
        if getattr(spec, "face_photo", False):
            obj.spawner_cfg.func = _card_spawner()
        return obj

    def object_states(env, names):
        states = stock_states(env, names)
        states = {**states, **_gripper_states(env)}
        return {**states, "copy_record": _record(states)}

    scene.task_object, scene.object_states = task_object, object_states
    scene._copy_card_patched = True


# WHY through mechanism's own import hook rather than a second one: two hooks that each look the module up through
# the other finders called each other without end ("maximum recursion depth exceeded", official sim, 2026-11-20).
_mech_scene_patch = mech._PATCHES["yam_arena.scene"]


def _scene_patches(scene) -> None:
    _mech_scene_patch(scene)
    _patch_scene(scene)


mech._PATCHES["yam_arena.scene"] = _scene_patches
if "yam_arena.scene" in sys.modules:
    _scene_patches(sys.modules["yam_arena.scene"])


# -- scoring: a function of the object states (the card, the printer's joints, the grippers, the press record) ---------
FACE_DOWN_DEG = 10.0
ALIGN_MM = 4.0  # each guided edge within this of its guide
SQUARE_DEG = 3.0
LID_SHUT_DEG = 2.0
LID_OPENED_DEG = 60.0
CLEAR = 0.02  # hands off: no point along either gripper's pads or fingertips within 2 cm of the card
COMPLETE_SCORE = 0.9  # score() for a finished copy; 1.0 comes only from success() holding 1 s, which ends the trial


def _rot(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def corners(card) -> np.ndarray:
    """The card's four corners (x, y) in the world, from its pose and true size (its middle plane)."""
    r, p = _rot(card.quat_xyzw), np.asarray(card.pos, float)
    hx, hy = CARD[0] / 2, CARD[1] / 2
    return np.array([(p + r @ np.array([sx * hx, sy * hy, 0.0]))[:2] for sx in (-1, 1) for sy in (-1, 1)])


def face_down(card) -> bool:
    return float(_rot(card.quat_xyzw)[2, 2]) <= -math.cos(math.radians(FACE_DOWN_DEG))


def face_up(card) -> bool:
    return float(_rot(card.quat_xyzw)[2, 2]) >= math.cos(math.radians(FACE_DOWN_DEG))


def on_glass(card) -> bool:
    """Lying flat on the glass, every corner inside the scan area (not on the frame or under it)."""
    c = corners(card)
    inside = bool(np.all(c[:, 0] >= GX0 - 0.0005) and np.all(c[:, 0] <= GX1 + 0.0005)
                  and np.all(c[:, 1] >= GY0 - 0.0005) and np.all(c[:, 1] <= GY1 + 0.0005))
    lying = abs(float(card.pos[2]) - (TOP + GLASS_Z + CARD[2] / 2)) <= 0.0015
    return inside and lying


def squareness_deg(card) -> float:
    """How far the card's edges are turned from the guides (degrees, 0-45)."""
    a = math.degrees(math.atan2(*_rot(card.quat_xyzw)[[1, 0], 0])) % 90.0
    return min(a, 90.0 - a)


def aligned(card) -> bool:
    """On the glass, its front edge within ALIGN_MM of the front guide and its right edge of the right guide, square."""
    c = corners(card)
    front, right = (float(c[:, 0].min()) - GX0) * 1000, (float(c[:, 1].min()) - GY0) * 1000
    return on_glass(card) and front <= ALIGN_MM and right <= ALIGN_MM and squareness_deg(card) <= SQUARE_DEG


def lid_shut(objects) -> bool:
    return float(objects["printer"].joints["lid"]) <= math.radians(LID_SHUT_DEG)


def copy_is_good(objects) -> bool:
    """What a Copy press at this moment would copy: the card's face, square at the corner, under the shut lid."""
    card = objects["card"]
    return lid_shut(objects) and face_down(card) and aligned(card)


def _touches(pads: Pads, box) -> bool:
    lo, hi = np.asarray(box.low, float) - CLEAR, np.asarray(box.high, float) + CLEAR
    along = [pads.back + s * (pads.tips - pads.back) for s in np.linspace(0.0, 1.0, 11)]
    return any(bool(np.all(p >= lo) and np.all(p <= hi)) for p in (*along, *(np.asarray(q, float) for q in pads.fingertips)))


def hands_off(objects) -> bool:
    pads = [objects.get(g) for g in GRIPPERS]
    return all(p is not None for p in pads) and not any(_touches(p, objects["card"]) for p in pads)


def grade(objects) -> float:
    """1.0: exactly one copy made, a Black Copy press with the lid shut and the card face down, square at the corner;
    the card still there under the shut lid, both hands 2 cm off it, everything still. 0.75: the card face down at the
    corner and the lid shut, but no copy yet, or a hand still near the card. 0.5: the card face down inside the scan
    area. 0.25: the lid opened past 60 degrees and the card on the glass face up. 0: anything else, and any wrong or
    extra copy, whatever follows it: a Color Copy press, a Black Copy press with the card not in place or the lid open,
    or a second copy of any kind."""
    card = objects["card"]
    record = objects.get("copy_record") or CopyRecord()
    if record.wrong_copy():
        return 0.0
    placed = face_down(card) and aligned(card) and lid_shut(objects)
    if placed:
        if record.asked_for_copy() and hands_off(objects) and card.resting():
            return 1.0
        return 0.75
    if face_down(card) and on_glass(card):
        return 0.5
    if face_up(card) and on_glass(card) and record.max_lid >= math.radians(LID_OPENED_DEG):
        return 0.25
    return 0.0


def _debug(objects, g) -> None:
    """With COPY_DEBUG=path set, one line per control step."""
    path = os.environ.get("COPY_DEBUG")
    if not path:
        return
    card = objects["card"]
    c = corners(card)
    j = objects["printer"].joints
    rec = objects.get("copy_record") or CopyRecord()

    def tips(name):
        pads = objects.get(name)
        return "?" if pads is None else "(%.0f,%.0f,%.0f)" % tuple(np.asarray(pads.tips, float) * 1000)

    def ends(name):  # the right gripper's two fingertips' middles, mm: where they are and how far apart
        pads = objects.get(name)
        if pads is None or len(pads.fingertips) < 6:
            return "?"
        a, b = np.asarray(pads.fingertips[1], float) * 1000, np.asarray(pads.fingertips[4], float) * 1000
        return "(%.0f,%.0f,%.1f)-(%.0f,%.0f,%.1f)" % (*a, *b)

    p = np.asarray(card.pos, float) * 1000
    with open(path, "a") as f:
        f.write(f"grade={g} card=({p[0]:.1f},{p[1]:.1f},{p[2] - TOP * 1000:.2f}) up_z={_rot(card.quat_xyzw)[2, 2]:.3f} "
                f"front_mm={(c[:, 0].min() - GX0) * 1000:.1f} right_mm={(c[:, 1].min() - GY0) * 1000:.1f} "
                f"square={squareness_deg(card):.1f} on_glass={on_glass(card)} lid_deg={math.degrees(j['lid']):.1f} "
                f"black={j['black_copy'] * 1000:.2f} color={j['color_copy'] * 1000:.2f} presses={rec.presses} "
                f"color_presses={rec.color} "
                f"good={rec.good} hands_off={hands_off(objects)} L={tips('left_gripper')} R={tips('right_gripper')} "
                f"Rends={ends('right_gripper')}\n")


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

X_AXIS, Y_AXIS = np.array([1.0, 0.0, 0.0]), np.array([0.0, 1.0, 0.0])
DOWN = yam_demo.grip_rotation(0.0)
LEVEL = axis_angle_matrix(Y_AXIS, -math.pi / 2) @ DOWN  # pointing at the printer (+x), fingers closing along y
# The left hand lifts the lid with its closed fingers; which way round they are turned about the pointing axis is
# chosen for the arm's reach (its wrist rolls only -120 to +120 degrees).
LEFT_LEVEL = axis_angle_matrix(X_AXIS, float(os.environ.get("COPY_LEFT_ROLL", "0")) * math.pi / 2) @ LEVEL
# The right hand pinches the card's edge with its fingers above and below it: level, rolled a quarter turn so the
# fingers close vertically; turning the card face down is a further half turn about the pointing axis, in two steps
# (the wrist rolls -120 to +120 degrees).
ROLL = float(os.environ.get("COPY_ROLL_SIGN", "1")) * math.pi / 2
PINCH = axis_angle_matrix(X_AXIS, ROLL) @ LEVEL
HALF = axis_angle_matrix(X_AXIS, -ROLL) @ PINCH
FLIPPED = axis_angle_matrix(X_AXIS, -2 * ROLL) @ PINCH
PLACE_TILT = math.radians(25.0)
PLACE = axis_angle_matrix(Y_AXIS, PLACE_TILT) @ FLIPPED  # tipped forward, the card's far edge lowest


def _pt(*v) -> tuple:
    return tuple(float(a) for a in v)


# Right hand: the card off the lid, turned over, laid on the glass, slid into the corner.
CARD_Z = CARD_START[2]
PINCH_TIPS_X = LID_X0 - 0.003  # pad tips just short of the lid's front edge
PINCH_TCP = _pt(PINCH_TIPS_X - PAD_TIP_PAST_TCP, CARD_XY[1], CARD_Z)
# Where the hand comes down: 4.7 cm right of the card, with its pad tips 3 mm over the card's edge (the arm reaches
# x 0.17 at that height all along y; at 0.18-0.20 it can't right of y -0.17: offline IK, 2026-11-20).
SIDE_X, SIDE_Y = 0.17, CARD_XY[1] - CARD[1] / 2 - 0.047
HOLD_AHEAD = CARD_XY[0] - PINCH_TCP[0]  # the card's middle, ahead of the TCP along the pointing axis
FAR_EDGE_AHEAD = HOLD_AHEAD + CARD[0] / 2
# The card is turned over beside the printer, off the lid's right side, clear of its sweep while the other hand lifts
# it (WHY: nearer the arm's base the pointing-forward grip is out of reach; over the lid, its edge sweeps through).
FLIP_AT = _pt(*(float(v) for v in os.environ.get("COPY_FLIP_AT", "0.26,-0.27,0.22").split(",")))
FLIP_AT = (FLIP_AT[0], FLIP_AT[1], TOP + FLIP_AT[2])
DROP_OFFSET = float(os.environ.get("COPY_DROP_OFFSET", "0.015"))  # laid down this far behind and left, then slid in
ALIGNED_CENTRE = (GX0 + CARD[0] / 2, GY0 + CARD[1] / 2)
DROP_CENTRE = (ALIGNED_CENTRE[0] + DROP_OFFSET, ALIGNED_CENTRE[1] + DROP_OFFSET)
_place_axis = PLACE[:, 2]
_far_edge = np.array([DROP_CENTRE[0] + CARD[0] / 2, DROP_CENTRE[1], TOP + GLASS_Z + 0.0015])
DROP_TCP = _pt(*(_far_edge - FAR_EDGE_AHEAD * _place_axis))
RELEASE = -0.004  # fingers part 8 mm: enough to free the card without the lower finger reaching the glass
WITHDRAW = 0.03  # then the hand draws back along its pointing axis, so the card's near edge slides off the lower finger
# How far into the card's top the fingertips are aimed, so the arm presses down hard enough to drag it.
# WHY 2.5 mm: aimed 0.3 mm in, the fingertips only rested on it (their ends 0.04 mm above its top) and slid 22 mm
# while the card moved 3.5 (official sim, 2026-11-20).
PRESS_IN = float(os.environ.get("COPY_PRESS_IN", "0.0025"))
# Where the card lands: drawing the hand back drags it about 12 mm towards the front (official sim, 2026-11-20: laid
# down for a middle at x 287 mm, it settled at 274.5).
LAND_DRAG = 0.012
LANDED = (DROP_CENTRE[0] - LAND_DRAG, DROP_CENTRE[1])
# Where the fingertips press (from the card's landed middle, towards its back left, so that they stay on the card when
# it stops against the guides and they slide on) and how far they push it: first to the right guide, then forward.
# WHY in turn, not diagonally: pushed diagonally, the card met the front guide first and then hardly moved sideways
# (its right edge stopped 12 mm from the right guide; official sim, 2026-11-20).
PRESS_FROM_MIDDLE = (float(os.environ.get("COPY_PRESS_DX", "0.012")), float(os.environ.get("COPY_PRESS_DY", "0.020")))
SLIDE_Y = float(os.environ.get("COPY_SLIDE_Y", "0.024"))  # towards -y: 15 mm to the guide, 9 mm on
SLIDE_X = float(os.environ.get("COPY_SLIDE_X", "0.008"))  # then towards -x
CARD_TOP = TOP + GLASS_Z + CARD[2]
PARK_RIGHT, PARK_LEFT = _pt(0.16, -0.33, TOP + 0.12), _pt(0.16, 0.33, TOP + 0.12)


def down_tcp(x, y, tips_z) -> tuple:
    return _pt(x, y, tips_z + PAD_TIP_PAST_TCP)


def _right(variant: str):
    px, py = LANDED[0] + PRESS_FROM_MIDDLE[0], LANDED[1] + PRESS_FROM_MIDDLE[1]
    over = down_tcp(px, py, CARD_TOP + 0.03)
    press = down_tcp(px, py, CARD_TOP - PRESS_IN)
    slid_right = down_tcp(px, py - SLIDE_Y, CARD_TOP - PRESS_IN)
    slid = down_tcp(px - SLIDE_X, py - SLIDE_Y, CARD_TOP - PRESS_IN)
    lift = _pt(slid[0], slid[1], slid[2] + 0.04)
    turned = FLIPPED if variant != "faceup" else PINCH
    half = HALF if variant != "faceup" else PINCH
    place = axis_angle_matrix(Y_AXIS, PLACE_TILT) @ turned
    drop = DROP_TCP
    axis = place[:, 2]
    withdrawn = _pt(*(np.asarray(drop) - WITHDRAW * axis))
    return [
        # WHY this way in: started nearer the base, the pointing-forward grip took a branch of the arm's IK that ran
        # its wrist roll into its limit (offline check, 2026-11-20).
        Waypoint(1.6, _pt(0.30, -0.25, TOP + 0.30), FINGER_OPEN, "above the printer's right, turning to point at it", PINCH),
        # WHY from the side, at the card's height: coming down onto it diagonally, the open lower finger swept the card
        # off the lid (official sim, 2026-11-20); from straight in front the arm can't reach. Beside the card, in front of
        # the lid, the fingers come down clear of both, then slide in, one above the card's overhang and one below it.
        Waypoint(1.2, _pt(SIDE_X, SIDE_Y, TOP + 0.22), FINGER_OPEN, "beside the card, above", PINCH),
        Waypoint(1.0, _pt(SIDE_X, SIDE_Y, CARD_Z), FINGER_OPEN, "down beside the card, at its height", PINCH),
        Waypoint(1.0, _pt(SIDE_X, PINCH_TCP[1], CARD_Z), FINGER_OPEN, "slide in sideways, astride the card's edge", PINCH),
        Waypoint(0.8, PINCH_TCP, FINGER_OPEN, "further onto it", PINCH),
        # WHY 1.5 s and a hold: the fingers close slower than they're commanded; at 0.8 s they were still 3 cm apart
        # (official sim, 2026-11-20).
        Waypoint(1.5, PINCH_TCP, FINGER_CLOSED, "pinch the card", PINCH),
        Waypoint(0.5, PINCH_TCP, FINGER_CLOSED, "hold", PINCH),
        Waypoint(0.6, _pt(PINCH_TCP[0], PINCH_TCP[1], CARD_Z + 0.015), FINGER_CLOSED, "lift it off the lid", PINCH),
        Waypoint(1.4, FLIP_AT, FINGER_CLOSED, "out to the side, clear of the lid", PINCH),
        Waypoint(1.0, FLIP_AT, FINGER_CLOSED, "turn it a quarter", half),
        Waypoint(1.0, FLIP_AT, FINGER_CLOSED, "face down", turned),
        Waypoint(LID_UP_S + 0.1, FLIP_AT, FINGER_CLOSED, "wait for the lid", turned),
        Waypoint(2.0, _pt(drop[0], drop[1], drop[2] + 0.05), FINGER_CLOSED, "over the glass, tipped forward", place),
        Waypoint(1.2, drop, FINGER_CLOSED, "far edge onto the glass", place),
        Waypoint(0.5, drop, RELEASE, "open a little", place),
        Waypoint(1.0, withdrawn, RELEASE, "draw back: the card slides off the lower finger onto the glass", place),
        Waypoint(0.6, _pt(withdrawn[0], withdrawn[1], withdrawn[2] + 0.03), RELEASE, "up", place),
        Waypoint(1.5, over, FINGER_CLOSED, "over the card, pointing down", DOWN),
        Waypoint(0.8, press, FINGER_CLOSED, "fingertips on the card", DOWN),
        Waypoint(1.5, slid_right, FINGER_CLOSED, "slide it to the right guide", DOWN),
        Waypoint(0.8, slid, FINGER_CLOSED, "and forward to the front guide", DOWN),
        Waypoint(0.6, lift, FINGER_CLOSED, "up after the slide", DOWN),
        Waypoint(1.5, PARK_RIGHT, FINGER_OPEN, "clear", DOWN),
        Waypoint(12.0, PARK_RIGHT, FINGER_OPEN, "stay", DOWN),
    ]


# Left hand: lifts the lid from under its front edge, holds it while the card goes in, lowers it, presses Black Copy.
LIFT_Y = 0.06
LIFT_TIPS_X = LID_X0 + 0.011  # under the lid's overhang, short of the body's front face
LIFT_UNDER = float(os.environ.get("COPY_LIFT_UNDER", "0.012"))  # the TCP this far below the lid's underside
LIFT_TCP0 = np.array([LIFT_TIPS_X - PAD_TIP_PAST_TCP, LIFT_Y, TOP + FRAME_Z - LIFT_UNDER])
LID_HOLD_DEG = 65.0
LID_UP_STEP_S = 0.27
LID_UP_S = LID_HOLD_DEG / 5.0 * LID_UP_STEP_S  # the lid goes up in 5-degree steps
HINGE_W = np.array([HINGE[0], HINGE[1], TOP + HINGE[2]])


def on_lid(deg: float) -> tuple:
    """The lifting TCP carried round the hinge with the lid opened ``deg``."""
    return _pt(*(HINGE_W + axis_angle_matrix(Y_AXIS, math.radians(deg)) @ (LIFT_TCP0 - HINGE_W)))


def _left(variant: str, lift_at: float, lower_at: float):
    steps = np.arange(5.0, LID_HOLD_DEG + 0.1, 5.0)
    up = [Waypoint(LID_UP_STEP_S, on_lid(d), FINGER_CLOSED, f"lid up {d:g} degrees", LEFT_LEVEL) for d in steps]
    down = [Waypoint(0.23, on_lid(d), FINGER_CLOSED, f"lid down {d:g} degrees", LEFT_LEVEL)
            for d in list(steps[::-1][1:]) + [0.0]]
    press_top = TOP + PANEL_Z + BUTTON_PROUD
    button = COLOR_XY if variant == "color" else BLACK_XY  # COPY_DEMO=color: a probe pressing Color Copy instead
    over_button = down_tcp(*button, press_top + 0.04)
    pressed = down_tcp(*button, press_top - 0.0022)
    start = _pt(LIFT_TCP0[0] - 0.015, LIFT_Y, LIFT_TCP0[2])
    plan = [
        # WHY from above the printer: started close in front of it, the arm's IK took a branch with its wrist roll at
        # the limit, 2-6 cm off target all the way up the lid's arc (offline check, 2026-11-20).
        Waypoint(2.0, _pt(0.30, 0.10, TOP + 0.30), FINGER_CLOSED, "above the printer, turning to point at it", LEFT_LEVEL),
        Waypoint(1.0, start, FINGER_CLOSED, "just in front of the lid's edge, below it", LEFT_LEVEL),
        Waypoint(lift_at - 3.8, start, FINGER_CLOSED, "wait for the card to come off and away", LEFT_LEVEL),
        Waypoint(0.8, on_lid(0.0), FINGER_CLOSED, "fingers under the lid's edge", LEFT_LEVEL),
        *up,
    ]
    return plan + [
        Waypoint(lower_at - lift_at - LID_UP_S, on_lid(LID_HOLD_DEG), FINGER_CLOSED,
                 "hold it open while the card goes in", LEFT_LEVEL),
        *down,
        # WHY straight out, then straight up: rising and moving in together, the hand caught the lid's edge and lifted
        # it 35 degrees again (official sim, 2026-11-20).
        Waypoint(0.8, _pt(LIFT_TCP0[0] - 0.03, LIFT_Y, LIFT_TCP0[2] - 0.008), FINGER_CLOSED, "out from under", LEFT_LEVEL),
        Waypoint(0.8, _pt(LIFT_TCP0[0] - 0.03, LIFT_Y, TOP + 0.24), FINGER_CLOSED, "straight up, clear of the lid", LEFT_LEVEL),
        Waypoint(1.0, _pt(0.26, 0.15, TOP + 0.30), FINGER_CLOSED, "over the control panel", LEFT_LEVEL),
        Waypoint(1.5, over_button, FINGER_CLOSED, "over the Copy button", DOWN),
        Waypoint(0.6, pressed, FINGER_CLOSED, "press it", DOWN),
        *([  # COPY_DEMO=twice: a probe making two copies. WHY quick: a good first copy ends the trial once it has held
            # for 1 s, so a second press 1.6 s later never happened (official sim, 2026-11-21).
            Waypoint(0.05, pressed, FINGER_CLOSED, "hold", DOWN),
            Waypoint(0.3, _pt(pressed[0], pressed[1], pressed[2] + 0.015), FINGER_CLOSED, "let it up", DOWN),
            Waypoint(0.3, pressed, FINGER_CLOSED, "press it again", DOWN),
        ] if variant == "twice" else []),
        Waypoint(0.4, pressed, FINGER_CLOSED, "hold", DOWN),
        Waypoint(0.6, over_button, FINGER_CLOSED, "let it up", DOWN),
        Waypoint(1.5, PARK_LEFT, FINGER_CLOSED, "clear", DOWN),
        Waypoint(4.0, PARK_LEFT, FINGER_CLOSED, "stay", DOWN),
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
    """Probes: COPY_DEMO=faceup lays the card face up and copies it; COPY_DEMO=color makes the copy with Color Copy;
    COPY_DEMO=twice presses Black Copy twice. Each is a wrong or extra copy and scores 0.

    The left hand's waits come from the right hand's plan: it starts lifting the lid once the card is out to the side,
    and lowers it once the right hand has slid the card into the corner and lifted off it."""
    variant = os.environ.get("COPY_DEMO", "")
    right = _right(variant)
    lift_at = _ends_of(right, "out to the side, clear of the lid") + 0.2
    lower_at = _ends_of(right, "up after the slide") + 0.2
    return {"left": _left(variant, lift_at, lower_at), "right": right}
