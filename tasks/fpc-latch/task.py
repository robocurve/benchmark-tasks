"""On the station table: a helping-hands station holding a bare sensor board up in two alligator clips, lit by its small
LED work light, and a tactile sensor pad with a flat flex cable (FPC) lying on a foam block in front of it.

The board is green, 50 x 72 mm, held level 11.5 cm above the bench, its connector edge towards the robots' right. On its
top at that edge is a flip-lock ZIF connector: a black housing with a slot facing the board's edge, and a brown actuator
hinged along the housing's back edge, with a small upright tab on its front. The actuator lies flat (locked); flipped up
about 70 degrees it is open, and the cable can be slid into the slot. Flipped back down it clamps the cable. Closed, its
clamping rib blocks the slot, so a cable cannot go in.

The sensor pad lies on a grey foam block, 10 cm tall, with its cable lying flat along the block towards the board. Near
the block's end a slot 30 mm wide and 45 mm deep is cut across it; the cable bridges the slot and its stiffened contact
end rests on the short end piece past it, 1.5 cm short of the board. The cable is a 10 mm wide, 115 mm long amber flex
cable: held by one end it stays nearly straight, and it bends easily when pushed; it drags its pad along when pulled.

One hand unclips the connector, the other picks the cable up by its end and slides it in; it holds the cable in while
the first hand clips the actuator shut, then both let go.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
           --instruction "<see task.json>" --scorer task_score ...
"""

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from labkit import Joint, on_reset, on_step, part  # noqa: E402
import yam_arena.demo as _demo  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402
from yam_arena.urdf import axis_angle_matrix  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

FR4 = (0.10, 0.45, 0.20)
BLACK = (0.05, 0.05, 0.06)
ACT_C = (0.45, 0.28, 0.12)  # the actuator's brown plastic
GOLD = (0.90, 0.72, 0.30)
IRON = (0.22, 0.22, 0.24)  # the station's cast iron base
STEEL = (0.70, 0.71, 0.73)  # its rods and the clips
FOAM_C = (0.55, 0.56, 0.58)  # grey foam block
FPC_C = (0.80, 0.50, 0.10)  # polyimide amber
STIFF_C = (0.15, 0.35, 0.80)  # the cable's blue stiffener
LED_C = (0.96, 0.96, 0.94)

# -- the board (fixed, held in the clips). Frame: origin on the board's underside under the connector's centre, x away
# from the robots, y to their left. -----------------------------------------------------------------------------------
HX, HY = 0.30, -0.04  # the connector's centre, world
BOARD_HX, BOARD_Y0, BOARD_Y1, BOARD_T = 0.025, -0.012, 0.060, 0.0016
ZT = BOARD_T
ZB = 0.115  # the board's underside above the bench
BOARD_Z0 = TOP + ZB

# The ZIF connector (heights above the board's top). The slot runs along +y from its mouth at y = -2.5 mm to its back
# at +1.5 mm, 13 mm wide, on the board's top, under a fixed roof 1.4 mm up over its front 2.5 mm. Behind the roof the
# actuator closes over the slot: a bar hinged at y = +2.5 mm, 1.8 mm up, with a clamping rib under it that comes down to
# 0.79 mm (onto a 0.8 mm cable in the slot) and an upright tab, 4 mm tall, on its front edge.
ZW, ZY0, ZY1 = 0.009, -0.0025, 0.0025  # housing: 18 mm wide, 5 mm deep
SLOT_HW, SLOT_BACK = 0.0065, 0.0015
ROOF0, ROOF1, WALL_H = 0.0014, 0.0018, 0.0018
HINGE_Y, HINGE_Z = 0.0025, 0.0018
ACT_T, RIB_Z, TAB_H, TAB_T = 0.0006, 0.00079, 0.004, 0.001
ACT_OPEN = -math.radians(70.0)  # the actuator flipped up (negative: its front edge rises), short of upright
DETENT = -math.radians(30.0)  # past this it snaps open, short of it it springs shut
ACT_K, ACT_D = 0.002, 0.00005  # N m/rad: a light detent, about 0.4 N at the tab

ACTUATOR = Joint(
    "actuator", kind="hinge", axis="X", anchor=(0.0, HINGE_Y, ZT + HINGE_Z), lower=ACT_OPEN - 0.1, upper=0.02,
    initial=0.0, stiffness=ACT_K, damping=ACT_D, target=0.0, mass=0.0005,
    boxes=[
        ((0.0, HINGE_Y / 2, ZT + HINGE_Z + ACT_T / 2), (2 * ZW - 0.001, HINGE_Y, ACT_T), ACT_C),  # the bar
        ((0.0, (SLOT_BACK + 0.0003) / 2, ZT + (RIB_Z + HINGE_Z) / 2), (2 * SLOT_HW - 0.0004, SLOT_BACK - 0.0003,
                                                                      HINGE_Z - RIB_Z), ACT_C),  # the clamping rib
        ((0.0, TAB_T / 2, ZT + HINGE_Z + ACT_T + TAB_H / 2), (0.010, TAB_T, TAB_H), ACT_C),  # the tab
    ],
)

_ys = (ZY0 + ZY1) / 2
BOARD = part(
    "sensor_board", pos=(HX, HY, BOARD_Z0), fixed=True, friction=0.7,
    boxes=[
        ((0.0, (BOARD_Y0 + BOARD_Y1) / 2, BOARD_T / 2), (2 * BOARD_HX, BOARD_Y1 - BOARD_Y0, BOARD_T), FR4),
        ((-0.008, 0.032, ZT + 0.0005), (0.008, 0.008, 0.001), BLACK),  # a chip
        # the ZIF housing: side walls, back wall under the hinge, and the fixed roof over the slot's mouth
        *[((s * (SLOT_HW + ZW) / 2, _ys, ZT + WALL_H / 2), (ZW - SLOT_HW, ZY1 - ZY0, WALL_H), BLACK) for s in (1, -1)],
        ((0.0, (SLOT_BACK + ZY1) / 2, ZT + WALL_H / 2), (2 * SLOT_HW, ZY1 - SLOT_BACK, WALL_H), BLACK),
        ((0.0, ZY0 / 2, ZT + (ROOF0 + ROOF1) / 2), (2 * ZW, -ZY0, ROOF1 - ROOF0), BLACK),
        # the contacts' gold tails behind the housing
        *[((x, ZY1 + 0.0008, ZT + 0.0001), (0.0003, 0.0016, 0.0002), GOLD) for x in np.linspace(-0.0045, 0.0045, 10)],
    ],
    joints=[ACTUATOR],
)

# -- the helping-hands station (fixed): a cast iron base behind the board, two steel rods ending in alligator clips on
# the board (one on its far end, one on its far edge) and a small LED work light on a third rod, 5 cm above the board
# and 9 cm beyond the connector, shining back at it under the hands. Same frame as the board's, origin on the bench. -----
_zb, _zt = ZB, ZB + BOARD_T
_jaw = 0.002  # jaw thickness


def _clip(cx, cy, along_x):
    """An alligator clip's two jaws on the board's faces at (cx, cy), and its body just past the board's edge."""
    jx, jy = (0.012, 0.008) if along_x else (0.008, 0.012)
    bx, by = (cx + 0.010, cy) if along_x else (cx, cy + 0.010)
    return [((cx, cy, _zt + _jaw / 2), (jx, jy, _jaw), STEEL), ((cx, cy, _zb - _jaw / 2), (jx, jy, _jaw), STEEL),
            ((bx, by, (_zb + _zt) / 2), (0.008, 0.008, BOARD_T + 2 * _jaw), STEEL)]


_RZ = _zt + 0.025  # the clip rods' height
_PX = 0.13  # the posts' x
_HEAD = (0.095, 0.0, _zt + 0.050)  # the LED light head's centre
_rod = 0.006
STATION = part(
    "helping_hands", pos=(HX, HY, TOP), fixed=True, friction=0.6,
    boxes=[
        ((0.10, 0.03, 0.0075), (0.10, 0.14, 0.015), IRON),  # the base
        ((_PX, 0.072, (0.015 + _RZ) / 2), (0.008, 0.008, _RZ - 0.015), STEEL),  # post for the clips
        # rod 1, to the clip on the board's far end (y = 60 mm)
        ((_PX / 2, 0.072, _RZ), (_PX, _rod, _rod), STEEL),
        ((0.0, 0.072, (_zt + _RZ) / 2 + _jaw), (_rod, _rod, _RZ - _zt), STEEL),
        *_clip(0.0, 0.056, False),
        # rod 2, to the clip on the board's far edge (x = 25 mm)
        ((_PX, (0.072 + 0.025) / 2, _RZ), (_rod, 0.047, _rod), STEEL),
        ((((0.034 + _PX) / 2), 0.025, _RZ), (_PX - 0.034, _rod, _rod), STEEL),
        ((0.034, 0.025, (_zt + _RZ) / 2 + _jaw), (_rod, _rod, _RZ - _zt), STEEL),
        *_clip(0.021, 0.025, True),
        # the LED light: a post, a rod and a small head with its LED panel facing the connector
        ((_PX + 0.015, -0.010, (0.015 + _HEAD[2]) / 2), (0.008, 0.008, _HEAD[2] - 0.015), STEEL),
        ((_HEAD[0] + 0.035, -0.010, _HEAD[2]), (0.035, _rod, _rod), STEEL),
        ((_HEAD[0] + 0.008, 0.0, _HEAD[2]), (0.016, 0.040, 0.020), BLACK),
        ((_HEAD[0] - 0.0005, 0.0, _HEAD[2]), (0.001, 0.034, 0.014), LED_C),
    ],
)
LIGHT_POS = (HX + _HEAD[0] - 0.012, HY + _HEAD[1], TOP + _HEAD[2])  # the LED light, just in front of its panel
LIGHT_INTENSITY, LIGHT_RADIUS = 4.0e5, 0.02

# -- the cable (loose): a tactile sensor pad with its flat flex cable. The pad is the main body; the cable leaves its front
# edge as a chain of short segments, each on a bend hinge (about x) with a light spring, ending in a rigid stiffened
# contact end. Frame: origin at the middle of the pad's front edge, on the cable's underside; the cable runs along +y
# (all hinges at 0). It lies flat on a foam block in line with the slot, its end sticking out past the block's end.
FPC_HW, FPC_T = 0.005, 0.0008
FPC_STIFF = 0.015  # the stiffened contact end, rigid
SEG, N_SEG = 0.005, 20  # 20 segments of 5 mm: the cable is 115 mm long
SEG_K, SEG_D = 0.4, 0.002  # N m/rad per hinge (as the solver treats it): held by one end, it stays nearly straight
EDGE = 0.002  # the first hinge, 2 mm past the pad's front edge
PAD_HW, PAD_L, PAD_T = 0.009, 0.015, 0.004  # the sensor pad: 18 x 15 x 4 mm
PAD_MASS = 0.010
SKIN_C = (0.12, 0.12, 0.13)
CABLE_L = SEG * N_SEG + FPC_STIFF
OVERHANG = 0.044  # the cable's end lies this far past the slot's near edge, on the block's short end piece
GAP = 0.015  # the block's end stops this far short of the board's edge
BLOCK_H, BLOCK_L, BLOCK_W = 0.100, 0.12, 0.04
SLOT_W, SLOT_D, END_L = 0.030, 0.045, 0.018  # the slot cut across the block under the cable, and the end piece past it
BLOCK_END = HY + BOARD_Y0 - GAP - END_L - SLOT_W  # the slot's near edge, world y
_hinge_y = [EDGE + SEG * k for k in range(N_SEG + 1)]  # hinge k at the front end of link k; link N_SEG is the tip
_links = []
for _k in range(N_SEG + 1):
    _y0 = _hinge_y[_k]
    if _k < N_SEG:
        _bx = [((0.0, _y0 + SEG / 2, FPC_T / 2), (2 * FPC_HW, SEG - 0.0006, FPC_T), FPC_C)]
        _m = 0.001
    else:  # the stiffened contact end
        _end = _y0 + FPC_STIFF
        _bx = [((0.0, _y0 + FPC_STIFF / 2 + 0.0002, FPC_T / 2), (2 * FPC_HW, FPC_STIFF - 0.0004, FPC_T), FPC_C),
               ((0.0, _y0 + FPC_STIFF / 2, FPC_T + 0.0001), (2 * FPC_HW, FPC_STIFF - 0.006, 0.0002), STIFF_C),
               *[((x, _end - 0.0015, FPC_T - 0.00005), (0.0004, 0.003, 0.00012), GOLD)
                 for x in np.linspace(-0.0045, 0.0045, 10)]]
        _m = 0.001
    _links.append(Joint(f"bend{_k}", kind="hinge", axis="X", anchor=(0.0, _y0, FPC_T / 2), lower=-1.7, upper=1.7,
                        initial=0.0, target=0.0, stiffness=SEG_K, damping=SEG_D, mass=_m, boxes=_bx,
                        parent=f"bend{_k - 1}" if _k else ""))
PINCH_D = -0.029  # the hand holds the cable this far behind its contact end, over the slot
FPC = part(
    "fpc", pos=(HX, BLOCK_END + OVERHANG - CABLE_L - EDGE, TOP + BLOCK_H), mass=PAD_MASS, friction=0.9,
    boxes=[((0.0, -PAD_L / 2, PAD_T / 2), (2 * PAD_HW, PAD_L, PAD_T), FR4),
           ((0.0, -PAD_L / 2, PAD_T + 0.0005), (2 * PAD_HW - 0.002, PAD_L - 0.002, 0.001), SKIN_C)],
    joints=_links,
)
BLOCK = part("foam_block", pos=(HX, BLOCK_END, TOP), fixed=True, friction=0.6,
             boxes=[((0.0, -BLOCK_L / 2, BLOCK_H / 2), (BLOCK_W, BLOCK_L, BLOCK_H), FOAM_C),
                    ((0.0, SLOT_W + END_L / 2, BLOCK_H / 2), (BLOCK_W, END_L, BLOCK_H), FOAM_C),
                    ((0.0, SLOT_W / 2, (BLOCK_H - SLOT_D) / 2), (BLOCK_W, SLOT_W, BLOCK_H - SLOT_D), FOAM_C)])


def cable_tip(fpc):
    """The cable's contact end, in world: (its end's middle on its underside, rotation whose y axis points out of the
    end and z axis up from its top), from the pad's pose and the bend hinges' angles."""
    rot = fpc.rotation()
    p = np.array([0.0, _hinge_y[0], FPC_T / 2])
    r = np.eye(3)
    for k in range(N_SEG + 1):
        r = r @ axis_angle_matrix(np.array([1.0, 0.0, 0.0]), float(fpc.joints[f"bend{k}"]))
        length = SEG if k < N_SEG else FPC_STIFF
        p = p + r @ np.array([0.0, length, 0.0])
    p = p - r @ np.array([0.0, 0.0, FPC_T / 2])
    return np.asarray(fpc.to_world(p), float), rot @ r

OBJECTS = [BOARD, STATION, FPC, BLOCK]

# -- the cable in the connector -------------------------------------------------------------------------------------------
IN_X = SLOT_HW - FPC_HW  # inserted: its sides inside the slot's walls (1.5 mm of play either way),
IN_YAW = math.radians(10.0)  # turned no more than 10 degrees from it,
IN_DEPTH = SLOT_BACK - 0.0005  # its end within 0.5 mm of the slot's back,
IN_Z = 0.0006  # and its underside within 0.6 mm of the board's top
ENTER = ZY0 + 0.002  # its end 2 mm into the slot counts as entering
CLOSED = math.radians(6.0)
OPENED = math.radians(50.0)
RELEASED = 0.030


def _wrap(a: float) -> float:
    return float((a + math.pi) % (2 * math.pi) - math.pi)


def _yaw(rot) -> float:
    return float(math.atan2(rot[1, 0], rot[0, 0]))


def cable_in_slot(board, fpc):
    """(the cable end's x and y in the board's frame, its underside's height above the board's top, its turn from the
    slot's axis in rad)."""
    end, rot = cable_tip(fpc)
    p = board.to_local(end)
    rel = board.rotation().T @ rot
    return float(p[0]), float(p[1]), float(p[2] - ZT), _wrap(_yaw(rel))


def inserted(board, fpc) -> bool:
    x, y, z, turn = cable_in_slot(board, fpc)
    return abs(x) <= IN_X and y >= IN_DEPTH and -0.0003 <= z <= IN_Z and abs(turn) <= IN_YAW


# -- the actuator's detent and the trial's record ------------------------------------------------------------------------
_state = {"opened": False, "forced": False, "hands_far": True, "lamp": False, "was_in": False}


@on_reset
def _reset_state(env):
    _state.update(opened=False, forced=False, hands_far=True, was_in=False)
    if not _state["lamp"]:
        _add_lamp(env)
    _cable_weightless(env)


def _cable_weightless(env):
    """The cable's own weight (a few grams over 11 cm) is left out: gravity acts on the sensor pad only, so the cable's
    springs hold it straight, as a real cable held by one end stays nearly straight."""
    try:
        import torch

        view = env.unwrapped.scene["fpc"].root_physx_view
        flags = view.get_disable_gravities()
        flags = flags.clone() if hasattr(flags, "clone") else torch.as_tensor(flags)
        flags[:, 1:] = 1
        view.set_disable_gravities(flags, torch.arange(flags.shape[0], device=flags.device))
    except Exception as e:
        print("cable gravity not changed:", e)


def _add_lamp(env):
    """The station's LED work light: a small sphere light in front of its panel."""
    try:
        import omni.usd
        from pxr import Gf, UsdGeom, UsdLux

        stage = omni.usd.get_context().get_stage()
        o = env.unwrapped.scene.env_origins[0].cpu().numpy()
        light = UsdLux.SphereLight.Define(stage, "/World/task_led_light")
        light.CreateIntensityAttr(LIGHT_INTENSITY)
        light.CreateRadiusAttr(LIGHT_RADIUS)
        light.CreateColorAttr(Gf.Vec3f(1.0, 0.98, 0.95))
        UsdGeom.XformCommonAPI(light.GetPrim()).SetTranslate(Gf.Vec3d(*(np.asarray(LIGHT_POS) + o).tolist()))
        _state["lamp"] = True
    except Exception as e:  # rendering only: never fail a trial over the light
        print("LED light not added:", e)


@on_step
def _watch(env):
    """The actuator's detent (snaps open past 30 degrees, springs shut short of it); whether it has been opened; whether
    the cable was pushed into the slot while it was shut (forced); whether both hands are clear of the connector."""
    import torch

    import labkit
    from yam_arena import runtime as _rt

    board = labkit._part_state(env, labkit._SPECS["sensor_board"])
    fpc = labkit._part_state(env, labkit._SPECS["fpc"])
    a = board.joints["actuator"]
    if -a >= OPENED:
        _state["opened"] = True
    x, y, z, turn = cable_in_slot(board, fpc)
    now_in = y >= ENTER and abs(x) <= SLOT_HW and z <= ROOF0
    if now_in and not _state["was_in"] and -a < -DETENT:  # it entered the slot while the actuator was shut
        _state["forced"] = True
    _state["was_in"] = now_in
    b = env.unwrapped
    o = b.scene.env_origins[0].cpu().numpy()
    zif = board.to_world((0.0, 0.0, ZT))
    _state["hands_far"] = all(float(np.linalg.norm(_rt.tcp_world(b.scene[n]) - o - zif)) >= RELEASED
                              for n in ("left_robot", "right_robot"))
    asset = b.scene["sensor_board"]
    ids = [list(asset.joint_names).index(ACTUATOR.joint_name)]
    target = ACT_OPEN if a <= DETENT else 0.0
    asset.set_joint_position_target(torch.tensor([[target]], device=asset.device), joint_ids=ids)
    cable = b.scene["fpc"]  # the cable's bend springs pull it straight
    cable.set_joint_position_target(torch.zeros((1, N_SEG + 1), device=cable.device))


# -- scoring -----------------------------------------------------------------------------------------------------------
SUCCESS_HOLD_S = 10.0


def score(objects):
    """0 if the cable was ever pushed 2 mm into the slot while the actuator was shut (forced). Otherwise: 1.0 if the cable
    is fully inserted (its end within 0.5 mm of the slot's back, its sides inside the slot's walls, turned no more than
    10 degrees, lying on the slot's floor), the actuator is shut (within 6 degrees of flat) and both hands are at least 3 cm
    from the connector; 0.5 if the cable is fully inserted but the actuator is not shut or a hand is still at the
    connector; 0.25 if the actuator has been flipped open (50 degrees or more) at some point but the cable is not fully
    inserted; 0 otherwise."""
    board, fpc = objects["sensor_board"], objects["fpc"]
    if _state["forced"]:
        return 0.0
    if inserted(board, fpc):
        shut = abs(board.joints["actuator"]) <= CLOSED
        return 1.0 if shut and _state["hands_far"] else 0.5
    return 0.25 if _state["opened"] else 0.0


def success(objects):
    return score(objects) == 1.0


def distance(objects):
    x, y, _, _ = cable_in_slot(objects["sensor_board"], objects["fpc"])
    return abs(x) + max(0.0, SLOT_BACK - y) + abs(objects["sensor_board"].joints["actuator"])


# -- scripted demo -----------------------------------------------------------------------------------------------------
# Left arm, fingers closed, pointing down: pushes the actuator's tab back until it flips open, then waits above and
# behind the connector. Right arm, level, pointing away from the robots, fingers closing up and down: slides its lower
# finger under the cable's overhanging end 25 mm behind its contact end and pinches it; lifts it, carries it level to
# the slot and slides it straight in to the back, the trim across the slot from the cable's offset as the wrist camera
# sees it; holds it there. Left arm: pushes the open actuator forward from behind until it snaps shut on the cable, and
# lifts off. Right arm: lets go and slides out. Both park.
TIP_ABOVE_TCP = 0.013
LEFT_PARK = (0.15, 0.25, TOP + 0.14)
RIGHT_PARK = (0.15, -0.25, TOP + 0.14)
FPC_WIDE, FPC_HOLD = -0.006, FINGER_CLOSED  # pads 12 mm apart round the flat cable, then closed on it
PICK_WIDE = -0.012  # pads 24 mm apart, one either side of the cable over the slot, to pick it up
PICK_SAG = 0.010  # the arm sits about 1 cm low at this reach, low and close in; aim that much higher
CABLE_IN = 0.007  # the finger tips this far in from the cable's near edge (it is 10 mm wide)
FLAT_YAW = math.pi / 2  # fingers closing along x

_orig_grip = _demo.grip_rotation
_down = getattr(_orig_grip, "_base", _orig_grip)
if not getattr(_orig_grip, "_rot_duck", False):
    def _grip_rotation(yaw=0.0):
        return yaw.rot if hasattr(yaw, "rot") else _down(yaw)

    _grip_rotation._rot_duck = True
    _grip_rotation._base = _down
    _demo.grip_rotation = _grip_rotation


class Orient(float):
    def __new__(cls, rot):
        obj = float.__new__(cls, 0.0)
        obj.rot = np.asarray(rot, float)
        return obj


POINT_X = Orient(axis_angle_matrix(np.array([0.0, 1.0, 0.0]), -math.pi / 2) @ _down(math.pi / 2))


class _PointXOnBoard(float):
    """Level, pointing away from the robots (turned with the board), fingers closing up and down."""

    def __new__(cls):
        return float.__new__(cls, 0.0)

    @property
    def rot(self):
        return axis_angle_matrix(np.array([0.0, 0.0, 1.0]), _seen["psi"]) @ POINT_X.rot

_prev_action7 = _demo.ArmPlan.action7
if not getattr(_prev_action7, "_clock", False):
    def _action7(self, t, _prev=_prev_action7):
        _demo._plan_clock = float(t)
        return _prev(self, t)

    _action7._clock = True
    _demo.ArmPlan.action7 = _action7

# What the demo sees: the board's pose and, once the cable is near the slot, the cable's offset across it (the wrist
# camera sees both), read each step. From lining up until the cable's end is in the slot's mouth, the hand's aim across
# the slot is trimmed by that offset; then the trim is frozen. While it pushes the cable in, the hand stops where it is
# once the cable's end is seen at the slot's back.
_seen = {"zif": None, "psi": 0.0}
_trim = {"x": 0.0, "t0": 1e9, "t1": 1e9, "y": 0.0, "p0": 1e9, "p1": 1e9, "yin": 0.0}
TRIM_GAIN, TRIM_STEP, TRIM_MAX = 0.15, 0.0003, 0.003


def _clock() -> float:
    return float(getattr(_demo, "_plan_clock", 0.0))


@on_reset
def _reset_seen(env):
    _seen.update(zif=None, psi=0.0)
    _trim.update(x=0.0, y=0.0)


@on_step
def _look(env):
    import labkit

    board = labkit._part_state(env, labkit._SPECS["sensor_board"])
    _seen.update(zif=np.asarray(board.to_world((0.0, 0.0, ZT)), float), psi=_yaw(board.rotation()))
    if _trim["t0"] <= _clock() < _trim["t1"]:
        fx, fy = cable_in_slot(board, labkit._part_state(env, labkit._SPECS["fpc"]))[:2]
        if fy >= ZY0 + 0.0015:  # its end is in the slot's mouth: hold the aim from here on
            _trim["t1"] = _clock()
        step = float(np.clip(TRIM_GAIN * fx, -TRIM_STEP, TRIM_STEP))
        _trim["x"] = float(np.clip(_trim["x"] - step, -TRIM_MAX, TRIM_MAX))
    if _trim["p0"] <= _clock() < _trim["p1"]:
        fy = cable_in_slot(board, labkit._part_state(env, labkit._SPECS["fpc"]))[1]
        if fy >= SLOT_BACK - 0.00015:  # seen at the slot's back: stop pushing, hold the hand where it is
            from yam_arena import runtime as _rt

            b = env.unwrapped
            hand = _rt.tcp_world(b.scene["right_robot"]) - b.scene.env_origins[0].cpu().numpy()
            _trim["y"] = float(board.to_local(hand)[1]) - _trim["yin"]
            _trim["p1"] = _clock()


ZIF0 = np.array([HX, HY, BOARD_Z0 + BOARD_T])  # the connector's base (nominal)


class _AtZif:
    """A TCP point given in the connector's frame (x, y along the board, z above the board's top), where the board is."""

    def __init__(self, x, y, z, trim=False):
        self.p = np.array([x, y, z])
        self.trim = trim

    def __array__(self, dtype=None, copy=None):
        base = _seen["zif"] if _seen["zif"] is not None else ZIF0
        c, s = math.cos(_seen["psi"]), math.sin(_seen["psi"])
        x, y, z = self.p
        if self.trim:
            x += _trim["x"]
            y += _trim["y"] if y == _trim["yin"] else 0.0
        return np.asarray((base[0] + c * x - s * y, base[1] + s * x + c * y, base[2] + z), dtype=dtype or float)


class _ZifYaw(float):
    """Pointing down, turned with the board; fingers closing across the slot (FLAT_YAW) or along it (0)."""

    def __new__(cls, yaw=FLAT_YAW):
        obj = float.__new__(cls, yaw)
        obj.yaw = yaw
        return obj

    @property
    def rot(self):
        return _down(self.yaw + _seen["psi"])


def _dur(wps):
    return sum(w.duration for w in wps)


def _wait_until(wps, t, pos, finger, label, orient):
    wps.append(Waypoint(max(0.3, t - _dur(wps)), pos, finger, label, orient))


def demo():
    _seen.update(zif=None, psi=0.0)
    _trim.update(x=0.0, y=0.0)
    yaw = _ZifYaw()
    shut = FINGER_CLOSED
    tip = TIP_ABOVE_TCP
    # Left 1. unclip: closed fingertips in front of the tab, 2.5 mm up it, push it back until it flips open
    tz = HINGE_Z + ACT_T + 0.0025 + tip
    behind_hi = _AtZif(0.0, 0.016, 0.06)
    left = [
        Waypoint(3.0, _AtZif(0.0, -0.006, 0.04), shut, "fingers closed, above the actuator's tab", yaw),
        Waypoint(1.5, _AtZif(0.0, -0.006, tz), shut, "fingertips down in front of the tab", yaw),
        Waypoint(0.7, _AtZif(0.0, -0.006, tz), shut, "settle", yaw),
        Waypoint(2.5, _AtZif(0.0, 0.0035, tz), shut, "push the tab back until the actuator flips open", yaw),
        Waypoint(1.0, _AtZif(0.0, 0.0035, 0.04), shut, "lift off", yaw),
        Waypoint(1.5, behind_hi, shut, "wait above and behind the connector", yaw),
    ]
    t_open = _dur(left)
    # Right 1. pick the cable up by its overhanging end
    ex = -FPC_HW + CABLE_IN - tip  # the TCP across the cable, from its centre line: the finger tips CABLE_IN in
    px = HX + ex
    py = BLOCK_END + OVERHANG + PINCH_D
    pz = TOP + BLOCK_H + FPC_T / 2 + PICK_SAG
    right = [
        Waypoint(3.0, (px - 0.05, py, pz + 0.02), PICK_WIDE, "level, beside the cable where it crosses the slot",
                 POINT_X),
        Waypoint(1.0, (px - 0.05, py, pz), PICK_WIDE, "fingers either side of it", POINT_X),
        Waypoint(1.5, (px, py, pz), PICK_WIDE, "slide in from its near edge, lower finger in the slot under it", POINT_X),
        Waypoint(0.5, (px, py, pz), PICK_WIDE, "settle", POINT_X),
        Waypoint(1.0, (px, py, pz), FPC_HOLD, "pinch the cable", POINT_X),
        Waypoint(1.5, (px, py, pz + 0.025), FPC_HOLD, "lift it", POINT_X),
    ]
    _wait_until(right, t_open, (px, py, pz + 0.025), FPC_HOLD, "wait for the connector to open", POINT_X)
    # Right 2. insert: level, its end laid on the board 5.5 mm in front of the slot's mouth, then straight in
    flat = _PointXOnBoard()
    cz = 0.0005 + FPC_T / 2  # the cable's underside 0.5 mm above the board's top: in the slot's mouth
    y0 = ZY0 - 0.0055 + PINCH_D  # its end over the board, 5.5 mm in front of the slot's mouth
    right += [
        Waypoint(2.5, _AtZif(ex, y0, 0.03), FPC_HOLD, "carry it over the board, in line with the slot", flat),
        Waypoint(1.5, _AtZif(ex, y0, cz), FPC_HOLD, "lay its end down on the board in front of the slot", flat),
    ]
    _trim["t0"] = _dur(right)
    right.append(Waypoint(2.0, _AtZif(ex, y0, cz, True), FPC_HOLD, "line it up across the slot", flat))
    yin = SLOT_BACK + 0.0005 + PINCH_D
    _trim["yin"] = yin
    _trim["t1"] = _dur(right) + 3.0  # through the push, until the cable's end is in the slot's mouth
    _trim["p0"] = _dur(right)
    right += [
        Waypoint(3.0, _AtZif(ex, yin, cz, True), FPC_HOLD, "slide it straight in to the back", flat),
        Waypoint(0.5, _AtZif(ex, yin, cz, True), FPC_HOLD, "hold it in", flat),
    ]
    t_in = _dur(right)
    _trim["p1"] = t_in
    # Left 2. clip: turned so the closed fingertips are thin along the slot (8 mm), behind the open actuator, push its tab
    # forward until it snaps shut on the cable, the fingertips stopping 1 mm behind the shut tab
    bz = HINGE_Z + 0.0017 + tip  # the fingertips 3.5 mm above the board: under the open tab's end, over the bar
    along = _ZifYaw(0.0)
    _wait_until(left, t_in, behind_hi, shut, "wait for the cable to be in", yaw)
    left += [
        Waypoint(1.0, _AtZif(0.0, 0.016, 0.03), shut, "turn the fingers along the slot", along),
        Waypoint(1.5, _AtZif(0.0, 0.016, bz), shut, "fingertips down behind the open actuator", along),
        Waypoint(3.0, _AtZif(0.0, 0.005, bz), shut, "push its tab forward until it snaps shut", along),
        Waypoint(0.5, _AtZif(0.0, 0.005, bz), shut, "settle", along),
        Waypoint(1.0, _AtZif(0.0, 0.005, 0.04), shut, "lift off", along),
        Waypoint(2.0, LEFT_PARK, FINGER_OPEN, "park"),
    ]
    t_shut = _dur(left) - 2.0
    _wait_until(right, t_shut, _AtZif(ex, yin, cz, True), FPC_HOLD, "hold the cable in while it is clipped", flat)
    right += [
        Waypoint(0.8, _AtZif(ex, yin, cz, True), PICK_WIDE, "let go", flat),
        Waypoint(1.5, _AtZif(ex - 0.04, yin, cz, True), PICK_WIDE, "slide the fingers out from the cable", flat),
        Waypoint(1.0, _AtZif(ex - 0.04, yin, 0.04, True), PICK_WIDE, "up", flat),
        Waypoint(2.0, RIGHT_PARK, FINGER_OPEN, "park"),
    ]
    t_end = max(_dur(left), _dur(right)) + 1.0
    left.append(Waypoint(t_end - _dur(left), LEFT_PARK, FINGER_OPEN, "hold"))
    right.append(Waypoint(t_end - _dur(right), RIGHT_PARK, FINGER_OPEN, "hold"))
    return {"left": left, "right": right}
