"""Pick up litter with a reacher-grabber. A 61 cm aluminium pick-up tool (trigger-grip litter picker) rests on the
table on its grip and its open jaws. Three pieces of litter lie at the far side of the table, at the far edge of the arms' reach:
a crushed drinks can, an empty 500 mL PET bottle on its side and a paper cup standing up. An open plastic tote, the bin,
stands on the right. The litter has to go into the bin, picked up with the tool.

The tool's trigger is a spring-returned slider under the handle, in front of the grip. Pulling it back toward the grip
closes the two jaws at the far end; letting it go opens them. One gripper holds the handle and the other works the
trigger, so the litter stays gripped while the tool is lifted and swung over the bin.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score ...
Demo:    python demo.py task.py demo_out
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from litterkit import PartState, Slider, gripper_poses, on_reset, on_step, part, set_slider_targets  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = "Put the litter in the bin using the pick-up tool."

BLACK = (0.10, 0.10, 0.11)
ALU = (0.75, 0.76, 0.78)
YELLOW = (0.98, 0.78, 0.08)
JAW = (0.20, 0.21, 0.23)
GREY = (0.45, 0.46, 0.48)


def _mm(x0, x1, y0, y1, z0, z1, color):
    """A box from corner coordinates in mm: (centre, size, colour) in m."""
    return (((x0 + x1) / 2000, (y0 + y1) / 2000, (z0 + z1) / 2000), ((x1 - x0) / 1000, (y1 - y0) / 1000,
                                                                      (z1 - z0) / 1000), color)


# -- the pick-up tool, in its own frame: origin on the shaft's axis 3 cm in front of the back of the handle, x along the
# shaft toward the jaws, z up when the grip hangs down. A 24-inch trigger-grip litter picker: 61 cm long, 0.22 kg (0.48 lb),
# aluminium shaft, jaws opening 13 cm wide.
TOOL_LEN = 0.610                    # back of the handle (x -0.03) to the jaw tips (x 0.58)
SHAFT_R = 0.0095                    # 19 mm aluminium tube
TRIG_TRAVEL = 0.028                 # the trigger's stroke, from rest (jaws open) to its stop at the grip (jaws shut)
JAW_TRAVEL = 0.062                  # each jaw's stroke: inner faces 130 mm apart at rest, 6 mm apart fully shut
JAW_OPEN_Y = 0.065                  # a jaw's inner face, open
JAW_RATE = 300.0                    # N/m: the jaws' pull toward the closure the trigger asks for (cable and spring)
TRIG_RATE = 120.0                   # N/m: the trigger's return spring (3.4 N at its stop)
TOOL_MASS = 0.16                    # handle, shaft and jaw head; with the trigger and jaws, 0.22 kg in all
TOOL_COM = (0.13, 0.0, -0.01)       # their centre of mass: the moulded handle outweighs the thin aluminium tube
GRIP_X = -0.010                     # where a gripper holds the handle (its pads across the housing, behind the grip)
GRIP_PAD_Z = -0.027                 # the pad tips there, 3 mm above the housing's lower edge
TRIGGER_X = 0.110                   # the trigger's centre at rest, where a gripper holds it
TRIGGER_PAD_Z = -0.029              # pad tips there: the pads hold its upper 17 mm, just under the shaft
CLAMP = np.array([0.530, 0.0, -0.0225])    # the middle of the jaws (where litter is held)
GRIP_FOOT = (0.065, -0.125)         # (x, z) of the grip's front lower corner: one of its resting points
JAW_FOOT = (0.580, -0.050)          # (x, z) of the jaws' lower front corners: the other


def _tool_boxes():
    return [
        _mm(-30, 35, -15, 15, -30, 15, BLACK),        # handle housing (moulded plastic), 30 mm wide, 45 mm tall
        _mm(35, 65, -15, 15, -125, -10, BLACK),       # the grip, hanging from the shaft in front of the housing
        _mm(460, 480, -20, 20, -20, 15, BLACK),       # jaw head at the end of the shaft
    ]


def _tool_cylinders():
    return [((0.2625, 0.0, 0.0), SHAFT_R, 0.395, ALU, "X")]   # shaft, x 65..460 mm


def _trigger():
    """The trigger: a 30 x 24 x 93 mm yellow block under the shaft, in front of the grip, on a slider along the tool.
    Pulled back 28 mm (to 2 mm short of the grip) it closes the jaws; a 120 N/m spring returns it."""
    block = _mm(95, 125, -12, 12, -105, -12, YELLOW)
    return Slider("trigger", rest=(TRIGGER_X, 0.0, -0.060), boxes=(block,), travel=TRIG_TRAVEL, rate=TRIG_RATE,
                  damping=5.0, axis="X", sign=-1, mass=0.03)


def _jaw(name, side):
    """One jaw: a 100 x 6 x 55 mm rubber-faced plate reaching forward of the jaw head, and a bar over the head, on a
    slider across the tool (y). At rest its inner face stands 65 mm off the centreline; it closes by up to 62 mm."""
    s = side
    y0, y1 = sorted((s * JAW_OPEN_Y * 1000, s * (JAW_OPEN_Y + 0.006) * 1000))
    plate = _mm(480, 580, y0, y1, -50, 5, JAW)
    b0, b1 = sorted((s * 20, s * (JAW_OPEN_Y + 0.006) * 1000))
    bar = _mm(462, 478, b0, b1, 16, 23, JAW)
    return Slider(name, rest=(0.530, s * 0.068, -0.020), boxes=(plate, bar), travel=JAW_TRAVEL, rate=JAW_RATE,
                  damping=4.0, axis="Y", sign=-s, mass=0.015)


REST_PITCH = math.atan2(JAW_FOOT[1] - GRIP_FOOT[1], JAW_FOOT[0] - GRIP_FOOT[0])   # 8.3 degrees, jaws down
TOOL_XY = (0.120, 0.0)              # the tool frame's origin, between the arms ...
TOOL_YAW = math.radians(-8)         # ... the tool pointing ahead, 8 degrees to the right, its jaws clear of the litter
TOOL_Z = TOP + 0.001 + GRIP_FOOT[0] * math.sin(REST_PITCH) - GRIP_FOOT[1] * math.cos(REST_PITCH)

TOOL = part("picker", pos=(*TOOL_XY, TOOL_Z), mass=TOOL_MASS, boxes=_tool_boxes(), cylinders=_tool_cylinders(),
            sliders=(_trigger(), _jaw("jaw_l", 1), _jaw("jaw_r", -1)), color=BLACK, friction=1.0, pitch=REST_PITCH,
            yaw=TOOL_YAW, com=TOOL_COM)

# -- the bin: an open plastic storage tote (a 16-quart storage box without its lid), 42.5 x 30.2 cm and 17.8 cm tall,
# 3 mm polypropylene walls, held in place. Origin at the centre of its floor.
BIN_L, BIN_W, BIN_H, WALL = 0.425, 0.302, 0.178, 0.003
BIN_XY = (0.585, -0.390)
_L, _W, _H, _T = BIN_L * 500, BIN_W * 500, BIN_H * 1000, WALL * 1000
BIN = part("bin", pos=(*BIN_XY, TOP), mass=None, fixed=True, color=GREY, friction=0.5, boxes=[
    _mm(-_L, _L, -_W, _W, 0, _T, GREY),
    _mm(-_L, -_L + _T, -_W, _W, 0, _H, GREY), _mm(_L - _T, _L, -_W, _W, 0, _H, GREY),
    _mm(-_L + _T, _L - _T, -_W, -_W + _T, 0, _H, GREY), _mm(-_L + _T, _L - _T, _W - _T, _W, 0, _H, GREY)])

# -- the litter, each with its origin at its centre.
def _cyl_part(name, xy, z, mass, color, cyls, yaw=0.0, boxes=()):
    """A rigid piece of litter from cylinders, with a small box hidden inside (the part builder needs one)."""
    inner = ((0.0, 0.0, 0.0), (0.02, 0.02, 0.02), color)
    return part(name, pos=(*xy, TOP + z), mass=mass, yaw=yaw, color=color, friction=0.5, boxes=[inner, *boxes],
                cylinders=[(c[0], c[1], c[2], color, *c[3:]) for c in cyls])


RED, CLEAR, WHITE = (0.80, 0.10, 0.12), (0.62, 0.80, 0.88), (0.95, 0.94, 0.90)
CAN_H, CAN_R = 0.035, 0.035                # a 355 mL can (66 mm, 122 mm tall) stamped flat end-on: 70 mm across, 35 mm
CAN = _cyl_part("can", (0.765, -0.120), CAN_H / 2, 0.015, RED, [((0, 0, 0), CAN_R, CAN_H)])
BOTTLE_R = 0.0325                          # an empty 500 mL PET bottle on its side, 207 mm long, 20 g, its body dented
                                           # flat along the underside (2.4 cm wide) so it doesn't roll
BOTTLE = _cyl_part("bottle", (0.770, 0.080), BOTTLE_R, 0.020, CLEAR, [
    ((-0.0235, 0, 0), BOTTLE_R, 0.160, "X"),      # body, 65 mm across
    ((0.0690, 0, 0), 0.022, 0.025, "X"),           # shoulder
    ((0.0925, 0, 0), 0.0145, 0.022, "X")],         # neck and cap
    boxes=[((-0.0235, 0, -0.0285), (0.150, 0.024, 0.008), CLEAR)],   # a flattened strip along its underside
    yaw=math.radians(8))                           # its cap pointing away from the arms
CUP_H = 0.112                              # a 12 oz paper hot cup standing up: 112 mm tall, 64 mm across the base, 90 mm
CUP = _cyl_part("cup", (0.640, 0.210), CUP_H / 2, 0.012, WHITE, [   # at the rim, 12 g
    ((0, 0, 0.018 - 0.056), 0.032, 0.036),
    ((0, 0, 0.0545 - 0.056), 0.037, 0.037),
    ((0, 0, 0.0905 - 0.056), 0.042, 0.035),
    ((0, 0, 0.110 - 0.056), 0.045, 0.004)])
LITTER = ("can", "bottle", "cup")
LITTER_HALF = {"can": np.array([CAN_R, CAN_R, CAN_H / 2]), "bottle": np.array([0.1035, BOTTLE_R, BOTTLE_R]),
               "cup": np.array([0.045, 0.045, CUP_H / 2])}

OBJECTS = [TOOL, BIN, CAN, BOTTLE, CUP]

# -- the trigger works the jaws ----------------------------------------------------------------------------------------
# There are no gear or cable joints in the simulator, so after every control step the jaws' spring drives are given the
# closure the trigger asks for: trigger pulled a fraction f of its stroke -> each jaw's target f of its stroke. A jaw
# stopped by litter presses on it with JAW_RATE x (target - position): about 9 N for the bottle with the trigger fully
# pulled.


def _trigger_pos(environment) -> float:
    asset = environment.unwrapped.scene["picker"]
    names = list(asset.joint_names)
    q = asset.data.joint_pos
    q = getattr(q, "torch", q)
    return float(q[0, names.index("trigger_slide")].item())


def jaw_target(trigger: float) -> float:
    """Each jaw's closure (m, toward the centreline) for a trigger position (m, negative when pulled)."""
    return JAW_TRAVEL * min(1.0, max(0.0, -trigger / TRIG_TRAVEL))


@on_step
def _drive_jaws(environment) -> None:
    c = jaw_target(_trigger_pos(environment))
    set_slider_targets(environment, "picker", {"jaw_l": -c, "jaw_r": c})


# -- scoring -----------------------------------------------------------------------------------------------------------
IN_BIN_PTS = 0.3            # each piece in the bin
LIFTED_PTS = 0.1            # each piece lifted clear of the table (its lowest point 5 cm up) but not in the bin
LIFT_H = 0.05
TABLE_X, TABLE_Y = (-0.19, 0.91), 0.65      # the table top's extent
TOUCH_MARGIN = 0.003        # litter within 3 mm of a gripper's fingers (or the space between them) counts as touched
_STATE: dict = {}


@on_reset
def _reset_state(environment=None) -> None:
    _STATE.update(lifted={n: False for n in LITTER}, touched=False, tool_off=False, recent=[])
    _STATE["_held"], _STATE["_step"] = 0, None
    if environment is not None:
        set_slider_targets(environment, "picker", {"jaw_l": 0.0, "jaw_r": 0.0})


_reset_state()


def _box_points(half: np.ndarray, n: int = 5) -> np.ndarray:
    g = np.linspace(-1.0, 1.0, n)
    return np.array([[a, b, c] for a in g for b in g for c in g]) * half


_LITTER_PTS = {n: _box_points(h) for n, h in LITTER_HALF.items()}


def gripper_touches(grippers: dict, piece: PartState, name: str) -> bool:
    """Whether a piece of litter (its bounding box, sampled) reaches into either gripper's fingers, the space between
    them, or the gripper body just above them, within TOUCH_MARGIN. ``grippers`` is litterkit.gripper_poses(env)."""
    pts = np.array([piece.to_world(p) for p in _LITTER_PTS[name]])
    m = TOUCH_MARGIN
    for pos, rot, finger in grippers.values():
        local = (pts - pos) @ rot              # into the gripper link frame: x closing axis, z toward the fingertips
        half_open = max(0.0, -finger)
        fingers = ((np.abs(local[:, 0]) <= half_open + 0.0105 + m) & (np.abs(local[:, 1]) <= 0.0165 + m)
                   & (local[:, 2] >= 0.040 - m) & (local[:, 2] <= 0.147 + m))
        body = ((np.abs(local[:, 0]) <= 0.055 + m) & (np.abs(local[:, 1]) <= 0.035 + m)
                & (local[:, 2] >= -0.02 - m) & (local[:, 2] < 0.040))
        if fingers.any() or body.any():
            return True
    return False


@on_step
def _watch_grippers(environment) -> None:
    from yam_arena.scene import object_states

    states = object_states(environment, list(LITTER))
    grippers = gripper_poses(environment)
    _STATE["touched"] |= any(gripper_touches(grippers, states[n], n) for n in LITTER)


def in_bin(piece: PartState) -> bool:
    """The piece's centre is inside the bin: within its inner walls and below its rim."""
    d = np.asarray(piece.pos, float) - np.array([BIN_XY[0], BIN_XY[1], TOP])
    return bool(abs(d[0]) < BIN_L / 2 - WALL and abs(d[1]) < BIN_W / 2 - WALL and WALL < d[2] < BIN_H)


def tool_off_table(tool: PartState) -> bool:
    p = np.asarray(tool.pos, float)
    on_top = TABLE_X[0] <= p[0] <= TABLE_X[1] and abs(p[1]) <= TABLE_Y
    return bool(p[2] < TOP - 0.03 or (not on_top and p[2] < TOP + 0.05))


def _assess(objects) -> tuple[float, bool]:
    _STATE["tool_off"] |= tool_off_table(objects["picker"])
    for n in LITTER:
        _STATE["lifted"][n] |= bool(objects[n].low[2] >= TOP + LIFT_H)
    if _STATE["touched"] or _STATE["tool_off"]:
        return 0.0, False
    inside = {n: in_bin(objects[n]) for n in LITTER}
    recent = _STATE["recent"]
    recent.append(np.concatenate([np.asarray(objects[n].pos, float) for n in LITTER]))
    del recent[:-20]
    still = len(recent) == 20 and all(np.abs(p - recent[-1]).max() < 0.001 for p in recent)
    if all(inside.values()):
        return 1.0, bool(still)
    pts = sum(IN_BIN_PTS if inside[n] else LIFTED_PTS if _STATE["lifted"][n] else 0.0 for n in LITTER)
    return round(pts, 2), False


FULL_HOLD_STEPS = 20        # the success condition must hold this many control steps in a row (1 s) for 1.0
UNHELD = 0.9                # the score while it has held for less


def _update(objects) -> tuple[float, bool]:
    """This step's (score, success). success() and score() are both called on each step's states; the step is
    assessed once. The score is 1.0 only once the success condition has held for 1 s; until then it is UNHELD."""
    last = _STATE.get("_step")
    if last is not None and last[0] is objects and last[2] < 2:
        _STATE["_step"] = (last[0], last[1], last[2] + 1)
        return last[1]
    raw, ok = _assess(objects)
    _STATE["_held"] = _STATE.get("_held", 0) + 1 if ok else 0
    out = (raw if raw < 1.0 else 1.0 if _STATE["_held"] >= FULL_HOLD_STEPS else UNHELD, ok)
    _STATE["_step"] = (objects, out, 1)
    return out


def success(objects) -> bool:
    """All three pieces in the bin and still, no litter touched by a gripper, the tool still on the table."""
    return _update(objects)[0] >= 1.0   # true only once the full score's 1 s hold is complete


def score(objects) -> float:
    """1.0 all three pieces in the bin; otherwise 0.3 per piece in the bin plus 0.1 per piece lifted 5 cm clear of the
    table but not in the bin. 0 if a gripper touched litter or the tool fell off the table."""
    return _update(objects)[0]
