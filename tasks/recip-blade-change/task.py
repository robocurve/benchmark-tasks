"""On the bench lies a cordless reciprocating saw on its side, its blade clamp toward the arms: a steel clamp block
with a 3 mm slot for a blade tang (lead-ins at its mouth) and a spring-loaded jaw whose pin drops into the
tang's hole. A worn blade has been taken out; a 150 mm wood blade lies flat across two 30 mm wooden rests to the left,
its teeth down and its tang toward the saw's far side.

The robot must fit that blade: push its tang into the clamp slot until the jaw's pin drops into the tang's hole and the
blade's shoulder comes up against the clamp face, with the blade square and its teeth down, then let go.

Everything is a real product at true size: a 150 mm reciprocating-saw blade (1.6 mm thick, 19 mm wide, 13 mm tang with
a 6.5 mm hole 10 mm behind the shoulder) and the tool-free push-in clamp used on current cordless recip saws, where the
blade is pushed straight in and a spring jaw clicks into the tang hole. The saw's body is cut short behind the clamp.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score ...
Demo:    python demo.py task.py demo_out
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from partkit import Hinge, PartState, finger_points, on_reset, on_step, part  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = "Fit the spare blade into the saw's clamp so it is locked, square and teeth down."

STEEL = (0.58, 0.60, 0.62)
DARK = (0.22, 0.23, 0.24)
BLADE_COLOR = (0.72, 0.74, 0.78)


def _mm(x0, x1, y0, y1, z0, z1, color, quat=None):
    """A box from millimetres in the part frame, optionally rotated by a (w, x, y, z) quaternion."""
    spec = (((x0 + x1) / 2000, (y0 + y1) / 2000, (z0 + z1) / 2000),
            ((x1 - x0) / 1000, (y1 - y0) / 1000, (z1 - z0) / 1000), color)
    return spec if quat is None else (*spec, quat)


# -- the saw. Part frame: origin on the table under the middle of the body; +x into the tool, y across, z up.
# The clamp's front face is 170 mm in front of the origin and the blade slot runs 25 mm back from it.
SAW_X, SAW_Y = 0.470, 0.050
SLOT_Y = 0.0                        # the slot's centreline in the part frame
SLOT_Z0, SLOT_Z1 = 0.0482, 0.0512   # 3.0 mm tall: a 1.6 mm tang with 1.4 mm of clearance (3.7 degrees of rock)
SNUG_Z1 = 0.0501                    # the roof over the slot's back 6 mm: 1.9 mm above the floor
SLOT_HALF_W = 0.0080                # 16 mm wide: a 13 mm tang with 1.5 mm each side
LEAD_X = -0.1663                    # the 15 degree lead-in ramps run from the clamp face to here
CLAMP_FACE = -0.170                 # part x of the clamp's front face (world 0.300)
SLOT_BACK = -0.145                  # the slot's back wall
PIN_X0, PIN_X1 = -0.1625, -0.1575   # the 5 mm jaw pin, 10 mm behind the clamp face (world 0.3075..0.3125)
PIN_TIP = 0.0493                    # the pin's tip at the jaw's closed angle: 0.5 mm into the tang's hole
HOLE_BEHIND_SHOULDER = 0.010        # the tang's hole, 10 mm behind the shoulder

_C15, _S15 = 0.99144, 0.13053         # quaternion halves of a 15 degree turn about y
SAW_BOXES = [
    _mm(-140, 140, -45, 45, 0, 95, DARK),                                     # body
    _mm(-170, -110, -55, 55, 0, 12, STEEL),                                   # shoe, on the table
    # clamp block: the roof above the slot, split by the jaw pin's hole (8 x 7 mm, room for the pin's swing)
    _mm(LEAD_X * 1000, -164, -20, 20, SLOT_Z1 * 1000, 65, STEEL),
    _mm(-156, -145, -20, 20, SLOT_Z1 * 1000, 65, STEEL),
    _mm(-164, -156, -20, -3.5, SLOT_Z1 * 1000, 65, STEEL),
    _mm(-164, -156, 3.5, 20, SLOT_Z1 * 1000, 65, STEEL),
    _mm(-170, LEAD_X * 1000, -20, 20, 53, 65, STEEL),                         # above the roof's lead-in
    _mm(LEAD_X * 1000, -145, -20, 20, 35, SLOT_Z0 * 1000, STEEL),             # the slot's floor
    _mm(-170, LEAD_X * 1000, -20, 20, 35, 46, STEEL),                         # below the floor's lead-in
    _mm(-145, -120, -20, 20, 35, 65, STEEL),                                  # the slot's back wall
    _mm(LEAD_X * 1000, -145, -20, -SLOT_HALF_W * 1000, 46, 54, STEEL),        # side wall, -y
    _mm(LEAD_X * 1000, -145, SLOT_HALF_W * 1000, 20, 46, 54, STEEL),          # side wall, +y
    _mm(-170, LEAD_X * 1000, -20, -11, 46, 54, STEEL),                        # behind the side lead-ins
    _mm(-170, LEAD_X * 1000, 11, 20, 46, 54, STEEL),
    # side lead-ins: the mouth is 20 mm wide at the clamp face and narrows to the 16 mm slot 3.7 mm in
    ((-0.16733, 0.01034, 0.050), (0.00447, 0.003, 0.008), STEEL, (0.97325, 0.0, 0.0, -0.22975)),
    ((-0.16733, -0.01034, 0.050), (0.00447, 0.003, 0.008), STEEL, (0.97325, 0.0, 0.0, 0.22975)),
    # the slot's back 6 mm is snug (1.9 mm tall), with a 15 degree ramp down to it, so a seated blade cannot rock
    _mm(-151, -145, -20, 20, SNUG_Z1 * 1000, SLOT_Z1 * 1000, STEEL),
    ((-0.15261, 0.0, (SNUG_Z1 * 1000 + 0.55 + 1.449) / 1000), (0.00424, 0.040, 0.003), STEEL, (_C15, 0.0, _S15, 0.0)),
    # lead-in ramps: the mouth is 4 mm tall at the clamp face and narrows to the 2 mm slot 3.7 mm in
    ((-0.16776, 0.0, (SLOT_Z1 * 1000 + 0.5 + 1.449) / 1000), (0.00383, 0.040, 0.003), STEEL, (_C15, 0.0, _S15, 0.0)),
    ((-0.16776, 0.0, (SLOT_Z0 * 1000 - 0.5 - 1.449) / 1000), (0.00383, 0.040, 0.003), STEEL, (_C15, 0.0, -_S15, 0.0)),
]
# The jaw: a lever on a hinge above the slot whose pin reaches down through the roof's hole. A 45 degree nose on the
# pin lets the tang's own chamfered nose cam the jaw open on the way in; the spring shuts it when the hole arrives.
JAW_PIVOT = (-0.168, 0.0, 0.068)  # on the mouth side of the pin, so a tang pushed in swings the pin up and away
JAW_ARM_Z = (0.066, 0.070)
JAW_BOXES = [
    _mm(-169, -152, -6, 6, JAW_ARM_Z[0] * 1000, JAW_ARM_Z[1] * 1000, STEEL),   # the lever's arm, from the pivot back over the pin
    _mm(PIN_X0 * 1000, PIN_X1 * 1000, -2.5, 2.5, PIN_TIP * 1000 + 2.47, 66, STEEL),  # the pin
    # the pin's pointed end: a 3.5 mm square bar turned 45 degrees, so the tang's front edge cams it up
    ((-0.160, 0.0, PIN_TIP + 0.00247), (0.0035, 0.005, 0.0035), STEEL, (0.92388, 0.0, 0.38268, 0.0)),
]
JAW_HINGE = "jaw"
JAW_SPRING = 0.05                   # N m/rad: a light spring presses the jaw onto its closed stop
JAW_OPEN = 0.5                      # rad the jaw can open (negative angles lift the pin); 0 is its closed stop

SAW = part("saw", pos=(SAW_X, SAW_Y, TOP), mass=12.0, boxes=SAW_BOXES, color=DARK, friction=0.4,
           hinges=(Hinge(JAW_HINGE, pivot=JAW_PIVOT, boxes=tuple(JAW_BOXES), lower=-JAW_OPEN, upper=0.0,
                         stiffness=JAW_SPRING, damping=0.02, target=0.0, mass=0.02, axis="Y"),))

# -- the blade. Part frame: origin at the middle of the body, x along the blade with the tip at +x.
BLADE_T = 0.0016                    # 1/16 in blade stock
BLADE_W = 0.019
TOOTH_T = 0.0004                    # the teeth stand proud of the body's lower edge
TANG_W = 0.013
HOLE_X0, HOLE_X1 = 0.07175, 0.07825  # the 6.5 mm tang hole, its centre 10 mm behind the shoulder at +65 mm
SHOULDER_X = 0.065                  # the shoulder: the tang runs from here to +86.5 mm, the tip is at -65 mm
REST_H = 0.030                      # the spare blade lies across two 30 mm wooden rests
REST_X = ((-0.060, -0.035), (0.050, 0.075))  # the rests' extent along the blade (blade frame)
BLADE_MASS = 0.035
BLADE_BOXES = [
    _mm(-65, 65, -9.5, 9.5, 0, 1.6, BLADE_COLOR),                             # 130 mm of 1.6 mm blade body
    _mm(-65, 65, -9.5, 9.5, -0.4, 0, BLADE_COLOR),                            # the teeth
    _mm(65, 71.75, -6.5, 6.5, 0, 1.6, BLADE_COLOR),                           # tang between shoulder and hole
    _mm(78.25, 86.5, -6.5, 6.5, 0, 1.6, BLADE_COLOR),                         # tang behind the hole
]
BLADE_Z0 = TOP + REST_H + TOOTH_T   # the blade's origin height, its teeth on the rests
BLADE_X0, BLADE_Y0 = 0.350, 0.150   # where the spare blade lies, its tang toward +x
BLADE = part("blade", pos=(BLADE_X0, BLADE_Y0, BLADE_Z0), mass=BLADE_MASS, boxes=BLADE_BOXES,
             color=BLADE_COLOR, friction=0.4)
RESTS = [Box(f"blade_rest_{i}", pos=(BLADE_X0 + (x0 + x1) / 2, BLADE_Y0, TOP + REST_H / 2),
             size=(x1 - x0, 0.030, REST_H), color=(0.76, 0.62, 0.42)) for i, (x0, x1) in enumerate(REST_X)]
OBJECTS = [SAW, BLADE, *RESTS]

BLADE_SIT_Z = TOP + SLOT_Z0              # the blade's origin when its tang rests on the slot's floor
SEAT_X = SAW_X + CLAMP_FACE - SHOULDER_X # the blade's origin with its shoulder on the clamp face (world 0.235)
PRE_X = SEAT_X - 0.0265                  # ... and with its tang tip 5 mm short of the clamp face

# -- scoring -----------------------------------------------------------------------------------------------------------
SAW_MOVE = 0.003                 # the saw shifted this far at any time: 0 for the rest of the trial
SEAT_GAP = 0.0015                # the blade's shoulder within 1.5 mm of the clamp face
SEAT_Y = 0.0015                  # ... and within 1.5 mm of the slot's centreline
SEAT_Z = 0.0015                  # ... on the slot's floor
SEAT_ANGLE = math.radians(4.0)   # square in pitch and in roll (the tang has 1.4 mm of play in the slot)
JAW_CLOSED = 0.015               # rad (0.9 deg): the jaw shut means its pin is in the tang's hole
CLEAR = 0.030                    # both grippers at least 3 cm from the blade and the jaw: let go
STILL_STEPS = 20                 # 1 s at 20 Hz
STILL_TOL = 0.001
_STATE: dict = {}


@on_reset
def _reset_state(environment=None) -> None:
    _STATE.update(saw0=None, failed=False, recent=[], released=False, steps=0)


_reset_state()


def shoulder_world(blade: PartState) -> np.ndarray:
    """The middle of the blade's shoulder (its trailing edge, where it meets the clamp face), world frame."""
    return blade.to_world((SHOULDER_X, 0.0, BLADE_T / 2))


def seated_gap(blade: PartState) -> float:
    """How far the blade's shoulder is short of the clamp face along x (m)."""
    return float((SAW_X + CLAMP_FACE) - shoulder_world(blade)[0])


def blade_seated(blade: PartState) -> bool:
    """Tang in the slot to the shoulder, teeth on the slot's floor, and square in yaw and roll."""
    r = blade.rotation()
    if abs(float(r[2, 0])) > math.sin(SEAT_ANGLE) or abs(float(r[2, 1])) > math.sin(SEAT_ANGLE):
        return False
    if float(r[2, 2]) < math.cos(SEAT_ANGLE):
        return False
    if abs(blade.pos[1] - (SAW_Y + SLOT_Y)) > SEAT_Y:
        return False
    if abs(blade.pos[2] - BLADE_SIT_Z) > SEAT_Z:
        return False
    return abs(seated_gap(blade)) <= SEAT_GAP


def blade_in_slot(blade: PartState) -> bool:
    """The tang has entered the slot and is on its way in (partial credit)."""
    r = blade.rotation()
    if float(r[2, 2]) < 0.995 or abs(float(r[2, 1])) > 0.05:
        return False
    if abs(blade.pos[1] - (SAW_Y + SLOT_Y)) > 0.004:
        return False
    if abs(blade.pos[2] - BLADE_SIT_Z) > 0.006:
        return False
    return -0.030 <= seated_gap(blade) <= 0.030


def jaw_closed(saw: PartState) -> bool:
    """The jaw is at its closed angle, so its pin has dropped into the tang's hole."""
    return abs(float(saw.hinges.get(JAW_HINGE, 1.0))) <= JAW_CLOSED


def _distance_to_part(world_point, part_state: PartState, lo, hi) -> float:
    p = part_state.to_local(world_point)
    return float(np.linalg.norm(np.maximum(0.0, np.maximum(np.asarray(lo) - p, p - np.asarray(hi)))))


BLADE_LO = np.array([-0.065, -BLADE_W / 2, -TOOTH_T])
BLADE_HI = np.array([0.0865, BLADE_W / 2, BLADE_T])
JAW_LO = np.array([-0.165, -0.006, 0.049])
JAW_HI = np.array([-0.120, 0.006, 0.070])


def observe(saw: PartState, blade: PartState, fingers) -> None:
    """Once per control step: the permanent saw-moved flag, whether both grippers are clear of the blade and jaw, and
    the 1 s window of blade, jaw and shoulder poses."""
    s = _STATE
    if s["saw0"] is None:
        s["saw0"] = np.array(saw.pos, float)
    if float(np.linalg.norm(np.asarray(saw.pos) - s["saw0"])) > SAW_MOVE:
        s["failed"] = True
    if fingers is None:
        s["released"] = False
    else:
        s["released"] = all(_distance_to_part(f, blade, BLADE_LO, BLADE_HI) >= CLEAR and
                            _distance_to_part(f, saw, JAW_LO, JAW_HI) >= CLEAR for f in fingers)
    row = np.concatenate([np.asarray(blade.pos, float), shoulder_world(blade),
                          [float(saw.hinges.get(JAW_HINGE, 0.0)) * 0.02]])
    s["recent"].append(row)
    del s["recent"][:-STILL_STEPS]
    s["steps"] += 1


@on_step
def _track(environment) -> None:
    from partkit import _part_state

    saw = _part_state(environment, SAW)
    blade = _part_state(environment, BLADE)
    try:
        fingers = [np.asarray(p, float) for p in finger_points(environment)]
    except Exception as exc:  # noqa: BLE001 - an unreadable gripper never counts as released
        if not _STATE.get("warned"):
            print(f"recip-blade-change: gripper check unavailable: {exc!r}", flush=True)
            _STATE["warned"] = True
        fingers = None
    observe(saw, blade, fingers)


def _still() -> bool:
    recent = _STATE["recent"]
    return len(recent) == STILL_STEPS and float(np.ptp(np.array(recent), axis=0).max()) < STILL_TOL


def distance(objects) -> float:
    """How far the blade is from seated: shoulder gap plus its offset across the slot and off the floor."""
    blade = objects["blade"]
    return float(abs(seated_gap(blade)) + abs(blade.pos[1] - (SAW_Y + SLOT_Y)) + abs(blade.pos[2] - BLADE_SIT_Z))


def _level(objects) -> tuple[float, bool]:
    saw, blade = objects["saw"], objects["blade"]
    if _STATE["failed"]:
        return 0.0, False
    seated = blade_seated(blade)
    if seated and jaw_closed(saw):
        if _still() and _STATE["released"]:
            return 1.0, True
        return 0.75, False
    if seated and _STATE["released"]:            # a seated blade counts only once both grippers have let go
        return 0.5, False
    if blade_in_slot(blade) or seated:
        return 0.25, False
    return 0.0, False


def success(objects) -> bool:
    """Blade seated to its shoulder with the jaw's pin in its hole, both grippers 3 cm clear, and nothing moving for
    1 s, with the saw never shifted more than 3 mm."""
    return _level(objects)[1]


def score(objects) -> float:
    """1.0 seated and locked, grippers clear and still for 1 s; 0.75 seated and locked but a gripper is within 3 cm or
    something still moves; 0.5 seated with the jaw still open and both grippers 3 cm clear; 0.25 the tang is in the
    slot (or seated but still held); else 0. 0 once the saw has been shifted 3 mm."""
    return _level(objects)[0]
