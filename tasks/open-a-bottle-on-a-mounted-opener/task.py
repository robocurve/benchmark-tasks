"""On the station table, in front of the right arm, stands a full 275 ml long-neck glass bottle closed with a red crown
cap on a standard 26 mm crown finish. Further forward a cast steel wall-mount bottle opener is screwed to a dark post:
a back plate with a jaw sticking out toward the robot. The jaw has an upper prong (the fulcrum over the cap) and, under
it, a slot whose lower edge is a catch lip that reaches 1 mm under a crimped cap's skirt. A round cork coaster lies on
the table near the middle.

A crown cap is crimped under the glass bead and cannot be pulled off by hand. To open the bottle on the opener, the
bottle is lifted, the rear of its cap is slid into the jaw's slot so that the catch lip is under the skirt's lower edge,
and the bottle is pulled down: the lip holds the back of the skirt while the bottle drops, the skirt is bent up off the
bead and the cap pops off. The open bottle is then stood upright on the coaster and let go.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from partkit import _SPECS, _part_state, on_reset, on_step, part, write_pose  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
TOP = TABLE_TOP[SCENE]

INSTRUCTION = "Open the bottle on the mounted bottle opener, then stand the open bottle on the coaster and let go."

GLASS = (0.16, 0.46, 0.24)
LABEL = (0.93, 0.92, 0.86)
CAP_RED = (0.80, 0.10, 0.08)
STEEL = (0.62, 0.64, 0.67)
POST = (0.18, 0.19, 0.21)
CORK = (0.72, 0.55, 0.36)

# -- the bottle: a 275 ml long-neck with a 26 mm crown finish; origin at the middle of its base ------------------------
BODY_R, BODY_H = 0.029, 0.110  # 58 mm body
GRIP_R, GRIP_Z0, GRIP_Z1 = 0.016, 0.130, 0.162  # the lower neck, 32 mm across, where the demo grips it
NECK_R = 0.01175  # 23.5 mm: the neck just under the finish
BEAD_R, BEAD_H = 0.0132, 0.0043  # 26.4 mm: the bead (transfer ring) the cap is crimped under
SEAT = 0.185  # top of the finish, where the cap's liner sits (above the base)
BOTTLE_MASS = 0.45  # full
BOTTLE_AT = (0.27, -0.25)

# -- the crown cap, crimped: 28.8 mm across, 6 mm tall; origin at its underside's centre (on the finish) ---------------
CAP_R = 0.0144  # outside of the crimped skirt
SKIRT_T = 0.0010  # the skirt ring's wall in the model: 13.4 to 14.4 mm from the axis
SKIRT_H = 0.0050  # skirt below the crown's top
TOP_T = 0.0010  # the crown's top
CAP_H = SKIRT_H + TOP_T
FLUTES = 12  # the skirt is modelled as 12 flat segments
CAP_MASS = 0.003

# -- the opener: back plate on a post, and a jaw (upper prong over a slot whose lower edge is the catch lip) -----------
ENGAGE = (0.40, -0.20)  # bottle axis with the cap's rear in the slot, the skirt against the slot's back
LIFT = 0.035  # bottle base above the table while the cap is in the slot
SLOT_BACK = ENGAGE[0] + CAP_R + 0.0001  # the slot's back wall: 0.1 mm behind the skirt at ENGAGE
LIP_REACH = 0.0010  # the catch lip sticks out of the slot's back wall this far, under the skirt
LIP_TIP = SLOT_BACK - LIP_REACH
SLOT_H = CAP_H + 0.0025  # 8.5 mm: the cap and 2.5 mm to tip in
LIP_TOP = TOP + LIFT + SEAT - SKIRT_H - 0.00125  # 1.25 mm under the skirt's edge on the way in
SLOT_TOP = LIP_TOP + SLOT_H
PRONG_REACH = 0.0080  # the upper prong reaches 8 mm past the slot's back over the cap's top
PRONG_T, LIP_T = 0.003, 0.004
JAW_W = 0.018  # the jaw's width (y)
FACE_X = ENGAGE[0] + 0.0455  # the back plate's front face: the jaw sticks out 31.5 mm in front of it
PLATE_T, PLATE_W = 0.005, 0.060
PLATE_Z0, PLATE_Z1 = LIP_TOP - LIP_T - 0.045, SLOT_TOP + PRONG_T + 0.010
POST_T, POST_W = 0.025, 0.045

# -- the coaster -------------------------------------------------------------------------------------------------------
COASTER = (0.31, -0.04)
COASTER_R, COASTER_H = 0.045, 0.004

# -- scoring -----------------------------------------------------------------------------------------------------------
TIPPED_DEG = 30.0  # the full bottle tilted more than this, ever: it spilled
UPRIGHT_DEG = 10.0
ON_COASTER = 0.030  # bottle axis within this of the coaster's centre
CATCH_IN = (0.0128, 0.0142)  # lip tip this far in from the cap's axis: past the bead (13.2 mm), under the skirt
CATCH_SIDE = 0.006  # cap axis within this of the jaw's centre line sideways
CATCH_GAP = (-0.0005, 0.0025)  # skirt's lower edge this far above the lip's top
POP_LIFT = 0.0015  # the skirt's caught edge bent up this far off the bead: the crimp lets go
SPRING_OFF = (0.010, 0.0055)  # the popped cap springs this far forward and up: its skirt clears the bead
REGLUE = 0.0005  # away from the lip the crimp holds: any shift of the cap off the finish is undone
LET_GO = 0.06  # each gripper's TCP at least this far from the bottle's grip point


def _cyl(z0, z1, r, color):
    return ((0.0, 0.0, (z0 + z1) / 2), r, z1 - z0, color)


BOTTLE = part(
    "bottle", pos=(BOTTLE_AT[0], BOTTLE_AT[1], TOP + 0.001), mass=BOTTLE_MASS, color=GLASS, friction=1.0,
    boxes=[((0.0, 0.0, 0.055), (0.03, 0.03, 0.08), GLASS)],
    cylinders=[_cyl(0.0, BODY_H, BODY_R, GLASS),
               _cyl(0.035, 0.085, BODY_R + 0.0004, LABEL),
               _cyl(BODY_H, 0.118, 0.025, GLASS),
               _cyl(0.118, 0.125, 0.020, GLASS),
               _cyl(0.125, GRIP_Z0, 0.017, GLASS),
               _cyl(GRIP_Z0, GRIP_Z1, GRIP_R, GLASS),
               _cyl(GRIP_Z1, 0.170, 0.0135, GLASS),
               _cyl(0.170, SEAT - BEAD_H, NECK_R, GLASS),
               _cyl(SEAT - BEAD_H, SEAT, BEAD_R, GLASS)])


def _skirt():
    r = CAP_R - SKIRT_T / 2
    w = 2 * r * math.tan(math.pi / FLUTES) + 0.0004
    out = []
    for i in range(FLUTES):
        a = 2 * math.pi * i / FLUTES  # a flat face toward the jaw (+x), not a corner
        q = (math.cos(a / 2), 0.0, 0.0, math.sin(a / 2))
        out.append(((r * math.cos(a), r * math.sin(a), -SKIRT_H / 2), (SKIRT_T, w, SKIRT_H), CAP_RED, q))
    return out


CAP = part(
    "cap", pos=(BOTTLE_AT[0], BOTTLE_AT[1], TOP + 0.001 + SEAT), mass=CAP_MASS, color=CAP_RED, friction=0.8,
    boxes=_skirt(), cylinders=[_cyl(0.0, TOP_T, CAP_R, CAP_RED)])

JAW_Y = ENGAGE[1]
OBJECTS = [
    Box("opener_post", pos=(FACE_X + PLATE_T + POST_T / 2, JAW_Y, (TOP + PLATE_Z1) / 2),
        size=(POST_T, POST_W, PLATE_Z1 - TOP), color=POST),
    Box("opener_plate", pos=(FACE_X + PLATE_T / 2, JAW_Y, (PLATE_Z0 + PLATE_Z1) / 2),
        size=(PLATE_T, PLATE_W, PLATE_Z1 - PLATE_Z0), color=STEEL),
    Box("opener_lip", pos=((LIP_TIP + FACE_X) / 2, JAW_Y, LIP_TOP - LIP_T / 2),
        size=(FACE_X - LIP_TIP, JAW_W, LIP_T), color=STEEL),
    Box("opener_jaw_back", pos=((SLOT_BACK + FACE_X) / 2, JAW_Y, (LIP_TOP + SLOT_TOP) / 2),
        size=(FACE_X - SLOT_BACK, JAW_W, SLOT_H), color=STEEL),
    Box("opener_prong", pos=((SLOT_BACK - PRONG_REACH + FACE_X) / 2, JAW_Y, SLOT_TOP + PRONG_T / 2),
        size=(FACE_X - SLOT_BACK + PRONG_REACH, JAW_W, PRONG_T), color=STEEL),
    Cylinder("coaster", pos=(COASTER[0], COASTER[1], TOP + COASTER_H / 2), radius=COASTER_R, height=COASTER_H,
             color=CORK),
    BOTTLE,
    CAP,
]


# -- the trial record, followed after every control step -----------------------------------------------------------------

def _fresh() -> dict:
    return {"engaged": False, "popped": False, "tipped": False, "steps": 0, "pop_step": None, "tcps": [],
            "max_edge_lift": 0.0}


_S = _fresh()
_DEBUG = bool(os.environ.get("BOTTLE_DEBUG"))


def _tilt_deg(state) -> float:
    z = state.rotation()[:, 2]
    return math.degrees(math.acos(max(-1.0, min(1.0, float(z[2])))))


def caught(cap_pos) -> bool:
    """The catch lip is under the skirt's lower edge at the back of the cap: the lip's tip is past the bead and inside
    the skirt, the cap is on the jaw's centre line, and the skirt's edge is on or just above the lip."""
    x, y, z = (float(v) for v in cap_pos)
    reach = LIP_TIP - x
    edge = z - SKIRT_H  # the skirt's lower edge
    return (CATCH_IN[0] < reach < CATCH_IN[1] and abs(y - JAW_Y) < CATCH_SIDE
            and CATCH_GAP[0] < edge - LIP_TOP < CATCH_GAP[1])


def edge_lift(bottle, cap) -> float:
    """How far the skirt's edge on the opener's side (+x) has been bent up off the finish, in the bottle's frame."""
    n = cap.rotation()[:, 2]
    u = np.array([1.0, 0.0, 0.0]) - n[0] * n
    u /= np.linalg.norm(u) + 1e-12
    return float(bottle.to_local(cap.pos + CAP_R * u)[2] - SEAT)


def grippers(environment) -> list[np.ndarray]:
    """World positions of both grippers' TCP (between the finger pads, 13 mm above the pad tips)."""
    base = environment.unwrapped
    origin = base.scene.env_origins[0]
    out = []
    for key in ("left_robot", "right_robot"):
        art = base.scene[key]
        names = list(art.body_names)
        pos = (getattr(art.data.body_pos_w, "torch", art.data.body_pos_w)[0] - origin).detach().cpu().numpy()
        g = pos[names.index("gripper")].astype(np.float64)
        mid = (pos[names.index("tip_left")] + pos[names.index("tip_right")]).astype(np.float64) / 2
        ax = (mid - g) / (np.linalg.norm(mid - g) + 1e-12)
        out.append(g + ax * 0.134)
    return out


def _seat_cap(environment, bottle) -> None:
    x, y, z, w = (float(v) for v in bottle.quat_xyzw)
    write_pose(environment, CAP.name, tuple(float(v) for v in bottle.to_world((0.0, 0.0, SEAT))), (x, y, z, w),
               lin_vel=tuple(float(v) for v in bottle.lin_vel), ang_vel=tuple(float(v) for v in bottle.ang_vel))


@on_reset
def _reset(environment=None):
    _S.clear()
    _S.update(_fresh())
    if environment is not None:
        _seat_cap(environment, _part_state(environment, _SPECS[BOTTLE.name]))


@on_step
def _follow(environment) -> None:
    """The crimp: until it has been sprung, the cap stays on the finish, except while the catch lip is under its skirt,
    when the physics decides; once the lip has bent the caught edge 1.5 mm up off the bead, the cap pops off."""
    _S["steps"] += 1
    bottle = _part_state(environment, _SPECS[BOTTLE.name])
    cap = _part_state(environment, _SPECS[CAP.name])
    if _tilt_deg(bottle) > TIPPED_DEG or bottle.pos[2] < TOP - 0.05:
        _S["tipped"] = True
    try:
        _S["tcps"] = grippers(environment)
    except (KeyError, ValueError, AttributeError):
        _S["tcps"] = []
    lifted = edge_lift(bottle, cap)
    if not _S["popped"]:
        engaged = caught(cap.pos) and _tilt_deg(bottle) < 20.0
        if engaged:
            _S["engaged"] = True
            _S["max_edge_lift"] = max(_S["max_edge_lift"], lifted)
        if engaged and lifted > POP_LIFT:
            _S["popped"], _S["pop_step"] = True, _S["steps"]
            # the crimp lets go with a snap: within one control step the cap springs up off the finish and forward,
            # clear of the bead and of the prong, its free front flipping up, and it falls toward the robot
            jump = np.array([-SPRING_OFF[0], 0.0, SPRING_OFF[1]])
            write_pose(environment, CAP.name, tuple(float(v) for v in np.asarray(cap.pos) + jump),
                       tuple(float(v) for v in cap.quat_xyzw), lin_vel=(-0.4, 0.0, 0.4), ang_vel=(0.0, 20.0, 0.0))
        elif not engaged:
            off = bottle.to_local(cap.pos)
            if abs(off[2] - SEAT) > REGLUE or np.linalg.norm(off[:2]) > REGLUE or _tilt_deg_rel(bottle, cap) > 1.0:
                _seat_cap(environment, bottle)
    if _DEBUG and _S["steps"] % 5 == 0:
        print(f"[bottle] step {_S['steps']} bottle {np.round(bottle.pos, 4)} tilt {_tilt_deg(bottle):.1f} "
              f"cap {np.round(cap.pos, 4)} reach {(LIP_TIP - float(cap.pos[0])) * 1000:.2f}mm "
              f"edge {(float(cap.pos[2]) - SKIRT_H - LIP_TOP) * 1000:.2f}mm lift {lifted * 1000:.2f}mm "
              f"engaged {_S['engaged']} popped {_S['popped']} tcps {[np.round(t, 3).tolist() for t in _S['tcps']]}",
              flush=True)


def _tilt_deg_rel(bottle, cap) -> float:
    c = float(np.clip(bottle.rotation()[:, 2] @ cap.rotation()[:, 2], -1.0, 1.0))
    return math.degrees(math.acos(c))


def on_coaster(bottle) -> bool:
    dx, dy = float(bottle.pos[0] - COASTER[0]), float(bottle.pos[1] - COASTER[1])
    base_z = float(bottle.pos[2])
    return (math.hypot(dx, dy) < ON_COASTER and TOP + COASTER_H - 0.003 < base_z < TOP + COASTER_H + 0.006
            and _tilt_deg(bottle) < UPRIGHT_DEG)


def released(bottle) -> bool:
    grip = bottle.to_world((0.0, 0.0, GRIP_UP))
    return bool(_S["tcps"]) and all(np.linalg.norm(np.asarray(t) - grip) > LET_GO for t in _S["tcps"])


def score(objects) -> float:
    bottle = objects[BOTTLE.name]
    if _S["tipped"]:
        return 0.0
    if not _S["popped"]:
        return 0.25 if _S["engaged"] else 0.0
    if not on_coaster(bottle):
        return 0.5
    if not (released(bottle) and bottle.resting()):
        return 0.75
    return 1.0


def success(objects) -> bool:
    return score(objects) >= 1.0


# -- the scripted demo (right arm) -----------------------------------------------------------------------------------

GRIP_UP = 0.154  # TCP above the bottle's base when gripping it: pads on the lower neck, 3.2 cm under the cap's top
PULL = 0.006  # how far the bottle is pulled down with the cap's rear in the slot
DROP = 0.012  # then down this far, so the finish is out of the popped cap
PUSH = float(os.environ.get("BOTTLE_PUSH", "0.004"))  # ... while pressing it this far back into the jaw
# the loaded arm misses its commanded pose by a pose-dependent offset (x, y, z); the demo aims the other way: SAG at the
# opener, SAG_DOWN over the coaster. Env overrides for tuning.
SAG = tuple(float(v) for v in os.environ.get("BOTTLE_SAG", "-0.0165,-0.001,0.0116").split(","))
SAG_DOWN = tuple(float(v) for v in os.environ.get("BOTTLE_SAG_DOWN", "0.002,0.016,0.009").split(","))
PRESS = float(os.environ.get("BOTTLE_PRESS", "-0.0002"))  # aim the cap this far past the skirt touching the slot's back


def _approach_z() -> float:
    """TCP height with the skirt's edge 1.25 mm over the lip (the cap's top 1.25 mm under the prong)."""
    return LIP_TOP + 0.00125 + SKIRT_H - SEAT + GRIP_UP


def demo():
    variant = os.environ.get("DEMO_VARIANT", "full")
    shortcut = _shortcut(variant)
    if shortcut is not None:
        return shortcut
    bx, by = BOTTLE_AT
    ex, ey = ENGAGE[0] + PRESS, ENGAGE[1]
    cx, cy = COASTER
    sx, sy, sz = SAG
    ex, ey, cx, cy = ex + sx, ey + sy, cx + SAG_DOWN[0], cy + SAG_DOWN[1]
    z_grip = TOP + 0.001 + GRIP_UP
    z_eng = _approach_z() + sz
    z_place = TOP + COASTER_H + 0.002 + GRIP_UP
    hover = TOP + 0.23
    right = [
        Waypoint(2.0, (bx, by, hover), FINGER_OPEN, "above the bottle"),
        Waypoint(1.5, (bx, by, z_grip), FINGER_OPEN, "down round the neck"),
        Waypoint(0.8, (bx, by, z_grip), FINGER_CLOSED, "grip"),
        Waypoint(1.2, (bx, by, z_eng), FINGER_CLOSED, "lift"),
        Waypoint(1.8, (ex - 0.04, ey, z_eng), FINGER_CLOSED, "in front of the opener"),
        Waypoint(0.8, (ex - 0.04, ey, z_eng), FINGER_CLOSED, "steady"),
    ]
    if variant in ("full", "no_place", "no_release"):
        right += [
            Waypoint(3.0, (ex, ey, z_eng), FINGER_CLOSED, "cap's rear into the slot"),
            Waypoint(0.6, (ex, ey, z_eng), FINGER_CLOSED, "settle"),
            Waypoint(2.0, (ex + PUSH, ey, z_eng - PULL), FINGER_CLOSED, "pull down, pressing back into the jaw"),
            Waypoint(0.8, (ex, ey, z_eng - PULL - DROP), FINGER_CLOSED, "drop clear of the cap"),
            Waypoint(1.2, (ex - 0.05, ey, z_eng - PULL - DROP), FINGER_CLOSED, "back out"),
        ]
    if variant == "no_place":
        right += [Waypoint(3.0, (ex - 0.05, ey, z_eng - PULL - DROP), FINGER_CLOSED, "hold it")]
        return {"right": right}
    right += [
        Waypoint(1.8, (cx, cy, z_eng), FINGER_CLOSED, "over the coaster"),
        Waypoint(1.5, (cx, cy, z_place), FINGER_CLOSED, "set down"),
    ]
    if variant == "no_release":
        right += [Waypoint(3.0, (cx, cy, z_place), FINGER_CLOSED, "keep holding")]
        return {"right": right}
    right += [
        Waypoint(0.8, (cx, cy, z_place), FINGER_OPEN, "let go"),
        Waypoint(1.0, (cx, cy, hover), FINGER_OPEN, "up"),
        Waypoint(1.8, (0.12, -0.42, TOP + 0.24), FINGER_OPEN, "clear"),
    ]
    return {"right": right}


def _shortcut(variant: str):
    """Shortcuts that must not score: doing nothing, carrying the closed bottle straight to the coaster, pinching the
    cap and pulling it straight up, and pressing the cap against the jaw above the slot and pulling down."""
    bx, by = BOTTLE_AT
    cx, cy = COASTER[0] + SAG_DOWN[0], COASTER[1] + SAG_DOWN[1]
    hover = TOP + 0.23
    z_grip = TOP + 0.001 + GRIP_UP
    z_place = TOP + COASTER_H + 0.002 + GRIP_UP
    pick = [
        Waypoint(2.0, (bx, by, hover), FINGER_OPEN, "above the bottle"),
        Waypoint(1.5, (bx, by, z_grip), FINGER_OPEN, "down round the neck"),
        Waypoint(0.8, (bx, by, z_grip), FINGER_CLOSED, "grip"),
    ]
    if variant == "nothing":
        return {"right": [Waypoint(3.0, (0.25, -0.30, hover), FINGER_OPEN, "wait")]}
    if variant == "skip_opener":
        z_carry = _approach_z() + SAG[2]
        return {"right": pick + [
            Waypoint(1.2, (bx, by, z_carry), FINGER_CLOSED, "lift"),
            Waypoint(2.0, (cx, cy, z_carry), FINGER_CLOSED, "over the coaster"),
            Waypoint(1.5, (cx, cy, z_place), FINGER_CLOSED, "set down"),
            Waypoint(0.8, (cx, cy, z_place), FINGER_OPEN, "let go"),
            Waypoint(1.0, (cx, cy, hover), FINGER_OPEN, "up"),
            Waypoint(1.8, (0.12, -0.42, TOP + 0.24), FINGER_OPEN, "clear"),
        ]}
    if variant == "under_prong":
        # the cap's front pushed under the prong tip, not into the slot (the lip never under the skirt), then pulled
        ex, ey, sz = ENGAGE[0] - 0.006 + SAG[0], ENGAGE[1] + SAG[1], SAG[2]
        z = _approach_z() + sz
        return {"right": pick + [
            Waypoint(1.2, (bx, by, z), FINGER_CLOSED, "lift"),
            Waypoint(1.8, (ex - 0.04, ey, z), FINGER_CLOSED, "in front of the opener"),
            Waypoint(1.5, (ex, ey, z), FINGER_CLOSED, "cap under the prong, short of the slot"),
            Waypoint(1.5, (ex, ey, z - PULL), FINGER_CLOSED, "pull down"),
            Waypoint(1.2, (ex - 0.05, ey, z - PULL), FINGER_CLOSED, "back out"),
            Waypoint(3.0, (ex - 0.05, ey, z - PULL), FINGER_CLOSED, "hold"),
        ]}
    if variant == "pull_cap":
        z_cap = TOP + 0.001 + SEAT + 0.004
        return {"right": [
            Waypoint(2.0, (bx, by, hover), FINGER_OPEN, "above the cap"),
            Waypoint(1.5, (bx, by, z_cap), FINGER_OPEN, "fingers round the cap"),
            Waypoint(0.8, (bx, by, z_cap), FINGER_CLOSED, "pinch it"),
            Waypoint(2.0, (bx, by, z_cap + 0.05), FINGER_CLOSED, "pull it straight up"),
            Waypoint(1.0, (bx, by, z_cap + 0.05), FINGER_CLOSED, "hold"),
            Waypoint(1.0, (bx, by, z_cap + 0.05), FINGER_OPEN, "let go"),
            Waypoint(1.8, (0.12, -0.42, TOP + 0.24), FINGER_OPEN, "clear"),
        ]}
    return None
