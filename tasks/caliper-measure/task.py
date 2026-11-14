"""On the station table: a 0 to 100 mm digital sliding caliper lying on a wooden rest block, its jaws pointing at the
robots and its head overhanging the block, and a short turned aluminium pin standing on end on the bench in front of
the robots.

The caliper's steel beam runs along y. The fixed jaw is near its left end (+y); the sliding jaw rides on a slider whose
dark housing carries a live display (the jaw gap in mm, to 0.01 mm) and, at its right end, a black thumb rest. Pushing
the thumb rest to the left closes the jaws. They start 40 mm apart, and the display reads 40.00.

Measuring the pin, as an inspector does with a part standing on the bench: one hand picks the caliper up by its head
and holds it level, the open jaws either side of the pin and close to it, its fixed jaw against the pin; the other hand
pushes the thumb rest until the sliding jaw has carried the pin onto the fixed jaw and both just touch it, then lets go
of the thumb rest. The caliper must stay in the hand: nothing of it may rest on the bench. The display reads the
slider, as the caliper's encoder does. The jaws are rigid; the scored jaw force is read from the reaction where the
sliding jaw is fixed to the slider, as a load cell there would read it.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
           --instruction "<see task.json>" --scorer task_score ...
"""

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import labkit  # noqa: E402
from labkit import Joint, on_reset, on_step, part  # noqa: E402
import yam_arena.demo as _demo  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402
from yam_arena.urdf import axis_angle_matrix  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

STEEL = (0.74, 0.75, 0.77)  # hardened stainless
HOUSING_C = (0.20, 0.21, 0.23)
SCREEN = (0.55, 0.60, 0.52)  # grey-green LCD
DIGIT = (0.05, 0.05, 0.06)  # dark LCD segments
BLACK = (0.06, 0.06, 0.07)
WOOD = (0.62, 0.45, 0.28)
BUTTON = (0.75, 0.15, 0.12)
ALU = (0.80, 0.82, 0.85)

# -- the pin (loose), standing on end on the bench -------------------------------------------------------------------
PIN_D = 0.02340  # the true diameter, 23.40 mm
PIN_R, PIN_H = PIN_D / 2, 0.040
PIN_XY = (0.27, 0.0)

# -- the caliper (loose, 120 g). Its frame: origin 50 mm below the beam's underside, under the beam's front edge (the
#    jaw side), level with the fixed jaw's measuring face; x away from the robots, y to their left. --------------------
HB = 0.050  # the beam's underside, above the frame origin
T = 0.0035  # beam and jaw thickness
BEAM_W = 0.016
BEAM_Y = (-0.190, 0.035)  # beam ends: the right end to the head, 35 mm beyond the fixed jaw (where it is held)
JAW_L = 0.045  # the jaws reach 45 mm out from the beam's front edge, towards the robots
JAW_T = 0.008  # jaw plate thickness along y
GAP0 = 0.040  # jaw gap at the start
GAP_MAX = 0.099
MF0 = -GAP0  # the sliding jaw's measuring face at the start (local y)
HOUSING_X = (-0.002, 0.036)
HOUSING_H = 0.009
HOUSING_Y = (-0.079, -0.002)  # from the sliding jaw's face
REST_OFF, REST_T, REST_X, REST_H = 0.085, 0.006, (0.0, 0.020), 0.016  # thumb rest: its pushed face 85 mm behind the jaw
SLIDE_DAMP, SLIDE_FRICTION = 3.0, 0.6  # N s/m; N: the slider stays where it is left while the caliper is carried
# The sliding jaw is rigid: a joint on the slider locked at 0 (no compliance). The jaw force is read from that joint's
# reaction, the force the jaw needs to stay put, as a load cell in the jaw would read it.
Z_TOP = HB + T  # the beam's top face
HOUSING_TOP = Z_TOP + HOUSING_H
CALIPER_MASS = 0.07  # the beam and head; the slider, jaw and display add about 0.06 kg (134 g in all)

# The wooden rest the caliper starts on: a block under the beam's middle, the head overhanging it.
REST_BLOCK = (0.040, 0.100, 0.030)  # x, y, z
REST_BLOCK_Y = -0.100  # its middle, in the caliper's local y
START_X, START_Y = 0.295, 0.250  # the caliper frame's origin at the start (x, y); its beam lies on the rest
START_Z = TOP + REST_BLOCK[2] - HB

JAW = Joint(  # the sliding jaw itself: locked rigid to the slider; its joint reaction is the jaw force
    "jaw", kind="slide", axis="Y", anchor=(-JAW_L / 2, MF0 - JAW_T / 2, HB + T / 2), lower=0.0, upper=0.0,
    initial=0.0, stiffness=0.0, damping=0.0, target=0.0, mass=0.01, parent="slider",
    boxes=[((-JAW_L / 2, MF0 - JAW_T / 2, HB + T / 2), (JAW_L, JAW_T, T), STEEL)],
)

# Live display on the slider: four 7-segment digits 14 mm tall (their tops away from the robots, so they read upright
# from the robots' side), a decimal point after the second. Each segment is a box on its own sprung slide riding on the
# slider, driven up flush with the screen when lit and down into the housing when not.
DISP_X, DISP_Y = 0.017, MF0 - 0.042  # screen centre (local, slider at its start)
SCREEN_SIZE = (0.020, 0.050, 0.0005)
FACE_Z = HOUSING_TOP + SCREEN_SIZE[2]
DIG_H, DIG_W, SEG_T, DIG_PITCH = 0.014, 0.008, 0.0016, 0.0105
SEG_SUNK = -0.003
SEG_Z = FACE_Z + 0.0004 - 0.0005  # centre of a lit segment: its top 0.4 mm proud of the screen
SEGS = "abcdefg"
GLYPHS = {"0": "abcdef", "1": "bc", "2": "abdeg", "3": "abcdg", "4": "bcfg", "5": "acdfg", "6": "acdefg",
          "7": "abc", "8": "abcdefg", "9": "abcdfg", "-": "g", " ": ""}


def _digit_y(i: int) -> float:
    return DISP_Y + (1.5 - i) * DIG_PITCH  # digit 0 (tens) on the left


def _segment(i: int, s: str):
    """(centre xy, size xy) of segment s of digit i, in the caliper frame with the slider at its start."""
    xc, yc = DISP_X, _digit_y(i)
    hl, vl = DIG_W - SEG_T - 0.0006, DIG_H / 2 - SEG_T / 2 - 0.0006
    side = DIG_W / 2 - SEG_T / 2
    return {
        "a": ((xc + DIG_H / 2 - SEG_T / 2, yc), (SEG_T, hl)),
        "b": ((xc + DIG_H / 4, yc - side), (vl, SEG_T)),
        "c": ((xc - DIG_H / 4, yc - side), (vl, SEG_T)),
        "d": ((xc - DIG_H / 2 + SEG_T / 2, yc), (SEG_T, hl)),
        "e": ((xc - DIG_H / 4, yc + side), (vl, SEG_T)),
        "f": ((xc + DIG_H / 4, yc + side), (vl, SEG_T)),
        "g": ((xc, yc), (SEG_T, hl)),
    }[s]


def display_text(mm: float) -> str:
    """The four characters the display shows for a gap (mm): tens (blank under 10), units, tenths, hundredths."""
    v = min(max(round(float(mm), 2), 0.0), 99.99)
    s = f"{v:5.2f}"
    return s[0] + s[1] + s[3] + s[4]


def _lit(text: str) -> list[float]:
    return [0.0 if s in GLYPHS[ch] else SEG_SUNK for ch in text for s in SEGS]


# Each segment is built where it sits at the start, with the opening gap showing, so the very first rendered frames
# already read it (the renderer takes about a second to pick up joint positions set at reset). A segment joint's
# position is its offset from that start height.
_START = _lit(display_text(GAP0 * 1000))
SEGMENT_JOINTS = []
for _i in range(4):
    for _k, _s in enumerate(SEGS):
        (_cx, _cy), (_sx, _sy) = _segment(_i, _s)
        _o = _START[7 * _i + _k]
        SEGMENT_JOINTS.append(Joint(
            f"seg{_i}{_s}", kind="slide", axis="Z", anchor=(_cx, _cy, SEG_Z + _o), lower=SEG_SUNK - _o, upper=-_o,
            initial=0.0, target=0.0, stiffness=200.0, damping=2.0, mass=0.0005, parent="slider",
            boxes=[((_cx, _cy, SEG_Z + _o), (_sx, _sy, 0.001), DIGIT)],
        ))


def _seg_targets(text: str) -> list[float]:
    return [q - o for q, o in zip(_lit(text), _START)]


_dot = (DISP_X - DIG_H / 2 + SEG_T / 2, (_digit_y(1) + _digit_y(2)) / 2)

SLIDER = Joint(  # the slider: housing, screen, buttons and thumb rest. Positive closes the jaws.
    "slider", kind="slide", axis="Y", anchor=(DISP_X, DISP_Y, Z_TOP), lower=GAP0 - GAP_MAX, upper=GAP0, initial=0.0,
    stiffness=0.0, damping=SLIDE_DAMP, friction=SLIDE_FRICTION, target=0.0, mass=0.03,
    boxes=[
        ((sum(HOUSING_X) / 2, MF0 + sum(HOUSING_Y) / 2, Z_TOP + HOUSING_H / 2),
         (HOUSING_X[1] - HOUSING_X[0], HOUSING_Y[1] - HOUSING_Y[0], HOUSING_H), HOUSING_C),
        ((-0.002, MF0 - 0.005, HB + (T + HOUSING_H) / 2), (0.004, 0.006, T + HOUSING_H), HOUSING_C),  # jaw root
        ((DISP_X, DISP_Y, HOUSING_TOP + SCREEN_SIZE[2] / 2), SCREEN_SIZE, SCREEN),
        ((_dot[0], _dot[1], SEG_Z), (SEG_T, SEG_T, 0.001), DIGIT),  # decimal point (always shown)
        *[((0.031, MF0 + dy, HOUSING_TOP + 0.0005), (0.006, 0.008, 0.001), BUTTON if k == 0 else BLACK)
          for k, dy in enumerate((-0.026, -0.058))],  # ZERO and ON/OFF buttons (not used)
        ((sum(REST_X) / 2, MF0 - REST_OFF + REST_T / 2, Z_TOP + REST_H / 2), (REST_X[1] - REST_X[0], REST_T, REST_H),
         BLACK),  # thumb rest
    ],
)

CALIPER = part(
    "caliper", pos=(START_X, START_Y, START_Z), mass=CALIPER_MASS, friction=0.5,
    boxes=[
        ((BEAM_W / 2, sum(BEAM_Y) / 2, HB + T / 2), (BEAM_W, BEAM_Y[1] - BEAM_Y[0], T), STEEL),  # beam
        ((-JAW_L / 2, JAW_T / 2, HB + T / 2), (JAW_L, JAW_T, T), STEEL),  # fixed jaw
        ((BEAM_W / 2, BEAM_Y[1] - 0.006, Z_TOP + 0.0005), (BEAM_W - 0.004, 0.008, 0.001), BLACK),  # name plate
    ],
    joints=[SLIDER, JAW, *SEGMENT_JOINTS],
)
labkit.TORSIONAL_PATCH["caliper"] = 0.008  # the pads on the beam's 16 mm-wide edges

REST = part("rest", pos=(START_X + BEAM_W / 2, START_Y + REST_BLOCK_Y, TOP), fixed=True, friction=0.6,
            boxes=[((0.0, 0.0, REST_BLOCK[2] / 2), REST_BLOCK, WOOD)])

PIN = part("pin", pos=(*PIN_XY, TOP), mass=0.046, friction=0.5,
           cylinders=[((0.0, 0.0, PIN_H / 2), PIN_R, PIN_H, ALU)])

OBJECTS = [REST, CALIPER, PIN]


# -- the display: shows the slider's position as the jaw gap ---------------------------------------------------------
def reading_mm(j: dict) -> float:
    """What the display shows (mm, to 0.01): the gap the slider's position gives, as the caliper's encoder reads it."""
    return round((GAP0 - j["slider"]) * 1000.0, 2)


def jaw_gap(j: dict) -> float:
    """The gap between the measuring faces (m): the jaw is rigid on the slider."""
    return GAP0 - j["slider"]


def jaw_force(j: dict = None) -> float:
    """N with which the sliding jaw presses on whatever is between the jaws: the reaction in its locked joint along the
    jaws' axis, averaged over the last half second (read every step in _watch)."""
    h = _trial["force_hist"]
    return float(np.mean(h)) if h else 0.0


_trial = {"text": display_text(GAP0 * 1000), "reading": GAP0 * 1000, "pin": None, "cal": None, "t": 0.0, "pts": [],
          "force_hist": [], "meas": None}
_ids: dict = {}
_DEBUG = bool(__import__("os").environ.get("CAL_DEBUG"))


@on_reset
def _clear(env):
    _trial.update(text=display_text(GAP0 * 1000), reading=GAP0 * 1000, pin=None, cal=None, t=0.0, force_hist=[],
                  pts=[], meas=None)
    _ids.pop("set", None)
    _demo_reset()


def _tcp(env, arm: str) -> np.ndarray:
    from yam_arena.constants import TCP_OFFSET

    base = env.unwrapped
    robot = base.scene[arm]
    i = next(k for k, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
    g = labkit._np(robot.data.body_link_pose_w)[0][i]
    origin = labkit._np(base.scene.env_origins[0])
    q = g[3:7]
    x, y, z, w = q  # xyzw, as labkit reads poses
    r = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                  [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                  [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
    return g[:3] - origin + r @ np.array(TCP_OFFSET)


def _jaw_reaction(env, cal) -> float:
    """|N| of the reaction in the sliding jaw's locked joint along the jaws' axis (the caliper's y)."""
    import warp as wp

    asset = env.unwrapped.scene["caliper"]
    if "jaw_link" not in _ids:
        names = list(asset.backend_body_names) if hasattr(asset, "backend_body_names") else list(asset.body_names)
        _ids["jaw_link"] = next(k for k, n in enumerate(names) if n.split("/")[-1] == "jaw")
    f = wp.to_torch(asset.root_view.get_link_incoming_joint_force())[0, _ids["jaw_link"], :3]
    f = np.asarray(f.detach().cpu().numpy(), float)
    return abs(float(cal.rotation()[:, 1] @ f))


FINGER_X0 = 0.0445119  # each finger link's origin, across the fingers, with the pads closed (yam URDF joints 7 and 8)


def _tips(env, arm: str) -> list[np.ndarray]:
    """The two fingertips (pad tips) of a gripper, each from its own finger: the pads' closed point is 13 mm past
    the TCP (yam_arena PAD_TIP_PAST_TCP); joint 7 (tip_left) moves its pad toward -x and joint 8 (tip_right) toward
    +x along the gripper's x, each by its own travel, read from that finger link's pose (the jaws stop independently).
    """
    from yam_arena.constants import PAD_TIP_PAST_TCP, TCP_OFFSET

    base = env.unwrapped
    robot = base.scene[arm]
    origin = labkit._np(base.scene.env_origins[0])
    poses = labkit._np(robot.data.body_link_pose_w)[0]
    names = [n.split("/")[-1] for n in robot.body_names]
    g = poses[names.index("gripper")]
    x, y, z, w = g[3:7]  # xyzw, as labkit reads poses
    r = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                  [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                  [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
    mid = g[:3] - origin + r @ (np.array(TCP_OFFSET) + np.array([0.0, 0.0, PAD_TIP_PAST_TCP]))  # pads closed here
    across = r[:, 0]  # the fingers close along the gripper's x
    left = float((poses[names.index("tip_left")][:3] - g[:3]) @ across) - FINGER_X0  # joint 7: <= 0
    right = float((poses[names.index("tip_right")][:3] - g[:3]) @ across) + FINGER_X0  # joint 8: >= 0
    return [mid + across * left, mid + across * right]


@on_step
def _watch(env):
    import torch

    cal = labkit._part_state(env, labkit._SPECS["caliper"])
    j = cal.joints
    text = display_text(reading_mm(j))
    _trial["reading"] = reading_mm(j)
    _trial["t"] += float(getattr(env.unwrapped, "step_dt", 0.05))
    _trial["pin"] = labkit._part_state(env, labkit._SPECS["pin"])
    _trial["cal"] = cal
    _trial["right_tcp"], _trial["left_tcp"] = _tcp(env, "right_robot"), _tcp(env, "left_robot")
    _trial["right_tips"], _trial["left_tips"] = _tips(env, "right_robot"), _tips(env, "left_robot")
    _trial["force_hist"] = (_trial["force_hist"] + [_jaw_reaction(env, cal)])[-10:]
    _trial["pts"] = (_trial["pts"] + [_surface_points(cal, _trial["pin"])])[-STILL_N:]
    if held(cal) and between_jaws(_trial["pin"], cal) and still() and free_hand_clear(cal, _trial["pin"]):
        _trial["meas"] = {"t": round(_trial["t"], 2), "reading": round(_trial["reading"], 2),
                          "score": _tier(cal, _trial["pin"])}
    _demo_follow()
    if _DEBUG and int(round(_trial["t"] / 0.05)) % 10 == 0:
        pin, r = _trial["pin"], cal.rotation()
        print(f"[cal] t={_trial['t']:5.1f}s read={_trial['reading']:6.2f} force={jaw_force(j):5.2f}N "
              f"low={(float(cal.low[2]) - TOP) * 1000:5.1f}mm tilt=({math.degrees(math.atan2(r[2, 1], r[2, 2])):5.1f},"
              f"{math.degrees(math.atan2(-r[2, 0], r[2, 2])):5.1f}) "
              f"cal={np.round(cal.pos * 1000).astype(int).tolist()} "
              f"pin={np.round(pin.pos * 1000).astype(int).tolist()} pin_tilt={pin.tilt_deg():4.1f} "
              f"between={between_jaws(pin, cal)} held={held(cal)} sq={off_square_deg(pin, cal):4.1f} "
              f"trim=({_hold['dz'] * 1000:4.1f}mm,{math.degrees(_hold['rx']):4.1f},{math.degrees(_hold['ry']):4.1f}) "
              f"push={_push['dy'] * 1000:4.1f}mm done={_push['done']} score={score({'caliper': cal, 'pin': pin})} "
              f"rtcp_y={(float(_trial['right_tcp'][1]) - float(_push_base()[1])) * 1000:5.1f}mm "
              f"ltcp_y={float(_trial['left_tcp'][1]) * 1000:6.1f}mm "
              f"seat={_pin_seated_mm():.2f} near={_jaw_to_pin_mm():.2f} "
              f"rtips_gap={float(np.linalg.norm(_trial['right_tips'][0] - _trial['right_tips'][1])) * 1000:.0f}mm "
              f"free_mm={free_hand_mm(cal, pin):.0f} free_clear={free_hand_clear(cal, pin)} "
              f"meas={_trial['meas']} on_rest={on_rest(cal)} hands_mm={hands_mm(cal, pin):.0f}",
              file=sys.stderr, flush=True)
    asset = env.unwrapped.scene["caliper"]
    if "segments" not in _ids:
        names = list(asset.joint_names)
        _ids["segments"] = [names.index(s.joint_name) for s in SEGMENT_JOINTS]
    if text != _trial["text"] or "set" not in _ids:
        targets = torch.tensor([_seg_targets(text)], device=asset.device)
        asset.set_joint_position_target(targets, joint_ids=_ids["segments"])
        _trial["text"], _ids["set"] = text, True


# -- scoring ---------------------------------------------------------------------------------------------------------
JAW_MID = HB + T / 2  # the jaws' mid-plane (caliper frame z)
END_MARGIN = 0.002
UPRIGHT_MIN = math.cos(math.radians(30.0))
HELD_CLEAR = 0.003  # every part of the caliper this far above the bench and the rest: held, not resting
TIP_REACH = 0.020  # a gripper's fingertip point (TCP) this close to the beam: the caliper is in a hand
TIERS = ((0.05, 1.0, 0.3, 1.0), (0.20, 3.0, 2.0, 0.5))  # (mm off the true size, deg off square, N of jaw force, score)
NEAR_MM = 1.0
STILL_N, STILL_MM = 21, 1.0  # every positive score: the caliper's and pin's surface points moved under 1 mm over 1 s
FREE_CLEAR = 0.08  # and the hand not holding the caliper 80 mm or more from the caliper and the pin


def _crossing(pin, cal):
    """Where the pin's axis crosses the jaws' mid-plane, in the caliper frame, and how far along the axis from the pin's
    bottom (m); None if the pin is not upright within 30 degrees of the jaws' normal."""
    rc = cal.rotation()
    axis = rc.T @ pin.rotation()[:, 2]
    if axis[2] < UPRIGHT_MIN:
        return None
    base = cal.to_local(pin.pos)
    s = (JAW_MID - base[2]) / axis[2]
    return base + s * axis, float(s)


def held(cal) -> bool:
    """The caliper is in a hand and resting on nothing: a gripper's fingertip point (TCP) within 20 mm of its beam, and
    its lowest point 3 mm or more above the bench and, where it is over the rest, 3 mm or more above the rest."""
    if float(cal.low[2]) < TOP + HELD_CLEAR:
        return False
    rx, ry = START_X + BEAM_W / 2, START_Y + REST_BLOCK_Y
    over_rest = (cal.low[0] < rx + REST_BLOCK[0] / 2 and cal.high[0] > rx - REST_BLOCK[0] / 2
                 and cal.low[1] < ry + REST_BLOCK[1] / 2 and cal.high[1] > ry - REST_BLOCK[1] / 2)
    if over_rest and float(cal.low[2]) < TOP + REST_BLOCK[2] + HELD_CLEAR:
        return False
    for tip in (_trial.get("left_tcp"), _trial.get("right_tcp")):
        if tip is None:
            continue
        q = cal.to_local(tip)
        if (-TIP_REACH <= q[0] <= BEAM_W + TIP_REACH and BEAM_Y[0] <= q[1] <= BEAM_Y[1]
                and HB - TIP_REACH <= q[2] <= Z_TOP + TIP_REACH):
            return True
    return False


def between_jaws(pin, cal) -> bool:
    """The pin's axis crosses the jaws' mid-plane between their two measuring faces, within the jaws' 45 mm length,
    and the jaws meet its side (the crossing at least 2 mm from either end of the pin)."""
    c = _crossing(pin, cal)
    if c is None:
        return False
    p, s = c
    return (-JAW_L + END_MARGIN <= p[0] <= 0.0 and -jaw_gap(cal.joints) <= p[1] <= 0.0
            and END_MARGIN <= s <= PIN_H - END_MARGIN)


def off_square_deg(pin, cal) -> float:
    """Angle (deg) between the pin's axis and the jaw faces: its lean towards or away from the fixed jaw."""
    axis = cal.rotation().T @ pin.rotation()[:, 2]
    return float(np.degrees(np.arcsin(min(1.0, abs(axis[1])))))


def _surface_points(cal, pin) -> np.ndarray:
    """Points on the caliper (its beam ends, the fixed jaw's tip, and the sliding jaw's tip and the thumb rest, which
    move with the slider) and on the pin (its top and bottom rims), world, m."""
    dy = float(cal.joints["slider"])  # the slider's travel (closing is +y)
    local = [(BEAM_W / 2, BEAM_Y[0], HB), (BEAM_W / 2, BEAM_Y[1], HB), (-JAW_L, 0.0, HB), (-JAW_L, MF0 + dy, HB),
             (sum(REST_X) / 2, MF0 - REST_OFF + dy, Z_TOP + REST_H)]
    pts = [cal.to_world(q) for q in local]
    for z in (0.0, PIN_H):
        for dx, dy in ((PIN_R, 0.0), (0.0, PIN_R), (-PIN_R, 0.0), (0.0, -PIN_R)):
            pts.append(pin.to_world((dx, dy, z)))
    return np.array(pts)


def still() -> bool:
    """The caliper's and pin's surface points moved under 1 mm (3D) over the last second (21 samples at 20 Hz)."""
    h = _trial["pts"]
    if len(h) < STILL_N:
        return False
    return max(float(np.linalg.norm(q - h[-1], axis=1).max()) for q in h) < STILL_MM / 1000.0


def _clearance(cal, pin, p) -> float:
    """Distance (m) from point p to the caliper (as a box round its beam and jaws) and to the pin, whichever is less."""
    q = cal.to_local(p)
    lo = np.array([-JAW_L, BEAM_Y[0], HB])
    hi = np.array([HOUSING_X[1], BEAM_Y[1], Z_TOP + REST_H])
    d_cal = float(np.linalg.norm(np.maximum(0.0, np.maximum(lo - q, q - hi))))
    r = pin.to_local(p)
    radial = max(0.0, math.hypot(float(r[0]), float(r[1])) - PIN_R)
    axial = max(0.0, -float(r[2]), float(r[2]) - PIN_H)
    return min(d_cal, math.hypot(radial, axial))


def free_hand_mm(cal, pin) -> float:
    """How far the nearer of the two actual fingertips of the hand not holding the caliper (the one whose fingertip
    point is farther from the beam) is from the caliper and the pin, in mm."""
    hands = [(_trial.get(f"{a}_tcp"), _trial.get(f"{a}_tips")) for a in ("left", "right")]
    if any(t is None or tips is None for t, tips in hands):
        return 0.0
    _, tips = max(hands, key=lambda h: _clearance(cal, pin, h[0]))
    return min(_clearance(cal, pin, q) for q in tips) * 1000


def free_hand_clear(cal, pin) -> bool:
    """The free hand has both actual fingertips 80 mm or more from the caliper and the pin (see free_hand_mm)."""
    return free_hand_mm(cal, pin) >= FREE_CLEAR * 1000


def _tier(cal, pin) -> float:
    """The measurement's score while held on the pin: 0.1; 0.25 if the display reads within 1.00 mm of the pin's
    23.40 mm; 0.5 within 0.20 mm, the pin square to the jaws within 3 degrees and the jaw pressing on it with at most
    2 N; 1.0 within 0.05 mm, square within 1 degree, at most 0.3 N (just touching)."""
    j = cal.joints
    err = abs(reading_mm(j) - PIN_D * 1000.0)
    sq, f = off_square_deg(pin, cal), jaw_force(j)
    for mm, deg, newton, s in TIERS:
        if err <= mm + 1e-6 and sq <= deg and f <= newton:
            return s
    return 0.25 if err <= NEAR_MM + 1e-6 else 0.1


REST_FLAT_DEG = 5.0  # back on the rest: the beam within 5 degrees of flat
REST_GAP = 0.003  # and its underside within 3 mm of the rest's top, over the rest


def on_rest(cal) -> bool:
    """The caliper lies back on its rest: over the rest's top, its lowest point within 3 mm of it, flat within
    5 degrees, and nothing of it within 3 mm of the table."""
    rx, ry = START_X + BEAM_W / 2, START_Y + REST_BLOCK_Y
    over = (cal.low[0] < rx + REST_BLOCK[0] / 2 and cal.high[0] > rx - REST_BLOCK[0] / 2
            and cal.low[1] < ry + REST_BLOCK[1] / 2 and cal.high[1] > ry - REST_BLOCK[1] / 2)
    top = TOP + REST_BLOCK[2]
    flat = float(cal.rotation()[2, 2]) >= math.cos(math.radians(REST_FLAT_DEG))
    return bool(over and abs(float(cal.low[2]) - top) <= REST_GAP and flat and float(cal.low[2]) > TOP + REST_GAP)


def hands_mm(cal, pin) -> float:
    """How far the nearest of all four actual fingertips (both hands) is from the caliper and the pin, mm."""
    tips = list(_trial.get("left_tips") or []) + list(_trial.get("right_tips") or [])
    if len(tips) < 4:
        return 0.0
    return min(_clearance(cal, pin, q) for q in tips) * 1000


def score(objects):
    """The measurement taken while the caliper was held on the pin (latched each step it was valid: held, the pin
    between the jaws, everything still for 1 s, the free hand 80 mm clear; scored by _tier; the most recent one
    counts), kept only if the trial ends with the caliper back on its rest, all four fingertips 80 mm or more from
    the caliper and the pin, and the caliper and the pin still for the last second. Otherwise 0."""
    cal, pin = objects["caliper"], objects["pin"]
    m = _trial.get("meas")
    if m is None or not (on_rest(cal) and still() and hands_mm(cal, pin) >= FREE_CLEAR * 1000):
        return 0.0
    return float(m["score"])


CHECK = __import__("os").environ.get("CAL_CHECK", "")  # "letgo": the shortcut check (see demo)
SUCCESS_HOLD_S = 1e9  # no early end on success: the trial runs on, so the hands leaving and the stillness are seen


def success(objects):
    return score(objects) == 1.0


def distance(objects):
    """Display error (m) once the pin is between the jaws of the held caliper; before that, how far the fixed jaw's
    face is from beside the pin."""
    cal, pin = objects["caliper"], objects["pin"]
    if held(cal) and between_jaws(pin, cal):
        return abs(reading_mm(cal.joints) / 1000.0 - PIN_D)
    want = pin.pos + np.array([0.025, PIN_R, JAW_MID - HB])  # roughly where the fixed jaw's root should be
    return float(np.linalg.norm(cal.to_world((0.0, 0.0, JAW_MID)) - want)) + GAP0


# -- scripted demo ---------------------------------------------------------------------------------------------------
# Left arm: takes the caliper by its head (straight down, fingers closing across the beam's 16 mm width, 25 mm beyond
# the fixed jaw), lifts it off its rest and carries it, level and 50 mm up, until its open jaws are either side of the
# pin, 20 mm in from the jaw tips, 12 mm above the bench, then holds it there. The target is placed from the pin's seen
# position and trimmed from the caliper's seen pose: under the caliper's weight the arm sags and tips it (about 16
# degrees at the wrist), so the hand's tilt (from the lift on), height and sideways place (the fixed jaw against
# the pin's side) are corrected until the caliper sits level where intended. Once the jaws are on the pin the hand
# stays put.
# Right arm: comes down behind the thumb rest with closed fingers, leaning 30 degrees towards the fixed jaw (``Orient``
# lets a Waypoint's ``yaw`` carry the full rotation), starting just short of the thumb rest where the display says it
# is, and pushes it to the left at 3 mm/s, 1 mm/s once the sliding jaw is seen within 3 mm of the pin. It stops when
# the jaw is seen on the pin, the pin seen against the fixed jaw and the reading has not fallen for 0.5 s: the finger
# is then held where it actually is (it lags its target by several mm), and after a moment backs slowly off. Everything
# the demo reads is what a camera sees: the pin's and caliper's poses, the fingers and the display.

_prev_action7 = _demo.ArmPlan.action7
if not getattr(_prev_action7, "_clock", False):
    def _action7(self, t, _prev=_prev_action7):
        _demo._plan_clock = float(t)  # the plan's time, for waypoints that move with the trial
        return _prev(self, t)

    _action7._clock = True
    _demo.ArmPlan.action7 = _action7


class Orient(float):
    """A Waypoint ``yaw`` that carries a full TCP rotation (``rot``, read every control step)."""

    def __new__(cls, rot):
        obj = float.__new__(cls, 0.0)
        obj._rot = np.asarray(rot, float)
        return obj

    @property
    def rot(self):
        return self._rot


_down_grip = _demo.grip_rotation
if not getattr(_down_grip, "_orient", False):
    def _grip_rotation(yaw=0.0):
        return yaw.rot if isinstance(yaw, Orient) else _down_grip(yaw)

    _grip_rotation._orient = True
    _demo.grip_rotation = _grip_rotation

# -- the holding hand -------------------------------------------------------------------------------------------------
HOLD_ROT = _down_grip(-math.pi / 2)  # left hand straight down, fingers closing along x
TIP_BELOW_TCP = 0.013  # the pad tips are this far below the TCP
GRIP_LOCAL = np.array([BEAM_W / 2, 0.025, HB - 0.002 + TIP_BELOW_TCP])  # TCP in the caliper frame: pad tips 2 mm below
BEAM_WIDE = -0.015  # pads 30 mm apart round the 16 mm beam
MEASURE_UNDER = 0.012  # the jaws' underside this far above the bench while measuring
FIXED_CLEAR = 0.0002  # the fixed jaw against the pin's side (0.2 mm), trimmed from what is seen, before closing
PIN_IN = 0.025  # the pin's axis this far out from the beam's front edge (20 mm in from the jaw tips)
K_POS, K_ROT = 0.15, 0.2  # trim gains per control step
MAX_DPOS, MAX_DROT = 0.0003, math.radians(0.5)
MAX_POS, MAX_ROT = 0.010, math.radians(22.0)

_hold = {"on": False, "tilt_on": False, "dy": 0.0, "dz": 0.0, "rx": 0.0, "ry": 0.0, "pin0": None}


def _measure_origin() -> np.ndarray:
    """Where the caliper frame's origin should be while measuring, from the pin's seen position (the start layout before
    the pin is seen)."""
    p = _hold["pin0"] if _hold["pin0"] is not None else np.array([*PIN_XY, TOP])
    return np.array([p[0] + PIN_IN, p[1] + PIN_R + FIXED_CLEAR, TOP + MEASURE_UNDER - HB])


def _trim_rot() -> np.ndarray:
    return axis_angle_matrix(np.array([1.0, 0.0, 0.0]), _hold["rx"]) @ \
        axis_angle_matrix(np.array([0.0, 1.0, 0.0]), _hold["ry"])


class _HoldPos:
    """The holding hand's TCP while measuring: the grip point of a caliper at the measuring pose, plus the trims."""

    def __init__(self, dz=0.0):
        self.dz = dz

    def __array__(self, dtype=None, copy=None):
        p = _measure_origin() + GRIP_LOCAL + [0.0, _hold["dy"], self.dz + _hold["dz"]]
        return np.asarray(p, dtype=dtype or float)


class _HoldRot(Orient):
    def __new__(cls):
        return Orient.__new__(cls, HOLD_ROT)

    @property
    def rot(self):
        return _trim_rot() @ HOLD_ROT


def _follow_hold():
    """Trim the holding hand from the caliper's seen pose: its jaws' height and its tilt about x and y."""
    cal, pin = _trial["cal"], _trial["pin"]
    if pin is not None and _hold["pin0"] is None:
        _hold["pin0"] = np.array(pin.pos, float)
    if not _hold["tilt_on"] or cal is None or _push["done"]:
        return  # once the jaws are on the pin the hand stays put: further trims would rub the caliper on the pin
    r = cal.rotation()
    if _hold["on"]:  # height only once lowered into place
        ez = float(cal.pos[2]) - float(_measure_origin()[2])
        _hold["dz"] = float(np.clip(_hold["dz"] - np.clip(K_POS * ez, -MAX_DPOS, MAX_DPOS), -MAX_POS, MAX_POS))
        if not _push["on"] and pin is not None:  # the fixed jaw FIXED_CLEAR from the pin's seen side, until the push
            ey = float(cal.pos[1]) - (float(pin.pos[1]) + PIN_R + FIXED_CLEAR)
            _hold["dy"] = float(np.clip(_hold["dy"] - np.clip(K_POS * ey, -MAX_DPOS, MAX_DPOS), -MAX_POS, MAX_POS))
    tilt_x = math.atan2(float(r[2, 1]), float(r[2, 2]))  # the caliper turned about x (its beam tipping)
    tilt_y = math.atan2(-float(r[2, 0]), float(r[2, 2]))  # turned about y (its jaws tipping)
    _hold["rx"] = float(np.clip(_hold["rx"] - np.clip(K_ROT * tilt_x, -MAX_DROT, MAX_DROT), -MAX_ROT, MAX_ROT))
    _hold["ry"] = float(np.clip(_hold["ry"] - np.clip(K_ROT * tilt_y, -MAX_DROT, MAX_DROT), -MAX_ROT, MAX_ROT))


# -- the pushing hand -------------------------------------------------------------------------------------------------
PUSH_TILT = math.radians(30.0)
PUSH_ROT = Orient(axis_angle_matrix(np.array([1.0, 0.0, 0.0]), PUSH_TILT) @ _down_grip(0.0))
FINGER_FACE = 0.0092  # the leaning fingers meet the thumb rest's face with the TCP this far behind it (from the hulls)
FINGER_LOW = 0.0123  # and their lowest point is this far below the TCP
PUSH_V = 0.003  # m/s
APPROACH = 0.004  # the finger starts this far short of the thumb rest
PUSH_MAX = (GAP0 - PIN_D) + APPROACH + 0.012  # at most this far from the start (the arm lags about 6 mm)
PUSH_V_NEAR, NEAR_MM = 0.001, 3.0  # m/s once the jaw is seen within 3 mm of the pin
PUSH_V_TOUCH, TOUCH_ZONE_MM = 0.0003, 1.0  # and 0.3 mm/s for the last 1 mm: the rigid jaw meets the pin gently
TOUCH_MM, STALL_S, STALL_MM = 1.0, 0.5, 0.01  # seen within 1 mm and the reading flat (under 0.01 mm) for 0.5 s: on it
SEATED_MM = 0.5  # and the pin seen within 0.5 mm of the fixed jaw
BACK_OFF = 0.005
RIGHT_PARK = (0.15, -0.25, TOP + 0.14)
LEFT_PARK = (0.12, 0.33, TOP + 0.20)  # the holding hand's park after setting the caliper down: 80 mm clear of it

_push = {"on": False, "t0": None, "tl": 0.0, "dy": 0.0, "hist": [], "done": False, "gap0": None}


def _push_base() -> np.ndarray:
    """The pushing finger's start: APPROACH short of the thumb rest where the display says it is (the slider may have
    moved while the caliper was carried), its fingers 5 mm above the beam (measuring pose)."""
    o = _measure_origin()
    gap = _push["gap0"] if _push["gap0"] is not None else _trial["reading"] / 1000.0
    rest = -gap - REST_OFF  # the thumb rest's pushed face, in the caliper frame
    return np.array([o[0] + sum(REST_X) / 2, o[1] + rest - FINGER_FACE - APPROACH,
                     o[2] + Z_TOP + 0.005 + FINGER_LOW])


def _jaw_to_pin_mm() -> float:
    """How far (mm) the sliding jaw's face is from the pin's near side, as a camera sees them: the pin's seen centre in
    the caliper frame and its seen width, against the gap the display shows."""
    cal, pin = _trial["cal"], _trial["pin"]
    if cal is None or pin is None:
        return 1e9
    y = float(cal.to_local(pin.pos)[1])  # the pin's centre across the jaws (the fixed jaw's face is at 0)
    return _trial["reading"] - (-y + PIN_R) * 1000.0


def _pin_seated_mm() -> float:
    """How far (mm) the pin's far side is from the fixed jaw's face, as seen."""
    cal, pin = _trial["cal"], _trial["pin"]
    if cal is None or pin is None:
        return 1e9
    return (-float(cal.to_local(pin.pos)[1]) - PIN_R) * 1000.0


def _release_at_actual():
    """Once the jaws are on the pin: hold the finger where it actually is (it lags its target by several mm, and would
    otherwise go on pressing), so the push stops at once."""
    tip = _trial.get("right_tcp")
    if tip is not None:
        _push["dy"] = float(tip[1]) - float(_push_base()[1])


def _follow_push():
    """Advance the finger at PUSH_V, slowing to PUSH_V_NEAR once the sliding jaw is seen within NEAR_MM of the pin; stop
    once the jaw is seen within TOUCH_MM of the pin and the reading has not fallen for STALL_S."""
    p = _push
    if not p["on"] or p["done"]:
        return
    t = _trial["t"]
    if p["t0"] is None:
        p.update(t0=t, tl=t, dy=0.0, hist=[], gap0=_trial["reading"] / 1000.0)
    near = _jaw_to_pin_mm()
    v = PUSH_V_TOUCH if near < TOUCH_ZONE_MM else PUSH_V_NEAR if near < NEAR_MM else PUSH_V
    p["dy"] = min(PUSH_MAX, p["dy"] + v * (t - p["tl"]))
    p["tl"] = t
    p["hist"].append((t, _trial["reading"]))
    p["hist"] = [h for h in p["hist"] if t - h[0] <= STALL_S + 1e-6]
    flat = t - p["hist"][0][0] >= STALL_S - 0.06 and p["hist"][0][1] - _trial["reading"] < STALL_MM
    if near < TOUCH_MM and _pin_seated_mm() < SEATED_MM and flat:
        p["done"] = True
        _release_at_actual()
    if p["dy"] >= PUSH_MAX:
        p["done"] = True


class _PushPos:
    def __init__(self, dy=0.0, dz=0.0):
        self.dy, self.dz = dy, dz

    def __array__(self, dtype=None, copy=None):
        b = _push_base()
        return np.asarray((b[0], b[1] + _push["dy"] + self.dy, b[2] + self.dz), dtype=dtype or float)


def _demo_reset():
    _hold.update(on=False, tilt_on=False, dy=0.0, dz=0.0, rx=0.0, ry=0.0, pin0=None)
    _push.update(on=False, t0=None, dy=0.0, hist=[], done=False, gap0=None)


def _demo_follow():
    _follow_hold()
    _follow_push()
    t = float(getattr(_demo, "_plan_clock", _trial["t"]))
    _hold["on"] = t >= _T["hold"]
    _hold["tilt_on"] = t >= _T["lift"]
    _push["on"] = t >= _T["push"]


_T = {"lift": 1e9, "hold": 1e9, "push": 1e9}


def _dur(wps):
    return sum(w.duration for w in wps)


def demo():
    _demo_reset()
    grip0 = np.array([START_X, START_Y, START_Z]) + GRIP_LOCAL
    lift = grip0 + [0.0, 0.0, 0.030]
    left = [
        Waypoint(2.5, tuple(grip0 + [0.0, 0.0, 0.060]), FINGER_OPEN, "above the caliper's head", Orient(HOLD_ROT)),
        Waypoint(1.2, tuple(grip0), BEAM_WIDE, "fingers either side of the beam, beyond the fixed jaw",
                 Orient(HOLD_ROT)),
        Waypoint(0.8, tuple(grip0), FINGER_CLOSED, "take hold of it", Orient(HOLD_ROT)),
    ]
    _T["lift"] = _dur(left)  # from the lift on, the hand's tilt is trimmed from the caliper's seen tilt
    left += [
        Waypoint(1.2, tuple(lift), FINGER_CLOSED, "lift it off the rest", _HoldRot()),
        Waypoint(2.5, _HoldPos(dz=0.050), FINGER_CLOSED, "carry it level, jaws 50 mm up, either side of the pin",
                 _HoldRot()),
        Waypoint(1.5, _HoldPos(), FINGER_CLOSED, "lower it: the open jaws either side of the pin", _HoldRot()),
    ]
    _T["hold"] = _dur(left)  # trims run once it is lowered into place
    left.append(Waypoint(3.5, _HoldPos(), FINGER_CLOSED, "hold it level, the fixed jaw on the pin (trimmed as seen)",
                         _HoldRot()))
    t_held = _dur(left)
    b = _push_base()
    right = [Waypoint(2.5, (b[0], b[1], b[2] + 0.10), FINGER_CLOSED, "above, behind the thumb rest, leaning", PUSH_ROT)]
    right.append(Waypoint(max(0.3, t_held - 1.2 - _dur(right)), (b[0], b[1], b[2] + 0.10), FINGER_CLOSED,
                          "wait for the caliper to be in place", PUSH_ROT))
    right.append(Waypoint(1.2, _PushPos(), FINGER_CLOSED, "down behind the thumb rest", PUSH_ROT))
    _T["push"] = _dur(right)
    window = PUSH_MAX / PUSH_V + NEAR_MM / 1000.0 / PUSH_V_NEAR + TOUCH_ZONE_MM / 1000.0 / PUSH_V_TOUCH + 1.0
    right += [
        Waypoint(window, _PushPos(), FINGER_CLOSED, "close the jaw until the reading stops falling", PUSH_ROT),
        Waypoint(0.8, _PushPos(), FINGER_CLOSED, "stay on the thumb rest while the holding hand settles", PUSH_ROT),
        Waypoint(2.5, _PushPos(dy=-BACK_OFF), FINGER_CLOSED, "back off the thumb rest, slowly", PUSH_ROT),
        Waypoint(1.0, _PushPos(dy=-BACK_OFF, dz=0.04), FINGER_CLOSED, "up", PUSH_ROT),
        Waypoint(2.0, RIGHT_PARK, FINGER_OPEN, "park"),
    ]
    t_meas = _dur(right) + 3.0  # held on the pin, still, the free hand parked: the reading is taken
    if CHECK == "letgo":  # shortcut check: measure, then let go of the caliper (it must score 0)
        left.append(Waypoint(t_meas - _dur(left), _HoldPos(), FINGER_CLOSED, "hold it on the pin", _HoldRot()))
        left += [Waypoint(1.0, _HoldPos(), BEAM_WIDE, "let go of the caliper (check)", _HoldRot()),
                 Waypoint(2.0, _HoldPos(dz=0.060), BEAM_WIDE, "hand away (check)", _HoldRot()),
                 Waypoint(2.0, LEFT_PARK, FINGER_OPEN, "park (check)")]
    else:
        left.append(Waypoint(t_meas - _dur(left), _HoldPos(), FINGER_CLOSED, "hold the caliper on the pin",
                             _HoldRot()))
        left += [
            Waypoint(2.0, _HoldPos(dz=0.060), FINGER_CLOSED, "lift it straight up off the pin", Orient(HOLD_ROT)),
            Waypoint(2.5, tuple(lift + [0.0, 0.0, 0.030]), FINGER_CLOSED, "carry it back over its rest",
                     Orient(HOLD_ROT)),
            Waypoint(1.5, tuple(grip0), FINGER_CLOSED, "lay it on its rest", Orient(HOLD_ROT)),
            Waypoint(1.0, tuple(grip0), BEAM_WIDE, "let go", Orient(HOLD_ROT)),
            Waypoint(1.0, tuple(grip0 + [0.0, 0.0, 0.060]), FINGER_OPEN, "hand up", Orient(HOLD_ROT)),
            Waypoint(2.0, LEFT_PARK, FINGER_OPEN, "park"),
        ]
    t_end = _dur(left) + 3.0  # the caliper on its rest, both hands away, still
    left.append(Waypoint(t_end - _dur(left), LEFT_PARK, FINGER_OPEN, "rest"))
    right.append(Waypoint(t_end - _dur(right), RIGHT_PARK, FINGER_OPEN, "rest"))
    return {"left": left, "right": right}
