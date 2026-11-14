"""On the station table, a single carton-flow lane runs left to right in front of the robot, falling 3 degrees toward
the robot's right, where a front stop holds the bins. Four blue small-parts shelf bins (13.7 x 10.5 x 7.6 cm, 0.35 kg
each with a bag of bolts) stand in the lane nose to tail from the stop. The red-tagged reject is the second or the third
from the stop, drawn at random for each run (FLOW_TAG_SLOT=2 or 3 fixes it), so a fixed replay cannot skip finding the
tag. A red reject tote stands on the table behind the lane.

The job is to take the tagged bin out and put it in the reject tote, keeping the next bin up the lane within 2 cm of where it
was until the tagged bin is in the tote, and to leave the lane closed up: the three remaining bins down against the stop
and each other, nothing held.

The lane is a fixed body tilted 3 degrees. Its skate wheels are drawn but not simulated: the bed is one flat collider
with friction 0.03 (combine mode "min", see flowkit.py), below tan(3 degrees) = 0.052, so a free bin runs down at about
0.22 m/s^2, close to a bin on real low-resistance wheels. The bins are loose rigid bodies with their real mass. Every
threshold below appears once, with the same value in task.json and the README.
"""
from __future__ import annotations

import math
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np  # noqa: E402

import yamkit as kit  # noqa: E402
import guardkit as gk  # noqa: E402
import flowkit as fk  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
DT = 1.0 / 20  # control step, s
RUN_LIMIT_STEPS = 2400
INSTRUCTION = ("Put the red-tagged bin in the reject tote. Until it is in the tote, the next bin up the lane must "
               "not move more than 2 cm. At the end, the other three bins must be down against the front stop and each "
               "other, with nothing holding them.")

# -- the lane (lane frame: origin on the bed's surface at the front stop's face, in the lane's middle; u across the
# lane = world x, s up the lane, h square to the bed) -----------------------------------------------------------------
SLOPE = math.radians(3.0)  # real carton flow: 0.5 to 0.75 in. of fall per foot (2.4 to 3.6 degrees)
SN, CS = math.sin(SLOPE), math.cos(SLOPE)
Q_SLOPE = (math.sin(SLOPE / 2), 0.0, 0.0, math.cos(SLOPE / 2))  # turns +y up the slope (about world x)
LANE_XC = 0.260  # m: the lane's middle, world x
STOP_Y = -0.340  # m: world y of the front stop's face (the lane falls toward -y; set toward the right arm so it
# reaches the tagged bin in slot 3 at the 9 cm lift, checked on the real IK)
BED_Z0 = TOP + 0.040  # m: the wheels' tops at the stop, 4 cm above the table
HALF_IN = 0.060  # m: the guide rails' inner faces at u = +-6 cm (12 cm between them)
GUIDE_T, GUIDE_H = 0.007, 0.025  # guide rail thickness and height above the wheels
STOP_T, STOP_H = 0.012, 0.040  # front stop thickness and height above the wheels
S_END = 0.580  # m: the lane's loading end, 58 cm up the lane from the stop (four bins and 2.8 cm to spare)
MU_BED = 0.03  # the bed's friction (combine "min"): stands in for the skate wheels
O_LANE = np.array([LANE_XC, STOP_Y, BED_Z0])
E_S = np.array([0.0, CS, SN])  # up the lane
E_H = np.array([0.0, -SN, CS])  # square to the bed

# -- the bins: polypropylene small-parts shelf bins, 13.7 x 10.5 x 7.6 cm (5.4 x 4.1 x 3 in.), hopper front down-lane;
# bin frame: origin at the middle of the bottom face, x across, y along the bin (front = -y), z up --------------------
BIN_L, BIN_W, BIN_H = 0.137, 0.105, 0.076
WALL = 0.003  # wall and floor thickness
HOPPER_H = 0.045  # the lower hopper front wall
BIN_MASS = 0.35  # kg: 0.10 kg bin + 0.25 kg bag of 1/4-20 zinc hex bolts
LOAD = ((0.0, 0.0, WALL + 0.0125), (0.060, 0.100, 0.025))  # the bag of bolts: center, size (bin frame)
GAP0 = 0.001  # m: start gaps, stop to first bin and bin to bin
N_SLOTS = 4
S_SLOT = {k: GAP0 + BIN_L / 2 + (k - 1) * (BIN_L + GAP0) for k in range(1, N_SLOTS + 1)}  # slot middles, along the lane
TAG_SLOTS = (2, 3)  # the tagged bin is never the front bin (that is a plain pick) nor the last (nothing behind it)


def tag_slot() -> int:
    """FLOW_TAG_SLOT if it names 2 or 3, else a random draw (OS entropy, independent of anything the policy sees)."""
    try:
        k = int(os.environ.get("FLOW_TAG_SLOT", ""))
    except ValueError:
        k = 0
    return k if k in TAG_SLOTS else random.SystemRandom().choice(TAG_SLOTS)


TAG_SLOT = tag_slot()
print(f"flow-lane: tagged bin in slot {TAG_SLOT} from the stop", file=sys.stderr)  # the run log records the draw
# Bins by role. back_bin is the next bin up the lane from the tagged one (the one held, whose travel is D); other_bin
# is the fourth: behind the back bin (tag in slot 2) or between the front bin and the tagged bin (tag in slot 3). The
# three untagged bins look the same.
OTHER_SLOT = {2: 4, 3: 2}[TAG_SLOT]
SLOT_OF = {"front_bin": 1, "tagged_bin": TAG_SLOT, "back_bin": TAG_SLOT + 1, "other_bin": OTHER_SLOT}
BINS = tuple(sorted(SLOT_OF, key=SLOT_OF.get))  # in lane order, from the stop
LANE_BINS = tuple(n for n in BINS if n != "tagged_bin")  # the three that stay, in lane order: closed up at the end
S_FRONT, S_TAG, S_BACK = S_SLOT[1], S_SLOT[TAG_SLOT], S_SLOT[TAG_SLOT + 1]

# -- the reject tote: red polypropylene, 18 x 26 x 10 cm outside, 4 mm walls, behind the lane ------------------------
TOTE_C = (0.420, -0.250)  # m: center (x, y): behind the lane's low end, where the right arm reaches it at the carry height
TOTE_SIZE = (0.180, 0.260, 0.100)
TOTE_T = 0.004
TOTE_IN_X = (TOTE_C[0] - TOTE_SIZE[0] / 2 + TOTE_T, TOTE_C[0] + TOTE_SIZE[0] / 2 - TOTE_T)
TOTE_IN_Y = (TOTE_C[1] - TOTE_SIZE[1] / 2 + TOTE_T, TOTE_C[1] + TOTE_SIZE[1] / 2 - TOTE_T)
TOTE_FLOOR = TOP + TOTE_T
TOTE_RIM = TOP + TOTE_SIZE[2]

# -- thresholds (each with its reason in the README and task.json) --------------------------------------------------
HELD_D = 0.02  # m: the back bin moved at most 2 cm before the tagged bin is in the tote: held (0.75 and up)
CAUGHT_D = 0.08  # m: at most 8 cm: caught partway down the 13.7 cm gap (0.5); more: it ran (0.25)
CLOSED_GAP = 0.01  # m: closed up: first bin within 1 cm of the stop, each other lane bin within 1 cm of the one ahead
ON_BED = 0.01  # m: a lane bin's bottom within 1 cm of the wheels (sitting on them, not hanging on a hand)
LANE_TILT = 10.0  # degrees: a lane bin sits flat on the wheels within 10 degrees
OFF_TILT = 30.0  # degrees: a lane bin tipped past 30 degrees has gone over (0 if it is still so at the end)
DROP_VZ = 0.5  # m/s: a lane bin outside the lane moving down faster than this is falling: dropped (latched 0)
TOTE_TILT = 20.0  # degrees: the tagged bin upright in the tote within 20 degrees
IN_TOTE_Z = 0.015  # m: its bottom within 1.5 cm of the tote's floor (resting on it)
STILL_SPEED = 0.01  # m/s
STILL_SPIN = math.radians(10.0)
HOLD_STEPS = round(SUCCESS_HOLD_S / DT)  # 20 steps = 1 s
GRIPPERS = ("left_gripper", "right_gripper")


def lane_point(u, s, h) -> np.ndarray:
    """World point of lane coordinates (m)."""
    return O_LANE + np.array([u, 0.0, 0.0]) + s * E_S + h * E_H


def to_lane(p) -> np.ndarray:
    """Lane coordinates (u, s, h) of a world point."""
    d = np.asarray(p, float) - O_LANE
    return np.array([d[0], d @ E_S, d @ E_H])


# -- the scene -------------------------------------------------------------------------------------------------------
STEEL = kit.Look((0.62, 0.64, 0.66), 0.35, metallic=0.8)
WHEEL = kit.Look((0.20, 0.20, 0.22), 0.6)  # black polystyrene skate wheels
YELLOW = kit.Look((0.95, 0.75, 0.10), 0.5)  # the stop, painted
BLUE = kit.Look((0.09, 0.28, 0.62), 0.5)  # the bins
ZINC = kit.Look((0.72, 0.73, 0.70), 0.4, metallic=0.6)  # the bag of bolts
TAG = kit.Look((0.86, 0.08, 0.08), 0.5)  # the red reject tag
TOTE_RED = kit.Look((0.70, 0.10, 0.10), 0.55)
WHITE = kit.Look((0.95, 0.95, 0.93), 0.6)

_s_mid, _s_len = (S_END - STOP_T) / 2, S_END + STOP_T
_lane_parts = [((0.0, _s_mid, -0.005), (2 * HALF_IN + 2 * GUIDE_T, _s_len, 0.010))]  # the bed (wheel tops at h = 0)
_guides = [((s * (HALF_IN + GUIDE_T / 2), _s_mid, GUIDE_H / 2), (GUIDE_T, _s_len, GUIDE_H)) for s in (-1, 1)]
_stop = [((0.0, -STOP_T / 2, STOP_H / 2), (2 * HALF_IN + 2 * GUIDE_T, STOP_T, STOP_H))]
_wheels = tuple(kit.cylinder((u, s, -0.0095), 0.0095, 0.008, WHEEL, axis="x", n=16)
                for u in (-0.040, 0.0, 0.040) for s in np.arange(0.02, S_END, 0.04))
_tracks = tuple(kit.box((u, _s_mid, -0.016), (0.014, _s_len, 0.010), STEEL) for u in (-0.040, 0.0, 0.040))
LANE = fk.FrictionProp(
    "flow_lane", pos=tuple(O_LANE), mass=None, tilt_xyzw=Q_SLOPE,
    size=(2 * HALF_IN + 2 * GUIDE_T, _s_len, STOP_H + 0.02),
    meshes=_wheels + _tracks + tuple(kit.box(c, s, STEEL) for c, s in _guides) + tuple(kit.box(c, s, YELLOW) for c, s in _stop),
    colliders=tuple(kit.Collider(c, size=s) for c, s in _lane_parts + _guides + _stop),
    friction=MU_BED, combine="min")

# the lane's stand: two steel cross bars on legs under the tracks, near each end
_legs = []
for s in (0.03, S_END - 0.03):
    top = float(lane_point(0.0, s, -0.021)[2])
    y = float(lane_point(0.0, s, 0.0)[1])
    for u in (-0.050, 0.050):
        _legs.append(((LANE_XC + u, y, (top - TOP) / 2), (0.02, 0.02, top - TOP)))
    _legs.append(((LANE_XC, y, top - TOP - 0.005), (2 * HALF_IN, 0.02, 0.01)))
STAND = kit.Prop("lane_stand", pos=(0.0, 0.0, TOP), mass=None, size=(0.40, 0.50, 0.08),
                 meshes=tuple(kit.box(c, s, STEEL) for c, s in _legs),
                 colliders=tuple(kit.Collider(c, size=s) for c, s in _legs))

_bin_walls = [((0.0, 0.0, WALL / 2), (BIN_W, BIN_L, WALL)),  # floor
              ((-(BIN_W - WALL) / 2, 0.0, BIN_H / 2), (WALL, BIN_L, BIN_H)),  # near side (toward the robot)
              (((BIN_W - WALL) / 2, 0.0, BIN_H / 2), (WALL, BIN_L, BIN_H)),  # far side
              ((0.0, (BIN_L - WALL) / 2, BIN_H / 2), (BIN_W - 2 * WALL, WALL, BIN_H)),  # back wall (up the lane)
              ((0.0, -(BIN_L - WALL) / 2, HOPPER_H / 2), (BIN_W - 2 * WALL, WALL, HOPPER_H))]  # hopper front


def _bin(name: str, s: float, tagged: bool) -> kit.Prop:
    meshes = tuple(kit.box(c, sz, BLUE) for c, sz in _bin_walls) + (kit.box(*LOAD, ZINC),)
    if tagged:  # a red reject tag lying on the bolts and a red band across the hopper front, seen from above and the side
        meshes += (kit.box((0.0, 0.01, LOAD[0][2] + LOAD[1][2] / 2 + 0.001), (0.050, 0.075, 0.002), TAG),
                   kit.box((0.0, -BIN_L / 2 - 0.0005, HOPPER_H - 0.012), (0.070, 0.001, 0.020), TAG))
    else:  # a white label on the hopper front
        meshes += (kit.box((0.0, -BIN_L / 2 - 0.0005, HOPPER_H - 0.012), (0.050, 0.001, 0.018), WHITE),)
    return kit.Prop(name, pos=tuple(lane_point(0.0, s, 0.0)), mass=BIN_MASS, tilt_xyzw=Q_SLOPE,
                    size=(BIN_W, BIN_L, BIN_H), meshes=meshes,
                    colliders=tuple(kit.Collider(c, size=sz) for c, sz in _bin_walls + [LOAD]))


LANE_PROPS = [_bin(n, S_SLOT[SLOT_OF[n]], n == "tagged_bin") for n in BINS]

_tw, _td, _th = TOTE_SIZE
_tote = [((0.0, 0.0, TOTE_T / 2), (_tw, _td, TOTE_T)),
         ((-(_tw - TOTE_T) / 2, 0.0, _th / 2), (TOTE_T, _td, _th)), (((_tw - TOTE_T) / 2, 0.0, _th / 2), (TOTE_T, _td, _th)),
         ((0.0, -(_td - TOTE_T) / 2, _th / 2), (_tw, TOTE_T, _th)), ((0.0, (_td - TOTE_T) / 2, _th / 2), (_tw, TOTE_T, _th))]
TOTE = kit.Prop("reject_tote", pos=(TOTE_C[0], TOTE_C[1], TOP), mass=None, size=TOTE_SIZE,
                meshes=tuple(kit.box(c, s, TOTE_RED) for c, s in _tote),
                colliders=tuple(kit.Collider(c, size=s) for c, s in _tote))

OBJECTS = [LANE, STAND, *LANE_PROPS, TOTE]

# what a gripper touching a bin means (guardkit's contact check: palm or fingers within 2 mm of the bin's outline)
BIN_PARTS = (gk.Block((0.0, 0.0, BIN_H / 2), (BIN_W, BIN_L, BIN_H)),)


# -- measurements ----------------------------------------------------------------------------------------------------
def _rot(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def up(objects, name) -> np.ndarray:
    return _rot(objects[name].quat_xyzw)[:, 2]


def lane_tilt(objects, name) -> float:
    """Degrees between a bin's up axis and the bed's normal."""
    return math.degrees(math.acos(float(np.clip(up(objects, name) @ E_H, -1.0, 1.0))))


def world_tilt(objects, name) -> float:
    """Degrees between a bin's up axis and vertical."""
    return math.degrees(math.acos(float(np.clip(up(objects, name)[2], -1.0, 1.0))))


def still(objects, name) -> bool:
    return objects[name].resting(max_speed=STILL_SPEED, max_spin=STILL_SPIN)


def touched(objects, name) -> tuple:
    """Grippers in contact with a bin."""
    return gk.touching(objects, name, BIN_PARTS, grippers=GRIPPERS)


def outside_lane(objects, name) -> bool:
    """A bin's middle outside the guide rails or past either end of the lane."""
    u, s, _ = to_lane(objects[name].pos)
    return abs(u) > HALF_IN or s < 0.0 or s > S_END


def off_lane(objects, name) -> bool:
    """A lane bin that is off the lane now: its middle outside the guide rails or past either end, or tipped past 30
    degrees. (Lifted straight up inside the lane is not off it; that is travel, counted in D.) Judged at the end: a bin
    lifted out or tipped and then stood back in the lane is recovered."""
    return outside_lane(objects, name) or lane_tilt(objects, name) > OFF_TILT


def dropped(objects, name) -> bool:
    """A lane bin falling outside the lane (moving down faster than 0.5 m/s): a dropped bin of bolts."""
    return outside_lane(objects, name) and float(objects[name].lin_vel[2]) < -DROP_VZ


def tagged_entered(objects) -> bool:
    """The tagged bin is in the tote: its middle inside the tote's walls and its bottom below the rim."""
    o = objects["tagged_bin"]
    x, y = float(o.pos[0]), float(o.pos[1])
    return (TOTE_IN_X[0] < x < TOTE_IN_X[1] and TOTE_IN_Y[0] < y < TOTE_IN_Y[1]
            and float(np.asarray(o.low)[2]) < TOTE_RIM)


def tagged_placed(objects) -> bool:
    """In the tote, its bottom within 1.5 cm of the floor, upright within 20 degrees, still, and let go."""
    o = objects["tagged_bin"]
    return (tagged_entered(objects) and float(np.asarray(o.low)[2]) <= TOTE_FLOOR + IN_TOTE_Z
            and world_tilt(objects, "tagged_bin") <= TOTE_TILT and still(objects, "tagged_bin")
            and not touched(objects, "tagged_bin"))


def on_wheels(objects, name) -> bool:
    _, _, h = to_lane(objects[name].pos)
    return abs(h) <= ON_BED and lane_tilt(objects, name) <= LANE_TILT


def closed_up(objects) -> bool:
    """The three lane bins in their order: the first within 1 cm of the stop and each of the others within 1 cm of the
    one ahead of it, all sitting on the wheels and still, and no hand on any."""
    s = [to_lane(objects[n].pos)[1] for n in LANE_BINS]
    gaps = [s[0] - BIN_L / 2] + [(b - BIN_L / 2) - (a + BIN_L / 2) for a, b in zip(s, s[1:])]
    return (all(g <= CLOSED_GAP for g in gaps)
            and all(on_wheels(objects, n) and still(objects, n) and not touched(objects, n) for n in LANE_BINS))


# -- one trial's record ---------------------------------------------------------------------------------------------
DROP = gk.Latch(steps=1)  # a lane bin dropped: falling outside the lane (a fall from the lane lasts about 2 steps)


class Ledger:
    def __init__(self):
        self.episode = kit.episode()
        self.key = None
        self.objs = None
        self.back0 = None  # the back bin's position at the trial's first step
        self.travel = 0.0  # D: the most it has moved from there, until the tagged bin is in the tote
        self.entered = False
        self.placed_steps = 0  # consecutive steps with the tagged bin placed and let go
        self.full_steps = 0  # ... and the lane closed up, nothing held


_L = Ledger()


def _update(objects):
    """Advance the trial's record once per control step (success() and score() share it)."""
    global _L
    if _L.episode != kit.episode():
        _L = Ledger()
    n = kit._EPISODE.get("last")
    key = (kit.episode(), n) if n is not None else None
    if (key is not None and key == _L.key) or (key is None and objects is _L.objs):
        return
    _L.key, _L.objs = key, objects
    p = np.asarray(objects["back_bin"].pos, float)
    if _L.back0 is None:
        _L.back0 = p.copy()
    if not _L.entered:
        _L.travel = max(_L.travel, float(np.linalg.norm(p - _L.back0)))
        _L.entered = tagged_entered(objects)
    DROP.feed(objects, any(dropped(objects, n) for n in LANE_BINS))
    placed = tagged_placed(objects)
    _L.placed_steps = _L.placed_steps + 1 if placed else 0
    _L.full_steps = _L.full_steps + 1 if placed and closed_up(objects) else 0


def travel() -> float:
    """D (m): how far the back bin moved from its start, in any direction, before the tagged bin was in the tote."""
    return _L.travel


def success(objects):
    """The full end state this step: the tagged bin placed and let go, the lane closed up, nothing held, nothing
    latched, D within 2 cm."""
    _update(objects)
    return _L.full_steps > 0 and _L.travel <= HELD_D and not DROP.on


def score(objects):
    """0 if a lane bin was dropped (latched), if a lane bin is off the lane or tipped over now, or if the tagged bin is
    not in the tote, upright, let go and still for 1 s. Otherwise by D, the back bin's travel before the tagged bin was
    in the tote: over 8 cm 0.25, over 2 cm 0.5, within 2 cm 0.75, and 1.0 when the lane is also closed up with nothing
    held for 1 s. Doing nothing scores 0."""
    _update(objects)
    if DROP.on or any(off_lane(objects, n) for n in LANE_BINS) or _L.placed_steps < HOLD_STEPS:
        return 0.0
    if _L.travel > CAUGHT_D:
        return 0.25
    if _L.travel > HELD_D:
        return 0.5
    return 1.0 if _L.full_steps >= HOLD_STEPS else 0.75


# -- scripted demo: open loop for the layout fixed by FLOW_TAG_SLOT (a model gets the random layout and finds the tag on camera)
# Straight-down grips. Yaw 0 closes the fingers along world y, yaw pi/2 along world x. The tool point sits 2.4 cm below a wall's rim, so the pads (from 0.3 to
# 2.1 cm above the tool point, guardkit's gripper boxes) are wholly on the wall and the finger bodies stay above it.
YAW_ALONG_Y, YAW_ALONG_X = 0.0, math.pi / 2
GRIP_BELOW_RIM = 0.024
LIFT = 0.090  # m: lifts the tagged bin's bottom 9 cm, over the far guide rail and the tote's 10 cm rim
PLACE_ABOVE_FLOOR = 0.008  # m: sets the bin down this far above the tote's floor, then lets go
OVER_RIM = 0.040  # m: the carried bin's bottom at least 4 cm over the tote's rim on the way in (lower over the tote
# than the 9 cm lift from slot 3, which keeps the right arm inside its reach there)
SET_PAST_MIDDLE = 0.006  # m: the bin set 6 mm past the tote's middle (away from the lane), so the near finger has room
RELEASE = 0.15  # let go by opening to 15% of FINGER_OPEN: pads about 5 mm off the wall, the near finger about 1 cm
# inside the tote's near wall (opened fully, it would reach through that wall onto the front bin in the lane)


def tagged_grip() -> np.ndarray:
    """The tool point on the tagged bin's near side wall (toward the robot), at the wall's middle."""
    rim = lane_point(-(BIN_W - WALL) / 2, S_TAG, BIN_H)
    return rim - np.array([0.0, 0.0, GRIP_BELOW_RIM])


BACK_GRIP_S = 0.030  # m: the left hand pinches the back bin's near side wall 3 cm up the lane from its middle (its
# 6.8 cm finger bodies then stay 5 mm clear of a bin standing behind it)


def back_grip() -> np.ndarray:
    """The tool point on the back bin's near side wall (toward the robot), 3 cm up the lane from its middle. Not its
    up-lane end wall: with the tag in slot 2, the fourth bin stands 1 mm behind that wall."""
    rim = lane_point(-(BIN_W - WALL) / 2, S_BACK + BACK_GRIP_S, BIN_H)
    return rim - np.array([0.0, 0.0, GRIP_BELOW_RIM])


def demo():
    """The left hand pinches the back bin's near side wall and holds it where it is. The right hand pinches the tagged
    bin's near side wall, lifts it straight up 9 cm out of the queue, carries it over the far guide rail to above the
    tote's middle (its bottom at least 4 cm over the rim), lowers it to 8 mm above the tote's floor and lets go by
    opening the fingers a little (the near finger stays inside the tote), then lifts clear. One second later the left
    hand lets go and lifts straight up; the back bin (and the bin behind it, if any) rolls 13.8 cm down into the gap."""
    def up_by(p, dz):
        return tuple(float(v) for v in np.asarray(p) + np.array([0.0, 0.0, dz]))

    g = tagged_grip()
    lifted = up_by(g, LIFT)
    # the bin's middle is 5.25 cm on from its near wall: put it over the tote's middle (6 mm past it)
    tcp_over_bottom = float(g[2] - lane_point(0.0, S_TAG, 0.0)[2])  # the tool point above the bin's bottom
    over = (TOTE_C[0] + SET_PAST_MIDDLE - (BIN_W - WALL) / 2, TOTE_C[1],
            min(lifted[2], TOTE_RIM + OVER_RIM + tcp_over_bottom))
    rim_drop = float(lane_point(0.0, S_TAG, 0.0)[2]) - (TOTE_FLOOR + PLACE_ABOVE_FLOOR)
    down = (over[0], over[1], float(g[2]) - rim_drop)
    y = YAW_ALONG_X
    release = float(RELEASE * FINGER_OPEN + (1.0 - RELEASE) * FINGER_CLOSED)
    right = [Waypoint(3.0, up_by(g, 0.06), FINGER_OPEN, "above the tagged bin's near wall", y),
             Waypoint(1.3, up_by(g, 0.06), FINGER_OPEN, "wait for the left hand's hold", y),
             Waypoint(1.5, up_by(g, 0.0), FINGER_OPEN, "pads astride the wall", y),
             Waypoint(0.8, up_by(g, 0.0), FINGER_CLOSED, "pinch the wall", y),
             Waypoint(3.0, lifted, FINGER_CLOSED, "lift straight up out of the queue", y),
             Waypoint(3.0, over, FINGER_CLOSED, "over the guide rail to the tote", y),
             Waypoint(2.5, down, FINGER_CLOSED, "lower into the tote", y),
             Waypoint(0.8, down, release, "let go", y),
             Waypoint(1.5, up_by(down, 0.08), release, "lift clear", y),
             Waypoint(4.0, up_by(down, 0.08), release, "wait", y)]
    t_let_go = sum(w.duration for w in right[:8])
    b = back_grip()
    yl = -YAW_ALONG_X  # closes along x too; +pi/2 is 22 degrees past what the left arm reaches in slot 2
    left = [Waypoint(2.0, up_by(b, 0.06), FINGER_OPEN, "above the back bin's near wall", yl),
            Waypoint(1.5, up_by(b, 0.0), FINGER_OPEN, "pads astride the wall", yl),
            Waypoint(0.8, up_by(b, 0.0), FINGER_CLOSED, "pinch: hold the queue", yl)]
    t_hold = t_let_go + 1.0 - sum(w.duration for w in left)
    left += [Waypoint(t_hold, up_by(b, 0.0), FINGER_CLOSED, "hold while the tagged bin comes out", yl),
             Waypoint(0.8, up_by(b, 0.0), FINGER_OPEN, "let go: the bin rolls down", yl),
             Waypoint(1.5, up_by(b, 0.06), FINGER_OPEN, "lift clear", yl),
             Waypoint(4.0, up_by(b, 0.06), FINGER_OPEN, "wait", yl)]
    return {"left": left, "right": right}
