"""On the station table: an empty white bowl (16.6 cm across, fixed) stands 36 cm in front of the arms on the centre
line. Nearer the arms, an apple sits at x 8 cm, 16 cm to the robot's left of the centre line, and an orange at x 12 cm,
28 cm to its right. Both fruits are loose. The robot must set each one on the table beside the bowl, level with it, the
apple on its left and the orange on its right, and never put either fruit in the bowl.

Instruction: Put the apple on the table beside the bowl, level with it, on the robot's left, and the orange on the table beside the bowl, level with it, on the robot's right. Do not put either fruit in the bowl.

Requires -E object_jitter=0: the scorer recognises a reset between trials by the fruits' start spots.

Run it:  inspect-robots run --embodiment yam_arena -E task=fruit_beside_not_in_bowl.py -E object_jitter=0 \\
           --instruction "$INSTR" --scorer task_score ...
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
from belimits import PeakTracker, Rect, bottom, inside, is_airborne, tilt_deg  # noqa: E402

from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint, pick_place_waypoints  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Asset, Sphere  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
SUCCESS_HOLD_S = 5.0  # long enough for the video to show both hands open and parked clear of the fruit

# The bowl: the station's ceramic bowl, fixed, 16.6 cm across its rim and 5.75 cm tall, centred on the midline.
BOWL_XY, BOWL_R = (0.36, 0.0), 0.083
BOWL = Asset("bowl", "bowl_ycb_robolab", xy=BOWL_XY, extent=2 * BOWL_R, color=(0.86, 0.84, 0.80))

# The fruits: the station's apple (6.6 cm, 120 g) and orange (6.4 cm, 110 g), loose, nearer the arms than the bowl, each
# on the side of the centre line it has to end up on. The layout is not mirrored: at READY the right arm's shadow covers
# the table at y -14 to -24 cm for x 4 to 12 cm, so the orange starts further out, at the lit spot y -28 cm.
START_X = {"apple": 0.08, "orange": 0.12}
START_Y = {"apple": 0.16, "orange": -0.28}
APPLE = Asset("apple", "apple_01_objaverse_robolab", xy=(START_X["apple"], START_Y["apple"]), extent=0.066, mass=0.12,
              color=(0.629, 0.104, 0.138), convex=True)
ORANGE = Asset("orange", "orange_02_fruits_veggies_robolab", xy=(START_X["orange"], START_Y["orange"]), extent=0.064,
               mass=0.11, color=(0.92, 0.33, 0.025))
FRUITS = {"apple": APPLE, "orange": ORANGE}
GRIP = {"apple": 0.031, "orange": 0.030}  # TCP height above the table that grips each fruit at its centre

OBJECTS = [BOWL, APPLE, ORANGE]

# Each fruit is judged as a ball of its width around its centre (not listed in OBJECTS, so nothing extra is in the
# scene). A fruit rolls, and a ball's outline and underside do not change when it does.
BALLS = {n: Sphere(f"{n}_outline", pos=(*f.xy, TOP + f.extent / 2), radius=f.extent / 2) for n, f in FRUITS.items()}

# In the bowl (checked every step): a fruit's centre over the bowl's rim circle while its underside is below the
# rim's height plus 2.5 mm. Above that, the fruit is being carried over the bowl, which is allowed.
IN_BOWL_R, IN_BOWL_Z = BOWL_R, TOP + 0.06

# Where each fruit must end: centre at x 28..44 cm (the bowl spans 27.7..44.3) and no further out than |y| 24 cm,
# its whole outline at |y| 9 cm or more, 7 mm past the rim on its own side. The orange starts at |y| 28 cm, so a hop
# straight forward misses its goal: it has to come at least 4 cm inward as well as 16 cm forward.
GOAL_X, GOAL_EDGE, GOAL_OUT = (0.28, 0.44), 0.09, 0.24
GOALS = {
    n: Rect((GOAL_X[0] - b.radius, GOAL_X[1] + b.radius), tuple(sign * v for v in (GOAL_EDGE, GOAL_OUT + b.radius)))
    for (n, b), sign in zip(BALLS.items(), (1, -1))
}
# A fruit touches the table when its underside is within 1 cm of the top (not belimits.is_airborne). A fruit must be
# carried, not pushed or rolled: it may travel at most 3 cm in total while touching the table outside its goal. Motion
# under 1 mm from the last counted point is not counted, so a resting fruit's jitter never adds up.
MAX_TABLE_TRAVEL, TRAVEL_DEADBAND = 0.03, 0.001
MAX_FALL_SPEED = 1.0  # m/s: a fruit that ever falls faster (about a 5 cm drop) was dropped or thrown and earns 0
OFF_TABLE = TOP - 0.05  # underside below this: off the table

# The robot keeps out too: no part of either arm below 25 cm above the table over the bowl's outline grown by
# 4 cm (checked offline with the URDF on the scripted demo; the scorer itself only sees the objects).
_KO = BOWL_R + 0.04
ARM_KEEP_OUT = [
    {"name": "bowl and 4 cm around it", "x": (BOWL_XY[0] - _KO, BOWL_XY[0] + _KO), "y": (-_KO, _KO),
     "z_top": TOP + 0.25},
]


def _bowl_dist(s) -> float:
    return float(np.hypot(*(np.asarray(s.pos, float)[:2] - np.asarray(BOWL_XY))))


def _in_bowl(name, s) -> bool:
    """Centre over the bowl's rim circle with the underside below rim height (+2.5 mm): in it, or on its rim."""
    return _bowl_dist(s) < IN_BOWL_R and bottom(s, BALLS[name]) < IN_BOWL_Z


def _over_bowl(name, s) -> bool:
    """Any part of the fruit's outline seen from above over the bowl's rim circle, at any height: in it, perched
    on or leaning over its rim, tucked under it, or still held above it (opening the gripper would drop it in)."""
    return _bowl_dist(s) < BOWL_R + BALLS[name].radius


def _in_goal(name, s) -> bool:
    """Centre in the goal, whole outline at |y| 9 cm or more."""
    return inside(s, BALLS[name], GOALS[name])


def _placed(name, s) -> bool:
    """Carried, not pushed, rolled, dropped or thrown, and now on the table in its goal, underside no more than 1 cm
    above the table top."""
    return bool(_state["travel"][name] <= MAX_TABLE_TRAVEL and _state["fall"].get(name, 0.0) <= MAX_FALL_SPEED
                and _in_goal(name, s) and not is_airborne(s, BALLS[name], [TOP]))


# -- per-step state: was a fruit ever in the bowl, how far has each fruit moved along the table, how fast has it fallen --
# score() only sees the current step, so module-level state remembers these. It clears on the first call and on an
# environment reset: both fruits on the table, within 1 cm and 3 deg of their start spots, horizontally still, and one
# of them more than 3 cm from where it was on the previous call. At 20 Hz that jump is over 0.6 m/s, which a reset does
# and leaves the fruit still; a fruit carried back to its spot does neither, so carrying it back cannot clear the state.
SPAWN_TOL, SPAWN_DEG, JUMP, STILL = 0.01, 3.0, 0.03, 0.05


def _fresh() -> dict:
    # PeakTracker with no home objects never forgets by itself: only a reset (a new tracker) clears the fall peak.
    return {"entered": False, "travel": {n: 0.0 for n in FRUITS}, "anchor": {}, "fall_tracker": PeakTracker({}), "fall": {}}


_state = {"prev": None, **_fresh()}


def _at_spawn(objects) -> bool:
    return all(
        np.hypot(*(np.asarray(objects[n].pos, float)[:2] - np.asarray(f.xy))) <= SPAWN_TOL
        and tilt_deg(objects[n]) <= SPAWN_DEG
        and np.hypot(*np.asarray(objects[n].lin_vel, float)[:2]) < STILL
        and not is_airborne(objects[n], BALLS[n], [TOP])
        for n, f in FRUITS.items()
    )


def _add_table_travel(name, s) -> None:
    """Add this step's motion along the table outside the goal to the fruit's total. A step from outside the goal into
    it counts in full. ``anchor`` is the last counted point and whether it was in the goal."""
    xy, in_goal = np.asarray(s.pos, float)[:2], _in_goal(name, s)
    last, last_in_goal = _state["anchor"].get(name, (xy, in_goal))
    if is_airborne(s, BALLS[name], [TOP]) or (in_goal and last_in_goal):
        _state["anchor"][name] = (xy, in_goal)
        return
    moved = float(np.hypot(*(xy - last)))
    if moved >= TRAVEL_DEADBAND:
        _state["travel"][name] += moved
        _state["anchor"][name] = (xy, in_goal)
    else:
        _state["anchor"].setdefault(name, (last, last_in_goal))


def _update(objects) -> bool:
    """Advance the per-step state with this step's objects; returns whether a fruit has been in the bowl. A call
    with the same positions as the previous call (score() and success() on the same step) changes nothing."""
    pos = tuple(tuple(round(float(v), 9) for v in objects[n].pos) for n in FRUITS)
    prev = _state["prev"]
    if pos == prev:
        return _state["entered"]
    jumped = prev is not None and any(np.hypot(p[0] - q[0], p[1] - q[1]) > JUMP for p, q in zip(pos, prev))
    if prev is None or (jumped and _at_spawn(objects)):
        _state.update(_fresh())
    _state["prev"] = pos
    for n in FRUITS:
        _state["entered"] = _state["entered"] or _in_bowl(n, objects[n])
        _add_table_travel(n, objects[n])
    _state["fall"] = _state["fall_tracker"].update(objects, {n: -float(objects[n].lin_vel[2]) for n in FRUITS})
    return _state["entered"]


def reset_state() -> None:
    """For offline tests: forget the previous episode."""
    _state.update(prev=None, **_fresh())


def score(objects):
    """0 if a fruit has been in the bowl at any step (centre over the rim circle, underside below rim height); if,
    at the end, any part of either fruit's outline is over the bowl's rim circle at any height; or a fruit's underside
    is more than 5 cm below the table top. Otherwise 0.5 per fruit carried to its goal and left on the table in it
    (see _placed)."""
    entered = _update(objects)
    if entered or any(_over_bowl(n, objects[n]) or bottom(objects[n], BALLS[n]) < OFF_TABLE for n in FRUITS):
        return 0.0
    return 0.5 * sum(_placed(n, objects[n]) for n in FRUITS)


def success(objects):
    """Both fruits placed, no fruit ever in the bowl, and both still."""
    return score(objects) == 1.0 and all(objects[n].resting() for n in FRUITS)


# -- the scripted demo ---------------------------------------------------------------------------------------------
TRAVEL = TOP + 0.14
DROP = {"apple": (0.36, 0.20), "orange": (0.36, -0.20)}
PARK = {"apple": (0.14, 0.28, TOP + 0.16), "orange": (0.14, -0.28, TOP + 0.16)}
RELEASE_GAP = 0.01
# The apple starts near the centre line, so a straight carry to its drop swings the left elbow to 3.8 cm from the bowl's
# outline. Bending the carry out to this point keeps every arm link at least 7 cm from it.
CARRY_VIA = {"apple": (0.22, 0.25)}


def _plan(name, wait=0.0):
    """Go above the fruit (and wait there ``wait`` seconds), pick it with the fingers closing across y, lift, carry it
    forward, turning the grip so the fingers close along x, set it down beside the bowl, open, and park the open hand
    clear of it."""
    f = FRUITS[name]
    pick = (*f.xy, TOP + GRIP[name])
    place = (*DROP[name], TOP + GRIP[name] + RELEASE_GAP)
    wps = pick_place_waypoints(pick, place, travel_z=TRAVEL, yaw=0.0, place_yaw=np.pi / 2)
    if name in CARRY_VIA:
        i = next(k for k, w in enumerate(wps) if w.label == "carry")
        c = wps[i]
        wps[i:i + 1] = [Waypoint(1.2, (*CARRY_VIA[name], TRAVEL), c.finger, "carry", np.pi / 4),
                        Waypoint(1.2, c.pos, c.finger, "carry", c.yaw)]
    if wait:
        wps.insert(1, Waypoint(wait, wps[0].pos, FINGER_OPEN, "wait", 0.0))
    return wps + [Waypoint(2.0, PARK[name], FINGER_OPEN, "park", np.pi / 2)]


def demo():
    """The left arm moves the apple to (36, +20) cm, bending its carry out through (22, +25) cm. The right arm comes to its waiting spot above the orange, waits
    there until the left arm has lifted the apple, then moves the orange to (36, -20) cm. Each arm stays on its own
    side, and no part of either arm comes within 4 cm of the bowl's outline below 25 cm above the table. Both hands
    end open and parked, clear of the fruit."""
    return {"left": _plan("apple"), "right": _plan("orange", wait=5.0)}
