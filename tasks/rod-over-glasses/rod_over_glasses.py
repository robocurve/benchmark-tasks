"""On the station table, in front of the two arms: a 40 cm wooden rod lies crosswise near the robot, a row of
three drinking glasses stands across the middle of the table, and two wooden rests stand beyond the glasses.
The rod is longer than the gaps between the glasses, so it has to be lifted over them: carried by both arms,
one hand near each end, it clears the glasses with a few centimetres to spare.

World frame (m): x forward from the arm bases, y to the robot's left, z up; the table top is at z 0.7475.
- rod: pine square dowel stained dark brown, 2.5 x 40 x 2.5 cm, 150 g, centred at (0.14, 0), lying along y.
- glass_left, glass_mid, glass_right: glass tumblers, 7 cm across, 10 cm tall, 250 g, standing at x 0.24,
  y +0.08 / 0 / -0.08 (9 cm gaps between them).
- rest_left, rest_right: fixed pine blocks stained dark brown, 5 x 3 x 4 cm (x, y, z), centred at x 0.34,
  y +0.16 / -0.16.

Full score: the rod lies still across both rests for 3 s and every glass still stands where it started.
"""

import numpy as np

from yam_arena.demo import pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 3.0  # the rod must stay on the rests for 3 s, long enough for a demo to let go and pull back

WOOD = (0.36, 0.20, 0.10)  # dark-stained, so the rod and rests stand out against the light table
GLASS = (0.80, 0.90, 0.95)

ROD_LEN, ROD_W = 0.40, 0.025
ROD_X = 0.14
GLASS_X, GLASS_YS, GLASS_R, GLASS_H = 0.24, (0.08, 0.0, -0.08), 0.035, 0.10
REST_X, REST_Y = 0.34, 0.16
REST_SIZE = (0.05, 0.03, 0.04)

ROD = Box("rod", pos=(ROD_X, 0.0, TOP + ROD_W / 2), size=(ROD_W, ROD_LEN, ROD_W), mass=0.15, color=WOOD)
GLASSES = [
    Cylinder(name, pos=(GLASS_X, y, TOP + GLASS_H / 2), radius=GLASS_R, height=GLASS_H, mass=0.25, color=GLASS)
    for name, y in zip(("glass_left", "glass_mid", "glass_right"), GLASS_YS)
]
RESTS = [
    Box(name, pos=(REST_X, y, TOP + REST_SIZE[2] / 2), size=REST_SIZE, color=WOOD)
    for name, y in (("rest_left", REST_Y), ("rest_right", -REST_Y))
]
OBJECTS = [ROD, *GLASSES, *RESTS]

# Thresholds, as a reviewer would check them with a tape measure.
GLASS_MOVE_MAX = 0.02  # a glass counts as disturbed once it is 2 cm or more from where it stood ...
GLASS_TILT_MAX = np.radians(10)  # ... or leans 10 degrees or more
REST_TOP = TOP + REST_SIZE[2]
PAST_GLASSES_X = GLASS_X + GLASS_R  # the far side of the glass row


def _tilt(quat_xyzw) -> float:
    """Angle between the object's own z axis and the vertical (rad)."""
    x, y, _, _ = quat_xyzw
    return float(np.arccos(np.clip(1.0 - 2.0 * (x * x + y * y), -1.0, 1.0)))


def _glasses_ok(objects) -> bool:
    for glass in GLASSES:
        state = objects[glass.name]
        moved = np.hypot(state.pos[0] - glass.pos[0], state.pos[1] - glass.pos[1])
        if moved >= GLASS_MOVE_MAX or _tilt(state.quat_xyzw) >= GLASS_TILT_MAX:
            return False
    return True


def _across_rests(rod) -> bool:
    """The rod lies on both rests: crosswise (its ends past both rests), its underside within 1 cm of their
    tops, and its middle over the gap between them."""
    on_top = abs(rod.low[2] - REST_TOP) < 0.01
    spans = rod.low[1] < -(REST_Y + REST_SIZE[1] / 2) + 0.01 and rod.high[1] > REST_Y + REST_SIZE[1] / 2 - 0.01
    over = abs(rod.pos[0] - REST_X) < REST_SIZE[0] / 2 and abs(rod.pos[1]) < 0.05
    return bool(on_top and spans and over)


def success(objects):
    """The rod lies still across both rests, and no glass was disturbed."""
    rod = objects["rod"]
    return _glasses_ok(objects) and _across_rests(rod) and rod.resting()


def distance(objects):
    rod = objects["rod"]
    return float(np.linalg.norm(rod.pos - np.array([REST_X, 0.0, REST_TOP + ROD_W / 2])))


def score(objects):
    """Partial credit (success scores 1.0): 0.5 when the rod ends up wholly beyond the glasses and still on the
    table but not across both rests; 0 if any glass was disturbed, whatever happened to the rod."""
    rod = objects["rod"]
    if not _glasses_ok(objects):
        return 0.0
    beyond = rod.low[0] > PAST_GLASSES_X and rod.low[2] > TOP - 0.01
    return 0.5 if beyond else 0.0


def demo():
    """Both arms together: each grips the rod 8 cm in from its own end (fingers closing across the rod, so the
    grip is turned 90 degrees, in opposite directions for the two arms to stay inside joint6's range), lifts it
    to 16 cm, carries it over the glasses and lowers it onto the rests. The two plans share every timing, so
    the rod stays level."""
    grip_dz = ROD_W / 2 + 0.005  # the TCP 5 mm above the rod's centre line
    plans = {}
    for arm, y, yaw in (("left", 0.12, -np.pi / 2), ("right", -0.12, np.pi / 2)):
        pick = (ROD_X, y, TOP + grip_dz)
        place = (REST_X, y, REST_TOP + grip_dz + 0.003)  # let go 3 mm above the rests
        plans[arm] = pick_place_waypoints(pick, place, travel_z=TOP + 0.16, yaw=yaw, speed=0.6)
    return plans
