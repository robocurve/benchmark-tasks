"""On the station table, the end of a filtration: scrape the filter cake into a collection container.

- Filter tray (on the midline, loose and light, 100 g): a white porcelain block 3.2 cm front to back x 21.5 cm side to
  side x 4 cm tall. Its flat top is the filter surface. Its long left end (robot's left) is clear for a hand to hold
  it well away from the hand that scrapes.
- Filter cake: twelve blue copper-sulfate crystals (8 mm cubes, 2 g each) in four rows of three on the right part
  of the filter surface.
- Side walls (fixed): two white porcelain walls, 4 mm thick, standing on the table 4 mm in front of and behind the tray and
  rising 1.5 cm above the filter surface. They run from the cake to past the tray's right end, so the cake can only
  leave the filter off that end.
- Collection container (fixed): the 6 cm stretch of the side walls past the tray's right end, closed by a glass far
  wall;
  the tray's end face closes the near side, so solid pushed off that end drops straight in.
- Spatula (in front of the right arm, loose, 50 g): a steel-grey flat paddle, 2.8 cm wide x 1.5 cm thick x 13 cm
  tall, standing upright on its edge. It clears the side walls by 6 mm on each side.

- Bench glassware (fixed, out of the arms' way along the front of the table): a filter flask, a beaker of water and an
  amber reagent bottle.

The tray slides if the spatula drags it, so one hand holds it down while the other scrapes.
"""

import math
import os

import numpy as np

from yam_arena.constants import PAD_TIP_PAST_TCP
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# ---------------------------------------------------------------- filter tray (loose)
TRAY_X = 0.28
TRAY_Y_LO, TRAY_Y_HI = -0.06, 0.155  # right end, left end
TRAY_W, TRAY_H = 0.032, 0.04  # 3.2 cm: the gripper opens about 4.7 cm, and a 4 cm tray caught the finger tips
TRAY_TOP = TOP + TRAY_H
TRAY = Box(
    "filter_tray",
    pos=(TRAY_X, (TRAY_Y_LO + TRAY_Y_HI) / 2, TOP + TRAY_H / 2 + 0.0005),
    size=(TRAY_W, TRAY_Y_HI - TRAY_Y_LO, TRAY_H),
    mass=0.1,
    color=(0.93, 0.93, 0.90),
)

# ---------------------------------------------------------------- filter cake (loose)
CUBE = 0.008
CAKE_ROWS_Y = (-0.048, -0.034, -0.020, -0.006)
CAKE = [
    Box(f"cake_{i}", pos=(x, y, TRAY_TOP + CUBE / 2 + 0.0005), size=(CUBE, CUBE, CUBE), mass=0.002, color=(0.15, 0.42, 0.85))
    for i, (x, y) in enumerate((x, y) for y in CAKE_ROWS_Y for x in (TRAY_X - 0.0085, TRAY_X, TRAY_X + 0.0085))
]

# ---------------------------------------------------------------- side walls and collection container (fixed)
WALL_T = 0.004
WALL_IN = TRAY_W / 2 + 0.004  # half the gap between the side walls: 4 mm clear of the tray on each side
WALL_TOP = TRAY_TOP + 0.015
WALL_Y_HI = 0.015  # the walls start just behind the cake (the left hand holds the tray further left)
BIN_X, BIN_IN = TRAY_X, 0.06
BIN_Y_HI = TRAY_Y_LO - 0.0005  # the tray's end face is the container's near side (no wall for cubes to land on)
BIN_Y_LO = BIN_Y_HI - BIN_IN
PORCELAIN = (0.93, 0.93, 0.90)
GLASS = (0.78, 0.86, 0.90)


def _walls() -> list[Box]:
    y_lo = BIN_Y_LO - WALL_T
    side_len = WALL_Y_HI - y_lo
    side_y = (WALL_Y_HI + y_lo) / 2
    off = WALL_IN + WALL_T / 2
    z = (TOP + WALL_TOP) / 2
    h = WALL_TOP - TOP
    return [
        Box("wall_back", pos=(BIN_X - off, side_y, z), size=(WALL_T, side_len, h), color=PORCELAIN),
        Box("wall_front", pos=(BIN_X + off, side_y, z), size=(WALL_T, side_len, h), color=PORCELAIN),
        Box("bin_far", pos=(BIN_X, BIN_Y_LO - WALL_T / 2, z), size=(2 * WALL_IN, WALL_T, h), color=GLASS),
    ]


# ---------------------------------------------------------------- spatula (loose)
SPATULA = Box("spatula", pos=(0.22, -0.20, TOP + 0.065 + 0.0005), size=(0.028, 0.015, 0.13), mass=0.05, color=(0.62, 0.64, 0.68))

# ---------------------------------------------------------------- bench glassware (fixed, for the look of the bench)
PROPS = [
    Cylinder("filter_flask", pos=(0.40, 0.24, TOP + 0.06), radius=0.04, height=0.12, color=GLASS),
    Cylinder("water_beaker", pos=(0.40, 0.06, TOP + 0.04), radius=0.032, height=0.08, color=(0.70, 0.82, 0.92)),
    Cylinder("reagent_bottle", pos=(0.40, -0.24, TOP + 0.05), radius=0.028, height=0.10, color=(0.55, 0.32, 0.10)),
]

OBJECTS = [TRAY, *CAKE, *_walls(), SPATULA, *PROPS]
FREE = {o.name: o for o in (TRAY, *CAKE, SPATULA)}

# ---------------------------------------------------------------- scoring
STEADY_MOVE = 0.01  # the tray may shift at most 1 cm ...
STEADY_DEG = 5.0  # ... and tilt or turn at most 5 degrees, at any time in the run


def _quat(s):
    return [float(v) for v in s.quat_xyzw]


def _tilt_deg(s) -> float:
    x, y, _, _ = _quat(s)
    return math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))))


def _yaw_deg(s) -> float:
    x, y, z, w = _quat(s)
    return math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def in_bin(s) -> bool:
    x, y = float(s.pos[0]), float(s.pos[1])
    return abs(x - BIN_X) < WALL_IN and BIN_Y_LO < y < BIN_Y_HI and float(s.low[2]) < TOP + 0.02


# score() is called after every control step; it keeps how far the tray has moved so far in this episode.
_state: dict = {}


def _reset_state() -> None:
    _state.clear()
    _state.update(move=0.0, tilt=0.0, turn=0.0, last={})


def _new_episode(objects) -> bool:
    """A reset puts the loose objects back where they started in one step; nothing moves that far that fast."""
    last = _state.get("last")
    if not last:
        return True
    for name, spec in FREE.items():
        p = np.asarray(objects[name].pos, dtype=float)
        if np.linalg.norm(p - last[name]) > 0.05 and math.hypot(p[0] - spec.pos[0], p[1] - spec.pos[1]) < 0.035:
            return True
    return False


def _update(objects) -> None:
    if _new_episode(objects):
        _reset_state()
    t = objects["filter_tray"]
    _state["move"] = max(_state["move"], math.hypot(float(t.pos[0]) - TRAY.pos[0], float(t.pos[1]) - TRAY.pos[1]))
    _state["tilt"] = max(_state["tilt"], _tilt_deg(t))
    _state["turn"] = max(_state["turn"], abs(_yaw_deg(t)))
    _state["last"] = {n: np.asarray(objects[n].pos, dtype=float).copy() for n in FREE}


def _steady() -> bool:
    return _state["move"] <= STEADY_MOVE and _state["tilt"] <= STEADY_DEG and _state["turn"] <= STEADY_DEG


DEBUG = os.environ.get("SCRAPE_DEBUG") == "1"  # SCRAPE_DEBUG=1: print progress once a second


def _debug(objects) -> None:
    _state["n"] = _state.get("n", 0) + 1
    if not _steady() and "unsteady_at" not in _state:
        _state["unsteady_at"] = _state["n"] / 20
        print(
            f"[scrape] tray no longer steady at t={_state['unsteady_at']:.2f}s: shift {_state['move'] * 1000:.1f} mm, "
            f"tilt {_state['tilt']:.1f}, turn {_state['turn']:.1f} deg",
            flush=True,
        )
    if _state["n"] % 20 == 0:
        n_in = sum(in_bin(objects[c.name]) for c in CAKE)
        print(
            f"[scrape t={_state['n'] / 20:5.1f}s] cake in bin {n_in}/6 | tray max shift {_state['move'] * 1000:5.1f} mm (<= 10), "
            f"tilt {_state['tilt']:4.1f}, turn {_state['turn']:4.1f} deg (<= 5) | steady {_steady()}",
            flush=True,
        )


def score(objects) -> float:
    """The share of the cake in the container, halved if the tray ever shifted more than 1 cm or tilted or turned
    more than 5 degrees."""
    _update(objects)
    if DEBUG:
        _debug(objects)
    share = sum(in_bin(objects[c.name]) for c in CAKE) / len(CAKE)
    return round(share * (1.0 if _steady() else 0.5), 3)


def success(objects) -> bool:
    """All the cake in the container and at rest, the tray kept steady throughout; must hold 1 s. Reads the record
    that score() keeps and doesn't update it."""
    if not _state or not _steady():
        return False
    return all(in_bin(objects[c.name]) and objects[c.name].resting() for c in CAKE)


def distance(objects) -> float:
    """Summed metres from each cube to the container's centre."""
    bin_y = (BIN_Y_HI + BIN_Y_LO) / 2
    return float(sum(math.hypot(float(objects[c.name].pos[0]) - BIN_X, float(objects[c.name].pos[1]) - bin_y) for c in CAKE))


# ---------------------------------------------------------------- demo
TRAVEL_Z = TOP + 0.14
# Left arm holds the tray's left end down with its closed fingers, as demo.py's press_plate presses its plate: the
# finger tips are commanded 5 mm into the filter surface. (Gripping the tray from the sides failed: one finger landed
# on the tray's top edge, rode up onto it and rocked the tray.) 13 cm from where the spatula starts.
HOLD_XY = (TRAY_X, 0.14)
HOLD_TOUCH_Z = TRAY_TOP + PAD_TIP_PAST_TCP + 0.005  # tips 5 mm above the surface
HOLD_PRESS_Z = TRAY_TOP + PAD_TIP_PAST_TCP - 0.005  # tips commanded 5 mm into it
# The gripper holds the spatula front to back only, so it can sit a few mm off-centre side to side: 6 mm
# clearance to each wall (a spatula 2 mm from the walls caught on the wall tops).
# Right arm: grasp the spatula at its centre (as the example grasps its cube), so its edge hangs 6.5 cm below the TCP
# and the gripper stays well above the tray and the side walls.
SPAT_HALF = SPATULA.size[2] / 2
SPAT_HALF_T = SPATULA.size[1] / 2
SCRAPE_Y0 = CAKE_ROWS_Y[-1] + CUBE / 2 + 0.003 + SPAT_HALF_T  # 3 mm behind the cake
SCRAPE_Y1 = TRAY_Y_LO - SPAT_HALF_T - 0.004  # past the tray's right end, over the container
TOUCH_Z = TRAY_TOP + 0.004 + SPAT_HALF  # edge 4 mm above the filter surface
PRESS_Z = TRAY_TOP + SPAT_HALF  # edge on the surface: pressed 1 mm into it, the spatula dragged the tray out of the hand
LEFT_GRIPPED = 1.5 + 2.0 + 0.8  # the left arm's first three segments: over the end, lower, press
RIGHT_WAIT = LEFT_GRIPPED + 0.3  # the left arm has closed on the tray before the right arm sets off
SCRAPE_S = 5.0  # slow, so the cake is pushed rather than flung against the tray


def _right() -> list[Waypoint]:
    sx, sy, sz = SPATULA.pos
    return [
        Waypoint(RIGHT_WAIT, (sx, sy, TRAVEL_Z), FINGER_OPEN, "wait above the spatula"),
        Waypoint(1.5, (sx, sy, sz), FINGER_OPEN, "descend"),
        Waypoint(0.8, (sx, sy, sz), FINGER_CLOSED, "grip"),
        Waypoint(1.2, (sx, sy, TRAVEL_Z), FINGER_CLOSED, "lift"),
        Waypoint(1.8, (TRAY_X, SCRAPE_Y0, TRAVEL_Z), FINGER_CLOSED, "over the tray"),
        Waypoint(1.2, (TRAY_X, SCRAPE_Y0, TOUCH_Z), FINGER_CLOSED, "lower behind the cake"),
        Waypoint(0.8, (TRAY_X, SCRAPE_Y0, PRESS_Z), FINGER_CLOSED, "edge on the filter"),
        Waypoint(SCRAPE_S, (TRAY_X, SCRAPE_Y1, PRESS_Z), FINGER_CLOSED, "scrape into the container"),
        Waypoint(1.0, (TRAY_X, SCRAPE_Y1, TRAVEL_Z), FINGER_CLOSED, "lift"),
        Waypoint(1.8, (sx, sy, TRAVEL_Z), FINGER_CLOSED, "carry back"),
        Waypoint(1.2, (sx, sy, sz + 0.005), FINGER_CLOSED, "set down"),
        Waypoint(0.6, (sx, sy, sz + 0.005), FINGER_OPEN, "release"),
        Waypoint(1.0, (sx, sy, TRAVEL_Z), FINGER_OPEN, "retreat"),
    ]


def _left(hold_s: float) -> list[Waypoint]:
    x, y = HOLD_XY
    return [
        Waypoint(1.5, (x, y, TRAVEL_Z), FINGER_CLOSED, "over the tray's end, fingers closed"),
        Waypoint(2.0, (x, y, HOLD_TOUCH_Z), FINGER_CLOSED, "lower onto the tray"),
        Waypoint(0.8, (x, y, HOLD_PRESS_Z), FINGER_CLOSED, "press it down"),
        Waypoint(hold_s, (x, y, HOLD_PRESS_Z), FINGER_CLOSED, "hold steady"),
        Waypoint(1.0, (x, y, TRAVEL_Z), FINGER_CLOSED, "rise"),
    ]


def demo():
    """Left arm presses the tray's left end down; right arm takes the spatula, scrapes the cake off the right end into the
    container, and puts the spatula back. The left arm lifts off once the spatula has left the tray.
    SCRAPE_DEMO_NO_HOLD=1 plays the right arm alone, to check that the tray does slide when nobody holds it."""
    right = _right()
    if os.environ.get("SCRAPE_DEMO_NO_HOLD") == "1":
        return {"right": right}
    t_lifted = 0.0
    for w in right:
        t_lifted += w.duration
        if w.label == "scrape into the container":
            break
    t_lifted += 1.0  # the spatula's lift off the tray
    return {"left": _left(t_lifted + 0.5 - LEFT_GRIPPED), "right": right}