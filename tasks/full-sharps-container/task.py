"""A blue procedure tray in front of the arms with a used syringe (white, 13 x 1.8 x 1.8 cm) lying on it.
Behind the tray stand two red open-top sharps containers (16 x 12 cm inside, 10 cm tall, fixed walls, the
table as the floor). The left one is filled with grey sharps up to its fill line (7.5 cm); the right one is
new and empty.
"""

from yam_arena.demo import pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Put the used syringe in the sharps container."  # read by our run script, not the framework

RED = (0.80, 0.10, 0.10)
H = 0.10  # container wall height
FILL = 0.075  # fill line, 3/4 of the height
IN_X, IN_Y = 0.16, 0.12  # inside dimensions
T = 0.01  # wall thickness
CX = 0.29  # both containers' centre x
FULL_Y, EMPTY_Y = 0.16, -0.16


def _container(prefix, cy):
    """Four fixed walls around a 16 x 12 cm opening; the table is the floor."""
    hx, hy = IN_X / 2 + T / 2, IN_Y / 2 + T / 2
    return [
        Box(f"{prefix}_front", pos=(CX - hx, cy, TOP + H / 2), size=(T, IN_Y + 2 * T, H), color=RED),
        Box(f"{prefix}_back", pos=(CX + hx, cy, TOP + H / 2), size=(T, IN_Y + 2 * T, H), color=RED),
        Box(f"{prefix}_left", pos=(CX, cy + hy, TOP + H / 2), size=(IN_X, T, H), color=RED),
        Box(f"{prefix}_right", pos=(CX, cy - hy, TOP + H / 2), size=(IN_X, T, H), color=RED),
    ]


TRAY = Box("tray", pos=(0.17, 0.0, TOP + 0.005), size=(0.20, 0.14, 0.01), color=(0.20, 0.40, 0.85))
SYRINGE = Box("syringe", pos=(0.17, 0.0, TOP + 0.01 + 0.009), size=(0.13, 0.018, 0.018), mass=0.01,
              color=(0.95, 0.95, 0.95))
# The full container's contents: a fixed grey block filling it to the fill line.
CONTENTS = Box("full_contents", pos=(CX, FULL_Y, TOP + FILL / 2), size=(IN_X - 0.002, IN_Y - 0.002, FILL),
               color=(0.55, 0.55, 0.55))
OBJECTS = [TRAY, SYRINGE, CONTENTS, *_container("full", FULL_Y), *_container("empty", EMPTY_Y)]


def _inside(s, cy):
    """The syringe's whole bounding box is within the container's opening and below its rim."""
    return bool(
        s.low[0] > CX - IN_X / 2 and s.high[0] < CX + IN_X / 2
        and s.low[1] > cy - IN_Y / 2 and s.high[1] < cy + IN_Y / 2
        and s.high[2] < TOP + H
    )


def success(objects):
    """1.0: the syringe is wholly inside the EMPTY container and still."""
    s = objects["syringe"]
    return _inside(s, EMPTY_Y) and s.resting()


def score(objects):
    """No partial credit: in or on the full container, on the tray, on the table, sticking out: all 0."""
    return 0.0


def demo():
    """Right arm: grip the syringe at its middle, carry it 14 cm up, release it 2 cm above the empty
    container's rim so it drops in."""
    grip = (SYRINGE.pos[0], SYRINGE.pos[1], TOP + 0.01 + 0.004 + 0.013)  # pad tips 4 mm above the tray
    drop = (CX, EMPTY_Y, TOP + H + 0.02 + 0.013)
    return {"right": pick_place_waypoints(grip, drop, travel_z=TOP + 0.14)}
