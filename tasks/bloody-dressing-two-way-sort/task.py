"""A blue procedure tray in front of the arms holds the waste from a wound-dressing change: a blood-soaked gauze
dressing (dark red, 8 x 5 x 2 cm) and the empty paper-and-plastic packaging it came in (white, 8 x 5 x 2 cm).
Behind the tray stand two open-top bins of fixed walls (the table is the floor): a grey regular trash bin in the
middle of the bench and a red biohazard bin at the robot's right, each 14 x 14 cm inside and 10 cm tall.
"""

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Throw away the waste from the dressing change."  # read by our run script, not the framework

PAD = 0.013  # the TCP sits this far above the finger pad tips
T = 0.01  # wall thickness
BIN_X = 0.32
H = 0.10
IN = 0.14
BINS = {  # name: (centre y, colour)
    "trash": (0.02, (0.45, 0.45, 0.45)),
    "biohazard": (-0.19, (0.80, 0.10, 0.10)),
}


def _bin(prefix, cy, color):
    h = IN / 2 + T / 2
    return [
        Box(f"{prefix}_front", pos=(BIN_X - h, cy, TOP + H / 2), size=(T, IN + 2 * T, H), color=color),
        Box(f"{prefix}_back", pos=(BIN_X + h, cy, TOP + H / 2), size=(T, IN + 2 * T, H), color=color),
        Box(f"{prefix}_left", pos=(BIN_X, cy + h, TOP + H / 2), size=(IN, T, H), color=color),
        Box(f"{prefix}_right", pos=(BIN_X, cy - h, TOP + H / 2), size=(IN, T, H), color=color),
    ]


TRAY_TOP = TOP + 0.01
TRAY = Box("tray", pos=(0.15, 0.03, TOP + 0.005), size=(0.16, 0.24, 0.01), color=(0.20, 0.40, 0.85))
DRESSING = Box("dressing", pos=(0.15, -0.03, TRAY_TOP + 0.01), size=(0.08, 0.05, 0.02), mass=0.03,
               color=(0.45, 0.02, 0.02))
WRAPPER = Box("wrapper", pos=(0.15, 0.09, TRAY_TOP + 0.01), size=(0.08, 0.05, 0.02), mass=0.025,
              color=(0.95, 0.95, 0.95))
OBJECTS = [TRAY, *(p for name, (cy, c) in BINS.items() for p in _bin(name, cy, c)), DRESSING, WRAPPER]


def _where(s):
    """The bin an item is in (its centre inside the walls, its top below the rim, still), or None."""
    if not s.resting() or s.high[2] > TOP + H:
        return None
    for name, (cy, _c) in BINS.items():
        if abs(s.pos[0] - BIN_X) < IN / 2 and abs(s.pos[1] - cy) < IN / 2:
            return name
    return None


def success(objects):
    """1.0: the dressing in the biohazard bin, the packaging in the regular trash."""
    return _where(objects["dressing"]) == "biohazard" and _where(objects["wrapper"]) == "trash"


def score(objects):
    """0 whenever the blood-soaked dressing is not in the biohazard bin (in the trash, or left out). With the
    dressing in the biohazard bin: 0.5 if the clean packaging is in the biohazard bin too (over-classified:
    safe, but regulated waste costs several times more to treat); 0.25 if the packaging is not in a bin."""
    if _where(objects["dressing"]) != "biohazard":
        return 0.0
    w = _where(objects["wrapper"])
    if w == "trash":
        return 1.0
    return 0.5 if w == "biohazard" else 0.25


def demo():
    """Right arm: the dressing into the biohazard bin, then parks. Left arm, after it: the packaging into the
    trash. Each item is released 2 cm above the rim."""
    travel = TOP + 0.18
    right = pick_place_waypoints((DRESSING.pos[0], DRESSING.pos[1], TRAY_TOP + 0.005 + PAD),
                                 (BIN_X, BINS["biohazard"][0], TOP + H + 0.02 + 0.015 + PAD), travel_z=travel,
                                 speed=0.8)
    right.append(Waypoint(1.5, (0.15, -0.25, TOP + 0.25), FINGER_OPEN, "park"))
    wait = sum(w.duration for w in right)
    left = [Waypoint(wait, (0.15, 0.25, TOP + 0.25), FINGER_OPEN, "wait")]
    left += pick_place_waypoints((WRAPPER.pos[0], WRAPPER.pos[1], TRAY_TOP + 0.005 + PAD),
                                 (BIN_X, BINS["trash"][0], TOP + H + 0.02 + 0.015 + PAD), travel_z=travel,
                                 speed=0.8)
    return {"right": right, "left": left}
