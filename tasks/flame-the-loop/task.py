"""A microbiology bench. On the robot's right stands a Bunsen burner (a grey post, 3 cm across and 7 cm tall, on a
dark 7 x 7 cm base, its top 2.5 cm orange for the flame) and, nearer the arms, the loop holder: a small open cup of four fixed grey walls,
5 cm tall, around a 4 x 4 cm inside. The inoculating loop stands upright in the cup: a slim rod, 2 x 2 x 10 cm, whose
bottom end is the wire loop. Beyond the burner is the source plate: a petri dish of amber agar (9 cm across, 1.2 cm
tall) with three white bacterial colonies in its middle. In the middle of the bench is a fresh agar plate, the same
size, with nothing on it.
"""

import numpy as np

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Streak the sample onto the fresh plate."

LOOP = (0.02, 0.02, 0.10)
CUP_X, CUP_Y, CUP_IN, CUP_H, WALL = 0.20, -0.20, 0.04, 0.05, 0.01
BURNER_X, BURNER_Y, BURNER_R, BURNER_H = 0.28, -0.12, 0.015, 0.07
BURNER_TOP = TOP + BURNER_H
FLAME_R, FLAME_H = 0.025, 0.05  # the flame: within 2.5 cm of the burner's axis, up to 5 cm above its top
FLAME_STEPS = 10  # the tip held in the flame for 10 control steps in a row (0.5 s at 20 Hz) sterilizes it
PLATE_R, PLATE_H = 0.045, 0.012
SOURCE = (0.36, -0.22)
FRESH = (0.30, -0.01)
PLATE_TOP = TOP + PLATE_H
TOUCH = 0.004  # the tip within 4 mm of the agar touches it
COLONIES = [(0.0, 0.0), (0.012, 0.008), (-0.01, 0.01)]
STREAK_MIN = 0.06  # at least 6 cm of tip travel across the fresh agar, with bacteria on the loop

AGAR = (0.93, 0.74, 0.38)
GREY = (0.55, 0.57, 0.60)


def _cup():
    half, z = CUP_IN / 2 + WALL / 2, TOP + CUP_H / 2
    long = CUP_IN + 2 * WALL
    return [
        Box("holder_back", pos=(CUP_X + half, CUP_Y, z), size=(WALL, long, CUP_H), color=GREY),
        Box("holder_front", pos=(CUP_X - half, CUP_Y, z), size=(WALL, long, CUP_H), color=GREY),
        Box("holder_left", pos=(CUP_X, CUP_Y + half, z), size=(CUP_IN, WALL, CUP_H), color=GREY),
        Box("holder_right", pos=(CUP_X, CUP_Y - half, z), size=(CUP_IN, WALL, CUP_H), color=GREY),
    ]


OBJECTS = [
    *_cup(),
    Box("burner_base", pos=(BURNER_X, BURNER_Y, TOP + 0.005), size=(0.07, 0.07, 0.01), color=(0.25, 0.25, 0.28)),
    Cylinder("burner", pos=(BURNER_X, BURNER_Y, TOP + (BURNER_H - 0.025) / 2), radius=BURNER_R,
             height=BURNER_H - 0.025, color=GREY),
    Cylinder("flame", pos=(BURNER_X, BURNER_Y, BURNER_TOP - 0.0125), radius=BURNER_R, height=0.025,
             color=(1.0, 0.45, 0.05)),
    Cylinder("source_plate", pos=(*SOURCE, TOP + PLATE_H / 2), radius=PLATE_R, height=PLATE_H, color=AGAR),
    *[Cylinder(f"colony_{i}", pos=(SOURCE[0] + dx, SOURCE[1] + dy, PLATE_TOP + 0.001), radius=0.005, height=0.002,
               color=(0.97, 0.97, 0.92)) for i, (dx, dy) in enumerate(COLONIES)],
    Cylinder("fresh_plate", pos=(*FRESH, TOP + PLATE_H / 2), radius=PLATE_R, height=PLATE_H, color=AGAR),
    Box("loop", pos=(CUP_X, CUP_Y, TOP + LOOP[2] / 2), size=LOOP, mass=0.03, color=(0.20, 0.35, 0.70)),
]
_START = {o.name: np.array(o.pos) for o in OBJECTS if o.mass is not None}


# --- What happened during the run. success() and score() are called after every control step, with no
# reset hook, so the run's history is kept here. A reset puts every free object back at its start, at rest, in one
# step; that jump is how a new trial is recognized. Objects moved by the arms travel a few cm per step at most.
class _History:
    def __init__(self):
        self.last = None
        self.flame_run = 0
        self.sterile = False  # flamed since it last touched anything
        self.loaded = False  # carrying bacteria from the source plate
        self.last_tip = None  # tip position at the previous step, for the streak length
        self.streak = 0.0  # cm of loaded tip travel across the fresh agar
        self.violation = None  # first breach of aseptic technique (never forgiven)


_H = _History()


def _tip(s):
    """The loop's tip: the middle of its lowest face (world-aligned box), so it works whichever way it is held."""
    return np.array([(s.low[0] + s.high[0]) / 2, (s.low[1] + s.high[1]) / 2, s.low[2]])


def _in_flame(t):
    return bool(np.hypot(t[0] - BURNER_X, t[1] - BURNER_Y) < FLAME_R and BURNER_TOP <= t[2] < BURNER_TOP + FLAME_H)


def _on_plate(t, center):
    return bool(np.hypot(t[0] - center[0], t[1] - center[1]) < PLATE_R and t[2] < PLATE_TOP + TOUCH)


def _at_colony(t):
    return _on_plate(t, SOURCE) and min(np.hypot(t[0] - SOURCE[0] - dx, t[1] - SOURCE[1] - dy)
                                        for dx, dy in COLONIES) < 0.015


def _in_holder(s):
    return bool(abs(s.pos[0] - CUP_X) < CUP_IN / 2 and abs(s.pos[1] - CUP_Y) < CUP_IN / 2
                and s.low[2] < TOP + CUP_H)


def _on_bench(t):
    """The tip down on the bench top (not on a plate, not in the holder)."""
    return t[2] < TOP + 0.005 and not (abs(t[0] - CUP_X) < CUP_IN / 2 + WALL and abs(t[1] - CUP_Y) < CUP_IN / 2 + WALL)


def _update(objects):
    global _H
    now = {n: np.asarray(objects[n].pos, dtype=float) for n in _START}
    if _H.last is not None:
        jumped = any(np.linalg.norm(now[n] - _H.last[n]) > 0.06 for n in now)
        at_start = all(np.linalg.norm(now[n] - _START[n]) < 0.015 for n in now)
        still = all(objects[n].resting() for n in now)
        if jumped and at_start and still:
            _H = _History()
    _H.last = now
    s = objects["loop"]
    t = _tip(s)
    _H.flame_run = _H.flame_run + 1 if _in_flame(t) else 0
    if _H.flame_run >= FLAME_STEPS:
        _H.sterile, _H.loaded = True, False  # the flame kills whatever was on it

    def breach(why):
        if _H.violation is None:
            _H.violation = why

    if _on_plate(t, SOURCE):
        if not _H.sterile and not _H.loaded:
            breach("touched the source culture with an unflamed loop")
        if _at_colony(t):
            _H.loaded = True
        _H.sterile = False
    if _on_plate(t, FRESH):
        if not _H.sterile and not _H.loaded:
            breach("touched the fresh plate with an unflamed loop")
        if _H.loaded and _H.last_tip is not None and _on_plate(_H.last_tip, FRESH):
            _H.streak += float(np.hypot(*(t[:2] - _H.last_tip[:2])))
        _H.sterile = False
    if _H.loaded and _in_holder(s):
        breach("put the loop back in its holder with live bacteria on it")
    if _H.loaded and _on_bench(t):
        breach("set the loop down on the bench with live bacteria on it")
    _H.last_tip = t


def _streaked():
    return _H.streak >= STREAK_MIN


def success(objects):
    """1.0: the loop was flamed, picked up a colony, streaked the fresh plate, was flamed again and stands back in its
    holder, with no breach of aseptic technique. Called first each step, so it also records the step's history."""
    _update(objects)
    s = objects["loop"]
    return (_H.violation is None and _streaked() and not _H.loaded and _H.sterile
            and _in_holder(s) and s.resting())


def score(objects):
    """0 after any breach, even if put right later: an unflamed loop touching either plate, or a loop with bacteria
    on it going into the holder or down on the bench. Otherwise 1.0 when the plate is streaked (6 cm or more of tip
    travel across the fresh agar after picking up a colony), the loop was flamed again afterwards and stands in its
    holder; 0.5 when the plate is streaked but the loop is not flamed and back in its holder; 0 if the plate is not
    streaked. Possible scores: 0, 0.5, 1."""
    if _H.violation is not None or not _streaked():
        return 0.0
    s = objects["loop"]
    return 1.0 if (not _H.loaded and _H.sterile and _in_holder(s) and s.resting()) else 0.5


def demo():
    """Right arm: lift the loop out of its holder, hold the tip in the flame, touch a colony on the source plate,
    zigzag across the fresh plate, flame the tip again and stand the loop back in its holder."""
    o, c = FINGER_OPEN, FINGER_CLOSED
    grip_dz = LOOP[2] - 0.012 + 0.013  # TCP above the tip, with the pads 12 mm below the loop's top
    travel = TOP + 0.24

    def at(xy, tip_z, dur, finger=c, label=""):
        return Waypoint(dur, (xy[0], xy[1], tip_z + grip_dz), finger, label)

    flame_z = BURNER_TOP + 0.02
    fx, fy = FRESH
    zig = [(fx - 0.03, fy - 0.025), (fx - 0.01, fy + 0.025), (fx + 0.01, fy - 0.025), (fx + 0.03, fy + 0.025)]
    touch_z = PLATE_TOP + 0.001
    right = [
        at((CUP_X, CUP_Y), TOP + 0.10, 1.5, o, "over the holder"),
        at((CUP_X, CUP_Y), TOP, 1.5, o, "down to the loop"),
        at((CUP_X, CUP_Y), TOP, 0.8, c, "grip"),
        at((CUP_X, CUP_Y), travel - grip_dz, 1.2, c, "lift out"),
        at((BURNER_X, BURNER_Y), travel - grip_dz, 1.2, c, "over the burner"),
        at((BURNER_X, BURNER_Y), flame_z, 1.0, c, "tip into the flame"),
        at((BURNER_X, BURNER_Y), flame_z, 1.5, c, "flame"),
        at((BURNER_X, BURNER_Y), travel - grip_dz, 1.0, c, "out of the flame"),
        at(SOURCE, travel - grip_dz, 1.2, c, "over the source plate"),
        at(SOURCE, PLATE_TOP + 0.02, 1.0, c, "down"),
        at(SOURCE, touch_z, 0.8, c, "touch a colony"),
        at(SOURCE, travel - grip_dz, 1.0, c, "up"),
        at(zig[0], travel - grip_dz, 1.5, c, "over the fresh plate"),
        at(zig[0], PLATE_TOP + 0.02, 1.0, c, "down"),
        at(zig[0], touch_z, 0.6, c, "touch the agar"),
        *[at(p, touch_z, 1.2, c, "streak") for p in zig[1:]],
        at(zig[-1], travel - grip_dz, 1.0, c, "up"),
        at((BURNER_X, BURNER_Y), travel - grip_dz, 1.5, c, "back to the burner"),
        at((BURNER_X, BURNER_Y), flame_z, 1.0, c, "tip into the flame"),
        at((BURNER_X, BURNER_Y), flame_z, 1.5, c, "flame again"),
        at((BURNER_X, BURNER_Y), travel - grip_dz, 1.0, c, "out of the flame"),
        at((CUP_X, CUP_Y), travel - grip_dz, 1.2, c, "over the holder"),
        at((CUP_X, CUP_Y), TOP + 0.002, 1.5, c, "into the holder"),
        at((CUP_X, CUP_Y), TOP + 0.002, 1.0, c, "settle to the bottom"),
        at((CUP_X, CUP_Y), TOP + 0.002, 0.6, o, "release"),
        at((CUP_X, CUP_Y), TOP + 0.12, 1.0, o, "up"),
        Waypoint(1.2, (0.16, -0.26, TOP + 0.22), o, "park"),
        Waypoint(2.0, (0.16, -0.26, TOP + 0.22), o, "hold"),
    ]
    return {"right": right}
