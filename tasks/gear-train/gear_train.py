"""On the station table, right of the midline: a fixed grey gear plate (150 x 75 mm) whose long side
runs out from the robot, with three upright shafts along its centre line, their tips 26 mm above the table. The shafts
are 28.0, 33.0 and 36.1 cm out from the arm bases (50.6 mm, then 30.5 mm apart), all 15 cm right of the midline. Three
loose spur gears, each 25 mm thick, lie flat on the table beside the plate: a large one (62 mm across) 40 cm out and
32 cm right, a medium one (42 mm) 24 cm out and 30 cm right, and a small one (21.8 mm) 20 cm out and 24 cm right.

The large gear goes on the nearest shaft, the medium on the middle one and the small on the farthest, where the medium
gear's teeth mesh with both others. A gear slides down its shaft only when its teeth line up with the gaps of the gear
it meshes with; out of phase it stops on the other gear's teeth, 7 to 10 mm above its seat. Seated, a gear's bottom is
6 mm above the table.
"""

import math

import numpy as np

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Asset

SCENE = "station"
TOP = TABLE_TOP[SCENE]

PLATE_XY = (0.31, -0.15)
GEARS = ("large_gear", "medium_gear", "small_gear")
SHAFT_DX = {"large_gear": -0.0303, "medium_gear": 0.0203, "small_gear": 0.0508}  # along the plate, from its centre
START = {"large_gear": (0.40, -0.32), "medium_gear": (0.24, -0.30), "small_gear": (0.20, -0.24)}
DIAMETER = {"large_gear": 0.062, "medium_gear": 0.042, "small_gear": 0.0218}
MASS = {"large_gear": 0.06, "medium_gear": 0.03, "small_gear": 0.01}

OBJECTS = [Asset("plate", "gear_base", xy=PLATE_XY, extent=0.150)] + [
    Asset(n, n, xy=START[n], extent=DIAMETER[n], mass=MASS[n]) for n in GEARS]

SEAT = TOP + 0.006  # a seated gear's bottom: measured in the sim, 5.9 to 6.6 mm over three gears
SEAT_TOL = 0.001
AXIS_TOL = 0.002  # gear centre to shaft axis; the shafts are 30 mm or more apart
MAX_TILT = 3.0  # degrees


def shaft(name):
    """The (x, y) of the shaft ``name`` belongs on."""
    return np.array([PLATE_XY[0] + SHAFT_DX[name], PLATE_XY[1]])


def tilt_deg(o):
    """Angle between the gear's axis and vertical."""
    x, y = o.quat_xyzw[0], o.quat_xyzw[1]
    return math.degrees(math.acos(float(np.clip(1 - 2 * (x * x + y * y), -1.0, 1.0))))


def off_axis(objects, name):
    return float(np.linalg.norm(objects[name].pos[:2] - shaft(name)))


def seated(objects, name):
    """All the way down its own shaft and level."""
    o = objects[name]
    return off_axis(objects, name) <= AXIS_TOL and abs(o.low[2] - SEAT) <= SEAT_TOL and tilt_deg(o) <= MAX_TILT


def success(objects):
    """All three gears seated on their shafts and still. Holds for 1 s."""
    return all(seated(objects, n) and objects[n].resting() for n in GEARS)


def distance(objects):
    return sum(off_axis(objects, n) for n in GEARS)


# WHY: only the medium gear meshes with the others (the large and small shafts are 81 mm apart, their tips 42 mm
# across), so credit goes to meshes: seating the large and small gears with no phase matching pays only 0.2
PAIRS = (("large_gear", "medium_gear"), ("medium_gear", "small_gear"))


def score(objects):
    """Partial credit (success scores 1.0): 0.1 for each seated gear and 0.3 for each meshing pair with both gears
    seated, so 0.2 with no mesh, 0.5 with one and 1.0 with all three seated."""
    down = {n for n in GEARS if seated(objects, n)}
    if len(down) == len(GEARS):
        return 1.0
    return round(0.1 * len(down) + 0.3 * sum(a in down and b in down for a, b in PAIRS), 2)


GRIP_Z = 0.016  # TCP height above a lying gear's bottom when the pads close on its rim
# Gear centre minus TCP at wrist yaw 0 after the lift, measured in the sim for this pick order
GRIP = {"large_gear": (0.0004, -0.0011), "medium_gear": (0.0007, -0.0013), "small_gear": (0.0006, -0.0001)}
# WHY: the arm wanders up to 3 mm off its target while holding a gear and the bore catches the tip only within about
# 2.5 mm, so each gear rests on its shaft's tip and the wrist spirals in from 3 mm until the bore drops over it
SPIRAL_R, SPIRAL_TURNS, SPIRAL_STEPS, SPIRAL_S = 0.003, 2, 16, 4.0
PRESS = 0.004  # during the phase search the TCP aims 4 mm above its seated height: light contact on the teeth
# WHY: friction drags the meshed gear along while a gear turns on its teeth, so only about a third of the turn changes
# the phase (measured). The sweeps cover 2.2 tooth pitches of the medium gear (9 deg) and 2.2 of the small one (18 deg)
SWEEP = {"medium_gear": math.radians(10), "small_gear": math.radians(20)}  # half-widths
SWEEP_S = {"medium_gear": 8.0, "small_gear": 10.0}


def demo():
    """The right arm seats the large gear, then the medium and the small ones. It knows neither where the arm will
    hold the gear to the millimetre nor how the teeth below are turned, so each insertion searches by contact: it
    rests the gear on its shaft's tip and spirals until the bore drops over it, then presses the gear lightly onto the
    teeth it meshes with and turns it through more than a tooth until it drops into mesh, then pushes it home."""
    hi = TOP + 0.10
    seat_tcp = SEAT + GRIP_Z
    above = seat_tcp + 0.029  # gear bottom 35 mm up: clear of the shaft tips and the seated gears' tops (31 mm)
    tip_tcp = TOP + 0.020 + GRIP_Z  # gear bottom 20 mm up: over the shaft if the bore caught its tip, above the teeth
    w = []

    def pick(name):
        x, y = START[name]
        return [Waypoint(2.0, (x, y, hi), FINGER_OPEN, f"to {name}"),
                Waypoint(2.0, (x, y, hi), FINGER_OPEN, "settle over"),  # WHY: an off-centre grip tilts the gear 3 deg
                Waypoint(1.5, (x, y, TOP + GRIP_Z), FINGER_OPEN, "down"),
                Waypoint(1.0, (x, y, TOP + GRIP_Z), FINGER_OPEN, "settle"),
                Waypoint(0.8, (x, y, TOP + GRIP_Z), FINGER_CLOSED, "grip"),
                Waypoint(1.5, (x, y, hi), FINGER_CLOSED, "lift")]

    def insert(name, via_top=True, dwell=1.5):
        def at(a, z, dx=0.0, dy=0.0):
            sx, sy = shaft(name)
            gx, gy = GRIP[name]
            c, s = math.cos(a), math.sin(a)
            return (sx + dx - (c * gx - s * gy), sy + dy - (s * gx + c * gy), z)
        half = SWEEP.get(name, 0.0)
        w = [Waypoint(2.0, at(0.0, hi), FINGER_CLOSED, "over shaft")] if via_top else []
        w += [Waypoint(2.0, at(0.0, above), FINGER_CLOSED, "above"),
              Waypoint(dwell, at(0.0, above), FINGER_CLOSED, "settle"),
              Waypoint(1.0, at(-half, above), FINGER_CLOSED, "turn", -half),
              Waypoint(1.5, at(-half, tip_tcp), FINGER_CLOSED, "onto the tip", -half)]
        for k in range(1, SPIRAL_STEPS + 1):
            r, ang = SPIRAL_R * (1 - k / SPIRAL_STEPS), 2 * math.pi * SPIRAL_TURNS * k / SPIRAL_STEPS
            w.append(Waypoint(SPIRAL_S / SPIRAL_STEPS, at(-half, tip_tcp, r * math.cos(ang), r * math.sin(ang)),
                              FINGER_CLOSED, "spiral", -half))
        if half:
            w += [Waypoint(1.5, at(-half, seat_tcp + PRESS), FINGER_CLOSED, "onto the teeth", -half),
                  Waypoint(SWEEP_S[name], at(half, seat_tcp + PRESS), FINGER_CLOSED, "turn into mesh", half)]
        return w + [Waypoint(1.5, at(half, seat_tcp + 0.001), FINGER_CLOSED, "push home", half),
                    Waypoint(1.0, at(half, seat_tcp + 0.001), FINGER_CLOSED, "settle low", half),
                    Waypoint(0.6, at(half, seat_tcp + 0.001), FINGER_OPEN, "release", half),
                    Waypoint(1.0, at(half, hi), FINGER_OPEN, "up", half)]

    w += pick("large_gear") + insert("large_gear")
    w += pick("medium_gear") + insert("medium_gear")
    # WHY: the small gear goes straight in low; a high approach left the arm 1.8 mm short in a probe
    w += pick("small_gear") + insert("small_gear", via_top=False, dwell=2.0)
    return {"right": w}
