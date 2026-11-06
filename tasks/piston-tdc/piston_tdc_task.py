"""On the station table, in front of the left arm: a small single-cylinder engine trainer with a vertical crankshaft
and a horizontal cylinder, built at the size of a small vertical-shaft engine (60 mm stroke). The crank throw,
connecting rod and piston are inside a closed dark-blue housing (11 x 22 cm, 7 cm tall); the cylinder points from
the crankshaft toward the right arm. On top of the housing, fixed to the crankshaft: a yellow degree wheel (17 cm
across) with a black tick every 10 degrees, a blue tick every 90 and a red zero tick, and a dark handle arm with an
orange handle sleeve (20 mm across, 40 mm tall, 7 cm from the axis) that spins freely on its pin. A green pointer
stands just outside the wheel's rim on the far side. The handle is not in line with the hidden crank throw, and the
wheel's zero tick is not at top dead centre: neither shows where the piston is.

A red piston stop (a T-shaped steel bar, 80 g) hangs in a slot in the housing lid near the cylinder's end: its
plate rests on the lid and a 20 x 30 x 35 mm grip block stands on top. While it is in, the piston hits it a little
before top dead centre from either side, so the crank cannot turn through that part of its turn. It lifts
straight up and out. A blue tray (16 x 14 cm inside, 12 mm rim) is fixed to the table in front of the right arm.
The crankshaft turns with a little bearing drag and stays where it is let go.
"""

from yam_arena.task_file import Box

from piston_tdc import design as d

SCENE = "station"
TRAY_COLOR = (0.10, 0.30, 0.90)


def _tray():
    (ix, iy), (t, h) = d.TRAY_INNER, d.TRAY_RIM
    x, y = (float(v) for v in d.TRAY_XY)
    top = d.TOP
    ox, oy = ix + 2 * t, iy + 2 * t
    parts = [Box("tray_floor", pos=(x, y, top + d.TRAY_FLOOR / 2), size=(ox, oy, d.TRAY_FLOOR), color=TRAY_COLOR)]
    zc = top + d.TRAY_FLOOR + h / 2
    parts += [Box("tray_rim_far", pos=(x + (ix + t) / 2, y, zc), size=(t, oy, h), color=TRAY_COLOR),
              Box("tray_rim_near", pos=(x - (ix + t) / 2, y, zc), size=(t, oy, h), color=TRAY_COLOR),
              Box("tray_rim_left", pos=(x, y + (iy + t) / 2, zc), size=(ix, t, h), color=TRAY_COLOR),
              Box("tray_rim_right", pos=(x, y - (iy + t) / 2, zc), size=(ix, t, h), color=TRAY_COLOR)]
    return parts


OBJECTS = _tray()


def success(objects):
    return False  # scored by the embodiment yam_piston_tdc (piston_tdc/scoring.py), from the trainer's part poses
