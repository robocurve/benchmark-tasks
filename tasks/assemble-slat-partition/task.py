"""On the station table: three slotted foam-board partition slats (16 mm thick, 8 cm tall), each standing on edge in a low
grey rack. A long slat (30 cm) has two slots cut halfway down from its top edge, 18 cm apart; two short slats (20 cm)
each have one slot cut halfway up from the bottom edge at their middle. The goal is to build a partition: stand the long
slat on the table at the marked spot, and slide each short slat down across it so the slots interlock, the slats'
top edges flush. A slat on edge tips over past about 11 degrees, so the first is steadied while the second goes on.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py \\
           --instruction "Build the partition: stand the long slat on the white mark and slide both short slats down across it so their slots interlock." \\
           --scorer task_score ...
"""

import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))
from rigidparts import quat_matrix, rigid_part  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

WOOD, RACK, MARK = (0.80, 0.66, 0.45), (0.55, 0.57, 0.60), (0.95, 0.95, 0.95)
T, H = 0.016, 0.08  # slat thickness and height: tall enough to grip at mid-height, over the pads' whole face
SLOT_W = 0.024  # 16 mm slats in 24 mm slots: 4 mm of play each way
LONG_L, SHORT_L = 0.30, 0.20
SLOT_Y = (0.09, -0.09)  # the long slat's slots, from its centre along its length
SLAT_MASS = 0.04


def _segments(length, gaps, z0, z1):
    """Boxes along a slat's length (its frame's x) from z0 to z1, leaving a SLOT_W gap at each position in ``gaps``."""
    edges = [-length / 2, *[e for g in sorted(gaps) for e in (g - SLOT_W / 2, g + SLOT_W / 2)], length / 2]
    return [(((a + b) / 2, 0.0, (z0 + z1) / 2), (b - a, T, z1 - z0), WOOD) for a, b in zip(edges[::2], edges[1::2])]


# Each slat's frame: origin at its centre, length along the frame's x, height along z.
LONG_BOXES = [((0.0, 0.0, -H / 4), (LONG_L, T, H / 2), WOOD), *_segments(LONG_L, SLOT_Y, 0.0, H / 2)]
SHORT_BOXES = [((0.0, 0.0, H / 4), (SHORT_L, T, H / 2), WOOD), *_segments(SHORT_L, (0.0,), -H / 2, 0.0)]

ASSEMBLY = (0.28, 0.0)  # where the long slat stands, along world y
LONG_START = (0.18, -0.15)
SHORT_STARTS = {"short_slat_left": (0.28, 0.20), "short_slat_right": (0.28, -0.22)}

LONG = rigid_part("long_slat", pos=(*LONG_START, TOP + H / 2), boxes=LONG_BOXES, mass=SLAT_MASS, yaw=np.pi / 2)
SHORTS = [rigid_part(n, pos=(*xy, TOP + H / 2), boxes=SHORT_BOXES, mass=SLAT_MASS) for n, xy in SHORT_STARTS.items()]


def _rack(name, centre, along_y):
    """Two low blocks either side of a standing slat, 3 cm long, 2 cm tall, 1 mm clear of its faces."""
    off = T / 2 + 0.001 + 0.004
    out = []
    for i, s in enumerate((-1, 1)):
        if along_y:
            pos, size = (centre[0] + s * off, centre[1], TOP + 0.01), (0.008, 0.03, 0.02)
        else:
            pos, size = (centre[0], centre[1] + s * off, TOP + 0.01), (0.03, 0.008, 0.02)
        out.append(Box(f"{name}_{i}", pos=pos, size=size, color=RACK))
    return out


RACKS = [
    *_rack("rack_long", (LONG_START[0], LONG_START[1] + 0.10), along_y=True),
    *_rack("rack_short_left", (SHORT_STARTS["short_slat_left"][0] - 0.04, SHORT_STARTS["short_slat_left"][1]), along_y=False),
    *_rack("rack_short_right", (SHORT_STARTS["short_slat_right"][0] - 0.04, SHORT_STARTS["short_slat_right"][1]), along_y=False),
]
MARK_LINE = Box("assembly_mark", pos=(ASSEMBLY[0] - 0.012, ASSEMBLY[1], TOP + 0.0005), size=(0.006, LONG_L, 0.001), color=MARK)
OBJECTS = [LONG, *SHORTS, *RACKS, MARK_LINE]


def _upright(state) -> bool:
    """Standing on edge: its bounding box as tall as the slat (within 5 mm), and on the table."""
    return bool(abs((state.high[2] - state.low[2]) - H) < 0.005 and state.low[2] < TOP + 0.004)


# Measured in the long slat's own frame (length along its x, thickness along its y): the short slat's centre, which is
# where its slot is, must sit in one of the long slat's slots. Across the long slat the slot leaves (SLOT_W - T) / 2 =
# 4 mm of play; along it, 6 mm. A short slat outside a slot is at least 2 cm off, so this can't credit a miss.
ALONG, ACROSS = 0.006, (SLOT_W - T) / 2 + 0.002
MARK_TOL, MARK_TURN = 0.02, 5.0  # how far (m) and how much (degrees) the long slat may sit off the white line


def interlocked(objects, short: str, slot_y: float) -> bool:
    """A short slat crosses the long one at a slot, slots interlocked: both upright on the table, the short slat's centre
    in the slot (within 6 mm along the long slat and 6 mm across it, in the long slat's frame), top edges flush within
    4 mm."""
    long_, s = objects["long_slat"], objects[short]
    local = quat_matrix(long_.quat_xyzw).T @ (np.asarray(s.pos) - np.asarray(long_.pos))
    return bool(
        _upright(long_) and _upright(s)
        and abs(local[0] - slot_y) < ALONG
        and abs(local[1]) < ACROSS
        and abs(s.high[2] - long_.high[2]) < 0.004
    )


def _crossings(objects) -> int:
    """How many of the long slat's two slots hold a short slat, either short slat in either slot."""
    used, n = set(), 0
    for slot_y in SLOT_Y:
        for name in SHORT_STARTS:
            if name not in used and interlocked(objects, name, slot_y):
                used.add(name)
                n += 1
                break
    return n


def on_mark(long_) -> bool:
    """The long slat stands on the white line: its centre within 2 cm of the line's centre, turned at most 5 degrees from
    the line's direction."""
    along = abs(quat_matrix(long_.quat_xyzw)[1, 0])  # the slat's length along world y, the line's direction
    return bool(
        np.hypot(long_.pos[0] - MARK_LINE.pos[0], long_.pos[1] - MARK_LINE.pos[1]) < MARK_TOL
        and along > np.cos(np.radians(MARK_TURN))
    )


def success(objects):
    """Both crossings interlocked, the long slat on the white line, everything still; must hold for 1 s."""
    return (
        _crossings(objects) == 2
        and on_mark(objects["long_slat"])
        and all(objects[n].resting() for n in ("long_slat", *SHORT_STARTS))
    )


def score(objects):
    """Partial credit (success scores 1.0): 0.4 for each crossing interlocked."""
    return 0.4 * _crossings(objects)


# How far each slat sits from where the demo grips it, measured in a traced run (closing the fingers shifts it a few
# millimetres, the same way every run with -E object_jitter=0).
LONG_GRIP_SHIFT = (0.0, 0.0)  # long slat centre minus its intended position while held (x, y; m)
SHORT_GRIP_SHIFT = {"short_slat_left": (0.003, 0.0), "short_slat_right": (0.018, -0.004)}  # the same, per short slat


def demo():
    """Right arm: take the long slat out of its rack by the middle of its top edge, stand it on the mark and hold it. Left arm,
    meanwhile: take the left short slat out of its rack, carry it above the long slat's left slot and, once the long
    slat is in place, lower it across. The cross now stands by itself: the right arm lets go, takes the right short slat
    and lowers it into the other slot."""
    hover, grip_dz = TOP + 0.16, H / 2 - 0.035  # grip 3.5 cm down from the top edge, so the pads hold mid-height
    up = 0.11  # carried with its bottom 11 cm up: 3 cm over the standing slats' top edges
    lx, ly = LONG_GRIP_SHIFT
    sxl, syl = SHORT_GRIP_SHIFT["short_slat_left"]
    sxr, syr = SHORT_GRIP_SHIFT["short_slat_right"]
    part = FINGER_OPEN * 0.5

    # The long slat: gripped 4.5 cm right of its centre, between its slots (close enough to its centre not to swing),
    # so the two hands stay about 16 cm apart while the left arm brings the first short slat in; fingers across it.
    g = -0.045
    l0 = (LONG_START[0], LONG_START[1] + g)
    l1 = (ASSEMBLY[0] - lx, ASSEMBLY[1] + g - ly)
    yaw_long = np.pi / 2
    z_grip = TOP + H / 2 + grip_dz
    right = [
        Waypoint(1.5, (*l0, hover), part, "above the long slat", yaw_long),
        Waypoint(1.5, (*l0, z_grip), part, "down to its top edge", yaw_long),
        Waypoint(0.8, (*l0, z_grip), FINGER_CLOSED, "grip", yaw_long),
        # Carried 10 cm up: its end sweeps past where the right short slat stands, so it has to clear that slat's top.
        Waypoint(1.5, (*l0, z_grip + 0.10), FINGER_CLOSED, "lift out of the rack", yaw_long),
        Waypoint(3.0, (*l1, z_grip + 0.10), FINGER_CLOSED, "carry to the mark", yaw_long),
        Waypoint(1.5, (*l1, z_grip + 0.002), FINGER_CLOSED, "stand it on the table", yaw_long),
        Waypoint(5.5, (*l1, z_grip + 0.002), FINGER_CLOSED, "hold it up while the first short slat goes on", yaw_long),
        Waypoint(0.6, (*l1, z_grip + 0.002), part, "let go", yaw_long),
        Waypoint(1.0, (*l1, hover), part, "up", yaw_long),
    ]
    # A short slat: gripped 8 cm from its centre toward its far end, clear of the long slat and the other hand.
    gs = 0.08
    r0 = SHORT_STARTS["short_slat_right"]
    r_target = (ASSEMBLY[0] + gs - sxr, ASSEMBLY[1] + SLOT_Y[1] - syr)
    right += [
        Waypoint(1.2, (r0[0] + gs, r0[1], hover), part, "above the right short slat"),
        Waypoint(1.2, (r0[0] + gs, r0[1], z_grip), part, "down to its top edge"),
        Waypoint(0.8, (r0[0] + gs, r0[1], z_grip), FINGER_CLOSED, "grip"),
        Waypoint(1.0, (r0[0] + gs, r0[1], z_grip + up), FINGER_CLOSED, "lift it clear"),
        Waypoint(2.5, (*r_target, z_grip + up), FINGER_CLOSED, "over the right slot"),
        Waypoint(1.5, (*r_target, z_grip + 0.002), FINGER_CLOSED, "slide it down across"),
        Waypoint(0.6, (*r_target, z_grip + 0.002), part, "let go"),
        Waypoint(1.0, (*r_target, hover), part, "back up"),
    ]
    s0 = SHORT_STARTS["short_slat_left"]
    s_target = (ASSEMBLY[0] + gs - sxl, ASSEMBLY[1] + SLOT_Y[0] - syl)
    left = [
        Waypoint(1.5, (s0[0] + gs, s0[1], hover), part, "above the left short slat"),
        Waypoint(1.5, (s0[0] + gs, s0[1], z_grip), part, "down to its top edge"),
        Waypoint(0.8, (s0[0] + gs, s0[1], z_grip), FINGER_CLOSED, "grip"),
        Waypoint(1.0, (s0[0] + gs, s0[1], z_grip + up), FINGER_CLOSED, "lift it clear"),
        # Wait out of the long slat's path (it passes under here on its way to the mark), then move over its slot.
        Waypoint(5.0, (s0[0] + gs, s0[1], z_grip + up), FINGER_CLOSED, "wait above the rack for the long slat"),
        Waypoint(2.0, (*s_target, z_grip + up), FINGER_CLOSED, "over the left slot"),
        Waypoint(1.5, (*s_target, z_grip + 0.002), FINGER_CLOSED, "slide it down across"),
        Waypoint(0.6, (*s_target, z_grip + 0.002), part, "let go"),
        Waypoint(1.0, (*s_target, hover), part, "back up"),
    ]
    return {"left": left, "right": right}
