"""On the station table, in front of the arms: a cast-aluminium terminal box (150 deep x 155 wide x 60 mm tall,
4 mm walls, lid off, fixed in place), as on a motor or a machine. A grey PVC-sheathed control cable (3 cores x
1.5 mm2, 8 mm round) comes in through a 12 mm entry in the box's right wall; inside, its sheath is stripped back and
its three cores (brown, blue, green-yellow, taped together) are already connected to the blue terminal block at the
box's far left side. 57 mm inside the entry wall is the cord clamp: the cable lies on a saddle under a steel clamp bar
(32 x 8 x 3 mm) held by two M4 socket cap screws (3 mm hexagon socket), which are loose: the bar rests on the
cable. The sheath's cut end lies under the bar's outer half (1 mm short of its centre line), so the bar is mostly over
the cores. Outside the box the cable lies on the table. The cores' bow is springy: pushed in and let go before
the clamp grips it, the cable springs back (the sheath's end to about 3-5 mm past the bar's centre line). A stubby 3 mm hex screwdriver (red handle) stands in a steel holder to the left of the box.

Each screw tightens clockwise (seen from above); about 1.5 turns bring its head onto the bar, and from 0.25 to 1.75
turns past that it is snug; 2 turns past that it crushes the cable.

Task: secure the cable in the terminal box's cord clamp: slide the cable in until its sheath reaches at least 5 mm
past the clamp bar and hold it there while you tighten the bar's two screws in turns with the hex screwdriver, so
that the bar presses on the sheath and stays parallel. Then let go of the cable and the screwdriver.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import cc_parts as cc  # noqa: E402

from yam_arena.task_file import TABLE_TOP  # noqa: E402

cc.install()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

MM = 1e-3
BOX_ORIGIN = (0.26, 0.0025, TOP)  # the box's footprint centre on the table (clamp at y -0.014, terminals 0.058)
CLAMP_ORIGIN = (BOX_ORIGIN[0], BOX_ORIGIN[1] + cc.SADDLE_Y, TOP)  # the clamp frame: the saddle's centre, table height
BAR_POS = (CLAMP_ORIGIN[0], CLAMP_ORIGIN[1], TOP + cc.BAR_Z0 + 0.1 * MM)  # resting on the cable
CABLE_ROOT = (BOX_ORIGIN[0], BOX_ORIGIN[1] + cc.TERMINAL_Y, TOP + cc.CABLE_Z)  # the cores end at the terminal block
CABLE_YAW = math.radians(-90.0)  # from the terminals toward the entry (-y)
HOLDER_ORIGIN = (0.17, 0.19, TOP + cc.HOLDER_SIZE[2] + 0.1 * MM)  # its top face's centre, left of the box
DRIVER_ORIGIN = (HOLDER_ORIGIN[0], HOLDER_ORIGIN[1], HOLDER_ORIGIN[2] - cc.SHAFT_TOP + 0.3 * MM)


def _headings():
    """(yaw, pitch) per link (cable frame: + yaw turns toward the box's far side, + pitch down): the cores and the
    first sheath link straight through the clamp and the entry, two links dropping to the table, then the rest
    lying on it, curving toward the arms."""
    # the connected cores bow up a little (slack: pushed in, the bow grows; laid straight and taut, a pushed cable
    # couldn't move at all, bench run 1)
    h = [(0.0, -35.0), (0.0, -15.0), (0.0, 15.0), (0.0, 35.0), (0.0, 0.0)][: cc.CORE_LINKS]  # (the last flat)
    # the sheath level through the clamp and the entry and two links beyond (pushed in, the cable's drop toward the
    # table stays clear of the entry; dropping right at the wall, its bend caught on the entry's edge, bench run 2)
    h += [(0.0, 0.0)] * 5 + [(0.0, 20.0), (0.0, 20.0)]
    rest = cc.SHEATH_LINKS - 7
    h += [(-55.0 * (i + 1) / rest, 0.0) for i in range(rest)]
    return h


CABLE_HEADINGS = _headings()
cc.CABLE_HEADINGS[:] = CABLE_HEADINGS

OBJECTS = [
    cc.part("box", "box", BOX_ORIGIN),
    cc.part("clamp", "clamp", CLAMP_ORIGIN),
    cc.part("bar", "bar", BAR_POS),
    cc.part("cable", "cable", CABLE_ROOT, yaw=CABLE_YAW),
    cc.part("holder", "holder", HOLDER_ORIGIN),
    cc.part("driver", "driver", DRIVER_ORIGIN),
]

# -- scoring ---------------------------------------------------------------------------------------------------------
CREDIT = {"sheath": 0.25, "holds": 0.5, "uneven": 0.75, "done": 1.0}
SHEATH_GOAL_Y = cc.BAR_SIZE[1] / 2 + cc.SHEATH_PAST  # clamp frame y: the bar's edge on the terminals' side + 5 mm


def sheath_past(objects) -> bool:
    """The sheath's cut end at least 5 mm past the bar (toward the terminals); measured before a failed pull test."""
    c, k = objects["cable"], objects["clamp"]
    y = max(c.sheath_y, k.pull_sheath_y) if k.pull == "failed" else c.sheath_y
    return bool(y >= SHEATH_GOAL_Y)


def holds(objects) -> bool:
    """The latest pull test (30 N on the cable outside the box, hands off): the cable moved less than 1 mm at the
    clamp."""
    return objects["clamp"].pull == "passed"


def both_snug(objects) -> bool:
    k = objects["clamp"]
    return all(c is not None and cc.SNUG_TURNS[0] <= p < cc.SNUG_TURNS[1]
               for c, p in zip(k.contact_turns, k.past_contact, strict=True))


def parallel(objects) -> bool:
    """The bar parallel: its tilt under 2 degrees, measured as the two screw heads' height difference over their
    22 mm spacing (the bar's ends sit at the heads): under 0.77 mm, i.e. the screws less than 1.1 turns apart."""
    return bool(objects["bar"].tilt_deg < cc.TILT_MAX_DEG)


def in_turns(objects) -> bool:
    """Tightened in turns: the two screws' turns past their heads' first contact differ by less than a quarter turn
    (read on the marks). One screw tightened all the way before the other leaves them further apart."""
    k = objects["clamp"]
    if any(c is None for c in k.contact_turns):
        return False
    return bool(abs(k.past_contact[0] - k.past_contact[1]) < cc.IN_TURNS_MAX)


def score(objects) -> float:
    """0 if the cores were ever clamped (the bar pressing while the sheath's end wasn't past it) or the cable crushed
    (a screw ever turned 2 turns past its head's first contact), or the sheath not 5 mm past the bar; 0.25 sheath
    past; 0.5 also holds the pull test; 0.75 also both screws snug, but not tightened in turns (their turns past
    first contact differ by a quarter turn or more) or the bar tilted 2 degrees or more; 1.0 both screws snug,
    tightened in turns and the bar parallel."""
    k = objects["clamp"]
    if k.crushed or k.cores_clamped or not sheath_past(objects):
        return 0.0
    if not holds(objects):
        return CREDIT["sheath"]
    if not both_snug(objects):
        return CREDIT["holds"]
    if not (in_turns(objects) and parallel(objects)):
        return CREDIT["uneven"]
    return CREDIT["done"]


def success(objects) -> bool:
    """Full score (the pull test needs both hands off) and the cable, the screws and the bar still."""
    return (score(objects) >= 1.0 - 1e-9 and objects["cable"].resting() and objects["clamp"].resting()
            and objects["bar"].resting())


def distance(objects) -> float:
    """Metres the sheath's end still has to go (a progress measure, not the score)."""
    return max(0.0, SHEATH_GOAL_Y - objects["cable"].sheath_y)
