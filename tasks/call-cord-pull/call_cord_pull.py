"""At the table's left side stands a grey bracket on a short pedestal: a pivot ridge on top carries a
long red pull rod balanced level across it, its grip end reaching out over the table. A real
call-for-aid unit hands its cord from a ceiling pivot near the bed or bath; a pull end that sits too
high off the floor is a life-threatening deficiency (NSPIRE D009: "pull cord is higher than 6 inches
off the floor"), because a fallen resident cannot reach it.

Calling for aid means pulling the cord down: draw the grip end down 15 cm — the rod swings about its
pivot until the grip end has dropped 15 cm. The rod must stay on its pivot: a knocked-off rod is not
a call, and the scorer rejects it.

Run it:  inspect-robots run --embodiment yam_arena -E task=call_cord_pull.py \
           --instruction "pull the call cord down 15 centimetres to call for aid" --scorer task_score ...
"""

import numpy as np

from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

MOUNT = Box("mount", pos=(0.38, 0.10, TOP + 0.0775), size=(0.06, 0.06, 0.15), color=(0.55, 0.57, 0.60))
RIDGE = Box("pivot", pos=(0.38, 0.10, TOP + 0.1575), size=(0.06, 0.012, 0.01), color=(0.45, 0.47, 0.50))
# The pull rod rests level on the ridge: 40 cm long (ends at y = -0.10 and +0.30), grip end out over
# the table. Rocking it about the ridge drops the grip end; 48.6 deg of swing = a 15 cm drop.
ROD = Box("cord", pos=(0.38, 0.10, TOP + 0.1685), size=(0.01, 0.40, 0.012), mass=0.08,
          color=(0.75, 0.10, 0.10))

OBJECTS = [MOUNT, RIDGE, ROD]

PIVOT_Z = TOP + 0.1625                       # the ridge top: the rod's rotation centre
END_ARM = 0.20                               # pivot-to-end distance along the rod (m)
PULL_DROP = 0.15                             # downward displacement of the grip end that counts (m)
HALF_DROP = 0.075
MAX_CENTER_SHIFT = 0.015                     # the rod must stay on its pivot (m, xy)
MIN_CENTER_Z = PIVOT_Z - 0.02                # ...and not have fallen off

SUCCESS_HOLD_S = 0.4                         # the plans end ~10 steps after the pull; keep margin


def _drop(objects):
    """How far the grip end (+y) has been pulled below its level height (m). The z component of the
    rod's local y axis is R[2][1] = 2(yz + wx); the grip end hangs at centre + END_ARM * R[2][1]."""
    r = objects["cord"]
    x, y, z, w = r.quat_xyzw
    end_drop = -END_ARM * 2.0 * (y * z + w * x)
    return max(0.0, end_drop)


def _on_pivot(objects):
    """True while the rod still rides its ridge (not knocked off and lying on the table)."""
    r = objects["cord"]
    return (abs(r.pos[0] - 0.38) < MAX_CENTER_SHIFT and abs(r.pos[1] - 0.10) < MAX_CENTER_SHIFT
            and r.pos[2] >= MIN_CENTER_Z)


def success(objects):
    """The grip end has been pulled down at least 15 cm while the rod stayed on its pivot."""
    return _drop(objects) >= PULL_DROP and _on_pivot(objects)


def distance(objects):
    """Metres of pull still missing before the call registers."""
    return max(0.0, PULL_DROP - _drop(objects))


def score(objects):
    """0.5 for half the pull; 1.0 when the grip end is down 15 cm or more, rod still on its pivot."""
    if not _on_pivot(objects):
        return 0.0
    d = _drop(objects)
    if d >= PULL_DROP:
        return 1.0
    return 0.5 if d >= HALF_DROP else 0.0


SCORE_OVER = "best"     # a pulled rod that swings back still earned its pull


def demo():
    """The left arm grips the rod's grip end and swings it down through the arc about the pivot; the
    last waypoint settles slowly while the call registers."""
    import math

    def arc(dur, tilt_deg):
        t = math.radians(tilt_deg)
        return Waypoint(dur, (0.38, 0.10 + END_ARM * math.cos(t) - 0.011,
                              PIVOT_Z + 0.006 - END_ARM * math.sin(t)),
                        -0.023, "swing the grip end down", yaw=1.5708)

    return {
        "left": [
            Waypoint(1.8, (0.38, 0.289, PIVOT_Z + 0.006), -0.023, "approach the rod's grip end", yaw=1.5708),
            Waypoint(0.6, (0.38, 0.289, PIVOT_Z + 0.006), -0.023, "close on the grip", yaw=1.5708),
            arc(1.0, 18),
            arc(1.0, 33),
            arc(1.0, 48.6),
            Waypoint(1.5, (0.38, 0.222, PIVOT_Z + 0.006 - 0.150), -0.023, "settle at full pull", yaw=1.5708),
        ],
        "right": [],
    }
