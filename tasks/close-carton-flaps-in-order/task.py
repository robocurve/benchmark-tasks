"""On the station table, centred between the arms: a filled corrugated shipping carton, fixed against the packing
bench's back stop, with its four top flaps standing open (leaning slightly in). Nothing else is on the table."""

# Closing a regular slotted carton's top before taping is a core, many-times-an-hour operation for hand packers and
# packagers and for stockers and order fillers (about 2.9 million US workers, BLS OEWS May 2023). The two short
# (minor) flaps must go down first and the two long (major) flaps over them; a major closed first blocks the minor
# under it, so the carton cannot be closed without reopening it.
#
# Physical difficulty: each flap is a hinged panel that must be pushed through a 85-degree arc with closed fingers,
# both arms working on opposite flaps at once, in the order the flaps overlap.
#
# World frame: x forward (away from the arms), y left, z up, metres. The arm bases stand at x 0, y +-0.305.

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from labkit import Joint, part  # noqa: E402
from taskbase import Trial, arc, fake_state, joint_point  # noqa: E402

from yam_arena.constants import FINGER_CLOSED  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = "Close the carton's top: the two short flaps first, then the two long flaps over them."

# -- the scene -------------------------------------------------------------------------------------------------------

KRAFT = (0.72, 0.55, 0.36)
KRAFT_DARK = (0.62, 0.46, 0.29)
W, L, H = 0.13, 0.22, 0.06  # inside-ish: 13 cm forward x 22 cm across x 6 cm tall
T = 0.004  # wall and flap thickness (corrugated board)
MINOR = W / 2  # flap depth: every flap is half the carton's width (regular slotted carton)
MAJOR = W / 2
MARGIN = 0.015  # minor flaps are 3 cm narrower than the carton: they clear the major flaps' hinges
OPEN = 1.48  # flaps start 85 degrees open, leaning slightly in
CARTON_POS = (0.25, 0.0, TOP)
HINGE = dict(friction=0.02, damping=0.01, mass=0.02)

FLAPS = {
    # name: (axis, anchor, centre at zero, size, lower, upper, initial)
    "minor_left": ("X", (0, L / 2, H), (0, L / 2 - MINOR / 2, H + T / 2), (W - 2 * MARGIN, MINOR, T), -1.65, 0.0, -OPEN),
    "minor_right": ("X", (0, -L / 2, H), (0, -L / 2 + MINOR / 2, H + T / 2), (W - 2 * MARGIN, MINOR, T), 0.0, 1.65, OPEN),
    "major_near": ("Y", (-W / 2, 0, H + T), (-W / 2 + MAJOR / 2, 0, H + 1.5 * T), (MAJOR, L, T), -1.65, 0.0, -OPEN),
    "major_far": ("Y", (W / 2, 0, H + T), (W / 2 - MAJOR / 2, 0, H + 1.5 * T), (MAJOR, L, T), 0.0, 1.65, OPEN),
}

JOINTS = [Joint(n, kind="hinge", axis=ax, anchor=a, lower=lo, upper=hi, initial=init, target=init,
                boxes=[(c, s, KRAFT_DARK if n.startswith("minor") else KRAFT)], **HINGE)
          for n, (ax, a, c, s, lo, hi, init) in FLAPS.items()]

CARTON = part("carton", pos=CARTON_POS, fixed=True, self_collide=True, joints=JOINTS, boxes=[
    ((0, 0, T / 2), (W, L, T), KRAFT),
    ((-W / 2 + T / 2, 0, H / 2), (T, L, H), KRAFT),
    ((W / 2 - T / 2, 0, H / 2), (T, L, H), KRAFT),
    ((0, L / 2 - T / 2, H / 2), (W - 2 * T, T, H), KRAFT),
    ((0, -L / 2 + T / 2, H / 2), (W - 2 * T, T, H), KRAFT),
])
OBJECTS = [CARTON]

# -- scoring ---------------------------------------------------------------------------------------------------------
#
# A flap is closed when its hinge is within 7 degrees of flat. Score at the end of the trial: 0.15 per closed flap
# (0 to 0.6). 1.0 when all four flaps are closed (which the board only allows with the short flaps under the long
# ones), the carton is still (flaps moving under 0.03 rad over the last 0.3 s) and no gripper tool point is within
# 4 cm of a flap's centre; held for 1 s. Closed less than 1 s before the run limit scores 0.6.

CLOSED = math.radians(7)


def flap_centre(state, name):
    ax, a, c, s, lo, hi, init = FLAPS[name]
    return joint_point(state.pos, a, ax, state.joints.get(name, 0.0), c)


def evaluate(objects, trial):
    c = objects["carton"]
    closed = [n for n in FLAPS if abs(c.joints.get(n, 9.0)) < CLOSED]
    trial.info["closed"] = len(closed)
    done = (len(closed) == 4 and trial.still("carton")
            and trial.clear_of([flap_centre(c, n) for n in FLAPS], 0.04))
    return 0.15 * len(closed), done


TRIAL = Trial(evaluate, trace_env="CARTON_TRACE")
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"


def success(objects) -> bool:
    return TRIAL.success(objects)


def score(objects) -> float:
    return TRIAL.score(objects)


def distance(objects) -> float:
    """Summed hinge angle (rad) still to close."""
    return float(sum(abs(objects["carton"].joints.get(n, OPEN)) for n in FLAPS))


# -- the demonstration -----------------------------------------------------------------------------------------------
#
# Scripted, open loop, both arms with closed fingers. Each arm first pushes the short flap on its side down through its
# arc (tool point 1 cm behind a point 4.5 cm out from the hinge; the finger pads sit about 1.15 cm from the tool point,
# so they press slightly into the face) and on 14 degrees past flat, so the flap is driven hard onto the carton wall, lifts clear, then pushes the long flap on
# its side (left arm the near one, right arm the far one) down over the short flaps, and lifts clear.

STANDOFF, LIFT, N = 0.010, 0.0, 12
OVER = 0.25  # drive the arc 14 degrees past flat: the wall top stops the flap, the arm keeps pressing


def _push(name, along, arm_y, dur=2.4):
    ax, a, c, s, lo, hi, init = FLAPS[name]
    if ax == "X":
        sign = -1 if a[1] > 0 else 1
        p0 = (arm_y, a[1] + sign * along, a[2] + T)
    else:
        sign = 1 if a[0] < 0 else -1
        p0 = (a[0] + sign * along, arm_y, a[2] + T)
    pts = arc(CARTON_POS, a, ax, p0, init, -OVER * (1 if init > 0 else -1), N, standoff=STANDOFF, lift=LIFT)
    first = pts[0]
    back = first - (pts[1] - pts[0]) / (np.linalg.norm(pts[1] - pts[0]) + 1e-9) * 0.03
    wps = [Waypoint(1.3, tuple(back + np.array((0, 0, 0.06))), FINGER_CLOSED, f"above {name}"),
           Waypoint(1.0, tuple(back), FINGER_CLOSED, f"behind {name}")]
    wps += [Waypoint(dur / N, tuple(p), FINGER_CLOSED, f"push {name}") for p in pts]
    end = pts[-1]
    wps += [Waypoint(0.6, tuple(end - np.array((0, 0, 0.005))), FINGER_CLOSED, f"press {name} flat"),
            Waypoint(0.4, tuple(end - np.array((0, 0, 0.005))), FINGER_CLOSED, f"hold {name} flat"),
            Waypoint(0.8, tuple(end + np.array((0, 0, 0.10))), FINGER_CLOSED, "lift clear")]
    return wps


def demo():
    left = _push("minor_left", 0.045, 0.0) + _push("major_near", 0.045, 0.075)
    right = _push("minor_right", 0.045, 0.0) + _push("major_far", 0.045, -0.075)
    rest_l, rest_r = (0.12, 0.25, TOP + 0.16), (0.12, -0.25, TOP + 0.16)
    left += [Waypoint(1.2, rest_l, FINGER_CLOSED, "clear"), Waypoint(2.5, rest_l, FINGER_CLOSED, "wait")]
    right += [Waypoint(1.2, rest_r, FINGER_CLOSED, "clear"), Waypoint(2.5, rest_r, FINGER_CLOSED, "wait")]
    return {"left": left, "right": right}


# -- offline cases (check_task.py) ----------------------------------------------------------------------------------

def CASES():
    def st(**j):
        joints = {n: FLAPS[n][6] for n in FLAPS}
        joints.update(j)
        return {"carton": fake_state(CARTON_POS, (W / 2, L / 2, H / 2), joints)}
    shut = {n: 0.0 for n in FLAPS}
    far = lambda: [np.array((0.1, 0.3, 1.0)), np.array((0.1, -0.3, 1.0))]
    on_flap = lambda: [np.array(flap_centre(st(**shut)["carton"], "major_near")) + np.array((0, 0, 0.01)),
                       np.array((0.1, -0.3, 1.0))]
    return [
        ("all flaps open", [st()] * 30, far, 0.0, False),
        ("both short flaps closed", [st(minor_left=0.0, minor_right=-0.0)] * 30, far, 0.3, False),
        ("all four closed, held 1 s", [st(**shut)] * 30, far, 1.0, True),
        ("all four closed 0.75 s before the limit", [st(**shut)] * 15, far, 0.6, True),
        ("all four closed but a gripper still on a flap", [st(**shut)] * 30, on_flap, 0.6, False),
        ("a flap 10 degrees open", [st(**{**shut, "major_far": 0.17})] * 30, far, 0.45, False),
        ("flaps still moving", [st(**{n: 0.05 * ((k % 2) * 2 - 1) for n in FLAPS}) for k in range(30)], far, 0.6, False),
    ]
