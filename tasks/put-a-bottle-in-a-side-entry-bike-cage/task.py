"""Put a water bottle into the side-entry cage on a small mountain bike's down tube. Created 11_16_26 (America/New_York).

On the station: an extra-small Salsa Rangefinder hardtail, wheels off, held level in a bike workstand by its
seat post, its left (non-drive) side toward the robot and its front to the robot's left. The frame's mid-plane is
0.38 m in front of the arm bases; the bottom bracket's centre is 0.10 m above the table top at y = -0.30; the cranks
are level, the near one (with its flat pedal) pointing back. An Elite Custom Race X bottle cage is bolted to the upper two of the down tube's
three bottle bosses; it is empty. On this frame size the cage sits so close under the top tube that a bottle cannot be
lifted in or out along the cage (5 cm of room above the bottle's top edge against the 11 cm the cage's front band
needs): it has to go in from the side, past the cage's sprung front arm, which springs shut round it. An empty Elite
Fly 550 ml bottle (74 mm across, 184.5 mm tall, 51 g) stands upright on the table at (0.30, -0.12), cap up.

Run it:  inspect-robots run --embodiment yam_bike_cage -E task=task.py -E object_jitter=0 --policy yam_scripted \\
           --no-prompt --instruction "Put the bottle in the bike's bottle cage so it stays put." --scorer task_score

Scoring is in score(); every criterion is judged on the final state except the two stated as "at any time" or
"for the last 3 s" (the arm's opening and the hands being clear).
"""

import math
import os

import numpy as np

from bike_cage import geom as G
from bike_cage import grip as GR
from yam_arena import ik
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, READY
from yam_arena.demo import Waypoint
from yam_arena.station import BASES
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]
XF = 0.38  # the frame's mid-plane (world x)
BB_Y = -0.30  # the bottom bracket's centre (world y) ...
BB_H = 0.10  # ... and its height above the table top
BOTTLE_START = (0.30, -0.12)  # where the bottle stands (its base's middle)

OBJECTS = [
    Box("bike", pos=(XF, BB_Y, TOP + BB_H), size=(0.01, 0.01, 0.01)),  # built by bike_cage.mech
    Box("bottle", pos=(BOTTLE_START[0], BOTTLE_START[1], TOP + 0.0005), size=(0.01, 0.01, 0.01), mass=G.BOTTLE_MASS),
]

# -- frames ----------------------------------------------------------------------------------------------------------
ORIGIN = np.array((XF, BB_Y, TOP + BB_H))
RC = G.cage_rotation()  # cage axes (s, n, a) in the world (the bike's local axes are the world's)
CAGE_O_W = ORIGIN + G.cage_to_frame(0.0, 0.0, 0.0)


def cage_world(s, n, a):
    return ORIGIN + G.cage_to_frame(s, n, a)


def to_cage(p_world):
    """(s, n, a) of a world point in the cage frame (origin: the seated bottle's base middle)."""
    return RC.T @ (np.asarray(p_world, float) - CAGE_O_W)


def _rot(q):
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


# -- criteria --------------------------------------------------------------------------------------------------------
SEAT_TOL = 0.010  # base's middle within this of the cage floor's middle (the seated position) ...
TILT_TOL_DEG = 10.0  # ... and the bottle's axis within this of the cage's
BAND_A = sum(G.ARM_A) / 2  # the front band's mid-height up the cage
ENCLOSED_TOL = 0.015  # the bottle's axis at the band's height within this of the cage's axis ...
ARM_CLOSED_DEG = 5.0  # ... with the arm shut round it (opened less than this)
TOUCH = dict(s=(-0.06, 0.06), n=(-G.AXIS_OFF - 0.005, 0.07), a=(-0.015, 0.06))  # base's middle in this box: on or against the cage
CLEAR_R = G.BOTTLE_R + 0.060  # a hand is clear with its fingertip midpoint this far from the bottle's axis (60 mm from its side)
CLEAR_S = 3.0  # both hands clear for at least the last 3 s
STILL_V = 0.01  # m/s: the bottle at rest at the end
OVERBENT_CAP = 0.25


def _fresh():
    return {"clear": 0.0, "max_arm": 0.0}


_trial = _fresh()


def reset_latch():
    _trial.clear()
    _trial.update(_fresh())


def bottle_axis(objects):
    return _rot(objects["bottle"].quat_xyzw)[:, 2]


def tilt_deg(objects):
    ax = RC.T @ bottle_axis(objects)
    return math.degrees(math.acos(float(np.clip(ax[2], -1.0, 1.0))))


def base_cage(objects):
    return to_cage(objects["bottle"].pos)


def seated(objects):
    return float(np.linalg.norm(base_cage(objects))) <= SEAT_TOL and tilt_deg(objects) <= TILT_TOL_DEG


def axis_at_band(objects):
    """(s, n) offset of the bottle's axis from the cage's axis at the band's height (inf if the bottle lies flat to it)."""
    b = base_cage(objects)
    ax = RC.T @ bottle_axis(objects)
    if ax[2] < 0.2:
        return np.array((math.inf, math.inf))
    t = (BAND_A - b[2]) / ax[2]
    p = b + t * ax
    return p[:2]


def arm_deg(objects):
    return float(objects["cage_arm"].low[0])


def enclosed(objects):
    return float(np.linalg.norm(axis_at_band(objects))) <= ENCLOSED_TOL and arm_deg(objects) <= ARM_CLOSED_DEG


def touching(objects):
    b = base_cage(objects)
    return all(TOUCH[k][0] <= v <= TOUCH[k][1] for k, v in zip(("s", "n", "a"), b))


def hand_clear(objects, hand):
    p = np.asarray(objects[hand].pos, float)
    b = np.asarray(objects["bottle"].pos, float)
    ax = bottle_axis(objects)
    t = float(np.clip(np.dot(p - b, ax), 0.0, G.BOTTLE_H))
    return float(np.linalg.norm(p - (b + t * ax))) >= CLEAR_R


def note_step(objects, dt):
    """After every control step: how long both hands have been clear without a break, and the arm's widest opening."""
    if all(hand_clear(objects, h) for h in ("left_tcp", "right_tcp")):
        _trial["clear"] += dt
    else:
        _trial["clear"] = 0.0
    _trial["max_arm"] = max(_trial["max_arm"], arm_deg(objects))


def overbent():
    return _trial["max_arm"] > G.OVERBENT_DEG


def retained(objects):
    return _trial["clear"] >= CLEAR_S - 1e-9 and float(np.linalg.norm(objects["bottle"].lin_vel)) < STILL_V


def score(objects):
    """1.0: seated (base's middle within 10 mm of the cage floor's middle, axis within 10 degrees of the cage's), the
    front arm shut round it (axis within 15 mm of the cage's at the band, arm open under 5 degrees), both hands clear
    (60 mm) for the last 3 s and the bottle at rest. 0.5: enclosed and retained like that but not seated. 0.25: the
    bottle seated, enclosed or resting on or against the cage without earning more (still held, not at rest, or outside
    the shut arm). 0 otherwise
    (on the table, dropped, elsewhere). The arm opened past 30 degrees at any time caps the score at 0.25."""
    enc, seat = enclosed(objects), seated(objects)
    if enc and retained(objects):
        s = 1.0 if seat else 0.5
    elif enc or seat or touching(objects):
        s = 0.25
    else:
        s = 0.0
    return min(s, OVERBENT_CAP) if overbent() else s


def success(objects):
    return score(objects) >= 1.0


def debug_line(objects, t):
    r = lambda v: np.round(1000 * np.asarray(v, float), 1).tolist()  # noqa: E731
    return (f"BIKE t={t:6.2f} base(cage) {r(base_cage(objects))} tilt {tilt_deg(objects):5.1f} band {r(axis_at_band(objects))} "
            f"arm {arm_deg(objects):5.1f} max {_trial['max_arm']:5.1f} clear {_trial['clear']:.2f} score {score(objects):.2f} "
            f"R {r(objects['right_tcp'].pos)}")


# -- scripted demo --------------------------------------------------------------------------------------------------
# Open loop from the fixed start; it reads no simulator state. The right hand takes the standing bottle round its body
# from the right-rear (fingers closing diagonally across it, 140 mm up), lifts it, tilts it to the down tube's angle as
# it carries it to the bike's near side, holds it there square to the cage and 12 mm high, pushes it sideways into the
# cage (the bottle's side opens the sprung arm, which springs shut behind it), lowers it onto the cage floor, opens fully
# and draws back sideways out of the frame before moving away. BIKE_DEMO switches the shortcut variants: "nothing":
# no motion. "hold": as the demo, but the hand never lets go. "lean": the bottle is pushed in only part way, its side
# against the arm's tip, and let go there. "drop": the bottle is let go 12 mm short of the cage's near side.
VARIANT = os.environ.get("BIKE_DEMO", "")


def _env(name, default):
    return float(os.environ.get(name, default))


GRIP_A = _env("BIKE_GRIP_A", 0.135)  # TCP this far up the bottle's axis ...
GRIP_F = _env("BIKE_GRIP_F", 0.008)  # ... and this far back from it along the fingers
PRE_S = _env("BIKE_PRE_S", 0.110)  # the bottle's axis this far out on the near side before the push
LIFT_A = _env("BIKE_LIFT_A", 0.012)  # held this high above the cage floor while pushed in
PUSH_END_S = _env("BIKE_PUSH_END", -0.002)  # the push aims this far past the cage's axis
PRESS_A = _env("BIKE_PRESS_A", -0.003)  # lowered to this (just into the floor: a light press)
PUSH_S = _env("BIKE_PUSH_S", 4.0)
RELEASE_FINGER = -0.001 * _env("BIKE_RELEASE_GAP_MM", 80.0) / 2  # opened to this pad gap (mm) to let go: 3 mm each side of the bottle
WITHDRAW_DOWN = _env("BIKE_WITHDRAW_DOWN", 0.02)  # and this far down the bottle while drawing back
BACK_S = 0.13  # draw back this far toward the robot before moving away
GRIP_PSI = math.radians(_env("BIKE_GRIP_PSI", 60.0))  # the grip's diagonal across the bottle, from the frame's plane
C_SEAT = RC @ np.array((-math.cos(GRIP_PSI), math.sin(GRIP_PSI), 0.0))  # closing direction, seated: far-front pad to near-back pad
F_SEAT = RC @ np.array((-math.sin(GRIP_PSI), -math.cos(GRIP_PSI), 0.0))  # fingertips, seated: away from the robot, toward the tube
R_SEAT = GR.frame(F_SEAT, C_SEAT)
TILT = math.degrees(math.atan2(-RC[1, 2], RC[2, 2]))  # the cage axis's lean from vertical, about world x


def rot_x(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array(((1.0, 0.0, 0.0), (0.0, c, -s), (0.0, s, c)))


R_START = rot_x(-TILT) @ R_SEAT  # the same grip on the bottle standing upright
PARK = (0.20, -0.30, TOP + 0.20)
DOWN = GR.frame((0.0, 0.0, -1.0), (0.0, -1.0, 0.0))


def held_tcp(base_world, axis_world, rot):
    """TCP for the bottle's base at base_world, its axis along axis_world, held with TCP rotation rot."""
    return np.asarray(base_world, float) + GRIP_A * np.asarray(axis_world, float) - GRIP_F * np.asarray(rot, float)[:, 2]


def tcp_at_cage(s, n, a):
    """TCP for the bottle's base at cage point (s, n, a), the bottle square to the cage."""
    return held_tcp(cage_world(s, n, a), RC[:, 2], R_SEAT)


class _Plan:
    def __init__(self):
        self.wps = {"left": [], "right": []}
        self.t = {"left": 0.0, "right": 0.0}
        self.last = {}
        for a in self.wps:
            pos, rot = ik.tcp_pose(np.array(READY, float), np.array(BASES[a]))
            self.last[a] = (np.asarray(pos, float), FINGER_OPEN, np.asarray(rot, float))

    def go(self, arm, dur, pos=None, finger=None, rot=None, label=""):
        p0, f0, r0 = self.last[arm]
        p = p0 if pos is None else np.asarray(pos, float)
        f = f0 if finger is None else finger
        r = r0 if rot is None else np.asarray(rot, float)
        self.wps[arm].append(Waypoint(dur, tuple(float(v) for v in p), f, label, GR.turned(r)))
        self.last[arm] = (p, f, r)
        self.t[arm] += dur

    def until(self, arm, t):
        if t > self.t[arm] + 1e-9:
            self.go(arm, t - self.t[arm], label="wait")


def demo():
    global LAST_PLAN
    plan = _Plan()
    if VARIANT == "nothing":
        for a in plan.wps:
            plan.go(a, 60.0, label="wait")
        LAST_PLAN = plan
        return plan.wps
    up = np.array((0.0, 0.0, 1.0))
    base0 = np.array((BOTTLE_START[0], BOTTLE_START[1], TOP))
    g0 = held_tcp(base0, up, R_START)
    f0 = R_START[:, 2]
    plan.go("right", 2.5, g0 - 0.08 * f0 + (0.0, 0.0, 0.06), FINGER_OPEN, R_START, "behind the bottle")
    plan.go("right", 2.0, g0, label="round the bottle")
    plan.go("right", 1.0, finger=FINGER_CLOSED, label="grip")
    plan.go("right", 0.5, label="settle")
    plan.go("right", 1.2, g0 + (0.0, 0.0, 0.06), label="lift")
    pre = tcp_at_cage(PRE_S, 0.0, LIFT_A)
    mid = (g0 + (0.0, 0.0, 0.06) + pre) / 2 + (0.0, 0.0, 0.04)
    plan.go("right", 2.0, mid, rot=rot_x(-TILT / 2) @ R_SEAT, label="carry, tilting")
    plan.go("right", 2.0, pre, rot=R_SEAT, label="square to the cage, near side")
    plan.go("right", 0.5, label="steady")
    end_s = {"lean": 0.045, "drop": 0.051}.get(VARIANT, PUSH_END_S)
    if VARIANT != "drop":
        plan.go("right", PUSH_S * (PRE_S - end_s) / PRE_S, tcp_at_cage(end_s, 0.0, LIFT_A), label="push in from the side")
        plan.go("right", 0.5, label="steady")
    else:
        plan.go("right", PUSH_S * (PRE_S - end_s) / PRE_S, tcp_at_cage(end_s, 0.0, LIFT_A), label="short of the cage")
    if VARIANT in ("", "hold"):
        plan.go("right", 1.0, tcp_at_cage(end_s, 0.0, PRESS_A), label="down onto the floor")
        plan.go("right", 0.5, label="steady")
    if VARIANT != "hold":
        s_now = end_s
        a_now = PRESS_A if VARIANT == "" else LIFT_A
        plan.go("right", 1.0, finger=RELEASE_FINGER, label="open a little")
        plan.go("right", 2.0, tcp_at_cage(s_now, 0.0, a_now) - BACK_S * F_SEAT - WITHDRAW_DOWN * RC[:, 2], label="draw back along the fingers")
        plan.go("right", 2.0, PARK, rot=DOWN, label="away")
    end = max(plan.t.values()) + 5.0
    for a in plan.wps:
        plan.until(a, end)
    LAST_PLAN = plan
    return plan.wps


LAST_PLAN = None


def ik_errors(wps):
    from yam_arena.demo import ArmPlan

    bad, worst = [], (0.0, 0.0)
    for arm, ws in wps.items():
        ap = ArmPlan(list(ws), BASES[arm])
        t, labels = 0.0, {}
        while t <= ap.duration:
            before = (ap.worst_ik_mm, ap.worst_ik_deg)
            _, label = ap.action7(t)
            if (ap.worst_ik_mm, ap.worst_ik_deg) != before and (ap.worst_ik_mm > 1.0 or ap.worst_ik_deg > 1.0):
                labels[label] = (round(ap.worst_ik_mm, 1), round(ap.worst_ik_deg, 2))
            t += 0.05
        worst = (max(worst[0], ap.worst_ik_mm), max(worst[1], ap.worst_ik_deg))
        bad += [(arm, k, v) for k, v in labels.items()]
    return worst, bad


if __name__ == "__main__":
    print("down tube deg", round(G.DT_DEG, 2), "tilt deg", round(TILT, 2), "cage origin", np.round(CAGE_O_W, 3))
    print("headroom along axis mm (top edge, centre)", round(1000 * G.headroom_along_axis(G.BOTTLE_R), 1),
          round(1000 * G.headroom_along_axis(0.0), 1), "gap to top tube mm", round(1000 * G.gap_to_top_tube(), 1))
    print("tip force N closed/15/30 deg", [round(G.tip_force(a), 2) for a in (0, 15, 30)])
    print("R_SEAT y axis vs cage axis", np.round(R_SEAT[:, 1], 3), np.round(RC[:, 2], 3))
    wps = demo()
    t = max(sum(w.duration for w in ws) for ws in wps.values())
    print(f"demo {t:.1f} s ({t * 20:.0f} steps), IK worst mm/deg, bad:", ik_errors(wps))
