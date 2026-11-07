"""On the station table, at 1:20 scale: a small yellow crane body (fixed) stands on the midline in front of the arms with
a short black outrigger beam on each side. Each outrigger will come down on a dark square landing mark 130 mm to its
side: the left mark is on a patch of brown soft clay, the right one on a patch of grey concrete. Two orange outrigger
pads lie on the table nearer the arms: a large one (60 mm square, 1.2 m at scale) on the right and a small one (45 mm,
0.9 m) on the left, each 12 mm thick.

Goal: a pad centred on each landing mark, with each pad big enough for its ground. Each outrigger presses 120 kN; clay
takes 100 kPa (needs 1.2 m2, so only the large pad) and concrete 400 kPa (needs 0.3 m2, so either pad). Each pad
starts on the far side from where it has to go.

Run it:  inspect-robots run --embodiment yam_arena -E task=crane_outrigger_pads.py -E object_jitter=0 \\
           --instruction "Put one outrigger pad flat on each dark landing mark, centred on it, so that neither pad \\
           overloads its ground. Each outrigger presses 120 kN on its pad. The brown clay can take 100 kPa and the grey \\
           concrete 400 kPa. At full size the large pad is 1.2 m square and the small pad 0.9 m square." \\
           --scorer task_score ...
"""

import math

import numpy as np

from yam_arena.constants import FINGER_OPEN, PAD_TIP_PAST_TCP
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the crane, ground patches and landing marks (all fixed) -------------------------------------------------------
CRANE_X = 0.33
MARK_Y = 0.13
GROUND_T, MARK_T = 0.0010, 0.0012  # patch thickness; landing mark stands 0.2 mm proud of its patch
MARK_TOP = TOP + MARK_T
MARKS = {"clay": (CRANE_X, MARK_Y), "concrete": (CRANE_X, -MARK_Y)}
BEARING_KPA = {"clay": 100.0, "concrete": 400.0}
LOAD_KN = 120.0
SCALE = 20.0

OBJECTS = [
    Box("crane_body", pos=(CRANE_X, 0.0, TOP + 0.025), size=(0.070, 0.080, 0.050), color=(0.95, 0.75, 0.10)),
    *[Box(f"outrigger_{s}", pos=(CRANE_X, d * 0.0525, TOP + 0.030), size=(0.012, 0.025, 0.012), color=(0.08, 0.08, 0.08))
      for s, d in (("left", 1), ("right", -1))],
    Box("clay_patch", pos=(*MARKS["clay"], TOP + GROUND_T / 2), size=(0.120, 0.110, GROUND_T), color=(0.45, 0.30, 0.18)),
    Box("concrete_patch", pos=(*MARKS["concrete"], TOP + GROUND_T / 2), size=(0.120, 0.110, GROUND_T),
        color=(0.62, 0.63, 0.65)),
    *[Box(f"mark_{g}", pos=(x, y, TOP + MARK_T / 2), size=(0.020, 0.020, MARK_T), color=(0.10, 0.10, 0.12))
      for g, (x, y) in MARKS.items()],
]

# -- the pads -------------------------------------------------------------------------------------------------------
PAD_T = 0.012
PADS = {
    "large_pad": {"side": 0.060, "mass": 0.08, "start": (0.22, -0.16)},
    "small_pad": {"side": 0.045, "mass": 0.05, "start": (0.22, 0.16)},
}
ORANGE = (0.95, 0.50, 0.10)
OBJECTS += [Box(n, pos=(*p["start"], TOP + PAD_T / 2), size=(p["side"], p["side"], PAD_T), mass=p["mass"], color=ORANGE)
            for n, p in PADS.items()]


def _area_m2(name):
    return (PADS[name]["side"] * SCALE) ** 2


def adequate(name, ground):
    """The pad spreads the outrigger load below the ground's bearing capacity: 120 kN / area <= capacity."""
    return LOAD_KN / _area_m2(name) <= BEARING_KPA[ground]


# -- scoring --------------------------------------------------------------------------------------------------------
CENTRE_MM, FLAT_DZ, FLAT_TILT_DEG = 10.0, 0.004, 5.0


def _tilt_deg(q):
    x, y, _, _ = (float(v) for v in q)
    return math.degrees(math.acos(min(1.0, abs(1 - 2 * (x * x + y * y)))))


def pad_on(objects, ground):
    """Name of the pad lying flat on that landing mark (centre within 10 mm of the mark's centre, within 4 mm of its
    resting height, tilted under 5 degrees), or None."""
    mx, my = MARKS[ground]
    for n in PADS:
        st = objects[n]
        if (math.hypot(st.pos[0] - mx, st.pos[1] - my) * 1000 <= CENTRE_MM
                and abs(st.pos[2] - (MARK_TOP + PAD_T / 2)) < FLAT_DZ and _tilt_deg(st.quat_xyzw) < FLAT_TILT_DEG):
            return n
    return None


def score(objects):
    """0.25 per landing mark with a pad centred on it; +0.5 if both marks have pads and neither ground is overloaded."""
    on = {g: pad_on(objects, g) for g in MARKS}
    s = 0.25 * sum(p is not None for p in on.values())
    if all(on.values()) and all(adequate(p, g) for g, p in on.items()):
        s += 0.5
    return s


def success(objects):
    return score(objects) >= 0.999 and all(objects[n].resting() for n in PADS)


GOAL = {"large_pad": "clay", "small_pad": "concrete"}


def distance(objects):
    return float(sum(math.hypot(objects[n].pos[0] - MARKS[g][0], objects[n].pos[1] - MARKS[g][1])
                     for n, g in GOAL.items()))


# -- scripted demo --------------------------------------------------------------------------------------------------
TRAVEL_Z = TOP + 0.09
GRIP_ABOVE_BOTTOM = PAD_TIP_PAST_TCP + 0.006
RELEASE_GAP = 0.003
SETTLE_S = 0.8
HANDOFF = (0.22, 0.0)  # table spot in front of the crane, reachable by both arms
YAW = {"left": -math.pi / 2, "right": math.pi / 2}  # fingers close along world x
REST = {"left": (0.20, 0.25), "right": (0.20, -0.25)}


def _move(name, src, dst, src_z, dst_z, yaw, label):
    open_w = -(PADS[name]["side"] / 2 + 0.012)
    pick = src_z + GRIP_ABOVE_BOTTOM
    drop = dst_z + RELEASE_GAP + GRIP_ABOVE_BOTTOM
    return [
        Waypoint(1.5, (*src, TRAVEL_Z), open_w, label, yaw),
        Waypoint(1.2, (*src, pick), open_w, "descend", yaw),
        Waypoint(0.8, (*src, pick), 0.0, "grip", yaw),
        Waypoint(1.0, (*src, TRAVEL_Z), 0.0, "lift", yaw),
        Waypoint(1.6, (*dst, TRAVEL_Z), 0.0, "carry", yaw),
        Waypoint(0.9, (*dst, drop + 0.012), 0.0, "lower", yaw),
        Waypoint(SETTLE_S, (*dst, drop + 0.012), 0.0, "settle", yaw),
        Waypoint(0.6, (*dst, drop), 0.0, "down", yaw),
        Waypoint(SETTLE_S, (*dst, drop), 0.0, "settle", yaw),
        Waypoint(0.5, (*dst, drop), open_w, "let go", yaw),
        Waypoint(0.9, (*dst, TRAVEL_Z), open_w, "up", yaw),
    ]


def _plan():
    tab, mark = TOP, MARK_TOP
    return [
        ("right", "large_pad", PADS["large_pad"]["start"], HANDOFF, tab, tab, "large pad to the handoff spot"),
        ("left", "large_pad", HANDOFF, MARKS["clay"], tab, mark, "large pad onto the clay mark"),
        ("left", "small_pad", PADS["small_pad"]["start"], HANDOFF, tab, tab, "small pad to the handoff spot"),
        ("right", "small_pad", HANDOFF, MARKS["concrete"], tab, mark, "small pad onto the concrete mark"),
    ]


def demo():
    """Four moves, one arm at a time, the idle arm waiting clear: the large pad crosses from the right to the clay mark
    and the small pad from the left to the concrete mark, each through a handoff spot in front of the crane."""
    out = {"left": [], "right": []}
    t = {"left": 0.0, "right": 0.0}
    for arm, name, src, dst, sz, dz, label in _plan():
        start = max(t.values())
        if start > t[arm]:
            out[arm].append(Waypoint(start - t[arm], (*REST[arm], TRAVEL_Z), FINGER_OPEN, "wait", YAW[arm]))
        seg = _move(name, src, dst, sz, dz, YAW[arm], label) + [
            Waypoint(1.2, (*REST[arm], TRAVEL_Z), FINGER_OPEN, "clear", YAW[arm])]
        out[arm] += seg
        t[arm] = start + sum(w.duration for w in seg)
    return out
