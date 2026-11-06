"""On the station table, a three-compartment warewashing sink runs across the front of both arms: three open steel
basins in a row (no water), from left to right the wash basin, the rinse basin and the sanitize basin, each with
walls 5 cm tall. To the left of the wash basin is the soiled drainboard with one small steel pan on it; to the right
of the sanitize basin is the clean side: an air-dry rack (three steel bars on a drip tray). Nearer the arms on the
right lies a white towel on the counter. Warewashing rule: the pan goes through the basins in order, wash, then
rinse, then sanitize; it stays in the sanitize basin for its contact time (at least 3 s here); then it goes onto the
air-dry rack to drain and dry. A sanitized pan is never towel-dried or set on the towel. The order and the contact
time are checked from the pan's path during the trial.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/three-compartment-sink/three_compartment_sink.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math

import numpy as np

from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]
HZ = 20.0  # control rate the trial runs at (-E control_hz=20)

# Frame: x forward from the arm bases, y left, z up, metres. Left arm base (0, +0.305, 0.75), right (0, -0.305, 0.75).
PAN = (0.080, 0.050, 0.025)  # small steel pan: 8 cm along x, 5 cm across, 2.5 cm deep (a solid box here)
PAN_KG = 0.120
SINK_X = 0.300
BASIN_Y = {"wash": 0.250, "rinse": 0.125, "sanitize": 0.000}
BASIN_INNER = (0.140, 0.105)
BASIN_WALL = (0.006, 0.050)  # thickness, height
IN_BASIN_Z = 0.015  # the pan is "in" a basin when its bottom is less than 15 mm above the basin floor
DWELL_S = 3.0  # sanitizer contact time asked for
DWELL_STEPS = round(DWELL_S * HZ)

BOARD = (0.140, 0.100, 0.004)  # soiled drainboard
BOARD_XY = (SINK_X, 0.400)
BOARD_TOP = TOP + BOARD[2]
PAN_START = BOARD_XY

RACK_XY = (SINK_X, -0.145)
DRIP = (0.120, 0.100, 0.004)
BAR = (0.008, 0.090, 0.025)
BAR_DX = (-0.030, 0.000, 0.030)
BAR_TOP = TOP + DRIP[2] + BAR[2]
TOWEL = (0.120, 0.100, 0.004)
TOWEL_XY = (0.170, -0.150)
TOWEL_TOP = TOP + TOWEL[2]

STEEL = (0.72, 0.73, 0.75)
DARK = (0.30, 0.31, 0.33)
PAN_C = (0.82, 0.83, 0.85)
WHITE = (0.95, 0.95, 0.93)


def _basin(name, cy):
    cx = SINK_X
    ix, iy = BASIN_INNER
    t, h = BASIN_WALL
    z = TOP + h / 2
    return [
        Box(f"{name}_basin_wall_left", pos=(cx, cy + iy / 2 + t / 2, z), size=(ix + 2 * t, t, h), color=STEEL),
        Box(f"{name}_basin_wall_right", pos=(cx, cy - iy / 2 - t / 2, z), size=(ix + 2 * t, t, h), color=STEEL),
        Box(f"{name}_basin_wall_front", pos=(cx - ix / 2 - t / 2, cy, z), size=(t, iy, h), color=STEEL),
        Box(f"{name}_basin_wall_back", pos=(cx + ix / 2 + t / 2, cy, z), size=(t, iy, h), color=STEEL),
    ]


PAN_OBJ = Box("pan", pos=(*PAN_START, BOARD_TOP + PAN[2] / 2), size=PAN, mass=PAN_KG, color=PAN_C)
FIXTURES = [
    *(w for name, y in BASIN_Y.items() for w in _basin(name, y)),
    Box("soiled_drainboard", pos=(*BOARD_XY, TOP + BOARD[2] / 2), size=BOARD, color=DARK),
    Box("drip_tray", pos=(*RACK_XY, TOP + DRIP[2] / 2), size=DRIP, color=DARK),
    *(Box(f"rack_bar_{k + 1}", pos=(RACK_XY[0] + dx, RACK_XY[1], TOP + DRIP[2] + BAR[2] / 2), size=BAR, color=STEEL)
      for k, dx in enumerate(BAR_DX)),
    Box("towel", pos=(*TOWEL_XY, TOP + TOWEL[2] / 2), size=TOWEL, color=WHITE),
]
OBJECTS = [PAN_OBJ, *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"


def _tilt(quat_xyzw) -> float:
    x, y, _, _ = (float(v) for v in quat_xyzw)
    return math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))))


def basin_of(st):
    """Which basin the pan is down in (centre inside its inner walls, bottom within 15 mm of the floor), or None."""
    for name, cy in BASIN_Y.items():
        if (abs(float(st.pos[0]) - SINK_X) < BASIN_INNER[0] / 2 and abs(float(st.pos[1]) - cy) < BASIN_INNER[1] / 2
                and float(st.low[2]) < TOP + IN_BASIN_Z):
            return name
    return None


def on_rack(st) -> bool:
    """At rest on the air-dry rack: centre over the bars, bottom on the bar tops, level."""
    return bool(
        abs(float(st.pos[0]) - RACK_XY[0]) < 0.030
        and abs(float(st.pos[1]) - RACK_XY[1]) < BAR[1] / 2
        and BAR_TOP - 0.003 < float(st.low[2]) < BAR_TOP + 0.006
        and _tilt(st.quat_xyzw) < 5.0
        and st.resting()
    )


def on_towel(st) -> bool:
    return bool(
        abs(float(st.pos[0]) - TOWEL_XY[0]) < TOWEL[0] / 2 + PAN[0] / 2
        and abs(float(st.pos[1]) - TOWEL_XY[1]) < TOWEL[1] / 2 + PAN[1] / 2
        and float(st.low[2]) < TOWEL_TOP + 0.010
    )


# The trial's history, built step by step from the pan's path. stage: 0 nothing yet, 1 washed, 2 rinsed after washing,
# 3 sanitized (3 s in the sanitize basin after rinsing). Going back into the wash or rinse basin drops the stage back to
# 1 or 2. dwell: consecutive steps in the sanitize basin. towel: the pan has been on the towel. score() and success()
# are both called on every control step with the same states dict, so a step is counted once. The history resets
# whenever the pan is back at its start (each new trial).
_HISTORY = {"stage": 0, "dwell": 0, "towel": False, "last": None}


def _at_start(objects) -> bool:
    st = objects["pan"]
    return bool(math.hypot(float(st.pos[0]) - PAN_START[0], float(st.pos[1]) - PAN_START[1]) < 0.005
                and float(st.low[2]) < BOARD_TOP + 0.003)


def _update(objects) -> dict:
    h = _HISTORY
    if objects is h["last"]:
        return h
    h["last"] = objects
    if _at_start(objects):
        h.update(stage=0, dwell=0, towel=False)
        return h
    st = objects["pan"]
    where = basin_of(st)
    if where == "wash":
        h["stage"] = 1
    elif where == "rinse" and h["stage"] >= 1:
        h["stage"] = 2
    h["dwell"] = h["dwell"] + 1 if where == "sanitize" else 0
    if where == "sanitize" and h["stage"] == 2 and h["dwell"] >= DWELL_STEPS:
        h["stage"] = 3
    if on_towel(st):
        h["towel"] = True
    return h


def score(objects) -> float:
    """0.2 once the pan has been down in the wash basin, 0.2 more once it has then been in the rinse basin, 0.2 more
    once it has then stayed 3 s in the sanitize basin, and the last 0.4 for the sanitized pan at rest on the air-dry
    rack. If the pan has ever been on the towel, the rack credit is lost."""
    h = _update(objects)
    s = 0.2 * min(h["stage"], 3)
    if h["stage"] == 3 and not h["towel"] and on_rack(objects["pan"]):
        s += 0.4
    return float(s)


def success(objects) -> bool:
    """Washed, rinsed, sanitized for 3 s in that order, and resting on the air-dry rack, never on the towel; holds 1 s."""
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    """Metres to go: the pan's centre to the air-dry rack (in the plane)."""
    st = objects["pan"]
    return float(math.hypot(float(st.pos[0]) - RACK_XY[0], float(st.pos[1]) - RACK_XY[1]))


# -- the scripted demo ------------------------------------------------------------------------------------------

TRAVEL = TOP + 0.110  # pan bottom 7 cm above the table: clear of the 5 cm basin walls
SQUEEZE = 0.008
TIP = 0.013
GRIP_DEPTH = 0.013  # the pad tips 13 mm below the pan's top
OPEN = -(PAN[1] + 0.020) / 2
GRIP = -(PAN[1] - SQUEEZE) / 2
X_TRIM = 0.004  # a carried 120 g pan lands a few mm short in x


def _tcp_z(bottom):
    return bottom + PAN[2] - GRIP_DEPTH + TIP


def _pick(xy, bottom, label):
    x, y = xy
    gz = _tcp_z(bottom)
    return [
        Waypoint(1.5, (x, y, TRAVEL), OPEN, f"{label}: over"),
        Waypoint(1.5, (x, y, gz + 0.015), OPEN, f"{label}: descend"),
        Waypoint(0.8, (x, y, gz), OPEN, f"{label}: down slowly"),
        Waypoint(0.5, (x, y, gz), OPEN, f"{label}: settle"),
        Waypoint(1.2, (x, y, gz), GRIP, f"{label}: close"),
        Waypoint(1.5, (x, y, TRAVEL), GRIP, f"{label}: lift"),
    ]


def _dip(y, label):
    """Carry the pan over a basin, lower it to 4 mm above the floor, hold it there 1 s, lift it out."""
    x = SINK_X + X_TRIM
    dz = _tcp_z(TOP + 0.004)
    return [
        Waypoint(1.5, (x, y, TRAVEL), GRIP, f"{label}: over the basin"),
        Waypoint(0.4, (x, y, TRAVEL), GRIP, f"{label}: settle"),
        Waypoint(1.8, (x, y, dz), GRIP, f"{label}: lower in"),
        Waypoint(1.0, (x, y, dz), GRIP, f"{label}: hold in the basin"),
        Waypoint(1.8, (x, y, TRAVEL), GRIP, f"{label}: lift out"),
    ]


def _set_down(xy, bottom, label):
    x, y = xy
    qz = _tcp_z(bottom + 0.002)
    return [
        Waypoint(1.5, (x, y, TRAVEL), GRIP, f"{label}: over"),
        Waypoint(0.5, (x, y, TRAVEL), GRIP, f"{label}: settle"),
        Waypoint(2.0, (x, y, qz), GRIP, f"{label}: set down"),
        Waypoint(0.6, (x, y, qz), GRIP, f"{label}: settle"),
        Waypoint(1.2, (x, y, qz), OPEN, f"{label}: release"),
        Waypoint(1.0, (x, y, TRAVEL), OPEN, f"{label}: retreat"),
    ]


def demo():
    """Left arm: pan off the soiled drainboard, dipped in the wash and rinse basins, set down in the sanitize basin and
    let go; it backs away. Right arm: waits, then (after more than 3 s of contact) lifts the pan out of the sanitizer
    and sets it on the air-dry rack. 44.1 s."""
    left = (_pick(PAN_START, BOARD_TOP, "pan")
            + _dip(BASIN_Y["wash"], "wash")
            + _dip(BASIN_Y["rinse"], "rinse")
            + _set_down((SINK_X + X_TRIM, BASIN_Y["sanitize"]), TOP, "sanitize"))
    left += [Waypoint(1.5, (0.220, 0.300, TOP + 0.200), OPEN, "clear of the sink")]
    t_free = sum(w.duration for w in left)
    park = (0.220, -0.300, TOP + 0.200)
    right = [Waypoint(3.0, park, OPEN, "to the waiting point"),
             Waypoint(t_free - 3.0 + 0.5, park, OPEN, "wait for the left arm")]
    right += _pick((SINK_X + 0.002, BASIN_Y["sanitize"]), TOP, "out of the sanitizer")
    right += _set_down((RACK_XY[0] + X_TRIM, RACK_XY[1]), BAR_TOP, "air-dry rack")
    right += [Waypoint(1.5, park, OPEN, "clear")]
    return {"left": left, "right": right}


def self_test(ObjectState=None):
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def pan(x, y, bottom, vel=(0.0, 0.0, 0.0), tilt_deg=0.0):
        t = math.radians(tilt_deg)
        q = np.array([math.sin(t / 2), 0.0, 0.0, math.cos(t / 2)])
        half = np.array(PAN) / 2
        pos = np.array([x, y, bottom + half[2]])
        return {"pan": ObjectState(pos, q, np.asarray(vel, float), np.zeros(3), pos - half, pos + half)}

    start = pan(*PAN_START, BOARD_TOP)
    up = pan(SINK_X, 0.2, TOP + 0.08)
    wash = pan(SINK_X, BASIN_Y["wash"], TOP + 0.004)
    rinse = pan(SINK_X, BASIN_Y["rinse"], TOP + 0.004)
    san = pan(SINK_X, BASIN_Y["sanitize"], TOP)
    rack = pan(*RACK_XY, BAR_TOP)
    towel = pan(*TOWEL_XY, TOWEL_TOP)

    def run(*seq):
        """Feed a path of (state, steps) to score() as a trial would, a fresh dict per step; return the last score."""
        s = None
        for st, n in ((start, 1), *seq):
            for _ in range(n):
                step = dict(st)
                s = (score(step), success(step))
        return s

    full = [(wash, 10), (up, 5), (rinse, 10), (up, 5), (san, DWELL_STEPS + 5), (up, 5)]
    cases = {
        "start": ([], 0.0, False),
        "washed": ([(wash, 5), (up, 5)], 0.2, False),
        "rinse only": ([(rinse, 5), (up, 5)], 0.0, False),
        "washed and rinsed": ([(wash, 5), (up, 3), (rinse, 5), (up, 3)], 0.4, False),
        "done": (full + [(rack, 30)], 1.0, True),
        "sanitizer too short (2 s)": ([(wash, 10), (up, 5), (rinse, 10), (up, 5), (san, DWELL_STEPS - 20), (up, 5),
                                       (rack, 30)], 0.4, False),
        "two short dips do not add up": ([(wash, 10), (up, 5), (rinse, 10), (up, 5), (san, 35), (up, 2), (san, 35),
                                          (up, 5), (rack, 30)], 0.4, False),
        "skipped the rinse": ([(wash, 10), (up, 5), (san, DWELL_STEPS + 5), (up, 5), (rack, 30)], 0.2, False),
        "rinse before wash": ([(rinse, 10), (up, 5), (wash, 10), (up, 5), (san, DWELL_STEPS + 5), (up, 5), (rack, 30)],
                              0.2, False),
        "sanitized, then set on the towel, then the rack": (full + [(towel, 10), (up, 5), (rack, 30)], 0.6, False),
        "sanitized and left on the towel": (full + [(towel, 30)], 0.6, False),
        "back into the wash after sanitizing": (full + [(wash, 10), (up, 5), (rack, 30)], 0.2, False),
        "straight from the drainboard to the rack": ([(up, 5), (rack, 30)], 0.0, False),
        "sanitized, still in the sanitizer": (full[:-1], 0.6, False),
        "on the rack but tilted off a bar": (full + [(pan(*RACK_XY, BAR_TOP, tilt_deg=12.0), 30)], 0.6, False),
    }
    for name, (seq, want, want_success) in cases.items():
        got, got_success = run(*seq)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
    return len(cases)


if __name__ == "__main__":
    # Offline checks, no GPU: the scoring rules on hand-built paths, then each arm's plan replayed through the
    # kinematics at 20 Hz, every joint inside the URDF limits with an IK error under 0.5 mm.
    from yam_arena import ik, station
    from yam_arena.demo import ArmPlan
    from yam_arena.task_file import load

    print(f"scoring: {self_test()} cases pass")
    task = load(__file__)
    for arm, waypoints in task.demo().items():
        plan = ArmPlan(list(waypoints), station.BASES[arm])
        q = np.array([plan.action7((k + 1) / HZ)[0] for k in range(round(plan.duration * HZ) + 5)])
        assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (arm, plan.worst_ik_mm, plan.worst_ik_deg)
        assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), arm
        print(f"{arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / {plan.worst_ik_deg:.3f} deg")
