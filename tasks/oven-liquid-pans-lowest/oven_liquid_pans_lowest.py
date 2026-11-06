"""On the station table, in front of the left arm, an open-front oven cavity drawn without its top: two steel side
walls along x, a back wall, and on the side walls three pairs of runners (pan ledges) at three heights, all empty.
On a steel counter nearer the arms stand three gastronorm-style pans: a deep pan of stock (brown) and two shallow pans
of solid food (a roast and vegetables). Kitchen food-safety practice puts liquids on the lowest runner, so a spill or
boil-over cannot drip into food below and the hot liquid is never carried at face height. Each pan rests across a
pair of runners by its two ends, so it slides in from the front at its level, which is also why the oven is filled
from the bottom up: the gripper reaches in above the pan. The stock must also be kept level all the way (checked every
step). Scored from the pan poses.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/oven-liquid-pans-lowest/oven_liquid_pans_lowest.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math

import numpy as np

from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. The left arm's base is at (0, +0.305, 0.75).
PAN_XY = (0.070, 0.110)  # every pan: 70 mm deep (x, the slide-in direction), 110 mm wide (y), resting on its two ends
DEEP_H, SHALLOW_H = 0.040, 0.020
PANS = {  # name: (height, mass, colour, start y on the counter)
    "stock_pan": (DEEP_H, 0.250, (0.55, 0.33, 0.12), 0.265),
    "roast_pan": (SHALLOW_H, 0.150, (0.45, 0.16, 0.12), 0.390),
    "vegetable_pan": (SHALLOW_H, 0.150, (0.25, 0.55, 0.20), 0.140),
}
LIQUID = "stock_pan"
COUNTER_X = 0.170
COUNTER = (0.110, 0.400, 0.004)
COUNTER_TOP = TOP + COUNTER[2]

OVEN_Y = 0.265
OVEN_FRONT, OVEN_BACK = 0.300, 0.440  # runner front ends and the back wall's inner face
HOME_X = 0.360  # a pan pushed in to its working position: 4.5 cm clear of the back wall
LEVEL = {1: TOP + 0.025, 2: TOP + 0.090, 3: TOP + 0.140}  # runner tops; level 1 is the lowest
RUNNER_W, RUNNER_T = 0.012, 0.004
RUNNER_IN = PAN_XY[1] / 2 - 0.008  # the runners' inner edges: a pan rests 8 mm on each
WALL_T = 0.006
WALL_IN = RUNNER_IN + RUNNER_W  # side walls' inner faces, 4 mm beyond a pan's ends
WALL_H = 0.170

STEEL = (0.72, 0.73, 0.75)
DARK_STEEL = (0.40, 0.41, 0.43)


def _oven():
    depth = OVEN_BACK - OVEN_FRONT
    cx = (OVEN_FRONT + OVEN_BACK) / 2
    parts = [Box("oven_floor", pos=(cx, OVEN_Y, TOP + 0.002), size=(depth, 2 * WALL_IN, 0.004), color=DARK_STEEL),
             Box("oven_back", pos=(OVEN_BACK + WALL_T / 2, OVEN_Y, TOP + WALL_H / 2),
                 size=(WALL_T, 2 * (WALL_IN + WALL_T), WALL_H), color=STEEL)]
    for s, sgn in (("ypos", 1), ("yneg", -1)):
        parts.append(Box(f"oven_wall_{s}", pos=(cx, OVEN_Y + sgn * (WALL_IN + WALL_T / 2), TOP + WALL_H / 2),
                         size=(depth, WALL_T, WALL_H), color=STEEL))
        for k, z in LEVEL.items():
            parts.append(Box(f"runner_{k}_{s}", pos=(cx, OVEN_Y + sgn * (RUNNER_IN + RUNNER_W / 2), z - RUNNER_T / 2),
                             size=(depth, RUNNER_W, RUNNER_T), color=DARK_STEEL))
    return parts


PAN_BOXES = [Box(n, pos=(COUNTER_X, y, COUNTER_TOP + h / 2), size=(*PAN_XY, h), mass=m, color=c)
             for n, (h, m, c, y) in PANS.items()]
FIXTURES = [Box("counter", pos=(COUNTER_X, 0.265, TOP + COUNTER[2] / 2), size=COUNTER, color=STEEL), *_oven()]
OBJECTS = [*PAN_BOXES, *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

ON_X = 0.015  # on a level: centre within 1.5 cm of the working position along the runners
ON_Y = 0.005  # within 5 mm of the oven's centreline (the walls allow 4)
ON_Z = 0.003  # bottom within 3 mm of the runner tops
LEVEL_DEG = 3.0
SPILL_DEG = 10.0  # the stock pan tipped more than 10 degrees at any step has spilled


def _tilt(quat_xyzw) -> float:
    x, y, z, w = (float(v) for v in quat_xyzw)
    return math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))))


def _yaw(quat_xyzw) -> float:
    x, y, z, w = (float(v) for v in quat_xyzw)
    return math.degrees(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def _level_of(st):
    """The runner level a pan rests on, or None."""
    if not (abs(float(st.pos[0]) - HOME_X) <= ON_X and abs(float(st.pos[1]) - OVEN_Y) <= ON_Y
            and _tilt(st.quat_xyzw) <= LEVEL_DEG and abs((_yaw(st.quat_xyzw) + 90.0) % 180.0 - 90.0) <= 5.0
            and st.resting()):
        return None
    for k, z in LEVEL.items():
        if abs(float(st.low[2]) - z) <= ON_Z:
            return k
    return None


# The trial's history: whether the stock was ever tipped. It resets when the scene is back at its start.
_HISTORY = {"spilled": False}


def _at_start(objects) -> bool:
    return all(math.dist([float(v) for v in objects[p.name].pos], p.pos) < 0.003 for p in PAN_BOXES)


def _update(objects) -> bool:
    if _at_start(objects):
        _HISTORY["spilled"] = False
    elif _tilt(objects[LIQUID].quat_xyzw) > SPILL_DEG:
        _HISTORY["spilled"] = True
    return _HISTORY["spilled"]


def score(objects) -> float:
    """0.5 for the stock pan resting level on the lowest runners, 0.25 for each solid-food pan resting level on
    runner level 2 or 3. The stock on an upper level (it could spill into the food below), or ever tipped more than
    10 degrees, caps the score at 0.25."""
    spilled = _update(objects)
    lv = {n: _level_of(objects[n]) for n in PANS}
    s = 0.5 * (lv[LIQUID] == 1) + 0.25 * sum(lv[n] in (2, 3) for n in PANS if n != LIQUID)
    if spilled or lv[LIQUID] in (2, 3):
        s = min(s, 0.25)
    return float(s)


def success(objects) -> bool:
    """Stock on the lowest runners, the two solid pans on the levels above, never spilled; holds for 1 s."""
    return score(objects) >= 1.0 - 1e-9


def spilled() -> bool:
    """Whether this trial has tipped the stock (for the log)."""
    return _HISTORY["spilled"]


def distance(objects) -> float:
    """Metres to go: each pan's bottom centre to its level in the demo's assignment (stock 1, roast 2, vegetables 3)."""
    goal = {"stock_pan": 1, "roast_pan": 2, "vegetable_pan": 3}
    return float(sum(math.dist((float(objects[n].pos[0]), float(objects[n].pos[1]), float(objects[n].low[2])),
                               (HOME_X, OVEN_Y, LEVEL[k])) for n, k in goal.items()))


# -- the scripted demo ------------------------------------------------------------------------------------------

YAW = -math.pi / 2  # the fingers close along x onto the pan's front and back faces; the pads stay between the runners
TIP = 0.013
TRAVEL = TOP + 0.180
SQUEEZE = 0.008
WIDE = -(PAN_XY[0] + 0.016) / 2  # 86 mm: the far pad stays 2.7 cm clear of the back wall
TIP_ABOVE = 0.006  # pad tips 6 mm above the pan's bottom
SLIDE_CLEAR = 0.010  # slide in with the pan's bottom 10 mm above the runner tops
DROOP = 0.000  # a held pan hangs about 3 mm below the commanded height (measured): inside the 10 mm slide clearance
OUTSIDE_X = OVEN_FRONT - PAN_XY[0] / 2 - 0.015  # the pan wholly in front of the runners before it slides in
X_TRIM = 0.0  # the pan slides on the runners, so its depth only needs to be within 1.5 cm


def _grip_z(bottom):
    return bottom + TIP_ABOVE + TIP


def _load(name, level):
    """Pick the pan off the counter, carry it high to the front of the oven, lower it in front of its level, slide it
    in level over the runners, lower it onto them, let go, rise clear of the pan and back out of the oven. 22.8 s."""
    h, py = PANS[name][0], PANS[name][3]
    gz = _grip_z(COUNTER_TOP)
    slide = _grip_z(LEVEL[level] + SLIDE_CLEAR) + DROOP
    seat = _grip_z(LEVEL[level] + 0.0015) + DROOP
    grip = -(PAN_XY[0] - SQUEEZE) / 2
    x, y = HOME_X + X_TRIM, OVEN_Y
    return [
        Waypoint(1.8, (COUNTER_X, py, TRAVEL), WIDE, f"{name}: over", YAW),
        Waypoint(2.0, (COUNTER_X, py, gz + 0.020), WIDE, f"{name}: descend", YAW),
        Waypoint(1.0, (COUNTER_X, py, gz), WIDE, f"{name}: down slowly", YAW),
        Waypoint(0.5, (COUNTER_X, py, gz), WIDE, f"{name}: settle", YAW),
        Waypoint(1.5, (COUNTER_X, py, gz), grip, f"{name}: close", YAW),
        Waypoint(1.5, (COUNTER_X, py, gz + 0.030), grip, f"{name}: lift gently", YAW),
        Waypoint(1.0, (COUNTER_X, py, max(slide, gz + 0.050)), grip, f"{name}: lift", YAW),
        Waypoint(2.2, (OUTSIDE_X, y, max(slide, gz + 0.050)), grip, f"{name}: carry to the oven front", YAW),
        Waypoint(1.2, (OUTSIDE_X, y, slide), grip, f"{name}: to level {level}", YAW),
        Waypoint(0.6, (OUTSIDE_X, y, slide), grip, f"{name}: settle", YAW),
        Waypoint(2.5, (x, y, slide), grip, f"{name}: slide in over the runners", YAW),
        Waypoint(0.6, (x, y, slide), grip, f"{name}: settle in", YAW),
        Waypoint(1.2, (x, y, seat), grip, f"{name}: lower onto the runners", YAW),
        Waypoint(0.5, (x, y, seat), grip, f"{name}: settle", YAW),
        Waypoint(1.5, (x, y, seat), WIDE, f"{name}: release", YAW),
        Waypoint(1.2, (x, y, seat + h + 0.015), WIDE, f"{name}: rise clear of the pan", YAW),
        Waypoint(2.0, (OUTSIDE_X, y, seat + h + 0.015), WIDE, f"{name}: out of the oven", YAW),
    ]


def demo():
    """Left arm only, bottom up: the stock onto level 1, the roast onto level 2, the vegetables onto level 3."""
    return {"left": _load("stock_pan", 1) + _load("roast_pan", 2) + _load("vegetable_pan", 3)}


def self_test(ObjectState=None):
    """The scoring rules on hand-built sequences of states, so the spill history is exercised too."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def pan(name, x, y, bottom, tilt_deg=0.0, yaw_deg=0.0, vel=(0.0, 0.0, 0.0)):
        yaw, tilt = math.radians(yaw_deg), math.radians(tilt_deg)
        x1, y1, z1, w1 = 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)
        x2, y2, z2, w2 = math.sin(tilt / 2), 0.0, 0.0, math.cos(tilt / 2)
        q = np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                      w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])
        half = np.array((*PAN_XY, PANS[name][0])) / 2
        pos = np.array([x, y, bottom + half[2]])
        return ObjectState(pos, q, np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    def on(name, k, dx=0.0, dy=0.0, dz=0.0, **kw):
        return {name: pan(name, HOME_X + dx, OVEN_Y + dy, LEVEL[k] + dz, **kw)}

    start = {n: pan(n, COUNTER_X, PANS[n][3], COUNTER_TOP) for n in PANS}
    s1 = start | on("stock_pan", 1, dx=-0.006)
    s2 = s1 | on("roast_pan", 2)
    done = s2 | on("vegetable_pan", 3, dx=0.010, dy=0.003)
    carried = start | {"stock_pan": pan("stock_pan", 0.25, OVEN_Y, TOP + 0.05, tilt_deg=4.0, vel=(0.05, 0, 0))}
    tipped = start | {"stock_pan": pan("stock_pan", 0.25, OVEN_Y, TOP + 0.05, tilt_deg=14.0, vel=(0.05, 0, 0))}

    def run(*frames):
        for f in frames[:-1]:
            score(f)
        return score(frames[-1]), success(frames[-1])

    cases = {
        "start": ((start,), 0.0, False),
        "stock on the lowest runners": ((start, carried, s1), 0.5, False),
        "stock and roast in": ((start, s1, s2), 0.75, False),
        "done bottom up": ((start, carried, s1, s2, done), 1.0, True),
        "done, solids swapped between levels 2 and 3": ((start, s1 | on("roast_pan", 3) | on("vegetable_pan", 2)), 1.0, True),
        "stock on the top level, solids below": ((start, start | on("stock_pan", 3) | on("roast_pan", 1) | on("vegetable_pan", 2)), 0.25, False),
        "stock on level 2, solids on 1 and 3": ((start, start | on("stock_pan", 2) | on("roast_pan", 1) | on("vegetable_pan", 3)), 0.25, False),
        "stock tipped 14 degrees on the way, then seated": ((start, tipped, done), 0.25, False),
        "stock left on the counter, solids in": ((start, start | on("roast_pan", 2) | on("vegetable_pan", 3)), 0.5, False),
        "stock only half in (3 cm out)": ((start, start | on("stock_pan", 1, dx=-0.030)), 0.0, False),
        "stock resting on one runner, tilted": ((start, start | on("stock_pan", 1, dz=0.004, tilt_deg=6.0)), 0.0, False),
        "stock still sliding": ((start, start | on("stock_pan", 1, vel=(0.03, 0, 0))), 0.0, False),
        "a solid pan on the oven floor": ((start, s1 | {"roast_pan": pan("roast_pan", HOME_X, OVEN_Y, TOP + 0.004)}), 0.5, False),
        "a new trial after a spill starts clean": ((start, tipped, start, s1, s2, done), 1.0, True),
    }
    for name, (frames, want, want_success) in cases.items():
        got, got_success = run(*frames)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
    score(start)
    assert not spilled()
    assert distance(done) < 0.02 and distance(start) > 0.5
    return len(cases)


if __name__ == "__main__":
    # Offline checks, no GPU: the scoring rules on hand-built states, then the left arm's plan replayed through the
    # kinematics at 20 Hz, every joint inside the URDF limits with an IK error under 0.5 mm.
    from yam_arena import ik, station
    from yam_arena.demo import ArmPlan
    from yam_arena.task_file import load

    print(f"scoring: {self_test()} cases pass")
    task = load(__file__)
    hz = 20.0
    for arm, waypoints in task.demo().items():
        plan = ArmPlan(list(waypoints), station.BASES[arm])
        q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
        assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (arm, plan.worst_ik_mm, plan.worst_ik_deg)
        assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), arm
        print(f"{arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / {plan.worst_ik_deg:.3f} deg")
