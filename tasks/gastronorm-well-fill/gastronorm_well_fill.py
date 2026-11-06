"""On the station table, in front of the left arm, one well of a hot-food steam table (scaled to about 0.3 of a real
full-size gastronorm well): a low stainless frame round a 11 x 16.8 cm opening, its long side along y. On the
counter nearer the arm stand four empty stainless gastronorm pans of the standard fractions, each scaled the same
way: a deep 1/2 pan, a 1/3 pan and two shallow 1/4 pans. Steam-table practice: every well is filled to its full size
with pans, adaptor bars or a lid so no opening is left; an open gap vents steam and the food in the other pans drops
below safe holding temperature. Of these pans only 1/2 + 1/4 + 1/4 fills the well exactly: the 1/2 pan across one
half of the well, the two 1/4 pans side by side across the other half (1/2 + 1/3 leaves a 1/6 opening). Scored from
the pans' poses.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/gastronorm-well-fill/gastronorm_well_fill.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. The left arm's base is at (0, +0.305, 0.75).
# Pans: (x, y, z) sizes. A real full-size well is 530 x 325 mm; the 1/2 pan is 325 x 265, the 1/3 325 x 176 and the
# 1/4 265 x 162 (here 0.3 of that in plan, with the 1/4 pans' long side along y). Depths are not to scale.
PANS = {
    "half_pan": ((0.100, 0.080, 0.055), 0.150),
    "third_pan": ((0.100, 0.053, 0.040), 0.090),
    "quarter_pan_1": ((0.0495, 0.080, 0.025), 0.060),
    "quarter_pan_2": ((0.0495, 0.080, 0.025), 0.060),
}
KIND = {"half_pan": "half", "third_pan": "third", "quarter_pan_1": "quarter", "quarter_pan_2": "quarter"}
START = {"half_pan": (0.200, 0.420), "third_pan": (0.200, 0.080), "quarter_pan_1": (0.160, 0.180),
         "quarter_pan_2": (0.160, 0.300)}

WELL_XY = (0.360, 0.260)
WELL_INNER = (0.110, 0.168)  # x, y
WALL_T, WALL_H = 0.006, 0.008
GAP_Y = (WELL_INNER[1] - 2 * 0.080) / 3  # equal air at both ends and between the halves
GAP_X = (WELL_INNER[0] - 2 * 0.0495) / 3  # likewise between the two 1/4 pans
_hy = 0.040 + GAP_Y / 2  # each half's centre is this far from the well's centre along y
_qx = 0.0495 / 2 + GAP_X / 2
# The two ways to fill the well exactly: the 1/2 pan in one half, the two 1/4 pans side by side in the other.
TILINGS = []
for _s in (+1, -1):
    _half = (WELL_XY[0], WELL_XY[1] + _s * _hy)
    _quarters = [(WELL_XY[0] + _qx, WELL_XY[1] - _s * _hy), (WELL_XY[0] - _qx, WELL_XY[1] - _s * _hy)]
    TILINGS.append({"half": _half, "quarters": _quarters})
SEAT_XY, SEAT_DEG = 0.006, 6.0

STEEL = (0.80, 0.81, 0.83)
PAN_COLOR = (0.68, 0.70, 0.72)
WELL_COLOR = (0.45, 0.46, 0.48)


def _frame(name, centre, inner, color):
    cx, cy = centre
    ix, iy = inner
    z = TOP + WALL_H / 2
    return [
        Box(f"{name}_wall_ypos", pos=(cx, cy + iy / 2 + WALL_T / 2, z), size=(ix + 2 * WALL_T, WALL_T, WALL_H), color=color),
        Box(f"{name}_wall_yneg", pos=(cx, cy - iy / 2 - WALL_T / 2, z), size=(ix + 2 * WALL_T, WALL_T, WALL_H), color=color),
        Box(f"{name}_wall_near", pos=(cx - ix / 2 - WALL_T / 2, cy, z), size=(WALL_T, iy, WALL_H), color=color),
        Box(f"{name}_wall_far", pos=(cx + ix / 2 + WALL_T / 2, cy, z), size=(WALL_T, iy, WALL_H), color=color),
    ]


PAN_OBJECTS = [Box(n, pos=(*START[n], TOP + size[2] / 2), size=size, mass=m, color=PAN_COLOR)
               for n, (size, m) in PANS.items()]
FIXTURES = [*_frame("well", WELL_XY, WELL_INNER, WELL_COLOR)]
OBJECTS = [*PAN_OBJECTS, *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"


def _yaw_tilt(quat_xyzw):
    x, y, z, w = (float(v) for v in quat_xyzw)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))))
    yaw = math.degrees(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))
    return yaw, tilt


def seated(st, slot) -> bool:
    """Down flat on the well floor, square to the well (a pan turned 180 degrees is square too), centre within 6 mm
    of the slot's centre, still."""
    yaw, tilt = _yaw_tilt(st.quat_xyzw)
    return bool(
        abs(float(st.pos[0]) - slot[0]) <= SEAT_XY
        and abs(float(st.pos[1]) - slot[1]) <= SEAT_XY
        and abs(float(st.low[2]) - TOP) <= 0.003
        and abs((yaw + 90.0) % 180.0 - 90.0) <= SEAT_DEG
        and tilt <= SEAT_DEG
        and st.resting()
    )


def _tiling_score(objects, tiling) -> float:
    s = 0.5 * seated(objects["half_pan"], tiling["half"])
    for slot in tiling["quarters"]:
        s += 0.25 * any(seated(objects[n], slot) for n in ("quarter_pan_1", "quarter_pan_2"))
    return s


def score(objects) -> float:
    """For whichever of the two exact fillings is further along: 0.5 for the 1/2 pan seated in its half of the well
    and 0.25 for each quarter of the other half with a 1/4 pan seated in it. The 1/3 pan fills no slot."""
    return float(max(_tiling_score(objects, t) for t in TILINGS))


def success(objects) -> bool:
    """The well filled with no opening: the 1/2 pan in one half, a 1/4 pan in each quarter of the other; holds 1 s."""
    return score(objects) >= 1.0 - 1e-9


def coverage(objects) -> float:
    """The fraction of the well's opening under a pan that is down in the well (for the log)."""
    x0, x1 = WELL_XY[0] - WELL_INNER[0] / 2, WELL_XY[0] + WELL_INNER[0] / 2
    y0, y1 = WELL_XY[1] - WELL_INNER[1] / 2, WELL_XY[1] + WELL_INNER[1] / 2
    gx, gy = np.meshgrid(np.linspace(x0, x1, 56), np.linspace(y0, y1, 85))
    hit = np.zeros(gx.shape, bool)
    for n in PANS:
        st = objects[n]
        if float(st.low[2]) < TOP + 0.010:
            hit |= (gx >= float(st.low[0])) & (gx <= float(st.high[0])) & (gy >= float(st.low[1])) & (gy <= float(st.high[1]))
    return float(hit.mean())


def distance(objects) -> float:
    """Metres from each pan of the first filling to its slot (the demo's filling)."""
    t = TILINGS[0]
    d = math.hypot(float(objects["half_pan"].pos[0]) - t["half"][0], float(objects["half_pan"].pos[1]) - t["half"][1])
    for n, slot in zip(("quarter_pan_1", "quarter_pan_2"), t["quarters"]):
        d += math.hypot(float(objects[n].pos[0]) - slot[0], float(objects[n].pos[1]) - slot[1])
    return float(d)


# -- the scripted demo ------------------------------------------------------------------------------------------

TIP = 0.013
TRAVEL = TOP + 0.150
SQUEEZE = 0.008
GRIP_TIPS = {"half": 0.029, "quarter": 0.012}  # pad tips above the pan's bottom: the 1/2 pan's pads clear the 1/4 pans
X_TRIM = 0.0015  # a carried 60-150 g pan lands about 1.5 mm short in x


def _move(name, place):
    """Top-down pick and place with the fingers across the pan's 80 mm width (along y), wide open, settling before
    every close and release and making the last 2 cm slowly. 16.6 s."""
    (px, py), (qx, qy) = START[name], place
    qx += X_TRIM
    tips = GRIP_TIPS[KIND[name]]
    gz = TOP + tips + TIP
    qz = gz + 0.0015
    grip = -(PANS[name][0][1] - SQUEEZE) / 2
    lab = name.replace("_", " ")
    return [
        Waypoint(1.8, (px, py, TRAVEL), FINGER_OPEN, f"{lab}: over"),
        Waypoint(1.5, (px, py, gz + 0.020), FINGER_OPEN, f"{lab}: descend"),
        Waypoint(1.0, (px, py, gz), FINGER_OPEN, f"{lab}: down slowly"),
        Waypoint(0.5, (px, py, gz), FINGER_OPEN, f"{lab}: settle"),
        Waypoint(1.2, (px, py, gz), grip, f"{lab}: close"),
        Waypoint(1.5, (px, py, TRAVEL), grip, f"{lab}: lift"),
        Waypoint(2.2, (qx, qy, TRAVEL), grip, f"{lab}: carry"),
        Waypoint(0.6, (qx, qy, TRAVEL), grip, f"{lab}: settle over the well"),
        Waypoint(2.0, (qx, qy, qz + 0.020), grip, f"{lab}: lower"),
        Waypoint(1.5, (qx, qy, qz), grip, f"{lab}: set in slowly"),
        Waypoint(0.6, (qx, qy, qz), grip, f"{lab}: settle"),
        Waypoint(1.2, (qx, qy, qz), FINGER_OPEN, f"{lab}: release"),
        Waypoint(1.0, (qx, qy, TRAVEL), FINGER_OPEN, f"{lab}: retreat"),
    ]


def demo():
    """Left arm only: the far 1/4 pan, then the near 1/4 pan into the right-hand half of the well (each has an empty
    slot on one side for its pad), then the deep 1/2 pan into the left half, its pads coming down above the shallow
    1/4 pans. The 1/3 pan stays on the counter. 49.8 s."""
    t = TILINGS[0]
    far, near = t["quarters"]
    return {"left": _move("quarter_pan_1", far) + _move("quarter_pan_2", near) + _move("half_pan", t["half"])}


def self_test(ObjectState=None):
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def pan(name, x, y, bottom=TOP, yaw_deg=0.0, tilt_deg=0.0, vel=(0.0, 0.0, 0.0)):
        size = PANS[name][0]
        a, t = math.radians(yaw_deg), math.radians(tilt_deg)
        q1 = np.array([0.0, 0.0, math.sin(a / 2), math.cos(a / 2)])
        x2 = math.sin(t / 2)
        w1, z1, w2 = q1[3], q1[2], math.cos(t / 2)
        q = np.array([w1 * x2, z1 * x2, z1 * w2, w1 * w2])
        hx = abs(math.cos(a)) * size[0] / 2 + abs(math.sin(a)) * size[1] / 2
        hy = abs(math.sin(a)) * size[0] / 2 + abs(math.cos(a)) * size[1] / 2
        half = np.array([hx, hy, size[2] / 2])
        pos = np.array([x, y, bottom + size[2] / 2])
        return ObjectState(pos, q, np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    start = {n: pan(n, *START[n]) for n in PANS}
    t0, t1 = TILINGS
    done = start | {"half_pan": pan("half_pan", *t0["half"]), "quarter_pan_1": pan("quarter_pan_1", *t0["quarters"][0]),
                    "quarter_pan_2": pan("quarter_pan_2", *t0["quarters"][1])}
    mirror = start | {"half_pan": pan("half_pan", *t1["half"]), "quarter_pan_1": pan("quarter_pan_1", *t1["quarters"][1]),
                      "quarter_pan_2": pan("quarter_pan_2", *t1["quarters"][0])}
    third_in = start | {"half_pan": pan("half_pan", *t0["half"]),
                        "third_pan": pan("third_pan", WELL_XY[0], WELL_XY[1] - 0.040 + 0.0265 - 0.04)}
    cases = {
        "start": (start, 0.0, False),
        "done": (done, 1.0, True),
        "done the mirror way (1/2 pan in the other half)": (mirror, 1.0, True),
        "done, a 1/4 pan turned 180 degrees": (done | {"quarter_pan_1": pan("quarter_pan_1", *t0["quarters"][0], yaw_deg=180)},
                                              1.0, True),
        "1/2 + 1/3, a 1/6 opening left": (third_in, 0.5, False),
        "the two 1/4 pans only": (start | {k: done[k] for k in ("quarter_pan_1", "quarter_pan_2")}, 0.5, False),
        "one 1/4 pan only": (start | {"quarter_pan_1": done["quarter_pan_1"]}, 0.25, False),
        "1/2 pan 8 mm off along y": (done | {"half_pan": pan("half_pan", t0["half"][0], t0["half"][1] + 0.008)}, 0.5, False),
        "a 1/4 pan propped on the wall": (done | {"quarter_pan_2": pan("quarter_pan_2", *t0["quarters"][1], bottom=TOP + 0.004,
                                                                       tilt_deg=9)}, 0.75, False),
        "a 1/4 pan turned 20 degrees": (done | {"quarter_pan_2": pan("quarter_pan_2", *t0["quarters"][1], yaw_deg=20)}, 0.75, False),
        "1/2 pan still settling": (done | {"half_pan": pan("half_pan", *t0["half"], vel=(0, 0, -0.03))}, 0.5, False),
        "1/4 pans stacked in one quarter": (start | {"half_pan": done["half_pan"], "quarter_pan_1": done["quarter_pan_1"],
                                                     "quarter_pan_2": pan("quarter_pan_2", *t0["quarters"][0], bottom=TOP + 0.025)},
                                            0.75, False),
        "mixed: 1/2 pan in one half, 1/4 pans in the same half's place": (
            start | {"half_pan": pan("half_pan", *t0["half"]), "quarter_pan_1": pan("quarter_pan_1", *t1["quarters"][0], bottom=TOP + 0.055)},
            0.5, False),
    }
    for name, (st, want, want_success) in cases.items():
        got, got_success = score(st), success(st)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
    assert coverage(done) > 0.8 > 0.75 > coverage(third_in), (coverage(done), coverage(third_in))
    return len(cases)


if __name__ == "__main__":
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
