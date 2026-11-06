"""On the station table, in front of the left arm, a machining fixture: a fixed steel fixture plate carrying a 3-2-1
locating set. Two short hardened locating pins stand along the plate's far side (the secondary datum, 60 mm apart)
and one pin stands on its left side (the tertiary datum); the plate's top is the primary datum. A loose aluminium
gearbox-housing casting, 11 cm deep and 13 cm wide, too wide for the 94 mm gripper in either direction, lies on the
plate a few centimetres short of the pins and turned 3 degrees. It must be slid, never lifted, until it bears on all
three pins and lies flat on the plate. Scored from the casting's pose at the end.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/push-casting-to-datums/push_casting_to_datums.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math

import numpy as np

from yam_arena.constants import FINGER_CLOSED
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. The left arm's base is at (0, +0.305, 0.75).
CAST = (0.110, 0.130, 0.040)  # depth (x), width (y), height: wider than the 94 mm gripper opening both ways
CAST_KG = 0.250  # a thin-walled die-cast housing
PLATE = (0.230, 0.260, 0.010)  # the fixture plate, the primary datum
PLATE_C = (0.310, 0.200)
PT = TOP + PLATE[2]

HOME = (0.330, 0.220)  # the casting's centre when it bears on all three pins
PIN_R = 0.006
PIN_H = 0.012
PINS = {  # pin centres: two on the far (secondary) datum line, one on the left (tertiary) datum line
    "pin_1": (HOME[0] + CAST[0] / 2 + PIN_R, HOME[1] - 0.030),
    "pin_2": (HOME[0] + CAST[0] / 2 + PIN_R, HOME[1] + 0.030),
    "pin_3": (HOME[0] + 0.025, HOME[1] + CAST[1] / 2 + PIN_R),
}
START = (0.295, 0.195)  # loose on the plate, 3.5 cm short of the far pins, 2.5 cm short of the side pin
START_YAW_DEG = 3.0

TOUCH = 0.0015  # a pin counts as bearing when the casting's face is within 1.5 mm of it
FLAT_Z = 0.003  # flat: bottom within 3 mm of the plate top
FLAT_DEG = 3.0

ALU = (0.80, 0.80, 0.78)
STEEL = (0.42, 0.44, 0.47)
PIN_COLOR = (0.12, 0.12, 0.13)

CASTING = Box("casting", pos=(*START, PT + CAST[2] / 2), size=CAST, mass=CAST_KG, color=ALU,
              yaw=math.radians(START_YAW_DEG))
FIXTURES = [
    Box("fixture_plate", pos=(*PLATE_C, TOP + PLATE[2] / 2), size=PLATE, color=STEEL),
    *(Cylinder(name, pos=(*xy, PT + PIN_H / 2), radius=PIN_R, height=PIN_H, color=PIN_COLOR) for name, xy in PINS.items()),
]
OBJECTS = [CASTING, *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"


def _yaw(quat_xyzw) -> float:
    x, y, z, w = (float(v) for v in quat_xyzw)
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def _tilt(quat_xyzw) -> float:
    """Tilt of the body's up axis from the vertical, degrees."""
    x, y, _, _ = (float(v) for v in quat_xyzw)
    return math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))))


def _gap(st, pin) -> float:
    """Air between a pin's surface and the casting's outline in plan view (negative: overlapping), metres."""
    a = _yaw(st.quat_xyzw)
    dx, dy = pin[0] - float(st.pos[0]), pin[1] - float(st.pos[1])
    u = math.cos(a) * dx + math.sin(a) * dy  # the pin centre in the casting's own frame
    v = -math.sin(a) * dx + math.cos(a) * dy
    ox, oy = abs(u) - CAST[0] / 2, abs(v) - CAST[1] / 2
    outside = math.hypot(max(ox, 0.0), max(oy, 0.0))
    return (outside if outside > 0 else max(ox, oy)) - PIN_R


def _bearing(st):
    """Which pins the casting bears on: its face within 1.5 mm of the pin and not more than 3 mm into it."""
    return {name: -0.003 <= _gap(st, xy) <= TOUCH for name, xy in PINS.items()}


def _flat(st) -> bool:
    return bool(abs(float(st.low[2]) - PT) <= FLAT_Z and _tilt(st.quat_xyzw) <= FLAT_DEG)


def _located(st) -> bool:
    return all(_bearing(st).values()) and _flat(st) and st.resting()


def success(objects) -> bool:
    """The casting bears on all three pins, flat on the plate and still; holds for 1 s."""
    return _located(objects["casting"])


def score(objects) -> float:
    """0.25 for each locating pin the casting bears on (face within 1.5 mm of the pin); 1.0 once it bears on all
    three, flat on the plate (bottom within 3 mm of its top, tilted under 3 degrees) and still."""
    st = objects["casting"]
    if _located(st):
        return 1.0
    return 0.25 * sum(_bearing(st).values())


def distance(objects) -> float:
    """Metres of air left between the casting and the three pins."""
    st = objects["casting"]
    return float(sum(max(0.0, _gap(st, xy)) for xy in PINS.values()))


# -- the scripted demo ------------------------------------------------------------------------------------------

TIP = 0.013
PAD = 0.010  # a closed pad's outer face stands 10 mm from the TCP along the closing axis
TRAVEL = TOP + 0.110
PUSH_Z = PT + 0.004 + TIP  # pad tips 4 mm above the plate: the pads bear on the casting's lower half
CLEAR = 0.015  # the pads come down this far behind the face they will push
# A pushing arm trails its command by about 8 mm per newton pushing forward (x) and 4 mm per newton pushing sideways
# (y) (measured in sim with a 400 g casting: 32 mm and 15 mm short). Each push is commanded this far past contact, so
# the pins, not the arm's lag, decide where the casting stops; the arm then backs off past the lag before lifting.
# Pressing much harder than this sideways starts to ride the casting up the side pin (0.4 degrees with 30 mm).
OVERPUSH = {"x": 0.040, "y": 0.020}
PUSH_V = 0.013  # m/s average along each push; the smoothed peak is 1.5 times this, under 2 cm/s


def _push(start_xy, end_xy, yaw, label, overpush):
    """Closed fingers down behind a face, a slow push along one axis, back up."""
    (sx, sy), (ex, ey) = start_xy, end_xy
    n = math.hypot(ex - sx, ey - sy)
    back = overpush + 0.012  # clear of the face before going up
    bx, by = ex - back * (ex - sx) / n, ey - back * (ey - sy) / n
    push_s = round(n / PUSH_V, 1)
    c = FINGER_CLOSED
    return [
        Waypoint(1.8, (sx, sy, TRAVEL), c, f"{label}: over the start", yaw),
        Waypoint(1.5, (sx, sy, PUSH_Z + 0.020), c, f"{label}: descend behind the casting", yaw),
        Waypoint(1.0, (sx, sy, PUSH_Z), c, f"{label}: down slowly", yaw),
        Waypoint(0.5, (sx, sy, PUSH_Z), c, f"{label}: settle", yaw),
        Waypoint(push_s, (ex, ey, PUSH_Z), c, f"{label}: push slowly", yaw),
        Waypoint(1.0, (ex, ey, PUSH_Z), c, f"{label}: hold against the pins", yaw),
        Waypoint(1.5, (bx, by, PUSH_Z), c, f"{label}: back off", yaw),
        Waypoint(1.2, (bx, by, TRAVEL), c, f"{label}: up", yaw),
    ]


def demo():
    """Left arm only, fingers closed, never lifting the casting. First the near face is pushed forward at its middle
    (pads across x) into both far pins, which squares it; then the right-hand face is pushed left at the casting's
    middle (pads across y) until it bears on the side pin. Each push runs at under 2 cm/s. 28.5 s."""
    near_face = START[0] - CAST[0] / 2
    x_start = (near_face - PAD - CLEAR, START[1])
    x_end = (HOME[0] - CAST[0] / 2 - PAD + OVERPUSH["x"], START[1])
    right_face = START[1] - CAST[1] / 2
    y_start = (HOME[0], right_face - PAD - CLEAR)
    y_end = (HOME[0], HOME[1] - CAST[1] / 2 - PAD + OVERPUSH["y"])
    return {"left": _push(x_start, x_end, math.pi / 2, "into the far pins", OVERPUSH["x"])
            + _push(y_start, y_end, 0.0, "into the side pin", OVERPUSH["y"])}


def self_test(ObjectState=None):
    """The scoring rules on hand-built states; runs with the real ObjectState or any stand-in with the same fields."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def cast(x, y, yaw_deg=0.0, bottom=PT, tilt_deg=0.0, vel=(0.0, 0.0, 0.0)):
        yaw, t = math.radians(yaw_deg), math.radians(tilt_deg)
        qz = np.array([0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)])
        qx = np.array([math.sin(t / 2), 0.0, 0.0, math.cos(t / 2)])
        x1, y1, z1, w1 = qz
        x2, y2, z2, w2 = qx
        q = np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                      w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])
        c, s = abs(math.cos(yaw)), abs(math.sin(yaw))
        half = np.array([c * CAST[0] / 2 + s * CAST[1] / 2, s * CAST[0] / 2 + c * CAST[1] / 2,
                         CAST[2] / 2 * math.cos(t) + CAST[1] / 2 * abs(math.sin(t))])
        pos = np.array([x, y, bottom + half[2]])
        return {"casting": ObjectState(pos, q, np.asarray(vel, float), np.zeros(3), pos - half, pos + half)}

    hx, hy = HOME
    cases = {
        "start": (cast(*START, START_YAW_DEG), 0.0, False),
        "located": (cast(hx - 0.0005, hy - 0.0005), 1.0, True),
        "located, 1 mm off every pin": (cast(hx - 0.001, hy - 0.001), 1.0, True),
        "on the far pins only": (cast(hx, hy - 0.010), 0.5, False),
        "on the side pin only": (cast(hx - 0.010, hy), 0.25, False),
        "3 mm short of the far pins": (cast(hx - 0.003, hy), 0.25, False),
        "turned 2 degrees: one far pin and the side pin": (cast(hx - 0.001, hy - 0.001, yaw_deg=2.0), 0.5, False),
        "on all three but still sliding": (cast(hx, hy, vel=(0.0, 0.02, 0.0)), 0.75, False),
        "on all three but tipped up 5 degrees": (cast(hx, hy, bottom=PT, tilt_deg=5.0), 0.75, False),
        "pushed past the plate onto the table": (cast(0.20, 0.10, bottom=TOP), 0.0, False),
    }
    for name, (st, want, want_success) in cases.items():
        got, got_success = score(st), success(st)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
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
