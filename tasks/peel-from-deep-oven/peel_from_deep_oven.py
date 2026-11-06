"""On the station table, in front of the left arm, a deep deck oven (drawn without its roof and door so the cameras see
in): a fixed raised deck between two side walls and a back wall, 30 cm deep, with two steel shelf runners along its
floor. A loaded baking pan sits on the runners at the very back, 20 cm in, past the arm's reach. On the table to the
right lies a wooden oven peel, a flat blade 21.5 cm long and 6 cm wide. A white pass pad waits on the table to the left. The kitchen
rule: nobody reaches to the back of a hot deck; the pan is drawn to the mouth with the peel, and only then lifted
out by hand. The peel's blade fits between the runners, under the pan. Scored from the pan's and the peel's poses at
the end.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/peel-from-deep-oven/peel_from_deep_oven.py \\
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
OY = 0.300  # the oven's centre line
MOUTH_X = 0.400  # the deck's front edge
DECK = (0.300, 0.180, 0.030)
DECK_TOP = TOP + DECK[2]
# Between the side walls: 15 mm of air either side of the pan. With 5 mm, the lifted pan (which shifts about 4.5 mm
# sideways as the peel rises) rubbed a wall, turned 9 degrees and jammed in sim.
INNER = 0.150
WALL_T = 0.010
WALL_H = 0.100
RUNNER = (0.280, 0.010, 0.036)
RUNNER_Y = 0.053  # either side of the centre line: 96 mm between their inner faces
RUNNER_TOP = DECK_TOP + RUNNER[2]

PAN = (0.070, 0.120, 0.030)  # depth (x), width (y), height: its sides rest 12 mm onto each runner
PAN_KG = 0.080
PAN_X0 = 0.600  # at the back of the deck, past reach

# The blade is 60 mm wide: on a 36 mm blade the lifted pan turned and slid sideways off it in sim.
PEEL = (0.215, 0.060, 0.016)  # one flat blade-and-handle, 60 mm wide, 16 mm thick
PEEL_KG = 0.090
PEEL_START = (0.250, 0.160)

PASS = (0.160, 0.160, 0.004)
PASS_XY = (0.220, 0.430)
PASS_TOP = TOP + PASS[2]

STEEL = (0.62, 0.63, 0.65)
DARK = (0.18, 0.18, 0.19)
WOOD = (0.74, 0.55, 0.32)
PAN_COLOR = (0.30, 0.31, 0.33)
WHITE = (0.93, 0.93, 0.93)

THE_PAN = Box("pan", pos=(PAN_X0, OY, RUNNER_TOP + PAN[2] / 2), size=PAN, mass=PAN_KG, color=PAN_COLOR)
THE_PEEL = Box("peel", pos=(*PEEL_START, TOP + PEEL[2] / 2), size=PEEL, mass=PEEL_KG, color=WOOD)
_DX = MOUTH_X + DECK[0] / 2
FIXTURES = [
    Box("oven_deck", pos=(_DX, OY, TOP + DECK[2] / 2), size=DECK, color=DARK),
    *(Box(f"oven_wall_{s}", pos=(_DX, OY + sy * (INNER / 2 + WALL_T / 2), DECK_TOP + WALL_H / 2),
          size=(DECK[0], WALL_T, WALL_H), color=DARK) for s, sy in (("left", 1.0), ("right", -1.0))),
    Box("oven_back", pos=(MOUTH_X + DECK[0] + WALL_T / 2, OY, TOP + (DECK[2] + WALL_H) / 2),
        size=(WALL_T, INNER + 2 * WALL_T, DECK[2] + WALL_H), color=DARK),
    *(Box(f"runner_{s}", pos=(MOUTH_X + 0.010 + RUNNER[0] / 2, OY + sy * RUNNER_Y, DECK_TOP + RUNNER[2] / 2),
          size=RUNNER, color=STEEL) for s, sy in (("left", 1.0), ("right", -1.0))),
    Box("pass_pad", pos=(*PASS_XY, TOP + PASS[2] / 2), size=PASS, color=WHITE),
]
OBJECTS = [THE_PAN, THE_PEEL, *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"


def _tilt(quat_xyzw) -> float:
    x, y, _, _ = (float(v) for v in quat_xyzw)
    return math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))))


def _drawn(pan) -> float:
    """How far the pan has come forward from the back of the deck, metres."""
    return PAN_X0 - float(pan.pos[0])


def _on_pass(pan) -> bool:
    """The pan at rest and level on the pass pad, its centre within 3.5 cm of the pad's centre."""
    return bool(
        abs(float(pan.pos[0]) - PASS_XY[0]) <= 0.035
        and abs(float(pan.pos[1]) - PASS_XY[1]) <= 0.035
        and abs(float(pan.low[2]) - PASS_TOP) <= 0.003
        and _tilt(pan.quat_xyzw) <= 5.0
        and pan.resting()
    )


def _peel_out(peel) -> bool:
    """The peel out of the oven (wholly in front of the deck's edge), at rest on the table."""
    return bool(float(peel.high[0]) < MOUTH_X and abs(float(peel.low[2]) - TOP) <= 0.005 and peel.resting())


def score(objects) -> float:
    """0.25 once the pan has been drawn 6 cm forward; 0.5 at 12 cm (its centre within the arm's reach at the mouth);
    0.75 with the pan's centre out of the oven; 1.0 with the pan level and at rest on the pass pad and the peel out of
    the oven, at rest on the table."""
    pan, peel = objects["pan"], objects["peel"]
    if _on_pass(pan) and _peel_out(peel):
        return 1.0
    if float(pan.pos[0]) < MOUTH_X:
        return 0.75
    d = _drawn(pan)
    if d >= 0.120:
        return 0.5
    if d >= 0.060:
        return 0.25
    return 0.0


def success(objects) -> bool:
    """The pan on the pass pad and the peel out of the oven; holds for 1 s."""
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    """Metres from the pan to the pass pad's centre."""
    pan = objects["pan"]
    return float(math.hypot(float(pan.pos[0]) - PASS_XY[0], float(pan.pos[1]) - PASS_XY[1]))


# -- the scripted demo ------------------------------------------------------------------------------------------

YAW = 0.0  # the peel is gripped across its 60 mm width; the closed pads fit between the runners
PAN_YAW = math.pi / 2  # the pan is gripped across its 70 mm depth, the pads in front of and behind it
# The peel turns about 4 degrees in the fingers as it is picked up and carried sideways (measured in sim), which walked
# the pan 8 mm sideways off one runner; the grip is turned back this much while the peel is in the oven.
YAW_TRIM = -0.052
PAN_SLIP = 0.008  # the pan came 1.2 cm further forward than the peel in sim; it is gripped about there
TIP = 0.013
TRAVEL = TOP + 0.120
SQUEEZE = 0.008
# The peel carries the pan cantilevered, and the grip's friction is all that holds it level: with 12 mm of squeeze and
# the pan 19.5 cm ahead of the grip it turned 7 degrees in the fingers in sim and the pan stayed on its runners. So the
# peel is squeezed harder (the finger force limit) and held 15.5 cm behind the pan's centre.
PEEL_SQUEEZE = 0.020
GRIP_BACK = 0.030  # the peel is held 3 cm from its near end; its far end is 18.5 cm ahead of the grip
REACH = PEEL[0] - GRIP_BACK
# The held peel hangs low and droops toward its tip: its tip rode 7 mm below the commanded bottom in sim (and caught
# the deck's front edge when commanded 4 mm above it), so the blade is commanded 14 mm above the deck to slide in.
SLIDE_Z = DECK_TOP + 0.014  # the blade's commanded bottom while it slides in and out between the runners
LIFT = 0.035  # the blade raised this far under the pan to lift it off the runners
X_IN = PAN_X0 + PAN[0] / 2 - 0.005 - REACH  # grip x with the blade's tip 5 mm short of the pan's far edge
X_MOUTH = X_IN - 0.140  # grip x with the pan drawn 14 cm forward, its centre at x 0.46
V_SLOW = 0.013  # m/s average while the blade is under the pan


def _tcp_z(bottom, thickness, depth):
    """TCP height with the pad tips ``depth`` below a part's top."""
    return bottom + thickness - depth + TIP


def demo():
    """Left arm only. Pick the peel off the table 3 cm from its near end; carry it to the oven's centre line and lower
    the blade to just above the deck; slide it in between the runners under the pan; raise it 3.5 cm, lifting the pan
    off the runners; draw it back 14 cm (1.3 cm/s) so the pan comes to the mouth; lower the pan back onto the runners,
    slide the blade out, and lay the peel back on the table. Then grip the pan front to back and carry it to the
    pass pad. 71.8 s."""
    peel_open = -(PEEL[1] + 0.024) / 2
    peel_grip = -(PEEL[1] - PEEL_SQUEEZE) / 2
    px, py = PEEL_START[0] - PEEL[0] / 2 + GRIP_BACK, PEEL_START[1]
    gz_table = _tcp_z(TOP, PEEL[2], PEEL[2] - 0.002)  # pad tips 2 mm above the table
    gz_slide = _tcp_z(SLIDE_Z, PEEL[2], PEEL[2] - 0.002)
    gz_lift = gz_slide + LIFT
    x_front = MOUTH_X - 0.010 - REACH  # the blade's tip 1 cm in front of the deck
    x_clear = MOUTH_X - 0.010 - REACH  # the tip back out of the oven, 1 cm in front of the deck
    x_near = PAN_X0 - PAN[0] / 2 - 0.020 - REACH  # the tip 2 cm short of the pan's near edge: fast up to here
    peel = [
        Waypoint(1.8, (px, py, TRAVEL), peel_open, "peel: over its handle", YAW),
        Waypoint(1.5, (px, py, gz_table + 0.020), peel_open, "peel: descend", YAW),
        Waypoint(1.0, (px, py, gz_table), peel_open, "peel: down slowly", YAW),
        Waypoint(0.5, (px, py, gz_table), peel_open, "peel: settle", YAW),
        Waypoint(1.5, (px, py, gz_table), peel_grip, "peel: close", YAW),
        Waypoint(1.5, (px, py, TRAVEL), peel_grip, "peel: lift", YAW),
        Waypoint(4.0, (x_front, OY, TRAVEL), peel_grip, "peel: carry to the oven mouth", YAW + YAW_TRIM),
        Waypoint(1.5, (x_front, OY, gz_slide), peel_grip, "peel: lower the blade to the deck", YAW + YAW_TRIM),
        Waypoint(0.6, (x_front, OY, gz_slide), peel_grip, "peel: settle", YAW + YAW_TRIM),
        Waypoint(3.0, (x_near, OY, gz_slide), peel_grip, "peel: slide in between the runners", YAW + YAW_TRIM),
        Waypoint(round((X_IN - x_near) / V_SLOW, 1), (X_IN, OY, gz_slide), peel_grip, "peel: slide under the pan", YAW + YAW_TRIM),
        Waypoint(0.6, (X_IN, OY, gz_slide), peel_grip, "peel: settle", YAW + YAW_TRIM),
        Waypoint(2.5, (X_IN, OY, gz_lift), peel_grip, "peel: raise, lifting the pan off the runners", YAW + YAW_TRIM),
        Waypoint(0.6, (X_IN, OY, gz_lift), peel_grip, "peel: settle", YAW + YAW_TRIM),
        Waypoint(round((X_IN - X_MOUTH) / V_SLOW, 1), (X_MOUTH, OY, gz_lift), peel_grip, "peel: draw the pan forward", YAW + YAW_TRIM),
        Waypoint(0.6, (X_MOUTH, OY, gz_lift), peel_grip, "peel: settle", YAW + YAW_TRIM),
        Waypoint(2.5, (X_MOUTH, OY, gz_slide), peel_grip, "peel: lower the pan onto the runners", YAW + YAW_TRIM),
        Waypoint(0.6, (X_MOUTH, OY, gz_slide), peel_grip, "peel: settle", YAW + YAW_TRIM),
        Waypoint(3.0, (x_clear, OY, gz_slide), peel_grip, "peel: slide out from under the pan", YAW + YAW_TRIM),
        Waypoint(1.5, (x_clear, OY, TRAVEL), peel_grip, "peel: lift clear", YAW + YAW_TRIM),
        Waypoint(2.5, (px, py, TRAVEL), peel_grip, "peel: carry back to its place", YAW),
        Waypoint(1.5, (px, py, gz_table + 0.022), peel_grip, "peel: lower", YAW),
        Waypoint(1.0, (px, py, gz_table + 0.002), peel_grip, "peel: set down slowly", YAW),
        Waypoint(1.5, (px, py, gz_table + 0.002), peel_open, "peel: release", YAW),
        Waypoint(1.0, (px, py, TRAVEL), peel_open, "peel: retreat", YAW),
    ]
    mx = PAN_X0 - 0.140 - PAN_SLIP
    gz_pan = _tcp_z(RUNNER_TOP, PAN[2], 0.015)
    qz = _tcp_z(PASS_TOP, PAN[2], 0.015) + 0.002
    pan_grip = -(PAN[0] - SQUEEZE) / 2
    pan = [
        Waypoint(1.8, (mx, OY, TRAVEL), FINGER_OPEN, "pan: over it at the mouth", PAN_YAW),
        Waypoint(1.5, (mx, OY, gz_pan + 0.020), FINGER_OPEN, "pan: descend", PAN_YAW),
        Waypoint(1.0, (mx, OY, gz_pan), FINGER_OPEN, "pan: down slowly", PAN_YAW),
        Waypoint(0.5, (mx, OY, gz_pan), FINGER_OPEN, "pan: settle", PAN_YAW),
        Waypoint(1.5, (mx, OY, gz_pan), pan_grip, "pan: close", PAN_YAW),
        Waypoint(1.5, (mx, OY, gz_pan + 0.030), pan_grip, "pan: lift straight up", PAN_YAW),
        Waypoint(1.2, (mx, OY, TRAVEL), pan_grip, "pan: lift", PAN_YAW),
        Waypoint(2.5, (*PASS_XY, TRAVEL), pan_grip, "pan: carry to the pass", PAN_YAW),
        Waypoint(0.6, (*PASS_XY, TRAVEL), pan_grip, "pan: settle over", PAN_YAW),
        Waypoint(1.5, (*PASS_XY, qz + 0.020), pan_grip, "pan: lower", PAN_YAW),
        Waypoint(1.5, (*PASS_XY, qz), pan_grip, "pan: set down slowly", PAN_YAW),
        Waypoint(0.6, (*PASS_XY, qz), pan_grip, "pan: settle", PAN_YAW),
        Waypoint(1.5, (*PASS_XY, qz), FINGER_OPEN, "pan: release", PAN_YAW),
        Waypoint(1.0, (*PASS_XY, TRAVEL), FINGER_OPEN, "pan: retreat", PAN_YAW),
    ]
    return {"left": peel + pan}


def self_test(ObjectState=None):
    """The scoring rules on hand-built states; runs with the real ObjectState or any stand-in with the same fields."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def body(name, size, x, y, bottom, tilt_deg=0.0, vel=(0.0, 0.0, 0.0)):
        t = math.radians(tilt_deg)
        q = np.array([math.sin(t / 2), 0.0, 0.0, math.cos(t / 2)])
        half = np.array(size) / 2
        pos = np.array([x, y, bottom + half[2]])
        return {name: ObjectState(pos, q, np.asarray(vel, float), np.zeros(3), pos - half, pos + half)}

    def pan(x, y=OY, bottom=RUNNER_TOP, **kw):
        return body("pan", PAN, x, y, bottom, **kw)

    def peel(x=PEEL_START[0], y=PEEL_START[1], bottom=TOP, **kw):
        return body("peel", PEEL, x, y, bottom, **kw)

    peel_in = peel(X_IN + REACH - PEEL[0] / 2, OY, SLIDE_Z)
    cases = {
        "start": (pan(PAN_X0) | peel(), 0.0, False),
        "pan riding on the peel, 7 cm forward": (pan(PAN_X0 - 0.07, bottom=RUNNER_TOP + 0.015, vel=(-0.013, 0, 0)) | peel_in, 0.25, False),
        "pan at the mouth on the runners": (pan(PAN_X0 - 0.14) | peel(), 0.5, False),
        "pan lifted out, held over the table": (pan(0.30, 0.38, TOP + 0.08) | peel(), 0.75, False),
        "pan on the pass, peel back on the table": (pan(*PASS_XY, PASS_TOP) | peel(), 1.0, True),
        "pan on the pass, peel laid anywhere outside": (pan(*PASS_XY, PASS_TOP) | peel(0.25, 0.30), 1.0, True),
        "pan on the pass, peel left in the oven": (pan(*PASS_XY, PASS_TOP) | peel_in, 0.75, False),
        "pan on the pass, 3 cm off centre": (pan(PASS_XY[0] + 0.03, PASS_XY[1], PASS_TOP) | peel(), 1.0, True),
        "pan beside the pass on the table": (pan(PASS_XY[0] + 0.10, PASS_XY[1], TOP) | peel(), 0.75, False),
        "pan tipped on the pass": (pan(*PASS_XY, PASS_TOP, tilt_deg=20) | peel(), 0.75, False),
        "pan on the pass, still sliding": (pan(*PASS_XY, PASS_TOP, vel=(0.03, 0, 0)) | peel(), 0.75, False),
        "pan pushed 4 cm only": (pan(PAN_X0 - 0.04) | peel(), 0.0, False),
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
