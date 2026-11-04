"""On the station table, in the left arm's lane, a 1 g bench analogue of clocking a loose part to a commanded angle
on a fixed degree dial used as a layout template. The white dial, 240 mm across and 6 mm tall, carries thirteen
black graduations every 15 degrees over its far half and no numerals. Zero is the red tick toward the robot's left,
90 is straight ahead and 180 is toward the right; 0, 90 and 180 are long ticks, the rest short. A loose blue
pointer bar, 170 x 24 x 20 mm, lies on the table inboard of the dial, parallel to the 90 degree line. It must be
laid across the dial centre along the 120 degree graduation, which looks like every other short tick: the policy
has to find zero and the direction the scale runs, then set the bar 30 degrees past the 90 tick, two graduations
on. The score reads the bar from its centre and quaternion: its lowest point, the level of its long axis, its
centre offset, the angle of its long axis and the point where its centre line meets the tick ring, both folded
because the bar is symmetric.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/gimbal-dial-setting/gimbal_dial_setting.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "Lay the blue pointer flat across the centre of the dial, lined up with the 120 degree graduation. The red tick is zero and the scale runs to 180 degrees." \\
           --scorer task_score --max-steps 2400 --seed 0 --no-prompt
"""

import math

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. The left arm's base is at (0, +0.305, 0.75).
# Dial angles: angle a points along (sin a, cos a) from the dial centre, so 0 is toward +y (the robot's left),
# 90 toward +x (straight ahead) and 180 toward -y. A bar along angle a has world yaw 90 - a degrees. The scale
# rises toward the midline so the 120 graduation lies on the side the raised left arm does not cover in the top
# camera while it places.
DIAL_C = (0.230, 0.305)  # straight in front of the left arm's base
DIAL_R = 0.120
DIAL_H = 0.006
DIAL_TOP = TOP + DIAL_H
MARK_T = 0.0005  # graduations and centre cross stand this proud of the dial top
TICK_W = 0.008
MINOR_R = (0.093, 0.118)  # inner and outer radius of a short graduation
MAJOR_R = (0.078, 0.118)  # 0, 90 and 180
RING_R = (MINOR_R[0] + MINOR_R[1]) / 2  # 105.5 mm, the middle of the short graduations: where a reading is taken
STEP_DEG = 15
TARGET_DEG = 120  # two short graduations past the long 90 tick; 105 and 135, one graduation off, score 0
TARGET_YAW_DEG = 90 - TARGET_DEG  # -30: the bar's world yaw on the target graduation
CROSS = (0.024, 0.004)  # each of the two centre bars

POINTER = (0.170, 0.024, 0.020)  # long, wide, thick; long axis along x at the start, i.e. on the 90 degree line
POINTER_MASS = 0.08
POINTER_START = (0.200, 0.110)  # inboard of the dial, 63 mm clear of its edge, 0.279 m from the left base; outboard, the
# left arm at its ready pose hides it from the top camera

WHITE = (0.92, 0.92, 0.90)
BLACK = (0.05, 0.05, 0.05)
RED = (0.85, 0.12, 0.12)
BLUE = (0.15, 0.40, 0.85)


def _tick(a_deg):
    inner, outer = MAJOR_R if a_deg % 90 == 0 else MINOR_R
    a = math.radians(a_deg)
    r = (inner + outer) / 2
    return Box(
        f"tick_{a_deg:03d}",
        pos=(DIAL_C[0] + math.sin(a) * r, DIAL_C[1] + math.cos(a) * r, DIAL_TOP + MARK_T / 2),
        size=(outer - inner, TICK_W, MARK_T),
        color=RED if a_deg == 0 else BLACK,
        yaw=math.pi / 2 - a,
    )


DIAL = Cylinder("dial", pos=(DIAL_C[0], DIAL_C[1], TOP + DIAL_H / 2), radius=DIAL_R, height=DIAL_H, color=WHITE)
TICKS = [_tick(a) for a in range(0, 181, STEP_DEG)]
CENTRE_CROSS = [
    Box("cross_x", pos=(DIAL_C[0], DIAL_C[1], DIAL_TOP + MARK_T / 2), size=(CROSS[0], CROSS[1], MARK_T), color=BLACK),
    Box("cross_y", pos=(DIAL_C[0], DIAL_C[1], DIAL_TOP + MARK_T / 2), size=(CROSS[1], CROSS[0], MARK_T), color=BLACK),
]
POINTER_BOX = Box("pointer", pos=(POINTER_START[0], POINTER_START[1], TOP + POINTER[2] / 2), size=POINTER, mass=POINTER_MASS, color=BLUE)
OBJECTS = [POINTER_BOX, DIAL, *TICKS, *CENTRE_CROSS]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

EPS = 1e-4  # 0.1 mm, so a length exactly on a limit passes
EPS_DEG = 1e-3
LEVEL_TOL_DEG = 5.0  # the bar's long axis to the horizontal; any roll about that axis, so any of its four long faces down
BOTTOM_TOL = 0.003  # its lowest point to the dial top; on the centre cross it is 0.5 mm up, held at release 6 mm
CENTRE_TOL = 0.012  # its centre to the dial centre, horizontally: half the bar's width
FULL_DEG = 5.0  # axis angle and scale angle to the 120 degree graduation for full credit
HALF_DEG = STEP_DEG / 2  # under this both are still nearer 120 than 105 or 135: half credit


def _rot(quat_xyzw):
    x, y, z, w = (float(v) for v in quat_xyzw)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def _off_target(dial_deg):
    """Degrees from the target graduation to a dial angle, 0..180."""
    d = (dial_deg - TARGET_DEG) % 360.0
    return min(d, 360.0 - d)


def _reading(pointer):
    """(placed, e, s), all from the bar's centre and quaternion. placed: at rest, long axis within 5 degrees of
    level (any of the four long faces down), lowest point within 3 mm of the dial top and centre within 12 mm of
    the dial centre. e: degrees between the long axis seen from above and the 120 degree ray, folded to 0..90
    because the bar is symmetric end for end. s: degrees about the dial centre between the 120 graduation and the
    point where the bar's centre line, extended, crosses the tick ring at radius 105.5 mm, the nearer of its two
    crossings; this is where the bar points on the scale, so a bar shifted sideways reads a different graduation."""
    rot = _rot(pointer.quat_xyzw)
    heading = math.atan2(float(rot[1, 0]), float(rot[0, 0]))
    d = (math.degrees(heading) - TARGET_YAW_DEG) % 180.0
    e = min(d, 180.0 - d)
    ux, uy = math.cos(heading), math.sin(heading)
    dx, dy = float(pointer.pos[0]) - DIAL_C[0], float(pointer.pos[1]) - DIAL_C[1]
    b = dx * ux + dy * uy
    disc = b * b - (dx * dx + dy * dy - RING_R * RING_R)
    s = 180.0
    if disc >= 0.0:
        for t in (-b + math.sqrt(disc), -b - math.sqrt(disc)):
            s = min(s, _off_target(math.degrees(math.atan2(dx + t * ux, dy + t * uy))))
    if not pointer.resting():
        return False, e, s
    lowest = float(pointer.pos[2]) - float(np.abs(rot[2, :]) @ (np.asarray(POINTER) / 2))
    level = math.degrees(math.asin(min(1.0, abs(float(rot[2, 0]))))) <= LEVEL_TOL_DEG + EPS_DEG
    on_dial = abs(lowest - DIAL_TOP) <= BOTTOM_TOL + EPS
    centred = math.hypot(dx, dy) <= CENTRE_TOL + EPS
    return bool(level and on_dial and centred), e, s


def success(objects) -> bool:
    """The pointer placed across the dial centre, its long axis and the point where its centre line meets the
    scale both within 5 degrees of the 120 degree graduation; holds for 1 s."""
    placed, e, s = _reading(objects["pointer"])
    return placed and max(e, s) <= FULL_DEG + EPS_DEG


def distance(objects) -> float:
    """Metres still to go: the pointer centre to the dial centre plus the arc its end, 85 mm out, must still sweep."""
    p = objects["pointer"]
    _, e, _ = _reading(p)
    return math.hypot(float(p.pos[0]) - DIAL_C[0], float(p.pos[1]) - DIAL_C[1]) + POINTER[0] / 2 * math.radians(e)


def score(objects) -> float:
    """1.0 placed with the axis angle e and the scale angle s both within 5 degrees of the 120 degree graduation
    (the success condition); 0.5 placed otherwise with both under 7.5 degrees, so the bar and the point where its
    line meets the scale are still nearer 120 than either neighbouring graduation; 0 otherwise, including a bar
    placed on any other graduation, shifted sideways onto one, off centre, tilted along its length, still moving
    or off the dial."""
    placed, e, s = _reading(objects["pointer"])
    if not placed:
        return 0.0
    worst = max(e, s)
    if worst <= FULL_DEG + EPS_DEG:
        return 1.0
    return 0.5 if worst < HALF_DEG - EPS_DEG else 0.0


TRAVEL = TOP + 0.145
TCP_UP = 0.018  # TCP above the bar's bottom face: pad tips 5 mm above the table at the pick, 15 mm of bar gripped
GRIP_Z = TOP + TCP_UP
PLACE_Z = DIAL_TOP + 0.006 + TCP_UP  # bar bottom 6 mm above the dial top, 5.5 mm above the cross; pad tips 11 mm up
ENTRY = 0.015  # the pause above the dial: bar bottom 21 mm above the dial top
SQUEEZE = 0.008  # each close is commanded 8 mm narrower than the part


def _pick_place(pick, place, *, width, entry, travel_z, yaw=0.0, place_yaw=None):
    """A top-down pick and place paced for the simulated arm, whose soft wrist trails a moving target by several
    millimetres. The arm settles before every close and release, stops ``entry`` above the place with the part
    clear of every fixture, and makes the last descent slowly. The fingers close to 8 mm narrower than the part's
    ``width`` across the pads rather than fully shut: each finger then pushes 4 mm x 5000 N/m = 20 N, under its
    50 N cap, so the grip centres the part, whereas a fully commanded close saturates both fingers and lets the part
    slide onto one of them. The gripper opens and closes over 1.5 s, inside the default 0.05 per step limiter.
    16.2 s."""
    (px, py, pz), (qx, qy, qz) = pick, place
    end = yaw if place_yaw is None else place_yaw
    grip = -(width - SQUEEZE) / 2  # finger joint target, m
    return [
        Waypoint(1.5, (px, py, travel_z), FINGER_OPEN, "to the object", yaw),
        Waypoint(1.5, (px, py, pz), FINGER_OPEN, "descend", yaw),
        Waypoint(0.6, (px, py, pz), FINGER_OPEN, "settle", yaw),
        Waypoint(1.5, (px, py, pz), grip, "close", yaw),
        Waypoint(1.2, (px, py, travel_z), grip, "lift", yaw),
        Waypoint(2.0, (qx, qy, travel_z), grip, "carry", end),
        Waypoint(0.6, (qx, qy, travel_z), grip, "settle over the place", end),
        Waypoint(1.2, (qx, qy, qz + entry), grip, "lower to the entry", end),
        Waypoint(0.8, (qx, qy, qz + entry), grip, "settle at the entry", end),
        Waypoint(2.0, (qx, qy, qz), grip, "set in", end),
        Waypoint(0.8, (qx, qy, qz), grip, "settle", end),
        Waypoint(1.5, (qx, qy, qz), FINGER_OPEN, "release", end),
        Waypoint(1.0, (qx, qy, travel_z), FINGER_OPEN, "retreat", end),
    ]


def demo():
    """Left arm only. A top-down pick at the pointer's centre with the fingers across its 24 mm width, a carry with
    the TCP 145 mm up that turns the grip 30 degrees clockwise seen from above, so the bar's long axis lies along
    the 120 degree graduation, and a release with the bar 6 mm above the dial centre. 16.2 s of plan, then the
    3 s hold: 384 steps at 20 Hz."""
    left = _pick_place(
        (POINTER_START[0], POINTER_START[1], GRIP_Z), (DIAL_C[0], DIAL_C[1], PLACE_Z),
        width=POINTER[1], entry=ENTRY, travel_z=TRAVEL, yaw=0.0, place_yaw=math.radians(TARGET_YAW_DEG),
    )
    return {"left": left}


def self_test(ObjectState=None):
    """The scoring rules on hand-built states; runs with the real ObjectState or any stand-in with the same fields."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def rz(a):
        return np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])

    def rx(a):
        return np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])

    def ry(a):
        return np.array([[math.cos(a), 0, math.sin(a)], [0, 1, 0], [-math.sin(a), 0, math.cos(a)]])

    def quat(r):  # rotation matrix to x, y, z, w, stable for any angle
        t = r[0, 0] + r[1, 1] + r[2, 2]
        if t > 0:
            s = 2 * math.sqrt(1 + t)
            q = [(r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s, s / 4]
        elif r[0, 0] >= r[1, 1] and r[0, 0] >= r[2, 2]:
            s = 2 * math.sqrt(1 + r[0, 0] - r[1, 1] - r[2, 2])
            q = [s / 4, (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s, (r[2, 1] - r[1, 2]) / s]
        elif r[1, 1] >= r[2, 2]:
            s = 2 * math.sqrt(1 + r[1, 1] - r[0, 0] - r[2, 2])
            q = [(r[0, 1] + r[1, 0]) / s, s / 4, (r[1, 2] + r[2, 1]) / s, (r[0, 2] - r[2, 0]) / s]
        else:
            s = 2 * math.sqrt(1 + r[2, 2] - r[0, 0] - r[1, 1])
            q = [(r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s, s / 4, (r[1, 0] - r[0, 1]) / s]
        return np.array(q)

    def bar(x, y, bottom=DIAL_TOP + MARK_T, yaw_deg=TARGET_YAW_DEG, roll=0.0, pitch=0.0, flip=False, vel=(0.0, 0.0, 0.0)):
        """The pointer with its long axis at world yaw ``yaw_deg``, rolled about and pitched across that axis,
        optionally upside down, its lowest point at ``bottom``."""
        r = rz(math.radians(yaw_deg)) @ ry(pitch) @ rx(roll + (math.pi if flip else 0.0))
        half = np.abs(r) @ (np.asarray(POINTER) / 2)
        pos = np.array([x, y, bottom + half[2]])
        return ObjectState(pos, quat(r), np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    def on(dial_deg, side=0.0, along=0.0, **kw):
        """Along the graduation at ``dial_deg``, its centre ``along`` m out along the bar and ``side`` m square to
        it toward lower dial angles (toward 105 for a bar near 120) from the dial centre."""
        a = math.radians(dial_deg)
        x = DIAL_C[0] + along * math.sin(a) - side * math.cos(a)
        y = DIAL_C[1] + along * math.cos(a) + side * math.sin(a)
        return bar(x, y, yaw_deg=90 - dial_deg, **kw)

    def scene(p):
        return {"pointer": p}

    cx, cy = DIAL_C
    end_lift = math.asin((DIAL_TOP - TOP) / POINTER[0])  # one end on the dial edge, the other on the table
    cases = {
        "start": (scene(bar(*POINTER_START, bottom=TOP, yaw_deg=0.0)), 0.0, False),
        "done, on the 120 graduation, resting on the centre cross": (scene(on(120)), 1.0, True),
        "done, upside down": (scene(on(120, flip=True)), 1.0, True),
        "done, reversed end for end": (scene(on(300)), 1.0, True),
        "done, flat on the dial face beside the cross": (scene(on(120, bottom=DIAL_TOP)), 1.0, True),
        "done, on its 170 x 20 side face": (scene(on(120, roll=math.pi / 2)), 1.0, True),
        "done, rolled 4 degrees": (scene(on(120, roll=math.radians(4))), 1.0, True),
        "4.9 degrees off": (scene(on(124.9)), 1.0, True),
        "exactly 5.0 degrees off": (scene(on(115)), 1.0, True),
        "set at 113, 7 degrees off": (scene(on(113)), 0.5, False),
        "set at 127, 7 degrees off": (scene(on(127)), 0.5, False),
        "set at 112.5, midway to 105": (scene(on(112.5)), 0.0, False),
        "set on the 90 graduation, carried without turning": (scene(on(90)), 0.0, False),
        "set on 105, one graduation short": (scene(on(105)), 0.0, False),
        "set on 135, one graduation over": (scene(on(135)), 0.0, False),
        "set on 75, one graduation the wrong way from 90": (scene(on(75)), 0.0, False),
        "set on 60, the scale read from the 180 end": (scene(on(60)), 0.0, False),
        "set on the red zero tick": (scene(on(0)), 0.0, False),
        "set on 180": (scene(on(180)), 0.0, False),
        "on 120, centre 11.9 mm off along the bar": (scene(on(120, along=0.0119)), 1.0, True),
        "on 120, centre exactly 12 mm off along the bar": (scene(on(120, along=-0.012)), 1.0, True),
        "on 120, centre 4 mm off sideways": (scene(on(120, side=0.004)), 1.0, True),
        "on 120, centre 9.2 mm off sideways, line meets the ring 5.0 degrees off": (scene(on(120, side=RING_R * math.sin(math.radians(5)))), 1.0, True),
        "on 120, centre 11.9 mm off sideways toward 105": (scene(on(120, side=0.0119)), 0.5, False),
        "on 120, centre 11.9 mm off sideways toward 135": (scene(on(120, side=-0.0119)), 0.5, False),
        "at 115.1, centre 11.9 mm off sideways toward 105, reads 105": (scene(on(115.1, side=0.0119)), 0.0, False),
        "at 115.1, centre 6 mm off sideways toward 105": (scene(on(115.1, side=0.006)), 0.0, False),
        "at 124.9, centre 9 mm off toward 105, line through the 120 tick": (scene(on(124.9, side=0.009)), 1.0, True),
        "centre 13 mm off": (scene(bar(cx + 0.013 * 0.6, cy + 0.013 * 0.8)), 0.0, False),
        "right angle, one end on the 120 graduation, not across the centre": (scene(on(120, along=0.040)), 0.0, False),
        "pitched 6 degrees along its length": (scene(on(120, pitch=math.radians(6))), 0.0, False),
        "bottom exactly 3 mm above the dial": (scene(on(120, bottom=DIAL_TOP + BOTTOM_TOL)), 1.0, True),
        "bottom 3.5 mm above the dial": (scene(on(120, bottom=DIAL_TOP + 0.0035)), 0.0, False),
        "held still at the release height, 6 mm up": (scene(on(120, bottom=DIAL_TOP + 0.006)), 0.0, False),
        "placed but still moving": (scene(on(120, vel=(0.02, 0.0, 0.0))), 0.0, False),
        "on the table beside the dial at the right angle": (scene(bar(cx, cy - DIAL_R - 0.045, bottom=TOP)), 0.0, False),
        "one end on the dial edge, the other on the table": (scene(bar(cx - DIAL_R - 0.050, cy, bottom=TOP, yaw_deg=0.0, pitch=-end_lift)), 0.0, False),
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
