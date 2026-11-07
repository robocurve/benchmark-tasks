"""On the station table, a 1 g bench analogue of an orbital replacement unit swap. A red unit, 6 x 4 x 9 cm and
0.15 kg, stands in a keyed bay on the midline 26 cm in front of the arm bases with its 60 mm side along x. A grey
spare of the same size and mass stands on a white mark pad 24 cm in front of the right arm's base, turned 90
degrees. A return slot 26 cm in front of the left arm's base is keyed 90 degrees to the bay, so the red unit only
fits it turned; each pocket leaves 5 mm per side. The bay, the slot and the pad are fixed; both units are loose.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/oru-bay-swap/oru_bay_swap.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "Pull the red unit out of the bay and stow it in the return slot, turned 90 degrees to fit, then seat the grey spare in the empty bay." \\
           --scorer task_score ...
"""

import math

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. Arm bases at (0, +0.305, 0.75) and (0, -0.305, 0.75).
UNIT = (0.060, 0.040, 0.090)  # the units' size at yaw 0: 60 mm along x, 40 mm along y, 90 mm tall
BAY = (0.260, 0.000)  # keyed for the 60 mm side along x; on the midline, within both arms' reach
SLOT = (0.260, 0.300)  # keyed for the 60 mm side along y; the left arm's reach only
PAD = (0.240, -0.320)  # the spare's start mark; the right arm's reach only
UNIT_MASS = 0.15
WALL_H = 0.040
WALL_T = 0.010
CLEAR = 0.005  # per side, so a unit inside a pocket is within 7.1 mm and 10.3 degrees of its key

RED = (0.85, 0.12, 0.12)
GREY = (0.70, 0.70, 0.72)
DARK = (0.25, 0.25, 0.27)
WHITE = (0.95, 0.95, 0.93)


def _pocket(prefix: str, centre: tuple[float, float], long_axis_x: bool) -> list[Box]:
    """Four fixed walls around an opening 70 x 50 mm (long side along x) or 50 x 70 mm, open floor, 40 mm tall."""
    cx, cy = centre
    inner_long = UNIT[0] + 2 * CLEAR  # 0.070
    inner_short = UNIT[1] + 2 * CLEAR  # 0.050
    outer_long = inner_long + 2 * WALL_T  # 0.090
    z = TOP + WALL_H / 2
    walls = []
    if long_axis_x:
        # long walls run along x, either side in y; short walls close the ends in x
        for s in (+1, -1):
            walls.append(Box(f"{prefix}_wall_y{'p' if s > 0 else 'n'}", pos=(cx, cy + s * (inner_short / 2 + WALL_T / 2), z), size=(outer_long, WALL_T, WALL_H), color=DARK))
            walls.append(Box(f"{prefix}_wall_x{'p' if s > 0 else 'n'}", pos=(cx + s * (inner_long / 2 + WALL_T / 2), cy, z), size=(WALL_T, inner_short, WALL_H), color=DARK))
    else:
        for s in (+1, -1):
            walls.append(Box(f"{prefix}_wall_x{'p' if s > 0 else 'n'}", pos=(cx + s * (inner_short / 2 + WALL_T / 2), cy, z), size=(WALL_T, outer_long, WALL_H), color=DARK))
            walls.append(Box(f"{prefix}_wall_y{'p' if s > 0 else 'n'}", pos=(cx, cy + s * (inner_long / 2 + WALL_T / 2), z), size=(inner_short, WALL_T, WALL_H), color=DARK))
    return walls


PAD_BOX = Box("mark_pad", pos=(PAD[0], PAD[1], TOP + 0.001), size=(0.090, 0.070, 0.002), color=WHITE)
UNIT_RED = Box("unit_red", pos=(BAY[0], BAY[1], TOP + UNIT[2] / 2), size=UNIT, mass=UNIT_MASS, color=RED)
UNIT_SPARE = Box("unit_spare", pos=(PAD[0], PAD[1], TOP + 0.002 + UNIT[2] / 2), size=UNIT, mass=UNIT_MASS, color=GREY, yaw=math.pi / 2)
OBJECTS = [UNIT_RED, UNIT_SPARE, PAD_BOX, *_pocket("bay", BAY, long_axis_x=True), *_pocket("slot", SLOT, long_axis_x=False)]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

UPRIGHT_COS = math.cos(math.radians(5.0))
KEY_SIN = math.sin(math.radians(11.0))  # a unit flat inside a pocket turns at most 10.3 degrees
BOTTOM_TOL = 0.003
CENTRE_TOL = 0.0075  # a unit flat in a pocket corner is 7.07 mm off centre
OUT_OF_BAY = 0.050


def _axes(state):
    """The unit's own x axis (its 60 mm side) and z axis (its 90 mm height) in the world, from the xyzw quaternion."""
    x, y, z, w = (float(v) for v in state.quat_xyzw)
    ex = np.array([1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y + z * w), 2.0 * (x * z - y * w)])
    ez = np.array([2.0 * (x * z + y * w), 2.0 * (y * z - x * w), 1.0 - 2.0 * (x * x + y * y)])
    return ex, ez


def _upright(state) -> bool:
    """Standing: the 90 mm axis within 5 degrees of vertical, either end down, the bottom within 3 mm of the table,
    at rest (under 10 mm/s and 0.5 rad/s). The units are uniform blocks, so either end down looks the same."""
    _, ez = _axes(state)
    return bool(abs(ez[2]) > UPRIGHT_COS and abs(float(state.low[2]) - TOP) < BOTTOM_TOL and state.resting())


def _seated(state, centre: tuple[float, float], long_axis_x: bool) -> bool:
    """Upright, centred within 7.5 mm of the pocket and keyed within 11 degrees of its long axis."""
    if not _upright(state):
        return False
    if math.hypot(float(state.pos[0]) - centre[0], float(state.pos[1]) - centre[1]) > CENTRE_TOL:
        return False
    ex, _ = _axes(state)
    h = math.hypot(ex[0], ex[1]) or 1.0
    across = abs(ex[1] / h) if long_axis_x else abs(ex[0] / h)
    return across < KEY_SIN


def _out_of_bay(state) -> bool:
    return math.hypot(float(state.pos[0]) - BAY[0], float(state.pos[1]) - BAY[1]) > OUT_OF_BAY


def success(objects) -> bool:
    """The spare is seated in the bay and the red unit is seated in the return slot; holds for 1 s."""
    return _seated(objects["unit_spare"], BAY, True) and _seated(objects["unit_red"], SLOT, False)


def distance(objects) -> float:
    """Metres still to go: the spare to the bay centre plus the red unit to the slot centre, at standing height."""
    goal_z = TOP + UNIT[2] / 2
    spare = objects["unit_spare"].pos - np.array([BAY[0], BAY[1], goal_z])
    red = objects["unit_red"].pos - np.array([SLOT[0], SLOT[1], goal_z])
    return float(np.linalg.norm(spare) + np.linalg.norm(red))


def score(objects) -> float:
    """End-state ladder. 0 while the red unit is still in the bay. Red out and standing on the table: 0.25. Red
    seated in the slot: 0.5. Spare seated in the bay: 0.5, or 0.75 if the red unit is standing, 1.0 if the red unit
    is seated in the slot. A lifted, lying or tilted unit earns nothing."""
    red, spare = objects["unit_red"], objects["unit_spare"]
    if not _out_of_bay(red):
        return 0.0
    red_up = _upright(red)
    red_slot = _seated(red, SLOT, False)
    spare_bay = _seated(spare, BAY, True)
    if spare_bay and red_slot:
        return 1.0
    if spare_bay:
        return 0.75 if red_up else 0.5
    if red_slot:
        return 0.5
    return 0.25 if red_up else 0.0


TRAVEL = TOP + 0.150
GRIP_Z = TOP + 0.060  # TCP 30 mm below the unit top, pads across the 40 mm faces, tips 7 mm above a pocket rim
PLACE_Z = TOP + 0.065  # unit bottom 5 mm above the pocket floor at release
ENTRY = 0.045  # the pause above a pocket: unit bottom 10 mm above the 40 mm walls


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
    """Left arm: pull the red unit straight up out of the bay, carry it to the slot turned minus 90 degrees, set it in, park out of the way. Right arm: hover over the
    spare with the grip turned 90 degrees, pick it up once the red unit is out of the bay, and carry it into the bay
    turned back to the bay's key; its carry starts after the left arm has left the bay. Left 17.7 s and right
    20.7 s of plan, then the 3 s hold: 474 steps at 20 Hz."""
    left = _pick_place((BAY[0], BAY[1], GRIP_Z), (SLOT[0], SLOT[1], PLACE_Z), width=UNIT[1], entry=ENTRY, travel_z=TRAVEL, yaw=0.0, place_yaw=-math.pi / 2)
    left.append(Waypoint(1.5, (0.160, SLOT[1], TRAVEL), FINGER_OPEN, "park", -math.pi / 2))
    hover = (PAD[0], PAD[1], TRAVEL)
    right = [
        Waypoint(1.5, hover, FINGER_OPEN, "hover", math.pi / 2),
        Waypoint(3.0, hover, FINGER_OPEN, "wait for the red unit to come out", math.pi / 2),
    ]
    right += _pick_place((PAD[0], PAD[1], GRIP_Z), (BAY[0], BAY[1], PLACE_Z), width=UNIT[1], entry=ENTRY, travel_z=TRAVEL, yaw=math.pi / 2, place_yaw=0.0)
    return {"left": left, "right": right}


def self_test(ObjectState=None):
    """The scoring ladder on hand-built states; runs with the real ObjectState or any stand-in with the same fields."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def unit(x, y, yaw=0.0, z=None, tilt=0.0, vel=(0.0, 0.0, 0.0)):
        pos = np.array([x, y, TOP + UNIT[2] / 2 if z is None else z])
        qz = math.sin(yaw / 2) * math.cos(tilt / 2)
        qw = math.cos(yaw / 2) * math.cos(tilt / 2)
        qx = math.sin(tilt / 2) * math.cos(yaw / 2)
        qy = math.sin(tilt / 2) * math.sin(yaw / 2)
        half = np.array([UNIT[0] / 2, UNIT[1] / 2, UNIT[2] / 2])
        return ObjectState(pos, np.array([qx, qy, qz, qw]), np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    on_pad = unit(*PAD, yaw=math.pi / 2, z=TOP + 0.002 + UNIT[2] / 2)
    cases = {
        "start": ({"unit_red": unit(*BAY), "unit_spare": on_pad}, 0.0, False),
        "red out, standing": ({"unit_red": unit(0.18, 0.12), "unit_spare": on_pad}, 0.25, False),
        "red out, lying": ({"unit_red": unit(0.18, 0.12, tilt=math.pi / 2, z=TOP + 0.02), "unit_spare": on_pad}, 0.0, False),
        "red stowed": ({"unit_red": unit(*SLOT, yaw=math.pi / 2), "unit_spare": on_pad}, 0.5, False),
        "spare seated, red standing": ({"unit_red": unit(0.18, 0.12), "unit_spare": unit(*BAY)}, 0.75, False),
        "spare seated, red lying": ({"unit_red": unit(0.18, 0.12, tilt=math.pi / 2, z=TOP + 0.02), "unit_spare": unit(*BAY)}, 0.5, False),
        "done": ({"unit_red": unit(*SLOT, yaw=math.pi / 2), "unit_spare": unit(*BAY)}, 1.0, True),
        "spare held in the air": ({"unit_red": unit(*SLOT, yaw=math.pi / 2), "unit_spare": unit(*BAY, z=TOP + 0.12, vel=(0, 0, 0.05))}, 0.5, False),
        "red across the slot key": ({"unit_red": unit(*SLOT, yaw=0.0), "unit_spare": unit(*BAY)}, 0.75, False),
        "red 5 mm and 10 deg off": ({"unit_red": unit(SLOT[0] + 0.005, SLOT[1] - 0.002, yaw=math.pi / 2 + math.radians(10)), "unit_spare": unit(*BAY)}, 1.0, True),
        "red 8 mm off": ({"unit_red": unit(SLOT[0] + 0.008, SLOT[1], yaw=math.pi / 2), "unit_spare": unit(*BAY)}, 0.75, False),
        "red 12 deg off its key": ({"unit_red": unit(*SLOT, yaw=math.pi / 2 + math.radians(12)), "unit_spare": unit(*BAY)}, 0.75, False),
        "red tilted 8 deg": ({"unit_red": unit(*SLOT, yaw=math.pi / 2, tilt=math.radians(8)), "unit_spare": unit(*BAY)}, 0.5, False),
        "spare left on pad": ({"unit_red": unit(*SLOT, yaw=math.pi / 2), "unit_spare": on_pad}, 0.5, False),
        "red beside the bay": ({"unit_red": unit(BAY[0], BAY[1] + 0.060), "unit_spare": on_pad}, 0.25, False),
        "red in a slot corner, 5 mm off both ways": ({"unit_red": unit(SLOT[0] + 0.005, SLOT[1] + 0.005, yaw=math.pi / 2), "unit_spare": unit(*BAY)}, 1.0, True),
        "spare seated upside down": ({"unit_red": unit(*SLOT, yaw=math.pi / 2), "unit_spare": unit(*BAY, tilt=math.pi)}, 1.0, True),
        "red out, standing upside down": ({"unit_red": unit(0.18, 0.12, tilt=math.pi), "unit_spare": on_pad}, 0.25, False),
        "red stacked on the seated spare": ({"unit_red": unit(*BAY, z=TOP + UNIT[2] * 1.5), "unit_spare": unit(*BAY)}, 0.0, False),
    }
    for name, (st, want, want_success) in cases.items():
        got, got_success = score(st), success(st)
        assert got == want and got_success == want_success, (name, got, got_success, want, want_success)
    return len(cases)


if __name__ == "__main__":
    # Offline checks, no GPU: the scoring ladder on hand-built states, then both arms' plans replayed through the
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
