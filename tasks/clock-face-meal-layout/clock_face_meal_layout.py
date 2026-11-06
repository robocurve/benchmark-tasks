"""On the station table, in the left arm's lane, a 1 g tabletop analogue of setting out a meal for a crewmate who
cannot see, by the clock-face method used for people who are blind or have low vision: 12 o'clock is the far side of
the plate from the diner and 6 o'clock the side nearest them. A fixed stand-in for the diner, a slate blue torso
block with a beige head, sits at the left end of the table facing the plate, so the diner's clock is turned 90
degrees from the arms' view: 12 points away from the diner (toward the right of the camera image), 3 toward the
arms. A fixed white plate stands in front of the diner. An orange spoon case, a green food pouch and a blue cup stand
in a supply row nearer the arms, all outside the scoring band and each at least 60 degrees off its own hour. Each
item counts when it rests on the table 85 to 125 mm from the plate centre within 15 degrees of its hour, measured
clockwise from the diner's 12 o'clock.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/clock-face-meal-layout/clock_face_meal_layout.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "Lay out the meal for the crewmate seated at the left end, who cannot see. By the clock face as they sit, put the blue cup at 2, the green pouch at 7 and the orange spoon case at 11 o'clock, 10 cm from the plate centre." \\
           --scorer task_score ...
"""

import math

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. The left arm's base is at (0, +0.305, 0.75).
# The diner sits at the +y end of the table facing -y. In the diner's clock, 12 o'clock is -y (the far side of the
# plate from the diner), 3 is -x (the diner's right, toward the arms), 6 is +y and 9 is +x.
PLATE_C = (0.240, 0.320)
PLATE_R = 0.060
PLATE_H = 0.012

TORSO = (0.300, 0.120, 0.160)  # x, y, z: shoulders across x, so the diner faces along -y
TORSO_C = (0.240, 0.580)  # square behind the plate, so the diner's centre line runs through the plate centre along -y
HEAD_R = 0.050
HEAD_H = 0.100

CUP_R = 0.020
CUP_H = 0.070
CUP_MASS = 0.060
POUCH = (0.060, 0.040, 0.025)  # long axis along x at the start
POUCH_MASS = 0.060
CASE = (0.070, 0.028, 0.030)  # long axis along x at the start
CASE_MASS = 0.035

# Seen from the plate centre the case starts at 30.5 degrees (60.5 off its 11 o'clock), the pouch at 6.7 (156.7 off
# its 7) and the cup at 340.6 (79.4 off its 2), so sliding the row straight in, or pulling each item straight toward
# the plate, scores 0.
SUPPLY_Y = 0.150
CASE_START = (0.140, SUPPLY_Y)
POUCH_START = (0.220, SUPPLY_Y)
CUP_START = (0.300, SUPPLY_Y)

WHITE = (0.95, 0.95, 0.95)
SLATE = (0.30, 0.35, 0.50)
BEIGE = (0.82, 0.70, 0.58)
BLUE = (0.20, 0.45, 0.85)
GREEN = (0.30, 0.65, 0.30)
ORANGE = (0.95, 0.55, 0.10)

HOURS = {"cup": 2, "pouch": 7, "spoon_case": 11}  # the instruction's hours, in the diner's clock
RADIUS = {"cup": 0.100, "pouch": 0.100, "spoon_case": 0.110}  # the demo's radii; the case's inner end stays 15 mm off the rim


def _hour_dir(hour):
    """Unit vector from the plate centre toward ``hour`` on the diner's clock: 12 is -y, and the hours run clockwise
    seen from above, so 3 is -x."""
    a = math.radians(30.0 * hour)
    return -math.sin(a), -math.cos(a)


def _target(name):
    dx, dy = _hour_dir(HOURS[name])
    return PLATE_C[0] + dx * RADIUS[name], PLATE_C[1] + dy * RADIUS[name]


CUP = Cylinder("cup", pos=(*CUP_START, TOP + CUP_H / 2), radius=CUP_R, height=CUP_H, mass=CUP_MASS, color=BLUE)
POUCH_BOX = Box("pouch", pos=(*POUCH_START, TOP + POUCH[2] / 2), size=POUCH, mass=POUCH_MASS, color=GREEN)
CASE_BOX = Box("spoon_case", pos=(*CASE_START, TOP + CASE[2] / 2), size=CASE, mass=CASE_MASS, color=ORANGE)
PLATE = Cylinder("plate", pos=(*PLATE_C, TOP + PLATE_H / 2), radius=PLATE_R, height=PLATE_H, color=WHITE)
DINER_TORSO = Box("diner_torso", pos=(*TORSO_C, TOP + TORSO[2] / 2), size=TORSO, color=SLATE)
DINER_HEAD = Cylinder("diner_head", pos=(*TORSO_C, TOP + TORSO[2] + HEAD_H / 2), radius=HEAD_R, height=HEAD_H, color=BEIGE)
OBJECTS = [CUP, POUCH_BOX, CASE_BOX, PLATE, DINER_TORSO, DINER_HEAD]

ITEMS = {"cup": ("cylinder", (CUP_R, CUP_H)), "pouch": ("box", POUCH), "spoon_case": ("box", CASE)}

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

R_MIN, R_MAX = 0.085, 0.125  # centre to plate centre: the instruction's 10 cm, -15 mm (the plate rim) to +25 mm
BEARING_TOL = 15.0  # degrees either side of the hour; hours are 30 degrees apart
TILT_MAX = math.radians(8.0)  # under the 9.9 degrees of a 70 mm case with one end up on the 12 mm plate rim
BOTTOM_TOL = 0.003  # the lowest point to the table: under the demo's 5 mm release height, so a part the demo still holds does not count
EPS = 1e-4  # 0.1 mm, so a value exactly on a limit passes


def _rot(quat_xyzw):
    x, y, z, w = (float(v) for v in quat_xyzw)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def _bottom_and_level(name, state):
    """The item's lowest point and whether it sits level. The cup counts only standing on an end (its axis within 8
    degrees of vertical, either way up); the pouch and the case may lie on any face (some body axis within 8 degrees
    of vertical). The lowest point comes from the centre and the rotation, not the axis-aligned bounds."""
    rot = _rot(state.quat_xyzw)
    kind, dims = ITEMS[name]
    z = float(state.pos[2])
    if kind == "cylinder":
        r, h = dims
        uz = float(rot[2, 2])
        bottom = z - (h / 2 * abs(uz) + r * math.sqrt(max(0.0, 1.0 - uz * uz)))
        return bottom, abs(uz) >= math.cos(TILT_MAX) - 1e-9
    half = np.asarray(dims, float) / 2
    bottom = z - float(np.abs(rot[2, :]) @ half)
    return bottom, float(np.max(np.abs(rot[2, :]))) >= math.cos(TILT_MAX) - 1e-9


def _bearing(x, y):
    """Degrees clockwise seen from above from the diner's 12 o'clock (-y), 0 to 360."""
    return math.degrees(math.atan2(-(x - PLATE_C[0]), -(y - PLATE_C[1]))) % 360.0


def _placed(name, state) -> bool:
    if not state.resting():
        return False
    bottom, level = _bottom_and_level(name, state)
    if not level or abs(bottom - TOP) > BOTTOM_TOL + EPS:
        return False
    x, y = float(state.pos[0]), float(state.pos[1])
    r = math.hypot(x - PLATE_C[0], y - PLATE_C[1])
    if r < R_MIN - EPS or r > R_MAX + EPS:
        return False
    off = abs((_bearing(x, y) - 30.0 * HOURS[name] + 180.0) % 360.0 - 180.0)
    return off <= BEARING_TOL + 1e-6


def score(objects) -> float:
    """1/3 per item at rest on the table at its hour: level (the cup on an end, the pouch and case on any face, tilt
    at most 8 degrees), its lowest point within 3 mm of the table, its centre 85 to 125 mm from the plate centre and
    its bearing within 15 degrees of its hour; 0, 1/3, 2/3 or 1.0."""
    return sum(1 for name in ITEMS if _placed(name, objects[name])) / 3.0


def success(objects) -> bool:
    """All three items at their hours, at rest; holds for 1 s."""
    return all(_placed(name, objects[name]) for name in ITEMS)


def distance(objects) -> float:
    """Metres still to go: each item's centre to its demo target, horizontally, summed."""
    total = 0.0
    for name in ITEMS:
        tx, ty = _target(name)
        total += math.hypot(float(objects[name].pos[0]) - tx, float(objects[name].pos[1]) - ty)
    return total


TRAVEL = TOP + 0.145
CUP_GRIP_Z = TOP + 0.035  # TCP at the cup's mid-height, pad tips 22 mm above the table
POUCH_GRIP_Z = TOP + 0.019  # pad tips 6 mm above the table, 19 mm of the 25 mm pouch between the pads
CASE_GRIP_Z = TOP + 0.022  # pad tips 9 mm above the table, 21 mm of the 30 mm case between the pads
DROP = 0.005  # each item is released with its bottom 5 mm above the table
ENTRY = 0.015  # the pause above each place: the item's bottom 20 mm above the table
POUCH_YAW = math.radians(60.0)  # the pouch's 60 mm side along the 7 o'clock radius
CASE_YAW = math.radians(-60.0)  # the case's long axis along the 11 o'clock radius
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
    """Left arm only. Three top-down pick and place moves from the supply row, each pick at yaw 0 with the pads
    closing along y: the cup from the row's far end to 2 o'clock, the pouch to 7 o'clock turned 60 degrees so its
    60 mm side lies along the radius, and the spoon case from the row's near end to 11 o'clock turned -60 degrees so
    its long axis lies along the radius, each carried at 14.5 cm and released 5 mm above the table. 48.6 s of plan,
    then the 3 s hold: 1,032 steps at 20 Hz."""
    moves = (
        ("cup", CUP_START, CUP_GRIP_Z, 2 * CUP_R, 0.0),
        ("pouch", POUCH_START, POUCH_GRIP_Z, POUCH[1], POUCH_YAW),
        ("spoon_case", CASE_START, CASE_GRIP_Z, CASE[1], CASE_YAW),
    )
    left = []
    for name, (sx, sy), grip_z, width, place_yaw in moves:
        tx, ty = _target(name)
        left += _pick_place((sx, sy, grip_z), (tx, ty, grip_z + DROP), width=width, entry=ENTRY, travel_z=TRAVEL, yaw=0.0, place_yaw=place_yaw)
    return {"left": left}


def self_test(ObjectState=None):
    """The scoring rules on hand-built states; runs with the real ObjectState or any stand-in with the same fields."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def qmul(a, b):
        ax, ay, az, aw = a
        bx, by, bz, bw = b
        return np.array([aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
                         aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz])

    def quat(yaw=0.0, tilt=0.0, axis="x"):  # tilt about the body's x or y axis, then yaw about the vertical
        s = math.sin(tilt / 2)
        qt = np.array([s, 0.0, 0.0, math.cos(tilt / 2)]) if axis == "x" else np.array([0.0, s, 0.0, math.cos(tilt / 2)])
        return qmul(np.array([0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)]), qt)

    def item(name, x, y, yaw=0.0, tilt=0.0, axis="x", bottom=TOP, vel=(0.0, 0.0, 0.0)):
        """``name`` with its lowest point at ``bottom``; low/high from the rotated box corners, as the simulator
        reports them."""
        q = quat(yaw, tilt, axis)
        rot = _rot(q)
        kind, dims = ITEMS[name]
        if kind == "cylinder":
            r, h = dims
            half = np.abs(rot) @ np.array([r, r, h / 2])
            uz = float(rot[2, 2])
            up = h / 2 * abs(uz) + r * math.sqrt(max(0.0, 1.0 - uz * uz))
        else:
            half = np.abs(rot) @ (np.asarray(dims, float) / 2)
            up = float(half[2])
        pos = np.array([x, y, bottom + up])
        return ObjectState(pos, q, np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    def at(name, hour=None, r=None, dbear=0.0, **kw):
        """``name`` at ``hour`` (its own by default) on the diner's clock, ``r`` from the plate centre, turned so the
        boxes' long axes lie along the radius."""
        h = HOURS[name] if hour is None else hour
        a = math.radians(30.0 * h + dbear)
        dx, dy = -math.sin(a), -math.cos(a)
        rr = RADIUS[name] if r is None else r
        kw.setdefault("yaw", math.atan2(dy, dx))
        return item(name, PLATE_C[0] + dx * rr, PLATE_C[1] + dy * rr, **kw)

    start = {"cup": item("cup", *CUP_START), "pouch": item("pouch", *POUCH_START), "spoon_case": item("spoon_case", *CASE_START)}

    def scene(**placed):
        states = dict(start)
        states.update(placed)
        return states

    def hours(c, p, s):
        return scene(cup=at("cup", c), pouch=at("pouch", p), spoon_case=at("spoon_case", s))

    done = dict(cup=at("cup"), pouch=at("pouch"), spoon_case=at("spoon_case"))

    def done_but(**changed):
        return scene(**{**done, **changed})

    lying = math.pi / 2
    plate_top = TOP + PLATE_H
    starts = {"cup": CUP_START, "pouch": POUCH_START, "spoon_case": CASE_START}

    def pushed(dx, dy):  # the whole supply row slid as one, untouched
        return {n: item(n, x + dx, y + dy) for n, (x, y) in starts.items()}

    def pulled_in(r):  # each item moved straight toward the plate centre until it is ``r`` out
        out = {}
        for n, (x, y) in starts.items():
            d = math.hypot(x - PLATE_C[0], y - PLATE_C[1])
            out[n] = item(n, PLATE_C[0] + (x - PLATE_C[0]) * r / d, PLATE_C[1] + (y - PLATE_C[1]) * r / d)
        return out

    rim_lean = math.asin(PLATE_H / CASE[0])  # 9.9 degrees: the case's inner end up on the plate rim, its outer end down
    cases = {
        "start": (scene(), 0.0, False),
        "done": (scene(**done), 1.0, True),
        "cup only": (scene(cup=at("cup")), 1 / 3, False),
        "pouch only": (scene(pouch=at("pouch")), 1 / 3, False),
        "spoon case only": (scene(spoon_case=at("spoon_case")), 1 / 3, False),
        "cup and spoon case": (scene(cup=at("cup"), spoon_case=at("spoon_case")), 2 / 3, False),
        "the robot's frame, 12 forward (diner's 11, 4, 8)": (hours(11, 4, 8), 0.0, False),
        "a diner across from the arms (diner's 5, 10, 2)": (hours(5, 10, 2), 0.0, False),
        "a diner at the other end (diner's 8, 1, 5)": (hours(8, 1, 5), 0.0, False),
        "counterclockwise in the diner's frame (10, 5, 1)": (hours(10, 5, 1), 0.0, False),
        "items swapped: cup at 7, pouch at 2": (scene(cup=at("cup", 7), pouch=at("pouch", 2), spoon_case=at("spoon_case")), 1 / 3, False),
        "a conventional setting for the arms' seat (cup at 10:30, case at 12, pouch at 6)": (hours(10.5, 6, 12), 0.0, False),
        "a conventional setting for the diner (cup at 2, case at 3, pouch at 9)": (hours(2, 9, 3), 1 / 3, False),
        "every item at 10 cm, the case tangential": (done_but(spoon_case=at("spoon_case", r=0.100, yaw=math.radians(30))), 1.0, True),
        "14 degrees off its hour counts": (done_but(cup=at("cup", dbear=14.0)), 1.0, True),
        "16 degrees off its hour does not": (done_but(cup=at("cup", dbear=-16.0)), 2 / 3, False),
        "85 mm out counts (the pouch tangential, its inner edge 5 mm off the rim)": (done_but(pouch=at("pouch", r=0.085, yaw=math.radians(150))), 1.0, True),
        "the pouch radial at 85 mm, one end up on the plate rim": (done_but(pouch=at("pouch", r=0.085, tilt=math.asin(PLATE_H / POUCH[0]), axis="y")), 2 / 3, False),
        "82 mm out does not": (done_but(cup=at("cup", r=0.082)), 2 / 3, False),
        "125 mm out counts": (done_but(spoon_case=at("spoon_case", r=0.125)), 1.0, True),
        "128 mm out does not": (done_but(spoon_case=at("spoon_case", r=0.128)), 2 / 3, False),
        "the cup lying on its side at 2": (done_but(cup=at("cup", tilt=lying)), 2 / 3, False),
        "the cup tilted 6 degrees counts": (done_but(cup=at("cup", tilt=math.radians(6))), 1.0, True),
        "the cup tilted 10 degrees does not": (done_but(cup=at("cup", tilt=math.radians(10))), 2 / 3, False),
        "the cup upside down counts": (done_but(cup=at("cup", tilt=math.pi)), 1.0, True),
        "the pouch on its edge counts": (done_but(pouch=at("pouch", tilt=lying)), 1.0, True),
        "the pouch standing on its end counts": (done_but(pouch=at("pouch", tilt=lying, axis="y")), 1.0, True),
        "the pouch leaning 15 degrees does not": (done_but(pouch=at("pouch", tilt=math.radians(15))), 2 / 3, False),
        "the case upside down counts": (done_but(spoon_case=at("spoon_case", tilt=math.pi)), 1.0, True),
        "the case resting on the plate rim at 6 mm": (done_but(spoon_case=at("spoon_case", bottom=TOP + 0.006)), 2 / 3, False),
        "the case leaning on the plate rim, one end on the table": (done_but(spoon_case=at("spoon_case", r=0.097, tilt=rim_lean, axis="y")), 2 / 3, False),
        "the cup held still at the 5 mm release height": (done_but(cup=at("cup", bottom=TOP + DROP)), 2 / 3, False),
        "the cup 3 mm up counts": (done_but(cup=at("cup", bottom=TOP + 0.003)), 1.0, True),
        "the cup on the plate": (done_but(cup=item("cup", 0.215, 0.305, bottom=plate_top)), 2 / 3, False),
        "the pouch at 7 stacked on the spoon case": (scene(cup=at("cup"), spoon_case=at("spoon_case", 7), pouch=at("pouch", bottom=TOP + CASE[2])), 1 / 3, False),
        "the cup still moving at 2": (done_but(cup=at("cup", vel=(0.0, 0.02, 0.0))), 2 / 3, False),
        "the case held 20 mm over 11": (done_but(spoon_case=at("spoon_case", bottom=TOP + 0.020)), 2 / 3, False),
        "everything knocked over where it stood": (scene(cup=item("cup", *CUP_START, tilt=lying), pouch=item("pouch", *POUCH_START, tilt=lying), spoon_case=item("spoon_case", *CASE_START, tilt=lying)), 0.0, False),
        "all three piled by the plate at 3": (hours(3, 3, 3), 0.0, False),
        "the row slid 90 mm straight toward the plate and the diner": (pushed(0.0, 0.090), 0.0, False),
        "the row slid 170 mm aimed to bring the case to 11: the case only": (pushed(0.170 * math.cos(math.radians(28)), 0.170 * math.sin(math.radians(28))), 1 / 3, False),
        "each item pulled straight in to 10 cm": (pulled_in(0.100), 0.0, False),
    }
    for name, (st, want, want_success) in cases.items():
        got, got_success = score(st), success(st)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
        assert not got_success or got == 1.0, name
    assert distance(scene(**done)) < 1e-9 and distance(scene()) > 0.0
    for name, (x, y) in (("cup", CUP_START), ("pouch", POUCH_START), ("spoon_case", CASE_START)):
        assert math.hypot(x - PLATE_C[0], y - PLATE_C[1]) > R_MAX + 0.02, name  # the supply row lies outside the band
        off = abs((_bearing(x, y) - 30.0 * HOURS[name] + 180.0) % 360.0 - 180.0)
        assert off >= 60.0, (name, off)  # and every item starts well off its own hour
    for ang in range(0, 360, 2):  # a straight slide of the whole row, any direction, up to 300 mm, brings at most one
        for mm in range(0, 301, 4):  # item to its hour, and none while it heads within 30 degrees of the diner (+y)
            a = math.radians(ang)  # and stays under 200 mm, which already carries the row past the plate centre
            got = score(pushed(mm / 1000 * math.cos(a), mm / 1000 * math.sin(a)))
            assert got < 0.5 and (got == 0.0 or not (60 <= ang <= 120 and mm <= 200)), (ang, mm, got)
    for r in np.arange(R_MIN, R_MAX + 1e-9, 0.0025):  # pulling each item straight in to any radius in the band earns 0
        assert score(pulled_in(float(r))) == 0.0, r
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
