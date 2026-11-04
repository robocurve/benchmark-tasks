"""On the station table, in the left arm's lane, a 1 g bench analogue of extending a spacecraft harness run: a
white harness board carries a black route line, and mounts added to a run continue the spacing already installed,
as a rivet-row repair keeps the existing pitch. Three grey standoffs are glued on the line at a 65 mm pitch,
starting next to a red start mark. Two identical loose standoffs stand on the table. Nothing marks where they go:
the pitch has to be read from the glued three and stepped on twice along the line, so the second station doubles
any pitch error. The score takes the loose standoffs on the line in order along the run, the nearer (or a lone one)
at the next station and the farther at the one after, each at full within 8 mm along the line and at half within
16 mm, from the standoff centres and quaternions; the bottom height comes from the centre and the axis.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/harness-standoff-pitch/harness_standoff_pitch.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "Stand the two loose standoffs on the black line so the run continues with the same spacing as the three glued standoffs." \\
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
# The run lies along y, the top camera's horizontal, and stays inboard of y 0.44: the left arm's ready pose hides
# the table beyond that from the top camera. The board runs 15 mm past the midline. Spreading the two standoffs
# evenly to its end in two gaps puts the nearer one 67 to 73 mm from S4, where the in-order rule judges it, and the
# farther 135 to 145 mm from S5; in three gaps both land more than 16 mm from both stations.
BOARD_X = (0.180, 0.300)
BOARD_Y = (-0.015, 0.435)
BOARD_T = 0.006
BOARD_TOP = TOP + BOARD_T
LINE_X = 0.240  # the route line's centre line, running along y the full length of the board
LINE_W = 0.010  # inlaid flush, so a standoff stands on one flat surface
MARK = (0.008, 0.050, 0.001)  # the red start mark across the line: along y, across, proud
MARK_Y = 0.415
PITCH = 0.065
GLUED_Y = (0.390, 0.325, 0.260)  # the installed run, from the start mark outward
STATIONS = (GLUED_Y[-1] - PITCH, GLUED_Y[-1] - 2 * PITCH)  # S4 at y 0.195 and S5 at y 0.130, unmarked
R, H = 0.010, 0.040  # every standoff: 20 mm diameter, 40 mm tall
STANDOFF_MASS = 0.015  # nylon, 1140 kg/m3: a 20 x 40 mm rod is 14.3 g
STARTS = ((0.120, 0.420), (0.120, 0.340))  # the loose pair, on the table beside the board's near edge

WHITE = (0.92, 0.92, 0.90)
BLACK = (0.05, 0.05, 0.05)
RED = (0.85, 0.12, 0.12)
GREY = (0.55, 0.55, 0.58)

_board_cy, _board_ly, _board_cz = sum(BOARD_Y) / 2, BOARD_Y[1] - BOARD_Y[0], TOP + BOARD_T / 2
_near_w = LINE_X - LINE_W / 2 - BOARD_X[0]
_far_w = BOARD_X[1] - (LINE_X + LINE_W / 2)
BOARD = [
    Box("board_near", pos=(BOARD_X[0] + _near_w / 2, _board_cy, _board_cz), size=(_near_w, _board_ly, BOARD_T), color=WHITE),
    Box("route_line", pos=(LINE_X, _board_cy, _board_cz), size=(LINE_W, _board_ly, BOARD_T), color=BLACK),
    Box("board_far", pos=(BOARD_X[1] - _far_w / 2, _board_cy, _board_cz), size=(_far_w, _board_ly, BOARD_T), color=WHITE),
    Box("start_mark", pos=(LINE_X, MARK_Y, BOARD_TOP + MARK[2] / 2), size=(MARK[1], MARK[0], MARK[2]), color=RED),
]
GLUED = [Cylinder(f"glued_{k}", pos=(LINE_X, y, BOARD_TOP + H / 2), radius=R, height=H, color=GREY) for k, y in enumerate(GLUED_Y)]
LOOSE = [Cylinder(f"standoff_{k}", pos=(x, y, TOP + H / 2), radius=R, height=H, mass=STANDOFF_MASS, color=GREY) for k, (x, y) in enumerate(STARTS)]
OBJECTS = [*LOOSE, *GLUED, *BOARD]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

TILT_TOL = math.radians(10.0)
BOTTOM_TOL = 0.003  # standoff bottom to the board top
LINE_TOL = 0.006  # standoff centre to the line's centre line, across the line
FULL_TOL = 0.008  # standoff centre to the station, along the line
HALF_TOL = 0.016
EPS = 1e-4  # 0.1 mm, so a value exactly on a limit passes


def _axis_z(state) -> float:
    x, y, _, _ = (float(v) for v in state.quat_xyzw)
    return 1.0 - 2.0 * (x * x + y * y)  # world z of the standoff's axis


def _bottom(state) -> float:
    """The standoff's lowest point from its centre and axis; the axis-aligned bounds of a tipped cylinder read low."""
    uz = _axis_z(state)
    return float(state.pos[2]) - (H / 2 * abs(uz) + R * math.sqrt(max(0.0, 1.0 - uz * uz)))


def _on_line(state) -> bool:
    """At rest, upright, its bottom on the board, centred on the line, and past the third glued standoff."""
    if not state.resting():
        return False
    if abs(_axis_z(state)) < math.cos(TILT_TOL) or abs(_bottom(state) - BOARD_TOP) > BOTTOM_TOL + EPS:
        return False
    return abs(float(state.pos[0]) - LINE_X) <= LINE_TOL + EPS and float(state.pos[1]) < GLUED_Y[-1]


def _band(y, station_y) -> int:
    """2 full, 1 half, 0 otherwise, along the line."""
    along = abs(y - station_y)
    if along <= FULL_TOL + EPS:
        return 2
    if along <= HALF_TOL + EPS:
        return 1
    return 0


def _levels(objects):
    """The standoffs on the line in order along the run: the nearer to the third glued standoff is the fourth of the
    run and is judged at S4, the farther is the fifth and is judged at S5. A lone one is the fourth of the run and is
    judged at S4: alone at S5 it cannot be told from one set at a doubled pitch or midway along the empty line.
    Judging by order keeps the score falling as the pitch error grows and stops a standoff set far down the line
    from claiming S5."""
    ys = sorted((float(objects[s.name].pos[1]) for s in LOOSE if _on_line(objects[s.name])), reverse=True)
    if len(ys) == 2:
        return [_band(ys[0], STATIONS[0]), _band(ys[1], STATIONS[1])]
    if len(ys) == 1:
        return [_band(ys[0], STATIONS[0]), 0]
    return [0, 0]


def success(objects) -> bool:
    """Both stations at full, every standoff at rest; holds for 1 s."""
    return all(level == 2 for level in _levels(objects))


def distance(objects) -> float:
    """Metres still to go: for each station, its nearest loose standoff's centre to the station, horizontally."""
    return sum(min(math.hypot(float(objects[s.name].pos[0]) - LINE_X, float(objects[s.name].pos[1]) - y) for s in LOOSE) for y in STATIONS)


def score(objects) -> float:
    """Per station, 0.5 at full (within 8 mm along the line) or 0.25 at half (within 16 mm), else 0, the standoffs
    taken in order along the run; summed: 0, 0.25, 0.5, 0.75 or 1.0."""
    return sum(level / 4.0 for level in _levels(objects))


TRAVEL = TOP + 0.145
GRIP_Z = TOP + 0.025  # TCP 25 mm above the standoff's bottom: pad tips 12 mm above the table, 28 mm of standoff in the grip
PLACE_Z = BOARD_TOP + 0.005 + 0.025  # the release: standoff bottom 5 mm above the board, pad tips 17 mm above it
ENTRY = 0.020  # the pause above a station: standoff bottom 25 mm above the board
PICK_YAW = 0.0  # fingers close along y at the pick; the other loose standoff is 70 mm away along y
# At the place the fingers close along x, across the line, so the neighbour 65 mm along y is beside the pads rather
# than in their path; -90 degrees, because +90 drives the wrist joint to its limit at these stations.
PLACE_YAW = -math.pi / 2
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
    """Left arm only, picking at yaw 0 and turning to -90 degrees on the carry. Standoff 0 to station S4, 65 mm
    past the third glued standoff, then standoff 1 to station S5, 130 mm past it; each a top-down pick, a carry at
    145 mm and a 5 mm drop onto the line. 32.4 s of plan, then the 3 s hold: 708 steps at 20 Hz."""
    left = []
    for (sx, sy), y in zip(STARTS, STATIONS):
        left += _pick_place((sx, sy, GRIP_Z), (LINE_X, y, PLACE_Z), width=2 * R, entry=ENTRY, travel_z=TRAVEL, yaw=PICK_YAW, place_yaw=PLACE_YAW)
    return {"left": left}


def self_test(ObjectState=None):
    """The scoring rules on hand-built states; runs with the real ObjectState or any stand-in with the same fields."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def standoff(x, y, bottom=BOARD_TOP, tilt=0.0, vel=(0.0, 0.0, 0.0)):
        """A standoff tilted ``tilt`` about the world x axis, its lowest point at ``bottom``; low/high from the
        rotated box corners, as the simulator reports them."""
        quat = np.array([math.sin(tilt / 2), 0.0, 0.0, math.cos(tilt / 2)])
        c, s = abs(math.cos(tilt)), abs(math.sin(tilt))
        half = np.array([R, R * c + H / 2 * s, H / 2 * c + R * s])
        pos = np.array([x, y, bottom + H / 2 * c + R * s])
        return ObjectState(pos, quat, np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    S4, S5 = STATIONS
    G3, END = GLUED_Y[-1], BOARD_Y[0] + R  # the third glued standoff; a standoff flush with the board end
    start = [standoff(x, y, bottom=TOP) for x, y in STARTS]

    def scene(a=None, b=None):
        return {LOOSE[0].name: a or start[0], LOOSE[1].name: b or start[1]}

    def on(y, dx=0.0, **kw):
        return standoff(LINE_X + dx, y, **kw)

    def pitch(p):
        return scene(on(G3 - p), on(G3 - 2 * p))

    cases = {
        "start": (scene(), 0.0, False),
        "done": (scene(on(S4), on(S5)), 1.0, True),
        "done with the standoffs swapped": (scene(on(S5), on(S4)), 1.0, True),
        "done within the full band": (scene(on(S4 + 0.006, dx=0.004), on(S5 - 0.007, dx=-0.005)), 1.0, True),
        "S4 only": (scene(on(S4)), 0.5, False),
        "S4 only, at half": (scene(on(S4 + 0.012)), 0.25, False),
        "S5 only, S4 skipped": (scene(b=on(S5)), 0.0, False),
        "a lone standoff at a 130 mm pitch": (scene(on(G3 - 0.130)), 0.0, False),
        "a lone standoff midway along the empty line": (scene(on((G3 + BOARD_Y[0]) / 2)), 0.0, False),
        "spread to the end in two gaps, the end one fallen off": (
            scene(on((G3 + BOARD_Y[0]) / 2), standoff(LINE_X, BOARD_Y[0] - 0.030, bottom=TOP, tilt=math.pi / 2)), 0.0, False),
        "S4 at half, S5 at full": (scene(on(S4 + 0.012), on(S5)), 0.75, False),
        "a 60 mm pitch: 5 and 10 mm off": (pitch(0.060), 0.75, False),
        "a 70 mm pitch: 5 and 10 mm off": (pitch(0.070), 0.75, False),
        "a 55 mm pitch: 10 and 20 mm off": (pitch(0.055), 0.25, False),
        "a 75 mm pitch: 10 and 20 mm off": (pitch(0.075), 0.25, False),
        "a 50 mm pitch: 15 and 30 mm off": (pitch(0.050), 0.25, False),
        "an 80 mm pitch: 15 and 30 mm off": (pitch(0.080), 0.25, False),
        "a 45 mm pitch: 20 and 40 mm off": (pitch(0.045), 0.0, False),
        "a 40 mm pitch: 25 and 50 mm off": (pitch(0.040), 0.0, False),
        "an 85 mm pitch: 20 and 40 mm off": (pitch(0.085), 0.0, False),
        "a 130 mm pitch, the second at the far end": (pitch(0.130), 0.0, False),
        "exactly 8.0 mm off is full": (scene(on(S4 - 0.008), on(S5)), 1.0, True),
        "8.5 mm off is half": (scene(on(S4 - 0.0085), on(S5)), 0.75, False),
        "exactly 16.0 mm off is half": (scene(on(S4 + 0.016), on(S5)), 0.75, False),
        "16.5 mm off earns nothing": (scene(on(S4 + 0.0165), on(S5)), 0.5, False),
        "set touching the glued run": (scene(on(G3 - 0.020), on(G3 - 0.040)), 0.0, False),
        "spread to the end: halfway and flush with the end": (scene(on((G3 + END) / 2), on(END)), 0.0, False),
        "spread to the end: halfway and centred on the end": (scene(on((G3 + END - R) / 2), on(END - R)), 0.0, False),
        "spread to the end in three equal gaps, flush": (scene(on(G3 - (G3 - END) / 3), on(G3 - 2 * (G3 - END) / 3)), 0.0, False),
        "spread to the end in three equal gaps, centred": (scene(on(G3 - (G3 - END + R) / 3), on(G3 - 2 * (G3 - END + R) / 3)), 0.0, False),
        "the run started one pitch late, at S5": (scene(on(S5), on(S5 - PITCH)), 0.0, False),
        "a second standoff crowding in before S4": (scene(on(S4 + 0.020), on(S4)), 0.0, False),
        "a second standoff just past S4": (scene(on(S4), on(S4 - 0.020)), 0.5, False),
        "one between the glued standoffs, one at S4": (scene(on((GLUED_Y[1] + G3) / 2), on(S4)), 0.5, False),
        "halfway between the stations": (scene(on((S4 + S5) / 2)), 0.0, False),
        "exactly 6.0 mm off the line counts": (scene(on(S4, dx=0.006), on(S5, dx=-0.006)), 1.0, True),
        "7 mm off the line at S5 earns nothing there": (scene(on(S4), on(S5, dx=0.007)), 0.5, False),
        "lying on its side at S5": (scene(on(S4), on(S5, tilt=math.pi / 2)), 0.5, False),
        "lying on its side at S4, so the one at S5 is alone": (scene(on(S4, tilt=math.pi / 2), on(S5)), 0.0, False),
        "tilted 8 degrees still counts": (scene(on(S4, tilt=math.radians(8)), on(S5)), 1.0, True),
        "tilted 12 degrees does not": (scene(on(S4), on(S5, tilt=math.radians(12))), 0.5, False),
        "an upside-down standoff counts": (scene(on(S4, tilt=math.pi), on(S5)), 1.0, True),
        "exactly 3.0 mm up counts": (scene(on(S4, bottom=BOARD_TOP + 0.003), on(S5)), 1.0, True),
        "held 10 mm over S5": (scene(on(S4), on(S5, bottom=BOARD_TOP + 0.010)), 0.5, False),
        "still moving at S5": (scene(on(S4), on(S5, vel=(0.0, 0.02, 0.0))), 0.5, False),
        "stacked on the one at S4": (scene(on(S4), on(S4, bottom=BOARD_TOP + H)), 0.5, False),
        "on the table beside the board, abeam S5": (scene(on(S4), standoff(0.160, S5, bottom=TOP)), 0.5, False),
        "on the board off the line, abeam S5": (scene(on(S4), standoff(0.275, S5)), 0.5, False),
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
