"""On the station table, in the left arm's lane, a 1 g tabletop terrain model of siting a surface radio relay: a
grey plateau carries a blue habitat, an orange rover and three dark ridges taller than everything else, and a white
relay beacon waits on the table beside it. Surface radio links need direct line of sight, so the beacon has to stand
where the straight line from its centre to each site's centre clears every ridge. One pocket of the plateau sees both
sites; the plateau centre, the midpoint between the sites and the plateau edge nearest the start each see one site or
none. The score uses the beacon centre and its quaternion for every horizontal and orientation test and computes its
lowest point from the centre and the axis, so a beacon on its side reads correctly.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/relay-line-of-sight/relay_line_of_sight.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "Stand the white relay beacon upright on the grey plateau where it has a clear straight line of sight from its centre to the centre of the blue habitat and to the centre of the orange rover. The dark ridges block sight." \\
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
PLATEAU_C = (0.240, 0.255)
PLATEAU = (0.220, 0.270, 0.012)  # spans x 0.130 to 0.350 and y 0.120 to 0.390
PLAT_TOP = TOP + PLATEAU[2]
PLAT_X = (PLATEAU_C[0] - PLATEAU[0] / 2, PLATEAU_C[0] + PLATEAU[0] / 2)
PLAT_Y = (PLATEAU_C[1] - PLATEAU[1] / 2, PLATEAU_C[1] + PLATEAU[1] / 2)
HABITAT_C = (0.325, 0.365)
HABITAT = (0.040, 0.040, 0.040)
ROVER_C = (0.325, 0.145)
ROVER = (0.040, 0.040, 0.030)
SITES = (("habitat", HABITAT_C), ("rover", ROVER_C))
RIDGE = (0.080, 0.012, 0.070)  # long axis along the ridge's yaw
RIDGES = (  # name, centre, yaw
    ("ridge_1", (0.268, 0.177), math.radians(120.0)),
    ("ridge_2", (0.308, 0.295), 0.0),
    ("ridge_3", (0.197, 0.296), math.radians(135.0)),
)
BEACON_R = 0.015
BEACON_H = 0.045
BEACON_MASS = 0.040
START = (0.170, 0.075)  # on the table, 30 mm from the plateau's inboard edge (y 0.120) to the beacon's edge
SPOT = (0.205, 0.365)  # the demo's spot, inside the pocket that sees both sites

GREY = (0.60, 0.60, 0.62)
CHARCOAL = (0.18, 0.18, 0.18)
BLUE = (0.45, 0.65, 0.90)
ORANGE = (0.95, 0.55, 0.10)
WHITE = (0.95, 0.95, 0.95)

BEACON = Cylinder("beacon", pos=(START[0], START[1], TOP + BEACON_H / 2), radius=BEACON_R, height=BEACON_H, mass=BEACON_MASS, color=WHITE)
FIXTURES = [
    Box("plateau", pos=(PLATEAU_C[0], PLATEAU_C[1], TOP + PLATEAU[2] / 2), size=PLATEAU, color=GREY),
    Box("habitat", pos=(HABITAT_C[0], HABITAT_C[1], PLAT_TOP + HABITAT[2] / 2), size=HABITAT, color=BLUE),
    Box("rover", pos=(ROVER_C[0], ROVER_C[1], PLAT_TOP + ROVER[2] / 2), size=ROVER, color=ORANGE),
    *[Box(name, pos=(c[0], c[1], PLAT_TOP + RIDGE[2] / 2), size=RIDGE, color=CHARCOAL, yaw=yaw) for name, c, yaw in RIDGES],
]
OBJECTS = [BEACON, *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

UPRIGHT = math.radians(5.0)  # beacon axis to vertical
BOTTOM_TOL = 0.004  # beacon's lowest point to the plateau top
SIGHT_MARGIN = 0.004  # the sight line must pass at least this far from every ridge footprint
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


def _axis_z(state) -> float:
    """World z of the beacon's axis; +-1 upright either end up, 0 lying."""
    return float(_rot(state.quat_xyzw)[2, 2])


def _bottom(state) -> float:
    """The beacon's lowest point from its centre and axis. The axis-aligned bounds of a lying cylinder come from its
    box corners and read too low, so they are not used."""
    uz = _axis_z(state)
    return float(state.pos[2]) - (BEACON_H / 2 * abs(uz) + BEACON_R * math.sqrt(max(0.0, 1.0 - uz * uz)))


def _seg_rect_dist(p, q, centre, yaw, half) -> float:
    """Metres between the segment p-q and a rectangle (centre, yaw, half sizes) in the plane; 0 if they cross."""
    c, s = math.cos(yaw), math.sin(yaw)

    def local(v):
        dx, dy = v[0] - centre[0], v[1] - centre[1]
        return (dx * c + dy * s, -dx * s + dy * c)

    (ax, ay), (bx, by) = local(p), local(q)
    hx, hy = half
    t0, t1, dx, dy = 0.0, 1.0, bx - ax, by - ay
    crosses = True
    for pk, qk in ((-dx, ax + hx), (dx, hx - ax), (-dy, ay + hy), (dy, hy - ay)):  # Liang-Barsky clip
        if abs(pk) < 1e-15:
            if qk < 0:
                crosses = False
                break
            continue
        t = qk / pk
        if pk < 0:
            t0 = max(t0, t)
        else:
            t1 = min(t1, t)
        if t0 > t1:
            crosses = False
            break
    if crosses:
        return 0.0

    def to_rect(x, y):
        return math.hypot(max(abs(x) - hx, 0.0), max(abs(y) - hy, 0.0))

    def to_seg(x, y):
        ll = dx * dx + dy * dy
        t = 0.0 if ll == 0 else max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / ll))
        return math.hypot(x - (ax + t * dx), y - (ay + t * dy))

    corners = [to_seg(sx * hx, sy * hy) for sx in (-1, 1) for sy in (-1, 1)]
    return min(to_rect(ax, ay), to_rect(bx, by), *corners)


def _sight_gap(xy, site) -> float:
    """Metres between the straight line from ``xy`` to the site's centre and the nearest ridge footprint."""
    return min(_seg_rect_dist(xy, site, c, yaw, (RIDGE[0] / 2, RIDGE[1] / 2)) for _, c, yaw in RIDGES)


def _clear_links(xy) -> int:
    """How many of the two sites the point sees: lines that pass at least 4 mm from every ridge."""
    return sum(_sight_gap(xy, site) >= SIGHT_MARGIN - EPS for _, site in SITES)


def _standing(state) -> bool:
    """Upright either end up, at rest, its lowest point on the plateau top and its centre inside the plateau outline."""
    x, y = float(state.pos[0]), float(state.pos[1])
    inside = PLAT_X[0] - EPS <= x <= PLAT_X[1] + EPS and PLAT_Y[0] - EPS <= y <= PLAT_Y[1] + EPS
    upright = abs(_axis_z(state)) >= math.cos(UPRIGHT)
    return bool(state.resting() and upright and inside and abs(_bottom(state) - PLAT_TOP) <= BOTTOM_TOL + EPS)


def _links(objects) -> int:
    beacon = objects["beacon"]
    if not _standing(beacon):
        return 0
    return _clear_links((float(beacon.pos[0]), float(beacon.pos[1])))


def success(objects) -> bool:
    """The beacon standing on the plateau with clear lines to both sites, at rest; holds for 1 s."""
    return _links(objects) == 2


def distance(objects) -> float:
    """Metres from the beacon's centre to the demo's spot in the pocket that sees both sites."""
    b = objects["beacon"]
    return math.hypot(float(b.pos[0]) - SPOT[0], float(b.pos[1]) - SPOT[1])


def score(objects) -> float:
    """The beacon standing upright on the plateau (tilt under 5 degrees, at rest, lowest point within 4 mm of the
    plateau top, centre inside the plateau outline): 1.0 with clear lines to both sites, 0.25 to exactly one, 0 to
    none. A line runs from the beacon's centre to the site's centre and is clear when it passes at least 4 mm from
    every ridge footprint, in plan. A beacon not standing on the plateau scores 0."""
    return {2: 1.0, 1: 0.25}.get(_links(objects), 0.0)


TRAVEL = TOP + 0.145  # beacon bottom 110 mm above the table, 28 mm over the 82 mm ridge tops
GRIP_UP = 0.035  # the TCP 35 mm above the beacon's bottom: pad tips 22 mm up its side, 23 mm of it between the pads
GRIP_Z = TOP + GRIP_UP
PLACE_Z = PLAT_TOP + 0.005 + GRIP_UP  # released 5 mm above the plateau top
ENTRY = 0.015
PICK_YAW = -math.pi / 2  # fingers close along x at the pick, away from the plateau edge
PLACE_YAW = math.pi / 4  # fingers close along ridge 3's long axis, so they open parallel to its face
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
    """Left arm only, one top-down pick and place: grip the beacon across its 30 mm diameter with the fingers closing
    along x, carry it at 145 mm with its bottom 28 mm over the ridge tops while turning the grip to 45 degrees, so the
    fingers open parallel to ridge 3, pause 15 mm above the spot, set it down slowly and release it 5 mm above the
    plateau. 16.2 s of plan, then the 3 s hold: 384 steps at 20 Hz."""
    return {"left": _pick_place((START[0], START[1], GRIP_Z), (SPOT[0], SPOT[1], PLACE_Z), width=2 * BEACON_R, entry=ENTRY, travel_z=TRAVEL, yaw=PICK_YAW, place_yaw=PLACE_YAW)}


def _grid_census(step=0.005):
    """Beacon centres on a grid over the plateau whose footprint stands clear of every ridge and site, sorted by how
    many sites they see: {0: n, 1: n, 2: n} and the cells that see both."""
    def clear_of(xy, centre, yaw, half):
        c, s = math.cos(yaw), math.sin(yaw)
        dx, dy = xy[0] - centre[0], xy[1] - centre[1]
        u, v = dx * c + dy * s, -dx * s + dy * c
        return math.hypot(max(abs(u) - half[0], 0.0), max(abs(v) - half[1], 0.0)) >= BEACON_R

    blocks = [(c, yaw, (RIDGE[0] / 2, RIDGE[1] / 2)) for _, c, yaw in RIDGES]
    blocks += [(HABITAT_C, 0.0, (HABITAT[0] / 2, HABITAT[1] / 2)), (ROVER_C, 0.0, (ROVER[0] / 2, ROVER[1] / 2))]
    counts, both = {0: 0, 1: 0, 2: 0}, []
    nx, ny = round(PLATEAU[0] / step), round(PLATEAU[1] / step)
    for i in range(nx + 1):
        for j in range(ny + 1):
            xy = (PLAT_X[0] + i * step, PLAT_Y[0] + j * step)
            if all(clear_of(xy, *b) for b in blocks):
                n = _clear_links(xy)
                counts[n] += 1
                if n == 2:
                    both.append(xy)
    return counts, both


def self_test(ObjectState=None):
    """The scoring rules on hand-built states and the layout's claims; runs with the real ObjectState or any stand-in
    with the same fields."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def quat(tilt=0.0, about=0.0):  # tilt the axis by ``tilt`` about a horizontal axis at angle ``about`` from x
        s = math.sin(tilt / 2)
        return np.array([math.cos(about) * s, math.sin(about) * s, 0.0, math.cos(tilt / 2)])

    def beacon(x, y, surface=PLAT_TOP, tilt=0.0, vel=(0.0, 0.0, 0.0)):
        q = quat(tilt)
        half = np.abs(_rot(q)) @ np.array([BEACON_R, BEACON_R, BEACON_H / 2])
        uz = math.cos(tilt)
        pos = np.array([x, y, surface + BEACON_H / 2 * abs(uz) + BEACON_R * abs(math.sin(tilt))])
        return ObjectState(pos, q, np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    def scene(b):
        return {"beacon": b}

    centre, midpoint = PLATEAU_C, (0.325, 0.255)
    near_start = (START[0], PLAT_Y[0] + BEACON_R + 0.005)  # the plateau spot nearest the start, footprint fully on it
    sees_habitat_only = (0.280, 0.370)
    ridge_2_top = PLAT_TOP + RIDGE[2]
    cases = {
        "start, on the table": (scene(beacon(*START, surface=TOP)), 0.0, False),
        "the demo spot": (scene(beacon(*SPOT)), 1.0, True),
        "the demo spot, upside down": (scene(beacon(*SPOT, tilt=math.pi)), 1.0, True),
        "the demo spot at 4 degrees of tilt": (scene(beacon(*SPOT, tilt=math.radians(4))), 1.0, True),
        "the demo spot at 8 degrees of tilt": (scene(beacon(*SPOT, tilt=math.radians(8))), 0.0, False),
        "lying on its side at the demo spot": (scene(beacon(*SPOT, tilt=math.pi / 2)), 0.0, False),
        "held 10 mm above the demo spot": (scene(beacon(*SPOT, surface=PLAT_TOP + 0.010)), 0.0, False),
        "the demo spot, still moving": (scene(beacon(*SPOT, vel=(0.02, 0.0, 0.0))), 0.0, False),
        "another point in the pocket": (scene(beacon(0.220, 0.340)), 1.0, True),
        "the plateau centre sees only the rover": (scene(beacon(*centre)), 0.25, False),
        "the midpoint between the sites sees only the rover": (scene(beacon(*midpoint)), 0.25, False),
        "a point beside the habitat sees only the habitat": (scene(beacon(*sees_habitat_only)), 0.25, False),
        "the plateau edge nearest the start sees neither": (scene(beacon(*near_start)), 0.0, False),
        "on the table past the plateau's left edge": (scene(beacon(SPOT[0], PLAT_Y[1] + 0.030, surface=TOP)), 0.0, False),
        "centre 1 mm outside the plateau outline": (scene(beacon(SPOT[0], PLAT_Y[1] + 0.001)), 0.0, False),
        "standing on ridge 2's top": (scene(beacon(*RIDGES[1][1], surface=ridge_2_top)), 0.0, False),
        "standing on the habitat's top": (scene(beacon(*HABITAT_C, surface=PLAT_TOP + HABITAT[2])), 0.0, False),
        "standing on the rover's top": (scene(beacon(*ROVER_C, surface=PLAT_TOP + ROVER[2])), 0.0, False),
        "the plateau centre, still moving": (scene(beacon(*centre, vel=(0.0, 0.02, 0.0))), 0.0, False),
        "lying on the table at the start": (scene(beacon(*START, surface=TOP, tilt=math.pi / 2)), 0.0, False),
    }
    for name, (st, want, want_success) in cases.items():
        got, got_success = score(st), success(st)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
        assert not got_success or got == 1.0, name

    # the segment test on a unit case: a 2 x 2 square at the origin
    sq = ((0.0, 0.0), 0.0, (1.0, 1.0))
    assert _seg_rect_dist((-3, 0), (3, 0), *sq) == 0.0
    assert abs(_seg_rect_dist((-3, 1.5), (3, 1.5), *sq) - 0.5) < 1e-12
    assert abs(_seg_rect_dist((2, 2), (3, 5), *sq) - math.sqrt(2)) < 1e-12
    assert abs(_seg_rect_dist((0, 3), (3, 0), *sq) - (3 / math.sqrt(2) - math.sqrt(2))) < 1e-12
    assert _seg_rect_dist((0.5, 0.5), (0.6, 0.6), *sq) == 0.0
    # layout claims
    tops = {"beacon on the plateau": PLAT_TOP + BEACON_H, "habitat": PLAT_TOP + HABITAT[2], "rover": PLAT_TOP + ROVER[2]}
    assert all(ridge_2_top - t >= 0.025 for t in tops.values()), tops  # ridges at least 25 mm above every other top
    for name, c, yaw in RIDGES:  # every ridge on the plateau, away from both sites
        ends = [(c[0] + sx * RIDGE[0] / 2 * math.cos(yaw) - sy * RIDGE[1] / 2 * math.sin(yaw), c[1] + sx * RIDGE[0] / 2 * math.sin(yaw) + sy * RIDGE[1] / 2 * math.cos(yaw)) for sx in (-1, 1) for sy in (-1, 1)]
        assert all(PLAT_X[0] < x < PLAT_X[1] and PLAT_Y[0] < y < PLAT_Y[1] for x, y in ends), name
    face = min(_seg_rect_dist(SPOT, SPOT, c, yaw, (RIDGE[0] / 2, RIDGE[1] / 2)) for _, c, yaw in RIDGES)
    assert face >= 0.045, face  # 48 mm from the spot to the nearest ridge face, ridge 3
    assert min(_sight_gap(SPOT, s) for _, s in SITES) >= 0.015  # both lines clear by at least 15 mm
    assert PLAT_Y[0] - (START[1] + BEACON_R) >= 0.030 - EPS  # the start is 30 mm off the plateau edge
    counts, both = _grid_census()
    total = sum(counts.values())
    assert 0.10 <= counts[2] / total <= 0.17 and 0.36 <= counts[1] / total <= 0.46, counts
    assert 0.22 <= (counts[2] + 0.25 * counts[1]) / total <= 0.26, counts  # a blind upright placement averages about 0.24
    # away from the back edge, the cells that see both form one connected pocket near the plateau's outboard edge
    cells = {(round((x - PLAT_X[0]) / 0.005), round((y - PLAT_Y[0]) / 0.005)) for x, y in both if x > PLAT_X[0] + 0.015}
    seen, todo = set(), [next(iter(cells))]
    while todo:
        i, j = todo.pop()
        if (i, j) in seen or (i, j) not in cells:
            continue
        seen.add((i, j))
        todo += [(i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)]
    assert seen == cells and len(cells) >= 0.95 * len(both), "one pocket"
    assert all(0.150 <= x <= 0.255 and 0.280 <= y <= 0.390 for x, y in both if x > PLAT_X[0] + 0.015), "pocket bounds"
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
