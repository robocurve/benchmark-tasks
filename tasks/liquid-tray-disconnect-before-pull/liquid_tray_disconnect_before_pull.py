"""On the station table, in front of the left arm, a liquid-cooled compute tray (a grey box) sits in a rack slot
between two fixed steel rails, its rear end toward the rack's fixed blue coolant manifold. Two blue quick-disconnect
couplers (short upright cylinders, the ends of the tray's supply and return hoses) stand on the tray's top near its
rear: the tray is still connected. Beside the slot, on the rack frame's foot, two fixed blue park pockets hold
disconnected couplers, and a white service mat lies nearer the arms. The service rule: disconnect both couplers and
park them before the tray leaves its slot. The order is checked every step: if the tray moves more than 1 cm while a
coupler is still on it, coolant is spilled and the trial scores 0 from then on.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/liquid-tray-disconnect-before-pull/liquid_tray_disconnect_before_pull.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. The left arm's base is at (0, +0.305, 0.75).
TRAY = (0.075, 0.130, 0.025)  # depth (x, the slide-out direction), width (y), height
TRAY_KG = 0.250
TRAY_XY = (0.360, 0.200)
TRAY_TOP = TOP + TRAY[2]
RAIL = (0.110, 0.008, 0.035)
RAIL_GAP = 0.003  # air between the tray's sides and each rail
RAIL_Y = (TRAY_XY[1] + TRAY[1] / 2 + RAIL_GAP + RAIL[1] / 2, TRAY_XY[1] - TRAY[1] / 2 - RAIL_GAP - RAIL[1] / 2)
MANIFOLD = (0.015, 0.150, 0.060)  # the rack's coolant manifold, a fixed bar behind the slot
MANIFOLD_X = 0.4375

QD_R = 0.012  # quick-disconnect coupler body: 24 mm across, 40 mm tall
QD_H = 0.040
QD_KG = 0.030
QD_START = {"supply_coupler": (0.380, 0.235), "return_coupler": (0.380, 0.165)}  # on the tray, near its rear
PARK = {"A": (0.380, 0.320), "B": (0.380, 0.375)}  # park pocket centres, on the table beside the left rail
PARK_INNER = (0.040, 0.040)
PARK_WALL = (0.005, 0.012)

MAT = (0.150, 0.180, 0.003)
MAT_XY = (0.200, 0.200)
MAT_TOP = TOP + MAT[2]

TRAY_MOVE = 0.010  # the tray counts as moved once its centre is 1 cm from its start

GREY = (0.45, 0.47, 0.50)
STEEL = (0.75, 0.76, 0.78)
BLUE = (0.15, 0.35, 0.85)
DARK_BLUE = (0.10, 0.20, 0.50)
WHITE = (0.92, 0.92, 0.92)


def _frame(name, centre, inner, wall, base_z, color):
    cx, cy = centre
    ix, iy = inner
    t, h = wall
    z = base_z + h / 2
    return [
        Box(f"{name}_wall_ypos", pos=(cx, cy + iy / 2 + t / 2, z), size=(ix + 2 * t, t, h), color=color),
        Box(f"{name}_wall_yneg", pos=(cx, cy - iy / 2 - t / 2, z), size=(ix + 2 * t, t, h), color=color),
        Box(f"{name}_wall_near", pos=(cx - ix / 2 - t / 2, cy, z), size=(t, iy, h), color=color),
        Box(f"{name}_wall_far", pos=(cx + ix / 2 + t / 2, cy, z), size=(t, iy, h), color=color),
    ]


COMPUTE_TRAY = Box("compute_tray", pos=(*TRAY_XY, TOP + TRAY[2] / 2), size=TRAY, mass=TRAY_KG, color=GREY)
COUPLERS = [Cylinder(name, pos=(*xy, TRAY_TOP + QD_H / 2), radius=QD_R, height=QD_H, mass=QD_KG, color=BLUE)
            for name, xy in QD_START.items()]
FIXTURES = [
    *(Box(f"rail_{s}", pos=(TRAY_XY[0], y, TOP + RAIL[2] / 2), size=RAIL, color=STEEL) for s, y in zip(("left", "right"), RAIL_Y)),
    Box("manifold", pos=(MANIFOLD_X, TRAY_XY[1], TOP + MANIFOLD[2] / 2), size=MANIFOLD, color=DARK_BLUE),
    *(w for k, xy in PARK.items() for w in _frame(f"park_{k}", xy, PARK_INNER, PARK_WALL, TOP, DARK_BLUE)),
    Box("service_mat", pos=(*MAT_XY, TOP + MAT[2] / 2), size=MAT, color=WHITE),
]
OBJECTS = [COMPUTE_TRAY, *COUPLERS, *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"


def _tilt(quat_xyzw) -> float:
    """Tilt of the body's up axis from the vertical, degrees."""
    x, y, _, _ = (float(v) for v in quat_xyzw)
    return math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))))


def _parked(st) -> bool:
    """A coupler at rest, upright, its centre inside either park pocket's opening, standing on the table."""
    return bool(
        any(abs(float(st.pos[0]) - px) < PARK_INNER[0] / 2 and abs(float(st.pos[1]) - py) < PARK_INNER[1] / 2
            for px, py in PARK.values())
        and TOP - 0.002 < float(st.low[2]) < TOP + 0.005
        and _tilt(st.quat_xyzw) < 10.0
        and st.resting()
    )


def _on_tray(coupler, tray) -> bool:
    """A coupler still on the tray: its centre over the tray's outline and its bottom at or above the tray's top."""
    return bool(
        float(tray.low[0]) < float(coupler.pos[0]) < float(tray.high[0])
        and float(tray.low[1]) < float(coupler.pos[1]) < float(tray.high[1])
        and float(coupler.low[2]) > float(tray.high[2]) - 0.005
    )


def _tray_moved(tray) -> bool:
    return math.dist((float(tray.pos[0]), float(tray.pos[1]), float(tray.pos[2])), COMPUTE_TRAY.pos) > TRAY_MOVE


def _tray_on_mat(tray) -> bool:
    """The tray at rest, flat on the service mat, its centre within 4 cm of the mat's centre."""
    return bool(
        abs(float(tray.pos[0]) - MAT_XY[0]) < 0.040
        and abs(float(tray.pos[1]) - MAT_XY[1]) < 0.040
        and MAT_TOP - 0.002 < float(tray.low[2]) < MAT_TOP + 0.005
        and _tilt(tray.quat_xyzw) < 5.0
        and tray.resting()
    )


# The trial's history: whether the tray ever moved while a coupler was still on it (a spill). The task file only sees
# object states, so the history resets whenever the scene is back in its start state (each new trial).
_HISTORY = {"spilled": False}


def _at_start(objects) -> bool:
    def near(name, xyz):
        return math.dist([float(v) for v in objects[name].pos], xyz) < 0.005

    return near("compute_tray", COMPUTE_TRAY.pos) and all(near(c.name, c.pos) for c in COUPLERS)


def _update(objects) -> bool:
    """Record a spill if the tray has moved while a coupler is on it; returns whether this trial has spilled."""
    tray = objects["compute_tray"]
    if _at_start(objects):
        _HISTORY["spilled"] = False
    elif _tray_moved(tray) and any(_on_tray(objects[c.name], tray) for c in COUPLERS):
        _HISTORY["spilled"] = True
    return _HISTORY["spilled"]


def score(objects) -> float:
    """0.25 for each coupler parked in a park pocket, plus 0.5 for the tray on the service mat with both couplers
    parked. A spill (the tray moved more than 1 cm with a coupler still on it) scores 0 for the rest of the trial."""
    if _update(objects):
        return 0.0
    parked = [_parked(objects[c.name]) for c in COUPLERS]
    s = 0.25 * sum(parked)
    if all(parked) and _tray_on_mat(objects["compute_tray"]):
        s += 0.5
    return float(s)


def success(objects) -> bool:
    """Both couplers parked and the tray on the service mat, with no spill; holds for 1 s."""
    return score(objects) >= 1.0 - 1e-9


def spilled() -> bool:
    """Whether this trial has moved the tray with a coupler still on it (for the log)."""
    return _HISTORY["spilled"]


def distance(objects) -> float:
    """Metres to go: each coupler to its nearest park pocket plus the tray to the mat's centre."""
    t = objects["compute_tray"]
    d = math.hypot(float(t.pos[0]) - MAT_XY[0], float(t.pos[1]) - MAT_XY[1])
    for c in COUPLERS:
        st = objects[c.name]
        d += min(math.hypot(float(st.pos[0]) - px, float(st.pos[1]) - py) for px, py in PARK.values())
    return float(d)


# -- the scripted demo ------------------------------------------------------------------------------------------

YAW = math.pi / 2  # every grip closes along x: on the couplers' front and back, on the tray's front and rear faces
TIP = 0.013
TRAVEL = TOP + 0.150
SQUEEZE = 0.008


def _move(pick, place, *, width, height, depth, pick_bottom, place_bottom, label, open_):
    (px, py), (qx, qy) = pick, place
    gz = pick_bottom + height - depth + TIP
    qz = place_bottom + height - depth + TIP + 0.002  # let go 2 mm above the resting height
    grip = -(width - SQUEEZE) / 2
    return [
        Waypoint(1.8, (px, py, TRAVEL), open_, f"{label}: over", YAW),
        Waypoint(1.5, (px, py, gz + 0.020), open_, f"{label}: descend", YAW),
        Waypoint(1.0, (px, py, gz), open_, f"{label}: down slowly", YAW),
        Waypoint(0.5, (px, py, gz), open_, f"{label}: settle", YAW),
        Waypoint(1.5, (px, py, gz), grip, f"{label}: close", YAW),
        Waypoint(1.5, (px, py, gz + 0.030), grip, f"{label}: lift straight up", YAW),
        Waypoint(1.2, (px, py, TRAVEL), grip, f"{label}: lift", YAW),
        Waypoint(2.0, (qx, qy, TRAVEL), grip, f"{label}: carry", YAW),
        Waypoint(0.6, (qx, qy, TRAVEL), grip, f"{label}: settle over", YAW),
        Waypoint(1.5, (qx, qy, qz + 0.020), grip, f"{label}: lower", YAW),
        Waypoint(1.5, (qx, qy, qz), grip, f"{label}: set down slowly", YAW),
        Waypoint(0.6, (qx, qy, qz), grip, f"{label}: settle", YAW),
        Waypoint(1.5, (qx, qy, qz), open_, f"{label}: release", YAW),
        Waypoint(1.0, (qx, qy, TRAVEL), open_, f"{label}: retreat", YAW),
    ]


def demo():
    """Left arm only, in the service order: the supply coupler off the tray into park pocket A, the return coupler
    into park pocket B (fingers 48 mm apart across each 24 mm body), then the tray straight up out of its slot, between
    the rails, and onto the service mat (fingers fully open across its 75 mm depth). 53.1 s."""
    qd_open = -(2 * QD_R + 0.024) / 2
    supply = _move(QD_START["supply_coupler"], PARK["A"], width=2 * QD_R, height=QD_H, depth=0.015,
                   pick_bottom=TRAY_TOP, place_bottom=TOP, label="supply coupler", open_=qd_open)
    ret = _move(QD_START["return_coupler"], PARK["B"], width=2 * QD_R, height=QD_H, depth=0.015,
                pick_bottom=TRAY_TOP, place_bottom=TOP, label="return coupler", open_=qd_open)
    tray = _move(TRAY_XY, MAT_XY, width=TRAY[0], height=TRAY[2], depth=0.015, pick_bottom=TOP, place_bottom=MAT_TOP,
                 label="tray", open_=FINGER_OPEN)
    return {"left": supply + ret + tray}


def self_test(ObjectState=None):
    """The scoring rules on hand-built sequences of states, so the history is exercised too."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def body(half, x, y, bottom, vel=(0.0, 0.0, 0.0), tilt_deg=0.0):
        t = math.radians(tilt_deg)
        q = np.array([math.sin(t / 2), 0.0, 0.0, math.cos(t / 2)])
        half = np.array(half)
        pos = np.array([x, y, bottom + half[2]])
        return ObjectState(pos, q, np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    def tray(x, y, bottom, **kw):
        return {"compute_tray": body(np.array(TRAY) / 2, x, y, bottom, **kw)}

    def qd(name, x, y, bottom, **kw):
        return {name: body((QD_R, QD_R, QD_H / 2), x, y, bottom, **kw)}

    S, R = "supply_coupler", "return_coupler"
    start = tray(*TRAY_XY, TOP) | qd(S, *QD_START[S], TRAY_TOP) | qd(R, *QD_START[R], TRAY_TOP)
    s_parked = start | qd(S, *PARK["A"], TOP)
    both = s_parked | qd(R, *PARK["B"], TOP)
    lifted = tray(*TRAY_XY, TOP + 0.05, vel=(0, 0, 0.05))
    on_mat = tray(*MAT_XY, MAT_TOP)
    done = both | on_mat
    # couplers that rode along on the lifted or moved tray
    ride = {**qd(S, QD_START[S][0], QD_START[S][1], TRAY_TOP + 0.05), **qd(R, QD_START[R][0], QD_START[R][1], TRAY_TOP + 0.05)}
    dx, dy = MAT_XY[0] - TRAY_XY[0], MAT_XY[1] - TRAY_XY[1]
    ride_mat = {**qd(S, QD_START[S][0] + dx, QD_START[S][1] + dy, MAT_TOP + TRAY[2]),
                **qd(R, QD_START[R][0] + dx, QD_START[R][1] + dy, MAT_TOP + TRAY[2])}

    def run(*frames):
        """Feed the states in order (as the simulator would, step by step); the last score and success."""
        for f in frames[:-1]:
            score(f)
        return score(frames[-1]), success(frames[-1])

    cases = {
        "start": ((start,), 0.0, False),
        "supply coupler parked": ((start, s_parked), 0.25, False),
        "both parked": ((start, s_parked, both), 0.5, False),
        "both parked, then the tray to the mat": ((start, s_parked, both, both | lifted, done), 1.0, True),
        "tray lifted with both couplers riding on it": ((start, start | lifted | ride), 0.0, False),
        "tray pulled with couplers on it, couplers parked afterwards":
            ((start, start | lifted | ride, start | on_mat | ride_mat, done), 0.0, False),
        "one parked, tray pulled with the other still on it":
            ((start, s_parked, s_parked | lifted | qd(R, QD_START[R][0], QD_START[R][1], TRAY_TOP + 0.05), done), 0.0, False),
        "tray nudged 5 mm with couplers on it is no spill": ((start, start | tray(TRAY_XY[0] - 0.005, TRAY_XY[1], TOP), done), 1.0, True),
        "a new trial after a spill starts clean": ((start, start | lifted | ride, start, s_parked, both, done), 1.0, True),
        "both parked, tray held above the mat": ((start, both, both | tray(*MAT_XY, MAT_TOP + 0.03)), 0.5, False),
        "both parked, tray on the table beside the mat": ((start, both, both | tray(0.200, 0.040, TOP)), 0.5, False),
        "coupler lying on its side in a pocket": ((start, start | qd(S, *PARK["A"], TOP, tilt_deg=90)), 0.0, False),
        "coupler on the table beside a pocket": ((start, start | qd(S, PARK["A"][0] - 0.05, PARK["A"][1], TOP)), 0.0, False),
        "coupler still sliding in its pocket": ((start, start | qd(S, *PARK["A"], TOP, vel=(0.02, 0, 0))), 0.0, False),
    }
    for name, (frames, want, want_success) in cases.items():
        got, got_success = run(*frames)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
    score(start)  # leave the history clean
    assert not spilled()
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
