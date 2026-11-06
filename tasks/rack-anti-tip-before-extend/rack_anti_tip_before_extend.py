"""On the station table, in front of the left arm, the foot of an equipment rack, seen from its front: three dark
uprights. On the right, a short-depth server (a grey box) sits in the rack between two fixed steel slide rails that
run out toward the arms to its service position. On the left, the rack's anti-tip stabiliser foot (a yellow bar)
lies retracted in a fixed guide at floor level, its pull end sticking 4 cm out of the rack's front. The rack-safety
rule: extend the stabiliser foot before any equipment is drawn out of the rack. The order is checked every step: if
the server moves more than 1 cm while the foot is still retracted, the rack could tip and the trial scores 0 from
then on. Both parts are slid along the table, not lifted. Scored from the parts' poses at the end.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/rack-anti-tip-before-extend/rack_anti_tip_before_extend.py \\
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
RACK_FRONT = 0.400  # the rack's front plane
POST = (0.020, 0.020, 0.150)  # the rack's front uprights, set 5 cm back so the gripper clears them at the pull ends
POST_X = RACK_FRONT + 0.060
POST_Y = (0.150, 0.315, 0.445)

SERVER = (0.160, 0.080, 0.045)  # depth (x, the slide-out direction), width (y), height
SERVER_KG = 0.350
SERVER_Y = 0.220
SERVER_X0 = 0.420  # in the rack
SERVICE = 0.100  # service position: drawn out 10 cm
SERVICE_TOL = 0.015
SLIDE = (0.280, 0.008, 0.015)  # the slide rails, from the rack's back out to the service position
SLIDE_GAP = 0.003
SLIDE_X = 0.360
SLIDE_Y = (SERVER_Y + SERVER[1] / 2 + SLIDE_GAP + SLIDE[1] / 2, SERVER_Y - SERVER[1] / 2 - SLIDE_GAP - SLIDE[1] / 2)

FOOT = (0.140, 0.030, 0.020)  # the stabiliser foot: a bar along x
FOOT_KG = 0.150
FOOT_Y = 0.380
FOOT_X0 = RACK_FRONT + FOOT[0] / 2 - 0.040  # retracted: its pull end 4 cm out of the plinth
EXTENDED = 0.070  # the foot counts as extended once drawn out at least 7 cm
FOOT_MAX = 0.110  # and still in its guide (its far end at least 1 cm inside the rack front)
GUIDE = (0.120, 0.005, 0.010)  # the foot's guide walls, under the plinth
GUIDE_GAP = 0.003
GUIDE_Y = (FOOT_Y + FOOT[1] / 2 + GUIDE_GAP + GUIDE[1] / 2, FOOT_Y - FOOT[1] / 2 - GUIDE_GAP - GUIDE[1] / 2)

SERVER_MOVE = 0.010  # the server counts as drawn out once its centre is 1 cm from its start

GREY = (0.40, 0.42, 0.45)
STEEL = (0.74, 0.75, 0.77)
DARK = (0.12, 0.12, 0.14)
YELLOW = (0.95, 0.78, 0.10)

THE_SERVER = Box("server", pos=(SERVER_X0, SERVER_Y, TOP + SERVER[2] / 2), size=SERVER, mass=SERVER_KG, color=GREY)
THE_FOOT = Box("stabiliser_foot", pos=(FOOT_X0, FOOT_Y, TOP + FOOT[2] / 2), size=FOOT, mass=FOOT_KG, color=YELLOW)
FIXTURES = [
    *(Box(f"rack_post_{k + 1}", pos=(POST_X, y, TOP + POST[2] / 2), size=POST, color=DARK)
      for k, y in enumerate(POST_Y)),
    *(Box(f"slide_{s}", pos=(SLIDE_X, y, TOP + SLIDE[2] / 2), size=SLIDE, color=STEEL) for s, y in zip(("left", "right"), SLIDE_Y)),
    *(Box(f"foot_guide_{s}", pos=(RACK_FRONT + GUIDE[0] / 2, y, TOP + GUIDE[2] / 2), size=GUIDE, color=DARK)
      for s, y in zip(("left", "right"), GUIDE_Y)),
]
OBJECTS = [THE_SERVER, THE_FOOT, *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"


def _yaw_deg(quat_xyzw) -> float:
    x, y, z, w = (float(v) for v in quat_xyzw)
    return math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def _tilt(quat_xyzw) -> float:
    x, y, _, _ = (float(v) for v in quat_xyzw)
    return math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))))


def _on_table(st) -> bool:
    return bool(abs(float(st.low[2]) - TOP) <= 0.003 and _tilt(st.quat_xyzw) <= 5.0)


def _foot_out(st) -> float:
    return FOOT_X0 - float(st.pos[0])


def _extended(st) -> bool:
    """The foot drawn out 7 to 11 cm, still in its guide, square within 10 degrees, flat on the table."""
    return bool(
        EXTENDED <= _foot_out(st) <= FOOT_MAX
        and abs(float(st.pos[1]) - FOOT_Y) <= 0.010
        and abs(_yaw_deg(st.quat_xyzw)) <= 10.0
        and _on_table(st)
    )


def _server_out(st) -> float:
    return SERVER_X0 - float(st.pos[0])


def _server_moved(st) -> bool:
    return math.dist([float(v) for v in st.pos], THE_SERVER.pos) > SERVER_MOVE


def _at_service(st) -> bool:
    """The server drawn out 10 cm (within 1.5 cm) on its rails, square within 5 degrees, flat on the table, still."""
    return bool(
        abs(_server_out(st) - SERVICE) <= SERVICE_TOL
        and abs(float(st.pos[1]) - SERVER_Y) <= 0.005
        and abs(_yaw_deg(st.quat_xyzw)) <= 5.0
        and _on_table(st)
        and st.resting()
    )


# The trial's history: whether the server was ever drawn out while the foot was retracted (a tip hazard). The task
# file only sees object states, so the history resets whenever the scene is back in its start state (each new trial).
_HISTORY = {"tipped": False}


def _at_start(objects) -> bool:
    return (math.dist([float(v) for v in objects["server"].pos], THE_SERVER.pos) < 0.003
            and math.dist([float(v) for v in objects["stabiliser_foot"].pos], THE_FOOT.pos) < 0.003)


def _update(objects) -> bool:
    if _at_start(objects):
        _HISTORY["tipped"] = False
    elif _server_moved(objects["server"]) and not _extended(objects["stabiliser_foot"]):
        _HISTORY["tipped"] = True
    return _HISTORY["tipped"]


def tipped() -> bool:
    """Whether this trial has drawn the server out with the foot retracted (for the log)."""
    return _HISTORY["tipped"]


def score(objects) -> float:
    """0.25 for the stabiliser foot extended (drawn out 7 to 11 cm, in its guide, flat); 0.5 once the server has also
    been drawn out at least 5 cm; 1.0 with the foot extended and the server at its service position (10 cm out within
    1.5 cm, on its rails, square, flat, still). Drawing the server out more than 1 cm while the foot is retracted
    scores 0 for the rest of the trial."""
    if _update(objects):
        return 0.0
    foot, server = objects["stabiliser_foot"], objects["server"]
    if not _extended(foot):
        return 0.0
    if _at_service(server):
        return 1.0
    if _server_out(server) >= 0.050 and _on_table(server):
        return 0.5
    return 0.25


def success(objects) -> bool:
    """Foot extended, then the server drawn out to its service position, with no tip hazard; holds for 1 s."""
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    """Metres still to draw: the foot to 8 cm out plus the server to 10 cm out."""
    foot, server = objects["stabiliser_foot"], objects["server"]
    return float(max(0.0, 0.080 - _foot_out(foot)) + abs(SERVICE - _server_out(server)))


# -- the scripted demo ------------------------------------------------------------------------------------------

YAW = 0.0  # every grip closes along y, across the foot's and the server's widths
TIP = 0.013
TRAVEL = TOP + 0.120
SQUEEZE = 0.008
PULL_V = 0.015  # m/s, average drawing speed (the smoothed peak is 1.5 times this)


def _draw(x, y, *, width, height, depth, pull, label, open_, x_lag):
    """Grip a part's sides, then draw it straight toward the arms along the table, let go, go up."""
    gz = TOP + height - depth + TIP
    grip = -(width - SQUEEZE) / 2
    ex = x - pull - x_lag
    return [
        Waypoint(1.8, (x, y, TRAVEL), open_, f"{label}: over", YAW),
        Waypoint(1.5, (x, y, gz + 0.020), open_, f"{label}: descend", YAW),
        Waypoint(1.0, (x, y, gz), open_, f"{label}: down slowly", YAW),
        Waypoint(0.5, (x, y, gz), open_, f"{label}: settle", YAW),
        Waypoint(1.5, (x, y, gz), grip, f"{label}: close", YAW),
        Waypoint(round((pull + x_lag) / PULL_V, 1), (ex, y, gz), grip, f"{label}: draw out slowly along the table", YAW),
        Waypoint(0.6, (ex, y, gz), grip, f"{label}: settle", YAW),
        Waypoint(1.5, (ex, y, gz), open_, f"{label}: release", YAW),
        Waypoint(1.0, (ex, y, TRAVEL), open_, f"{label}: retreat", YAW),
    ]


# A dragging arm trails its command by about 4 mm per newton of table friction when pulling toward its base (measured
# in sim: the 350 g server came 11.2 cm out for 12.5 cm commanded), so each draw is commanded this much further.
FOOT_LAG = 0.006
SERVER_LAG = 0.013


def demo():
    """Left arm only: grip the stabiliser foot's pull end across its 3 cm width and draw it 8 cm out along the floor;
    then grip the server 3 cm in from its front across its 8 cm width (fingers fully open on the way down) and draw it
    10 cm out on its rails to the service position. 32.0 s."""
    foot = _draw(RACK_FRONT - 0.025, FOOT_Y, width=FOOT[1], height=FOOT[2], depth=0.012, pull=0.080,
                 label="stabiliser foot", open_=-(FOOT[1] + 0.024) / 2, x_lag=FOOT_LAG)
    server = _draw(SERVER_X0 - SERVER[0] / 2 + 0.030, SERVER_Y, width=SERVER[1], height=SERVER[2], depth=0.015,
                   pull=SERVICE, label="server", open_=FINGER_OPEN, x_lag=SERVER_LAG)
    return {"left": foot + server}


def self_test(ObjectState=None):
    """The scoring rules on hand-built sequences of states, so the order history is exercised too."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def body(name, size, x, y, bottom=TOP, yaw_deg=0.0, vel=(0.0, 0.0, 0.0)):
        yaw = math.radians(yaw_deg)
        q = np.array([0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)])
        c, s = abs(math.cos(yaw)), abs(math.sin(yaw))
        half = np.array([c * size[0] / 2 + s * size[1] / 2, s * size[0] / 2 + c * size[1] / 2, size[2] / 2])
        pos = np.array([x, y, bottom + half[2]])
        return {name: ObjectState(pos, q, np.asarray(vel, float), np.zeros(3), pos - half, pos + half)}

    def foot(out, **kw):
        return body("stabiliser_foot", FOOT, FOOT_X0 - out, FOOT_Y, **kw)

    def server(out, **kw):
        return body("server", SERVER, SERVER_X0 - out, SERVER_Y, **kw)

    def run(*frames):
        for f in frames[:-1]:
            score(f)
        return score(frames[-1]), success(frames[-1])

    start = foot(0.0) | server(0.0)
    f_out = foot(0.080) | server(0.0)
    half = foot(0.080) | server(0.060, vel=(-0.015, 0, 0))
    done = foot(0.080) | server(0.100)
    cases = {
        "start": ((start,), 0.0, False),
        "foot extended": ((start, f_out), 0.25, False),
        "foot extended, server half out": ((start, f_out, half), 0.5, False),
        "foot extended, server at service": ((start, f_out, half, done), 1.0, True),
        "server at service, 1.2 cm short": ((start, f_out, foot(0.080) | server(0.088)), 1.0, True),
        "server drawn out too far (13 cm)": ((start, f_out, foot(0.080) | server(0.130)), 0.5, False),
        "server first, then the foot": ((start, foot(0.0) | server(0.040), done), 0.0, False),
        "server nudged 5 mm first is no hazard": ((start, foot(0.0) | server(0.005), f_out, done), 1.0, True),
        "foot only 5 cm out when the server moves": ((start, foot(0.050) | server(0.0), foot(0.050) | server(0.030), done), 0.0, False),
        "foot pulled out of its guide (13 cm)": ((start, foot(0.130) | server(0.0)), 0.0, False),
        "foot lifted off the floor": ((start, foot(0.080, bottom=TOP + 0.02) | server(0.0)), 0.0, False),
        "server lifted off its rails at service": ((start, f_out, foot(0.080) | server(0.100, bottom=TOP + 0.02)), 0.25, False),
        "a new trial after a hazard starts clean": ((start, foot(0.0) | server(0.040), start, f_out, done), 1.0, True),
    }
    for name, (frames, want, want_success) in cases.items():
        got, got_success = run(*frames)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
    score(start)  # leave the history clean
    assert not tipped()
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
