"""A toilet seat with quick-release hinges, taken off for cleaning and left lying on the bowl rim 45 mm forward of its
hinge posts. The seat (0.38 m long, 1.2 kg with its hinge piece) has two hinge sockets 0.16 m apart at its back edge.
Each socket must drop over its post, and each post carries a spring button that the socket has to push down 8.5 mm
(about 11 N) before it clicks. The two sockets are one rigid hinge piece, so they go down together; cocked, one
socket goes on and the other sits on its post top or jams. The seat ring turns on that hinge piece (0 to 100 deg), so
once the sockets are home the seat lifts on its hinges like a real one.

Mechanism parts come from mechkit.py (in this folder), which adds joints to the stock runtime in memory.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/toilet-seat-quick-release-hinge/toilet_seat_quick_release_hinge.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 --max-steps 1200 \\
           --instruction "Put the toilet seat back on." --scorer task_score ...
"""

import math
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402

from yam_arena.constants import FINGER_CLOSED  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

WHITE = (0.94, 0.94, 0.92)
SEAT_C = (0.97, 0.97, 0.95)
CHROME = (0.75, 0.76, 0.78)
BLUE = (0.20, 0.40, 0.80)
GREY_C = (0.55, 0.56, 0.58)

# Frame: x forward from the arm bases, y left, z up, metres.
R = TOP + 0.020  # bowl rim top
PX = 0.380  # hinge posts, x
PY = 0.080  # posts at y = +-PY (0.16 m apart)
SIDES = {"L": 1, "R": -1}
START_DX = -0.045  # the seat starts this far forward of home

# -- posts and buttons -----------------------------------------------------------------------------------------
POST = (0.012, 0.012, 0.020)
BTN = (0.012, 0.012, 0.006)
BTN_K = 600.0  # N/m
BTN_PRELOAD = 0.010  # 6 N preload against the top stop
BTN_LATCH = 0.0085  # pressed this far, it clicks (about 11 N)
BTN_LOCK = 0.010  # held here once clicked
kit = mk.Kit()
for s, sg in SIDES.items():
    kit.part(f"post_{s}", (PX, sg * PY, R), [mk.box(POST, at=(0, 0, POST[2] / 2), color=CHROME)], mass=None,
             color=CHROME)
    kit.part(f"button_{s}", (PX, sg * PY, R), [mk.box(BTN, at=(0, 0, 0.033), color=BLUE)], mass=0.030, color=BLUE)
    kit.slider(f"button_{s}", f"post_{s}", f"button_{s}", axis=(0, 0, 1), lower=-0.012, upper=0.0, start=0.0,
               stiffness=BTN_K, rest=BTN_PRELOAD, damping=2.0)
    t = np.eye(4)
    t[:3, 3] = (PX, sg * PY, R - BTN_LOCK)
    kit.catch(f"lock_{s}", f"button_{s}", t)

# -- hinge piece and seat ---------------------------------------------------------------------------------------
# Frame of both parts: origin on the hinge line between the sockets, at the seat's underside. Home = origin at
# (PX, 0, R). The hinge piece carries the two sockets; the seat ring turns on it about the hinge axis (y), which runs
# through the hinge line at the seat's top face.
SEAT_H = 0.020
CAV = 0.026  # socket bore (posts 12 mm)
WALL = 0.003
HOUS = CAV + 2 * WALL  # 32 mm housing
ROOF_IN = 0.024  # bore ceiling above the housing bottom
ROOF = 0.006
_SEAT = [mk.box((0.050, 0.340, SEAT_H), at=(-0.345, 0, SEAT_H / 2)),  # front
         mk.box((0.044, 0.340, SEAT_H), at=(-0.038, 0, SEAT_H / 2)),  # back, ends 16 mm ahead of the hinge line
         mk.box((0.260, 0.050, SEAT_H), at=(-0.190, 0.145, SEAT_H / 2)),
         mk.box((0.260, 0.050, SEAT_H), at=(-0.190, -0.145, SEAT_H / 2))]
_HINGE = []
for sg in SIDES.values():
    y = sg * PY
    _HINGE += [mk.box((WALL, HOUS, ROOF_IN), at=(sx * (CAV + WALL) / 2, y, ROOF_IN / 2), color=GREY_C)
               for sx in (1, -1)]
    _HINGE += [mk.box((CAV, WALL, ROOF_IN), at=(0, y + sy * (CAV + WALL) / 2, ROOF_IN / 2), color=GREY_C)
               for sy in (1, -1)]
    _HINGE.append(mk.box((HOUS, HOUS, ROOF), at=(0, y, ROOF_IN + ROOF / 2), color=GREY_C))
HINGE_KG = 0.2
SEAT_KG = 1.0
SEAT0 = (PX + START_DX, 0.0, R + 0.0005)
kit.part("hinge", SEAT0, _HINGE, mass=HINGE_KG, color=GREY_C)
kit.part("seat", SEAT0, _SEAT, mass=SEAT_KG, color=SEAT_C)
SWING_MAX = 100.0  # deg: the seat stands up past vertical against its stop
kit.hinge("swing", "hinge", "seat", anchor=(SEAT0[0], 0.0, SEAT0[2] + SEAT_H), axis=(0, 1, 0), lower_deg=0.0,
          upper_deg=SWING_MAX, damping=0.05)
CLICK_Z = BTN[2] + 0.030 - BTN_LOCK - ROOF_IN  # socket bottom above the rim on a clicked button: 2 mm
HOME = np.eye(4)
HOME[:3, 3] = (PX, 0.0, R + CLICK_Z)
kit.catch("seat_home", "hinge", HOME)
SEAT_PARTS = ("hinge", "seat")

BOWL = (0.435, 0.360, R - TOP)
FIXTURES = [
    Box("bowl", pos=(0.395 - BOWL[0] / 2, 0.0, TOP + BOWL[2] / 2), size=BOWL, color=WHITE),
]
OBJECTS = [*kit.objects(), *FIXTURES]

OVER_XY = 0.008  # bore centred on its post within 8 mm (the post can sit at most 7 mm off centre in the bore)
OVER_Z = 0.005  # down home, for the weld: bottom within 5 mm of the rim (a clicked button holds it 2 mm up)
ON_Z = 0.016  # on its post: the socket's bottom below this (an unclicked button holds it 12 mm up)
ON_PTS = 0.2  # per socket on its post
CLICK_PTS = 0.3  # per socket clicked on


def socket(objects, s):
    """World point at the bottom centre of socket s."""
    return kit.point(objects, "hinge", (0.0, SIDES[s] * PY, 0.0))


def aligned(objects, s) -> bool:
    c = socket(objects, s)
    return math.hypot(float(c[0]) - PX, float(c[1]) - SIDES[s] * PY) <= OVER_XY


def over(objects, s) -> bool:
    """Socket s down over its post: centred within 8 mm and its bottom within 5 mm of the rim."""
    return aligned(objects, s) and abs(float(socket(objects, s)[2]) - R) <= OVER_Z


def on_post(objects, s) -> bool:
    """Socket s on its post (the post inside the bore): centred within 8 mm, its bottom under 16 mm above the rim."""
    return aligned(objects, s) and float(socket(objects, s)[2]) - R <= ON_Z


def pressed(objects, s) -> float:
    """How far button s is pressed down (m), from its cap's bounding box (the cap only slides up and down)."""
    st = objects[f"button_{s}"]
    return R + 0.033 - (float(st.low[2]) + float(st.high[2])) / 2


def clicked(objects, s) -> bool:
    """Socket s clicked on: its button clicked under it (pressed 8.5 mm by the socket with the post in the bore) and
    the socket is still on its post. (Alone, a clicked socket is not held down in sim: a rigid seat lying on a flat
    rim keeps it level, so with the other side up on its button it rests 12 mm up on the post.)"""
    return bool(mk.MEMORY.get(f"lock_{s}")) and on_post(objects, s)


STEP_S = 0.05  # control step at 20 Hz
STILL_V = 0.01  # m/s: still when no tracked point of the seat or hinge piece moves faster than this over a step


def _track(objects) -> np.ndarray:
    """Points whose motion tells if the seat is still: both sockets and the seat ring's front corners."""
    return np.array([socket(objects, s) for s in SIDES]
                    + [kit.point(objects, "seat", (-0.370, sy * 0.170, 0.0)) for sy in (1, -1)])


def _rule(objects):
    M = mk.MEMORY
    M["n"] = M.get("n", 0) + 1
    # Stillness from how far the parts moved over the last control step (the hinge piece's reported spin jitters
    # by ~0.15 rad/s while it sits motionless on the buttons, so instantaneous velocities are not used in sim).
    pts = _track(objects)
    if "pts" in M:
        M["speed"] = float(np.max(np.linalg.norm(pts - M["pts"], axis=1))) / STEP_S
    M["pts"] = pts
    for s in SIDES:
        # it clicks only when the socket itself pressed it: the post inside the bore (a wall on the cap does not count)
        if not M.get(f"lock_{s}") and pressed(objects, s) >= BTN_LATCH and on_post(objects, s):
            kit.engage(f"lock_{s}")
            M[f"lock_{s}"] = True
    if M.get("lock_L") and M.get("lock_R") and not M.get("home") and over(objects, "L") and over(objects, "R"):
        kit.engage("seat_home")
        M["home"] = True
    if min(float(objects[p].pos[2]) for p in SEAT_PARTS) < TOP - 0.050:
        M["fell"] = True
    if os.environ.get("TOILET_DEBUG") and M["n"] % 20 == 0:
        print(f"[toilet] step {M['n']} btn L {pressed(objects, 'L') * 1000:.1f} R "
              f"{pressed(objects, 'R') * 1000:.1f} mm sock L {np.round(socket(objects, 'L'), 4)} "
              f"R {np.round(socket(objects, 'R'), 4)} swing {kit.q_deg(objects, 'swing'):.1f} deg "
              f"flags { {k: v for k, v in M.items() if k in ('lock_L', 'lock_R', 'home', 'fell')} } "
              f"score {score(objects)}", flush=True)
    if os.environ.get("TOILET_DEBUG") == "2":
        h = mk.quat_to_mat(objects["hinge"].quat_xyzw)
        print(f"[toilet2] {M['n']} pitch {math.degrees(math.atan2(-h[2, 0], h[0, 0])):.1f} "
              f"roll {math.degrees(math.atan2(h[2, 1], h[1, 1])):.1f} v {np.round(objects['hinge'].lin_vel, 3)} "
              f"pressed {pressed(objects, 'L') * 1000:.1f} {pressed(objects, 'R') * 1000:.1f} "
              f"z {float(socket(objects, 'L')[2]) - R:.4f} {float(socket(objects, 'R')[2]) - R:.4f} "
              f"x {float(socket(objects, 'L')[0]) - PX:.4f} {float(socket(objects, 'R')[0]) - PX:.4f} "
              f"rel {[_released(objects[p]) for p in SEAT_PARTS]} "
              f"lv {[round(float(np.linalg.norm(objects[p].lin_vel)), 4) for p in SEAT_PARTS]} "
              f"av {[round(float(np.linalg.norm(objects[p].ang_vel)), 4) for p in SEAT_PARTS]}", flush=True)


kit.on_step(_rule)

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

_TEST_TCPS = None


def _released(st, margin=0.020) -> bool:
    tcps = mk.tcps() if _TEST_TCPS is None else _TEST_TCPS
    lo, hi = np.asarray(st.low) - margin, np.asarray(st.high) + margin
    return not any(bool(np.all(np.asarray(t) >= lo) and np.all(np.asarray(t) <= hi)) for t in tcps)


def failed(objects) -> bool:
    """Latched: the seat fell off the bench."""
    return bool(mk.MEMORY.get("fell"))


def score(objects) -> float:
    """0 if the seat fell off the bench, a gripper is within 2 cm of the seat or its hinge piece, or a socket or a
    front corner of the seat moved faster than 1 cm/s over the last step. Otherwise 0.2 for each socket on its post
    plus 0.3 for each socket clicked on: 0.2 one socket on, 0.4 both on unclicked, 0.7 one clicked and the other
    on its post, 1.0 both clicked."""
    sts = [objects[p] for p in SEAT_PARTS]
    speed = mk.MEMORY.get("speed")
    if speed is None:  # offline states (no step history): use their velocities
        still = all(st.resting(max_speed=STILL_V, max_spin=0.1) for st in sts)
    else:
        still = speed < STILL_V
    if failed(objects) or not still or not all(_released(st) for st in sts):
        return 0.0
    return round(sum(ON_PTS * on_post(objects, s) + CLICK_PTS * clicked(objects, s) for s in SIDES), 4)


def success(objects) -> bool:
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    return float(sum(np.linalg.norm(socket(objects, s) - np.array([PX, SIDES[s] * PY, R])) for s in SIDES))


# -- the scripted demo ------------------------------------------------------------------------------------------
# Each arm pinches one socket housing across its sides (fingers along y), lifts the back of the seat 45 mm (the
# front stays on the rim and the seat turns in the pinch), carries it 45 mm back over the posts, lowers both sockets
# onto the buttons together, pushes both down (the TCP aimed 45 mm below where it pinches a clicked socket), lets
# go and backs off.

TIP = 0.013
GRIP = 0.012 + TIP  # pad tips 12 mm above the seat underside
OPEN = -0.030
WIDE = -0.045  # pads 90 mm apart, round a 50 mm seat rail
LIFT = 0.045
X_TRIM = {"L": 0.023, "R": -0.016}
Y_TRIM = {"L": 0.0065, "R": 0.0065}  # open-loop correction for where each arm lands its socket
PRESS = -0.045  # TCP target below home (the arm's position error pushes the button)
ONE_SIDE_PRESS = -0.060  # one_side check: the left arm pushes alone against the held right side


def _place(s):
    """Pinch socket s, lift the back of the seat, carry it over the post and lower it onto the button."""
    y = SIDES[s] * PY + Y_TRIM[s]
    x0, x1 = PX + START_DX, PX + X_TRIM[s]
    z = R + GRIP
    return [Waypoint(2.0, (x0, y, z + 0.050), OPEN, "over the socket", 0.0),
            Waypoint(1.5, (x0, y, z), OPEN, "down round the socket", 0.0),
            Waypoint(1.0, (x0, y, z), FINGER_CLOSED, "pinch the socket", 0.0),
            Waypoint(2.0, (x0, y, z + LIFT), FINGER_CLOSED, "lift the back of the seat", 0.0),
            Waypoint(2.5, (x1, y, z + LIFT), FINGER_CLOSED, "carry it back over the post", 0.0),
            Waypoint(0.5, (x1, y, z + LIFT), FINGER_CLOSED, "settle", 0.0),
            Waypoint(2.0, (x1, y, z + 0.012), FINGER_CLOSED, "lower onto the button", 0.0)]


def _push(s, press=PRESS):
    x1, y, z = PX + X_TRIM[s], SIDES[s] * PY + Y_TRIM[s], R + GRIP
    return [Waypoint(1.5, (x1, y, z + press), FINGER_CLOSED, "push the socket down", 0.0),
            Waypoint(1.0, (x1, y, z + press), FINGER_CLOSED, "hold", 0.0)]


def _leave(plan, s, finger=OPEN):
    x1, y = plan[-1].pos[0], plan[-1].pos[1]
    zl = plan[-1].pos[2]
    return [Waypoint(1.0, (x1, y, zl), finger, "let go", 0.0),
            Waypoint(1.5, (x1, y, R + GRIP + 0.070), finger, "up", 0.0),
            Waypoint(1.5, (x1 - 0.06, y + SIDES[s] * 0.06, R + GRIP + 0.080), finger, "clear", 0.0)]


# Lift check: with both sockets clicked home, the left arm pinches the seat's left rail and swings the seat up on
# its hinges to SWING_UP and back down, while the right arm keeps hold of its socket (so the run does not end).
RAIL_X, RAIL_Y = 0.200, 0.145 + Y_TRIM["L"]
RAIL_TCP = 0.009  # TCP above the middle of the rail's thickness (pad tips 4 mm above the rail underside)
SWING_UP = (10.0, 20.0, 30.0)


def _rail_tcp(deg):
    """TCP over the pinch point on the left rail with the seat swung up ``deg`` about the hinge axis."""
    dx, dz = RAIL_X - PX, -SEAT_H / 2  # the pinch point from the hinge axis (on the seat's top face)
    c, s_ = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return (PX + c * dx + s_ * dz, RAIL_Y, R + SEAT_H + (-s_ * dx + c * dz) + RAIL_TCP)


def _lift_seat():
    p0 = _push("L")[-1].pos
    plan = [Waypoint(1.0, p0, OPEN, "let go of the socket", 0.0),
            Waypoint(1.5, (p0[0], p0[1], R + GRIP + 0.070), OPEN, "up", 0.0),
            Waypoint(2.0, (RAIL_X, RAIL_Y, _rail_tcp(0)[2] + 0.050), WIDE, "over the left rail", 0.0),
            Waypoint(1.5, _rail_tcp(0), WIDE, "down round the rail", 0.0),
            Waypoint(1.0, _rail_tcp(0), FINGER_CLOSED, "pinch the rail", 0.0)]
    plan += [Waypoint(1.0, _rail_tcp(d), FINGER_CLOSED, f"swing the seat up to {d:.0f} deg", 0.0) for d in SWING_UP]
    plan.append(Waypoint(1.0, _rail_tcp(SWING_UP[-1]), FINGER_CLOSED, "hold it up", 0.0))
    plan += [Waypoint(1.0, _rail_tcp(d), FINGER_CLOSED, f"lower it to {d:.0f} deg", 0.0)
             for d in (*SWING_UP[::-1][1:], 0.0)]
    plan.append(Waypoint(0.5, _rail_tcp(0), FINGER_CLOSED, "settle", 0.0))
    plan.append(Waypoint(1.0, _rail_tcp(0), WIDE, "let go of the rail", 0.0))
    plan.append(Waypoint(1.5, (RAIL_X, RAIL_Y, _rail_tcp(0)[2] + 0.070), WIDE, "up", 0.0))
    plan.append(Waypoint(1.5, (RAIL_X - 0.06, RAIL_Y + 0.06, _rail_tcp(0)[2] + 0.080), WIDE, "clear", 0.0))
    return plan


def demo():
    # DEMO_VARIANT selects a degenerate or check run (see README); unset runs the full-score demo
    v = os.environ.get("DEMO_VARIANT", "")
    if v == "unlatched":  # sockets set on the posts, not pushed down
        plans = {}
        for s in SIDES:
            p = _place(s) + [Waypoint(2.5, _place(s)[-1].pos, FINGER_CLOSED, "hold", 0.0)]
            plans[s] = p + _leave(p, s)
        return {"left": plans["L"], "right": plans["R"]}
    if v == "one_side":  # both sockets set on the posts; the right arm holds its side up while the left one is
        # pushed down, so only the left button clicks; then both let go and the right socket rests on its button
        left = _place("L") + _push("L", press=ONE_SIDE_PRESS)
        left += _leave(left, "L")
        right = _place("R")
        right.append(Waypoint(5.0, right[-1].pos, FINGER_CLOSED, "hold the right side up", 0.0))
        return {"left": left, "right": right + _leave(right, "R")}
    if v == "lift_seat":  # full fit, then the seat is swung up on its hinges and back down before letting go
        lift = _lift_seat()
        left = _place("L") + _push("L") + lift
        right = _place("R") + _push("R")
        wait = sum(w.duration for w in lift[:-1])  # hold the right socket until the left arm has let go and risen
        right.append(Waypoint(wait, right[-1].pos, FINGER_CLOSED, "hold while the seat is swung", 0.0))
        return {"left": left, "right": right + _leave(right, "R")}
    left, right = _place("L") + _push("L"), _place("R") + _push("R")
    return {"left": left + _leave(left, "L"), "right": right + _leave(right, "R")}


# -- offline self-test ------------------------------------------------------------------------------------------


def self_test():
    global _TEST_TCPS
    up = mk.mat_to_quat(np.eye(3))

    def roll(dz_r):  # left socket down on its clicked button, right one dz_r higher
        q = mk.mat_to_quat(mk.axis_angle((1, 0, 0), -math.atan2(dz_r, 2 * PY)))
        return (np.array([PX, 0.0, R + CLICK_Z + dz_r / 2]), q)

    home = (np.array([PX, 0.0, R + CLICK_Z]), up)
    tilt = mk.mat_to_quat(mk.axis_angle((0, 1, 0), -math.atan2(0.012, 0.37)))
    rest = (np.array([PX, 0.0, R + 0.012]), tilt)
    off = (np.array([PX - 0.010, 0.0, R + CLICK_Z]), up)
    on_top = (np.array([PX, 0.0, R + 0.036]), up)  # both housings sitting on the buttons' tops, not over the posts

    def st(p, swing=0.0):
        return kit.fake_states(q={"swing": math.radians(swing)}, moved={"hinge": p})

    near = [np.array([PX, PY, R + GRIP])]
    both = {"lock_L": True, "lock_R": True}
    cases = {
        "start": (kit.fake_states(), [], {}, 0.0),
        "home, both clicked": (st(home), [], both, 1.0),
        "home, both clicked, seat stood up on its hinges": (st(home, SWING_MAX), [], both, 1.0),
        "home, both clicked, still pinched": (st(home), near, both, 0.0),
        "home, only the left button clicked": (st(home), [], {"lock_L": True}, 0.7),
        "home, no button clicked": (st(home), [], {}, 0.4),
        "both resting on the buttons": (st(rest), [], {}, 0.4),
        "left clicked down, right resting on its button": (st(roll(0.010)), [], {"lock_L": True}, 0.7),
        "left clicked, both resting 12 mm up": (st(rest), [], {"lock_L": True}, 0.7),
        "left clicked down, right on its post top": (st(roll(0.034)), [], {"lock_L": True}, 0.5),
        "left on its post, right on its post top": (st(roll(0.034)), [], {}, 0.2),
        "both on the button tops": (st(on_top), [], {}, 0.0),
        "10 mm off the posts": (st(off), [], both, 0.0),
        "home after the seat fell off the bench": (st(home), [], {**both, "fell": True}, 0.0),
        "home, both clicked, still moving 2 cm/s": (st(home), [], {**both, "speed": 0.02}, 0.0),
        "home, both clicked, moving 5 mm/s": (st(home), [], {**both, "speed": 0.005}, 1.0),
    }
    try:
        for name, (s_, tcps, mem, want) in cases.items():
            mk.new_episode()
            _TEST_TCPS = tcps
            mk.MEMORY.update(mem)
            got = score(s_)
            assert abs(got - want) < 1e-9, (name, got, want)
        s0 = kit.fake_states()
        for s in SIDES:  # at the start each socket is clear of its post, ahead of it
            assert socket(s0, s)[0] + HOUS / 2 < PX - POST[0] / 2
        up100 = st(home, SWING_MAX)  # the swing joint reads back what it was set to
        assert abs(kit.q_deg(up100, "swing") - SWING_MAX) < 1e-6
    finally:
        mk.new_episode()
        _TEST_TCPS = None
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
