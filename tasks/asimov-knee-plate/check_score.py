"""The scorer on hand-built states, with no simulator: each rung, each rule and a case that fails it.

    python check_score.py

Every case is a run of per-step states fed to task.py's own Trial. Exits non-zero if any case scores other than the scoring text says.
"""

from __future__ import annotations

import importlib.util
import math
import os
import pathlib
import sys
import types

import numpy as np

os.environ["KNEE_TRACE"] = "0"
os.environ.pop("KNEE_GATE", None)
os.environ.pop("KNEE_LEG0", None)
HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import gatekit as gk  # noqa: E402

# stand-ins for the simulator side of the kit: task.py builds its props at import, and the scorer reads states through m.state and m.fingertips
dk = sys.modules["demokit"] = types.ModuleType("demokit")
dk.TABLE_TOP = 0.0
dk.CONTROL_HZ = 20
mk = sys.modules["mechkit"] = types.ModuleType("mechkit")
mk.Look = lambda *a, **kw: None
mk.rgb = lambda *a: a
class _Mesh(types.SimpleNamespace):
    def turned(self, *a, **kw):
        return self

    def moved(self, *a, **kw):
        return self


mk.Mesh = lambda v, f: _Mesh(v=np.asarray(v, float), f=np.asarray(f, int))
for fn in ("prism", "lathe", "box", "cylinder", "sphere", "join"):
    setattr(mk, fn, lambda *a, **kw: _Mesh(v=np.zeros((4, 3)), f=np.zeros((1, 3), int)))
for fn in ("col_hull", "col_cylinder", "col_box", "slider", "hinge", "mechanism", "free", "fixed", "on_step", "on_reset", "push"):
    setattr(mk, fn, lambda *a, **kw: None)
mk.state = lambda env, name: env[name]
mk.fingertips = lambda env: env["tips"]
spec = importlib.util.spec_from_file_location("knee_task", HERE / "task.py")
t = importlib.util.module_from_spec(spec)
spec.loader.exec_module(t)

P = gk.Pose


def leg(R=None, o=None, knee=0.0, vel=(0.0, 0.0, 0.0), spin=(0.0, 0.0, 0.0)):
    """The leg's state as the scorer reads it: the thigh's (the frame's) pose and the knee joint (radians, 0 straight, negative flexed)."""
    return P(t.O_BACK if o is None else o, t.R_BACK if R is None else R, lin_vel=vel, ang_vel=spin, joints={"knee_joint": knee})


def tipped(deg: float):
    """The leg turned `deg` back over its edge from its back toward its side (85.9 is the start)."""
    Rt = gk.rot(tuple(t.EDGE_W), deg)
    return leg(Rt @ t.R_BACK, t.EDGE_P + Rt @ (t.O_BACK - t.EDGE_P))


BACK, SIDE = leg(), tipped(t.TIP_DEG)


def plate(dx=0.0, dy=0.0, h=0.0, tilt=0.0, yaw=0.0, speed=0.0, flip=False, on=BACK):
    """The plate on the pocket of the leg `on`: dx, dy off the bearing's axis along the plate's x and y, h up from seated (its origin), tilt
    about its y, yaw about its axis."""
    R_pk, o = t.pocket(on)
    R = R_pk @ t._rz(yaw) @ gk.rot((0.0, 1.0, 0.0), tilt)
    if flip:
        R = R @ np.diag([1.0, -1.0, -1.0])
    return P(o + R_pk @ np.array([dx, dy, h]), R, lin_vel=(speed, 0.0, 0.0))


FAR = gk.tips()  # both hands parked far away
S_POST = P(t.PLATE0, t.R_POST)  # the start: boss down in the post's socket


def e(pl, tips=None, on=BACK, home=2, driven=None):
    return {"plate": pl, "tips": FAR if tips is None else tips, "leg": on, "home": home, "driven": home if driven is None else driven}


def hands_at(p, side="right"):
    return gk.tips(right=gk.hand(p)) if side == "right" else gk.tips(left=gk.hand(p))


LIFT = t.PLATE0 + [0, 0, 0.045]
PICK = [(2, e(S_POST, on=SIDE)), (12, e(P(LIFT, t.R_POST), hands_at(LIFT), on=SIDE))]  # a gripper lifts the plate 4.5 cm out of the post (the leg not yet turned)
TURN = [(5, e(S_POST, on=tipped(60.0))), (5, e(S_POST))]  # the leg turned onto its back


def run(phases):
    tr = t.TRIAL
    tr.reset()
    for n, env in phases:
        for _ in range(n):
            t.BOLTS.n_home = env["home"]  # the bolts driven home (their own rules are checked below, on Bolts itself)
            t.BOLTS.started = set(t.BOLT_HOLES[:env["driven"]])
            t.BOLTS.turns[:] = 0.0
            for k in t.BOLTS.started:
                t.BOLTS.turns[k] = t.HOME_TURNS
            tr.step(env)
    return tr.score()


bad = 0


def check(label, got, want):
    global bad
    ok = abs(got - want) < 1e-9
    bad += not ok
    print(f"{'ok ' if ok else 'BAD'} {got:.2f} (want {want:.2f})  {label}")


HOLD = t.HOLD_STEPS + 2
SEATED = plate()
R0, O0 = t.pocket(BACK)
up = lambda z: O0 + np.array([0.0, 0.0, z])  # noqa: E731
GO = PICK + TURN

# the rungs
check("doing nothing: the leg on its side plate away, the plate in the post's socket", run([(40, e(S_POST, on=SIDE))]), 0.0)
check("0.2: the leg turned onto its back, the plate left in the post", run(TURN + [(HOLD, e(S_POST))]), 0.2)
check("0.2: on its back with the knee bent 30 deg", run(TURN + [(HOLD, e(S_POST, on=leg(knee=-math.radians(30))))]), 0.2)
check("the leg turned 60 deg of its 86 (balanced on its edge), the plate in the post", run([(HOLD, e(S_POST, on=tipped(26.0)))]), 0.0)
check("the leg on its back but held 3 cm up off the table", run([(HOLD, e(S_POST, on=leg(o=t.O_BACK + [0, 0, 0.03])))]), 0.0)
check("the leg face down (turned 180 deg about its length)", run([(HOLD, e(S_POST, on=leg(R=t.R_BACK @ np.diag([1.0, -1.0, -1.0]), o=t.O_BACK - [0, 0, 0.05])))]), 0.0)
check("the leg stood on its end (calf down)", run([(HOLD, e(S_POST, on=leg(R=gk.rot((1.0, 0.0, 0.0), -90.0) @ t.R_BACK, o=t.O_BACK + [0, 0, 0.20])))]), 0.0)
check("0.2: on its back, the plate on the mouth 9 mm above the face", run(GO + [(HOLD, e(plate(h=0.009)))]), 0.2)
check("0.2: on its back, the plate cocked on the mouth (tilt 3 deg)", run(GO + [(HOLD, e(plate(h=0.0057, tilt=3.0)))]), 0.2)
check("0.2: the rim, pad and boss 3 mm in, not yet flush", run(GO + [(HOLD, e(plate(h=0.0035)))]), 0.2)
check("0.8: seated flush, a hand still at the plate", run(GO + [(HOLD, e(SEATED, hands_at(up(0.01))))]), 0.8)
check("0.8: seated flush but still moving (3 mm/s)", run(GO + [(HOLD, e(plate(speed=0.003)))]), 0.8)
check("0.2: flush with the far hole 1.7 mm off their inserts (turned 1 deg about the boss axis)", run(GO + [(HOLD, e(plate(yaw=1.0)))]), 0.2)
check("0.2: flush, no bolt in, turned 0.3 deg about the boss axis (the far hole 0.50 mm off its insert)", run(GO + [(HOLD, e(plate(yaw=0.3), home=0, driven=0))]), 0.2)
check("0.2: flush, no bolt in, turned 0.6 deg (hole 6, the nearest hole on the long axis, 0.48 mm off)", run(GO + [(HOLD, e(plate(yaw=0.6), home=0, driven=0))]), 0.2)
check("1.0: both bolts home, turned 0.3 deg: holes 8 and 7 are read by their bolts, the other seven within 0.45 mm", run(GO + [(HOLD, e(plate(yaw=0.3)))]), 1.0)
check("0.2: both bolts home, turned 0.6 deg: hole 6 0.48 mm off its insert", run(GO + [(HOLD, e(plate(yaw=0.6)))]), 0.2)
check("0.4: seated flush, hands clear, no bolt driven", run(GO + [(HOLD, e(SEATED, home=0, driven=0))]), 0.4)
check("0.6: seated, one bolt driven (not home), hands clear", run(GO + [(HOLD, e(SEATED, home=0, driven=1))]), 0.6)
check("0.8: seated, both bolts driven, neither home", run(GO + [(HOLD, e(SEATED, home=0, driven=2))]), 0.8)
check("0.8: seated, both bolts driven, one home", run(GO + [(HOLD, e(SEATED, home=1, driven=2))]), 0.8)
check("0.2: both bolts driven and home but the plate 2 mm proud (not seated)", run(GO + [(HOLD, e(plate(h=0.002)))]), 0.2)
check("1.0: seated flush, turned 0.25 deg about the boss axis (the far hole 0.42 mm off, the pin gauge still passes)", run(GO + [(HOLD, e(plate(yaw=0.25)))]), 1.0)
check("1.0: on its back, seated, holes over their inserts, hands clear, all still for 1 s", run(GO + [(HOLD, e(SEATED))]), 1.0)
kneed = leg(knee=-math.radians(20))
check("1.0: as above with the knee bent 20 deg (the pocket turns with the housing)", run(GO + [(HOLD, e(plate(on=kneed), on=kneed))]), 1.0)
check("0.8: seated, held for only 10 steps (under 1 s)", run(GO + [(10, e(SEATED))]), 0.8)

# the leg: not turned, half turned, lifted, knocked over, rocking, sliding
check("the leg not turned (on its side, plate away), the plate seated flush in its pocket, hands clear", run(PICK + [(HOLD, e(plate(on=SIDE), on=SIDE))]), 0.0)
check("the leg turned half way (43 deg), the plate seated flush in its pocket", run(PICK + [(HOLD, e(plate(on=tipped(43.0)), on=tipped(43.0)))]), 0.0)
check("the plate seated with the leg at 45 deg, then the leg turned onto its back (the order of the act reversed)", run(PICK + [(HOLD, e(plate(on=tipped(45.0)), on=tipped(45.0))), (HOLD, e(SEATED))]), 0.2)
check("seated and held 1 s, then the leg knocked 20 deg up off its back with the plate in it", run(GO + [(HOLD, e(SEATED)), (3, e(plate(on=tipped(20.0)), on=tipped(20.0)))]), 0.0)
check("seated with the leg rocking on its back (0.05 rad/s)", run(GO + [(HOLD, e(SEATED, on=leg(spin=(0.05, 0.0, 0.0))))]), 0.8)
check("seated with the leg sliding 5 mm/s", run(GO + [(HOLD, e(SEATED, on=leg(vel=(0.005, 0.0, 0.0))))]), 0.8)
slid = leg(o=t.O_BACK + [0.03, 0.0, 0.0])
check("the plate on the mouth, then pressed home while the leg slid 3 cm away under it (no hand on the leg)", run(GO + [(5, e(plate(h=0.008))), (5, e(plate(h=0.004, on=leg(o=t.O_BACK + [0.015, 0, 0])), on=leg(o=t.O_BACK + [0.015, 0, 0]))), (HOLD, e(plate(on=slid), on=slid))]), 0.2)
nudged = leg(o=t.O_BACK + [0.008, 0.0, 0.0])
check("1.0: the plate on the mouth, then pressed home while the leg shifted 8 mm (under 15)", run(GO + [(5, e(plate(h=0.008))), (HOLD, e(plate(on=nudged), on=nudged))]), 1.0)
check("1.0: the leg slid 3 cm while turning (no plate on it yet), then seated", run(PICK + [(5, e(S_POST, on=leg(o=t.O_BACK + [0.03, 0, 0])))] + [(5, e(S_POST))] + [(HOLD, e(SEATED))]), 1.0)

# the zeros and the failures
check("the plate flipped (boss up) lying on the face over the pocket", run(GO + [(HOLD, e(plate(h=0.011, flip=True)))]), 0.2)
check("the plate 3 mm off the bearing's axis, 3 mm above the face: not on the pocket", run(GO + [(HOLD, e(plate(dx=0.003, h=0.003)))]), 0.2)
check("the plate turned 90 deg to the pocket, its pad and boss in but the rim resting on the face (5 mm up)", run(GO + [(HOLD, e(plate(h=0.005, yaw=90.0)))]), 0.2)
check("the plate off the table (below its top), then lifted back seated", run(GO + [(3, e(P((0.3, 0.0, -0.05), t.R_POST))), (HOLD, e(SEATED))]), 0.0)
check("the plate past the table's edge, then back seated", run(GO + [(3, e(P((0.3, 0.7, 0.0), t.R_POST))), (HOLD, e(SEATED))]), 0.0)
off = leg(o=t.O_BACK + [0.50, 0.0, -0.10])
check("the leg pushed off the table's far edge", run(PICK + [(3, e(S_POST, on=off)), (HOLD, e(SEATED))]), 0.0)
check("the leg dropped off the table's side (below its top), then back on its back with the plate seated", run(PICK + [(3, e(S_POST, on=leg(o=t.O_BACK - [0, 0, 0.15]))), (HOLD, e(SEATED))]), 0.0)

# the lazy routes and the hands
check("seated but a hand 6 cm over it (under 10 cm)", run(GO + [(HOLD, e(SEATED, hands_at(up(0.06))))]), 0.8)
check("seated, the left hand 6 cm over it, the right far", run(GO + [(HOLD, e(SEATED, hands_at(up(0.06), "left")))]), 0.8)
check("seated, a hand 4 cm over the narrow end", run(GO + [(HOLD, e(SEATED, hands_at(O0 + R0 @ np.array([0.1085, 0.0, 0.04]))))]), 0.8)
check("seated, a hand 5 cm over the face's rim beside the pocket (holding the leg)", run(GO + [(HOLD, e(SEATED, hands_at(O0 + np.array([-0.055, 0.0, 0.05]), "left")))]), 0.8)
check("seated, a fingertip 2 cm over the calf's end (holding the leg)", run(GO + [(HOLD, e(SEATED, hands_at(O0 + R0 @ t.CALF + np.array([0.0, 0.0, 0.02]))))]), 0.8)
check("seated, a fingertip 2 cm over the thigh's end (holding the leg)", run(GO + [(HOLD, e(SEATED, hands_at(BACK.to_world(t.THIGH_END)[0] + np.array([0.0, 0.0, 0.02]), "left")))]), 0.8)
check("1.0: seated, hands 12 cm clear", run(GO + [(HOLD, e(SEATED, hands_at(up(0.13))))]), 1.0)

# more routes to 0.75 or 1.0 without the act
check("seated flush but no gripper ever lifted the plate from the post (thrown, tapped or dropped from far)", run(TURN + [(HOLD, e(SEATED))]), 0.2)
check("rose 4.5 cm out of the post with no fingertip near it (flicked), then seated", run([(2, e(S_POST)), (2, e(P(LIFT, t.R_POST))), (HOLD, e(SEATED))]), 0.2)
check("rose 4.5 cm out of the post with a hand 12 cm to the side only (a prod), then seated", run([(2, e(S_POST)), (12, e(P(LIFT, t.R_POST), hands_at(LIFT + [0.0, 0.12, 0.0]))), (HOLD, e(SEATED))]), 0.2)
check("seated and held 1 s, then pulled out 3 cm (the 1.0 does not stay; the leg still on its back)", run(GO + [(HOLD, e(SEATED)), (3, e(plate(h=0.03)))]), 0.2)
check("seated and held 1 s, then knocked 2 mm proud (not flush)", run(GO + [(HOLD, e(SEATED)), (3, e(plate(h=0.002)))]), 0.2)
check("the plate flipped (boss up) bridging the pocket, its outer face down on the housing's face", run(GO + [(HOLD, e(plate(flip=True)))]), 0.2)
check("seated and held 1 s, then a hand 1 cm over the plate (the 1.0 is read at the end)", run(GO + [(HOLD, e(SEATED)), (3, e(SEATED, hands_at(up(0.01))))]), 0.8)
check("seated and held 1 s, then the plate sliding 5 mm/s", run(GO + [(HOLD, e(SEATED)), (3, e(plate(speed=0.005)))]), 0.8)

# the loose leg: held, propped, shoved or rocked
mid_thigh = BACK.to_world(np.array([-0.18, 0.0, -0.02]))[0]  # on the thigh's top, half way along it
check("seated, a hand gripping the thigh's middle at the end (outside the plate's 10 cm and the ends' reach)", run(GO + [(HOLD, e(SEATED, hands_at(mid_thigh, "left")))]), 0.8)
propped = leg(o=t.O_BACK + [0, 0, 0.010])
check("seated, the leg held 10 mm up off the table by a hand on the thigh's middle", run(GO + [(HOLD, e(plate(on=propped), hands_at(propped.to_world(np.array([-0.18, 0.0, -0.02]))[0], "left"), on=propped))]), 0.8)
shoved = leg(o=t.O_BACK + [0.04, 0.0, 0.0])
check("the plate cocked on the mouth, lifted off it, the leg shoved 4 cm, then the plate seated (the anchor stays at the first mouth)", run(GO + [(5, e(plate(h=0.006, tilt=1.5))), (5, e(plate(h=0.03))), (5, e(plate(h=0.03, on=shoved), on=shoved)), (HOLD, e(plate(on=shoved), on=shoved))]), 0.2)
rocked = leg(R=gk.rot((1.0, 0.0, 0.0), 5.0) @ t.R_BACK)  # rolled 5 deg about its length, the face 7.6 deg from up
check("pressed home while the leg rocked 5 deg further over on its back (still within 10 of level)", run(GO + [(5, e(plate(h=0.004))), (5, e(plate(h=0.002, on=rocked), on=rocked)), (HOLD, e(SEATED))]), 0.2)
check("1.0: pressed home while the leg rocked 2 deg onto its other three points and back", run(GO + [(5, e(plate(h=0.004))), (5, e(plate(h=0.002, on=leg(R=gk.rot((1.0, 0.0, 0.0), 2.0) @ t.R_BACK)), on=leg(R=gk.rot((1.0, 0.0, 0.0), 2.0) @ t.R_BACK))), (HOLD, e(SEATED))]), 1.0)

# the bolts' own rules, on Bolts itself: a bolt carried in the plate lands on its insert only once the plate is down over it,
# and the thread advances only while the bit is in the socket, pressed with PUSH_MIN, and turned clockwise (the ratchet frees the other way)
B = t.BOLTS
B._place = lambda env, k: env["leg"].joints.__setitem__(f"seat{k}", t.START_S)
B._write = lambda env: None
t._torque = lambda *a: None
mk.set_visible = lambda *a: None
mk.ENV_PATH = "/env"
HB = P((t.HOLDER[0], t.HOLDER[1], t.TOP + 0.0005), np.eye(3))
RP, OP = t.pocket(BACK)
U = RP[:, 2]


def bleg(**seats):
    return P(t.O_BACK, t.R_BACK, joints={"knee_joint": 0.0, **{f"seat{k}": t.PARK for k in t.BOLT_HOLES}, **seats})


def drv(tip, R=np.eye(3)):
    return P(tip, R)


def benv(tip, R=np.eye(3), lg=None, pl=None):
    return {"driver": drv(tip, R), "leg": lg or bleg(), "bolts": HB, "plate": SEATED if pl is None else pl}


def floor(lg, k, up=0.0, side=0.0):
    return OP + RP @ np.array([t.HOLES[k, 0] + side, t.HOLES[k, 1], lg.joint(f"seat{k}") + t.FLOOR_UP + up])


def mouth_tip(k):
    return OP + RP @ np.array([t.HOLES[k, 0], t.HOLES[k, 1], -0.003]) + t.BOLT_L * U


def turn(lg, k, deg_total, push=7.0, side=0.0, up=0.0, step=10.0):
    """Turn the bit `deg_total` degrees (negative: clockwise from above, tightening), the seat pressed by `push` newtons."""
    n = int(abs(deg_total) / step)
    for i in range(n + 1):
        lg.joints[f"seat{k}"] = B.target(k) - push / t.SEAT_K
        th = math.copysign(step * i, deg_total)
        B.step(benv(floor(lg, k, up, side), RP @ t._rz(th), lg))


E0 = benv(np.array([0.0, 0.0, 1.0]))


def pick(i, lg=None):
    """The bit's tip set 0.5 mm into the socket of the bolt standing in the holder's slot i: the magnet takes it."""
    B.step(benv(B.head(E0, i) + [0.0, 0.0, 0.0005], np.eye(3), lg))


def start(k, pl=None, side=0.0, hover=True, slide=0.0):
    """The plate seated over its inserts: a bolt picked from the holder and set on hole k's first thread, square, starts there."""
    B.reset()
    lg = bleg()
    for i, kk in zip(t.PICK_SLOTS, t.BOLT_HOLES):
        pick(i, lg)
        for _ in range(t.START_STEPS if hover else 0):  # the bolt lined up over the hole first, then let down its axis
            B.step(benv(B.start_at(E0, kk) + (side + slide) * RP[:, 0] + 0.007 * RP[:, 2], RP, lg, pl))
        if slide:
            B.step(benv(B.start_at(E0, kk) + slide * RP[:, 0] + 0.001 * RP[:, 2], RP, lg, pl))  # brought in 1 mm over the thread, then slid sideways onto the axis
            B.step(benv(B.start_at(E0, kk) + 0.001 * RP[:, 2], RP, lg, pl))
        B.step(benv(B.start_at(E0, kk) + side * RP[:, 0], RP, lg, pl))
    return lg


lg = start(8)
check("bolts: the plate seated, a bolt picked from the holder and set on each of holes 8 and 7 starts there (2 = both started)", float(len(B.started)), 2.0)
start(8, hover=False)
check("bolts: a carried bolt put on the first thread without first hanging 6 mm over the hole on its axis: no bolt starts", float(len(B.started)), 0.0)
start(8, slide=0.005)
check("bolts: a carried bolt hung 5 mm aside of the hole, then slid onto its axis 1 mm over the thread and let down: no bolt starts", float(len(B.started)), 0.0)
start(8, pl=plate(h=0.008))
check("bolts: the plate held 8 mm over seated: no bolt starts on its insert", float(len(B.started)), 0.0)
start(8, side=0.002)
check("bolts: the carried bolt set 2 mm off the hole's axis: no bolt starts", float(len(B.started)), 0.0)
B.reset()
B.step(benv(B.head(E0, 2) + [0.003, 0.0, 0.0005]))
check("bolts: the bit 3 mm off the holder bolt's socket: nothing picked", float(B.carried is not None), 0.0)
lg = start(8)
turn(lg, 8, -360.0 * 9.2)
lg.joints["seat8"] = -0.0003
B.step(benv(floor(lg, 8), RP, lg))
check(f"bolts: 9 turns clockwise then tightened, pressed 7 N in the socket: home (turns {B.turns[8]:.2f}, {t.SNUG_K * B.snug[8]:.2f} N m)", float(B.n_home), 1.0)
lg = start(8)
turn(lg, 8, -360.0 * 9.0 - 10.0)
check(f"bolts: head down after 9 turns but only 10 deg of tightening ({t.SNUG_K * B.snug[8]:.2f} N m, under {t.TIGHT_T}): not home", float(B.n_home), 0.0)
lg = start(8)
turn(lg, 8, 0.0, step=1.0)
for _ in range(400):
    turn(lg, 8, 0.0)
check("bolts: pressed 7 N in the socket for 400 steps, never turned: no advance", B.turns[8], 0.0)
c15, s15 = math.cos(math.radians(15.0)), math.sin(math.radians(15.0))
RX15 = np.array([[1.0, 0.0, 0.0], [0.0, c15, -s15], [0.0, s15, c15]])
lg = start(8)
for i in range(int(360 * 9.2 / 10) + 1):
    lg.joints["seat8"] = B.target(8) - 7.0 / t.SEAT_K
    B.step(benv(floor(lg, 8), RP @ RX15 @ t._rz(-10.0 * i), lg))
check("bolts: pressed and turned with the bit 15 deg off the bolt's axis: no advance", B.turns[8], 0.0)
B.reset()
B.step(benv(np.array([0.0, 0.0, 1.0]), pl=plate(tilt=4.0)))
check("bolts: the plate seated 4 deg askew (a bolt would cross-thread): no bolt starts", float(len(B.started)), 0.0)
lg = start(8)
turn(lg, 8, -360.0 * 9.2, push=0.0)
check("bolts: the same turns with the bit resting in the socket unpressed (0 N): no advance", B.turns[8], 0.0)
lg = start(8)
turn(lg, 8, -360.0 * 9.2, push=3.0)
check("bolts: pressed 3 N (under PUSH_MIN 5): no advance", B.turns[8], 0.0)
lg = start(8)
turn(lg, 8, -360.0 * 9.2, side=0.002)
check("bolts: pressed 7 N with the tip 2 mm off the socket's centre: no advance", B.turns[8], 0.0)
lg = start(8)
turn(lg, 8, 360.0 * 9.2)
check("bolts: turned anticlockwise (the ratchet's free way): no advance", B.turns[8], 0.0)
lg = start(8)
turn(lg, 8, -360.0 * 9.2, up=0.004)
check("bolts: pressed and turned 4 mm above the socket's floor: no advance", B.turns[8], 0.0)
lg = start(8)
turn(lg, 8, -360.0 * 9.2)
lg.joints["seat8"] = 0.002
B.step(benv(floor(lg, 8, up=0.01), RP, lg))
check("bolts: 9 turns counted but the head held 2 mm proud of the face: not home", float(B.n_home), 0.0)

run(GO + [(HOLD, e(SEATED))])
ok1 = t.success(None)
run(TURN + [(HOLD, e(SEATED))])
ok2 = t.success(None)
print(f"{'ok ' if ok1 and not ok2 else 'BAD'} success(): seated after a lift {ok1}, seated without a lift {ok2}")
bad += 0 if ok1 and not ok2 else 1

print("FAILED" if bad else "all cases as the scoring text says")
sys.exit(1 if bad else 0)
