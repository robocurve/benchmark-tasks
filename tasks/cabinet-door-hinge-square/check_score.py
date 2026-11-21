"""The scorer on hand-built trials, with no simulator: python3 check_score.py.

Loads task.py with stand-ins for the harness's modules, then feeds its real Trial step by step with made-up states (the honest run and the lazy
routes) and checks each score against the scoring text. The door's gaps come from the task's own law (`gaps_from`) on the screws' turns, so a turn of
the wrong screw or the wrong way shows in the gaps as it would in the scene. Nothing here runs a simulation.
"""

from __future__ import annotations

import dataclasses
import importlib.util
import math
import pathlib
import sys
import types

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent

ya = types.ModuleType("yam_arena")
tf = types.ModuleType("yam_arena.task_file")


@dataclasses.dataclass(frozen=True)
class Box:
    name: str
    pos: tuple = (0.0, 0.0, 0.0)
    size: tuple = (0.1, 0.1, 0.1)
    mass: float | None = None
    yaw_deg: float = 0.0
    start_quat: tuple | None = None
    solver_iters: int = 32


@dataclasses.dataclass(frozen=True)
class ObjectState:
    pos: np.ndarray
    quat_xyzw: np.ndarray
    lin_vel: np.ndarray
    ang_vel: np.ndarray
    low: np.ndarray
    high: np.ndarray


tf.Box, tf.ObjectState, tf.TABLE_TOP = Box, ObjectState, {"station": 0.7475}
ya.task_file = tf
sys.modules["yam_arena"], sys.modules["yam_arena.task_file"] = ya, tf
sys.path.insert(0, str(HERE))
spec = importlib.util.spec_from_file_location("task", HERE / "task.py")
T = importlib.util.module_from_spec(spec)
spec.loader.exec_module(T)
m = T.m
TOP = T.TOP


def Rz(deg):
    a = math.radians(deg)
    return np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1.0]])


class Part:
    def __init__(self, pos, R, joints=None, v=(0, 0, 0), w=(0, 0, 0)):
        self.pos, self.R, self.joints = np.asarray(pos, float), np.asarray(R, float), joints or {}
        self.lin_vel, self.ang_vel = np.asarray(v, float), np.asarray(w, float)

    def rotation(self):
        return self.R

    def joint(self, k):
        return self.joints[k]

    def deg(self, k):
        return math.degrees(self.joints[k])

    def to_world(self, p):
        return np.asarray(p, float) @ self.R.T + self.pos

    def to_local(self, p):
        return (np.asarray(p, float) - self.pos) @ self.R


S = {}  # the made-up scene for the next step


def door_pose():
    """The left door's link pose: turned open about its hinge line."""
    R = Rz(S["swing"])
    hinge = np.array([T.XH, T.YH, 0.0])
    return R, hinge + T.OFF - R @ hinge


def gaps():
    """The gaps (bottom, top) by the task's own law on the screws' turns when shut; wide open otherwise."""
    return T.gaps_from(T.THREAD.s("t"), T.THREAD.s("b")) if abs(S["swing"]) < 1.0 else (0.09, 0.09)


def tool_at_screw():
    """The driver seated in the top screw, along its axis."""
    R = np.column_stack([[0, -1.0, 0], [1.0, 0, 0], [0, 0, 1.0]])
    tip = T.HEAD["t"]
    return Part(tip - R @ np.array([T.BIT, 0, 0]), R)


DRIVER_HOME = Part(T.SD_START + [0, 0, TOP], Rz(T.SD_YAW))
FAR = np.array([[0.10, 0.25, TOP + 0.35]] * 24)
FAR_BODY = np.array([[0.10, 0.25, TOP + 0.35]] * 30)


def hands_on(p):
    return np.array([p] * 24), np.array([p] * 30)


m.state = lambda env, name: {"door": Part((0, 0, 0), np.eye(3), {"swing": math.radians(S["swing"])}, w=(0, 0, S.get("door_w", 0.0))), "driver": S["driver"]}[name]
m.fingertips = lambda env: S["tips"]
m.hand_body = lambda env: S["body"]
T.link_pose = lambda env, link: door_pose()
T.door_gaps = lambda env: gaps()


def reset():
    T.THREAD.reset()
    T.TRIAL.reset()


def run(steps, swing=0.0, tips=None, body=None, driver=None, turn=None, k="t", stripped=False):
    """`steps` steps of one made-up state; `turn` turns of screw `k` spread over them."""
    for _ in range(steps):
        if turn:
            T.THREAD.turns[k] += turn / steps
        if stripped:
            T.THREAD.stripped = True
        S.update(swing=swing, tips=FAR if tips is None else tips, body=FAR_BODY if body is None else body, driver=driver or DRIVER_HOME)
        T.TRIAL.step(None)
    return T.TRIAL.score


SEATED = tool_at_screw()
AT_SCREW = hands_on(SEATED.to_world([[0.05, 0, 0]])[0])
AT_PULL = hands_on(np.array([T.DX0 - 0.03, T.LY1 - 0.03 + T.DY, TOP + 0.255]))


def honest(turns=0.63, k="t"):
    reset()
    run(20, swing=0.0, tips=AT_PULL[0], body=AT_PULL[1])          # the door taken by its pull
    run(20, swing=45.0, tips=AT_PULL[0], body=AT_PULL[1])         # open
    run(30, swing=45.0, tips=AT_SCREW[0], body=AT_SCREW[1], driver=SEATED, turn=turns, k=k)  # the screw turned
    run(20, swing=0.0, tips=AT_PULL[0], body=AT_PULL[1])          # shut by the pull
    run(40)                                                       # driver back on its block, hands away
    return T.TRIAL.score


CASES = []


def case(name, got, want, text):
    CASES.append((name, got, want, text))


gb0, gt0 = T.gaps_from(T.ST0, T.SB0)
print(f"start gaps: bottom {1000 * gb0:.2f} mm, top {1000 * gt0:.2f} mm")
for tt in (0.28, 0.63, 0.99):
    gb, gt = T.gaps_from(T.ST0 + tt * T.PITCH, T.SB0)
    print(f"after {tt:.2f} turns of the top screw: bottom {1000 * gb:.2f} mm, top {1000 * gt:.2f} mm")

case("honest: top screw 0.63 turns, shut, let go", honest(), 1.0, "1.0")
case("honest, short: 0.35 turns", honest(0.35), 1.0, "1.0 (gaps 2.0 to 3.0, within 0.5)")
case("bottom screw turned instead", honest(0.63, "b"), 0.0, "turned the bottom screw: gaps wrong, the top screw never turned: 0")
case("top screw the wrong way", honest(-0.63), 0.0, "0")
case("too far: 1.3 turns", honest(1.3), 0.25, "gaps out of the window: 0.25")
reset()
run(20, swing=45.0, tips=AT_PULL[0], body=AT_PULL[1])
run(30, swing=45.0, tips=AT_SCREW[0], body=AT_SCREW[1], driver=SEATED)
case("hover: seated, never turned, door shut", run(40), 0.0, "0")
reset()
run(20, swing=45.0)
case("door only: opened and shut, nothing turned", run(40), 0.0, "0")
reset()
run(30, swing=45.0, tips=AT_SCREW[0], body=AT_SCREW[1], driver=SEATED, turn=0.63)
case("open end: turned, door left open", run(40, swing=45.0), 0.25, "0.25")
reset()
honest()
case("opened again after the end state", run(40, swing=30.0), 0.25, "0.25: the tiers read the state at the end")
reset()
honest()
run(1, tips=AT_PULL[0], body=AT_PULL[1])
case("hand left on the pull", run(40, tips=AT_PULL[0], body=AT_PULL[1]), 0.5, "0.5")
reset()
honest()
palm = np.array([T.DX0 - 0.02, 0.10 + T.DY, TOP + 0.25])  # fingers 6 cm off the door, the palm on its face
case("palm on the door, fingertips clear", run(40, tips=np.array([palm - [0.08, 0, 0]] * 24), body=np.array([palm] * 30)), 0.5, "0.5: the whole gripper must be 30 mm clear")
reset()
honest()
case("driver left in the screw, hands clear", run(40, driver=SEATED), 0.5, "0.5: the driver tip must be out of every screw")
reset()
honest()
case("screw forced past its stop", run(40, stripped=True), 0.0, "gate: 0")

# -- the screw rule itself: Thread.step on made-up driver poses, pushes and turns ------------------------------------
class _Asset:
    def set_joint_position_target(self, *a, **k):
        pass


m._joint = lambda env, part, link: (_Asset(), 0)
m._tensor = lambda asset, v: v
T._torque = lambda env, part, tau: None


def screw_turns(push=15.0, tilt_deg=0.0, off=(0.0, 0.0, 0.0), short=0.0, roll_turns=0.6, steps=60):
    """The driver's tip on the top head (or off it), pushed with `push` N, turned `roll_turns` about its own axis: the turns the screw takes."""
    T.THREAD.reset()
    a = math.radians(tilt_deg)
    axis = np.array([-math.sin(a), -math.cos(a), 0.0])  # tilted toward the cabinet's front, as the driver comes in
    tip = T.HEAD["t"] + np.asarray(off) - short * T.EIN
    for i in range(steps + 1):
        r = 2 * math.pi * roll_turns * i / steps
        side0 = np.cross([0.0, 0.0, 1.0], axis)
        side0 /= np.linalg.norm(side0)
        up0 = np.cross(axis, side0)
        side = math.cos(r) * up0 + math.sin(r) * side0  # turned about the axis: what _roll reads
        R = np.column_stack([axis, side, np.cross(axis, side)])
        drv = Part(tip - R @ np.array([T.BIT, 0, 0]), R)
        seat = -push / T.SEAT_K  # the seat pressed toward -y
        m.state = lambda env, name, drv=drv, seat=seat: {"door": Part((0, 0, 0), np.eye(3), {"seat_t": seat, "seat_b": 0.0, "swing": 0.0}), "driver": drv}[name]
        T.THREAD.step(None)
    return abs(T.THREAD.turns["t"])


case("rule: seated on axis, 15 N, turned 0.6", float(screw_turns() > 0.55), 1.0, "the screw turns with the driver")
case("rule: 14 deg off the axis, 15 N", float(screw_turns(tilt_deg=14.0) > 0.55), 1.0, "within 15 deg: turns")
case("rule: 20 deg off the axis", float(screw_turns(tilt_deg=20.0) > 0.0), 0.0, "cams out: no turn")
case("rule: pushed 5 N only", float(screw_turns(push=5.0) > 0.0), 0.0, "under 10 N: no turn")
case("rule: hover-turn, 1.5 mm short, no push", float(screw_turns(push=0.0, short=0.0015) > 0.0), 0.0, "no turn")
case("rule: tip 2.8 mm off the recess centre, 15 N", float(screw_turns(off=(0.0, 0.0, 0.0028)) > 0.0), 0.0, "unseated: no turn")

bad = 0
for name, got, want, text in CASES:
    ok = abs(got - want) < 1e-9
    bad += not ok
    print(f"{'ok ' if ok else 'BAD'} {got:.2f} (want {want:.2f}) {name}: {text}")
print("all match the scoring text" if not bad else f"{bad} case(s) differ")
sys.exit(1 if bad else 0)
