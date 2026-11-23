"""Open a beverage can: lift its stay-on ring-pull tab until the score line tears and the tab stands up.

A standard 330 ml aluminium can (66 mm across, 115 mm tall, 202 end) stands full on the station table at one fixed
place, its ring-pull tab riveted flat on the end with the ring pointing to the left arm side (+y). The tab lies 1.5 mm over the
end panel; a finger well under the ring lets a fingertip get under the ring's far bar. Lifting the ring about the
rivet needs about 15 N at its tip to tear the score line (tabkit.py), more than the can's weight, so the can has to
be held down by the other hand while the ring is lifted.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from scenekit import Shape, UsdBody, on_reset, on_step  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

import tabkit  # noqa: E402
import touch  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0

INSTRUCTION = "Open the can."

MM = 0.001
T = TABLE_TOP[SCENE]
Z0 = T + 0.0005

CAN_XY = (0.300, -0.050)
CAN_H = 0.120
CAN = UsdBody("can", usd="assets/can.usda", pos=(CAN_XY[0], CAN_XY[1], Z0), size=(0.066, 0.066, CAN_H),
              articulation=True, material=False, solver_iters=64, joint_stiffness=tabkit.K, joint_damping=tabkit.DAMPING,
              joint_pos=((tabkit.JOINT, 0.0),))

# the tab lifter (assets/opener.usda) stands like a screwdriver in a rack in front of the right arm, its tongue
# pointing +x; its origin is the tongue's underside on the plate's centre line
GREY = (0.35, 0.36, 0.38)
RACK_XY = (0.220, -0.170)
RACK_FLOOR, RACK_H = 0.003, 0.028
SLOT_X0, SLOT_X1, SLOT_Y = -0.005, 0.009, 0.005  # the slot: x from the tool's origin, and half-width in y
OPENER = UsdBody("opener", usd="assets/opener.usda", pos=(RACK_XY[0], RACK_XY[1], T + RACK_FLOOR + 0.0003),
                 size=(0.016, 0.016, 0.12), mass=0.03, material=False, torsional_patch=0.012)
_w = 0.012
_L = 0.014 + 2 * _w
RACK = [
    Shape("rack_floor", pos=(RACK_XY[0] + 0.002, RACK_XY[1], T + RACK_FLOOR / 2), size=(_L, 2 * SLOT_Y + 2 * _w, RACK_FLOOR), color=GREY),
    Shape("rack_wall_y0", pos=(RACK_XY[0] + 0.002, RACK_XY[1] - SLOT_Y - _w / 2, T + RACK_H / 2), size=(_L, _w, RACK_H), color=GREY),
    Shape("rack_wall_y1", pos=(RACK_XY[0] + 0.002, RACK_XY[1] + SLOT_Y + _w / 2, T + RACK_H / 2), size=(_L, _w, RACK_H), color=GREY),
    Shape("rack_wall_x0", pos=(RACK_XY[0] + SLOT_X0 - _w / 2, RACK_XY[1], T + RACK_H / 2), size=(_w, 2 * SLOT_Y, RACK_H), color=GREY),
    Shape("rack_wall_x1", pos=(RACK_XY[0] + SLOT_X1 + _w / 2, RACK_XY[1], T + RACK_H / 2), size=(_w, 2 * SLOT_Y, RACK_H), color=GREY),
]
OBJECTS = [CAN, OPENER, *RACK]

# -- scoring ---------------------------------------------------------------------------------------------------
TAB_1, TAB_2, TAB_3 = (math.radians(a) for a in (20.0, 35.0, 45.0))
UPRIGHT = math.radians(10.0)  # at the end the can's axis within this of vertical
HEIGHT_TOL = 0.003  # and standing on the table: its foot within this of where it started
_STATE: dict = {"tab": None, "free": True, "t": 0.0}


def tab_angle(c) -> float:
    return float(c.joints.get(tabkit.JOINT, 0.0))


def tilt(c) -> float:
    return math.acos(max(-1.0, min(1.0, float(c.rotation()[2, 2]))))


def score(objects) -> float:
    """0: the tab is lifted less than 20 degrees (the score line has not torn).
    0.25: lifted 20 degrees or more; 0.5: 35 or more.
    0.75: lifted 45 degrees or more (the can open), but the can tipped over (axis more than 10 degrees off vertical), off
    the table or still held.
    1.0: lifted 45 degrees or more, the can standing upright on the table and let go of."""
    c = objects["can"]
    a = tab_angle(c)
    if a < TAB_1:
        return 0.0
    if a < TAB_2:
        return 0.25
    if a < TAB_3:
        return 0.5
    standing = tilt(c) < UPRIGHT and abs(float(c.pos[2]) - Z0) < HEIGHT_TOL
    return 1.0 if standing and _STATE["free"] else 0.75


def success(objects) -> bool:
    return score(objects) == 1.0


@on_reset
def _reset(environment) -> None:
    can = environment.unwrapped.scene["can"]
    if _STATE["tab"] is None or _STATE["tab"].can is not can:
        _STATE["tab"] = tabkit.Tab(can)
    else:
        _STATE["tab"].reset()
    touch.reset()
    _STATE.update(free=True, t=0.0)


@on_step
def _watch(environment) -> None:
    t = touch.touching(environment, ["can", "can/tab"])
    _STATE["free"] = not any(t.values())
    _STATE["t"] += environment.unwrapped.step_dt


def Rx(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def demo():
    from yam_arena.constants import FINGER_CLOSED as C
    from yam_arena.constants import FINGER_OPEN as O
    from yam_arena.demo import Waypoint as W

    import tiltkit

    tiltkit.install()
    E = lambda k, d: float(os.environ.get(k, d))  # noqa: E731
    cx, cy = CAN_XY
    NN, GN = -0.030, -0.004  # opened to 60 mm to go round the handle (16 mm); closed on it, letting the pads give
    right, left = [], []

    def add(arm, t, pos, finger, label, app=(0, 0, -1), close=(0, -1, 0), joint=False):
        arm.append(W(t, tuple(float(v) for v in pos), finger, label, tiltkit.grip(app, close, joint=joint)))

    right.append(W(2.0, (0.20, -0.17, T + 0.22), O, "start"))
    left.append(W(2.0, (0.20, 0.20, T + 0.22), O, "start"))

    # 1. the left hand takes the can by its body from the robot's side (fingers pointing +x, tipped down 40 degrees)
    HZ = E("HOLD_Z", 0.060)
    he = math.radians(E("HOLD_DEG", 20))
    hap = (math.cos(he), 0, -math.sin(he))
    hold = np.array([cx, cy, Z0 + HZ])
    back = np.array([-math.cos(he), 0, math.sin(he)]) * 0.10
    add(left, 2.5, hold + back, O, "left hand beside the can", app=hap, close=(0, -1, 0), joint=True)
    add(left, 2.0, hold, O, "to the can", app=hap, close=(0, -1, 0))
    add(left, 1.0, hold, C, "hold the can down", app=hap, close=(0, -1, 0))

    # 2. the right hand takes the tab lifter by its handle, PINCH over the tongue's underside
    PINCH = E("PINCH", 0.045)
    HX = 0.00375  # the pinch is this far from the tongue's plate (+x), over the bar's contact
    rx, ry = RACK_XY
    rest = np.array([rx + HX, ry, T + RACK_FLOOR + 0.0003 + PINCH])
    up = np.array([0.0, 0.0, 0.07])
    add(right, 2.0, rest + up, NN, "over the lifter's handle", joint=True)
    add(right, 1.5, rest, NN, "down beside the handle")
    add(right, 1.0, rest, C, "pinch the handle")
    add(right, 1.5, rest + (0, 0, 0.09), C, "lift it out of the rack")
    # 3. over the ring's hole, the plate down through it, the tongue slid under the ring's side bar
    HY = E("HOOK_LY", 0.0135)  # the plate's place along the ring from the rivet (m)
    hy = cy + HY  # world y of the hole's middle
    ins = E("HOOK_IN", 0.0010)  # the plate's centre across the ring when it goes down (world x offset)
    sl = E("HOOK_SLIDE", 0.0055)
    zf = Z0 + 0.1115  # the tongue's underside when it is in the well (2.5 mm under the end panel's top)
    add(right, 3.0, np.array([cx + HX + ins - 0.06, hy - 0.06, zf + 0.07 + PINCH]), C, "carry it over the can")
    add(right, 2.0, np.array([cx + HX + ins, hy, zf + 0.020 + PINCH]), C, "over the hole")
    add(right, 2.0, np.array([cx + HX + ins, hy, zf + PINCH]), C, "plate down through the hole")
    add(right, 1.5, np.array([cx + HX + ins + sl, hy, zf + PINCH]), C, "slide the tongue under the side bar")
    p0 = np.array([cx + HX + ins + sl, hy, zf + PINCH + E("SLACK", 0.020)])
    add(right, 1.0, p0, C, "take up the slack")
    # 4. lift the ring about the rivet, the lifter turning with it
    riv = np.array([cx, cy - 0.001, Z0 + 0.115])
    ARC_MAX = E("ARC_MAX", 90)
    for deg in [d for d in range(10, int(ARC_MAX), 10)] + [ARC_MAX]:
        a = math.radians(deg)
        p = riv + Rx(a) @ (p0 - riv)
        add(right, E("STEP_T", 2.5), p, C, f"tab to {deg} degrees", app=tuple(Rx(a) @ (0, 0, -1)), close=tuple(Rx(a) @ (0, -1, 0)))
    A = math.radians(ARC_MAX)
    pe = riv + Rx(A) @ (p0 - riv)
    papp, pcl = tuple(Rx(A) @ (0, 0, -1)), tuple(Rx(A) @ (0, -1, 0))
    add(right, 1.5, pe, C, "tab up", app=papp, close=pcl)
    add(right, 3.5, pe, C, "hold it up while the left hand lets go of the can", app=papp, close=pcl)
    d = sum(w.duration for w in right) - sum(w.duration for w in left)
    if d > 0.05:
        left.append(W(d - 3.0, left[-1].pos, left[-1].finger, "wait", left[-1].yaw))
        add(left, 1.2, hold, O, "open the fingers round the can", app=hap, close=(0, -1, 0))
        add(left, 1.8, hold + back, O, "back away", app=hap, close=(0, -1, 0))
    return {"left": left, "right": right}


def self_test(ObjectState=None):
    from scenekit import BodyState

    def can(a=0.0, tilt_deg=0.0, dz=0.0):
        t = math.radians(tilt_deg)
        q = np.array([math.sin(t / 2), 0.0, 0.0, math.cos(t / 2)])
        pos = np.array([CAN_XY[0], CAN_XY[1], Z0 + dz])
        return BodyState(pos=pos, quat_xyzw=q, lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=pos, high=pos,
                         joints={tabkit.JOINT: math.radians(a)})

    cases = {
        "shut": (can(0), True, 0.0),
        "lifted 15": (can(15), True, 0.0),
        "lifted 25": (can(25), True, 0.25),
        "lifted 50, let go": (can(50), True, 1.0),
        "lifted 40": (can(40), True, 0.5),
        "lifted 45, let go": (can(45), True, 1.0),
        "lifted 85, let go": (can(85), True, 1.0),
        "lifted 85, still held": (can(85), False, 0.75),
        "lifted 85, tipped over": (can(85, tilt_deg=60), True, 0.75),
        "lifted 85, off the table": (can(85, dz=0.05), True, 0.75),
    }
    for name, (obj, free, want) in cases.items():
        _STATE["free"] = free
        got = score({"can": obj})
        assert abs(got - want) < 1e-9, (name, got, want)
    _STATE["free"] = True
    s = 0.0
    for a in np.linspace(0.0, math.radians(60), 300):
        s = tabkit.plastic_step(s, a, 0.0 if a < tabkit.TORN else a)
    assert s > math.radians(50), s
    assert tabkit.BREAK / 0.020 < 20.0
    return len(cases) + 2


if __name__ == "__main__":
    print(f"self-test: {self_test()} cases pass")
