"""Lift the steeped tea infuser out of a glass teapot: lid off, basket out by its handle bar, shaken over the pot, onto a saucer.

On the station table stand a 1 L-class borosilicate glass teapot (150 mm body, mouth 96 mm across, spout and handle out to
242 mm), its glass lid on, and a stainless steel infuser basket (88 mm across, 140 mm tall, a 2.4 mm bar across its top as
the handle, whose top sits 4 mm below the pot's rim) resting on the pot's floor, holding the wet tea leaves (about 200
leaf pieces). To the left stands a 142 mm glass saucer.

The instruction asks for the infuser to come out of the pot: take the lid off by its knob and set it down, pinch the bar
and lift the basket clear (it clears the 96 mm mouth by 4 mm a side), shake it up and down three times over the pot to shed the drips, set it on the
saucer with the leaves still in it, and put the lid back on.

Simulation: the teapot, lid, infuser and saucer are NVIDIA USDCraft-10K models (Apache-2.0, ds_R089_03 and ds_R086_06)
split into separate bodies by scripts/make_assets.py; the leaves are rigid spheres (soilkit); the water is not
simulated (the pot's mass includes 500 g of it); shedding drips is a rule (three quick strokes of the basket over the pot, clear of its rim; see ``Shake``).

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
Demo:    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py -E object_jitter=0 ...
Offline: python3 task.py   (the scoring rules and the demo's kinematics, no GPU)
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

import soilkit  # noqa: E402
from scenekit import UsdBody, _state, on_reset, on_step  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0

INSTRUCTION = ("Take the steeped infuser out of the teapot: lid off, basket out by its handle bar, shaken up and down three "
               "times over the pot to shed the drips, set on the saucer, then the lid back on the pot.")

MM = 0.001
T = TABLE_TOP[SCENE]
Z0 = T + 0.001  # every body's origin sits on the table, 1 mm up

# -- the teapot (origin at the centre of its foot; spout towards local -x to 110 mm, handle towards +x to 132 mm),
# turned 90 degrees so its handle points to the robot's left and its spout to the right.
POT_XY = (float(os.environ.get("POT_X", "0.360")), float(os.environ.get("POT_Y", "0.0")))
POT_H = 0.150  # to the rim
MOUTH_R = 0.048  # the mouth's inside radius
POT_YAW = math.pi / 2
POT_Q = (0.0, 0.0, math.sin(POT_YAW / 2), math.cos(POT_YAW / 2))
TEAPOT = UsdBody("teapot", usd="assets/teapot.usdc", pos=(*POT_XY, Z0), rot_xyzw=POT_Q, size=(0.242, 0.15, POT_H), mass=0.90,
                 friction=(0.6, 0.5), torsional_patch=0.006, solver_iters=32)
# -- the lid (105 mm across, its knob 27 mm across, top 180 mm over the foot), loose on the rim
LID_SEAT = 0.1495  # its underside over the pot's foot when seated
LID = UsdBody("lid", usd="assets/lid.usdc", pos=(*POT_XY, Z0 + 0.003), size=(0.105, 0.105, 0.031), mass=0.08,
              friction=(0.6, 0.5), torsional_patch=0.006, solver_iters=32)
# -- the infuser: its bar runs along the table's x. Origin on the pot's axis; its floor's underside 6 mm up, its bar's
# top 145.7 mm up (bar 86.7 long, 2.4 thick, 10.5 tall).
BAR_TOP, BASKET_FOOT = 0.1457, 0.006
BAR_MID = 0.1405  # the pinch height on the bar, over the basket's origin
BASKET_R = 0.0441
INFUSER = UsdBody("infuser", usd="assets/infuser.usdc", pos=(*POT_XY, Z0 + 0.003), size=(0.088, 0.088, 0.14), mass=0.139,
                  friction=(0.6, 0.5), torsional_patch=float(os.environ.get("INF_TORSION", "0.020")), solver_iters=32)
# -- the saucer (142 mm across, 13.8 mm tall), to the left
SAUCER_XY = (float(os.environ.get("SAUCER_X", "0.300")), float(os.environ.get("SAUCER_Y", "0.230")))
SAUCER_H = 0.0138
SAUCER = UsdBody("tea_saucer", usd="assets/saucer.usdc", pos=(*SAUCER_XY, Z0), size=(0.142, 0.142, SAUCER_H), mass=0.143,
                 friction=(0.7, 0.6), solver_iters=16)
# -- where the demo parks the lid
PARK_XY = (0.300, -0.220)
# -- the wet leaves: about 200 rigid pieces (r 2.5-3.5 mm, 1000 kg/m3: about 23 g, the wet weight of 8 g of dry leaf for a 1 L pot) in the basket
LEAVES = soilkit.Soil("leaves", pos=(*POT_XY, Z0 + 0.03), size=(0.07, 0.07, 0.04), r_min=0.0025, r_max=0.0035,
                      density=1000.0, friction=(0.7, 0.6), angular_damping=2.0,
                      colors=((0.20, 0.30, 0.08), (0.26, 0.34, 0.10), (0.16, 0.24, 0.06), (0.30, 0.36, 0.12)),
                      start_file=os.environ.get("LEAVES_START", "assets/leaves_start.npz"))
OBJECTS = [TEAPOT, LID, INFUSER, SAUCER, LEAVES]

# -- scoring -----------------------------------------------------------------------------------------------------------
SHAKE_AMP, SHAKE_N, SHAKE_SPAN = 0.02, 3, 2.0  # a shake: 3 down-up strokes of at least 2 cm, the 3rd ending within 2 s of the 1st
CLEAR_OVER = 0.010  # m its foot is over the pot's rim, for it to count as clear
OVER_R = 0.040  # m its axis from the pot's: over the mouth
KEPT = 0.90  # of the leaves that must still be in the basket
POT_MOVED, POT_TILTED = 0.040, math.radians(15.0)  # the pot shifted or tipped further than these at any time scores 0
class Shake:
    """Counts down-up strokes of the basket's height while it is over the pot. A stroke is a fall of at least SHAKE_AMP from
    the highest point, then a rise of at least SHAKE_AMP from the lowest; it ends with the rise. Shaken (latched) once 3
    strokes in a row have ended within SHAKE_SPAN of each other's ends (the 3rd within 2 s of the 1st). Leaving the region
    over the pot restarts the count; shaken stays."""

    def __init__(self) -> None:
        self.reset()
        self.shaken = False

    def reset(self) -> None:
        self.mode, self.peak, self.trough, self.ends = None, 0.0, 0.0, []

    def update(self, z: float, t: float, over: bool) -> None:
        if not over:
            self.reset()
            return
        if self.mode is None:
            self.mode, self.peak = "up", z
        elif self.mode == "up":
            self.peak = max(self.peak, z)
            if z < self.peak - SHAKE_AMP:
                self.mode, self.trough = "down", z
        else:
            self.trough = min(self.trough, z)
            if z > self.trough + SHAKE_AMP:
                self.mode, self.peak = "up", z
                self.ends.append(t)
                if len(self.ends) >= SHAKE_N and self.ends[-1] - self.ends[-SHAKE_N] <= SHAKE_SPAN:
                    self.shaken = True


_STATE: dict = {"shake": Shake(), "pot_bad": False, "t": 0.0, "last": -1.0}


def tilt(rot) -> float:
    """The angle (rad) between a body's z axis and the vertical."""
    return float(math.acos(np.clip(np.asarray(rot)[2, 2], -1.0, 1.0)))


def leaves_kept(grains, infuser) -> float:
    """The share of the leaves inside the basket: within its radius of its axis, from its floor to 4 cm over its bar."""
    p, _ = grains
    local = (p - infuser.pos) @ infuser.rotation()
    inside = (np.hypot(local[:, 0], local[:, 1]) < BASKET_R) & (local[:, 2] > BASKET_FOOT - 0.002) & (local[:, 2] < BAR_TOP + 0.04)
    return float(inside.mean())


def status(objects, grains=None) -> dict:
    """What scoring needs, from the bodies' poses."""
    pot, lid, inf, sau = objects["teapot"], objects["lid"], objects["infuser"], objects["tea_saucer"]
    grains = grains if grains is not None else soilkit.grains("leaves")
    foot = inf.pos[2] + BASKET_FOOT  # the basket's underside
    d_pot = float(np.hypot(inf.pos[0] - pot.pos[0], inf.pos[1] - pot.pos[1]))
    d_sau = float(np.hypot(inf.pos[0] - sau.pos[0], inf.pos[1] - sau.pos[1]))
    lid_off = np.asarray(lid.pos) - np.asarray(pot.pos)
    return {
        "out": d_pot > OVER_R + 0.035 and foot < pot.pos[2] + POT_H,  # clear of the mouth's footprint, down on a surface
        "on_saucer": d_sau < 0.030 and foot < sau.pos[2] + SAUCER_H + 0.008 and tilt(inf.rotation()) < math.radians(15.0),
        "kept": leaves_kept(grains, inf),
        "shaken": _STATE["shake"].shaken,
        "lid_on": bool(np.hypot(lid_off[0], lid_off[1]) < 0.006 and abs(lid_off[2]) < 0.004 and tilt(lid.rotation()) < math.radians(5.0)),
        "pot_bad": _STATE["pot_bad"],
    }


def score_status(st: dict) -> float:
    """0 if the pot was shifted 4 cm or tipped 15 degrees at any time. Else 0.25 with the basket out of the pot and
    resting clear of it; 0.5 with it on the saucer (axis within 3 cm of the saucer's centre, tilted under 15 degrees);
    0.75 with at least 90 % of the leaves still in it and its having been shaken over the pot (3 down-up strokes of
    at least 2 cm, the 3rd ending within 2 s of the 1st); 1.0 with the lid back on the pot (within 6 mm of its axis, flat to 4 mm and 5 degrees)."""
    if st["pot_bad"] or not st["out"]:
        return 0.0
    s = 0.25
    if st["on_saucer"]:
        s = 0.5
        if st["kept"] >= KEPT and st["shaken"]:
            s = 0.75
            if st["lid_on"]:
                s = 1.0
    return s


def score(objects) -> float:
    return score_status(status(objects))


def success(objects) -> bool:
    return score(objects) == 1.0


@on_reset
def _reset(environment) -> None:
    _STATE.update(shake=Shake(), pot_bad=False, t=0.0, last=-1.0)


@on_step
def _watch(environment) -> None:
    """Latch the pot's being shifted or tipped, and time the basket held clear over the pot."""
    try:
        dt = environment.unwrapped.step_dt
        pot, inf = _state(environment, TEAPOT), _state(environment, INFUSER)
        if math.hypot(pot.pos[0] - POT_XY[0], pot.pos[1] - POT_XY[1]) > POT_MOVED or tilt(pot.rotation()) > POT_TILTED:
            _STATE["pot_bad"] = True
        foot = inf.pos[2] + BASKET_FOOT
        over = (math.hypot(inf.pos[0] - pot.pos[0], inf.pos[1] - pot.pos[1]) < OVER_R and foot > pot.pos[2] + POT_H + CLEAR_OVER
                and tilt(inf.rotation()) < math.radians(25.0))
        _STATE["shake"].update(foot, _STATE["t"], over)
        _STATE["t"] += dt
        if os.environ.get("TEA_LOG") and _STATE["t"] - _STATE["last"] >= 0.5:
            _STATE["last"] = _STATE["t"]
            objs = {"teapot": pot, "lid": _state(environment, LID), "infuser": inf, "tea_saucer": _state(environment, SAUCER)}
            st = status(objs)
            lid = objs["lid"]
            print(f"LID t={_STATE['t']:.1f} xy={lid.pos[0]:.3f},{lid.pos[1]:.3f} z={(lid.pos[2] - Z0) * 1000:.0f}mm tilt={math.degrees(tilt(lid.rotation())):.0f}", flush=True)
            print(f"LOG t={_STATE['t']:.1f} basket_z={(inf.pos[2] - Z0) * 1000:.0f}mm basket_xy={inf.pos[0]:.3f},{inf.pos[1]:.3f} "
                  f"strokes={len(_STATE['shake'].ends)} shaken={st['shaken']} out={st['out']} saucer={st['on_saucer']} kept={st['kept']:.2f} "
                  f"lid_on={st['lid_on']} pot_bad={st['pot_bad']} score={score_status(st):.2f}", flush=True)
    except Exception:
        import traceback

        traceback.print_exc()
        raise


def self_test() -> int:
    """The scoring rules on hand-built states, and the layout's clearances."""
    def st(out=False, on_saucer=False, kept=1.0, shaken=False, lid_on=False, pot_bad=False):
        return {"out": out, "on_saucer": on_saucer, "kept": kept, "shaken": shaken, "lid_on": lid_on, "pot_bad": pot_bad}

    cases = {
        "start (basket in the pot)": (st(lid_on=True), 0.0),
        "lid off only": (st(), 0.0),
        "basket on the table": (st(out=True), 0.25),
        "basket on the saucer, leaves spilled": (st(out=True, on_saucer=True, kept=0.4), 0.5),
        "basket on the saucer, never shaken": (st(out=True, on_saucer=True, shaken=False, lid_on=True), 0.5),
        "on the saucer, shaken, 85 % of the leaves": (st(out=True, on_saucer=True, kept=0.85, shaken=True, lid_on=True), 0.5),
        "on the saucer, shaken, leaves kept, lid off": (st(out=True, on_saucer=True, shaken=True), 0.75),
        "everything, lid on": (st(out=True, on_saucer=True, shaken=True, lid_on=True), 1.0),
        "everything but the pot was knocked over": (st(out=True, on_saucer=True, shaken=True, lid_on=True, pot_bad=True), 0.0),
        "basket on the table, pot shifted": (st(out=True, pot_bad=True), 0.0),
    }
    for name, (status_, want) in cases.items():
        got = score_status(status_)
        assert abs(got - want) < 1e-9, (name, got, want)
    def strokes(heights, dt=0.05, over=True):
        sh = Shake()
        for i, z in enumerate(heights):
            sh.update(z, i * dt, over)
        return sh

    def wave(n, amp, period, dt=0.05, z0=0.2, lead=4):
        t = np.arange(0, n * period + 1e-9, dt)
        return list(z0 + np.zeros(lead)) + list(z0 - amp / 2 + amp / 2 * np.cos(2 * np.pi * t / period))

    shakes = {
        "three quick 4 cm strokes": (strokes(wave(3, 0.04, 0.5)).shaken, True),
        "three quick 2.4 cm strokes": (strokes(wave(3, 0.024, 0.5)).shaken, True),
        "three quick 1.5 cm strokes": (strokes(wave(3, 0.015, 0.5)).shaken, False),
        "two quick strokes": (strokes(wave(2, 0.04, 0.5)).shaken, False),
        "three slow strokes (period 1.5 s)": (strokes(wave(3, 0.04, 1.5)).shaken, False),
        "five strokes, the last three quick": (strokes(wave(2, 0.04, 1.4) + wave(3, 0.04, 0.5)).shaken, True),
        "a steady hold": (strokes([0.2] * 80).shaken, False),
        "strokes made away from the pot": (strokes(wave(3, 0.04, 0.5), over=False).shaken, False),
    }
    for name, (got, want) in shakes.items():
        assert got == want, (name, got, want)
    mixed = Shake()  # leaving the region over the pot mid-shake restarts the count
    zs = wave(3, 0.04, 0.5)
    for i, z in enumerate(zs):
        mixed.update(z, i * 0.05, i != len(zs) // 2)
    assert not mixed.shaken
    assert abs(tilt(np.eye(3))) < 1e-9 and abs(tilt(np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0.0]])) - math.pi / 2) < 1e-9

    class B:  # a bare pose
        def __init__(self, pos, rot=None):
            self.pos, self._r = np.asarray(pos, float), np.eye(3) if rot is None else rot

        def rotation(self):
            return self._r

    pot = B((*POT_XY, Z0))
    inf_in = B((*POT_XY, Z0))
    leaves = np.load(soilkit.HERE / os.environ.get("LEAVES_START", "assets/leaves_start.npz"))
    pos = leaves["pos"]
    assert leaves_kept((pos, leaves["radius"]), inf_in) == 1.0, "the leaves do not start in the basket"
    assert leaves_kept((pos + (0.2, 0, 0), leaves["radius"]), inf_in) == 0.0
    half = pos.copy()
    half[: len(half) // 2] += (0.2, 0, 0)
    assert abs(leaves_kept((half, leaves["radius"]), inf_in) - 0.5) < 0.01
    # the layout: the saucer, the parked lid and the pot's handle and spout clear each other
    assert math.dist(SAUCER_XY, POT_XY) - 0.071 > 0.132 + 0.005, "the saucer is in the handle's way"
    assert math.dist(PARK_XY, POT_XY) - 0.0525 > 0.110 + 0.005, "the parked lid is in the spout's way"
    assert BAR_TOP < POT_H - 0.003 and MOUTH_R - BASKET_R > 0.003, "the basket does not sit in the mouth as stated"
    assert abs(len(pos) - 200) < 40 and (1000.0 * 4 / 3 * np.pi * leaves["radius"] ** 3).sum() < 0.03
    return len(cases) + len(shakes) + 1


# -- the scripted demo ------------------------------------------------------------------------------------------------
#   1. the right hand pinches the lid's knob, lifts the lid off and sets it down to the right of the spout;
#   2. the left hand pinches the bar across the basket (the hand tilted 45 degrees forward, closing along y), lifts the
#      basket out, holds it over the pot for 3.5 s, carries it to the saucer and sets it down;
#   3. the right hand fetches the lid and sets it back on the pot.
def Ry(a: float) -> np.ndarray:
    return np.array([[math.cos(a), 0, math.sin(a)], [0, 1, 0], [-math.sin(a), 0, math.cos(a)]])


def demo():
    from yam_arena.constants import FINGER_CLOSED as C
    from yam_arena.constants import FINGER_OPEN as O
    from yam_arena.demo import Waypoint as W

    import tiltkit

    tiltkit.install()
    NN, GN = -0.020, -0.007  # a 40 mm opening, for going round the part; a 14 mm opening, for letting go
    right, left = [], []

    def add(arm, t, pos, finger, label, deg, joint=False):
        a = math.radians(deg)
        arm.append(W(t, tuple(float(v) for v in pos), finger, label,
                     tiltkit.grip(tuple(Ry(a) @ (0, 0, -1)), tuple(Ry(a) @ (0, -1, 0)), joint=joint)))

    def wait(arm, other):
        d = sum(w.duration for w in other) - sum(w.duration for w in arm)
        if d > 0.05:
            arm.append(W(d, arm[-1].pos, arm[-1].finger, "wait", arm[-1].yaw))

    KNOB = 0.172  # the pinch height on the knob over the foot
    px, py = POT_XY
    DEG_L, DEG_B = float(os.environ.get("LID_DEG", "-30")), float(os.environ.get("BASKET_DEG", "-60"))
    knob = np.array([px, py, Z0 + KNOB])
    # the lid lies on the table at the park point: its underside (149.5 mm over its origin) on the table, so its knob's
    # pinch height is the seated one less 150 mm. It is carried over the pot's rim and the spout at the seated height + 35 mm.
    park = np.array([PARK_XY[0], PARK_XY[1], Z0 + KNOB - LID_SEAT - 0.001])
    carry = np.array([PARK_XY[0], PARK_XY[1], Z0 + KNOB + 0.035])
    up = np.array([0.0, 0.0, 0.035])
    # the right arm starts over the table; the left waits out of the way
    right.append(W(2.0, (0.20, -0.17, T + 0.22), O, "start"))
    left.append(W(2.0, (0.20, 0.20, T + 0.22), O, "start"))

    def lid_pick(P, label):
        add(right, 2.0, P + (-0.05, 0, 0.05), NN, f"over the {label}", DEG_L, joint=True)
        add(right, 1.5, P, NN, f"down beside the knob", DEG_L)
        add(right, 1.0, P, C, "pinch the knob", DEG_L)

    # 1. the lid off
    lid_pick(knob, "lid's knob")
    add(right, 1.5, knob + up, C, "lift the lid off", DEG_L)
    add(right, 3.0, carry, C, "carry it to the right, clear of the rim and the spout", DEG_L)
    add(right, 3.0, park + (0, 0, 0.002), C, "lower it onto the table", DEG_L)
    add(right, 1.0, park + (0, 0, 0.002), NN, "let go (the knob is 27 mm across: open past it)", DEG_L)
    add(right, 1.5, park + (-0.02, 0, 0.07), O, "back and up", DEG_L)
    right.append(W(1.5, (0.20, -0.17, T + 0.22), O, "clear of the pot"))
    # 2. the basket out
    wait(left, right)
    B = np.array([px, py, Z0 + BAR_MID])
    ad = Ry(math.radians(DEG_B)) @ np.array([0.0, 0.0, -1.0])  # the approach direction
    add(left, 2.5, B - 0.09 * ad, NN, "over the bar, on the approach line", DEG_B, joint=True)
    add(left, 2.0, B, NN, "down to the bar", DEG_B)
    add(left, 1.0, B, C, "pinch the bar", DEG_B)
    LIFT = float(os.environ.get("BASKET_LIFT", "0.22"))
    AMP, LEG = float(os.environ.get("SHAKE_AMP", "0.025")), float(os.environ.get("SHAKE_LEG", "0.35"))
    add(left, 3.0, B + (0, 0, LIFT), C, "lift the basket clear", DEG_B)
    add(left, 0.8, B + (0, 0, LIFT), C, "steady over the pot", DEG_B)
    for k in range(3):  # three quick down-up strokes, 4 cm each, a stroke every 0.5 s
        add(left, LEG, B + (0, 0, LIFT - AMP), C, f"shake {k + 1}: down", DEG_B)
        add(left, LEG, B + (0, 0, LIFT), C, f"shake {k + 1}: up", DEG_B)
    add(left, 0.8, B + (0, 0, LIFT), C, "settle over the pot", DEG_B)
    S = np.array([SAUCER_XY[0], SAUCER_XY[1], Z0 + SAUCER_H + 0.003 - BASKET_FOOT + BAR_MID])
    add(left, 3.0, S + (0, 0, 0.12), C, "carry it over the saucer", DEG_B)
    add(left, 2.0, S, C, "set it down", DEG_B)
    add(left, 1.0, S, GN, "let go", DEG_B)
    add(left, 1.5, S + (-0.03, 0, 0.10), O, "back and up", DEG_B)
    left.append(W(1.5, (0.20, 0.20, T + 0.22), O, "clear"))
    # 3. the lid back on
    wait(right, left)
    park_pick = park + (-0.005, -0.003, 0.0)  # where the lid lies after it settles (measured in the simulation)
    lid_pick(park_pick, "parked lid's knob")
    kp = knob + (0.0, 0.004, 0.0)  # the lid slides 0-7 mm towards the right as the fingers open (measured)
    add(right, 3.0, carry, C, "lift it clear of the table", DEG_L)
    add(right, 3.0, kp + up, C, "carry it over the pot", DEG_L)
    add(right, 1.5, kp + (0, 0, 0.003), C, "lower it onto the rim", DEG_L)
    add(right, 1.0, kp + (0, 0, 0.003), NN, "let go (open past the 27 mm knob)", DEG_L)
    add(right, 1.5, knob + (-0.03, 0, 0.08), O, "back and up", DEG_L)
    wait(left, right)
    return {"left": left, "right": right}


if __name__ == "__main__":
    # Offline checks, no GPU: the scoring rules, then each arm's plan through the kinematics at 20 Hz.
    from yam_arena import ik, station
    from yam_arena.demo import ArmPlan
    from yam_arena.task_file import load

    print(f"scoring: {self_test()} cases pass")
    task = load(__file__)
    hz = 20.0
    for arm, waypoints in task.demo().items():
        plan = ArmPlan(list(waypoints), station.BASES[arm])
        q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
        print(f"{arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / {plan.worst_ik_deg:.3f} deg")
        assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (arm, plan.worst_ik_mm, plan.worst_ik_deg)
        assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), arm
