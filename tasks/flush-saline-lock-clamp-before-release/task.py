"""Flushing a peripheral IV saline lock with a prefilled saline syringe, then clamping its extension line by the
positive-pressure method. A training forearm pad with an IV catheter taped in place; a short extension set runs from
the catheter hub to a needleless connector held in a securement clip, with an open pinch clamp on its tubing. A 10 mL
prefilled flush syringe (5 mL of saline) is already screwed onto the connector.

One hand pushes the plunger and keeps pushing; the other squeezes the pinch clamp shut while the plunger is still under
pressure; only then does the first hand let go. Letting go of the plunger with the line still open is the classic
mistake: the syringe's rubber stopper, squeezed by the push, springs back when the thumb comes off and pulls a little
fluid back out of the line, and with a negative-displacement connector that draws blood back into the catheter tip,
where it clots and blocks the line.

Kits: yamkit (the syringe's plunger and the clamp's jaw as sliding links, gripper states), guardkit (gk.touching: a
gripper's own shape in contact with the plunger's thumb press or the clamp's jaws). The plunger's law (`Plunger`,
below) stands for the fluid: it moves only as fast as saline can flow out through the catheter (a damper), cannot be
drawn back, and is locked once the clamp is shut.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import yamkit as kit  # noqa: E402
import mechkit as mk  # noqa: E402
import guardkit as gk  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Flush the IV saline lock with the syringe and clamp the line."

# --- Scene, world coordinates (m): x forward from the arm bases, y to the robot's left, z up from the table top.
# Everything lies along y at x = SX: the syringe on the robot's right (plunger toward -y), the connector in its
# securement clip, the pinch clamp on the tubing, then the catheter in the training forearm pad on the left.
SX = 0.300  # the line's x
AXIS_Z = 0.036  # the syringe's and the tubing's axis, above the table

# The syringe: a 10 mL prefilled saline flush syringe (BD PosiFlush-type), 5 mL fill. Barrel 14.5 mm inside
# (1.65 cm^2: 6.05 mm of plunger per mL), 16.5 mm outside.
BORE_R, BARREL_R = 0.00725, 0.00825
ML_PER_M = math.pi * BORE_R ** 2 * 1e6  # mL per metre of plunger travel (165)
FILL = 5.0  # mL of saline at the start
TIP_IN_Y = -0.042  # the barrel's inside end (the stopper's front face there = empty)
FLANGE_Y = -0.115  # the finger flange
STROKE = FILL / ML_PER_M  # 0.0303 m of plunger travel for 5 mL
STOP0_Y = TIP_IN_Y - STROKE  # the stopper's front face at the start
STOPPER_L = 0.008
ROD = 0.085  # the stopper's front face to the thumb press's front face
PRESS_R, PRESS_T = 0.011, 0.003  # thumb press: 22 mm across, 3 mm thick
PRESS0_Y = STOP0_Y - ROD - PRESS_T / 2  # the thumb press's center at the start

# The needleless connector (negative-displacement split septum) in a securement clip, the extension tubing and the
# pinch clamp on it.
CONN_Y = (-0.030, -0.005)
CONN_R = 0.006
TUBE_R = 0.0018  # 3.6 mm extension tubing
PAD_Y = (0.050, 0.290)  # the forearm pad along y
PAD_X = (0.240, 0.360)
PAD_H = 0.032
HUB_Y = 0.080  # the catheter hub, taped on the pad
CLAMP_Y = 0.022  # the pinch clamp's center along the tubing
JAW_T, JAW_W = 0.003, 0.014  # each jaw: 3 mm thick (x), 14 mm along the tubing (y)
JAW_Z = (0.026, 0.056)  # the jaws' height span above the table
SQUEEZE = 0.0045  # m the moving jaw travels from open to shut (the tubing pinched flat)
SHUT_ZONE = 0.0008  # within 0.8 mm of shut its ratchet teeth catch, and it stays shut
FIXED_JAW_X = SX + TUBE_R + JAW_T / 2  # the fixed jaw on the tubing's +x side
MOVING_JAW_X0 = SX - TUBE_R - JAW_T / 2 - SQUEEZE  # the moving jaw open, on the -x side

# --- The fluid. The plunger is a damper: saline leaves through the catheter at a rate set by the push. Scaled to what
# one YAM arm pushes sideways comfortably: about 1.5 N on the thumb press moves 1 mL/s (6 mm/s of plunger), so
# 1.5 N / 0.006 m/s = 250 N*s/m. (A hand pushes harder on a real 22 G catheter; what is scored is the order of push,
# clamp and release, not the pressure.) Pinching the tubing cuts the flow with the square of its opening.
FLOW_DAMP = 250.0
MIN_OPEN = 0.03

# --- Scoring thresholds.
FULL_ML = 4.8  # the whole 5 mL flushed (within 0.2 mL: the plunger within 1.2 mm of the barrel's end)
HALF_ML = 3.0  # at least 3 mL: more than twice the line's volume (catheter + 10 cm extension set, about 1.2 mL)
PART_ML = 1.0  # at least 1 mL: the catheter and connector cleared once, real progress
PUSHED_ML = 0.1  # mL a contact must push before letting go of it counts as a release under pressure
RELEASE_STEPS = 3  # a hand off the thumb press for 3 control steps (0.15 s) has let go (contact chatter is not)
STILL = 0.002  # m/s: a plunger or jaw still moving does not count

GLASS = kit.Look((0.92, 0.95, 0.97), 0.05, opacity=0.30)
WHITE = kit.Look((0.95, 0.95, 0.94), 0.5)
RUBBER = kit.Look((0.18, 0.18, 0.20), 0.8)
CLEAR = kit.Look((0.85, 0.92, 0.95), 0.1, opacity=0.45)
BLUE = kit.Look((0.15, 0.40, 0.85), 0.5)
SKIN = kit.Look((0.80, 0.52, 0.45), 0.7)  # a pinkish skin tone, to stand out from the table
VEIN = kit.Look((0.35, 0.45, 0.75), 0.6)
TAPE = kit.Look((0.97, 0.97, 0.97), 0.6, opacity=0.85)
INK = kit.Look((0.05, 0.05, 0.05), 0.6)
CLAMP_WHITE = kit.Look((0.97, 0.96, 0.92), 0.5)


class Plunger:
    """The syringe's plunger on the fluid. Pushed, it moves only as fast as saline can leave through the catheter (a
    damper, `FLOW_DAMP`, stiffer as the clamp pinches the tubing); it cannot be drawn back (its lower stop follows it
    in, so a hand cannot aspirate); with the clamp shut the line is closed and the plunger is locked where it is.
    Memory: "max_q" (m pushed in), "shut" (the clamp shut)."""

    sets_limits = True

    def fresh(self, lk):
        return {"max_q": 0.0, "limits": None, "shut": False, "lock_q": None}

    def drive(self, key, q, qd, J, mem):
        mem["max_q"] = max(mem["max_q"], min(q, STROKE))
        frac = clamp_open_fraction(J.get("clamp", 0.0))
        mem["shut"] = frac <= 0.0
        if mem["shut"]:
            if mem.get("lock_q") is None:
                mem["lock_q"] = mem["max_q"]  # a closed line: the plunger stays where the clamp caught it
            mem["max_q"] = mem["lock_q"]
            mem["limits"] = (mem["lock_q"], mem["lock_q"] + 1e-5)
            return mem["lock_q"], 2000.0, 0.0, FLOW_DAMP
        mem["limits"] = (mem["max_q"], STROKE)
        return q, 0.0, 0.0, FLOW_DAMP / max(frac, MIN_OPEN) ** 2


def clamp_open_fraction(jq):
    """How open the pinch clamp leaves the tubing: 1 with the jaw back, 0 once its teeth catch (shut)."""
    if jq >= SQUEEZE - SHUT_ZONE:
        return 0.0
    return float(min(1.0, max(0.0, 1.0 - jq / SQUEEZE)))


def _iv_set():
    """The syringe's barrel, the connector in its securement clip, the tubing, the clamp's fixed jaw, the catheter hub
    and the forearm pad (fixed); the plunger and the clamp's moving jaw slide."""
    o = np.array([SX, 0.0, 0.0])

    def at(x, y, z):
        return (x - o[0], y, z)

    m, c = [], []
    # the training forearm pad: skin-toned silicone with a blue vein along it, the catheter taped in
    pad_c = at(sum(PAD_X) / 2, sum(PAD_Y) / 2, PAD_H / 2)
    pad_s = (PAD_X[1] - PAD_X[0], PAD_Y[1] - PAD_Y[0], PAD_H)
    m.append(kit.box(pad_c, pad_s, SKIN))
    c.append(kit.Collider(pad_c, size=pad_s))
    m.append(kit.box(at(SX, (HUB_Y + PAD_Y[1]) / 2, PAD_H + 0.0004), (0.006, PAD_Y[1] - HUB_Y, 0.001), VEIN))
    m.append(kit.cylinder(at(SX, HUB_Y + 0.006, PAD_H + 0.003), 0.004, 0.016, CLEAR, axis="y"))  # catheter hub
    m.append(kit.box(at(SX, HUB_Y + 0.006, PAD_H + 0.0015), (0.022, 0.010, 0.003), BLUE))  # its wings
    m.append(kit.box(at(SX, HUB_Y + 0.006, PAD_H + 0.0065), (0.050, 0.030, 0.001), TAPE))  # the dressing
    # the extension tubing from the connector, past the clamp, down onto the pad to the hub
    path = [(SX, CONN_Y[1], AXIS_Z), (SX, PAD_Y[0] - 0.010, AXIS_Z), (SX, PAD_Y[0] + 0.012, PAD_H + 0.004),
            (SX, HUB_Y - 0.002, PAD_H + 0.003)]
    for a, b in zip(path, path[1:]):
        a, b = np.asarray(a, float), np.asarray(b, float)
        m.append(kit.cylinder(tuple((a + b) / 2 - o), TUBE_R, float(np.linalg.norm(b - a)), CLEAR, axis=tuple(b - a)))
    # the needleless connector (clear body, blue septum cap) and the white securement clip under it
    m.append(kit.cylinder(at(SX, sum(CONN_Y) / 2, AXIS_Z), CONN_R, CONN_Y[1] - CONN_Y[0], CLEAR, axis="y"))
    m.append(kit.cylinder(at(SX, CONN_Y[0] + 0.003, AXIS_Z), CONN_R + 0.0005, 0.006, BLUE, axis="y"))
    clip_c, clip_s = at(SX, sum(CONN_Y) / 2, (AXIS_Z - CONN_R) / 2), (0.032, 0.022, AXIS_Z - CONN_R)
    m.append(kit.box(clip_c, clip_s, WHITE))
    c.append(kit.Collider(clip_c, size=clip_s))
    c.append(kit.Collider(at(SX, sum(CONN_Y) / 2, AXIS_Z), size=(2 * CONN_R, CONN_Y[1] - CONN_Y[0], 2 * CONN_R)))
    # the syringe's barrel: clear, with 1 mL graduation rings, a finger flange and the luer tip into the connector
    m.append(kit.lathe([(BORE_R, 0.0), (BARREL_R, 0.0), (BARREL_R, TIP_IN_Y - FLANGE_Y), (BORE_R, TIP_IN_Y - FLANGE_Y)],
                       GLASS, center=at(SX, FLANGE_Y, AXIS_Z), axis="y"))
    for k in range(21):  # a printed scale on the barrel's top: a tick every 0.5 mL, longer at each mL
        y = TIP_IN_Y - 0.5 * k / ML_PER_M
        if y > FLANGE_Y + 0.002:
            m.append(kit.box(at(SX - 0.0015 * (k % 2 == 0), y, AXIS_Z + BARREL_R + 0.0002),
                             (0.006 if k % 2 == 0 else 0.003, 0.0004, 0.0004), INK))
    m.append(kit.box(at(SX, FLANGE_Y - 0.0015, AXIS_Z), (0.034, 0.003, 0.012), GLASS))  # finger flange
    m.append(kit.cylinder(at(SX, (TIP_IN_Y + CONN_Y[0]) / 2, AXIS_Z), 0.003, CONN_Y[0] - TIP_IN_Y, GLASS, axis="y"))
    c.append(kit.Collider(at(SX, (FLANGE_Y + TIP_IN_Y) / 2, AXIS_Z),
                          size=(2 * BARREL_R, TIP_IN_Y - FLANGE_Y, 2 * BARREL_R)))
    c.append(kit.Collider(at(SX, FLANGE_Y - 0.0015, AXIS_Z), size=(0.034, 0.003, 0.012)))
    # the clamp's fixed jaw (with its hinge block on the far side of the tubing) on the tubing's +x side
    jaw_c, jaw_s = at(FIXED_JAW_X, CLAMP_Y, sum(JAW_Z) / 2), (JAW_T, JAW_W, JAW_Z[1] - JAW_Z[0])
    m.append(kit.box(jaw_c, jaw_s, CLAMP_WHITE))
    c.append(kit.Collider(jaw_c, size=jaw_s))
    hinge_c = at(FIXED_JAW_X + 0.004, CLAMP_Y, JAW_Z[0] + 0.004)
    m.append(kit.box(hinge_c, (0.005, JAW_W, 0.008), CLAMP_WHITE))
    c.append(kit.Collider(hinge_c, size=(0.005, JAW_W, 0.008)))

    # the plunger: rubber stopper inside the barrel, white rod back out past the flange to the thumb press
    rod_mid = -(STOPPER_L + ROD) / 2
    plunger = kit.Link(
        "plunger", "prismatic", "Y", at(SX, STOP0_Y, AXIS_Z), lower=0.0, upper=STROKE, start=0.0, mass=0.010,
        meshes=(kit.cylinder((0, -STOPPER_L / 2, 0), BORE_R - 0.0002, STOPPER_L, RUBBER, axis="y"),
                kit.box((0, rod_mid, 0), (0.010, ROD - STOPPER_L, 0.010), WHITE),
                kit.cylinder((0, -ROD - PRESS_T / 2, 0), PRESS_R, PRESS_T, WHITE, axis="y")),
        colliders=(kit.Collider((0, -ROD - PRESS_T / 2, 0), size=(2 * PRESS_R, PRESS_T, 2 * PRESS_R)),
                   kit.Collider((0, -ROD / 2 - 0.004, 0), size=(0.010, ROD - STOPPER_L - 0.010, 0.010))),
        law=Plunger())
    # the clamp's moving jaw: slides +x onto the tubing; its teeth catch once shut (yamkit Latch); a light spring
    # (implicit, mk.Spring) opens it until then
    jaw = kit.Link(
        "clamp", "prismatic", "X", at(MOVING_JAW_X0, CLAMP_Y, sum(JAW_Z) / 2), lower=0.0, upper=SQUEEZE, start=0.0,
        mass=0.004, law=mk.Spring(rest=0.0, k=30.0, damp=0.5),
        meshes=(kit.box((0, 0, 0), jaw_s, CLAMP_WHITE),
                kit.box((-0.002, 0, -(jaw_s[2] / 2) + 0.004), (0.004, JAW_W, 0.008), CLAMP_WHITE)),
        colliders=(kit.Collider((0, 0, 0), size=jaw_s),),
        latch=kit.Latch(at=SQUEEZE, zone=SHUT_ZONE, hold=400.0))
    return kit.Mechanism("iv_set", pos=(SX, 0.0, TOP), size=(0.14, 0.60, 0.07), meshes=tuple(m), colliders=tuple(c),
                         links=(plunger, jaw))


OBJECTS = [_iv_set()]

# Contact shapes, in each link's own frame: the thumb press, and the moving jaw (with the fixed jaw, in the base's).
PRESS_SHAPE = gk.Block((0.0, -ROD - PRESS_T / 2, 0.0), (2 * PRESS_R, PRESS_T + 0.004, 2 * PRESS_R))
# The white rod between the stopper and the thumb press is a collider too: a hand can grip it and push the plunger
# without touching the thumb press, so a hand on the rod counts as a hand on the plunger (4 mm margin all round).
ROD_SHAPE = gk.Block((0.0, -(STOPPER_L + ROD) / 2, 0.0), (0.010 + 0.004, ROD - STOPPER_L + 0.004, 0.010 + 0.004))
JAW_SHAPE = gk.Block((0.0, 0.0, 0.0), (JAW_T, JAW_W, JAW_Z[1] - JAW_Z[0]))
FIXED_JAW_SHAPE = gk.Block((FIXED_JAW_X - SX, CLAMP_Y, sum(JAW_Z) / 2), (JAW_T, JAW_W, JAW_Z[1] - JAW_Z[0]))


def plunger_q(objects):
    return kit.joint(objects, "iv_set/plunger").pos


def clamp_q(objects):
    return kit.joint(objects, "iv_set/clamp").pos


def clamp_shut(objects):
    j = kit.joint(objects, "iv_set/clamp")
    return bool(j.latched or j.pos >= SQUEEZE - SHUT_ZONE)


def flushed_ml(objects):
    """mL of saline pushed into the line (the plunger's furthest travel)."""
    return max(kit.law_state("iv_set/plunger").get("max_q", 0.0), plunger_q(objects)) * ML_PER_M


def on_thumb_press(objects):
    return bool(gk.touching(objects, "iv_set/plunger", PRESS_SHAPE))


def on_press(objects):
    """A hand on the plunger: on its thumb press or gripping its rod (either can push it)."""
    return bool(on_thumb_press(objects) or gk.touching(objects, "iv_set/plunger", ROD_SHAPE))


def on_clamp(objects):
    return bool(gk.touching(objects, "iv_set/clamp", JAW_SHAPE) or gk.touching(objects, "iv_set", FIXED_JAW_SHAPE))


# --- The release tracker: run once per control step (the hooks call score() and success() on the same `objects`).
_T = {"episode": None, "objects": None, "contact_q": None, "off": 0, "reflux": None, "releases": 0, "step": 0}


def _track(objects):
    if _T["episode"] != kit.episode():
        _T.update(episode=kit.episode(), objects=None, contact_q=None, off=0, reflux=None, releases=0, step=0)
    if objects is _T["objects"]:
        return
    _T["objects"] = objects
    _T["step"] += 1
    q = plunger_q(objects)
    if os.environ.get("FLUSH_DEBUG"):
        jc = kit.joint(objects, "iv_set/clamp")
        r, l = objects.get("right_gripper"), objects.get("left_gripper")
        print(f"FLUSH {_T['step']} q={q * ML_PER_M:.3f}mL qd={kit.joint(objects, 'iv_set/plunger').vel:.4f} "
              f"clamp={jc.pos * 1000:.2f}mm latched={jc.latched} press={on_press(objects)} onclamp={on_clamp(objects)} "
              f"R={np.round(r.pos, 4) if r else None} L={np.round(l.pos, 4) if l else None} "
              f"Lopen={kit.opening(objects, 'left_gripper'):.2f} reflux={_T['reflux']}", flush=True)
    if on_press(objects):
        if _T["contact_q"] is None:
            _T["contact_q"] = q  # a hand comes onto the thumb press
        _T["off"] = 0
        return
    if _T["contact_q"] is None:
        return
    _T["off"] += 1
    if _T["off"] < RELEASE_STEPS:
        return
    pushed = (q - _T["contact_q"]) * ML_PER_M
    _T["contact_q"], _T["off"] = None, 0
    if pushed >= PUSHED_ML:
        _T["releases"] += 1
        if not clamp_shut(objects) and _T["reflux"] is None:
            _T["reflux"] = _T["step"]  # let go under pressure with the line open: the stopper springs back


def reflux(objects):
    """The control step at which the plunger was let go under pressure with the clamp open (latched), or None."""
    _track(objects)
    return _T["reflux"]


def _job(objects):
    _track(objects)
    if _T["reflux"] is not None:
        return 0.0  # let go of the plunger with the line open: blood drawn back into the catheter (latched)
    if on_press(objects) or on_clamp(objects):
        return 0.0  # a hand still on the thumb press, the rod or the clamp: not finished
    if abs(kit.joint(objects, "iv_set/plunger").vel) > STILL or abs(kit.joint(objects, "iv_set/clamp").vel) > STILL:
        return 0.0
    if not clamp_shut(objects):
        return 0.0  # the line left open
    v = flushed_ml(objects)
    if v >= FULL_ML:
        return 1.0
    if v >= HALF_ML:
        return 0.5
    if v >= PART_ML:
        return 0.25
    return 0.0


def success(objects):
    """All 5 mL flushed, the clamp shut while the plunger was still pressed, hands off."""
    return _job(objects) == 1.0


def score(objects):
    """0 if the plunger was let go (a hand off its thumb press for 0.15 s after pushing at least 0.1 mL) while the
    clamp was still open, at any time (latched): the syringe's rubber stopper springs back and draws blood into the
    catheter tip. 0 if a gripper still touches the thumb press or the clamp at the end, if either is still moving
    faster than 2 mm/s, or if the clamp is not shut. Otherwise by the saline flushed: 1.0 at 4.8 mL or more (the whole
    5 mL syringe); 0.5 at 3.0 mL or more (over twice the line's volume); 0.25 at 1.0 mL or more; 0 below. The final
    state must hold 1 s. Possible scores: 0, 0.25, 0.5, 1.0."""
    return _job(objects)


# --- Demo: the right arm pushes the plunger, the left arm squeezes the clamp shut while it is still pushing.

FINGER_HALF = 0.010  # m: closed fingertips' pad face from the tool point, along the closing axis (measured in sim)
PUSH_Z = AXIS_Z + 0.004  # tool point height pushing: fingertips 1.3 cm below it, across the thumb press


def demo():
    """(Set DEMO_LET_GO_FIRST=1 for the negative control: the right arm lets go before the clamp shuts.)
    Right arm: closed fingers lowered behind the thumb press (fingers closing along y, so a finger's flat outer
    face is the push face), pushed +y at about 0.6 mL/s to 4 mm past the barrel's end, held there.
    Left arm: fingers open across the clamp's jaws (closing along x), lowered, closed on the jaws while the plunger is
    still pressed; then open and lifted. Right arm lifts off only after the clamp has caught. Both go home."""
    face0 = PRESS0_Y - PRESS_T / 2  # the thumb press's back face at the start
    ry0 = face0 - FINGER_HALF - 0.006
    ry1 = face0 + STROKE - FINGER_HALF + 0.008  # 8 mm past where the press stops at the barrel's end
    above = TOP + AXIS_Z + 0.07
    right = [Waypoint(2.0, (SX, ry0, above), FINGER_CLOSED, "over the plunger", 0.0),
             Waypoint(1.2, (SX, ry0, TOP + PUSH_Z), FINGER_CLOSED, "down behind the thumb press", 0.0),
             Waypoint(0.8, (SX, face0 - FINGER_HALF + 0.001, TOP + PUSH_Z), FINGER_CLOSED, "onto the press", 0.0),
             Waypoint(8.0, (SX, ry1, TOP + PUSH_Z), FINGER_CLOSED, "push the saline in", 0.0),
             Waypoint(5.0, (SX, ry1, TOP + PUSH_Z), FINGER_CLOSED, "keep pushing while the clamp shuts", 0.0)]
    t_push_end = sum(w.duration for w in right)
    if os.environ.get("DEMO_LET_GO_FIRST"):  # negative control: let go of the plunger before the clamp is shut
        right[-1] = Waypoint(0.5, (SX, ry1, TOP + PUSH_Z), FINGER_CLOSED, "let go before the clamp", 0.0)
        right.append(Waypoint(4.5, (SX, ry1 - 0.020, TOP + PUSH_Z + 0.04), FINGER_CLOSED, "off the press", 0.0))
    cz = TOP + sum(JAW_Z) / 2 - 0.012  # finger pads centered on the jaws
    cx = SX  # between the jaws once shut
    ly = -math.pi / 2  # fingers closing along x (the grip is 180-degree symmetric; -90 keeps joint6 in range)
    left = [Waypoint(2.0, (cx, CLAMP_Y, cz + 0.07), FINGER_OPEN, "over the clamp", ly),
            Waypoint(t_push_end - 5.0 - 2.0 - 1.0 + 1.5 + (3.0 if os.environ.get("DEMO_LET_GO_FIRST") else 0.0),
                     (cx, CLAMP_Y, cz + 0.07), FINGER_OPEN, "wait", ly),
            Waypoint(1.0, (cx, CLAMP_Y, cz), FINGER_OPEN, "fingers either side of the clamp", ly),
            Waypoint(1.2, (cx, CLAMP_Y, cz), FINGER_CLOSED, "squeeze the clamp shut", ly),
            Waypoint(0.8, (cx, CLAMP_Y, cz), FINGER_CLOSED, "hold", ly),
            Waypoint(0.6, (cx, CLAMP_Y, cz), FINGER_OPEN, "open", ly),
            Waypoint(0.8, (cx, CLAMP_Y, cz + 0.07), FINGER_OPEN, "lift clear", ly),
            kit.Home(1.8), kit.Home(1.5, label="stay home")]
    right += [Waypoint(0.8, (SX, ry1 - 0.020, TOP + PUSH_Z), FINGER_CLOSED, "back off the press", 0.0),
              Waypoint(0.8, (SX, ry1 - 0.020, above), FINGER_CLOSED, "lift clear", 0.0),
              kit.Home(1.8), kit.Home(1.5, label="stay home")]
    pad = sum(w.duration for w in right) - sum(w.duration for w in left)
    if pad > 0:
        left.append(kit.Home(pad, label="stay home"))
    elif pad < 0:
        right.append(kit.Home(-pad, label="stay home"))
    return {"left": left, "right": right}
