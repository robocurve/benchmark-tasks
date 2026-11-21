"""Unscrew a superglue tube's cap, flip it, and pierce the nozzle's foil seal with the cap's spike.

A superglue-style tube stands in a bench holder.  Its ribbed cap sits on a 1.5 mm-pitch neck thread and
carries a moulded piercing spike on the outside of its crown.  The cap must be turned 1.5 turns counter-clockwise to
free it, turned over, and driven spike-first, centred and square, into the foil that seals the nozzle (a breakable
joint that gives at 3 N), then withdrawn and released.  Squeezing the compliant tube body, shearing the nozzle with
a tilted spike, breaching the foil without the spike, or dropping the cap off the bench zeroes the episode.
All scoring logic lives in ``scoring.py`` (no simulator needed); this file builds the scene and feeds poses to it.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import bodykit  # noqa: E402
from bodykit import (Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, body_state, hinge,  # noqa: E402
                     on_reset, on_step, rotation, set_slide_target, set_thread_on, thread)
import scoring  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
INSTRUCTION = ("Unscrew the glue tube's cap, flip it over, and use the pin on top of the cap to pierce the foil seal on "
               "the nozzle without squeezing the tube. Leave the cap and tube released.")

TUBE_POS = np.array([0.300, 0.000, TOP + 0.100])   # shoulder base; the body hangs below, the neck rises above
CAP_POS = TUBE_POS + [0.0, 0.0, scoring.CAP_Z0]
FOIL_POS = TUBE_POS + [0.0, 0.0, scoring.FOIL_Z - 0.0006]   # 1.2 mm collision disc, outer face at FOIL_Z

BLUE = (0.10, 0.28, 0.72)
GREY = (0.82, 0.83, 0.85)
RED = (0.82, 0.10, 0.10)
FOIL_COLOR = (0.95, 0.80, 0.30)
DARK = (0.28, 0.30, 0.34)


def _holder() -> Body:
    shapes = [BoxShape((0, 0, 0.010), (0.070, 0.050, 0.020), DARK)]
    return Body.build("bench_holder", (TUBE_POS[0], TUBE_POS[1], TOP), 1.0, shapes, color=DARK, kinematic=True)


def _tube() -> Body:
    shapes = [
        BoxShape((0, 0, -0.040), (0.042, 0.014, 0.080), BLUE),                    # compliant LDPE body
        CylShape((0, 0, 0.006), 0.010, 0.012, GREY),                              # rigid shoulder
        CylShape((0, 0, 0.018), scoring.NECK_R, 0.012, GREY),                     # M14 x 1.5 neck
        CylShape((0, 0, 0.027), 0.0025, 0.006, GREY),                             # nozzle
    ]
    return Body.build("tube", TUBE_POS, 0.060, shapes, color=BLUE, kinematic=True)


def _foil() -> Body:
    # Held only by its breakable seal joint; once torn it is pushed into the (unmodelled) bore, not onto plastic.
    return Body.build("foil", FOIL_POS, 0.0005, [CylShape((0, 0, 0), 0.0045, 0.0012, FOIL_COLOR)],
                      color=FOIL_COLOR, friction=(0.6, 0.5), solver_iterations=32, ignore=("tube",))


def _cap() -> Body:
    """Hollow ribbed skirt (twelve wall panels, so the neck passes inside), crown plate, and the outer spike."""
    n, wall, height, r = 12, 0.001, 0.020, 0.0085
    chord = 2 * r * math.sin(math.pi / n) + 0.001
    shapes = [CylShape((0, 0, 0.021), 0.009, 0.002, RED), CylShape((0, 0, 0.0235), 0.0015, 0.0035, RED)]
    for k in range(n):
        a = 2 * math.pi * k / n
        shapes.append(BoxShape((r * math.cos(a), r * math.sin(a), height / 2), (chord, wall, height), RED,
                               yaw=a + math.pi / 2))
    # Ribbed polypropylene settles on the bench instead of rolling (rolling damping).
    # The nozzle's bore is not modelled (it is a solid primitive), so the cap does not collide with the tube body;
    # the thread joint carries the cap while seated and only the foil seal resists the pin.
    return Body.build("cap", CAP_POS, 0.008, shapes, color=RED, friction=(1.2, 1.0), damping=(0.3, 2.0),
                      solver_iterations=48, com=(0.0, 0.0, 0.012), ignore=("tube",))


HOLDER, TUBE, FOIL, CAP = _holder(), _tube(), _foil(), _cap()
OBJECTS = [HOLDER, TUBE, FOIL, CAP]

# Foil: a thin disc on a locked revolute joint that PhysX breaks for good at 3 N (a sharp pin through thin foil).
hinge("foil_seal", "tube", "foil", tuple(FOIL_POS), "Z", (-0.5, 0.5), 3.0, None)
# Thread: slides 3 mm at most while engaged; the task file feeds the slide target from the cap's turning.
thread("cap_thread", "tube", "cap", (0.0, 0.0, scoring.CAP_Z0), (0, 0, 1), (0.0, 0.0, 0.0), (0, 0, 1),
       (-0.0005, 0.0030), 0.0005, 3000.0, 4.0, 50.0, play=0.0003, tilt_deg=2.0)

TRACKER = scoring.Tracker(TUBE_POS, TOP)
LAY_DROP_Z = float(os.environ.get("TL_LAY_Z", "0.045"))  # lowest reachable tilted-grip height
_DBG = [0]


@on_reset
def _reset_episode():
    TRACKER.reset()
    set_slide_target("cap_thread", 0.0)
    set_thread_on("cap_thread", True)


def _arm_points(env):
    """TCP and finger-pad positions of both arms, in the task frame."""
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    tcps, pads = [], []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        poses = as_numpy(robot.data.body_link_pose_w)[0]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        tcps.append(poses[i, :3] - origin + rotation(poses[i, 3:7]) @ np.asarray(TCP_OFFSET))
        pads += [poses[k, :3] - origin for k, n in enumerate(robot.body_names)
                 if "tip_left" in n or "tip_right" in n]
    return tcps, pads


@on_step
def _follow(env):
    cap = body_state(env, CAP)
    foil = body_state(env, FOIL)
    tcps, pads = _arm_points(env)
    cmd = TRACKER.update(scoring.Frame(np.asarray(cap.pos), cap.rotation(), np.asarray(foil.pos), pads, tcps,
                                       bool(cap.resting() and foil.resting())))
    if cmd.release_thread:
        set_thread_on("cap_thread", False)
    elif not TRACKER.thread_released:
        set_slide_target("cap_thread", cmd.slide_target)
    _DBG[0] += 1
    if os.environ.get("TL_DEBUG") and _DBG[0] % 20 == 0:
        print("[tl]", {"step": _DBG[0], "unwind_deg": round(math.degrees(TRACKER.unwind), 1),
                       "released_thread": TRACKER.thread_released, "flags": {k: v for k, v in TRACKER.flags.items() if v},
                       "cap_mm": np.round(np.asarray(cap.pos) * 1000, 1).tolist(),
                       "foil_mm": np.round(np.asarray(foil.pos) * 1000, 2).tolist(),
                       "cap_axis": np.round(cap.rotation()[:, 2], 3).tolist(),
                       "tcps_mm": [np.round(t * 1000, 1).tolist() for t in tcps],
                       "score": TRACKER.score()}, flush=True)


def score(objects=None) -> float:
    return TRACKER.score()


def task_score(*args, **kwargs) -> float:
    return TRACKER.score()


def success(objects=None) -> bool:
    return score() == 1.0


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def demo():
    """Right arm only (the left stays at ready): ratchet the cap off in four counter-clockwise strokes, tip it onto its
    side and set it down, re-grip it from above and tilt it pin-down, drive the pin through the foil, withdraw, turn
    the cap back onto its side and lay it on the bench.

    Open-loop waypoints from the known start layout; it reads no simulator state.  Reaches 1.0 in YAM Arena (seed 0)."""
    down = grip_rotation(0.0)
    right: list[Waypoint] = []

    def go(t, p, finger, label, rot=down):
        right.append(Waypoint(t, tuple(float(v) for v in p), finger, label, rot))

    grip_z = scoring.CAP_Z0 + 0.012
    grip = np.array([TUBE_POS[0], TUBE_POS[1], TUBE_POS[2] + grip_z])
    # Four 160-degree counter-clockwise ratchet strokes (640 deg > the 1.5-turn thread): each stroke starts with the
    # wrist turned 60 deg clockwise, which keeps joint 6 inside its +-120 deg range at this reach.
    start_yaw, stroke_deg, strokes = -60.0, 160.0, 4
    rise = scoring.PITCH * stroke_deg / 360.0
    go(2.5, grip + [0, 0, 0.035], FINGER_OPEN, "move over the cap", _rz(math.radians(start_yaw)) @ down)
    for stroke in range(strokes):
        z0 = rise * stroke
        r0 = _rz(math.radians(start_yaw)) @ down
        go(1.0, grip + [0, 0, z0], FINGER_OPEN, "lower beside the cap", r0)
        go(0.6, grip + [0, 0, z0], FINGER_CLOSED, "pinch the cap", r0)
        for k in range(1, 9):
            go(0.35, grip + [0, 0, z0 + rise * k / 8], FINGER_CLOSED, "turn counter-clockwise",
               _rz(math.radians(start_yaw + stroke_deg * k / 8)) @ down)
        r1 = _rz(math.radians(start_yaw + stroke_deg)) @ down
        go(0.5, grip + [0, 0, z0 + rise], FINGER_OPEN, "let go", r1)
        go(0.5, grip + [0, 0, z0 + rise + 0.004], FINGER_OPEN, "lift off the ribs", r1)
        go(1.0, grip + [0, 0, z0 + rise + 0.004], FINGER_OPEN, "turn the wrist back", r0)
    top_z = rise * strokes
    # Take the freed cap with the fingers closing along x, so a 90-degree tilt about x keeps them across the cap.
    d90 = _rz(math.pi / 2) @ down
    tilt = _rx(math.pi / 2) @ d90            # approach along +y, fingers along x
    go(1.0, grip + [0, 0, top_z + 0.004], FINGER_OPEN, "turn the open fingers across the cap", d90)
    go(1.0, grip + [0, 0, top_z], FINGER_OPEN, "lower around the freed cap", d90)
    go(0.6, grip + [0, 0, top_z], FINGER_CLOSED, "take the freed cap", d90)
    if os.environ.get("TL_STAGE") == "unscrew":
        go(1.5, grip + [0, 0, top_z + 0.035], FINGER_CLOSED, "lift the cap clear of the neck", d90)
        go(3.0, grip + [0, 0, top_z + 0.035], FINGER_CLOSED, "hold it up", d90)
        return {"right": right}
    # First quarter turn: lay the cap on its side (pin toward -y) on the bench beside the tube.
    lay = np.array([TUBE_POS[0], TUBE_POS[1] - 0.080, 0.0])
    go(1.5, grip + [0, 0, top_z + 0.035], FINGER_CLOSED, "lift the cap clear of the neck", d90)
    go(2.0, (lay[0], lay[1], TOP + 0.100), FINGER_CLOSED, "carry it beside the tube", d90)
    go(2.5, (lay[0], lay[1], TOP + 0.100), FINGER_CLOSED, "tip it onto its side", tilt)
    go(1.5, (lay[0], lay[1], TOP + LAY_DROP_Z), FINGER_CLOSED, "lower it toward the bench", tilt)
    go(0.5, (lay[0], lay[1], TOP + LAY_DROP_Z), FINGER_OPEN, "let it settle on its side", tilt)
    go(1.0, (lay[0], lay[1], TOP + 0.100), FINGER_OPEN, "back off", tilt)
    # Second quarter turn: pick the lying cap from above across its body and tilt it pin-down.
    # Pinch toward the open end (+y) so the fingertips stay above the neck when the pin reaches the foil.
    lying = np.array([lay[0], lay[1] + 0.008, TOP + 0.009])
    go(2.0, lying + [0, 0, 0.050], FINGER_OPEN, "move over the lying cap", d90)
    go(1.2, lying, FINGER_OPEN, "lower around the lying cap", d90)
    go(0.6, lying, FINGER_CLOSED, "pinch it across its body", d90)
    go(1.2, lying + [0, 0, 0.090], FINGER_CLOSED, "lift it", d90)
    go(2.5, lying + [0, 0, 0.090], FINGER_CLOSED, "tilt it pin-down", tilt)
    plane = TUBE_POS[2] + scoring.FOIL_Z
    # Pin tip relative to the TCP once the regrasped cap hangs pin-down (measured in YAM Arena).
    tcp_to_pin = np.array([0.0007, 0.0058, -0.0180])

    def tcp_for_depth(d):
        return np.array([TUBE_POS[0], TUBE_POS[1], plane - d]) - tcp_to_pin

    go(2.5, tcp_for_depth(-0.030), FINGER_CLOSED, "bring the pin over the nozzle", tilt)
    go(1.5, tcp_for_depth(-0.006), FINGER_CLOSED, "descend until the pin nearly touches", tilt)
    go(3.5, tcp_for_depth(0.0250), FINGER_CLOSED, "press straight down through the foil", tilt)
    go(0.5, tcp_for_depth(0.0250), FINGER_CLOSED, "hold", tilt)
    go(1.5, tcp_for_depth(-0.040), FINGER_CLOSED, "withdraw the pin", tilt)
    # Turn it back onto its side before putting it down: pin-first it would balance and wobble on the pin.
    go(2.0, lying + [0, 0, 0.090], FINGER_CLOSED, "carry the cap aside", tilt)
    go(2.5, lying + [0, 0, 0.090], FINGER_CLOSED, "turn it back onto its side", d90)
    go(1.5, lying + [0, 0, 0.002], FINGER_CLOSED, "lay it on the bench", d90)
    go(0.6, lying + [0, 0, 0.002], FINGER_OPEN, "let go", d90)
    go(1.2, lying + [0, 0, 0.080], FINGER_OPEN, "lift clear", d90)
    go(3.0, lying + [0, 0, 0.080], FINGER_OPEN, "hold clear of the cap and nozzle", d90)
    # The left arm is omitted, so the scripted policy holds it at its ready posture.
    return {"right": right}
