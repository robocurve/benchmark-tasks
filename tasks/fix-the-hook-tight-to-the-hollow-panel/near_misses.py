"""Near-successes and unintended strategies, scored offline by the factory checker. Each case: (label, {object: spec},
expected score, expected success); a spec is a ready state from the part's own geometry (HOOK.state(...)). Cases run
in order and share the whole-trial record (the toggle running off the bolt), cleared only by a scene reset.

Offline there is no robot and no physics, so the arms' clearance is set here as measured clear (in the simulator it is
measured every step and is None, no credit, until then), and the pull test's response is given as the state the hook
takes while it is pulled: a clamped hook does not move, a loose one lifts until its wings meet the back of the board.
"""

import math

import numpy as np

import parts
from task import (DRIVER, FACE, GAP0, HOLE, HOOK, LEAD, LIFT_MAX, LIFT_MIN, PLATE_Y, PULL_N, PULL_STEPS, RUN_OFF,
                  SETTLE_STEPS, STILL_STEPS, TOP)

parts.__dict__.setdefault("_hook_task_grip", {})["clear"] = True
# Offline stand-in for the simulator's wrench: PULL_N is reported as applied on every pull step, except while the
# hook is in a state marked by no_wrench() (the pull failed to act).
_NO_WRENCH = set()
parts.__dict__.setdefault("_hook_task_pull", {"phase": "idle"})["offline_applied"] = (
    lambda objects: 0.0 if id(objects["hook"]) in _NO_WRENCH else PULL_N)

X0, Y0, Z0 = HOOK.pos


def quat_z(deg):
    a = math.radians(deg) / 2
    return (0.0, 0.0, math.sin(a), math.cos(a))


def quat_x(deg):
    a = math.radians(deg) / 2
    return (math.sin(a), 0.0, 0.0, math.cos(a))


def hook(thread=GAP0, dz=0.0, yaw=0.0, quat=None, vel=None, pos=None):
    q = quat if quat is not None else quat_z(yaw)
    p = pos if pos is not None else (X0, Y0, Z0 + dz)
    return {"hook": HOOK.state(pos=p, quat_xyzw=q, lin_vel=vel,
                               joints={"thread": thread, "nut": thread / LEAD * 2 * math.pi})}


def rocked(lift, thread=GAP0):
    """The plate's hook end lifted by `lift`, the plate pivoting on its other end's edge on the face (rocking)."""
    a = math.atan2(lift, PLATE_Y[1] - PLATE_Y[0])
    r = parts._quat_to_matrix(quat_x(math.degrees(a)))
    pivot = np.array([X0, Y0 + PLATE_Y[0], Z0])
    pos = pivot - r @ np.array([0.0, PLATE_Y[0], 0.0])
    return hook(thread=thread, quat=quat_x(math.degrees(a)), pos=tuple(pos))


def no_wrench(state):
    _NO_WRENCH.add(id(state["hook"]))
    return state


TIGHT = hook()
GIVE = 0.00027  # measured on the GPU: a tight hook's end rocks up 0.27 mm under the 5 N pull
TIGHT_PULLED = rocked(GIVE)
RESET = {}


def tested(label, state, score, pulled=None, success=None):
    """A state reached, held still for 1 s, then put through the pull test. score() counts control steps with the
    hook's pose unchanged and at rest; after 20 of them (1 s at 20 Hz) the test starts: PULL_STEPS steps pulled (the
    hook in `pulled`, the state the simulator shows under the pull; default: a tight hook's measured give), then SETTLE_STEPS steps
    released, after which the score is given. success() sees the result one step later. Each case starts from the hook
    lifted 2 mm (a different pose), which also restarts the test."""
    pulled = TIGHT_PULLED if pulled is None else pulled
    out = [(f"{label}: hook still settling, 2 mm up", {**state, **hook(dz=0.002)}, 0.0, False)]
    out += [(f"{label} (step {k + 1} of the 1 s hold)", state, 0.0, False) for k in range(STILL_STEPS)]
    out.append((f"{label} (held 1 s: the pull test starts, no credit yet)", state, 0.0, False))
    out += [(f"{label} (pulled, step {k + 1})", pulled, 0.0, False) for k in range(PULL_STEPS)]
    out += [(f"{label} (pull off, step {k + 1})", state, 0.0, False) for k in range(SETTLE_STEPS - 1)]
    out.append((f"{label} (pull test done)", state, score, False))
    out.append((f"{label} (pull test done + 1 step)", state, score, score == 1.0 if success is None else success))
    return out


CASES = [
    ("start: hook loose, wings 3 mm behind the back face", {}, 0.0, False),
    *tested("clamped tight and square, hook end gives 0.27 mm under the pull", TIGHT, 1.0),
    # fail closed: the pull must really have been applied, and must visibly act
    *tested("clamped, but the pull was never applied (wrench not set)", TIGHT, 0.0, pulled=no_wrench(rocked(GIVE))),
    *tested(f"clamped, pull applied but no corner rose {LIFT_MIN * 1000:.2f} mm", TIGHT, 0.0,
            pulled=hook(dz=LIFT_MIN - 0.00001)),
    ("then the hook moving: the hold and the test start again", hook(vel=(0.0, 0.0, 0.02)), 0.0, False),
    ("tight and at rest again, first step", TIGHT, 0.0, False),
    *tested("clamped, 2.9 degrees off the line", hook(yaw=2.9), 1.0),
    *tested("clamped, 3.1 degrees off the line", hook(yaw=-3.1), 0.5),
    *tested("clamped, 10 degrees off the line", hook(yaw=10.0), 0.5),
    *tested("clamped, 10.5 degrees off the line (no test)", hook(yaw=10.5), 0.0),
    *tested("clamped but turned round (plate on the hole's other side)", hook(yaw=180.0), 0.0),
    # the pull test: displacement under the pull
    *tested(f"clamped, plate lifts {LIFT_MAX * 1000:.1f} mm under the pull (limit)", TIGHT, 1.0,
            pulled=hook(dz=LIFT_MAX - 1e-7)),
    *tested(f"plate lifts {LIFT_MAX * 1000 + 0.05:.2f} mm under the pull", TIGHT, 0.0,
            pulled=hook(dz=LIFT_MAX + 0.00005)),
    *tested(f"hook end rocks up {LIFT_MAX * 1000:.1f} mm under the pull (limit)", TIGHT, 1.0,
            pulled=rocked(LIFT_MAX - 1e-7)),
    *tested(f"hook end rocks up {LIFT_MAX * 1000 + 0.05:.2f} mm under the pull", TIGHT, 0.0, pulled=rocked(LIFT_MAX + 0.00005)),
    # loose hooks resting flat on the face: under the pull they lift until the wings meet the back of the board
    *tested("resting flat, wings 0.5 mm off the back face (unclamped)", hook(thread=GAP0 - 0.0005), 0.0,
            pulled=rocked(0.0022, thread=GAP0 - 0.0005)),  # 2.2 mm measured on the GPU
    *tested("resting flat, half the thread taken up", hook(thread=GAP0 / 2), 0.0,
            pulled=hook(thread=GAP0 / 2, dz=GAP0 / 2)),
    *tested("turned without the pull: toggle spun, no travel", hook(thread=0.0), 0.0,
            pulled=hook(thread=0.0, dz=GAP0)),
    *tested("pulled clear and not back where it was after the test", TIGHT, 0.0, pulled=hook(dz=0.0002)) [:-2]
    + [("pulled clear and not back where it was after the test (pull test done)", hook(dz=0.0003), 0.0, False)],
    # flush on the face, measured from outside
    *tested("clamped, plate 0.4 mm off the face", hook(dz=0.0004), 1.0, pulled=hook(dz=0.0004 + GIVE)),
    *tested("clamped, plate 0.6 mm off the face (no test)", hook(dz=0.0006), 0.0),
    *tested("plate tilted 1 degree (one end lifted, no test)", hook(quat=quat_x(1.0)), 0.0),
    *tested("clamped, screwdriver left standing in the head",
            {**TIGHT, "screwdriver": DRIVER.state(pos=(HOLE[0], HOLE[1], TOP + FACE + 0.0046))}, 0.0),
    *tested("clamped, screwdriver standing on the panel 4 mm from the plate",
            {**TIGHT, "screwdriver": DRIVER.state(pos=(HOLE[0] + 0.02 + 0.004 + 0.0175, HOLE[1] + 0.05, TOP + FACE))},
            0.0),
    *tested("clamped, screwdriver back in its holder", TIGHT, 1.0),
    # whole trial: the toggle run off the bolt's end (unscrewed) is lost in the cavity
    ("toggle unscrewed to the bolt's end", hook(thread=RUN_OFF), 0.0, False),
    *tested("then tight: still 0", TIGHT, 0.0),
    ("hook lifted 30 mm", hook(thread=0.0, dz=0.030), 0.0, False),
    ("scene reset (hook back at its start in one step)", RESET, 0.0, False),
    *tested("after the reset, tight and square", TIGHT, 1.0),
]
