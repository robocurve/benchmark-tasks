"""Near-successes and unintended strategies, scored offline.

Each case: (label, {object: ready state}, expected score, expected success). Cases run in order on one copy of the
task module, so the largest hook extension carries over from case to case, as it does over a trial. The cases that
push the hook deep come last.
"""

import math

from task import (BODY, HANG_X, HOOK_REST, PIN_TIP_X, PIN_Z, POST, RING_IN_H, STATIC_EXT, TOP, W_START, WEIGHT)

MOVING = (0.0, 0.0, -0.05)


def hook(ext):
    return POST.state(joints={"hook": HOOK_REST + ext})


def hung(ext, x=HANG_X, dz=0.0, quat=None, vel=None):
    """The weight with its ring's top bar on the pin, the hook stretched ext below its unloaded rest."""
    z = PIN_Z - HOOK_REST - ext - (BODY[2] / 2 + RING_IN_H) + dz
    return WEIGHT.state(pos=(x, 0.0, z), quat_xyzw=quat, lin_vel=vel)


def tilt_x(deg):
    a = math.radians(deg) / 2
    return (math.sin(a), 0.0, 0.0, math.cos(a))


S = STATIC_EXT
CASES = [
    ("hung and let go, hook at its static stretch", {"spring_hook": hook(S), "weight": hung(S)}, 1.0, True),
    ("hook 2 mm past static (limit)", {"spring_hook": hook(S + 0.002), "weight": hung(S + 0.002)}, 1.0, True),
    ("hook 2 mm short of static (limit)", {"spring_hook": hook(S - 0.002), "weight": hung(S - 0.002)}, 1.0, True),
    ("hung, hanging 8 deg out of plumb", {"spring_hook": hook(S), "weight": hung(S, quat=tilt_x(8))}, 1.0, True),
    ("gripper still carries part of the weight (hook 2.1 mm short)",
     {"spring_hook": hook(S - 0.0021), "weight": hung(S - 0.0021)}, 0.0, False),
    ("gripper pressing the weight down (hook 2.1 mm past static)",
     {"spring_hook": hook(S + 0.0021), "weight": hung(S + 0.0021)}, 0.0, False),
    ("hung but still bouncing", {"spring_hook": hook(S), "weight": hung(S, vel=MOVING)}, 0.0, False),
    ("weight back on the table where it started", {"weight": WEIGHT.state(pos=W_START)}, 0.0, False),
    ("weight on the table under the hook", {"weight": WEIGHT.state(pos=(HANG_X, 0.0, TOP + BODY[2] / 2))}, 0.0, False),
    ("body balanced on top of the pin, ring not threaded",
     {"spring_hook": hook(S), "weight": WEIGHT.state(pos=(HANG_X, 0.0, PIN_Z - HOOK_REST - S + BODY[2] / 2))},
     0.0, False),
    ("ring in front of the pin's tip", {"spring_hook": hook(S), "weight": hung(S, x=PIN_TIP_X - 0.01)}, 0.0, False),
    # history: the largest extension of the trial
    ("hook once pulled to 36.9 mm, then hung and let go",
     {"spring_hook": hook(0.0369), "weight": hung(0.0369)}, 0.0, False),
    ("... then settled at static: pointer just above the red", {"spring_hook": hook(S), "weight": hung(S)}, 1.0, True),
    ("hook reaches 37.0 mm (the red band)", {"spring_hook": hook(0.037), "weight": hung(0.037)}, 0.0, False),
    ("... then settled at static: pointer in the red", {"spring_hook": hook(S), "weight": hung(S)}, 0.0, False),
    ("... weight taken off and put back gently: still in the red", {"weight": WEIGHT.state(pos=W_START)}, 0.0, False),
    ("... hung again", {"spring_hook": hook(S), "weight": hung(S)}, 0.0, False),
]


if __name__ == "__main__":
    import task

    bad = 0
    start = {o.name: o.state() for o in task.OBJECTS}
    for label, changes, want, want_ok in CASES:
        st = dict(start) | changes
        ok = bool(task.success(st))
        got = 1.0 if ok else float(task.score(st))
        flag = "ok  " if (got, ok) == (want, want_ok) else "FAIL"
        bad += flag == "FAIL"
        print(f"{flag} {got:4.2f}  {label}")
    raise SystemExit(bad)
