"""Near-successes and unintended strategies, scored offline by the factory checker.

Each case: (label, {object: (centre, size or None, velocity or None[, quat_xyzw])}, expected score, expected
success). Cases run in order and share the task's whole-trial tilt record.
"""

import numpy as np

from task import HB as H, SPACE_HALF_Y, SPACE_X0, SPACE_X1, START, TOP, W

Z = TOP + H / 2


def yaw(a_deg):
    a = np.radians(a_deg) / 2
    return (0.0, 0.0, np.sin(a), np.cos(a))


def tilted(a_deg, about="x"):
    a = np.radians(a_deg) / 2
    return (np.sin(a), 0.0, 0.0, np.cos(a)) if about == "x" else (0.0, np.sin(a), 0.0, np.cos(a))


def lifted_tilt(a_deg):
    """Resting on one bottom edge, tilted a_deg about the x axis: centre height of the tilted box."""
    a = np.radians(a_deg)
    return (W / 2) * np.sin(a) + (H / 2) * np.cos(a)


IN = SPACE_X0 + W / 2  # centre x with the near face exactly on the space's near line
MOVING = (0.0, 0.0, -0.05)
START_POS = (START[0], START[1], Z)
W_ = "washer"

CASES = [
    ("start pose", {W_: (START_POS, None, None)}, 0.0, False),
    ("fully in the space, square", {W_: ((IN + 0.03, 0.0, Z), None, None)}, 1.0, True),
    ("near face exactly on the near line", {W_: ((IN, 0.0, Z), None, None)}, 1.0, True),
    ("near face 1 mm short of the line (moved 50 mm)", {W_: ((IN - 0.001, 0.0, Z), None, None)}, 0.5, False),
    ("far face exactly on the far line", {W_: ((SPACE_X1 - W / 2, 0.0, Z), None, None)}, 1.0, True),
    ("over the far line by 1 mm", {W_: ((SPACE_X1 - W / 2 + 0.001, 0.0, Z), None, None)}, 0.5, False),
    ("side exactly on the left line", {W_: ((IN + 0.03, SPACE_HALF_Y - W / 2, Z), None, None)}, 1.0, True),
    ("over the left line by 1 mm", {W_: ((IN + 0.03, SPACE_HALF_Y - W / 2 + 0.001, Z), None, None)}, 0.5, False),
    ("in the space, turned 15 degrees (corners inside)", {W_: ((IN + 0.03, 0.0, Z), None, None, yaw(15))}, 1.0, True),
    ("centred in the space but turned 30 degrees: a corner over the line",
     {W_: ((IN + 0.01, 0.0, Z), None, None, yaw(30))}, 0.5, False),
    ("in the space but still moving", {W_: ((IN + 0.03, 0.0, Z), None, MOVING)}, 0.0, False),
    ("in the space, leaning 6 degrees (held on an edge)",
     {W_: ((IN + 0.03, 0.0, TOP + lifted_tilt(6)), None, None, tilted(6))}, 0.0, False),
    ("in the space, rocked 0.5 degree (corners within 1.5 mm)",
     {W_: ((IN + 0.03, 0.0, TOP + lifted_tilt(0.5)), None, None, tilted(0.5))}, 1.0, True),
    ("in the space, rocked 2 degrees on one edge (a corner 3.8 mm up)",
     {W_: ((IN + 0.03, 0.0, TOP + lifted_tilt(2)), None, None, tilted(2))}, 0.0, False),
    ("in the space, held 2 mm off the table (lift limit)", {W_: ((IN + 0.03, 0.0, Z + 0.002), None, None)}, 0.0, False),
    ("then set down in the space (lifted exactly 2 mm: allowed)", {W_: ((IN + 0.03, 0.0, Z), None, None)}, 1.0, True),
    ("lifted clear, 2.1 mm up", {W_: ((IN + 0.03, 0.0, Z + 0.0021), None, None)}, 0.0, False),
    ("then set down in the space after the lift", {W_: ((IN + 0.03, 0.0, Z), None, None)}, 0.0, False),
    ("then 30 mm into its walk", {W_: ((START[0] + 0.03, 0.0, Z), None, None)}, 0.0, False),
    ("scene reset after the lift", {W_: (START_POS, None, None)}, 0.0, False),
    ("moved 30 mm toward the space (limit)", {W_: ((START[0] + 0.030, 0.0, Z), None, None)}, 0.5, False),
    ("moved 29 mm", {W_: ((START[0] + 0.029, 0.0, Z), None, None)}, 0.0, False),
    ("slid 40 mm sideways, not forward", {W_: ((START[0], 0.04, Z), None, None)}, 0.0, False),
    ("pulled back toward the robot", {W_: ((START[0] - 0.05, 0.0, Z), None, None)}, 0.0, False),
    # Whole-trial record: a 19 degree rock is fine, a 21 degree one is not, even once it is set back upright.
    ("rocked 19 degrees on an edge (mid-walk)", {W_: ((START[0], 0.0, TOP + lifted_tilt(19)), None, None, tilted(19))},
     0.0, False),
    ("then in the space after a 19 degree rock", {W_: ((IN + 0.03, 0.0, Z), None, None)}, 1.0, True),
    ("rocked 21 degrees", {W_: ((IN + 0.03, 0.0, TOP + lifted_tilt(21)), None, None, tilted(21))}, 0.0, False),
    ("then set back upright in the space", {W_: ((IN + 0.03, 0.0, Z), None, None)}, 0.0, False),
    ("then walked back 30 mm into the space direction", {W_: ((START[0] + 0.03, 0.0, Z), None, None)},
     0.0, False),
    ("scene reset to the start pose", {W_: (START_POS, None, None)}, 0.0, False),
    ("after the reset, in the space", {W_: ((IN + 0.03, 0.0, Z), None, None)}, 1.0, True),
    ("tipped over onto its side, lying in the space",
     {W_: ((IN + 0.10, 0.0, TOP + W / 2), (W, W, H), None, tilted(90, "y"))}, 0.0, False),
    ("after a tip-over, set back upright in the space", {W_: ((IN + 0.03, 0.0, Z), None, None)}, 0.0, False),
    ("scene reset", {W_: (START_POS, None, None)}, 0.0, False),
    ("tipped forward 30 degrees, resting against an arm", {W_: ((IN, 0.0, Z), None, None, tilted(30, "y"))}, 0.0, False),
    ("upright again afterwards, moved 40 mm", {W_: ((START[0] + 0.04, 0.0, Z), None, None)}, 0.0, False),
]


def _run() -> int:
    import task
    from yam_arena.task_file import ObjectState

    def state(pos, size, vel=None, quat=None):
        pos = np.asarray(pos, float)
        q = np.array([0.0, 0.0, 0.0, 1.0]) if quat is None else np.asarray(quat, float) / np.linalg.norm(quat)
        r = task._rot(q)
        half = np.abs(r) @ (np.asarray(size or (W, W, H), float) / 2)
        v = np.zeros(3) if vel is None else np.asarray(vel, float)
        return ObjectState(pos, q, v, np.zeros(3), pos - half, pos + half)

    bad = 0
    for label, changes, want, want_ok in CASES:
        st = {n: state(*spec) for n, spec in changes.items()}
        ok = bool(task.success(st))
        got = 1.0 if ok else float(task.score(st))
        flag = "" if (ok == want_ok and abs(got - want) < 1e-9) else "  <-- MISMATCH"
        bad += bool(flag)
        print(f"{got:4.2f} {ok!s:5} {label}{flag}")
    return bad


if __name__ == "__main__":
    raise SystemExit(_run())
