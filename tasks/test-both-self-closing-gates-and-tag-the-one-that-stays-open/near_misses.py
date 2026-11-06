"""Near-successes and unintended strategies, scored offline by the factory checker.

Each case: (label, {object: (centre, size or None, velocity or None[, quat_xyzw]) or a ready state}, expected score,
expected success). Cases run in order against one copy of the task, so the trial record (how far each gate opened,
whether the faulty gate stood open) carries from case to case like control steps; each case is one step.
"""

import math

from task import LEFT_GATE, PAD_TOP, PAD_XY, RIGHT_GATE, TAG, TAG_BOX, TOP

D = math.radians
HALF = TAG[2] / 2


def gates(left=0.0, right=0.0):
    return {"left_gate": LEFT_GATE.state(joints={"gate": left}), "right_gate": RIGHT_GATE.state(joints={"gate": right})}


def tag_at(x, y, z=PAD_TOP + HALF, vel=None, quat=None):
    return {"red_tag": ((x, y, z), None, vel, quat)} if quat else {"red_tag": ((x, y, z), None, vel)}


RX, RY = PAD_XY[0], -PAD_XY[1]  # right (faulty) gate's pad
ON_RIGHT = tag_at(RX, RY)
ON_LEFT = tag_at(PAD_XY[0], PAD_XY[1])
START_TAG = {"red_tag": (TAG_BOX.pos, None, None)}


def scene_reset():
    return [("tag far away", {**gates(), **tag_at(0.10, 0.20, TOP + HALF)}, 0.0, False),
            ("scene reset: everything back at its start", {**gates(), **START_TAG}, 0.0, False)]


s20 = math.sin(D(20))
c20 = math.cos(D(20))
TILT20 = (math.sin(D(10)), 0.0, 0.0, math.cos(D(10)))  # 20 degrees about x
YAW30 = (0.0, 0.0, math.sin(D(15)), math.cos(D(15)))
YAW45 = (0.0, 0.0, math.sin(D(22.5)), math.cos(D(22.5)))
YAW45_REACH = HALF * math.sqrt(2)  # corner reach of the tag turned 45 degrees

def steps(left, right, n, tag=START_TAG, label="step"):
    return [(f"{label} (left {math.degrees(left):.1f}, right {math.degrees(right):.1f} deg) step {k + 1}",
             {**gates(left, right), **tag}, 0.0, False) for k in range(n)]


def full_test(right=D(30), left=D(30)):
    """Left opened and let go (it swings shut while the right gate is still), then the right left standing 1 s."""
    return [*steps(left, 0.0, 1, label="open left"), *steps(0.0, 0.0, 1, label="left swings shut"),
            *steps(0.0, right, 21, label="right standing open")]


CASES = [
    # Guessing without testing.
    ("tag on the faulty gate's pad, nothing tested", {**gates(), **ON_RIGHT}, 0.0, False),
    *scene_reset(),
    # The intended order: test both, then tag.
    *full_test(),
    ("tested; tag still at its start", {**gates(0.0, D(30)), **START_TAG}, 0.0, False),
    ("tag on right pad, right gate still open 30 deg", {**gates(0.0, D(30)), **ON_RIGHT}, 0.0, False),
    ("tag on right pad, both shut", {**gates(), **ON_RIGHT}, 1.0, True),
    ("right gate exactly 5 deg open", {**gates(0.0, D(5)), **ON_RIGHT}, 1.0, True),
    ("right gate 5.1 deg open", {**gates(0.0, D(5.1)), **ON_RIGHT}, 0.0, False),
    ("left gate held 6 deg open", {**gates(D(6), 0.0), **ON_RIGHT}, 0.0, False),
    ("tag on the working gate's pad, both shut", {**gates(), **ON_LEFT}, 0.0, False),
    ("tag still moving on the right pad", {**gates(), **tag_at(RX, RY, vel=(0.0, 0.0, -0.05))}, 0.0, False),
    ("tag held 2 mm above the right pad", {**gates(), **tag_at(RX, RY, PAD_TOP + 0.002 + HALF)}, 0.0, False),
    ("tag exactly 1 mm above the right pad", {**gates(), **tag_at(RX, RY, PAD_TOP + 0.001 + HALF)}, 1.0, True),
    ("tag sunk 1 mm (pad top 1 mm above its bottom)", {**gates(), **tag_at(RX, RY, PAD_TOP - 0.001 + HALF)}, 1.0, True),
    ("tag flush with the pad's outer edge", {**gates(), **tag_at(RX, RY - 0.015)}, 1.0, True),
    ("tag 1 mm over the pad's outer edge", {**gates(), **tag_at(RX, RY - 0.016)}, 0.0, False),
    ("tag 1 mm over the pad's far edge", {**gates(), **tag_at(RX + 0.016, RY)}, 0.0, False),
    ("tag on the table beside the pad", {**gates(), **tag_at(RX, RY + 0.06, TOP + HALF)}, 0.0, False),
    ("tag turned 30 deg on the pad", {**gates(), **tag_at(RX, RY, quat=YAW30)}, 1.0, True),
    ("tag turned 45 deg, corner 1 mm over the edge", {**gates(), **tag_at(RX + 0.030 - YAW45_REACH + 0.001, RY,
                                                                           quat=YAW45)}, 0.0, False),
    ("tag tipped 20 deg, corner on the pad",
     {**gates(), **tag_at(RX, RY, PAD_TOP + HALF * (c20 + s20), quat=TILT20)}, 0.0, False),
    ("tag on its side on the pad", {**gates(), **tag_at(RX, RY, quat=(math.sin(D(45)), 0, 0, math.cos(D(45))))},
     1.0, True),
    ("tag on the centre panel's top", {**gates(), **tag_at(0.33, 0.0, TOP + 0.090 + HALF)}, 0.0, False),
    ("tag back on the right pad", {**gates(), **ON_RIGHT}, 1.0, True),
    # A new trial: the record is cleared by the reset.
    *scene_reset(),
    ("after reset, tag on right pad, both shut", {**gates(), **ON_RIGHT}, 0.0, False),
    # Probe: tag on the right pad first, then test both gates (the tag stays on the pad throughout).
    *scene_reset(),
    ("probe: tag set on the right pad before any test", {**gates(), **ON_RIGHT}, 0.0, False),
    *[(label, {**ch, **ON_RIGHT}, sc, su) for label, ch, sc, su in full_test()],
    ("probe: both tested after the tag, both shut", {**gates(), **ON_RIGHT}, 0.0, False),
    # The tag lowered to 5 mm over a pad before testing counts as touching it; 6 mm does not.
    *scene_reset(),
    ("tag carried 5 mm over the left pad before testing", tag_at(PAD_XY[0], PAD_XY[1], PAD_TOP + 0.005 + HALF,
                                                                    vel=(0.1, 0, 0)), 0.0, False),
    *full_test(),
    ("tag on right pad after touching the left one early", {**gates(), **ON_RIGHT}, 0.0, False),
    *scene_reset(),
    ("tag carried 6 mm over the left pad before testing", tag_at(PAD_XY[0], PAD_XY[1], PAD_TOP + 0.006 + HALF,
                                                                    vel=(0.1, 0, 0)), 0.0, False),
    *full_test(),
    ("tag on right pad, the earlier pass was 6 mm up", {**gates(), **ON_RIGHT}, 1.0, True),
    # Left gate shut by hand while the right gate is being moved: not a self-close.
    *scene_reset(),
    *steps(D(30), 0.0, 1, label="open left"),
    *steps(0.0, D(10), 1, label="left back shut while right moves"),
    *steps(0.0, D(30), 21, label="right standing open"),
    ("tag on right pad, left never seen to self-close", {**gates(), **ON_RIGHT}, 0.0, False),
    # Left reaching only 5.1 deg from shut does not count as shut.
    *scene_reset(),
    *steps(D(30), 0.0, 1, label="open left"),
    *steps(D(5.1), 0.0, 1, label="left back to 5.1 deg"),
    *steps(D(5.1), D(30), 21, label="right standing open"),
    ("tag on right pad, left stopped at 5.1 deg", {**gates(D(5.1), 0.0), **ON_RIGHT}, 0.0, False),
    # Exactly 15 degrees counts; 14.9 does not.
    *scene_reset(),
    *full_test(right=D(15), left=D(14.9)),
    ("tag on right pad, left reached only 14.9 deg", {**gates(), **ON_RIGHT}, 0.0, False),
    *scene_reset(),
    *full_test(right=D(15), left=D(15)),
    ("tag on right pad, both reached exactly 15 deg", {**gates(), **ON_RIGHT}, 1.0, True),
    # Faulty gate held at 14.9 deg for 2 s does not count as standing open.
    *scene_reset(),
    *full_test(right=D(14.9)),
    *steps(0.0, D(14.9), 20, label="right at 14.9"),
    ("tag on right pad, faulty never stood 1 s at 15 deg", {**gates(), **ON_RIGHT}, 0.0, False),
    # Moving slowly (0.2 deg a step) while open is not standing still.
    *scene_reset(),
    *steps(D(30), 0.0, 1, label="open left"), *steps(0.0, 0.0, 1, label="left swings shut"),
    *[(f"right gate drifting, step {k}", {**gates(0.0, D(20 + 0.2 * k)), **START_TAG}, 0.0, False) for k in range(30)],
    ("tag on right pad after a drifting test", {**gates(), **ON_RIGHT}, 0.0, False),
]


def _run() -> int:
    """Score every case with task.py's own success() then score(), as the simulator calls them."""
    import numpy as np

    import task
    from yam_arena.task_file import ObjectState

    def state(pos, size, vel=None, quat=None):
        pos = np.asarray(pos, float)
        q = np.array([0.0, 0.0, 0.0, 1.0]) if quat is None else np.asarray(quat, float) / np.linalg.norm(quat)
        x, y, z, w = q
        r = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])
        half = np.abs(r) @ (np.asarray(size, float) / 2)
        v = np.zeros(3) if vel is None else np.asarray(vel, float)
        return ObjectState(pos, q, v, np.zeros(3), pos - half, pos + half)

    start = {o.name: (o.state() if hasattr(o, "state") else state(o.pos, o.size)) for o in task.OBJECTS}
    sizes = {o.name: o.size for o in task.OBJECTS}
    task.score(dict(start))
    bad = 0
    for label, changes, want, want_success in CASES:
        st = dict(start)
        for name, spec in changes.items():
            if isinstance(spec, ObjectState):
                st[name] = spec
                continue
            pos, size, vel, *quat = spec
            st[name] = state(pos, sizes[name] if size is None else size, vel, quat[0] if quat else None)
        ok = bool(task.success(st))
        raw = float(task.score(st))
        got = 1.0 if ok else raw
        good = (got, ok) == (want, want_success) and (not ok or raw == 1.0)
        bad += not good
        if not good or "step" not in label:
            print(f"{'ok  ' if good else 'FAIL'} {got:4.2f}  {label}")
    return bad


if __name__ == "__main__":
    raise SystemExit(_run())
