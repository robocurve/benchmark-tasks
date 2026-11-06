"""Near-successes and unintended strategies, scored offline by the factory checker.

Each case: (label, {object: ready state}, expected score, expected success). The mortar's set depth and the
"ever sunk" record are kept across calls, so the single-state CASES only cover the start; presses are simulated
step by step in this file's own _run(), each after an episode reset.
"""

from task import NEW, NEW_TILES, SAG, YIELD

T = {p.name: p for p in NEW_TILES}


def at(q_left, q_right):
    return {"new_tile_left": T["new_tile_left"].state(joints={"bed": q_left}),
            "new_tile_right": T["new_tile_right"].state(joints={"bed": q_right})}


CASES = [("untouched", at(SAG, SAG), 0.0, False)]


def _run() -> int:
    import numpy as np

    import task

    fixed = {}
    from yam_arena.task_file import ObjectState
    for o in task.OBJECTS:
        if o.name not in NEW:
            p, h = np.asarray(o.pos, float), np.asarray(o.size, float) / 2
            fixed[o.name] = ObjectState(p, np.array([0, 0, 0, 1.0]), np.zeros(3), np.zeros(3), p - h, p + h)
    bad = 0

    def report(got, want, label):
        nonlocal bad
        flag = "ok  " if abs(got - want) < 1e-9 else "FAIL"
        bad += flag == "FAIL"
        print(f"{flag} {got:4.2f}  {label}")

    q = {"new_tile_left": SAG, "new_tile_right": SAG}

    def call():
        s = dict(fixed)
        s.update(at(q["new_tile_left"], q["new_tile_right"]))
        out = task.score(s)
        assert task.success(s) == (out >= 1.0)
        return out

    def reset():
        q.update(new_tile_left=0.010, new_tile_right=0.010)
        call()
        q.update(new_tile_left=SAG, new_tile_right=SAG)
        call()

    def press(name, lip):
        """Pressed down slowly until it rests `lip` above flush once let go, then let go (springs back YIELD)."""
        target = NEW[name][1] - lip  # set depth + sag
        while q[name] < target - SAG + YIELD - 1e-9:
            q[name] = min(q[name] + 0.0005, target - SAG + YIELD)
            call()
        q[name] = target  # let go: back up by the yield, resting on its sag
        out = 0.0
        for _ in range(10):
            out = call()
        return out

    for label, changes, want, _ in CASES:
        report(call(), want, label)

    def both(l1, l2, label, want):
        reset()
        press("new_tile_left", l1)
        report(press("new_tile_right", l2), want, label)

    both(0.0, 0.0, "both pressed flush", 1.0)
    both(0.0010, -0.0010, "one 1.0 mm proud, the other 1.0 mm low (limits)", 1.0)
    both(0.0011, 0.0, "left 1.1 mm proud, right flush", 0.5)
    both(0.0, 0.0050, "left flush, right still 5 mm proud", 0.5)
    both(0.0, -0.0011, "left flush, right 1.1 mm below flush", 0.0)
    both(-0.0020, 0.0, "left sunk 2 mm, right flush", 0.0)
    both(0.0030, 0.0030, "both pressed but 3 mm proud", 0.0)
    both(0.0, 0.0090, "right untouched, left flush", 0.5)

    reset()
    report(press("new_tile_left", 0.0), 0.5, "left flush only (right untouched)")
    report(press("new_tile_right", 0.0), 1.0, "then right flush too")
    reset()
    report(call(), 0.0, "after a scene reset, back to the start")

    def held_flush():
        """Both pressed to flush and still pressed (1.0 mm past the set depth, the yield force on them)."""
        reset()
        press("new_tile_left", 0.0)
        press("new_tile_right", 0.0)
        for n in NEW:
            q[n] = task._SET[n] + YIELD
        return call()

    report(held_flush(), 0.0, "both flush but the arms still pressing on them")

    def left_held():
        reset()
        press("new_tile_left", 0.0)
        press("new_tile_right", 0.0)
        q["new_tile_left"] = task._SET["new_tile_left"] + task.RELEASED + 0.0001
        return call()

    report(left_held(), 0.5, "both flush, left still pressed lightly (0.29 N)")

    def held_deep():
        """Held 1.2 mm below flush by the arm before easing off: the mortar has set 1.2 mm low."""
        reset()
        press("new_tile_left", 0.0)
        return press("new_tile_right", -0.0012)

    report(held_deep(), 0.0, "right pushed to 1.2 mm low (sunk; cannot be pulled back up)")
    return bad


if __name__ == "__main__":
    raise SystemExit(_run())
