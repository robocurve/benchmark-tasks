"""The score without the simulator, from made-up positions. It needs NumPy and yam_arena's package on the path. Run: python test_refit_score.py"""

from __future__ import annotations

import numpy as np
import refit_score as s
import refit_usd as u

UP = np.array([0.0, 0.0, 1.0])
FAR = np.array([[0.2, 0.3, 1.0], [0.2, -0.3, 1.0]])  # fingertips well away
DT = 0.05
HOLES = u.SCREW_HOLES
LONGS = [s.SCREW_NAME[h] for h in u.DEEP_HOLES]
SHORTS = [s.SCREW_NAME[h] for h in HOLES if h not in u.DEEP_HOLES]


def body(xy, z, up=UP, speed=0.0) -> s.Body:
    return s.Body(np.array([xy[0], xy[1], z]), np.array(up, float), speed)


def in_hole(hole: str, proud: float = 0.0) -> s.Body:
    return body(s.hole_world(hole), s.PULLEY_TOP + proud)


def in_slot(i: int) -> s.Body:
    return body(s.slot_world(i), s.RACK_TOP)


WORN_ON_STAND = body(s.STAND_XY, s.STAND_TOP)
NEW_ON_MAT = body(s.NEW_MAT_XY, s.MAT_TOP)
WORN_ON_MAT = body(s.WORN_MAT_XY, s.MAT_TOP)
NEW_ON_STAND = body(s.STAND_XY, s.STAND_TOP)
START = {s.SCREW_NAME[h]: in_hole(h) for h in HOLES}
IN_RACK = {s.SCREW_NAME[h]: in_slot(i) for i, h in enumerate(HOLES)}
BACK = dict(START)


def hang_all(c: s.Scorer) -> None:
    c.update(WORN_ON_STAND, NEW_ON_MAT, IN_RACK, FAR)


def expect(name: str, got: float, want: float) -> None:
    assert abs(got - want) < 1e-6, f"{name}: got {got}, want {want}"
    print(f"ok   {name}: {got}")


def run() -> None:
    c = s.Scorer(DT)
    expect("start", c.update(WORN_ON_STAND, NEW_ON_MAT, START, FAR)["score"], 0.0)

    # The full job: the screws hang in the rack, the pulleys swap, the screws go back.
    c = s.Scorer(DT)
    hang_all(c)
    r = c.update(WORN_ON_MAT, NEW_ON_STAND, BACK, FAR)
    expect("everything back, before the 1 s hold", r["score"], 0.9)
    for _ in range(19):
        r = c.update(WORN_ON_MAT, NEW_ON_STAND, BACK, FAR)
    expect("everything back, after the 1 s hold", r["score"], 1.0)

    # A screw that skips the rack earns nothing, whether it goes straight back or lies on the table first.
    c = s.Scorer(DT)
    expect("straight back, never in the rack", c.update(WORN_ON_MAT, NEW_ON_STAND, BACK, FAR)["score"], 0.2)
    c = s.Scorer(DT)
    flat = {n: body((0.15, -0.1 + 0.03 * i), s.TOP + 0.002, up=(1.0, 0.0, 0.0)) for i, n in enumerate(BACK)}
    c.update(WORN_ON_STAND, NEW_ON_MAT, flat, FAR)
    expect("laid on the table, then back", c.update(WORN_ON_MAT, NEW_ON_STAND, BACK, FAR)["score"], 0.2)
    c = s.Scorer(DT)
    one_flat = dict(IN_RACK)
    one_flat["screw_a"] = body((0.15, 0.1), s.TOP + 0.002, up=(1.0, 0.0, 0.0))
    c.update(WORN_ON_STAND, NEW_ON_MAT, one_flat, FAR)
    expect("one screw laid on the table, three in the rack", c.update(WORN_ON_MAT, NEW_ON_STAND, BACK, FAR)["score"], 0.8)
    c = s.Scorer(DT)
    held = np.array([[*s.slot_world(0), s.RACK_TOP + 0.002], *FAR])
    c.update(WORN_ON_STAND, NEW_ON_MAT, {**IN_RACK}, held)
    expect("still gripped in its slot, so that screw has not hung", c.update(WORN_ON_MAT, NEW_ON_STAND, BACK, FAR)["score"], 0.8)
    c = s.Scorer(DT)
    over = dict(IN_RACK)
    over["screw_a"] = body(s.slot_world(0), s.RACK_TOP + 0.02)
    c.update(WORN_ON_STAND, NEW_ON_MAT, over, FAR)
    expect("held 2 cm over a slot has not hung", c.update(WORN_ON_MAT, NEW_ON_STAND, BACK, FAR)["score"], 0.8)

    # A long screw bottoming out in either shallow hole loses that hole, held or not, and lifting it out does not undo it.
    for shallow in [h for h in HOLES if h not in u.DEEP_HOLES]:
        c = s.Scorer(DT)
        hang_all(c)
        long_name, short_name = LONGS[0], s.SCREW_NAME[shallow]
        probe = {**IN_RACK, long_name: body(s.hole_world(shallow), s.hole_floor(shallow) + u.LONG)}
        tips = np.array([[*s.hole_world(shallow), s.PULLEY_TOP + 0.04], *FAR])
        r = c.update(WORN_ON_STAND, NEW_ON_MAT, probe, tips)
        assert shallow in r["lost"], r["lost"]
        r = c.update(WORN_ON_MAT, NEW_ON_STAND, BACK, FAR)
        expect(f"long screw bottomed in {shallow}, then all correct", r["score"], 0.8)
        for _ in range(30):
            r = c.update(WORN_ON_MAT, NEW_ON_STAND, BACK, FAR)
        expect(f"and it never reaches 1.0 ({shallow})", r["score"], 0.8)
        assert long_name != short_name
    c = s.Scorer(DT)
    shallow = next(h for h in HOLES if h not in u.DEEP_HOLES)
    r = c.update(WORN_ON_STAND, NEW_ON_MAT, {**IN_RACK, LONGS[0]: body(s.hole_world(shallow), s.hole_floor(shallow) + u.LONG + 0.005)}, FAR)
    assert not r["lost"], r["lost"]
    print("ok   a long screw 5 mm short of a shallow floor loses nothing")

    # Wrong layouts: both swaps of different lengths score the two wrong holes as nothing; swaps of equal length are fine.
    c = s.Scorer(DT)
    hang_all(c)
    deep, other = u.DEEP_HOLES[0], next(h for h in HOLES if h not in u.DEEP_HOLES)
    wrong = dict(BACK)
    wrong[s.SCREW_NAME[other]] = in_hole(deep)  # a short screw seated in the deep hole
    wrong[s.SCREW_NAME[deep]] = in_hole(other, proud=u.LONG - u.PULLEY_THICK - u.SHALLOW)  # a long screw standing proud in the shallow one
    r = c.update(WORN_ON_MAT, NEW_ON_STAND, wrong, FAR)
    assert other in r["lost"]
    expect("a long and a short swapped", r["score"], 0.6)
    assert r["in_holes"][deep] == s.SCREW_NAME[other] and r["in_holes"][other] is None
    c = s.Scorer(DT)
    hang_all(c)
    r = c.update(WORN_ON_MAT, NEW_ON_STAND, {**BACK, LONGS[0]: BACK[LONGS[1]], LONGS[1]: BACK[LONGS[0]], SHORTS[0]: BACK[SHORTS[1]], SHORTS[1]: BACK[SHORTS[0]]}, FAR)
    expect("two of each length swapped among themselves", r["score"], 0.9)

    # The pulleys.
    c = s.Scorer(DT)
    expect("worn pulley held still 5 cm above its mat", c.update(body(s.WORN_MAT_XY, s.MAT_TOP + 0.05), NEW_ON_MAT, START, FAR)["score"], 0.0)
    expect("worn pulley on its mat", c.update(WORN_ON_MAT, NEW_ON_MAT, START, FAR)["score"], 0.1)
    expect("worn pulley moving over its mat", c.update(body(s.WORN_MAT_XY, s.MAT_TOP, speed=0.05), NEW_ON_MAT, START, FAR)["score"], 0.0)
    c = s.Scorer(DT)
    hang_all(c)
    expect("screws back in the worn pulley: no credit", c.update(WORN_ON_STAND, NEW_ON_MAT, BACK, FAR)["score"], 0.0)
    c = s.Scorer(DT)
    hang_all(c)
    expect("new pulley 6 mm off centre", c.update(WORN_ON_MAT, body((s.STAND_XY[0] + 0.006, 0.0), s.STAND_TOP), BACK, FAR)["score"], 0.1)

    # Grippers.
    c = s.Scorer(DT)
    hang_all(c)
    near = np.array([[*s.hole_world(HOLES[0]), s.PULLEY_TOP + 0.02], *FAR])
    expect("a fingertip within 3 cm of a screw", c.update(WORN_ON_MAT, NEW_ON_STAND, BACK, near)["score"], 0.8)
    near = np.array([[*s.hole_world(HOLES[0]), s.PULLEY_TOP + 0.048], *FAR])
    c.reset()
    hang_all(c)
    for _ in range(30):
        r = c.update(WORN_ON_MAT, NEW_ON_STAND, BACK, near)
    expect("a fingertip 4 cm from a screw, held 1.5 s", r["score"], 0.9)

    # A new trial clears everything kept.
    c = s.Scorer(DT)
    hang_all(c)
    c.update(WORN_ON_STAND, NEW_ON_MAT, {**IN_RACK, LONGS[0]: body(s.hole_world(shallow), s.hole_floor(shallow) + u.LONG)}, FAR)
    assert c.lost and c.hung
    c.reset()
    assert not c.lost and not c.hung and c.full_steps == 0
    print("ok   a new trial clears the kept state")


if __name__ == "__main__":
    run()
    print("all passed")
