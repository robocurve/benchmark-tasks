"""Scorer checks on hand-built states, no simulator: python check_score.py (harness Python, from this folder)."""

import numpy as np

import task as t

ok = True


def check(name, got, want):
    global ok
    good = got == want
    ok = ok and good
    print(f"{'ok ' if good else 'BAD'} {name}: {got} (want {want})")


# a stone is in while it is anywhere within the shoe's envelope, and on the collar's lip
check("toe box stone in", bool(t.inside_shoe([0.24, 0.0, 0.032])[0]), True)
check("heel stone in", bool(t.inside_shoe([0.05, 0.01, 0.040])[0]), True)
check("stone on the collar lip, 5 mm above the edge, in", bool(t.inside_shoe([0.06, 0.030, t.top_z(0.06) + 0.005])[0]), True)
check("stone 3 mm beside the side wall, out", bool(t.inside_shoe([0.20, t.half_w(0.20) + 0.003, 0.004])[0]), False)
check("stone 4 mm behind the heel, out", bool(t.inside_shoe([-0.004, 0.0, 0.004])[0]), False)
check("stone on the vamp's top, out", bool(t.inside_shoe([0.20, 0.0, t.top_z(0.20) + 0.004])[0]), False)
check("stone on the table under the heel's edge, out", bool(t.inside_shoe([0.007, 0.002, -0.002])[0]), False)
check("stone 4 mm past the toe, out", bool(t.inside_shoe([t.L + 0.004, 0.0, 0.004])[0]), False)
# every start position is inside
for k, (x, y) in enumerate(t.START):
    check(f"pebble {k} starts in", bool(t.inside_shoe([x, y, float(t.insole_z(x)) + 0.004])[0]), True)
# hands clear: a finger on the pull tab or in the collar is not clear
check("finger on the tab not clear", bool(t.gap_to_shoe([[t.BACK + 0.003, 0.0, 0.12]])[0] >= t.CLEAR_M), False)
check("finger in the collar not clear", bool(t.gap_to_shoe([[0.06, 0.0, 0.08]])[0] >= t.CLEAR_M), False)
check("hand 4 cm above the vamp clear", bool(t.gap_to_shoe([[0.20, 0.0, t.top_z(0.20) + 0.04]])[0] >= t.CLEAR_M), True)
# tiers: count of stones out, and all out plus set down for 1.0
check("none out", t.tier(0, True), 0.0)
check("1 out", t.tier(1, True), 0.0)
check("2 out", t.tier(2, False), 0.25)
check("5 out", t.tier(5, False), 0.5)
check("7 out, set down", t.tier(7, True), 0.75)
check("8 out, not set down", t.tier(8, False), 0.75)
check("8 out, set down", t.tier(8, True), 1.0)
# the trial's score: 1.0 only after 40 steps of the full end state, 0 after a void
tr = t.Trial()
tr.reset()
tr.n_out, tr.set_down, tr.ok_steps = 8, True, 39
check("8 out set down 39 steps", tr.score(), 0.75)
tr.ok_steps = 40
check("8 out set down 40 steps", tr.score(), 1.0)
tr.void = True
check("void", tr.score(), 0.0)
print("ALL OK" if ok else "FAILURES")
