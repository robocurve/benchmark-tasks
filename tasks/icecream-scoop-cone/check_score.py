"""The scorer's tier table on hand-built moments, with no simulator: python check_score.py (needs yam_arena importable)."""

import task as T

full = {"cracked": False, "served": True, "on_cone": True, "held": True, "upright": True, "ball_in_bowl": False, "clear": True}
cases = [
    (full, 1.0),  # the ball on the held, upright cone (1.0 once this lasts 3 s)
    ({**full, "upright": False}, 0.7),  # on the held cone, the cone tipped over 20 degrees
    ({**full, "held": False}, 0.0),  # on a cone left in its stand or let go
    ({**full, "held": False, "upright": False}, 0.0),
    ({**full, "on_cone": False, "ball_in_bowl": True}, 0.3),  # a ball in the bowl, clear of the tub
    ({**full, "on_cone": False, "ball_in_bowl": True, "clear": False}, 0.0),  # still in the tub
    ({**full, "on_cone": False}, 0.0),  # the ball dropped elsewhere
    ({**full, "cracked": True}, 0.0),  # the cone cracked: void
    ({**full, "on_cone": False, "ball_in_bowl": True, "cracked": True}, 0.0),
    ({**full, "served": False}, 0.0),  # the ball reached a cone the left hand did not hold out of its stand (set in the stand, lifted after): void
    ({**full, "served": False, "on_cone": False, "ball_in_bowl": True}, 0.0),
    ({**full, "served": None, "on_cone": False, "ball_in_bowl": True}, 0.3),  # the ball has not reached the mouth yet
]
bad = 0
for c, want in cases:
    got = T.moment(c)
    ok = abs(got - want) < 1e-9
    bad += not ok
    print(("ok  " if ok else "FAIL"), c, got, "want", want)
# the 1.0 tier also needs this moment for HOLD_STEPS steps in a row (task.py, Trial.step); until then the score is capped at 0.7
print("all pass" if not bad else f"{bad} failed")
