"""The scorer's table on hand-built records, with no simulator: python check_score.py (needs yam_arena importable, or stubs)."""

import task as T

base = {"gaps_mm": [0.0, -0.2, 0.0, -0.2], "roll": 0.03, "pitch": 0.0, "yaw": 0.0, "back_mm": 7.0, "inside": True, "z_mm": 210.0, "grippers_clear": True}
cases = [
    ({}, 1.0),
    ({"yaw": 1.5}, 0.5),
    ({"roll": 2.0}, 0.5),
    ({"back_mm": 25.0}, 0.5),
    ({"gaps_mm": [0.0, 0.0, 0.0, 4.0]}, 0.25),
    ({"gaps_mm": [None, None, 30.0, 30.0], "z_mm": 240.0}, 0.25),
    ({"gaps_mm": [None, None, None, None], "z_mm": 27.0}, 0.0),
    ({"inside": False}, 0.0),
    ({"grippers_clear": False}, 0.0),
]
bad = 0
for change, want in cases:
    T.TRACK.last = {**base, **change}
    T.TRACK.hold = T.HOLD_STEPS if T._tier(T.TRACK.last) == 1.0 else 0  # held 1 s wherever every 1.0 condition is met
    got = T.score(None)
    bad += T.success(None) != (got == 1.0)  # success() and score() agree
    ok = abs(got - want) < 1e-9
    bad += not ok
    print(("ok  " if ok else "FAIL"), change, got, "want", want)
T.TRACK.last, T.TRACK.hold = dict(base), T.HOLD_STEPS - 1
unheld = T.score(None)  # a run ending inside the 1 s hold
bad += T.success(None)
print(("ok  " if unheld == 0.5 else "FAIL"), "full but not yet held 1 s at the end", unheld, "want", 0.5)
bad += unheld != 0.5
print("all pass" if not bad else f"{bad} failed")
