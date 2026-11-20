"""The scorer's table on hand-built trial records, with no simulator: python check_score.py (needs yam_arena importable)."""

import task as T


class Latch:
    def __init__(self, on):
        self.on = on

    def __bool__(self):
        return self.on


base = {"n": 500, "touch": False, "fault": False, "n_ok": 4, "hold": T.HOLD_STEPS, "pin": T.PIN_STEPS}
cases = [
    (base, 1.0),
    ({**base, "hold": T.HOLD_STEPS - 1}, 0.5),  # all four on the plate, the skewer not yet put aside and still 2 s
    ({**base, "hold": 0, "pin": 0}, 0.5),  # all four on the plate, the skewer let go on the plate
    ({**base, "n_ok": 3, "hold": 0}, 0.25),  # three on the plate, the skewer still held
    ({**base, "n_ok": 3, "hold": 0, "pin": 0}, 0.0),  # three on the plate, the skewer let go
    ({**base, "n_ok": 0, "hold": 0}, 0.25),  # held 1 s, nothing off
    ({**base, "n_ok": 0, "hold": 0, "pin": T.PIN_STEPS - 1}, 0.0),  # held under 1 s
    ({**base, "n_ok": 0, "hold": 0, "pin": 0}, 0.0),  # nothing
    ({**base, "touch": True}, 0.0),  # a gripper on a vegetable: void
    ({**base, "fault": True}, 0.0),  # a piece off without the fork and the hold, or on the bench: void
    ({**base, "n": 0}, 0.0),
]
bad = 0
for rec, want in cases:
    t = T.TRIAL
    t.n, t.touch, t.fault, t.n_ok, t.hold, t.pin = rec["n"], Latch(rec["touch"]), Latch(rec["fault"]), rec["n_ok"], rec["hold"], rec["pin"]
    got = T.score(None)
    ok = abs(got - want) < 1e-9 and T.success(None) == (got == 1.0)
    bad += not ok
    print(("ok  " if ok else "FAIL"), {k: rec[k] for k in ("n_ok", "hold", "pin", "touch", "fault")}, got, "want", want)
print("all pass" if not bad else f"{bad} failed")
