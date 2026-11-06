"""Near-successes and unintended strategies, scored offline.

Each case: (label, {object: state}, expected score, expected success). A state is the basket's Part.state(). Cases
run in order on one copy of the task module, so the whole-trial record (basket tipped past 45 deg) carries over; the
cases that trip it come last.
"""

import math

from task import BASKET, CENTRE_TOL, DECK_H, TOP, TRAY, TRAY_FLOOR, WELL


def basket(x, y, z, tilt_deg=0.0, axis="x", vel=None, handle=0.0):
    a = math.radians(tilt_deg) / 2
    q = {"x": (math.sin(a), 0.0, 0.0, math.cos(a)), "y": (0.0, math.sin(a), 0.0, math.cos(a))}[axis]
    return BASKET.state(pos=(x, y, z), quat_xyzw=q, lin_vel=vel, joints={"handle": handle})


TX, TY = TRAY
WX, WY = WELL
F = TRAY_FLOOR
T = CENTRE_TOL
CASES = [
    ("on the tray, handle upright", {"skimmer_basket": basket(TX, TY, F, handle=math.pi / 2)}, 1.0, True),
    ("on the tray, handle folded back down", {"skimmer_basket": basket(TX, TY, F)}, 1.0, True),
    ("on the tray, 15 mm off centre in x (limit)", {"skimmer_basket": basket(TX + T, TY, F)}, 1.0, True),
    ("on the tray, 15 mm off centre in y (limit)", {"skimmer_basket": basket(TX, TY - T, F)}, 1.0, True),
    ("16 mm off centre: over the tray wall", {"skimmer_basket": basket(TX, TY + T + 0.001, F)}, 0.0, False),
    ("on the tray, tilted 5 deg (limit)", {"skimmer_basket": basket(TX, TY, F, 5.0)}, 1.0, True),
    ("on the tray, tilted 6 deg (propped on the lip)", {"skimmer_basket": basket(TX, TY, F, 6.0, "y")}, 0.0, False),
    ("over the tray, base 1 mm up (limit)", {"skimmer_basket": basket(TX, TY, F + 0.001)}, 1.0, True),
    ("over the tray, held 2 mm up", {"skimmer_basket": basket(TX, TY, F + 0.002)}, 0.0, False),
    ("over the tray, held 3 mm up", {"skimmer_basket": basket(TX, TY, F + 0.003)}, 0.0, False),
    ("over the tray, held 7 mm up (released height)", {"skimmer_basket": basket(TX, TY, F + 0.007)}, 0.0, False),
    ("on the tray, still swinging", {"skimmer_basket": basket(TX, TY, F, vel=(0.0, 0.05, 0.0))}, 0.0, False),
    ("still in the well", {"skimmer_basket": basket(WX, WY, TOP)}, 0.0, False),
    ("in the well, handle raised", {"skimmer_basket": basket(WX, WY, TOP, handle=math.pi / 2)}, 0.0, False),
    ("lifted half out of the well and held", {"skimmer_basket": basket(WX, WY, TOP + 0.04, handle=1.5)}, 0.0, False),
    ("on the deck beside the tray", {"skimmer_basket": basket(TX, TY + 0.11, TOP + DECK_H)}, 0.0, False),
    ("on the deck between well and tray", {"skimmer_basket": basket(WX, 0.16, TOP + DECK_H)}, 0.0, False),
    ("on the table off the deck", {"skimmer_basket": basket(0.12, 0.10, TOP)}, 0.0, False),
    ("on the tray, tilted 40 deg on the way (below spill) then settled",
     {"skimmer_basket": basket(TX, TY, F + 0.02, 40.0)}, 0.0, False),
    ("... then on the tray: never past 45 deg, passes", {"skimmer_basket": basket(TX, TY, F)}, 1.0, True),
    # whole-trial record from here on
    ("tipped 50 deg on the way (leaves spill)", {"skimmer_basket": basket(TX, TY, F + 0.03, 50.0, "y")}, 0.0, False),
    ("... then set upright on the tray: still 0", {"skimmer_basket": basket(TX, TY, F)}, 0.0, False),
    ("... and on the tray with handle upright: still 0",
     {"skimmer_basket": basket(TX, TY, F, handle=math.pi / 2)}, 0.0, False),
]


def _run() -> int:
    import task

    start = {"skimmer_basket": BASKET.state()}
    bad = 0
    for label, changes, want, want_ok in CASES:
        st = dict(start)
        st.update(changes)
        ok = bool(task.success(st))
        got = 1.0 if ok else float(task.score(st))
        flag = "ok  " if (got, ok) == (want, want_ok) else "FAIL"
        bad += flag == "FAIL"
        print(f"{flag} {got:4.2f}  {label}")
    return bad


if __name__ == "__main__":
    raise SystemExit(_run())
