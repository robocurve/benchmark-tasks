"""The scorer's lines on hand-built states, with no simulator. Run it with the harness's Python, from this folder: python check_scorer.py"""

from __future__ import annotations

import task as t

DT = 0.05
E = 12.1


def titrate(v_target: float, swirl_while_dosing: bool, swirl_after_s: float = 0.0):
    """Dose at full flow to `v_target`, then wait or swirl: the chemistry alone. Returns the delivered volume, whether the bulk reached E, and the fraction delivered while swirling."""
    speed = 0.06 if swirl_while_dosing else 0.0
    v = unmixed = mixed = swirled = 0.0
    while v < v_target:
        dv = min(t.flow_from(1.0) * DT, v_target - v)
        v += dv
        unmixed += dv
        swirled += dv if speed >= t.SWIRL_SPEED else 0.0
        unmixed, mixed = t.mix_step(unmixed, mixed, speed, DT)
    for _ in range(int(2.0 / DT)):
        unmixed, mixed = t.mix_step(unmixed, mixed, 0.0, DT)
    for _ in range(int(swirl_after_s / DT)):
        unmixed, mixed = t.mix_step(unmixed, mixed, 0.06, DT)
    return v, mixed >= E, swirled / v if v else 0.0


def score(v, mixed, swirled, spilled=0.0, knocked=False, put_back=True, e=E):
    return t.task_score(mixed, v, e, swirled, spilled, knocked, put_back)


CASES = [
    ("swirled throughout, 0.4 mL past E, put back", lambda: score(*titrate(E + 0.4, True)), 1.0),
    ("the right final reading, no swirl at all: the bulk never turns", lambda: score(*titrate(E + 0.4, False)), 0.0),
    ("the right final reading, dosed still, swirled only afterwards (also what one arm alternating gets)", lambda: score(*titrate(E + 0.4, False, 5.0)), 0.5),
    ("no swirl, poured on until the burette is nearly empty", lambda: score(*titrate(24.0, False)), 0.0),
    ("stopped short: E - 0.1 mL", lambda: score(*titrate(E - 0.1, True)), 0.0),
    ("stopped at the first local colour: 0.5 mL", lambda: score(*titrate(0.5, True)), 0.0),
    ("V = E + 1.0 mL, the last full-score reading", lambda: score(E + 1.0, True, 1.0), 1.0),
    ("V = E + 1.01 mL", lambda: score(E + 1.01, True, 1.0), 0.5),
    ("V = E + 3.0 mL", lambda: score(E + 3.0, True, 1.0), 0.5),
    ("V = E + 3.01 mL", lambda: score(E + 3.01, True, 1.0), 0.0),
    ("80% swirled", lambda: score(E + 0.4, True, 0.80), 1.0),
    ("79% swirled", lambda: score(E + 0.4, True, 0.79), 0.5),
    ("0.2 mL outside the flask", lambda: score(E + 0.4, True, 1.0, spilled=0.2), 1.0),
    ("0.21 mL outside the flask", lambda: score(E + 0.4, True, 1.0, spilled=0.21), 0.0),
    ("flask knocked over at any moment", lambda: score(E + 0.4, True, 1.0, knocked=True), 0.0),
    ("a good titration, not stood back or let go", lambda: score(E + 0.4, True, 1.0, put_back=False), 0.5),
    ("over by 2 mL, not stood back", lambda: score(E + 2.0, True, 1.0, put_back=False), 0.25),
    ("under 80% swirled, not stood back", lambda: score(E + 0.4, True, 0.5, put_back=False), 0.25),
    ("end state: all good", lambda: t.end_state_ok(0.01, 0.0, 1.0, 0.09, 0.09, True, 0.0), True),
    ("end state: flask 3.1 cm off the tile's centre", lambda: t.end_state_ok(0.031, 0.0, 1.0, 0.09, 0.09, True, 0.0), False),
    ("end state: flask still in the air", lambda: t.end_state_ok(0.01, 0.008, 1.0, 0.09, 0.09, True, 0.0), False),
    ("end state: 11 degrees off upright", lambda: t.end_state_ok(0.01, 0.0, 11.0, 0.09, 0.09, True, 0.0), False),
    ("end state: fingers 2.9 cm from the flask", lambda: t.end_state_ok(0.01, 0.0, 1.0, 0.029, 0.09, True, 0.0), False),
    ("end state: fingers left round the clip", lambda: t.end_state_ok(0.01, 0.0, 1.0, 0.09, 0.0, True, 0.0), False),
    ("end state: flask still moving", lambda: t.end_state_ok(0.01, 0.0, 1.0, 0.09, 0.09, False, 0.0), False),
    ("end state: clip still open", lambda: t.end_state_ok(0.01, 0.0, 1.0, 0.09, 0.09, True, 0.4), False),
    ("a clip let go passes nothing", lambda: t.flow_from(0.0), 0.0),
    ("a clip at full squeeze passes 0.8 mL/s", lambda: t.flow_from(1.0), 0.8),
]


def main() -> None:
    bad = 0
    for label, fn, want in CASES:
        got = fn()
        ok = got == want
        bad += not ok
        print(f"{'ok ' if ok else 'BAD'} {got!s:6} want {want!s:6} {label}")
    ends = {s: t.endpoint(s) for s in range(20)}
    print("endpoints by seed:", ends)
    hits = sum(score(12.0, True, 1.0, e=e) == 1.0 and e <= 12.0 for e in ends.values())
    print(f"a burette stopped at a fixed 12.0 mL mark, swirled: full score on {hits} of 20 seeds")
    print("FAILED" if bad else "all lines agree")
    raise SystemExit(1 if bad else 0)


if __name__ == "__main__":
    main()
