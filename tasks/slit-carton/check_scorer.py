"""The scoring table, the hold, the blade test, the board test and the tape rule, on hand-built values, with no simulator and no GPU.
Run with the harness's Python, from this folder: python check_scorer.py"""

from __future__ import annotations

import sys

import numpy as np

import task as t


def main() -> int:
    bad = 0

    def check(name, ok, got=""):
        nonlocal bad
        bad += not ok
        print(f"{'ok ' if ok else 'BAD'} {name}{': ' + str(got) if got != '' else ''}")

    N = t.N_SEG
    for name, args, want in (  # (opened, clean, blade_in, failed)
        ("opened, nothing else cut, blade back in", (True, True, True, False), 1.0),
        ("opened, but the product or the carton's board cut", (True, False, True, False), 0.5),
        ("opened, but the blade left out", (True, True, False, False), 0.5),
        ("not opened, everything else clean", (False, True, True, False), 0.0),
        ("carton tipped", (True, True, True, True), 0.0),
    ):
        got = t.table_score(*args)
        check(name, got == want, got)

    trial = t.Trial()
    trial.reset()
    for _ in range(t.HOLD_STEPS - 1):
        trial.hold_step(1.0)
    check(f"the full result held {t.HOLD_STEPS - 1} steps: not yet 1.0", trial.score == 0.5 and not trial.full, trial.score)
    trial.hold_step(1.0)
    check(f"held {t.HOLD_STEPS} steps (3 s): 1.0, and success", trial.score == 1.0 and trial.full, trial.score)
    trial.hold_step(0.5)
    trial.hold_step(1.0)
    check("broken and begun again: the hold starts over", trial.score == 0.5 and trial.hold == 1, trial.score)

    class Carton:  # the identity pose: the carton's frame is the world's
        def to_local(self, p):
            return np.atleast_2d(p)

    flip = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]])  # the knife pointing along x, rolled edge up: its spine side (+y) points down
    pos = np.array([0.0, 0.0, t.PRODUCT_TOP + 0.002])
    edge = t.blade_edge_local(pos, flip, Carton())
    blade = t.blade_local(pos, flip, Carton())
    check("edge up: the edge is over the product's top, the blade's back is under it", edge[:, 2].min() > t.PRODUCT_TOP and blade[:, 2].min() < t.PRODUCT_TOP,
          f"edge {1000 * (edge[:, 2].min() - t.PRODUCT_TOP):.1f} mm, back {1000 * (blade[:, 2].min() - t.PRODUCT_TOP):.1f} mm")

    lz, th10 = t.LZ, np.radians(10.0)
    for name, pts, angles, want in (
        ("the blade in the seam's gap, under the tape: not the board", [[0.0, 0.001, lz - 0.0015]], (0.0, 0.0), False),
        ("under the tape strip, 2 cm from the seam: part of the seam", [[0.0, 0.02, lz - 0.002]], (0.0, 0.0), False),
        ("in a flap's face, 4 cm from the seam: the board cut", [[0.0, 0.04, lz - 0.002]], (0.0, 0.0), True),
        ("the same place after that flap rose 10 degrees: under it, not in it", [[0.0, 0.04, lz - 0.002]], (0.0, -th10), False),
        ("in the risen flap's face, 4 cm from the seam: the board cut", [[0.0, 0.04, lz - 0.002 + (t.LY / 2 - 0.04) * np.sin(th10)]], (0.0, -th10), True),
        ("in an end wall, 1 cm under the flaps", [[-t.LX / 2 + 0.002, 0.0, lz - t.BOARD - 0.01]], (0.0, 0.0), True),
        ("past the carton's end, level with the tape: nothing cut", [[-t.LX / 2 - 0.01, 0.0, lz - 0.0015]], (0.0, 0.0), False),
    ):
        got = t.board_cut(np.array(pts), *angles)
        check(name, got == want, got)

    trial = t.Trial()
    trial.reset()
    z = t.LZ - t.CUT_BELOW - 0.001
    tip_at = lambda x: np.array([[x - 0.019, 0.0, z + 0.004], [x, 0.0, z]])
    xs = np.linspace(-t.LX / 2 - 0.01, t.LX / 2 + 0.005, 90)
    for a, b in zip(xs[:-1], xs[1:]):
        trial.cut(tip_at(a), tip_at(b))
    check(f"a stroke down the seam's gap, under the surface, edge to edge, cuts all {N} pieces", bool(trial.seg_cut.all()), int(trial.seg_cut.sum()))
    for name, ext, want in (("the blade never pushed out: nothing cut", 0.0, 0), ("the blade half out: it cuts all the same", 0.010, N), ("the blade fully out: all cut", t.BLADE_OUT, N)):
        trial.reset()
        for a, b in zip(xs[:-1], xs[1:]):
            trial.blade_step(tip_at(b), ext)
        check(name, int(trial.seg_cut.sum()) == want, int(trial.seg_cut.sum()))
    trial.reset()
    for a, b in zip(xs[:-1], xs[1:]):
        trial.cut(tip_at(a) + [0, 0, 0.003], tip_at(b) + [0, 0, 0.003])
    check("the same stroke 2 mm over the surface cuts nothing", not trial.seg_cut.any(), int(trial.seg_cut.sum()))
    trial.reset()
    for a, b in zip(xs[:-1], xs[1:]):
        trial.cut(tip_at(a) + [0, 0.01, 0], tip_at(b) + [0, 0.01, 0])
    check("a stroke 1 cm beside the seam cuts nothing", not trial.seg_cut.any(), int(trial.seg_cut.sum()))
    check("over the product inside the walls, not outside them", t.over_product(np.array([0.0, 0.0, 0.0])) and not t.over_product(np.array([t.LX / 2 + 0.005, 0.0, 0.0])))
    print("All agree" if not bad else f"{bad} DISAGREE")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
