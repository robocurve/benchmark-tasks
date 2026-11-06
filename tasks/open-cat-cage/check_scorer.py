"""Check the scorer's pure rules without the simulator: the key's resting place and the score table. Run with the harness's Python: python check_scorer.py"""

from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).parent))

import cagekit  # noqa: E402
import scene_parts as sp  # noqa: E402

TOP = cagekit.TOP
LOCK = (sp.CAGE_XY[0] + sp.LOCK_XY[0], sp.CAGE_XY[1] + sp.LOCK_XY[1])
KEY_CASES = [
    ("lying on the table, where the demo leaves it", (0.2506, 0.2458, TOP + 0.004), True),
    ("in the lock", (*LOCK, cagekit.KEY_SEATED_Z), False),
    ("lifted over the lock", (*LOCK, cagekit.KEY_SEATED_Z + 0.05), False),
    ("on top of the glass cage", (0.24, 0.05, TOP + sp.TRAY_H + sp.WALL_H + 0.004), False),
    ("inside the glass cage, on its floor", (0.24, 0.05, TOP + sp.TRAY_H + 0.006), False),
    ("on top of the lock column, beside the lock", (LOCK[0] - 0.02, LOCK[1] + 0.025, TOP + sp.LOCK_TOP + 0.004), False),
    ("on top of the cat's cage", (0.65, 0.30, TOP + sp.CRATE_H + 0.004), False),
    ("off the table's far edge, at table height", (0.95, 0.0, TOP + 0.004), False),
    ("on the floor", (0.25, 0.25, TOP - 0.70), False),
]
SCORE_CASES = [  # unsafe, touched, shut, locked, slid, full
    ("idle", (False, False, False, False, 0.0, False), 0.0),
    ("door shut only", (False, False, True, False, 0.0, False), 0.1),
    ("locked only", (False, False, True, True, 0.0, False), 0.25),
    ("locked, cat's door cracked", (False, False, True, True, 0.05, False), 0.5),
    ("locked, cat's door open, key left in or no hold", (False, False, True, True, 0.25, False), 0.75),
    ("full", (False, False, True, True, 0.25, True), 1.0),
    ("latched unsafe, then everything right", (True, False, True, True, 0.25, True), 0.0),
    ("touched a toy, then everything right", (False, True, True, True, 0.25, True), 0.0),
    ("door not shut, bolt out", (False, False, False, False, 0.0, False), 0.0),
]


def main() -> None:
    bad = 0
    for name, p, want in KEY_CASES:
        got = cagekit.key_down(p, LOCK)
        bad += got != want
        print(f"{'ok ' if got == want else 'BAD'} key {name}: {got}")
    for name, args, want in SCORE_CASES:
        got = cagekit.task_score(*args)
        bad += got != want
        print(f"{'ok ' if got == want else 'BAD'} score {name}: {got}")
    print("all ok" if not bad else f"{bad} wrong")
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
