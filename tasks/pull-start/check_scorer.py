"""The scoring table on hand-built states, with no simulator and no GPU. Run it with the harness's Python, from this folder: python check_scorer.py"""

from __future__ import annotations

import sys

import task as t


def main() -> int:
    bad = 0
    for name, args, want in (
        ("a full pull in 0.3 s, the engine where it was", (False, 0.3), 1.0),
        ("the limit itself, 0.5 s", (False, 0.5), 1.0),
        ("a full pull too slow, 1 s", (False, 1.0), 0.0),
        ("never a full pull", (False, None), 0.0),
        ("the engine moved past 10 cm", (True, 0.3), 0.0),
    ):
        got = t.task_score(*args)[0]
        ok = got == want
        bad += not ok
        print(f"{'ok ' if ok else 'BAD'} {name}: {got}")
    print("All agree" if not bad else f"{bad} DISAGREE")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
