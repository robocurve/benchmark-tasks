"""Physics rehearsal for qc_met10 in MuJoCo, with the real YAM arms and finger contact (no welds).

The dogbone is a loose rigid body picked by real finger contact; each lever is a spring-return slide joint held by the gripped
handle, whose wedge lifts a spring-closed upper jaw. Rebuilds the scene (test stand with a jogged crosshead, two upper
jaw parts, the loose dogbone on its tray),
drives the scripted demo plan and the adversarial policies, and scores every run with the task's own
score(). Uses the generic MuJoCo gate's rollout() for the arm/finger/contact model.

    python rehearsal.py <path to mj_check.py>        (or set MJ_CHECK)
"""

from __future__ import annotations

import importlib
import json
import os
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
TASK = str(HERE / "qc_met10.py")
RUNS = [("demo", {}), ("demo", {"friction_scale": 0.7}), ("demo", {"friction_scale": 1.3}),
        ("demo", {"jitter_seed": 0}), ("demo", {"jitter_seed": 3}), ("nothing", {}), ("sweep", {}),
        ("liftdrop", {}), ("half", {}), ("undo", {})]


def main() -> int:
    path = sys.argv[1] if len(sys.argv) > 1 else os.environ.get("MJ_CHECK", "")
    if not path or not pathlib.Path(path).is_file():
        print("usage: python rehearsal.py <mj_check.py>  (or MJ_CHECK=<mj_check.py>)")
        return 2
    sys.path.insert(0, str(pathlib.Path(path).resolve().parent))
    mj = importlib.import_module(pathlib.Path(path).stem)
    expect = {"demo": 1.0, "nothing": 0.0, "sweep": 0.0, "liftdrop": 0.0, "half": 0.0, "undo": 0.2}
    ok = True
    for mode, kw in RUNS:
        r = mj.rollout(TASK, mode, **kw)
        good = abs(float(r["score"]) - expect[mode]) < 1e-6
        ok &= good
        print(json.dumps({"mode": mode, **kw, "score": r["score"], "success": r["success"], "steps": r["steps"],
                          "contacts": r["contacts"], "expected": expect[mode], "ok": good}))
    print("REHEARSAL", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
