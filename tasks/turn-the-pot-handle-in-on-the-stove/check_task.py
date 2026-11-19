"""Offline checks for this task: no simulator, NumPy and yam_arena only (python check_task.py, a few seconds).

1. The task file loads as the embodiment loads it (labkit parts included).
2. The scripted demonstration: every arm's plan solves with the URDF inverse kinematics at 20 Hz, and the two tool
   points never come closer than 12 cm.
3. Scoring: the task's own offline cases (``CASES`` in task.py) score what the rules say, including a finished state
   reached less than 1 s before the run limit (not 1.0) and each defect (0).

Joint physics (pushing a lid, a latch catching) can only be tested in the simulator: run the scripted demo there.
"""

from __future__ import annotations

import importlib.util
import math
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from yam_arena import ik, station, task_file  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402

FAILS: list[str] = []


def check(ok, what):
    print(("  ok    " if ok else "  FAIL  ") + what)
    if not ok:
        FAILS.append(what)


def main() -> int:
    print("1. load")
    tf = task_file.load(HERE / "task.py")
    check(tf.scene == "station", f"loads: {len(tf.objects)} objects, free: {list(tf.free_objects)}")
    spec = importlib.util.spec_from_file_location("task_under_check", HERE / "task.py")
    T = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(T)

    print("2. demo inverse kinematics")
    plan = T.demo()
    arms = {s: ArmPlan(plan[s], station.BASES[s]) for s in plan}
    dur = max(a.duration for a in arms.values())
    min_sep = 9.0
    for k in range(int(math.ceil(dur * 20)) + 1):
        t = k / 20
        tc = {}
        for s, a in arms.items():
            ta = min(t, a.duration)
            a.action7(ta)
            tc[s] = np.asarray(ik.tcp_pose(a.q, np.array(station.BASES[s]))[0])
        if len(tc) == 2:
            min_sep = min(min_sep, float(np.linalg.norm(tc["left"] - tc["right"])))
    for s, a in arms.items():
        check(a.worst_ik_mm < 3.0, f"{s} arm: worst IK position error {a.worst_ik_mm:.2f} mm over {a.duration:.1f} s")
    if len(arms) == 2:
        check(min_sep > 0.12, f"tool points at least {min_sep * 100:.0f} cm apart")
    check(dur * 20 <= 2400, f"demo {dur:.1f} s = {int(dur * 20)} steps (run limit 2,400)")

    print("3. scoring cases")
    for label, frames, tcps, want_score, want_success in T.CASES():
        T.TRIAL.new(offline_tcps=tcps or (lambda: []))
        s = ok = None
        for objs in frames:
            T.TRIAL.tick()
            ok = T.success(objs)
            s = T.score(objs)
        good = (want_score is None or abs(s - want_score) < 1e-6) and (want_success is None or ok == want_success)
        check(good, f"{label}: score {s}, success {ok} (want {want_score}{'' if want_success is None else ', ' + str(want_success)})")

    print("\n" + ("ALL CHECKS PASSED" if not FAILS else f"{len(FAILS)} CHECK(S) FAILED: " + "; ".join(FAILS)))
    return 1 if FAILS else 0


if __name__ == "__main__":
    sys.exit(main())
