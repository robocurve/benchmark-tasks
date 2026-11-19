"""Offline checks for this task: no simulator, NumPy and yam_arena only (python check_task.py, a few seconds).

1. The task file loads as the embodiment loads it.
2. Gripper clearance: the gripper's housing (5.1 cm either side of the tool point along the closing axis, 3.4 cm
   across, from the robot's collision meshes) clears neighbouring apples and the crate at every grip.
3. The scripted demo: the left arm's plan solves with the URDF IK at 20 Hz.
4. Scoring: a kinematic replay of the demo (an apple rides with the closed gripper at it and drops straight down
   when let go) rises 0.2, 0.4, 0.6, 0.8 and then 1.0 only after the 1 s hold; negative controls score what the
   rules say, including the bruised apple packed (0) and a pack completed at the run limit (0.8, not 1.0).
"""

from __future__ import annotations

import importlib.util
import math
import sys
from pathlib import Path

import numpy as np

from yam_arena import ik, station, task_file
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState

HERE = Path(__file__).resolve().parent
TASK_PATH = HERE / "task.py"
HZ = 20.0
FAILS: list[str] = []
HOUSING_ALONG, HOUSING_ACROSS = 0.051, 0.034


def check(ok: bool, what: str) -> None:
    print(("  ok    " if ok else "  FAIL  ") + what)
    if not ok:
        FAILS.append(what)


def load_task():
    spec = importlib.util.spec_from_file_location("pack_task", TASK_PATH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def sphere(pos, r, vel=(0, 0, 0)) -> ObjectState:
    pos = np.asarray(pos, float)
    return ObjectState(pos=pos, quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=np.asarray(vel, float),
                       ang_vel=np.zeros(3), low=pos - r, high=pos + r)


def boxes(T) -> dict:
    out = {}
    for o in [*T.CRATE, *T.TRAY]:
        p = np.asarray(o.pos, float)
        h = np.asarray(o.size) / 2
        out[o.name] = ObjectState(pos=p, quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                                  low=p - h, high=p + h)
    return out


def frame(T, positions: dict, vels: dict | None = None) -> dict:
    objs = boxes(T)
    for name, p in positions.items():
        objs[name] = sphere(p, T.APPLE_R, (vels or {}).get(name, (0, 0, 0)))
    return objs


def start_positions(T) -> dict:
    pos = {n: np.array((x, y, T.APPLE_Z)) for n, (x, y) in zip(T.GOOD_NAMES, T.GOOD_STARTS)}
    pos["bruised_apple"] = np.array((*T.BRUISED_START, T.APPLE_Z))
    return pos


def in_cell(T, c) -> np.ndarray:
    return np.array((c[0], c[1], T.TRAY_FLOOR_TOP + T.APPLE_R))


def run(T, seq, tcps=None):
    """seq: list of {name: position} frames, one per control step."""
    T.new_trial(offline_tcps=tcps)
    out = []
    for positions in seq:
        T.tick()
        objs = frame(T, positions)
        ok = T.success(objs)
        out.append((T.score(objs), ok))
    return out


def replay_demo(T):
    arm = ArmPlan(T.demo()["left"], station.BASES["left"])
    pos = start_positions(T)
    prev = {k: v.copy() for k, v in pos.items()}
    held, off = None, None
    tcp_now = {"l": None}
    T.new_trial(offline_tcps=lambda: [tcp_now["l"]])
    results, events = [], []
    for k in range(int(math.ceil(arm.duration * HZ)) + 1):
        t = k / HZ
        arm.action7(t)
        _p, _r, finger, _l = arm.target(t)
        tcp = np.asarray(ik.tcp_pose(arm.q, np.array(station.BASES["left"]))[0])
        tcp_now["l"] = tcp
        closed = finger > -0.003
        if closed and held is None:
            for n, p in pos.items():
                if np.linalg.norm(tcp - p) < 0.02:
                    held, off = n, p - tcp
                    events.append((round(t, 2), f"grips {n}"))
                    break
        if held is not None and not closed:
            events.append((round(t, 2), f"lets go of {held}"))
            held = None
        for n in pos:
            if n == held:
                pos[n] = tcp + off
            else:
                c = T._cell_of(sphere(pos[n], T.APPLE_R))
                floor = T.TRAY_FLOOR_TOP if c is not None else (
                    T.CRATE_FLOOR_TOP if abs(pos[n][0] - T.CRATE_XY[0]) <= T.CRATE_IN[0] / 2
                    and abs(pos[n][1] - T.CRATE_XY[1]) <= T.CRATE_IN[1] / 2 else T.TOP)
                pos[n] = np.array((pos[n][0], pos[n][1], floor + T.APPLE_R))
        vels = {n: (pos[n] - prev[n]) * HZ for n in pos}
        prev = {n: v.copy() for n, v in pos.items()}
        T.tick()
        objs = frame(T, pos, vels)
        ok = T.success(objs)
        results.append((t, T.score(objs), ok))
    return results, events, arm.worst_ik_mm


def main() -> int:
    print("1. load")
    tf = task_file.load(TASK_PATH)
    check(tf.scene == "station" and len(tf.free_objects) == 5, f"loads: {len(tf.objects)} objects, free: {tf.free_objects}")
    T = load_task()

    print("2. gripper clearance at each grip (fingers close along y)")
    starts = [*T.GOOD_STARTS, T.BRUISED_START]
    worst = 9.0
    for (ax, ay) in T.GOOD_STARTS:
        for (bx, by) in starts:
            if (bx, by) == (ax, ay):
                continue
            dx, dy = abs(bx - ax), abs(by - ay)
            gap = max(dy - HOUSING_ALONG - T.APPLE_R, dx - HOUSING_ACROSS - T.APPLE_R)
            worst = min(worst, gap)
    check(worst >= 0.005, f"housing clears every neighbouring apple by at least {worst * 100:.1f} cm")
    tips = T.GRIP_Z - T.TIP_PAST_TCP
    check(tips > T.CRATE_FLOOR_TOP + T.CRATE_LIP, f"finger tips at the grip ({(tips - T.TOP) * 100:.1f} cm) above the crate lip")
    check(T.RELEASE_Z - T.TIP_PAST_TCP > T.TRAY_RIM, "finger tips above the tray dividers at release")
    clear = (T.CARRY_Z - T.APPLE_R) - T.APPLE_TOP
    check(clear >= 0.03, f"a carried apple clears the tops of the other apples by {clear * 100:.1f} cm")

    print("3./4. demo replay")
    results, events, worst_mm = replay_demo(T)
    for ev in events:
        print(f"        t={ev[0]:6.2f} s  {ev[1]}")
    check(worst_mm < 3.0, f"worst IK position error along the plan: {worst_mm:.2f} mm")
    firsts = {}
    for t, s, _ in results:
        firsts.setdefault(s, t)
    print("        first time at each score: " + ", ".join(f"{s:.1f} at {t:.2f} s" for s, t in sorted(firsts.items())))
    check([s for s in sorted(firsts) if s > 0] == [0.2, 0.4, 0.6, 0.8, 1.0], "scores rise 0.2, 0.4, 0.6, 0.8, 1.0")
    t_ok = next((t for t, _, ok in results if ok), None)
    check(t_ok is not None and firsts.get(1.0, 0) - t_ok >= T.SUCCESS_HOLD_S - 1e-6,
          f"1.0 only after the hold (success from {t_ok} s, 1.0 from {firsts.get(1.0)} s)")
    check(len(results) <= 2400, f"demo length {len(results) / HZ:.1f} s = {len(results)} steps (run limit 2,400)")

    print("5. negative controls")
    base = start_positions(T)
    packed = dict(base)
    for n, c in zip(T.GOOD_NAMES, T.DEMO_CELLS):
        packed[n] = in_cell(T, c)
    s = run(T, [base] * 30)
    check(s[-1][0] == 0.0, "nothing packed: 0")
    s = run(T, [packed] * 10)
    check(s[-1] == (0.8, True), f"pack completed 0.5 s before the run limit: {s[-1][0]} (not 1.0)")
    s = run(T, [packed] * 25)
    check(s[-1][0] == 1.0, f"pack complete and held 1.25 s: {s[-1][0]}")
    three = dict(packed)
    three[T.GOOD_NAMES[3]] = base[T.GOOD_NAMES[3]]
    s = run(T, [three] * 25)
    check(s[-1][0] == 0.6, f"three good apples packed: {s[-1][0]}")
    cull = dict(three)
    cull["bruised_apple"] = in_cell(T, T.DEMO_CELLS[3])
    s = run(T, [cull] * 25)
    check(s[-1][0] == 0.0, "bruised apple packed in the fourth pocket: 0")
    doubled = dict(packed)
    doubled[T.GOOD_NAMES[3]] = in_cell(T, T.DEMO_CELLS[0]) + np.array((0.01, 0, 0))
    s = run(T, [doubled] * 25)
    check(s[-1][0] == 0.4 and not s[-1][1], f"two apples in one pocket count for neither: {s[-1][0]}")
    dropped = dict(packed)
    dropped[T.GOOD_NAMES[0]] = np.array((0.3, 0.6, T.TOP - 0.3))
    s = run(T, [dropped] * 25)
    check(s[-1][0] == 0.0, "an apple dropped off the table: 0")
    s = run(T, [packed] * 25, tcps=lambda: [packed[T.GOOD_NAMES[2]]])
    check(s[-1][0] == 0.8 and not s[-1][1], "all four in but one still held: 0.8, not success")

    print("\n" + ("ALL CHECKS PASSED" if not FAILS else f"{len(FAILS)} CHECK(S) FAILED: " + "; ".join(FAILS)))
    return 1 if FAILS else 0


if __name__ == "__main__":
    sys.exit(main())
