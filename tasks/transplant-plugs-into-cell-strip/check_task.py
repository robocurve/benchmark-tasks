"""Offline checks for this task: no simulator, NumPy and yam_arena only (python check_task.py, a few seconds).

1. The task file loads as the embodiment loads it.
2. Gripper clearance: the gripper's housing (5.1 cm either side of the tool point along the closing axis, here x;
   3.4 cm across, from the robot's collision meshes) clears neighbouring plugs at every grip and every release, and
   the finger tips stay above the flat's lip and the cell walls.
3. The scripted demo: both arms' plans solve with the URDF IK at 20 Hz, and the two tool points stay apart.
4. Scoring: a kinematic replay of the demo (a plug rides with the closed gripper at it and drops straight down when
   let go) rises 0.2, 0.4, 0.6, 0.8 and then 1.0 only after the 1 s hold; negative controls score what the rules
   say, including a plug set into an occupied cell, an original plant knocked out (0) and a strip completed at the run
   limit (0.8, not 1.0).
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
    spec = importlib.util.spec_from_file_location("plug_task", TASK_PATH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def plug(T, pos, vel=(0, 0, 0)) -> ObjectState:
    pos = np.asarray(pos, float)
    h = np.array((T.PLUG_R, T.PLUG_R, T.PLUG_H / 2))
    return ObjectState(pos=pos, quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=np.asarray(vel, float),
                       ang_vel=np.zeros(3), low=pos - h, high=pos + h)


def boxes(T) -> dict:
    out = {}
    for o in [*T.STRIP, *T.FLATS]:
        p = np.asarray(o.pos, float)
        h = np.asarray(o.size) / 2
        out[o.name] = ObjectState(pos=p, quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                                  low=p - h, high=p + h)
    return out


def frame(T, positions: dict, vels: dict | None = None) -> dict:
    objs = boxes(T)
    for name, p in positions.items():
        objs[name] = plug(T, p, (vels or {}).get(name, (0, 0, 0)))
    return objs


def start_positions(T) -> dict:
    pos = {n: np.asarray(o.pos, float) for n, o in zip(T.NEW_NAMES, T.NEW)}
    pos.update({n: np.asarray(o.pos, float) for n, o in zip(T.OLD_NAMES, T.OLD)})
    return pos


def in_cell(T, c) -> np.ndarray:
    return np.array((*T.CELLS[c], T.CELL_PLUG_Z))


def run(T, seq, tcps=None):
    T.new_trial(offline_tcps=tcps)
    out = []
    for positions in seq:
        T.tick()
        objs = frame(T, positions)
        ok = T.success(objs)
        out.append((T.score(objs), ok))
    return out


def floor_under(T, p) -> float:
    c = T._cell_of(plug(T, p))
    if c is not None:
        return T.STRIP_FLOOR_TOP
    for side, fy in T.FLAT_Y.items():
        if abs(p[0] - T.FLAT_X) <= T.FLAT_IN[0] / 2 and abs(p[1] - fy) <= T.FLAT_IN[1] / 2:
            return T.FLAT_FLOOR_TOP
    return T.TOP


def replay_demo(T):
    plan = T.demo()
    arms = {s: ArmPlan(plan[s], station.BASES[s]) for s in ("left", "right")}
    dur = max(a.duration for a in arms.values())
    pos = start_positions(T)
    prev = {k: v.copy() for k, v in pos.items()}
    held = {"left": None, "right": None}
    tcp_now = {}
    T.new_trial(offline_tcps=lambda: [tcp_now["left"], tcp_now["right"]])
    results, events, min_sep = [], [], 9.0
    for k in range(int(math.ceil(dur * HZ)) + 1):
        t = k / HZ
        for s, arm in arms.items():
            ta = min(t, arm.duration)
            arm.action7(ta)
            _p, _r, finger, _l = arm.target(ta)
            tcp = np.asarray(ik.tcp_pose(arm.q, np.array(station.BASES[s]))[0])
            tcp_now[s] = tcp
            closed = finger > -0.003
            if closed and held[s] is None:
                for n, p in pos.items():
                    if np.linalg.norm(tcp - p) < 0.03 and n not in [h[0] for h in held.values() if h]:
                        held[s] = (n, p - tcp)
                        events.append((round(t, 2), f"{s} grips {n}"))
                        break
            if held[s] is not None and not closed:
                events.append((round(t, 2), f"{s} lets go of {held[s][0]}"))
                held[s] = None
        min_sep = min(min_sep, float(np.linalg.norm(tcp_now["left"] - tcp_now["right"])))
        carried = {h[0]: tcp_now[s] + h[1] for s, h in held.items() if h}
        for n in pos:
            if n in carried:
                pos[n] = carried[n]
            else:
                pos[n] = np.array((pos[n][0], pos[n][1], floor_under(T, pos[n]) + T.PLUG_H / 2))
        vels = {n: (pos[n] - prev[n]) * HZ for n in pos}
        prev = {n: v.copy() for n, v in pos.items()}
        T.tick()
        objs = frame(T, pos, vels)
        ok = T.success(objs)
        results.append((t, T.score(objs), ok))
    return results, events, max(a.worst_ik_mm for a in arms.values()), min_sep, pos


def main() -> int:
    print("1. load")
    tf = task_file.load(TASK_PATH)
    check(tf.scene == "station" and len(tf.free_objects) == 6, f"loads: {len(tf.objects)} objects, free: {tf.free_objects}")
    T = load_task()

    print("2. gripper clearance (pads close along x)")
    worst = 9.0
    for side, starts in T.PLUG_STARTS.items():
        for (ax, ay) in starts:
            for (bx, by) in starts:
                if (bx, by) != (ax, ay):
                    worst = min(worst, max(abs(bx - ax) - HOUSING_ALONG - T.PLUG_R,
                                           abs(by - ay) - HOUSING_ACROSS - T.PLUG_R))
    check(worst >= 0.005, f"at the flats, housing clears the other plug by {worst * 100:.1f} cm")
    worst = 9.0
    for c in T.EMPTY_CELLS:
        cx, cy = T.CELLS[c]
        for d in range(6):
            if d == c:
                continue
            dx, dy = T.CELLS[d]
            worst = min(worst, max(abs(dx - cx) - HOUSING_ALONG - T.PLUG_R, abs(dy - cy) - HOUSING_ACROSS - T.PLUG_R))
    check(worst >= 0.004, f"at every cell, housing clears plugs in the neighbouring cells by {worst * 100:.1f} cm")
    tips = T.GRIP_Z - T.TIP_PAST_TCP
    check(tips > T.FLAT_FLOOR_TOP + T.FLAT_LIP, f"finger tips at the grip ({(tips - T.TOP) * 100:.1f} cm) above the flat's lip")
    check(T.RELEASE_Z - T.TIP_PAST_TCP > T.STRIP_RIM, "finger tips above the cell walls at release")
    drop = (T.RELEASE_Z + T.GRIP_BELOW_TOP - T.PLUG_H) - T.STRIP_FLOOR_TOP
    check(0 < drop < 0.015, f"a released plug drops {drop * 100:.1f} cm onto the cell floor")
    gap = 2 * (T.CELL_IN / 2 - T.PLUG_R)
    check(T.PLUG_H > T.CELL_IN and gap >= 0.012, f"cells leave {gap * 100:.1f} cm of play and are too small for a plug to lie flat")
    clear = (T.CARRY_Z + T.GRIP_BELOW_TOP - T.PLUG_H) - T.PLUG_TOP
    check(clear >= 0.03, f"a carried plug clears the tops of the planted plugs by {clear * 100:.1f} cm")

    print("3./4. demo replay")
    results, events, worst_mm, min_sep, end = replay_demo(T)
    for ev in events:
        print(f"        t={ev[0]:6.2f} s  {ev[1]}")
    check(worst_mm < 3.0, f"worst IK position error along both plans: {worst_mm:.2f} mm")
    check(min_sep > 0.15, f"the two tool points stay at least {min_sep * 100:.0f} cm apart")
    firsts = {}
    for t, s, _ in results:
        firsts.setdefault(s, t)
    print("        first time at each score: " + ", ".join(f"{s:.1f} at {t:.2f} s" for s, t in sorted(firsts.items())))
    check([s for s in sorted(firsts) if s > 0] == [0.4, 0.8, 1.0] or [s for s in sorted(firsts) if s > 0] == [0.2, 0.4, 0.6, 0.8, 1.0],
          "scores rise in 0.2 steps (two arms plant in pairs) to 1.0")
    t_ok = next((t for t, _, ok in results if ok), None)
    check(t_ok is not None and firsts.get(1.0, 0) - t_ok >= T.SUCCESS_HOLD_S - 1e-6,
          f"1.0 only after the hold (success from {t_ok} s, 1.0 from {firsts.get(1.0)} s)")
    check(len(results) <= 2400, f"demo length {len(results) / HZ:.1f} s = {len(results)} steps (run limit 2,400)")

    print("5. negative controls")
    base = start_positions(T)
    done = dict(base)
    for n, c in zip(T.NEW_NAMES, T.DEMO_CELLS):
        done[n] = in_cell(T, c)
    s = run(T, [base] * 30)
    check(s[-1][0] == 0.0, "nothing planted: 0")
    s = run(T, [done] * 15)
    check(s[-1] == (0.8, True), f"completed 0.45 s before the run limit: {s[-1][0]} (not 1.0)")
    s = run(T, [done] * 30)
    check(s[-1][0] == 1.0, f"complete, still for 0.3 s and held 1 s more: {s[-1][0]}")
    three = dict(done)
    three[T.NEW_NAMES[3]] = base[T.NEW_NAMES[3]]
    s = run(T, [three] * 25)
    check(s[-1][0] == 0.6, f"three plugs planted: {s[-1][0]}")
    doubled = dict(three)
    doubled[T.NEW_NAMES[3]] = in_cell(T, T.PLANTED_CELLS[0]) + np.array((0.012, 0, 0.0))
    s = run(T, [doubled] * 25)
    check(s[-1][0] == 0.6 and not s[-1][1], f"fourth plug set into an occupied cell counts for nothing: {s[-1][0]}")
    stacked = dict(done)
    stacked[T.NEW_NAMES[3]] = in_cell(T, T.DEMO_CELLS[0]) + np.array((0.01, 0, 0))
    s = run(T, [stacked] * 25)
    check(s[-1][0] == 0.4, f"two new plugs in one cell count for neither: {s[-1][0]}")
    knocked = dict(done)
    knocked[T.OLD_NAMES[0]] = np.array((0.36, 0.09, T.TOP + T.PLUG_R))
    s = run(T, [knocked] * 25)
    check(s[-1][0] == 0.0, "an original plant knocked out of its cell: 0")
    lifted = dict(base)
    lifted[T.OLD_NAMES[1]] = in_cell(T, T.PLANTED_CELLS[1]) + np.array((0, 0, 0.05))
    s = run(T, [lifted] * 3 + [base] * 20)
    check(s[-1][0] == 0.0, "an original plant pulled up 5 cm and put back: 0")
    s = run(T, [done] * 25)
    jit = [{**done, T.NEW_NAMES[0]: done[T.NEW_NAMES[0]] + np.array((0.002 * (-1) ** k, 0, 0.001 * (k % 2)))} for k in range(30)]
    s = run(T, jit)
    check(s[-1][0] == 1.0, f"a planted plug jittering by 2 mm still counts: {s[-1][0]}")
    slide = [{**done, T.NEW_NAMES[0]: done[T.NEW_NAMES[0]] + np.array((0, 0.001 * k, 0))} for k in range(40)]
    s = run(T, slide)
    check(s[-1][0] == 0.6, f"a plug still sliding (2 cm/s) is not planted: {s[-1][0]}")
    hover = dict(done)
    hover[T.NEW_NAMES[0]] = in_cell(T, T.DEMO_CELLS[0]) + np.array((0, 0, 0.03))
    s = run(T, [hover] * 25)
    check(s[-1][0] == 0.6, f"a plug hanging 3 cm above its cell floor is not planted: {s[-1][0]}")
    dropped = dict(done)
    dropped[T.NEW_NAMES[0]] = np.array((0.3, 0.6, T.TOP - 0.3))
    s = run(T, [dropped] * 25)
    check(s[-1][0] == 0.0, "a plug dropped off the table: 0")
    s = run(T, [done] * 25, tcps=lambda: [done[T.NEW_NAMES[1]] + np.array((0, 0, 0.012)), np.array((0.1, -0.3, 0.9))])
    check(s[-1][0] == 0.8 and not s[-1][1], "all four in but one still held: 0.8, not success")

    print("\n" + ("ALL CHECKS PASSED" if not FAILS else f"{len(FAILS)} CHECK(S) FAILED: " + "; ".join(FAILS)))
    return 1 if FAILS else 0


if __name__ == "__main__":
    sys.exit(main())
