"""Offline checks for this task: no simulator, NumPy and yam_arena only (python check_task.py, a few seconds).

1. The task file loads as the embodiment loads it.
2. Fit and clearance: each part fits its own pocket with play for an open-loop placement; the long shaft and the gear
   do not fit the smaller look-alike pockets; the gripper's housing (5.1 cm either side of the tool point along the
   closing axis, here y; 3.4 cm across, from the robot's collision meshes) clears neighbouring parts at every grip and
   every release; finger tips stay above the table and the pocket walls.
3. The scripted demo: both arms' plans solve with the URDF IK at 20 Hz, and the two tool points stay apart.
4. Scoring: a kinematic replay of the demo rises in 0.2 steps to 0.8 and then 1.0 only after the 1 s hold; negative
   controls score what the rules say, including the short shaft in the long shaft's pocket, a kit completed at the
   run limit (0.8, not 1.0) and a part dropped off the table (0).
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
    spec = importlib.util.spec_from_file_location("kit_task", TASK_PATH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def half_extent(T, name):
    kind, size, _, _ = T.PARTS[name]
    return np.array(size) / 2 if kind == "box" else np.array((size[0], size[0], size[1] / 2))


def state(T, name, pos, vel=(0, 0, 0)) -> ObjectState:
    pos = np.asarray(pos, float)
    h = half_extent(T, name)
    return ObjectState(pos=pos, quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=np.asarray(vel, float),
                       ang_vel=np.zeros(3), low=pos - h, high=pos + h)


def frame(T, positions: dict) -> dict:
    objs = {}
    for o in T.TRAY:
        p = np.asarray(o.pos, float)
        h = np.asarray(o.size) / 2
        objs[o.name] = ObjectState(pos=p, quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=np.zeros(3),
                                   ang_vel=np.zeros(3), low=p - h, high=p + h)
    for name, p in positions.items():
        objs[name] = state(T, name, p)
    return objs


def start_positions(T) -> dict:
    return {n: np.asarray(o.pos, float) for n, o in zip(T.PART_NAMES, T.PART_OBJECTS)}


def in_pocket(T, part, pocket=None) -> np.ndarray:
    cx, cy = T.POCKETS[pocket or part][:2]
    return np.array((cx, cy, T.TRAY_FLOOR_TOP + T.half_height(part)))


def run(T, seq, tcps=None):
    T.new_trial(offline_tcps=tcps)
    out = []
    for positions in seq:
        T.tick()
        objs = frame(T, positions)
        ok = T.success(objs)
        out.append((T.score(objs), ok))
    return out


def rest_z(T, name, p) -> float:
    for (cx, cy, ix, iy) in T.POCKETS.values():
        if abs(p[0] - cx) <= ix / 2 + 0.01 and abs(p[1] - cy) <= iy / 2 + 0.01:
            return T.TRAY_FLOOR_TOP + T.half_height(name)
    return T.TOP + T.half_height(name)


def replay_demo(T):
    plan = T.demo()
    arms = {s: ArmPlan(plan[s], station.BASES[s]) for s in ("left", "right")}
    dur = max(a.duration for a in arms.values())
    pos = start_positions(T)
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
            pos[n] = carried[n] if n in carried else np.array((pos[n][0], pos[n][1], rest_z(T, n, pos[n])))
        T.tick()
        objs = frame(T, pos)
        ok = T.success(objs)
        results.append((t, T.score(objs), ok))
    return results, events, max(a.worst_ik_mm for a in arms.values()), min_sep


def main() -> int:
    print("1. load")
    tf = task_file.load(TASK_PATH)
    check(tf.scene == "station" and len(tf.free_objects) == 4, f"loads: {len(tf.objects)} objects, free: {tf.free_objects}")
    T = load_task()

    print("2. fit and clearance (pads close along y)")
    for n in T.PART_NAMES:
        h = half_extent(T, n)
        _, _, ix, iy = T.POCKETS[n]
        play = min(ix / 2 - h[0], iy / 2 - h[1])
        check(play >= 0.0075, f"{n}: {play * 1000:.1f} mm of play in its own pocket")
    lh = half_extent(T, "long_shaft")
    check(lh[0] > T.POCKETS["short_shaft"][2] / 2, "the long shaft does not fit the short shaft's pocket")
    check(T.PARTS["gear"][1][0] > T.POCKETS["bearing"][2] / 2, "the gear does not fit the bearing's pocket")
    sh = half_extent(T, "short_shaft")
    check(sh[0] < T.POCKETS["long_shaft"][2] / 2, "the short shaft also fits the long shaft's pocket (the trap)")

    def clear_at(xy, others):
        worst = 9.0
        for name, (ox, oy) in others:
            h = half_extent(T, name)
            worst = min(worst, max(abs(oy - xy[1]) - HOUSING_ALONG - h[1], abs(ox - xy[0]) - HOUSING_ACROSS - h[0]))
        return worst

    worst = 9.0
    for n in T.PART_NAMES:
        worst = min(worst, clear_at(T.STARTS[n], [(m, T.STARTS[m]) for m in T.PART_NAMES if m != n]))
    check(worst >= 0.005, f"at the grips, housing clears every other part by {worst * 100:.1f} cm")
    worst = 9.0
    done = []
    order = [T.ORDER["left"][0], T.ORDER["right"][0], T.ORDER["left"][1], T.ORDER["right"][1]]
    for n in order:
        worst = min(worst, clear_at(T.POCKETS[n][:2], [(m, T.POCKETS[m][:2]) for m in done]))
        done.append(n)
    check(worst >= 0.005, f"at each release, housing clears the parts already kitted by {worst * 100:.1f} cm")
    for n in T.PART_NAMES:
        tips = T.grip_z(n) - T.TIP_PAST_TCP
        check(tips >= T.TOP + 0.002, f"{n}: finger tips {(tips - T.TOP) * 1000:.0f} mm above the table at the grip")
    check(T.RELEASE_Z - T.TIP_PAST_TCP > T.TRAY_RIM, "finger tips above the pocket walls at release")
    for n in T.PART_NAMES:
        drop = (T.RELEASE_Z - (T.grip_z(n) - (T.TOP + T.half_height(n)))) - T.half_height(n) - T.TRAY_FLOOR_TOP
        check(0 < drop < 0.035, f"{n}: drops {drop * 100:.1f} cm onto the pocket floor when let go")
    w = {n: 2 * half_extent(T, n)[1] for n in T.PART_NAMES}
    for n in T.PART_NAMES:
        gap = -T.OPEN[n] * 2 - w[n]
        check(gap > 0.01, f"{n}: pads open {(-T.OPEN[n] * 2) * 1000:.0f} mm around {w[n] * 1000:.0f} mm")
    lowest = min(T.CARRY_Z - (T.grip_z(n) - (T.TOP + T.half_height(n))) - T.half_height(n) for n in T.PART_NAMES)
    check(lowest - T.TRAY_RIM >= 0.03, f"a carried part's bottom clears the tray rim by {(lowest - T.TRAY_RIM) * 100:.1f} cm")

    print("3./4. demo replay")
    results, events, worst_mm, min_sep = replay_demo(T)
    for ev in events:
        print(f"        t={ev[0]:6.2f} s  {ev[1]}")
    check(len(events) == 8, "each arm grips and lets go of its two parts")
    check(worst_mm < 3.0, f"worst IK position error along both plans: {worst_mm:.2f} mm")
    check(min_sep > 0.15, f"the two tool points stay at least {min_sep * 100:.0f} cm apart")
    firsts = {}
    for t, s, _ in results:
        firsts.setdefault(s, t)
    print("        first time at each score: " + ", ".join(f"{s:.1f} at {t:.2f} s" for s, t in sorted(firsts.items())))
    seq = [s for s in sorted(firsts) if s > 0]
    check(seq[-1] == 1.0 and all(round(b - a, 3) <= 0.4 for a, b in zip([0.0] + seq[:-1], seq[:-1])),
          "scores rise in 0.2 steps to 0.8, then 1.0")
    t_ok = next((t for t, _, ok in results if ok), None)
    check(t_ok is not None and firsts.get(1.0, 0) - t_ok >= T.SUCCESS_HOLD_S - 1e-6,
          f"1.0 only after the hold (success from {t_ok} s, 1.0 from {firsts.get(1.0)} s)")
    check(len(results) <= 2400, f"demo length {len(results) / HZ:.1f} s = {len(results)} steps (run limit 2,400)")

    print("5. negative controls")
    base = start_positions(T)
    kit = {n: in_pocket(T, n) for n in T.PART_NAMES}
    s = run(T, [base] * 30)
    check(s[-1][0] == 0.0, "nothing kitted: 0")
    s = run(T, [kit] * 15)
    check(s[-1] == (0.8, True), f"kit completed 0.45 s before the run limit: {s[-1][0]} (not 1.0)")
    s = run(T, [kit] * 30)
    check(s[-1][0] == 1.0, f"kit complete, still for 0.3 s and held 1 s more: {s[-1][0]}")
    trap = dict(kit)
    trap["short_shaft"] = in_pocket(T, "short_shaft", "long_shaft")
    trap["long_shaft"] = base["long_shaft"]
    s = run(T, [trap] * 30)
    check(s[-1][0] == 0.4, f"short shaft in the long shaft's pocket, long shaft left out: {s[-1][0]}")
    swap = dict(kit)
    swap["bearing"] = in_pocket(T, "bearing", "gear")
    swap["gear"] = base["gear"]
    s = run(T, [swap] * 30)
    check(s[-1][0] == 0.4, f"bearing in the gear's pocket, gear left out: {s[-1][0]}")
    shared = dict(kit)
    shared["bearing"] = in_pocket(T, "bearing", "gear") + np.array((0.02, 0.02, 0))
    s = run(T, [shared] * 30)
    check(s[-1][0] == 0.4, f"bearing dropped into the gear's pocket beside the gear: {s[-1][0]}")
    on_rim = dict(kit)
    on_rim["gear"] = in_pocket(T, "gear") + np.array((0, 0, 0.02))
    s = run(T, [on_rim] * 30)
    check(s[-1][0] == 0.6, f"gear resting on the pocket walls, not down in it: {s[-1][0]}")
    sliding = [{**kit, "short_shaft": kit["short_shaft"] + np.array((0.001 * k, 0, 0))} for k in range(40)]
    s = run(T, sliding)
    check(s[-1][0] == 0.6, f"a shaft still sliding (2 cm/s): {s[-1][0]}")
    dropped = dict(kit)
    dropped["bearing"] = np.array((0.3, -0.6, T.TOP - 0.3))
    s = run(T, [dropped] * 30)
    check(s[-1][0] == 0.0, "a part dropped off the table: 0")
    s = run(T, [kit] * 30, tcps=lambda: [kit["long_shaft"] + np.array((0, 0, 0.01)), np.array((0.1, -0.3, 0.9))])
    check(s[-1][0] == 0.8 and not s[-1][1], "all four in but one still held: 0.8, not success")

    print("\n" + ("ALL CHECKS PASSED" if not FAILS else f"{len(FAILS)} CHECK(S) FAILED: " + "; ".join(FAILS)))
    return 1 if FAILS else 0


if __name__ == "__main__":
    sys.exit(main())
