"""Offline checks for this task: no simulator, NumPy and yam_arena only (python check_task.py, a few seconds).

1. The task file loads as the embodiment loads it.
2. Geometry: the tote fits a bin with play for an open-loop placement; the finger tips stay above the table at the grip
   and above the bin walls at release; a carried tote clears row B's walls and the stored totes; the gripper's housing
   (5.1 cm either side of the tool point along the closing axis, y; from the robot's collision meshes) clears the
   stored totes at release.
3. The scripted demo: the left arm's plan solves with the URDF IK at 20 Hz.
4. Scoring: a kinematic replay of the demo rises 0.25, 0.5, 0.75 and then 1.0 only after the 1 s hold; negative
   controls score what the rules say, including the tote put away in A2, B1 or B3 (0), a stored tote knocked out (0)
   and a put-away completed at the run limit (0.75, not 1.0).
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
HOUSING_ALONG = 0.051


def check(ok: bool, what: str) -> None:
    print(("  ok    " if ok else "  FAIL  ") + what)
    if not ok:
        FAILS.append(what)


def load_task():
    spec = importlib.util.spec_from_file_location("putaway_task", TASK_PATH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def state(T, pos) -> ObjectState:
    pos = np.asarray(pos, float)
    h = np.array(T.TOTE) / 2
    return ObjectState(pos=pos, quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=pos - h, high=pos + h)


def frame(T, positions: dict) -> dict:
    objs = {}
    for o in T.RACK:
        p = np.asarray(o.pos, float)
        h = np.asarray(o.size) / 2
        objs[o.name] = ObjectState(pos=p, quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=np.zeros(3),
                                   ang_vel=np.zeros(3), low=p - h, high=p + h)
    for name, p in positions.items():
        objs[name] = state(T, p)
    return objs


def at(T, loc) -> np.ndarray:
    cx, cy, row = T.LOCATIONS[loc]
    return np.array((cx, cy, T.floor_top(row) + T.TOTE[2] / 2))


def start(T) -> dict:
    pos = {"tote": np.array((*T.TOTE_START, T.TOP + T.TOTE[2] / 2))}
    for st, loc in zip(T.STORED_TOTES, T.STORED_IN):
        pos[st.name] = at(T, loc)
    return pos


def run(T, seq, tcps=None):
    T.new_trial(offline_tcps=tcps)
    out = []
    for positions in seq:
        T.tick()
        objs = frame(T, positions)
        ok = T.success(objs)
        out.append((T.score(objs), ok))
    return out


def rest_z(T, p) -> float:
    for loc, (cx, cy, row) in T.LOCATIONS.items():
        if abs(p[0] - cx) <= T.BIN_IN[0] / 2 + 0.003 and abs(p[1] - cy) <= T.BIN_IN[1] / 2 + 0.003:
            return T.floor_top(row) + T.TOTE[2] / 2
    return T.TOP + T.TOTE[2] / 2


def replay_demo(T):
    arm = ArmPlan(T.demo()["left"], station.BASES["left"])
    pos = start(T)
    held = None
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
        if closed and held is None and np.linalg.norm(tcp[:2] - pos["tote"][:2]) < 0.02:
            held = pos["tote"] - tcp
            events.append((round(t, 2), "grips the tote"))
        if held is not None and not closed:
            events.append((round(t, 2), "lets go"))
            held = None
        pos["tote"] = tcp + held if held is not None else np.array((pos["tote"][0], pos["tote"][1], rest_z(T, pos["tote"])))
        T.tick()
        objs = frame(T, pos)
        ok = T.success(objs)
        results.append((t, T.score(objs), ok))
    return results, events, arm.worst_ik_mm, pos


def main() -> int:
    print("1. load")
    tf = task_file.load(TASK_PATH)
    check(tf.scene == "station" and len(tf.free_objects) == 3, f"loads: {len(tf.objects)} objects, free: {tf.free_objects}")
    T = load_task()

    print("2. geometry")
    play = min(T.BIN_IN[0] - T.TOTE[0], T.BIN_IN[1] - T.TOTE[1]) / 2
    check(play >= 0.012, f"the tote has {play * 1000:.0f} mm of play in a bin")
    tips = T.TOP + T.GRIP_ABOVE_BOTTOM - T.TIP_PAST_TCP
    check(tips > T.TOP + 0.01, f"finger tips {(tips - T.TOP) * 1000:.0f} mm above the table at the grip")
    check(T.RELEASE_Z - T.TIP_PAST_TCP > T.rim("B"), "finger tips above row B's bin walls at release")
    drop = (T.RELEASE_Z - T.GRIP_ABOVE_BOTTOM) - T.floor_top("B")
    check(0 < drop < 0.03, f"the tote drops {drop * 100:.1f} cm onto the B2 floor when let go")
    bottom = T.CARRY_Z - T.GRIP_ABOVE_BOTTOM
    check(bottom - T.rim("B") >= 0.03, f"a carried tote's bottom clears row B's walls by {(bottom - T.rim('B')) * 100:.1f} cm")
    # the carry route: along y in front of the rack, then straight along x over A2 into B2; no stored tote under it
    hx, hy = T.TOTE[0] / 2, T.TOTE[1] / 2
    cx2, cy2, _ = T.LOCATIONS[T.TARGET]
    xf = T.ROWS["A"][0] - 0.08
    under = []
    for loc in T.STORED_IN:
        sx, sy, _ = T.LOCATIONS[loc]
        on_leg1 = abs(sx - xf) < 2 * hx and min(T.TOTE_START[1], cy2) - 2 * hy < sy < max(T.TOTE_START[1], cy2) + 2 * hy
        on_leg2 = abs(sy - cy2) < 2 * hy and xf - 2 * hx < sx < cx2 + 2 * hx
        if on_leg1 or on_leg2:
            under.append(loc)
    check(not under, f"the carry route passes over no stored tote (under it: {under or 'none'})")
    cx, cy, _ = T.LOCATIONS[T.TARGET]
    worst = min(abs(T.LOCATIONS[l][1] - cy) - HOUSING_ALONG - T.TOTE[1] / 2 for l in T.STORED_IN
                if abs(T.LOCATIONS[l][0] - cx) < 0.05)
    check(worst > 0.01, f"at release, housing clears the stored tote beside B2 by {worst * 100:.1f} cm")
    check(-T.OPEN * 2 - T.TOTE[1] > 0.02, f"pads open {-T.OPEN * 2000:.0f} mm around the 55 mm tote")

    print("3./4. demo replay")
    results, events, worst_mm, end = replay_demo(T)
    for ev in events:
        print(f"        t={ev[0]:6.2f} s  {ev[1]}")
    check(worst_mm < 3.0, f"worst IK position error along the plan: {worst_mm:.2f} mm")
    firsts = {}
    for t, s, _ in results:
        firsts.setdefault(s, t)
    print("        first time at each score: " + ", ".join(f"{s:.2f} at {t:.2f} s" for s, t in sorted(firsts.items())))
    check([s for s in sorted(firsts) if s > 0] == [0.25, 0.5, 0.75, 1.0], "scores rise 0.25, 0.5, 0.75, 1.0")
    t_ok = next((t for t, _, ok in results if ok), None)
    check(t_ok is not None and firsts.get(1.0, 0) - t_ok >= T.SUCCESS_HOLD_S - 1e-6,
          f"1.0 only after the hold (success from {t_ok} s, 1.0 from {firsts.get(1.0)} s)")
    check(len(results) <= 2400, f"demo length {len(results) / HZ:.1f} s = {len(results)} steps (run limit 2,400)")

    print("5. negative controls")
    s0 = start(T)
    done = {**s0, "tote": at(T, "B2")}
    lift = {**s0, "tote": s0["tote"] + np.array((0, 0, 0.06))}
    s = run(T, [s0] * 30)
    check(s[-1][0] == 0.0, "nothing done: 0")
    s = run(T, [lift] * 5 + [s0] * 25)
    check(s[-1][0] == 0.25, f"tote lifted and put back down: {s[-1][0]}")
    s = run(T, [lift] * 3 + [done] * 15)
    check(s[-1] == (0.75, True), f"put away 0.45 s before the run limit: {s[-1][0]} (not 1.0)")
    s = run(T, [lift] * 3 + [done] * 30)
    check(s[-1][0] == 1.0, f"put away in B2, still and held 1 s: {s[-1][0]}")
    for wrong in ("A2", "B1", "A3"):
        s = run(T, [lift] * 3 + [{**s0, "tote": at(T, wrong)}] * 30)
        check(s[-1][0] == 0.0, f"put away in {wrong}: 0")
    s = run(T, [lift] * 3 + [{**s0, "tote": at(T, "A2")}] * 3 + [done] * 30)
    check(s[-1][0] == 1.0, f"lowered into A2 for 0.15 s (not still), then put in B2: {s[-1][0]}")
    s = run(T, [lift] * 3 + [{**s0, "tote": at(T, "A2")}] * 10 + [done] * 30)
    check(s[-1][0] == 0.0, f"set down in A2 (still), then moved to B2: 0")
    knocked = {**done, "stored_B3": np.array((0.40, -0.10, T.TOP + 0.025))}
    s = run(T, [lift] * 3 + [knocked] * 30)
    check(s[-1][0] == 0.0, "a stored tote knocked out of B3: 0")
    s = run(T, [lift] * 3 + [done] * 30, tcps=lambda: [at(T, "B2") + np.array((0, 0, 0.013))])
    check(s[-1][0] == 0.75 and not s[-1][1], "in B2 but still held: 0.75, not success")
    s = run(T, [lift] * 3 + [{**done, "tote": np.array((0.3, 0.6, T.TOP - 0.3))}] * 30)
    check(s[-1][0] == 0.0, "tote dropped off the table: 0")

    print("\n" + ("ALL CHECKS PASSED" if not FAILS else f"{len(FAILS)} CHECK(S) FAILED: " + "; ".join(FAILS)))
    return 1 if FAILS else 0


if __name__ == "__main__":
    sys.exit(main())
