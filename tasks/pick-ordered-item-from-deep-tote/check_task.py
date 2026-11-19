"""Offline checks for this task: no simulator, NumPy and yam_arena only (python check_task.py, a few seconds).

1. The task file loads as the embodiment loads it.
2. Occlusion: how much of the red item the tote's walls hide from the station's top camera.
3. The scripted demo: the left arm's plan solves with the URDF IK at 20 Hz.
4. Scoring: a kinematic replay of the demo (the item rides with the closed gripper and drops straight down when let
   go) scores 0.25, 0.5, 0.75 and then 1.0 only after the 1 s hold; negative controls score what the rules say,
   including a pick completed at the run limit (0.75, not 1.0) and a mis-pick of the orange look-alike (0).

The physics trial in the simulator is the real test; this catches layout, reach and scoring mistakes before it.
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


def check(ok: bool, what: str) -> None:
    print(("  ok    " if ok else "  FAIL  ") + what)
    if not ok:
        FAILS.append(what)


def load_task():
    spec = importlib.util.spec_from_file_location("pick_task", TASK_PATH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _hits(p0, p1, centre, size) -> bool:
    d = p1 - p0
    lo, hi = np.asarray(centre) - np.asarray(size) / 2, np.asarray(centre) + np.asarray(size) / 2
    t0, t1 = 0.0, 1.0
    for k in range(3):
        if abs(d[k]) < 1e-12:
            if p0[k] < lo[k] or p0[k] > hi[k]:
                return False
            continue
        a, b = sorted(((lo[k] - p0[k]) / d[k], (hi[k] - p0[k]) / d[k]))
        t0, t1 = max(t0, a), min(t1, b)
        if t0 > t1:
            return False
    return t1 < 0.999


def hidden_fraction(T, cam) -> float:
    walls = [(o.pos, o.size) for o in T.TOTE if "wall" in o.name]
    lx, ly, lz = T.ITEM_SIZE
    cx, cy, cz = T.RED_START
    pts = [np.array((cx + x, cy + y, cz + lz / 2)) for x in np.linspace(-lx / 2, lx / 2, 17)
           for y in np.linspace(-ly / 2, ly / 2, 11)]
    return sum(any(_hits(np.asarray(cam), p, c, s) for c, s in walls) for p in pts) / len(pts)


def state(pos, size, vel=(0, 0, 0)) -> ObjectState:
    pos = np.asarray(pos, float)
    half = np.asarray(size) / 2
    return ObjectState(pos=pos, quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=np.asarray(vel, float),
                       ang_vel=np.zeros(3), low=pos - half, high=pos + half)


def fixed(T) -> dict:
    return {o.name: state(o.pos, o.size) for o in [*T.TOTE, *T.ORDER_BIN]}


def frame(T, red, orange, red_vel=(0, 0, 0)) -> dict:
    objs = fixed(T)
    objs["red_item"] = state(red, T.ITEM_SIZE, red_vel)
    objs["orange_item"] = state(orange, T.ITEM_SIZE)
    return objs


def run(T, red_path, orange=None, tcps=None):
    """Feed a sequence of red-item positions, one per control step; return the per-step (score, success)."""
    orange = orange or T.ORANGE_START
    T.new_trial(offline_tcps=tcps)
    out, prev = [], None
    for p in red_path:
        T.tick()
        vel = (0, 0, 0) if prev is None else (np.asarray(p) - np.asarray(prev)) * HZ
        prev = p
        objs = frame(T, p, orange, vel)
        ok = T.success(objs)
        out.append((T.score(objs), ok))
    return out


def replay_demo(T):
    plan = T.demo()
    arm = ArmPlan(plan["left"], station.BASES["left"])
    red = np.array(T.RED_START, float)
    grip_off = None
    tcp_now = {"left": None}
    T.new_trial(offline_tcps=lambda: [tcp_now["left"]])
    results, events, prev = [], [], red.copy()
    steps = int(math.ceil(arm.duration * HZ)) + 1
    for k in range(steps):
        t = k / HZ
        arm.action7(t)
        _pos, _rot, finger, _label = arm.target(t)
        tcp = np.asarray(ik.tcp_pose(arm.q, np.array(station.BASES["left"]))[0])
        tcp_now["left"] = tcp
        closed = finger > -0.003
        near = np.all(np.abs(tcp - red) <= np.array(T.ITEM_SIZE) / 2 + 0.02)
        if closed and near and grip_off is None:
            grip_off = red - tcp
            events.append((round(t, 2), "grips the red item"))
        if grip_off is not None and not closed:
            grip_off = None
            events.append((round(t, 2), "lets go"))
        if grip_off is not None:
            red = tcp + grip_off
        else:
            if T._within(red[0], red[1], T.BIN_XY, T.BIN_IN):
                floor = T.BIN_FLOOR_TOP
            elif T._within(red[0], red[1], T.TOTE_XY, T.TOTE_IN):
                floor = T.TOTE_FLOOR_TOP
            else:
                floor = T.TOP
            red[2] = floor + T.ITEM_SIZE[2] / 2
        T.tick()
        vel = (red - prev) * HZ
        prev = red.copy()
        objs = frame(T, red, T.ORANGE_START, vel)
        ok = T.success(objs)
        results.append((t, T.score(objs), ok))
    return results, events, arm.worst_ik_mm, red


def main() -> int:
    print("1. load")
    tf = task_file.load(TASK_PATH)
    check(tf.scene == "station" and set(tf.free_objects) == {"red_item", "orange_item"},
          f"loads: {len(tf.objects)} objects, free: {tf.free_objects}")
    T = load_task()

    print("2. occlusion (share of the red item's top face hidden from the top camera by the tote's walls)")
    for variant in ("calibrated", "demo"):
        f = hidden_fraction(T, station.top_camera(variant).pos)
        print(f"        {variant:10s} top camera: {f:.0%}")

    print("2b. gripper housing clearance at the grip (housing reaches 5.1 cm along the closing axis, 3.4 cm across)")
    rx, ry = T.RED_START[0], T.RED_START[1]
    front_in = T.TOTE_XY[0] - T.TOTE_IN[0] / 2
    back_in = T.TOTE_XY[0] + T.TOTE_IN[0] / 2
    right_in = T.TOTE_XY[1] - T.TOTE_IN[1] / 2
    left_in = T.TOTE_XY[1] + T.TOTE_IN[1] / 2
    gaps = {"front wall": rx - 0.051 - front_in, "back wall": back_in - (rx + 0.051),
            "right wall": ry - 0.034 - right_in, "left wall": left_in - (ry + 0.034)}
    for k, g in gaps.items():
        check(g >= 0.005, f"housing clears the tote's {k} by {g * 100:.1f} cm")

    print("3./4. demo replay")
    results, events, worst_mm, final = replay_demo(T)
    for ev in events:
        print(f"        t={ev[0]:5.2f} s  {ev[1]}")
    check(worst_mm < 3.0, f"worst IK position error along the plan: {worst_mm:.2f} mm")
    firsts = {}
    for t, s, _ in results:
        firsts.setdefault(s, t)
    print("        first time at each score: " + ", ".join(f"{s:.2f} at {t:.2f} s" for s, t in sorted(firsts.items())))
    check([s for s in sorted(firsts) if s > 0] == [0.25, 0.5, 0.75, 1.0], "scores rise 0.25, 0.5, 0.75, 1.0")
    t_success = next((t for t, _, ok in results if ok), None)
    t_one = firsts.get(1.0)
    check(t_success is not None and t_one is not None and t_one - t_success >= T.SUCCESS_HOLD_S - 1e-6,
          f"1.0 only after the hold: success from {t_success} s, score 1.0 from {t_one} s")
    held = sum(1 for _, _, ok in results if ok) / HZ
    check(held >= T.SUCCESS_HOLD_S, f"success holds {held:.1f} s in the demo (needs {T.SUCCESS_HOLD_S} s)")
    check(len(results) <= 2400, f"demo length {len(results) / HZ:.1f} s = {len(results)} steps (run limit 2,400)")

    print("5. negative controls")
    start = T.RED_START
    lifted = (start[0], start[1], start[2] + 0.05)
    out_of_tote = (T.BIN_XY[0], T.BIN_XY[1], T.TOTE_RIM + 0.06)
    in_bin = (T.BIN_XY[0], T.BIN_XY[1], T.BIN_FLOOR_TOP + T.ITEM_SIZE[2] / 2)
    s = run(T, [start] * 40)
    check(s[-1][0] == 0.0, "red item left in the tote: 0")
    s = run(T, [lifted, out_of_tote] + [in_bin] * 10)
    check(s[-1] == (0.75, True), f"pick completed 0.5 s before the run limit: {s[-1][0]} (not 1.0), success true")
    s = run(T, [lifted, out_of_tote] + [in_bin] * 25)
    check(s[-1][0] == 1.0, f"pick completed and held 1.25 s: {s[-1][0]}")
    s = run(T, [lifted, out_of_tote] + [in_bin] * 25, orange=in_bin)
    check(s[-1][0] == 0.0 and not s[-1][1], "orange look-alike put in the order bin (mis-pick): 0")
    s = run(T, [lifted, out_of_tote] + [in_bin] * 25, orange=(0.40, 0.45, T.TOP + T.ITEM_SIZE[2] / 2))
    check(s[-1][0] == 0.0, "orange look-alike taken out of the tote and left on the table: 0")
    s = run(T, [lifted, out_of_tote, (0.3, 0.5, T.TOP - 0.3)])
    check(s[-1][0] == 0.0, "red item dropped off the table: 0")
    s = run(T, [lifted, out_of_tote] + [in_bin] * 25,
            tcps=lambda: [np.array((T.BIN_XY[0], T.BIN_XY[1], T.BIN_FLOOR_TOP + 0.05))])
    check(s[-1][0] == 0.75 and not s[-1][1], "in the bin but still held: 0.75, not success")

    print("\n" + ("ALL CHECKS PASSED" if not FAILS else f"{len(FAILS)} CHECK(S) FAILED: " + "; ".join(FAILS)))
    return 1 if FAILS else 0


if __name__ == "__main__":
    sys.exit(main())
