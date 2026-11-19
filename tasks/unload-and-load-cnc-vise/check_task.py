"""Offline checks for this task: no simulator, NumPy and yam_arena only (python check_task.py, a few seconds).

1. The task file loads as the embodiment loads it.
2. Geometry: the finished part starts seated (its far and left faces on the fixed jaw and side stop); grips on the
   part's upper half keep the finger tips above the stops; the gripper's housing (5.1 cm either side of the tool
   point along the closing axis, y; from the robot's collision meshes) is above the stops when gripping in the vise;
   a carried part clears the stops and the tray lip.
3. The scripted demo: both arms' plans solve with the URDF IK at 20 Hz, the two tool points stay apart, and the left
   arm reaches the vise only after the right arm has left it.
4. Scoring: a kinematic replay (a part rides with the closed gripper at it, is stopped by the jaw and side stop, and
   drops straight down when let go) rises 0.25, 0.5, 0.75 and then 1.0 only after the 1 s hold; negative controls
   score what the rules say, including the blank loaded with the finished part still in the vise (0), a blank 4 mm
   off its stops or turned 10 degrees (0.75, not seated) and a changeover completed at the run limit (0.75).
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
    spec = importlib.util.spec_from_file_location("cnc_task", TASK_PATH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def state(T, pos, yaw=0.0) -> ObjectState:
    pos = np.asarray(pos, float)
    c, s = abs(math.cos(yaw)), abs(math.sin(yaw))
    hx, hy, hz = np.array(T.PART) / 2
    h = np.array((c * hx + s * hy, s * hx + c * hy, hz))
    return ObjectState(pos=pos, quat_xyzw=np.array([0, 0, math.sin(yaw / 2), math.cos(yaw / 2)]),
                       lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=pos - h, high=pos + h)


def frame(T, positions: dict, yaws: dict | None = None) -> dict:
    objs = {}
    for o in [*T.VISE, *T.TRAY]:
        p = np.asarray(o.pos, float)
        h = np.asarray(o.size) / 2
        objs[o.name] = ObjectState(pos=p, quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=np.zeros(3),
                                   ang_vel=np.zeros(3), low=p - h, high=p + h)
    for name, p in positions.items():
        objs[name] = state(T, p, (yaws or {}).get(name, 0.0))
    return objs


def run(T, seq, tcps=None, yaws=None):
    T.new_trial(offline_tcps=tcps)
    out = []
    for positions in seq:
        T.tick()
        objs = frame(T, positions, yaws)
        ok = T.success(objs)
        out.append((T.score(objs), ok))
    return out


SEATED = None


def settle(T, p):
    """Where a released part comes to rest (straight down), and the jaw/stop limits on a part in the vise."""
    x, y = p[0], p[1]
    vx0, vy0 = T.VISE_XY[0] - T.VISE_SIZE[0] / 2, T.VISE_XY[1] - T.VISE_SIZE[1] / 2
    if vx0 <= x <= T.JAW_FACE_X and vy0 <= y <= T.STOP_FACE_Y:
        return np.array((x, y, T.SEATED_Z))
    if abs(x - T.TRAY_XY[0]) <= T.TRAY_IN / 2 and abs(y - T.TRAY_XY[1]) <= T.TRAY_IN / 2:
        return np.array((x, y, T.TRAY_FLOOR + T.PART[2] / 2))
    return np.array((x, y, T.TOP + T.PART[2] / 2))


def replay_demo(T):
    plan = T.demo()
    arms = {s: ArmPlan(plan[s], station.BASES[s]) for s in ("left", "right")}
    dur = max(a.duration for a in arms.values())
    pos = {"finished_part": np.array((*T.SEATED_XY, T.SEATED_Z)),
           "blank": np.array((*T.BLANK_START, T.TOP + T.PART[2] / 2))}
    held = {"left": None, "right": None}
    tcp_now = {}
    T.new_trial(offline_tcps=lambda: [tcp_now["left"], tcp_now["right"]])
    results, events, min_sep = [], [], 9.0
    left_in_vise = right_left_vise = None
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
                    if np.linalg.norm(tcp[:2] - p[:2]) < 0.02 and abs(tcp[2] - p[2]) < 0.04 \
                            and n not in [h[0] for h in held.values() if h]:
                        held[s] = (n, p - tcp)
                        events.append((round(t, 2), f"{s} grips {n}"))
                        break
            if held[s] is not None and not closed:
                events.append((round(t, 2), f"{s} lets go of {held[s][0]}"))
                held[s] = None
        near_vise = lambda tc: abs(tc[0] - T.VISE_XY[0]) < 0.09 and abs(tc[1] - T.VISE_XY[1]) < 0.09
        if right_left_vise is None and t > 3 and not near_vise(tcp_now["right"]):
            right_left_vise = t
        if left_in_vise is None and near_vise(tcp_now["left"]):
            left_in_vise = t
        min_sep = min(min_sep, float(np.linalg.norm(tcp_now["left"] - tcp_now["right"])))
        carried = {h[0]: tcp_now[s] + h[1] for s, h in held.items() if h}
        for n in pos:
            if n in carried:
                p = carried[n].copy()
                vx0, vy0 = T.VISE_XY[0] - T.VISE_SIZE[0] / 2, T.VISE_XY[1] - T.VISE_SIZE[1] / 2
                if p[2] - T.PART[2] / 2 < T.VISE_FLOOR + T.STOP_H and vx0 <= p[0] and vy0 <= p[1]:
                    p[0] = min(p[0], T.SEATED_XY[0])  # stopped by the fixed jaw
                    p[1] = min(p[1], T.SEATED_XY[1])  # and the side stop
                pos[n] = p
            else:
                pos[n] = settle(T, pos[n])
        T.tick()
        objs = frame(T, pos)
        ok = T.success(objs)
        results.append((t, T.score(objs), ok))
    return results, events, max(a.worst_ik_mm for a in arms.values()), min_sep, right_left_vise, left_in_vise, pos


def main() -> int:
    print("1. load")
    tf = task_file.load(TASK_PATH)
    check(tf.scene == "station" and len(tf.free_objects) == 2, f"loads: {len(tf.objects)} objects, free: {tf.free_objects}")
    T = load_task()

    print("2. geometry")
    f = state(T, (*T.SEATED_XY, T.SEATED_Z))
    check(abs(T.JAW_FACE_X - f.high[0]) < 1e-9 and abs(T.STOP_FACE_Y - f.high[1]) < 1e-9,
          "the finished part starts against the fixed jaw and side stop")
    tips_above_part_bottom = T.GRIP_ABOVE_BOTTOM - T.TIP_PAST_TCP
    check(tips_above_part_bottom > T.STOP_H, f"finger tips {(tips_above_part_bottom - T.STOP_H) * 1000:.0f} mm above the stops' tops when gripping in the vise")
    check(T.GRIP_ABOVE_BOTTOM <= T.PART[2] + 0.002, f"tool point within the part's height ({T.GRIP_ABOVE_BOTTOM * 1000:.0f} of {T.PART[2] * 1000:.0f} mm): pads grip {(T.PART[2] - tips_above_part_bottom) * 1000:.0f} mm of it")
    check(-T.OPEN * 2 - T.PART[1] > 0.02, f"pads open {-T.OPEN * 2000:.0f} mm around the 40 mm part")
    carry_bottom = T.CARRY_Z - T.GRIP_ABOVE_BOTTOM
    check(carry_bottom - (T.VISE_FLOOR + T.STOP_H) >= 0.03, f"a carried part clears the stops by {(carry_bottom - T.VISE_FLOOR - T.STOP_H) * 100:.1f} cm")
    check(carry_bottom - (T.TRAY_FLOOR + T.TRAY_LIP) >= 0.03, "a carried part clears the tray lip by at least 3 cm")
    check(T.PART[0] < T.TRAY_IN - 0.05, "the tray leaves 3.5 cm of play around the part")

    print("3./4. demo replay")
    results, events, worst_mm, min_sep, r_out, l_in, end = replay_demo(T)
    for ev in events:
        print(f"        t={ev[0]:6.2f} s  {ev[1]}")
    check(len(events) == 4, "each arm grips and lets go of its part")
    check(worst_mm < 3.0, f"worst IK position error along both plans: {worst_mm:.2f} mm")
    check(min_sep > 0.15, f"the two tool points stay at least {min_sep * 100:.0f} cm apart")
    check(r_out is not None and l_in is not None and l_in > r_out, f"right arm leaves the vise at {r_out} s, left arm arrives at {l_in} s")
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
    seated = np.array((*T.SEATED_XY, T.SEATED_Z))
    blank0 = np.array((*T.BLANK_START, T.TOP + T.PART[2] / 2))
    in_tray = np.array((*T.TRAY_XY, T.TRAY_FLOOR + T.PART[2] / 2))
    start = {"finished_part": seated, "blank": blank0}
    done = {"finished_part": in_tray, "blank": seated}
    s = run(T, [start] * 30)
    check(s[-1][0] == 0.0, "nothing done: 0")
    s = run(T, [{"finished_part": np.array((0.2, -0.05, T.TOP + 0.015)), "blank": blank0}] * 30)
    check(s[-1][0] == 0.25, f"finished part out of the vise, on the table: {s[-1][0]}")
    s = run(T, [{"finished_part": in_tray, "blank": blank0}] * 30)
    check(s[-1][0] == 0.5, f"finished part in the tray: {s[-1][0]}")
    s = run(T, [done] * 15)
    check(s[-1] == (0.75, True), f"changeover completed 0.45 s before the run limit: {s[-1][0]} (not 1.0)")
    s = run(T, [done] * 30)
    check(s[-1][0] == 1.0, f"changeover complete, still for 0.3 s and held 1 s more: {s[-1][0]}")
    off = {"finished_part": in_tray, "blank": seated - np.array((0.004, 0, 0))}
    s = run(T, [off] * 30)
    check(s[-1][0] == 0.75, f"blank 4 mm off the fixed jaw: {s[-1][0]}")
    off = {"finished_part": in_tray, "blank": seated - np.array((0, 0.004, 0))}
    s = run(T, [off] * 30)
    check(s[-1][0] == 0.75, f"blank 4 mm off the side stop: {s[-1][0]}")
    turned = {"finished_part": in_tray, "blank": seated - np.array((0.006, 0.006, 0))}
    s = run(T, [turned] * 30, yaws={"blank": math.radians(10)})
    check(s[-1][0] == 0.75, f"blank turned 10 degrees: {s[-1][0]}")
    crash = [{"finished_part": seated, "blank": blank0}] * 5 + [{"finished_part": seated, "blank": seated - np.array((0.0, 0.03, 0))}] * 3 \
        + [done] * 30
    s = run(T, crash)
    check(s[-1][0] == 0.0, "blank put into the vise with the finished part still in it, then fixed: 0")
    s = run(T, [{"finished_part": np.array((0.3, -0.6, T.TOP - 0.3)), "blank": seated}] * 30)
    check(s[-1][0] == 0.0, "finished part dropped off the table: 0")
    s = run(T, [done] * 30, tcps=lambda: [seated + np.array((0, 0, 0.016)), np.array((0.1, -0.3, 0.9))])
    check(s[-1][0] == 0.75 and not s[-1][1], "blank seated but still held: 0.75, not success")

    print("\n" + ("ALL CHECKS PASSED" if not FAILS else f"{len(FAILS)} CHECK(S) FAILED: " + "; ".join(FAILS)))
    return 1 if FAILS else 0


if __name__ == "__main__":
    sys.exit(main())
