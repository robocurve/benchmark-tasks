"""Offline checks for this task: no simulator, NumPy and yam_arena only (python check_task.py, ~1 minute).

1. The task file loads as the embodiment loads it.
2. Reach: neither arm can do the task alone (sampled over each arm's joint limits).
3. Occlusion: how much of a level lying in the slot the caddy's walls hide from the station's top camera.
4. The scripted demo: both arms' plans solve with the URDF IK at 20 Hz, and the two grippers stay apart.
5. Scoring: a kinematic replay of the demo (the level moves with the gripper holding it and drops into the slot when
   let go) scores 0.25, 0.5, 0.75 and 1.0 in order; negative controls score what the rules say.

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
    spec = importlib.util.spec_from_file_location("stow_level_task", TASK_PATH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


# -- 2. reach ----------------------------------------------------------------------------------------------------------

JOINT_LIMITS = [(-2.61799, 3.14159), (0.0, 3.66519), (0.0, 3.14159), (-1.69297, 1.5708), (-1.5708, 1.5708),
                (-2.0944, 2.0944)]  # yam_arm.urdf, joints 1-6


def max_reach(samples: int = 300_000, seed: int = 0) -> float:
    """Farthest the tool point or the pad tips get from the arm's base, horizontally, at or above the table."""
    rng = np.random.default_rng(seed)
    base = np.array(station.BASES["right"])
    best = 0.0
    lo = np.array([a for a, _ in JOINT_LIMITS])
    hi = np.array([b for _, b in JOINT_LIMITS])
    for q in rng.uniform(lo, hi, size=(samples, 6)):
        p, rot = ik.tcp_pose(q, base)
        for pt in (p, p + rot[:, 2] * 0.013):
            if pt[2] >= station.TABLE_TOP - 0.005:
                best = max(best, float(math.hypot(pt[0] - base[0], pt[1] - base[1])))
    return best


def nearest(base_xy, lo_xy, hi_xy) -> float:
    """Horizontal distance from a point to an axis-aligned rectangle."""
    dx = max(lo_xy[0] - base_xy[0], 0.0, base_xy[0] - hi_xy[0])
    dy = max(lo_xy[1] - base_xy[1], 0.0, base_xy[1] - hi_xy[1])
    return math.hypot(dx, dy)


# -- 3. occlusion ------------------------------------------------------------------------------------------------------


def _segment_hits_box(p0, p1, centre, size) -> bool:
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


def hidden_fraction(T, camera_pos, y_offset: float) -> float:
    """Share of the level's top face and inner side face hidden by the caddy's walls, the level lying in the slot."""
    walls = [(o.pos, o.size) for o in T.CADDY if o.name.startswith("caddy_wall")]
    lx, ly, lz = T.LEVEL_SIZE
    cy = T.CADDY_XY[1] + y_offset
    pts = [np.array((T.CADDY_XY[0] + x, cy + y, T.FLOOR_TOP + lz))
           for x in np.linspace(-lx / 2, lx / 2, 61) for y in np.linspace(-ly / 2, ly / 2, 6)]
    pts += [np.array((T.CADDY_XY[0] + x, cy + ly / 2, T.FLOOR_TOP + z))
            for x in np.linspace(-lx / 2, lx / 2, 31) for z in np.linspace(0.002, lz, 6)]
    cam = np.asarray(camera_pos)
    return sum(any(_segment_hits_box(cam, p, c, s) for c, s in walls) for p in pts) / len(pts)


# -- 4./5. the demo, replayed kinematically ----------------------------------------------------------------------------


def _quat_from_rot(r):
    return ik.matrix_to_quat_xyzw(r)


def _state(pos, rot, size, vel=(0.0, 0.0, 0.0), spin=(0.0, 0.0, 0.0)) -> ObjectState:
    corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * np.asarray(size) / 2
    w = corners @ np.asarray(rot).T + np.asarray(pos)
    return ObjectState(pos=np.asarray(pos, float), quat_xyzw=np.asarray(_quat_from_rot(rot), float),
                       lin_vel=np.asarray(vel, float), ang_vel=np.asarray(spin, float), low=w.min(0), high=w.max(0))


def fixed_states(T) -> dict:
    return {o.name: _state(o.pos, np.eye(3), o.size) for o in [T.TRAY, *T.CADDY]}


def replay_demo(T):
    """Play both arm plans at 20 Hz. The level rides with a closed gripper that is at it; when no gripper holds it, it
    falls straight down onto whatever is below (tray, slot floor or table). Yields per-step scores."""
    plans = T.demo()
    bases = station.BASES
    arms = {side: ArmPlan(plans[side], bases[side]) for side in ("left", "right")}
    duration = max(a.duration for a in arms.values())
    level_pos = np.array(T.LEVEL_START, float)
    holder, grip_offset = None, None
    tcp_now = {}
    T.new_trial(offline_tcps=lambda: [tcp_now["left"], tcp_now["right"]])
    scores, events, min_gap, worst_mm = [], [], 9.0, 0.0
    steps = int(math.ceil(duration * HZ)) + 1
    prev_level = level_pos.copy()
    for k in range(steps):
        t = k / HZ
        fingers = {}
        for side, arm in arms.items():
            arm.action7(t)  # advances the IK warm start and records the worst IK error
            pos, _rot, finger, _label = arm.target(t)
            tcp_now[side] = np.asarray(ik.tcp_pose(arm.q, np.array(bases[side]))[0])
            fingers[side] = finger
        worst_mm = max(worst_mm, *(a.worst_ik_mm for a in arms.values()))
        min_gap = min(min_gap, float(np.linalg.norm(tcp_now["left"] - tcp_now["right"])))
        # grasping: a gripper nearly closed (pads within 3 mm of the level's 25 mm thickness) at the level
        closed = {s: (fingers[s] - T.FINGER_CLOSED) > -0.003 for s in fingers}  # FINGER_CLOSED is 0, open is -0.047
        for side in ("left", "right"):
            near = np.all(np.abs(tcp_now[side] - level_pos) <= np.array(T.LEVEL_SIZE) / 2 + 0.02)
            if closed[side] and near and holder != side and (holder is None or not closed[holder] or side == "right"):
                if holder is None or not closed[holder] or side == "right" and holder == "left":
                    if holder != side:
                        events.append((round(t, 2), f"{side} takes the level"))
                    holder, grip_offset = side, level_pos - tcp_now[side]
        if holder is not None and not closed[holder]:
            events.append((round(t, 2), f"{holder} lets go"))
            holder = None
        if holder is not None:
            level_pos = tcp_now[holder] + grip_offset
        else:  # fall straight down to the surface below
            x, y = level_pos[:2]
            half = np.array(T.LEVEL_SIZE) / 2
            if T._over_slot(x - half[0], y - half[1]) and T._over_slot(x + half[0], y + half[1]):
                floor = T.FLOOR_TOP
            elif abs(y - T.TRAY_XY[1]) < T.TRAY_SIZE[1] / 2:
                floor = T.TRAY_TOP
            else:
                floor = T.TOP
            level_pos[2] = floor + half[2]
        vel = (level_pos - prev_level) * HZ
        prev_level = level_pos.copy()
        objs = fixed_states(T)
        objs["spirit_level"] = _state(level_pos, np.eye(3), T.LEVEL_SIZE, vel=vel)
        scores.append((t, T.score(objs), T.success(objs)))
    return scores, events, min_gap, worst_mm, level_pos


def main() -> int:
    print("1. load")
    tf = task_file.load(TASK_PATH)
    check(tf.scene == "station" and tf.free_objects == ("spirit_level",), f"loads: {len(tf.objects)} objects, "
          f"free: {tf.free_objects}")
    T = load_task()
    T.FINGER_CLOSED = 0.0

    print("2. reach (neither arm alone)")
    r = max_reach()
    lx, ly, _ = T.LEVEL_SIZE
    level_lo = (T.LEVEL_START[0] - lx / 2, T.LEVEL_START[1] - ly / 2)
    level_hi = (T.LEVEL_START[0] + lx / 2, T.LEVEL_START[1] + ly / 2)
    d_right = nearest(station.BASES["right"][:2], level_lo, level_hi)
    d_left = nearest(station.BASES["left"][:2], T.SLOT_LO, T.SLOT_HI)
    print(f"        an arm's farthest horizontal reach above the table: {r * 100:.1f} cm")
    check(d_right > r + 0.02, f"the level on its tray is {d_right * 100:.1f} cm from the right arm's base")
    check(d_left > r + 0.02, f"the slot is {d_left * 100:.1f} cm from the left arm's base")

    print("3. occlusion (share of a level lying in the slot hidden from the top camera)")
    gap = (T.SLOT[1] - ly) / 2
    for variant in ("calibrated", "demo"):
        cam = station.top_camera(variant).pos
        f = [hidden_fraction(T, cam, off) for off in (-gap, 0.0, gap)]
        print(f"        {variant:10s} top camera: against the outer wall {f[0]:.0%}, centred {f[1]:.0%}, "
              f"against the inner wall {f[2]:.0%}")
        if variant == "calibrated":
            check(min(f) >= 0.25, "the walls hide at least a quarter of the level wherever it lies in the slot")

    print("4./5. demo replay")
    scores, events, min_gap, worst_mm, final = replay_demo(T)
    for ev in events:
        print(f"        t={ev[0]:5.2f} s  {ev[1]}")
    check(worst_mm < 3.0, f"worst IK position error along both plans: {worst_mm:.2f} mm")
    check(min_gap > 0.12, f"closest the two tool points come: {min_gap * 100:.1f} cm")
    firsts = {}
    for t, s, ok in scores:
        firsts.setdefault(s, t)
    print("        first time at each score: " + ", ".join(f"{s:.2f} at {t:.2f} s" for s, t in sorted(firsts.items())))
    check([s for s in sorted(firsts) if s > 0] == [0.25, 0.5, 0.75, 1.0], "scores rise 0.25, 0.5, 0.75, 1.0")
    held = sum(1 for _, _, ok in scores if ok)
    check(held / HZ >= T.SUCCESS_HOLD_S, f"success holds {held / HZ:.1f} s (needs {T.SUCCESS_HOLD_S} s)")
    t_end = len(scores) / HZ
    check(t_end * HZ <= 2400, f"demo length {t_end:.1f} s = {int(t_end * HZ)} steps (run limit 2,400)")

    print("5b. negative controls")
    T.new_trial()
    objs = fixed_states(T)
    objs["spirit_level"] = _state(T.LEVEL_START, np.eye(3), T.LEVEL_SIZE)
    check(T.score(objs) == 0.0, "level left on its tray: 0")
    T.new_trial()
    on_tray_lifted = np.array(T.LEVEL_START) + (0, 0, 0.05)
    objs["spirit_level"] = _state(on_tray_lifted, np.eye(3), T.LEVEL_SIZE)
    T.score(objs)
    objs["spirit_level"] = _state((0.3, 0.75, T.TOP - 0.2), np.eye(3), T.LEVEL_SIZE)
    check(T.score(objs) == 0.0, "lifted, then dropped off the table: 0")
    T.new_trial()
    slid = (T.LEVEL_X, T.CADDY_XY[1], T.FLOOR_TOP + T.LEVEL_SIZE[2] / 2)
    objs["spirit_level"] = _state(slid, np.eye(3), T.LEVEL_SIZE)
    check(T.score(objs) == 0.0 and not T.success(objs), "in the slot without having been lifted and carried: 0")
    T.new_trial()
    for p in (on_tray_lifted, (T.LEVEL_X, -0.35, T.TOP + 0.15)):
        objs["spirit_level"] = _state(p, np.eye(3), T.LEVEL_SIZE)
        T.score(objs)
    check(T.score(objs) == 0.5, "lifted and carried across, not in the slot: 0.5")
    tilt = np.array([[math.cos(1.2), 0, math.sin(1.2)], [0, 1, 0], [-math.sin(1.2), 0, math.cos(1.2)]])
    objs["spirit_level"] = _state((T.LEVEL_X, T.CADDY_XY[1], T.FLOOR_TOP + 0.14), tilt, T.LEVEL_SIZE)
    s, ok = T.score(objs), T.success(objs)
    check(s == 0.75 and not ok, f"stuck in the slot sticking out above the rim: {s}")
    T.new_trial(offline_tcps=lambda: [np.array((0.12, 0.2, 1.0)),
                                      np.array((T.RIGHT_X, T.CADDY_XY[1], T.FLOOR_TOP + 0.045))])
    for p in (on_tray_lifted, (T.LEVEL_X, -0.35, T.TOP + 0.15), slid):
        objs["spirit_level"] = _state(p, np.eye(3), T.LEVEL_SIZE)
        s = T.score(objs)
    check(s == 0.75 and not T.success(objs), "in the slot but still held: 0.75, not success")

    print("\n" + ("ALL CHECKS PASSED" if not FAILS else f"{len(FAILS)} CHECK(S) FAILED: " + "; ".join(FAILS)))
    return 1 if FAILS else 0


if __name__ == "__main__":
    sys.exit(main())
