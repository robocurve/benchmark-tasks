"""Cheap strategies for task.py: the same scene, scorer and hooks (loaded from task.py), only demo() differs.

    GAUGE_SORT_VARIANT=no_checks inspect-robots run --embodiment yam_arena --policy yam_scripted -P hold_s=2 \\
      -E task=variants.py -E object_jitter=0 -E control_hz=20 --scorer task_score --max-steps 9600 ...

- nothing: both arms go to their park pose and stay there for the whole run.
- no_checks: every block straight to its correct tray, no gauge checks.
- all_green: every block into the green tray (stacked two high), no gauge checks.
- held_checks: every block set on GO (and NO-GO where needed) but held there with closed fingers, never let go;
  then to its correct tray.
- clearance: a calibration probe, not a strategy: the gaps and block widths below replace the task's, each block is
  set on its gap three times (A, B, C on GO; D, E, F on NO-GO), then laid in the red tray.
"""

import importlib.util
import os
import pathlib

_spec = importlib.util.spec_from_file_location("gauge_sort_task", pathlib.Path(__file__).with_name("task.py"))
T = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(T)
globals().update({k: v for k, v in vars(T).items() if not k.startswith("__")})  # OBJECTS, score(), hooks
__doc__ = T.__doc__  # agents read the scene description, not this one

VARIANT = os.environ.get("GAUGE_SORT_VARIANT", "nothing")
PROBE_GAPS_MM = {"go": 41.0, "nogo": 39.0}
# drop in with 0.75, 0.75 and 0.6 mm a side on each gap
PROBE_WIDTHS_MM = {"a": 39.5, "b": 39.5, "c": 39.8, "d": 37.5, "e": 37.5, "f": 37.8}
PROBE_GAP = {"a": "go", "b": "go", "c": "go", "d": "nogo", "e": "nogo", "f": "nogo"}
PROBE_REPEATS = 3

if VARIANT == "clearance":
    from yam_arena.task_file import Box

    T.GAPS.update({g: v / 1000 for g, v in PROBE_GAPS_MM.items()})
    T.WIDTHS.update({k: v / 1000 for k, v in PROBE_WIDTHS_MM.items()})
    T.BLOCKS.update({f"block_{k}": w for k, w in T.WIDTHS.items()})
    OBJECTS = [
        *(Box(n, pos=(*T.START[n], T.TOP + T.BLOCK_H / 2), size=(T.BLOCK_LEN, w, T.BLOCK_H), mass=T.block_mass(w),
              color=T.ALUMINIUM) for n, w in T.BLOCKS.items()),
        *T._gauge(),
        *(o for o in T.OBJECTS if o.name.startswith(("green_", "red_"))),
    ]


def _to_tray(arm, name, tray, slot, level=0):
    to = (T.TRAY_X, T.TRAY_Y[tray] + T.TRAY_SLOTS_Y[slot])
    z = T.TRAY_FLOOR_TOP + level * T.BLOCK_H + T.RELEASE_GAP + T.GRASP_DZ
    return [
        T._wp(arm, T.MOVE_S, to, T.TRAVEL_Z, T.HOLD, f"carry {name} to {tray}"),
        T._wp(arm, T.DESCEND_S, to, z, T.HOLD, "lower"),
        T._wp(arm, T.GRIP_S, to, z, T.FINGER_OPEN, f"release {name} in {tray}"),
        T._wp(arm, T.LET_GO_S, to, z, T.FINGER_OPEN, "let go"),
        T._wp(arm, T.LIFT_S, to, T.TRAVEL_Z, T.FINGER_OPEN, "retreat"),
        T._wp(arm, T.MOVE_S, T.PARK[arm], T.TRAVEL_Z, T.FINGER_OPEN, "park"),
    ]


def _held_visit(arm, name, gap):
    visit = T._gauge_visit(arm, name, gap)
    upto = [w.label for w in visit].index(f"set on {gap}") + 1
    set_on = visit[upto - 1]
    return visit[:upto] + [
        T.Waypoint(2 * T.DWELL_S, set_on.pos, T.HOLD, f"dwell {gap}, still holding", set_on.yaw),
        visit[-1],  # lift
    ]


def _cycle(arm, name, i):
    good = T.in_tolerance(T.BLOCKS[name])
    if VARIANT == "clearance":
        gap = PROBE_GAP[name[-1]]
        visits = [w for _ in range(PROBE_REPEATS) for w in T._gauge_visit(arm, name, gap)]
        return T._fetch(arm, name) + visits + _to_tray(arm, name, "red", i % 3, i // 3)
    if VARIANT == "all_green":
        return T._fetch(arm, name) + _to_tray(arm, name, "green", i % 3, i // 3)
    visits = []
    if VARIANT == "held_checks":
        visits = _held_visit(arm, name, "go")
        if T.drops_in(T.BLOCKS[name], "go"):
            visits += _held_visit(arm, name, "nogo")
    return T._fetch(arm, name) + visits + _to_tray(arm, name, "green" if good else "red", _cycle.used[good])


def demo():
    if VARIANT == "nothing":
        hold = T.RUN_LIMIT_STEPS / T.CONTROL_HZ
        return {arm: [T._wp(arm, hold, T.PARK[arm], T.TRAVEL_Z, T.FINGER_OPEN, "park and stay")] for arm in T.PARK}
    plans = {arm: [T._wp(arm, T.MOVE_S, T.PARK[arm], T.TRAVEL_Z, T.FINGER_OPEN, "park")] for arm in ("left", "right")}
    rows = {arm: sorted((n for n in T.BLOCKS if T.ARM_OF_ROW[int(T.np.sign(T.START[n][1]))] == arm),
                        key=lambda n: T.START[n][0]) for arm in plans}
    _cycle.used = {True: 0, False: 0}
    i = 0
    for k in range(3):
        for arm, idle in (("left", "right"), ("right", "left")):
            name = rows[arm][k]
            good = T.in_tolerance(T.BLOCKS[name])
            cycle = _cycle(arm, name, i)
            _cycle.used[good] += 1
            i += 1
            plans[arm] += cycle
            wait = sum(w.duration for w in cycle)
            plans[idle].append(T._wp(idle, wait, T.PARK[idle], T.TRAVEL_Z, T.FINGER_OPEN, "wait"))
    return plans
