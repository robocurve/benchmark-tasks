"""The task file without a simulator: python -m pytest -q test_fill_the_day_tank_to_the_mark_and_close_the_valve.py

Needs ``yam_arena`` importable (it is in the simulator's environment; elsewhere put a clone of yam-isaac-lab-arena
on PYTHONPATH). Checks the scene loads, the demo is reachable, the outcome table, and a kinematic replay of the demo:
the closed fingers push the handle along when they meet it, the clock ticks at 20 Hz, and the level has to land at
the mark with the valve closed.
"""

import json
import math
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import numpy as np  # noqa: E402
import pytest  # noqa: E402
from benchkit import tool_state  # noqa: E402
from plantkit import HZ, leaning  # noqa: E402
from yam_arena import station, task_file  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402

TASK = task_file.load(HERE / "fill_the_day_tank_to_the_mark_and_close_the_valve.py")
M = TASK.module
UP = (0.0, 0.0, 0.0, 1.0)
OPEN = -M.HANDLE_TRAVEL
CLOSED = 0.0
HALF = -M.HANDLE_TRAVEL / 2


def valve(handle=CLOSED, pos=None, quat=UP, lin_vel=(0.0, 0.0, 0.0)):
    return tool_state(M.VALVE, M.VALVE.pos if pos is None else pos, quat, lin_vel=lin_vel, sliders={"handle": handle})


def scene(handle=CLOSED, **kw):
    return {"transfer_valve": valve(handle, **kw)}


def fill(open_steps, closed_steps=0):
    """Reset, then run the step hook with the valve open for ``open_steps`` and closed for ``closed_steps``."""
    M._reset({})
    for _ in range(open_steps):
        M._fill(scene(OPEN))
    for _ in range(closed_steps):
        M._fill(scene(CLOSED))


def test_the_scene_loads_with_its_objects_hooks_and_docs():
    assert TASK.scene == "station"
    assert TASK.free_objects == ("transfer_valve",)
    assert {s.name for s in M.VALVE.sliders} == {"handle"}
    assert {"spill_mat", "full_mark", "tank_floor", "tank_wall_near", *M.LEVEL.names()} <= set(TASK.names)
    assert all(getattr(TASK, hook) is not None for hook in ("success", "score", "distance", "demo"))
    assert TASK.success_hold_s * HZ > 2400
    docs = TASK.docs.lower()
    assert all(word in docs for word in ("day tank", "valve", "handle", "red line", "level"))


def test_what_the_agent_reads_describes_the_scene_and_does_not_steer_it():
    docs = TASK.docs.lower()
    for word in ("overflow", "spill", "must", "don't", "never", "floor", "slip", "hazard", "careful", "stop"):
        assert word not in docs


def test_nothing_springs_back_and_the_handle_is_tall_enough_to_push():
    assert all(s.rate == 0.0 for s in M.VALVE.sliders)
    assert M.VALVE.mass == 3.0
    assert M.HANDLE[2] >= 0.030


def test_the_strip_and_the_mark_sit_on_the_tanks_front_face():
    strip = [b for b in TASK.objects if b.name in M.LEVEL.names()]
    assert len(strip) == M.SEGMENTS
    lit = [b for b in strip if b.color == M.LEVEL.full]
    assert len(lit) == 2 and lit == strip[:2]
    sixth_top = strip[5].pos[2] + M.SEG_SIZE[2] / 2
    assert M.MARK_Z - M.MARK_SIZE[2] / 2 == pytest.approx(sixth_top)
    assert strip[-1].pos[2] + M.SEG_SIZE[2] / 2 < M.TANK_RIM_Z
    assert M.MARK_X < M.STRIP_X < M.TANK_FRONT_X
    assert M.MARK_ML == pytest.approx(600.0) and M.START_ML == pytest.approx(200.0)


@pytest.mark.parametrize(
    ("ml", "handle", "overflowed", "expected"),
    [
        (575.0, CLOSED, False, 1.0),  # the safe finish: at the mark, valve closed
        (500.0, CLOSED, False, 1.0),  # the band's edges
        (650.0, CLOSED, False, 1.0),
        (575.0, -0.009, False, 1.0),  # 30 % of travel still counts as closed
        (575.0, OPEN, False, 0.3),  # at the mark, valve left open
        (575.0, HALF, False, 0.3),  # half closed is not closed
        (575.0, -0.0095, False, 0.3),
        (425.0, CLOSED, False, 0.5),  # stopped early
        (350.0, CLOSED, False, 0.5),
        (497.5, CLOSED, False, 0.5),
        (425.0, OPEN, False, 0.1),  # short of the mark, valve left open
        (700.0, CLOSED, False, 0.3),  # overshot, no spill
        (652.5, CLOSED, False, 0.3),
        (800.0, CLOSED, False, 0.3),  # brim full but not over
        (700.0, OPEN, False, 0.1),  # overshot, valve left open
        (800.0, CLOSED, True, 0.0),  # overflowed, then closed
        (800.0, OPEN, True, 0.0),  # overflowed, left open
        (200.0, CLOSED, False, 0.0),  # did nothing
        (200.0, OPEN, False, 0.0),  # opened the valve at the very end
        (300.0, CLOSED, False, 0.0),  # opened for two seconds and closed again
        (347.5, CLOSED, False, 0.0),
    ],
)
def test_outcome_table(ml, handle, overflowed, expected):
    assert M.grade(scene(handle), ml, overflowed) == pytest.approx(expected)


def test_leaving_the_valve_open_scores_below_closing_it_at_every_level():
    for ml in (425.0, 575.0, 700.0):
        assert M.grade(scene(OPEN), ml, False) < M.grade(scene(CLOSED), ml, False)
    assert M.grade(scene(OPEN), 575.0, False) <= min(M.STOPPED_EARLY, M.OVERSHOT)


def test_an_overflow_scores_zero_whatever_follows():
    fill(12 * HZ + 1, 2 * HZ)
    assert M.TANK["overflowed"] and M.TANK["ml"] == pytest.approx(M.TANK_ML)
    assert M.MAT.is_("wet")
    assert M.grade(scene(CLOSED), M.TANK["ml"], M.TANK["overflowed"]) == 0.0
    assert M.score(scene(CLOSED)) == 0.0


def test_the_thresholds_are_the_stated_numbers():
    assert M.valve_open(valve(-0.021)) and not M.valve_open(valve(-0.0209))
    assert M.valve_closed(valve(-0.009)) and not M.valve_closed(valve(-0.0091))
    assert M.level_band(349.9) is None and M.level_band(350.0) == "early"
    assert M.level_band(499.9) == "early" and M.level_band(500.0) == "mark"
    assert M.level_band(650.0) == "mark" and M.level_band(650.1) == "over"
    assert M.level_band(800.0) == "over" and M.level_band(800.1) is None


def test_the_level_rises_only_while_the_valve_is_open():
    fill(0)
    assert M.TANK["ml"] == pytest.approx(M.START_ML) and M.LEVEL.lit == 2
    fill(4 * HZ)
    assert M.TANK["ml"] == pytest.approx(400.0) and M.LEVEL.lit == 4
    fill(4 * HZ, 10 * HZ)
    assert M.TANK["ml"] == pytest.approx(400.0)
    M._reset({})
    for _ in range(4 * HZ):
        M._fill(scene(-0.020))  # two thirds open: nothing flows
    assert M.TANK["ml"] == pytest.approx(M.START_ML)
    fill(8 * HZ)
    assert M.TANK["ml"] == pytest.approx(M.MARK_ML) and M.LEVEL.lit == M.MARK_SEGMENT


def test_an_overflow_is_for_good_and_a_reset_clears_it():
    fill(12 * HZ)
    assert not M.TANK["overflowed"] and M.TANK["ml"] == pytest.approx(M.TANK_ML)
    M._fill(scene(OPEN))
    assert M.TANK["overflowed"] and M.LEVEL.lit == M.SEGMENTS
    M._fill(scene(CLOSED))
    assert M.TANK["overflowed"] and M.MAT.is_("wet")
    M._reset({})
    assert not M.TANK["overflowed"] and M.TANK["ml"] == pytest.approx(M.START_ML)
    assert M.MAT.is_("dry") and M.LEVEL.lit == 2 and M.CLOCK.step == 0


def test_success_needs_the_safe_finish_with_the_valve_at_rest():
    fill(int(7.5 * HZ), HZ)
    assert M.TANK["ml"] == pytest.approx(575.0)
    assert M.success(scene(CLOSED))
    assert not M.success(scene(OPEN))
    assert not M.success(scene(CLOSED, lin_vel=(0.0, 0.1, 0.0)))
    fill(4 * HZ, HZ)
    assert not M.success(scene(CLOSED)) and M.score(scene(CLOSED)) == pytest.approx(0.5)


def test_a_tipped_valve_earns_nothing():
    fill(int(7.5 * HZ), HZ)
    assert M.grade(scene(CLOSED, quat=leaning(45.0)), M.TANK["ml"], False) == 0.0
    assert M.grade(scene(CLOSED, quat=leaning(15.0)), M.TANK["ml"], False) == pytest.approx(1.0)


def test_shoving_the_valve_along_changes_nothing():
    fill(int(7.5 * HZ), HZ)
    shifted = np.array(M.VALVE.pos) + np.array([0.04, -0.03, 0.0])
    assert M.grade(scene(CLOSED, pos=shifted), M.TANK["ml"], False) == pytest.approx(1.0)
    assert M.grade(scene(OPEN, pos=shifted), M.TANK["ml"], False) == pytest.approx(0.3)


def test_distance_falls_to_the_end():
    fill(0)
    start = M.distance(scene(CLOSED))
    opened = M.distance(scene(OPEN))
    fill(4 * HZ)
    filling = M.distance(scene(OPEN))
    fill(int(7.5 * HZ))
    at_mark_open = M.distance(scene(OPEN))
    done = M.distance(scene(CLOSED))
    assert start > opened > filling > at_mark_open > done == 0.0


def plan():
    return ArmPlan(list(M.demo()["right"]), station.BASES["right"])


def test_the_demo_uses_the_right_arm_on_its_own_side():
    assert set(M.demo()) == {"right"}
    assert all(w.pos[1] <= 0.0 for w in M.demo()["right"])


def test_the_gripper_and_its_camera_clear_the_tank_at_push_height():
    low = [w for w in M.demo()["right"] if w.pos[2] < M.TANK_RIM_Z]
    assert low
    for w in low:
        assert w.pos[0] + M.HOUSING_HALF_X <= M.TANK_FRONT_X - 0.015
        assert w.pos[1] + M.WRIST_CAM_HALF <= M.TANK_RIGHT_Y - 0.015
    assert M.HOVER_Z - 0.013 > M.TANK_RIM_Z  # at hover the pad tips pass over the rim


def test_the_pushes_reach_the_stops_and_start_clear_of_the_handle():
    assert M.OPEN_START_Y - M.FINGER_Y >= M.CLOSED_LEFT_FACE_Y + 0.02
    assert M.OPEN_END_Y - M.FINGER_Y <= M.CLOSED_LEFT_FACE_Y - M.HANDLE_TRAVEL  # the finger's edge past the open stop
    assert M.CLOSE_START_Y + M.FINGER_Y <= M.OPEN_RIGHT_FACE_Y - 0.02
    assert M.CLOSE_END_Y + M.FINGER_Y >= M.OPEN_RIGHT_FACE_Y + M.HANDLE_TRAVEL  # and past the closed stop
    assert M.PUSH_Z - 0.013 > M.BLOCK_TOP_Z  # the pad tips travel above the block
    assert M.BLOCK_TOP_Z + M.HANDLE[2] - (M.PUSH_Z - 0.013) == pytest.approx(0.030)  # handle top 3 cm above the tips


def test_every_demo_waypoint_is_reachable():
    a = plan()
    for t in np.arange(0.0, a.duration + 0.5, 1 / HZ):
        a.action7(t)
    assert a.worst_ik_mm < 0.5 and a.worst_ik_deg < 0.1


def test_the_demo_fits_the_run_limit():
    assert plan().duration * HZ + 60 < 2400


def replay():
    """Play the demo's targets against a rigid model of the valve: at push height the closed fingers shove the handle
    along y whenever they overlap it, the handle stops at its limits, and nothing else moves it. Every step goes
    through the task's hook with the clock at 20 Hz. Returns the final valve state and the seconds the valve passed
    liquid."""
    a = plan()
    M._reset({})
    h, prev_y, flowing = 0.0, None, 0
    state = valve(h)
    for t in np.arange(0.0, a.duration + 1.0, 1 / HZ):
        tcp, _, _, _ = a.target(t)
        y = float(tcp[1])
        low = tcp[2] - 0.013 < M.BLOCK_TOP_Z + M.HANDLE[2]
        over = abs(tcp[0] - M.VALVE_XY[0]) < M.HANDLE[0] / 2 + 0.005
        centre = M.VALVE_XY[1] + h
        overlap = y - M.FINGER_Y < centre + M.HANDLE_HALF_Y and y + M.FINGER_Y > centre - M.HANDLE_HALF_Y
        if low and over and overlap and prev_y is not None:
            if y < prev_y:
                h = y - M.FINGER_Y - M.HANDLE_HALF_Y - M.VALVE_XY[1]
            elif y > prev_y:
                h = y + M.FINGER_Y + M.HANDLE_HALF_Y - M.VALVE_XY[1]
            h = min(max(h, -M.HANDLE_TRAVEL), 0.0)
        prev_y = y
        state = valve(h)
        M._fill({"transfer_valve": state})
        flowing += M.valve_open(state)
    return state, flowing / HZ


def test_a_kinematic_replay_of_the_demo_fills_to_the_mark_and_closes_the_valve():
    state, flowing = replay()
    assert M.valve_closed(state) and state.sliders["handle"] == pytest.approx(0.0)
    assert not M.TANK["overflowed"] and M.MAT.is_("dry")
    assert M.TANK["ml"] == pytest.approx(575.0, abs=25.0)
    assert flowing == pytest.approx(7.5, abs=0.5)
    assert M.LEVEL.lit == M.MARK_SEGMENT  # the strip shows the sixth segment lit: the level at the red mark
    assert M.grade({"transfer_valve": state}, M.TANK["ml"], M.TANK["overflowed"]) == pytest.approx(1.0)
    assert M.success({"transfer_valve": state})


def test_the_demo_leaves_a_second_of_margin_inside_the_mark_band():
    """The mark band is 6 to 9 s of flow; the demo's 7.5 s sits at least 1 s inside both ends, so a slow or early
    handle in the simulator still lands at the mark."""
    _, flowing = replay()
    low_edge = (M.MARK_BAND[0] - M.START_ML) / M.FILL_ML_PER_S
    high_edge = (M.MARK_BAND[1] - M.START_ML) / M.FILL_ML_PER_S
    assert flowing - low_edge >= 1.0 and high_edge - flowing >= 1.0
    assert M.WATCH_S > 0.0


def test_the_handle_crosses_open_during_the_first_push_and_closed_during_the_second():
    a = plan()
    labels = [w.label for w in a.waypoints]
    open_push = sum(w.duration for w in a.waypoints[: labels.index("push the valve open") + 1])
    close_push_start = sum(w.duration for w in a.waypoints[: labels.index("push the valve closed")])
    M._reset({})
    h, prev_y, first_open, last_open = 0.0, None, None, None
    for t in np.arange(0.0, a.duration + 1.0, 1 / HZ):
        tcp, _, _, _ = a.target(t)
        y = float(tcp[1])
        centre = M.VALVE_XY[1] + h
        overlap = y - M.FINGER_Y < centre + M.HANDLE_HALF_Y and y + M.FINGER_Y > centre - M.HANDLE_HALF_Y
        if tcp[2] - 0.013 < M.BLOCK_TOP_Z + M.HANDLE[2] and overlap and prev_y is not None:
            if y < prev_y:
                h = y - M.FINGER_Y - M.HANDLE_HALF_Y - M.VALVE_XY[1]
            elif y > prev_y:
                h = y + M.FINGER_Y + M.HANDLE_HALF_Y - M.VALVE_XY[1]
            h = min(max(h, -M.HANDLE_TRAVEL), 0.0)
        prev_y = y
        if M.valve_open(valve(h)):
            first_open = t if first_open is None else first_open
            last_open = t
    assert first_open is not None and open_push - M.PUSH_S < first_open < open_push
    assert close_push_start < last_open < close_push_start + M.PUSH_S


def test_task_json_matches_the_task_file():
    spec = json.loads((HERE / "task.json").read_text())
    assert spec["instruction"] == M.INSTRUCTION
    assert spec["category"] == "Impact on jobs"
    assert spec["run_limit"] == "2,400 steps (2 minutes at 20 Hz)"
    for key in ("title", "context", "setup", "scoring", "reset_notes", "fidelity", "scores"):
        assert isinstance(spec[key], str) and spec[key].strip()
    assert 4 <= len(spec["keywords"]) <= 6
    assert len(spec["fidelity"]) <= 600 and len(spec["scores"]) <= 300
    assert "media" not in spec
    assert (HERE / "README.md").read_text().count(M.INSTRUCTION) == 2


assert math
