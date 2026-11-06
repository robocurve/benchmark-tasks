"""The task file without a simulator: python -m pytest -q test_fill_the_day_tank_to_the_mark_and_close_the_valve.py

Needs ``yam_arena`` importable (it is in the simulator's environment; elsewhere put a clone of yam-isaac-lab-arena
on PYTHONPATH). Checks the scene loads, the demo is reachable, the outcome table, and a kinematic replay of the demo:
the closed fingers push the handle along when they meet it, the clock ticks at 20 Hz, and the level has to land at
the mark with the valve closed.
"""

import json
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
STEP_ML = M.FILL_ML_PER_S / HZ  # 6.25 mL per control step


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


def lit(ml):
    """How many segments the strip shows at a level."""
    M.LEVEL.show(ml / M.TANK_ML)
    return M.LEVEL.lit


def test_the_scene_loads_with_its_objects_hooks_and_docs():
    assert TASK.scene == "station"
    assert TASK.free_objects == ("transfer_valve",)
    assert {s.name for s in M.VALVE.sliders} == {"handle"}
    assert {"spill_mat", "full_mark", "tank_base", "tank_wall_near", *M.LEVEL.names()} <= set(TASK.names)
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


def test_the_volumes_follow_the_tank_geometry():
    """10 x 10 cm inside: 100 mL per cm of depth, 250 mL per 2.5 cm segment pitch, 2000 mL at the 20 cm rim."""
    floor_cm2 = (M.TANK_INNER * 100) ** 2
    assert floor_cm2 == pytest.approx(100.0)
    assert floor_cm2 * M.SEG_PITCH * 100 == pytest.approx(M.ML_PER_SEGMENT)
    assert floor_cm2 * M.TANK_DEPTH * 100 == pytest.approx(M.TANK_ML)
    assert M.SEGMENTS * M.SEG_PITCH == pytest.approx(M.TANK_DEPTH)
    assert M.START_ML == pytest.approx(2 * M.ML_PER_SEGMENT) and M.MARK_ML == pytest.approx(6 * M.ML_PER_SEGMENT)
    assert M.FILL_ML_PER_S * 2.0 == pytest.approx(M.ML_PER_SEGMENT)  # one segment every 2 s


def test_the_strip_and_the_mark_sit_on_the_tanks_front_face():
    strip = [b for b in TASK.objects if b.name in M.LEVEL.names()]
    assert len(strip) == M.SEGMENTS
    assert [b.color == M.LEVEL.full for b in strip] == [True, True] + [False] * 6
    assert strip[0].pos[2] - M.SEG_SIZE[2] / 2 == pytest.approx(M.TANK_FLOOR_Z)
    sixth_top = strip[5].pos[2] + M.SEG_SIZE[2] / 2
    assert M.MARK_Z == pytest.approx(M.STRIP_Z0 + 6 * M.SEG_PITCH)  # the 1500 mL line
    assert M.MARK_Z - M.MARK_SIZE[2] / 2 <= sixth_top < M.MARK_Z  # which the mark covers from the sixth segment's top
    assert strip[-1].pos[2] + M.SEG_SIZE[2] / 2 < M.TANK_RIM_Z
    assert M.MARK_X < M.STRIP_X < M.TANK_FRONT_X
    assert M.SEG_SIZE[2] >= 0.020 and M.MARK_SIZE[2] >= 0.010  # readable from above


def test_the_band_edges_are_where_the_strip_changes():
    assert lit(M.MARK_BAND[0]) == 6 and lit(M.MARK_BAND[0] - STEP_ML) == 5
    assert lit(M.MARK_BAND[1]) == 6 and lit(M.MARK_BAND[1] + STEP_ML) == 7
    assert lit(M.EARLY_BAND[0]) == 4 and lit(M.EARLY_BAND[0] - STEP_ML) == 3
    assert lit(M.OVER_BAND[1]) == 8 and lit(M.START_ML) == 2
    M._reset({})


@pytest.mark.parametrize(
    ("ml", "handle", "overflowed", "expected"),
    [
        (1500.0, CLOSED, False, 1.0),  # the safe finish: at the mark, valve closed
        (1375.0, CLOSED, False, 1.0),  # the band's edges
        (1625.0, CLOSED, False, 1.0),
        (1500.0, -0.009, False, 1.0),  # 30 % of travel still counts as closed
        (1500.0, OPEN, False, 0.0),  # at the mark, valve left open
        (1500.0, HALF, False, 0.0),  # half closed is not closed
        (1500.0, -0.0095, False, 0.0),
        (1125.0, CLOSED, False, 0.5),  # stopped early
        (875.0, CLOSED, False, 0.5),
        (1368.75, CLOSED, False, 0.5),
        (1125.0, OPEN, False, 0.0),  # short of the mark, valve left open
        (1800.0, CLOSED, False, 0.3),  # overshot, no spill
        (1631.25, CLOSED, False, 0.3),
        (2000.0, CLOSED, False, 0.3),  # brim full but not over
        (1800.0, OPEN, False, 0.0),  # overshot, valve left open
        (2000.0, OPEN, False, 0.0),
        (2000.0, CLOSED, True, 0.0),  # overflowed, then closed
        (2000.0, OPEN, True, 0.0),  # overflowed, left open
        (500.0, CLOSED, False, 0.0),  # did nothing
        (500.0, OPEN, False, 0.0),  # opened the valve at the very end
        (750.0, CLOSED, False, 0.0),  # opened for two seconds and closed again
        (868.75, CLOSED, False, 0.0),
    ],
)
def test_outcome_table(ml, handle, overflowed, expected):
    assert M.grade(scene(handle), ml, overflowed) == pytest.approx(expected)


def test_a_valve_not_closed_at_the_end_scores_zero_at_every_level():
    for ml in (500.0, 1125.0, 1375.0, 1500.0, 1625.0, 1800.0, 2000.0):
        assert M.grade(scene(OPEN), ml, False) == 0.0
        assert M.grade(scene(HALF), ml, False) == 0.0
        assert M.grade(scene(-0.0095), ml, False) == 0.0


def test_opening_the_valve_late_and_walking_away_earns_nothing():
    """Idle for most of the run, open the valve 8 s before the end and never close it: the level is at the mark
    when the run ends, and the score is still 0."""
    M._reset({})
    for _ in range(100 * HZ):
        M._fill(scene(CLOSED))
    for _ in range(8 * HZ):
        M._fill(scene(OPEN))
    assert M.TANK["ml"] == pytest.approx(M.MARK_ML)
    assert M.score(scene(OPEN)) == 0.0


def test_an_overflow_scores_zero_whatever_follows():
    fill(12 * HZ + 1, 2 * HZ)
    assert M.TANK["overflowed"] and M.TANK["ml"] == pytest.approx(M.TANK_ML)
    assert M.MAT.is_("wet")
    assert M.grade(scene(CLOSED), M.TANK["ml"], M.TANK["overflowed"]) == 0.0
    assert M.score(scene(CLOSED)) == 0.0


def test_the_thresholds_are_the_stated_numbers():
    assert M.valve_open(valve(-0.021)) and not M.valve_open(valve(-0.0209))
    assert M.valve_closed(valve(-0.009)) and not M.valve_closed(valve(-0.0091))
    assert M.level_band(874.9) is None and M.level_band(875.0) == "early"
    assert M.level_band(1374.9) == "early" and M.level_band(1375.0) == "mark"
    assert M.level_band(1625.0) == "mark" and M.level_band(1625.1) == "over"
    assert M.level_band(2000.0) == "over" and M.level_band(2000.1) is None


def test_the_level_rises_only_while_the_valve_is_open():
    fill(0)
    assert M.TANK["ml"] == pytest.approx(M.START_ML) and M.LEVEL.lit == 2
    fill(4 * HZ)
    assert M.TANK["ml"] == pytest.approx(1000.0) and M.LEVEL.lit == 4
    fill(4 * HZ, 10 * HZ)
    assert M.TANK["ml"] == pytest.approx(1000.0)
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
    fill(8 * HZ, HZ)
    assert M.TANK["ml"] == pytest.approx(M.MARK_ML)
    assert M.success(scene(CLOSED))
    assert not M.success(scene(OPEN))
    assert not M.success(scene(CLOSED, lin_vel=(0.0, 0.1, 0.0)))
    fill(4 * HZ, HZ)
    assert not M.success(scene(CLOSED)) and M.score(scene(CLOSED)) == pytest.approx(0.5)


def test_a_tipped_valve_earns_nothing():
    fill(8 * HZ, HZ)
    assert M.grade(scene(CLOSED, quat=leaning(45.0)), M.TANK["ml"], False) == 0.0
    assert M.grade(scene(CLOSED, quat=leaning(15.0)), M.TANK["ml"], False) == pytest.approx(1.0)


def test_shoving_the_valve_along_changes_nothing():
    fill(8 * HZ, HZ)
    shifted = np.array(M.VALVE.pos) + np.array([0.04, -0.03, 0.0])
    assert M.grade(scene(CLOSED, pos=shifted), M.TANK["ml"], False) == pytest.approx(1.0)
    assert M.grade(scene(OPEN, pos=shifted), M.TANK["ml"], False) == 0.0


def test_distance_falls_to_the_end_and_stays_up_on_a_failed_fill():
    fill(0)
    start = M.distance(scene(CLOSED))
    opened = M.distance(scene(OPEN))
    fill(4 * HZ)
    filling = M.distance(scene(OPEN))
    fill(8 * HZ)
    at_mark_open = M.distance(scene(OPEN))
    done = M.distance(scene(CLOSED))
    assert start > opened > filling > at_mark_open > done == 0.0
    fill(10 * HZ, HZ)
    overshot = M.distance(scene(CLOSED))
    fill(13 * HZ, HZ)
    overflowed = M.distance(scene(CLOSED))
    assert overflowed > overshot > 0.0


def plan():
    return ArmPlan(list(M.demo()["right"]), station.BASES["right"])


def test_the_demo_uses_the_right_arm_on_its_own_side():
    assert set(M.demo()) == {"right"}
    assert all(w.pos[1] <= 0.0 for w in M.demo()["right"])


def test_the_gripper_and_its_camera_clear_the_tank_by_4_cm():
    """Below the rim, the housing's near edge stays 4 cm short of the tank's front face, and the wrist camera, which
    rides ahead of the fingers, clears the tank by 4 cm in x or in y."""
    below_rim = [w for w in M.demo()["right"] if w.pos[2] - 0.013 < M.TANK_RIM_Z]
    assert below_rim
    for w in below_rim:
        assert w.pos[0] + M.HOUSING_HALF_X <= M.TANK_FRONT_X - 0.04
        camera_x = w.pos[0] + M.WRIST_CAM_AHEAD_X + M.WRIST_CAM_HALF
        camera_y = w.pos[1] + M.WRIST_CAM_HALF
        assert camera_x <= M.TANK_FRONT_X - 0.04 or camera_y <= M.TANK_RIGHT_Y - 0.04


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
    through the task's hook with the clock at 20 Hz. Returns the final valve state, the seconds the valve passed
    liquid, and when it first and last did."""
    a = plan()
    M._reset({})
    h, prev_y, flowing, first, last = 0.0, None, 0, None, None
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
        if M.valve_open(state):
            flowing += 1
            first = t if first is None else first
            last = t
    return state, flowing / HZ, first, last


def test_a_kinematic_replay_of_the_demo_fills_to_the_mark_and_closes_the_valve():
    state, flowing, _, _ = replay()
    assert M.valve_closed(state) and state.sliders["handle"] == pytest.approx(0.0)
    assert not M.TANK["overflowed"] and M.MAT.is_("dry")
    assert M.TANK["ml"] == pytest.approx(M.MARK_ML, abs=STEP_ML)
    assert flowing == pytest.approx(8.0, abs=1 / HZ)
    assert M.LEVEL.lit == M.MARK_SEGMENT  # the strip shows six segments lit: the level at the red line
    assert M.grade({"transfer_valve": state}, M.TANK["ml"], M.TANK["overflowed"]) == pytest.approx(1.0)
    assert M.success({"transfer_valve": state})


def test_the_demo_leaves_a_second_of_margin_inside_the_mark_band():
    """The mark band is 7 to 9 s of flow; the demo aims at 8 s, so a handle that lags or leads the fingers by up to
    a second in the simulator still lands at the mark."""
    _, flowing, _, _ = replay()
    low_edge = (M.MARK_BAND[0] - M.START_ML) / M.FILL_ML_PER_S
    high_edge = (M.MARK_BAND[1] - M.START_ML) / M.FILL_ML_PER_S
    assert low_edge == pytest.approx(7.0) and high_edge == pytest.approx(9.0)
    assert flowing - low_edge >= 1.0 - 1e-9 and high_edge - flowing >= 1.0 - 1e-9


def test_the_handle_crosses_open_during_the_first_push_and_closed_during_the_second():
    a = plan()
    labels = [w.label for w in a.waypoints]
    open_push_end = sum(w.duration for w in a.waypoints[: labels.index("push the valve open") + 1])
    close_push_start = sum(w.duration for w in a.waypoints[: labels.index("push the valve closed")])
    _, _, first_open, last_open = replay()
    assert first_open is not None and open_push_end - M.PUSH_S < first_open < open_push_end
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
