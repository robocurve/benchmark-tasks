"""The task file without a simulator: python -m pytest -q test_move_a_plate_from_the_shelf_to_the_reader.py

Needs ``yam_arena`` importable (it is in the simulator's environment; elsewhere put a clone of yam-isaac-lab-arena
on PYTHONPATH). Checks the scene loads, the demo is reachable, and the outcome table.
"""

import json
import math
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import numpy as np  # noqa: E402
import pytest  # noqa: E402
from yam_arena import ik, station, task_file  # noqa: E402
from yam_arena.demo import ArmPlan, Waypoint  # noqa: E402

TASK = task_file.load(HERE / "move_a_plate_from_the_shelf_to_the_reader.py")
M = TASK.module

from benchkit import tool_state  # noqa: E402

UP = (0.0, 0.0, 0.0, 1.0)
HALF_TURN = (0.0, 0.0, 1.0, 0.0)  # about the vertical
QUARTER_TURN = (0.0, 0.0, 0.7071068, 0.7071068)
SHELF_Z_LID_DOWN = M.SHELF_TOP + M.LID[2] / 2
TABLE_LID_Z = M.TOP + M.LID[2] / 2
NEST_Z = M.FLOOR_Z + M.PLATE[2] / 2
NEST_XY = M.READER_XY


def tilted(degrees):
    half = math.radians(degrees) / 2
    return (math.sin(half), 0.0, 0.0, math.cos(half))


def reader(door=0.0, pos=None, quat=UP):
    """``door`` is how far it has been pulled shut. The joint itself runs negative."""
    return tool_state(M.READER, M.READER.pos if pos is None else pos, quat, sliders={"door": -door})


def plate(xy=M.PLATE_XY, z=M.PLATE_Z, quat=UP):
    return tool_state(M.SAMPLE_PLATE, (*xy, z), quat)


def lid(xy=M.PLATE_XY, z=M.LID_Z, quat=UP):
    return tool_state(M.SAMPLE_LID, (*xy, z), quat)


def start():
    return {"plate": plate(), "lid": lid(), "reader": reader()}


def loaded(turned=True, door=M.DOOR_TRAVEL, lid_state=None):
    return {"plate": plate(NEST_XY, NEST_Z, HALF_TURN if turned else UP),
            "lid": lid(M.LID_SET_XY, TABLE_LID_Z) if lid_state is None else lid_state,
            "reader": reader(door)}


def lid_on_it(xy=NEST_XY, base_z=NEST_Z):
    return lid(xy, base_z + M.LID_Z - M.PLATE_Z)


def test_the_scene_loads():
    assert set(TASK.free_objects) == {"plate", "lid", "reader"}
    assert "orange corner" in " ".join(TASK.docs.split())


def test_the_plate_is_the_standard_footprint():
    assert sorted(M.PLATE[:2]) == pytest.approx([0.0855, 0.1276])
    assert max(M.SAMPLE_PLATE.size[:2]) == pytest.approx(0.1276)


def test_the_grip_is_tolerant():
    open_gap = 0.094
    assert (open_gap - M.GRIP_W) / 2 == pytest.approx(0.012)  # plate's body, either side of the fingers
    assert M.LID[0] <= M.GRIP_W  # the same with the lid still on
    assert (open_gap - M.TAB[1]) / 2 >= 0.030  # the lid's tab
    pad_tips = M.SIDE_GRIP_Z - M.PAD_TIP_PAST_TCP
    assert pad_tips - (M.SHELF_TOP + M.SKIRT_H) >= 0.004  # they come down on the body, over the skirt


def test_loading_it_right_scores_one():
    assert M.grade(loaded(), False) == 1.0


def test_doing_nothing_scores_zero():
    assert M.grade(start(), False) == 0.0


def test_the_start_is_the_wrong_way_round_for_the_reader():
    objects = loaded(turned=False)
    assert M.plate_in(objects["plate"], objects["reader"])
    assert not M.corner_right(objects["plate"], objects["reader"])


def test_door_left_open():
    assert M.grade(loaded(door=0.0), False) == 0.7


def test_door_nearly_shut_counts_and_part_way_does_not():
    assert M.grade(loaded(door=M.DOOR_TRAVEL - 0.009), False) == 1.0
    assert M.grade(loaded(door=M.DOOR_TRAVEL - 0.011), False) == 0.7


def test_wrong_way_round():
    assert M.grade(loaded(turned=False, door=0.0), False) == 0.3


def test_wrong_way_round_and_shut_is_worse_than_left_open():
    assert M.grade(loaded(turned=False), False) == 0.2


def test_lid_left_on():
    assert M.grade(loaded(door=0.0, lid_state=lid_on_it()), False) == 0.2
    assert M.grade(loaded(turned=False, door=0.0, lid_state=lid_on_it()), False) == 0.1


def test_lid_taken_off_but_plate_left_on_the_shelf():
    objects = {"plate": plate(), "lid": lid(M.LID_SET_XY, TABLE_LID_Z), "reader": reader()}
    assert M.grade(objects, False) == 0.1


def test_lid_on_the_shelf_counts_as_off():
    objects = loaded(lid_state=lid((M.PLATE_XY[0], M.PLATE_XY[1] + 0.03), SHELF_Z_LID_DOWN))
    assert M.grade(objects, False) == 1.0


def test_lid_shoved_half_off_is_not_off():
    objects = {"plate": plate(), "lid": lid((M.PLATE_XY[0], M.PLATE_XY[1] - 0.06), M.PLATE_TOP + 0.02, tilted(12)),
               "reader": reader()}
    assert M.grade(objects, False) == 0.0


def test_door_shut_on_an_empty_reader_earns_nothing():
    assert M.grade({**start(), "reader": reader(M.DOOR_TRAVEL)}, False) == 0.0
    objects = {"plate": plate(), "lid": lid(M.LID_SET_XY, TABLE_LID_Z), "reader": reader(M.DOOR_TRAVEL)}
    assert M.grade(objects, False) == 0.1


def test_lid_put_in_the_reader_instead_of_the_plate():
    objects = {"plate": plate(), "lid": lid(NEST_XY, M.FLOOR_Z + M.LID[2] / 2), "reader": reader(M.DOOR_TRAVEL)}
    assert M.grade(objects, False) == 0.0


def test_lid_put_in_the_reader_beside_the_plate_is_not_off():
    objects = loaded(door=0.0, lid_state=lid((NEST_XY[0] - 0.06, NEST_XY[1]), M.FLOOR_Z + 0.03))
    assert M.grade(objects, False) == 0.2


def test_lid_propped_on_the_table_against_the_reader_is_not_off():
    x = M.READER_XY[0] + M.FRONT_END - M.LID[0] / 2 + 0.005
    objects = loaded(lid_state=lid((x, M.READER_XY[1]), TABLE_LID_Z))
    assert M.grade(objects, False) == 0.2
    objects = loaded(lid_state=lid((x - 0.01, M.READER_XY[1]), TABLE_LID_Z))
    assert M.grade(objects, False) == 1.0


def test_lid_knocked_off_the_table():
    objects = loaded(lid_state=lid((0.9, 0.1), 0.02))
    assert M.grade(objects, False) == 0.2


def test_plate_left_on_the_table_scores_zero():
    objects = {"plate": plate((0.12, 0.05), M.TOP + M.PLATE[2] / 2), "lid": lid(M.LID_SET_XY, TABLE_LID_Z),
               "reader": reader(M.DOOR_TRAVEL)}
    assert M.grade(objects, False) == 0.0


def test_plate_on_top_of_the_reader_is_not_in_it():
    objects = loaded(door=M.DOOR_TRAVEL)
    objects["plate"] = plate(NEST_XY, M.TOP + M.BODY_H + 0.01 + M.PLATE[2] / 2, HALF_TURN)
    assert M.grade(objects, False) == 0.0


def test_plate_resting_on_a_low_wall_is_not_in():
    objects = loaded()
    objects["plate"] = plate(NEST_XY, M.FLOOR_Z + M.NEST_H + M.PLATE[2] / 2, HALF_TURN)
    assert not M.plate_in(objects["plate"], objects["reader"])


def test_plate_across_the_walls_a_quarter_turn_out_is_not_in():
    objects = loaded()
    objects["plate"] = plate(NEST_XY, NEST_Z, QUARTER_TURN)
    assert not M.plate_in(objects["plate"], objects["reader"])


def test_plate_on_a_wall_edge_is_not_in():
    objects = loaded()
    objects["plate"] = plate((NEST_XY[0] + 0.012, NEST_XY[1]), NEST_Z, HALF_TURN)
    assert not M.plate_in(objects["plate"], objects["reader"])


def test_any_placement_inside_the_low_walls_counts():
    for dx in (-0.006, 0.006):
        for dy in (-0.006, 0.006):
            objects = loaded()
            objects["plate"] = plate((NEST_XY[0] + dx, NEST_XY[1] + dy), NEST_Z, HALF_TURN)
            assert M.grade(objects, False) == 1.0


def test_tipping_the_plate_at_any_time_scores_zero():
    M._reset({})
    M._watch({"plate": plate(quat=tilted(35))})
    assert M.TIPPED["yes"]
    assert M.score(loaded()) == 0.0
    M._reset({})
    assert not M.TIPPED["yes"]
    M._watch({"plate": plate(quat=tilted(25))})
    assert not M.TIPPED["yes"]
    assert M.score(loaded()) == 1.0


def test_turning_the_plate_over_to_swap_its_corners_tips_it():
    flipped = plate(NEST_XY, NEST_Z, (0.0, 1.0, 0.0, 0.0))
    assert flipped.tilt_deg() > M.TIP_LIMIT_DEG
    assert not M.plate_in(flipped, reader())


def test_turning_the_reader_round_scores_zero():
    home = M.READER.pos
    objects = loaded(turned=False)
    objects["reader"] = reader(M.DOOR_TRAVEL, home, (0.0, 0.0, 1.0, 0.0))
    assert not M.reader_in_place(objects["reader"])
    assert M.grade(objects, False) == 0.0


def test_shoving_the_reader_scores_zero():
    home = np.array(M.READER.pos)
    objects = loaded()
    objects["reader"] = reader(M.DOOR_TRAVEL, home + (0.0, -0.04, 0.0))
    objects["plate"] = plate((NEST_XY[0], NEST_XY[1] - 0.04), NEST_Z, HALF_TURN)
    assert M.grade(objects, False) == 0.0


def test_small_nudges_of_the_reader_do_not_matter():
    home = np.array(M.READER.pos)
    objects = loaded()
    objects["reader"] = reader(M.DOOR_TRAVEL, home + (0.01, 0.01, 0.0))
    objects["plate"] = plate((NEST_XY[0] + 0.01, NEST_XY[1] + 0.01), NEST_Z, HALF_TURN)
    assert M.grade(objects, False) == 1.0


def test_the_window_shows_the_plate_not_the_floor():
    x0, x1, y0, y1 = M.WINDOW
    margin = (M.NEST[0] - M.PLATE[0]) / 2, (M.NEST[1] - M.PLATE[1]) / 2
    assert x1 + margin[0] <= M.PLATE[0] / 2 and y1 + margin[1] <= M.PLATE[1] / 2  # any seated plate fills it
    assert x0 - margin[0] >= -M.PLATE[0] / 2 + 0 and y0 - margin[1] >= -M.PLATE[1] / 2
    right_mark = (-M.MARK_XY[0], -M.MARK_XY[1])  # after the half turn
    half = M.MARK / 2
    assert right_mark[0] - half <= x0 and x1 <= right_mark[0] + half
    assert right_mark[1] - half <= y0 and y1 <= right_mark[1] + half


def test_the_window_of_an_empty_reader_shows_no_orange():
    x1 = M.WINDOW[1]
    assert M.CORNER_MARK_XY[0] - M.CORNER_MARK / 2 > x1 + 0.010  # 1 cm of slack for a camera that isn't overhead


def test_the_reader_mark_is_on_the_corner_it_scores():
    half = M.CORNER_MARK / 2
    assert all(abs(c - m) < half for c, m in zip(M.CORNER_XY, M.CORNER_MARK_XY))


def test_the_door_runs_front_to_back_and_starts_clear_of_the_opening():
    door = M.READER.sliders[0]
    assert (door.axis, door.lower, door.upper) == ("X", -M.DOOR_TRAVEL, 0.0)
    assert M.FRONT_END + M.DOOR_TRAVEL >= M.OPENING[0] / 2  # its front edge, open
    assert M.FRONT_END + M.DOOR_TRAVEL + M.DOOR_W <= M.REAR_END  # its back edge, open


def test_the_door_clears_a_seated_plate():
    door_underside = M.TOP + M.BODY_H + M.DOOR_GAP
    assert door_underside > M.FLOOR_Z + M.PLATE[2]


def test_a_lid_left_on_stops_the_door():
    tab_top = M.FLOOR_Z + M.PLATE[2] + M.LID[2] + M.TAB[2]
    assert tab_top > M.TOP + M.BODY_H + M.DOOR_GAP


def test_the_fingers_clear_the_low_walls_setting_it_down():
    tips = M.PLATE_SET_Z - M.PAD_TIP_PAST_TCP
    assert tips > M.FLOOR_Z + M.NEST_H


def test_the_demo_pulls_the_door_all_the_way_from_behind_the_handle():
    assert M.PULL_END_X - M.FINGER_HALF < M.HANDLE_SHUT_X
    assert M.HANDLE_OPEN_X + M.FINGER_HALF < M.PULL_START_X
    assert M.PULL_START_X + 0.020 < M.READER_XY[0] + M.HOOD_X  # room for the fingers in front of the hood


def plan_with_yaws(*moves):
    waypoints = [Waypoint(3.0, pos, M.FINGER_OPEN, "", yaw) for pos, yaw in moves]
    return ArmPlan(waypoints, station.BASES["left"])


def worst_ik(arm):
    for t in np.arange(0.0, arm.duration + 0.5, 1 / 20):
        arm.action7(t)
    return arm.worst_ik_mm, arm.worst_ik_deg


SHELF_LOW = (*M.PLATE_XY, M.SIDE_GRIP_Z)
SHELF_HIGH = (*M.PLATE_XY, M.TRAVEL_Z)
READER_LOW = (*M.READER_XY, M.PLATE_SET_Z)
READER_HIGH = (*M.READER_XY, M.TRAVEL_Z)
Q = M.QUARTER


def test_the_half_turn_is_the_demos_and_it_seats_the_corner():
    plan = M.demo()["left"]
    start_yaw = next(w.yaw for w in plan if w.label == "close on its long sides")
    end_yaw = next(w.yaw for w in plan if w.label == "lower into the reader")
    assert (start_yaw, end_yaw) == (Q, -Q)
    half = (end_yaw - start_yaw) / 2
    turned = plate(NEST_XY, NEST_Z, (0.0, 0.0, math.sin(half), math.cos(half)))
    assert M.corner_right(turned, reader())


def test_the_half_turn_stays_off_the_wrists_stops():
    lo, hi = ik.LOWER[5], ik.UPPER[5]
    arm = ArmPlan(list(M.demo()["left"]), station.BASES["left"])
    wrist = [arm.action7(t)[0][5] for t in np.arange(0.0, arm.duration, 1 / 20)]
    assert min(wrist) - lo > 0.5 and hi - max(wrist) > 0.5


def test_the_other_way_round_is_out_of_reach():
    _, degrees = worst_ik(plan_with_yaws((SHELF_HIGH, -Q), (READER_HIGH, Q)))
    assert degrees > 1.0


def test_the_straight_move_is_in_reach_so_backwards_is_a_real_mistake():
    mm, degrees = worst_ik(plan_with_yaws((SHELF_HIGH, -Q), (SHELF_LOW, -Q), (SHELF_HIGH, -Q), (READER_HIGH, -Q),
                                          (READER_LOW, -Q)))
    assert mm < 0.5 and degrees < 0.1


def test_every_demo_waypoint_is_reachable():
    plan = M.demo()
    for arm, waypoints in plan.items():
        a = ArmPlan(list(waypoints), station.BASES[arm])
        for t in np.arange(0.0, a.duration + 0.5, 1 / 20):
            a.action7(t)
        assert a.worst_ik_mm < 0.5 and a.worst_ik_deg < 0.1
        assert a.duration * 20 + 60 < 1800


def test_the_arms_keep_to_their_own_sides():
    plan = M.demo()
    assert set(plan) == {"left"}
    assert all(w.pos[1] >= 0 for w in plan["left"])


def test_task_json_matches_the_task_file():
    spec = json.loads((HERE / "task.json").read_text())
    assert spec["instruction"] == M.INSTRUCTION
    assert spec["category"] == "Impact on jobs"
    assert spec["run_limit"] == "2,400 steps (2 minutes at 20 Hz)"
    assert len(spec["fidelity"]) <= 400 and len(spec["setup"]) <= 750 and len(spec["scoring"]) <= 750
    assert len(spec["context"]) <= 450 and len(spec["title"]) < 55
    assert (HERE / "README.md").read_text().count(M.INSTRUCTION) == 2
