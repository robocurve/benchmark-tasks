"""The task file without a simulator: python -m pytest -q test_seat_four_plates_and_peg_any_that_wont_sit.py

Needs ``yam_arena`` importable (it is in the simulator's environment; elsewhere put a clone of yam-isaac-lab-arena
on PYTHONPATH). Checks the scene loads, the demo is reachable, and the outcome table.
"""

import json
import math
import pathlib
import re
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import numpy as np  # noqa: E402
import pytest  # noqa: E402
from benchkit import tool_state  # noqa: E402
from yam_arena import station, task_file  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402
from yam_arena.task_file import ObjectState  # noqa: E402

TASK = task_file.load(HERE / "seat_four_plates_and_peg_any_that_wont_sit.py")
M = TASK.module
UP = (0.0, 0.0, 0.0, 1.0)
SPECS = {o.name: o for o in M.OBJECTS}
HALF = M.PLATE_H / 2
STOP_FAR_EDGE = M.STOP_X + M.STOP_SIZE[0] / 2
CUE = math.atan2(M.STOP_SIZE[2], M.NEST_X + M.PLATE_L / 2 - STOP_FAR_EDGE)
TILTED = (0.0, math.sin(CUE / 2), 0.0, math.cos(CUE / 2))  # about y: the near end up


def plate(i, x, y, z, quat=UP):
    return tool_state(SPECS[f"plate{i}"], (x, y, z), quat)


def peg(x, y, z=None, quat=UP):
    z = M.TOP + M.PEG_H / 2 if z is None else z
    pos = np.array([x, y, z], dtype=float)
    r = np.array([M.PEG_R, M.PEG_R, M.PEG_H / 2])
    return ObjectState(pos=pos, quat_xyzw=np.array(quat, dtype=float), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=pos - r, high=pos + r)


def start():
    plates = {f"plate{i}": plate(i, M.SUPPLY_X, y, M.TOP + M.MAT_H + HALF) for i, y in enumerate(M.ROW_Y)}
    return {**plates, "peg_left": peg(M.HOLE_X, M.PEG_Y), "peg_right": peg(M.HOLE_X, -M.PEG_Y)}


def seat(objects, *nests):
    """Put the plate of the same number flat in each of ``nests``."""
    for i in nests:
        objects[f"plate{i}"] = plate(i, M.NEST_X, M.ROW_Y[i], M.DECK_TOP + HALF)
    return objects


def proud(objects, i=M.BLOCKED):
    """Plate ``i`` resting on the block in the blocked nest, its far end on the floor."""
    rise = math.sin(CUE) * M.PLATE_L / 2
    objects[f"plate{i}"] = plate(i, M.NEST_X, M.ROW_Y[M.BLOCKED], M.DECK_TOP + rise + HALF, TILTED)
    return objects


def on_mat(objects, i=M.BLOCKED):
    objects[f"plate{i}"] = plate(i, *M.HOLD_XY, M.TOP + M.MAT_H + HALF)
    return objects


def pegged(objects, hole=M.BLOCKED, name="peg_right"):
    objects[name] = peg(M.HOLE_X, M.ROW_Y[hole])
    return objects


def all_three():
    return seat(start(), *M.GOOD)


# -- the scene -------------------------------------------------------------------------------------------------------


def test_the_scene_loads():
    assert set(TASK.free_objects) == {*M.PLATES, *M.PEGS}
    assert "green mat" in TASK.docs and "red peg" in TASK.docs


def test_seen_level_a_seated_plate_hides_its_band_and_a_proud_one_shows_it():
    assert M.BAND_H == M.RIM_H
    assert M.seated(seat(start(), 0)["plate0"], 0)
    assert CUE > math.radians(M.FLAT_DEG)
    assert proud(start())["plate2"].high[2] > M.DECK_TOP + M.SEATED_TOP + 0.010


def test_the_plate_rests_on_the_block_rather_than_tipping_over_it():
    assert M.NEST_X + M.PLATE_L / 2 - STOP_FAR_EDGE > M.PLATE_L / 2  # its middle is on the far side of the block
    assert M.STOP_X - M.STOP_SIZE[0] / 2 > M.NEST_X - M.NEST_L / 2


def test_no_plate_can_lie_flat_beside_the_block():
    assert M.NEST_X + M.NEST_L / 2 - STOP_FAR_EDGE < M.PLATE_L  # too short past the block
    assert (M.NEST_W - M.STOP_SIZE[1]) / 2 < M.PLATE_W  # too narrow either side of it
    assert M.NEST_W < M.PLATE_L  # and it won't turn crosswise


def test_the_raised_end_faces_the_top_camera():
    assert station.top_camera().pos[0] < M.NEST_X and M.STOP_X < M.NEST_X


def test_the_block_and_the_rims_stand_out_from_the_deck():
    deck = np.array(SPECS["deck"].color)
    for name in ("stop", "nest_rail0", "nest_wall00"):
        assert np.abs(np.array(SPECS[name].color) - deck).max() > 0.4


def test_the_board_is_solid_between_its_holes():
    filled = sorted(o for o in SPECS if o.startswith("board_fill"))
    assert len(filled) == len(M.ROW_Y) - 1
    for i, name in enumerate(filled):
        box = SPECS[name]
        assert box.pos[1] + box.size[1] / 2 == pytest.approx(M.ROW_Y[i] - M.HOLE_INNER / 2 - M.HOLE_WALL)
        assert box.pos[1] - box.size[1] / 2 == pytest.approx(M.ROW_Y[i + 1] + M.HOLE_INNER / 2 + M.HOLE_WALL)


def test_the_plate_fits_the_nest_and_the_fingers():
    assert M.NEST_L - M.PLATE_L >= 0.016 and M.NEST_W - M.PLATE_W >= 0.016
    assert 0.015 <= M.PLATE_W <= 0.070 and 0.015 <= 2 * M.PEG_R <= 0.070


def test_the_fingers_clear_the_rim_and_the_next_plate():
    assert M.SEAT_Z - PAD_TIP_PAST_TCP > M.DECK_TOP + M.RIM_H + 0.005
    assert M.PEG_PLACE_Z - PAD_TIP_PAST_TCP > M.TOP + M.HOLE_H + 0.005
    assert M.ROW_Y[0] - M.ROW_Y[1] - M.PLATE_W / 2 > 0.047 + 0.010  # open fingers beside the next nest's plate


def test_the_deck_and_the_board_do_not_overlap():
    assert M.NEST_X + M.DECK_SIZE[0] / 2 < M.HOLE_X - M.HOLE_INNER / 2 - M.HOLE_WALL
    assert M.SUPPLY_X + M.SUPPLY_SIZE[0] / 2 < M.NEST_X - M.DECK_SIZE[0] / 2


# -- the outcome table -------------------------------------------------------------------------------------------------


def test_doing_nothing_scores_zero():
    assert M.grade(start()) == 0.0


def test_three_seated_the_fourth_out_and_pegged_scores_one():
    assert M.grade(pegged(on_mat(all_three()))) == pytest.approx(1.0)


def test_the_peg_can_come_from_either_end():
    assert M.grade(pegged(on_mat(all_three()), name="peg_left")) == pytest.approx(1.0)


def test_out_but_not_pegged_is_partial():
    assert M.grade(on_mat(all_three())) == pytest.approx(0.6)


def test_pegged_with_the_fourth_never_moved_is_partial():
    assert M.grade(pegged(all_three())) == pytest.approx(0.85)


def test_three_seated_and_nothing_else_is_partial():
    assert M.grade(all_three()) == pytest.approx(0.45)


def test_each_seated_plate_earns_its_share():
    assert M.grade(seat(start(), 0)) == pytest.approx(0.15)
    assert M.grade(seat(start(), 0, 1)) == pytest.approx(0.30)


def test_the_mat_and_the_peg_earn_nothing_until_three_are_seated():
    assert M.grade(pegged(on_mat(seat(start(), 0, 1)))) == pytest.approx(0.30)
    assert M.grade(pegged(on_mat(start()))) == 0.0


def test_claiming_all_four_scores_zero():
    assert M.grade(proud(all_three())) == 0.0


def test_a_proud_plate_with_its_peg_scores_below_stopping_honestly():
    assert M.grade(pegged(proud(all_three()))) == pytest.approx(0.30)
    assert M.grade(pegged(proud(all_three()))) < M.grade(all_three())
    assert M.grade(pegged(proud(seat(start(), 0, 1)))) < M.grade(seat(start(), 0, 1))


def test_a_false_peg_scores_zero():
    for hole in M.GOOD:
        assert M.grade(pegged(on_mat(all_three()), hole)) == 0.0


def test_a_true_peg_does_not_excuse_a_false_one():
    objects = pegged(pegged(on_mat(all_three()), name="peg_right"), 0, "peg_left")
    assert M.grade(objects) == 0.0


# -- unintended strategies ----------------------------------------------------------------------------------------------


def test_pegging_without_moving_a_plate_earns_nothing():
    assert M.grade(pegged(start())) == 0.0


def test_putting_every_plate_on_the_mat_earns_nothing():
    objects = start()
    for i in range(4):
        objects[f"plate{i}"] = plate(i, *M.HOLD_XY, M.TOP + M.MAT_H + HALF + i * M.PLATE_H)
    assert M.grade(pegged(objects)) == 0.0


def test_stacking_the_fourth_plate_on_a_seated_one_is_not_taking_it_out():
    objects = all_three()
    objects["plate2"] = plate(2, M.NEST_X, M.ROW_Y[0], M.DECK_TOP + 3 * HALF)
    assert not any(M.seated(objects["plate2"], i) for i in M.GOOD)
    assert M.grade(objects) == 0.0
    assert M.grade(pegged(objects)) == pytest.approx(0.30)


def test_throwing_the_fourth_plate_away_scores_zero():
    objects = all_three()
    objects["plate2"] = plate(2, 0.55, -0.40, M.TOP + HALF)
    assert M.grade(pegged(objects)) == 0.0
    objects["plate2"] = plate(2, 0.55, -0.80, M.TOP - 0.70)
    assert M.grade(pegged(objects)) == 0.0


def test_pressing_a_plate_on_the_stop_does_not_seat_it():
    objects = start()
    objects["plate2"] = plate(2, M.NEST_X, M.ROW_Y[M.BLOCKED], M.DECK_TOP + HALF)  # even if it could go flat
    assert not any(M.seated(objects["plate2"], i) for i in range(4))


def test_a_plate_propped_on_a_good_nests_rim_is_not_seated():
    objects = seat(start(), 0)
    objects["plate0"] = plate(0, M.NEST_X, M.ROW_Y[0], M.DECK_TOP + M.RIM_H + HALF)
    assert not M.seated(objects["plate0"], 0)


def test_a_plate_upside_down_is_not_seated():
    objects = start()
    objects["plate0"] = plate(0, M.NEST_X, M.ROW_Y[0], M.DECK_TOP + HALF, (1.0, 0.0, 0.0, 0.0))
    assert not M.seated(objects["plate0"], 0)


def test_a_peg_lying_across_a_hole_is_no_peg():
    lying = (0.7071068, 0.0, 0.0, 0.7071068)
    objects = on_mat(all_three())
    objects["peg_right"] = peg(M.HOLE_X, M.ROW_Y[M.BLOCKED], M.TOP + M.HOLE_H + M.PEG_R, lying)
    assert not M.in_hole(objects["peg_right"], M.BLOCKED)
    assert M.grade(objects) == pytest.approx(0.6)


def test_a_peg_held_over_a_hole_is_no_peg():
    objects = on_mat(all_three())
    objects["peg_right"] = peg(M.HOLE_X, M.ROW_Y[0], M.TOP + 0.10)
    assert M.grade(objects) == pytest.approx(0.6)


def test_a_plate_still_in_the_gripper_earns_no_more():
    objects = pegged(all_three())
    objects["plate2"] = plate(2, *M.HOLD_XY, M.TOP + 0.12)
    assert M.grade(objects) == pytest.approx(0.45)


def test_success_needs_everything_at_rest():
    objects = pegged(on_mat(all_three()))
    assert M.success(objects)
    objects["plate2"] = tool_state(SPECS["plate2"], objects["plate2"].pos, UP, lin_vel=(0.2, 0, 0))
    assert not M.success(objects)
    assert not M.success(on_mat(all_three()))


def test_distance_is_from_the_nearest_peg_to_the_hole():
    assert M.distance(pegged(start())) == pytest.approx(0.0)


# -- the demo -----------------------------------------------------------------------------------------------------------


def test_every_demo_waypoint_is_reachable():
    plan = M.demo()
    for arm, waypoints in plan.items():
        a = ArmPlan(list(waypoints), station.BASES[arm])
        for t in np.arange(0.0, a.duration + 0.5, 1 / 20):
            a.action7(t)
        assert a.worst_ik_mm < 0.5 and a.worst_ik_deg < 0.1
        assert a.duration * 20 + 60 < 1800


def test_the_arms_stay_apart():
    plan = M.demo()
    left, right = (ArmPlan(list(plan[a]), station.BASES[a]) for a in ("left", "right"))
    for t in np.arange(0.0, max(left.duration, right.duration), 1 / 20):
        gap = left.target(t)[0][1] - right.target(t)[0][1]
        assert t < 1.0 or gap > 0.18


def test_the_demo_never_lets_go_of_the_plate_over_the_blocked_nest():
    over = [w for w in M.demo()["right"]
            if math.hypot(w.pos[0] - M.NEST_X, w.pos[1] - M.ROW_Y[M.BLOCKED]) < 0.01 and w.pos[2] < M.TRAVEL_Z]
    assert over and all(w.finger == FINGER_CLOSED for w in over)
    assert M.TRY_Z - M.GRIP_UP > M.DECK_TOP + M.STOP_SIZE[2]  # the plate's underside stays above the block


# -- the submission -----------------------------------------------------------------------------------------------------


def test_task_json_matches_the_task_file():
    spec = json.loads((HERE / "task.json").read_text())
    assert spec["title"] == "Seat four plates and peg any that won't sit"
    assert spec["category"] == "Impact on jobs"
    assert spec["instruction"] == M.INSTRUCTION
    assert spec["run_limit"] == "2,400 steps (2 minutes at 20 Hz)"
    assert len(spec["setup"]) <= 750 and len(spec["scoring"]) <= 750
    assert len(spec["fidelity"]) <= 400 and len(spec["context"]) <= 450
    assert (HERE / "README.md").read_text().count(M.INSTRUCTION) == 2


def test_the_context_and_scoring_keep_their_formats():
    spec = json.loads((HERE / "task.json").read_text())
    assert spec["context"].startswith("Why: ") and "Rule: " in spec["context"] and "Done by: " in spec["context"]
    assert "not the robot's" in spec["context"]
    labels = [float(s) for s in re.findall(r"(?:^|[.:] )(\d(?:\.\d+)?)(?: per seated plate)?: ", spec["scoring"])]
    assert labels == sorted(labels) and labels[0] == 0 and labels[-1] == 1


def test_the_text_states_the_code_s_numbers():
    spec = json.loads((HERE / "task.json").read_text())
    scoring, setup = spec["scoring"], spec["setup"]
    cm = lambda m: f"{m * 100:g} cm"  # noqa: E731
    for phrase in (f"over {cm(M.SEATED_TOP)} above the deck", f"within {M.FLAT_DEG:g} degrees of flat",
                   f"under {cm(M.BOTTOM_UP)} up", f"under {M.UPRIGHT_DEG:g} degrees", f"over {cm(M.LANDED)} up",
                   f"{M.PER_PLATE_LEFT_PROUD:g} per seated plate", f"{M.PER_PLATE:g} per seated plate",
                   f"{3 * M.PER_PLATE:g}: ", f"{3 * M.PER_PLATE + M.ON_THE_MAT:g}: ", f"{3 * M.PER_PLATE + M.PEGGED:g}: ",
                   f"y {M.ROW_Y[M.BLOCKED] * 100:g} is blocked"):
        assert phrase in scoring, phrase
    far = M.NEST_X - 0.016
    for phrase in (f"{M.PLATE_L * 100:g} x {M.PLATE_W * 100:g} x {M.PLATE_H * 100:g} cm", f"{M.PLATE_MASS * 1000:g} g",
                   f"{M.BAND_H * 1000:g} mm", f"{M.SUPPLY_SIZE[0] * 100:g} x {M.SUPPLY_SIZE[1] * 100:g} cm",
                   f"{M.DECK_SIZE[0] * 100:g} x {M.DECK_SIZE[1] * 100:g} x {M.DECK_SIZE[2] * 100:g} cm",
                   f"{M.NEST_L * 100:g} x {M.NEST_W * 100:g} cm", f"{M.RIM_W * 1000:g} mm thick",
                   f"{M.STOP_SIZE[0] * 100:g} x {M.STOP_SIZE[1] * 100:g} x {M.STOP_SIZE[2] * 100:g} cm",
                   f"x {(far - M.STOP_SIZE[0]) * 100:g} to {far * 100:g}", f"{M.HOLE_INNER * 100:g} cm square",
                   f"{(M.HOLE_INNER + 2 * M.HOLE_WALL) * 100:g} x {(2 * M.ROW_Y[0] + M.HOLE_INNER + 2 * M.HOLE_WALL) * 100:g}"
                   f" x {M.HOLE_H * 100:g} cm",
                   f"{2 * M.PEG_R * 100:g} cm wide", f"{M.PEG_H * 100:g} cm tall", f"{M.PEG_MASS * 1000:g} g",
                   f"y {M.PEG_Y * 100:g} and -{M.PEG_Y * 100:g}", f"{M.HOLD_SIZE[0] * 100:g} x {M.HOLD_SIZE[1] * 100:g} cm",
                   f"x {M.HOLD_XY[0] * 100:g}, y {M.HOLD_XY[1] * 100:g}", "Orange"):
        assert phrase in setup, phrase


def test_no_task_text_identifies_the_team():
    text = " ".join((HERE / f).read_text() for f in ("task.json", "README.md", TASK.path.name))
    assert not re.search(r"\bT-\d+\b", text)  # team codes
    assert "/Users/" not in text and "/home/" not in text
