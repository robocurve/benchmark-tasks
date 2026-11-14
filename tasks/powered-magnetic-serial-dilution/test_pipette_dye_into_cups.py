"""The task file without a simulator: python -m pytest -q test_pipette_dye_into_cups.py

Needs ``yam_arena`` importable (it is in the simulator's environment; elsewhere put a clone of yam-isaac-lab-arena
on PYTHONPATH). Checks the scene loads, every demo waypoint is reachable, the outcome table, and a kinematic replay
of the demo: the pipette follows the gripper rigidly while held, which is enough to score the liquid moves.
"""

import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import numpy as np  # noqa: E402
import pytest  # noqa: E402
from benchkit import tool_state  # noqa: E402
from yam_arena import station, task_file  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402

TASK = task_file.load(HERE / "pipette_dye_into_cups.py")
M = TASK.module
UPRIGHT = (0.0, 0.0, 0.0, 1.0)
HZ = 20.0


def pipette_at(tip_xyz, squeeze=0.0, quat=UPRIGHT):
    origin = np.asarray(tip_xyz, dtype=float) - np.array(M.TIP_LOCAL)
    return tool_state(M.PIPETTE, origin, quat, sliders={"side_l": -squeeze, "side_r": squeeze})


def feed(steps):
    """Run (tip position, squeeze) pairs through the task's step hook."""
    M.LEDGER.reset()
    for tip, sq in steps:
        M._track({"pipette": pipette_at(tip, sq)})


def mix_wait(tip, steps=None):
    return [(tip, 0.0)] * (steps if steps is not None else M.MIX_STEPS)


def xyz(cup, z):
    return (M.CUP_X[cup], M.ROW_Y, z)


def transfer_steps(src, dst, dip_dst=False):
    full = M.SQUEEZE_TRAVEL
    return [
        (xyz(src, M.TIP_PRIME), 0.0), (xyz(src, M.TIP_PRIME), full), (xyz(src, M.TIP_DIP), full),
        (xyz(src, M.TIP_DIP), 0.0), (xyz(src, M.TIP_TRAVEL), 0.0), (xyz(dst, M.TIP_TRAVEL), 0.0),
        (xyz(dst, M.TIP_DIP if dip_dst else M.TIP_DISPENSE), 0.0), (xyz(dst, M.TIP_DISPENSE), full),
        (xyz(dst, M.TIP_TRAVEL), full), (xyz(dst, M.TIP_TRAVEL), 0.0),
        *mix_wait(xyz(dst, M.TIP_TRAVEL)),
    ]


PARKED = pipette_at((*M.HOLDER_XY, M.TIP_PARK))
HELD_UP = pipette_at((*M.HOLDER_XY, M.TIP_TRAVEL))


def test_the_task_loads_with_its_objects_hooks_and_docs():
    assert TASK.scene == "station"
    assert {"success", "score", "distance", "demo"} <= {k for k in ("success", "score", "distance", "demo")
                                                         if getattr(TASK, k) is not None}
    assert TASK.free_objects == ("pipette",)
    assert "transfer pipette" in TASK.docs


def test_the_full_protocol_scores_one():
    feed(transfer_steps("stock", "cup1") + transfer_steps("cup1", "cup2") + transfer_steps("cup2", "cup3"))
    assert M.cups_in_order(M.LEDGER) == 3
    assert M.grade(M.LEDGER, PARKED) == pytest.approx(1.0)
    assert M.success({"pipette": PARKED})


def test_doing_nothing_scores_zero():
    M.LEDGER.reset()
    assert M.grade(M.LEDGER, PARKED) == 0.0


def test_drawing_from_a_cup_before_its_powered_mix_dwell_scores_zero():
    first = transfer_steps("stock", "cup1")
    first = first[:-M.MIX_STEPS]
    feed(first + transfer_steps("cup1", "cup2") + transfer_steps("cup2", "cup3"))
    assert M.LEDGER.unmixed_draw
    assert M.grade(M.LEDGER, PARKED) == 0.0


def test_all_four_cups_are_on_the_powered_stirrer():
    assert M.LEDGER.powered_mixers == frozenset(M.ORDER)
    assert M.MIX_STEPS == round(M.MIX_SECONDS * M.CONTROL_HZ)
    assert M.STIR_SPEED_RPM == 600


def test_partial_credit_counts_cups_in_order():
    feed(transfer_steps("stock", "cup1"))
    assert M.grade(M.LEDGER, HELD_UP) == pytest.approx(0.3)
    feed(transfer_steps("stock", "cup1") + transfer_steps("cup1", "cup2"))
    assert M.grade(M.LEDGER, HELD_UP) == pytest.approx(0.6)
    feed(transfer_steps("stock", "cup1") + transfer_steps("cup1", "cup2") + transfer_steps("cup2", "cup3"))
    assert M.grade(M.LEDGER, HELD_UP) == pytest.approx(0.9)  # the pipette not back in its holder


def test_skipping_a_cup_breaks_the_chain():
    feed(transfer_steps("stock", "cup2"))  # cup 2 gets a 1:2, not 1:4; cup 1 untouched
    assert M.cups_in_order(M.LEDGER) == 0


def test_stock_into_every_cup_is_not_a_dilution_series():
    feed(transfer_steps("stock", "cup1") + transfer_steps("stock", "cup2"))
    assert M.grade(M.LEDGER, HELD_UP) == 0.0


def test_going_back_to_an_earlier_cup_scores_zero():
    feed(transfer_steps("stock", "cup1", dip_dst=True) + transfer_steps("stock", "cup2"))
    assert M.LEDGER.went_back
    assert M.grade(M.LEDGER, HELD_UP) == 0.0


def test_a_spill_caps_the_score():
    steps = transfer_steps("stock", "cup1") + transfer_steps("cup1", "cup2") + transfer_steps("cup2", "cup3")
    full = M.SQUEEZE_TRAVEL
    bench = (0.35, -0.05, M.TOP + 0.05)
    steps += [(xyz("cup3", M.TIP_PRIME), 0.0), (xyz("cup3", M.TIP_PRIME), full), (xyz("cup3", M.TIP_DIP), full),
              (xyz("cup3", M.TIP_DIP), 0.0), (bench, 0.0), (bench, full)]
    feed(steps)
    assert M.LEDGER.spilled > 0
    assert M.grade(M.LEDGER, PARKED) == pytest.approx(M.SPILL_CAP)


def test_a_pipette_lying_down_is_not_in_its_holder():
    lying = pipette_at((*M.HOLDER_XY, M.TIP_PARK), quat=(0.7071068, 0.0, 0.0, 0.7071068))
    assert not M.in_holder(lying)
    assert M.in_holder(PARKED)


def plan():
    return ArmPlan(list(M.demo()["left"]), station.BASES["left"])


def test_every_demo_waypoint_is_reachable_within_the_joint_limits():
    arm = plan()
    for t in np.arange(0.0, arm.duration + 0.5, 1.0 / HZ):
        arm.action7(t)
    assert arm.worst_ik_mm < 0.5 and arm.worst_ik_deg < 0.1


def test_the_demo_fits_the_run_limit():
    assert plan().duration * HZ + 3 * HZ < 2400


def test_a_kinematic_replay_of_the_demo_scores_one():
    """Hold the pipette rigidly under the TCP while the fingers are narrower than the bulb; drop it where it is
    let go. Squeeze follows the finger gap."""
    arm = plan()
    origin = np.array(M.PIPETTE.pos, dtype=float)
    held = False
    M.LEDGER.reset()
    pipette = pipette_at(origin + np.array(M.TIP_LOCAL))
    for t in np.arange(0.0, arm.duration + 1.0, 1.0 / HZ):
        tcp, _, finger, _ = arm.target(t)
        width = -2.0 * finger
        touching = width < M.REST_WIDTH
        near = np.linalg.norm(tcp - (origin + np.array([0, 0, M.GRIP_ABOVE_CENTRE]))) < 0.005
        if touching and (held or near):
            held = True
            origin = np.asarray(tcp) - np.array([0, 0, M.GRIP_ABOVE_CENTRE])
        else:
            held = False
        side = min(max((M.REST_WIDTH - width) / 2, 0.0), M.SQUEEZE_TRAVEL) if held else 0.0
        pipette = tool_state(M.PIPETTE, origin, UPRIGHT, sliders={"side_l": -side, "side_r": side})
        M._track({"pipette": pipette})
    assert not M.LEDGER.went_back and M.LEDGER.spilled == 0.0
    assert [e.kind for e in M.LEDGER.events if e.kind not in ("dip", "mixed")] == ["aspirate", "dispense"] * 3
    assert M.grade(M.LEDGER, pipette) == pytest.approx(1.0)


def test_the_hold_grip_does_not_count_as_a_squeeze():
    side_at_hold = (M.REST_WIDTH + 2 * M.HOLD) / 2
    assert 0 < side_at_hold / M.SQUEEZE_TRAVEL < 0.3
    side_at_squeeze = min((M.REST_WIDTH + 2 * M.SQUEEZE) / 2, M.SQUEEZE_TRAVEL)
    assert side_at_squeeze / M.SQUEEZE_TRAVEL >= 0.7


def test_holding_the_pipette_in_its_holder_is_not_putting_it_away():
    feed(transfer_steps("stock", "cup1") + transfer_steps("cup1", "cup2") + transfer_steps("cup2", "cup3"))
    gripped = pipette_at((*M.HOLDER_XY, M.TIP_PARK), squeeze=0.0015)
    assert M.grade(M.LEDGER, gripped) == pytest.approx(0.9)
    assert not M.success({"pipette": gripped})


def test_a_full_pipette_put_away_is_not_full_marks():
    steps = transfer_steps("stock", "cup1") + transfer_steps("cup1", "cup2") + transfer_steps("cup2", "cup3")
    full = M.SQUEEZE_TRAVEL
    # Dip into cup 3 again and draw a pipetteful up, then park with it.
    steps += [(xyz("cup3", M.TIP_PRIME), full), (xyz("cup3", M.TIP_DIP), full), (xyz("cup3", M.TIP_DIP), 0.0)]
    feed(steps)
    assert M.LEDGER.tip_volume > 0
    assert M.grade(M.LEDGER, PARKED) < 1.0


def test_squeezing_just_over_a_cup_lands_in_it():
    full = M.SQUEEZE_TRAVEL
    over = xyz("cup1", M.STIRRER_TOP + M.CUP_HEIGHT + 0.01)
    feed([(xyz("stock", M.TIP_PRIME), full), (xyz("stock", M.TIP_DIP), full), (xyz("stock", M.TIP_DIP), 0.0),
          (over, 0.0), (over, full), (over, 0.0), *mix_wait(over)])
    assert M.LEDGER.spilled == 0.0 and M.cups_in_order(M.LEDGER) == 1


def test_two_pipettefuls_of_stock_into_cup_1_is_wrong_by_volume():
    feed(transfer_steps("stock", "cup1") + transfer_steps("stock", "cup1"))
    cup = M.LEDGER.vessels["cup1"]
    assert cup.volume == pytest.approx(3.0)
    assert M.cups_in_order(M.LEDGER) == 0


def test_strength_band_matches_two_fold_reference_cups():
    lo, hi = M.CONC_BAND
    assert lo * 0.5 == pytest.approx(0.5 / 2**0.5) and hi * 0.5 == pytest.approx(0.5 * 2**0.5)


def test_the_dip_height_is_under_one_pipetteful_and_clear_of_the_film():
    assert M.STIRRER_TOP + M.FILM < M.TIP_DIP < M.STIRRER_TOP + M.LIQUID_PER_ALIQUOT
    assert M.TIP_DISPENSE > M.STIRRER_TOP + 2 * M.LIQUID_PER_ALIQUOT
    assert M.TIP_PRIME > M.STIRRER_TOP + 2 * M.LIQUID_PER_ALIQUOT
    assert M.TIP_PRIME < M.STIRRER_TOP + M.CUP_HEIGHT


def test_the_stem_clears_the_cup_walls_with_room_to_spare():
    assert (M.CUP_INNER - M.STEM) / 2 >= 0.01


def test_task_json_matches_the_task_file():
    import json

    spec = json.loads((HERE / "task.json").read_text())
    assert spec["instruction"] == M.INSTRUCTION
    assert len(spec["fidelity"]) <= 500 and len(spec["scores"]) <= 300
    readme = (HERE / "README.md").read_text()
    assert readme.count(M.INSTRUCTION) == 2


def test_two_stock_pipettefuls_then_the_normal_chain_scores_zero():
    feed(transfer_steps("stock", "cup1") + transfer_steps("stock", "cup1") + transfer_steps("cup1", "cup2")
         + transfer_steps("cup2", "cup3"))
    assert M.grade(M.LEDGER, PARKED) == 0.0


def test_carrying_liquid_back_up_the_row_scores_zero():
    steps = transfer_steps("stock", "cup1") + transfer_steps("cup1", "cup2") + transfer_steps("cup2", "cup3")
    feed(steps + transfer_steps("cup3", "cup2"))
    assert M.LEDGER.went_back and M.grade(M.LEDGER, PARKED) == 0.0
