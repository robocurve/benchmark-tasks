"""The liquid ledger, without a simulator: python -m pytest -q test_ledger.py"""

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import pytest  # noqa: E402
from ledger import Ledger, Vessel  # noqa: E402

TRAVEL = 0.01
DOWN, UP = TRAVEL, 0.0


def two_vessels():
    a = Vessel("a", (0.0, 0.0), (0.02, 0.02), floor_z=0.0, rim_z=0.04, height_per_aliquot=0.005, contents={"dye": 4.0})
    b = Vessel("b", (0.1, 0.0), (0.02, 0.02), floor_z=0.0, rim_z=0.04, height_per_aliquot=0.005, contents={"water": 1.0})
    return Ledger([a, b], travel=TRAVEL, order=["a", "b"])


AIR_A, IN_A = (0.0, 0.0, 0.08), (0.0, 0.0, 0.005)
AIR_B, IN_B = (0.1, 0.0, 0.08), (0.1, 0.0, 0.003)
BENCH = (0.3, 0.3, 0.08)


def run(ledger, steps):
    for tip, plunger in steps:
        ledger.update(tip, plunger)


def transfer(ledger):
    run(ledger, [(AIR_A, UP), (AIR_A, DOWN), (IN_A, DOWN), (IN_A, UP), (AIR_A, UP),
                 (AIR_B, UP), (IN_B, UP), (IN_B, DOWN), (AIR_B, DOWN), (AIR_B, UP)])


def test_a_proper_transfer_moves_one_aliquot_and_mixes():
    ledger = two_vessels()
    transfer(ledger)
    a, b = ledger.vessels["a"], ledger.vessels["b"]
    assert a.volume == pytest.approx(3.0)
    assert b.volume == pytest.approx(2.0)
    assert b.fraction("dye") == pytest.approx(0.5)
    assert ledger.tip_volume == 0.0 and ledger.spilled == 0.0 and not ledger.went_back
    assert [e.kind for e in ledger.events if e.kind != "dip"] == ["aspirate", "dispense"]


def test_releasing_in_air_draws_nothing():
    ledger = two_vessels()
    run(ledger, [(AIR_A, DOWN), (AIR_A, UP)])
    assert ledger.tip_volume == 0.0 and ledger.vessels["a"].volume == pytest.approx(4.0)


def test_releasing_while_still_dipped_after_dispensing_draws_back_up():
    ledger = two_vessels()
    run(ledger, [(AIR_A, DOWN), (IN_A, DOWN), (IN_A, UP), (AIR_A, UP), (IN_B, UP), (IN_B, DOWN), (IN_B, UP)])
    b = ledger.vessels["b"]
    assert b.volume == pytest.approx(1.0)  # one aliquot of the 1:1 mix went back into the tip
    assert ledger.tip_volume == pytest.approx(1.0)
    assert ledger.tip["dye"] == pytest.approx(0.5)


def test_dispensing_outside_any_vessel_is_a_spill():
    ledger = two_vessels()
    run(ledger, [(AIR_A, DOWN), (IN_A, DOWN), (IN_A, UP), (BENCH, UP), (BENCH, DOWN)])
    assert ledger.spilled == pytest.approx(1.0)
    assert ledger.vessels["b"].volume == pytest.approx(1.0)


def test_pressing_inside_a_vessel_above_its_liquid_still_dispenses_into_it():
    ledger = two_vessels()
    run(ledger, [(AIR_A, DOWN), (IN_A, DOWN), (IN_A, UP), ((0.1, 0.0, 0.03), UP), ((0.1, 0.0, 0.03), DOWN)])
    assert ledger.vessels["b"].volume == pytest.approx(2.0) and ledger.spilled == 0.0


def test_a_short_vessel_gives_what_it_has():
    ledger = two_vessels()
    ledger.vessels["a"].contents = {"dye": 0.4}
    run(ledger, [(AIR_A, DOWN), ((0.0, 0.0, 0.0005), DOWN), ((0.0, 0.0, 0.0005), UP)])
    assert ledger.tip_volume == pytest.approx(0.4) and ledger.vessels["a"].volume == pytest.approx(0.0)


def test_going_back_to_an_earlier_vessel_is_flagged():
    ledger = two_vessels()
    transfer(ledger)
    assert not ledger.went_back
    run(ledger, [(AIR_A, DOWN), (IN_A, DOWN)])
    assert ledger.went_back


def test_hysteresis_ignores_a_half_press():
    ledger = two_vessels()
    run(ledger, [(AIR_A, 0.5 * TRAVEL), (IN_A, 0.5 * TRAVEL), (IN_A, UP)])
    assert ledger.tip_volume == 0.0


def test_reset_restores_the_start():
    ledger = two_vessels()
    transfer(ledger)
    ledger.reset()
    assert ledger.vessels["a"].volume == pytest.approx(4.0) and ledger.vessels["b"].volume == pytest.approx(1.0)
    assert ledger.events == [] and ledger.dips == [] and not ledger.pressed


def powered_ledger(mix_steps=4):
    vessels = [
        Vessel("stock", (0.0, 0.0), (0.02, 0.02), 0.0, 0.04, 0.005, {"dye": 2.0}),
        Vessel("cup1", (0.1, 0.0), (0.02, 0.02), 0.0, 0.04, 0.005, {"water": 1.0}),
    ]
    return Ledger(vessels, travel=TRAVEL, order=["stock", "cup1"],
                  powered_mixers={"stock", "cup1"}, mix_steps=mix_steps)


def test_powered_stirrer_marks_a_dispensed_cup_mixed_after_the_dwell():
    ledger = powered_ledger(mix_steps=4)
    transfer(ledger)
    cup = ledger.vessels["cup1"]
    assert not cup.mixed
    run(ledger, [(AIR_B, UP)] * 4)
    assert cup.mixed
    assert any(e.kind == "mixed" and e.vessel == "cup1" for e in ledger.events)


def test_drawing_before_powered_stirring_finishes_is_flagged():
    ledger = powered_ledger(mix_steps=20)
    transfer(ledger)
    run(ledger, [(AIR_B, DOWN), (IN_B, DOWN), (IN_B, UP)])
    assert ledger.unmixed_draw
    assert any(e.kind == "unmixed_aspirate" for e in ledger.events)


def test_reset_restores_mixed_state_and_clears_mixing_faults():
    ledger = powered_ledger(mix_steps=20)
    transfer(ledger)
    assert not ledger.vessels["cup1"].mixed
    ledger.unmixed_draw = True
    ledger.reset()
    assert all(v.mixed and v.mix_ready_step == 0 for v in ledger.vessels.values())
    assert not ledger.unmixed_draw
