"""The liquid height comes only from measured compression of the chamber's wall, both sides, squeezed and released."""

import numpy as np
import pytest

import task
from priming_meter import PrimingMeter

FULL = 0.004


def both(c: float) -> tuple[float, float]:
    return (c, c)


def cycle(meter: PrimingMeter, squeeze: float = FULL, valid: bool = True) -> None:
    for c in (0.0, squeeze / 2, squeeze, squeeze / 2, 0.0):
        meter.observe(both(c), valid=valid)


def test_two_full_squeezes_reach_half_height():
    meter = PrimingMeter()
    cycle(meter)
    assert meter.fill_height == pytest.approx(0.011)
    assert not meter.primed
    cycle(meter)
    assert meter.fill_height == pytest.approx(0.022)
    assert meter.primed


def test_partial_compression_adds_proportional_height():
    meter = PrimingMeter()
    cycle(meter, squeeze=FULL / 2)
    assert meter.fill_height == pytest.approx(0.0055)


def test_no_compression_no_liquid_however_the_gripper_moves():
    """Opening and closing a gripper near the chamber does not move its wall, so it adds nothing."""
    meter = PrimingMeter()
    for _ in range(10):
        for c in (0.0, 0.0002, 0.0004, 0.0, 0.0):  # below the 0.5 mm release band: jitter, not a squeeze
            meter.observe(both(c), valid=True)
    assert meter.fill_height == 0.0


def test_squeeze_before_lock_or_off_vertical_does_not_count():
    meter = PrimingMeter()
    cycle(meter, valid=False)
    assert meter.fill_height == 0.0


def test_squeeze_held_through_locking_does_not_count_on_release():
    meter = PrimingMeter()
    meter.observe(both(FULL), valid=False)  # carrying and seating, wall pressed
    meter.observe(both(FULL), valid=True)  # the spike locks while it is still pressed
    meter.observe(both(0.0), valid=True)  # the insertion grasp lets go: arms the meter, adds nothing
    assert meter.fill_height == 0.0
    cycle(meter)
    assert meter.fill_height == pytest.approx(0.011)


def test_unreleased_squeeze_adds_nothing():
    meter = PrimingMeter()
    meter.observe(both(0.0), valid=True)
    meter.observe(both(FULL), valid=True)
    meter.observe(both(FULL), valid=True)
    assert meter.fill_height == 0.0


def test_rocking_a_held_squeeze_is_not_a_release():
    """Easing one side off while the other stays pressed is not a release: no liquid until both panels are back."""
    meter = PrimingMeter()
    meter.observe(both(0.0), valid=True)
    for _ in range(4):
        meter.observe(both(FULL), valid=True)
        meter.observe((FULL, 0.0009), valid=True)  # one side eases below the both-sides threshold
        meter.observe((FULL, 0.0), valid=True)  # and right back to rest, the other still pressed
    assert meter.fill_height == 0.0
    meter.observe(both(0.0), valid=True)  # now both are released: one squeeze, counted once
    assert meter.fill_height == pytest.approx(0.011)


def test_one_sided_squeeze_then_release_adds_nothing():
    meter = PrimingMeter()
    meter.observe(both(0.0), valid=True)
    meter.observe((FULL, 0.0), valid=True)
    meter.observe(both(0.0), valid=True)
    assert meter.fill_height == 0.0


def test_third_full_squeeze_is_over_target_but_not_yet_a_spill():
    meter = PrimingMeter()
    for _ in range(3):
        cycle(meter)
    assert meter.fill_height == pytest.approx(0.033)
    assert not meter.primed
    assert not meter.spilled


def test_fourth_squeeze_latches_spill_and_never_scores():
    meter = PrimingMeter()
    for _ in range(4):
        cycle(meter)
    assert meter.fill_height == pytest.approx(meter.usable_height)
    assert meter.spilled
    assert not meter.primed


def test_one_sided_push_is_not_a_squeeze():
    """A squeeze is the panels' mean travel, and only once both moved: pressing one side moves nothing."""
    g_pos, g_rot = np.zeros(3), np.eye(3)
    pushed = task.wall_travel(g_pos, g_rot, np.array([-0.003, 0.0, 0.0]), +1)
    untouched = task.wall_travel(g_pos, g_rot, np.zeros(3), -1)
    assert pushed == pytest.approx(0.003) and untouched == 0.0
    assert task.squeeze_of([pushed, untouched]) == 0.0
    assert task.squeeze_of([0.0008, 0.004]) == 0.0  # a brush on one side and a full push on the other: no squeeze
    assert task.squeeze_of([0.003, 0.004]) == pytest.approx(0.0035)  # an off-centre squeeze counts its mean travel


def test_wall_travel_follows_the_set_when_it_is_turned():
    g_rot = task.SET_R0
    g_pos = np.array([0.4, -0.1, 0.8])
    inward = g_pos + g_rot @ np.array([-0.0025, 0.0, 0.0])  # +x panel pushed 2.5 mm toward the axis
    assert task.wall_travel(g_pos, g_rot, inward, +1) == pytest.approx(0.0025)
    assert task.wall_travel(g_pos, g_rot, g_pos + g_rot @ np.array([0.0025, 0.0, 0.0]), -1) == pytest.approx(0.0025)


def test_wall_is_squeezable_and_reachable_between_the_collars():
    assert task.WALL_TRAVEL == task.PrimingMeter().full_squeeze == FULL
    assert task.WALL_K * task.WALL_TRAVEL == pytest.approx(4.0)  # about 4 N a side for a full squeeze, a fingertip squeeze
    assert task.WALL_H / 2 <= task.GAP_Y  # the panels fill the bare gap between the collars
    assert task.WALL_X + task.WALL_T / 2 == pytest.approx(task.CH_R)  # flush with the chamber's surface at rest
    assert task.WALL_X - task.WALL_T / 2 - task.WALL_TRAVEL > task.CORE_R  # a full squeeze stops short of the core
    set_shapes = task._SPEC["giving_set"].shapes
    # Nothing rigid of the set reaches past the core inside the collar gap, so the fingers can only meet the panels there.
    for s in set_shapes:
        if not s.collide or type(s).__name__ != "CylShape":
            continue
        lo, hi = s.centre[1] - s.height / 2, s.centre[1] + s.height / 2
        if lo < task.GAP_Y - 1e-9 and hi > -task.GAP_Y + 1e-9:
            assert s.radius <= task.CORE_R
    source = open(task.__file__).read()
    assert "SQUEEZE_AT" not in source and "near=" not in source
