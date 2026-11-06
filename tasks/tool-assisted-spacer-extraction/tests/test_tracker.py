from __future__ import annotations

import numpy as np
import pytest

from spacer_extraction.tracker import (
    NEST,
    PUSHER_HALF_X,
    SPACER_START,
    PUSHER_START,
    TOP,
    SpacerTrialTracker,
)


SPACER_X = float(SPACER_START[0])
TOOL_X = SPACER_X - 0.014 - PUSHER_HALF_X - 0.002


class ObjectState:
    """Small pure-Python stand-in for Arena's observable ObjectState."""

    def __init__(self, pos, half_size, quat_xyzw=(0.0, 0.0, 0.0, 1.0)):
        self.pos = np.asarray(pos, dtype=float).copy()
        self.quat_xyzw = np.asarray(quat_xyzw, dtype=float).copy()
        self.lin_vel = np.zeros(3, dtype=float)
        self.ang_vel = np.zeros(3, dtype=float)
        half = np.asarray(half_size, dtype=float)
        x, y, z, w = self.quat_xyzw / np.linalg.norm(self.quat_xyzw)
        rotation = np.array([
            [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
            [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
            [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
        ])
        extent = np.abs(rotation) @ half
        self.low = self.pos - extent
        self.high = self.pos + extent

    def resting(self, *, max_speed: float, max_spin: float) -> bool:
        return bool(np.linalg.norm(self.lin_vel) < max_speed and np.linalg.norm(self.ang_vel) < max_spin)


def _objects(tool_x: float = TOOL_X, spacer_x: float = SPACER_X, *, nest: bool = False):
    if nest:
        return {
            "pusher": ObjectState(PUSHER_START, (PUSHER_HALF_X, 0.008, 0.008)),
            "spacer": ObjectState(NEST, (0.014, 0.012, 0.012)),
        }
    return {
        "pusher": ObjectState((tool_x, 0.100, TOP + 0.008), (PUSHER_HALF_X, 0.008, 0.008)),
        "spacer": ObjectState((spacer_x, 0.100, TOP + 0.012), (0.014, 0.012, 0.012)),
    }


def _push_to_ejection(tracker: SpacerTrialTracker, *, steps: int = 41):
    # The pusher tip begins 2 mm behind the spacer's rear face. They advance
    # together in aligned contact, making this a mediated push rather than a
    # direct spacer transport.
    snapshot = tracker.update(_objects(), time_s=0.0)
    for step in range(1, steps + 1):
        x_delta = step * 0.0011
        snapshot = tracker.update(
            _objects(TOOL_X + x_delta, SPACER_X + x_delta),
            time_s=step * 0.05,
        )
    return snapshot


def test_score_boundaries_for_push_recovery_and_release_dwell():
    tracker = SpacerTrialTracker()
    ejected = _push_to_ejection(tracker)
    assert ejected.verified_push_m >= 0.040
    assert ejected.ejected
    assert ejected.score == 0.25

    recovered = tracker.update(_objects(nest=True), time_s=2.1, released=True)
    assert recovered.recovered and recovered.tool_parked
    assert recovered.stable and recovered.stable_s == 0.0
    assert recovered.score == 0.50

    full = recovered
    for step in range(1, 21):
        full = tracker.update(_objects(nest=True), time_s=2.1 + step * 0.05, released=True)
    assert full.success_ready
    assert full.score == 1.0


def test_direct_spacer_transport_does_not_earn_ejection_credit():
    tracker = SpacerTrialTracker()
    tracker.update(_objects(), time_s=0.0)
    transported = tracker.update(_objects(tool_x=TOOL_X, spacer_x=SPACER_X + 0.049), time_s=0.05)
    assert transported.verified_push_m == 0.0
    assert not transported.mediated_push
    assert not transported.ejected
    assert transported.score == 0.0


def test_moving_both_objects_into_contact_from_separated_state_is_not_a_push():
    tracker = SpacerTrialTracker()
    # At x=0.160, the pusher tip is 27 mm from the spacer rear face. A one-step
    # teleport into contact must not retroactively count even its 0.5 mm spacer
    # displacement as a mediated push.
    tracker.update(_objects(tool_x=TOOL_X - 0.025, spacer_x=SPACER_X), time_s=0.0)
    teleported = tracker.update(_objects(tool_x=TOOL_X + 0.001, spacer_x=SPACER_X + 0.0005), time_s=0.05)
    assert teleported.verified_push_m == 0.0
    assert not teleported.mediated_push
    assert teleported.score == 0.0


def test_ejection_credit_requires_forty_millimetres_of_verified_push():
    below = SpacerTrialTracker()
    below_state = _push_to_ejection(below, steps=36)
    assert below_state.verified_push_m < 0.040
    assert not below_state.mediated_push
    assert below_state.score == 0.0

    at_boundary = SpacerTrialTracker()
    boundary_state = _push_to_ejection(at_boundary, steps=37)
    assert boundary_state.verified_push_m >= 0.040
    assert boundary_state.mediated_push and boundary_state.ejected
    assert boundary_state.score == 0.25


def test_release_recontact_interrupts_full_score_dwell():
    tracker = SpacerTrialTracker()
    _push_to_ejection(tracker)
    full = tracker.update(_objects(nest=True), time_s=2.1, released=True)
    for step in range(1, 21):
        full = tracker.update(_objects(nest=True), time_s=2.1 + step * 0.05, released=True)
    assert full.score == 1.0

    recontact = tracker.update(_objects(nest=True), time_s=3.15, released=False)
    assert not recontact.stable
    assert recontact.stable_s == 0.0
    assert recontact.score == 0.50

    restarted = recontact
    for step in range(1, 22):
        restarted = tracker.update(_objects(nest=True), time_s=3.15 + step * 0.05, released=True)
    assert restarted.score == 1.0


@pytest.mark.parametrize("bad_time", [float("nan"), float("inf"), -0.1])
def test_invalid_time_is_rejected(bad_time: float):
    tracker = SpacerTrialTracker()
    with pytest.raises((ValueError, TypeError)):
        tracker.update(_objects(), time_s=bad_time)


def test_nonmonotonic_time_is_rejected():
    tracker = SpacerTrialTracker()
    tracker.update(_objects(), time_s=0.0)
    with pytest.raises((ValueError, TypeError)):
        tracker.update(_objects(), time_s=0.0)


def test_large_positive_time_gap_is_rejected():
    tracker = SpacerTrialTracker()
    tracker.update(_objects(), time_s=0.0)
    with pytest.raises((ValueError, TypeError)):
        tracker.update(_objects(), time_s=0.5)


def test_nonfinite_object_geometry_is_rejected():
    tracker = SpacerTrialTracker()
    objects = _objects()
    objects["pusher"].pos[0] = np.nan
    with pytest.raises((ValueError, TypeError)):
        tracker.update(objects, time_s=0.0)


def test_reset_clears_push_and_release_history():
    tracker = SpacerTrialTracker()
    _push_to_ejection(tracker)
    tracker.update(_objects(nest=True), time_s=2.1, released=True)
    tracker.reset()
    reset_state = tracker.update(_objects(), time_s=0.0, released=False)
    assert reset_state.score == 0.0
    assert reset_state.verified_push_m == 0.0
    assert not reset_state.ejected
    assert not reset_state.recovered


def test_back_and_forth_contact_cannot_farm_the_required_stroke():
    tracker = SpacerTrialTracker()
    tracker.update(_objects(), time_s=0.0)
    for step in range(1, 41):
        delta = 0.003 if step % 2 else 0.0
        result = tracker.update(_objects(TOOL_X + delta, SPACER_X + delta), time_s=step * 0.05)
    assert result.verified_push_m < 0.004
    assert not result.mediated_push
    assert tracker.update(_objects(nest=True), time_s=2.05, released=True).score == 0.0


def test_returning_spacer_to_channel_removes_partial_credit():
    tracker = SpacerTrialTracker()
    assert _push_to_ejection(tracker).score == 0.25
    result = tracker.update(_objects(), time_s=2.1)
    assert not result.ejected
    assert result.score == 0.0


def test_discarding_required_tool_is_a_permanent_zero():
    tracker = SpacerTrialTracker()
    _push_to_ejection(tracker)
    outside = _objects(nest=True)
    outside["pusher"] = ObjectState((1.2, 0.34, TOP + 0.008), (PUSHER_HALF_X, 0.008, 0.008))
    assert tracker.update(outside, time_s=2.1).failure is not None
    result = tracker.update(_objects(nest=True), time_s=2.15, released=True)
    assert result.score == 0.0


def test_motion_breaks_the_release_dwell():
    tracker = SpacerTrialTracker()
    _push_to_ejection(tracker)
    for step in range(1, 20):
        tracker.update(_objects(nest=True), time_s=2.05 + step * 0.05, released=True)
    moving = _objects(nest=True)
    moving["pusher"].lin_vel[0] = 0.02
    result = tracker.update(moving, time_s=3.05, released=True)
    assert result.stable_s == 0.0 and result.score == 0.5


def test_removal_then_pushing_outside_cannot_fake_channel_extraction():
    tracker = SpacerTrialTracker()
    tracker.update(_objects(), time_s=0.0)
    # Remove the spacer without moving the tool, then bring the tool alongside.
    # Moving both together here would describe a fast paired push instead.
    tracker.update(_objects(TOOL_X, SPACER_X + 0.11), time_s=0.05)
    for step in range(1, 42):
        result = tracker.update(_objects(TOOL_X + 0.11 + step * 0.0011,
                                         SPACER_X + 0.11 + step * 0.0011),
                                time_s=(step + 1) * 0.05)
    assert result.failure == "extraction_without_verified_pusher_contact"
    assert result.score == 0.0
    assert tracker.update(_objects(nest=True), time_s=2.15, released=True).score == 0.0


def test_rotated_world_bounds_overlap_does_not_prove_tool_contact():
    # Whole-bar AABBs overlap, and the projected tip gap is within tolerance,
    # although the actual boxes have a 17.48 mm separating gap.
    yaw = np.radians(9.9)
    tool = ObjectState((0.206, 0.115, TOP + 0.008),
                       (PUSHER_HALF_X, 0.008, 0.008),
                       (0.0, 0.0, np.sin(yaw / 2), np.cos(yaw / 2)))
    spacer = ObjectState((SPACER_X, 0.100, TOP + 0.012), (0.014, 0.012, 0.012))
    overlap_y, overlap_z = SpacerTrialTracker._contact_overlap(tool, spacer)
    assert overlap_y > 0.008 and overlap_z > 0.006
    assert not SpacerTrialTracker._aligned_contact(tool, spacer)
    tracker = SpacerTrialTracker()
    tracker.update({"pusher": tool, "spacer": spacer}, time_s=0.0)
    delta = np.array([0.0011, 0.0, 0.0])
    moved_tool = ObjectState(tool.pos + delta, (PUSHER_HALF_X, 0.008, 0.008), tool.quat_xyzw)
    moved_spacer = ObjectState(spacer.pos + delta, (0.014, 0.012, 0.012))
    result = tracker.update({"pusher": moved_tool, "spacer": moved_spacer}, time_s=0.05)
    assert result.verified_push_m == 0.0


def test_contact_above_channel_roof_is_rejected():
    objects = _objects()
    for state in objects.values():
        state.pos[2] += 0.050
        state.low[2] += 0.050
        state.high[2] += 0.050
    assert not SpacerTrialTracker._aligned_contact(objects["pusher"], objects["spacer"])


@pytest.mark.parametrize("quat", [
    (0.0, 0.0, 0.0, 1.0),
    (0.0, 0.0, 1.0, 0.0),  # Reverse the leading end.
    (np.sqrt(0.5), 0.0, 0.0, np.sqrt(0.5)),  # Rest on another square face.
])
def test_square_face_symmetry_preserves_valid_contact(quat):
    objects = _objects()
    tool = ObjectState(objects["pusher"].pos, (PUSHER_HALF_X, 0.008, 0.008), quat)
    spacer = ObjectState(objects["spacer"].pos, (0.014, 0.012, 0.012), quat)
    assert SpacerTrialTracker._aligned_contact(tool, spacer)


@pytest.mark.parametrize("step_m,steps", [(0.0002, 201), (0.006, 7)])
def test_verified_extraction_is_independent_of_push_speed(step_m, steps):
    # 4 mm/s and 120 mm/s follow the same aligned path and earn the same score.
    tracker = SpacerTrialTracker()
    tracker.update(_objects(), time_s=0.0)
    for step in range(1, steps + 1):
        delta = step * step_m
        result = tracker.update(_objects(TOOL_X + delta, SPACER_X + delta), time_s=step * 0.05)
    assert result.verified_push_m == pytest.approx(step_m * steps)
    assert result.failure is None
    assert result.mediated_push and result.score == 0.25
    # Recovery/release is identical regardless of the preceding push velocity.
    start_time = (steps + 1) * 0.05
    for dwell_step in range(22):
        result = tracker.update(_objects(nest=True), time_s=start_time + dwell_step * 0.05,
                                released=True)
    assert result.score == 1.0


@pytest.mark.parametrize("step_m", [0.0002, 0.006])
def test_slow_or_fast_oscillation_cannot_accumulate_verified_stroke(step_m):
    tracker = SpacerTrialTracker()
    tracker.update(_objects(), time_s=0.0)
    for step in range(1, 101):
        delta = step_m if step % 2 else 0.0
        result = tracker.update(_objects(TOOL_X + delta, SPACER_X + delta), time_s=step * 0.05)
    assert result.verified_push_m == pytest.approx(0.0, abs=1e-12)
    assert not result.mediated_push
    assert result.score == 0.0


@pytest.mark.parametrize("name,axis,limit", [
    ("spacer", 0, 0.008), ("spacer", 1, 0.008), ("spacer", 2, 0.004),
    ("pusher", 0, 0.012), ("pusher", 1, 0.012), ("pusher", 2, 0.010),
])
@pytest.mark.parametrize("sign", [-1, 1])
@pytest.mark.parametrize("margin,accepted", [(-1e-6, True), (1e-6, False)])
def test_recovery_and_parking_position_tolerance(name, axis, limit, sign, margin, accepted):
    tracker = SpacerTrialTracker()
    _push_to_ejection(tracker)
    objects = _objects(nest=True)
    state = objects[name]
    shift = np.zeros(3)
    shift[axis] = sign * (limit + margin)
    state.pos += shift
    state.low += shift
    state.high += shift
    snapshot = tracker.update(objects, time_s=2.1, released=True)
    result = snapshot.recovered if name == "spacer" else snapshot.tool_parked
    assert bool(result) is accepted
    assert snapshot.stable is accepted


@pytest.mark.parametrize("name", ["spacer", "pusher"])
@pytest.mark.parametrize("margin,accepted", [(-1e-6, True), (1e-6, False)])
def test_recovery_and_parking_orientation_tolerance(name, margin, accepted):
    tracker = SpacerTrialTracker()
    _push_to_ejection(tracker)
    objects = _objects(nest=True)
    angle = np.radians(10.0 + margin)
    half = (0.014, 0.012, 0.012) if name == "spacer" else (PUSHER_HALF_X, 0.008, 0.008)
    objects[name] = ObjectState(objects[name].pos, half,
                               (0.0, 0.0, np.sin(angle / 2), np.cos(angle / 2)))
    snapshot = tracker.update(objects, time_s=2.1, released=True)
    assert bool(snapshot.recovered if name == "spacer" else snapshot.tool_parked) is accepted
    assert snapshot.stable is accepted


@pytest.mark.parametrize("name,field,limit", [
    ("spacer", "lin_vel", 0.015), ("spacer", "ang_vel", 0.75),
    ("pusher", "lin_vel", 0.010), ("pusher", "ang_vel", 0.5),
])
@pytest.mark.parametrize("margin,accepted", [(-1e-6, True), (1e-6, False)])
def test_recovery_and_release_speed_tolerance(name, field, limit, margin, accepted):
    tracker = SpacerTrialTracker()
    _push_to_ejection(tracker)
    objects = _objects(nest=True)
    getattr(objects[name], field)[0] = limit + margin
    snapshot = tracker.update(objects, time_s=2.1, released=True)
    assert snapshot.stable is accepted
    if name == "spacer":
        assert bool(snapshot.recovered) is accepted


@pytest.mark.parametrize("field,value", [
    ("quat_xyzw", np.zeros(4)),
    ("quat_xyzw", np.array([0.0, 0.0, 0.0, np.nan])),
    ("quat_xyzw", np.ones(3)),
    ("low", np.array([np.nan, 0.0, 0.0])),
    ("high", np.zeros(2)),
])
def test_malformed_quaternion_or_bounds_is_rejected(field, value):
    objects = _objects()
    setattr(objects["pusher"], field, value)
    with pytest.raises(ValueError):
        SpacerTrialTracker().update(objects, time_s=0.0)


def test_inverted_bounds_are_rejected():
    objects = _objects()
    objects["spacer"].low[0] = objects["spacer"].high[0] + 1e-6
    with pytest.raises(ValueError, match="bounds are inverted"):
        SpacerTrialTracker().update(objects, time_s=0.0)
