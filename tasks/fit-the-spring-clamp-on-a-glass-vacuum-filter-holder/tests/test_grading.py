"""Synthetic states exercise the rubric's boundary conditions independently of PhysX."""

import numpy as np
import pytest

from rdb_filter_clamp import fixed_scene as F
from rdb_filter_clamp import grading as G


def sample(height=0., support=None, *, clamp=True, tilt=0., funnel_lift=0., flask_dx=0.):
    holder = F.HOLDER_START + (flask_dx, 0., height)
    angle = np.radians(tilt)
    rotation = np.array([[1., 0., 0.], [0., np.cos(angle), -np.sin(angle)], [0., np.sin(angle), np.cos(angle)]])
    joint = holder + rotation @ np.array([0., 0., F.JOINT[2]])
    funnel = joint + (0., 0., funnel_lift)
    tcps = [np.array((.05, .25, .3)), np.array((.05, -.25, .3))]
    contacts = {}
    for i, arm in enumerate(("left", "right")):
        for body in ("holder", "funnel", "knuckle", "jawL", "jawR"):
            contacts[f"{arm}_arm_on_{body}"] = 0.
        contacts[f"{arm}_tips_on_funnel"] = [0., 0.]
        if arm == support:
            contacts[f"{arm}_tips_on_funnel"] = [9., 9.]
            contacts[f"{arm}_arm_on_funnel"] = 18.
            tcps[i] = funnel + rotation @ [0., 0., .065]
    return dict(holder_pos=holder, holder_rot=rotation, funnel_pos=funnel, funnel_rot=rotation,
                clamp_pos=joint if clamp else F.CLAMP_START, clamp_rot=rotation,
                jaw_open_deg=[1., 1.], tcp=tcps, contacts=contacts)


def grade():
    return G.FilterClampGrade(tuple(F.HOLDER_START), tuple(F.CLAMP_START))


def repeat(g, s, n):
    for _ in range(n):
        out = g.update(s)
    return out


def test_only_seated_and_clamped_assembly_earns_half():
    g = grade()
    assert g.update(sample(clamp=False))["score"] == 0.
    assert g.update(sample(funnel_lift=.015))["score"] == 0.
    assert g.update(dict(sample(), jaw_open_deg=[25., 25.]))["score"] == 0.
    assert repeat(g, sample(), 100)["score"] == .5


@pytest.mark.parametrize("arm", ["left", "right"])
def test_upper_only_lift_then_set_down_and_release(arm):
    g = grade()
    g.update(sample(support=arm))
    assert not repeat(g, sample(.020, arm), 19)["lift_verified"]
    assert g.update(sample(.020, arm))["lift_verified"]
    assert repeat(g, sample(.020, arm), 10)["score"] == .5
    assert repeat(g, sample(), 9)["score"] == .5
    assert g.update(sample())["score"] == 1.


def test_takeoff_outside_mark_tolerance_requires_a_new_lift_from_the_mark():
    g = grade()
    dx = 2 * G.MARK_TOL
    assert dx < G.FAIL_TRAVEL
    takeoff = g.update(sample(support="left", flask_dx=dx))
    assert takeoff["on_table"] and not takeoff["on_mark"]
    assert takeoff["failed"] is None
    assert not repeat(g, sample(.023, "left", flask_dx=dx), G.HOLD_TICKS)["lift_verified"]
    assert repeat(g, sample(), G.RELEASE_TICKS)["score"] == .5

    g.update(sample(support="left"))
    assert repeat(g, sample(.023, "left"), G.HOLD_TICKS)["lift_verified"]
    assert repeat(g, sample(), G.RELEASE_TICKS)["score"] == 1.


@pytest.mark.parametrize("on_mark_ticks", [0, G.RELEASE_TICKS - 1])
def test_off_mark_set_down_or_release_invalidates_the_lift(on_mark_ticks):
    g = grade()
    g.update(sample(support="left"))
    assert repeat(g, sample(.023, "left"), G.HOLD_TICKS)["lift_verified"]
    for _ in range(on_mark_ticks):
        g.update(sample())
    off_mark = g.update(sample(flask_dx=2 * G.MARK_TOL))
    assert off_mark["failed"] is None
    assert off_mark["on_table"] and not off_mark["on_mark"]
    assert not off_mark["lift_verified"] and off_mark["release_ticks"] == 0
    assert repeat(g, sample(), G.RELEASE_TICKS)["score"] == .5


@pytest.mark.parametrize("fault", ["under_20", "only_funnel", "open_clamp", "second_hand", "same_hand_lower",
                                    "single_finger", "touching_handles", "grip_flange", "no_contacts", "tilted_foot"])
def test_false_lifts_do_not_count(fault):
    g = grade()
    g.update(sample(support="left"))
    s = sample(.022, "left")
    if fault == "under_20":
        s = sample(.0199, "left")
    elif fault == "only_funnel":
        s = sample(support="left", funnel_lift=.024)
    elif fault == "open_clamp":
        s["jaw_open_deg"] = [20., 20.]
    elif fault == "second_hand":
        s["contacts"]["right_arm_on_holder"] = 5.
    elif fault == "same_hand_lower":
        s["contacts"]["left_arm_on_holder"] = 5.
    elif fault == "single_finger":
        s["contacts"]["left_tips_on_funnel"] = [8., 0.]
    elif fault == "touching_handles":
        s["contacts"]["right_arm_on_jawL"] = 2.
    elif fault == "grip_flange":
        s["tcp"][0] = s["funnel_pos"]
    elif fault == "no_contacts":
        s["contacts"] = None
    elif fault == "tilted_foot":
        s = sample(.022, "left", tilt=5.)
    assert not repeat(g, s, 25)["lift_verified"]
    assert repeat(g, sample(), 25)["score"] == .5


def test_gauge_top_counts_for_the_full_hold():
    g = grade()
    g.update(sample(support="left"))
    assert repeat(g, sample(.026, "left"), G.HOLD_TICKS)["lift_verified"]
    assert repeat(g, sample(), G.RELEASE_TICKS)["score"] == 1.


@pytest.mark.parametrize("prior_hold_ticks", [0, G.HOLD_TICKS])
def test_above_gauge_latches_zero_even_after_return_to_the_mark(prior_hold_ticks):
    g = grade()
    g.update(sample(support="left"))
    for _ in range(prior_hold_ticks):
        g.update(sample(.023, "left"))
    assert g.update(sample(.0261, "left"))["score"] == 0.
    assert repeat(g, sample(.023, "left"), G.HOLD_TICKS)["score"] == 0.
    assert repeat(g, sample(), G.RELEASE_TICKS)["score"] == 0.


def test_helper_cannot_hand_over_in_midair_and_disassembly_invalidates_verified_lift():
    g = grade()
    g.update(sample(support="left"))
    assisted = sample(.010, "left")
    assisted["contacts"]["right_arm_on_holder"] = 4.
    g.update(assisted)
    assert not repeat(g, sample(.025, "left"), 30)["lift_verified"]
    g.update(sample(support="left"))
    assert repeat(g, sample(.025, "left"), 20)["lift_verified"]
    assert not g.update(sample(clamp=False))["lift_verified"]
    assert repeat(g, sample(), 10)["score"] == .5


def test_hold_must_be_continuous_and_failure_latches():
    g = grade()
    g.update(sample(support="left"))
    repeat(g, sample(.023, "left"), 15)
    g.update(sample(.018, "left"))
    assert not repeat(g, sample(.023, "left"), 15)["lift_verified"]
    assert g.update(sample(tilt=12.))["failed"]
    assert repeat(g, sample(), 40)["score"] == 0.
