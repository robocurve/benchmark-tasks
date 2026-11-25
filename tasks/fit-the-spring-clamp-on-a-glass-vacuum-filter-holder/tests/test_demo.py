"""The kinematic reach check lacks contact forces and cannot certify a proof lift."""

import os
import sys

import numpy as np
import pytest

pytest.importorskip("scipy")
if os.environ.get("YAM_ARENA"):
    sys.path.insert(0, os.environ["YAM_ARENA"])
pytest.importorskip("yam_arena")

from dryrun_demo import dry_run


def test_gripper_reaches_the_proof_lift_and_attempts_to_restore_the_assembly():
    report = dry_run()
    phases = [name for _, name in report["phases"]]
    assert phases.index("proof lift by upper funnel") < phases.index("lower assembly")
    assert any("glass flanges separated" in e["event"] for e in report["ref_events"])
    assert "proof hold" not in phases
    assert phases.index("lower assembly") < phases.index("funnel release")
    assert report["worst_ik_mm"] < 1.
    assert not report["final"]["lift_verified"] and report["score"] < 1.


def test_proof_lift_stops_when_flange_gap_exceeds_seated_tolerance(monkeypatch):
    from rdb_filter_clamp import fixed_scene as F
    from rdb_filter_clamp.scripted import FilterClampReference

    reference = FilterClampReference()
    upright = np.eye(3)
    parts = {"holder": (F.HOLDER_START, upright), "funnel": (F.JOINT + (0., 0., .004), upright)}
    reference.target["left"] = (F.JOINT + (0., 0., .065), upright)
    monkeypatch.setattr(reference, "part", parts.__getitem__)
    monkeypatch.setattr(reference, "emit", lambda: None)

    def unexpected_motion(*args):
        pytest.fail("The proof lift issued a motion command after the flanges separated")

    monkeypatch.setattr(reference, "command", unexpected_motion)
    list(reference.proof_lift())
    assert reference.log["events"][0]["event"] == "proof lift stopped: glass flanges separated"


@pytest.mark.parametrize(("foot_xy_mm", "clearance_mm", "raises"), [
    ((319.0, -23.5), .202, False),
    ((320.9, -25.6), 2.53, True),
    ((330.0, -30.0), 0., True),
])
def test_proof_lift_commands_use_the_flask_foot_and_fixed_mark(monkeypatch, foot_xy_mm, clearance_mm, raises):
    from rdb_filter_clamp import fixed_scene as F
    from rdb_filter_clamp.scripted import FilterClampReference

    reference = FilterClampReference()
    upright = np.eye(3)
    # Use the recorded XY positions and clearances in synthetic upright poses, with the funnel over the mark.
    foot = np.asarray((*foot_xy_mm, clearance_mm)) / 1000
    funnel = F.JOINT + (0., 0., foot[2])
    parts = {"holder": (foot, upright), "funnel": (funnel, upright)}
    point = funnel + (0., 0., .065)
    reference.target["left"] = (point, upright)
    monkeypatch.setattr(reference, "part", parts.__getitem__)
    monkeypatch.setattr(reference, "solve", lambda *args: reference.joints["left"])
    monkeypatch.setattr(reference, "emit", lambda: None)

    next(reference.proof_lift())

    delta = reference.target["left"][0] - point
    np.testing.assert_array_equal(np.sign(delta[:2]), np.sign(F.HOLDER_START[:2] - foot[:2]))
    if raises:
        assert delta[2] > 0.
    else:
        assert delta[2] == 0.


@pytest.mark.parametrize("reaches_band", [True, False])
def test_slow_on_mark_takeoff_uses_episode_budget_without_starving_release(monkeypatch, reaches_band):
    from rdb_filter_clamp import fixed_scene as F
    from rdb_filter_clamp import grading as G
    from rdb_filter_clamp.scripted import FilterClampReference

    reference = FilterClampReference()
    reference.tick = start_tick = 925
    upright = np.eye(3)
    reference.target["left"] = (F.JOINT + (0., 0., .065), upright)

    def part(name):
        # Synthetic trace with on-mark takeoff near the old cutoff and a delayed rise into the band.
        elapsed = reference.tick - start_tick
        height = 0.
        if elapsed >= 316:
            height = .005503
        if reaches_band and elapsed >= 325:
            height = .023
        position = F.HOLDER_START if name == "holder" else F.JOINT
        return position + (0., 0., height), upright

    monkeypatch.setattr(reference, "part", part)
    monkeypatch.setattr(reference, "command", lambda *args: None)
    list(reference.proof_lift())
    list(reference.let_go_of_funnel())
    events = reference.log["events"]
    hold = next(row for row in events if row["event"] == "proof hold ended")
    timeouts = [row for row in events if row["event"] == "proof lift step budget exhausted"]

    if reaches_band:
        assert hold["tick"] > start_tick + 16 * F.CONTROL_HZ
        assert hold["held_ticks"] >= G.HOLD_TICKS
        assert not timeouts
        assert reference.tick <= F.MAX_STEPS - G.RELEASE_TICKS
    else:
        assert hold["held_ticks"] == 0
        assert len(timeouts) == 1
        assert reference.tick == F.MAX_STEPS - G.RELEASE_TICKS


def test_idle_robot_scores_zero():
    assert dry_run(idle=True, max_steps=100)["score"] == 0.
