"""The kinematic reach check lacks contact forces and cannot certify a proof lift."""

import os
import sys
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


def test_idle_robot_scores_zero():
    assert dry_run(idle=True, max_steps=100)["score"] == 0.
