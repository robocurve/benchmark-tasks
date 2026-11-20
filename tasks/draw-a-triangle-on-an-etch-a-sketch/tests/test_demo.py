"""Kinematic dry run of the scripted demonstration through the real scorer (needs yam_arena)."""

import numpy as np
import pytest

pytest.importorskip("yam_arena")

from dryrun_demo import dry_run  # noqa: E402
from rdb_etch_triangle import fixed_scene as fixture  # noqa: E402


def test_nominal_dry_run_scores_one_within_the_run_limit():
    report = dry_run()
    assert report["score"] == 1. and report["steps"] < .7 * fixture.MAX_STEPS
    assert report["longest_step_mm"] <= .6 and not report["stray"]
    assert report["worst_ik_mm"] < .5 and report["worst_limit_margin_deg"] > 20.
    assert report["min_pad_tip_above_frame_mm"] > 3. and report["min_tcp_gap_mm"] > 150.
    assert report["regrips"]["left"] >= 8 and report["regrips"]["right"] >= 6


@pytest.mark.parametrize("kw", [dict(follow=.92, arm_lag_s=.2, backlash_deg=25.),
                                dict(knob_shift_mm=(2., -2.), backlash_deg=15.)])
def test_dry_run_tolerates_slip_lag_and_misplacement(kw):
    report = dry_run(**kw)
    assert report["score"] == 1. and report["steps"] < fixture.MAX_STEPS


def test_joint6_alone_turns_the_grip_about_the_knob_axis():
    from yam_arena import ik

    from rdb_etch_triangle.scripted import GRIP_Z, STROKE, EtchReference

    reference = EtchReference()
    for arm in fixture.ARMS:
        q = reference.plan(arm, GRIP_Z, 0.)
        points = []
        for j6 in np.linspace(STROKE[0] - np.radians(25.), STROKE[1] + np.radians(25.), 15):
            q6 = q.copy()
            q6[5] = j6
            points.append(ik.tcp_pose(q6, fixture.BASES[arm])[0])
        spread = np.ptp(np.asarray(points), axis=0)
        assert spread.max() < 2e-4, (arm, spread)


def test_grip_correction_keeps_joint6_on_its_stroke():
    """Run 1 stopped when a 1.1 degree joint 6 shift from a corrected grip point raised an error."""
    from rdb_etch_triangle.scripted import GRIP_Z, STROKE, EtchReference

    reference = EtchReference()
    for arm in fixture.ARMS:
        for offset in ((.012, -.01, -.004), (-.015, .015, .003)):
            reference.offset[arm] = np.asarray(offset)
            for j6 in STROKE:
                q = reference.plan(arm, GRIP_Z, j6)
                assert q[5] == j6
    assert reference.worst_ik < 5e-4
