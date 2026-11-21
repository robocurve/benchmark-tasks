"""Kinematic dry runs of the scripted demonstration and its two controls through the real scorer."""

import pytest

pytest.importorskip("yam_arena")
pytest.importorskip("scipy")

import dryrun_demo  # noqa: E402


@pytest.mark.parametrize("variant,expected", [("nominal", 1.), ("one_paddle", 0.), ("release_mismatch", .5)])
def test_dry_run(variant, expected):
    last, reference, trace = dryrun_demo.run(variant)
    assert last["score"] == expected
    assert reference.worst_ik < 1e-3


def test_nominal_finishes_well_inside_the_run_limit():
    last, reference, trace = dryrun_demo.run("nominal")
    assert trace[-1][0] < 800 and last["locked_at"] == 9 and last["clear"]


def test_no_ik_branch_jumps_and_hands_end_clear():
    """Run 3 lost 1.0 when the last retreat step swung joint 1 by 0.52 rad into the converter."""
    import numpy as np

    from rdb_desk_converter.scripted import MAX_JUMP

    history = []
    last, reference, trace = dryrun_demo.run("nominal", history=history)
    assert not [e for e in reference.log["events"] if e["event"] == "ik jump refused"]
    steps = np.array([q for _, q, _ in history])
    arm_cols = [i for i in range(14) if i % 7 != 6]
    assert np.abs(np.diff(steps[:, arm_cols], axis=0)).max() < MAX_JUMP
    assert min(last["hand_from_handle_mm"].values()) > 80.
