"""Check reach and joint margins through opening, extracting and turning over the cap."""

import pytest

pytest.importorskip("yam_arena")


def test_opening_and_turnover_waypoints_are_reachable():
    from dryrun_demo import dry_run

    report = dry_run()
    assert report["waypoints"] > 100
    assert report["max_position_error_mm"] < 1.
    assert report["max_orientation_error_deg"] < .5
    assert report["min_joint_limit_margin_deg"] > 5.
