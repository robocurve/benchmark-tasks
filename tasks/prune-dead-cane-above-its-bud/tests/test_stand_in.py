"""The stand-in's bench check matches the simulated cut, and the README quotes the same figures."""

import math
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import scene_assets as sa  # noqa: E402
import stand_in  # noqa: E402
import task  # noqa: E402


def test_stand_in_cut_force_is_the_simulated_one():
    assert math.isclose(stand_in.cut_force(), task.F_CUT, rel_tol=1e-12)
    assert math.isclose(task.F_CUT, 142.25, abs_tol=0.01)


def test_check_uses_the_demo_cut_radius():
    assert stand_in.CUT_R == task.CUT_RADIUS
    assert sa.JAW_REACH[0] < stand_in.CUT_R < sa.JAW_REACH[1]


def test_pull_point_is_on_the_red_grip():
    near, far = sorted(-y for y in sa.GRIP_Y)
    assert near < stand_in.PULL_R < far


def test_stand_in_sizes_match_the_simulated_canes():
    assert math.isclose(stand_in.STAND_IN_DEAD_D, 0.0045, abs_tol=1e-9)
    live_d = {round(2 * r, 6) for _, _, r in sa.GREEN_CANES}
    assert live_d == {stand_in.STAND_IN_LIVE_D}


def test_bud_bead_matches_the_simulated_bud():
    v = sa.bud_mesh().v
    # The bud's part outside the cane: points farther from the cane axis than its surface.
    radial = (v[:, 0] ** 2 + v[:, 1] ** 2) ** 0.5
    outside = v[radial > sa.R0]
    width = outside[:, 1].max() - outside[:, 1].min()
    height = radial.max() - sa.R0
    assert abs(width - stand_in.BUD_BEAD_D) < 0.0008
    assert abs(height - stand_in.BUD_BEAD_H) < 0.0008


def test_band_and_readme_agree():
    lo, hi = stand_in.check_band()
    assert 21.0 < lo < 23.0 and 35.5 < hi < 37.5
    readme = (ROOT / "README.md").read_text()
    assert f"{lo:.0f} to {hi:.0f} N" in readme
    assert f"{stand_in.PULL_R * 100:.0f} cm from the pivot" in readme
    assert f"{stand_in.CUT_R * 1000:.0f} mm from the pivot" in readme


def test_spring_is_small_against_the_cut():
    assert 0.0 < stand_in.spring_torque_at_cut() < 0.25 * stand_in.cut_torque()
