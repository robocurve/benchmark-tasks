"""Scorer tests with negative controls (no simulator):  python -m pytest -q test_scoring.py"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import layout as LY  # noqa: E402
import scoring as SC  # noqa: E402
import sizing as S  # noqa: E402
from sizing import DEVICE as D  # noqa: E402

FAR = np.array([[0.0, 0.6, 1.2], [0.0, -0.6, 1.2]])  # fingers far away (world)


def seated_against_guide(spec, fwd=0.0, yaw=0.0):
    """Heel at the cup's back wall, form straight on the centre line (its medial ball 1 mm off the guide)."""
    geo = SC.FormGeometry(spec)
    return geo, LY.to_world(fwd, 0, 0), LY.form_quat_xyzw(yaw)


def bar_touching(geo, pos, q):
    lat = SC.to_device_pts(geo.lateral, pos, q)
    m = (lat[:, 0] >= D["bar_u"][0]) & (lat[:, 0] <= D["bar_u"][1])
    return -(D["guide_v"] - lat[m, 1].min()) / 1000.0  # bar joint (m)


def run(spec, *, fwd=0.0, bar=None, ptr=None, fingers=FAR, vel=0.0, quat=None, pos=None):
    geo, p, q = seated_against_guide(spec, fwd)
    if quat is not None:
        q = quat
    if pos is not None:
        p = pos
    b = bar_touching(geo, p, q) if bar is None else bar
    pt = spec["A"] / 1000.0 if ptr is None else ptr
    m = SC.measure(geo, p, q, (vel, 0, 0), (0, 0, 0), pt, b, 0.0, 0.0, fingers)
    return m, SC.score(m, bool(SC.faults(m)))


@pytest.mark.parametrize("spec", S.FORMS, ids=[f["name"] for f in S.FORMS])
def test_full_score_when_measured(spec):
    m, s = run(spec)
    assert m["heel_gap_mm"] < 1 and m["bar_gap_mm"] < 0.5 and m["nose_gap_mm"] < 1.5, m
    assert abs(m["width_steps"] - m["width_ref"]) < 0.5 and abs(m["arch_size"] - spec["size"]) < 0.3, m
    assert s == 1.0


@pytest.mark.parametrize("spec", S.FORMS, ids=[f["name"] for f in S.FORMS])
def test_start_state_scores_zero(spec):
    k = [f["name"] for f in S.FORMS].index(spec["name"])
    form, pos, q = LY.form_start(k)
    geo = SC.FormGeometry(form)
    m = SC.measure(geo, pos, q, (0, 0, 0), (0, 0, 0), S.PTR_START / 1000, -S.BAR_START / 1000, 0, 0, FAR)
    assert SC.score(m, False) == 0.0


def test_single_push_earns_nothing():
    """Heel pushed into the cup but both sliders left at their start: 0."""
    spec = S.FORMS[1]
    _, s = run(spec, bar=-S.BAR_START / 1000, ptr=S.PTR_START / 1000)
    assert s == 0.0


def test_one_slider_on_foot_but_off_reading_gives_quarter():
    spec = S.FORMS[1]
    m, s = run(spec, bar=-S.BAR_START / 1000, ptr=(spec["A"] - 1.5 * S.SIZE_STEP_ARCH) / 1000)
    assert SC.levels(m)["ptr_on"] and not SC.levels(m)["ptr_ok"] and s == 0.25, m


def test_heel_out_after_slider_moved_latches():
    spec = S.FORMS[1]
    m, _ = run(spec, fwd=12.0)
    assert "heel_out" in SC.faults(m, armed=True) and "heel_out" not in SC.faults(m, armed=False)


def test_heel_not_seated_gives_nothing():
    spec = S.FORMS[1]
    _, s = run(spec, fwd=5.0)
    assert s == 0.0


def test_held_form_gets_no_credit():
    spec = S.FORMS[1]
    post = LY.to_world(0.21 * spec["L"], 0, 100)
    _, s = run(spec, fingers=np.array([post, [0, -0.6, 1.2]]))
    assert s == 0.0


def test_bar_left_wide_loses_width():
    spec = S.FORMS[0]
    m, s = run(spec, bar=-S.BAR_START / 1000)
    assert s == 0.5 and not SC.levels(m)["bar_ok"]


def test_bar_closed_on_crooked_form_reads_wrong():
    """A form left turned 5 deg toes-out: the bar touches it but reads more than one width too wide."""
    spec = S.FORMS[0]
    geo, p, q = seated_against_guide(spec, yaw=-5.0)
    b = bar_touching(geo, p, q)
    m = SC.measure(geo, p, q, (0, 0, 0), (0, 0, 0), spec["A"] / 1000, b, 0, 0, FAR)
    assert m["bar_gap_mm"] < 0.5 and m["width_steps"] - m["width_ref"] > 1.0 and not SC.levels(m)["bar_ok"]


def test_pointer_left_back_loses_arch():
    spec = S.FORMS[2]
    m, s = run(spec, ptr=S.PTR_START / 1000)
    assert s == 0.5 and not SC.levels(m)["ptr_ok"]


def test_pointer_one_size_off():
    spec = S.FORMS[1]
    m, s = run(spec, ptr=(spec["A"] - S.SIZE_STEP_ARCH) / 1000)
    assert not SC.levels(m)["ptr_ok"]


def test_gripper_near_slider_blocks_full_score():
    spec = S.FORMS[1]
    fin = LY.to_world((D["bar_u"][0] + D["bar_u"][1]) / 2, D["guide_v"] - spec["W"] - D["bar_t"] - 40, 60)
    _, s = run(spec, fingers=np.array([fin, [0, -0.6, 1.2]]))
    assert s == 0.75


def test_moving_blocks_full_score():
    _, s = run(S.FORMS[1], vel=0.02)
    assert s == 0.75


def test_tipped_form_latches_zero():
    spec = S.FORMS[1]
    q = LY.form_quat_xyzw(0.0)
    tilt = (math.sin(math.radians(12.5)), 0.0, 0.0, math.cos(math.radians(12.5)))  # 25 deg about world x
    x1, y1, z1, w1 = tilt
    x2, y2, z2, w2 = q
    qq = (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
          w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)
    m, s = run(spec, quat=qq)
    assert "tipped" in SC.faults(m) and s == 0.0


def test_fallen_form_latches_zero():
    spec = S.FORMS[1]
    m, s = run(spec, pos=LY.to_world(0, -240, -15))
    assert "fallen" in SC.faults(m) and s == 0.0


def test_scales_cover_forms_and_tables():
    # spot values printed in the published table (inches): men's 9: foot 10 1/3, arch 7 17/50, D width 3 7/8
    assert abs(S.foot_length(9) - (10 + 1 / 3) * 25.4) < 1e-9
    assert abs(S.arch_length(9) - (7 + 17 / 50) * 25.4) < 1e-9
    assert abs(S.width(9, "D") - (3 + 7 / 8) * 25.4) < 1e-9
    assert abs(S.width(10.5, "E") - (4 + 1 / 4) * 25.4) < 1e-9
    assert abs(S.width(8, "C") - (3 + 9 / 16) * 25.4) < 1e-9
