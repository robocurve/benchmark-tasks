# SPDX-License-Identifier: MIT
"""Boundary and invariance checks for the visible ink ruler and template."""
import json
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from bevel_angle_transfer.scoring import score_strokes


START, END = (-.07, 0.), (.07, 0.)
TICK = (-.068, 0.)


def line(first=-.067, last=.067, y=0., n=2):
    return np.linspace((first, y), (last, y), n)


def grade(strokes):
    return score_strokes(strokes, START, END, tick=TICK, board_width=.14)


def test_full_line_subdivision_reversal_and_retracing_invariant():
    expected = grade([line()])
    dense = line(n=200)
    assert expected["score"] == 1.
    assert expected["marked_coverage"] == pytest.approx(.136)
    assert expected["max_line_width"] == pytest.approx(.002)
    for points in (dense, dense[::-1], np.repeat(dense, 3, axis=0),
                   np.concatenate((dense, dense[::-1], dense))):
        result = grade([points])
        for key in ("score", "marked_coverage", "max_gap", "tick_distance", "max_line_width"):
            assert result[key] == pytest.approx(expected[key])
    json.dumps(expected, allow_nan=False)


def test_tick_and_coverage_are_independent_thresholds():
    assert grade([line(first=-.064, last=.060)])["score"] == 1.
    assert grade([line(first=-.06401, last=.05998)])["score"] == .5
    assert grade([line(first=-.064, last=.004)])["score"] == .5
    assert grade([line(first=-.064, last=.0039)])["score"] == 0.
    assert grade([line(first=-.062, last=.067)])["score"] == .5
    assert grade([line(first=-.0619, last=.067)])["score"] == 0.
    assert grade([line(first=-.05)])["score"] == 0.


def test_template_band_contains_entire_ink_footprint():
    assert grade([line(y=.002)])["score"] == 1.
    assert grade([line(y=.00201)])["score"] == .5
    assert grade([line(y=.004)])["score"] == .5
    assert grade([line(y=.00401)])["score"] == 0.


def test_internal_gaps_measure_unmarked_board_between_caps():
    left = line(last=-.0025)
    assert grade([left, line(first=.0025)])["score"] == 1.
    assert grade([left, line(first=.0026)])["score"] == .5
    assert grade([left, line(first=.0045)])["score"] == .5
    assert grade([left, line(first=.0046)])["score"] == 0.
    broken = grade([np.vstack((left, [np.nan, 0], line(first=.01)))])
    assert broken["invalid_points"] == 1
    assert broken["max_gap"] == pytest.approx(.0105)
    assert broken["score"] == 0.


def test_collinear_joined_strokes_overlapping_stamps_and_repeated_dashes():
    expected = grade([line()])
    dots = [[[x, 0.]] for x in np.linspace(-.067, .067, 135)]
    for strokes in ([line(last=0.), line(first=0.)], dots, dots*2):
        result = grade(strokes)
        assert result["score"] == 1.
        assert result["marked_coverage"] == pytest.approx(expected["marked_coverage"])
        assert not result["extra_marks"]


def test_extra_speck_parallel_line_or_excess_width_fails():
    speck = grade([line(), [[0., .0035]]])
    assert speck["extra_marks"]
    assert speck["score"] == 0.
    assert grade([line(), line(y=.003)])["extra_marks"]
    thick = grade([line(y=y) for y in (-.0011, 0., .0011)])
    assert thick["max_line_width"] > .004
    assert thick["score"] == 0.
    assert grade([line(y=y) for y in (-.001, 0., .001)])["score"] == 1.


def test_board_width_projection_differs_from_slanted_template_length():
    start = np.array([-.07, -.03])
    end = start + [.14, -.05]
    inner = np.array([start + [.005, -.05*.005/.14], end - [.005, -.05*.005/.14]])
    result = score_strokes([inner], start, end, tick=inner[0], board_width=.14)
    assert result["score"] == 1.
    assert result["marked_coverage"] == pytest.approx(.132)
    assert result["target_length"] > .14


def test_separate_far_ink_works_with_bounded_raster():
    result = grade([line(), [[0, .3]]])
    assert result["ink_outside_grading_region"]
    assert result["raster_cells"] <= 2000000
    assert result["score"] == 0.
    assert grade([line(first=-.08)])["score"] == 0.


def test_empty_and_stationary_ink_never_looks_like_a_line():
    for strokes in ([], [[]], [[[0., 0.]]], [np.zeros((300, 2))], [line(-.01, .01)]):
        assert grade(strokes)["score"] == 0.


def test_malformed_geometry_raises_and_standalone_target_can_be_vertical():
    for strokes in ([np.ones((2, 3))], [[1, 2]], [[[1]]]):
        with pytest.raises(ValueError, match="shape"):
            grade(strokes)
    for end in ((0, 0), (np.nan, 0)):
        with pytest.raises(ValueError):
            score_strokes([], (0, 0), end)
    start, end = [.4, -.3], [.4, -.18]
    assert score_strokes([np.linspace(start, end, 20)], start, end)["score"] == 1.
