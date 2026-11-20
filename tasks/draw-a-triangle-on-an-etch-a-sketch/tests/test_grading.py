"""The scorer on drawn paths: full score only for a triangle whose slanted side was drawn with both knobs together."""

import numpy as np

from rdb_etch_triangle import fixed_scene as fixture
from rdb_etch_triangle import grading
from rdb_etch_triangle.grading import TriangleGrade

R = fixture.TOY_ROTATION
HOME = (*fixture.TOY_XY, 0.)


def line(a, b, step=.4):
    """Points every ``step`` mm from a to b (a excluded), in mm."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    n = max(1, int(np.ceil(np.linalg.norm(b - a) / step)))
    return [a + (b - a) * k / n for k in range(1, n + 1)]


def staircase(a, b, step):
    """One axis at a time from a to b, steps of ``step`` mm along X (and 3/4 of it along Y)."""
    out, p = [], np.asarray(a, float)
    b = np.asarray(b, float)
    while np.linalg.norm(b - p) > 1e-9:
        dx = np.sign(b[0] - p[0]) * min(step, abs(b[0] - p[0]))
        out += line(p, p + (dx, 0.), .3)
        p = p + (dx, 0.)
        dy = np.sign(b[1] - p[1]) * min(.75 * step, abs(b[1] - p[1]))
        out += line(p, p + (0., dy), .3)
        p = p + (0., dy)
    return out


def run(points_mm, *, hold_ticks=0, release_ticks=grading.LET_GO_TICKS, position=HOME, rotation=R):
    grade = TriangleGrade(start_xy=fixture.TOY_XY, start_yaw=fixture.TOY_YAW)
    result = None
    for p in points_mm:
        result = grade.update(np.asarray(p) / 1000, position, rotation, True)
    for _ in range(hold_ticks):
        result = grade.update(np.asarray(points_mm[-1]) / 1000, position, rotation, True)
    for _ in range(release_ticks):
        result = grade.update(np.asarray(points_mm[-1]) / 1000, position, rotation, False)
    return result, grade


A, B, C = (0., 0.), (80., 0.), (80., 60.)
BOTTOM, UPRIGHT, SLANT = line(A, B), line(B, C), line(C, A)


def test_perfect_triangle_scores_one():
    result, grade = run(BOTTOM + UPRIGHT + SLANT)
    assert result["score"] == 1. and grade.finished
    assert result["covered_mm"] == {"bottom": 80, "upright": 60, "slant": 100}
    assert result["longest_step_mm"] < grading.MIN_RUN_MM and not result["stray"]


def test_partial_credit():
    assert run(BOTTOM)[0]["score"] == .25
    assert run(BOTTOM + UPRIGHT)[0]["score"] == .5
    assert run(line(A, (69., 0.)))[0]["score"] == 0.          # 69 of 80 mm
    assert run(line(A, (71., 0.)))[0]["score"] == .25
    # The bottom side's first 3.3 mm lie within 2 mm of the slanted side and cover its last 2.7 mm.
    short = line(C, (3.6, 2.7))                                   # stops 4.5 mm short of the dot
    assert run(BOTTOM + UPRIGHT + short)[0]["covered_mm"]["slant"] == 99
    assert run(BOTTOM + UPRIGHT + short)[0]["score"] == .5
    near = line(C, (2.0, 1.5))                                    # stops 2.5 mm short
    assert run(BOTTOM + UPRIGHT + near)[0]["score"] == 1.


def test_a_staircase_is_not_a_slanted_side():
    for step in (1.5, 2.5):                                       # one knob at a time, steps of 1.5 and 2.5 mm along X
        result, _ = run(BOTTOM + UPRIGHT + staircase(C, A, step))
        assert not result["stray"] and result["covered_mm"]["slant"] == 100
        assert result["longest_step_mm"] > grading.STEP_MM and result["score"] == .5
    # Steps under 1 mm (knobs alternated every 10 degrees or less) are a slanted line for this task: a single arm
    # could not alternate that often within the run limit, so they need both hands as well.
    result, _ = run(BOTTOM + UPRIGHT + staircase(C, A, .8))
    assert result["score"] == 1.


def test_slightly_wobbly_but_simultaneous_slant_passes():
    rng = np.random.default_rng(0)
    points, p = [], np.asarray(C)
    for _ in range(400):
        ratio = .75 * (1 + .5 * rng.uniform(-1, 1))               # each step's ratio within +-50 %
        p = p + (-.32, -.32 * ratio)
        p[1] = np.clip(p[1], .6 * p[0] - 1.5, .75 * p[0] + 1.5)   # stays within 2 mm of the ideal line
        points.append(p.copy())
        if p[0] <= .01:
            break
    points += line(points[-1], A)
    result, _ = run(BOTTOM + UPRIGHT + points)
    assert not result["stray"] and result["longest_step_mm"] <= grading.STEP_MM and result["score"] == 1.


def test_a_stray_line_caps_the_score():
    one_to_one = line(C, (20., 0.))                               # 45 degrees: leaves the slant within 10 mm
    result, _ = run(BOTTOM + UPRIGHT + one_to_one + line((20., 0.), A))
    assert result["stray"] and result["score"] == .25
    overshoot = line(A, (83., 0.))                                # 3 mm past the corner, then the rest drawn well
    result, _ = run(overshoot + line((83., 0.), B) + UPRIGHT + SLANT)
    assert result["stray"] and result["tier"] == 1. and result["score"] == .25
    wrong = line(A, (0., -3.))                                    # first move away from the triangle, then nothing
    result, _ = run(wrong)
    assert result["stray"] and result["score"] == 0.


def test_any_order_and_direction():
    reverse = line(A, C) + line(C, B) + line(B, A)
    assert run(reverse)[0]["score"] == 1.


def test_robot_must_let_go_and_leave_the_toy_where_it_was():
    path = BOTTOM + UPRIGHT + SLANT
    assert run(path, release_ticks=grading.LET_GO_TICKS - 1)[0]["score"] == 0.
    pushed = np.asarray(HOME) + (.011, 0., 0.)
    assert run(path, position=pushed)[0]["void"] == "toy pushed off its marks"
    assert run(path, position=np.asarray(HOME) + (.009, 0., 0.))[0]["score"] == 1.
    tilt = np.radians(1.5)                                        # one long edge raised about 3 mm
    tilted = np.array(((1, 0, 0), (0, np.cos(tilt), -np.sin(tilt)), (0, np.sin(tilt), np.cos(tilt)))) @ R
    result, grade = run(path, rotation=tilted)
    assert result["void"] == "toy lifted or tilted" and result["score"] == 0. and grade.finished
    turned = np.array(((np.cos(.1), -np.sin(.1), 0), (np.sin(.1), np.cos(.1), 0), (0, 0, 1))) @ R
    assert run(path, rotation=turned)[0]["score"] == 0.


def test_nothing_drawn_scores_zero():
    result, grade = run([(0., 0.)] * 10)
    assert result["score"] == 0. and not grade.finished
