"""Scorer rules that need a sequence of control steps: the pencil-path record, strokes, sticky rules and the 1 s window."""
import itertools
from pathlib import Path

import numpy as np
import pytest
from yam_arena.task_file import load

HELPERS = ("scribe_runtime", "yam_parts", "yam_joints", "yam_mesh")
MODULE = next(p for p in sorted(Path(__file__).parent.glob("*.py"))
              if not p.name.startswith("test_") and p.stem not in HELPERS)
T = load(MODULE).module
from yam_parts import PartState  # noqa: E402  (the task folder is on the path after loading)

EPISODES = itertools.count(10**6)  # never reuse an episode number across tests
AWAY = {"left": (np.array([0.13, 0.11, 0.90]), 1.0), "right": (np.array([0.20, -0.25, 0.95]), 1.0)}


@pytest.fixture(autouse=True)
def isolate():
    saved = dict(T._RUN)
    T._RUN.update(env=None, t=0.0, step=0, episode=next(EPISODES))
    T.TEST_GRIPPERS, T.TEST_STROKES, T.TEST_LAYOUT = AWAY, None, None
    T.reset_path()
    yield
    T.TEST_GRIPPERS = T.TEST_STROKES = T.TEST_LAYOUT = None
    T.ACTIVE_LAYOUT = 0
    T.reset_path()
    T._RUN.clear()
    T._RUN.update(saved)


def state(spec, origin, vel=(0.0, 0.0, 0.0)):
    origin = np.asarray(origin, float)
    low, high = spec.aabb(origin, np.eye(3))
    return PartState((low + high) / 2, np.array([0.0, 0.0, 0.0, 1.0]), np.asarray(vel, float), np.zeros(3),
                     low, high, origin)


def scene(k, compass_origin, shelf_shift=(0.0, 0.0, 0.0), compass_vel=(0.0, 0.0, 0.0)):
    shelf = T.shelf_origin(k) - [0, 0, 0.001] + np.asarray(shelf_shift)
    return {"wall": state(T.WALL, T.WALL.pos), "shelf": state(T.SHELF, shelf),
            "compass": state(T.COMPASS, compass_origin, compass_vel), "stand": state(T.STAND, T.STAND.pos)}


def compass_at(k, local_xy, depth=0.0015):
    """Compass origin that puts the lead's lowest point at shelf-local (x, y), depth into the top face."""
    shelf = T.shelf_origin(k) - [0, 0, 0.001]
    lead = shelf + [local_xy[0], local_xy[1], T.SHELF_T - depth]
    return lead - T.LEAD_CENTRE + [0, 0, T.LEAD_R]


def step(objects, dt=0.05, skip=0):
    T._RUN["step"] += 1 + skip
    T._RUN["t"] += dt * (1 + skip)
    return T.score(objects)


def scribe(k, line, bias=0.0):
    for x, y in line:
        step(scene(k, compass_at(k, (x, y - bias)), compass_vel=(0.01, 0.0, 0.0)))


def hang_and_wait(k, seconds, shelf_shift=(0.0, 0.0, 0.0)):
    out = []
    for _ in range(int(round(seconds / 0.05))):
        out.append(step(scene(k, T.HANG_ORIGIN, shelf_shift=shelf_shift)))
    return out


def test_live_stroke_then_hang_scores_one_after_a_second():
    line = T.ideal_line(0, step=0.0005)
    scribe(0, line)
    assert len(T._STROKES) == 1 and len(T._STROKES[0]) == len(line)
    scores = hang_and_wait(0, 1.3)
    assert scores[0] == 0.75 and scores[-1] == 1.0
    assert T.success(scene(0, T.HANG_ORIGIN))


def test_compass_held_after_the_stroke_stays_at_075():
    scribe(0, T.ideal_line(0))
    T.TEST_GRIPPERS = {"left": (T.HANG_ORIGIN + T.TCP_LOCAL, 0.0), "right": AWAY["right"]}
    assert hang_and_wait(0, 2.0)[-1] == 0.75


def test_seed_layout_decides_the_targets():
    """Layout 1's shelf and layout 1's line score; layout 0's line drawn on layout 1's shelf does not."""
    T.ACTIVE_LAYOUT = 1
    scribe(1, T.ideal_line(1))
    assert hang_and_wait(1, 1.3)[-1] == 1.0
    T._RUN.update(t=0.0, step=0, episode=next(EPISODES))
    line0 = T.ideal_line(0)
    scribe(1, line0)
    assert hang_and_wait(1, 1.3)[-1] == 0.0


def test_lift_and_skipped_step_start_new_strokes_and_repeat_calls_do_not_add_points():
    line = T.ideal_line(0)
    half = len(line) // 2
    scribe(0, line[:half])
    step(scene(0, compass_at(0, line[half], depth=-0.003)))      # lifted 3 mm: not marking
    scribe(0, line[half + 1:half + 20])
    T._RUN["step"] += 1                                              # a step the scorer did not see
    scribe(0, line[half + 20:])
    assert len(T._STROKES) == 3
    n = sum(len(s) for s in T._STROKES)
    T.score(scene(0, compass_at(0, line[-1])))                      # same step again
    assert sum(len(s) for s in T._STROKES) == n


def test_new_episode_clears_the_record():
    scribe(0, T.ideal_line(0))
    T._RUN.update(t=0.0, step=0, episode=next(EPISODES))
    assert hang_and_wait(0, 1.3)[-1] == 0.0
    assert T._STROKES == []


def test_second_line_15mm_off_unmarks_stations():
    scribe(0, T.ideal_line(0))
    step(scene(0, compass_at(0, T.ideal_line(0)[-1], depth=-0.003)))
    scribe(0, T.ideal_line(0), bias=-0.0015)
    assert hang_and_wait(0, 1.3)[-1] == 0.0


def test_shelf_shoved_is_sticky():
    scribe(0, T.ideal_line(0))
    step(scene(0, T.HANG_ORIGIN, shelf_shift=(0.035, 0.0, 0.0)))
    assert hang_and_wait(0, 1.3)[-1] == 0.0


def test_shelf_dragged_away_from_the_wall_while_scribing_shifts_the_line():
    """A shelf let 0.8 mm off the wall gets a line 0.8 mm too near its end: half credit."""
    away = (0.0, -0.0008, 0.0)
    for x, y in T.ideal_line(0):
        # the compass rides the wall, so the lead runs where it would have; the shelf has moved under it
        step(scene(0, compass_at(0, (x, y)), shelf_shift=away))
    assert hang_and_wait(0, 1.3, shelf_shift=away)[-1] == 0.5


def test_pencil_hovering_or_too_deep_does_not_mark():
    for x, y in T.ideal_line(0)[:50]:
        step(scene(0, compass_at(0, (x, y), depth=-0.0003)))   # 0.3 mm above the face
    for x, y in T.ideal_line(0)[50:100]:
        step(scene(0, compass_at(0, (x, y), depth=0.0025)))    # 2.5 mm into the board
    assert T._STROKES == []


def test_single_touches_at_the_stations_score_nothing():
    """A touch on target at each of the 14 stations, lifting between them: the stations read on target, but the
    marks are 7 to 10 mm apart, not one line, so there is no line credit."""
    for (x, y), in T.station_dots(0):
        step(scene(0, compass_at(0, (x, y)), compass_vel=(0.01, 0.0, 0.0)))
        step(scene(0, compass_at(0, (x, y), depth=-0.003)))         # lifted 3 mm: not marking
    assert len(T._STROKES) == 14 and all(len(s) == 1 for s in T._STROKES)
    assert T.line_quality(T._STROKES, 0)[0] == 1.0                    # the stations alone would read full marks
    assert hang_and_wait(0, 1.3)[-1] == 0.0


def _scribe_with_lift(skip):
    """Layout 0's line with the pencil lifted for one step after 61 mm; it marks again `skip` points (0.5 mm
    apart) after the last mark."""
    line = T.ideal_line(0)
    i = int(np.searchsorted(line[:, 0], -T.SHELF_D / 2 + 0.061))
    scribe(0, line[:i])
    step(scene(0, compass_at(0, line[i], depth=-0.003)))
    scribe(0, line[i - 1 + skip:])
    assert len(T._STROKES) == 2


def test_lift_leaving_1_5_mm_still_one_line():
    _scribe_with_lift(3)
    assert hang_and_wait(0, 1.3)[-1] == 1.0


def test_lift_leaving_2_5_mm_breaks_the_line():
    _scribe_with_lift(5)
    assert hang_and_wait(0, 1.3)[-1] == 0.0
