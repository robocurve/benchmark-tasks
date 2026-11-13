"""Scorer rules that need a sequence of control steps: the paint record, sticky rules and the 1 s window."""
import itertools
import math
from pathlib import Path

import numpy as np
import pytest
from yam_arena.task_file import load

HELPERS = ("paint_runtime", "yam_parts", "yam_joints")
MODULE = next(p for p in sorted(Path(__file__).parent.glob("*.py"))
              if not p.name.startswith("test_") and p.stem not in HELPERS)
T = load(MODULE).module

EPISODES = itertools.count(10**6)  # never reuse an episode number across tests
AWAY = {"left": (np.array([0.15, 0.30, T.TOP + 0.25]), 1.0), "right": (np.array([0.20, -0.30, T.TOP + 0.25]), 1.0)}
ON_BOARD = T.TOP + T.BOARD_H - T.WHEEL_LINE      # E's height with the wheels on the board's top


@pytest.fixture(autouse=True)
def isolate():
    saved = dict(T._RUN)
    T._RUN.update(env=None, t=0.0, step=0, episode=next(EPISODES))
    T.TEST_GRIPPERS, T.TEST_PAINT = AWAY, None
    T.reset_paint()
    yield
    T.TEST_GRIPPERS = T.TEST_PAINT = None
    T.reset_paint()
    T._RUN.clear()
    T._RUN.update(saved)


def at(face, swivel_deg=0.0, vel=(0.0, 0.0, 0.0), yaw_deg=0.0):
    return {"edger": T.edger_state(np.asarray(face, float), yaw_deg=yaw_deg, swivel_deg=swivel_deg, vel=vel)}


def step(objects, dt=0.05):
    T._RUN["step"] += 1
    T._RUN["t"] += dt
    return T.score(objects)


def stroke(y0=0.250, y1=0.050, x=T.WALL_X, z=ON_BOARD, n=101):
    out = None
    for y in np.linspace(y0, y1, n):
        out = step(at((x, y, z)))
    return out


def home_for(seconds):
    out = []
    for _ in range(int(round(seconds / 0.05))):
        out.append(step(at(T.EDGER_HOME)))
    return out


def test_full_stroke_then_home_scores_after_one_second():
    assert stroke() == 0.75                       # painted, still held at the wall
    scores = home_for(1.2)
    assert scores[0] == 0.75 and scores[-1] == 1.0
    assert scores.index(1.0) >= 20                # the 1 s window
    band, cut, trim = T.coverage(T._PAINT["cells"])
    assert band == 1.0 and cut == 1.0 and trim == 0


def test_paint_stays_when_the_pad_leaves_the_wall():
    stroke()
    step(at((T.X_CLEAR, 0.050, ON_BOARD + 0.012)))   # pulled off the wall
    assert T.coverage(T._PAINT["cells"])[:2] == (1.0, 1.0)


def test_wheels_riding_high_leave_the_cut_line_bare():
    assert stroke(z=ON_BOARD + 0.002) == 0.25
    assert home_for(1.2)[-1] == 0.25


def test_pad_dropping_onto_the_board_caps_at_a_quarter_for_good():
    stroke(0.250, 0.150)
    step(at((T.WALL_X, 0.150, ON_BOARD - 0.002)))   # the pad's edge on the board's top
    stroke(0.150, 0.050)
    assert home_for(1.2)[-1] == 0.25
    assert T.coverage(T._PAINT["cells"])[2] > 0


def test_pad_off_the_wall_paints_nothing():
    assert stroke(x=T.WALL_X - 0.003) == 0.0
    assert T._PAINT["cells"].sum() == 0


def test_held_in_the_rest_is_not_finished():
    stroke()
    T.TEST_GRIPPERS = {"left": (T.EDGER_HOME + T.TCP_IN_E, 0.0), "right": AWAY["right"]}
    assert home_for(1.5)[-1] == 0.75
    T.TEST_GRIPPERS = AWAY
    scores = home_for(1.2)
    assert scores[0] == 0.75 and scores[-1] == 1.0


def test_swivel_still_turning_is_not_still():
    stroke()
    edger = T.edger_state(T.EDGER_HOME)
    from dataclasses import replace
    turning = {"edger": replace(edger, joint_vel={"head": 0.2})}
    for _ in range(30):
        assert step(turning) == 0.75


def test_pad_on_the_table_is_sticky():
    stroke()
    face_down = {"edger": T.EDGER.offline_state(np.array([0.15, 0.42, T.TOP]), 0.0, {"head": 0.0}, R=T._rot("y", 90))}
    assert step(face_down) == 0.0
    assert home_for(1.5)[-1] == 0.0


def test_a_new_episode_starts_clean():
    stroke()
    home_for(1.2)
    T._RUN["episode"] = next(EPISODES)
    T._RUN["t"], T._RUN["step"] = 0.0, 0
    assert step(at(T.EDGER_HOME)) == 0.0
    assert T._PAINT["cells"].sum() == 0


def test_repeat_calls_in_one_step_are_harmless():
    stroke()
    s = at(T.EDGER_HOME)
    for _ in range(3):
        step(s)
    t, k = T._RUN["t"], T._RUN["step"]
    assert T.score(s) == T.score(s)
    assert (T._RUN["t"], T._RUN["step"]) == (t, k)


def test_paint_follows_the_head_on_its_swivel():
    """A dab with the head turned 10 degrees on its swivel: the paint's lower edge slopes with the pad, not the grip."""
    step(at((T.WALL_X, 0.150, ON_BOARD + 0.020), swivel_deg=10.0))
    cells = T.WALL_CELLS[T._PAINT["cells"][:T.N_WALL]]
    left = cells[cells[:, 1] > 0.150 + 0.045]
    right = cells[cells[:, 1] < 0.150 - 0.045]
    assert len(left) and len(right)
    assert left[:, 2].min() > right[:, 2].min() + 0.010     # positive swivel lifts the +y end


def test_paint_reach_matches_the_fixture():
    stroke()
    assert np.array_equal(T._PAINT["cells"][T.BAND], T.stroke_paint()[T.BAND])
    assert not math.isnan(T.coverage(T._PAINT["cells"])[0])
