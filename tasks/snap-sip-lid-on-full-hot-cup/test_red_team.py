"""Scorer rules that need a run of control steps: the sticky rules, the 1 s windows, release and the episode reset."""
import itertools
import math
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest
from yam_arena.task_file import load

HELPERS = ("yam_parts", "yam_joints", "yam_mesh")
MODULE = next(p for p in sorted(Path(__file__).parent.glob("*.py"))
              if not p.name.startswith("test_") and p.stem not in HELPERS)
T = load(MODULE).module
EPISODES = itertools.count(10**6)  # never reuse an episode number across tests
SEATED = -T.SEAT_TRAVEL
FAR = {"left": (np.array([0.11, 0.22, T.TOP + 0.17]), 0.0), "right": (np.array([0.11, -0.22, T.TOP + 0.17]), 0.0)}


@pytest.fixture(autouse=True)
def isolate():
    saved = dict(T._RUN)
    T._RUN.update(env=None, t=0.0, step=0, episode=next(EPISODES))
    T.TEST_GRIPPERS = FAR
    yield
    T.TEST_GRIPPERS = None
    T._RUN.clear()
    T._RUN.update(saved)


def rot_x(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def scene(seated=8, lid_at="cup", cup_shift=(0.0, 0.0, 0.0), cup_tilt=0.0, panels=None, lid_dz=0.0, lid_vel=0.0):
    """The cup (moved, tipped about x through its bottom centre, panels pushed) and the lid on it or elsewhere."""
    R = rot_x(cup_tilt)
    origin = np.array([*T.CUP_XY, T.TOP]) + np.asarray(cup_shift, float)
    cup = T.CUP.offline_state(origin, 0.0, {**{n: 0.0 for n in T.PANEL_NAMES}, **(panels or {})}, R=R)
    q = {n: (SEATED if i < seated else 0.0) for i, n in enumerate(T.SECTOR_NAMES)}
    if lid_at == "cup":
        lid = T.LID.offline_state(origin + R @ np.array([0, 0, T.CUP_H + lid_dz]), 0.0, q, R=R)
    else:
        lid = T.LID.offline_state(np.array([*T.LID_XY, T.TOP + T.LID_REST_Z]), 0.0, q)
    if lid_vel:
        lid = replace(lid, lin_vel=np.array([lid_vel, 0.0, 0.0]))
    return {"cup": cup, "lid": lid}


def tick(objects, n=1):
    """n control steps at 20 Hz showing the same objects; success() then score(), as the harness calls them."""
    out = None
    for _ in range(n):
        T._RUN["step"] += 1
        T._RUN["t"] = T._RUN["step"] / 20.0
        T.success(objects)
        out = T.score(objects)
    return out


def test_full_credit_needs_one_second_released_and_still():
    assert tick(scene(), 10) == 0.5  # under a full 1 s window
    assert tick(scene(), 12) == 1.0
    assert T.success(scene())  # a repeat call in the same control step returns the same result


def test_held_lid_is_capped_at_half():
    T.TEST_GRIPPERS = {"left": (np.array([*T.press_point(2), T.SEATED_TOP + 0.010]), 0.0), "right": FAR["right"]}
    assert tick(scene(), 40) == 0.5
    T.TEST_GRIPPERS = FAR
    assert tick(scene(), 25) == 1.0


def test_partial_tiers_need_release_too():
    assert tick(scene(seated=6), 25) == 0.75
    T._RUN["episode"] = next(EPISODES)
    assert tick(scene(seated=4), 25) == 0.5
    T._RUN["episode"] = next(EPISODES)
    assert tick(scene(seated=0), 25) == 0.25


def test_moving_lid_never_reaches_full_credit():
    for k in range(40):
        assert tick(scene(lid_dz=0.0012 * (k % 2))) == 0.5  # rocking 1.2 mm on the rim: never still for 1 s
    T._RUN["episode"] = next(EPISODES)
    assert tick(scene(lid_vel=0.05), 40) == 0.5


def test_tipped_cup_is_sticky():
    tick(scene(seated=2), 5)
    assert tick(scene(seated=2, cup_tilt=16.0)) == 0.0
    assert tick(scene(), 40) == 0.0


def test_tilt_below_the_limit_is_forgiven():
    tick(scene(seated=2, cup_tilt=12.0), 5)
    assert tick(scene(), 25) == 1.0


def test_lifted_cup_is_sticky():
    tick(scene(cup_shift=(0, 0, 0.012)), 2)
    assert tick(scene(), 40) == 0.0


def test_squeezed_wall_is_sticky():
    tick(scene(panels={"panel_2": -0.0052}), 1)
    assert tick(scene(), 40) == 0.0


def test_diagonal_panel_counts_its_radial_share():
    tick(scene(panels={"panel_1": -0.0068}), 3)  # 6.8 mm along x = 4.8 mm radially: allowed
    assert tick(scene(), 25) == 1.0
    T._RUN["episode"] = next(EPISODES)
    tick(scene(panels={"panel_1": -0.0072}), 1)  # 5.1 mm radially
    assert tick(scene(), 40) == 0.0


def test_steadying_hold_under_the_limit_is_fine():
    tick(scene(seated=0, panels={"panel_4": 0.004, "panel_0": -0.004}), 20)
    assert tick(scene(), 25) == 1.0


def test_sector_forced_down_off_the_cup_is_sticky():
    tick(scene(seated=3, lid_at="counter"), 2)
    assert tick(scene(seated=8), 40) == 0.0


def test_lid_on_the_counter_scores_nothing():
    assert tick(scene(seated=0, lid_at="counter"), 30) == 0.0


def test_cup_slid_off_its_mark_counts_at_the_end_only():
    tick(scene(cup_shift=(0.040, 0.0, 0.0)), 5)
    assert tick(scene(cup_shift=(0.040, 0.0, 0.0))) == 0.0
    assert tick(scene(), 25) == 1.0


def test_new_episode_clears_sticky_rules_and_windows():
    tick(scene(cup_tilt=20.0), 2)
    assert tick(scene(), 30) == 0.0
    T._RUN.update(episode=next(EPISODES), step=0, t=0.0)
    assert tick(scene(), 10) == 0.5
    assert tick(scene(), 12) == 1.0


def test_skipped_step_restarts_the_window():
    tick(scene(), 15)
    T._RUN["step"] += 3  # three control steps not observed
    T._RUN["t"] = T._RUN["step"] / 20.0
    assert tick(scene(), 10) == 0.5
    assert tick(scene(), 12) == 1.0
