"""Checks of clear_beam.py's scoring and demo, without the simulator. Needs yam_arena (the official stack):

    python -m pytest -q test_clear_beam.py
"""

import math
from pathlib import Path

import numpy as np
import pytest

from yam_arena import station
from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

TASK = load(Path(__file__).with_name("clear_beam.py"))
M = TASK.module
B1, B2 = M.BLOCKERS  # carton_1 clips the beam near the photo-eye; carton_4 is across it near the reflector
NEAR_MISS, FAR = "carton_3", "carton_2"  # 2.0 and 3.8 cm clear
REACH = M.CARTON[0] / 2 * (abs(M.ACROSS[0]) + abs(M.ACROSS[1]))  # a square carton's reach across the beam


def _rot_x(t):
    return np.array([[1.0, 0.0, 0.0], [0.0, math.cos(t), -math.sin(t)], [0.0, math.sin(t), math.cos(t)]])


def _rot_z(s):
    return np.array([[math.cos(s), -math.sin(s), 0.0], [math.sin(s), math.cos(s), 0.0], [0.0, 0.0, 1.0]])


def carton_state(xy, bottom_z=None, yaw_deg=0.0, tilt_deg=0.0, moving=False):
    """A carton with its centre over ``xy``, turned ``yaw_deg`` about the vertical, then tilted ``tilt_deg`` about world
    x, with its lowest point at ``bottom_z`` (the table by default). Its bounding box is built the way Arena 0.3.1
    builds it: the 8 corners of the carton, turned with it."""
    bottom_z = M.TOP if bottom_z is None else bottom_z
    t, s = math.radians(tilt_deg), math.radians(yaw_deg)
    rot = _rot_x(t) @ _rot_z(s)
    half = np.array(M.CARTON) / 2
    corners = np.array([[i, j, k] for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)]) * half @ rot.T
    centre = np.array([xy[0], xy[1], bottom_z - corners[:, 2].min()])
    world = corners + centre
    st, ct, ss, cs = math.sin(t / 2), math.cos(t / 2), math.sin(s / 2), math.cos(s / 2)
    return ObjectState(
        pos=centre,
        quat_xyzw=np.array([st * cs, -st * ss, ct * ss, ct * cs]),  # the tilt about x times the turn about z
        lin_vel=np.array([0.05, 0.0, 0.0]) if moving else np.zeros(3),
        ang_vel=np.zeros(3),
        low=world.min(axis=0),
        high=world.max(axis=0),
    )


def fixed_state(box):
    centre, half = np.array(box.pos), np.array(box.size) / 2
    return ObjectState(pos=centre, quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=centre - half, high=centre + half)


def scene(**cartons):
    """The start, except the cartons given as name=ObjectState."""
    out = {box.name: fixed_state(box) for box in M.POSTS}
    for name, xy in M.START.items():
        out[name] = cartons.get(name, carton_state(xy))
    return out


def shifted(name, across_m, along_m=0.0):
    """``name`` moved ``across_m`` across the beam (to its left) and ``along_m`` along it, from where it stood."""
    along = np.array([math.cos(M.BEAM_YAW), math.sin(M.BEAM_YAW)])
    return carton_state(tuple(np.array(M.START[name]) + across_m * M.ACROSS + along_m * along))


def offset(name) -> float:
    """Where ``name`` started across the beam: its centre's distance to the left of the line (m)."""
    return float(np.dot(np.array(M.START[name]) - np.array(M.EYE_XY), M.ACROSS))


def away(name, metres):
    """``name`` moved ``metres`` straight away from the beam, on its own side."""
    return shifted(name, math.copysign(metres, offset(name)))


def cleared(**extra):
    """Both blocking cartons moved as the demo moves them; then the overrides."""
    return scene(**{**{n: carton_state(xy) for n, xy in M.demo_targets().items()}, **extra})


def test_at_the_start_exactly_the_two_blockers_block_the_beam():
    s = scene()
    assert [n for n in M.NAMES if M.blocks(s[n])] == list(M.BLOCKERS)
    assert abs(offset(B1)) - REACH == pytest.approx(-0.0105, abs=0.0005)  # clips the line by 1 cm
    assert M.beam_gap(s[NEAR_MISS]) == pytest.approx(0.020, abs=0.001)
    assert M.beam_gap(s[FAR]) == pytest.approx(0.038, abs=0.001)


def test_the_layout_doesnt_give_the_blockers_away():
    # Review: the blockers were the two nearest the beam's middle, and the middle two along it.
    mid = (np.array(M.EYE_XY) + np.array(M.REFLECTOR_XY)) / 2
    nearest_middle = sorted(M.NAMES, key=lambda n: np.linalg.norm(np.array(M.START[n]) - mid))
    assert nearest_middle[0] == NEAR_MISS and set(nearest_middle[:2]) != set(M.BLOCKERS)
    beam_dir = np.array([math.cos(M.BEAM_YAW), math.sin(M.BEAM_YAW)])
    along = sorted(M.NAMES, key=lambda n: float(np.dot(np.array(M.START[n]) - np.array(M.EYE_XY), beam_dir)))
    assert set(along[1:3]) == set(M.BYSTANDERS)  # the middle two along the beam are the ones to leave


def test_the_beam_runs_between_the_posts_at_4_cm():
    assert math.degrees(M.BEAM_YAW) == pytest.approx(37.6, abs=0.1)
    assert M.BEAM_Z - M.TOP == pytest.approx(0.04)
    assert M.BEAM_Z < M.TOP + M.POST[2] and M.BEAM_Z < M.TOP + M.CARTON[2]


def test_the_objects_start_apart():
    xy = {**{n: np.array(p) for n, p in M.START.items()}, "photo_eye": np.array(M.EYE_XY),
          "reflector": np.array(M.REFLECTOR_XY)}
    names = list(xy)
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            assert np.linalg.norm(xy[a] - xy[b]) > 0.06, (a, b)


def test_the_start_scores_zero():
    assert not M.success(scene()) and M.score(scene()) == 0.0


def test_moving_both_blockers_clear_is_success():
    assert M.success(cleared()) and M.score(cleared()) == 1.0


@pytest.mark.parametrize("name", M.BLOCKERS)
def test_moving_one_blocker_clear_scores_a_quarter(name):
    s = scene(**{name: carton_state(M.demo_targets()[name])})
    assert not M.success(s) and M.score(s) == 0.25


def test_moving_a_bystander_as_well_scores_zero():
    assert M.score(cleared(**{NEAR_MISS: away(NEAR_MISS, 0.02)})) == 0.0


@pytest.mark.parametrize("metres", [0.25, 0.09])
def test_clearing_everything_scores_less_than_half_the_job_done_right(metres):
    # Review: every carton moved 25 cm, or pushed 9 cm across, scored 0.5 against 0.25 for one blocker done right.
    everything = scene(**{n: away(n, metres) for n in M.NAMES})
    one_right = scene(**{B1: carton_state(M.demo_targets()[B1])})
    assert not any(M.blocks(everything[n]) for n in M.NAMES)
    assert M.score(everything) == 0.0 < M.score(one_right) == 0.25


def test_a_bystander_nudged_under_5_mm_still_counts_as_untouched():
    assert M.score(cleared(**{FAR: shifted(FAR, 0.004)})) == 1.0


def test_a_blocker_moved_more_than_10_cm_scores_a_half():
    assert M.score(cleared(**{B1: away(B1, 0.11)})) == 0.5


def test_a_blocker_slid_along_the_beam_still_blocks():
    s = scene(**{B2: shifted(B2, 0.0, along_m=-0.03)})
    assert M.blocks(s[B2]) and M.score(s) == 0.0


def test_a_carton_within_1_cm_of_the_beam_still_blocks():
    s = cleared(**{B1: shifted(B1, -(REACH + 0.008) - offset(B1))})
    assert M.beam_gap(s[B1]) == pytest.approx(0.008, abs=1e-6)
    assert M.blocks(s[B1]) and M.score(s) == 0.25


@pytest.mark.parametrize("yaw_deg", [0.0, 20.0, 37.6, 60.0, 82.6])
def test_the_gap_is_measured_to_the_cartons_outline_however_it_is_turned(yaw_deg):
    # Across the beam, a carton turned ``rel`` from the beam's direction reaches 3 cm x (|cos rel| + |sin rel|) from its
    # centre: 3 cm square to the beam, 4.2 cm with a corner toward it.
    xy = M.demo_targets()[B1]
    centre_off = float(np.dot(np.array(xy) - np.array(M.EYE_XY), M.ACROSS))
    rel = math.radians(yaw_deg) - M.BEAM_YAW
    reach = M.CARTON[0] / 2 * (abs(math.cos(rel)) + abs(math.sin(rel)))
    assert M.beam_gap(carton_state(xy, yaw_deg=yaw_deg)) == pytest.approx(abs(centre_off) - reach, abs=1e-6)


def test_a_carton_lifted_above_the_beam_doesnt_block_it_but_isnt_cleared():
    s = scene(**{B1: carton_state(M.START[B1], bottom_z=M.BEAM_Z + 0.01)})
    assert not M.blocks(s[B1]) and M.score(s) == 0.0


def test_a_knocked_over_carton_scores_zero():
    assert M.score(cleared(**{FAR: carton_state(M.START[FAR], tilt_deg=90)})) == 0.0


def test_a_carton_upside_down_still_counts():
    assert M.score(cleared(**{B1: carton_state(M.demo_targets()[B1], tilt_deg=180)})) == 1.0


def test_a_carton_off_the_table_scores_zero():
    s = cleared(**{B2: carton_state((0.3, 0.6), bottom_z=M.TOP - 0.3)})
    assert M.score(s) == 0.0


def test_a_moving_carton_is_not_yet_success():
    # Still sliding, B2 isn't cleared yet: what counts so far is B1, moved clear.
    s = cleared(**{B2: carton_state(M.demo_targets()[B2], moving=True)})
    assert not M.success(s) and M.score(s) == 0.25


def test_the_demo_moves_each_blocker_3_cm_clear_and_under_6_cm():
    for name, xy in M.demo_targets().items():
        moved = np.linalg.norm(np.array(xy) - np.array(M.START[name]))
        assert moved < 0.06
        assert M.beam_gap(carton_state(xy)) == pytest.approx(0.030, abs=0.001)


def test_the_demo_releases_where_it_aims():
    releases = [np.array(w.pos) for w in M.demo()["left"] if w.label == "release"]
    assert [tuple(np.round(p[:2], 9)) for p in releases] == [tuple(np.round(v, 9)) for v in M.demo_targets().values()]
    assert all(p[2] == pytest.approx(M.TOP + M.DROP + M.GRIP_ABOVE) for p in releases)


def test_the_open_fingers_miss_every_other_object():
    # The fingers close along y: open, each reaches 47 mm from the grip's centre plus ~12 mm of finger.
    targets = M.demo_targets()
    for name in M.BLOCKERS:
        for where in (M.START[name], targets[name]):
            others = {n: (targets.get(n, M.START[n]) if where is targets[name] else M.START[n]) for n in M.NAMES
                      if n != name}
            others.update({"photo_eye": M.EYE_XY, "reflector": M.REFLECTOR_XY})
            for other, xy in others.items():
                half = 0.015 if other in ("photo_eye", "reflector") else M.CARTON[0] / 2
                dx, dy = abs(xy[0] - where[0]) - half, abs(xy[1] - where[1]) - half
                assert dx > 0.02 or dy > 0.047 + 0.012 + 0.01, (name, other)


def test_a_carried_carton_clears_every_top():
    assert M.TRAVEL_Z - M.GRIP_ABOVE - (M.TOP + max(M.CARTON[2], M.POST[2])) >= 0.02


def test_the_grip_centres_the_carton():
    from yam_arena.constants import GRIPPER

    squeeze = (M.CARTON[1] - 0.094 * M.GRIP_FINGER / FINGER_OPEN) / 2
    force = GRIPPER["stiffness"] * squeeze
    assert 0.002 <= squeeze and force < 0.5 * GRIPPER["effort"]
    assert 2 * force * 1.0 > 10 * M.CARTON_KG * 9.81


@pytest.mark.parametrize("arm", list(M.demo()))
def test_the_demo_is_within_reach(arm):
    plan = ArmPlan(M.demo()[arm], station.BASES[arm])
    for t in np.arange(0.0, plan.duration, 0.05):
        plan.action7(t)
    assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (plan.worst_ik_mm, plan.worst_ik_deg)


def test_the_demo_finishes_well_within_the_run_limit():
    assert sum(w.duration for w in M.demo()["left"]) < 2400 / 20 / 2
