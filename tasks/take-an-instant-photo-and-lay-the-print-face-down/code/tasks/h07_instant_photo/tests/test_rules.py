"""The instant photo's rules without the simulator: states are set directly and the task file's own functions read
them. Run in the task's environment from the code folder: ``python -m pytest -q tasks/h07_instant_photo/tests``."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest

from rta import spec
from rta.spec import ObjectState, Snapshot, Trial

TASK = Path(__file__).resolve().parents[1] / "task.py"


@pytest.fixture(scope="module")
def task():
    return spec.load(TASK)


@pytest.fixture(scope="module")
def m(task):
    return task.module


def quat_y(deg):
    """Rotation about the world y axis (positive turns the lens down)."""
    a = math.radians(deg) / 2
    return np.array([0.0, math.sin(a), 0.0, math.cos(a)])


def rot_of(q):
    return spec.quat_matrix(q)


def box_state(centre, half, quat=(0, 0, 0, 1.0), vel=(0, 0, 0)):
    """An ObjectState whose world-aligned box is the rotated local box ``half`` around ``centre``."""
    r = rot_of(np.asarray(quat, float))
    corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * np.asarray(half, float)
    w = corners @ r.T + np.asarray(centre, float)
    return ObjectState(np.asarray(centre, float), np.asarray(quat, float), np.asarray(vel, float), np.zeros(3), w.min(0), w.max(0))


def camera(m, origin=None, quat=(0, 0, 0, 1.0), vel=(0, 0, 0)):
    """The camera with its frame's origin (on its underside, under its centre) at ``origin``."""
    origin = np.array([m.CAM_XY[0], m.CAM_XY[1], m.TOP]) if origin is None else np.asarray(origin, float)
    half = np.array([m.L["extent"]["camera"] / 2, m.CAM_SIZE[1] / 2, m.CAM_SIZE[2] / 2])
    return box_state(origin + rot_of(np.asarray(quat, float)) @ m.BOX_C, half, quat, vel)


def print_at(m, centre, quat=(0, 0, 0, 1.0), vel=(0, 0, 0)):
    return box_state(centre, np.asarray(m.PRINT) / 2, quat, vel)


def print_in(m, cam, depth):
    """The print in the camera's slot with its centre ``depth`` along the lens axis (the rollers' place for it)."""
    return print_at(m, m.to_world(cam, (depth, 0.0, m.PRINT_Z + m.PRINT[2] / 2)), cam.quat_xyzw)


def apple(m):
    c = (m.APPLE_XY[0], m.APPLE_XY[1], m.TOP + m.APPLE_EXT / 2)
    return box_state(c, (m.APPLE_EXT / 2,) * 3)


PARKED = {"left": (0.12, 0.30, 0.91), "right": (0.12, -0.30, 0.91)}
FACE_DOWN = (1.0, 0.0, 0.0, 0.0)  # the print turned over about x: its picture side down


DOWN = np.array([[0.0, -1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])  # pointing down, closing along world y


def across(pr):
    """A hand rotation that closes the fingers along the print's face normal (a pinch across its faces)."""
    n = rot_of(pr.quat_xyzw)[:, 2]
    w = np.cross(n, [1.0, 0.0, 0.0]) if abs(n[0]) < 0.9 else np.cross(n, [0.0, 1.0, 0.0])
    w /= np.linalg.norm(w)
    return np.column_stack([n, w, np.cross(n, w)])


def snap(m, t, cam, pr, shutter_in=0.0, tcp=None, finger=None, tcp_rot=None):
    return Snapshot({"camera": cam, "print": pr, "apple": apple(m)}, t=t, tcp=tcp or PARKED,
                    finger=finger or {"left": -0.047, "right": -0.047}, joints={"camera": {"shutter": -shutter_in}},
                    tcp_rot=tcp_rot or {"left": DOWN, "right": DOWN})


@pytest.fixture()
def state(m):
    return m.setup(np.random.default_rng(0), Trial(scene_id="seed-0", init_seed=0, seed=0))


class World:
    """What ``events`` sees: the step's objects and a pose setter that records."""

    def __init__(self, objects):
        self.objects = objects
        self.set = {}

    def set_pose(self, name, pos, quat=None):
        self.set[name] = (np.asarray(pos, float), quat)


# --- the start -----------------------------------------------------------------------------------------------------
def test_one_start_for_every_seed(m):
    """The task has one start: its state and placements do not depend on the seed (yam_memory spawns without jitter)."""
    starts = [(m.setup(np.random.default_rng(i), Trial(scene_id=f"seed-{i}", init_seed=i, seed=i)),) for i in range(10)]
    assert all(s == starts[0] for s in starts)
    assert all(m.placements(s[0]) == m.placements(starts[0][0]) for s in starts)


def test_face_down_is_read_from_the_print_itself(m):
    """The picture is the model's +z face (assets/build.py): the score reads the print body's own rotation."""
    up = print_at(m, (0.36, -0.04, m.TOP + 0.002))
    down = print_at(m, (0.36, -0.04, m.TOP + 0.002), FACE_DOWN)
    assert m.picture_up(up) == pytest.approx(1.0) and m.picture_up(down) == pytest.approx(-1.0)


# --- the shutter ---------------------------------------------------------------------------------------------------
def test_press_short_of_home_takes_no_photo(m, state):
    cam = camera(m)
    s = snap(m, 3.0, cam, print_in(m, cam, m.HIDDEN_X), shutter_in=0.99 * m.PRESSED)
    assert m.milestones(s, state) == [False] * 4 and m.failure(s, state) is None


def test_first_press_home_takes_the_photo(m, state):
    cam = camera(m)
    angle, dist = m.aim({"camera": cam, "apple": apple(m)})
    assert angle < 1.0 and m.AIM_DIST[0] <= dist <= m.AIM_DIST[1]
    s = snap(m, 3.0, cam, print_in(m, cam, m.HIDDEN_X), shutter_in=1.01 * m.PRESSED)
    assert m.milestones(s, state) == [True, False, False, False] and state["shot_t"] == 3.0
    assert state["release_t"] is None  # still held down


@pytest.mark.parametrize("how", ["tilted", "moving", "turned away"])
def test_first_press_home_off_the_shot_wastes_the_exposure(m, state, how):
    if how == "tilted":
        cam = camera(m, quat=quat_y(m.LEVEL_DEG + 3))
    elif how == "moving":
        cam = camera(m, vel=(1.5 * m.STEADY, 0.0, 0.0))
    else:
        a = math.radians(m.AIM_DEG + 3) / 2
        cam = camera(m, quat=(0.0, 0.0, math.sin(a), math.cos(a)))
    s = snap(m, 3.0, cam, print_in(m, cam, m.HIDDEN_X), shutter_in=m.SHUTTER_TRAVEL)
    assert m.failure(s, state) == "wasted_exposure" and state["shot_t"] is None
    good = camera(m)
    later = snap(m, 6.0, good, print_in(m, good, m.HIDDEN_X), shutter_in=m.SHUTTER_TRAVEL)
    assert m.milestones(later, state)[0] is False  # one exposure: the first press home was it


def test_print_comes_out_only_when_the_button_is_let_go(m, state):
    cam = camera(m)
    pr = print_in(m, cam, m.HIDDEN_X)
    m.milestones(snap(m, 3.0, cam, pr, shutter_in=m.SHUTTER_TRAVEL), state)
    w = World(snap(m, 4.0, cam, pr, shutter_in=m.SHUTTER_TRAVEL))
    m.milestones(w.objects, state)
    m.events(4.0, state, w)
    assert state["release_t"] is None and w.set["print"][0][0] == pytest.approx(m.to_world(cam, (m.HIDDEN_X, 0, m.PRINT_Z))[0])
    up = snap(m, 4.5, cam, pr, shutter_in=0.9 * m.RELEASED)
    m.milestones(up, state)
    assert state["release_t"] == 4.5
    w = World(snap(m, 4.5 + m.EJECT_S / 2, cam, pr))
    m.events(w.objects.t, state, w)
    half = (m.HIDDEN_X + m.EJECTED_X) / 2
    assert w.set["print"][0][0] == pytest.approx(m.to_world(cam, (half, 0, m.PRINT_Z))[0], abs=1e-6) and state["out_t"] is None
    w = World(snap(m, 4.5 + m.EJECT_S, cam, pr))
    m.events(w.objects.t, state, w)
    assert state["out_t"] == pytest.approx(4.5 + m.EJECT_S)
    assert w.set["print"][0][0] == pytest.approx(m.to_world(cam, (m.EJECTED_X, 0, m.PRINT_Z))[0], abs=1e-6)
    # out of the slot by the protrusion, the last part still between the rollers
    assert m.EJECTED_X + m.PRINT[0] / 2 - m.FRONT == pytest.approx(m.P["protrude"], abs=1e-4)
    assert m.EJECTED_X - m.PRINT[0] / 2 < m.FRONT


def ejected_state(m, state, out_t=10.0):
    state.update(shot_t=out_t - m.EJECT_S - 1.0, release_t=out_t - m.EJECT_S, out_t=out_t, _came_out=True, depth=m.EJECTED_X)
    return state


# --- the rollers ---------------------------------------------------------------------------------------------------
def test_rollers_hold_an_unpinched_print_wherever_the_camera_goes(m, state):
    ejected_state(m, state)
    tipped = camera(m, origin=(0.25, 0.05, m.TOP + 0.08), quat=quat_y(-90))  # lifted, the lens turned up
    drifted = m.to_world(tipped, (m.EJECTED_X - 0.003, 0.002, m.PRINT_Z + 0.002))  # where physics left it this step
    pr = print_at(m, drifted, tipped.quat_xyzw)
    w = World(snap(m, 12.0, tipped, pr))
    m.events(12.0, state, w)
    want = m.to_world(tipped, (m.EJECTED_X, 0, m.PRINT_Z))
    assert np.allclose(w.set["print"][0], want) and not state["loose"]


def test_pinched_print_is_left_to_the_gripper_and_freed_once_out(m, state):
    ejected_state(m, state)
    cam = camera(m)
    pr = print_in(m, cam, m.EJECTED_X)
    tcp = {"left": PARKED["left"], "right": tuple(pr.pos)}
    pinch = {"left": -0.047, "right": -0.001}
    # squeezed by its two side edges from above (the closing line along its width): the rollers keep it
    w = World(snap(m, 12.0, cam, pr, tcp=tcp, finger=pinch, tcp_rot={"left": DOWN, "right": DOWN}))
    m.events(12.0, state, w)
    assert "print" in w.set and not state["loose"]
    # pinched across its faces: left to the gripper
    w = World(snap(m, 12.05, cam, pr, tcp=tcp, finger=pinch, tcp_rot={"left": DOWN, "right": across(pr)}))
    m.events(12.05, state, w)
    assert "print" not in w.set and not state["loose"]
    out = print_in(m, cam, m.FRONT + m.PRINT[0] / 2 + 0.002)  # drawn clear of the slot
    w = World(snap(m, 13.0, cam, out, tcp={"left": PARKED["left"], "right": tuple(out.pos)}, finger=pinch,
                   tcp_rot={"left": DOWN, "right": across(out)}))
    m.events(13.0, state, w)
    assert state["loose"] and state["freed_t"] == 13.0


def test_rollers_drag_the_camera_while_the_print_is_pulled(m, state):
    ejected_state(m, state)
    cam = camera(m)
    tcp = {"left": PARKED["left"], "right": tuple(print_in(m, cam, m.EJECTED_X).pos)}
    pinch = {"left": -0.047, "right": -0.001}
    for v, want in ((2 * m.RETAIN_V, m.RETAIN_N), (m.RETAIN_V / 2, m.RETAIN_N / 2), (-m.RETAIN_V, 0.0)):
        pr = print_at(m, print_in(m, cam, m.EJECTED_X).pos, vel=(v, 0.0, 0.0))
        rot = {"left": DOWN, "right": across(pr)}
        f = m.forces(snap(m, 12.0, cam, pr, tcp=tcp, finger=pinch, tcp_rot=rot), state)
        assert f["camera"] == pytest.approx((want, 0.0, 0.0), abs=1e-9) and f["print"] == (0.0, 0.0, 0.0)  # the hand has it
    unpinched = print_at(m, print_in(m, cam, m.EJECTED_X).pos, vel=(0.05, 0.0, 0.0))
    f = m.forces(snap(m, 12.0, cam, unpinched), state)  # in the rollers' hold: no drag, its weight carried
    assert f["camera"] == (0.0, 0.0, 0.0) and f["print"] == pytest.approx((0.0, 0.0, m.P["mass"] * m.GRAVITY))
    state["loose"] = True  # drawn free: nothing on either, every step (a pushed force stays until the next push)
    assert m.forces(snap(m, 14.0, cam, unpinched), state) == {"camera": (0.0, 0.0, 0.0), "print": (0.0, 0.0, 0.0)}


def test_unheld_camera_slides_under_the_drag(m):
    """The README's claim: the drag (1.5 N) is more than the friction that keeps the camera in place."""
    weight = m.C["mass"] * 9.81
    assert m.RETAIN_N > m.C["foot_friction"][0] * weight > m.C["foot_friction"][1] * weight


# --- the end -------------------------------------------------------------------------------------------------------


def ended(m, state, freed_after, print_quat=FACE_DOWN, cam_quat=(0, 0, 0, 1.0), print_xy=(0.36, -0.04), tcp=None):
    ejected_state(m, state)
    state.update(loose=True, freed_t=state["out_t"] + freed_after)
    cam = camera(m, quat=cam_quat)
    pr = print_at(m, (print_xy[0], print_xy[1], m.TOP + m.PRINT[2] / 2), print_quat)
    return snap(m, 40.0, cam, pr, tcp=tcp)


def test_full_score_end(m, state):
    s = ended(m, state, freed_after=m.SHIELD_S + 0.1)
    assert m.milestones(s, state) == [True, True, True, True] and m.success(s, state) and m.failure(s, state) is None


def test_drawn_out_before_the_shield_wait_stops_at_three_quarters(m, state):
    s = ended(m, state, freed_after=m.SHIELD_S - 0.2)
    assert m.milestones(s, state) == [True, True, True, False]


def test_face_up_or_on_edge_is_not_face_down(m, state):
    assert m.milestones(ended(m, state, 6.0, print_quat=(0, 0, 0, 1.0)), state) == [True, True, False, False]
    a = math.radians(m.FACE_DOWN_DEG + 5) / 2  # face down but tilted past the limit
    tilted = (math.cos(a), 0.0, math.sin(a), 0.0)
    assert m.milestones(ended(m, state, 6.0, print_quat=tilted), state)[2] is False


def test_hand_near_the_print_or_camera_is_not_done(m, state):
    near_print = {"left": PARKED["left"], "right": (0.36, -0.04, m.TOP + m.PRINT[2] + 0.03)}
    assert m.milestones(ended(m, state, 6.0, tcp=near_print), state)[2] is False
    cam_z = m.TOP + m.CAM_SIZE[2] + 0.03
    near_camera = {"left": (m.CAM_XY[0], m.CAM_XY[1], cam_z), "right": PARKED["right"]}
    assert m.milestones(ended(m, state, 6.0, tcp=near_camera), state) == [True, True, True, False]


def test_camera_left_tipped_is_not_standing_and_on_its_side_ends_the_trial(m, state):
    assert m.milestones(ended(m, state, 6.0, cam_quat=quat_y(m.LEVEL_DEG + 5)), state)[3] is False
    s = ended(m, state, 6.0, cam_quat=quat_y(-90))
    s["camera"] = camera(m, origin=(m.CAM_XY[0], m.CAM_XY[1], m.TOP + 0.0529), quat=quat_y(-90))  # lying on its back
    assert m.failure(s, state) == "camera_knocked_over"


def test_off_the_bench(m, state):
    s = ended(m, state, 6.0)
    s["print"] = print_at(m, (0.36, -0.04, m.TOP - 0.2))
    assert m.failure(s, state) == "dropped_off_bench"


# --- the score over a trial: the per-step bookkeeping yam_memory runs (rta.spec.MemoryTask.score_step) -------------
HZ = 20.0


def run_steps(task, rows):
    """Feed (milestones() flags, success()) per control step through the embodiment's own bookkeeping; returns
    (q_score, success) after each step."""
    flags, held, out = [False] * len(task.milestone_names), 0, []
    for now, ok in rows:
        flags, held, success, q = task.score_step(flags, now, held, ok, HZ)
        out.append((q, success))
    return out


def test_events_latch_end_states_are_read_every_step(task):
    assert task.latched == (True, True, False, False) and task.memory is False
    # the photo and the print drawn free stay counted after the print is put back down and picked up again
    assert run_steps(task, [([1, 1, 0, 0], False), ([1, 0, 0, 0], False)])[-1] == (0.5, False)


def test_end_held_one_second_then_the_camera_knocked_over_is_not_full(task):
    end = ([1, 1, 1, 1], True)  # print face down, camera standing, hands off, the wait kept
    rows = [end] * round(1.0 * HZ) + [([1, 1, 1, 0], False)] * 10  # 1 s, then the camera falls over
    qs = [q for q, _ in run_steps(task, rows)]
    assert max(qs) == 0.75 and qs[-1] == 0.75  # never 1.0 before the 3 s hold


def test_print_flipped_face_up_after_three_quarters_scores_half(task):
    rows = [([1, 1, 1, 0], False)] * 10 + [([1, 1, 0, 0], False)] * 5
    out = run_steps(task, rows)
    assert out[9][0] == 0.75 and out[-1][0] == 0.5


def test_full_score_only_after_the_three_second_hold(task):
    hold = round(task.success_hold_s * HZ)
    out = run_steps(task, [([1, 1, 1, 1], True)] * hold)
    assert all(r == (0.75, False) for r in out[:-1]) and out[-1] == (1.0, True)
