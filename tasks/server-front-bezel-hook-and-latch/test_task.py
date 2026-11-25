"""Offline geometry, scoring and full-rate reach checks for the 1U bezel task."""

import math
import os
import sys
from collections import deque
from pathlib import Path

import numpy as np
import pytest

if os.environ.get("YAM_ARENA"):
    sys.path.insert(0, os.path.expanduser(os.environ["YAM_ARENA"]))

from yam_arena import ik, station  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402
from yam_arena.task_file import ObjectState, load  # noqa: E402

TASK = load(Path(__file__).with_name("task.py"))
M = TASK.module


@pytest.fixture(autouse=True)
def fresh_score():
    M._STATE.update(last_t=None, fault=False, history=deque(), step=0, logged_t=None)


def state(pos, yaw=0.0, vel=(0, 0, 0)):
    p = np.asarray(pos, float)
    q = np.array([0, 0, math.sin(yaw / 2), math.cos(yaw / 2)])
    h = np.array([0.0125, 0.2235, 0.0205])
    return ObjectState(p, q, np.asarray(vel, float), np.zeros(3), p - h, p + h)


def goal(yaw=0.0, dx=0.0, dy=0.0, latch_compression=0.0):
    # Like the sim, the bezel's pos is its rotated local bounds centre, not its origin p.
    p = np.array([M.BEZEL_FINAL_X + dx, dy, M.TOP + M.BEZEL_HEIGHT / 2])
    r = M._rot(state(p, yaw))
    b = state(p + r @ M.BEZEL_BOUNDS_C, yaw)
    local = np.asarray(M.LATCH_LOCAL) - np.array([0, latch_compression, 0])
    l = state(p + r @ local, yaw)
    return {"bezel": b, "latch": l}


def test_task_loads_with_real_geometry_and_spring():
    assert TASK.scene == "station" and TASK.score_over == "end"
    assert len(TASK.objects) == 4
    assert M.SERVER_WIDTH == 0.4826 and M.SERVER_HEIGHT == 0.0432
    assert M.BEZEL_WIDTH == 0.430 and M.BEZEL_HEIGHT == 0.041
    assert M.BEZEL_DEPTH == 0.025 and M.BEZEL_MASS == 0.350
    assert M.LATCH_JOINT.parent == "bezel"
    assert M.LATCH_JOINT.stiffness == 350.0 and M.LATCH_JOINT.target == 0.0
    assert M.SLOT_HEIGHT - M.TAB_SIZE[2] >= M.TAB_FREE_PLAY_MM / 1000
    assert abs(M.SLOT_OUTER_Y - M.SLOT_INNER_Y) - M.TAB_SIZE[1] >= 0.003
    assert len([p for p in M.bezel_pieces if p.size[1] >= M.BEZEL_WIDTH]) >= 3
    assert all(type(v) is float for o in TASK.objects for v in o.pos)


def test_every_demo_step_reachable_at_20_hz():
    assert set(TASK.demo()) == {"left", "right"}
    for side, waypoints in TASK.demo().items():
        plan = ArmPlan(waypoints, station.BASES[side])
        assert plan.duration >= M.FINAL_REST_S + 15.0
        for k in range(round(plan.duration * M.CONTROL_HZ)):
            action, _ = plan.action7((k + 1) / M.CONTROL_HZ)
            q = action[:6]
            assert np.all(q > ik.LOWER + 1e-5), (side, k, q)
            assert np.all(q < ik.UPPER - 1e-5), (side, k, q)
        assert plan.worst_ik_mm < 0.5, (side, plan.worst_ik_mm)
        assert plan.worst_ik_deg < 0.1, (side, plan.worst_ik_deg)
        print(f"REACH side={side} steps={round(plan.duration * M.CONTROL_HZ)} "
              f"duration_s={plan.duration:.2f} worst_ik_mm={plan.worst_ik_mm:.6f} "
              f"worst_ik_deg={plan.worst_ik_deg:.6f}")
        assert all(FINGER_OPEN <= w.finger <= FINGER_CLOSED for w in waypoints)
        assert any(w.label == "pinch_upper_edge" for w in waypoints)


def test_slots_tabs_ramp_and_spawn_separation():
    # 10+ mm tab width, 6 mm thickness, and >=3 mm total vertical free play.
    assert M.TAB_SIZE[0] >= 0.010 and M.TAB_SIZE[2] == 0.006
    assert M.SLOT_HEIGHT - M.TAB_SIZE[2] >= 0.003
    assert M.CATCH_END_X - 0.391 >= 0.003
    assert M.BEZEL_START_X + M.BEZEL_DEPTH / 2 < 0.391
    assert M.EAR_INNER_Y > M.BEZEL_WIDTH / 2
    assert 0.015 <= M.BEZEL_DEPTH <= 2 * abs(FINGER_OPEN) - 0.01
    assert M.BEZEL_START_X + M.BEZEL_DEPTH / 2 < M.EAR_FACE_X


def test_goal_and_partial_scores():
    g = goal()
    assert M.tabs_seated(g["bezel"])
    assert M.latch_travel_mm(g) == pytest.approx(0.0, abs=1e-8)
    assert M.catch_overlap_mm(g) >= M.LATCH_OVERLAP_MM
    assert M.flush_mm(g["bezel"]) == pytest.approx(0.0)
    assert M.score(g, tcps=[], force_n=0.0) == 1.0
    assert M.success(g, tcps=[], force_n=0.0)
    no_latch = goal(latch_compression=0.010)
    assert M.score(no_latch, tcps=[], force_n=0.0) == 0.5
    open_angle = math.radians(12)
    # Swung open about the seated tab centre, as a hinge.
    c, s = math.cos(open_angle), math.sin(open_angle)
    tl = np.array([M.TAB_X_LOCAL, M.TAB_Y])
    o = np.array([M.BEZEL_FINAL_X, 0]) + tl - np.array([[c, -s], [s, c]]) @ tl
    tab_only = goal(yaw=open_angle, dx=o[0] - M.BEZEL_FINAL_X, dy=o[1])
    assert M.tabs_seated(tab_only["bezel"])
    assert M.score(tab_only, tcps=[], force_n=0.0) == 0.25
    flush_only = goal(dy=0.020)
    assert M.score(flush_only, tcps=[], force_n=0.0) == 0.25


def test_shortcuts_and_faults_score_zero():
    g = goal()
    assert M.score(g, tcps=[np.asarray(g["bezel"].pos)], force_n=0.0) == 0.0
    assert not M.success(g, tcps=[np.asarray(g["bezel"].pos)], force_n=0.0)
    M._STATE["fault"] = False
    assert M.score(g, tcps=[], force_n=M.FORCE_FAULT_N + 0.1) == 0.0
    assert M.score(g, tcps=[], force_n=0.0) == 0.0  # latched fault
    M._STATE["fault"] = False
    wrong = goal(dx=-0.10)
    assert M.score(wrong, tcps=[], force_n=0.0) == 0.0
    moving = goal()
    moving["bezel"] = state(moving["bezel"].pos, vel=(0.2, 0, 0))
    assert not M.success(moving, tcps=[], force_n=0.0)


@pytest.mark.parametrize("compression", [0.0, 0.0015, 0.0021, 0.007, 0.014])
@pytest.mark.parametrize("angles", [(0, 0, 0), (0.2, -0.1, 0.8), (-0.3, 0.1, -1.2)])
def test_pose_latch_travel_is_joint_axis_projection(compression, angles):
    from yam_arena.urdf import axis_angle_matrix

    r = np.eye(3)
    for axis, angle in zip(np.eye(3), angles):
        r = r @ axis_angle_matrix(axis, angle)
    origin = np.array((0.31, -0.04, M.TOP + 0.03))
    b = state(origin + r @ M.BEZEL_BOUNDS_C)
    b.quat_xyzw[:] = ik.matrix_to_quat_xyzw(r)
    local = np.array([M.LATCH_LOCAL[0], M.LATCH_LOCAL[1] - compression,
                      M.LATCH_LOCAL[2] - M.BEZEL_HEIGHT / 2])
    latch = state(origin + r @ local)
    assert M.latch_travel_mm({"bezel": b, "latch": latch}) == pytest.approx(compression * 1000, abs=1e-9)


def test_swing_speed_and_minimal_right_release():
    plans = M.demo()
    right = next(w for w in plans["right"] if w.label == "release_seated_tabs")
    assert right.finger == -0.015
    assert 2 * abs(right.finger) - M.BEZEL_DEPTH == pytest.approx(0.005)
    left = plans["left"]
    for prev, w in zip(left, left[1:]):
        if w.label in ("insert_right_tabs", "swing_in_press_latch_ramp"):
            peak_speed = 1.5 * np.linalg.norm(np.array(w.pos) - prev.pos) / w.duration
            print(f"CONTACT_SPEED label={w.label} peak_m_s={peak_speed:.6f}")
            assert peak_speed <= 0.05
    assert M.GRIP_Y <= M.BEZEL_WIDTH / 2 - 0.015
    assert max(sum(w.duration for w in p) for p in plans.values()) <= 27.0


def test_latch_meets_ramp_slope_and_fits_ear_window():
    from robocurve_ext import read_obj

    v = np.array(read_obj(str(Path(__file__).with_name("catch_ramp.obj")))[0]) + M.CATCH_PART.pos - [0, 0, M.TOP]
    lead = v[v[:, 0] == v[:, 0].min()]
    latch_outer = M.LATCH_LOCAL[1] + M.LATCH_SIZE[1] / 2
    # Lead-in is a knife edge outside the extended latch: the nose meets the slope, never a square face.
    assert np.ptp(lead[:, 1]) == pytest.approx(0) and lead[0, 1] - latch_outer >= 0.0015
    back_inner = v[v[:, 0] == v[:, 0].max()][:, 1].min()
    assert latch_outer - back_inner <= M.LATCH_STROKE - 0.002
    # Flush and extended, the latch box overlaps no steel piece.
    lo = np.array([M.BEZEL_FINAL_X + M.LATCH_LOCAL[0], M.LATCH_LOCAL[1], M.LATCH_LOCAL[2]]) - np.array(M.LATCH_SIZE) / 2
    hi = lo + M.LATCH_SIZE
    for p in M.server_pieces:
        c = np.array(M.SERVER_C) + p.offset
        assert np.any(lo >= c + np.array(p.size) / 2 - 1e-9) or np.any(hi <= c - np.array(p.size) / 2 + 1e-9), p
    assert lo[0] >= M.CATCH_END_X + 0.001


def test_scorer_reads_sim_bounds_centre_not_origin():
    # Values measured on the GPU at spawn: origin x 0.255, pos (0.2575, -0.0085) before the tab fix.
    assert M.BEZEL_BOUNDS_C[1] == pytest.approx(-0.0085)
    assert M._origin(goal()["bezel"]) == pytest.approx([M.BEZEL_FINAL_X, 0, M.TOP + M.BEZEL_HEIGHT / 2])
    assert M.flush_mm(goal()["bezel"]) == pytest.approx(0.0)
    # A latch held 7 mm in still overlaps the catch, but has not returned: no latch credit.
    held_in = goal(latch_compression=0.007)
    assert M.latch_travel_mm(held_in) == pytest.approx(7.0)
    assert M.catch_overlap_mm(held_in) >= M.LATCH_OVERLAP_MM
    assert M.score(held_in, tcps=[], force_n=0.0) == 0.5


def test_flush_bezel_has_side_play_and_tabs_clear_the_panel():
    gap = 2 * M.EAR_INNER_Y - M.BEZEL_WIDTH
    assert gap >= 0.002  # >= 1 mm each end
    panel = M.server_pieces[0]
    panel_x0 = M.SERVER_C[0] + panel.offset[0] - panel.size[0] / 2
    # Flush, every bezel piece (tabs included) stops in front of the steel panel, so only the panel face limits depth.
    assert all(M.BEZEL_FINAL_X + p.offset[0] + p.size[0] / 2 < panel_x0 for p in M.bezel_pieces)
    # Shifted to either end of its play, both tabs stay between the slot walls.
    for shift in (-gap / 2, gap / 2):
        assert M.TAB_Y + shift - M.TAB_SIZE[1] / 2 > M.SLOT_OUTER_Y
        assert M.tabs_seated(goal(dy=shift)["bezel"])


def test_tilted_bezel_front_corner_enters_between_the_ears():
    # Inserting at OPEN_YAW about the tab centre swings the right front corner outboard; it must still pass the ear.
    th = M.OPEN_YAW
    d = np.array([-M.BEZEL_DEPTH / 2 - M.TAB_X_LOCAL, -M.BEZEL_WIDTH / 2 - M.TAB_Y])
    corner_y = M.TAB_Y + d[0] * math.sin(th) + d[1] * math.cos(th)
    assert corner_y >= -M.EAR_INNER_Y + 0.0005  # GPU run: the real pivot sat a further 0.8 mm inboard
