"""The converter's numbers: published figures reproduced, forces within what one YAM arm delivers, lock geometry."""

import math

import numpy as np
import pytest

from rdb_desk_converter import layout as L

YAM_PUSH_N = 15.            # what one arm may be asked to push at the hand


def test_published_figures():
    assert L.WIDTH == pytest.approx(.762, abs=1e-3) and L.DEPTH == pytest.approx(.7557, abs=1e-3)
    assert L.SETTINGS == 11 and L.H_LOW == pytest.approx(.1143) and L.H_HIGH == pytest.approx(.4445)


def test_arc_reproduces_rise_and_forward_travel():
    rise = L.surface_height(L.THETA_11) - L.surface_height(L.THETA_1)
    forward = L.top_origin(L.THETA_1)[0] - L.top_origin(L.THETA_11)[0]
    assert rise == pytest.approx(L.RISE, abs=1e-6) and forward == pytest.approx(L.FORWARD, abs=1e-6)
    assert L.surface_height(L.theta(1)) == pytest.approx(L.H_LOW, abs=1e-9)


def test_net_force_per_arm_is_small_over_the_task_range():
    pushes = [L.net_push(L.theta(s)) for s in np.linspace(4, 9.5, 56)]
    assert min(pushes) > 0.                         # the top sinks if let go: nothing springs up by itself
    assert max(pushes) / 2 < YAM_PUSH_N / 2         # under half of an arm's push, per arm
    assert L.net_push(L.theta(2)) < 0.              # the spring outweighs the load near the bottom (reviews)


def test_grasp_points_inside_the_reach_box():
    for s in np.linspace(4, 9.6, 30):
        for side in ("left", "right"):
            p = L.grasp_point(side, L.theta(s))
            assert .2 <= p[0] <= .6 and abs(p[1]) <= .45 and p[2] >= .14


def test_lock_geometry():
    assert L.PIN_R[0] - L.PADDLE_TRAVEL < L.NOTCH_R[0]                 # a squeezed pin is clear of the teeth
    assert L.PIN_R[1] - L.PIN_R[0] >= .008                             # engaged at least 8 mm
    spacing = (L.PIN_R[0] + L.PIN_R[1]) / 2 * L.STEP
    assert spacing - L.TOOTH_TANGENT - L.PIN_TANGENT > .002            # slot wider than the pin
    angles = L.notch_angles()
    assert angles[L.START_SETTING - 1] == pytest.approx(-math.pi / 2 - L.SEAT_PLAY, abs=1e-4)   # pin on its low side


def test_seated_pose_matches_isaac_run_1_and_the_reset_window():
    # run 1 (asset authored at the slot centre) settled at lift -0.0117 rad, latches 0, top 3.8 mm lower
    assert L.SEAT_PLAY == pytest.approx(.0117, abs=2e-4)
    assert L.pin_gap(0.) == pytest.approx(.00114, abs=5e-5) and abs(L.pin_gap(-L.SEAT_PLAY)) < 1e-6
    drop = L.surface_height(L.theta(4)) - L.surface_height(L.seated(4))
    assert drop == pytest.approx(.00382, abs=2e-4)
    from rdb_desk_converter import fixed_scene as F
    assert F.START_THETA == pytest.approx(L.seated(4), abs=1e-4)
    # the reset window covers the pins resting on either side of the slot
    for angle in (L.theta(4) - L.SEAT_PLAY, L.theta(4) + L.SEAT_PLAY):
        assert abs(L.setting_of(angle) - L.setting_of(F.START_THETA)) <= F.RESET_SETTING_TOL
        assert np.linalg.norm(L.top_origin(angle) - F.TOP_START) <= F.RESET_TOP_TOL
    # run 1's measured state passes the new check (setting 3.9111, top (641.05, 0, 248.8) mm)
    assert abs(3.9111 - L.setting_of(F.START_THETA)) <= F.RESET_SETTING_TOL
    assert np.linalg.norm(np.array((.64105, 0., .2488)) - F.TOP_START) <= F.RESET_TOP_TOL


def test_mass_split_adds_up_to_the_product():
    total = L.BASE_MASS + L.TOP_MASS + L.ARM_MASS + L.REAR_ARM_MASS + 2 * L.LATCH_MASS
    assert total == pytest.approx(L.PRODUCT_MASS, abs=.3)
    assert L.LOAD_MASS < L.RATED_LOAD
