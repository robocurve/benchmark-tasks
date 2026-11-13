from math import pi, radians

import numpy as np
from yam_arena.urdf import axis_angle_matrix

import scoring
import task

CLEAR = {key: True for key in ("fixture", "cradle", "tray")}
TRAY = (task.REJECT[0] - .0015, task.REJECT[1])


def pose(xy, z, yaw=0., tilt=0., *, speed=0.):
    rot = axis_angle_matrix(np.array([0., 0., 1.]), yaw)
    rot = rot @ axis_angle_matrix(np.array([1., 0., 0.]), tilt)
    return scoring.Pose(np.array([*xy, task.TOP + z]), rot, speed)


def reset():
    return {
        "original_guide": pose(task.FIX, task.FLOOR),
        "matching_spare": pose(task.MATCH, 0),
        "wrong_spare": pose(task.WRONG, 0),
        "lock_pin": pose(task.PIN_HOLE, task.TAB_TOP),
        "test_part": pose(task.CRADLE, .006, pi),
    }


def observe(repair, poses, xy, z, yaw, tilt=0, clear=CLEAR, steps=20):
    poses["test_part"] = pose(xy, z, yaw, tilt)
    for _ in range(steps):
        score = repair.update(poses, clear)
    return score


def exchange(poses, spare, *, pinned=True):
    poses["original_guide"] = pose(TRAY, .006)
    poses[spare] = pose(task.FIX, task.FLOOR)
    poses["lock_pin"] = pose(task.PIN_HOLE, task.TAB_TOP) if pinned else pose(task.PARK, task.PARK_TOP)


def test_reset_layout_has_original_locked_in():
    poses = reset()
    assert scoring.installed(poses) == "original_guide"
    assert not any(scoring.entered(poses[name]) for name in ("matching_spare", "wrong_spare"))
    assert scoring.Repair(20).update(poses, CLEAR) == 0


def test_blind_swap_cannot_earn_credit():
    repair, poses = scoring.Repair(20), reset()
    exchange(poses, "matching_spare")
    assert observe(repair, poses, task.FIX, task.SEAT, 0) == 0
    assert observe(repair, poses, task.FIX, task.SEAT + task.BLOCK, pi) == 0
    assert observe(repair, poses, task.CRADLE, .006, pi) == 0


def test_unpinned_or_unslid_guide_is_not_installed():
    poses = reset()
    poses["lock_pin"] = pose(task.PARK, task.PARK_TOP)
    assert scoring.installed(poses) is None
    poses = reset()
    poses["lock_pin"] = pose(task.PIN_HOLE, task.TAB_TOP + .005)
    assert scoring.installed(poses) is None
    poses = reset()
    poses["original_guide"] = pose((task.FIX[0] - task.ENTRY, task.FIX[1]), task.FLOOR)
    assert scoring.installed(poses) is None


def test_reading_needs_the_pin_and_trying_the_428_spare_caps_score():
    repair, poses = scoring.Repair(20), reset()
    assert observe(repair, poses, task.FIX, task.SEAT, pi) == .25
    exchange(poses, "matching_spare", pinned=False)
    assert observe(repair, poses, task.FIX, task.SEAT, 0) == .25
    # Dropping the 428 spare into the slot, even briefly, is trial and error.
    poses["wrong_spare"] = pose((task.FIX[0] - task.ENTRY, task.FIX[1]), task.FLOOR)
    repair.update(poses, CLEAR)
    poses["wrong_spare"] = pose(task.WRONG, 0)
    exchange(poses, "matching_spare")
    assert observe(repair, poses, task.FIX, task.SEAT, 0) == .25
    assert observe(repair, poses, task.FIX, task.SEAT + task.BLOCK, pi) == .25
    assert observe(repair, poses, task.CRADLE, .006, pi) == .25


def test_428_spare_locked_in_earns_nothing_more():
    repair, poses = scoring.Repair(20), reset()
    observe(repair, poses, task.FIX, task.SEAT, pi)
    exchange(poses, "wrong_spare")
    assert scoring.installed(poses) == "wrong_spare"
    assert observe(repair, poses, task.FIX, task.SEAT, pi) == .25
    assert observe(repair, poses, task.FIX, task.SEAT + task.BLOCK, 0) == .25


def test_rejects_jams_assistance_and_off_centre_parts():
    repair, poses = scoring.Repair(20), reset()
    assert observe(repair, poses, task.FIX, task.SEAT, pi, steps=19) == 0
    assert repair.update(poses, CLEAR) == .25
    exchange(poses, "matching_spare")
    assert observe(repair, poses, task.FIX, task.SEAT, 0) == .5
    assert observe(repair, poses, task.FIX, task.SEAT + task.BLOCK, pi, radians(7)) == .5
    assert observe(repair, poses, task.FIX, task.SEAT + task.BLOCK, pi,
                   clear={**CLEAR, "fixture": False}) == .5
    assert observe(repair, poses, task.FIX, task.SEAT + task.BLOCK + .004, pi) == .5
    assert observe(repair, poses, (task.FIX[0] + .009, task.FIX[1]), task.SEAT + task.BLOCK, pi) == .5


def test_repair_and_retest_in_either_order_then_return_part():
    for reverse_first in (False, True):
        repair, poses = scoring.Repair(20), reset()
        assert observe(repair, poses, task.FIX, task.SEAT, pi) == .25
        exchange(poses, "matching_spare")
        cases = [(task.SEAT, 0), (task.SEAT + task.BLOCK, pi)]
        if reverse_first:
            cases.reverse()
        for index, (depth, yaw) in enumerate(cases):
            assert observe(repair, poses, task.FIX, depth, yaw) == .5 + .25 * index
        assert observe(repair, poses, task.CRADLE, .006, pi, steps=19) == .75
        assert repair.update(poses, CLEAR) == 1.


def test_pulling_pin_clears_verification_and_fall_disqualifies():
    repair, poses = scoring.Repair(20), reset()
    observe(repair, poses, task.FIX, task.SEAT, pi)
    exchange(poses, "matching_spare")
    observe(repair, poses, task.FIX, task.SEAT, 0)
    assert observe(repair, poses, task.FIX, task.SEAT + task.BLOCK, pi) == .75
    poses["lock_pin"] = pose(task.PARK, task.PARK_TOP)
    assert repair.update(poses, CLEAR) == .5
    exchange(poses, "matching_spare")
    assert observe(repair, poses, task.CRADLE, .006, pi) == .5
    poses["wrong_spare"] = pose(task.WRONG, -.1)
    assert repair.update(poses, CLEAR) == 0
    poses["wrong_spare"] = pose(task.WRONG, 0)
    assert repair.update(poses, CLEAR) == 0


def test_settling_within_the_slot_keeps_evidence():
    repair, poses = scoring.Repair(20), reset()
    observe(repair, poses, task.FIX, task.SEAT, pi)
    exchange(poses, "matching_spare")
    observe(repair, poses, task.FIX, task.SEAT, 0)
    poses["matching_spare"] = pose((task.FIX[0] - .003, task.FIX[1]), task.FLOOR + .0015)
    assert observe(repair, poses, task.CRADLE, .006, pi) == .5
    assert repair.verified == {("aligned", "seated")}
    poses["matching_spare"] = pose(task.FIX, task.FLOOR)
    assert observe(repair, poses, task.FIX, task.SEAT + task.BLOCK, pi) == .75


def test_original_fits_tray_bounds():
    assert scoring.in_reject(pose(TRAY, .006))
