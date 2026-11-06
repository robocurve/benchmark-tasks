from math import pi, radians

import numpy as np
from yam_arena.urdf import axis_angle_matrix

import scoring
import task

CLEAR = {key: True for key in ("fixture", "cradle", "tray")}


def pose(xy, z, yaw=0., tilt=0., *, speed=0.):
    rot = axis_angle_matrix(np.array([0., 0., 1.]), yaw)
    rot = rot @ axis_angle_matrix(np.array([1., 0., 0.]), tilt)
    return scoring.Pose(np.array([*xy, task.TOP + z]), rot, speed)


def reset():
    return {
        "original_guide": pose(task.FIX, task.FLOOR),
        "good_spare": pose(task.GOOD, 0),
        "blocked_spare": pose(task.BAD, 0),
        "test_part": pose(task.CRADLE, .006, pi),
    }


def observe(repair, poses, xy, z, yaw, tilt=0, clear=CLEAR, steps=20):
    poses["test_part"] = pose(xy, z, yaw, tilt)
    for _ in range(steps):
        score = repair.update(poses, clear)
    return score


def exchange(poses, spare):
    poses["original_guide"] = pose((task.REJECT[0] - .015, task.REJECT[1]), .006)
    poses[spare] = pose(task.FIX, task.FLOOR)


def test_blind_swap_cannot_earn_credit():
    repair, poses = scoring.Repair(20), reset()
    exchange(poses, "good_spare")
    assert observe(repair, poses, task.FIX, task.SEAT, 0) == 0
    assert observe(repair, poses, task.FIX, task.SEAT + task.BLOCK, pi) == 0
    assert observe(repair, poses, task.CRADLE, .006, pi) == 0


def test_rejects_jams_assistance_and_stops_that_block_both():
    repair, poses = scoring.Repair(20), reset()
    assert observe(repair, poses, task.FIX, task.SEAT, pi, steps=19) == 0
    assert repair.update(poses, CLEAR) == .25
    exchange(poses, "blocked_spare")
    assert observe(repair, poses, task.FIX, task.SEAT + task.BLOCK, pi) == .5
    assert observe(repair, poses, task.FIX, task.SEAT + task.BLOCK, 0) == .5
    assert repair.verified == set()
    poses["blocked_spare"] = pose(task.BAD, 0)
    exchange(poses, "good_spare")
    assert observe(repair, poses, task.FIX, task.SEAT, 0) == .5
    assert observe(repair, poses, task.FIX, task.SEAT + task.BLOCK, pi, radians(7)) == .5
    assert observe(repair, poses, task.FIX, task.SEAT + task.BLOCK, pi,
                   clear={**CLEAR, "fixture": False}) == .5
    assert observe(repair, poses, task.FIX, task.SEAT + task.BLOCK + .004, pi) == .5
    assert observe(repair, poses, (task.FIX[0] + .009, task.FIX[1]), task.FLOOR + task.BLOCK, pi) == .5


def test_repair_and_retest_in_either_order_then_return_part():
    for reverse_first in (False, True):
        repair, poses = scoring.Repair(20), reset()
        assert observe(repair, poses, task.FIX, task.SEAT, pi) == .25
        exchange(poses, "good_spare")
        cases = [(task.SEAT, 0), (task.SEAT + task.BLOCK, pi)]
        if reverse_first:
            cases.reverse()
        for index, (depth, yaw) in enumerate(cases):
            assert observe(repair, poses, task.FIX, depth, yaw) == .5 + .25 * index
        assert observe(repair, poses, task.CRADLE, .006, pi, steps=19) == .75
        assert repair.update(poses, CLEAR) == 1.


def test_removing_spare_clears_verification_and_fall_disqualifies():
    repair, poses = scoring.Repair(20), reset()
    observe(repair, poses, task.FIX, task.SEAT, pi)
    exchange(poses, "good_spare")
    observe(repair, poses, task.FIX, task.SEAT, 0)
    assert observe(repair, poses, task.FIX, task.SEAT + task.BLOCK, pi) == .75
    poses["good_spare"] = pose(task.GOOD, 0)
    assert repair.update(poses, CLEAR) == .5
    exchange(poses, "good_spare")
    assert observe(repair, poses, task.CRADLE, .006, pi) == .5
    poses["blocked_spare"] = pose(task.BAD, -.1)
    assert repair.update(poses, CLEAR) == 0
    poses["blocked_spare"] = pose(task.BAD, 0)
    assert repair.update(poses, CLEAR) == 0


def test_small_rock_within_nest_is_not_a_guide_removal():
    repair, poses = scoring.Repair(20), reset()
    observe(repair, poses, task.FIX, task.SEAT, pi)
    exchange(poses, "good_spare")
    observe(repair, poses, task.FIX, task.SEAT, 0)
    poses["good_spare"] = pose(task.FIX, task.FLOOR + .0025, tilt=radians(7))
    assert observe(repair, poses, task.CRADLE, .006, pi) == .5
    assert repair.verified == {("aligned", "seated")}
    poses["good_spare"] = pose(task.FIX, task.FLOOR)
    assert observe(repair, poses, task.FIX, task.SEAT + task.BLOCK, pi) == .75
