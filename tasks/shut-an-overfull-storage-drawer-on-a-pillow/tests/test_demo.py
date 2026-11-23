"""Dry runs of the scripted demonstration in the kinematic world, scored by the real scorer."""

import numpy as np
import pytest

pytest.importorskip("yam_arena")
pytest.importorskip("scipy")

from dryrun_demo import dry_run  # noqa: E402
from rdb_overfull_drawer import fixed_scene as fixture  # noqa: E402
from rdb_overfull_drawer import layout as L  # noqa: E402
from rdb_overfull_drawer.grading import DrawerGrade  # noqa: E402
from world import World  # noqa: E402


def test_demonstration_reaches_full_score_within_the_limit():
    report, reference = dry_run("full")
    assert report["score"] == 1. and report["steps"] < fixture.MAX_STEPS / 2
    assert report["cycles"] == [(3, True, True), (2, True, True), (1, True, True)]
    assert report["worst_limit_margin_deg"] > 15. and report["worst_ik_mm"] < .5
    assert reference.phases[-1][1] == "wait"


@pytest.mark.parametrize("options", [
    dict(lag_s=.25, arm_stiffness=250., press_offset=(.005, 0., -.006), slack=-.003),
    dict(lag_s=.08, arm_stiffness=1500., press_offset=(-.005, 0., .008), push_offset=-.006, slack=.003),
])
def test_demonstration_tolerates_arm_and_pillow_errors(options):
    report, _ = dry_run("full", **options)
    assert report["score"] == 1.


@pytest.mark.parametrize("variant", ["no_press", "no_hold"])
def test_controls_stay_caught_on_the_frame(variant):
    report, _ = dry_run(variant, max_steps=700)
    assert report["peak_score"] == 0. and report["open_mm"] > 210.


def test_world_drawer_cannot_be_slammed_past_the_pillow():
    from yam_arena import ik
    from rdb_overfull_drawer.scripted import PUSH, PUSH_Y, PUSH_Z, DrawerReference

    reference = DrawerReference()
    world = World(arm_stiffness=5000.)
    # the pushing hand is driven straight to where the shut drawer's front would be
    joints = reference.solve(PUSH, (L.RAIL_X, PUSH_Y, PUSH_Z), seed=np.array((.5169, 1.6483, .9807, -.2729, .3233, .4107)), iters=400)
    command = world.joints.copy()
    command[7:13], command[13] = joints, 0.
    grade = DrawerGrade()
    for _ in range(100):
        world.step(command)
        result = grade.update(*world.grade_inputs())
    assert abs(world.open - L.blocking_open(3)) < .004 and result["score"] == 0.
    assert np.all(np.minimum(joints - ik.LOWER, ik.UPPER - joints) > 0)
