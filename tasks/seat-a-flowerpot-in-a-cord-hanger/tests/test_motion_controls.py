from types import SimpleNamespace

import numpy as np
import pytest

from rdb_planter import fixed_scene as scene
from rdb_planter.reference import Reference


@pytest.mark.parametrize(("variant", "parked"), [
    ("idle", slice(0, 14)),
    ("left_only", slice(7, 14)),
    ("right_only", slice(0, 7)),
])
def test_control_holds_reset_command_for_entire_episode(variant, parked):
    rig = SimpleNamespace(
        poses={"pot": (scene.POT.copy(), np.eye(3)),
               "bottom_knot": (scene.HOOK + [0, 0, -.405], np.eye(3))},
        final_grade={},
        tick=0,
    )
    policy = Reference(rig, variant)
    policy.reset(None)
    start = np.linspace(.05, .35, 14)
    measured = start.copy()
    moved = False
    for tick in range(scene.MAX_STEPS):
        rig.tick = tick
        observation = SimpleNamespace(state={"joint_pos": measured})
        command = policy.act(observation).actions[0].data
        np.testing.assert_array_equal(command[parked], start[parked])
        moved |= bool(np.any(np.abs(command - start) > .01))
        measured = command.copy()
        # Drift in measured joints must not become a new parking command.
        measured[parked] += .002
    assert moved == (variant != "idle")

    policy.reset(None)
    new_start = start + .03
    first = policy.act(SimpleNamespace(state={"joint_pos": new_start})).actions[0].data
    np.testing.assert_array_equal(first[parked], new_start[parked])
