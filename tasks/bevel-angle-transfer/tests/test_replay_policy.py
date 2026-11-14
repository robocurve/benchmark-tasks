# SPDX-License-Identifier: MIT
from importlib.resources import files
import json
from types import SimpleNamespace

import numpy as np
import pytest


pytest.importorskip("yam_arena")
from bevel_angle_transfer.replay_policy import ACTION_LABELS, CONTROL_HZ, _load_actions, make_demo_policy


@pytest.fixture
def program():
    return files("bevel_angle_transfer").joinpath("data", "fixed_demo_actions.json").read_bytes()


def test_frozen_program_covers_the_recorded_ticks(program):
    actions = _load_actions(program)
    assert actions.shape == (1897, 14)
    source = json.loads(program)["source"]
    assert source["action_log_sha256"] == "49b6d98f63c5a48d29258604da37b7629498fc13aa2dcc892d208537fa8dd0a5"
    assert source["task_source_revision"] == "5344030a5c7e856388f7b144e1f04681a4c5ff6560d2ff6a06e2db5ad18129dc"
    with pytest.raises(ValueError):
        actions.flags.writeable = True


@pytest.mark.parametrize(
    ("field", "invalid", "error"),
    [
        ("schema", None, "schema"),
        ("control_hz", 10, "20 Hz"),
        ("labels", list(reversed(ACTION_LABELS)), "channel order"),
        ("sample_count", 0, "action count"),
        ("actions", [[0.0] * 13], "14-D numeric"),
        ("actions", [[False] + [0.0] * 13], "14-D numeric"),
        ("actions", [[float("nan")] + [0.0] * 13], "finite"),
        ("actions", [[10.0] + [0.0] * 13], "limits"),
        ("actions", [[0.0] * 13 + [1.1]], "limits"),
    ],
)
def test_rejects_invalid_program_data(field, invalid, error):
    document = {
        "schema": "bevel-angle-transfer-fixed-demo-v1",
        "control_hz": CONTROL_HZ,
        "labels": list(ACTION_LABELS),
        "sample_count": 1,
        "actions": [[0.0] * 14],
    }
    document[field] = invalid
    with pytest.raises(ValueError, match=error):
        _load_actions(json.dumps(document).encode())


class UnreadableObservation:
    def __getattribute__(self, name):
        raise AssertionError(f"fixed demo read observation or scene field {name}")


def test_replays_every_tick_and_resets_without_reading_observations(program):
    pytest.importorskip("inspect_robots_yam.config")
    policy = make_demo_policy()
    policy.bind(SimpleNamespace(control_hz=20))
    with pytest.raises(ValueError, match="20 Hz"):
        policy.bind(SimpleNamespace(control_hz=19.999))
    with pytest.raises(ValueError, match="20 Hz"):
        policy.bind(SimpleNamespace(control_hz=None))

    unreadable = UnreadableObservation()
    actions = _load_actions(program)
    for _ in range(2):
        policy.reset(unreadable)
        for step, expected in enumerate(actions):
            chunk = policy.act(unreadable)
            assert chunk.control_hz == 20
            (action,) = chunk.actions
            np.testing.assert_array_equal(action.data, expected)
            assert action.meta.get("request_stop", False) is (step == len(actions) - 1)
            action.data[0] = 0.0

        (held,) = policy.act(unreadable).actions
        np.testing.assert_array_equal(held.data, actions[-1])
        assert held.meta == {"request_stop": True, "stop_reason": "fixed_demo_motion_complete"}
