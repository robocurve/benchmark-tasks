"""Offline failure-path checks; no Dream request, credential access or simulator."""
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

if __package__:
    from .common import table_clearance, tool_vertices
    from .dream_policy import continuous_client_class, public_transcript
    from .run import HERE, load_plan
else:
    from common import table_clearance, tool_vertices
    from dream_policy import continuous_client_class, public_transcript
    from run import HERE, load_plan


class SessionExpired(Exception):
    pass


class FakeClient:
    session_id = None

    def __init__(self):
        self.reply_session = "private-session-one"
        self.error = None

    def get_action(self, *, commanded=None):
        if self.error:
            raise self.error
        self.session_id = self.reply_session
        return {"action": [1]}


class DreamContinuityTests(unittest.TestCase):
    def make(self):
        requests, sessions = [], {"session_count": 0}
        client = continuous_client_class(FakeClient, SessionExpired, requests, sessions)()
        return client, requests, sessions

    def test_one_episode_succeeds_without_recording_private_identifiers(self):
        client, requests, sessions = self.make()
        self.assertEqual(client.get_action(), {"action": [1]})
        self.assertEqual(client.get_action(), {"action": [1]})
        self.assertEqual(sessions["session_count"], 1)
        self.assertTrue(all(r["status"] == "success" for r in requests))
        self.assertNotIn("private-session", json.dumps([requests, sessions]))

    def test_replacement_reply_never_reaches_controller(self):
        client, requests, sessions = self.make()
        client.get_action()
        client.reply_session = "private-session-two"
        with self.assertRaisesRegex(RuntimeError, "will not execute"):
            client.get_action()
        self.assertEqual(requests[-1]["status"], "session_replaced")
        self.assertEqual(sessions["session_count"], 2)

    def test_expiry_becomes_nonretryable_runtime_error(self):
        client, requests, _ = self.make()
        client.error = SessionExpired("Private server detail")
        with self.assertRaisesRegex(RuntimeError, "trial stopped"):
            client.get_action()
        self.assertEqual(requests[-1]["status"], "session_replaced")
        self.assertNotIn("Private", json.dumps(requests))

    def test_transcript_omits_identifiers_and_preserves_request_facts(self):
        class Policy:
            def transcript(self):
                return {"server_url": "private", "model": "dream-1",
                        "requests": [{"session_id": "secret-id", "step": 32, "attempts": 1}]}
        value = public_transcript(Policy())
        self.assertEqual(value["requests"], [{"step": 32, "attempts": 1}])
        self.assertNotIn("server_url", value)


class PhysicalEvidenceTests(unittest.TestCase):
    def test_clearance_includes_rotated_handle_and_rim(self):
        vertices = tool_vertices()
        self.assertAlmostEqual(table_clearance(vertices, [0, 0, .757, 0, 0, 0, 1]), .0005)
        # Turn the tool 90 degrees about Y: its 40 mm handle end is now lowest.
        q = np.sqrt(.5)
        self.assertAlmostEqual(table_clearance(vertices, [0, 0, 1., 0, q, 0, q]), 1.-.7475-.04)
        # The opposite turn makes the far 104 mm rim edge the lowest point.
        self.assertAlmostEqual(table_clearance(vertices, [0, 0, 1., 0, -q, 0, q]), 1.-.7475-.104)

    def test_invalid_and_overlong_plans_fail_before_simulator_start(self):
        plan = load_plan(HERE/"motion-plan.json")
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"plan.json"
            plan["frames"][0]["action14"][0] = float("nan")
            path.write_text(json.dumps(plan))
            with self.assertRaisesRegex(ValueError, "finite"):
                load_plan(path)
            plan = load_plan(HERE/"motion-plan.json")
            plan["frames"] *= 4
            path.write_text(json.dumps(plan))
            with self.assertRaisesRegex(ValueError, "run limit"):
                load_plan(path)


if __name__ == "__main__":
    unittest.main()
