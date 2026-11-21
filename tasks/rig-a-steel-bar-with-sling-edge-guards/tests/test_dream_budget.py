"""The Dream trial ends, and is scored, when its last allowed action request has been used up. Needs Inspect Robots
(dream_audit.py imports it); skipped without it."""

from collections import deque
import importlib.util
import sys
from types import SimpleNamespace
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

HAVE = importlib.util.find_spec("inspect_robots") is not None and importlib.util.find_spec("numpy") is not None


@unittest.skipUnless(HAVE, "needs inspect_robots and numpy")
class Budget(unittest.TestCase):
    def check(self, calls, queued, cap=300):
        from rdb_sling_guards.dream_audit import request_budget_spent

        client = SimpleNamespace(calls=calls)
        policy = SimpleNamespace(_trial=SimpleNamespace(queue=deque(range(queued))))
        return request_budget_spent(client, policy, cap)

    def test_ends_only_when_the_last_reply_is_used_up(self):
        self.assertIsNone(self.check(299, 0))           # the next request is still allowed
        self.assertIsNone(self.check(300, 5))           # rows of the last reply still to execute
        self.assertEqual(self.check(300, 0), "dream_request_cap")


HAVE_DREAM = HAVE and importlib.util.find_spec("dream") is not None


@unittest.skipUnless(HAVE_DREAM, "needs inspect_robots, numpy and dreamlabs-client")
class CapacityWait(unittest.TestCase):
    """A Dream-1 'at capacity' refusal is waited out (physics is paused); every other refusal still ends the trial."""

    def client(self, outcomes):
        from rdb_sling_guards.dream_audit import BudgetedDreamClient

        events, pauses, queue = [], [], list(outcomes)

        def get_action():
            item = queue.pop(0)
            if isinstance(item, Exception):
                raise item
            return item

        audit = SimpleNamespace(event=lambda kind, **fields: events.append(kind))
        client = BudgetedDreamClient(SimpleNamespace(get_action=get_action), 300, audit, sleep=pauses.append)
        return client, events, pauses

    def test_waits_while_dream_is_at_capacity(self):
        import dream

        busy = dream.DreamServerError("busy", code="DA-2000", reason="pool_full")
        late = dream.DreamServerError("busy", code="DA-2000", reason="max_serves", retry_after_s=75.)
        client, events, pauses = self.client([busy, late, "reply"])
        self.assertEqual(client._ask(), "reply")
        self.assertEqual(pauses, [30., 75.])
        self.assertEqual((client.capacity_waits, client.capacity_wait_s, client.calls), (2, 105., 0))
        self.assertEqual(events, ["capacity_wait", "capacity_wait"])

    def test_other_refusals_still_end_the_trial(self):
        import dream

        for error in (dream.DreamSessionExpired("expired", code="DA-0001", reason="idle"),
                      dream.DreamServerError("failed", code="DA-5000"),
                      dream.DreamAuthError("refused", code="DA-1000", reason="over_cap")):
            client, events, pauses = self.client([error, "reply"])
            with self.assertRaises(type(error)):
                client._ask()
            self.assertEqual((pauses, events), ([], []))


if __name__ == "__main__":
    unittest.main()
