"""Offline tests: no credentials or API requests."""
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
import tempfile
import unittest

from dream_budget import BudgetStopped, DEFAULTS, Ledger
from dream_runner import RequestStopped, guard_exchange


class FakeTransport:
    def __init__(self): self.drops = 0
    def _drop(self): self.drops += 1


def reserve_many(args):
    path, task, count = args
    ledger = Ledger(path)
    accepted = 0
    for _ in range(count):
        try:
            ledger.reserve(task, task)
            accepted += 1
        except BudgetStopped:
            break
    return accepted


class BudgetTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = Path(self.tmp.name) / 'ledger.sqlite3'

    def test_task_limit_persists_across_runs(self):
        ledger = Ledger.initialize(self.path)
        for _ in range(20): ledger.reserve('task', 'first')
        with self.assertRaisesRegex(BudgetStopped, 'task_budget_limit'):
            Ledger(self.path).reserve('task', 'second')
        self.assertEqual(ledger.status('task')['reserved']['total_micro_usd'], 30000)
        self.assertEqual(ledger.status()['last_stop'][3], 'task_budget_limit')

    def test_parallel_processes_do_not_overspend(self):
        ledger = Ledger.initialize(self.path)
        with ProcessPoolExecutor(4) as pool:
            counts = list(pool.map(reserve_many, [(self.path, 'shared-task', 20)] * 4))
        self.assertEqual(sum(counts), 20)
        self.assertEqual(ledger.status('shared-task')['reserved']['task_micro_usd'], 30000)

    def test_daily_window_and_total(self):
        now = [1000000.0]
        cfg = dict(DEFAULTS, day_micro_usd=3000, total_micro_usd=4500)
        ledger = Ledger.initialize(self.path, limits=cfg, clock=lambda: now[0])
        ledger.reserve('a', 'a'); ledger.reserve('b', 'b')
        with self.assertRaisesRegex(BudgetStopped, 'day_budget_limit'): ledger.reserve('c', 'c')
        now[0] += 86401
        ledger.reserve('c', 'c')
        with self.assertRaisesRegex(BudgetStopped, 'total_budget_limit'): ledger.reserve('d', 'd')

    def test_missing_corrupt_and_reinit_fail_closed(self):
        with self.assertRaises(BudgetStopped): Ledger(self.path).reserve('x', 'x')
        ledger = Ledger.initialize(self.path)
        with self.assertRaises(FileExistsError): Ledger.initialize(self.path)
        self.path.write_text('corrupt')
        with self.assertRaises(BudgetStopped): ledger.reserve('x', 'x')

    def test_expired_and_clock_backwards(self):
        now = [1000000.0]
        ledger = Ledger.initialize(self.path, clock=lambda: now[0])
        ledger.reserve('a', 'a'); now[0] -= 1
        with self.assertRaisesRegex(BudgetStopped, 'clock_moved_backwards'): ledger.reserve('a', 'a')
        now[0] += 21 * 86400
        with self.assertRaisesRegex(BudgetStopped, 'budget_period_expired'): ledger.reserve('a', 'a')

    def test_transport_error_counts_once_and_is_not_retried(self):
        ledger = Ledger.initialize(self.path)
        seen = []
        def broken(*args):
            seen.append(True)
            raise ConnectionResetError('sensitive-header-placeholder')
        exchange = guard_exchange(broken, ledger, 'a', 'r')
        with self.assertRaises(RequestStopped) as caught: exchange(FakeTransport(), b'', {})
        self.assertNotIn('sensitive-header', str(caught.exception))
        self.assertEqual(len(seen), 1)
        self.assertEqual(ledger.status('a', 'r')['reserved']['run_calls'], 1)

    def test_limit_prevents_network_and_release_is_free(self):
        ledger = Ledger.initialize(self.path, limits=dict(DEFAULTS, task_micro_usd=1500))
        seen = []
        def send(*args): seen.append(args[3]); return (200, b'{}', {})
        exchange = guard_exchange(send, ledger, 'a', 'r')
        transport=FakeTransport()
        exchange(transport, b'', {})
        with self.assertRaises(BudgetStopped): exchange(transport, b'', {})
        exchange(transport, b'', {}, '/v1/end_session')
        self.assertEqual(seen, ['/v1/get_action', '/v1/end_session'])
        self.assertEqual(transport.drops, 1)
        self.assertEqual(ledger.status('a', 'r')['reserved']['run_calls'], 1)

    def test_every_inference_starts_a_fresh_connection(self):
        ledger=Ledger.initialize(self.path)
        observed=[]
        def send(transport,*args):
            observed.append(transport.drops)
            return (200,b'{}',{})
        exchange=guard_exchange(send,ledger,'a','r')
        transport=FakeTransport()
        exchange(transport,b'',{});exchange(transport,b'',{})
        self.assertEqual(observed,[1,2])


if __name__ == '__main__':
    unittest.main()
