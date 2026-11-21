import json
from pathlib import Path
import tempfile
import unittest
from dream_budget import BudgetExceeded, MAX_ATTEMPTS, reserve


class BudgetTests(unittest.TestCase):
    def test_reservations_survive_reopening_and_stop_before_excess(self):
        with tempfile.TemporaryDirectory() as directory:
            ledger = Path(directory)/'ledger.jsonl'
            for _ in range(MAX_ATTEMPTS):
                reserve(ledger)
            before = ledger.read_bytes()
            with self.assertRaises(BudgetExceeded):
                reserve(Path(str(ledger)))
            self.assertEqual(ledger.read_bytes(),before)
            self.assertEqual(len(before.splitlines()),MAX_ATTEMPTS)

    def test_malformed_ledger_fails_closed(self):
        with tempfile.TemporaryDirectory() as directory:
            ledger = Path(directory)/'ledger.jsonl'
            ledger.write_text('{incomplete')
            with self.assertRaises(json.JSONDecodeError):
                reserve(ledger)
            self.assertEqual(ledger.read_text(),'{incomplete')


if __name__=='__main__':
    unittest.main()
