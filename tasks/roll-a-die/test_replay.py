"""Reproduce v2 scores from the committed anonymous measured trajectories."""

import gzip
import hashlib
import json
from pathlib import Path
import unittest

from replay import replay
from scoring import SCORE_VERSION

HERE = Path(__file__).resolve().parent


class RecordedEvidenceTests(unittest.TestCase):
    def check_recording(self, name, expected_score):
        validation = json.loads((HERE / "VALIDATION.json").read_text())
        run = validation["runs"][name]
        path = HERE / run["evidence_file"]
        content = gzip.decompress(path.read_bytes())
        self.assertEqual(hashlib.sha256(content).hexdigest(), run["input_trajectory_sha256"])
        self.assertEqual(len(content.splitlines()), run["observations"])
        result = replay(path)
        self.assertEqual(result["score_version"], SCORE_VERSION)
        self.assertEqual(result["score"], expected_score)
        self.assertAlmostEqual(result["max_tilt_excursion_degrees"],
                               run["result"]["max_tilt_excursion_degrees"], places=8)

    def test_recorded_partial_turn_remains_partial(self):
        self.check_recording("die-debug-003", 0.25)

    def test_recorded_demonstration_reaches_full_score(self):
        self.check_recording("die-debug-004", 1.0)

    def test_recorded_dream_baseline_has_no_pickup(self):
        self.check_recording("die-dream-002", 0.0)


if __name__ == "__main__":
    unittest.main()
