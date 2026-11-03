"""Verify the final v3 simulation evidence and the source that produced it.

The measured traces are anonymous, unchanged JSONL bytes compressed with gzip.
Replay ignores their stored metrics and recomputes from measured states. These
checks cover one successful fixed-layout demo and one model trial; they are not
an estimate of physical fidelity or repeated success probability.
"""

import gzip
import hashlib
import json
from pathlib import Path
import unittest

from replay import replay
from scoring import RollTracker, SCORE_VERSION

HERE = Path(__file__).resolve().parent
CORE_SOURCES = ("task.py", "scoring.py", "run.py", "demo_policy.py", "visuals.py")


class RecordedEvidenceTests(unittest.TestCase):
    def setUp(self):
        self.validation = json.loads((HERE / "VALIDATION.json").read_text())

    def assert_equivalent(self, actual, expected):
        """Allow harmless float differences across NumPy versions/platforms."""
        if isinstance(expected, bool) or expected is None or isinstance(expected, str):
            self.assertEqual(actual, expected)
        elif isinstance(expected, (int, float)):
            self.assertAlmostEqual(actual, expected, places=8)
        elif isinstance(expected, list):
            self.assertEqual(len(actual), len(expected))
            for a, e in zip(actual, expected):
                self.assert_equivalent(a, e)
        else:
            self.assertEqual(set(actual), set(expected))
            for key in expected:
                self.assert_equivalent(actual[key], expected[key])

    def check_recording(self, name, expected_score):
        run = self.validation["runs"][name]
        path = HERE / run["evidence_file"]
        compressed = path.read_bytes()
        self.assertEqual(hashlib.sha256(compressed).hexdigest(), run["evidence_gzip_sha256"])
        content = gzip.decompress(compressed)
        self.assertEqual(hashlib.sha256(content).hexdigest(), run["input_trajectory_sha256"])
        samples = [json.loads(line) for line in content.splitlines()]
        self.assertEqual(len(samples), run["observations"])
        self.assertEqual(len(samples) - 1, run["actions"])
        self.assertEqual(run["executed_action_audit"]["executed_actions"], run["actions"])
        self.assertEqual(run["executed_action_audit"]["range_violations"], 0)
        self.assertEqual(run["executed_action_audit"]["adjacent_action_step_limit_violations"], 0)
        self.assertEqual(samples[0]["time"], 0.0)
        self.assertAlmostEqual(samples[-1]["time"], run["simulation_seconds"])
        self.assertEqual(run["review_video"]["frames"], len(samples))
        for index, sample in enumerate(samples):
            self.assertAlmostEqual(sample["time"], index / run["control_hz"])

        result = replay(path)
        self.assertEqual(result["score_version"], SCORE_VERSION)
        self.assertEqual(result["score"], expected_score)
        self.assert_equivalent(result, run["result"])
        tracker = RollTracker()
        first_full = None
        for sample in samples:
            tracker.update(sample)
            if tracker.value == 1.0 and first_full is None:
                first_full = sample["time"]
        self.assert_equivalent(first_full, run["first_full_score_simulation_seconds"])
        return result

    def test_final_marked_demonstration_reaches_full_score(self):
        result = self.check_recording("die-v3-demo-002", 1.0)
        self.assertEqual(result["turnover_faces"], ["-X"])
        up = next(e for e in result["events"] if e["event"] == "face_up" and e["face"] == "-X")
        down = next(e for e in result["events"] if e["event"] == "face_down" and e["face"] == "-X")
        self.assertEqual((up["pips"], up["color"]), (5, "cyan"))
        self.assertAlmostEqual(up["time"], 7.85)
        self.assertAlmostEqual(down["time"], 8.0)
        self.assertAlmostEqual(result["rest_seconds"], 1.0)

    def test_final_direct_dream_trial_has_no_pickup(self):
        result = self.check_recording("die-v3-dream-002", 0.0)
        self.assertFalse(result["lifted"])
        self.assertFalse(result["released"])
        self.assertFalse(result["turned_over"])
        run = self.validation["runs"]["die-v3-dream-002"]
        self.assertEqual(run["dream"]["transport"], "direct")
        self.assertEqual(run["dream"]["retries"], 0)
        self.assertEqual(run["dream"]["sessions"], 1)

    def test_current_core_sources_match_both_recorded_runs(self):
        self.assertEqual(self.validation["score_version"], SCORE_VERSION)
        self.assertEqual(set(self.validation["runs"]), {"die-v3-demo-002", "die-v3-dream-002"})
        expected = self.validation["core_source_sha256"]
        self.assertEqual(set(expected), set(CORE_SOURCES))
        for filename in CORE_SOURCES:
            current = hashlib.sha256((HERE / filename).read_bytes()).hexdigest()
            self.assertEqual(current, expected[filename], f"{filename} changed: refresh final validation")
            for run in self.validation["runs"].values():
                self.assertEqual(run["source_sha256"][filename], current)


if __name__ == "__main__":
    unittest.main()
