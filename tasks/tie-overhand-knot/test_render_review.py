"""Guard against misleading labels caused by corrupt or misaligned telemetry."""
import json
from pathlib import Path
import tempfile
import unittest

from render_review import read_labels, write_subtitles


class ReviewLabelsTest(unittest.TestCase):
    def labels(self, rows):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "trajectory.jsonl"
            path.write_text("".join(json.dumps(row) + "\n" for row in rows))
            return read_labels(path)

    def assay(self, phase="seating", stamp=.05):
        return {"time": stamp, "phase": phase, "proof": {
            "active": True, "source": "simulator_applied_external_force",
            "applied_force_n": .123, "endpoint_forces_n": [[-.123, 0, 0], [.123, 0, 0]]}}

    def test_actual_load_not_target_or_score_is_shown(self):
        rows = [{"time": 0, "phase": "policy", "metrics": {"score": 1}}, self.assay()]
        rows[1]["proof"]["target_force_n"] = .981
        labels = self.labels(rows)
        self.assertIn("Applied load: 0.123 N", labels[1][1])
        self.assertNotIn("0.981", labels[1][1])
        self.assertNotIn("score", " ".join(text for _, text in labels).lower())

    def test_duplicate_missing_and_nonzero_start_times_rejected(self):
        for stamps in ([0, 0], [0, .10], [.05, .10]):
            with self.subTest(stamps=stamps), self.assertRaises(ValueError):
                self.labels([{"time": t, "phase": "policy"} for t in stamps])

    def test_claimed_load_without_consistent_buffer_values_rejected(self):
        for key, value in (("source", "requested_force"), ("applied_force_n", .981),
                           ("active", False), ("applied_force_n", float("nan"))):
            row = self.assay()
            row["proof"][key] = value
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                self.labels([{"time": 0, "phase": "policy"}, row])

    def test_assay_cannot_be_mislabelled_as_robot_actions(self):
        with self.assertRaises(ValueError):
            self.labels([{"time": 0, "phase": "policy"}, self.assay("policy")])

    def test_unknown_skipped_or_reversed_phase_rejected(self):
        for rows in ([{"time": 0, "phase": "unknown"}],
                     [{"time": 0, "phase": "policy"}, self.assay("proof")],
                     [{"time": 0, "phase": "policy"}, self.assay(), {"time": .1, "phase": "policy"}]):
            with self.subTest(rows=rows), self.assertRaises(ValueError):
                self.labels(rows)

    def test_labels_cover_exactly_one_frame_including_last(self):
        labels = self.labels([{"time": 0, "phase": "policy"}, self.assay(), self.assay("proof", .1)])
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "labels.ass"
            write_subtitles(path, labels, "Arial")
            events = [line for line in path.read_text().splitlines() if line.startswith("Dialogue:")]
        self.assertEqual(len(events), 3)
        self.assertIn("0:00:00.00,0:00:00.05", events[0])
        self.assertIn("0:00:00.10,0:00:00.15", events[-1])


if __name__ == "__main__":
    unittest.main()
