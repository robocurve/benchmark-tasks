import csv
import math
import tempfile
import unittest
from pathlib import Path

from judge_waveform import read_trace
from waveform import evaluate_waveform, waveform_samples


class ProbeModelTest(unittest.TestCase):
    def test_compensation_flattens_the_trace(self):
        matched = evaluate_waveform(*waveform_samples(0.0))
        self.assertEqual(matched.score, 1.0)
        self.assertAlmostEqual(matched.vpp, 5.0)
        self.assertAlmostEqual(matched.frequency_hz, 1000.0)

        start_t, start_v = waveform_samples(-0.8)
        _, high_v = waveform_samples(0.8)
        self.assertEqual(evaluate_waveform(start_t, start_v).score, 0.0)
        self.assertLess(max(start_v), 5.0)
        self.assertGreater(max(high_v), 5.0)
        self.assertLess(min(high_v), 0.0)

    def test_adjusting_toward_match_earns_partial_credit(self):
        angles = (-0.8, -0.5, -0.3, -0.1, 0.0)
        scores = [evaluate_waveform(*waveform_samples(angle)).score for angle in angles]
        self.assertEqual(scores, [0.0, 0.25, 0.5, 0.75, 1.0])

    def test_wrong_or_incomplete_signal_fails(self):
        time_s, voltage_v = waveform_samples(0.0)
        clipped_span = [1.0 + 0.5 * v for v in voltage_v]
        sparse_time = time_s[::10]
        sparse_voltage = voltage_v[::10]
        cases = (
            (time_s, [2.5] * len(time_s)),
            (time_s, clipped_span),
            (sparse_time, sparse_voltage),
            (time_s[:256], voltage_v[:256]),
            waveform_samples(0.0, frequency_hz=800.0),
            (time_s, [math.nan] + list(voltage_v[1:])),
        )
        for bad_time, bad_voltage in cases:
            self.assertEqual(evaluate_waveform(bad_time, bad_voltage).score, 0.0)

    def test_waveform_score_survives_csv_export(self):
        time_s, voltage_v = waveform_samples(0.0)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "waveform.csv"
            with path.open("w", newline="") as target:
                writer = csv.writer(target)
                writer.writerow(["time_s", "voltage_v"])
                writer.writerows(zip(time_s, voltage_v))
            result = evaluate_waveform(*read_trace(path))
            self.assertEqual(result.score, 1.0)

            with path.open("w", newline="") as target:
                writer = csv.writer(target)
                for i, (t, v) in enumerate(zip(time_s, voltage_v)):
                    metadata = ["Record Length", len(time_s)] if i == 0 else ["", ""]
                    writer.writerow([*metadata, "", t - 0.001, v])
            result = evaluate_waveform(*read_trace(path))
            self.assertEqual(result.score, 1.0)


if __name__ == "__main__":
    unittest.main()
