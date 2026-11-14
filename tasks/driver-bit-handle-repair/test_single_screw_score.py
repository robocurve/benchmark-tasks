import unittest

from single_screw_score import score_from_evidence


class SingleScrewScoreTests(unittest.TestCase):
    def test_completed_outcomes_and_prerequisites(self):
        cases = (
            (False, False, False, 0.0),
            (False, True, True, 0.0),  # apparent repair cannot skip bit assembly
            (True, False, False, 0.2),
            (True, False, True, 0.2),  # returning an unfastened tool is not repair
            (True, True, False, 0.8),  # repair without return gets meaningful credit
            (True, True, True, 1.0),
        )
        for assembly, load, returned, expected in cases:
            with self.subTest(assembly=assembly, load=load, returned=returned):
                self.assertEqual(
                    score_from_evidence(
                        assembly_pass=assembly, load_pass=load, return_pass=returned
                    ),
                    expected,
                )


if __name__ == "__main__":
    unittest.main()
