import unittest

from scoring import Evidence, score_evidence


class ScoringTests(unittest.TestCase):
    def complete(self, **changes):
        values = dict(old_tip_removed=True, old_tip_in_discard=True,
                      mesh_marks_clean=8, contact_marks_clean=6,
                      new_tip_seated=True, tools_released=True,
                      earbud_released=True, objects_resting=True)
        values.update(changes)
        return Evidence(**values)

    def test_complete_handoff_scores_one(self):
        self.assertEqual(score_evidence(self.complete()), 1.0)

    def test_each_documented_milestone(self):
        base = dict(old_tip_removed=True, old_tip_in_discard=True)
        self.assertEqual(score_evidence(Evidence(**base)), 0.15)
        self.assertEqual(score_evidence(Evidence(**base, mesh_marks_clean=7)), 0.40)
        self.assertEqual(score_evidence(Evidence(**base, mesh_marks_clean=7,
                                                 contact_marks_clean=5)), 0.60)
        self.assertEqual(score_evidence(Evidence(**base, mesh_marks_clean=7,
                                                 contact_marks_clean=5,
                                                 new_tip_seated=True)), 0.85)

    def test_80_percent_threshold_rounds_up(self):
        self.assertEqual(score_evidence(self.complete(mesh_marks_clean=6)), 0.15)
        self.assertEqual(score_evidence(self.complete(mesh_marks_clean=7)), 1.0)

    def test_cleaning_cannot_bypass_disposal(self):
        self.assertEqual(score_evidence(self.complete(old_tip_in_discard=False)), 0.0)

    def test_fitted_but_not_released_is_partial(self):
        self.assertEqual(score_evidence(self.complete(tools_released=False)), 0.85)

    def test_live_fit_can_be_lost(self):
        self.assertEqual(score_evidence(self.complete(new_tip_seated=False)), 0.60)

    def test_no_empty_inspection_shortcut(self):
        self.assertEqual(score_evidence(self.complete(mesh_marks_total=0)), 0.15)


if __name__ == "__main__":
    unittest.main()
