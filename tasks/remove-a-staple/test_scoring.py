import unittest
from scoring import score, CHECKS


def record():
    return {"schema": "staple-judging-v4", "checks": dict.fromkeys(CHECKS, True),
            "legs_clear_of_both_sheets": 2, "maximum_new_tear_mm": 0.,
            "any_new_damage_away_from_holes": False,
            "staple_touching_remover": False, "remover_released": True,
            "minimum_staple_to_paper_clearance_mm":10., "release_seconds": 3.}


class ScoringTests(unittest.TestCase):
    def test_complete_extraction_and_partial_leg_withdrawal(self):
        r = record()
        self.assertEqual(score(r)["score"], 1.)
        r["legs_clear_of_both_sheets"] = 1
        self.assertEqual(score(r)["score"], .5)
        r["legs_clear_of_both_sheets"] = 0
        self.assertEqual(score(r)["score"], 0.)

    def test_torn_paper_zeroes_partial_and_full(self):
        for legs in (1,2):
            r = record()
            r["legs_clear_of_both_sheets"] = legs
            r["maximum_new_tear_mm"] = 2.001
            self.assertEqual(score(r)["score"], 0.)

    def test_independent_damage_is_not_hidden_by_hole_tolerance(self):
        r = record()
        r["any_new_damage_away_from_holes"] = True
        self.assertEqual(score(r)["score"], 0.)

    def test_full_extraction_needs_clearance_and_release(self):
        r=record();r['minimum_staple_to_paper_clearance_mm']=9.999
        self.assertEqual(score(r)['score'],.5)
        r=record();r['release_seconds']=2.999
        self.assertEqual(score(r)['score'],.5)

    def test_broken_staple_never_earns_credit(self):
        for legs in (1,2):
            r=record();r['legs_clear_of_both_sheets']=legs;r['checks']['staple_intact']=False
            self.assertEqual(score(r)['score'],0.)

    def test_paper_transport_is_not_required(self):
        r=record()
        self.assertNotIn('final_relative_corner_offset_mm',r)
        self.assertEqual(score(r)['score'],1.)

    def test_released_remover_can_retain_the_extracted_staple(self):
        r=record();r['staple_touching_remover']=True
        self.assertEqual(score(r)['score'],1.)
        r['remover_released']=False
        self.assertEqual(score(r)['score'],.5)
        # Holding an unused remover elsewhere does not affect a released staple.
        r['staple_touching_remover']=False
        self.assertEqual(score(r)['score'],1.)
        del r['remover_released']
        self.assertFalse(score(r)['record_valid'])

    def test_missing_or_nonfinite_observations_fail_closed(self):
        r = record()
        del r["checks"]["both_sheet_faces_inspected"]
        self.assertFalse(score(r)["record_valid"])
        r = record()
        r["maximum_new_tear_mm"] = float("nan")
        self.assertFalse(score(r)["record_valid"])


if __name__ == "__main__":
    unittest.main()
