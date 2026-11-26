import unittest

from scoring import CartCheck, CartState


class CartCheckTest(unittest.TestCase):
    def state(self, **changes):
        return CartState(**{
            "floor_seated": True,
            "both_rails_engaged": True,
            "both_walls_straight": True,
            "supported": True,
            "inward_load": (False, False),
            "hands_clear": True,
            **changes,
        })

    def proof(self, check, load=(True, True)):
        for step in range(1, 16):
            check.sample(step, self.state(inward_load=load, hands_clear=False))

    def test_requires_proof_before_hands_clear(self):
        check = CartCheck()
        self.assertEqual(check.sample(1, self.state()), .75)

    def test_full_score_after_proof_and_retreat(self):
        check = CartCheck()
        self.proof(check)
        self.assertEqual(check.sample(16, self.state()), 1.)

    def test_unlocked_and_folded_never_credit_proof(self):
        check = CartCheck()
        for step in range(1, 21):
            value = check.sample(step, self.state(
                both_rails_engaged=False, both_walls_straight=False,
                inward_load=(True, True), hands_clear=False,
            ))
        self.assertEqual(value, .25)
        self.assertFalse(check.proven)

    def test_single_side_pressure_does_not_prove_both_locks(self):
        check = CartCheck()
        self.proof(check, (True, False))
        self.assertEqual(check.sample(16, self.state()), .75)

    def test_floor_off_support_cannot_pass(self):
        check = CartCheck()
        self.proof(check)
        self.assertEqual(check.sample(16, self.state(floor_seated=False)), 0.)

    def test_repeat_samples_do_not_count_as_time(self):
        check = CartCheck()
        for _ in range(30):
            check.sample(1, self.state(inward_load=(True, True), hands_clear=False))
        self.assertEqual(check.sample(2, self.state()), .75)

    def test_proof_lost_when_wall_opens_after_pressure(self):
        check = CartCheck()
        self.proof(check)
        check.sample(16, self.state(both_walls_straight=False))
        self.assertEqual(check.sample(17, self.state()), .75)


if __name__ == "__main__":
    unittest.main()
