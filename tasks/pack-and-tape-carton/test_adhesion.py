import unittest
from adhesion import ContactBond


class Tests(unittest.TestCase):
    def press(self,bond,**changes):
        s=dict(dt=1/240,gap=0.,alignment=1.,normal_force=1.,adhesive_side=True)
        s.update(changes)
        for _ in range(26):bond.contact(**s)

    def test_nearby_tool_does_not_bond_tape(self):
        b=ContactBond();self.press(b,normal_force=0.);self.assertFalse(b.attached)

    def test_back_face_is_not_adhesive(self):
        b=ContactBond();self.press(b,adhesive_side=False);self.assertFalse(b.attached)

    def test_gap_prevents_bond(self):
        b=ContactBond();self.press(b,gap=.002);self.assertFalse(b.attached)

    def test_real_contact_generates_restoring_force_and_can_peel(self):
        b=ContactBond();self.press(b);self.assertTrue(b.attached)
        self.assertEqual(b.force((.0005,0.,0.)),(-1.,0.,0.))
        self.assertEqual(b.force((.002,0.,0.)),(0.,0.,0.));self.assertTrue(b.broken)
        self.press(b);self.assertFalse(b.attached)

if __name__=='__main__':unittest.main()
