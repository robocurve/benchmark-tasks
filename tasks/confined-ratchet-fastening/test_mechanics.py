import math,unittest
from ratchet_task.mechanics import Ratchet,Screw
class Mechanics(unittest.TestCase):
    def test_take_up_and_loaded_reference(self):
        c=Ratchet();phase,_=c.update(0.,0.)
        self.assertAlmostEqual(phase,math.radians(5))
        self.assertEqual(c.update(math.radians(4),1.)[0],phase)
        self.assertEqual(c.update(math.radians(5),1.)[0],phase)
    def test_return_reindexes_only_with_opening_gap(self):
        c=Ratchet();c.update(0.,0.)
        self.assertAlmostEqual(c.update(math.radians(-15),-1.)[0],math.radians(-10))
        self.assertAlmostEqual(c.update(math.radians(-11),1.)[0],math.radians(-10))
    def test_closing_motion_does_not_reindex(self):
        c=Ratchet();phase,_=c.update(0.,0.)
        self.assertEqual(c.update(-1.,1.)[0],phase)
    def test_drag_always_removes_energy(self):
        for speed in [-3.,-.1,0.,.1,3.]:
            c=Ratchet();_,drag=c.update(0.,speed)
            self.assertLessEqual(-drag*speed,0.)
    def test_helix_pitch_and_direction(self):
        s=Screw();s.capture(.1,0.)
        self.assertAlmostEqual(s.expected_z(-2*math.pi),.09875)
        self.assertAlmostEqual(s.expected_z(2*math.pi),.10125)
    def test_capture_reference_nonzero_angle(self):
        s=Screw();s.capture(.1,7.)
        self.assertAlmostEqual(s.expected_z(7.),.1)
        self.assertAlmostEqual(s.expected_z(7.-2*math.pi),.09875)
    def test_uncaptured_rejects_prediction(self):
        with self.assertRaises(RuntimeError):Screw().expected_z(0.)
if __name__=='__main__':unittest.main()
