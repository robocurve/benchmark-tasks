"""Causal coupling and invalid-strategy checks; no simulator required."""
import math
import unittest
import numpy as np
from mechanics import DrainState, connector_metrics, receiver_stroke, INFLATED_RADIUS, COLLAPSED_RADIUS

def aligned():
    return {'catheter':np.array([.375,-.045,.851,0,0,0,1.]),
            'valve':np.array([.336,-.139,.851,0,0,0,1.]),
            'barrel':np.array([.264,-.139,.851,0,0,0,1.]),
            'plunger':np.array([.264,-.139,.851,0,0,0,1.])}

class CouplingTests(unittest.TestCase):
    def test_disconnected_receiver_cannot_deflate(self):
        p=aligned();p['plunger'][0]-=.05
        s=DrainState();m=s.update(p)
        self.assertFalse(m['connected']);self.assertEqual(s.radius,INFLATED_RADIUS)

    def test_wrong_axis_cannot_connect(self):
        p=aligned();p['barrel'][3:7]=[0,0,1,0]
        self.assertFalse(DrainState().connect(p))

    def test_off_axis_cannot_connect(self):
        p=aligned();p['barrel'][1]+=.01
        self.assertFalse(DrainState().connect(p))

    def test_prewithdrawn_receiver_is_not_empty(self):
        p=aligned();p['plunger'][0]-=.02
        self.assertFalse(DrainState().connect(p))

    def test_connection_alone_does_not_drain(self):
        p=aligned();s=DrainState();self.assertTrue(s.connect(p))
        for _ in range(300):s.update(p)
        self.assertAlmostEqual(s.radius,INFLATED_RADIUS)

    def test_volume_follows_measured_stroke(self):
        p=aligned();s=DrainState();self.assertTrue(s.connect(p))
        p['plunger'][0]-=.025;s.update(p)
        self.assertAlmostEqual(s.fraction,.5)
        self.assertAlmostEqual(s.radius**3,(INFLATED_RADIUS**3+COLLAPSED_RADIUS**3)/2)
        p['plunger'][0]-=.025;m=s.update(p)
        self.assertTrue(m['collapsed']);self.assertAlmostEqual(s.radius,COLLAPSED_RADIUS)

    def test_reverse_stroke_reinflates_connected_balloon(self):
        p=aligned();s=DrainState();self.assertTrue(s.connect(p))
        p['plunger'][0]-=.05;self.assertTrue(s.update(p)['collapsed'])
        p['plunger'][0]+=.025;m=s.update(p)
        self.assertFalse(m['collapsed']);self.assertAlmostEqual(s.fraction,.5)
        self.assertGreater(s.radius,.005)

    def test_measured_pull_off_releases_without_external_command(self):
        p=aligned();s=DrainState();self.assertTrue(s.connect(p))
        p['plunger'][0]-=.05;s.update(p)
        p['barrel'][0]-=.013;p['plunger'][0]-=.013;m=s.update(p)
        self.assertFalse(m['connected']);self.assertTrue(m['ever_disconnected'])
        self.assertFalse(m['failed']);self.assertTrue(m['collapsed'])

    def test_premature_pull_off_latches_failure(self):
        p=aligned();s=DrainState();self.assertTrue(s.connect(p))
        p['barrel'][0]-=.013;p['plunger'][0]-=.013;m=s.update(p)
        self.assertTrue(m['failed']);self.assertEqual(m['failure'],'receiver_removed_before_deflation')

    def test_nozzle_withdrawal_closes_fluid_valve_before_detent_release(self):
        p=aligned();s=DrainState();self.assertTrue(s.connect(p))
        p['plunger'][0]-=.05;s.update(p)
        p['barrel'][0]-=.004;p['plunger'][0]-=.004;m=s.update(p)
        self.assertTrue(m['connected']);self.assertFalse(m['fluid_connected'])
        p['plunger'][0]+=.02;m=s.update(p)
        self.assertTrue(m['collapsed'])

    def test_reinserting_nozzle_reopens_fluid_valve_and_reinflates(self):
        p=aligned();s=DrainState();self.assertTrue(s.connect(p))
        p['plunger'][0]-=.05;s.update(p)
        p['barrel'][0]-=.004;p['plunger'][0]-=.004;s.update(p)
        p['plunger'][0]+=.02
        p['barrel'][0]+=.004;p['plunger'][0]+=.004;m=s.update(p)
        self.assertTrue(m['fluid_connected']);self.assertFalse(m['collapsed'])

    def test_inflated_extraction_is_irreversible_failure(self):
        p=aligned();s=DrainState();p['catheter'][1]-=.1
        self.assertTrue(s.update(p)['failed'])
        p=aligned();self.assertTrue(s.update(p)['failed'])

    def test_nonfinite_measurement_rejected(self):
        p=aligned();p['catheter'][0]=math.nan
        with self.assertRaises(ValueError):DrainState().update(p)

if __name__=='__main__':unittest.main()
