"""Scoring boundary/shortcut fixtures; not physical robot demonstrations."""
import unittest
import numpy as np
from stock import Stock
from scoring import end_score, saw_clear, clamp_state


def groove(x=.150,depth=.010,rows=slice(None)):
    s=Stock();i=round(x/s.dx)
    s.cleared[i:i+4,rows]=round(depth/s.dz)
    s.progress[:]=s.cleared*s.dz
    return s


class Stages(unittest.TestCase):
    def value(self,s,clamp=True,clear=True,loose_cut=False):
        return end_score(s.stages(),clamp,clear,loose_cut)

    def test_clamp_and_endpoint_vetoes(self):
        for s in (Stock(),groove(),groove(depth=.020)):
            self.assertEqual(self.value(s,clamp=False),0.)
            self.assertEqual(self.value(s,clear=False),0.)
            self.assertEqual(self.value(s,loose_cut=True),0.)
        self.assertEqual(self.value(Stock()),.25)

    def test_depth_threshold_and_fractional_work(self):
        self.assertEqual(self.value(groove(depth=.0095)),.25)
        self.assertEqual(self.value(groove(depth=.010)),.5)
        self.assertEqual(self.value(groove(depth=.0195)),.5)
        s=groove(depth=.0095);s.progress[600:604,:]=.01049
        self.assertEqual(self.value(s),.25)

    def test_width_and_connected_groove(self):
        self.assertEqual(self.value(groove(depth=.020,rows=slice(0,9))),.25)
        s=groove();s.cleared[:,5]=0;s.progress[:,5]=0
        self.assertEqual(self.value(s),.25)
        # Separate half-width deep cuts do not add up to a full-width groove.
        s=groove(rows=slice(0,5));b=groove(x=.151,rows=slice(5,10))
        s.cleared+=b.cleared;s.progress+=b.progress
        self.assertEqual(self.value(s),.25)

    def test_length_boundaries_and_wrong_length_outcomes(self):
        for x in (.149,.151):
            self.assertEqual(self.value(groove(x=x)),.5)
            self.assertEqual(self.value(groove(x=x,depth=.020)),1.)
        self.assertEqual(self.value(groove(x=.15125)),.25)
        self.assertEqual(self.value(groove(x=.15125,depth=.020)),0.)
        self.assertEqual(self.value(groove(x=.14875)),0.)
        self.assertEqual(self.value(groove(x=.155,depth=.020)),0.)

    def test_unrelated_cut_veto_even_after_valid_cut(self):
        for d in (.010,.020):
            s=groove(depth=d);s.cleared[100,0]=1;s.progress[100,0]=s.dz
            self.assertEqual(self.value(s),0.)
        s=groove();s.cleared[615:616,:]=20;s.progress[:]=s.cleared*s.dz
        self.assertEqual(self.value(s),.5) # target work zone includes kerf side
        s.cleared[616,0]=1 # x154mm is outside work zone
        self.assertEqual(self.value(s),0.)

    def test_measurable_damage_boundary(self):
        for depth, layers, expected in ((.000499999,0,.5),(.0005,1,0.),(.001,2,0.)):
            s=groove();s.progress[100,0]=depth;s.cleared[100,0]=layers
            self.assertEqual(self.value(s),expected)
        # Fractional work distributed over many columns is not a deep cut.
        s=groove();s.progress[:100,:]=.00049
        self.assertEqual(self.value(s),.5)

    def test_accumulated_loose_contacts(self):
        s=Stock();a=[.150,-.01,.0198];b=[.150,.01,.0198]
        # Physically driven removal, below one layer after one contact.
        s.remove(a,b,.0001,1.,clamped=False)
        self.assertGreater(s.loose_progress.max(),0.)
        self.assertFalse(s.unsafe_loose_cut)
        for _ in range(1000):
            s.remove(a,b,.0001,1.,clamped=False)
            if s.unsafe_loose_cut:break
        self.assertTrue(s.unsafe_loose_cut)
        self.assertGreaterEqual(s.cleared.max(),1)
        # Clamp does not erase earlier damage; a fresh stock resets history.
        s.remove(a,b,.0001,1.,clamped=True)
        self.assertTrue(s.unsafe_loose_cut)
        self.assertFalse(Stock().unsafe_loose_cut)
        # Independent loose episodes accumulate at the same column.
        s=Stock();s.loose_progress[600,0]=.000249999
        self.assertFalse(s.unsafe_loose_cut)
        s.loose_progress[600,0]+=.00025
        self.assertFalse(s.unsafe_loose_cut)
        s.loose_progress[600,0]+=.000000001
        self.assertTrue(s.unsafe_loose_cut)
        s.loose_progress[600,0]+=.0001
        self.assertTrue(s.unsafe_loose_cut)

    def test_contact_accumulation_and_attribution(self):
        s=groove();a=[.100,-.01,.0198];b=[.100,.01,.0198]
        for _ in range(3):s.remove(a,b,.0001,1.,clamped=False)
        self.assertEqual(self.value(s),.5)
        loose=s.loose_progress.copy()
        for _ in range(3):s.remove(a,b,.0001,1.,clamped=True)
        np.testing.assert_array_equal(loose,s.loose_progress)
        self.assertFalse(s.unsafe_loose_cut)
        # Separated contact episodes at the same columns still clear a layer.
        for _ in range(1000):
            s.remove(a,b,.0001,1.,clamped=False)
            if s.cleared[400:404].any():break
        self.assertEqual(self.value(s),0.)
        # Broad sub-layer activity cannot synthesize the depth of one cut.
        s=Stock();s.loose_progress[:,:]=.000499
        self.assertFalse(s.unsafe_loose_cut)

    def test_actual_clamp_boundaries_and_rotation(self):
        wp=np.array([.20,-.03,.8275,0,0,0,1.])
        for gap in (.0192,.0202):
            self.assertTrue(clamp_state(wp,-.035-gap,1.50001)[0])
        for gap in (.019199,.020201):
            self.assertFalse(clamp_state(wp,-.035-gap,2.)[0])
        self.assertFalse(clamp_state(wp,-.055,1.5)[0])
        wp[0]+=.003001
        self.assertFalse(clamp_state(wp,-.055,2.)[0])
        wp[0]=.20;wp[3:]=[0,0,np.sin(np.deg2rad(1.01)),np.cos(np.deg2rad(1.01))]
        self.assertFalse(clamp_state(wp,-.055,2.)[0])

    def test_clearance_any_direction_and_offcut(self):
        wp=np.array([0,0,0,0,0,0,1.])
        wood=[(wp,Stock().boxes())]
        # Teeth at 25mm: exactly 5mm above original20mm top is accepted.
        sp=np.array([.15,.155,.060,0,0,0,1.])
        self.assertTrue(saw_clear(sp,wood))
        sp[2]-=1e-6;self.assertFalse(saw_clear(sp,wood))
        sp=np.array([.330,.155,.035,0,0,0,1.])
        self.assertTrue(saw_clear(sp,wood)) # side withdrawal at tooth height
        sp[0]=.15;self.assertFalse(saw_clear(sp,wood))
        sp[0]=.330
        offcut=np.array([.32,0,0,0,0,0,1.])
        self.assertFalse(saw_clear(sp,wood+[(offcut,[(.01,0,.01,.02,.02,.02)])]))


if __name__=='__main__':unittest.main()
