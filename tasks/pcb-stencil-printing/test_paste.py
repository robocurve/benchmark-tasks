"""Independent material-volume and visible-grading counterexamples."""
import json,unittest
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from stencil_paste_model import StencilPaste
DATA=json.loads((Path(__file__).parent/'board-geometry.json').read_text());R=Rotation.from_euler('x',45,degrees=True)

def print_case(force=8.,volume=120.,gap=0.,shift=0.):
 m=StencilPaste(DATA,volume);s=np.array([.33+shift,-.065,.7805625,0,0,0,1.])
 for y in np.linspace(-.031,.034,301):
  b=s[:3]+[0,y,.0000625+gap]-R.apply([0,-.00025,-.0325]);m.step(s,np.r_[b,R.as_quat()],force,.025)
 s[2]+=.005;m.step(s,np.r_[b+[0,0,.01],R.as_quat()],0,.05)
 return m
class PhysicalGradingTests(unittest.TestCase):
    def setUp(self):
        # Adjacent 0.2 mm pads separated by a 0.2 mm gap. The visible
        # deposit elements are each 0.1 mm square.
        data = dict(board_bbox_m=[.06858, .05334, .0016], pads=[
            dict(xy_m=[x, 0.], size_m=[.0002, .0001]) for x in [-.0002, .0002]
        ])
        self.paste = StencilPaste(data)
        self.paste.released = True

    def deposit_at(self, offsets):
        m = self.paste
        m.deposit[:] = 0.
        for i, offset in enumerate(offsets):
            m.deposit[i] = m.capacity[i]
            m.deposit_xy[i] = np.array([330., -65.]) + offset
        m.metrics_cache = None
        return m.metrics()

    def test_narrow_visible_bridge_counts_but_two_isolated_spots_do_not(self):
        # Only 0.005 mm² of each pad is wetted by this continuous filament.
        bridged = self.deposit_at([[-.1, 0], [0, 0], [.1, 0]])
        self.assertEqual(bridged['bridges'], 1)
        self.assertEqual(self.deposit_at([[-.1, 0], [.1, 0]])['bridges'], 0)

    def test_mass_on_scale_excludes_fully_and_partly_off_board_paste(self):
        mass = self.paste.capacity[0] * 4.2
        for x, expected in [(0., mass), (34.29, mass/2), (35., 0.)]:
            with self.subTest(x=x):
                self.assertAlmostEqual(self.deposit_at([[x, 0.]])['deposited_mass_mg'], expected)


class PasteTests(unittest.TestCase):
 def test_loaded_contact_prints_conservatively(self):
  m=print_case();v=m.metrics();self.assertGreater(v['qualified_pad_fraction'],.9);self.assertGreater(v['deposited_mass_mg'],50);self.assertEqual(v['bridges'],0);self.assertLess(v['conservation_error_ul'],1e-7)
 def test_hover_no_load_and_empty_cannot_print(self):
  for kw in [{'force':0},{'force':.8},{'gap':.002},{'volume':0}]:self.assertEqual(print_case(**kw).metrics()['deposited_ul'],0)
 def test_native_scale_jitter_does_not_throw_bead_behind_blade(self):
  m=StencilPaste(DATA);s=np.array([.33,-.065,.7805625,0,0,0,1.]);initial=m.roll_y.copy();radius=np.cbrt(m.reservoir[0]*3/(4*np.pi));y0=(-24-radius)/1000
  for i in range(100):
   y=y0+(1e-6 if i%2 else 0);b=s[:3]+[0,y,.0000625]-R.apply([0,-.00025,-.0325]);m.step(s,np.r_[b,R.as_quat()],8,.0125)
  self.assertLess(np.max(np.abs(m.roll_y-initial)),.002)
 def test_blade_ahead_of_bead_cannot_pull_it_forward(self):
  m=StencilPaste(DATA);s=np.array([.33,-.065,.7805625,0,0,0,1.]);old=m.roll_y.copy()
  for y in np.linspace(-.023,.03,200):
   b=s[:3]+[0,y,.0000625]-R.apply([0,-.00025,-.0325]);m.step(s,np.r_[b,R.as_quat()],8,.025)
  np.testing.assert_array_equal(m.roll_y,old);self.assertEqual(m.fill.sum(),0.)
 def test_registration_comes_from_actual_positions(self):self.assertLess(print_case(shift=.001).metrics()['qualified_pad_fraction'],.5)
 def test_insufficient_mass_cannot_receive_full_grade(self):self.assertLess(print_case(volume=2).metrics()['deposited_mass_mg'],12.5)
 def test_overlapping_footprints_not_double_counted(self):
  m=print_case();m.deposit_xy[:]=m.deposit_xy[0];m.metrics_cache=None;self.assertLess(m.metrics()['mean_pad_coverage'],.01)
 def test_post_print_contact_can_destroy_registration(self):
  m=print_case();s=m.last_stencil;before=m.metrics()['qualified_pad_fraction'];m.previous_world_edge=None
  for y in np.linspace(-.095,-.035,200):
   b=np.array([.33,y,.7805])-R.apply([0,-.00025,-.0325]);m.step(s,np.r_[b,R.as_quat()],0,.05,board_force=1.)
  self.assertLess(m.metrics()['qualified_pad_fraction'],before)
if __name__=='__main__':unittest.main()
