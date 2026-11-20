import unittest
import numpy as np
from turnover_contacts import summarize,TARGETS,Reader
from types import SimpleNamespace
class Tests(unittest.TestCase):
 def test_force_family_sum_does_not_rely_on_filter_order(self):
  a=np.arange(6*len(TARGETS),dtype=float).reshape(1,2,len(TARGETS),3)/10
  x=summarize(a,['tip_left','tip_right']);y=summarize(a[:,:,::-1],['tip_left','tip_right'])
  np.testing.assert_allclose(x['family_normal_force_on_fingertips_w_N'],y['family_normal_force_on_fingertips_w_N'])
  self.assertFalse(x['individual_filter_column_order_native_verified'])
 def test_opposed_sensor_reactions_remain_opposed(self):
  a=np.zeros((1,2,len(TARGETS),3));a[0,0,0,1]=2.
  l=summarize(a,['tip_left','tip_right']);r=summarize(-a,['tip_right','tip_left'])
  self.assertLess(np.dot(l['family_normal_force_on_fingertips_w_N'],r['family_normal_force_on_fingertips_w_N']),0.)
 def test_balanced_clamp_has_nonzero_force_magnitudes_even_if_net_zero(self):
  a=np.zeros((1,2,len(TARGETS),3));a[0,0,0,0]=2.;a[0,1,0,0]=-2.
  r=summarize(a,['tip_left','tip_right']);self.assertEqual(r['family_normal_force_norm_N'],0.);self.assertEqual(r['family_sum_of_normal_force_magnitudes_N'],4.)
 def test_bad_shape_wrong_body_and_nonfinite_rejected(self):
  a=np.zeros((1,2,len(TARGETS),3))
  for value,names in [(a[:,:,:3],['tip_left','tip_right']),(a,['base','tip_left']),(a+np.nan,['tip_left','tip_right'])]:
   with self.assertRaises(ValueError):summarize(value,names)
class ReaderTests(unittest.TestCase):
 def make(self):
  sensors={side:SimpleNamespace(data=SimpleNamespace(force_matrix_w=np.zeros((1,2,len(TARGETS),3))),body_names=['tip_left','tip_right'],_timestamp=np.array([0.]),_timestamp_last_update=np.array([0.]))for side in ('left','right')}
  base=SimpleNamespace(scene={s+'_carton_contacts':v for s,v in sensors.items()},sim=SimpleNamespace(get_physics_dt=lambda:1/240))
  return Reader(base),sensors
 def test_repeated_pre_post_reads_do_not_create_new_measurements(self):
  r,s=self.make();a=r.read(0.,'pre_action');b=r.read(0.,'post_step');self.assertEqual(a['sample_id'],b['sample_id'])
  for x in s.values():x._timestamp[:]=.05;x._timestamp_last_update[:]=.05
  c=r.read(.05,'post_step');d=r.read(.05,'pre_action');self.assertEqual(c['sample_id'],a['sample_id']+1);self.assertEqual(c['sample_id'],d['sample_id']);self.assertTrue(d['clock_verified'])
 def test_request_time_cannot_make_stale_native_data_fresh(self):
  r,s=self.make();r.read(0.)
  with self.assertRaisesRegex(ValueError,'stale'):r.read(.05)
 def test_unupdated_unsynchronised_and_reset_clocks_fail(self):
  for mode in ('unupdated','unsync','reset'):
   r,s=self.make();r.read(0.)
   for x in s.values():x._timestamp[:]=.05;x._timestamp_last_update[:]=.05
   if mode=='unupdated':s['left']._timestamp_last_update[:]=0.
   elif mode=='unsync':s['right']._timestamp[:]=.04;s['right']._timestamp_last_update[:]=.04
   else:
    r.read(.05)
    for x in s.values():x._timestamp[:]=0.;x._timestamp_last_update[:]=0.
   with self.assertRaises(ValueError):r.read(.1 if mode=='reset'else .05)
if __name__=='__main__':unittest.main()
