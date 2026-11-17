"""Adversarial geometry and temporal tests, not physical success evidence."""
import sys,unittest
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from carton_probe import geometry as G,folding as F
from carton_probe.scoring import Outcome

class ScoreTests(unittest.TestCase):
    def make(self,beta=65,outer=120):
        G.MECHANICAL_ONLY=True
        c=G.contract();names=list(G.CENTRES);s=F.poses(outer,beta)
        q=np.array([np.r_[np.array(s[n]['center'])+G.ORIGIN,s[n]['quat_xyzw']] for n in names])
        support=np.zeros((len(names),1),int);support[names.index('carton_body')]=4
        return Outcome(c,names),q,support
    def test_actual_initial_state_and_overfold_stay_zero(self):
        for beta in [0,90]:
            s,q,t=self.make(beta)
            for _ in range(900):v=s.update(q,0,t)
            self.assertEqual(v,0)
    def test_three_seconds_is_721_poses(self):
        s,q,t=self.make()
        for _ in range(720):self.assertEqual(s.update(q,0,t),.5)
        self.assertEqual(s.update(q,0,t),1)
    def test_held_or_unsupported_opening_only_half(self):
        for contact,supported in [(1,True),(0,False)]:
            s,q,t=self.make()
            if not supported:t*=0
            for _ in range(900):v=s.update(q,contact,t)
            self.assertEqual(v,.5)
    def test_release_interruption_restarts_full_three_seconds(self):
        s,q,t=self.make()
        for _ in range(700):s.update(q,0,t)
        s.update(q,1,t)
        for _ in range(720):self.assertEqual(s.update(q,0,t),.5)
        self.assertEqual(s.update(q,0,t),1)
    def test_initial_same_outer_fin_orientation_is_not_open(self):
        s,q,t=self.make(0,62.3335)
        for _ in range(900):v=s.update(q,0,t)
        self.assertEqual(v,0)
    def test_small_motion_with_functional_opening_is_not_penalized(self):
        s,q,t=self.make()
        for k in range(900):
            moving=q.copy();moving[:,:3]+=[.00001*k,0,0];v=s.update(moving,0,t)
        self.assertEqual(v,1)
    def test_slow_refolding_below_opening_threshold_loses_full_credit(self):
        s,q,t=self.make()
        for _ in range(721):s.update(q,0,t)
        self.assertEqual(s.value,1)
        for beta in np.linspace(65,55,400):
            _,moving,_=self.make(beta);v=s.update(moving,0,t)
        self.assertEqual(v,.25)
        self.assertEqual(s.detail['qualified_pose_count'],0)
    def test_rigid_global_yaw_translation_does_not_change_result(self):
        s,q,t=self.make();r=Rotation.from_euler('z',73,degrees=True)
        q[:,:3]=r.apply(q[:,:3])+[.05,-.1,0];q[:,3:]=(r*Rotation.from_quat(q[:,3:])).as_quat()
        for _ in range(721):v=s.update(q,0,t)
        self.assertEqual(v,1)
    def test_upsidedown_carton_is_zero(self):
        s,q,t=self.make();r=Rotation.from_euler('x',180,degrees=True)
        q[:,:3]=r.apply(q[:,:3]);q[:,3:]=(r*Rotation.from_quat(q[:,3:])).as_quat()
        self.assertEqual(s.update(q,0,t),0)
    def test_detached_facet_latches_zero_even_if_replaced(self):
        s,q,t=self.make();bad=q.copy();bad[s.index['carton_inner_fin_minus'],0]+=.003
        self.assertEqual(s.update(bad,0,t),0)
        for _ in range(721):v=s.update(q,0,t)
        self.assertEqual(v,0);self.assertIsNotNone(s.invalid_reason)
    def test_invalid_data_and_capacity_never_recover(self):
        for badkind in ['nan','contactnan','supportnan','capacity','broken']:
            s,q,t=self.make();bad=q.copy();contact=0;t=t.astype(float);broken=False
            if badkind=='nan':bad[0,0]=np.nan
            if badkind=='contactnan':contact=np.nan
            if badkind=='supportnan':t[0,0]=np.nan
            if badkind=='capacity':contact=65536
            if badkind=='broken':broken=True
            self.assertEqual(s.update(bad,contact,t,broken=broken),0)
            self.assertIsNotNone(s.invalid_reason)
    def test_incomplete_body_coverage_rejected(self):
        with self.assertRaises(ValueError):Outcome(G.contract(),list(G.CENTRES)[:-1])
    def test_upper_pouring_lip_must_cross_wall(self):
        s,q,t=self.make(55)
        self.assertEqual(s.update(q,0,t),.25)
        self.assertGreater(s.detail['apex_projection_mm'],8)
        self.assertLess(s.detail['upper_lip_projection_mm'],8)
    def test_quaternion_roundoff_does_not_create_motion(self):
        s,q,t=self.make()
        for k in range(721):
            v=q.copy();v[:,3:]*=1-1e-4*(k%2)
            score=s.update(v,0,t)
        self.assertEqual(score,1)

    def test_small_yaw_with_functional_opening_is_not_penalized(self):
        s,q,t=self.make()
        for k in range(721):
            r=Rotation.from_euler('z',3*k/720,degrees=True)
            moving=q.copy()
            moving[:,:3]=r.apply(q[:,:3]-G.ORIGIN)+G.ORIGIN
            moving[:,3:]=(r*Rotation.from_quat(q[:,3:])).as_quat()
            v=s.update(moving,0,t)
        self.assertEqual(v,1)

    def test_shape_changes_that_keep_the_opening_usable_are_permitted(self):
        s,_,t=self.make()
        for k in range(721):
            _,q,_=self.make(64+2*k/720)
            v=s.update(q,0,t)
        self.assertEqual(v,1)
        self.assertTrue(s.detail['open_form'])

    def test_closure_after_full_score_loses_full_credit(self):
        s,q,t=self.make()
        for _ in range(721):s.update(q,0,t)
        _,closed,_=self.make(0)
        self.assertEqual(s.update(closed,0,t),0)
        for _ in range(720):self.assertEqual(s.update(q,0,t),.5)
        self.assertEqual(s.update(q,0,t),1)

    def test_irrelevant_outer_fin_rotation_is_permitted(self):
        s,q,t=self.make();name='carton_fin_minus';i=s.index[name]
        roof=s.index['carton_roof_minus'];r0=Rotation.from_quat(q[roof,3:])
        edge=r0.apply(np.array([G.TIP,G.INNER])-G.CENTRES['carton_roof_minus'])+q[roof,:3]
        axis=edge[1]-edge[0];axis/=np.linalg.norm(axis)
        for k in range(721):
            r=Rotation.from_rotvec(axis*np.deg2rad(3*np.sin(k/30)))
            moving=q.copy();moving[i,:3]=r.apply(q[i,:3]-edge[0])+edge[0]
            moving[i,3:]=(r*Rotation.from_quat(q[i,3:])).as_quat()
            v=s.update(moving,0,t)
        self.assertEqual(v,1)
        self.assertIsNone(s.invalid_reason)

    def test_support_interruption_restarts_hold_even_with_identical_pose(self):
        s,q,t=self.make()
        for _ in range(721):s.update(q,0,t)
        self.assertEqual(s.update(q,0,t*0),.5)
        for _ in range(720):self.assertEqual(s.update(q,0,t),.5)
        self.assertEqual(s.update(q,0,t),1)

    def test_dipping_below_threshold_then_recovering_restarts_hold(self):
        s,q,t=self.make()
        for _ in range(721):s.update(q,0,t)
        _,low_lip,_=self.make(55)
        self.assertEqual(s.update(low_lip,0,t),.25)
        for _ in range(720):self.assertEqual(s.update(q,0,t),.5)
        self.assertEqual(s.update(q,0,t),1)

    def test_tipping_an_open_carton_resets_full_credit(self):
        s,q,t=self.make()
        for _ in range(721):s.update(q,0,t)
        r=Rotation.from_euler('y',6,degrees=True);tipped=q.copy()
        tipped[:,:3]=r.apply(q[:,:3]-G.ORIGIN)+G.ORIGIN
        tipped[:,3:]=(r*Rotation.from_quat(q[:,3:])).as_quat()
        self.assertEqual(s.update(tipped,0,t),0)
        self.assertEqual(s.detail['qualified_pose_count'],0)

if __name__=='__main__':unittest.main()
