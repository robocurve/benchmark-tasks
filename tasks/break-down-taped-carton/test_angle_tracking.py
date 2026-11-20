"""Branch integrity tests; these do not certify native integration."""
import math
import unittest
import numpy as np
from angle_tracking import (ContinuousBottomAngles, BODIES, WALLS, PHYSICS_DT,
                            evidence_error)


def state(angles=(0.,)*4, world=0.):
    poses={}; velocities={}
    for wall,angle in zip(WALLS,angles):
        # Common rotation about X cancels in the relative hinge measurement.
        for name,degrees in ((wall,world),(wall+'_bottom',world+angle)):
            a=math.radians(degrees)/2
            poses[name]=[0.,0.,0.,math.sin(a),0.,0.,math.cos(a)]
            velocities[name]=[0.]*6
    return poses,velocities


class AngleTrackingTests(unittest.TestCase):
    def tracker(self):
        t=ContinuousBottomAngles({n:5729.58 for n in BODIES})
        self.assertTrue(t.initialize(0,0.,*state()))
        return t

    def walk(self,t,targets,steps=24,world=0.):
        start=t.angles.copy(); offset=t.counter
        for j in range(1,steps+1):
            values=start+(np.asarray(targets)-start)*j/steps
            k=offset+j
            self.assertTrue(t.advance(k,k*PHYSICS_DT,*state(values,world*j/steps)),t.failure)

    def test_lower_stop_overshoot_never_becomes_open(self):
        t=self.tracker();self.walk(t,[-85.1]*4)
        np.testing.assert_allclose(t.angles,[-85.1]*4,atol=1e-10)
        self.assertFalse(all(t.angles>=150.))

    def test_real_opening_crosses_principal_branch(self):
        t=self.tracker();self.walk(t,[274.9]*4,96)
        np.testing.assert_allclose(t.angles,[274.9]*4,atol=1e-10)
        self.walk(t,[150.]*4,48)
        np.testing.assert_allclose(t.angles,[150.]*4,atol=1e-10)

    def test_common_body_rotation_cancels(self):
        t=self.tracker();self.walk(t,[160.,170.,180.,190.],96,world=270.)
        np.testing.assert_allclose(t.angles,[160.,170.,180.,190.],atol=1e-10)

    def test_gap_is_sticky_and_atomic(self):
        t=self.tracker();before=t.angles.copy()
        self.assertFalse(t.advance(2,2*PHYSICS_DT,*state([5.]*4)))
        self.assertFalse(t.advance(1,PHYSICS_DT,*state([5.]*4)))
        np.testing.assert_array_equal(t.angles,before)

    def test_reset_reseed_is_rejected(self):
        t=self.tracker();self.assertFalse(t.initialize(0,0.,*state()))
        self.assertFalse(t.evidence(1,12)['valid'])

    def test_unknown_initial_branch_is_rejected(self):
        t=ContinuousBottomAngles({n:5729.58 for n in BODIES})
        self.assertFalse(t.initialize(0,0.,*state([270.]*4)))

    def test_rate_violation_is_rejected(self):
        t=self.tracker();self.assertFalse(t.advance(1,PHYSICS_DT,*state([80.]*4)))

    def test_velocity_violation_is_rejected(self):
        t=self.tracker();p,v=state();v['north'][3]=101.
        self.assertFalse(t.advance(1,PHYSICS_DT,p,v))

    def test_missing_body_is_rejected(self):
        t=self.tracker();p,v=state();del p['west_bottom']
        self.assertFalse(t.advance(1,PHYSICS_DT,p,v))

    def test_control_evidence_requires_full_fresh_interval(self):
        t=self.tracker();self.walk(t,[12.]*4,12)
        e=t.evidence(1,12)
        self.assertIsNone(evidence_error(e,t.angles,.05,1))
        for key,value in [('substeps_in_control',11),('counter',11),('time_s',.045),('angles_deg',[0.]*4),('control_index',2)]:
            with self.subTest(key=key):
                bad=dict(e);bad[key]=value
                self.assertIsNotNone(evidence_error(bad,t.angles,.05,1))

    def test_historical_wrapped_logs_are_not_evidence(self):
        self.assertIsNotNone(evidence_error(None,[274.]*4,.05,1))

    def test_nonunique_sampling_rate_is_rejected(self):
        with self.assertRaises(ValueError):
            ContinuousBottomAngles({n:50000. for n in BODIES})


    def test_arbitrary_common_world_rotation_cancels(self):
        t=self.tracker()
        def multiply(a,b):
            va,wa=np.array(a[:3]),a[3];vb,wb=np.array(b[:3]),b[3]
            return [*(wa*vb+wb*va+np.cross(va,vb)),wa*wb-np.dot(va,vb)]
        axis=np.array([1.,2.,3.]);axis/=np.linalg.norm(axis)
        for k in range(1,97):
            p,v=state([k*2.]*4)
            half=math.radians(k*3.)/2
            q=[*(axis*math.sin(half)),math.cos(half)]
            for n in BODIES:p[n][3:]=multiply(q,p[n][3:])
            self.assertTrue(t.advance(k,k*PHYSICS_DT,p,v),t.failure)
        np.testing.assert_allclose(t.angles,[192.]*4,atol=1e-10)

    def test_timestamp_gap_rejected(self):
        t=self.tracker()
        self.assertFalse(t.advance(1,2*PHYSICS_DT,*state()))

    def test_nonfinite_data_rejected(self):
        t=self.tracker();p,v=state();p['east'][3]=float('nan')
        self.assertFalse(t.advance(1,PHYSICS_DT,p,v))


if __name__=='__main__':unittest.main()
