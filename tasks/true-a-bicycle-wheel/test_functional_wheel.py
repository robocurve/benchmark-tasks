import unittest
import numpy as np
from functional_wheel import Model
from wheel_scene import spoke_phase


def metadata():
    spokes=[]
    for i in range(32):
        theta=i*2*np.pi/32;side=1 if i%2==0 else -1
        spokes.append(dict(hub_anchor_local_m=[.025*np.cos(theta+side*.6),.025*np.sin(theta+side*.6),side*.025],
            **{k:f'{i}/{k}' for k in ('rim_path','inner_path','outer_path','nipple_path')}))
    return dict(reset_profile='two-defect-candidate',wheel_clocking_rad=0.,
                nipple_flat_phase_rad=0.,axle_position_m=[0,0,0],hub_path='hub',spokes=spokes)


class TestModel(unittest.TestCase):
    def test_key_turn_retains_and_wrong_direction_worsens(self):
        values=[]
        for direction in (-1,1):
            m=Model(metadata());p,R=m.geometry()['0/nipple_path']
            T=R[:,[1,2,0]]
            m.step(p,T,[],.05)
            self.assertEqual(m.engaged,0)
            for angle in np.linspace(0,direction*np.pi/2,21)[1:]:
                m.step(p,R@spoke_phase(angle)@R.T@T,[],.05)
            expected=-.75-direction*.25
            self.assertAlmostEqual(m.turns[0],expected,places=3)
            before=m.turns.copy()
            m.step(p+[.1,0,0],T,[],.05)
            for _ in range(20):m.step(p+[.1,0,0],T,[],.05)
            np.testing.assert_array_equal(m.turns,before)
            values.append(m.result['tension_n'][0])
        self.assertGreater(values[0],values[1])

    def test_no_contact_no_adjustment(self):
        m=Model(metadata());before=m.turns.copy()
        for _ in range(20):m.step(np.array([1,1,1]),np.eye(3),[],.05)
        np.testing.assert_array_equal(m.turns,before)


if __name__=='__main__':unittest.main()
