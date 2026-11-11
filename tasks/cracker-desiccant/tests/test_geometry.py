"""Exercise measured-state classifier with deterministic geometric fixtures."""
import itertools,sys,unittest
from pathlib import Path
from types import SimpleNamespace
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from demo_runtime import StateReader, projected_overlap_m2
from scoring import CRACKERS

class GeometryTests(unittest.TestCase):
    def reader(self):
        r=StateReader.__new__(StateReader)
        r._refresh=lambda:None;r._controls_cache=None
        r.specs={'serving_basket':SimpleNamespace(position=(0,0,0))}
        r.vertices={n:np.array(list(itertools.product([-.021,.021],[-.021,.021],[0,.007]))) for n in CRACKERS}
        r.vertices['desiccant_packet']=np.array(list(itertools.product([-.0275,.0275],[-.017,.017],[0,.012])))
        origins={n:np.array([(-.045 if i%2==0 else .045),0,.012+.007*(i//2)]) for i,n in enumerate(CRACKERS)}
        origins['desiccant_packet']=np.array([-.3,0,.012])
        r.test_origins=origins;r.test_tcp=np.array([1.,1.,1.])
        def body(n):
            p=r.vertices[n]+origins[n]
            return dict(pose=np.r_[origins[n],0,0,0,1],rotation=np.eye(3),points=p,
                velocity=np.zeros(6),low=p.min(0),high=p.max(0))
        r.body=body;r.arm=lambda side:(None,None,r.test_tcp,None)
        return r
    def test_real_rounded_stack_overlap(self):
        import json
        fixture=json.loads((Path(__file__).parent/'fixtures/rounded_stack.json').read_text())
        a,b=[np.array(h,np.float32).reshape(-1,1,2) for h in fixture['hulls']]
        self.assertAlmostEqual(projected_overlap_m2(a,b),fixture['expected_overlap_m2'],places=8)
    def test_floor_and_stacked_crackers(self):
        r=self.reader();c=r.controls()
        self.assertTrue(all(c['eligible'].values()));self.assertFalse(c['packet_in_basket'])
    def test_held_cracker_is_not_delivery(self):
        r=self.reader();r.test_tcp=r.test_origins['cracker_1'].copy()
        self.assertFalse(r.controls()['eligible']['cracker_1'])
    def test_hovering_and_tray_supported_food_not_delivery(self):
        r=self.reader()
        for n in CRACKERS:r.test_origins[n][2]+=.012
        self.assertFalse(any(r.controls()['eligible'].values()))
    def test_overhanging_cracker_rejected(self):
        r=self.reader();r.test_origins['cracker_1'][0]=.13
        self.assertFalse(r.controls()['eligible']['cracker_1'])
    def test_packet_inside_veto(self):
        r=self.reader();r.test_origins['desiccant_packet']=np.array([0,0,.012])
        self.assertTrue(r.controls()['packet_in_basket'])
    def test_packet_on_rim_veto(self):
        r=self.reader();r.test_origins['desiccant_packet']=np.array([.13,0,.061])
        self.assertTrue(r.controls()['packet_in_basket'])
    def test_packet_above_basket_not_yet_entry(self):
        r=self.reader();r.test_origins['desiccant_packet']=np.array([0,0,.10])
        self.assertFalse(r.controls()['packet_in_basket'])
    def test_packet_beside_basket_is_allowed(self):
        r=self.reader();r.test_origins['desiccant_packet']=np.array([.17,0,.012])
        self.assertFalse(r.controls()['packet_in_basket'])

if __name__=='__main__':unittest.main()
