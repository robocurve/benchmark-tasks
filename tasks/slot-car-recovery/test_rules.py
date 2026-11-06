"""Pure scoring/geometry checks. These do not validate native physics or contacts."""
import importlib.util
import json
import math
from pathlib import Path
import sys
import unittest

import numpy as np

FOLDER=Path(__file__).parent
sys.path.insert(0,str(FOLDER))
spec=importlib.util.spec_from_file_location('_slot_rules',FOLDER/'task.py',submodule_search_locations=[str(FOLDER)])
task=importlib.util.module_from_spec(spec);sys.modules[spec.name]=task;spec.loader.exec_module(task)


class Rules(unittest.TestCase):
    def test_user_scores_and_contact_override(self):
        for yellow,blue,want in ((False,False,0),(True,False,.5),(False,True,.5),(True,True,1)):
            inserted=dict(yellow=yellow,blue=blue)
            self.assertEqual(task.score(inserted,False),want)
            self.assertEqual(task.score(inserted,True),0)

    def test_assigned_gripper_release(self):
        for name,arm,other in (('yellow','LeftArm','RightArm'),('blue','RightArm','LeftArm')):
            self.assertTrue(task.assigned_release(name,{arm},{arm}))
            self.assertFalse(task.assigned_release(name,{other},{other}))
            self.assertFalse(task.assigned_release(name,{arm,other},{arm,other}))
            self.assertFalse(task.assigned_release(name,{arm},set()))
            self.assertFalse(task.assigned_release(name,set(),set()))

    def test_oval_progress_and_native_float_serialization(self):
        r=task.RADIUS; h=task.HALF_STRAIGHT
        for point,want in (((.4,-h),0),((.4,h),2*h),((.8,h),2*h+math.pi*r),((.8,-h),4*h+math.pi*r)):
            offset,s=task.track_coordinates(np.asarray(point,dtype=np.float32))
            self.assertAlmostEqual(offset,0,places=6)
            self.assertAlmostEqual(s,want,places=6)
            json.dumps(dict(offset=offset,distance=s))

    def test_documented_instruction_matches_runner(self):
        draft=json.loads((FOLDER/'task.json').read_text())
        pilot=json.loads((FOLDER/'pilot.json').read_text())
        self.assertEqual(draft['instruction'],pilot['instruction'])
        self.assertIn(draft['instruction'],(FOLDER/'README.md').read_text())

    def test_road_surface_is_closed_and_faces_up(self):
        from collections import Counter
        from _slot_rules.geometry import oval_strip
        shape=oval_strip(.202,.243,.75,.76)
        points=np.asarray(shape['mesh']['points'])
        upward=0;edges=Counter()
        for face in shape['mesh']['faces']:
            p=points[face]
            if np.allclose(p[:,2],.76):
                self.assertGreater(np.cross(p[1]-p[0],p[2]-p[0])[2],0)
                upward+=1
            for a,b in zip(face,face[1:]+face[:1]):edges[tuple(sorted((a,b)))]+=1
            # No hidden cap across either curve/straight join.
            self.assertFalse(np.allclose(p[:,1],task.HALF_STRAIGHT))
            self.assertFalse(np.allclose(p[:,1],-task.HALF_STRAIGHT))
        self.assertEqual(upward,362)
        self.assertEqual(set(edges.values()),{2})

    def test_off_track_reset_does_not_bury_guide(self):
        from _slot_rules.geometry import OFF_TRACK_PITCH,OFF_TRACK_Z,WHEEL_RADIUS
        t=OFF_TRACK_PITCH
        rear_bottom=OFF_TRACK_Z-.02968*math.sin(t)-WHEEL_RADIUS
        guide_bottom=OFF_TRACK_Z+.038*math.sin(t)-.0115*math.cos(t)
        self.assertAlmostEqual(rear_bottom,.7476,places=8)
        self.assertAlmostEqual(guide_bottom,.7476,places=8)

    def test_guide_is_closed_outward_convex_geometry(self):
        from collections import Counter
        from _slot_rules.geometry import guide_shape,geometry
        assets,_=geometry()
        for name in ('red','yellow','blue'):
            guide=assets[name]['bodies'][-1]
            self.assertTrue(np.all(np.asarray(guide['inertia']['diagonal'])>0))
        shape=guide_shape();points=np.asarray(shape['mesh']['points'])
        np.testing.assert_allclose(np.ptp(points,axis=0),[.012,.0016,.005])
        self.assertLess(len(points),256)
        edges=Counter()
        for face in shape['mesh']['faces']:
            p=points[face];normal=np.cross(p[1]-p[0],p[2]-p[0])
            self.assertGreater(np.dot(normal,p.mean(axis=0)),0)
            self.assertLessEqual(np.max((points-p[0])@normal),1e-14)
            for a,b in zip(face,face[1:]+face[:1]):edges[tuple(sorted((a,b)))]+=1
        self.assertEqual(set(edges.values()),{2})


if __name__=='__main__':unittest.main()
