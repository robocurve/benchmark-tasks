"""CPU regressions for the side-contact bug; no simulator success claims."""
import json
from pathlib import Path
import unittest
from unittest.mock import patch
import numpy as np
from switch_geometry import FingerGeometry, clipped_support, minimum_separation
from robot import DryerProbe, LEFT_R, HANDLE_GRASP_LOCAL, HANDLE_APPROACH_OFFSET
from yam_arena import ik
from yam_arena.constants import READY
from config import DRYER_START


class CollisionSupportTests(unittest.TestCase):
    def test_triangle_interior_counts_without_vertices_in_footprint(self):
        # Entire rectangular footprint is inside this triangle, though every
        # triangle vertex is outside. Surface is z=x+2y, so extrema are exact.
        triangle = np.array([[[-2,-2,-6],[2,-2,-2],[0,3,6]]])
        lo,hi = clipped_support(triangle,[-.1,-.1],[.1,.1])
        self.assertAlmostEqual(lo,-.3)
        self.assertAlmostEqual(hi,.3)
        self.assertIsNone(clipped_support(triangle,[4,4],[5,5]))

    def test_old_point_penetrates_side_new_support_clears_bottom(self):
        spec = json.loads(Path(__file__).with_name('switch_point.json').read_text())
        rotation,point = np.array(spec['rotation']),np.array(spec['point_tcp'])
        geometry = FingerGeometry()
        slider = np.array((-.037,0,-.045))
        old_tcp = np.array((-.039,0,-.0545))-rotation@point
        old = geometry.support([-.028,-.028],old_tcp,rotation,slider)
        self.assertGreater(old[1]-(-.052),.005)
        tcp = np.array((-.0375,0,0))-rotation@point
        tcp[2] = 0.
        support = geometry.support([-.028,-.028],tcp,rotation,slider)
        tcp[2] = -.052-.001-support[1]
        new = geometry.support([-.028,-.028],tcp,rotation,slider)
        self.assertAlmostEqual(new[1],-.053,places=10)
        # Jaw readback matters: changing joint8 shifts the actual support.
        moved = geometry.support([-.028,-.030],tcp,rotation,slider)
        self.assertNotEqual(new,moved)

    def test_timed_alignment_never_advances_without_measured_settle(self):
        probe = DryerProbe.__new__(DryerProbe)
        probe.phase,probe.phase_start,probe.stable_frames = 'align_below_switch',0.,0
        probe.events,probe.bias,probe.failure = [],[np.zeros(3),np.zeros(3)],None
        target = (np.zeros(3),np.eye(3))
        actual = (np.array((.042,0,0)),np.eye(3))
        for t in (4.,4.05,5.,7.,8.9):
            settled = probe.settled(actual,target,np.array((-.028,-.028)),-.028)
            probe.gate(settled,t,4.,5.,'insert_below_switch',t)
        self.assertEqual(probe.phase,'align_below_switch')
        self.assertEqual(probe.stable_frames,0)
        probe.gate(False,9.05,4.,5.,'insert_below_switch',9.05)
        self.assertIn('did not settle',probe.failure)

    def test_phase_requires_five_consecutive_correct_measured_frames(self):
        probe = DryerProbe.__new__(DryerProbe)
        probe.phase,probe.phase_start,probe.stable_frames = 'insert_below_switch',0.,0
        probe.events,probe.bias,probe.failure = [],[np.zeros(3),np.ones(3)*.003],None
        for i,condition in enumerate((True,True,False,True,True,True,True)):
            probe.gate(condition,2+i*.05,2.,5.,'push_slider_up',2+i*.05)
        self.assertEqual(probe.phase,'insert_below_switch')
        probe.gate(True,2.35,2.,5.,'push_slider_up',2.35)
        self.assertEqual(probe.phase,'push_slider_up')
        self.assertTrue(np.all(probe.bias[1]==0.))


class LoadedBimanualClearanceTests(unittest.TestCase):
    def test_loaded_hands_clear_entire_insert_stroke_retreat_volume(self):
        geometry = FingerGeometry()
        spec = json.loads(Path(__file__).with_name('switch_point.json').read_text())
        rotation,point = np.array(spec['rotation']),np.array(spec['point_tcp'])
        tcp = np.array((-.0375,0,0))-rotation@point
        tcp[2] = 0.
        support = geometry.support([-.028,-.028],tcp,rotation,np.array((-.037,0,-.045)))
        tcp[2] = -.053-support[1]
        sweep = [np.concatenate([a@rotation.T+tcp+[x,0,z]
                                for x in (-.025,0.) for z in (0.,.008)])
                 for _,a in geometry.pieces([-.028,-.028])]
        for jaw in (-.012,-.015,-.018,-.020):
            left = [a@LEFT_R.T+HANDLE_GRASP_LOCAL for _,a in geometry.pieces([jaw,jaw])]
            self.assertGreater(minimum_separation(left,sweep),.005)
        old = [a@LEFT_R.T+[-.019,-.003,-.065] for _,a in geometry.pieces([-.015,-.015])]
        self.assertEqual(minimum_separation(old,sweep),0.)

    def test_lower_handle_entry_and_lift_are_reachable(self):
        q = np.array(READY)
        for offset in (HANDLE_APPROACH_OFFSET,np.zeros(3),np.array((0,0,.1))):
            target = np.array(DRYER_START)+HANDLE_GRASP_LOCAL+offset
            q,position_error,rotation_error = ik.solve(target,ik.matrix_to_quat_xyzw(LEFT_R),q,
                                                       base_pos=(0,.305,.75),iters=300)
            self.assertLess(position_error,.0001)
            self.assertLess(rotation_error,.001)
