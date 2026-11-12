"""Synthetic reports exercise filtering; these are not dynamic validation."""
import math
from types import SimpleNamespace as NS
import unittest
import numpy as np
from ratchet_task.config import AXIS, TABLE, HEAD_HEIGHT, PITCH, SEAT_STOP_GAP
from ratchet_task.mechanics import Screw
from ratchet_task.seating import measure_seat, SeatWindow


class Seating(unittest.TestCase):
    def report(self, *, gap=0., pair=(10,20), point=None, normal=(0,0,1),
               impulse=(0,0,.001), separation=0., rotation=None, reverse=False):
        if point is None:point=AXIS+np.array([.006,0,0])
        if reverse:pair=pair[::-1];normal=-np.array(normal);impulse=-np.array(impulse)
        h=NS(collider0=pair[0],collider1=pair[1],contact_data_offset=0,num_contact_data=1)
        c=NS(position=point,normal=normal,impulse=impulse,separation=separation)
        return measure_seat([h],[c],head_id=10,bracket_ids={20:'fixture/Bracket/Sector00'},
            bolt=AXIS+np.array([0,0,gap]),bolt_rotation=np.eye(3) if rotation is None else rotation,
            fixture=np.array([AXIS[0],AXIS[1],TABLE]),fixture_rotation=np.eye(3),dt=1/480)

    def test_correct_surfaces_in_either_collider_order(self):
        for reverse in [False,True]:
            r=self.report(reverse=reverse)
            self.assertTrue(r['seat_contact']);self.assertEqual(r['seat_contact_points'],1)
            self.assertAlmostEqual(r['seat_force_n'],.48)
            self.assertAlmostEqual(r['seat_gap_m'],0.)

    def test_arbitrary_bolt_contacts_are_rejected(self):
        for pair in [(11,20),(12,20),(10,30),(30,20)]:
            self.assertFalse(self.report(pair=pair)['seat_contact'])

    def test_head_top_side_bore_and_receiver_are_rejected(self):
        for delta in [[.006,0,HEAD_HEIGHT],[.0065,0,.002],[0,0,0],[.006,0,-.003]]:
            self.assertFalse(self.report(point=AXIS+delta)['seat_contact'])
        self.assertFalse(self.report(normal=(1,0,0))['seat_contact'])

    def test_speculative_contacts_and_zero_impulse_do_not_count(self):
        self.assertFalse(self.report(gap=.0001961,separation=.0001961)['seat_contact'])
        self.assertFalse(self.report(separation=.00001)['seat_contact'])
        self.assertFalse(self.report(gap=.00001)['seat_contact'])
        self.assertFalse(self.report(impulse=(0,0,0))['seat_contact'])
        self.assertFalse(self.report(impulse=(.001,0,0))['seat_contact'])

    def test_shallow_numerical_penetration_accepts_but_deep_rejects(self):
        self.assertTrue(self.report(gap=-.00001,separation=-.00001)['seat_contact'])
        self.assertFalse(self.report(gap=-.0003,separation=-.0003)['seat_contact'])

    def test_tilted_edge_contact_does_not_mean_head_seated(self):
        a=.02;r=np.array([[math.cos(a),0,math.sin(a)],[0,1,0],[-math.sin(a),0,math.cos(a)]])
        self.assertFalse(self.report(rotation=r)['seat_contact'])

    def test_nonfinite_report_is_ignored(self):
        for changes in [dict(separation=float('nan')),dict(impulse=(0,0,float('nan'))),dict(point=(float('nan'),0,0))]:
            self.assertFalse(self.report(**changes)['seat_contact'])

    def test_window_includes_no_contact_steps_and_never_reuses_old_contacts(self):
        w=SeatWindow();hit=self.report();miss=self.report(impulse=(0,0,0))
        for _ in range(23):w.add(hit)
        w.add(miss)
        summary=w.summary();self.assertFalse(summary['seat_contact'])
        self.assertAlmostEqual(summary['seat_contact_fraction'],23/24)
        self.assertEqual(summary['seat_physics_samples'],24)
        self.assertFalse(w.summary()['seat_contact']);self.assertEqual(w.summary()['seat_contact_fraction'],0.)

    def test_backup_angular_stop_is_behind_physical_seat(self):
        from pxr import Usd, UsdGeom, UsdPhysics
        from ratchet_task.physics import Hardware
        h=Hardware.__new__(Hardware);h.stage=Usd.Stage.CreateInMemory()
        UsdGeom.Xform.Define(h.stage,'/World/envs/env_0/bolt')
        h.screw=Screw();h.screw.capture(AXIS[2]+.0025,.37)
        theta=.37-2*math.pi*1.8
        h.install_seat_stop(theta,np.array([0.,0.,0.,1.]))
        limit=UsdPhysics.LimitAPI(h.stage.GetPrimAtPath('/World/envs/env_0/SeatStop'),'rotX')
        stop_angle=theta+math.radians(limit.GetLowAttr().Get())
        self.assertAlmostEqual(h.screw.expected_z(stop_angle),AXIS[2]+SEAT_STOP_GAP,places=9)
        self.assertLess(SEAT_STOP_GAP,0.)


if __name__=='__main__':unittest.main()
