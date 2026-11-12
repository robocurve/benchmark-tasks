import unittest
from ratchet_task.scoring import State,History
def seated(**kwargs):
    fields=dict(captured=True,turns=2.,gap=0.,socket_engaged=True,ratchet_cycles=6,
                seat_contact=True,seat_contact_fraction=1.,seat_force_n=.14,
                seat_gap_min_m=0.,seat_gap_max_m=0.)
    fields.update(kwargs)
    return State(**fields)
class Scoring(unittest.TestCase):
    def test_initial_assembly_has_no_progress_credit(self):self.assertEqual(History().update(State(captured=True,turns=0),.05),0)
    def test_push_without_thread_has_no_score(self):self.assertEqual(History().update(State(gap=0),.05),0)
    def test_wall_torque_cannot_tighten(self):
        h=History();s=seated(interface_torque=.5,socket_engaged=False)
        for _ in range(40):h.update(s,.05)
        self.assertFalse(h.tightened);self.assertEqual(h.score,.7)
    def test_seated_hold_completes(self):
        h=History();s=seated(interface_torque=.5)
        for _ in range(10):h.update(s,.05)
        self.assertTrue(h.tightened);self.assertEqual(h.score,1.)
        s.socket_engaged=False;s.interface_torque=0;s.released=True;s.still=True
        for _ in range(40):h.update(s,.05)
        self.assertEqual(h.score,1.)
        s.gap=.001;self.assertEqual(h.update(s,.05),.4)
    def test_faults_latch(self):
        for field,value in [('valid',False),('interface_torque',1.01),('fixture_displacement',.0021),('gap',float('nan'))]:
            h=History();s=State(captured=True);setattr(s,field,value);self.assertEqual(h.update(s,.05),0)
            self.assertEqual(h.update(State(captured=True,turns=3,gap=0),.05),0)
    def test_cycles_required(self):
        h=History();s=seated(interface_torque=.5,ratchet_cycles=5)
        for _ in range(20):h.update(s,.05)
        self.assertFalse(h.tightened)
    def test_nearly_seated_short_thread_cannot_complete(self):
        h=History();s=seated(turns=.8,interface_torque=.5,ratchet_cycles=20,released=True,still=True)
        for _ in range(50):h.update(s,.05)
        self.assertFalse(h.tightened);self.assertEqual(h.score,.2)
    def test_separated_head_including_old_demo_never_seats(self):
        for gap in [.00019609642028806107,.00001,.001]:
            h=History();s=seated(gap=gap,seat_gap_min_m=gap,seat_gap_max_m=gap)
            for _ in range(20):h.update(s,.05)
            self.assertEqual(h.score,.4);self.assertFalse(h.tightened)
    def test_geometry_alone_and_intermittent_contact_cannot_seat(self):
        for fields in [dict(seat_contact=False),dict(seat_contact_fraction=.5),dict(seat_force_n=0.),dict(seat_gap_max_m=.0001)]:
            h=History();s=seated(**fields)
            for _ in range(20):h.update(s,.05)
            self.assertEqual(h.score,.4)
    def test_contact_loss_clears_full_score_and_restarts_hold(self):
        h=History();s=seated()
        for _ in range(10):h.update(s,.05)
        self.assertEqual(h.score,1.)
        s.seat_contact=False;self.assertEqual(h.update(s,.05),.4)
        self.assertFalse(h.tightened);self.assertEqual(h.proof,0.)
        s.seat_contact=True
        for _ in range(9):h.update(s,.05)
        self.assertEqual(h.score,.7)
        self.assertEqual(h.update(s,.05),1.)
    def test_deep_penetration_latches_integrity_failure(self):
        h=History();self.assertEqual(h.update(seated(gap=-.0003,seat_gap_min_m=-.0003),.05),0.)
        self.assertEqual(h.reason,'seat_penetration');self.assertEqual(h.update(seated(),.05),0.)
    def test_contact_nonfinite_values_invalidate(self):
        for key in ['seat_force_n','seat_contact_fraction','seat_gap_min_m','seat_gap_max_m']:
            h=History();self.assertEqual(h.update(seated(**{key:float('nan')}),.05),0.)
            self.assertTrue(h.invalid)
if __name__=='__main__':unittest.main()
