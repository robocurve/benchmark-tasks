import unittest
from ratchet_task.scoring import State,History
class Scoring(unittest.TestCase):
    def test_initial_assembly_has_no_progress_credit(self):self.assertEqual(History().update(State(captured=True,turns=0),.05),0)
    def test_push_without_thread_has_no_score(self):self.assertEqual(History().update(State(gap=0),.05),0)
    def test_wall_torque_cannot_tighten(self):
        h=History();s=State(captured=True,turns=11,gap=0,interface_torque=.5,ratchet_cycles=6)
        for _ in range(40):h.update(s,.05)
        self.assertFalse(h.tightened);self.assertEqual(h.score,.7)
    def test_seated_hold_completes(self):
        h=History();s=State(captured=True,turns=11,gap=0,interface_torque=.5,socket_engaged=True,ratchet_cycles=6)
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
        h=History();s=State(captured=True,turns=11,gap=0,interface_torque=.5,socket_engaged=True,ratchet_cycles=5)
        for _ in range(20):h.update(s,.05)
        self.assertFalse(h.tightened)
    def test_nearly_seated_short_thread_cannot_complete(self):
        h=History();s=State(captured=True,turns=.8,gap=0,interface_torque=.5,socket_engaged=True,ratchet_cycles=20,released=True,still=True)
        for _ in range(50):h.update(s,.05)
        self.assertFalse(h.tightened);self.assertEqual(h.score,.2)
if __name__=='__main__':unittest.main()
