import unittest
from model import BladeCoverage,CUT_PATHS
from scoring import BreakdownScorer
from angle_tracking import VERSION,PHYSICS_DT


def state(**changes):
    s=dict(time=.05,blade_tip_local=(0.,0.,-.001),blade_alignment=1.,cutter_held=False,
           bottom_angles=[0.]*4,top_angles=[160.]*4,crease_separation=0.,
           min_height=0.,max_height=.25,carton_speed=0.,cutter_parked=False,hands_clear=False)
    s.update(changes);return s


def update(t,s):
    # Synthetic complete interval evidence; actual native tracking has separate tests.
    index=round(s['time']/.05)
    s['angle_evidence']=dict(version=VERSION,valid=True,failure=None,
        initialized_from_authored_zero=True,initial_counter=0,
        physics_dt_s=PHYSICS_DT,control_index=index,counter=12*index,
        time_s=s['time'],substeps_in_control=12,angles_deg=list(s['bottom_angles']))
    return t.update(s)


def cut_all(t,s):
    for _,a,b in CUT_PATHS:
        s.update(cutter_held=False);s['time']+=.05;update(t,s)
        for step in range(111):
            s.update(cutter_held=True,blade_tip_local=tuple(x+(y-x)*step/110 for x,y in zip(a,b)))
            s['time']+=.05;update(t,s)


class Tests(unittest.TestCase):
    def test_start(self):
        t=BreakdownScorer(require_turnover=False);update(t,state());self.assertEqual(t.value,0.)

    def test_three_cut_paths_required(self):
        t=BreakdownScorer(require_turnover=False);s=state();update(t,s);cut_all(t,s)
        self.assertTrue(t.coverage.complete);self.assertEqual(t.value,.25)

    def test_forced_open_latches_failure(self):
        t=BreakdownScorer(require_turnover=False);s=state(bottom_angles=[20.,0.,0.,0.]);update(t,s)
        s['bottom_angles']=[0.]*4;cut_all(t,s)
        self.assertEqual(t.value,0.);self.assertIn('sealed',t.invalid)

    def test_knife_must_be_held(self):
        c=BladeCoverage()
        for i in range(101):c.update((-.11+i*.0022,0.,0.),1.,False,.05)
        self.assertFalse(c.cut)

    def test_teleported_knife_cannot_cut(self):
        c=BladeCoverage();c.update((-.11,0.,0.),1.,True,.05)
        c.update((.11,0.,0.),1.,True,.05);self.assertFalse(c.cut)

    def test_wrong_depth_no_cut(self):
        c=BladeCoverage()
        for i in range(101):c.update((-.11+i*.0022,0.,.01),1.,True,.05)
        self.assertFalse(c.cut)

    def test_wrong_blade_orientation_no_cut(self):
        c=BladeCoverage()
        for i in range(101):c.update((-.11+i*.0022,0.,0.),.5,True,.05)
        self.assertFalse(c.cut)

    def test_full_score_requires_eight_flaps_and_flat_released_hold(self):
        t=BreakdownScorer(require_turnover=False);s=state();update(t,s);cut_all(t,s)
        s.update(bottom_angles=[160.]*4,cutter_held=False)
        s['time']+=.05;update(t,s)
        self.assertEqual(t.value,.5)
        s.update(bottom_angles=[90.]*4,top_angles=[270.]*4,min_height=.002,max_height=.022,
                 cutter_parked=True,hands_clear=True)
        for _ in range(21):s['time']+=.05;update(t,s)
        self.assertEqual(t.value,1.)
        s['top_angles'][0]=0.;s['time']+=.05;update(t,s)
        self.assertEqual(t.value,.5)

    def test_destroying_carton_never_counts(self):
        t=BreakdownScorer(require_turnover=False);s=state();update(t,s);cut_all(t,s)
        s.update(bottom_angles=[160.]*4,crease_separation=.02,min_height=0.,max_height=.02,
                 cutter_parked=True,hands_clear=True)
        for _ in range(21):s['time']+=.05;update(t,s)
        self.assertEqual(t.value,0.)

    def test_flat_bounding_box_alone_is_not_success(self):
        t=BreakdownScorer(require_turnover=False);update(t,state(max_height=.02,cutter_parked=True,hands_clear=True))
        self.assertEqual(t.value,0.)

if __name__=='__main__':unittest.main()

class BladeEdgeDepthTests(unittest.TestCase):
    def test_cutting_edge_penetrating_film_counts_xy_coverage(self):
        from model import BladeCoverage,FILM_Z
        m=BladeCoverage()
        for i in range(111):m.update((-.11+.002*i,0.,FILM_Z+.0002),1.,True,.05)
        self.assertEqual(m.groups()['seam']['cut'],110)
    def test_edge_outside_film_does_not_cut(self):
        from model import BladeCoverage,FILM_Z
        m=BladeCoverage()
        for i in range(111):m.update((-.11+.002*i,0.,FILM_Z-.0001),1.,True,.05)
        self.assertEqual(m.groups()['seam']['cut'],0)

class FlatteningGeometryTests(unittest.TestCase):
    def test_opening_and_final_coplanarity_are_different_requirements(self):
        from math import cos,radians
        # A horizontal wall places the flap's local Y direction vertically
        # at angle zero. At 150 degrees an87mm flap needs75mm of height;
        # at90degrees it lies in the same plane as its wall.
        self.assertGreater(.087*abs(cos(radians(150))),.024)
        self.assertLess(.087*abs(cos(radians(90))),1e-12)
        t=BreakdownScorer(require_turnover=False);s=state();update(t,s);cut_all(t,s)
        s.update(bottom_angles=[160.]*4,top_angles=[270.]*4,min_height=0.,max_height=.02,cutter_parked=True,hands_clear=True)
        for _ in range(25):s['time']+=.05;update(t,s)
        self.assertEqual(t.value,.5)
        s['bottom_angles']=[90.]*4
        for _ in range(25):s['time']+=.05;update(t,s)
        self.assertEqual(t.value,1.)


class EvidenceIntegrityTests(unittest.TestCase):
    def test_old_wrapped_only_sample_is_rejected_stickily(self):
        t=BreakdownScorer(require_turnover=False);s=state(bottom_angles=[274.9]*4)
        t.update(s)
        self.assertEqual(t.value,0.)
        self.assertIn('continuous angle evidence',t.invalid)
        s['time']+=.05;update(t,s)
        self.assertEqual(t.value,0.)

    def test_flaps_must_be_open_together(self):
        t=BreakdownScorer(require_turnover=False);s=state();update(t,s);cut_all(t,s)
        for i in range(4):
            s['bottom_angles']=[160. if i==j else 0. for j in range(4)]
            s['time']+=.05;update(t,s)
        self.assertFalse(t.opened)
        self.assertEqual(t.value,.25)
