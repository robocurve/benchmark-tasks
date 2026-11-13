"""Wrapped tape must be severed at all three load-bearing cuts."""
import unittest
from config import *
from scoring import UnboxScorer


def sample(t=0.,x=None):
    return dict(time=t,tip=[SEAM[0] if x is None else x,0.,LID_Z],
                cutter=dict(pos=list(CUTTER_START),speed=0.),
                item=dict(pos=[*CENTER,TOP+.023],speed=0.),held=True,
                blade_down=1.,blade_forward=[1.,0.,0.],flap_angles=[0.,0.],item_released=False)


def cut_path(tracker,path,t=0.,reverse=False,alter=None):
    cells=path['segments']*path['cells_per_segment']
    for i in range(cells+1):
        u=(cells-i if reverse else i)/cells
        s=sample(t+i*.05)
        s['tip'][path['axis']]=path['start']+u*path['length']
        s['tip'][1-path['axis']]=path['fixed']
        s['blade_forward']=[0.,0.,0.];s['blade_forward'][path['axis']]=1.
        if alter:alter(s)
        tracker.update(s)
    return t+(cells+1)*.05+.1


def cut_all(tracker):
    t=0.
    for p in CUT_PATHS:t=cut_path(tracker,p,t)
    return t


class TapeTests(unittest.TestCase):
    def test_initial_zero(self):
        t=UnboxScorer();t.update(sample());self.assertEqual(t.value,0)

    def test_all_three_cuts_free_tape(self):
        t=UnboxScorer();cut_all(t)
        self.assertEqual(sum(t.cut),CUT_SEGMENTS);self.assertEqual(t.value,.25)
        self.assertTrue(all(g['cut']==g['total'] for g in t.groups().values()))

    def test_top_seam_alone_does_not_free_wrapped_ends(self):
        t=UnboxScorer();cut_path(t,CUT_PATHS[0])
        self.assertEqual(sum(t.cut),22);self.assertEqual(t.value,0.)
        self.assertEqual(t.groups()['front_fold']['cut'],0)

    def test_one_uncut_end_still_holds_box_closed(self):
        t=UnboxScorer();tm=cut_path(t,CUT_PATHS[0]);cut_path(t,CUT_PATHS[1],tm)
        self.assertEqual(t.value,0.);self.assertEqual(t.groups()['back_fold']['cut'],0)

    def test_stationary_tip_does_not_cut(self):
        t=UnboxScorer()
        for i in range(100):t.update(sample(i*.05))
        self.assertFalse(any(t.cut))

    def test_unheld_wrong_height_or_blade_orientation_does_not_cut(self):
        for mode in ('held','height','down','yaw'):
            t=UnboxScorer()
            def alter(s):
                if mode=='held':s['held']=False
                if mode=='height':s['tip'][2]+=.02
                if mode=='down':s['blade_down']=0.
                if mode=='yaw':s['blade_forward']=[0.,1.,0.]
            cut_path(t,CUT_PATHS[0],alter=alter)
            self.assertFalse(any(t.cut),mode)

    def test_transverse_stroke_needs_transverse_blade(self):
        t=UnboxScorer()
        cut_path(t,CUT_PATHS[1],alter=lambda s:s.update(blade_forward=[1.,0.,0.]))
        self.assertFalse(any(t.cut))

    def test_jump_cannot_cut(self):
        t=UnboxScorer();t.update(sample());t.update(sample(.05,SEAM[1]));self.assertFalse(any(t.cut))

    def test_air_jump_cannot_bridge(self):
        t=UnboxScorer();t.update(sample());s=sample(.05,SEAM[1]);s['tip'][2]+=.1;t.update(s)
        t.update(sample(.1,SEAM[1]));self.assertFalse(any(t.cut))

    def test_reverse_cuts(self):
        t=UnboxScorer();tm=0.
        for p in CUT_PATHS:tm=cut_path(t,p,tm,True)
        self.assertEqual(t.value,.25)

    def test_forced_open_with_uncut_fold_invalidates(self):
        t=UnboxScorer();tm=cut_path(t,CUT_PATHS[0]);s=sample(tm);s['flap_angles']=[45,0];t.update(s)
        cut_all(t);self.assertTrue(t.invalid);self.assertEqual(t.value,0)

    def test_full_score_requires_placement_return_release_and_hold(self):
        t=UnboxScorer();tm=cut_all(t);s=sample(tm);s['flap_angles']=[100,100];s['held']=False
        t.update(s);self.assertEqual(t.value,.5)
        s['item']['pos']=[*DESTINATION,TOP+.022];s['item_released']=True;t.update(s)
        self.assertEqual(t.value,.5)
        s['time']=tm+1.01;t.update(s);self.assertEqual(t.value,1)
        s['item']['pos'][2]=TOP-.1;s['time']+=.1;t.update(s);self.assertEqual(t.value,0)

    def test_closed_flaps_cannot_earn_full_score(self):
        t=UnboxScorer();tm=cut_all(t);s=sample(tm);s['held']=False;s['flap_angles']=[100,100];t.update(s)
        s['flap_angles']=[0,0];s['item']['pos']=[*DESTINATION,TOP+.022];s['item_released']=True
        for i in range(40):s['time']=tm+1+i*.05;t.update(s)
        self.assertEqual(t.value,.5)

if __name__=='__main__':unittest.main()
