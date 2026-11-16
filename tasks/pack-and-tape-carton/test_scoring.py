import unittest
from model import TapeLedger,MIN_STRIP,OUTER_RADIUS
from scoring import PackingScorer,NOMINAL_FLOOR_HEIGHT,FLOOR_CONTACT_TOLERANCE


def state(**changes):
    item=dict(local_min=(-.04,-.025,.008),local_max=(.04,.025,.038),speed=0.)
    second=dict(local_min=(-.03,.035,.008),local_max=(.03,.075,.038),speed=0.)
    s=dict(time=0.,payout=0.,ribbon_length=0.,flap_angles=[160.]*4,
           items=[item,second],seam_bond=[0.]*44,end_bond=[0.,0.],carton_speed=0.,
           crease_separation=0.,max_strain=0.,canary_min_height=0.,
           inner_flaps_under_outer=False,single_strip=False,strip_still_on_roll=True,
           hands_clear=False,tools_parked=False,continuous_material=False)
    s.update(changes);return s


def sealed():
    return state(payout=.32,ribbon_length=.32,flap_angles=[0.]*4,
                 seam_bond=[1.]*44,end_bond=[.04,.04],single_strip=True,
                 strip_still_on_roll=False,inner_flaps_under_outer=True,continuous_material=True,
                 cut_coordinate=.32,blade_crossing=True,hands_clear=True,tools_parked=True)


class Tests(unittest.TestCase):
    def test_preloaded_items_alone_earn_nothing(self):
        t=PackingScorer();t.update(state());self.assertEqual(t.value,0.)

    def test_floor_contact_solver_settling_still_counts_as_contained(self):
        s=state()
        for item,height in zip(s['items'],(.005988815,.005992245)):
            item['local_min']=(*item['local_min'][:2],height)
        t=PackingScorer();t.update(s);self.assertEqual(t.value,0.)

    def test_floor_penetration_beyond_contact_tolerance_rejected(self):
        s=state();item=s['items'][0]
        item['local_min']=(*item['local_min'][:2],NOMINAL_FLOOR_HEIGHT-FLOOR_CONTACT_TOLERANCE-.000001)
        t=PackingScorer();t.update(s);self.assertEqual(t.value,0.)

    def test_floor_contact_tolerance_does_not_relax_sides_or_lid(self):
        for field,index,value in [('local_min',0,-.104001),('local_min',1,-.084001),('local_max',2,.092001)]:
            s=state();values=list(s['items'][0][field]);values[index]=value;s['items'][0][field]=values
            t=PackingScorer();t.update(s);self.assertEqual(t.value,0.)

    def test_no_root_center_shortcut_for_item_protruding(self):
        t=PackingScorer();s=state();s['items'][0]['local_max']=(.15,.025,.038)
        t.update(s);self.assertEqual(t.value,0.)

    def test_close_flaps_before_sealing(self):
        t=PackingScorer();s=sealed();s['flap_angles'][0]=20.;t.update(s)
        self.assertEqual(t.value,0.)

    def test_inner_flaps_must_be_under_outer(self):
        t=PackingScorer();s=sealed();s['inner_flaps_under_outer']=False;t.update(s)
        self.assertEqual(t.value,0.)

    def test_both_end_wraps_required(self):
        t=PackingScorer();s=sealed();s['end_bond'][0]=.01;t.update(s)
        self.assertEqual(t.value,.5)

    def test_disconnected_patches_not_a_seal(self):
        t=PackingScorer();s=sealed();s['single_strip']=False;s['continuous_material']=False;t.update(s)
        self.assertEqual(t.value,0.)

    def test_bond_cells_not_tool_path(self):
        t=PackingScorer();s=sealed();s['seam_bond'][10]=.2;t.update(s)
        self.assertEqual(t.value,.25)

    def test_still_attached_to_roll_not_finished(self):
        t=PackingScorer();s=sealed();s['strip_still_on_roll']=True;t.update(s)
        self.assertEqual(t.value,.5)

    def test_full_score_needs_two_second_release_hold(self):
        t=PackingScorer();s=sealed();t.update(s);self.assertEqual(t.value,.75)
        s.pop('cut_coordinate');s.pop('blade_crossing')
        for _ in range(41):s['time']+=.05;t.update(s)
        self.assertEqual(t.value,1.)
        s['hands_clear']=False;s['time']+=.05;t.update(s);self.assertEqual(t.value,.75)

    def test_payout_shrinks_roll(self):
        t=TapeLedger();t.update(.32,.32)
        self.assertIsNone(t.invalid);self.assertLess(t.radius,OUTER_RADIUS)

    def test_no_unlimited_tape(self):
        t=TapeLedger();t.update(t.capacity+1.,t.capacity+1.)
        self.assertIsNotNone(t.invalid)

    def test_no_fake_cut(self):
        t=TapeLedger();t.update(.32,.32,.32,False)
        self.assertIsNotNone(t.invalid);self.assertIsNone(t.cut_at)

    def test_no_short_strip(self):
        t=TapeLedger();t.update(.1,.1,.1,True)
        self.assertIsNotNone(t.invalid)

    def test_no_spawning_extra_ribbon(self):
        t=TapeLedger();t.update(.32,.5)
        self.assertIsNotNone(t.invalid)

    def test_no_stretch_exploit(self):
        t=PackingScorer();s=sealed();s['max_strain']=.2;t.update(s)
        self.assertEqual(t.value,0.)

    def test_no_detached_carton_panels(self):
        t=PackingScorer();s=sealed();s['crease_separation']=.01;t.update(s)
        self.assertEqual(t.value,0.)

if __name__=='__main__':unittest.main()
