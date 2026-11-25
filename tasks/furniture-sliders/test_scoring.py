"""Counterexamples for public credit, continuous transport and released retention."""
import copy
import math
import unittest
from config import FLOOR_Z,FOOT_CENTERS,PAD_H,P
from scoring import Scorer,snapshot,points,rotation,TRANSPORT


def state(step=0,x=.32,held=False):
    return {'step':step,'furniture':[x,0,FLOOR_Z+PAD_H,0,0,0,1],
            'pads':{f'slider_{i}':[x+dx,dy,FLOOR_Z,0,0,0,1] for i,(dx,dy,_) in enumerate(FOOT_CENTERS)},
            'hand_clearance_m':0 if held else .1,
            'pad_loads_N':{f'slider_{i}':4.4 for i in range(4)},'pad_held':{}}


class ScoreTests(unittest.TestCase):
    def hold(self,scorer,m,frames=41):
        for _ in range(frames):
            m=copy.deepcopy(m)
            m['step']=0 if scorer.previous is None else scorer.previous['step']+1
            scorer.update(m)
        return m

    def test_static_goal_does_not_replace_transport(self):
        s=Scorer(); self.hold(s,state(x=.52))
        self.assertEqual(s.score,.4); self.assertFalse(s.success)

    def test_continuous_slide_then_release(self):
        s=Scorer(); self.hold(s,state())
        for i in range(1,101): s.update(state(s.previous['step']+1,.32+i*.002,True))
        self.hold(s,state(x=.52))
        self.assertEqual(s.score,1.)
        self.assertTrue(s.gate_pass)

    def test_carried_furniture_has_no_supported_witness(self):
        s=Scorer(); self.hold(s,state())
        for i in range(1,101):
            m=state(s.previous['step']+1,.32+i*.002,True)
            m['furniture'][2]+=.05
            for p in m['pads'].values(): p[2]+=.05
            s.update(m)
        self.hold(s,state(x=.52))
        self.assertEqual(s.score,.4)

    def test_missing_flipped_displaced_and_duplicate_supports(self):
        for kind in ('missing','flipped','displaced','unloaded','duplicate'):
            with self.subTest(kind=kind):
                m=state()
                if kind=='missing': del m['pads']['slider_0']
                if kind=='flipped': m['pads']['slider_0'][3:]=[1,0,0,0]
                if kind=='displaced': m['pads']['slider_0'][0]+=.012
                if kind=='unloaded': m['pads']['slider_0'][2]-=.004
                if kind=='duplicate': m['pads']['slider_0']=m['pads']['slider_1'].copy()
                s=Scorer(); self.hold(s,m)
                self.assertEqual(s.score,.3)

    def test_held_off_panel_or_wrong_goal_no_full_credit(self):
        s=Scorer(); self.hold(s,state(held=True))
        self.assertEqual(s.score,0.)
        self.hold(s,state(x=.9)); self.assertEqual(s.score,0.)
        self.hold(s,state()); self.assertEqual(s.score,.4)

    def test_loss_resets_witness_but_recovery_can_restart(self):
        s=Scorer(); self.hold(s,state())
        s.update(state(s.previous['step']+1,.5,True))
        m=state(s.previous['step']+1,.5,True); del m['pads']['slider_0']; s.update(m)
        self.hold(s,state(x=.52)); self.assertEqual(s.score,.4)
        for x in (.32,.52): s.update(state(s.previous['step']+1,x,True))
        self.hold(s,state(x=.52)); self.assertEqual(s.score,1.)

    def test_current_partial_credit_not_historical(self):
        s=Scorer(); self.hold(s,state()); self.assertEqual(s.score,.4)
        m=state(); del m['pads']['slider_0']; self.hold(s,m)
        self.assertEqual(s.score,.3)

    def test_submillimetre_hover_is_not_contact(self):
        m=state(); m['furniture'][2]+=.0008
        s=Scorer(); self.hold(s,m)
        self.assertEqual(s.score,0.)

    def test_transport_does_not_require_submillimetre_gaps(self):
        # The video rule tolerates small height excursions. Precise contact
        # must still be established at the final released stationary state.
        for lifted in ('furniture', 'furniture_and_pads'):
            with self.subTest(lifted=lifted):
                s=Scorer(); self.hold(s,state())
                for i in range(1,101):
                    m=state(s.previous['step']+1,.32+i*.002,True)
                    m['furniture'][2]+=.0008
                    if lifted=='furniture_and_pads':
                        for pad in m['pads'].values(): pad[2]+=.0008
                    d=s.update(m)
                    self.assertEqual(d['matches'],[])
                    self.assertTrue(d['transport_retained'])
                self.hold(s,state(x=.52))
                self.assertEqual(s.score,1.)

    def test_stationary_hover_still_prevents_full_score_after_transport(self):
        s=Scorer(); self.hold(s,state())
        for i in range(1,101): s.update(state(s.previous['step']+1,.32+i*.002,True))
        m=state(x=.52); m['furniture'][2]+=.0008
        self.hold(s,m)
        self.assertGreater(s.transport_max,.15)
        self.assertEqual(s.score,0.)

    def test_transport_alignment_is_coarse_but_not_unbounded(self):
        for offset,retained in ((.012,True),(.020,True),(.0201,False)):
            with self.subTest(offset=offset):
                m=state();m['pads']['slider_0'][0]+=offset
                d=snapshot(m)
                self.assertEqual(len(d['matches']),3)
                self.assertEqual(d['transport_retained'],retained)

    def test_visible_lift_resets_transport_even_if_pad_follows_foot(self):
        for lifted in ('furniture','pad','both'):
            with self.subTest(lifted=lifted):
                s=Scorer();self.hold(s,state())
                s.update(state(s.previous['step']+1,.50,True))
                m=state(s.previous['step']+1,.50,True)
                if lifted in ('furniture','both'):m['furniture'][2]+=.0101
                if lifted in ('pad','both'):m['pads']['slider_0'][2]+=.0101
                self.assertFalse(s.update(m)['transport_retained'])
                self.assertEqual(s.transport_max,0.)
                self.hold(s,state(x=.52));self.assertEqual(s.score,.4)

    def test_transport_height_band_is_absolute_not_per_frame(self):
        s=Scorer();self.hold(s,state())
        for i in range(1,21):
            m=state(s.previous['step']+1,.32+i*.01,True)
            m['furniture'][2]+=i*.001
            for pad in m['pads'].values():pad[2]+=i*.001
            s.update(m)
        self.hold(s,state(x=.52));self.assertEqual(s.score,.4)

    def test_transport_height_and_tilt_boundaries(self):
        m=state();m['furniture'][2]+=.010
        for pad in m['pads'].values():pad[2]+=.010
        self.assertTrue(snapshot(m)['transport_retained'])
        m=state();a=math.radians(15.)
        m['pads']['slider_0'][3:]=[math.sin(a/2),0,0,math.cos(a/2)]
        self.assertTrue(snapshot(m)['transport_retained'])
        a=math.radians(15.1)
        m['pads']['slider_0'][3:]=[math.sin(a/2),0,0,math.cos(a/2)]
        self.assertFalse(snapshot(m)['transport_retained'])

    def test_expelled_or_flipped_pad_resets_completed_transport(self):
        for kind in ('expelled','flipped'):
            with self.subTest(kind=kind):
                s=Scorer();self.hold(s,state())
                s.update(state(s.previous['step']+1,.50,True))
                m=state(s.previous['step']+1,.50,True)
                if kind=='expelled':m['pads']['slider_0'][0]+=.09
                else:m['pads']['slider_0'][3:]=[1,0,0,0]
                s.update(m)
                self.hold(s,state(x=.52));self.assertEqual(s.score,.4)

    def test_switching_pad_identity_restarts_transport(self):
        s=Scorer();self.hold(s,state())
        s.update(state(s.previous['step']+1,.50,True))
        m=state(x=.50)
        m['pads']['slider_0'],m['pads']['slider_1']=m['pads']['slider_1'],m['pads']['slider_0']
        self.hold(s,m)
        self.assertEqual(s.transport_max,0.)
        self.assertEqual(s.score,.4)

    def test_transport_rejects_off_panel_duplicate_and_nonfinite_states(self):
        for kind in ('off_panel','duplicate','nan'):
            with self.subTest(kind=kind):
                m=state(x=.9 if kind=='off_panel' else .32)
                if kind=='duplicate':m['pads']['slider_0']=m['pads']['slider_1'].copy()
                if kind=='nan':m['furniture'][2]=float('nan')
                self.assertFalse(snapshot(m)['transport_retained'])

    def test_transport_does_not_read_force_or_attachment_diagnostics(self):
        m=state();m.pop('pad_loads_N');m.pop('pad_held')
        self.assertTrue(snapshot(m)['transport_retained'])

    def test_legacy_recording_defaults_match_published_transport_rule(self):
        self.assertEqual(TRANSPORT,{'foot_center_offset_m':.020,
                                    'height_tolerance_m':.010,
                                    'upright_tolerance_deg':15.})

    def test_tilted_pad_uses_bottom_rim_for_floor_contact(self):
        import numpy as np
        a=math.radians(1.)
        q=[math.sin(a/2),0,0,math.cos(a/2)]
        pad=np.array((.17,.15,FLOOR_Z))
        foot=np.asarray(FOOT_CENTERS[0])
        origin=pad+rotation(q)@(np.array((0,0,PAD_H))-foot)
        m=state();m['furniture']=[*origin,*q];m['pads']={'slider_0':[*pad,*q]}
        self.assertEqual(snapshot(m)['matches'],[])
        # Moving the origin up by the true rim depth makes an edge contact.
        lift=P['diameter_m']/2*math.sin(a)
        m['furniture'][2]+=lift;m['pads']['slider_0'][2]+=lift
        self.assertEqual(snapshot(m)['matches'],[(0,'slider_0')])

    def test_forearm_support_is_not_released(self):
        m=state();m['robot_contact']=True
        s=Scorer();self.hold(s,m)
        self.assertEqual(s.score,0.)

    def test_side_propping_is_not_released_standing_credit(self):
        m=state();a=math.radians(92.)
        m['furniture']=[.32,0,FLOOR_Z+PAD_H+.175*math.sin(a),0,math.sin(a/2),0,math.cos(a/2)]
        m['pads']={f'slider_{i}':[*points(m['furniture'],[FOOT_CENTERS[i]])[0][:2],FLOOR_Z,0,0,0,1]
                   for i in (2,3)}
        self.assertEqual(len(snapshot(m)['matches']),2)
        s=Scorer();self.hold(s,m)
        self.assertEqual(s.score,0.)

    def test_one_pad_requires_unheld_continuous_retention(self):
        s=Scorer(); self.hold(s,state())
        m=state(s.previous['step']+1,.45,True); m['pad_held']['slider_0']=True; s.update(m)
        self.hold(s,state(x=.45)); self.assertFalse(s.gate_pass)

    def test_replay_rejects_missing_samples(self):
        s=Scorer(); s.update(state())
        with self.assertRaises(ValueError): s.update(state(2))


if __name__=='__main__': unittest.main()
