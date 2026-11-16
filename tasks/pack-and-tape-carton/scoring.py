"""Packing, four-flap closure and adhesion-based seal rubric.

No robot-ready runtime yet. Adhesion/payout/cutting measurements must come from
native deformable/ribbon mechanics, not a sequence-triggered state machine.
"""
from math import isfinite
from model import TapeLedger

# Two 3 mm bottom-board layers support the products at 6 mm in the carton
# frame. Native contact solves can settle a few micrometres into that plane;
# allow at most 0.2 mm here without relaxing side or upper containment bounds.
NOMINAL_FLOOR_HEIGHT=.006
FLOOR_CONTACT_TOLERANCE=.0002


class PackingScorer:
    def __init__(self):
        self.tape=TapeLedger();self.time=None;self.invalid=None;self.hold=0.;self.value=0.

    def update(self,s):
        if len(s['flap_angles'])!=4 or len(s['items'])!=2 or len(s['seam_bond'])!=44:
            raise ValueError('four flaps, two items and 44 seam cells required')
        numbers=[s['time'],s['payout'],s['ribbon_length'],*s['flap_angles'],
                 *s['seam_bond'],*s['end_bond'],s['carton_speed'],s['crease_separation'],
                 s['max_strain'],s['canary_min_height']]
        for item in s['items']:numbers += [*item['local_min'],*item['local_max'],item['speed']]
        if not all(isfinite(x) for x in numbers):
            self.invalid='nonfinite state';self.value=0.;return self.result()
        dt=0. if self.time is None else s['time']-self.time;self.time=s['time']
        if not 0 <= dt <= .051:self.invalid='missing or nonmonotonic control samples'
        if len(s['end_bond'])!=2:raise ValueError('both 40 mm end wraps required')
        if s['crease_separation'] > .004:self.invalid='torn or disconnected carton'
        if s['canary_min_height'] < -.03:self.invalid='object dropped below table'
        if s['max_strain'] > .05:self.invalid='tape stretched beyond model limit'
        self.tape.update(s['payout'],s['ribbon_length'],s.get('cut_coordinate'),s.get('blade_crossing',False))
        if self.tape.invalid:self.invalid=self.tape.invalid
        # Item AABBs in carton frame, both wholly inside with clearance; no
        # root-centre shortcuts or objects protruding through side walls.
        inside=all(all(lo >= bound for lo,bound in zip(item['local_min'],(-.104,-.084,NOMINAL_FLOOR_HEIGHT-FLOOR_CONTACT_TOLERANCE))) and
                   all(hi <= bound for hi,bound in zip(item['local_max'],(.104,.084,.092))) and
                   item['speed'] < .02 for item in s['items'])
        closed=all(abs(a) <= 5. for a in s['flap_angles']) and s['inner_flaps_under_outer']
        # 44 x 5 mm seam cells must each have >=80% width adhesion. End wrap
        # values are bonded lengths, not tool-path positions. The same single
        # connected material strip must supply the entire seal.
        seal=(self.tape.cut_at is not None and s['single_strip'] and s.get('continuous_material',False) and
              all(.8 <= x <= 1. for x in s['seam_bond']) and all(x >= .035 for x in s['end_bond']) and
              not s['strip_still_on_roll'])
        connected=s.get('continuous_material',False) and self.tape.paid_out>=.055
        seam_complete=(connected and all(.8<=x<=1. for x in s['seam_bond']))
        seam_started=(connected and sum(.8<=x<=1. for x in s['seam_bond'])>=11)
        ready=(inside and closed and seal and s['hands_clear'] and s['tools_parked'] and s['carton_speed'] < .01)
        self.hold=self.hold+dt if ready else 0.
        self.value=(0. if self.invalid else 1. if self.hold >= 2. else
                    .75 if inside and closed and seal else .5 if inside and closed and seam_complete else .25 if inside and closed and seam_started else 0.)
        return self.result()

    def result(self):
        return dict(score=self.value,invalid=self.invalid,payout=self.tape.paid_out,
                    roll_radius=self.tape.radius,cut_coordinate=self.tape.cut_at,release_hold=self.hold)
