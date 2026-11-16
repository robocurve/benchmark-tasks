"""End-state rubric for a genuinely taped, four-bottom-flap recycling task.

The future native adapter must derive all inputs from live body/contact state.
It must not expose these fields as policy controls or mutate them to progress.
"""
from math import isfinite,sin,cos,radians
from model import BladeCoverage
from angle_tracking import evidence_error


class BreakdownScorer:
    def __init__(self):
        self.coverage=BladeCoverage();self.time=None;self.invalid=None
        self.opened=False;self.hold=0.;self.value=0.;self.control_index=0

    def update(self,s):
        self.control_index+=1
        error=evidence_error(s.get('angle_evidence'),s['bottom_angles'],
                             s['time'],self.control_index)
        if error:
            if self.invalid is None:self.invalid=error
            self.value=0.;return self.result()
        numbers=[s['time'],*s['blade_tip_local'],s['blade_alignment'],
                 *s['bottom_angles'],*s['top_angles'],s['crease_separation'],
                 s['min_height'],s['max_height'],s['carton_speed'],s.get('cutter_height',0.)]
        if not all(isfinite(x) for x in numbers):
            self.invalid='nonfinite state';self.value=0.;return self.result()
        if len(s['bottom_angles'])!=4 or len(s['top_angles'])!=4:
            raise ValueError('four real flaps required at each end')
        dt=0. if self.time is None else s['time']-self.time;self.time=s['time']
        if not 0 <= dt <= .051:self.invalid='missing or nonmonotonic control samples'
        if s['crease_separation'] > .004:self.invalid='torn or disconnected cardboard'
        if s['min_height'] < -.03:self.invalid='carton fell below table'
        if s.get('cutter_height',0.) < -.03:self.invalid='cutter fell below table'
        # Only the two outer flaps are restrained by the sealed seam.
        # Fail before processing the current cut, so forced opening and a cut
        # in the same sample cannot erase the violation.
        if not self.coverage.complete and max(abs(a) for a in s['bottom_angles'][:2]) > 12.:
            self.invalid='forced a sealed bottom flap'
        self.coverage.update(s['blade_tip_local'],s['blade_alignment'],s['cutter_held'],dt)
        bottom_open=all(a >= 150. for a in s['bottom_angles'])
        if self.coverage.complete and bottom_open:self.opened=True
        # Opening >=150 degrees is a historical manipulation requirement.
        # Final flattening requires each flap coplanar with its wall (90/270
        # degrees), not still perpendicular to that wall at 180 degrees.
        # Entire carton must rest flat on the table, including all eight flaps.
        # Heights refer to all actual cardboard vertices, not one root body.
        flat=(s['max_height']-s['min_height'] <= .024 and
              -.002 <= s['min_height'] <= .008 and s['max_height'] <= .030)
        coplanar=all(abs(sin(radians(a)))>=cos(radians(15.)) for a in [*s['bottom_angles'],*s['top_angles']])
        ready=(self.opened and coplanar and
               flat and s['carton_speed'] < .01 and s['cutter_parked'] and s['hands_clear'])
        self.hold=self.hold+dt if ready else 0.
        self.value=(0. if self.invalid else 1. if self.hold >= 1. else
                    .5 if self.opened else .25 if self.coverage.complete else 0.)
        return self.result()

    def result(self):
        return dict(score=self.value,invalid=self.invalid,cuts=self.coverage.groups(),
                    bottom_opened=self.opened,flat_release_hold=self.hold)
