"""Measured-state tape sweep and ordered unboxing milestones, without simulator imports."""
import math
from config import CUT_PATHS, CUT_SEGMENTS, LID_Z, DESTINATION, CUTTER_START, TOP

class UnboxScorer:
    def __init__(self):
        self.cut = [False]*CUT_SEGMENTS
        self.previous_tip = None
        self.previous_valid = {}
        self.value = 0.
        self.opened = False
        self.rest_since = None
        self.invalid = False
        self.reason = None

    def update(self,s):
        t=s['time']; tip=s['tip']; c=s['cutter']; item=s['item']
        values=[t,*tip,*c['pos'],c['speed'],*item['pos'],item['speed'],
                s['blade_down'],*s['blade_forward'],*s['flap_angles']]
        if not all(math.isfinite(v) for v in values):
            self.invalid=True; self.reason='nonfinite telemetry'
        if item['pos'][2]<TOP-.02 or c['pos'][2]<TOP-.03:
            self.invalid=True; self.reason='item or cutter dropped off table'
        # A guarded blade must be held, aligned down, at tape height. Sweeps faster
        # than 0.25 m/s are excluded to prevent teleport-like segments earning cuts.
        valid = {}
        forward=s['blade_forward']
        block_offset=cell_offset=0
        for path in CUT_PATHS:
            axis=path['axis'];other=1-axis;name=path['name']
            valid[name]=(s['held'] and s['blade_down']>.9 and abs(forward[axis])>.85
                         and abs(tip[other]-path['fixed'])<=.006
                         and abs(tip[2]-LID_Z)<=.004)
            if valid[name] and self.previous_valid.get(name,False) and self.previous_tip is not None:
                a=self.previous_tip;dt=t-self.previous_time;dist=math.dist(a,tip)
                along=abs(tip[axis]-a[axis])
                if 0<dt<=.1 and .002*dt<=dist<=.25*dt and along>=.85*dist:
                    lo,hi=sorted((a[axis],tip[axis]))
                    cells=path['segments']*path['cells_per_segment']
                    for j in range(cells):
                        x=path['start']+(j+.5)*path['length']/cells
                        if lo<=x<=hi:self.covered.add(cell_offset+j)
            for i in range(path['segments']):
                start=cell_offset+i*path['cells_per_segment']
                self.cut[block_offset+i]=all(j in self.covered for j in range(start,start+path['cells_per_segment']))
            block_offset+=path['segments'];cell_offset+=path['segments']*path['cells_per_segment']
        self.previous_tip=tip;self.previous_valid=valid;self.previous_time=t
        sealed=not all(self.cut)
        angles=s['flap_angles']
        if sealed and max(abs(a) for a in angles)>12:
            self.invalid=True; self.reason='flap forced open before tape was cut'
        if not sealed and min(angles)>=80: self.opened=True
        returned=(math.dist(c['pos'][:2],CUTTER_START[:2])<=.045
                  and abs(c['pos'][2]-CUTTER_START[2])<.025 and c['speed']<.02
                  and not s['held'])
        placed=(abs(item['pos'][0]-DESTINATION[0])<=.025
                and abs(item['pos'][1]-DESTINATION[1])<=.025
                and abs(item['pos'][2]-(TOP+.022))<=.008
                and item['speed']<.01 and s['item_released'])
        ready=self.opened and min(angles)>=70 and returned and placed
        self.rest_since=t if ready and self.rest_since is None else self.rest_since if ready else None
        self.value=0. if sealed else .25
        if self.opened: self.value=.5
        if self.rest_since is not None and t-self.rest_since>=1.-1e-8: self.value=1.
        if self.invalid: self.value=0.
        return self.result()

    @property
    def covered(self):
        if not hasattr(self,'_covered'): self._covered=set()
        return self._covered

    def groups(self):
        result={};offset=0
        for path in CUT_PATHS:
            count=path['segments'];result[path['name']]=dict(cut=sum(self.cut[offset:offset+count]),total=count)
            offset+=count
        return result

    def result(self):
        return dict(score=self.value,cut_segments=sum(self.cut),total_segments=CUT_SEGMENTS,cut_groups=self.groups(),
                    opened=self.opened,invalid=self.invalid,reason=self.reason)
