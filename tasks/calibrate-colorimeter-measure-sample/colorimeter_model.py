"""Pose/joint-driven Beer–Lambert instrument. Pure Python, no renderer or scorer.

Tokens are (episode, control step). Repeated observations in a step are inert.
The reference is an intensity, with no knowledge of the correct protocol.
"""
import math

WAVELENGTHS = (430,470,565,635)
ABSORBANCE = (.12,.22,.20,.78)
I0 = (1.,.94,1.06,.98)


class Instrument:
    def __init__(self,well_xy,floor,width=.0145,body_width=.0125,closed_lid=0.):
        self.well_xy,self.floor,self.clearance=tuple(well_xy),floor,(width-body_width)/2+.002
        self.closed_lid=closed_lid
        self.reset()

    def reset(self,start=0):
        self.w,self.ref,self.last_failure=start,None,None
        self.down=dict.fromkeys(('less','more','cal_button'),False)
        self.token,self.reading,self.seated=None,None,()
        self.events=[]

    def is_seated(self,c):
        return (c['tilt']<=3.+1e-10 and self.floor-.002-1e-12<=c['lowest']<=self.floor+.002+1e-12 and
                all(abs(c['bottom'][i]-self.well_xy[i])<self.clearance-1e-12 for i in (0,1)))

    def intensity(self,c):
        # Opposite clear faces are a directionless axis, measured against world x.
        axis=c['clear_axis']; length=math.sqrt(sum(v*v for v in axis))
        alignment=abs(axis[0])/length if length else 0.
        clear=alignment>=math.cos(math.radians(10))-1e-12
        # Other directions attenuate as frosted faces. Intermediate yaws cannot
        # fit the physical square throat; defining them also makes fixtures total.
        factor=1. if clear else 10**(-.6)
        return I0[self.w]*10**(-(0. if c['blank'] else ABSORBANCE[self.w]))*factor

    def update(self,state,layout,token,t):
        if token==self.token: return self
        if self.token is None or token[0]!=self.token[0]: self.reset(layout[1])
        self.events=[]
        self.seated=tuple(n for n,c in state['cuvettes'].items() if self.is_seated(c))
        closed=abs(state['lid']-self.closed_lid)<=.002+1e-12
        for name in ('less','more','cal_button'):
            travel=state['buttons'][name]
            pressed=self.down[name]
            if travel>=.002-1e-12: pressed=True
            elif travel<=.001+1e-12: pressed=False
            edge=pressed and not self.down[name]
            self.down[name]=pressed
            if not edge: continue
            if name!='cal_button':
                new=max(0,min(3,self.w+(-1 if name=='less' else 1)))
                if new!=self.w:
                    self.w,self.ref=new,None
                    self.events.append(('wavelength',self.w))
            elif closed and len(self.seated)==1:
                intensity=self.intensity(state['cuvettes'][self.seated[0]])
                self.ref=(self.w,intensity)
                self.last_failure=None
                self.events.append(('CAL','stored',self.w,math.log10(I0[self.w]/intensity)))
            else:
                self.last_failure=t
                self.events.append(('CAL','failed',self.w,None))
        self.reading=None
        if closed and len(self.seated)==1 and self.ref is not None and self.ref[0]==self.w:
            self.reading=math.log10(self.ref[1]/self.intensity(state['cuvettes'][self.seated[0]]))
        self.token=token
        return self

    def cal_lamp(self,t):
        if self.last_failure is not None and 0<=t-self.last_failure<2.: return 'red'
        return 'green' if self.ref is not None and self.ref[0]==self.w else 'off'

    def display(self):
        if self.reading is None: return '----'
        # Sign plus three digits. The physical model's range is -1.38..+1.38 AU.
        return ('-' if self.reading<-.005 else '+')+f'{abs(self.reading):.2f}'
