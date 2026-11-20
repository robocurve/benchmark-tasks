"""Substep measurements in the fixture frame; SI inputs, an irreversible trainer overload and an unloaded final proof."""
import math
K=350.
TRIP_N=5.
def rotate(q,v):
    x,y,z,w=q
    ax,ay,az=v
    tx,ty,tz=2*(y*az-z*ay),2*(z*ax-x*az),2*(x*ay-y*ax)
    return (ax+w*tx+y*tz-z*ty,ay+w*ty+z*tx-x*tz,az+w*tz+x*ty-y*tx)
class Trial:
    def __init__(self):
        self.peak=0.;self.tripped=False;self.work=False;self.unloaded=False;self.dwell=0.;self.prev=None;self.last=None;self.moved=False
        self.farthest_mm=0.
        self.guided_mm=0.
    def observe(self,poses,dt,touch_release,touch_arm,other_contact=False):
        if not (0<dt<=.01):raise ValueError('Unexpected physics dt')
        for n in ('frame','yoke','sleeve','arm'):
            pos,q=poses[n]
            if len(pos)!=3 or len(q)!=4 or not all(math.isfinite(v) for v in (*pos,*q)) or abs(sum(v*v for v in q)-1)>.01:raise ValueError('Invalid solver pose '+n)
        base,bq=poses['frame'];upper=poses['yoke'][0];sleeve,sq=poses['sleeve'];arm=poses['arm'][0]
        # The calibrated mount is axis-aligned. The two slider guides constrain roll and yaw.
        gap=1000*(upper[2]-base[2]-.043-.065)
        stroke=1000*(-.041-(sleeve[0]-base[0]))
        elong=max(0.,sleeve[0]-arm[0]-.090)
        tension=K*elong
        ends=[sleeve[0]+rotate(sq,(.068,y,0))[0]-base[0] for y in (-.0175,.0175)]
        clear=sum(x<-.034 for x in ends)
        if self.prev is None:
            if abs(stroke)>4 or not (-1<gap<10) or clear or tension>1.25:raise ValueError('Reset is not clamped, intact, and loaded at the datum')
            self.base0=tuple(base)
        still=self.prev is not None and max(math.dist(cur,pr) for cur,pr in zip((sleeve,arm),self.prev))/dt<.018
        self.prev=(tuple(sleeve),tuple(arm))
        self.peak=max(self.peak,tension)
        self.tripped |= tension>=TRIP_N
        self.moved |= math.dist(base,self.base0)>.002 or abs(bq[3])<.9995
        # Every guide is prismatic. A wall crossing or dislodged coupon is a rig failure.
        self.moved |= any(abs(pos[1]-base[1])>.004 for pos in (upper,sleeve,arm))
        self.moved |= any(abs(pos[2]-base[2]-.0785)>.004 for pos in (sleeve,arm))
        self.moved |= not (-11<gap<42 and -9<stroke<140) or abs(sq[3])<.998
        if gap>=9 and touch_release:self.unloaded=True
        # Only count newly traversed sleeve material in the first 61 mm (captured in the nip).
        # Repeated travel over the same section, or touching an already freed arm, earns no work.
        captured=min(61.,max(0.,stroke))
        advance=max(0.,captured-self.farthest_mm)
        self.farthest_mm=max(self.farthest_mm,captured)
        if self.unloaded and gap>=9 and touch_arm:
            self.guided_mm+=advance
        self.work=self.guided_mm>=10.
        independent=not(touch_arm or touch_release or other_contact)
        arm_clear=arm[0]-base[0]<-.195
        ok=(self.guided_mm>=45 and not self.tripped and not self.moved and stroke>=75 and clear==2 and arm_clear and tension<1.0 and independent and still)
        self.dwell=self.dwell+dt if ok else 0.
        self.last=dict(withdraw_mm=stroke,gap_mm=gap,tension_N=tension,free_ends=clear,arm_clear=arm_clear,independent=independent,dwell_s=self.dwell,trip=self.tripped)
        return self.last
    @property
    def score(self):
        if self.tripped or self.moved:return 0.
        return 1. if self.dwell>=.5-1e-9 else (.25 if self.work else 0.)
