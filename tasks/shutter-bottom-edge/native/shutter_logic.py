"""Trainer electronics and judging. Units are mm except forces (N) and time (s)."""
from dataclasses import dataclass
from math import isfinite

NAMES=("frame","door","cassette","left","right","sled","guide","block","button")
SIDES=("left","right")
DT=1/240

@dataclass(frozen=True)
class Reading:
    height: float
    carriage: float
    cassette: float
    crush: float
    button: float
    left: float
    right: float
    contact_left: float
    contact_right: float
    button_hand: bool
    assist: bool
    guide_lift_mm: float=0.

    def check(self):
        if not all(isfinite(x) for x in vars(self).values()):
            raise ValueError("Nonfinite substep measurement")
        if min(self.contact_left,self.contact_right)<0:
            raise ValueError("Invalid contact magnitude")

    def site(self):
        for side,position in zip(SIDES,(66,-66)):
            if abs(self.carriage-position)<=6:
                return side
        return None

def reading(poses,forces,pairs,hands):
    p={k:v[:3] for k,v in poses.items()}
    def d(a,b,axis):
        return 1000*(p[a][axis]-p[b][axis])
    return Reading(d("door","frame",2),d("sled","frame",1),d("cassette","door",0),
                   18-d("block","guide",2),42-d("button","frame",2),
                   14+d("left","door",2),14+d("right","cassette",2),
                   *forces,bool(set(pairs["button"])&hands),
                   any(pairs[n] for n in ("door","left","right","block","sled","guide","cassette")),d("guide","sled",2)-16)

def intact(p,origin,base_z):
    """Rigid guides allow one translation each. Reject off-axis motion or rotation."""
    if set(p)!=set(NAMES) or any(len(v)!=7 or not all(isfinite(x) for x in v) for v in p.values()):
        raise ValueError("Missing or nonfinite rigid transform")
    if any(abs(abs(v[6])-1)>.0004 for v in p.values()):
        return False
    for axis in (0,1):
        if abs(p["frame"][axis]-origin[axis])>.002 or abs(p["door"][axis]-origin[axis])>.002:
            return False
    if abs(p["frame"][2]-base_z)>.002:
        return False
    if not .014 <= p["guide"][2]-p["sled"][2] <= .034:
        return False
    constraints=[
        ("cassette","door",{1:-.066,2:0}),("left","door",{0:0,1:.066}),
        ("right","cassette",{0:0,1:0}),("block","guide",{0:0,1:0}),("guide","sled",{0:0,1:0}),
        ("sled","frame",{0:0,2:.020}),("button","frame",{0:-.155,1:.210})]
    return all(abs(p[a][axis]-p[b][axis]-offset)<=.0025 for a,b,coords in constraints for axis,offset in coords.items())

class Trainer:
    """The edge switches can reverse the motor at either site, even in an invalid trial."""
    def __init__(self):
        self.mode="open"
        self.target=150.
        self.armed=True
        self.cycles=0
        self.completed=set()
        self.start_site=None
        self.engaged=None
        self.trip_height=0.
        self.low=150.
        self.peak=0.
        self.reason=""
        self.dwell=0.
        self.seated=False

    def observe(self,s,dt):
        s.check()
        if not 0<dt<=.01:
            raise ValueError("Invalid physics timestep")
        self.seated=abs(s.cassette)<=3
        if not self.seated:
            self.completed.discard("right")
        self.peak=max(self.peak,s.crush)
        if s.crush>=15:
            self.reason=self.reason or "crush"
        if not (-2<=s.cassette<=50 and abs(s.carriage)<=70 and 27<=s.height<=152
                and -2<=s.crush<=24 and max(abs(s.left),abs(s.right))<=11):
            self.reason=self.reason or "guide"
        if s.button<2:
            self.armed=True
        if self.mode=="open":
            if s.button>=5 and self.armed:
                self.armed=False
                if s.button_hand:
                    self.mode="down"
                    self.start_site=s.site()
                    self.engaged=None
                    self.cycles+=1
                    self.low=s.height
        if self.mode!="open":
            if abs(s.guide_lift_mm)>2:
                self.reason=self.reason or "unseated witness"
            self.low=min(s.height,self.low)
            if s.assist:
                self.reason=self.reason or "assisted proof"
            if self.start_site and s.site()!=self.start_site:
                self.reason=self.reason or "moving witness during proof"
        if self.mode=="down":
            switch=[name for name in SIDES if getattr(s,name)>=2.0]
            if switch:
                self.mode="up"
                self.trip_height=s.height
                site=self.start_site
                if (site and site in switch and getattr(s,"contact_"+site)>=.04 and
                    150-s.height>=50 and s.site()==site):
                    self.engaged=site
                elif site is None and any(getattr(s,"contact_"+name)>=.04 for name in switch):
                    pass  # A genuine off-mark test is permitted; neither marked proof is complete.
                else:
                    self.reason=self.reason or "bypassed reversal"
            elif s.height<=31 or s.crush>=15:
                self.mode="up"  # independent travel/overload return on the trainer
        if self.mode=="up" and s.height>=149.5:
            if self.engaged and (self.engaged=="left" or self.seated) and s.height-self.low>=65:
                self.completed.add(self.engaged)
            self.mode="open"
        if self.mode=="down":
            self.target=max(30.,self.target-65*dt)
        elif self.mode=="up":
            self.target=min(150.,self.target+90*dt)
        else:
            self.target=150.
        if self.mode=="open" and self.seated and len(self.completed)==2 and not s.assist and s.button<2:
            self.dwell+=dt
        else:
            self.dwell=0.

    @property
    def score(self):
        if self.reason or not self.seated:
            return 0.
        if self.dwell+1e-9>=.5:
            return 1.
        return .25 if self.completed else 0.
