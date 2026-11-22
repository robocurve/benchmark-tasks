"""Same end-state evaluator for solver readback and regression playback."""
from dataclasses import asdict, dataclass
from math import cos, isfinite, radians
from fixture import INCLINE
from peg_return import HOME, END_STROKE, ReturnClutch

# Beam's far edge + setback of the longest fingertip + a visible 10 mm witness gap.
CLEAR = (.015 + .022 + .010) / cos(INCLINE)
STABLE_TICKS = 30   # 1.5 s at the official 20 Hz
HIT_N = 1.0         # contact witness force per pair at a solver step
@dataclass(frozen=True)
class Frame:
    q: dict
    v: dict
    digit_n: tuple
    seat_n: tuple
    human_contact: bool
    robot_touch: bool
    arms_clear: bool
    landing_n: tuple = (0.,0.)
    key_robot_touch: bool = False
    return_active: bool = False

NAMES = {"dial_left","dial_right","flex_middle","flex_index","flex_ring","flex_little","heave","roll","withdraw","advance_left","advance_right","pad_left","pad_right"}

class Evaluator:
    def __init__(self):
        self.initial = None
        self.fault = ""
        self.score = 0.
        self.dwell = 0
        self.clear_once = False
        self.previous = 0.
        self.loaded_travel = 0.
        self.relief_counts=[0,0]
        self.unloaded_pad=[None,None]
        self.tick = 0
        self.clutch = ReturnClutch()
        self.pegs_at_rest = False

    def fail(self, reason):
        self.fault = self.fault or reason
        self.score = 0.
        self.dwell = 0

    def observe(self, f):
        self.tick += 1
        if set(f.q)!=NAMES or set(f.v)!=NAMES or len(f.digit_n)!=2 or len(f.seat_n)!=2 or len(f.landing_n)!=2:
            self.fail("missing_observation")
            return
        if not all(isfinite(x) for x in (*f.q.values(),*f.v.values(),*f.digit_n,*f.seat_n,*f.landing_n)):
            self.fail("nonfinite_observation")
            return
        pads = [f.q["pad_"+s] for s in ("left","right")]
        travel = f.q["withdraw"]
        if self.initial is None:
            self.initial = pads
            self.previous = travel
            if not (-.001 <= travel <= .003 and min(f.digit_n)>.20 and max(f.seat_n)<.10 and
                    max(abs(f.q["advance_"+s]) for s in ("left","right"))<.003):
                self.fail("invalid_pinned_start")
        if f.return_active != self.clutch.active:
            self.fail("return_state_mismatch")
        at_rest=(max(abs(f.q["advance_"+s]) for s in ("left","right"))<=HOME and
                 max(abs(f.v["advance_"+s]) for s in ("left","right"))<.004)
        if f.return_active and f.key_robot_touch and not (self.pegs_at_rest and at_rest):
            self.fail("return_hit_robot")
        self.pegs_at_rest = f.return_active and at_rest
        self.clutch.update(f.q,f.digit_n,f.seat_n,f.robot_touch,f.arms_clear)
        if f.human_contact:
            self.fail("hand_struck")
        # A first release can occur before the fingertip clears the edge.
        # Detect a real return of load and compression on the same digit pair.
        for i,(load,position) in enumerate(zip(f.digit_n,pads)):
            if load<.23 and min(f.seat_n)>.30:
                self.relief_counts[i]+=1
                if self.relief_counts[i]>=2:
                    self.unloaded_pad[i]=position
            else:
                self.relief_counts[i]=0
            relieved=self.unloaded_pad[i]
            if relieved is not None and travel<CLEAR and load>1.0 and relieved-position>.001:
                self.fail("repinned_"+("left","right")[i])
        if any(start-now>.003 for start,now in zip(self.initial,pads)):
            self.fail("additional_finger_compression")
        if max(f.digit_n)>.45:
            self.loaded_travel += max(0.,travel-self.previous)
        self.previous=travel
        if self.loaded_travel>.005:
            self.fail("dragged_under_load")
        if self.clear_once and (travel < CLEAR-.004 or max(f.digit_n)>.45):
            self.fail("hand_returned_under_load")
        if self.fault:
            return
        clear = travel>=CLEAR and max(f.digit_n)<.15
        self.clear_once |= clear
        parked = (f.return_active and travel>=END_STROKE and min(f.landing_n)>.30 and max(f.seat_n)<.15
                  and max(abs(f.q['advance_'+s]) for s in ('left','right'))<=HOME)
        slow = all(abs(f.v[n]) < (radians(3) if n=="roll" or n.startswith(("flex_","dial_")) else .004) for n in NAMES)
        # Positive bearing at both ends, plus continuous absence of robot
        # contact. A harmless hover off the hardware does not carry its load.
        stable = clear and parked and slow and not f.robot_touch
        self.dwell = self.dwell+1 if stable else 0
        self.score = 1. if self.dwell>=STABLE_TICKS else .5 if clear else 0.

    def record(self, f):
        return dict(tick=self.tick,**asdict(f),fault=self.fault,score=self.score,dwell=self.dwell)
