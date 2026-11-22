"""Measured roller clearance and the peak elastic load at the surrogate wrist."""
import math
from dataclasses import dataclass,asdict
from fixture import TRIP_N, FOREARM_LIMIT_N, GLOVE_LIMIT_N
from withdrawal import MIN_CLEAR
CLEAR_X=-.029
REST_N=.45
HOLD=31
@dataclass
class Frame:
    tip_max_x:float      # maximum x over every solid corner of every textile patch
    wrist_x:float
    gap_m:float         # rise of shoe at roller crown
    peak_n:float        # maximum spring tension in this action interval
    load_n:float
    nip_n:float         # shoe-to-textile normal force
    rest_n:float        # table-to-arm vertical support
    robot_touch:bool
    arm_speed:float     # absolute elbow angular speed, rad/s
    cloth_speed:float   # fastest seam angular speed, rad/s
    left_lever:bool
    right_arm:bool
    hand_n:float  # peak direct robot or pressure-shoe contact on the glove
    left_open:float
    right_open:float
    left_clear_m:float
    right_clear_m:float
    right_surface:bool
    base_clear_m:float
    feed_clear_m:float
    bench_clear_m:float
    forearm_n:float  # maximum simultaneous sum of robot normal loads across the sleeved limb
    table_n:float  # peak direct robot/feed-board load, diagnostic only

class Evaluator:
    def __init__(self):
        self.tick=0;self.dwell=0;self.fault="";self.score=0.
    def observe(self,f):
        self.tick+=1
        if not all(math.isfinite(v) for v in asdict(f).values()) or min(f.load_n,f.peak_n,f.nip_n,f.rest_n,f.hand_n,f.forearm_n,f.table_n,f.left_clear_m,f.right_clear_m)<0 or f.peak_n+.0001<f.load_n:
            self.fault=self.fault or "invalid_measurement"
        elif self.tick==1 and (f.nip_n<.5 or f.tip_max_x<.010 or f.gap_m>.008 or f.wrist_x>-.065 or f.wrist_x<-.155):
            self.fault="invalid_capture"
        elif f.base_clear_m<=0.:self.fault=self.fault or "right_base_clearance"
        elif f.forearm_n>=FOREARM_LIMIT_N:self.fault=self.fault or "forearm_crush"
        elif f.hand_n>=GLOVE_LIMIT_N:self.fault=self.fault or "glove_struck"
        elif f.peak_n>=TRIP_N:self.fault=self.fault or "wrist_overload"
        cleared=f.tip_max_x<=CLEAR_X and f.wrist_x<-.165 and f.load_n<1.
        withdrawn=min(f.left_clear_m,f.right_clear_m)>=MIN_CLEAR
        settled=cleared and withdrawn and f.rest_n>=REST_N and not f.robot_touch and f.arm_speed<.06 and f.cloth_speed<.45
        self.dwell=self.dwell+1 if settled and not self.fault else 0
        self.score=0. if self.fault or not cleared else 1. if self.dwell>=HOLD else .5
    def record(self,f):
        return dict(**asdict(f),tick=self.tick,dwell=self.dwell,score=self.score,fault=self.fault)
