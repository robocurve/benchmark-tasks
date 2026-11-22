"""Machine controller and independent outcome scorer; SI units and 20 Hz samples."""
from dataclasses import dataclass
from math import isfinite

DT = .05
HOME = .005
HOLD = 21
ACTION_LIMIT = 1200

@dataclass(frozen=True)
class Sample:
    height: float
    switch_travel: float
    foot_roll: float
    insertion: float
    pointer: float
    button_z: float
    trip_flag: float
    probe_edge: bool
    robot_touch: bool
    hazardous_contact: bool
    parked: bool
    peak_compression: float
    operator_contact: bool = False

class Machine:
    def __init__(self):
        self.target = 0.
        self.retained = False
        self.phase = "idle"
        self.pressed = False
        self.signal = False
        self.cycle = 0
    def step(self,s):
        if s.foot_roll < -.50: self.retained = True
        if s.foot_roll > -.31: self.retained = False
        pressed = s.button_z < -.006
        if pressed and not self.pressed and self.phase=="idle":
            self.cycle += 1
            self.phase = "closing"
            self.signal = False
        self.pressed = pressed
        if max(-s.pointer,s.peak_compression)>=.030 and self.phase!="inhibited":
            self.phase="inhibited"
            self.target=s.height
        if self.phase=="closing":
            if s.switch_travel >= .0043:
                self.signal = True
                self.phase = "reversing"
            elif self.target <= -.122 and s.height <= -.117:
                self.phase = "return_limit"
        if self.phase=="closing": self.target=max(-.122,self.target-.020*DT)
        if self.phase in ("reversing","return_limit"):
            self.target=min(0.,self.target+.025*DT)
            if self.target==0. and abs(s.height)<=HOME: self.phase="idle"
        return {"height":self.target,"foot_roll":-1.36 if self.retained else s.foot_roll*(1.-.13/4.5),
                "trip_flag":.032 if self.signal else 0.}

class Evaluator:
    def __init__(self):
        self.tick=0
        self.fault=""
        self.score=0.
        self.dwell=0
        self.cycle=0
        self.tripped=False
        self.proved=False
        self.low=0.
        self.deep=0.
        self.saw_contact=False
        self.previous_signal=False
    def observe(self,s,m):
        self.tick+=1
        if not all(isfinite(v) for v in vars(s).values()): self.fault=self.fault or "invalid_measurement"
        if self.tick==1 and (abs(s.height)>HOME or s.foot_roll<-.30 or s.insertion>.010 or -s.pointer>.005):
            self.fault=self.fault or "invalid_reset"
        if s.operator_contact: self.fault=self.fault or "operator_contact"
        if s.hazardous_contact: self.fault=self.fault or "robot_in_closing_zone"
        if max(-s.pointer,s.peak_compression)>=.030: self.fault=self.fault or "probe_overtravel"
        seated=s.foot_roll<=-1.02
        if m.cycle!=self.cycle:
            self.cycle=m.cycle
            self.saw_contact=self.tripped=self.proved=False
            self.deep=0.
            self.low=s.height
        if self.cycle and m.phase in ("closing","reversing"):
            self.low=min(self.low,s.height)
            if s.insertion>=.102 and s.probe_edge and seated and not s.robot_touch:
                self.deep=max(self.deep,-s.pointer)
                if m.signal and not self.previous_signal and self.low<=-.035:
                    self.saw_contact=True
                if self.saw_contact and m.signal and self.deep>=.015: self.tripped=True
        self.previous_signal=m.signal
        if self.tripped and m.phase=="idle" and abs(s.height)<=HOME and s.trip_flag>=.025:
            self.proved=True
        if not seated:
            self.proved=self.tripped=self.saw_contact=False
            self.deep=0.
        if self.tick>=ACTION_LIMIT and s.insertion>.008:
            self.fault=self.fault or "probe_left_in_path"
        done=(self.proved and seated and m.phase=="idle" and abs(s.height)<=HOME and
              s.insertion<=.008 and -s.pointer<=.005 and not s.robot_touch and s.parked and not self.fault)
        self.dwell=self.dwell+1 if done else 0
        self.score=0. if self.fault else 1. if self.dwell>=HOLD else .75 if self.proved else .5 if seated else 0.
