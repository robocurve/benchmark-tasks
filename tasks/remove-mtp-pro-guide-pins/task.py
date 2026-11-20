"""Keep the field-tool release held while capturing two pins in the extractor."""
from dataclasses import dataclass
from bench import TABLE, TIP, OPEN, HOVER, Track, fixed, rig, bodies, contacts, register_trial, proof, waypoints
from runtime import asset, install_writer, wrench
from touch import exchanger_face_contact

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
MAX_STEPS = 840
INSTRUCTION = "Hold the black field-tool release. Grip the green pin exchanger and pull it out while keeping the release pressed. Keep clear of the ferrule."
BASE = (.284,0,TABLE)
STATE = {}


@dataclass
class Trial:
    gate: bool = False
    invalid: bool = False
    cleared: bool = False
    retained: bool = False
    hold: float = 0.0
    value: float = 0.0

    def step(self, black, extraction, pins, black_hand, green_hand, face, dt):
        self.gate = black >= .007 and black_hand and green_hand
        self.invalid |= face or (not self.cleared and extraction > .003 and not self.gate)
        self.cleared |= extraction >= .031 and self.gate
        captured = sum(p >= .029 and abs(p - extraction) <= .002 for p in pins)
        self.retained |= self.gate and extraction >= .032 and captured == 2
        self.invalid |= self.retained and not self.gate and any(abs(p - extraction) > .005 for p in pins)
        good = self.retained and captured == 2 and not green_hand and not black_hand
        self.hold = self.hold + dt if good and not self.invalid else 0.0
        self.value = (0.0 if self.invalid else 1.0 if self.hold >= 1-1e-8 else
                      0.25 * captured if captured else 0.20 if self.gate else 0.0)


_trial = Trial()


def score(objects): return _trial.value
def success(objects): return _trial.value == 1
def telemetry(): return dict(STATE, simultaneous=_trial.gate, invalid=_trial.invalid, hold_s=round(_trial.hold,2))


def setup():
    fixture = rig(BASE, (.20,.23,.06), {"black":0.,"pull":0.,"pin0":0.,"pin1":0.},
                  {"black":0.0,"pull":0.0,"pin.*":.4}, 0.0, 5.0)
    face = fixed("ferrule",(BASE[0],BASE[1], TABLE+.041),(.0012,.007,.003))

    def sample(env):
        unit = asset(env, "rig")
        if not getattr(unit, "_pin_latch", False):
            black = unit.joint_names.index("black")
            pull = unit.joint_names.index("pull")
            pins = [unit.joint_names.index(f"pin{i}") for i in range(2)]
            pin_bodies = [unit.body_names.index(f"guide_pin{i}") for i in range(2)]

            def latch():
                import torch
                q = unit.data.joint_pos.torch
                extracted = q[:, pull:pull+1]
                effort = q.new_zeros(q.shape)
                pin_force = q.new_zeros((q.shape[0], 2, 3))
                target = extracted if _trial.gate or _trial.retained else extracted * 0
                pin_speed = unit.data.joint_vel.torch[:, pins]
                pin_force[:, :, 0] = ((target - q[:, pins])*8 - .16*pin_speed).clamp(-.15,.15)
                wrench(unit, pin_force, ids=pin_bodies)
                effort[:, black:black+1] = (-180 * q[:, black:black+1]).clamp(-4, 4)
                effort[:, pull:pull+1] = torch.where((extracted < .025) & (not _trial.gate), -180*extracted, extracted*0)
                unit.set_joint_effort_target(effort.clamp(-5,5))

            install_writer(unit, latch)
            unit._pin_latch = True
        q = bodies(env, ["rig"])["rig"].joints
        hands = contacts(env, ["rig/black_handle", "rig/exchanger", "ferrule"])
        pins = [q["pin0"], q["pin1"]]
        face_hit = hands["ferrule"] or exchanger_face_contact(env)
        STATE.update(black_mm=round(q["black"]*1000,2), pull_mm=round(q["pull"]*1000,2),
                     pin_mm=[round(x*1000,2) for x in pins], face_contact=face_hit)
        return q["black"], q["pull"], pins, hands["rig/black_handle"], hands["rig/exchanger"], face_hit

    return register_trial([fixture,face],Trial,STATE,sample,globals())


def tracks():
    l,r = Track(),Track()
    a = (BASE[0]-.072, .106, TABLE+.036+TIP)
    p = (a[0]+.037,a[1],a[2])
    l.go((*a[:2],HOVER),OPEN,"hover above black release")
    l.go(a,0,"approach black release")
    l.go(p,0,"hold the field tool release",1.8)
    g=(BASE[0]+.030,-.107,TABLE+.045+TIP-.002)
    e=(g[0]+.044,g[1],g[2])
    r.go((*g[:2],HOVER),OPEN,"wait above green control")
    r.wait_until(l.duration+.35)
    r.go(g,OPEN,"lower around green control")
    r.go(g,-.007,"grip pickup control")
    r.go(e,-.007,"extract while holding black release",3.6)
    r.go(e,-.007,"settle both captured pins",1.0)
    r.go(e,OPEN,"release captured exchanger",.9)
    r.go((*e[:2],HOVER),OPEN,"withdraw from interconnect")
    l.wait_until(r.duration)
    l.go(a,0,"release the black control")
    l.go((*a[:2],HOVER),OPEN,"clear field tool")
    r.wait_until(l.duration)
    return {"left":l,"right":r}


def demo():
    t=tracks()
    return waypoints(t["left"],t["right"])


if __name__=="__main__":
    import sys
    t=Trial()
    t.step(.010,.010,(.010,.010),False,True,False,.25)
    assert t.invalid and t.value==0
    t=Trial()
    t.step(.010,.032,(.032,.032),True,True,True,.25)
    assert t.invalid and t.value==0
    t=Trial()
    t.step(.010,.041,(.040,.040),True,True,False,.05)
    t.step(0,.041,(.040,.040),False,False,False,1)
    assert t.value == 1
    t.step(0,.041,(0,.040),False,False,False,.05)
    t.step(0,.041,(.040,.040),False,False,False,1)
    assert t.invalid and t.value == 0
    proof(tracks(),MAX_STEPS,sys.argv)
else:
    OBJECTS=setup()
