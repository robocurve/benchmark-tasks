"""Commission a guarded linear encoder in the dual-YAM station."""
import json
import math
import os
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
from parts import Slider, on_reset, part
from yam_arena.demo import Waypoint

SCENE = "station"
SUCCESS_HOLD_S = 0.0
SCORE_OVER = "end"
INSTRUCTION = "Slide the blue GAP tab to its end mark, slide the orange LOCK tab, move the black carriage to the far stop and back to its starting stop, then hold the green CAL button for half a second and release it."

NIGHT = (.095,.13,.20)
METAL = (.41,.48,.55)
SILVER = (.65,.72,.78)
BLUE = (.08,.52,.88)
ORANGE = (.98,.51,.12)
GREEN = (.22,.84,.49)
BLACK = (.075,.085,.095)
Z = .7475
Y_RAIL = -.048
X_HOME = .420
X_END = .170
Y_TABS = .098
GAP_X, LOCK_X = .229,.327
CAL_X, CAL_Y = .346,-.127


def box(x, y, z, dx, dy, dz, color):
    return ((x,y,z),(dx,dy,dz),color)


def tab(name, x, color):
    p = (x,Y_TABS,.798)
    return Slider(name, rest=p, boxes=(box(x,Y_TABS,.798,.047,.027,.026,color),
        box(x,Y_TABS,.779,.028,.015,.014,NIGHT)),
        travel=.015, rate=.001, damping=8.0, mass=.065, axis="Y")


OBJECTS = [part("encoder_fixture", pos=(0.,0.,0.), mass=14., friction=1.2,
    color=NIGHT, boxes=(
        box(.295,-.001,Z+.004,.370,.287,.008,NIGHT),
        box(.295,Y_RAIL,.763,.327,.065,.014,METAL),
        box(.295,Y_RAIL-.031,.779,.325,.010,.025,SILVER),
        box(.295,Y_RAIL+.030,.779,.325,.010,.025,SILVER),
        box(.295,Y_RAIL+.030,.792,.296,.010,.004,NIGHT),
        box(.295,Y_RAIL+.030,.795,.004,.011,.002,ORANGE),
        box(.170,Y_RAIL+.030,.795,.003,.011,.002,GREEN),
        box(.420,Y_RAIL+.030,.795,.003,.011,.002,GREEN),
        box(.277,.086,.762,.177,.068,.011,METAL),
        box(GAP_X,.062,.769,.054,.006,.004,GREEN),
        box(LOCK_X,.062,.769,.054,.006,.004,ORANGE),
        box(CAL_X,CAL_Y,.764,.058,.032,.017,NIGHT),
    ), sliders=(
        tab("gap", GAP_X, BLUE), tab("lock", LOCK_X, ORANGE),
        Slider("carriage", rest=(X_HOME,Y_RAIL,.801), boxes=(
            box(X_HOME,Y_RAIL,.799,.042,.046,.014,METAL),
            box(X_HOME,Y_RAIL,.815,.044,.062,.026,BLACK),
            box(X_HOME,Y_RAIL,.830,.032,.043,.004,SILVER),
        ), travel=.250, rate=.001, damping=4., mass=.18, axis="X"),
        Slider("cal", rest=(CAL_X,CAL_Y,.790), boxes=(
            box(CAL_X,CAL_Y,.792,.039,.026,.017,GREEN),
        ), travel=.007, rate=2100., damping=14., mass=.045, axis="Z"),
    ))]


def crossing(a,b):
    return min(a,b) <= .125 <= max(a,b)


class Trial:
    def __init__(self):
        self.reset()

    def reset(self):
        self.phase, self.frames, self.cal, self.confirm = 0,0,0,0
        self.out_index = self.back_index = self.fault = False
        self.old_x = 0.
        self.result = 0.

    def observe(self, sliders, pos, quat):
        self.frames += 1
        gap, lock, x, button = (-float(sliders[n]) for n in ("gap","lock","carriage","cal"))
        ready = gap >= .012 and lock >= .012
        pressed = button >= .0035
        gap_mm = 1.75 + (.35/.015)*gap
        if (not all(math.isfinite(v) for v in (gap, lock, x, button, *pos, *quat))
            or not -.001 <= x <= .251 or math.dist(pos,(0.,0.,0.)) > .009
            or abs(float(quat[3])) < .998):
            self.fault = True
        if not ready:
            self.phase, self.cal, self.confirm = 0,0,0
            self.out_index = self.back_index = False
        if self.phase == 0 and ready and abs(x) <= .003:
            self.phase = 1
        if self.phase == 1:
            self.out_index |= crossing(self.old_x,x) and x > self.old_x
            if x >= .247 and self.out_index:
                self.phase = 2
        elif self.phase == 2:
            self.back_index |= crossing(self.old_x,x) and x < self.old_x
            if x <= .003 and self.back_index:
                self.phase = 3
        if pressed and (self.phase < 3 or abs(x) > .003):
            self.fault = True
        if self.phase >= 3:
            if abs(x) > .003:
                self.phase, self.cal, self.confirm = 0,0,0
                self.out_index = self.back_index = False
            elif pressed:
                self.cal += 1
                self.confirm = 0
            elif self.cal >= 10:
                self.confirm += 1
                if self.confirm >= 40:
                    self.phase = 4
            else:
                self.cal = 0
        self.old_x = x
        self.result = 0. if self.fault else .25*self.phase
        if os.getenv("TASK_TRACE"):
            with open(os.environ["TASK_TRACE"],"a",encoding="utf-8") as trace:
                trace.write(json.dumps(dict(step=self.frames,gap_mm=gap_mm,gap=gap,lock=lock,x=x,cal=button,
                    phase=self.phase,fault=self.fault,fixture=list(map(float,pos)),score=self.result))+"\n")
        return self.result


trial = Trial()
last = None


@on_reset
def reset(*_):
    global last
    last = None
    trial.reset()


def score(objects):
    global last
    if last is not objects:
        last = objects
        state = objects["encoder_fixture"]
        trial.observe(state.sliders,state.pos,state.quat_xyzw)
    return trial.result


def success(objects):
    return score(objects) == 1.


def demo():
    l,r = [],[]
    lh,rh = (.213,.209,.855),(.213,-.209,.855)
    finger = -.015
    def phase(arm, duration, pos, label, yaw=0.):
        if arm == "left":
            l.append(Waypoint(duration,pos,0. if label.startswith("CAL") else finger,label,yaw)); r.append(Waypoint(duration,rh,finger,"wait"))
        else:
            l.append(Waypoint(duration,lh,finger,"wait")); r.append(Waypoint(duration,pos,0. if label.startswith("CAL") else finger,label,yaw))
    phase("left",1.,lh,"settle")
    for name,x in (("gap",GAP_X),("lock",LOCK_X)):
        phase("left",1.,(x,.151,.859),name+" approach",math.pi/2)
        phase("left",.8,(x,.151,.809),name+" lower",math.pi/2)
        phase("left",1.8,(x,.110,.809),name+" press",math.pi/2)
        phase("left",.6,(x,.110,.859),name+" lift",math.pi/2)
    phase("left",.9,lh,"park")
    for label,x0,x1 in (("out",.473,.194),("back",.117,.389)):
        phase("right",1.2,(x0,Y_RAIL,.867),label+" approach")
        phase("right",.8,(x0,Y_RAIL,.820),label+" lower")
        phase("right",4.,(x1,Y_RAIL,.820),label+" push")
        phase("right",.65,(x1,Y_RAIL,.867),label+" lift")
    if not os.getenv("SKIP_CAL"):
        phase("right",1.,(CAL_X,CAL_Y,.866),"CAL approach")
        phase("right",1.,(CAL_X,CAL_Y,.789),"CAL press")
        phase("right",.75,(CAL_X,CAL_Y,.789),"CAL hold")
        phase("right",.65,(CAL_X,CAL_Y,.866),"CAL release")
    phase("right",1.,rh,"park")
    phase("right",2.5,rh,"confirmation")
    return {"left":l,"right":r}
