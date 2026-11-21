"""Restore the shutter's contact edge and use the orange proof block to demonstrate reversal at both white marks."""
import os
import sys
from pathlib import Path
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
from yam_arena.task_file import TABLE_TOP, Box
from shutter_bridge import install

SCENE="station"
X=.335
OBJECTS=[Box("shutter_trainer",pos=(X,0,TABLE_TOP[SCENE]),size=(.001,.001,.001))]
SUCCESS_HOLD_S=.6
SCORE_OVER="end"
monitor=None

def score(objects):
    if monitor is None:
        raise RuntimeError("Substep observer missing")
    return monitor.trial.score

def success(objects):
    return score(objects)==1.

def demo():
    from yam_arena.demo import Waypoint
    from yam_arena.constants import FINGER_OPEN as O,FINGER_CLOSED as C,PAD_TIP_PAST_TCP
    mode=os.environ.get("SHUTTER_CONTROL","reference")
    if mode not in ("reference","alternate","one_site","dead_edge","idle"):
        raise ValueError(mode)
    plans={"left":[],"right":[]}
    homes={"right":(-.085,-.310,.145,C),"left":(-.085,.310,.145,C)}
    def phase(arm,seq):
        for t,(x,y,z,f),label in seq:
            for hand in plans:
                xx,yy,zz,ff=(x,y,z,f) if hand==arm else homes[hand]
                plans[hand].append(Waypoint(t,(X+xx,yy,TABLE_TOP[SCENE]+zz+PAD_TIP_PAST_TCP),ff,label))
    def repair():
        # Bring the retracted shoe into the door plane using its orange handle.
        handle_grip=-.003
        phase("right",[(1.4,(-.043,-.066,.18,O),"approach cassette"),
                       (1.0,(-.043,-.066,.163,O),"align jaws"),
                       (1.15,(-.043,-.066,.163,handle_grip),"grip orange handle"),
                       (.40,(-.043,-.066,.163,handle_grip),"verify handle grip"),
                       (1.65,(-.094,-.066,.163,handle_grip),"slide to witness line"),
                       (.45,(-.094,-.066,.163,handle_grip),"verify cassette seat"),
                       (.9,(-.094,-.066,.163,O),"release"),
                       (.85,(-.160,-.066,.163,O),"withdraw straight"),
                       (.55,(-.160,-.150,.163,O),"move outboard open"),
                       (1.0,homes["right"],"park right outboard")])
    def transfer(a,b):
        hand="right" if a<0 else "left"
        x=-.161
        pickup_y=a+(.0005 if a>0 else -.0005)
        # The 24 mm pickup seats between jaws commanded to a 18 mm gap.
        pinch=-.009
        phase(hand,[(1.25,(x,pickup_y,.115,O),"hover open"),
                    (1.15,(x,pickup_y,.054,O),"straddle pickup"),
                    (1.35,(x,pickup_y,.054,pinch),"close on pickup"),
                    (.55,(x,pickup_y,.054,pinch),"verify hold"),
                    (.85,(x,pickup_y,.069,pinch),"lift vertically"),
                    (.45,(x,pickup_y,.069,pinch),"verify lift"),
                    (2.8,(x,b,.069,pinch),"carry pickup"),
                    (.4,(x,b,.069,pinch),"settle held"),
                    (.85,(x,b,.054,pinch),"lower at mark"),
                    (.45,(x,b,.054,pinch),"verify seated"),
                    (1.1,(x,b,.054,O),"open at mark"),
                    (.85,(x,b,.115,O),"clear pickup"),
                    (.95,(x,.2 if hand=="left" else -.2,.115,O),"withdraw outboard"),
                    (.9,homes[hand],"retreat")])
    def cycle():
        phase("left",[(1.1,(-.155,.210,.096,C),"align to start"),
                      (.9,(-.155,.210,.040,C),"press green button"),
                      (.45,(-.155,.210,.039,C),"hold briefly"),
                      (.7,homes["left"],"release start"),(2.,homes["left"],"observe return")])
    if mode in ("reference","one_site","alternate"):
        repair()
    if mode=="reference":
        transfer(.066,-.066); cycle(); transfer(-.066,.066); cycle()
    elif mode=="dead_edge":
        transfer(.066,-.066);cycle()
    elif mode in ("alternate","one_site"):
        cycle()
        if mode=="alternate":
            transfer(.066,-.066);cycle()
    phase("left",[(1.6,homes["left"],"hands clear")])
    return plans

install(HERE/"entry.py")
