"""Station Cartesian wrist paths. Only the arms receive commands."""
from math import pi
from fixture import WORLD
from yam_arena.constants import FINGER_OPEN as OPEN, FINGER_CLOSED as CLOSED
from yam_arena.demo import Waypoint
from fixture import MODES
def wp(t,x,y,z,f,tag,yaw):
    return Waypoint(t,(WORLD[0]+x,WORLD[1]+y,WORLD[2]+z),f,tag,yaw)
def plan(mode="reference"):
    if mode not in MODES:raise ValueError(mode)
    if mode=="idle":return {}
    if mode=="recover_after_tug":
        # A complete second attempt tests whether an attractive final pose erases the first peak.
        first=plan("closed_pull")["right"]
        safe=plan("reference")
        delay=sum(w.duration for w in first)
        wait=wp(delay,-.065,.304,.194,OPEN,"wait outside the table until the first draw ends",pi/2)
        return {"left":[wait,*safe["left"]],"right":[*first,*safe["right"]]}
    slow=1.25 if mode=="slow_reference" else 1.
    left=[];right=[]
    def l(t,x,z,f,tag):left.append(wp(t*slow,x+.003,.140,z+.004,f,tag,pi/2))
    def r(t,x,y,z,f,tag):right.append(wp(t*slow,x,y,z,f,tag,pi/2))
    if mode=="surface_touch":
        r(2.,-.215,-.18,.165,OPEN,"approach clear part of table")
        r(1.8,-.215,-.18,.050,CLOSED,"touch feed table")
        r(1.4,-.215,-.18,.17,OPEN,"withdraw from surface")
        return {"right":right}
    if mode=="hand_press":
        r(2.5,-.11,.022,.16,CLOSED,"approach glove")
        r(2.,-.11,.022,.065,CLOSED,"press palm")
        r(2.,-.11,.022,.16,CLOSED,"withdraw")
        return {"right":right}
    if mode!="closed_pull":
        # Follow the centre of the red handle about the shoe pivot. Its flat grip
        # is across the two jaw faces. The same open approach precedes both strokes.
        from math import sin,cos
        def handle(angle):
            return (.060-.036*cos(angle)+.074*sin(angle), .088+.036*sin(angle)+.074*cos(angle))
        x0,z0=handle(-.135)
        left.append(wp(1.7*slow,x0+.003,.207,.165,OPEN,"open outside release handle",pi/2))
        l(1.25,x0,z0-.003,OPEN,"straddle red grip")
        pinch=-.004
        l(.85,x0,z0-.003,pinch,"close around red grip")
        end=-.083 if mode=="partial_release" else .30
        for j in range(1,5):
            x,z=handle(-.135+(end+.135)*j/4)
            l(.55,x,z-.003,pinch,"lift pressure shoe by the grip")
        if mode=="abandon_release":
            l(.45,x,z+.003,OPEN,"release grip before draw")
            left.append(wp(.75*slow,x+.003,.207,.165,OPEN,"clear returning shoe",pi/2))
        else:
            l(7.4,x,z-.003,pinch,"hold red grip around both faces")
            l(.65,x,z-.003,OPEN,"open left jaws before retreat")
            left.append(wp(1.*slow,x+.003,.207,.165,OPEN,"clear returning shoe",pi/2))
        left.append(wp(2.2*slow,-.065,.304,.194,OPEN,"park beyond table",pi/2))
        left.append(wp(2.0*slow,-.065,.304,.194,OPEN,"stay clear",pi/2))
    if mode!="release_only":
        # Centre the jaws on the natural forearm sleeve before the elbow moves.
        # Each SDK finger travels 25 mm from closed; object contact arrests closure.
        grasp=-.025
        y0=-.102
        z0=.090
        x0=-.113
        r(2.5,x0,y0,.176,OPEN,"over proximal sleeve")
        r(1.8,x0,y0,z0,OPEN,"straddle forearm away from palm")
        r(1.4,x0,y0,z0,grasp,"close on fabric")
        r(1.4,x0,y0,z0,grasp,"steady forearm")
        from math import sin,cos
        radius=.238+y0
        for j in range(1,7):
            angle=.68*j/6
            x=x0-1.13*radius*sin(angle)
            y=-.238+radius*cos(angle)
            r(.59,x,y,z0,grasp,"guide sleeve about passive elbow")
        if mode!="held_arm":
            r(.6,x,y,z0,OPEN,"open both jaws around sleeve")
            r(1.1,x,y,.182,OPEN,"lift vertically before retreat")
            if mode=="open_hover":
                r(6.,x,y,.182,OPEN,"hover above sleeve")
            else:
                parked=CLOSED if mode=="park_closed" else OPEN
                r(2.3,-.155,-.353,.185,parked,"park outside feed table")
                r(2.,-.155,-.353,.185,parked,"hold parking pose")
        else:
            r(8.,x,y,z0,grasp,"hold the clothed forearm")
    return {s:p for s,p in (("left",left),("right",right)) if p}
