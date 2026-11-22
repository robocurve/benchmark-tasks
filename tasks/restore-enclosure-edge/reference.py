"""Only YAM joints receive policy actions. Cartesian waypoints are in the station frame."""
from math import radians
from fixture import WORLD, MODES
from yam_arena.constants import FINGER_OPEN as OPEN, FINGER_CLOSED as CLOSED
from yam_arena.demo import Waypoint

def plan(mode="reference"):
    if mode not in MODES: raise ValueError(mode)
    if mode=="idle": return {}
    factor=1.18 if mode=="slow_reference" else 1.
    left,right=[],[]
    left_yaw = 0 if mode == "operator_touch" else -85
    def add(arm,t,x,y,z,f,label,yaw=None):
        if yaw is None:
            yaw = left_yaw if arm is left else 0
        arm.append(Waypoint(t*factor,tuple(a+b for a,b in zip(WORLD,(x,y,z))),f,label,radians(yaw)))
    if mode=="operator_touch":
        add(left,2.2,-.14,.065,.14,OPEN,"move over the fixed loading shelf")
        add(left,1.5,-.14,.065,.042,CLOSED,"touch the stationary glove")
        return {"left":left}
    park_l=(-.115,.240,.185)
    park_r=(-.118,-.273,.167)
    repair=mode not in ("unrepaired_probe","displaced_empty_cycle")
    watch_r=(-.110,-.090,.195)
    if repair:
        add(left,2.2,-.088,-.007,.17,CLOSED,"approach turned-out rubber face from front")
        add(left,.9,-.062,-.007,.163,CLOSED,"line up with the lower exposed lip")
        add(left,1.6,-.013,-.007,.161,CLOSED,"roll the loose clip foot into its retainer")
        add(left,.5,-.012,-.007,.161,CLOSED,"finish seating")
        add(left,.85,-.083,-.007,.163,CLOSED,"back away from rail")
    else:
        add(left,6.05,*park_l,OPEN,"wait")
    add(left,1.1,*park_l,OPEN,"clear station")
    if mode=="repair_only": return {"left":left}
    deploy=mode not in ("empty_cycle","displaced_empty_cycle")
    if deploy:
        add(right,2.,-.153,-.258,.126,CLOSED,"approach rear face of test carriage ear")
        add(right,1.,-.145,-.258,.058,CLOSED,"line up beside handle")
        add(right,2.6,.012,-.258,.058,CLOSED,"advance carriage to test station")
        add(right,1.5,-.070,-.268,.137,OPEN,"release carriage")
        add(right,1.1,*watch_r,OPEN,"observe guard and cylinder from the front",yaw=-70)
    left_yaw = -45
    add(left,2.4,-.133,.160,.125,CLOSED,"above cycle button")
    add(left,.8,-.133,.160,.072,CLOSED,"press cycle button")
    add(left,.65,-.133,.160,.125,OPEN,"release button")
    add(left,1.3,*park_l,OPEN,"withdraw")
    if deploy and mode not in ("leave_probe","unrepaired_probe"):
        add(right,10.,*watch_r,OPEN,"observe contact and reversal",yaw=-70)
        add(right,1.4,-.140,-.205,.205,OPEN,"back away before rounding the guide post",yaw=-70)
        add(right,1.65,.051,-.258,.125,CLOSED,"above opposite face of carriage ear")
        add(right,.85,.051,-.258,.062,CLOSED,"line up for return")
        add(right,2.6,-.103,-.258,.062,CLOSED,"return instrument carriage to holder")
        add(right,1.3,*park_r,OPEN,"withdraw")
    return dict(left=left,**({"right":right} if right else {}))
