"""Diagnostic arm commands for direct bilateral lifting of the cover saddles."""
from yam_arena.constants import FINGER_OPEN, FINGER_CLOSED
from yam_arena.demo import Waypoint
from fixture import SIDES,WORLD
from math import pi
EARLY=("unload_left","unload_right","unload_both","keep_support","hands_off_early")

def retreat(side, duration, label):
    sign=1 if side=="left" else -1
    return Waypoint(duration,(WORLD[0]+.020,sign*.185,WORLD[2]+.169),FINGER_OPEN,label,-sign*7*pi/12)

def plan(mode="reference"):
    if mode=="touch_cuff":
        return {"right":[Waypoint(t,(WORLD[0]+.09,-.012,WORLD[2]+z),FINGER_CLOSED,label)
                for t,z,label in [(3.,.14,"approach glove"),(3.,.045,"touch"),(2.,.14,"retreat")]]}
    arms={}
    for side,sgn in zip(SIDES,(1,-1)):
        if mode=="idle" or (mode=="one_key" and side=="right"):continue
        def w(tm,z,f,label,x=.041):
            return Waypoint(tm,(WORLD[0]+x,sgn*.108,WORLD[2]+z),f,label,-sgn*7*pi/12)
        moves=[w(3,.17,FINGER_OPEN,"approach outboard tongue"),w(3.,.080,FINGER_OPEN,"straddle lift tongue"),
               w(2.,.080,FINGER_CLOSED,"grip textured side faces"),w(3.5,.136,FINGER_CLOSED,"lift cover")]
        moves.append(w(5.5,.136,FINGER_CLOSED,"maintain bilateral relief"))
        if mode not in ("holding","one_key"):
            moves.extend([retreat(side,.55,"open and withdraw"),retreat(side,3,"observe landing")])
        arms[side]=moves
    return arms

def intervention(side,mode,position):
    x,y,z=map(float,position)
    def w(t,z,f,label):
        return Waypoint(t,(x,y,z),f,label,(-1 if side=="left" else 1)*7*pi/12)
    low=mode in ("unload_both","hands_off_early","unload_"+side)
    if low:
        if mode=="hands_off_early":
            return [w(.25,z,FINGER_OPEN,"release early"),retreat(side,1.,"retreat")]
        return [w(.55,WORLD[2]+.064,FINGER_CLOSED,"lower this side"),w(1.,WORLD[2]+.064,FINGER_CLOSED,"hold low"),w(.4,WORLD[2]+.065,FINGER_OPEN,"disengage"),retreat(side,1.,"retreat")]
    h=WORLD[2]+.136
    return [w(1.6,h,FINGER_CLOSED,"continue lifting"),w(5.,h,FINGER_CLOSED,"hold until clear"),
            retreat(side,.55,"open and withdraw"),retreat(side,3,"observe")]

def may_branch(row):
    return (row["q"]["withdraw"]<.018 and min(row["seat_n"])>.30
            and max(row["digit_n"])<.23 and min(row["q"]["advance_"+s] for s in SIDES)>.012)
