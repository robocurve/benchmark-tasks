"""Stock cart envelope, floor shape, hinged side leaves, and captive lock profiles."""

from math import cos, radians, sin

import numpy as np

WIDTH = .4318
DEPTH = .4064
HEIGHT = .4318
SURFACE = .525
REAR_X = .064
HALF_W = WIDTH/2-.010
BASE_Z = SURFACE+.059
RAIL_Z = SURFACE+HEIGHT-.009
LENGTH = (DEPTH-.022)/2
FOLD = radians(8.)
FLOOR_TILT = radians(86.)
BAR_LEN = .176
BAR_START = .094
BAR_TRAVEL = .092
PANEL = (.27,.28,.30)
RIB = (.18,.19,.21)
SLIDER = (.105,.11,.12)
METAL = (.51,.54,.57)
WHEEL = (.13,.14,.16)
HUB = (.16,.43,.68)
HANDLE = (.19,.19,.20)


def box(centre, dimensions, color=PANEL):
    return (tuple(centre), dimensions, color)


def rear_y(sign):
    return sign*HALF_W


def rear_hinge(sign):
    return np.array([REAR_X,rear_y(sign),BASE_Z])


def middle_hinge(sign):
    return rear_hinge(sign)+[LENGTH*cos(FOLD),-sign*LENGTH*sin(FOLD),0.]


def fore_hinge(sign):
    return rear_hinge(sign)+[2*LENGTH*cos(FOLD),0.,0.]


def folding_side():
    h = RAIL_Z-BASE_Z
    span = LENGTH-.004
    parts = [
        box((LENGTH/2,0.,.100),(span,.010,.186)),
        box((LENGTH/2,0.,.225),(span,.010,.066)),
        box((LENGTH/2,0.,h-.070),(span,.010,.022)),
        box((LENGTH/2,0.,h-.023),(span,.010,.026)),
        box((LENGTH/2,0.,h), (span,.016,.015),RIB),
    ]
    for x in (.010,LENGTH-.010):
        parts.append(box((x,0.,h/2),(.016,.011,h-.026),RIB))
    for z in (.066,.164,.259):
        parts.append(box((LENGTH/2,0.,z),(span-.015,.015,.040),RIB))
    return tuple(parts)


def endwall(front):
    h = RAIL_Z-BASE_Z
    # Each molded end wall keeps the real clear hand opening.
    parts = [
        box((0.,0.,(h-.098)/2),(.014,WIDTH,h-.098)),
        box((0.,0.,h-.009),(.022,WIDTH,.026),RIB),
        box((0.,0.,.009),(.026,WIDTH,.032),RIB),
    ]
    for sign in (-1,1):
        parts.extend([
            box((0.,sign*(.052+(WIDTH/2-.052)/2),h-.058),
                (.014,WIDTH/2-.052,.079)),
            box((0.,sign*(HALF_W-.004),h/2),(.024,.017,h),RIB),
            box(((-.018 if front else .023),sign*.158,(.0065 if front else .008)),
                (.049,.062,.020),RIB),
        ])
        if front:
            parts.append(box((-.008,sign*.184,-.031),(.040,.028,.056),RIB))
        else:
            parts.append(box((-.018,sign*.145,h-.109),(.023,.031,.223),RIB))
            parts.append(box((-.031,sign*.145,h+.019),(.016,.018,.069),METAL))
    if not front:
        parts.append(box((-.031,0.,h+.058),(.030,.317,.023),HANDLE))
    return tuple(parts)


def slider():
    # Captive over-rim stock profile. The grooves capture the wall rail without a tall grip.
    roof = box((0.,0.,.0115),(BAR_LEN,.027,.0045),SLIDER)
    sides = [box((0.,s*.012,-.002),(BAR_LEN,.0045,.025),SLIDER) for s in (-1,1)]
    ribs = [box((x,s*.0149,-.002),(.0035,.0025,.016),RIB)
            for s in (-1,1) for x in (-.047,-.035,-.023,-.011,.001,.013,.025,.037,.049)]
    return (roof,*sides,*ribs)


def floor():
    # Three main molded regions leave the factory finger access at the free edge.
    length = 2*LENGTH-.032
    width = 2*HALF_W-.030
    close = length-.020
    inner = close-.015
    parts = [
        box((inner/2,0.,0.),(inner,width,.010),PANEL),
        box(((inner+close)/2,0.,0.),(close-inner,width-.004,.010),PANEL),
    ]
    for side in (-1,1):
        shelf = (width-.080)/2
        parts.append(box((close+.010,side*(.035+shelf/2),0.),(.020,shelf,.010),PANEL))
        for y in (.085,.154):
            parts.append(box((close/2,side*y,-.006),(close-.026,.009,.0035),RIB))
    return tuple(parts)
