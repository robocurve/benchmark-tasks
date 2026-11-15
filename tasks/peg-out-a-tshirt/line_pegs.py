"""Candidate ordinary reset: spare pegs clipped to the empty clothesline.

Both jaws remain free rigid bodies joined only to each other. The line is the
sole support; no world joint, fixed grasp or fabric attachment is added. Handles
point downward, placing the peg centre of mass below its line contact.
"""
import math
import os
from scene import LINE_X,LINE_Z

BASE_ANGLE=-math.pi/2
OPEN_ANGLE=-.075


def positions():
    if os.environ.get('ROBOCURVE_SHIRT_CENTER_PEG_DIAGNOSTIC')=='1':
        return {'Left':(LINE_X+.0011,0.,LINE_Z-.030),
                'Right':(LINE_X+.0011,.06,LINE_Z-.030)}
    return {'Left':(LINE_X+.0011,.115,LINE_Z-.030),
            'Right':(LINE_X+.0011,-.115,LINE_Z-.030)}


def author(stage):
    from native_cloth import author_peg
    for label,position in positions().items():
        author_peg(stage,'/World/'+label+'Peg',pos=position,base_angle=BASE_ANGLE,
                   open_angle=OPEN_ANGLE,fixed_lower=False,spring=True)
