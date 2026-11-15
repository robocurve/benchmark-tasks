"""Explicit fixed fixture layouts; no runtime object motion.

The alternative is an unvalidated reach diagnostic. Its base stays on the
table, while a10mm shorter upright lowers the wheel and axle arm. The physical
wheel, wrench, contacts and materials are otherwise unchanged.
"""
import numpy as np

REFERENCE = 'reference'
RELOCATED = 'forward10-right50-lower10'
RIGHT_WORKSPACE = 'forward10-right200-lower5'
LAYOUTS = (REFERENCE, RELOCATED, RIGHT_WORKSPACE)


def offset(layout=REFERENCE):
    if layout not in LAYOUTS:
        raise ValueError('Unknown fixed wheel fixture layout')
    return np.array({REFERENCE: (0., 0., 0.),
                     RELOCATED: (.010, -.050, -.010),
                     RIGHT_WORKSPACE: (.010, -.200, -.005)}[layout])


def axle_position(layout=REFERENCE):
    return np.array((.35, 0., 1.005))+offset(layout)


def pointer_position(layout=REFERENCE):
    return np.array((.373, 0., .785))+offset(layout)


def stand_parts(layout=REFERENCE):
    d = offset(layout)
    horizontal = np.array((d[0], d[1], 0.))
    return [
        ('base', np.array((.48, .02, .7575))+horizontal, (.14, .25, .02)),
        ('upright', np.array((.48, 0., .88625))+horizontal+
         np.array((0., 0., d[2]/2)), (.025, .025, .2375+d[2])),
        ('axle_arm', np.array((.4475, 0., 1.005))+d, (.065, .025, .015)),
    ]
