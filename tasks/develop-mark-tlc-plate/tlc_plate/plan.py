"""Reference plan touches each physical item through the 14-D YAM action API."""

import numpy as np

from . import config as C
from .motion import Program, rotation

DOWN = np.diag([1., -1., -1.])
LID_DOWN = rotation("X", -.24) @ rotation("Y", -.52) @ rotation("Z", -np.pi / 2) @ DOWN
PLATE_DOWN = rotation("Y", C.PLATE_ANGLE) @ DOWN @ rotation("X", .85)
PENCIL_DOWN = rotation("Z", -1.35) @ DOWN
PENCIL_PITCH = rotation("Y", -.39)
PENCIL_TURN = PENCIL_PITCH @ PENCIL_DOWN
PENCIL_CATCH = C.PENCIL + [.031, 0., 0.]
PENCIL_TIP_OFFSET = C.PENCIL_START @ C.PENCIL_TIP - (PENCIL_CATCH - C.PENCIL)
MARK_ROTATION = PENCIL_PITCH


def lid(p, opening):
    return np.asarray(p) + C.LID_GRIP_OFFSET, LID_DOWN, opening


def plate_grasp(p, opening, shift=(0., 0., 0.)):
    return (np.asarray(p) + C.PLATE_R @ C.PLATE_GRIP_OFFSET + shift,
            PLATE_DOWN, opening)


def pencil_pick(p, opening):
    return p, PENCIL_DOWN, opening


def pencil_tip(tip, opening=.013):
    return np.array(tip) - MARK_ROTATION @ PENCIL_TIP_OFFSET, PENCIL_TURN, opening


def front_point(across, distance=.00035, height=.0382):
    return C.WORK + C.PLATE_R @ [across, height, distance]


def waypoints(mode):
    if mode == "hold":
        return {}
    p = Program()
    p.add(2.4, "Approach the glass lid",
          left=lid(C.LID + [0., 0., .055], .44))
    p.add(11.7, "Watch the wet front approach the target height")
    p.add(.8, "Lower around the lid knob", left=lid(C.LID, .43))
    p.add(.5, "Grip the lid knob", left=lid(C.LID, .23))
    p.add(.8, "Lift the lid clear of the glass rim",
          left=lid(C.LID + [0., 0., .052], .23))
    p.add(1.2, "Move the lid aside and approach the dry plate margin",
          left=lid(C.COVER_PARK + [0., 0., .059], .23),
          right=plate_grasp(C.PLATE_CENTER, .48, (0., 0., .036)))
    p.add(1.15, "Park the lid and reach the top margin",
          left=lid(C.COVER_PARK, .23), right=plate_grasp(C.PLATE_CENTER, .32))
    p.add(.75, "Let the open finger pads settle around the edge")
    p.add(1.35, "Release the lid and grip the dry margin",
          left=lid(C.COVER_PARK, .74), right=plate_grasp(C.PLATE_CENTER, .006))
    p.add(1.45, "Lift the plate clear with its coating untouched",
          left=lid(C.COVER_PARK + [0., 0., .11], .74),
          right=plate_grasp(C.PLATE_CENTER, .006, (0., 0., .110)))
    p.add(1.5, "Bring the upright plate clear of the chamber walls",
          left=pencil_pick(PENCIL_CATCH + [0., 0., .089], .39),
          right=plate_grasp(C.PLATE_CENTER, .006, (-.098, 0., .115)))
    p.add(2.3, "Support the upright plate and reach the pencil",
          left=pencil_pick(PENCIL_CATCH, .36),
          right=plate_grasp(C.WORK, .006))
    gap = .00035 if mode == "demo" else .013
    p.add(1.05, "Grip the pencil", left=pencil_pick(PENCIL_CATCH, .013))
    p.add(1.15, "Lift the pencil clear of its rest",
          left=pencil_pick(PENCIL_CATCH + [0., 0., .087], .013))
    p.add(1.80, "Pitch the graphite point toward the wet front",
          left=pencil_tip(front_point(-.021, .027)))
    p.add(.9, "Lower the graphite to the solvent front",
          left=pencil_tip(front_point(-.021, gap)))
    p.add(1.5, "Mark the wet front while the other arm supports the plate",
          left=pencil_tip(front_point(.003, gap, .0407)))
    p.add(.75, "Lift the pencil off the marked plate",
          left=pencil_tip(front_point(.001, .043)))
    p.add(1.3, "Withdraw the pencil over its side of the bench",
          left=pencil_pick(PENCIL_CATCH + [-.04, .10, .09], .013))
    p.add(1.50, "Align the upright plate above the drying rest",
          right=plate_grasp(C.PLACEMENT, .006, (0., 0., .080)))
    p.add(1.40, "Lower the lower edge into its support",
          right=plate_grasp(C.PLACEMENT, .006, (0., 0., .002)))
    p.add(.85, "Release the plate coating-out into the rest",
          right=plate_grasp(C.PLACEMENT, .49, (0., 0., .002)))
    p.add(1.50, "Clear the finished plate",
          right=plate_grasp(C.PLACEMENT, .49, (-.08, -.105, .065)))
    return p.moves
