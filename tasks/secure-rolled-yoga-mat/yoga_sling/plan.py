"""Cartesian reference for wrapping, loading and releasing both elastic loops."""

import math

import numpy as np
from yam_arena import ik

from . import config as c
from .motion import Program, rotation


def waypoints(mode):
    p = Program()
    if mode == "hold":
        return p.moves
    if mode not in ("demo", "one-loop"):
        raise ValueError(f"Unknown native mode {mode!r}")
    down = rotation("X", math.pi)
    brace = rotation("Z", math.radians(-20.)) @ rotation("Y", math.radians(-15.)) @ down
    across = rotation("Z", -math.pi / 2) @ down
    stock_band = np.array([.2913, -.3470, .8003])
    stock_frame = ik.quat_xyzw_to_matrix(np.array([0., -.2687, -.0018, .9632]))
    stock_grip = stock_band + stock_frame @ [0., 0., c.BAND_UPPER_HEIGHT - .002]
    stock_normal = stock_frame[:, 2]
    press = c.MAT_Z + c.MAT_RADIUS - .008

    def stock(g, offset=0.):
        return stock_grip + offset * stock_normal, stock_frame @ across, g

    def steady(y, z, g=.25):
        return (c.MAT_X - .007, y, z), brace, g

    def band(center, lean=0., g=.05):
        angle = rotation("X", math.radians(lean))
        crown = np.asarray(center) + angle @ np.array([0., -.002, c.BAND_UPPER_HEIGHT - .001])
        return crown, angle @ across, g

    axis_x = c.MAT_X + .004
    axis_z = c.MAT_Z + .006
    p.add(2.8, "approach the rolled mat and right loop",
          left=steady(.060, press + .055), right=stock(.65, .053))
    p.add(1.5, "lower to the stock loop and compress the loose outer turn",
          left=steady(.060, press), right=stock(.65))
    p.add(.7, "grasp the upper webbing and brace the roll", right=stock(.05))
    p.add(1.0, "lift the grasped webbing clear",
          right=(stock_grip + [0., 0., .035], stock_frame @ across, .05))
    p.add(1.2, "tip the loop so its distal arc can lead",
          right=band((c.BAND_X + .121, -.355, axis_z), 16.))
    p.add(1.6, "align the distal arc with the lower edge of the roll",
          right=band((axis_x, -.358, axis_z), 16.))
    p.add(.4, "settle the angled band against the grip",
          right=band((axis_x, -.358, axis_z), 16.))
    p.add(2.1, "slide the distal arc under the compressed edge",
          right=band((axis_x, -.306, axis_z), 16.))
    p.add(1.4, "work the loop onto the edge from below",
          right=band((axis_x, -.268, axis_z), 9.))
    p.add(1.0, "center the loop across the retaining area",
          right=band((axis_x, -c.SEATED_Y, axis_z), 2.))
    p.add(.8, "release the right loop",
          right=band((axis_x, -c.SEATED_Y, axis_z), 2., .75))
    p.add(1.2, "move outward to clear the roll",
          right=band((axis_x - .045, -.30, axis_z + .037), 2., .8),
          left=steady(.060, press + .065, .8))
    p.add(1.90, "separate the wrists vertically before exchanging roles",
          left=steady(.060, press + .108, .90),
          right=band((axis_x - .045, -.30, axis_z + .048), 2., .90))
    p.add(.70, "settle after clearing the mat")
    if mode == "one-loop":
        return p.moves

    left_band = np.array([.2949, .3468, .7981])
    left_frame = ik.quat_xyzw_to_matrix(np.array([0., -.243, .001, .970]))
    left_axis = left_frame[:, 2]
    left_crown = left_band + left_frame @ [0., 0., c.BAND_UPPER_HEIGHT - .002]
    left_axis_x = axis_x + .013
    left_axis_z = axis_z - .009
    opposite_brace = rotation("Z", math.radians(20.)) @ rotation("Y", math.radians(-15.)) @ down

    def left_stock(g, distance=0.):
        return left_crown + distance * left_axis, left_frame @ across, g

    def right_steady(lift):
        return np.array([c.MAT_X - .007, -.060, press + lift]), opposite_brace, .25

    p.add(2.9, "exchange roles at the other end",
          left=left_stock(.65, .112), right=right_steady(.110))
    p.add(.6, "settle clear above the second loop")
    p.add(2.0, "compress the outer turn and reach the left webbing",
          left=left_stock(.65), right=right_steady(0.))
    p.add(.75, "grasp the left loop while the right hand compresses",
          left=left_stock(.05))
    p.add(1.05, "raise the second loop before turning",
          left=left_stock(.05, .038))
    p.add(1.2, "turn the lower arc toward the other end",
          left=band((c.BAND_X + .121, .355, left_axis_z), -16.))
    p.add(1.6, "square the second loop to the compressed cross section",
          left=band((left_axis_x, .358, left_axis_z), -16.))
    p.add(.4, "settle the second loop before threading",
          left=band((left_axis_x, .358, left_axis_z), -16.))
    p.add(2.1, "feed the lower arc over the left end",
          left=band((left_axis_x, .306, left_axis_z), -16.))
    p.add(1.4, "work the second loop fully over the end",
          left=band((left_axis_x, .268, left_axis_z), -9.))
    p.add(1.0, "bring the second loop inward",
          left=band((left_axis_x, c.SEATED_Y, left_axis_z), -2.))
    p.add(.8, "release the left loop",
          left=band((left_axis_x, c.SEATED_Y, left_axis_z), -2., .75))
    p.add(1.2, "let the mat bear against both loops",
          left=band((axis_x - .045, .300, axis_z + .037), -2., .8),
          right=right_steady(.065))
    p.add(1.7, "allow both bands to settle around the roll")
    crowns = {
        "left": np.array([.3674, .2961, .868]),
        "right": np.array([.3889, -.2740, .873]),
    }
    wrist = {
        "left": ik.quat_xyzw_to_matrix(np.array([-.0638, -.2462, -.0856, .9633])) @ across,
        "right": ik.quat_xyzw_to_matrix(np.array([.0007, -.0689, .0028, .9976])) @ across,
    }

    def lift_grip(side, g, dx=0., dy=0., dz=0.):
        return crowns[side] + [dx, dy, dz], wrist[side], g

    p.add(1.7, "stop above both elastic loops before regrasping",
          left=lift_grip("left", .65, dx=-.030, dz=.074),
          right=lift_grip("right", .65, dx=-.045, dz=.055))
    p.add(.6, "settle both open jaws over the loops")
    p.add(1.55, "approach the outer crowns of both bands",
          left=lift_grip("left", .65), right=lift_grip("right", .65))
    p.add(.80, "pinch the outer webbing of both bands",
          left=lift_grip("left", 0.), right=lift_grip("right", 0.))
    p.add(1.55, "take up the elastic bands and slide them toward the center",
          left=lift_grip("left", 0., dy=-.040, dz=.009),
          right=lift_grip("right", 0., dy=.029, dz=.011))
    p.add(1.75, "draw both bands snug about the roll",
          left=lift_grip("left", 0., dy=-.049, dz=.094),
          right=lift_grip("right", 0., dy=.029, dz=.100))
    p.add(1.25, "align the two band loops under tension")
    p.add(1.6, "lower the tensioned sling",
          left=lift_grip("left", 0., dy=-.049, dz=.039),
          right=lift_grip("right", 0., dy=.029, dz=.042))
    p.add(.55, "allow the mat and both bands to settle")
    p.add(.95, "open the jaws to regrasp the seated loops",
          left=lift_grip("left", .80, dy=-.049, dz=.039),
          right=lift_grip("right", .80, dy=.029, dz=.042))
    p.add(1.1, "clear the mat for the final lift grip",
          left=lift_grip("left", .9, dy=-.049, dz=.081),
          right=lift_grip("right", .9, dy=.029, dz=.078))
    p.add(1.65, "settle the centered bands before regrasping")
    accepted_crowns = {
        "left": np.array([.441861, .238967, .869305]),
        "right": np.array([.381521, -.210943, .884681]),
    }
    accepted_wrists = {
        "left": ik.quat_xyzw_to_matrix(np.array([.006822, .137145, .021889, -.990286])) @ across,
        "right": ik.quat_xyzw_to_matrix(np.array([-.000831, -.042245, -.019939, .998908])) @ across,
    }

    def secured_band(side, opening, dz=0., dx=0., tilt=0.):
        orientation = rotation("Y", math.radians(tilt)) @ accepted_wrists[side]
        return accepted_crowns[side] + [dx, 0., dz], orientation, opening

    p.add(1.4, "align above exposed webbing on both seated bands",
          left=secured_band("left", .68, .061, -.025), right=secured_band("right", .68, .058))
    p.add(1.55, "approach the outer webbing clear of the foam",
          left=secured_band("left", .68), right=secured_band("right", .68))
    p.add(.85, "grasp both seated bands firmly",
          left=secured_band("left", .0), right=secured_band("right", .0))
    p.add(2.1, "lift the mat by the attached sling",
          left=secured_band("left", .0, .105, -.053),
          right=secured_band("right", .0, .092, -.036, -12.))
    p.add(1.2, "support the roll by the two elastic loops")
    p.add(1.8, "set the load back down and leave the bands seated",
          left=secured_band("left", .0, .040, -.053),
          right=secured_band("right", .0, .038, -.036))
    p.add(.65, "let the bench take the load before opening the fingers")
    p.add(.85, "release the carrier after putting the mat down",
          left=secured_band("left", .85, .040, -.053),
          right=secured_band("right", .85, .038, -.036))
    p.add(1.35, "withdraw both wrists vertically",
          left=secured_band("left", .9, .086, -.053),
          right=secured_band("right", .9, .080, -.036))
    return p.moves
