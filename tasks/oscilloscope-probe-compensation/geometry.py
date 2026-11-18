"""Full-size TDS2024B front panel and P2220 probe geometry for the first native experiment.

The case envelope is from Tektronix. Tool, port and slot dimensions and turning
resistance are prototype estimates until checked against the stock hardware.
"""

import math

import numpy as np

from .massprops import body, box, description
from .waveform import waveform_samples

TABLE_Z = 0.7475
START_ANGLE = -0.8
TRIMMER_CENTER = np.array([0.418, 0.075, 0.816])
TOOL_REST = np.array([0.287, 0.140, TABLE_Z + 0.005])
TOOL_GRIP = np.array([0.0, 0.0, 0.025])
TOOL_TIP = np.array([0.0, 0.0, -0.036])
TOOL_ROLL = (-0.5, 0.5, 0.5, 0.5)  # Rx(-90°) @ Rz(90°), xyzw
PANEL_X = 0.485
SCREEN = (0.227, 0.839, 0.116, 0.087)  # y, z, width, height


def trace_points(samples):
    y, z, width, height = SCREEN
    time, volts = np.asarray(samples)
    u = y + 0.48 * width - time * (0.96 * width / (time[-1] - time[0]))
    v = z + (volts - 2.5) * (height / 16.0)
    dy, dz = np.gradient(u), np.gradient(v)
    length = np.hypot(dy, dz)
    offset_y, offset_z = -0.00105 * dz / length, 0.00105 * dy / length
    return [[PANEL_X - 0.0045, float(yy + sign * oy), float(zz + sign * oz)]
            for yy, zz, oy, oz in zip(u, v, offset_y, offset_z) for sign in (-1, 1)]


def display_trace():
    points = trace_points(waveform_samples(START_ANGLE))
    return dict(type="mesh", mesh=dict(points=points,
                faces=[[2*k, 2*k+1, 2*k+3, 2*k+2] for k in range(len(points)//2 - 1)]),
                collision=False, color=[1.0, 0.82, 0.06, 1.0], emissive=[1.0, 0.82, 0.06])


def cylinder(radius, length, position, color, **kwargs):
    return dict(type="cylinder", size=[radius, length / 2], position=list(position),
                color=list(color), **kwargs)


def cable(points, radius, color):
    return [dict(type="capsule", size=[radius], fromto=[*a, *b],
                 color=list(color), collision=False)
            for a, b in zip(points, points[1:])]


def geometry():
    case = (0.48, 0.52, 0.55, 1)
    dark = (0.075, 0.09, 0.10, 1)
    silver = (0.58, 0.63, 0.65, 1)
    knobs = (0.72, 0.74, 0.72, 1)
    tip_color = (0.18, 0.19, 0.21, 1)
    tool_color = (0.27, 0.40, 0.56, 1)
    hole_x, hole_y, face_z = TRIMMER_CENTER
    screen_y, screen_z, screen_width, screen_height = SCREEN
    about_y = [math.sqrt(0.5), 0, math.sqrt(0.5), 0]
    panel_z = TABLE_Z + 0.079
    housing_z = face_z - 0.0053

    # Four walls form a physical access port amid the original BNC housing.
    housing = [
        box((0.009, 0.0085, 0.0055), (hole_x - 0.0113, hole_y, housing_z), dark),
        box((0.027, 0.0085, 0.0055), (hole_x + 0.0293, hole_y, housing_z), dark),
        *[box((0.0023, 0.0031, 0.0055), (hole_x, hole_y + side * 0.0054, housing_z), dark)
          for side in (-1, 1)],
    ]
    panel_shapes = [
        box((0.0621, 0.16315, 0.079), (PANEL_X + 0.0621, 0.154, panel_z), case),
        box((0.0017, screen_width / 2 + 0.006, screen_height / 2 + 0.006),
            (PANEL_X - 0.0017, screen_y, screen_z), dark),
        cylinder(0.0072, 0.011, (PANEL_X - 0.0055, hole_y, face_z - 0.006), silver,
                 orientation=about_y),
        *[cylinder(0.008, 0.006, (PANEL_X - 0.003, y, z), knobs, orientation=about_y)
          for y in (0.021, 0.054, 0.087, 0.105) for z in (0.852, 0.875)],
        *[cylinder(0.0055, 0.005, (PANEL_X - 0.0025, y, face_z - 0.006), silver,
                   orientation=about_y) for y in (0.025, 0.048, 0.101)],
        *housing,
    ]
    panel_shapes.extend(
        box((0.0001, 0.00025, screen_height / 2),
            (PANEL_X - 0.0035, y, screen_z), (0.20, 0.25, 0.25, 1), collision=False)
        for y in np.linspace(screen_y - screen_width / 2, screen_y + screen_width / 2, 11)
    )
    # The attached lead is dressed to the side, away from the radial access port.
    # It is visual geometry for the fixed electrical boundary, not a loose cable.
    cord_color = (0.04, 0.044, 0.047, 1)
    coax = [(0.389, .075, .810), (.364, .073, .808), (.343, .054, .785),
            (.326, .025, .751), (.330, -.012, .750), (.372, -.039, .750),
            (.428, -.079, .750), (.530, -.111, .750), (.578, -.120, .750),
            (.609, -.142, .750), (.600, -.185, .750), (.512, -.189, .750),
            (.460, -.178, .766), (.445, -.143, .789), (.430, -.115, .812)]
    panel_shapes.extend(cable(coax, .0017, cord_color))
    panel_shapes.extend([
        dict(type="capsule", size=[.0047], fromto=[*coax[-1], .465, -.016, .834],
             color=list(dark), collision=False),
        cylinder(.003, .006, (.482, .004, .836), silver,
                 orientation=about_y, collision=False),
        cylinder(.003, .006, (.482, .004, .818), silver,
                 orientation=about_y, collision=False),
        *cable([(.465, -.016, .834), (.475, .004, .836), (.482, .004, .836)],
               .0019, tip_color),
        *cable([(.444, -.091, .821), (.454, -.066, .804), (.476, .004, .818)],
               .0010, cord_color),
    ])
    panel_shapes.extend(
        box((0.0001, screen_width / 2, 0.00025),
            (PANEL_X - 0.0035, screen_y, z), (0.20, 0.25, 0.25, 1), collision=False)
        for z in np.linspace(screen_z - screen_height / 2, screen_z + screen_height / 2, 9)
    )
    panel_shapes.append(display_trace())
    rotor = body("trimmer", [
        cylinder(0.0019, 0.0036, (0, 0, -0.0030), silver, contact_offset=0.00005),
        # Jaw halves leave a 0.70 mm slot, 1.2 mm deep. The only turning input is contact.
        *[box((0.00135, 0.0006, 0.0006), (0, side * 0.00095, -0.0006), silver,
              contact_offset=0.00005) for side in (-1, 1)],
    ], mass=0.001, position=TRIMMER_CENTER)
    scope = description("Scope", [body("instrument", panel_shapes, mass=2.0), rotor],
        articulation=True, fixed_base="instrument", self_collisions=False,
        solver_position_iterations=32, solver_velocity_iterations=8,
        joints=[dict(name="compensation", type="revolute", axis="Z", parent="instrument",
            child="trimmer", position_parent=TRIMMER_CENTER.tolist(), lower=-1.35, upper=1.35,
            static_friction_effort=0.003, dynamic_friction_effort=0.002)])
    tool = description("AdjustmentTool", [body("driver", [
        box((0.005, 0.0048, 0.017), (0, 0, 0.025), tool_color, friction=(0.92, 0.7)),
        cylinder(0.00125, 0.038, (0, 0, -0.0105), tip_color, contact_offset=0.00005),
        box((0.001, 0.00025, 0.00325), (0, 0, -0.03275), tip_color,
            contact_offset=0.00005),
    ], mass=0.0045)], solver_position_iterations=32, solver_velocity_iterations=8)
    return {"scope": scope, "tool": tool}
