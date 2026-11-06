"""Geometry of the guarded reset-button station for ``press_button_through_hole.py``: the clear
guard, the control panel and its button, and the rack the three bars sit on. Pure Python (NumPy
only): no Isaac Sim, so the task file stays importable before the simulator starts.

Every part is a :class:`simkit.Part`, so each carries its own MDL material or opacity, its own
friction and restitution, and its own collision approximation. Parts without a ``mass`` are static
colliders bolted down, so the only free bodies are the button and the three bars.

Frame: x forward from the arms, y left, z up, metres; ``top`` is the bench surface.

The layout, and why
-------------------
* **A vertical control panel carries the button.** The red reset button is a 25 mm plunger in a bore
  through a steel panel standing upright at the back of the guard, facing the arms, as a machine's
  reset sits on a vertical escutcheon. It is sprung, so it returns the moment nothing holds it in and
  a press has to be *held* to count.
* **A vertical clear guard stands in front of it.** A polycarbonate screen with sides and a roof --
  all clear, so the top camera sees the button through them -- and one 34 mm square service port on
  the button's axis. The closed sides and roof are what stop the gripper reaching around.
* **The port blocks the gripper.** With the fingers closed, the hand's cross-section grows from
  27.4 mm at 25 mm behind the pad tips to 35.6 mm at 30 mm (measured off the URDF collision hulls by
  ``check_reach.py``), so the port admits the closed fingertips about 25 mm and stops them -- the
  same depth a 30 mm port would, because the hand's width jumps across both. The button face is
  90 mm in, so no gripper pose can touch it. The extra 4 mm over 30 is lateral slop for the 12 mm
  rod, which a gripper cannot centre to better than about a centimetre.
* **The bars sit on a rack at the port's height.** Each bar rests across two posts whose tops put the
  rod's axis on the port's axis, so the job is one horizontal sweep: nothing is lifted up and set
  down, and the arm never passes over the guard.
"""

from __future__ import annotations

import numpy as np

import simkit
from simkit import Part, Piece, Spring

# -- look -------------------------------------------------------------------------------

CLEAR = (0.82, 0.88, 0.90)  # tint of the polycarbonate guard
PANEL_GREY = (0.46, 0.49, 0.52)
RACK_GREY = (0.33, 0.35, 0.38)
BUTTON_RED = (0.88, 0.08, 0.08)
# Glazing: Isaac's Clear_Glass MDL.
GUARD_GLASS = "glass"
FRAME_GREY = (0.20, 0.22, 0.25)  # the dark steel frame that makes the guard read as a barrier
FRAME = 0.012  # square section of a frame member
BEZEL = 0.010  # the border that outlines the service port
MULLION = 0.010  # upright frame bars across the front, so the glazed screen reads as a barrier

# -- guard ------------------------------------------------------------------------------

# Height of a plinth under the whole station.
PLINTH_H = 0.0

WALL = 0.006  # guard glazing thickness
CAGE_X0 = 0.365  # outer face of the guard screen
CAGE_WIDTH = 0.200
CAGE_H = 0.100
CAGE_YC = 0.0  # centred on the midline between the arms

CAGE_Y0 = CAGE_YC - CAGE_WIDTH / 2
CAGE_Y1 = CAGE_YC + CAGE_WIDTH / 2
IN_X0 = CAGE_X0 + WALL  # inner face of the guard screen
IN_Y0 = CAGE_Y0 + WALL
IN_Y1 = CAGE_Y1 - WALL
IN_TOP_REL = CAGE_H - WALL

# Service port: a square opening on the button's axis.
PORT = 0.040
PORT_ZC_REL = 0.045
PORT_Z0_REL = PORT_ZC_REL - PORT / 2
PORT_Z1_REL = PORT_ZC_REL + PORT / 2
PORT_Y0 = CAGE_YC - PORT / 2
PORT_Y1 = CAGE_YC + PORT / 2

# -- control panel and button -----------------------------------------------------------

BUTTON_D = 0.034  # plunger diameter
BUTTON_LEN = 0.018  # how far it stands proud of the panel at rest
BUTTON_DEPTH = 0.130  # its face this far behind the inner face of the guard
BUTTON_MASS = 0.030

BUTTON_FACE_X = IN_X0 + BUTTON_DEPTH
BUTTON_CX = BUTTON_FACE_X + BUTTON_LEN / 2
BUTTON_ZC_REL = PORT_ZC_REL  # the button's axis is the port's

PANEL_T = 0.012  # steel escutcheon, and the back of the guard
PANEL_X0 = BUTTON_FACE_X + BUTTON_LEN  # the panel face the button stands proud of
PANEL_X1 = PANEL_X0 + PANEL_T
CAGE_X1 = PANEL_X1  # the control panel closes the back
IN_X1 = PANEL_X0  # the inner volume ends at the panel face
# The button recesses into a bore through the panel when it is pressed; without it the panel would
# be a solid backstop and the button could not move at all.
BORE = 0.038

PRESSED_M = 0.005  # the button counts as pressed once it has travelled this far
SPRING_TRAVEL = 0.014  # how far it goes before it is flush with the panel
SPRING_STIFFNESS = 220.0  # N/m: a firm industrial push-button
SPRING_DAMPING = 6.0

# -- the three bars, and the rack they sit on --------------------------------------------

RACK_TOP_REL = PLINTH_H + PORT_ZC_REL - 0.008  # post tops: puts the rod's axis on the port's axis
POST = 0.022  # square section of a rack post

ROD_LEN = 0.280  # overall: an 80 mm handle plus a 200 mm shaft
ROD_SIDE = 0.016
ROD_AHEAD = 0.240  # from the grip point (the handle's middle) to the shaft's far end
# The rod is a reach tool: a thin shaft with a chunky handle at the near end for the gripper.
ROD_MASS = 0.055
HANDLE_LEN = 0.080  # the gripped section at the near end
HANDLE_SIDE = 0.028
ROD_Y = 0.200  # close enough to the guard's axis that the swing across is short: a long
# lateral sweep yaws a bar that is gripped 210 mm from its far end, and 26 mm of yaw at the tip was
# enough to slide past the button instead of pressing it
ROD_CX = 0.250  # centre; it lies along x, spanning 0.110 .. 0.390
ROD_POSTS = (0.200, 0.360)  # both beyond the grip point, which sits at the handle

RULE_LEN = 0.140  # far too short to reach the button
RULE_W = 0.030
RULE_T = 0.005
RULE_MASS = 0.020
RULE_Y = 0.300
RULE_CX = 0.200
RULE_POSTS = (0.155, 0.245)

BATON_LEN = 0.360  # too thick for the 40 mm port
BATON_SIDE = 0.055
BATON_MASS = 0.590  # oak, 750 kg/m3
BATON_Y = -0.195
BATON_CX = 0.210
BATON_POSTS = (0.075, 0.355)

FINGER_PER_M = 0.04695 / 0.094  # finger-joint travel per metre of jaw opening (FINGER_OPEN / 94 mm)
GRIP_SQUEEZE = 0.008  # how much narrower than the bar the jaws close. Holding a bar 270 mm from
# its far end needs a hard squeeze: the pads are only ~20 mm long, so the couple that resists the
# bar pitching comes almost entirely from how hard they are pressed together


def finger_for(opening: float) -> float:
    """The ``Waypoint.finger`` target that holds the jaws ``opening`` metres apart.

    Commanding ``FINGER_CLOSED`` on a slender bar drives both jaws hard against it, and the two
    contacts do not start in the same solver step, so the bar is shoved sideways before it is held --
    measured at 8 mm, enough to miss the port. Closing only to just under the bar's width grips it
    firmly and leaves it where it was.
    """
    return -max(opening, 0.0) * FINGER_PER_M


def _panel(name, x, y, z, *, clear=True, material=None) -> Part:
    """A fixed panel spanning the given world ranges."""
    return Part(
        name,
        pos=((x[0] + x[1]) / 2, (y[0] + y[1]) / 2, (z[0] + z[1]) / 2),
        size=(x[1] - x[0], y[1] - y[0], z[1] - z[0]),
        color=CLEAR if clear else PANEL_GREY,
        material=GUARD_GLASS if clear else material,
        friction=simkit.FRICTION["abs_on_steel" if clear else "steel_on_steel"],
        restitution=simkit.RESTITUTION["abs" if clear else "steel"],
        collision="convexHull",
    )


def _frame(name, x, y, z) -> Part:
    """A dark steel frame member. The glazing is nearly invisible by design, so the frame is what
    makes the guard legible as an upright barrier, in the cameras and to a person watching."""
    return Part(
        name,
        pos=((x[0] + x[1]) / 2, (y[0] + y[1]) / 2, (z[0] + z[1]) / 2),
        size=(x[1] - x[0], y[1] - y[0], z[1] - z[0]),
        color=FRAME_GREY,
        material="steel_painted",
        friction=simkit.FRICTION["steel_on_steel"],
        restitution=simkit.RESTITUTION["steel"],
        collision="convexHull",
    )


def frame_objects(top: float) -> list[Part]:
    """Four corner posts, a top and bottom rail across the front, and a bezel outlining the port."""
    top = top + PLINTH_H
    z_all = (top, top + CAGE_H)
    bz = (top + PORT_Z0_REL, top + PORT_Z1_REL)
    roof_z = (top + IN_TOP_REL, top + CAGE_H)
    front_x = (CAGE_X0 - 0.002, IN_X0)
    bez_x = (CAGE_X0 - 0.008, CAGE_X0 + 0.002)  # stands proud of the glazing, so the port outlines
    mul_x = (CAGE_X0 - 0.004, CAGE_X0 + 0.002)
    parts = []
    for i, (px, py) in enumerate(
        ((CAGE_X0, CAGE_Y1), (CAGE_X0, CAGE_Y0), (CAGE_X1, CAGE_Y1), (CAGE_X1, CAGE_Y0))
    ):
        sx = FRAME if px == CAGE_X0 else -FRAME
        sy = -FRAME if py == CAGE_Y1 else FRAME
        parts.append(_frame(f"guard_post{i}", tuple(sorted((px, px + sx))), tuple(sorted((py, py + sy))), z_all))
    parts += [
        _frame("guard_rail_bottom", front_x, (CAGE_Y0, CAGE_Y1), (top, top + FRAME)),
        _frame("guard_rail_top", front_x, (CAGE_Y0, CAGE_Y1), (top + CAGE_H - FRAME, top + CAGE_H)),
        # The roof is glazed, so from above it would read as an open top unless it is framed.
        _frame("roof_edge_front", (CAGE_X0, CAGE_X0 + FRAME), (CAGE_Y0, CAGE_Y1), roof_z),
        _frame("roof_edge_back", (CAGE_X1 - FRAME, CAGE_X1), (CAGE_Y0, CAGE_Y1), roof_z),
        _frame("roof_edge_left", (CAGE_X0, CAGE_X1), (CAGE_Y1 - FRAME, CAGE_Y1), roof_z),
        _frame("roof_edge_right", (CAGE_X0, CAGE_X1), (CAGE_Y0, CAGE_Y0 + FRAME), roof_z),
        _frame("port_bezel_top", bez_x, (PORT_Y0 - BEZEL, PORT_Y1 + BEZEL), (bz[1], bz[1] + BEZEL)),
        _frame("port_bezel_bottom", bez_x, (PORT_Y0 - BEZEL, PORT_Y1 + BEZEL), (bz[0] - BEZEL, bz[0])),
        _frame("port_bezel_left", bez_x, (PORT_Y1, PORT_Y1 + BEZEL), (bz[0] - BEZEL, bz[1] + BEZEL)),
        _frame("port_bezel_right", bez_x, (PORT_Y0 - BEZEL, PORT_Y0), (bz[0] - BEZEL, bz[1] + BEZEL)),
        # Upright mullions between the port and each side: without them the glazed screen is so
        # clear at the top camera's oblique angle that the front reads as an open face.
        _frame("guard_mullion_left", mul_x, (PORT_Y1 + 0.028, PORT_Y1 + 0.028 + MULLION), (top, top + CAGE_H)),
        _frame("guard_mullion_right", mul_x, (PORT_Y0 - 0.028 - MULLION, PORT_Y0 - 0.028), (top, top + CAGE_H)),
    ]
    return parts


def guard_objects(top: float) -> list[Part]:
    """The plinth, the clear guard -- screen in four pieces around the port, two sides and a roof --
    and the steel control panel that closes the back and carries the button."""
    bench = top
    top = top + PLINTH_H  # everything above the plinth is measured from its deck
    z_all = (top, top + CAGE_H)
    z_port = (top + PORT_Z0_REL, top + PORT_Z1_REL)
    z_bore = (top + BUTTON_ZC_REL - BORE / 2, top + BUTTON_ZC_REL + BORE / 2)
    parts = [
        _panel("guard_screen_low", (CAGE_X0, IN_X0), (IN_Y0, IN_Y1), (top, z_port[0])),
        _panel("guard_screen_high", (CAGE_X0, IN_X0), (IN_Y0, IN_Y1), (z_port[1], top + CAGE_H)),
        _panel("guard_screen_left", (CAGE_X0, IN_X0), (PORT_Y1, IN_Y1), z_port),
        _panel("guard_screen_right", (CAGE_X0, IN_X0), (IN_Y0, PORT_Y0), z_port),
        _panel("guard_side_left", (CAGE_X0, CAGE_X1), (IN_Y1, CAGE_Y1), z_all),
        _panel("guard_side_right", (CAGE_X0, CAGE_X1), (CAGE_Y0, IN_Y0), z_all),
        _panel("guard_roof", (CAGE_X0, CAGE_X1), (CAGE_Y0, CAGE_Y1), (top + IN_TOP_REL, top + CAGE_H)),
        # The control panel, in four pieces around the bore the button recesses into.
        _panel("control_panel_low", (PANEL_X0, PANEL_X1), (CAGE_Y0, CAGE_Y1), (top, z_bore[0]),
               clear=False, material="steel"),
        _panel("control_panel_high", (PANEL_X0, PANEL_X1), (CAGE_Y0, CAGE_Y1), (z_bore[1], top + CAGE_H),
               clear=False, material="steel"),
        _panel("control_panel_left", (PANEL_X0, PANEL_X1), (BORE / 2, CAGE_Y1), z_bore,
               clear=False, material="steel"),
        _panel("control_panel_right", (PANEL_X0, PANEL_X1), (CAGE_Y0, -BORE / 2), z_bore,
               clear=False, material="steel"),
    ]
    if PLINTH_H > 0:
        parts.append(
            _panel("plinth", (CAGE_X0, CAGE_X1), (CAGE_Y0, CAGE_Y1), (bench, bench + PLINTH_H),
                   clear=False, material="steel_painted")
        )
    return parts


def button_object(top: float) -> Part:
    """The red reset button: a plunger in a bore through the control panel, on a world-anchored
    prismatic joint whose spring drive returns it. That joint *is* the bore -- the only degree of
    freedom the button has -- so it cannot be knocked sideways or spun."""
    return Part(
        "button",
        pos=(BUTTON_CX, CAGE_YC, top + PLINTH_H + BUTTON_ZC_REL),
        size=(BUTTON_D / 2, BUTTON_LEN, BUTTON_D / 2),
        shape="cylinder",
        axis="X",
        mass=BUTTON_MASS,
        color=BUTTON_RED,
        # An illuminated reset button, as a real one usually is. It also means the button still reads
        # clearly through the glazing at 320 x 176, which the task needs to be observable at all.
        emissive=(0.55, 0.02, 0.02),
        friction=simkit.FRICTION["abs_on_steel"],
        restitution=simkit.RESTITUTION["abs"],
        collision="convexHull",
        spring=Spring(
            axis="X",
            lower=0.0,
            upper=SPRING_TRAVEL,
            stiffness=SPRING_STIFFNESS,
            damping=SPRING_DAMPING,
            target=0.0,
        ),
    )


def _post(name: str, x: float, y: float, top: float) -> Part:
    return Part(
        name,
        pos=(x, y, top + RACK_TOP_REL / 2),
        size=(POST, POST, RACK_TOP_REL),
        color=RACK_GREY,
        material="steel_painted",
        friction=simkit.FRICTION["steel_on_steel"],
        restitution=simkit.RESTITUTION["steel"],
        collision="convexHull",
    )


def _bar(name, length, width, thick, cx, y, mass, color, material, top, pieces=()) -> Part:
    metal = name == "rod"
    return Part(
        name,
        pos=(cx, y, top + RACK_TOP_REL + thick / 2),
        size=(length, width, thick),
        mass=mass,
        color=color,
        material=material,
        friction=simkit.FRICTION["aluminium_on_steel" if metal else "wood_on_steel"],
        restitution=simkit.RESTITUTION["aluminium" if metal else "wood"],
        collision="convexHull",
        pieces=pieces,
    )


def rack_objects(top: float) -> list[Part]:
    """Two posts under each bar, clear both of where the arm grips and of the swept path."""
    posts = []
    for tag, y, xs in (("rod", ROD_Y, ROD_POSTS), ("rule", RULE_Y, RULE_POSTS), ("baton", BATON_Y, BATON_POSTS)):
        for i, x in enumerate(xs):
            posts.append(_post(f"rack_{tag}{i}", x, y, top))
    return posts


def bar_objects(top: float) -> list[Part]:
    """The three bars resting across the rack, each along x."""
    return [
        # Shaft as the root, handle welded on at the near end: one rigid body, and PhysX puts the
        # centre of mass where the bulk is -- next to the jaws rather than out along the shaft.
        _bar("rod", ROD_LEN - HANDLE_LEN, ROD_SIDE, ROD_SIDE, ROD_CX + HANDLE_LEN / 2, ROD_Y, ROD_MASS,
             (0.72, 0.74, 0.76), "aluminium_brushed", top,
             pieces=(
                 Piece(
                     "box",
                     (HANDLE_LEN, HANDLE_SIDE, HANDLE_SIDE),
                     (-(ROD_LEN - HANDLE_LEN) / 2 - HANDLE_LEN / 2 + 0.004, 0.0, 0.0),
                     color=(0.22, 0.24, 0.27),
                     material=None,
                 ),
             )),
        _bar("short_rule", RULE_LEN, RULE_W, RULE_T, RULE_CX, RULE_Y, RULE_MASS,
             (0.86, 0.74, 0.48), "oak", top),
        _bar("thick_baton", BATON_LEN, BATON_SIDE, BATON_SIDE, BATON_CX, BATON_Y, BATON_MASS,
             (0.55, 0.38, 0.22), "oak", top),
    ]


def bar_rest_z(top: float, thick: float) -> float:
    """Height of a bar's axis while it sits on the rack."""
    return top + RACK_TOP_REL + thick / 2


# -- measurements the scorer takes ------------------------------------------------------


def rotation(quat_xyzw) -> np.ndarray:
    x, y, z, w = quat_xyzw
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def bar_ends(state, length: float) -> tuple[np.ndarray, np.ndarray]:
    """(far end, near end) of a bar: the centres of its two end faces, far end (larger x) first."""
    axis = rotation(state.quat_xyzw)[:, 0] * length / 2
    if axis[0] < 0:
        axis = -axis
    return state.pos + axis, state.pos - axis


def bar_tilt_deg(state) -> float:
    """How far a bar's long axis is off horizontal, in degrees."""
    axis = rotation(state.quat_xyzw)[:, 0]
    return float(np.degrees(np.arcsin(np.clip(abs(axis[2]), 0.0, 1.0))))


def button_travel(state) -> float:
    """How far the button has been pushed along +x from rest (m; negative if pulled out)."""
    return float(state.pos[0] - BUTTON_CX)


def deck_z(top: float) -> float:
    """Height of the plinth's deck -- the surface everything in the guard is measured from."""
    return top + PLINTH_H


def port_axis_z(top: float) -> float:
    """Height of the service port's axis, which is also the button's and the rod's."""
    return deck_z(top) + PORT_ZC_REL


def guard_roof_z(top: float) -> float:
    return deck_z(top) + CAGE_H


def inside_guard(point, top: float) -> bool:
    """Whether a point lies in the guarded volume, between the screen and the control panel."""
    deck = top + PLINTH_H
    return bool(
        IN_X0 <= point[0] <= IN_X1
        and IN_Y0 <= point[1] <= IN_Y1
        and deck <= point[2] <= deck + IN_TOP_REL
    )


def rod_grip_x() -> float:
    """x of the grip point -- the handle's middle -- while the tool sits on its rack."""
    return ROD_CX + ROD_LEN / 2 - ROD_AHEAD


def rod_grip_width() -> float:
    """What the jaws actually close on: the handle, not the shaft."""
    return HANDLE_SIDE


def rod_rest_pos(top: float) -> np.ndarray:
    """Where the rod sits on its rack at the start of a trial."""
    return np.array([ROD_CX, ROD_Y, bar_rest_z(top, ROD_SIDE)])


TAKEN_M = 0.030  # a bar counts as taken off the rack once its centre has moved this far


def taken_from_rack(state, top: float, tol: float = TAKEN_M) -> bool:
    """Whether the rod has been taken off its rack.

    The rack holds the rod at the port's height, so "lifted off" cannot be judged by height alone --
    the rod is carried to the port at the very height it started at. Displacement of its centre from
    the rack is the measurable equivalent, and a judge can check it with a ruler.
    """
    return bool(np.linalg.norm(np.asarray(state.pos) - rod_rest_pos(top)) > tol)
