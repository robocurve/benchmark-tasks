"""On the station table, in front of the right arm: a light grey cabinet, 17 cm deep, 28 cm wide and 20 cm tall, on a
dark grey plinth 9 cm high. Its open front is closed by a dark teal panel, hinged along its right edge, with a black
vertical pull bar. A spring plunger on the cabinet's left wall locks the panel: its steel pin passes through a grey
strike lug on the panel's free edge, and its spring pushes the pin back in whenever its red knob, 6 cm to the left
of the cabinet, is let go. Pulling the knob about 12 mm to the left frees the panel. Nothing else is on the table.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
           --instruction "<INSTRUCTION, below>" --scorer task_score --max-steps 600 ...  (in full in README.md)
Demo:    python demo.py --out runs/demo
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from mechanism import Joint, Link, Shape, mechanism, on_reset  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"  # the panel's angle when the run ends counts
SUCCESS_HOLD_S = 1.0
INSTRUCTION = ("Open the teal panel on the grey cabinet so that it stands open 37 degrees or more. A spring "
               "plunger locks it: pulling its red knob draws the pin back.")
TOP = TABLE_TOP[SCENE]
ORIGIN = (0.0, 0.0, TOP)  # the mechanism's frame: on the table top, midway between the arm bases; x forward, y left

# sRGB colours
LIGHT_GREY = (0.80, 0.80, 0.78)
DARK_GREY = (0.27, 0.28, 0.30)
TEAL = (0.05, 0.36, 0.40)
BLACK = (0.06, 0.06, 0.07)
STEEL = (0.62, 0.63, 0.66)
PIN_STEEL = (0.75, 0.76, 0.78)
RED = (0.80, 0.07, 0.05)
MATERIALS = {"wood": (0.6, 0.5), "painted": (0.5, 0.4), "steel": (0.3, 0.25), "grip": (1.2, 1.0)}
TIGHT = 0.0005  # contact offset of the pin and the strike (m), below their 0.6 mm clearance


def box(center_cm, size_mm, color, material, offset=0.002) -> Shape:
    """A box from its centre (cm) and edge lengths (mm), in the mechanism's frame."""
    return Shape("box", tuple(c / 100 for c in center_cm), tuple(s / 1000 for s in size_mm), color,
                 material=material, contact_offset=offset)


def rod(kind, center_cm, radius_mm, length_mm, axis, color, material, offset=0.002) -> Shape:
    return Shape(kind, tuple(c / 100 for c in center_cm), (radius_mm / 1000, length_mm / 1000), color, axis=axis,
                 material=material, contact_offset=offset)


# -- the cabinet (fixed): plinth, plywood shell open at the front, the plunger's bracket and guide housing ----------
CABINET_SHAPES = (
    box((49.25, -15.0, 4.5), (160, 280, 90), DARK_GREY, "wood"),            # plinth
    box((56.75, -15.0, 19.0), (10, 260, 200), LIGHT_GREY, "wood"),          # back
    box((48.5, -27.5, 19.0), (165, 10, 200), LIGHT_GREY, "wood"),           # right wall, on the hinge side
    box((48.5, -2.0, 14.9), (165, 20, 118), LIGHT_GREY, "wood"),            # left wall (20 mm), below the recess
    box((48.5, -2.0, 28.3), (165, 20, 14), LIGHT_GREY, "wood"),             # left wall, above it
    box((50.75, -2.0, 24.2), (120, 20, 68), LIGHT_GREY, "wood"),            # left wall, behind it
    box((42.5, -0.5, 24.2), (45, 10, 68), LIGHT_GREY, "wood"),              # left wall's outer skin over it
    box((48.5, -15.0, 28.5), (165, 240, 10), LIGHT_GREY, "wood"),           # top
    box((48.5, -15.0, 9.5), (165, 240, 10), LIGHT_GREY, "wood"),            # bottom
    box((40.75, 0.9, 25.0), (20, 28, 22), STEEL, "steel"),                  # bracket on the left wall
    box((39.25, 1.4, 25.0), (20, 22, 16), STEEL, "steel"),                  # bracket arm holding the housing
    rod("cylinder", (37.85, 1.4, 25.0), 7.5, 40, "Y", STEEL, "steel"),      # guide housing, 15 mm, 4 cm long
)

# -- the panel (on the hinge): aluminium sheet, hinge leaves, pull bar on standoffs, strike lug on its standoff ------
# The strike lug is 20 x 28 mm and 8 mm thick, its 7.2 mm square bore centred on the pin (x 37.85, z 25).
PANEL_SHAPES = (
    box((39.9, -15.0, 19.0), (3, 260, 200), TEAL, "painted"),               # the panel: x 39.75..40.05
    box((39.75, -28.0, 11.5), (10, 24, 36), BLACK, "steel"),                # hinge leaves
    box((39.75, -28.0, 26.5), (10, 24, 36), BLACK, "steel"),
    rod("capsule", (35.75, -14.5, 16.0), 5, 60, "Z", BLACK, "grip"),        # pull bar, 10 mm, z 12.5..19.5
    box((37.75, -14.5, 13.0), (40, 10, 10), BLACK, "steel"),                # its standoffs
    box((37.75, -14.5, 19.0), (40, 10, 10), BLACK, "steel"),
    box((37.17, -2.0, 25.0), (6.4, 8, 28), STEEL, "steel", TIGHT),          # strike lug: in front of the bore
    box((38.53, -2.0, 25.0), (6.4, 8, 28), STEEL, "steel", TIGHT),          # behind it (the side the pin holds)
    box((37.85, -2.0, 25.88), (7.2, 8, 10.4), STEEL, "steel", TIGHT),       # above it
    box((37.85, -2.0, 24.12), (7.2, 8, 10.4), STEEL, "steel", TIGHT),       # below it
    box((39.25, -2.6, 25.0), (16, 20, 28), STEEL, "steel", TIGHT),          # the lug's standoff on the panel
)

# -- the plunger (on its slide): 6 mm pin and pull rod in one piece, red knob --------------------------------------
PLUNGER_SHAPES = (
    rod("cylinder", (37.85, 1.45, 25.0), 3, 85, "Y", PIN_STEEL, "steel", TIGHT),  # y -2.8..5.7: tip 4 mm past the lug
    rod("cylinder", (37.85, 6.0, 25.0), 11, 10, "Y", RED, "grip", 0.005),         # knob, 22 mm across, 10 mm thick
)

HINGE_AXIS = (0.3975, -0.28)   # the panel's vertical hinge axis (x, y), m, mechanism frame
PIN_AXIS = (0.3785, 0.25)      # the pin's axis (x, z): along y
PIN_RADIUS = 0.003
PIN_SPAN = (-0.028, 0.057)     # the pin and rod along y at rest (m); pulling the knob moves both by the travel
PANEL_MAX = math.radians(65.9)
PLUNGER_STROKE = 0.014

CABINET = mechanism(
    "cabinet",
    pos=ORIGIN,
    base=Link("cabinet_body", 8.0, CABINET_SHAPES),
    links=(Link("panel", 0.4922, PANEL_SHAPES), Link("plunger", 0.022, PLUNGER_SHAPES)),
    joints=(
        # Opens the free edge toward the arms. Friction and damping as in the hinge's nominal values.
        Joint("panel_hinge", "revolute", "panel", (0.3975, -0.28, 0.19), "Z", 0.0, PANEL_MAX,
              damping=0.004, friction=0.025),
        # Pulled toward +y (the robot's left) against a 150 N/m spring preloaded 2 mm: 0.3 N at rest, 2.4 N at 14 mm.
        Joint("plunger_slide", "prismatic", "plunger", (0.3785, -0.005, 0.25), "Y", 0.0, PLUNGER_STROKE,
              stiffness=150.0, damping=1.0, target=-0.002),
    ),
    materials=MATERIALS,
    color=LIGHT_GREY,
)
OBJECTS = [CABINET]


def world(point_m) -> np.ndarray:
    """A point in the mechanism's frame, in the world frame."""
    return np.asarray(point_m, float) + np.asarray(ORIGIN)


KNOB = world((0.3785, 0.060, 0.25))      # the knob's centre at rest
HANDLE = world((0.3575, -0.145, 0.16))   # the pull bar's centre with the panel closed


def handle_at(angle: float) -> np.ndarray:
    """The pull bar's centre (world) with the panel open ``angle`` rad."""
    hx, hy = HINGE_AXIS
    c, s = math.cos(angle), math.sin(angle)
    dx, dy = HANDLE[0] - hx, HANDLE[1] - hy
    return np.array([hx + c * dx - s * dy, hy + s * dx + c * dy, HANDLE[2]])


# -- scoring ---------------------------------------------------------------------------------------------------------
FULL_DEG, HALF_DEG, QUARTER_DEG = 37.0, 7.0, 2.0
FORCED_MM = 1.0           # the pin pushed more than this into the strike or its standoff: the latch was forced
INTO_CABINET_DEG = -0.25  # the panel pushed past closed by more than this (about 1 mm at its free edge): forced

# Each part of the strike in the panel's frame (x0, x1, y0, y1, z0, z1), m: the four sides of the bore and the standoff.
STRIKE_PARTS = []
for _shape in PANEL_SHAPES[-5:]:
    _low, _high = _shape.bounds()
    STRIKE_PARTS.append((_low[0], _high[0], _low[1], _high[1], _low[2], _high[2]))


def pin_overlap_mm(angle: float, travel: float) -> float:
    """How deep the pin is inside the strike lug or its standoff (mm, 0 when clear), from the panel's angle (rad)
    and the plunger's travel (m). The pin, taken as a 6 mm square rod, is moved into the panel's frame; each overlap
    is the least of its three axis-aligned depths, the smallest shift that would part them."""
    hx, hy = HINGE_AXIS
    px, pz = PIN_AXIS
    c, s = math.cos(-angle), math.sin(-angle)

    def to_panel(x, y):
        return hx + c * (x - hx) - s * (y - hy), hy + s * (x - hx) + c * (y - hy)

    tip, end = PIN_SPAN[0] + travel, PIN_SPAN[1] + travel
    _, tip_y = to_panel(px, tip)
    _, end_y = to_panel(px, end)
    worst = 0.0
    for x0, x1, y0, y1, z0, z1 in STRIKE_PARTS:
        # Where the pin's axis crosses the middle of this part, along the panel's y.
        y_mid = (y0 + y1) / 2
        y_world = hy + (y_mid - hy + math.sin(angle) * (px - hx)) / math.cos(angle)
        x_pin, _ = to_panel(px, y_world)
        dx = min(x_pin + PIN_RADIUS, x1) - max(x_pin - PIN_RADIUS, x0)
        dy = min(end_y, y1) - max(tip_y, y0)
        dz = min(pz + PIN_RADIUS, z1) - max(pz - PIN_RADIUS, z0)
        if dx > 0 and dy > 0 and dz > 0:
            worst = max(worst, min(dx, dy, dz))
    return 1000.0 * worst


_EPISODE = {"forced": False, "worst_overlap_mm": 0.0, "max_angle_deg": 0.0}


@on_reset
def _new_episode() -> None:
    _EPISODE.update(forced=False, worst_overlap_mm=0.0, max_angle_deg=0.0)


def panel_angle_deg(objects) -> float:
    return math.degrees(objects["cabinet"].joints["panel_hinge"])


def plunger_travel_mm(objects) -> float:
    return 1000.0 * objects["cabinet"].joints["plunger_slide"]


def _update(objects) -> float:
    """Track the episode (forcing, widest opening) and return the panel's angle now (deg)."""
    joints = objects["cabinet"].joints
    angle, travel = joints["panel_hinge"], joints["plunger_slide"]
    overlap = pin_overlap_mm(angle, travel)
    degrees = math.degrees(angle)
    _EPISODE["worst_overlap_mm"] = max(_EPISODE["worst_overlap_mm"], overlap)
    _EPISODE["max_angle_deg"] = max(_EPISODE["max_angle_deg"], degrees)
    if overlap > FORCED_MM or degrees < INTO_CABINET_DEG:
        _EPISODE["forced"] = True
    return degrees


def episode() -> dict:
    """This episode so far: whether the latch was forced, the deepest overlap (mm) and the widest opening (deg)."""
    return dict(_EPISODE)


def score(objects) -> float:
    """By the panel's angle at the hinge: 1.0 from 37 degrees, 0.5 from 7, 0.25 from 2, else 0; 0 if the latch was
    forced at any step (the pin more than 1 mm inside the strike, or the panel pushed into the cabinet)."""
    degrees = _update(objects)
    if _EPISODE["forced"]:
        return 0.0
    if degrees >= FULL_DEG:
        return 1.0
    if degrees >= HALF_DEG:
        return 0.5
    return 0.25 if degrees >= QUARTER_DEG else 0.0


def success(objects) -> bool:
    """Open 37 degrees or more, and the latch never forced (held for SUCCESS_HOLD_S)."""
    return _update(objects) >= FULL_DEG and not _EPISODE["forced"]
