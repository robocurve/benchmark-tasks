"""Self-service bag-drop scene with a printer-held tag and a side handle."""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from partkit import Slider, part
from lettering import label
from yam_arena.task_file import TABLE_TOP

SCENE = "station"
TOP = TABLE_TOP[SCENE]
# The 450 mm case sits centred between the arms, clear of both READY grippers
# (fingertips at |y| >= .234). The printer lane at y = -.265 keeps the drawn
# strip outside the case footprint.
BAG_XY = (.330, .000)
BAG_YAW = math.pi / 2
KIOSK_XY = (.675, -.265)
INSTRUCTION = (
    "Attach the printed baggage tag securely to the suitcase's long-side "
    "handle, with the barcode facing out. Press the designated ends together, "
    "release the tag and withdraw both arms."
)

NAVY = (.17, .26, .40)
SEAM = (.035, .055, .090)
RIB = (.13, .22, .35)
HANDLE = (.14, .16, .18)
HANDLE_X = 0.0
# The source suitcase's long-side mounting region is centred at y≈-.02;
# the hard shell's bounding-box centre is not its visible mounting centre.
HANDLE_Y = -.020
SILVER = (.58, .64, .67)
SCALE = (.20, .27, .30)
SCALE_DARK = (.08, .12, .15)
WHITE = (.94, .95, .90)
BLACK = (.035, .045, .055)
YELLOW = (.90, .67, .12)
BLUE = (.13, .39, .65)


def box(x, y, z, sx, sy, sz, color, yaw=None, tilt=None):
    result = ((x, y, z), (sx, sy, sz), color)
    return result if yaw is None else result + (yaw, tilt or 0.0)


# At rest the fabric side grip lies close to the case: its centre rises only
# 30 mm above the shell, leaving a fingertip gap. The passive slider lets a
# grip move if contacted; routing uses the existing gap without a required lift.
HANDLE_ARCH = (
    (-.115, .319, .011), (-.103, .322, .011), (-.087, .331, .012), (-.065, .342, .013),
    (-.045, .347, .015), (-.020, .349, .015), (.005, .347, .015), (.025, .342, .013),
    (.047, .331, .012), (.063, .322, .011), (.075, .319, .011),
)
HANDLE_WORLD_X = BAG_XY[0] + math.cos(BAG_YAW) * HANDLE_X - math.sin(BAG_YAW) * HANDLE_Y
HANDLE_WORLD_Y = BAG_XY[1] + math.sin(BAG_YAW) * HANDLE_X + math.cos(BAG_YAW) * HANDLE_Y


def handle_arch_boxes(thickness=.006):
    """Tilted box colliders along the arch, so legs and grip both block the strip."""
    out = []
    for (y0, z0, w0), (y1, z1, w1) in zip(HANDLE_ARCH, HANDLE_ARCH[1:]):
        length = math.hypot(y1 - y0, z1 - z0) + .002
        out.append(box(HANDLE_X, (y0 + y1) / 2, (z0 + z1) / 2, w0 + w1, length, thickness, HANDLE,
                       0.0, math.atan2(z1 - z0, y1 - y0)))
    return tuple(out)


# A moulded hard-shell carry-on provides the ribs, zipper, corners and wheels;
# its simple rectangular collision shell keeps the physical dimensions clear.
scale = part(
    "bag_drop_scale", (*BAG_XY, TOP), [
        box(0, 0, .011, .240, .460, .022, SCALE),
        box(-.115, 0, .024, .010, .460, .007, SILVER),
        box(.115, 0, .024, .010, .460, .007, SILVER),
        box(0, -.223, .024, .240, .010, .007, SILVER),
        box(0, .223, .024, .240, .010, .007, SILVER),
    ], mass=10.0, fixed_to_world=True, yaw=BAG_YAW,
)

bag = part(
    "checked_suitcase", (*BAG_XY, TOP), [
        box(0, .010, .166, .171, .380, .306, NAVY),
        box(HANDLE_X, HANDLE_Y - .085, .310, .032, .027, .016, HANDLE),
        box(HANDLE_X, HANDLE_Y + .085, .310, .032, .027, .016, HANDLE),
    ], mass=8.0, fixed_to_world=True, friction=.15, yaw=BAG_YAW,
    sliders=(Slider("carry_handle", (0.0, 0.0, 0.0),
                    boxes=handle_arch_boxes(),
                    travel=.040, rate=90.0, damping=5.0, mass=.10,
                    axis="Z", sign=1),),
)
bag.meta["visual_usd"] = str(pathlib.Path(__file__).with_name("assets") /
                             "rolling-suitcase-side-up.usda")
# The handle is still a simple rigid collider on a spring slider; cloth
# deformation is not simulated.
bag.meta["visual_boxes"] = (
    box(HANDLE_X, HANDLE_Y - .085, .319, .041, .032, .004, SEAM),
    box(HANDLE_X, HANDLE_Y + .085, .319, .041, .032, .004, SEAM),
)
bag.meta["slider_visual_usd"] = {
    "carry_handle": str(pathlib.Path(__file__).with_name("assets") /
                        "rolling-suitcase-soft-side-handle.usda")
}
bag.meta["invisible_colliders"] = True
bag.meta["friction_combine_mode"] = "min"

# Representative cold-seal contact patches: front of panel09 to back of
# panel00. This is a declared adhesive approximation, not a calibrated product.
# Actual 160–170 um stock is represented by 1 mm collision sheets so PhysX can
# resolve the gripper contacts. No liner, staple or pre-joined ends are present.
# The 430 x 50 mm representative strip leaves the printer printed-side up.
SEGMENTS = 10
SEGMENT_LENGTH = .043
TAG_WIDTH = .050
TAG_COLLISION_THICKNESS = .001
# Hinge drive stiffness and damping, USD units per degree. 2.3e-4 N·m/deg
# (0.013 N·m/rad per 43 mm hinge) gives the 2 g panels the same weight-to-
# stiffness ratio as ~0.36 g of 170 um stock with EI ≈ 1e-4 N·m², so the
# strip droops like paper; much stiffer drives on these light bodies leave
# alternate hinges unconverged and limp.
HINGE_STIFFNESS = float(os.environ.get("BAGTAG_HINGE_K", "2.3e-4"))
HINGE_DAMPING = float(os.environ.get("BAGTAG_HINGE_C", "2e-5"))
# A straight 340 mm internal feed channel retains the trailing stock while
# about 90 mm of the leading end emerges from the printer slot. There is no
# exposed support rod; the free end can droop under its own weight.
# All ten panels exist at reset; none are spawned or teleported on a pull.
TAG_X0 = KIOSK_XY[0] + .165
TAG_Y = KIOSK_XY[1]
TAG_Z = TOP + .240


def barcode_boxes():
    bars = []
    for i, width in enumerate((.0020, .0011, .0027, .0014, .0030, .0010, .0020)):
        for side in (-1, 1):
            bars.append(box(-.014 + i * .0042, 0, side * .00065,
                            width, .033, .0004, BLACK))
    return bars


tag = []
for i in range(SEGMENTS):
    x = TAG_X0 - (i + .5) * SEGMENT_LENGTH
    name = f"tag_panel_{i:02d}"
    hinge = None if i == 0 else (
        f"tag_panel_{i-1:02d}",
        (-SEGMENT_LENGTH / 2, 0, 0),
        (SEGMENT_LENGTH / 2, 0, 0),
        155.0, -155.0, 0.0, "Y",
        .002 if i in (1, 9) else HINGE_STIFFNESS,
        .00004 if i in (1, 9) else HINGE_DAMPING, .6,
    )
    panel = part(
        name, (x, TAG_Y, TAG_Z),
        [box(0, 0, 0, SEGMENT_LENGTH, TAG_WIDTH,
             TAG_COLLISION_THICKNESS, WHITE)] +
        ([box(0,0,(-1 if i==0 else 1)*.00055,.039,.046,.0001,BLUE)] if i in (0,9) else []),
        mass=.002, friction=1.25,
        hinge_attachment=hinge,
        # Presenter rollers release the cut tag under a light ~2 N pull; a
        # 1 mm sheet between closed YAM pads holds only about 4 N by friction.
        breakable_attachment=("self_bag_drop_kiosk", (*KIOSK_XY, TOP),
                              2.0, .4) if i == 0 else None,
    )
    if i in (1, 5):
        panel.meta["visual_boxes"] = tuple(barcode_boxes())
    # Mark the actual mating faces: blue adhesive 09 front, blue receiver 00 back.
    # The marked 0.1 mm surface patch is a collider so native contact points
    # distinguish pressing the adhesive from touching the uncoated outer rim.
    if i in (0, 9):
        face = -1 if i == 0 else 1
    if i == 8:
        panel.meta["visual_boxes"] = (
            box(0, 0, .00065, .022, .030, .0004, YELLOW),)
    tag.append(panel)

# Counter-height self-service bag-drop terminal next to the scale. A front
# display, check-in panel and one tag output slot identify the appliance.
kiosk = part(
    "self_bag_drop_kiosk", (*KIOSK_XY, TOP), [
        # The exterior is solid below and above a 21 mm internal feed gap.
        # The gap and front mouth are hollow collision geometry, not a visual
        # mask over a solid box; the full-size tag can slide out by contact.
        box(0, 0, .105, .350, .145, .210, SILVER),
        box(0, 0, .226, .350, .145, .012, SILVER),
        box(0, 0, .294, .350, .145, .082, WHITE),
        box(0, 0, .347, .350, .145, .080, WHITE),
        box(0, -.069, .242, .350, .007, .030, SILVER),
        box(0, .069, .242, .350, .007, .030, SILVER),
        box(.172, 0, .242, .006, .145, .030, SILVER),
        # Front-facing check-in display and card/receipt scanner.
        box(-.178, 0, .345, .006, .132, .070, SCALE_DARK),
        box(-.182, 0, .345, .002, .118, .058, BLUE),
        box(-.178, -.043, .302, .006, .042, .018, BLACK),
        box(-.182, -.043, .302, .002, .032, .010, YELLOW),
        # One framed outlet with an integral short lip, not projecting rods.
        box(-.178, 0, .257, .007, .090, .008, BLACK),
        box(-.178, 0, .223, .007, .090, .008, BLACK),
        box(-.178, -.050, .240, .007, .010, .034, BLACK),
        box(-.178, .050, .240, .007, .010, .034, BLACK),
        box(-.184, 0, .222, .012, .082, .004, SILVER),
        # Status lamp is visual context, not a task control.
        box(-.178, .060, .296, .003, .015, .010, YELLOW),
    ], mass=8.0, fixed_to_world=True,
)
kiosk.meta["visual_boxes"] = tuple(label("BAG DROP", .010, .015, .389,
                                         .0033, SCALE_DARK))

OBJECTS = [scale, bag, kiosk, *tag]


# Adhesive is restricted to the two declared paper patches; never a gripper.
# No position/orientation is changed when their contact bond is created.
from partkit import on_reset, on_step
import tag_closure
on_reset(tag_closure.reset)
on_step(tag_closure.step)
SCORE_OVER = "end"

def score(objects):
    return tag_closure.score(objects)

def success(objects):
    return score(objects) >= 1.0

def closure_diagnostics():
    return tag_closure.diagnostics()
