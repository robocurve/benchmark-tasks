"""Repair an unpowered assembly guide. The marked test part and fixture have
white arrows. A part that seats reaches the lower green depth mark; a correctly
blocked part stops at the upper white mark. The three grey guides are removable.
"""

from dataclasses import dataclass
from math import pi

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]
CONTROL_HZ = 20
LIMIT = 3600
FIX = (0.335, -0.195)
CRADLE = (0.225, -0.118)
REJECT = (0.226, -0.299)
GOOD = (0.350, -0.325)
BAD = (0.465, -0.210)
FLOOR = 0.010
INSERT_BASE = 0.003
SEAT = FLOOR + INSERT_BASE
BLOCK = 0.012
HEIGHT = 0.070
GUIDES = ("original_guide", "good_spare", "blocked_spare")
INSTRUCTION = (
    "Show that the test part seats in the installed guide with the arrows opposed. "
    "Put the faulty guide in the red tray. Install a spare that lets the part seat with the "
    "arrows aligned and stops it short with the arrows opposed. Show both results, letting go "
    "each time. Leave that guide installed and return the test part to its cradle."
)
DARK = (0.20, 0.23, 0.28)
GREY = (0.53, 0.58, 0.64)
STEEL = (0.72, 0.77, 0.81)
BLUE = (0.17, 0.43, 0.82)
RED = (0.70, 0.16, 0.14)
WHITE = (0.96, 0.96, 0.92)
GREEN = (0.25, 0.82, 0.43)


@dataclass(frozen=True)
class Solid:
    name: str
    pos: tuple
    size: tuple
    color: tuple
    collision: bool = True
    yaw: float = 0.0
    vertices: tuple = ()


PARTS = {}
OBJECTS = []


def body(name, xy, z, solids, *, mass=None, yaw=0.0):
    PARTS[name] = tuple(solids)
    OBJECTS.append(Box(name=name, pos=(*xy, TOP + z), size=solids[0].size,
                       color=solids[0].color, mass=mass, yaw=yaw))


def arrow(x, y, z, span=1.0):
    return [
        Solid("arrow_stem", (x - .004 * span, y, z), (.018 * span, .003, .0006), WHITE, False),
        Solid("arrow_left", (x + .006 * span, y + .004 * span, z),
              (.012 * span, .003, .0006), WHITE, False, -pi / 4),
        Solid("arrow_right", (x + .006 * span, y - .004 * span, z),
              (.012 * span, .003, .0006), WHITE, False, pi / 4),
    ]


def lead_in(name, axis, side, inside, mouth, outside, span, low, high, color):
    """A solid chamfer extruded along the other horizontal axis."""
    contour = ((inside, low), (outside, low), (outside, high), (mouth, high))
    vertices = tuple(
        (side * distance, lateral, z) if axis == "x" else (lateral, side * distance, z)
        for lateral in (-span / 2, span / 2)
        for distance, z in contour
    )
    return Solid(name, (0., 0., 0.), (1., 1., 1.), color, vertices=vertices)


body("fixture", FIX, 0, [
    Solid("base", (0, 0, .005), (.092, .084, .010), DARK),
    *[
        Solid(f"nest_rail_{axis}_{index}",
              (side * .039, 0, .012) if axis == "x" else (0, side * .035, .012),
              (.012, .058, .004) if axis == "x" else (.090, .012, .004), DARK)
        for axis in ("x", "y") for index, side in enumerate((-1, 1))
    ],
    *[
        lead_in(f"nest_chamfer_{axis}_{index}", axis, side,
                *(.033, .040, .045, .072) if axis == "x" else (.029, .036, .041, .090),
                .014, .022, DARK)
        for axis in ("x", "y") for index, side in enumerate((-1, 1))
    ],
    Solid("arrow_plate", (-.042, .049, .030), (.036, .025, .004), DARK),
    Solid("arrow_post", (-.040, .034, .018), (.008, .018, .026), DARK),
    *arrow(-.042, .049, .0324, .8),
    Solid("scale_post", (-.018, .042, .052), (.008, .009, .094), DARK),
    Solid("seated_mark", (-.0222, .042, SEAT + HEIGHT), (.001, .011, .002), GREEN, False),
    Solid("blocked_mark", (-.0222, .042, SEAT + HEIGHT + BLOCK), (.001, .011, .002), WHITE, False),
])
body("cradle", CRADLE, 0, [
    Solid("base", (0, 0, .003), (.067, .064, .006), DARK),
    Solid("left_rail", (0, .025, .007), (.067, .006, .008), DARK),
    Solid("right_rail", (0, -.025, .007), (.067, .006, .008), DARK),
    Solid("front_rail", (-.029, 0, .007), (.006, .044, .008), DARK),
    Solid("back_rail", (.029, 0, .007), (.006, .044, .008), DARK),
])
body("reject_tray", REJECT, 0, [
    Solid("base", (0, 0, .003), (.135, .084, .006), RED),
    Solid("left_lip", (0, .040, .008), (.135, .004, .008), RED),
    Solid("right_lip", (0, -.040, .008), (.135, .004, .008), RED),
    Solid("front_lip", (-.0655, 0, .008), (.004, .076, .008), RED),
    Solid("back_lip", (.0655, 0, .008), (.004, .076, .008), RED),
])


def guide(name, xy, z, key):
    # Opening: 39 x 31 mm. Outside: 60 x 53 mm. All guide floors
    # have the same height; the posts support the blocked part level.
    solids = [
        Solid("floor", (0, 0, INSERT_BASE / 2), (.039, .031, INSERT_BASE), GREY),
        Solid("left_wall", (0, .021, .0135), (.060, .011, .027), GREY),
        Solid("right_wall", (0, -.021, .0135), (.060, .011, .027), GREY),
        Solid("front_wall", (-.02475, 0, .0135), (.0105, .031, .027), GREY),
        Solid("back_wall", (.02475, 0, .0135), (.0105, .031, .027), GREY),
        *[
            lead_in(f"guide_chamfer_{axis}_{index}", axis, side,
                    *(.0195, .0275, .030, .053) if axis == "x" else (.0155, .0235, .0265, .060),
                    .027, .036, GREY)
            for axis in ("x", "y") for index, side in enumerate((-1, 1))
        ],
        Solid("handle_bridge", (.039, 0, .032), (.026, .020, .010), GREY),
        Solid("handle", (.049, 0, .046), (.024, .022, .022), GREY),
    ]
    if key == "asymmetric":
        for index, (x, y) in enumerate(((-.010, -.007), (.010, -.007), (0, .008))):
            solids.append(Solid(f"key_lobe_{index}", (x, y, INSERT_BASE + BLOCK / 2),
                                (.004, .004, BLOCK), STEEL))
    elif key == "blocked":
        solids.append(Solid("oversize_key", (0, 0, INSERT_BASE + BLOCK / 2),
                            (.034, .026, BLOCK), STEEL))
    body(name, xy, z, solids, mass=.115)


guide("original_guide", FIX, FLOOR, None)
guide("good_spare", GOOD, 0, "asymmetric")
guide("blocked_spare", BAD, 0, "blocked")


def tip():
    # Horizontal slices of the 16 mm deep pocket pattern in the part tip.
    rows = (
        (-.013, -.012, ((-.017, .017),)),
        (-.012, -.002, ((-.017, -.015), (-.005, .005), (.015, .017))),
        (-.002, .003, ((-.017, .017),)),
        (.003, .013, ((-.017, -.005), (.005, .017))),
    )
    return [
        Solid(f"tip_{i}_{j}", ((x0 + x1) / 2, (y0 + y1) / 2, .008),
              (x1 - x0, y1 - y0, .016), BLUE)
        for i, (y0, y1, intervals) in enumerate(rows)
        for j, (x0, x1) in enumerate(intervals)
    ]


body("test_part", CRADLE, .006, [
    *tip(),
    Solid("main", (0, 0, .043), (.034, .026, .054), BLUE),
    *arrow(0, 0, HEIGHT + .0004),
    Solid("front_pointer", (-.0174, 0, HEIGHT - .001), (.001, .022, .002), WHITE, False),
    Solid("back_pointer", (.0174, 0, HEIGHT - .001), (.001, .022, .002), WHITE, False),
], mass=.062, yaw=pi)


def demo():
    """The right arm demonstrates the original fault, exchanges the guide, and retests."""
    points = []
    hover = TOP + .170
    closed, opened = FINGER_CLOSED, FINGER_OPEN
    a, b = pi / 2, -pi / 2
    coupon_grip = .073
    guide_grip = .051

    def wp(dt, xy, z, fingers, label, yaw):
        points.append(Waypoint(dt, (*xy, z), fingers, label, yaw))

    def move(src, src_z, dst, dst_z, y0, y1, label, *, prove=False):
        wp(1.25, src, hover, opened, f"approach {label}", y0)
        wp(1.1, src, TOP + src_z, opened, f"lower to {label}", y0)
        wp(.55, src, TOP + src_z, closed, f"grasp {label}", y0)
        wp(1.0, src, hover, closed, f"lift {label}", y0)
        if y0 != y1:
            wp(.9, src, hover, closed, "turn through a quarter turn", 0.0)
        wp(1.4, dst, hover, closed, f"carry {label}", y1)
        wp(1.5, dst, TOP + dst_z + .001, closed, f"lower {label}", y1)
        wp(.55, dst, TOP + dst_z + .001, opened, f"release {label}", y1)
        wp(1.05, dst, hover, opened, f"withdraw from {label}", y1)
        if prove:
            wp(1.2, dst, hover, opened, "observe depth after release", y1)

    def handle(xy):
        return (xy[0] + .049, xy[1])

    move(CRADLE, .006 + coupon_grip, FIX, SEAT + coupon_grip, a, a, "backward part", prove=True)
    move(FIX, SEAT + coupon_grip, CRADLE, .006 + coupon_grip, a, a, "test part")
    move(handle(FIX), FLOOR + guide_grip, handle((REJECT[0] - .015, REJECT[1])), .006 + guide_grip, 0., 0., "faulty guide")
    move(handle(GOOD), guide_grip, handle(FIX), FLOOR + guide_grip, 0., 0., "replacement guide")
    move(CRADLE, .006 + coupon_grip, FIX, SEAT + coupon_grip, a, b, "aligned part", prove=True)
    move(FIX, SEAT + coupon_grip, FIX, SEAT + BLOCK + coupon_grip, b, a, "backward part", prove=True)
    move(FIX, SEAT + BLOCK + coupon_grip, CRADLE, .006 + coupon_grip, a, a, "test part")
    wp(1.5, (.23, -.257), hover, opened, "park", a)
    return {"left": [], "right": points}
