"""Repair an unpowered assembly fixture whose guide lets a part seat backward.

Each guide drops into an entry position, slides 16 mm under the fixture's lip
rails to a stop, and is locked there by a T-handle pin through its tab into the
fixture base. The fixture data plate gives the guide part number (427); each
guide carries its part number on its tab. The marked test part and fixture have
white arrows. A part that seats reaches the lower green depth mark; a correctly
blocked part stops at the upper white mark.
"""

from dataclasses import dataclass
from math import cos, pi, sin

from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]
CONTROL_HZ = 20
LIMIT = 4000
FIX = (0.335, -0.195)
CRADLE = (0.225, -0.110)
REJECT = (0.226, -0.302)
MATCH = (0.370, -0.335)
WRONG = (0.465, -0.210)
FLOOR = 0.010                 # fixture base top, where an installed guide stands
INSERT_BASE = 0.003           # guide floor thickness
SEAT = FLOOR + INSERT_BASE
BLOCK = 0.012
HEIGHT = 0.070
ENTRY = 0.016                 # slide from the drop-in position to the stop
TAB_TOP = FLOOR + 0.004       # pin collar rests here when fully inserted
PIN_OFFSET = -0.050           # pin axis, guide frame
PIN_HOLE = (FIX[0] + PIN_OFFSET, FIX[1])
PARK = (FIX[0] + 0.010, FIX[1] - 0.060)
PARK_TOP = 0.026
LIP = 0.0155                  # underside of the lip rails: 1.5 mm over the lugs
PART_NUMBER = "427"
GUIDES = ("original_guide", "matching_spare", "wrong_spare")
INSTRUCTION = (
    "Show that the test part seats in the installed guide with the arrows opposed. Pull the "
    "lock pin, slide the faulty guide out and put it in the red tray. Install the spare whose "
    "part number matches the fixture's data plate: slide it in to the stop and lock it with the "
    "pin. Show that the part seats with the arrows aligned and stops short with the arrows "
    "opposed, letting go each time. Return the test part to its cradle."
)
DARK = (0.20, 0.23, 0.28)
INK = (0.08, 0.09, 0.11)
GREY = (0.53, 0.58, 0.64)
STEEL = (0.72, 0.77, 0.81)
BLUE = (0.17, 0.43, 0.82)
RED = (0.70, 0.16, 0.14)
WHITE = (0.96, 0.96, 0.92)
GREEN = (0.25, 0.82, 0.43)
ORANGE = (0.95, 0.52, 0.10)


@dataclass(frozen=True)
class Solid:
    name: str
    pos: tuple
    size: tuple
    color: tuple
    collision: bool = True
    yaw: float = 0.0
    vertices: tuple = ()      # convex prism: first ring, then the matching second ring


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


def lead_in(name, axis, side, inside, mouth, outside, span, low, high, color, *, center=0.):
    """A solid chamfer extruded along the other horizontal axis."""
    contour = ((inside, low), (outside, low), (outside, high), (mouth, high))
    vertices = tuple(
        (side * distance, center + lateral, z) if axis == "x" else (center + lateral, side * distance, z)
        for lateral in (-span / 2, span / 2)
        for distance, z in contour
    )
    return Solid(name, (0., 0., 0.), (1., 1., 1.), color, vertices=vertices)


SEGMENTS = {  # seven-segment digits: a b c d e f g
    "0": "abcdef", "1": "bc", "2": "abdeg", "3": "abcdg", "4": "bcfg",
    "5": "acdfg", "6": "acdefg", "7": "abc", "8": "abcdefg", "9": "abcdfg",
}


def stamp(prefix, text, x, y, z, height, width, stroke):
    """White digits read from the robot side: up is +X, left to right is -Y."""
    pitch = width + .3 * height
    solids = []
    for index, char in enumerate(text):
        u0 = (index - (len(text) - 1) / 2) * pitch
        places = {
            "a": (0, height / 2, width, stroke), "d": (0, -height / 2, width, stroke),
            "g": (0, 0, width, stroke),
            "b": (width / 2, height / 4, stroke, height / 2),
            "c": (width / 2, -height / 4, stroke, height / 2),
            "e": (-width / 2, -height / 4, stroke, height / 2),
            "f": (-width / 2, height / 4, stroke, height / 2),
        }
        for segment in SEGMENTS[char]:
            u, v, size_u, size_v = places[segment]
            solids.append(Solid(f"{prefix}_{index}{segment}", (x + v, y - u0 - u, z),
                                (size_v, size_u, .0004), WHITE, False))
    return solids


def box(name, x0, x1, y0, y1, z0, z1, color, collision=True):
    return Solid(name, ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2),
                 (x1 - x0, y1 - y0, z1 - z0), color, collision)


def ring(prefix, cx, cy, hole, x0, x1, y0, y1, z0, z1, color):
    """A plate with a rectangular through-hole (``hole``: side, or (x, y)) centred at cx, cy."""
    hx, hy = (hole / 2, hole / 2) if isinstance(hole, float) else (hole[0] / 2, hole[1] / 2)
    pieces = [
        box(f"{prefix}_xm", x0, cx - hx, y0, y1, z0, z1, color),
        box(f"{prefix}_xp", cx + hx, x1, y0, y1, z0, z1, color),
        box(f"{prefix}_ym", cx - hx, cx + hx, y0, cy - hy, z0, z1, color),
        box(f"{prefix}_yp", cx - hx, cx + hx, cy + hy, y1, z0, z1, color),
    ]
    return [p for p in pieces if min(p.size) > 1e-6]


def riser(name, side):
    # Vertical side face 2 mm outside the lugs, opening to 6.5 mm in its top 3.5 mm.
    x0, x1 = -.046, .040
    inside, mouth, outside = .0355, .040, .042
    vertices = tuple((x, side * distance, z) for x in (x0, x1)
                     for distance, z in ((inside, .012), (outside, .012), (outside, LIP), (mouth, LIP)))
    return [box(f"{name}_low", x0, x1, *sorted((side * inside, side * outside)), .010, .012, DARK),
            Solid(f"{name}_lead", (0., 0., 0.), (1., 1., 1.), DARK, vertices=vertices)]


def lip(name, x, side):
    """A 12 mm hold-down lip whose entry (-X) edge is bevelled up over 3 mm."""
    x0, x1, bevel, top = x - .006, x + .006, .003, LIP + .0035
    y0, y1 = sorted((side * .0295, side * .042))
    vertices = tuple((xx, y, z) for y in (y0, y1)
                     for xx, z in ((x0, top - .001), (x0 + bevel, LIP), (x0 + bevel, top), (x0, top)))
    return [box(name, x0 + bevel, x1, y0, y1, LIP, top, DARK),
            Solid(f"{name}_bevel", (0., 0., 0.), (1., 1., 1.), DARK, vertices=vertices)]


body("fixture", FIX, 0, [
    box("base_main", -.042, .044, -.044, .044, 0, .010, DARK),
    *ring("base_pin", PIN_OFFSET, 0, (.012, .009), -.058, -.042, -.044, .044, 0, .010, DARK),
    box("base_rear", -.076, -.058, -.044, .044, 0, .010, DARK),
    *riser("riser_left", 1), *riser("riser_right", -1),
    *[part for i, side in enumerate((-1, 1)) for j, x in enumerate((-.015, .015))
      for part in lip(f"lip_{i}_{j}", x, side)],
    box("stop", .030, .038, -.020, .020, .010, .018, DARK),
    # Parking bore for the lock pin while the guide is out.
    box("park_arm", 0, .020, -.050, -.044, 0, .010, DARK),
    box("park_floor", 0, .020, -.070, -.050, 0, .004, DARK),
    *ring("park", .010, -.060, .008, 0, .020, -.070, -.050, .004, PARK_TOP, DARK),
    # Data plate and orientation arrow on the base, facing the robot.
    box("data_plate", -.075, -.057, .018, .042, .010, .0103, INK, False),
    *stamp("plate", PART_NUMBER, -.066, .030, .0105, .012, .0055, .0012),
    *arrow(-.066, -.028, .0103),
    # Depth scale beside the bore, outside the left lip rail.
    box("scale_foot", -.024, -.012, .044, .056, 0, .010, DARK),
    box("scale_post", -.022, -.014, .0445, .0535, .010, .099, DARK),
    box("seated_mark", -.0222, -.0218, .0445, .0535, SEAT + HEIGHT - .001, SEAT + HEIGHT + .001, GREEN, False),
    box("blocked_mark", -.0222, -.0218, .0445, .0535, SEAT + HEIGHT + BLOCK - .001,
        SEAT + HEIGHT + BLOCK + .001, WHITE, False),
])
body("cradle", CRADLE, 0, [
    Solid("base", (0, 0, .003), (.067, .064, .006), DARK),
    Solid("left_rail", (0, .025, .007), (.067, .006, .008), DARK),
    Solid("right_rail", (0, -.025, .007), (.067, .006, .008), DARK),
    Solid("front_rail", (-.029, 0, .007), (.006, .044, .008), DARK),
    Solid("back_rail", (.029, 0, .007), (.006, .044, .008), DARK),
])
body("reject_tray", REJECT, 0, [
    Solid("base", (0, 0, .003), (.150, .090, .006), RED),
    Solid("left_lip", (0, .043, .008), (.150, .004, .008), RED),
    Solid("right_lip", (0, -.043, .008), (.150, .004, .008), RED),
    Solid("front_lip", (-.073, 0, .008), (.004, .082, .008), RED),
    Solid("back_lip", (.073, 0, .008), (.004, .082, .008), RED),
])


def guide(name, xy, z, key, number):
    # Bore: 39 x 31 mm. Body: 60 x 53 mm. Four 10 x 7 x 4 mm lugs ride under the
    # fixture lips; the 28 x 28 mm tab carries the part number and a 12 x 8.5 mm slotted pin hole.
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
        *[box(f"lug_{i}_{j}", x - .005, x + .005, *sorted((side * .0265, side * .0335)), 0, .004, GREY)
          for i, side in enumerate((-1, 1)) for j, x in enumerate((-.015, .015))],
        box("tab_root", -.042, -.030, -.014, .014, 0, .004, GREY),
        *ring("tab", PIN_OFFSET, 0, (.012, .0085), -.0585, -.042, -.014, .014, 0, .004, GREY),
        box("label", -.045, -.031, -.013, .013, .004, .0043, INK, False),
        *stamp("pn", number, -.038, 0, .0045, .010, .0055, .0011),
    ]
    prongs = ((-.010, -.007), (.010, -.007), (0, .008))
    if key is not None:
        for index, (x, y) in enumerate(prongs):
            solids.append(Solid(f"key_lobe_{index}", (x, key * y, INSERT_BASE + BLOCK / 2),
                                (.004, .004, BLOCK), STEEL))
    body(name, xy, z, solids, mass=.125)


# The original 427 guide has lost its key; 428 is the mirror-hand variant.
guide("original_guide", FIX, FLOOR, None, PART_NUMBER)
guide("matching_spare", MATCH, 0, 1, PART_NUMBER)
guide("wrong_spare", WRONG, 0, -1, "428")


def octagon(name, low, high, flats_low, flats_high, color):
    def corners(flats, z):
        r = flats / 2 / cos(pi / 8)
        return [(r * cos(pi / 8 + k * pi / 4), r * sin(pi / 8 + k * pi / 4), z) for k in range(8)]
    return Solid(name, (0., 0., 0.), (1., 1., 1.), color,
                 vertices=tuple(corners(flats_low, low) + corners(flats_high, high)))


# Origin at the collar's underside: 11 mm, 6 mm across-flats shank with a 2 mm
# lead taper, 20 mm square collar and an 18 x 26 mm, 45 mm tall handle.
body("lock_pin", PIN_HOLE, TAB_TOP, [
    Solid("collar", (0, 0, .0015), (.020, .020, .003), STEEL),
    octagon("shank", -.011, 0., .006, .006, STEEL),
    octagon("tip", -.013, -.011, .0035, .006, STEEL),
    Solid("handle", (0, 0, .0255), (.018, .026, .045), ORANGE),
], mass=.030)


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
