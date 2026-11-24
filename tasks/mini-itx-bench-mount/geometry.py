"""Mini-ITX board and computer assembly fixture, in metres."""
import math
from yam_arena.task_file import Box, Cylinder, TABLE_TOP
from parts import part

TOP = TABLE_TOP["station"]
BOARD_SIDE = .170
HOLE_RADIUS = .00396 / 2
PIN_RADIUS = .0024 / 2
BOARD_THICKNESS = .0016
BENCH_XY = (.350, 0.0)
BOARD_START = (.100, 0.0, TOP + .004 + BOARD_THICKNESS / 2)
BOARD_GOAL = (BENCH_XY[0], BENCH_XY[1], TOP + .036 + BOARD_THICKNESS / 2)
# Mini-ITX addendum v1.1, Figure 3: C, F, H, J are a subset of ATX holes.
# The first drawing coordinate is depth from the rear I/O edge; the second is
# distance along that edge. Here +X faces the rear I/O opening and the board
# origin is its centre. Coordinates below are mm from the rear-side corner:
# C, F, H and J respectively.
_DATUM_HOLES_MM = ((10.16, 6.35), (33.02, 163.83),
                   (165.10, 6.35), (165.10, 163.83))
HOLES = tuple(((85.0 - u) / 1000, (v - 85.0) / 1000)
              for u, v in _DATUM_HOLES_MM)
PARK = {"left": (.12, .29, TOP + .17), "right": (.12, -.29, TOP + .17)}

def _pcb_boxes():
    """Approximate four round through-holes with thin collision strips.

    The strip aperture encloses the specified circle so collision boxes do
    not block a physically valid pin fit; scoring uses the exact circle.
    """
    edge = BOARD_SIDE / 2
    bands = 16
    y_edges = {-edge, edge}
    for _, hy in HOLES:
        y_edges.update(hy - HOLE_RADIUS + 2 * HOLE_RADIUS * i / bands
                       for i in range(bands + 1))
    boxes = []
    y_edges = sorted(y_edges)
    for ya, yb in zip(y_edges[:-1], y_edges[1:]):
        cuts = []
        for hx, hy in HOLES:
            nearest = 0 if ya <= hy <= yb else min(abs(ya - hy), abs(yb - hy))
            if nearest < HOLE_RADIUS:
                half_width = math.sqrt(HOLE_RADIUS**2 - nearest**2)
                cuts.append((hx - half_width, hx + half_width))
        xa = -edge
        for cut_lo, cut_hi in sorted(cuts):
            if cut_lo > xa:
                boxes.append((((xa + cut_lo) / 2, (ya + yb) / 2, 0),
                              (cut_lo - xa, yb - ya, BOARD_THICKNESS), (.04, .30, .19)))
            xa = cut_hi
        if edge > xa:
            boxes.append((((xa + edge) / 2, (ya + yb) / 2, 0),
                          (edge - xa, yb - ya, BOARD_THICKNESS), (.04, .30, .19)))
    return tuple(boxes)


_pcb = _pcb_boxes()

OBJECTS = [
    Box("esd_mat", pos=(.100, 0, TOP + .002), size=(.21, .21, .004), color=(.12, .14, .16)),
    Box("bench_plate", pos=(.350, 0, TOP + .006), size=(.24, .24, .012), color=(.43, .46, .49)),
    # Rear wall is split around the actual port opening; the board itself ends
    # before this wall, while its I/O housing passes into the opening.
    Box("rear_wall_lower", pos=(.455, 0, TOP + .024), size=(.006, .100, .020), color=(.41, .44, .47)),
    Box("rear_wall_left", pos=(.455, -.080, TOP + .045), size=(.006, .060, .066), color=(.41, .44, .47)),
    Box("rear_wall_right", pos=(.455, +.080, TOP + .045), size=(.006, .060, .066), color=(.41, .44, .47)),
]
for i, (dx, dy) in enumerate(HOLES):
    x, y = BENCH_XY[0] + dx, BENCH_XY[1] + dy
    OBJECTS.append(Cylinder(f"standoff_{i}", pos=(x, y, TOP + .024),
                            radius=.007, height=.024, color=(.70, .60, .37)))
    OBJECTS.append(Cylinder(f"locating_pin_{i}", pos=(x, y, TOP + .039),
                            radius=PIN_RADIUS, height=.006, color=(.70, .60, .37)))

OBJECTS.append(part("motherboard", BOARD_START, mass=.44, boxes=(
    *_pcb,
    ((-.019, 0, +.006), (.048, .048, .010), (.48, .50, .52)),  # CPU socket cover
    ((+.040, -.035, +.005), (.010, .060, .008), (.14, .15, .16)),
    ((+.040, +.035, +.005), (.010, .060, .008), (.14, .15, .16)),
    ((+.087, 0, +.011), (.038, .060, .018), (.62, .64, .67)),  # rear I/O housing
    ((-.067, +.060, +.002), (.012, .016, .002), (.83, .69, .21)),
    # Protective polymer edge clamps stay clear of the mounting holes.
    ((-.060, +.079, +.010), (.016, .026, .018), (.20, .22, .25)),
    ((+.050, -.079, +.010), (.016, .026, .018), (.20, .22, .25)),
), friction=.9))

MOVABLE = ("motherboard",)
STARTS = {"motherboard": BOARD_START}
GOALS = {"motherboard": BOARD_GOAL}
