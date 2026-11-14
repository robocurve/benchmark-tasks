"""Full-size Mini-ITX board and open test bench, in metres."""
from yam_arena.task_file import Box, Cylinder, TABLE_TOP
from parts import part

TOP = TABLE_TOP["station"]
BOARD_SIDE = .170
HOLE_OFFSET = .072
HOLE_SIDE = .008
PIN_RADIUS = .002
BOARD_THICKNESS = .0016
BENCH_XY = (.350, 0.0)
BOARD_START = (.100, 0.0, TOP + .004 + BOARD_THICKNESS / 2)
BOARD_GOAL = (BENCH_XY[0], BENCH_XY[1], TOP + .036 + BOARD_THICKNESS / 2)
HOLES = tuple((x, y) for x in (-HOLE_OFFSET, HOLE_OFFSET)
              for y in (-HOLE_OFFSET, HOLE_OFFSET))
PARK = {"left": (.12, .29, TOP + .17), "right": (.12, -.29, TOP + .17)}

# Split the 170 mm PCB into rectangles around four real apertures. The apertures
# are 8 mm square here so the rigid, 4 mm locating pins have clearance.
_edges = (-.085, -.076, -.068, .068, .076, .085)
_pcb = []
for i, (xa, xb) in enumerate(zip(_edges[:-1], _edges[1:])):
    for j, (ya, yb) in enumerate(zip(_edges[:-1], _edges[1:])):
        if i in (1, 3) and j in (1, 3):
            continue
        _pcb.append((((xa + xb) / 2, (ya + yb) / 2, 0),
                     (xb - xa, yb - ya, BOARD_THICKNESS), (.04, .30, .19)))

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
    # Removable polymer edge clamps protect opposite board edges during
    # handling and provide 26 mm wide, raised surfaces for the grippers.
    ((-.060, +.079, +.010), (.016, .026, .018), (.20, .22, .25)),
    ((+.050, -.079, +.010), (.016, .026, .018), (.20, .22, .25)),
), friction=.9))

MOVABLE = ("motherboard",)
STARTS = {"motherboard": BOARD_START}
GOALS = {"motherboard": BOARD_GOAL}
