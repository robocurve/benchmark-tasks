"""Every dimension of the storage drawer and the pillow, in metres. NumPy only.

Tabletop frame: x forward from the line through the two arm bases, y to the robot's left, z up from the
table top. The drawer slides along x; "open" is the distance from the frame's front face to the outer face
of the drawer front.

Each number is marked PUBLISHED (read on the maker's or a retailer's page, see README.md), DERIVED from
published numbers, or ESTIMATE.
"""

import numpy as np

# -- the storage drawer: Sterilite 16 Qt. Stacking Drawer, model 2301 -------------------------------------
FRAME_SIZE = (.432, .365, .175)          # PUBLISHED 17 x 14 3/8 x 6 7/8 in: front to back, width, height
DRAWER_INNER = (.416, .302, .124)        # PUBLISHED "interior at bottom of drawer" 16 3/8 x 11 7/8 x 4 7/8 in
FRAME_FRONT_X = .390                     # where the unit stands: its front face, centred on y = 0
RAIL_Z = .152                            # ESTIMATE underside of the frame's top at the opening
FRAME_WALL = .008                        # modelled side wall (the real moulding is about 3 mm; outside sizes are true)
FRAME_BACK = .002
FLOOR_Z = .022                           # ESTIMATE inside surface of the drawer's floor above the table
RIM_Z = FLOOR_Z + DRAWER_INNER[2]        # DERIVED top of the drawer's side walls: 146 mm
DRAWER_FLOOR_T = .008
DRAWER_WALL = .008                       # modelled (real about 2.5 mm; the inside sizes are true)
DRAWER_BACK = .003
FRONT_T = .008                           # drawer front panel, modelled thickness
FRONT_WIDTH = .343                       # ESTIMATE: fills the frame's opening with 3 mm to spare each side
FRONT_Z = (.012, .148)                   # ESTIMATE: bottom and top edge of the front panel
PULL_SIZE = (.012, .120, .008)           # ESTIMATE the moulded pull: a lip standing 12 mm proud of the front
PULL_Z = .132                            # its middle height
OPEN_START = .220                        # how far the drawer is pulled out at the start
OPEN_LIMIT = .300                        # ESTIMATE the stop that keeps the drawer from coming right out
RUNNER_FRICTION_N = 3.                   # ESTIMATE force to slide the loaded drawer on its plastic runners
DRAWER_MASS = .50                        # ESTIMATE (maker's shipping weight for a case of six: 19 lb, about 1.4 kg each)

# -- the pillow: Therm-a-Rest Compressible Pillow Cinch, Regular ------------------------------------------
PILLOW_PUBLISHED = (.46, .33, .15)       # PUBLISHED 18 x 13 in, 6 in thick, lying free
PILLOW_MASS = .325                       # PUBLISHED 325 g
# Stuffed into the 416 x 302 mm drawer it is squeezed to the drawer's plan. It is modelled as a base slab with
# two columns of eight sprung pads on it: 16 pads, each on its own vertical slide with a spring and a damper.
ROWS, COLS = 8, 2
BASE_SIZE = (.410, .296, .020)
PAD_SIZE = (.048, .143, .025)            # along the slide, across, thick
ROW_PITCH = BASE_SIZE[0] / ROWS          # 51.25 mm
PAD_Y = (.0745, -.0745)                  # centres of the left and the right column
# Height of the pillow's top above the drawer's floor, row by row from the drawer front, left alone.
REST_TOP = (.118, .140, .150, .150, .150, .150, .140, .118)   # middle PUBLISHED (15 cm); the fall-off to the edges ESTIMATE
PAD_TRAVEL = .060                        # how far a pad can be pressed down
PAD_STIFFNESS = 60.                      # ESTIMATE N/m per pad: a 1.5 kg book on the middle sinks to about the rim
PAD_DAMPING = 6.                         # ESTIMATE N s/m per pad: springs back in about a third of a second
PAD_PRELOAD_M = .004                     # the spring is still 4 mm compressed when a pad is at its top stop
PAD_MASS = .015
BASE_MASS = PILLOW_MASS - ROWS * COLS * PAD_MASS
PILLOW_GAP = (DRAWER_INNER[0] - BASE_SIZE[0]) / 2     # 3 mm of play at each end of the drawer

# -- friction (all ESTIMATE) --------------------------------------------------------------------------------
FRICTION = {"frame": .20, "drawer": .20, "fabric": .40}       # polypropylene, polypropylene, polyester cover

# -- derived places ---------------------------------------------------------------------------------------
RAIL_X = FRAME_FRONT_X                   # the edge the pillow catches on: front lower edge of the frame's top
ROW_PROUD = tuple(FLOOR_Z + top - RAIL_Z for top in REST_TOP)        # > 0: stands above the opening
PROUD_ROWS = tuple(i for i, proud in enumerate(ROW_PROUD) if proud > 0)
PADS = tuple((row, col) for row in range(ROWS) for col in range(COLS))


def pad_name(row, col):
    return f"pad_{row}_{'lr'[col]}"


ACTORS = ("drawer", "pillow_base", *(pad_name(row, col) for row, col in PADS))
MASSES = (DRAWER_MASS, BASE_MASS, *([PAD_MASS] * len(PADS)))


def row_lead(row):
    """Leading (far) edge of a row's pads, measured from the outer face of the drawer front, with the pillow in
    the middle of its play."""
    return FRONT_T + PILLOW_GAP + (row + 1) * ROW_PITCH - (ROW_PITCH - PAD_SIZE[0]) / 2


def row_centre(row):
    return row_lead(row) - PAD_SIZE[0] / 2


def blocking_open(row):
    """Opening of the drawer at which this row's leading edge meets the frame."""
    return row_lead(row)


def drawer_x(open_m):
    """Tabletop x of the drawer body's origin (outer face of its front, on the table plane)."""
    return FRAME_FRONT_X - open_m


def under_frame_at_start(row):
    return row_lead(row) > OPEN_START + .001


def start_poses():
    """Tabletop start position (x, y, z) of every body's origin. Drawer: outer face of its front on the table
    plane. Pillow base: middle of its underside. Pad: middle of its top face. Pads that start under the
    frame's top start pressed down to just below it."""
    front = drawer_x(OPEN_START)
    poses = {"drawer": (front, 0., 0.),
             "pillow_base": (front + FRONT_T + DRAWER_INNER[0] / 2, 0., FLOOR_Z + .0005)}
    for row, col in PADS:
        top = FLOOR_Z + REST_TOP[row] + .0005
        if under_frame_at_start(row):
            top = min(top, RAIL_Z - .0008)
        poses[pad_name(row, col)] = (front + row_centre(row), PAD_Y[col], top)
    return poses


def press_force(row, depth):
    """Force (N) to hold both pads of a row ``depth`` below where they rest."""
    return COLS * PAD_STIFFNESS * (PAD_PRELOAD_M + depth)


def unit_box(open_m):
    """Bounding box (low, high) of the frame, the drawer and its pull, for the hands-clear rule."""
    front = drawer_x(max(0., open_m)) - PULL_SIZE[0]
    low = np.array((front, -FRAME_SIZE[1] / 2, 0.))
    high = np.array((FRAME_FRONT_X + FRAME_SIZE[0], FRAME_SIZE[1] / 2, FRAME_SIZE[2]))
    return low, high


assert abs(BASE_MASS - .085) < 1e-9 and PROUD_ROWS == (1, 2, 3, 4, 5, 6)
assert not under_frame_at_start(3) and under_frame_at_start(4)
