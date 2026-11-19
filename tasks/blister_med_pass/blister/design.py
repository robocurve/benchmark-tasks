"""Blister cards, tablets, the medicine cup and the medication record: dimensions and contents. Pure Python.

Card frame: origin at the centre of the card's underside, x along its length (the pharmacy label at the -x end), y
across it, z up. Cells are numbered 1 to 30, three to a row, left to right (+y to -y) and from the label outwards.
"""

from __future__ import annotations

from dataclasses import dataclass

# -- card ------------------------------------------------------------------------------------------------------------
CARD_SIZE = (0.27, 0.08, 0.004)  # length (x), width (y), thickness (z) [m]: card and blister layer as one slab
CARD_MASS = 0.03  # kg
ROWS, COLS = 10, 3
CELL_PITCH = (0.020, 0.022)  # x, y [m]
ROW0_X = -0.065  # the first row's centre: 6 cm of label area lie before it
HOLE = 0.012  # square opening of each cell through the card [m]
LABEL_LENGTH = 0.055  # the printed pharmacy label, from the card's -x end [m]

# -- foil: one hinged flap under each filled cell -------------------------------------------------------------------
FLAP_T = 0.002  # thickness [m] (thicker than real foil, so a pressed tablet cannot sink into it); flush with the card's underside
FLAP_MASS = 0.0005  # kg
FLAP_DRIVE = {"stiffness": 3.0, "damping": 0.002, "armature": 1e-6}  # N m/rad, N m s/rad, kg m^2: holds the tablet
OPEN_ANGLE = 1.5  # rad: once torn, the flap swings this far down
FLAP_LIMIT = 1.75  # rad: the joint's travel, downwards from the card's underside

# -- bubble: a dome over each filled cell that a fingertip presses down onto the tablet --------------------------------
# The press is measured on the bubble: its face sits on a stiff spring just above the tablet, and the foil tears once
# the bubble is pressed with TEAR_FORCE.
TEAR_FORCE = 10.0  # N: pressing a tablet through blister foil takes 10 N or more
DOME = (0.011, 0.011, 0.003)  # its pressing face (x, y, z) [m]
DOME_GAP = 0.002  # the face's underside above the tablet [m]
DOME_MASS = 0.0005  # kg
DOME_TRAVEL = 0.006  # m it can be pressed down
DOME_TEAR = 0.00125  # m: pressed this far down, it carries TEAR_FORCE
DOME_DRIVE = {"stiffness": TEAR_FORCE / DOME_TEAR, "damping": 40.0, "armature": 0.05}  # N/m (8,000), N s/m, kg: the
# armature (inertia on the joint, no weight) keeps a fingertip from pushing through the thin face in PhysX

# -- tablet ----------------------------------------------------------------------------------------------------------
PILL_RADIUS = 0.0045  # m (9 mm round tablet, 6 mm thick)
PILL_HEIGHT = 0.006
PILL_MASS = 0.001  # kg (a real tablet is 0.2 to 0.4 g)
PILL_Z = FLAP_T + PILL_HEIGHT / 2  # its centre above the card's underside, resting on its flap
DOME_Z = FLAP_T + PILL_HEIGHT + DOME_GAP + DOME[2] / 2  # the dome's centre at rest, above the tablet

# -- medicine cup (1 oz paper souffle cup) ---------------------------------------------------------------------------
CUP_RADIUS = 0.022  # inner radius at the rim [m]
CUP_HEIGHT = 0.03
CUP_WALL = 0.003  # m
CUP_MASS = 0.02  # kg
CUP_SOLVER_ITERATIONS = 32  # the card is pressed onto its rim with 10 N or more
CUP_STAVES = 16

# -- medication administration record --------------------------------------------------------------------------------
MAR_SIZE = (0.216, 0.279, 0.001)  # a US letter page in landscape: x (depth), y (width), thickness [m]


@dataclass(frozen=True)
class Card:
    """A resident's 30-day blister card for one drug: cells ``first_full`` .. 30 still hold a tablet."""

    name: str  # scene object name
    drug: str
    strength: str
    directions: str
    pill_color: tuple[float, float, float]
    band_color: tuple[float, float, float]  # the label's colour band
    first_full: int

    def full_cells(self) -> range:
        return range(self.first_full, ROWS * COLS + 1)

    def pill_name(self, cell: int) -> str:
        return f"{self.name}_tab{cell}"


def cell_xy(cell: int) -> tuple[float, float]:
    """A cell's centre in the card frame."""
    row, col = divmod(cell - 1, COLS)
    return ROW0_X + row * CELL_PITCH[0], (1 - col) * CELL_PITCH[1]


def flap_joint(cell: int) -> str:
    return f"flap_{cell}"


def dome_joint(cell: int) -> str:
    return f"dome_{cell}"


RESIDENT = "M. OKAFOR"
ROOM = "Room 14B"
CARDS = (
    Card("furosemide", "FUROSEMIDE", "40 mg tablet", "Give 2 tablets by mouth every morning",
         (0.96, 0.96, 0.94), (0.20, 0.45, 0.85), first_full=25),
    Card("metoprolol", "METOPROLOL TARTRATE", "25 mg tablet", "Give 1 tablet by mouth twice daily",
         (0.95, 0.55, 0.65), (0.85, 0.35, 0.25), first_full=27),
    Card("atorvastatin", "ATORVASTATIN", "20 mg tablet", "Give 1 tablet by mouth at bedtime",
         (0.98, 0.92, 0.60), (0.25, 0.65, 0.35), first_full=27),
)  # fmt: skip
# The medication administration record: drug -> (tablets per dose, times given).
MAR = {
    "furosemide": (2, ("08:00",)),
    "metoprolol": (1, ("08:00", "20:00")),
    "atorvastatin": (1, ("21:00",)),
}
NOW = "08:00"


def due() -> dict[str, int]:
    """Tablets of each card due now."""
    return {name: n if NOW in times else 0 for name, (n, times) in MAR.items()}
