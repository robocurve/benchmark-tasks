"""Fixed plain-carton geometry and wrapped-tape cut paths, SI units."""
TOP = .7475
CENTER = (.27, 0.)
LENGTH, WIDTH, HEIGHT, WALL = .22, .18, .075, .003
LID_Z = TOP + HEIGHT + WALL/2

# Ordinary corner-cut clearance allows the rigid panels to fold between walls.
# The centre seam is 2 mm; there are no handles or extra gripping colliders.
SEAM_GAP = .002
FLAP_WIDTH = (WIDTH-SEAM_GAP)/2
FLAP_CENTER_Y = WIDTH/2-FLAP_WIDTH/2
FLAP_LENGTH = LENGTH-2*WALL-.0005
FLAP_OUTWARD_LIMIT = 170.  # Carton flaps fold outward, clear of the opposite wrist.
FLAP_END_X = (CENTER[0]-FLAP_LENGTH/2,CENTER[0]+FLAP_LENGTH/2)

TAPE_WIDTH = .048
TAPE_DROP = .040
SEAM = (CENTER[0]-LENGTH/2,CENTER[0]+LENGTH/2)
END_CUT_X = FLAP_END_X
CUT_PATHS = (
    dict(name='seam',axis=0,fixed=0.,start=SEAM[0],length=LENGTH,segments=22,cells_per_segment=10),
    dict(name='front_fold',axis=1,fixed=END_CUT_X[0],start=-TAPE_WIDTH/2,length=TAPE_WIDTH,segments=4,cells_per_segment=12),
    dict(name='back_fold',axis=1,fixed=END_CUT_X[1],start=-TAPE_WIDTH/2,length=TAPE_WIDTH,segments=4,cells_per_segment=12),
)
CUT_SEGMENTS = sum(p['segments'] for p in CUT_PATHS)
CUT_CELLS = sum(p['segments']*p['cells_per_segment'] for p in CUT_PATHS)
CUTTER_START = (.13,.19,TOP+.022)
BLADE_THICKNESS = .0006
TIP_LOCAL = (.065,0.,-.020)
DESTINATION = (.18,-.23)
INSTRUCTION = ('Use the orange package cutter to cut the top seam and both taped end folds of the parcel. '
               'Put the cutter back on its orange pad, open both top flaps, and place the blue item on the green pad.')
