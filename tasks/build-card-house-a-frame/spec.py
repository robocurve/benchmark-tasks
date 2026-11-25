"""Two playing cards in place-card stands on a felt mat, in front of two YAM arms; SI units.

World frame: x forward from the line between the arm bases, y to the left, z up from the floor.
A card's own axes: x through its thickness, y across its width, z up its height.
"""
import math

CONTROL_HZ = 20
PHYSICS_HZ = 960
FINGER_FORCE_LIMIT = 40.
PAD_TORSIONAL_RADIUS = .002

# Table and felt card mat.
TABLE_CENTRE = (.450, 0.)
TABLE_SIZE = (.500, .700)
TABLE_TOP_Z = .760
MAT_CENTRE = (.430, 0.)
MAT_SIZE = (.400, .560, .003)
MAT_TOP_Z = TABLE_TOP_Z + MAT_SIZE[2]

# Standard poker-size paper playing cards.
CARD_W = .0635
CARD_H = .0889
CARD_T = .00029
CARD_MASS = .0018
# A paper-thin box lets an edge pushed into a face pass through it, so the card's collision surface stands CARD_REST
# off its drawn faces (PhysX rest offset): the cards collide as 1.09 mm slabs with 0.4 mm rounded edges.
CARD_REST = .0004
CARD_CONTACT = .0015
CARD_SKIN_T, CARD_SKIN_H = CARD_T + 2 * CARD_REST, CARD_H + 2 * CARD_REST

# Wooden place-card stands: a block with a slot across its length; each card stands upright in one.
STAND_SIZE = (.020, .060, .015)      # x (across the slot), y (along it), z
STAND_MASS = .040
SLOT_W = .0016
SLOT_DEPTH = .010
STAND_X = .370
STAND_Y = {'near': -.150, 'far': .150}   # 'near' is the right arm's card, 'far' the left arm's
STAND_TOP_Z = MAT_TOP_Z + STAND_SIZE[2]
CARD_START_Z = STAND_TOP_Z - SLOT_DEPTH + CARD_H / 2   # card centre, standing on the slot's floor

# Where the A-frame is built: the two top edges meet on a ridge along y.
RIDGE_X = .410
STAGGER = .012                       # the A-frame's cards sit this far apart sideways (near to -y, far to +y), as built by the demo
LEAN_DEG = 15.                       # each card from vertical, as built by the demo

FELT_FRICTION = (.60, .50)
CARD_FRICTION = (.40, .35)
WOOD_FRICTION = (.45, .40)

# Scoring.
ON_FELT = .002                       # a card's lowest point within this of the felt
APEX_GAP = .005                      # the two cards' top edges within this of each other
FEET_GAP = .020                      # the two cards' bottom edges at least this far apart
LEAN_RANGE = (10., 35.)              # each card's lean from vertical, degrees
STAND_STEPS = 100                    # 5 s at 20 Hz standing with no robot contact
FALLEN_Z = TABLE_TOP_Z - .050        # a card lower than this has fallen off the table
WORKSPACE = ((-.40, -1.40, 0.), (1.40, 1.00, 1.60))

INSTRUCTION_FALLBACK = (
    "Build the first A-frame of a house of cards: take both cards out of their stands and lean them against "
    "each other on the felt so that they stand up on their own."
)


def lean_pose(side, lean_deg=LEAN_DEG, ridge_x=RIDGE_X, lift=0.):
    """Centre and quaternion (w, x, y, z) of a card leaning lean_deg from vertical with its top edge on the ridge
    and its lowest edge lift above the felt. The near card leans away from the arms, the far card towards them."""
    a = math.radians(lean_deg)
    s = 1. if side == 'near' else -1.
    x = ridge_x - s * (CARD_SKIN_T / 2 * math.cos(a) + CARD_SKIN_H / 2 * math.sin(a))
    z = MAT_TOP_Z + lift + CARD_SKIN_H / 2 * math.cos(a) + CARD_SKIN_T / 2 * math.sin(a)
    return (x, -s * STAGGER, z), (math.cos(s * a / 2), 0., math.sin(s * a / 2), 0.)


START_TILT_DEG = math.degrees(math.atan((SLOT_W - CARD_SKIN_T) / SLOT_DEPTH))   # resting on the far jaw's edge


def start_pose(side):
    """A card standing in its stand's slot, its bottom against the slot's near wall and leaning on the far jaw."""
    a = math.radians(START_TILT_DEG)
    floor = STAND_TOP_Z - SLOT_DEPTH
    # Its lowest corner (far face, bottom) rests on the slot's floor just inside the near wall.
    corner = (STAND_X - SLOT_W / 2 + CARD_SKIN_T * math.cos(a) + .00005, floor + .0001)
    x = corner[0] - (CARD_SKIN_T / 2 * math.cos(a) - CARD_SKIN_H / 2 * math.sin(a))
    z = corner[1] + CARD_SKIN_T / 2 * math.sin(a) + CARD_SKIN_H / 2 * math.cos(a)
    return (x, STAND_Y[side], z), (math.cos(a / 2), 0., math.sin(a / 2), 0.)


def card_corners():
    """The card's eight corners in its own frame."""
    return [(sx * CARD_T / 2, sy * CARD_W / 2, sz * CARD_H / 2) for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]


def segment_distance(p0, p1, q0, q1):
    """Shortest distance between segments p0-p1 and q0-q1."""
    def sub(a, b):
        return [a[i] - b[i] for i in range(3)]

    def dot(a, b):
        return sum(a[i] * b[i] for i in range(3))
    d1, d2, r = sub(p1, p0), sub(q1, q0), sub(p0, q0)
    a, e, f = dot(d1, d1), dot(d2, d2), dot(d2, r)
    c, b = dot(d1, r), dot(d1, d2)
    denom = a * e - b * b
    s = min(max((b * f - c * e) / denom, 0.), 1.) if denom > 1e-15 else 0.
    t = (b * s + f) / e
    if t < 0.:
        t, s = 0., min(max(-c / a, 0.), 1.)
    elif t > 1.:
        t, s = 1., min(max((b - c) / a, 0.), 1.)
    gap = [p0[i] + d1[i] * s - q0[i] - d2[i] * t for i in range(3)]
    return math.sqrt(dot(gap, gap))


def finger_joint_position(aperture):
    if not 0 <= aperture <= .075:
        raise ValueError('Aperture exceeds 75 mm')
    return -min(aperture, .0749) / 2
