"""On the station table: a mannequin head that faces the arms, and a pair of open reading glasses that stand on
the table in front of it and to its left, with the temples pointing at the head.

The head is made of fixed blocks: a face 11 cm wide, a nose, two ears and a forehead that steps back. Each eye
is a recessed socket with a light 3 mm plate in it, flush with the face. A light touch tips the plate back into
the socket, and it stays down: this is how the simulation records a touch on an eye.

Score (task_score), on the final state, after the robot had the whole run to let go:
  0     an eye plate is down (more than 10 degrees from upright), or none of the states below
  0.25  the glasses are off the table (lowest point 2 cm or more above it) and not on the head
  0.5   on the head, not seated: both temple tips behind the face, one on each side of the head
  1.0   seated: the pads or the bridge are over the nose, 3 mm below to 6 mm above its top, and each temple is
        over its own ear, 3 mm below to 1 cm above its top
There is no success() hook: it would end the episode as soon as the glasses are seated, while the gripper still
holds them.

Run it:  inspect-robots run --embodiment yam_arena -E task=put_glasses_on_head.py -E object_jitter=0 \\
           --instruction "Put the glasses on the person in front of you." --scorer task_score ...
"""

import numpy as np

from yam_arena.constants import FINGER_CLOSED
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Asset, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# ---- the glasses model (glasses_hot3d_robolab), in its own frame: x across, y toward the temples, z up (m) ----
# Measured from the model's mesh. EXTENT is its native depth, so the model keeps its real size (scale 1).
EXTENT = 0.1639
BBOX_CENTRE = np.array([0.0, 0.05905, -0.00755])  # the point the simulator reports as the object's pos
BRIDGE = np.array([0.0, -0.0185, 0.0065])  # centre of the bridge, where the demo grips it
BRIDGE_BOTTOM_Z = 0.002  # underside of the bridge at the centre line
PAD_LOW = np.array([0.0067, -0.0112, -0.0075])  # lowest point of the nose pads within the nose width, held
# Underside within the nose width, from the front of the bridge to the back of the pads, every 1.5 mm of y.
# Either the pads or, if the glasses slide forward, the bridge can rest on the nose.
NOSE_UNDER = np.array(
    [[0.0, -0.023, 0.0101], [0.0, -0.0215, 0.0019], [0.0, -0.02, -0.0016], [0.0, -0.0185, -0.0025],
     [0.0, -0.017, -0.0001], [0.0, -0.0155, -0.0017], [0.0, -0.014, -0.0012], [0.0, -0.011, -0.0086],
     [0.0, -0.0095, -0.0084]]
)
EAR_REST = np.array([0.064, 0.1195, -0.0013])  # lowest point of a temple over the ear, held (+x side; mirror for -x)
EAR_SPAN_Y = (0.09, 0.12)  # the part of each temple that lies over the ear (in front of the curl at the tip)
TEMPLE_TIP = np.array([0.061, 0.131, 0.0])  # end of a temple (+x side; mirror for -x)
# Underside of the +x temple where it can lie on the ear (mirror for -x), every 5 mm of y.
TEMPLE_UNDER = np.array(
    [[0.0673, 0.085, 0.0007], [0.0673, 0.09, 0.0024], [0.0655, 0.095, 0.0026], [0.0653, 0.1, 0.002],
     [0.0661, 0.105, 0.0014], [0.0647, 0.11, 0.0006], [0.0642, 0.115, -0.0005], [0.0639, 0.12, -0.0016],
     [0.0634, 0.125, -0.0032]]
)
LENS_LOW = np.array([0.0, -0.0088, -0.0305])  # lowest point of the lenses: the front edge it rests on
REST_PITCH = np.radians(9.45)  # on the table it tips back from LENS_LOW until the temple tips touch
MASS = 0.03  # real reading glasses: 25 to 35 g

# ---- layout (m) ----
X_FACE = 0.30  # front plane of the face (brow, cheeks, eye plates)
HALF_W = 0.055  # half the width of the head
DEPTH = 0.18  # front of the face to the back of the head
Z_EYE = TOP + 0.11  # eye centres
EYE_Y = 0.032  # eye centre, each side of the midline
SOCKET_W, SOCKET_H, SOCKET_D = 0.026, 0.024, 0.02  # eye socket opening and depth
Z_BROW, Z_FOREHEAD, Z_CROWN = Z_EYE + 0.025, Z_EYE + 0.055, Z_EYE + 0.10  # top of each band
FOREHEAD_BACK, CROWN_BACK = 0.012, 0.035  # how far the forehead and crown step back from the face
NOSE_W, NOSE_OUT = 0.014, 0.024  # nose width and how far it comes out of the face
EAR_W = 0.022  # how far each ear comes out of the head side
GLASSES_XY = (0.20, 0.14)  # start: bounding-box centre on the table

# ---- thresholds ----
MAX_PLATE_TILT_DEG = 10.0
LIFT = 0.02  # "off the table": lowest point at least this high above the table
# "Rests on": this far below to above the top of the nose or ear. The ear band is wider: the glasses collide as a
# convex decomposition, and in the demo one temple settled 5 to 7 mm above its ear while the frame sat on the nose.
NOSE_BAND, EAR_BAND = (-0.003, 0.006), (-0.003, 0.01)

SKIN, PLATE_RED = (0.86, 0.80, 0.74), (0.75, 0.15, 0.15)


def _rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


# The glasses as they rest on the table and as the demo holds them: temples along +x, tipped back REST_PITCH.
R_HELD = _ry(REST_PITCH) @ _rz(-np.pi / 2)
# Seated: the eye-level point 8 mm behind the nose pads lies on the face plane. The lenses are then 14 mm in front
# of the eye plates (12 mm is typical).
SEAT_ORIGIN = np.array([X_FACE, 0.0, Z_EYE]) - R_HELD @ np.array([0.0, 0.008, -0.003])


def _mirror(p):
    return p * np.array([-1.0, 1.0, 1.0])


def _seat(p):
    return SEAT_ORIGIN + R_HELD @ p


EAR_SEATS = (_seat(EAR_REST), _seat(_mirror(EAR_REST)))  # model +x temple is on the world -y side
NOSE_TOP = _seat(PAD_LOW)[2]
EAR_TOP = min(s[2] for s in EAR_SEATS) - 0.0015  # the two temples differ by 0.7 mm


def _block(name, x, y, z, color=SKIN):
    """A fixed box spanning the intervals x, y and z."""
    lo, hi = np.array([x[0], y[0], z[0]]), np.array([x[1], y[1], z[1]])
    return Box(name, pos=tuple((lo + hi) / 2), size=tuple(hi - lo), color=color)


_X, _BACK = X_FACE, X_FACE + DEPTH
_W = (-HALF_W, HALF_W)
_Z_SOCKET = (Z_EYE - SOCKET_H / 2, Z_EYE + SOCKET_H / 2)
_IN, _OUT = EYE_Y - SOCKET_W / 2, EYE_Y + SOCKET_W / 2  # socket edges from the midline
_EAR_X = tuple(float(_seat(np.array([0.0, y, 0.0]))[0]) for y in EAR_SPAN_Y)
_EAR_Z = (EAR_TOP - 0.05, EAR_TOP)
_NOSE_X = (_X - NOSE_OUT, _X)


def _eye_plate(name, y):
    return Box(
        name,
        pos=(_X + 0.002, y, _Z_SOCKET[0] + 0.011 + 0.0005),  # front 0.5 mm behind the face, on the socket floor
        size=(0.003, SOCKET_W - 0.002, 0.022),
        mass=0.003,
        color=PLATE_RED,
    )


HEAD = [
    _block("jaw", (_X, _BACK), _W, (TOP, _Z_SOCKET[0])),
    _block("socket_band", (_X + SOCKET_D, _BACK), _W, _Z_SOCKET),
    _block("nasal_root", (_X, _X + SOCKET_D), (-_IN, _IN), _Z_SOCKET),
    _block("eye_side_l", (_X, _X + SOCKET_D), (_OUT, HALF_W), _Z_SOCKET),
    _block("eye_side_r", (_X, _X + SOCKET_D), (-HALF_W, -_OUT), _Z_SOCKET),
    _block("brow", (_X, _BACK), _W, (_Z_SOCKET[1], Z_BROW)),
    _block("forehead", (_X + FOREHEAD_BACK, _BACK), _W, (Z_BROW, Z_FOREHEAD)),
    _block("crown", (_X + CROWN_BACK, _BACK), _W, (Z_FOREHEAD, Z_CROWN)),
    _block("nose", _NOSE_X, (-NOSE_W / 2, NOSE_W / 2), (NOSE_TOP - 0.03, NOSE_TOP)),
    _block("ear_l", _EAR_X, (HALF_W, HALF_W + EAR_W), _EAR_Z),
    _block("ear_r", _EAR_X, (-HALF_W - EAR_W, -HALF_W), _EAR_Z),
]
EYES = [_eye_plate("eye_l", EYE_Y), _eye_plate("eye_r", -EYE_Y)]
GLASSES = Asset("glasses", "glasses_hot3d_robolab", xy=GLASSES_XY, extent=EXTENT, mass=MASS, yaw=-np.pi / 2)
OBJECTS = [*HEAD, *EYES, GLASSES]


# ---- task quantities ----
def _rotation(quat_xyzw) -> np.ndarray:
    x, y, z, w = quat_xyzw
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def glasses_point(objects, p) -> np.ndarray:
    """World position of the point p, given in the glasses model's frame."""
    g = objects["glasses"]
    return g.pos + _rotation(g.quat_xyzw) @ (np.asarray(p) - BBOX_CENTRE)


def eye_touched(objects) -> bool:
    """An eye plate tilted past MAX_PLATE_TILT_DEG. A light touch tips it into its socket, and it stays down."""
    for name in ("eye_l", "eye_r"):
        up_z = _rotation(objects[name].quat_xyzw)[2, 2]
        if np.degrees(np.arccos(np.clip(up_z, -1.0, 1.0))) > MAX_PLATE_TILT_DEG:
            return True
    return False


def lifted(objects) -> bool:
    return bool(objects["glasses"].low[2] >= TOP + LIFT)


def on_head(objects) -> bool:
    """Both temple tips are behind the face, within the head's height, one on each side of the midline. The head
    is solid, so tips there straddle it."""
    tips = [glasses_point(objects, TEMPLE_TIP), glasses_point(objects, _mirror(TEMPLE_TIP))]
    behind = all(t[0] > X_FACE and TOP < t[2] < Z_CROWN for t in tips)
    return bool(behind and tips[0][1] * tips[1][1] < 0 and lifted(objects))


def _rests_on(points, x, y, top, band) -> bool:
    """Any of the world points is over the box footprint x, y and within band of its top."""
    p = np.atleast_2d(points)
    over = (x[0] <= p[:, 0]) & (p[:, 0] <= x[1]) & (y[0] <= p[:, 1]) & (p[:, 1] <= y[1])
    near = (top + band[0] <= p[:, 2]) & (p[:, 2] <= top + band[1])
    return bool(np.any(over & near))


def seated(objects) -> bool:
    """The nose pads rest on the nose and each temple rests on its own ear. The model's +x temple belongs on the
    world -y ear, so glasses that are upside down do not count."""
    g = objects["glasses"]
    rot = _rotation(g.quat_xyzw)

    def world(p):
        return g.pos + (np.atleast_2d(p) - BBOX_CENTRE) @ rot.T

    nose = _rests_on(world(NOSE_UNDER), _NOSE_X, (-NOSE_W / 2, NOSE_W / 2), NOSE_TOP, NOSE_BAND)
    right = _rests_on(world(TEMPLE_UNDER), _EAR_X, (-HALF_W - EAR_W, -HALF_W), EAR_TOP, EAR_BAND)
    left = _rests_on(world(_mirror(TEMPLE_UNDER)), _EAR_X, (HALF_W, HALF_W + EAR_W), EAR_TOP, EAR_BAND)
    return nose and right and left


# ---- hooks read by yam_arena (task_score) ----
def score(objects):
    """The score of the final state: 0 if an eye plate is down; 1.0 seated; 0.5 on the head; 0.25 off the
    table; otherwise 0. Seated does not need resting(): the reported velocity of the glasses jitters at up to
    0.03 m/s while their pose stays fixed to 0.01 mm."""
    if eye_touched(objects):
        return 0.0
    if seated(objects):
        return 1.0
    if on_head(objects):
        return 0.5
    return 0.25 if lifted(objects) else 0.0


# ---- scripted demo ----
PAD_TIP_BELOW_TCP = 0.013  # the pad tips are this far below the TCP
TIP_ABOVE_BRIDGE_BOTTOM = 0.001
FINGER_GRASP_OPEN = -0.012  # pads 24 mm apart: room for the 9 mm bridge, clear of the nose pads behind it
SLIDE_RISE = 0.02  # slide on this far above the seat, so the temple tips pass over the ears
SLIDE_START = 0.18  # bridge this far in front of its seat before the slide: temple tips clear of the face
# In the first sim run the left gripper arrived 1 to 2 cm to the +y side of its IK target (fitted from the top
# camera), so it pinched the frame beside the bridge and the glasses rolled. Every left-arm target moves by this.
LEFT_ARM_DY = -0.015


def _rest_pose():
    """(origin, rotation) of the glasses at rest on the table: spawned upright with their bounding-box centre over
    GLASSES_XY, then tipped back about the front edge of the lenses."""
    r_up = _rz(-np.pi / 2)
    origin_up = np.array([*GLASSES_XY, 0.0]) - r_up @ BBOX_CENTRE
    origin_up[2] = TOP - LENS_LOW[2]
    pivot = origin_up + r_up @ LENS_LOW
    tip = _ry(REST_PITCH)
    return pivot + tip @ (origin_up - pivot), tip @ r_up


def _tcp_at_bridge(origin, rot):
    """TCP over the bridge centre, with the pad tips 1 mm above the bridge's underside."""
    bridge = origin + rot @ BRIDGE
    bottom_z = (origin + rot @ np.array([0.0, BRIDGE[1], BRIDGE_BOTTOM_Z]))[2]
    return np.array([bridge[0], bridge[1], bottom_z + TIP_ABOVE_BRIDGE_BOTTOM + PAD_TIP_BELOW_TCP])


def demo():
    """Left arm: grip the bridge front to back, lift, carry the glasses in front of the face at 2 cm above their
    seat, slide them on until the bridge is over the nose, lower them onto the nose and ears, open a little while
    the back finger stays put, rise clear of the bridge and move away."""
    yaw = -np.pi / 2  # the fingers close along world x, across the bridge (+pi/2 is past joint6's limit)
    grasp = _tcp_at_bridge(*_rest_pose())
    seat = _tcp_at_bridge(SEAT_ORIGIN, R_HELD)
    above = seat + (0.0, 0.0, SLIDE_RISE)
    start = above - (SLIDE_START, 0.0, 0.0)
    travel = (grasp[0], grasp[1], start[2] + 0.03)
    opened = -0.006  # each finger 6 mm out; the TCP moves 6 mm back so the back finger keeps its place
    release = seat - (0.006, 0.0, 0.0)
    clear = release + (0.0, 0.0, 0.03)
    steps = [
        (1.5, travel, FINGER_GRASP_OPEN, "to the glasses"),
        (1.5, grasp, FINGER_GRASP_OPEN, "descend"),
        (0.8, grasp, FINGER_CLOSED, "close"),
        (1.2, travel, FINGER_CLOSED, "lift"),
        (2.0, start, FINGER_CLOSED, "in front of the face"),
        (3.0, above, FINGER_CLOSED, "slide on"),
        (1.5, seat, FINGER_CLOSED, "lower onto nose and ears"),
        (0.8, release, opened, "release"),
        (1.0, clear, opened, "rise"),
        (1.5, clear - (0.12, 0.0, -0.03), opened, "move away"),
        (1.5, (0.18, 0.22, TOP + 0.15), opened, "out of the top camera's view of the head"),
    ]
    shift = np.array([0.0, LEFT_ARM_DY, 0.0])
    return {"left": [Waypoint(d, tuple(np.asarray(p) + shift), f, label, yaw) for d, p, f, label in steps]}
