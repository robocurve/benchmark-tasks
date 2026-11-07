"""Open-loop demo: only the known starting geometry and elapsed plan time."""
import math
import numpy as np
import yam_arena.demo as motion
from yam_arena.constants import FINGER_OPEN as OPEN, FINGER_CLOSED as CLOSED, TCP_OFFSET
from yam_arena.urdf import axis_angle_matrix
from hoist_geometry import TOP


class GripOrientation(float):
    """Planar closure azimuth for coarse replay, full rotation for arm IK."""
    def __new__(cls, matrix):
        obj = super().__new__(cls, math.atan2(-matrix[0, 0], matrix[1, 0]))
        obj.matrix = np.asarray(matrix, float).copy()
        return obj


# GPU-proven tilted-wrist extension, used only by the scripted demo.
# Re-installed on every import and keyed on the `matrix` attribute, not the
# class: a re-imported copy of this module defines a new GripOrientation class,
# which a patch installed by an earlier copy would not recognise.
_original_grip_rotation = getattr(motion.grip_rotation, '_hoist_original', motion.grip_rotation)


def grip_rotation(yaw=0.):
    matrix = getattr(yaw, 'matrix', None)
    if isinstance(matrix, np.ndarray) and matrix.shape == (3, 3):
        return matrix
    if isinstance(yaw, np.ndarray) and yaw.shape == (3, 3):
        return yaw
    return _original_grip_rotation(yaw)


grip_rotation._hoist_matrix = True
grip_rotation._hoist_original = _original_grip_rotation
motion.grip_rotation = grip_rotation

PINCH_HEIGHT = .300
# cs-hoist-1 delivered 196/200 mm; 160 mm rise from the settled table.
STROKE = .160*(.200/.196)
DRAW = .130
LEFT_STAGE_Y = .100
# The deeper landing makes the old 35 mm retreat unreachable during the turn.
# An 18 mm slide clears the top wire and keeps the original rise and wrist turn.
LEFT_RELEASE_RETREAT = .018
RAISED_BOTTOM = .160
BAIL_PINCH = .090
DRAW_S = 3.
# A 15-degree downward pitch clears both the rim and the top wire while
# keeping pad closure along x and the pinch 20 mm above the rim.
LEFT_PITCH_DEG = 15.
LEFT_ROLL_DEG = 0.


def grip(side):
    a = math.radians(106. if side == 1 else 90.)
    return np.array(((side*math.sin(a), 0., math.cos(a)),
                     (-math.cos(a), 0., side*math.sin(a)), (0., -1., 0.)))


def load_length(y, bottom=RAISED_BOTTOM):
    """Straight tangent plus upper wrap at nominal 30 mm pitch radius."""
    r = .030
    dy, dz = y+.030, .400-(bottom+.110)
    d = math.hypot(dy, dz)
    phi = math.atan2(-dz, dy)+math.acos(r/d)
    return math.sqrt(d*d-r*r)+r*(math.pi-phi)


DRAW_PAYOUT = load_length(DRAW)-load_length(0.)


def demo():
    W = motion.Waypoint
    plans = {'right': [], 'left': []}
    R, L = grip(1), axis_angle_matrix(np.array((1., 0., 0.)), math.radians(LEFT_PITCH_DEG))@grip(-1)@axis_angle_matrix(np.array((0., 0., 1.)), math.radians(LEFT_ROLL_DEG))
    Rstage = np.array(((1., 0., 0.), (0., 0., 1.), (0., -1., 0.)))
    rp = np.array((.180, -.060, TOP+PINCH_HEIGHT))
    lp = np.array((.146, 0., TOP+RAISED_BOTTOM+BAIL_PINCH))
    ra, la = rp+(0., -.035, 0.), lp+(0., LEFT_STAGE_Y, 0.)
    def add(dt, label, right, left, rf=OPEN, lf=OPEN, rr=R, lr=L):
        rr = GripOrientation(rr) if isinstance(rr, np.ndarray) else rr
        lr = GripOrientation(lr) if isinstance(lr, np.ndarray) else lr
        plans['right'].append(W(dt, tuple(right), rf, label, rr))
        plans['left'].append(W(dt, tuple(left), lf, label, lr))
    add(2., 'approach staging', ra, la, rr=Rstage)
    add(.5, 'wait settled', ra, la, rr=Rstage)
    add(1.5, 'approach hauling fall', rp, la)
    add(1., 'close on rope', rp, la, CLOSED)
    for i in range(1, 5):
        right = rp+(0., 0., -STROKE*i/4)
        add(1., f'haul {i}', right, la, CLOSED)
        add(.5, f'haul {i} pause', right, la, CLOSED)
    top = right.copy()
    add(1., 'top hold', top, la, CLOSED)
    add(1.5, 'approach bail leg', top, lp, CLOSED)
    add(.7, 'close on bail leg', top, lp, CLOSED, CLOSED)
    # Sample a quintic minimum-jerk path at 50 ms. Its geometric payout keeps
    # the rope length constant while the pail moves horizontally over boards.
    for i in range(1, 61):
        u = i/60
        s = u*u*u*(10+u*(-15+6*u))
        y = DRAW*s
        right = top+(0., 0., load_length(y)-load_length(0.))
        left = lp+(0., y, 0.)
        add(.05, 'draw in', right, left, CLOSED, CLOSED)
    draw_right, draw_left = right.copy(), left.copy()
    add(2., 'land on boards', draw_right+(0., 0., .020), draw_left+(0., 0., -.023), CLOSED, CLOSED)
    right, left = draw_right+(0., 0., .045), draw_left+(0., 0., -.023)
    add(.7, 'pay out slack', right, left, CLOSED, CLOSED)
    add(.6, 'open on slack and bail', right, left)
    right, left = right+(0., -.050, 0.), left+(0., LEFT_RELEASE_RETREAT, 0.)
    add(.6, 'back away clear', right, left, rr=Rstage)
    # First slide completely off the wire; rising during that withdrawal
    # sweeps a fingertip into the horizontal top bar.
    left = left+(0., 0., .060)
    add(.6, 'rise clear', right, left, rr=Rstage)
    # Keep the palm fixed while turning the TCP downward, as in the probe.
    pr, pl = right-Rstage@TCP_OFFSET, left-L@TCP_OFFSET
    for degrees in (30, 60, 90):
        rr = Rstage@axis_angle_matrix(np.array((1., 0., 0.)), -math.radians(degrees))
        lr = axis_angle_matrix(np.array((1., 0., 0.)), math.radians(degrees*(90.-LEFT_PITCH_DEG)/90.))@L
        add(.5, f'turn down {degrees}', pr+rr@TCP_OFFSET, pl+lr@TCP_OFFSET, rr=rr, lr=lr)
    add(1.5, 'park', (.170, -.305, TOP+.170), (.146, .305, TOP+.170), rr=math.pi/2, lr=-math.pi/2)
    add(3., 'final rest', (.170, -.305, TOP+.170), (.146, .305, TOP+.170), rr=math.pi/2, lr=-math.pi/2)
    return plans
