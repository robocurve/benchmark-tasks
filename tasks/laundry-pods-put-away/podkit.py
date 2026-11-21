"""Geometry and measurements for putting spilled laundry detergent pods away.

On a laundry-room counter: a torn refill pouch of laundry detergent pods with six pods spilled beside it;
the pods' own tub (flip-top child-resistant lid hinged at the back, standing slightly open); and an
empty glass cookie jar with its lid off, nearer the spill than the tub.

Frame: metres, x away from the arm bases, y left, z up; TOP is the counter (station table top).
Every body spawns with identity orientation, so a measured rotation is the body's rotation from rest.
All sizes marked est are estimates from product photographs until measured.
"""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np

from simkit import Hinge, Part, Piece

TOP = 0.7475

# -- colours (sRGB) -------------------------------------------------------------------------------
ORANGE = (0.95, 0.36, 0.05)          # the tub and pouch
ORANGE_D = (0.80, 0.28, 0.03)
LABEL_Y = (0.98, 0.80, 0.10)
LABEL_B = (0.10, 0.30, 0.75)
LABEL_W = (0.97, 0.97, 0.97)
POD_W = (0.96, 0.96, 0.94)
POD_B = (0.12, 0.25, 0.80)
POD_O = (0.95, 0.45, 0.10)
POD_G = (0.10, 0.65, 0.55)
GLASS = (0.80, 0.90, 0.95)
STEEL = (0.75, 0.76, 0.78)
RED = (0.80, 0.08, 0.08)

ASSETS = __import__("pathlib").Path(__file__).resolve().parent / "assets"

# -- the pods' tub (sizes from the CAD model, itself estimated from product photographs) -------------
TUB_C = (0.330, 0.030)               # centre of its foot on the counter
TUB_A, TUB_B = 0.073, 0.105          # inside half-depth (x) and half-width (y) of its rounded body
TUB_H = 0.153                        # body height to the shoulder
COLLAR_Z = 0.165                     # top of the raised collar round the mouth
WALL = 0.004
MOUTH = (0.090, 0.125)               # the opening in the top: x deep, y wide
TUB_MASS = 0.30                      # the tub with a few pods left in it
TUB_POS = (TUB_C[0], TUB_C[1], TOP + WALL / 2)       # root piece: the floor
LID = (0.112, 0.150, 0.004)          # the flip-top lid's top plate (its skirt is visual only)
LID_MASS = 0.035
HINGE = (0.057, 0.0, 0.161)          # hinge line in the tub's frame (CAD joints.json)
LID_TOP_Z = 0.169
LID_POS = (TUB_C[0], TUB_C[1], TOP + LID_TOP_Z - LID[2] / 2)    # as spawned: shut
LID_PIVOT = (TUB_C[0] + HINGE[0], TUB_C[1], TOP + HINGE[2])
LID_AJAR_TARGET = 53.0               # the hinge spring's rest angle; with the lid's weight it stands near 33 degrees
LID_K = 0.0008                       # N m per degree: a light spring, so the lid must be held open and falls back when let go
LID_OPEN_MAX = 110.0

# -- the glass cookie jar (CAD: 166 mm across, 205 mm tall) -------------------------------------------
JAR_S = 0.78                         # the CAD jar (166 x 205 mm) at 0.78: a half-gallon jar, 130 mm across, 160 mm tall
JAR_C = (0.200, -0.470)
JAR_R = 0.0785 * JAR_S               # inside radius
JAR_H = 0.205 * JAR_S
JAR_FLOOR = 0.006 * JAR_S

# -- the torn pouch and the spilled pods --------------------------------------------------------------
POUCH_C = (0.540, -0.440)
POUCH = (0.260, 0.200, 0.028)
POUCH_YAW = 0.35
POD_R, POD_H, POD_MASS = 0.021, 0.024, 0.024         # collider: 42 mm across, 24 mm thick; 24 g (est)
POD_Z0 = 0.0135                      # height of the pod's CAD origin (its flange plane) above the counter at rest
PODS = ((0.310, -0.235), (0.380, -0.235), (0.240, -0.235), (0.310, -0.325), (0.380, -0.325), (0.240, -0.325))
N_PODS = len(PODS)
COOKIES = ((0.014, 0.008, 0.0), (-0.018, -0.012, 0.4), (0.000, 0.020, 1.1), (0.010, -0.020, 2.0), (-0.014, 0.016, 2.6))


def _ring(a, b, h, z0, n, color, thick=WALL, **kw):
    """Upright staves round an ellipse (half-axes a along x, b along y): a hollow wall of convex pieces."""
    out = []
    for k in range(n):
        t = 2 * math.pi * (k + 0.5) / n
        x, y = a * math.cos(t), b * math.sin(t)
        yaw = math.atan2(a * math.sin(t), b * math.cos(t))
        seg = 2 * math.pi * math.hypot(a * math.sin(t), b * math.cos(t)) / n + 0.004
        out.append(Piece("box", (thick, seg, h), (x, y, z0 + h / 2), rot=(0.0, 0.0, yaw), color=color, **kw))
    return out


def _vis(names, offset):
    names = (names,) if isinstance(names, str) else names
    return dict(visual_usd=";".join(str(ASSETS / f"{n}.usdc") for n in names),
                visual_prim=";".join(f"/{n}" for n in names), visual_offset=offset)


def tub_object() -> Part:
    """The pods' tub. Colliders (hidden): floor, rounded wall, shoulder and the collar round the mouth; the look is
    the CAD mesh. One rigid body."""
    z0 = WALL / 2
    wall = _ring(TUB_A + WALL / 2, TUB_B + WALL / 2, TUB_H, z0 - WALL / 2, 20, ORANGE)
    mx, my = MOUTH[0] / 2, MOUTH[1] / 2
    sh = TUB_H - WALL
    shoulder = [
        Piece("box", (TUB_A - mx, 2 * TUB_B * 0.90, WALL), (-(TUB_A + mx) / 2, 0.0, sh), color=ORANGE),
        Piece("box", (TUB_A - mx, 2 * TUB_B * 0.90, WALL), ((TUB_A + mx) / 2, 0.0, sh), color=ORANGE),
        Piece("box", (MOUTH[0], TUB_B - my, WALL), (0.0, -(TUB_B + my) / 2, sh), color=ORANGE),
        Piece("box", (MOUTH[0], TUB_B - my, WALL), (0.0, (TUB_B + my) / 2, sh), color=ORANGE),
    ]
    ch = COLLAR_Z - TUB_H + WALL
    cz = TUB_H - WALL + ch / 2 - z0
    collar = [
        Piece("box", (0.007, MOUTH[1] + 0.014, ch), (-mx - 0.0035, 0.0, cz), color=ORANGE_D),
        Piece("box", (0.007, MOUTH[1] + 0.014, ch), (mx + 0.0035, 0.0, cz), color=ORANGE_D),
        Piece("box", (MOUTH[0], 0.007, ch), (0.0, -my - 0.0035, cz), color=ORANGE_D),
        Piece("box", (MOUTH[0], 0.007, ch), (0.0, my + 0.0035, cz), color=ORANGE_D),
    ]
    return Part("tub", pos=TUB_POS, size=(2 * TUB_A * 0.9, 2 * TUB_B * 0.9, WALL), exact_z=True, mass=TUB_MASS,
                color=ORANGE, friction=(0.6, 0.5), pieces=tuple(wall + shoulder + collar), **_vis(("tub", "tub_label"), (0.0, 0.0, -z0)))


def lid_object() -> Part:
    hinge = Hinge(pivot=LID_PIVOT, axis="Y", damping=0.0008, lower_deg=0.0, upper_deg=LID_OPEN_MAX,
                  stiffness=LID_K, target_deg=LID_AJAR_TARGET, parent="tub", parent_pos=TUB_POS)
    tab = Piece("box", (0.014, 0.044, 0.004), (-LID[0] / 2 - 0.006, 0.0, -0.003), color=ORANGE_D)
    off = tuple(a - b for a, b in zip((TUB_C[0], TUB_C[1], TOP), LID_POS))
    return Part("lid", pos=LID_POS, size=LID, exact_z=True, mass=LID_MASS, color=ORANGE, friction=(0.6, 0.5),
                hinge=hinge, pieces=(tab,), contact_offset=0.002, **_vis(("lid", "lid_label"), off))


def jar_object() -> Part:
    """A glass cookie jar with its lid off and a few cookies left in it: fixed. Pods can be dropped into it."""
    wall = _ring(JAR_R + 0.002, JAR_R + 0.002, JAR_H - JAR_FLOOR, JAR_FLOOR / 2, 20, GLASS, thick=0.004)
    return Part("jar", pos=(JAR_C[0], JAR_C[1], TOP + JAR_FLOOR / 2), size=(JAR_R, JAR_FLOOR, JAR_R), shape="cylinder",
                exact_z=True, color=GLASS, friction=(0.5, 0.4), pieces=tuple(wall), visual_scale=JAR_S, visual_glass_opacity=0.10, **_vis("jar", (0.0, 0.0, -JAR_FLOOR / 2)))


COOKIE = (0.72, 0.52, 0.28)
CHIP = (0.25, 0.14, 0.07)


def cookies_object() -> Part:
    """Five chocolate-chip cookies lying in the jar (fixed set dressing; they show the jar is for food)."""
    pieces = []
    for k, (x, y, a) in enumerate(COOKIES):
        z = 0.004 + 0.009 * k
        pieces.append(Piece("cylinder", (0.031, 0.008, 0.031), (x, y, z), rot=(0.12 * math.cos(a), 0.12 * math.sin(a), 0.0),
                            color=COOKIE))
        for j in range(5):
            b = a + 1.256 * j
            pieces.append(Piece("sphere", (0.0035, 0.0035, 0.0035), (x + 0.017 * math.cos(b), y + 0.017 * math.sin(b), z + 0.0042),
                                color=CHIP, collide=False))
    return Part("cookies", pos=(JAR_C[0], JAR_C[1], TOP + JAR_FLOOR + 0.004), size=(0.031, 0.008, 0.031), shape="cylinder",
                exact_z=True, color=COOKIE, friction=(0.6, 0.5), pieces=tuple(pieces))


def pouch_object() -> Part:
    """The torn refill pouch, lying flat (fixed set dressing). The look is the CAD mesh."""
    return Part("pouch", pos=(POUCH_C[0], POUCH_C[1], TOP + 0.006), size=(POUCH[0] * 0.9, POUCH[1] * 0.9, 0.012),
                exact_z=True, color=ORANGE, friction=(0.5, 0.4), yaw=POUCH_YAW, **_vis("pouch", (0.0, 0.0, -0.006)))


def pod_object(i: int) -> Part:
    x, y = PODS[i]
    return Part(f"pod{i + 1}", pos=(x, y, TOP + POD_H / 2 + 0.001), size=(POD_R, POD_H, POD_R), shape="cylinder",
                exact_z=True, mass=POD_MASS, color=POD_W, friction=(0.9, 0.8), restitution=0.02,
                contact_offset=0.002, yaw=0.9 * i, **_vis("pod", (0.0, 0.0, -POD_H / 2)))


# -- measurements -----------------------------------------------------------------------------------

def quat_matrix(q):
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def lid_open_deg(tub, lid) -> float:
    """How far the lid stands open (0 = shut flat on the rim)."""
    r = quat_matrix(np.asarray(tub, float)[3:7]).T @ quat_matrix(np.asarray(lid, float)[3:7])
    return math.degrees(math.atan2(r[0, 2], r[0, 0]))


def in_jar(p) -> bool:
    """A pod's centre inside the jar, or lying on what is in it: within the jar's inside radius, up to 3 cm above its rim."""
    p = np.asarray(p, float)
    return bool(math.hypot(p[0] - JAR_C[0], p[1] - JAR_C[1]) <= JAR_R + 0.004 and p[2] <= TOP + JAR_H + 0.030)


def in_tub(tub, p) -> bool:
    """A pod's centre inside the tub's body (in the tub's own frame): inside its wall and below its shoulder."""
    tub = np.asarray(tub, float)
    q = quat_matrix(tub[3:7]).T @ (np.asarray(p, float)[:3] - tub[:3])
    return bool((q[0] / TUB_A) ** 2 + (q[1] / TUB_B) ** 2 <= 1.0 and 0.0 <= q[2] <= TUB_H)


def tub_tilt_deg(tub) -> float:
    r = quat_matrix(np.asarray(tub, float)[3:7])
    return math.degrees(math.acos(float(np.clip(r[2, 2], -1.0, 1.0))))
