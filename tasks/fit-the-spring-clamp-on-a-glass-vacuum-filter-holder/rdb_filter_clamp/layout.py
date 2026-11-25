"""Every dimension of the filter holder, flask, funnel and spring clamp, and the geometry the scorer uses.

NumPy only. Lengths in metres, angles in radians unless a name says otherwise.

Status of each number: PUBLISHED (read in a document or on a page that was opened), ESTIMATE (ours; no maker
publishes it; measure the real part and put the value here).

Sources opened (see README): the MilliporeSigma user guide "47 mm Glass Vacuum Filter Holders" (cat. no.
XX1014700: assembled height 23.5 cm, filter holder diameter 7.6 cm, 300 mL funnel, anodised aluminium clamp,
size 8 silicone stopper with a 15 mm hole, effective filter area 9.6 cm2, parts XX1014702 base + stopper,
XX1014703 clamp, XX1014704 funnel); the Fisher Scientific listing of XX1014700 (23.5 cm, 7.6 cm, 300 mL);
the Thermo Scientific Nalgene filter funnel instructions ("Pinch clamp handle together and remove clamp from the
flange"; "Vacuum should be applied slowly to avoid ... accidentally tipping the holder").

Frames. Holder frame: origin on the holder's axis at table height, z up, x forward (away from the robot).
Funnel frame: origin on its axis at its ground bottom face (the joint plane when seated), z up. Clamp frame:
origin at the ring's centre on the joint plane, z up, the open mouth toward +x, the hinge pin and the two
handles toward -x. With everything seated the clamp frame sits on the holder axis at the joint height.
"""

import numpy as np

MM = 1e-3

# ---- flask and holder base (one rigid body in the simulation) ---------------------------------------------
JOINT_Z = .288                  # joint plane (top of the base's ground flange) above the table; follows from below
FLASK_R = .0625                 # ESTIMATE: 1 L heavy-wall filtering flask, about 125 mm across the base
FLASK_CONE_TOP = .165           # ESTIMATE: where the cone meets the neck
FLASK_NECK_R = .023             # ESTIMATE: neck outside radius (takes a size 8 stopper)
FLASK_RIM = .205                # ESTIMATE: height of the neck's rim
FLASK_MASS = .55                # ESTIMATE: heavy-wall borosilicate, 1 L
STOPPER = (.0205, .017, .025)   # size 8 stopper: top radius, bottom radius, height (ESTIMATE from size tables)
STOPPER_TOP = .210              # it stands 5 mm proud of the rim
STEM_TIP = .178                 # base's drip tip inside the flask
STEM_R = .0075                  # fits the stopper's 15 mm hole (PUBLISHED hole size)
CUP_BOTTOM_Z, CUP_BOTTOM_R = .215, .010       # the base's funnel-shaped body under the frit
FLANGE_R = .038                 # PUBLISHED: filter holder diameter 7.6 cm
FLANGE_T = .006                 # ESTIMATE: each ground flange 6 mm thick
CHAMFER_R = .035                # ESTIMATE: the flange's back edge is bevelled from r 35 to 38 mm ...
CHAMFER_Z = .003                # ... over the 3 mm next to its back face
BASE_MASS = .17                 # ESTIMATE: fritted base 0.14 kg and stopper 0.03 kg
HOLDER_MASS = FLASK_MASS + BASE_MASS
FRIT_R = .0235                  # PUBLISHED effective area 9.6 cm2 -> 35 mm open diameter; frit drawn 47 mm
MEMBRANE_R = .0235              # a 47 mm membrane
MEMBRANE_T = .00015

# ---- funnel ------------------------------------------------------------------------------------------------
NECK_R, NECK_TOP = .025, .014   # ESTIMATE: short neck above the flange
BODY_R = .032                   # ESTIMATE: 64 mm outside across the graduated cylinder
TAPER_TOP = .026
FUNNEL_H = .125                 # ESTIMATE, chosen so that funnel + base + stem give the PUBLISHED 23.5 cm
FUNNEL_WALL = .0025
FUNNEL_MASS = .20               # ESTIMATE: about 86 cm3 of borosilicate glass
FUNNEL_COM = .065
BODY_SIDES = 16                 # collider: 16 flats, two of them facing the fingers (+-x)

# ---- spring clamp ------------------------------------------------------------------------------------------
PIVOT = np.array((-.047, 0.))   # hinge pin, clamp frame
LIP_R, ROOT_R, OUT_R = .0345, .040, .045      # ESTIMATE: lips' inner edge, end of the V faces, outer rim
HALF_H = .009                   # ring 18 mm tall
FACE_Z_LIP, FACE_Z_ROOT = .0065, .0010        # each V face runs from (LIP_R, 6.5 mm) to (ROOT_R, 1 mm): z = 41 mm - r
ARC = (np.radians(8.), np.radians(172.))      # each jaw's arc about the ring centre, from the mouth to the hinge
SEGMENTS = 10
TAB_LEN, TAB_T, TAB_H, TAB_START = .030, .005, .016, .005    # handles behind the pin
THETA_MAX = np.radians(27.)     # each jaw's opening with the handles pressed together (the handles touch)
THETA_MIN = np.radians(-2.)     # jaws' tips touch with nothing between them
SPRING = .30                    # ESTIMATE: N m/rad on each jaw ...
PRELOAD = np.radians(40.)       # ... with the spring's rest this far past closed (a pre-wound torsion spring)
JAW_MASS, KNUCKLE_MASS = .020, .006
CLAMP_MASS = 2 * JAW_MASS + KNUCKLE_MASS
GRIP_IN_CLAMP = np.array((PIVOT[0] - .024, 0., .008))   # fingertip centre on the handles, clamp frame
# Force at the handles with them pressed together: each handle's torque over its lever to the pad centre.
HANDLE_LEVER = .021
HANDLE_FORCE_N = SPRING * (THETA_MAX + PRELOAD) / HANDLE_LEVER      # about 16.7 N
SEATED_DEG = (-.5, 7.)          # jaw angle band (degrees, opening positive) that means "closed on the flanges"

# ---- start layout, tabletop frame --------------------------------------------------------------------------
AXIS = np.array((.330, -.030))          # the holder's axis
FUNNEL_START = np.array((.270, .175))   # the funnel standing on its flange on a clean wipe
CLAMP_START = np.array((.300, -.215))   # the clamp lying flat and shut, mouth away from the robot
CLAMP_REST_Z = HALF_H                   # it lies on its jaws' lower faces


def ring(r, z, n, phase=0.):
    a = phase + np.arange(n) * 2 * np.pi / n
    return np.column_stack((r * np.cos(a), r * np.sin(a), np.full(n, float(z))))


# ---- collision pieces (each a vertex set whose convex hull is the collider) --------------------------------
def funnel_pieces():
    """Funnel frame. The flange (bevelled ring), neck, taper and body; the bore is closed for collision
    (nothing goes inside the funnel in the simulation)."""
    body_r = BODY_R / np.cos(np.pi / BODY_SIDES)              # apothem BODY_R: flats 64 mm apart
    return {
        "flange": np.vstack((ring(FLANGE_R, 0., 20), ring(FLANGE_R, CHAMFER_Z, 20), ring(CHAMFER_R, FLANGE_T, 20))),
        "neck": np.vstack((ring(NECK_R, FLANGE_T - .0005, 16), ring(NECK_R, NECK_TOP, 16))),
        "taper": np.vstack((ring(NECK_R, NECK_TOP, 20), ring(body_r, TAPER_TOP, 20))),
        "body": np.vstack((ring(body_r, TAPER_TOP, BODY_SIDES, np.pi / BODY_SIDES),
                           ring(body_r, FUNNEL_H, BODY_SIDES, np.pi / BODY_SIDES))),
    }


def holder_pieces():
    """Holder frame. Flask, stopper and the base with its bevelled flange, as one rigid body."""
    z = JOINT_Z
    flange = np.vstack((ring(FLANGE_R, z, 20), ring(FLANGE_R, z - CHAMFER_Z, 20), ring(CHAMFER_R, z - FLANGE_T, 20)))
    return {
        "flask_foot": np.vstack((ring(FLASK_R - .004, 0., 24), ring(FLASK_R, .006, 24))),
        "flask_cone": np.vstack((ring(FLASK_R, .006, 24), ring(FLASK_NECK_R, FLASK_CONE_TOP, 24))),
        "flask_neck": np.vstack((ring(FLASK_NECK_R, FLASK_CONE_TOP, 16), ring(FLASK_NECK_R, FLASK_RIM, 16))),
        "stopper": np.vstack((ring(STOPPER[1], STOPPER_TOP - STOPPER[2], 16), ring(STOPPER[0], STOPPER_TOP, 16))),
        "cup": np.vstack((ring(CUP_BOTTOM_R, CUP_BOTTOM_Z, 16), ring(CHAMFER_R - .002, z - FLANGE_T, 16))),
        "flange": flange,
    }


def rot2(t):
    return np.array(((np.cos(t), -np.sin(t)), (np.sin(t), np.cos(t))))


def jaw_pieces(side):
    """Clamp frame, closed position (jaw angle 0). side +1: the jaw on the +y side. Per angular segment an
    upper and a lower convex prism whose inner faces are the V that bears on the flanges' bevels."""
    edges = np.linspace(ARC[0], ARC[1], SEGMENTS + 1)
    profile = [(LIP_R, FACE_Z_LIP), (ROOT_R, FACE_Z_ROOT), (OUT_R, FACE_Z_ROOT), (OUT_R, HALF_H), (LIP_R, HALF_H)]
    pieces = {}
    for k in range(SEGMENTS):
        for tag, sgn in (("u", 1), ("l", -1)):
            pts = [(r * np.cos(side * a), r * np.sin(side * a), sgn * zz) for a in (edges[k], edges[k + 1])
                   for r, zz in profile]
            pieces[f"seg{k}{tag}"] = np.asarray(pts)
    # the handle: at full opening it lies parallel to -x with its inner face 0.5 mm off the centre line
    y0, y1 = side * .0005, side * (.0005 + TAB_T)
    corners = np.asarray([(x, y) for x in (-TAB_START, -TAB_START - TAB_LEN) for y in (y0, y1)])
    corners = corners @ rot2(-side * THETA_MAX).T + PIVOT
    pieces["handle"] = np.asarray([(x, y, zz) for x, y in corners for zz in (-TAB_H / 2, TAB_H / 2)])
    return pieces


def knuckle_pieces():
    return {"pin": np.vstack((ring(.005, -HALF_H, 12) + (*PIVOT, 0.), ring(.005, HALF_H, 12) + (*PIVOT, 0.)))}


def jaw_points(side, opening):
    """All jaw vertices with the jaw opened by ``opening`` (rad) about the pin, clamp frame."""
    v = np.vstack(list(jaw_pieces(side).values()))
    xy = (v[:, :2] - PIVOT) @ rot2(side * opening).T + PIVOT
    return np.column_stack((xy, v[:, 2]))


def mouth_clearance(opening, samples=31):
    """Smallest clearance (m) between the jaws' V faces and the flange rim as the flange stack is moved from in
    front of the open mouth to the ring's centre (negative: they would touch)."""
    best = np.inf
    for side in (1, -1):
        for name, piece in jaw_pieces(side).items():
            if name == "handle":
                continue
            v = piece.copy()
            xy = (v[:, :2] - PIVOT) @ rot2(side * opening).T + PIVOT
            pts = [xy[i] * (1 - s) + xy[j] * s for i in range(len(xy)) for j in range(len(xy)) for s in np.linspace(0, 1, 5)]
            pts = np.asarray(pts)
            for cx in np.linspace(.12, 0., samples):
                best = min(best, float((np.hypot(pts[:, 0] - cx, pts[:, 1]) - FLANGE_R).min()))
    return best


# ---- poses -------------------------------------------------------------------------------------------------
def quat_to_matrix(q):
    x, y, z, w = q
    return np.array(((1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
                     (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
                     (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y))))


def rz(t):
    c, s = np.cos(t), np.sin(t)
    return np.array(((c, -s, 0.), (s, c, 0.), (0., 0., 1.)))


def tilt_deg(rotation):
    return float(np.degrees(np.arccos(np.clip(np.asarray(rotation)[2, 2], -1., 1.))))


def funnel_on_base(holder_pos, holder_rot, funnel_pos, funnel_rot):
    """Where the funnel sits on the base, measured in the holder's frame: horizontal offset of its axis from
    the base's (m), the gap between the ground faces at the funnel's lowest point (m), and the tilt between
    the two axes (deg)."""
    hr = np.asarray(holder_rot, dtype=float)
    p = hr.T @ (np.asarray(funnel_pos, dtype=float) - np.asarray(holder_pos, dtype=float))
    a = hr.T @ np.asarray(funnel_rot, dtype=float)[:, 2]
    tilt = float(np.degrees(np.arccos(np.clip(a[2], -1., 1.))))
    lowest = p[2] - FLANGE_R * np.sin(np.radians(tilt))
    return {"offset": float(np.hypot(p[0], p[1])), "offset_xy": p[:2].copy(), "gap": float(lowest - JOINT_Z),
            "tilt_deg": tilt}


def clamp_on_joint(holder_pos, holder_rot, clamp_pos, clamp_rot, jaw_open_deg):
    """The clamp ring's place on the joint, holder frame: centre off the axis (m), height off the joint plane
    (m), tilt (deg), and both jaws' opening (deg)."""
    hr = np.asarray(holder_rot, dtype=float)
    p = hr.T @ (np.asarray(clamp_pos, dtype=float) - np.asarray(holder_pos, dtype=float))
    tilt = float(np.degrees(np.arccos(np.clip((hr.T @ np.asarray(clamp_rot, dtype=float)[:, 2])[2], -1., 1.))))
    lo, hi = SEATED_DEG
    closed = all(lo <= float(v) <= hi for v in jaw_open_deg)
    return {"off_axis": float(np.hypot(p[0], p[1])), "height": float(p[2] - JOINT_Z), "tilt_deg": tilt,
            "jaws_closed": bool(closed), "jaw_open_deg": [float(v) for v in jaw_open_deg]}
