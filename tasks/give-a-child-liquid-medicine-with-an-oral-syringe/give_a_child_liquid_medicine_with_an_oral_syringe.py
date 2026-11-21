"""On the station table: a toddler care manikin's head (about 2 to 3 years old) on a short stand, upright, its face
toward the robot and its mouth open. Inside the mouth are the primary teeth on upper and lower gum ridges, the tongue,
the throat behind it, and the two inner cheek walls, which give outward a little when pressed (sprung). Between the
side teeth and each inner cheek is the cheek pocket (the buccal vestibule), 2 to 7.5 mm wide. To the robot's left a
5 mL oral dosing syringe, filled to the 5 mL line, lies on two rests in a small tray.

Task (instruction): give the child its 5 mL dose with the oral syringe, then put the syringe back on its rests. The
procedure (Great Ormond Street Hospital, "How to give your child liquid medicines"): "Put the tip of the oral syringe
inside your child's mouth between the gums and the inside surface of their cheek." "Gently push the plunger to squirt
small amounts of medicine into the side of your child's mouth." "Allow your child to swallow before you carry on
pushing the plunger." "Do not squirt the entire dose into your child's mouth in one go" ("they may choke").
ASHP SafeMedication (droppers or oral syringes): "Never squirt the liquid from the dropper or oral syringe directly
onto the back of a child's throat."; "Be sure to take breaks to allow your child to swallow."

Why two hands: a parallel gripper that holds the barrel cannot also push the plunger (a person does it with the thumb
of the holding hand). One hand keeps the syringe's tip in the cheek pocket, at a slant toward the cheek, while the
other pushes the plunger in small steps; the plunger's friction (0.8 N) is taken by the holding hand, not by the
child's cheek. The head rests on its stand (no hand is free to steady it, as when the child leans on a pillow).

World frame (m): x forward from the arm bases, y to the robot's left, z up; the table top is at z 0.7475.
Head frame (child_head.py): origin at the nasal root, x back from the face, y = world y, z up; the head is not
turned, so head axes are world axes. The nasal root is at (0.310, 0.000), 0.200 above the table; the middle of the
lips' opening is 0.150 above the table.
- mouth: lips opened 46 x 26 mm; upper and lower primary teeth (incisors, canines, first and second molars: the outer
  (cheek-side) faces of the side teeth 14 to 19.5 mm from the midline) on gum ridges; tongue; throat.
- inner cheeks: one plate per side (23.5 x 22 mm, x 3 to 26.5 mm behind the nasal root), its face 21.5 mm from the
  midline, on a sprung slide outward (0.06 N/mm, 0.08 N preload, 4 mm travel).
- oral syringe (5 mL): barrel 14 mm across, 12.0 mm bore, 58 mm long, a 3 mm cone and a 10 mm x 4.5 mm tip; finger
  flange 30 x 12 x 2.5 mm; the plunger, filled to 5 mL, stands 52 mm out of the barrel (44.2 mm of travel = 5 mL,
  0.113 mL per mm), its thumb disc 14 mm across; plunger friction 0.8 N; 17 g (with the dose) + 4 g.
- tray: 110 x 44 x 4 mm at (0.200, 0.120), with two notched rests (one under the barrel's front end, one under the
  flange); the syringe lies on them along x, tip forward, its axis 3.4 cm above the table.

Score (read every 50 ms step; the dose is the plunger's travel, 0.113 mL per mm, counted once beyond the farthest the
plunger has been):
- Each bit of plunger travel is booked where the tip is at that moment: "in the cheek pocket" when the tip is beside
  the side teeth (2 to 24 mm behind the nasal root, at least 0.5 mm farther out than the teeth's outer faces, within
  12 mm of the mouth's mid-height) AND it presses the inner cheek (that cheek pushed out at least 0.5 mm, i.e. at
  least 0.11 N); "toward the throat" when the tip is inside the row of teeth (more than 2 mm in from their outer
  faces) or more than 27 mm back (behind the last molar); anything else is spilled.
- A squirt is the plunger travel between pauses; a pause is 1 s in which the plunger moves less than 0.5 mm.
- A fault: any squirt of more than 1.5 mL, more than 0.2 mL toward the throat, or more than 0.3 mL spilled. A hazard:
  more than 1.5 mL toward the throat (choking).
- On its rests: the syringe released (no gripper within 4 cm of its middle), tip forward (its axis within 10 deg of
  the tray's long axis), its tip within 8 mm of its starting place across the table, and its barrel's axis 30.8 to
  36.8 mm above the table at both the tip and the flange (it lies at 33.8 mm in the two notches; lying on the tray
  or the table, or with one end off a rest, it is at most 15 mm up at one end), still for 1 s (under 1.5 mm).
- Score: partial credit is the dose given into the cheek pocket only: 0.8 x (mL in the pocket / 4.7), at most 0.8;
  water spilled or given toward the throat earns nothing. 1.0 (success, held 1 s) when at least 4.7 mL went into
  the pocket with no fault and the syringe is back on its rests. A fault caps the score at 0.5 for the rest of the
  run; the hazard sets 0 for the rest of the run.
"""

from __future__ import annotations

import itertools
import math
import os
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import child_head as ch  # noqa: E402
import manikin_face as mf  # noqa: E402
from bodykit import Body, BoxShape, CylShape, Joint, Link, Linkage, MeshShape, SphereShape  # noqa: E402
from bodykit import allow_matrix_grips, as_numpy, on_reset, on_step, rotation  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
MM = 0.001

SKIN = (0.90, 0.74, 0.63)
MUCOSA = (0.80, 0.42, 0.42)
CHEEK_C = (0.74, 0.36, 0.38)
GUM_C = (0.86, 0.52, 0.52)
TONGUE_C = (0.86, 0.50, 0.50)
TOOTH = (0.96, 0.95, 0.90)
EYE_WHITE, IRIS, PUPIL = (0.93, 0.92, 0.89), (0.30, 0.22, 0.15), (0.05, 0.04, 0.04)
HAIR = (0.35, 0.24, 0.15)
STAND_C = (0.22, 0.23, 0.26)
BARREL_C = (0.88, 0.91, 0.94)
MED_C = (0.85, 0.30, 0.35)
PLUNGER_C = (0.95, 0.95, 0.95)
STOPPER_C = (0.15, 0.15, 0.16)
MARK_C = (0.10, 0.10, 0.12)
TRAY_C = (0.30, 0.55, 0.75)


def _v(p) -> tuple:
    return tuple(float(x) for x in p)


def _quat(m: np.ndarray) -> tuple:
    from bodykit import _quat_from_matrix

    return _quat_from_matrix(np.asarray(m, float))


def unit(v) -> np.ndarray:
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


# -- the head ---------------------------------------------------------------------------------------------------------
HO = np.array([0.310, 0.000, TOP + 0.200])  # the head frame's origin (the nasal root), world
STAND_TOP = TOP + 0.030
ch.set_base(np.array([0.0, 0.0, 1.0]), float(STAND_TOP - HO[2]))


def hw(p_mm) -> np.ndarray:
    return HO + np.asarray(p_mm, float) * MM


def to_head_mm(p) -> np.ndarray:
    return (np.asarray(p, float) - HO) / MM


# primary teeth (head frame, mm): (x, |y|, half x, half y) of each crown; outer faces at |y| + half y
TEETH = [(3.0, 2.8, 1.8, 2.8), (5.2, 7.6, 2.0, 2.3), (8.5, 11.5, 2.5, 2.5), (14.5, 14.5, 3.5, 3.0),
         (22.5, 16.0, 4.5, 3.5)]
UPPER_CROWN, LOWER_CROWN = (-43.0, -37.0), (-63.0, -57.0)  # z ranges (mm): 14 mm between the front teeth
UPPER_GUM, LOWER_GUM = (-37.0, -31.0), (-69.0, -63.0)
GUM_PAD = 0.8  # the gum ridge is this much wider than the crowns
TONGUE_BOX = ((24.0, 0.0, -62.0), (14.0, 10.5, 5.0))
MOUTH_Z = -50.0
# the outer face of the row of side teeth, |y| (mm) against x (mm): where the cheek pocket begins
BUCCAL_X = [2.0, 4.8, 8.5, 11.0, 14.5, 18.0, 22.5, 27.0]
BUCCAL_Y = [5.6, 9.9, 14.0, 14.4, 17.5, 17.5, 19.5, 19.5]
# the inner cheeks
CHEEK_FACE = 21.5  # mm from the midline at rest
CHEEK_T = 3.0
CHEEK_X, CHEEK_Z = (3.0, 26.5), (-61.0, -39.0)
CHEEK_K, CHEEK_PRE_N, CHEEK_TRAVEL = 60.0, 0.08, 0.004  # N/m, N, m


def _box_mm(c, h, color, collide=True) -> BoxShape:
    return BoxShape(_v(np.asarray(c, float) * MM), _v(2 * np.asarray(h, float) * MM), color, collide=collide)


def _mouth_shapes() -> list:
    out = []
    for x, y, hx, hy in TEETH:
        for s in (1.0, -1.0):
            for crown, gum in ((UPPER_CROWN, UPPER_GUM), (LOWER_CROWN, LOWER_GUM)):
                out.append(_box_mm((x, s * y, sum(crown) / 2), (hx * 0.92, hy * 0.92, (crown[1] - crown[0]) / 2),
                                   TOOTH))
                out.append(_box_mm((x, s * y, sum(gum) / 2), (hx + GUM_PAD, hy + GUM_PAD, (gum[1] - gum[0]) / 2),
                                   GUM_C))
    c, h = TONGUE_BOX
    out.append(_box_mm(c, h, TONGUE_C))
    return out


def _face_details() -> list:
    out = []
    eye_r = 10.0 * MM
    for s in (1.0, -1.0):
        c = np.array([5.0 * MM + 0.5 * MM, s * 23.0 * MM, -3.0 * MM])
        out.append(SphereShape(_v(c), eye_r, EYE_WHITE, collide=False))
        out.append(CylShape(_v(c - np.array([eye_r - 0.4 * MM, 0, 0])), 5.0 * MM, 1.0 * MM, IRIS, collide=False,
                            axis="X"))
        out.append(CylShape(_v(c - np.array([eye_r - 0.1 * MM, 0, 0])), 2.2 * MM, 0.8 * MM, PUPIL, collide=False,
                            axis="X"))
        ep, ef, en = mf.ear_mesh(s, np.array([0.072, s * 0.062, -0.010]))
        ep = (ep - np.array([0.072, s * 0.062, -0.010])) * 0.75 + np.array([0.072, s * 0.060, -0.012])
        out.append(MeshShape(ep, ef, en, SKIN, collide=False))
    return out


def _head_shapes() -> list:
    pts, faces, normals, inner = ch.cached_mesh()
    return ([MeshShape(pts, faces[~inner], normals, SKIN), MeshShape(pts, faces[inner], normals, MUCOSA)]
            + _mouth_shapes() + _face_details())


HEAD = Body.build("head", pos=_v(HO), mass=1.5, shapes=_head_shapes(), color=SKIN, fixed=True,
                  friction=(0.5, 0.4), contact_offset=0.001)
STAND = Body.build("stand", pos=(float(HO[0] + 0.066), 0.0, TOP + 0.015), mass=2.0, fixed=True, color=STAND_C,
                   shapes=[CylShape((0.0, 0.0, 0.0), 0.050, 0.030, STAND_C)])


def _cheek(side: float) -> Body:
    c = np.array([sum(CHEEK_X) / 2, side * (CHEEK_FACE + CHEEK_T / 2), MOUTH_Z])
    h = np.array([(CHEEK_X[1] - CHEEK_X[0]) / 2, CHEEK_T / 2, (CHEEK_Z[1] - CHEEK_Z[0]) / 2])
    pre = CHEEK_PRE_N / CHEEK_K
    rng = (0.0, CHEEK_TRAVEL) if side > 0 else (-CHEEK_TRAVEL, 0.0)
    return Body.build(f"cheek_{'l' if side > 0 else 'r'}", pos=_v(HO), mass=0.02, color=CHEEK_C, gravity=False,
                      shapes=[_box_mm(c, h, CHEEK_C)],
                      joint=Joint(_v(hw(c)), "slide", "Y", axial=(CHEEK_K, 3.0, CHEEK_TRAVEL), slide_range=rng,
                                  slide_target=-side * pre),
                      friction=(0.5, 0.4), contact_offset=0.001, damping=(2.0, 2.0))


CHEEK_L, CHEEK_R = _cheek(1.0), _cheek(-1.0)  # world +y (the side the dose goes) and -y

# -- the oral syringe (a floating articulation: the barrel, and the plunger on a slide with friction) -------------------
# body frame: origin at the tip, the syringe's axis along +x (toward the tip), z up as it lies in the tray
TIP_L, TIP_R = 0.010, 0.00225
CONE_L, CONE_R = 0.003, 0.0045
BARREL_L, BARREL_R, BORE_R = 0.058, 0.007, 0.006
BARREL_FRONT = -(TIP_L + CONE_L)  # -0.013
BARREL_BACK = BARREL_FRONT - BARREL_L  # -0.071
FLANGE_T, FLANGE_W, FLANGE_H = 0.0025, 0.030, 0.012
STROKE = 0.0442  # 5 mL in a 12.0 mm bore
ML_PER_M = 5.0 / STROKE
STOPPER_L = 0.006
THUMB_R, THUMB_T = 0.007, 0.0025
INNER_FRONT = BARREL_FRONT - 0.001
STOP_FRONT0 = INNER_FRONT - STROKE  # the stopper's front at 5 mL
ROD_BACK0 = BARREL_BACK - FLANGE_T - STROKE - 0.008  # where the rod meets the thumb disc at 5 mL
THUMB_BACK0 = ROD_BACK0 - THUMB_T
PLUNGER_FRICTION = float(os.environ.get("OSYR_FRICTION", "0.8"))  # N
GRIP_AT = -0.046  # the holding grip's middle on the barrel (body x)

REST_H = 0.022  # the two rests hold the barrel's underside this high above the tray
SYR_TIP0 = np.array([0.255, 0.120, TOP + 0.004 + REST_H + BARREL_R + 0.0008])  # the tip as it lies on its rests
TRAY_XY = np.array([0.200, 0.120])


def _syringe() -> Linkage:
    o = SYR_TIP0
    def at(x, y=0.0, z=0.0):  # noqa: E306  (linkage shapes are placed relative to its pos, the tip)
        return _v(np.array([x, y, z]))
    barrel = [CylShape(at(-TIP_L / 2), TIP_R, TIP_L, BARREL_C, axis="X"),
              CylShape(at(-TIP_L - CONE_L / 2), CONE_R, CONE_L, BARREL_C, axis="X"),
              CylShape(at((BARREL_FRONT + BARREL_BACK) / 2), BARREL_R, BARREL_L, BARREL_C, axis="X"),
              BoxShape(at(BARREL_BACK - FLANGE_T / 2), (FLANGE_T, FLANGE_W, FLANGE_H), BARREL_C)]
    # the medicine seen through the barrel, and the scale's marks every 1 mL
    barrel.append(CylShape(at((INNER_FRONT + STOP_FRONT0) / 2), BARREL_R + 0.0002, STROKE - 0.001, MED_C,
                           collide=False, axis="X"))
    for k in range(6):
        barrel.append(CylShape(at(INNER_FRONT - k * STROKE / 5), BARREL_R + 0.0004, 0.0006, MARK_C, collide=False,
                               axis="X"))
    rod_len = (STOP_FRONT0 - STOPPER_L) - ROD_BACK0
    rod_c = (STOP_FRONT0 - STOPPER_L + ROD_BACK0) / 2
    plunger = [CylShape(at(STOP_FRONT0 - STOPPER_L / 2), BORE_R - 0.0003, STOPPER_L, STOPPER_C, collide=False,
                        axis="X"),
               BoxShape(at(rod_c), (rod_len, 0.009, 0.0016), PLUNGER_C),
               BoxShape(at(rod_c), (rod_len, 0.0016, 0.009), PLUNGER_C),
               CylShape(at(THUMB_BACK0 + THUMB_T / 2), THUMB_R, THUMB_T, PLUNGER_C, axis="X")]
    link = Link("plunger", tuple(plunger), 0.004, anchor=_v(o), axis="X", kind="prismatic",
                limits=(0.0, STROKE), friction=PLUNGER_FRICTION, damping=0.5)
    return Linkage.build("syringe", pos=_v(o), mass=0.017, shapes=barrel, links=[link], color=BARREL_C,
                         floating=True, friction=(0.7, 0.6), contact_offset=0.001, solver_iterations=32)


SYRINGE = _syringe()


def _tray() -> Body:
    sh = [BoxShape((0.0, 0.0, 0.002), (0.110, 0.044, 0.004), TRAY_C)]
    # two rests under the barrel's ends (8 mm long), each with a notch: two cheeks 3 mm thick either side of it
    # (the front one under the barrel's front end, the back one under the finger flange, 1 mm higher)
    for bx, ln, h, gap in ((BARREL_FRONT - 0.004, 0.008, REST_H, BARREL_R),
                           (BARREL_BACK - FLANGE_T / 2 - 0.0001, 0.0026, REST_H + 0.001, FLANGE_W / 2 + 0.0003)):
        x = float(SYR_TIP0[0] + bx - TRAY_XY[0])
        sh.append(BoxShape((x, 0.0, 0.004 + h / 2), (ln, 0.040, h), TRAY_C))
        for s_ in (1.0, -1.0):
            sh.append(BoxShape((x, s_ * (gap + 0.002), 0.004 + h + 0.002), (ln, 0.004, 0.004), TRAY_C))
    return Body.build("tray", pos=(float(TRAY_XY[0]), float(TRAY_XY[1]), TOP), mass=0.1, fixed=True, color=TRAY_C,
                      shapes=sh)


TRAY = _tray()

OBJECTS = [HEAD, STAND, CHEEK_L, CHEEK_R, TRAY, SYRINGE]

# ======================================================================================================================
# Tracking and scoring
# ======================================================================================================================
POCKET_X = (2.0, 24.0)  # mm behind the nasal root
POCKET_OUT = 0.5  # mm farther out than the teeth's outer faces
POCKET_Z = 12.0  # mm from the mouth's mid-height
PRESS_MM = 0.5  # the inner cheek pushed out at least this far (0.11 N)
THROAT_IN = 2.0  # mm inside the teeth's outer faces
THROAT_X = 27.0
PAUSE_STEPS, PAUSE_MM = 20, 0.5
BIG_SQUIRT_ML = 1.5
THROAT_ML, SPILL_ML = 0.2, 0.3
CHOKE_ML = 1.5  # more than this toward the throat: a choking hazard, 0 for the rest of the run
FULL_ML = 4.7
CLEAR = 0.040
REST_ANG, REST_XY, REST_DZ = 10.0, 0.008, 0.003  # on its rests: deg, m (tip across the table), m (axis height)
STILL_STEPS, STILL_MM = 20, 1.5
_DEBUG = os.environ.get("OSYR_DEBUG") == "1"
_DEBUG_EVERY = int(os.environ.get("OSYR_DEBUG_EVERY", "5"))
_EP: dict = {}
_IDS: dict = {}


def buccal_y(x_mm: float) -> float:
    return float(np.interp(x_mm, BUCCAL_X, BUCCAL_Y))


def classify_tip(tip_h: np.ndarray, pressed: bool) -> str:
    """'pocket', 'throat' or 'out' for the tip at head-frame point ``tip_h`` (mm)."""
    x, y, z = tip_h
    b = buccal_y(x)
    inside_mouth = x > POCKET_X[0] and abs(z - MOUTH_Z) < 20.0
    if inside_mouth and (x > THROAT_X or abs(y) < b - THROAT_IN):
        return "throat"
    if (POCKET_X[0] <= x <= POCKET_X[1] and abs(y) >= b + POCKET_OUT and abs(z - MOUTH_Z) <= POCKET_Z
            and pressed):
        return "pocket"
    return "out"


@on_reset
def _reset() -> None:
    _EP.pop("squirt_q0", None)
    _EP.update(tick=0, step=0, near_l=1.0, q_prev=None, hist=[], ml={"pocket": 0.0, "throat": 0.0, "out": 0.0}, squirt=0.0,
               squirts=[], big=0.0, where="out", tip_h=np.zeros(3), press=(0.0, 0.0), recent=[], tcp={}, gaps={},
               q=0.0, near=1.0, flat=False, low=False, rests=False)


_reset()


def _pose(base, name, origin):
    p = as_numpy(base.scene[name].data.root_link_pose_w)[0]
    return p[:3] - origin, rotation(p[3:7])


@on_step
def _track(env) -> None:
    try:
        _track_inner(env)
    except Exception:
        import traceback

        print("OSYR_TRACEBACK\n" + traceback.format_exc(), flush=True)
        raise


def _track_inner(env) -> None:
    from yam_arena import ik, station
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS

    _EP["tick"] += 1
    base = getattr(env, "unwrapped", env)
    origin = as_numpy(base.scene.env_origins[0])
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        if side not in _IDS:
            _IDS[side] = list(robot.find_joints(ARM_JOINTS, preserve_order=True)[0])
            _IDS[side + "_f"] = list(robot.find_joints(FINGER_JOINTS, preserve_order=True)[0])
        q = as_numpy(robot.data.joint_pos)[0]
        _tp = ik.tcp_pose(q[_IDS[side]], station.BASES[side])
        _EP["tcp"][side] = np.asarray(_tp[0], float)
        _EP.setdefault("tcpr", {})[side] = np.asarray(_tp[1], float)
        _EP["gaps"][side] = float(np.abs(q[_IDS[side + "_f"]]).sum())
    syr = base.scene["syringe"]
    if "plunger" not in _IDS:
        _IDS["plunger"] = list(syr.joint_names).index("plunger_turn")
    p = as_numpy(syr.data.root_link_pose_w)[0]
    sp, sr = p[:3] - origin, rotation(p[3:7])
    qj = float(as_numpy(syr.data.joint_pos)[0][_IDS["plunger"]])
    tip_h = to_head_mm(sp)  # the body origin is the tip
    cl, _ = _pose(base, "cheek_l", origin)
    cr, _ = _pose(base, "cheek_r", origin)
    press = ((cl[1] - HO[1]) / MM, -(cr[1] - HO[1]) / MM)  # outward displacement (mm) of each cheek from its body origin
    pressed = (press[0] if tip_h[1] > 0 else press[1]) >= PRESS_MM
    where = classify_tip(tip_h, pressed)
    if _EP["q_prev"] is None:
        _EP["q_prev"] = qj
    if qj > _EP["q_prev"]:  # the plunger beyond the farthest it has been: that much more medicine is out
        _EP["ml"][where] += (qj - _EP["q_prev"]) * ML_PER_M
        _EP["q_prev"] = qj
    hist = _EP["hist"]
    hist.append(qj)
    del hist[:-(PAUSE_STEPS + 1)]
    moving = len(hist) <= PAUSE_STEPS or (max(hist) - min(hist)) * 1000 >= PAUSE_MM
    if moving:  # a squirt: what has come out since the last pause
        if "squirt_q0" not in _EP:
            _EP["squirt_q0"] = _EP["q_prev"]
        _EP["squirt"] = (_EP["q_prev"] - _EP["squirt_q0"]) * ML_PER_M
    else:
        if _EP["squirt"] > 0.005:
            _EP["squirts"].append(round(_EP["squirt"], 2))
        _EP["squirt"] = 0.0
        _EP.pop("squirt_q0", None)
    _EP["big"] = max(_EP["big"], _EP["squirt"])
    axis = sr[:, 0]
    mid = sp + axis * ((BARREL_FRONT + BARREL_BACK) / 2)
    _EP["near"] = min(float(np.linalg.norm(_EP["tcp"][s] - mid)) for s in ("left", "right"))
    _EP["flat"] = abs(float(axis[2])) < math.sin(math.radians(20.0))
    _EP["low"] = float(mid[2]) < TOP + 0.050
    back = sp + axis * BARREL_BACK  # the barrel's axis at the flange
    _EP["rests"] = bool(float(axis[0]) > math.cos(math.radians(REST_ANG))
                        and float(np.linalg.norm(sp[:2] - SYR_TIP0[:2])) < REST_XY
                        and abs(float(sp[2] - SYR_TIP0[2])) < REST_DZ and abs(float(back[2] - SYR_TIP0[2])) < REST_DZ)
    _EP["near_l"] = float(np.linalg.norm(_EP["tcp"]["left"] - (sp + axis * GRIP_AT)))
    _EP["thumb"] = sp + axis * (THUMB_BACK0 + qj)
    _EP.update(where=where, tip_h=tip_h, press=press, q=qj, sp=sp)
    _servo_holder(sp, sr)
    _servo_pusher(sp, sr)


def _tally() -> dict:
    recent = _EP["recent"]
    if _EP.get("counted") != _EP["tick"] and "sp" in _EP:
        _EP["counted"] = _EP["tick"]
        recent.append(np.concatenate([_EP["sp"], [_EP["q"]]]))
        del recent[:-STILL_STEPS]
    still = len(recent) == STILL_STEPS and all(np.abs(p - recent[-1]).max() * 1000 < STILL_MM for p in recent)
    ml = _EP["ml"]
    fault = _EP["big"] > BIG_SQUIRT_ML or ml["throat"] > THROAT_ML or ml["out"] > SPILL_ML
    full = ml["pocket"] >= FULL_ML
    given = sum(ml.values()) >= FULL_ML
    choke = ml["throat"] > CHOKE_ML
    down = bool(_EP["rests"] and _EP["near"] >= CLEAR and still)
    return dict(still=still, fault=fault, full=full, given=given, choke=choke, down=down,
                done=bool(full and not fault and down))


def _debug(t) -> None:
    _EP["step"] += 1
    if (_EP["step"] - 1) % _DEBUG_EVERY:
        return
    ml = _EP["ml"]
    print(f"DBG step={_EP['step']} q={_EP['q'] * 1000:.1f}mm where={_EP['where']} tip_h="
          f"{np.round(_EP['tip_h'], 1).tolist()} press=({_EP['press'][0]:.2f},{_EP['press'][1]:.2f})mm "
          f"ml=pocket {ml['pocket']:.2f} throat {ml['throat']:.2f} out {ml['out']:.2f} squirt={_EP['squirt']:.2f} "
          f"squirts={_EP['squirts']} big={_EP['big']:.2f} fault={int(t['fault'])} flat={int(_EP['flat'])} "
          f"low={int(_EP['low'])} rests={int(_EP['rests'])} near={_EP['near'] * 1000:.0f}mm nearL={_EP['near_l'] * 1000:.0f}mm still={int(t['still'])} "
          f"thumb_vs_rtcp={np.round((_EP.get('thumb', np.zeros(3)) - _EP['tcp'].get('right', np.zeros(3))) * 1000, 1).tolist()} | "
          + " ".join(f"{k} tcp=({v[0]:.3f},{v[1]:.3f},{v[2] - TOP:.3f}) gap={_EP['gaps'][k] * 1000:.0f}"
                     for k, v in _EP["tcp"].items()), flush=True)


def score(objects) -> float:
    """0.8 x (mL in the cheek pocket / 4.7), at most 0.8; 1.0 is success() (the full dose in the pocket, no fault, the
    syringe back on its rests). A fault caps at 0.5; more than 1.5 mL toward the throat scores 0."""
    t = _tally()
    if _DEBUG:
        _debug(t)
    pocket = _EP["ml"]["pocket"]
    s = 0.8 * min(pocket, FULL_ML) / FULL_ML
    if t["full"] and t["down"]:
        s = 1.0
    if t["fault"]:
        s = min(s, 0.5)
    if t["choke"]:
        s = 0.0
    return round(s, 3)


def success(objects) -> bool:
    return bool(_tally()["done"])


# ======================================================================================================================
# Scripted demo (open loop, from the known layout)
# ======================================================================================================================
OPEN_C = -0.047
PINCH = float(os.environ.get("OSYR_PINCH", "0.0"))  # finger value on the 14 mm barrel (commanded shut: a firm squeeze)
MODE = os.environ.get("OSYR_MODE", "demo")  # demo | one | throat | spill | noreturn
if os.environ.get("OSYR_ONE") == "1":
    MODE = "one"  # no plunger hand
if os.environ.get("OSYR_THROAT") == "1":
    MODE = "throat"  # the tip aimed down the middle, toward the throat
if os.environ.get("OSYR_SPILL") == "1":
    MODE = "spill"  # the whole dose squirted outside the mouth (the tip against the outside of the cheek)
if os.environ.get("OSYR_NORETURN") == "1":
    MODE = "noreturn"  # the dose given right, then the syringe laid on the table beside the tray, not on its rests
TIP_TARGET = np.array([float(v) for v in os.environ.get("OSYR_TIP", "9,23,-52").split(",")])  # head mm
SPILL_TIP = np.array([float(v) for v in os.environ.get("OSYR_SPILL_TIP", "-8,36,-59").split(",")])  # head mm
NORETURN_Y = float(os.environ.get("OSYR_NORETURN_Y", "0.050"))  # m: the table beside the tray
YAW = math.radians(float(os.environ.get("OSYR_YAW", "36")))  # the syringe's slant toward the cheek
PITCH = math.radians(float(os.environ.get("OSYR_PITCH", "8")))  # tip lower than the plunger
HOLD_SIGN = float(os.environ.get("OSYR_HSIGN", "-1"))
PUSH_SIGN = float(os.environ.get("OSYR_PSIGN", "1"))
PUSH_LAG = float(os.environ.get("OSYR_LAG", "0.5")) * MM  # commanded past the plunger's place
SQUIRT_ML = float(os.environ.get("OSYR_SQUIRT", "1.0"))
HOLD_BACK = float(os.environ.get("OSYR_HOLD_BACK", "2.5")) * MM  # the push moves the held syringe this far in
PUSH_DZ = float(os.environ.get("OSYR_PUSH_DZ", "3")) * MM  # the arm sags this much when level: aim higher


def syringe_rot(yaw: float, pitch: float) -> np.ndarray:
    d = np.array([math.cos(yaw) * math.cos(pitch), math.sin(yaw) * math.cos(pitch), -math.sin(pitch)])
    z = unit(np.array([0.0, 0.0, 1.0]) - d[2] * d)
    return np.column_stack([d, np.cross(z, d), z])


# the holding grip: the TCP 6 mm above the barrel's axis (the fingertips clear of the tray)


LEAN = math.radians(float(os.environ.get("OSYR_LEAN", "40")))  # the holding hand leans back, away from the face


def _rot_axis(axis, a: float) -> np.ndarray:
    axis = unit(axis)
    k = np.array([[0.0, -axis[2], axis[1]], [axis[2], 0.0, -axis[0]], [-axis[1], axis[0], 0.0]])
    return np.eye(3) + math.sin(a) * k + (1 - math.cos(a)) * (k @ k)


def hold_pose(tip_w, rs, lean: float | None = None) -> tuple:
    """TCP position and rotation holding the barrel: fingers closing across it horizontally, the barrel along the
    pads, the hand above it leaning ``lean`` back (toward the plunger end) from straight down."""
    lean = LEAN if lean is None else lean
    d = rs[:, 0]
    c = HOLD_SIGN * unit(np.cross([0.0, 0.0, 1.0], d))  # horizontal, across the barrel
    down = _rot_axis(c, 0.0) @ unit(np.cross(c, d) * HOLD_SIGN)  # perpendicular to the barrel, downward
    if down[2] > 0:
        down = -down
    a = _rot_axis(c, lean) @ down
    if (a - down) @ d < 0:  # lean the other way: the hand's top goes back along -d
        a = _rot_axis(c, -lean) @ down
    rot = np.column_stack([c, np.cross(a, c), a])
    grip = np.asarray(tip_w) + d * GRIP_AT
    return grip - a * 0.006, rot


def push_pose(tip_w, rs, plunger_q: float) -> tuple:
    """The pushing hand: level along the syringe's axis, fingers closed, their tips on the thumb disc's back face
    with the plunger at ``plunger_q`` (m pushed in)."""
    d = rs[:, 0]
    x = unit(np.array([0.0, 0.0, 1.0]) - d[2] * d) * PUSH_SIGN  # fingers close up and down
    rot = np.column_stack([x, np.cross(d, x), d])
    thumb_back = np.asarray(tip_w) + d * (THUMB_BACK0 + plunger_q)
    return thumb_back - d * 0.0125 + np.array([0.0, 0.0, PUSH_DZ]), rot


def _good_ik(arm: str, pos, rot):
    from yam_arena import ik, station

    best = None
    b = station.BASES[arm]
    j1 = math.atan2(float(pos[1]) - b[1], float(pos[0]) - b[0])
    for sd in itertools.product((j1,), (0.8, 1.3), (0.6, 1.2), (-0.8, 0.8), (0.0,), (-1.8, -0.9, 0.0, 0.9, 1.8)):
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.asarray(sd, float),
                             base_pos=station.BASES[arm], iters=150)
        if pe < 0.001 and re < math.radians(0.5):
            room = float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())
            if best is None or room > best[1]:
                best = (q, room)
                if room > 0.3:
                    break
    if best is None:
        raise ValueError(f"no IK solution for the {arm} arm at {pos}")
    return best[0]


def _lead_in(arm: str, pos, rot, finger: float, label: str, seconds: float = 3.0, steps: int = 15) -> list:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    q0, q1 = np.array(READY, float), _good_ik(arm, pos, rot)
    out = []
    for k in range(1, steps + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / steps, station.BASES[arm])
        out.append(Waypoint(seconds / steps, tuple(float(v) for v in p), finger, label, r))
    return out


def _w(t, p, finger, label, rot) -> Waypoint:
    return Waypoint(float(t), tuple(float(v) for v in p), finger, label, np.asarray(rot, float))


def _stay(arm: str, t: float = 120.0) -> list:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    pos, rot = ik.tcp_pose(np.array(READY, float), station.BASES[arm])
    return [Waypoint(t, tuple(float(v) for v in pos), OPEN_C, "stay put", rot)]


def _dur(path) -> float:
    return float(sum(w.duration for w in path)) if path else 0.0


def _hold_until(path: list, t: float, label: str) -> None:
    last = path[-1]
    gap = t - _dur(path)
    if gap > 0:
        path.append(Waypoint(gap, last.pos, last.finger, label, last.yaw))


def mouth_pose():
    """(tip, syringe rotation) with the tip in the cheek pocket, or, for the wrong way, aimed down the middle."""
    if MODE == "throat":
        return hw([16.0, 6.0, MOUTH_Z]), syringe_rot(math.radians(22.0), PITCH)
    if MODE == "spill":
        return hw(SPILL_TIP), syringe_rot(YAW, PITCH)
    return hw(TIP_TARGET), syringe_rot(YAW, PITCH)


def _lw(t: float, tip, rs, finger: float, label: str) -> Waypoint:
    """A holding-hand waypoint given by where the syringe should be (its tip and rotation); once the syringe is lifted,
    the demo measures how it sits in the hand (as v24 allows) and re-aims these so that the syringe, not the hand,
    follows the plan."""
    g, gr = hold_pose(tip, rs)
    wp = _w(t, g, finger, label, gr)
    _shared()["left_tracked"].append((wp, np.asarray(tip, float).copy(), np.asarray(rs, float).copy()))
    return wp


def _left_path() -> tuple[list, float, float]:
    """Pick the syringe up from its tray, slide its tip into the cheek pocket and hold it there; returns the path,
    the time the tip is in place and the time the syringe is lifted (when its hold is measured)."""
    r0 = np.eye(3)
    tip0 = SYR_TIP0.copy()
    g0, gr0 = hold_pose(tip0, r0)
    up = np.array([0.0, 0.0, 1.0])
    out = _lead_in("left", g0 + up * 0.06, gr0, OPEN_C, "above the syringe")
    out += [_w(1.5, g0 + up * 0.002, OPEN_C, "down onto the barrel", gr0),
            _w(1.2, g0 + up * 0.002, PINCH, "close on the barrel", gr0),
            _w(0.6, g0 + up * 0.002, PINCH, "settle the grip", gr0),
            _w(1.5, g0 + up * 0.06, PINCH, "lift the syringe", gr0),
            _w(0.8, g0 + up * 0.06, PINCH, "hold still", gr0)]
    t_lift = _dur(out)
    tip_m, rs = mouth_pose()
    d = rs[:, 0]
    pre = tip_m - d * 0.018
    out.append(_lw(2.5, pre - d * 0.010 + up * 0.008, rs, PINCH, "turn it toward the corner of the mouth"))
    out.append(_lw(1.5, pre, rs, PINCH, "line the tip up with the corner of the mouth"))
    out.append(_lw(2.0, tip_m, rs, PINCH, "slide the tip in between the gums and the cheek"))
    out.append(_lw(1.0, tip_m, rs, PINCH, "hold it in the cheek pocket"))
    return out, _dur(out), t_lift


def _left_finish(path: list) -> None:
    tip_m, rs = mouth_pose()
    d = rs[:, 0]
    path.append(_lw(2.0, tip_m - d * 0.035, rs, PINCH, "draw the tip out of the mouth"))
    up = np.array([0.0, 0.0, 1.0])
    r0 = np.eye(3)
    if MODE == "noreturn":  # laid flat on the table beside the tray instead of on its rests
        t = np.array([SYR_TIP0[0], NORETURN_Y, TOP + BARREL_R + 0.007])
        path += [_lw(2.5, t + up * 0.06, r0, PINCH, "over the table beside the tray"),
                 _lw(1.5, t, r0, PINCH, "lay it down on the table"),
                 _lw(1.0, t, r0, OPEN_C, "let go"),
                 _lw(1.5, t + up * 0.08, r0, OPEN_C, "rise clear")]
        return
    path += [_lw(2.5, SYR_TIP0 + up * 0.06, r0, PINCH, "back over the tray"),
             _lw(1.5, SYR_TIP0 + up * 0.003, r0, PINCH, "lay it on its rests"),
             _lw(1.0, SYR_TIP0 + up * 0.003, r0, OPEN_C, "let go"),
             _lw(1.5, SYR_TIP0 + up * 0.08, r0, OPEN_C, "rise clear")]


def _right_path(t_start: float) -> tuple[list, float]:
    """Push the plunger in 1 mL squirts with pauses (or the wrong ways)."""
    tip_m, rs = mouth_pose()
    d = rs[:, 0]
    p_back, r = push_pose(tip_m, rs, -0.025)
    out = _stay("right", t_start - 2.0)
    out += _lead_in("right", p_back + np.array([-0.06, -0.06, 0.03]), r, 0.0, "round behind the syringe", seconds=3.0)
    tracked = _shared()["tracked"]
    def tw(t, q, label):  # noqa: E306  (a waypoint re-aimed at the syringe's measured pose while it runs)
        p, _ = push_pose(tip_m, rs, q)
        wp = _w(t, p, 0.0, label, r)
        tracked.append((wp, q))
        return wp
    out.append(tw(2.0, -0.025, "behind the plunger"))
    out.append(tw(1.5, 0.0, "fingertips onto the thumb disc"))
    out.append(tw(0.5, 0.0, "wait"))
    n = int(round(5.0 / SQUIRT_ML))
    for k in range(1, n + 1):
        q = min(STROKE, k * SQUIRT_ML / ML_PER_M)
        extra = 0.0
        out.append(tw(2.0, q + PUSH_LAG + extra, f"squirt {k}"))
        out.append(tw(2.5, q + PUSH_LAG + extra, f"pause {k}: let the child swallow"))
    out.append(tw(1.5, -0.03, "back off the plunger"))
    p_off = np.asarray(out[-1].pos)
    out.append(_w(1.5, p_off + np.array([-0.04, -0.03, 0.01]), 0.0, "move away", r))
    return out, _dur(out)


def _shared() -> dict:
    """State the scripted demo and the step hook share (kept on bodykit, imported once, in case the task file is
    imported twice)."""
    import bodykit

    if not hasattr(bodykit, "_osyr_shared"):
        bodykit._osyr_shared = {"tracked": [], "right": None, "left_tracked": [], "t_lift": None, "held": None}
    return bodykit._osyr_shared


SERVO = os.environ.get("OSYR_SERVO", "1") == "1"


def _servo_holder(sp, sr) -> None:
    """Once, just after the lift: measure how the syringe sits in the holding hand and re-aim the holding hand's
    later waypoints so that the syringe follows its planned poses."""
    sh = _shared()
    if not SERVO or sh["held"] is not None or sh["t_lift"] is None or _EP["tick"] * 0.05 < sh["t_lift"] - 0.1:
        return
    rt, pt = _EP["tcpr"]["left"], _EP["tcp"]["left"]
    a, b = sr.T @ rt, sr.T @ (pt - sp)
    sh["held"] = (a, b)
    for wp, tip, rs in sh["left_tracked"]:
        wp.pos = tuple(float(v) for v in np.asarray(tip) + rs @ b)
        wp.yaw = rs @ a


def _servo_pusher(sp, sr) -> None:
    """The scripted demo reads the syringe's pose (as v24 allows) and re-aims the pushing hand's remaining waypoints
    at the thumb disc where it really is (the syringe sits in the holding hand a little off the planned pose)."""
    sh = _shared()
    if not SERVO or not sh["tracked"] or sh["right"] is None or _EP["near_l"] > 0.03:
        return
    t = _EP["tick"] * 0.05
    elapsed, times = 0.0, {}
    for w in sh["right"]:
        times[id(w)] = elapsed
        elapsed += w.duration
    for wp, q in sh["tracked"]:
        if times.get(id(wp), 0.0) + wp.duration >= t:
            p, r = push_pose(sp, sr, q)
            p0, _ = push_pose(*mouth_pose(), q)
            if np.linalg.norm(p - p0) > 0.02 or p[2] < TOP + 0.14:  # not where the plan has it: keep the plan
                continue
            wp.pos = tuple(float(v) for v in p)
            wp.yaw = r


def demo():
    sh = _shared()
    sh.update(tracked=[], right=None, left_tracked=[], t_lift=None, held=None)
    out = _demo()
    sh["right"] = out["right"]
    return out


def _demo():
    left, t_in, t_lift = _left_path()
    _shared()["t_lift"] = t_lift
    if MODE == "one":
        tip_m, rs = mouth_pose()
        left.append(_lw(20.0, tip_m, rs, PINCH, "hold it in the cheek pocket (nobody pushes the plunger)"))
        _left_finish(left)
        left.append(_lw(4.0, SYR_TIP0 + np.array([0.0, 0.0, 0.08]), np.eye(3), OPEN_C, "stay"))
        return {"left": left, "right": _stay("right", _dur(left))}
    right, t_done = _right_path(t_in - 3.5)
    tip_m, rs = mouth_pose()
    left.append(_lw(1.0, tip_m - rs[:, 0] * HOLD_BACK, rs, PINCH, "brace against the plunger's push"))
    left.append(_lw(t_done - 2.5 - _dur(left), tip_m - rs[:, 0] * HOLD_BACK, rs, PINCH,
                    "hold it while the dose is given"))
    _left_finish(left)
    tend = max(_dur(left), _dur(right)) + 4.0
    park = np.array([SYR_TIP0[0], NORETURN_Y, TOP + BARREL_R + 0.007]) if MODE == "noreturn" else SYR_TIP0
    left.append(_lw(tend - _dur(left), park + np.array([0.0, 0.0, 0.08]), np.eye(3), OPEN_C, "stay"))
    _hold_until(right, tend, "stay")
    return {"left": left, "right": right}
