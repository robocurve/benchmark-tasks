"""Clipper-over-comb taper on a training head. A barber's training head on a stand faces away from the robot, so the
robot works on the back of the head, where twelve tufts of hair (three columns, four rows from nape to crown) all
start 35 mm long. A comb stands in a rack and a hair clipper lies on a holder in front of it.

Goal: taper the back: nape row 8 mm, then 12, 16 and 22 mm up to the crown row. The clipper has no guard. Run
straight over the hair it lays the hair flat and shaves it to the scalp where it touches; the length is set only by
holding the comb in the hair, teeth out, at the right distance from the scalp, and running the clipper along the
comb's back. The head is curved both across and up, so a comb held flat across all three columns leaves the side
columns about 6 mm long: the comb has to follow the head.

Offline check (no GPU):  python clipper_comb.py
"""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

import hingekit as hk  # noqa: E402
import yam_arena.demo as _demo  # noqa: E402
from yam_arena import ik  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = ("Taper the back of the training head with the clipper over the comb: the bottom row of hair 8 mm, "
               "then 12 mm, 16 mm and 22 mm up to the top row. Then put the comb and the clipper back where they "
               "were.")

# -- the head (m) -------------------------------------------------------------------------------------------------
HX = float(os.environ.get("CC_HX", "0.40"))  # head axis, forward of the arm-base line; the back of the head faces the robot (-x)
# horizontal slices of the head, bottom to top: (z0, z1, radius). The middle four carry the hair rows.
HZ = float(os.environ.get("CC_HZ", "-0.06"))
HY = float(os.environ.get("CC_HY", "0.0"))  # head axis, sideways (+ left)
# horizontal slices of the head, bottom to top: (z0, z1, radius). Slices 1-4 carry the four hair rows.
_R = (0.082, 0.086, 0.089, 0.088)
SLICES = [(0.088 + HZ, 0.100 + HZ, 0.072)] + [(0.100 + 0.025 * k + HZ, 0.125 + 0.025 * k + HZ, _R[k]) for k in range(4)] + \
    [(0.200 + HZ, 0.222 + HZ, 0.080), (0.222 + HZ, 0.240 + HZ, 0.062)]
NECK = (0.035, 0.0, 0.088 + HZ)  # neck radius, from table up to the head
ROWS = [(0.1125 + 0.025 * k + HZ, _R[k]) for k in range(4)]  # (height above table, scalp radius)
COLS_DEG = (-20.0, 0.0, 20.0)  # azimuth round the back of the head; + is toward the robot's left
TARGET = (0.008, 0.012, 0.016, 0.022)  # per row, nape to crown
L0 = 0.035  # every tuft starts 35 mm
TOL_SHORT, TOL_LONG = 0.002, 0.003  # a tuft is right from 2 mm short to 3 mm long of its target
GOUGE = 0.005  # any tuft under 5 mm is a bald patch: the cut can't be undone
FLAT_REACH = 0.006  # with no comb in the hair, the clipper shaves what it passes within 6 mm of the scalp
EAR_AZ = 80.0  

EAR_Z = (0.125 + HZ, 0.175 + HZ)  # ears either side
SKIN = (0.86, 0.70, 0.58)
HAIR = (0.25, 0.16, 0.09)
SCALP = (0.20, 0.13, 0.08)  # the head's short hair
DARK = (0.16, 0.16, 0.18)
STEEL = (0.78, 0.78, 0.80)
MAT = (0.28, 0.30, 0.32)
COMB_C = (0.12, 0.12, 0.13)

kit = hk.Kit()
_head = [hk.cyl(NECK[0], NECK[2], at=(0, 0, NECK[2] / 2), color=SKIN)]
_head += [hk.cyl(r, z1 - z0, at=(0, 0, (z0 + z1) / 2), color=SCALP) for z0, z1, r in SLICES]  # short hair all over
_head.append(hk.box((0.010, 0.100, 0.115), at=(0.083, 0.0, SLICES[1][0] + 0.0575), color=SKIN))  # the face, in front
for sgn in (1, -1):  # ears: flat pads on the sides
    _head.append(hk.box((0.030, 0.008, EAR_Z[1] - EAR_Z[0]), at=(0.0, sgn * 0.094, sum(EAR_Z) / 2), color=SKIN))
_head.append(hk.cyl(0.07, 0.012, at=(0, 0, 0.006), color=DARK))  # stand foot
kit.part("head", origin=(HX, HY, TOP), color=SKIN, shapes=_head)
# clipper: a body with the blade face at local +x; part frame at the body's centre
CLIP_L, CLIP_W, CLIP_H = 0.100, 0.045, 0.025
BLADE_X = CLIP_L / 2 + 0.002  # the cutting face
HOLDER_H = float(os.environ.get("CC_HH", "0.100"))  # the clipper lies on a holder block
HOLDER_W = 0.060
CLIP_AT = np.array([float(os.environ.get("CC_CX", "0.20")), float(os.environ.get("CC_CY", "-0.23")), TOP + 0.003 + HOLDER_H + CLIP_H / 2])
# comb: a flat toothed plate with a spine along its top edge, standing in a slotted rack. Part frame: the plate's
# centre on its inner face (the teeth's tips); local +x points into the hair, y along the comb, z up.
COMB_T, COMB_W, COMB_L = 0.003, float(os.environ.get("CC_CW", "0.030")), 0.120
HANDLE = (0.010, 0.080, COMB_W)  # a thicker handle beyond the teeth, toward the robot's left
HANDLE_Y = COMB_L / 2 + HANDLE[1] / 2
RACK_H = float(os.environ.get("CC_RACK", "0.006"))
RACK_SLOT = 0.011  # the slot holds the 1 cm handle within about 10 degrees of upright
COMB_AT = np.array([0.20, 0.13, TOP + 0.003 + RACK_H - 0.005 + COMB_W / 2])  # 5 mm down in the rack's slot
_GREY = (0.35, 0.37, 0.40)
MAT_X = 0.20
kit.part("mat", origin=(MAT_X, 0.0, TOP), color=MAT, shapes=[
    hk.box((0.14, 0.56, 0.003), at=(0, 0, 0.0015)),
    hk.box((0.080, HOLDER_W, HOLDER_H), at=(CLIP_AT[0] - MAT_X, CLIP_AT[1], 0.003 + HOLDER_H / 2), color=_GREY),
    *[hk.box((0.016, COMB_L + HANDLE[1] + 0.02, RACK_H), at=(COMB_AT[0] - MAT_X - COMB_T / 2 + sg * (RACK_SLOT / 2 + 0.008), COMB_AT[1] + HANDLE[1] / 2,
                                                  0.003 + RACK_H / 2), color=_GREY) for sg in (-1, 1)],  # comb rack
])
kit.part("comb", origin=tuple(COMB_AT), mass=0.03, color=COMB_C, shapes=[
    hk.box((COMB_T, COMB_L, COMB_W), at=(-COMB_T / 2, 0, 0)),
    hk.box(HANDLE, at=(-COMB_T / 2, HANDLE_Y, 0)),
])
kit.part("clipper", origin=tuple(CLIP_AT), mass=0.20, color=DARK, shapes=[
    hk.box((CLIP_L, CLIP_W, CLIP_H), at=(0, 0, 0)),
    hk.box((0.004, CLIP_W, CLIP_H * 0.8), at=(CLIP_L / 2, 0, 0), color=STEEL),
])


def _u(az_deg):
    a = math.radians(az_deg)
    return np.array([-math.cos(a), math.sin(a), 0.0])


STRANDS = ((0.0, 0.0), (0.0022, 0.0012), (-0.0022, 0.0012), (0.0012, -0.0022), (-0.0012, -0.0022))
DROOP_DEG = 55.0  # a tuft the comb isn't holding is drawn hanging this far down from straight out
DROOP_RATE = 12.0  # degrees per control step it swings toward hanging or standing
TUFTS = []  # (name, row, col, scalp point, outward unit)
for i, (z, r) in enumerate(ROWS):
    for j, az in enumerate(COLS_DEG):
        u = _u(az)
        base = np.array([HX, HY, TOP + z]) + r * u
        name = f"tuft_{i}{j}"
        TUFTS.append((name, i, j, base, u))
        kit.part(name, origin=tuple(base), color=HAIR, collide=False,  # a lock of five strands, 1.5 mm each
                 shapes=[hk.box((L0, 0.0015, 0.0015), at=(L0 / 2, dy, dz)) for dy, dz in STRANDS])
PB_NOCOLLIDE = [t[0] for t in TUFTS]  # hair is drawn, not simulated: nothing collides with it

OBJECTS = kit.objects()


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def tuft_pose(k, length, droop_deg=0.0):
    """The lock drawn from its root on the scalp, `droop_deg` below straight out, its tip `length` from the root
    (the rest of each strand hidden inside the head)."""
    name, i, j, base, u = TUFTS[k]
    a = math.radians(droop_deg)
    d = math.cos(a) * u - math.sin(a) * np.array([0.0, 0.0, 1.0])
    side = np.cross(np.array([0.0, 0.0, 1.0]), u)
    z = np.cross(d, side)
    return hk.T(np.column_stack([d, side, z]), base + d * (length - L0))


# -- the cut ------------------------------------------------------------------------------------------------------
CONTROL_HZ = 20.0
DEBUG = os.environ.get("CC_DEBUG") == "1"
MEM: dict = {}


def _reset() -> None:
    MEM.clear()
    MEM.update(t=0.0, length=[L0] * len(TUFTS), ear=False, cuts=0, droop=[DROOP_DEG] * len(TUFTS))


_reset()
kit.on_reset(_reset)


def _cross(frame, origin_local_n, half_a, half_b, axes, base, u):
    """Where the tuft line base + s*u crosses a face of a part: the face is the plane through local point with
    local normal axes[0], bounded by +-half_a along axes[1] and +-half_b along axes[2]. Returns s, or None."""
    R, p = frame[:3, :3], frame[:3, 3]
    n = R[:, axes[0]]
    c = R @ origin_local_n + p
    den = float(n @ u)
    if abs(den) < 0.3:  # the face must be roughly across the hair
        return None
    s = float(n @ (c - base)) / den
    if s < -0.002 or s > L0 + 0.02:
        return None
    q = base + s * u - c
    if abs(float(R[:, axes[1]] @ q)) > half_a or abs(float(R[:, axes[2]] @ q)) > half_b:
        return None
    return s


EAR_CLEAR = 0.010
EAR_POINTS = np.array([[HX + dx, HY + sgn * (0.094 + dy), TOP + sum(EAR_Z) / 2 + dz]
                       for sgn in (1, -1) for dx in np.linspace(-0.015, 0.015, 4) for dy in (-0.004, 0.004)
                       for dz in np.linspace(-(EAR_Z[1] - EAR_Z[0]) / 2, (EAR_Z[1] - EAR_Z[0]) / 2, 6)])  # pad surface


def monitor(objects, grippers=None) -> None:
    MEM["t"] += 1.0 / CONTROL_HZ
    if grippers is not None:
        MEM["tips"] = [[round(float(v), 3) for v in g] for g in grippers]
    if DEBUG and abs(MEM["t"] / 2.0 - round(MEM["t"] / 2.0)) < 1e-6:  # every 2 s in Isaac
        fc_, fk_ = kit.frame(objects, "clipper"), kit.frame(objects, "comb")
        tc = [[round(float(v), 3) for v in g] for g in hk.tcps()]
        print(f"[clipper_comb dbg] t={MEM['t']:.1f} clip={np.round(fc_[:3, 3], 3).tolist()} "
              f"comb={np.round(fk_[:3, 3], 3).tolist()} comb_up={fk_[2, 2]:.2f} tcps={tc} "
              f"len={[round(1000 * v, 1) for v in MEM['length']]} ear={MEM['ear']}", flush=True)
    fc, fk = kit.frame(objects, "clipper"), kit.frame(objects, "comb")
    for k, (name, i, j, base, u) in enumerate(TUFTS):  # display: a lock stands out while the comb is in it
        s_in = _cross(fk, np.zeros(3), COMB_L / 2, COMB_W / 2, (0, 1, 2), base, u)
        goal = 0.0 if s_in is not None and s_in < MEM["length"][k] else DROOP_DEG
        MEM["droop"][k] += float(np.clip(goal - MEM["droop"][k], -DROOP_RATE, DROOP_RATE))
    for k, (name, i, j, base, u) in enumerate(TUFTS):
        L = MEM["length"][k]
        s_blade = _cross(fc, np.array([BLADE_X, 0, 0]), CLIP_W / 2, CLIP_H / 2 * 0.8, (0, 1, 2), base, u)
        if s_blade is None or s_blade >= L:
            continue
        s_comb = _cross(fk, np.zeros(3), COMB_L / 2, COMB_W / 2, (0, 1, 2), base, u)
        if s_comb is not None and s_comb < s_blade + COMB_T + 0.002:  # combed out: the clipper cuts what stands
            new = max(s_blade, s_comb + COMB_T)  # past the comb; the teeth shield the hair inside them
        elif s_blade <= FLAT_REACH:  # no comb: the hair lies flat and is shaved where the blade passes
            new = 0.001
        else:
            continue
        if new < L - 1e-4:
            MEM["length"][k] = new
            MEM["cuts"] += 1
    boxes = {"clipper": (np.zeros(3), np.array([CLIP_L, CLIP_W, CLIP_H]) / 2),
             "comb": (np.array([0.0, HANDLE[1] / 2, 0.0]), np.array([HANDLE[0], COMB_L + HANDLE[1], COMB_W]) / 2)}
    for part, (c, half) in boxes.items():  # a tool within 1 cm of an ear pad (any point of it) touches it
        t = kit.frame(objects, part)
        loc = (EAR_POINTS - t[:3, 3]) @ t[:3, :3] - c
        if float(np.min(np.linalg.norm(np.maximum(np.abs(loc) - half, 0), axis=1))) < EAR_CLEAR:
            if not MEM["ear"]:
                MEM["ear_t"] = (round(MEM["t"], 2), part)
            MEM["ear"] = True


kit.on_step(monitor)


def kinematic_poses(angles: dict | None = None) -> dict:
    """Each tuft drawn at its current length, hanging down unless the comb holds it out (display only)."""
    return {TUFTS[k][0]: tuft_pose(k, MEM["length"][k], MEM["droop"][k]) for k in range(len(TUFTS))}


kit.on_kinematic(kinematic_poses)


def right_count():
    n = 0
    for k, (name, i, j, base, u) in enumerate(TUFTS):
        d = MEM["length"][k] - TARGET[i]
        n += -TOL_SHORT <= d <= TOL_LONG
    return n


def on_mat(objects, part) -> bool:
    """A tool put back: put down where it started. The comb resting in or beside its rack (centre within
    3 cm of its start, sideways, no more than 1 cm higher); the clipper lying flat (within 20 degrees) on its holder
    or beside it (centre within 6 cm of its start, sideways, no more than 1 cm higher)."""
    t = kit.frame(objects, part)
    if part == "comb":
        return float(np.linalg.norm(t[:2, 3] - COMB_AT[:2])) < 0.03 and float(t[2, 3]) < COMB_AT[2] + 0.01
    flat = float(t[2, 2]) >= math.cos(math.radians(20))
    return float(np.linalg.norm(t[:2, 3] - CLIP_AT[:2])) < 0.06 and float(t[2, 3]) < CLIP_AT[2] + 0.01 and flat


CUT_BY = 0.001  # a tuft counts as cut once it is at least 1 mm shorter than it started
LONG_CREDIT = 0.25  # a tuft cut but still too long: the technique worked and it can still be trimmed


def tuft_credit(k: int) -> float:
    """1 if the tuft is right (2 mm short to 3 mm long of its row's length); 0.25 if it was cut but is still too
    long; 0 if it is uncut or too short."""
    length, target = MEM["length"][k], TARGET[TUFTS[k][1]]
    d = length - target
    if -TOL_SHORT <= d <= TOL_LONG:
        return 1.0
    if d > TOL_LONG and length <= L0 - CUT_BY:
        return LONG_CREDIT
    return 0.0


def score(objects) -> float:
    """0 once any tuft is under 5 mm (a bald patch) or a tool has touched an ear. Otherwise 1.0 when all 12 tufts
    are right and both tools are back; else 0.9 x the average tuft credit (0 to 0.9, rounded to 0.001)."""
    if MEM["ear"] or min(MEM["length"]) < GOUGE:
        return 0.0
    credits = [tuft_credit(k) for k in range(len(TUFTS))]
    if min(credits) == 1.0 and on_mat(objects, "comb") and on_mat(objects, "clipper"):
        return 1.0
    return round(0.9 * sum(credits) / len(credits), 3)


def success(objects) -> bool:
    return score(objects) >= 0.999


def distance(objects) -> float:
    return float(12 - right_count())


def log_state(objects) -> dict:
    fc, fk = kit.frame(objects, "clipper"), kit.frame(objects, "comb")
    return {"len_mm": [round(1000 * v, 1) for v in MEM["length"]], "right": right_count(), "ear": MEM["ear"],
            "cuts": MEM["cuts"], "clip": [round(float(v), 3) for v in fc[:3, 3]],
            "comb": [round(float(v), 3) for v in fk[:3, 3]], "comb_up": round(float(fk[2, 2]), 3),
            "back": [on_mat(objects, "comb"), on_mat(objects, "clipper")], "ear_t": MEM.get("ear_t"), "tips": MEM.get("tips")}


# -- scripted demo ------------------------------------------------------------------------------------------------
def _target(self, t):
    prev_pos, prev_rot, prev_finger = self.start_pos, self.start_rot, self.finger_start
    for w in self.waypoints:
        goal = np.asarray(w.pos, dtype=float)
        rot = getattr(w, "rot", None)
        rot = grip_rotation(w.yaw) if rot is None else np.asarray(rot, float)
        if t < w.duration:
            s = ik.smoothstep(t / w.duration) if getattr(w, "smooth", True) else t / w.duration
            return prev_pos + s * (goal - prev_pos), ik.slerp_matrix(prev_rot, rot, s), \
                prev_finger + s * (w.finger - prev_finger), w.label
        t -= w.duration
        prev_pos, prev_rot, prev_finger = goal, rot, w.finger
    return prev_pos, prev_rot, prev_finger, "done"


_demo.ArmPlan.target = _target


class RotWaypoint(Waypoint):
    def __init__(self, duration, pos, finger, label="", rot=None, smooth=True):
        super().__init__(duration, tuple(float(v) for v in pos), finger, label, 0.0)
        object.__setattr__(self, "rot", None if rot is None else np.asarray(rot, float))
        object.__setattr__(self, "smooth", smooth)


OPEN, SHUT = -0.040, 0.0
# grasps, as the TCP pose in the part frame (top-down on the mat)
_RX = (lambda a: np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]]))
COMB_GRASP = hk.T(_RX(math.radians(-20)) @ grip_rotation(-math.pi / 2), (-COMB_T / 2, float(os.environ.get("CC_GY", "0.069")), COMB_W / 2 - float(os.environ.get("CC_GZ", "0.006"))))  # onto the handle by the teeth, over the centre of mass, tilted 20 degrees in from the left
CLIP_GRASP = hk.T(grip_rotation(0.0), (float(os.environ.get("CC_CGX", "0.0")), 0.0, float(os.environ.get("CC_CGZ", "0.008"))))
# Where the clipper actually sits relative to where the hand is told to go, once gripped: in Isaac it ends up 2.2 cm
# to one side (along the jaws) and 5 mm lower than a centred grip would put it, the same every run (fitted from two
# runs: the offset moved one-for-one with the hand). The demo aims with this, and puts the clipper back with it.
CLIP_HOLD = hk.T(grip_rotation(0.0), (0.0, float(os.environ.get("CC_HOLD_DY", "0.022")),
                                     float(os.environ.get("CC_HOLD_DZ", "0.013"))))  # down onto its middle
SLIDE_END = float(os.environ.get("CC_SE", "-0.008"))  # the clipper runs from 4.5 cm before the tuft to 8 mm short of centring on it (its blade is 4.5 cm wide, so the tuft is well inside the blade by then)
CLIP_OUT = 0.05  # the clipper waits this far out while the comb moves
COMB_OUT = float(os.environ.get("CC_COMB_OUT", "-0.0005"))  # the comb's back (outer face) this far past the row's length
FACE = float(os.environ.get("CC_FACE", "0.0005"))
FACE_TILT = float(os.environ.get("CC_FACE_TILT", "0.0015"))  # the clipper's blade face this far past the row's length
CLEAR = float(os.environ.get("CC_CLEAR", "0.012"))  # stand-off when coming in and going out


COMB_AT_TUFT = 0.025  # the tuft goes through the teeth 2.5 cm from their middle, toward the free end


def comb_pose(i, j, standoff):
    """The comb in the hair at tuft (row i, column j): standing, tangent to the head there, the teeth's tips
    `standoff` from the scalp, across the head."""
    name, _, _, base, u = TUFTS[3 * i + j]
    R = _rz(-math.radians(COLS_DEG[j]))
    return hk.T(R, base + u * standoff + R @ np.array([0.0, COMB_AT_TUFT, 0.0]))


def clip_pose(i, j, face_out, slide):
    """The clipper with its blade face `face_out` from the scalp, across tuft (i, j), slid `slide` along the comb."""
    name, _, _, base, u = TUFTS[3 * i + j]
    az = math.radians(COLS_DEG[j])
    R = _rz(-az)  # local x -> -u (blade face toward the head)
    along = R @ np.array([0.0, 1.0, 0.0])
    return hk.T(R, base + u * face_out - (R @ np.array([BLADE_X, 0, 0])) + along * slide)


def tcp(part_pose, grasp):
    t = part_pose @ grasp
    return t[:3, 3], t[:3, :3]


def demo():
    """Both arms pick up their tool; then for each column, bottom row to top, the left arm sets the comb in the hair
    at the row's stand-off and the right runs the clipper along it across the tuft; then both put their tools back."""
    left, right = [], []

    def both(dur, lpos, lrot, lf, rpos, rrot, rf, label, smooth=True):
        left.append(RotWaypoint(dur, lpos, lf, label, lrot, smooth))
        right.append(RotWaypoint(dur, rpos, rf, label, rrot, smooth))

    c0 = hk.T(None, COMB_AT)
    k0 = hk.T(None, CLIP_AT)
    lp, lr = tcp(c0, COMB_GRASP)
    rp, rr = tcp(k0, CLIP_GRASP)
    rh, rhr = tcp(k0, CLIP_HOLD)  # the hand that puts the clipper back exactly on its holder
    up = np.array([0, 0, 0.08])
    back = up
    both(2.0, lp + up, lr, OPEN, rp + back, rr, OPEN, "to the tools")
    both(1.2, lp, lr, OPEN, rp, rr, OPEN, "onto them")
    both(0.8, lp, lr, SHUT, rp, rr, SHUT, "grip")
    both(1.2, lp + up, lr, SHUT, rp + up, rr, SHUT, "lift")
    order = [(i, j) for j in (1, 0, 2) for i in range(4)]
    for n, (i, j) in enumerate(order):
        s = TARGET[i] + COMB_OUT - COMB_T  # the comb's inner face (teeth tips) from the scalp
        # the clipper's blade face from the scalp; the in-hand offset leaves it a little further out on the right-hand
        # column and a little closer on the left (Isaac), so the demo leans it in or out by column
        face = TARGET[i] + FACE + FACE_TILT * COLS_DEG[j] / 20.0
        cp_in = comb_pose(i, j, s + CLEAR)
        cp = comb_pose(i, j, s)
        lpi, lri = tcp(cp_in, COMB_GRASP)
        lpc, lrc = tcp(cp, COMB_GRASP)
        kp_out = clip_pose(i, j, face + CLEAR, -0.045)
        kp_a = clip_pose(i, j, face, -0.045)
        kp_b = clip_pose(i, j, face, SLIDE_END)
        kp_b_out = clip_pose(i, j, face + CLEAR, SLIDE_END)
        rpo, rro = tcp(kp_out, CLIP_HOLD)
        rpa, rra = tcp(kp_a, CLIP_HOLD)
        rpb, rrb = tcp(kp_b, CLIP_HOLD)
        rpbo, rrbo = tcp(kp_b_out, CLIP_HOLD)
        tag = f"row {i + 1} col {j + 1}"
        kp_far = clip_pose(i, j, face + CLIP_OUT, -0.045)
        kp_far_b = clip_pose(i, j, face + CLIP_OUT, SLIDE_END)
        rpf, rrf = tcp(kp_far, CLIP_HOLD)
        rpfb, rrfb = tcp(kp_far_b, CLIP_HOLD)
        both(2.5 if n == 0 else 1.2, lpi, lri, SHUT, rpf, rrf, SHUT, f"{tag}: comb comes in")
        both(1.0, lpc, lrc, SHUT, rpf, rrf, SHUT, f"{tag}: comb into the hair")
        both(0.8, lpc, lrc, SHUT, rpo, rro, SHUT, f"{tag}: clipper comes in")
        both(1.0, lpc, lrc, SHUT, rpa, rra, SHUT, f"{tag}: clipper onto the comb")
        both(0.8, lpc, lrc, SHUT, rpa, rra, SHUT, f"{tag}: settle")
        both(3.0, lpc, lrc, SHUT, rpb, rrb, SHUT, f"{tag}: run the clipper along the comb", smooth=False)
        both(0.6, lpc, lrc, SHUT, rpbo, rrbo, SHUT, f"{tag}: clipper off")
        both(0.8, lpc, lrc, SHUT, rpfb, rrfb, SHUT, f"{tag}: clipper back")
        both(0.6, lpi, lri, SHUT, rpfb, rrfb, SHUT, f"{tag}: comb out")
    both(2.5, lp + up, lr, SHUT, rh + up, rhr, SHUT, "back over the mat")
    both(1.0, lp + [0, 0, 0.03], lr, SHUT, rh + [0, 0, 0.03], rhr, SHUT, "line up over the rack and holder")
    both(1.0, lp + [0, 0, 0.03], lr, SHUT, rh + [0, 0, 0.03], rhr, SHUT, "hold")
    seat = np.array([0, 0, float(os.environ.get("CC_SEAT", "-0.003"))])  # aim a little low: the comb seats fully
    rseat = np.array([0, 0, float(os.environ.get("CC_CSEAT", "-0.004"))])  # aim a little low: it rests on the holder
    both(1.5, lp + seat, lr, SHUT, rh + rseat, rhr, SHUT, "set down")
    both(1.5, lp + seat, lr, SHUT, rh + rseat, rhr, SHUT, "settle")
    both(0.8, lp + seat, lr, OPEN, rh + rseat, rhr, OPEN, "let go")
    both(1.2, lp + up, lr, OPEN, rh + back, rhr, OPEN, "away")
    return {"left": left, "right": right}




def _selftest() -> None:
    from yam_arena import station
    # rule check: the scripted geometry cuts tuft 00 to its target; a flat-across comb leaves the side long
    for side, way in demo().items():
        plan = _demo.ArmPlan(way, station.BASES[side])
        t = 0.0
        while t < plan.duration:
            plan.action7(t)
            t += 1 / CONTROL_HZ
        print(f"{side}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.2f} mm / {plan.worst_ik_deg:.2f} deg")
    # a straight comb across the head at the middle column's pose: what would the side tufts get?
    for i in range(4):
        r = ROWS[i][1]
        s = TARGET[i]
        side = (r + s) / math.cos(math.radians(20)) - r
        print(f"row {i + 1}: target {1000 * s:.0f} mm; flat comb leaves the side columns {1000 * side:.1f} mm")


if __name__ == "__main__":
    _selftest()

# pybullet preview only: weld each tool to its gripper once gripped (tests the reach, the comb's placement and the
# cut rule, not the grasp; Isaac runs the real grasp), and release it at the end.
PB_ATTACH = {"left": ("comb", "lift"), "right": ("clipper", "lift")}
PB_RELEASE = {"left": "let go", "right": "let go"}
PB_GRASP = {"left": COMB_GRASP, "right": CLIP_HOLD}  # the preview puts the clipper where Isaac's hand holds it
PB_VIEW = ([float(os.environ.get("CC_VX", "0.36")), float(os.environ.get("CC_VY", "-0.42")), TOP + 0.20], [0.33, 0.0, TOP + 0.08])
