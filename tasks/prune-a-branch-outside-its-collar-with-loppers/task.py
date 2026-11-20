"""Pruning a branch back to its collar with two-handed bypass loppers. A short log lies across the table with one
branch growing straight up out of it; at the branch's base is its collar, the slight swelling where branch and trunk
tissue meet. The loppers lie open on the table in front of the arms, one handle toward each arm.

Loppers take both hands: each holds one handle, the tool is carried to the branch with its jaws open around it, and
the handles are pulled together to cut. Where the cut lands is the skill. The pruning standard (ANSI A300 Part 1) is a
removal cut just outside the branch collar: a flush cut into the collar wounds the trunk and lets decay in, and a cut
further out leaves a stub that dies back and rots. The instruction only says to prune the branch off.

Kit: yamkit (the loppers as a free-floating mechanism, two arms on a pivot; the log; the branch's upper part on a
hinge that is released once it is cut; gripper states).
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import yamkit as kit  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Prune this branch off the log with the loppers."

# --- Scene, world coordinates (m): x forward from the arm bases, y to the robot's left, z up from the table top.
LOG_X, LOG_R, LOG_L = 0.520, 0.045, 0.300  # the log: 9 cm across, 30 cm long, lying along y
LOG_TOP = 2 * LOG_R  # its top, 9 cm up
BRANCH = (LOG_X, 0.000)  # the branch grows straight up from the log's top
BRANCH_R = 0.010  # 2 cm across
BRANCH_H = 0.200  # 20 cm above the collar
COLLAR_R, COLLAR_H = 0.017, 0.012  # the collar: a swelling 3.4 cm across and 1.2 cm tall at the branch's base
COLLAR_TOP = LOG_TOP + COLLAR_H  # the collar's top edge, 10.2 cm up: the cut's target is just above it

# The loppers: 18-inch (45 cm) bypass loppers. Arm A carries the hook (the anvil the branch sits against) and the
# left handle; arm B carries the curved blade and the right handle; they cross at the pivot. In each arm's own frame
# (origin at the pivot, z up when lying flat) its handle runs along -x. Arm A is the base: its hook points along
# HOOK_ANG. Arm B turns about z by q: its handle is at angle pi + q and its blade at angle q. The handles bend 14
# degrees each way off the jaws' line, so with the jaws shut (q = HOOK_ANG) they are still 28 degrees apart, 14.5 cm
# at the grips. The jaws' opening is q - HOOK_ANG.
JAW_L = 0.080  # pivot to jaw tip
HANDLE_L = 0.340  # pivot to handle end
GRIP_R = 0.300  # where a hand holds each handle (the middle of its rubber grip)
HO = math.radians(14.0)  # each handle's bend away from the jaws' line
HOOK_ANG = 2 * HO  # the hook's direction in arm A's frame
OPEN = math.radians(35.0)  # jaws open (as they lie on the table): a 2 cm branch fits 4.5 cm into the throat
Q_OPEN = HOOK_ANG + OPEN  # the joint at that opening
Q_SHUT = HOOK_ANG  # the joint with the jaws shut
Q_MIN, Q_MAX = HOOK_ANG - math.radians(3.0), HOOK_ANG + math.radians(42.0)  # the handles' bumper; the jaws' widest
JAW_T = 0.006  # jaws 6 mm thick
GRIP_W, GRIP_T = 0.028, 0.020  # the rubber grips' oval section: 2.8 cm wide (flat) by 2.0 cm thick
HANDLE_Z = 0.010  # the handles' centerline above the jaws' plane (they rest on their grips lying flat)
CUT_OPEN = math.radians(4.0)  # the jaws count as closed (the cut made) below this opening
THROAT = (0.015, 0.075)  # where in the jaws a branch is cut, m from the pivot
THROAT_ANG = (-0.10, 0.50)  # and its angle in the loppers' frame (from the hook toward the open blade)
MAX_TILT = math.radians(30.0)  # the jaws' plane within 30 degrees of square to the branch, or the blade skids

# --- The cut's resistance. Nothing is cut in the physics (the branch has no collider in the jaws' path), so the force of
# a real cut is a constant torque the blade arm's joint law applies against closing while the jaws are biting the
# branch: from first contact (the 2 cm branch fills the gap 4.5 cm out when the jaws are 26 degrees open) down to the
# jaws' close (4 degrees). Scaled to what one YAM arm closes and the other holds still: 4 N at the grip, where real loppers need
# roughly thirty times that on a 2 cm hardwood branch.
BITE_OPEN = math.radians(26.0)
RESIST_N = 4.0  # N at the grip
RESIST_K = 200.0  # N m/rad: the drive's gain (the torque is held by offsetting the target)
RESIST_TORQUE = RESIST_N * GRIP_R


P0 = (0.380, 0.004)  # the pivot as the loppers lie on the table (jaws toward +x, short of the log)
YAW0 = -Q_OPEN / 2  # so that the two handles lie symmetric about -x

# --- Scoring: the stub left above the collar's top edge.
FULL_STUB = 0.015  # 0 to 1.5 cm above the collar: just outside it, a proper removal cut
HALF_STUB = 0.030  # 1.5 to 3 cm: a short stub
# above 3 cm: a stub (0.25); below the collar's top (into the collar, or flush with the log): 0

GREEN = kit.Look((0.20, 0.50, 0.20), 0.7)  # rubber grips
ALU = kit.Look((0.70, 0.72, 0.74), 0.4, metallic=0.6)
BLADE = kit.Look((0.86, 0.87, 0.89), 0.15, metallic=1.0)
HOOK = kit.Look((0.20, 0.20, 0.22), 0.5, metallic=0.5)
BARK = kit.Look((0.36, 0.27, 0.19), 0.9)
WOOD = kit.Look((0.80, 0.68, 0.48), 0.8)
COLLAR = kit.Look((0.43, 0.33, 0.23), 0.9)
RING = kit.Look((0.95, 0.85, 0.15), 0.6)  # a painted line at the collar's top edge, as in the real build


def _rotz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _bar(a, b, w, t, look, z=0.0):
    """A flat bar from 2-D point a to b (loppers' frame), w wide, t thick, centered at height z."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    d = b - a
    L = float(np.linalg.norm(d))
    ang = math.atan2(d[1], d[0])
    mesh = kit.box((0, 0, 0), (L, w, t), look)
    R = _rotz(ang)
    c = np.array([*(a + b) / 2, z])
    pts = [tuple(c + R @ np.asarray(p)) for p in mesh.points]
    return kit.Mesh(tuple(pts), mesh.counts, mesh.indices, look)


def _bar_collider(a, b, w, t, z=0.0):
    """Axis-aligned colliders can't turn, so a slanted jaw is covered by a row of small boxes along it."""
    a, b = np.asarray(a, float)[:2], np.asarray(b, float)[:2]
    L = float(np.linalg.norm(b - a))
    n = max(2, int(L / 0.025))
    out = []
    for k in range(n):
        p = a + (b - a) * (k + 0.5) / n
        out.append(kit.Collider((float(p[0]), float(p[1]), z), size=(min(w, 0.03), min(w, 0.03), t)))
    return out


def _arm(jaw_ang, jaw_look, jaw_curve):
    """One arm of the loppers in its own frame: the handle along -x (a rubber grip on its last 12 cm), the jaw from
    the pivot along `jaw_ang`, curving toward the other jaw by `jaw_curve`."""
    m, c = [], []
    R = _rotz(jaw_ang)
    tip = R @ np.array([JAW_L, jaw_curve, 0.0])
    mid = R @ np.array([JAW_L * 0.55, jaw_curve * 1.6, 0.0])
    m.append(_bar((0.0, 0.0), mid[:2], 0.020, JAW_T, jaw_look))
    m.append(_bar(mid[:2], tip[:2], 0.012, JAW_T, jaw_look))
    c += _bar_collider(R @ np.array([0.014, 0.0, 0.0]), tip, 0.012, JAW_T)
    shaft = GRIP_R - 0.06
    m.append(kit.box((-shaft / 2, 0, HANDLE_Z / 2), (shaft, 0.016, 0.012), ALU))
    m.append(kit.box((-(shaft + HANDLE_L) / 2, 0, HANDLE_Z), (HANDLE_L - shaft, GRIP_W, GRIP_T), GREEN))
    c.append(kit.Collider((-(shaft + HANDLE_L) / 2, 0, HANDLE_Z), size=(HANDLE_L - shaft, GRIP_W, GRIP_T)))
    return m, c


class CutResist:
    """The blade arm's joint law: free, except while the jaws bite an uncut branch in the throat, when a constant torque
    pushes them open (resisting the close). The tracker sets the flag each step."""

    def fresh(self, lk):
        return {}

    def drive(self, key, q, qd, J, mem):
        if _CUT.get("episode") == kit.episode() and _CUT.get("resist"):
            return q + RESIST_TORQUE / RESIST_K, RESIST_K, 0.0, 0.5
        return q, 0.0, 0.0, 0.05


def _loppers():
    """The loppers (free): arm A (hook, left handle) is the base; arm B (blade, right handle) turns about z at the
    pivot. A pivot bolt between them."""
    ma, ca = _arm(HOOK_ANG, HOOK, -0.004)
    ma.append(kit.cylinder((0, 0, 0), 0.010, 0.014, ALU))
    mb, cb = _arm(0.0, BLADE, 0.004)
    arm_b = kit.Link("blade_arm", "revolute", "Z", (0.0, 0.0, 0.0), lower=Q_MIN, upper=Q_MAX, start=Q_OPEN,
                     mass=0.26, meshes=tuple(mb), colliders=tuple(cb), friction=0.05, damping=0.05, law=CutResist())
    return kit.Mechanism("loppers", pos=(*P0, TOP + GRIP_T / 2 - HANDLE_Z), yaw=YAW0, size=(0.42, 0.30, 0.03),
                         mass=0.28, meshes=tuple(ma), colliders=tuple(ca), links=(arm_b,))


_CUT = {"episode": None, "step": None, "height": None, "why": None}


class BranchHold:
    """The branch above the collar stays up (a stiff drive at upright) until it is cut; then it is let go and falls
    over the log away from the robot."""

    def fresh(self, lk):
        return {}

    def drive(self, key, q, qd, J, mem):
        if _CUT["episode"] == kit.episode() and _CUT["step"] is not None:
            return q, 0.0, 0.004, 0.002  # cut: free, nudged to fall away from the robot (+x)
        return 0.0, 5.0, 0.0, 0.2


def _log():
    """The log and the branch's base (fixed): bark, sawn end grain, the collar with a painted line at its top edge;
    the branch above the collar is a link hinged there."""
    m = [kit.cylinder((0, 0, LOG_R), LOG_R, LOG_L, BARK, axis="y"),
         kit.cylinder((0, LOG_L / 2 + 0.0005, LOG_R), LOG_R - 0.004, 0.001, WOOD, axis="y"),
         kit.cylinder((0, -LOG_L / 2 - 0.0005, LOG_R), LOG_R - 0.004, 0.001, WOOD, axis="y"),
         kit.lathe([(COLLAR_R, LOG_TOP - 0.004), (COLLAR_R, LOG_TOP + COLLAR_H * 0.5),
                    (BRANCH_R + 0.002, COLLAR_TOP)], COLLAR, center=(0, 0, 0))]  # the collar: no painted line
    c = [kit.Collider((0, 0, LOG_R), size=(2 * LOG_R * 0.9, LOG_L, 2 * LOG_R))]
    top = kit.Link("branch", "revolute", "Y", (0.0, 0.0, COLLAR_TOP), lower=-0.02, upper=1.45, start=0.0, mass=0.03,
                   meshes=(kit.cylinder((0, 0, BRANCH_H / 2), BRANCH_R, BRANCH_H, BARK),
                           kit.cylinder((0.025, 0.0, BRANCH_H * 0.8), 0.004, 0.06, BARK, axis="x")),
                   colliders=(kit.Collider((0, 0, BRANCH_H - 0.015), size=(0.02, 0.02, 0.03)),),
                   law=BranchHold())
    return kit.Mechanism("log", pos=(LOG_X, 0.0, TOP), size=(2 * LOG_R, LOG_L, COLLAR_TOP + BRANCH_H), meshes=tuple(m),
                         colliders=tuple(c), links=(top,))


OBJECTS = [_loppers(), _log()]


# --- the cut ---------------------------------------------------------------------------------------------------
def _rot(q):
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def jaws_q(objects):
    return kit.joint(objects, "loppers/blade_arm").pos


def branch_in_jaws(objects):
    """Where the branch's axis crosses the jaws' plane: (r from the pivot, angle in the loppers' frame, height of the
    crossing above the table, the plane's tilt from square to the branch), or None if it does not cross it."""
    o = objects["loppers"]
    R = _rot(o.quat_xyzw)
    n = R[:, 2]  # the jaws' plane normal
    p0 = np.asarray(o.pos, float)
    if abs(n[2]) < 1e-6:
        return None
    b = np.array([BRANCH[0], BRANCH[1], 0.0])
    t = float(np.dot(n, p0 - b) / n[2])  # height along the (vertical) branch axis where it meets the plane
    hit = b + np.array([0.0, 0.0, t])
    loc = R.T @ (hit - p0)
    r = float(math.hypot(loc[0], loc[1]))
    ang = float(math.atan2(loc[1], loc[0]))
    tilt = float(math.acos(min(1.0, abs(n[2]))))
    return r, ang, t - TOP, tilt


def _track(objects):
    if _CUT["episode"] != kit.episode():
        _CUT.update(episode=kit.episode(), step=None, height=None, why=None, prev_q=None, n=0, objects=None, resist=False)
    if objects is _CUT.get("objects"):
        return
    _CUT["objects"] = objects
    _CUT["n"] += 1
    q = jaws_q(objects) - Q_SHUT  # the jaws' opening
    prev = _CUT["prev_q"]
    _CUT["prev_q"] = q
    hit = branch_in_jaws(objects)
    if os.environ.get("LOP_DEBUG"):
        lo = objects["loppers"]
        print(f"LOP {_CUT['n']} open={math.degrees(q):.1f} pivot={np.round(lo.pos, 3)} "
              f"hit={None if hit is None else tuple(round(v, 3) for v in hit)} cut={_CUT['height']} "
              f"L={np.round(objects['left_gripper'].pos, 3)} R={np.round(objects['right_gripper'].pos, 3)}",
              flush=True)
    _CUT["resist"] = False
    if _CUT["step"] is not None or hit is None:
        return
    r, ang, h, tilt = hit
    ang -= HOOK_ANG  # from the hook toward the blade
    on_branch = LOG_TOP <= h <= COLLAR_TOP + BRANCH_H  # the jaws' plane crosses the branch itself, not air above its tip
    in_bite = THROAT[0] <= r <= THROAT[1] and THROAT_ANG[0] <= ang <= THROAT_ANG[1] and tilt <= MAX_TILT and on_branch
    if in_bite and CUT_OPEN < q <= BITE_OPEN:
        _CUT["resist"] = True  # the blade is in the wood: the law resists the close
    if prev is None or not (prev >= CUT_OPEN > q):
        return
    if in_bite:
        _CUT.update(step=_CUT["n"], height=h, why="cut")


def cut_height(objects):
    """m above the table where the branch was cut (the jaws' plane at the branch's axis), or None if not cut."""
    _track(objects)
    return _CUT["height"]


def stub(objects):
    """m of branch left above the collar's top edge (negative: the cut went into the collar), or None."""
    h = cut_height(objects)
    return None if h is None else h - COLLAR_TOP


def _job(objects):
    s = stub(objects)
    if s is None:
        return 0.0  # not cut: the jaws never closed on the branch in the throat with the blade square to it
    if s < 0.0:
        return 0.0  # into the collar (a flush cut): the trunk's defense zone is wounded
    if s <= FULL_STUB:
        return 1.0
    if s <= HALF_STUB:
        return 0.5
    return 0.25


def success(objects):
    """The branch cut off just outside its collar: 0 to 1.5 cm above the collar's top edge."""
    return _job(objects) == 1.0


def score(objects):
    """By the first cut (the jaws closing below 4 degrees with the branch in the throat, 1.5 to 7.5 cm from the pivot,
    the jaws' plane within 30 degrees of square to the branch), measured as the stub left above the collar's top edge:
    1.0 for 0 to 1.5 cm (just outside the collar); 0.5 for 1.5 to 3 cm; 0.25 above 3 cm (a stub); 0 below the
    collar's top edge (into the collar: a flush cut) or with no cut. The first cut counts; later ones do not. Possible
    scores: 0, 0.25, 0.5, 1.0."""
    return _job(objects)


# --- Demo: both arms lift the loppers by their grips, carry the open jaws round the branch at the cut height, then the
# right arm closes its handle while the left holds the hook still.
# aim 25 mm below the collar's top edge: the arms hold the loppers about 34 mm high under their weight and the cut's
# resistance, so the cut lands near 9 mm above it.
# DEMO_STUB_CM overrides it (negative controls: -3.5 for a cut into the collar, 0 for a stub).
CUT_STUB = float(os.environ.get("DEMO_STUB_CM", "-2.5")) / 100


def _grip_xy(pivot, yaw, ang, r=GRIP_R):
    a = yaw + ang
    return pivot[0] + r * math.cos(a), pivot[1] + r * math.sin(a)


def _grip_yaw(yaw, ang):
    """Fingers closing across the handle (yaw 0 closes along y; the handle points along yaw + ang); the grip is
    180-degree symmetric, so the yaw is folded into (-pi/2, pi/2]."""
    a = yaw + ang
    while a > math.pi / 2:
        a -= math.pi
    while a <= -math.pi / 2:
        a += math.pi
    return a


def demo():
    """Left arm: fingers across the left (hook) grip, lift, carry, hold the hook arm still through the cut. Right arm:
    fingers across the right (blade) grip, lift and carry with the left, then sweep the blade handle round the pivot
    until the jaws close. The pivot goes to where the branch sits 4.5 cm into the throat, against the hook, at the
    cut height. Then both draw the loppers back off the stub, set them down on the table, let go and go home."""
    yaw = YAW0
    # the target pivot: the branch 4.5 cm into the throat, 0.22 rad off the hook toward the blade
    r, a = 0.045, HOOK_ANG + 0.22
    off = _rotz(yaw) @ np.array([r * math.cos(a), r * math.sin(a), 0.0])
    pc = (BRANCH[0] - off[0], BRANCH[1] - off[1])
    cut_z = COLLAR_TOP + CUT_STUB  # the jaws' plane
    zg_table = GRIP_T / 2 + 0.006  # the tool point over a grip lying on the table: fingertips 4 mm above it
    zg = lambda plane: plane + HANDLE_Z + 0.006  # noqa: E731  tool point over a grip with the jaws' plane at `plane`
    lift = cut_z + 0.03
    aL, aR0 = math.pi, math.pi + Q_OPEN  # the handles' angles in arm A's frame
    open_w = FINGER_OPEN * 0.6
    L0, R0 = _grip_xy(P0, yaw, aL), _grip_xy(P0, yaw, aR0)
    L1, R1 = _grip_xy(pc, yaw, aL), _grip_xy(pc, yaw, aR0)
    yl, yr = _grip_yaw(yaw, aL), _grip_yaw(yaw, aR0)
    left = [Waypoint(2.0, (*L0, TOP + zg_table + 0.06), open_w, "over the left grip", yl),
            Waypoint(1.2, (*L0, TOP + zg_table), open_w, "fingers either side of the grip", yl),
            Waypoint(0.8, (*L0, TOP + zg_table), FINGER_CLOSED, "hold the left handle", yl),
            Waypoint(1.6, (*L0, TOP + zg(lift)), FINGER_CLOSED, "lift together", yl),
            Waypoint(2.4, (*L1, TOP + zg(lift)), FINGER_CLOSED, "jaws open round the branch", yl),
            Waypoint(1.2, (*L1, TOP + zg(cut_z)), FINGER_CLOSED, "down to the cut height", yl)]
    right = [Waypoint(2.0, (*R0, TOP + zg_table + 0.06), open_w, "over the right grip", yr),
             Waypoint(1.2, (*R0, TOP + zg_table), open_w, "fingers either side of the grip", yr),
             Waypoint(0.8, (*R0, TOP + zg_table), FINGER_CLOSED, "hold the right handle", yr),
             Waypoint(1.6, (*R0, TOP + zg(lift)), FINGER_CLOSED, "lift together", yr),
             Waypoint(2.4, (*R1, TOP + zg(lift)), FINGER_CLOSED, "jaws open round the branch", yr),
             Waypoint(1.2, (*R1, TOP + zg(cut_z)), FINGER_CLOSED, "down to the cut height", yr)]
    # the cut: the right grip sweeps round the pivot from open to 3 degrees past closed (onto the bumper)
    n = 8
    for k in range(1, n + 1):
        q = Q_OPEN + (Q_MIN - Q_OPEN) * k / n
        right.append(Waypoint(0.35, (*_grip_xy(pc, yaw, math.pi + q), TOP + zg(cut_z)), FINGER_CLOSED,
                              "close the handles", _grip_yaw(yaw, math.pi + q)))
        left.append(Waypoint(0.35, (*L1, TOP + zg(cut_z)), FINGER_CLOSED, "hold the hook still", yl))
    R2 = _grip_xy(pc, yaw, math.pi + Q_MIN)
    yr2 = _grip_yaw(yaw, math.pi + Q_MIN)
    right += [Waypoint(1.0, (*R2, TOP + zg(cut_z)), FINGER_CLOSED, "hold closed", yr2),
              Waypoint(1.6, (R2[0] - 0.08, R2[1], TOP + zg(cut_z)), FINGER_CLOSED, "draw back off the stub", yr2),
              Waypoint(1.6, (R2[0] - 0.08, R2[1], TOP + zg_table + 0.004), FINGER_CLOSED, "set them down", yr2),
              Waypoint(0.8, (R2[0] - 0.08, R2[1], TOP + zg_table + 0.004), open_w, "let go", yr2),
              Waypoint(0.8, (R2[0] - 0.08, R2[1], TOP + zg_table + 0.08), open_w, "lift clear", yr2),
              kit.Home(1.8), kit.Home(1.0, label="stay home")]
    left += [Waypoint(1.0, (*L1, TOP + zg(cut_z)), FINGER_CLOSED, "hold", yl),
             Waypoint(1.6, (L1[0] - 0.08, L1[1], TOP + zg(cut_z)), FINGER_CLOSED, "draw back off the stub", yl),
             Waypoint(1.6, (L1[0] - 0.08, L1[1], TOP + zg_table + 0.004), FINGER_CLOSED, "set them down", yl),
             Waypoint(0.8, (L1[0] - 0.08, L1[1], TOP + zg_table + 0.004), open_w, "let go", yl),
             Waypoint(0.8, (L1[0] - 0.08, L1[1], TOP + zg_table + 0.08), open_w, "lift clear", yl),
             kit.Home(1.8), kit.Home(1.0, label="stay home")]
    pad = sum(w.duration for w in right) - sum(w.duration for w in left)
    if pad > 0:
        left.append(kit.Home(pad, label="stay home"))
    elif pad < 0:
        right.append(kit.Home(-pad, label="stay home"))
    return {"left": left, "right": right}
