"""Cover a cake with its pastry display dome (yam_arena task file, built with labkit)."""
import math
import pathlib
import sys

import os as _os

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import labkit  # noqa: E402
from labkit import Joint, on_reset, on_step, part  # noqa: E402
import yam_arena.demo as _demo  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402
from yam_arena.urdf import axis_angle_matrix  # noqa: E402

SCENE = "station"
INSTRUCTION = "Put the cake where the empty plate is, under the dome, and cover it with the dome. Don't touch the cake."
TOP = TABLE_TOP[SCENE]
X, Y, Z = np.eye(3)

# Clear polycarbonate pastry display dome, American Metalcraft LFTD8: 206 mm across, 168 mm tall overall (official
# listing), shaped as in the official photo: a short straight skirt, a rounded top and a ball knob on a short neck.
# Here: a 20 mm skirt, a cap on a 103 mm sphere drawn as 5 rings of 32 flat panels (each panel's chord within 1.3 mm of
# the sphere) closed by a flat disc, 2.5 mm wall; a 30 mm neck 8 mm tall and a 40 mm ball 37 mm tall (stacked discs;
# knob sizes scaled from the photo against the 206 mm rim). 0.43 kg (2.5 mm polycarbonate over its area).
GLASS, CERAMIC, SPONGE, KNOB_GREY = (0.78, 0.86, 0.90), (0.94, 0.93, 0.90), (0.86, 0.66, 0.42), (0.35, 0.36, 0.38)
D_R, D_WALL, SKIRT, D_H = 0.103, 0.0025, 0.020, 0.123  # rim radius, wall, skirt height, top of the body
STEM_R, STEM_H, HEAD_R, HEAD_H = 0.015, 0.008, 0.020, 0.037  # neck and ball: 168 mm overall
N_STAVE, N_TIER, CAP_TOP = 32, 5, math.radians(72.0)
_rm = D_R - D_WALL / 2
_w0 = 2 * D_R * math.tan(math.pi / N_STAVE) + 0.001
_yaws = [2 * math.pi * k / N_STAVE for k in range(N_STAVE)]
_DOME = [((_rm * math.cos(a), _rm * math.sin(a), SKIRT / 2), (D_WALL, _w0, SKIRT), GLASS, a) for a in _yaws]
for t in range(N_TIER):
    fa, fb = CAP_TOP * t / N_TIER, CAP_TOP * (t + 1) / N_TIER
    fm = (fa + fb) / 2
    rc = _rm * (math.cos(fa) + math.cos(fb)) / 2
    zc = SKIRT + _rm * (math.sin(fa) + math.sin(fb)) / 2
    ln = 2 * _rm * math.sin((fb - fa) / 2) + 0.001
    wd = 2 * D_R * math.cos(fa) * math.tan(math.pi / N_STAVE) + 0.001
    _DOME += [((rc * math.cos(a), rc * math.sin(a), zc), (D_WALL, wd, ln), GLASS, (a, -fm)) for a in _yaws]
_disc_z = SKIRT + _rm * math.sin(CAP_TOP)
_DOME_CYL = [((0.0, 0.0, (_disc_z + D_H) / 2), D_R * math.cos(CAP_TOP) + 0.002, D_H - _disc_z, GLASS, "Z"),
             ((0.0, 0.0, D_H + STEM_H / 2), STEM_R, STEM_H, GLASS, "Z"),
             ((0.0, 0.0, D_H + STEM_H + 0.003), 0.016, 0.006, GLASS, "Z"),
             ((0.0, 0.0, D_H + STEM_H + 0.013), HEAD_R, 0.014, GLASS, "Z"),
             ((0.0, 0.0, D_H + STEM_H + 0.0245), 0.017, 0.009, GLASS, "Z"),
             ((0.0, 0.0, D_H + STEM_H + 0.0325), 0.011, 0.007, GLASS, "Z"),
             ((0.0, 0.0, D_H + STEM_H + HEAD_H - 0.001), 0.005, 0.002, GLASS, "Z")]
labkit.GLASS_COLORS.add(GLASS)
# Display spot: where the empty plate starts, under the dome (the counter's display position).
SPOT = np.array([0.30, 0.10])
P_R, P_T = 0.125, 0.012
DOME_XY = SPOT
DOME = part("dome", pos=(SPOT[0], SPOT[1], TOP + P_T), mass=0.43, friction=0.6, color=GLASS, boxes=_DOME,
            cylinders=_DOME_CYL)

# Two flat glazed ceramic plates, 250 mm across, 12 mm thick, 0.60 kg each, loose on the counter (glazed foot on
# laminate, friction 0.35): the empty display plate
# on the spot under the dome, and the cake's plate 290 mm to its right (a 40 mm gap between their edges).
_PLATE_CYL = [((0.0, 0.0, P_T / 2), P_R, P_T, CERAMIC, "Z")]
EMPTY = part("empty_plate", pos=(SPOT[0], SPOT[1], TOP), mass=0.60, friction=0.35, color=CERAMIC, cylinders=_PLATE_CYL)
PLATE_XY = np.array([0.30, -0.19])
PLATE = part("plate", pos=(PLATE_XY[0], PLATE_XY[1], TOP), mass=0.60, friction=0.35, color=CERAMIC,
             cylinders=_PLATE_CYL)

# The cake: a 6 x 2.4 in polystyrene cake dummy (152 x 60 mm) under a 1.5 mm skin of non-drying modelling clay, so
# 155 mm across and 61.5 mm tall, ballasted to 0.50 kg; on its plate's centre (rigid in the simulation).
C_R, C_H = 0.0775, 0.0615
CAKE = part("cake", pos=(PLATE_XY[0], PLATE_XY[1], TOP + P_T), mass=0.50, friction=1.0, color=SPONGE,
            cylinders=[((0.0, 0.0, C_H / 2), C_R, C_H, SPONGE, "Z")])
OBJECTS = [EMPTY, PLATE, CAKE, DOME]

def score(objects):
    return 0.0


def success(objects):
    return False


def distance(objects):
    return 0.0


from yam_arena import ik as _ik  # noqa: E402
from yam_arena.constants import READY as _READY  # noqa: E402


def _tips(env):
    from yam_arena import runtime as _rt

    b = env.unwrapped
    o = b.scene.env_origins[0].cpu().numpy()
    out = []
    for n in ("left_robot", "right_robot"):
        robot = b.scene[n]
        mid = _rt.pad_tips_world(robot) - o
        g = _rt.link_poses(robot)["gripper"]
        ids = robot.find_joints(["joint7", "joint8"], preserve_order=True)[0]
        j7, j8 = (abs(float(v)) for v in robot.data.joint_pos.torch[0, ids].cpu().numpy())
        out += [mid - j7 * g[:3, 0], mid + j8 * g[:3, 0]]  # (joint7 moves towards -x of the gripper, joint8 towards +x)
    return out


def _finger_points(env):
    """Points over both fingers of both hands, world frame: each finger is a plate 16 mm wide at its tip widening to
    68 mm 60 mm up from it, sampled at 6 heights and 5 points across, plus the gripper body's lower corners."""
    from yam_arena import runtime as _rt

    b = env.unwrapped
    o = b.scene.env_origins[0].cpu().numpy()
    out = []
    for n in ("left_robot", "right_robot"):
        robot = b.scene[n]
        mid = _rt.pad_tips_world(robot) - o
        g = _rt.link_poses(robot)["gripper"]
        ids = robot.find_joints(["joint7", "joint8"], preserve_order=True)[0]
        j7, j8 = (abs(float(v)) for v in robot.data.joint_pos.torch[0, ids].cpu().numpy())
        close = g[:3, 0]
        up = (g[:3, 3] - o) - mid
        up = up - np.dot(up, close) * close
        up /= max(np.linalg.norm(up), 1e-9)
        wide = np.cross(up, close)
        for tip in (mid - j7 * close, mid + j8 * close):
            for h in (0.0, 0.012, 0.024, 0.036, 0.048, 0.060):
                half = 0.008 + (0.034 - 0.008) * h / 0.060
                for f in (-1.0, -0.5, 0.0, 0.5, 1.0):
                    out.append(tip + h * up + f * half * wide)
        for f in (-1.0, 1.0):
            for e in (-1.0, 1.0):
                out.append(mid + 0.070 * up + f * 0.051 * close + e * 0.034 * wide)
    return out


def tilt_deg(R, down=False):
    c = float(R[2, 2]) * (-1.0 if down else 1.0)
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))

def _ik_err(pe, re):
    return pe * 1000.0 + math.degrees(re)


FREE_MOVES = ("wrist turned", "hand turned", "up and clear")


def _endpoints(plan):
    """Joint angles for every waypoint's end, chained: warm-started from the previous one, and, where that solve misses,
    the closest of several seeded solves that reach it (so the wrist turns the way the waypoints mean)."""
    rng = np.random.default_rng(7)
    seeds = [np.array(_READY, float)] + [np.array(_READY, float) + rng.normal(0.0, 0.8, 6) for _ in range(10)]
    q_prev, out, switch, forced = plan.q.copy(), [], set(), set()
    for i, w in enumerate(plan.waypoints):
        pos, quat = np.asarray(w.pos, float), _ik.matrix_to_quat_xyzw(_demo.grip_rotation(w.yaw))
        q, pe, re = _ik.solve(pos, quat, q_prev, base_pos=plan.base_pos, iters=300)
        if any(m in w.label for m in FREE_MOVES):
            forced.add(i)  # a free-space move: always a joint-space blend
        if _ik_err(pe, re) > 1.5:
            switch.add(i)
            best = (_ik_err(pe, re), 0.0, q)
            for s in [q_prev + rng.normal(0.0, 0.3, 6) for _ in range(4)] + seeds:
                qs, ps, rs = _ik.solve(pos, quat, s, base_pos=plan.base_pos, iters=200)
                e, d = _ik_err(ps, rs), float(np.max(np.abs(qs - q_prev)))
                if (e < 1.0 and (best[0] >= 1.0 or d < best[1])) or (best[0] >= 1.0 and e < best[0]):
                    best = (e, d, qs)
            q = best[2]
        elif (float(np.max(np.abs(q[3:5] - q_prev[3:5]))) > math.radians(70.0)
              or float(np.max(np.abs(q - q_prev))) > math.radians(100.0)):
            switch.add(i)  # reached, but by turning the wrist over
        out.append(q)
        q_prev = q
    # then backwards: where the arm changes its wrist's branch between two waypoints, move the change earlier if the
    # earlier waypoint can be reached in the later branch (so it happens in free space, before an approach)
    for i in range(len(out) - 1, 0, -1):
        if i not in switch or i in forced:
            continue
        w = plan.waypoints[i - 1]
        pos, quat = np.asarray(w.pos, float), _ik.matrix_to_quat_xyzw(_demo.grip_rotation(w.yaw))
        q, pe, re = _ik.solve(pos, quat, out[i], base_pos=plan.base_pos, iters=300)
        if (_ik_err(pe, re) < 1.5 and float(np.max(np.abs(q - out[i])))
                < float(np.max(np.abs(out[i - 1] - out[i]))) - math.radians(10.0)):
            out[i - 1] = q
            switch.discard(i)
            switch.add(i - 1)
    return out, switch | forced


def _seg(plan, t):
    acc = 0.0
    for i, w in enumerate(plan.waypoints):
        if t < acc + w.duration:
            return i, _ik.smoothstep((t - acc) / w.duration)
        acc += w.duration
    return len(plan.waypoints) - 1, 1.0


def _action7(self, t):
    """[j1..j6, grip] at time t: IK warm-started from the previous step, or, where that misses, from the joint-space
    blend of the segment's end solutions. It also sets the plan clock the trims run on."""
    _demo._plan_clock = float(t)
    if getattr(self, "_qends", None) is None:
        self._qends, self._switch = _endpoints(self)
    pos, rot, finger, label = self.target(t)
    quat = _ik.matrix_to_quat_xyzw(rot)
    i, s = _seg(self, t)
    q0 = self._qends[i - 1] if i > 0 else self._q_start
    seed = q0 + s * (self._qends[i] - q0)
    if i in self._switch:
        # the wrist changes branch on this (free-space) segment: follow the joint-space blend
        self.q = seed
        grip = float(np.clip(finger / FINGER_OPEN, 0.0, 1.0)) if FINGER_OPEN != FINGER_CLOSED else 1.0
        return np.r_[seed, grip], label
    q, pe, re = _ik.solve(pos, quat, self.q, base_pos=self.base_pos, iters=80)
    if _ik_err(pe, re) > 2.0:
        q2, p2, r2 = _ik.solve(pos, quat, seed, base_pos=self.base_pos, iters=80)
        if _ik_err(p2, r2) < _ik_err(pe, re) - 1.0:
            q, pe, re = q2, p2, r2
    self.q = q
    self.worst_ik_mm = max(self.worst_ik_mm, pe * 1000)
    self.worst_ik_deg = max(self.worst_ik_deg, float(np.degrees(re)))
    grip = float(np.clip(finger / FINGER_OPEN, 0.0, 1.0)) if FINGER_OPEN != FINGER_CLOSED else 1.0
    return np.r_[q, grip], label


_prev_init = _demo.ArmPlan.__init__
if not getattr(_prev_init, "_fa", False):
    def _init(self, *a, _prev=_prev_init, **k):
        _prev(self, *a, **k)
        self._q_start, self._qends = self.q.copy(), None

    _init._fa = True
    _demo.ArmPlan.__init__ = _init
    _action7._clock = True
    _demo.ArmPlan.action7 = _action7


class Orient(float):
    """A Waypoint ``yaw`` that carries a full TCP rotation."""

    def __new__(cls, rot):
        obj = float.__new__(cls, 0.0)
        obj.rot = np.asarray(rot, float)
        return obj


_prev_grip = _demo.grip_rotation
if not getattr(_prev_grip, "_rot", False):
    def _grip(yaw=0.0):
        return yaw.rot if hasattr(yaw, "rot") else _prev_grip(yaw)

    _grip._rot = True
    _demo.grip_rotation = _grip

TIP_ABOVE_TCP = 0.013

class _Plan:
    """Two arms' waypoints, built in phases that start together; the clock is the phases' running time."""

    def __init__(self):
        self.L, self.R = [], []
        self.t = 0.0

    def hold(self, plan, dur, label="hold"):
        w = plan[-1]
        plan.append(Waypoint(dur, w.pos, w.finger, label, w.yaw))

    def phase(self, right=(), left=()):
        """Each list: (duration, pos, finger, label, yaw) tuples. The shorter one is padded with a hold."""
        tr, tl = sum(w[0] for w in right), sum(w[0] for w in left)
        dur = max(tr, tl)
        for plan, ws, tot in ((self.R, right, tr), (self.L, left, tl)):
            for w in ws:
                plan.append(Waypoint(*w))
            if dur - tot > 1e-9:
                self.hold(plan, dur - tot)
        t0 = self.t
        self.t += dur
        return t0

    def window(self, t0, t1, key, fn, watch):
        _ct["wins"].append((t0, t1, key, fn, watch))

    def gate(self, t0, t1, key, cond):
        _ct["gates"].append((t0, t1, key, cond))


# trims: each a 3-vector added to the TCPs of _Dyn waypoints with its key; integrated in windows from seen positions
_ct = {"trim": {}, "wins": [], "last": {}, "gates": [], "gate": {}}


@on_reset
def _reset_ct(env):
    _ct["trim"] = {}
    _ct["last"] = {}
    _ct["gate"] = {}


@on_step
def _gtrack(env):
    """Gates: from t0 on, latch ``key`` once ``cond`` sees what it waits for (at t1 at the latest)."""
    t = getattr(_demo, "_plan_clock", 0.0)
    for t0, t1, key, cond in _ct["gates"]:
        if t0 <= t and not _ct["gate"].get(key) and (t >= t1 or cond(env)):
            _ct["gate"][key] = t


class _Gate:
    """A TCP that is ``a`` until the gate ``key`` latches, then ``b``."""

    def __init__(self, a, b, key):
        self.a, self.b, self.key = np.asarray(a, float), np.asarray(b, float), key

    def __array__(self, dtype=None, copy=None):
        return np.asarray(self.b if _ct["gate"].get(self.key) else self.a, dtype=dtype or float)


def _seen(env, name):
    return np.asarray(labkit._part_state(env, labkit._SPECS[name]).pos, float)


@on_step
def _ctrack(env):
    t = getattr(_demo, "_plan_clock", 0.0)
    for t0, t1, key, fn, watch in _ct["wins"]:
        if not t0 <= t < t1:
            continue
        p = _seen(env, watch)
        last = _ct["last"].get(key)
        _ct["last"][key] = p
        if last is None or np.max(np.abs(p - last)) > 0.0005:
            continue  # only while what is watched holds still
        e = np.asarray(fn(env), float)
        tr = _ct["trim"].get(key, np.zeros(3))
        step, cap = TRIM_LIMITS.get(key, (0.0003, 0.020))
        _ct["trim"][key] = np.clip(tr + np.clip(TRIM_GAIN.get(key, 0.2) * e, -step, step), -cap, cap)


class _Dyn:
    """A TCP plus the live trim ``key``."""

    def __init__(self, tcp, key):
        self.tcp, self.key = np.asarray(tcp, float), key

    def __array__(self, dtype=None, copy=None):
        return np.asarray(self.tcp + _ct["trim"].get(self.key, np.zeros(3)), dtype=dtype or float)




R0 = _prev_grip(0.0)
RIGHT_PARK = np.array([0.15, -0.32, TOP + 0.20])
LEFT_PARK = np.array([0.15, 0.40, TOP + 0.20])
TRIM_LIMITS = {"over": (0.0003, 0.020), "lift": (0.001, 0.070), "seat": (0.0004, 0.060), "push": (0.0005, 0.070)}
TRIM_GAIN = {"over": 0.1, "lift": 0.2, "seat": 0.3, "push": 0.2}
TILT = math.radians(30.0)
D = np.array([0.0, -math.cos(TILT), -math.sin(TILT)])  # in from the +y side, tipped 30 degrees down


def _frame(d, c):
    z = np.asarray(d, float) / np.linalg.norm(d)
    x = np.asarray(c, float) - np.dot(c, z) * z
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


R_KNOB = _frame(D, X)  # jaws close along x across the knob's neck; the dome hangs plumb about that line
R_PUSH = _frame(-Z, X)  # right hand pointing down, fingers shut side by side along x: a flat face to push with
OPEN = -0.030  # 60 mm between the pads: clears the 40 mm ball
HOLD = -0.0120  # 24 mm: a firm pinch on the 30 mm neck and the ball's waist, the ball resting on the fingers
STEM_C = np.array([0.0, 0.0, D_H + STEM_H])  # the pinch point on the knob's axis, where neck meets ball (dome frame)
TCP_IN = STEM_C + 0.004 * D - TIP_ABOVE_TCP * D  # pad tips 4 mm past the knob's axis
UP = 0.097  # rim height above the counter before lowering: 25 mm over the cake's top on its plate
HIGH = 0.120  # rim height while the plates are swapped: 48 mm over the cake's top
ASIDE = SPOT + np.array([0.0, 0.085])  # held to the left while the plates are swapped: clear of the pushing wrist
PUSH_Z = TOP + 0.004 + TIP_ABOVE_TCP  # TCP height with the fingertips 4 mm above the counter, low on the plate's rim
FACE = 0.010  # from the fingertips' line to the face that touches the plate's edge (estimate)


def _tcp(xy, rim_z, back=0.0):
    return np.array([xy[0], xy[1], TOP + rim_z]) + TCP_IN - back * D


def _push_tcp(dy, up=0.0):
    """Right hand behind the cake plate's -y edge, moved dy towards +y from first touch."""
    return np.array([PLATE_XY[0], PLATE_XY[1] - P_R - FACE + dy, PUSH_Z + up])


class _Dyn2:
    """A TCP plus the live trims ``keys`` (seen corrections added together)."""

    def __init__(self, tcp, *keys):
        self.tcp, self.keys = np.asarray(tcp, float), keys

    def __array__(self, dtype=None, copy=None):
        return np.asarray(self.tcp + sum((_ct["trim"].get(k, np.zeros(3)) for k in self.keys), np.zeros(3)),
                          dtype=dtype or float)


def _lift_err(env):
    """Seen shortfall of the dome's rim below the holding height (z only): the arm sags under the load."""
    dm = labkit._part_state(env, DOME)
    return np.array([0.0, 0.0, TOP + HIGH - float(np.asarray(dm.pos)[2])])


def _push_err(env):
    """Seen distance (y only) still to go for the cake's plate to sit on the display spot."""
    pl = labkit._part_state(env, PLATE)
    return np.array([0.0, SPOT[1] - float(np.asarray(pl.pos)[1]), 0.0])


def _over_err(env):
    """Seen offset (x, y) of the cake plate's centre from the dome's."""
    dm = labkit._part_state(env, DOME)
    pl = labkit._part_state(env, PLATE)
    e = np.asarray(pl.pos, float) - np.asarray(dm.pos, float)
    return np.array([e[0], e[1], 0.0])


def _seat_err(env):
    """Seen height (z only) of the dome's rim above the cake plate's top: lower until the rim rests on it."""
    dm = labkit._part_state(env, DOME)
    pl = labkit._part_state(env, PLATE)
    return np.array([0.0, 0.0, float(np.asarray(pl.to_world((0.0, 0.0, P_T)))[2]) - float(np.asarray(dm.pos)[2])])


VARIANT = _os.environ.get("DEMO_VARIANT", "")  # negative variants for the test batch: "touch", "noswap"


def demo():
    P = _Plan()
    rk, rp, lp, rr = Orient(R_KNOB), Orient(R_PUSH), Orient(R0), Orient(R0)
    k, L = "over", "lift"
    # 1. the left hand pinches the knob and lifts the dome off the empty plate, up clear of the cake's height
    P.phase(left=[(2.5, tuple(_tcp(SPOT, P_T, 0.06)), OPEN, "beside the knob", rk),
                  (1.5, tuple(_tcp(SPOT, P_T)), OPEN, "fingers round the neck", rk),
                  (1.0, tuple(_tcp(SPOT, P_T)), HOLD, "pinch the knob", rk),
                  (1.5, tuple(_tcp(SPOT, P_T + 0.012)), HOLD, "lift off the empty plate", rk),
                  (1.5, _Dyn2(_tcp(SPOT, HIGH), L), HOLD, "up, clear of the cake's height", rk),
                  (2.0, _Dyn2(_tcp(ASIDE, HIGH), L), HOLD, "aside to the left, out of the other hand's way", rk)],
            right=[(2.0, tuple(RIGHT_PARK), FINGER_OPEN, "parked", rr)]
            + ([(2.0, tuple(np.array([PLATE_XY[0], PLATE_XY[1], TOP + P_T + C_H + 0.002 + TIP_ABOVE_TCP])),
                 FINGER_CLOSED, "negative variant: fingertips down onto the cake's top", rp)] if VARIANT == "touch" else [])
            + [(3.0 if VARIANT != "touch" else 1.0, tuple(_push_tcp(-0.03, 0.08)), FINGER_CLOSED,
                "above the cake plate's near edge", rp)])
    t_l = P.t
    P.phase(left=[(3.0, _Dyn2(_tcp(ASIDE, HIGH), L), HOLD, "hang still: raise to holding height, as seen", rk)],
            right=[(3.0, tuple(_push_tcp(-0.03)), FINGER_CLOSED, "down behind the edge", rp)])
    P.window(t_l + 0.2, t_l + 3.0, L, _lift_err, "dome")
    # 2. while the left hand holds the dome up, the right pushes the cake plate in under it; the cake plate pushes
    #    the empty plate out the far side. It stops short, then closes the last gap to the spot as seen.
    run = SPOT[1] - PLATE_XY[1]
    hold = _Dyn2(_tcp(ASIDE, HIGH), L)
    t_p = P.t
    noswap = VARIANT == "noswap"  # negative variant: the cake plate is never pushed in
    P.phase(left=[(19.5, hold, HOLD, "hold the dome up while the plates are swapped", rk)],
            right=[(19.5, tuple(RIGHT_PARK), FINGER_OPEN, "negative variant: stays parked", rr)] if noswap else [(9.0, tuple(_push_tcp(run + 0.010)), FINGER_CLOSED, "push the cake plate in, slowly", rp),
                   (2.5, _Dyn2(_push_tcp(run + 0.010), "push"), FINGER_CLOSED, "hold still: see what is left", rp),
                   (4.0, _Dyn2(_push_tcp(run + 0.010), "push"), FINGER_CLOSED, "close the last gap, as seen", rp),
                   (1.0, _Dyn2(_push_tcp(run - 0.015), "push"), FINGER_CLOSED, "back off", rp),
                   (1.0, _Dyn2(_push_tcp(run - 0.015, 0.10), "push"), FINGER_CLOSED, "up", rp),
                   (2.0, tuple(RIGHT_PARK), FINGER_OPEN, "away, to park", rr)])
    P.window(t_p + 9.2, t_p + 15.5, "push", _push_err, "plate")
    # 3. centre the dome over the cake plate as seen, lower it round the cake until the rim is seen on the plate
    t0 = P.t
    P.phase(left=[(3.0, _Dyn2(_tcp(SPOT, UP), L), HOLD, "back over the spot, just over the cake", rk),
                  (3.0, _Dyn2(_tcp(SPOT, UP), L, k), HOLD, "hang still: centre over the cake plate, as seen", rk),
                  (4.0, _Dyn2(_tcp(SPOT, P_T + 0.020), L, k), HOLD, "lower round the cake", rk),
                  (2.5, _Dyn2(_tcp(SPOT, P_T - 0.002), L, k, "seat"), HOLD, "rim down onto the plate", rk)])
    P.window(t0 + 3.3, t0 + 6.0, k, _over_err, "dome")
    t_s = P.t
    P.phase(left=[(8.0, _Dyn2(_tcp(SPOT, P_T - 0.002), L, k, "seat"), HOLD,
                   "keep lowering until the rim is seen resting on the plate", rk)])
    P.window(t_s, t_s + 8.0, "seat", _seat_err, "dome")
    # 4. let go, back out sideways under the ball, up and park
    P.phase(left=[(0.8, _Dyn2(_tcp(SPOT, P_T - 0.002), L, k, "seat"), OPEN, "let go", rk),
                  (1.5, _Dyn2(_tcp(SPOT, P_T - 0.002) + np.array([0.0, 0.09, 0.0]), L, k, "seat"), OPEN,
                   "back out sideways, under the knob's ball", rk),
                  (1.0, _Dyn2(_tcp(SPOT, P_T + 0.05) + np.array([0.0, 0.09, 0.0]), L, k, "seat"), OPEN, "up, clear", rk),
                  (2.0, tuple(LEFT_PARK), FINGER_OPEN, "away, to park", lp),
                  (3.0, tuple(LEFT_PARK), FINGER_OPEN, "parked", lp)],
            right=[(3.0, tuple(RIGHT_PARK), FINGER_OPEN, "parked", rr)])
    return {"left": P.L, "right": P.R}




# -- scoring ----------------------------------------------------------------------------------------------------------
# 0 for the whole trial once the cake has been knocked (moved 3 mm or tilted 3 degrees relative to its plate, which
# the dome or a hand touching it causes; moving or tipping the plate with the cake on it is not counted). How the
# plates and the dome are moved is left open (the dome may be held up, or set down or slid on the empty plate).
# Otherwise from the final state: 0.25 the cake's plate centred within 15 mm of the display spot (where the empty plate started); 0.5 also the
# dome over the cake (its rim below the cake's top, the whole cake inside its wall and under its top, tilt under 10
# degrees); 1.0 also its rim resting wholly on the cake's plate (within 2 mm of the plate's top, its centre within
# 20 mm of the plate's, tilt under 2 degrees). Every positive tier needs the four pad tips and each hand's pad midpoint
# at least 80 mm from the dome, both plates and the cake, and all four still: over the last 21 samples (1 s at 20 Hz)
# every two positions of each tracked point lie under 1 mm apart (three or more points per part, not on one line).
CAKE_MOVE, CAKE_TILT = 0.003, 3.0
SPOT_XY, SEAT_Z, SEAT_XY, SEAT_TILT, COVER_TILT = 0.015, 0.002, P_R - D_R - 0.002, 2.0, 10.0
STILL_STEPS = 21  # 1 s at 20 Hz: every positive score needs this whole window still
STILL_TOL, RELEASE = 0.001, 0.080
_DM_BOX = [(np.array([0.0, 0.0, D_H / 2]), np.array([D_R, D_R, D_H / 2])),
           (np.array([0.0, 0.0, D_H + (STEM_H + HEAD_H) / 2]), np.array([HEAD_R, HEAD_R, (STEM_H + HEAD_H) / 2]))]
_PL_BOX = [(np.array([0.0, 0.0, P_T / 2]), np.array([P_R, P_R, P_T / 2]))]
_CK_BOX = [(np.array([0.0, 0.0, C_H / 2]), np.array([C_R, C_R, C_H / 2]))]
_DM_PTS = ([np.array([D_R * math.cos(a), D_R * math.sin(a), 0.0]) for a in (0.0, 2.1, 4.2)]
           + [np.array([0.0, 0.0, D_H + STEM_H + HEAD_H])])
_PL_PTS = [np.array([P_R * math.cos(a), P_R * math.sin(a), 0.0]) for a in (0.0, 2.1, 4.2)]
_CK_PTS = [np.array([C_R * math.cos(a), C_R * math.sin(a), C_H]) for a in (0.0, 2.1, 4.2)]


def _box_dist(q, c, h):
    return float(np.linalg.norm(np.maximum(np.abs(np.asarray(q, float) - c) - h, 0.0)))


def _clearance(parts, pts):
    best = float("inf")
    for p, boxes in parts:
        for t in pts:
            q = np.asarray(p.to_local(np.asarray(t, float)), float)
            best = min(best, min(_box_dist(q, c, h) for c, h in boxes))
    return best


TOUCH_HAND, TOUCH_DOME = 0.004, 0.001
# Points on the cake's surface (cake frame): its top edge and two rings down its side, 24 round each, and its top face.
_CK_SURF = ([np.array([C_R * math.cos(a), C_R * math.sin(a), z]) for z in (C_H, 0.66 * C_H, 0.33 * C_H)
             for a in np.linspace(0.0, 2 * math.pi, 24, endpoint=False)]
            + [np.array([r * math.cos(a), r * math.sin(a), C_H]) for r in (0.0, C_R / 2)
               for a in np.linspace(0.0, 2 * math.pi, 8, endpoint=False)])
# Points on the dome's rim ring (dome frame), for a rim brought down onto the cake's top.
_DM_RIM = [np.array([_rm * math.cos(a), _rm * math.sin(a), 0.0]) for a in np.linspace(0.0, 2 * math.pi, 48,
                                                                                         endpoint=False)]


def _cake_gap_hand(ck, pts):
    """Smallest distance from any of the finger points to the cake (a 155 x 61.5 mm cylinder)."""
    best = float("inf")
    for q in pts:
        v = np.asarray(ck.to_local(np.asarray(q, float)), float)
        dr = max(math.hypot(v[0], v[1]) - C_R, 0.0)
        dz = max(-v[2], v[2] - C_H, 0.0)
        best = min(best, math.hypot(dr, dz))
    return best


def _shell_gap(v):
    """Distance from a point in the dome's frame to the dome's wall (skirt, spherical top, top disc), wall excluded."""
    r, z = math.hypot(v[0], v[1]), float(v[2])
    if z < 0.0:
        d = math.hypot(r - _rm, z) if r < _rm else math.hypot(r - _rm, z)
    elif z <= SKIRT:
        d = abs(r - _rm)
    else:
        phi = math.atan2(z - SKIRT, r)
        if phi <= CAP_TOP:
            d = abs(math.hypot(r, z - SKIRT) - _rm)
        else:
            d = abs(z - _disc_z) if r <= D_R * math.cos(CAP_TOP) else math.hypot(r - _rm * math.cos(CAP_TOP),
                                                                              z - _disc_z)
    return d - D_WALL / 2


def _cake_gap_dome(ck, dm):
    """Smallest gap between the cake's surface and the dome's wall, and between the dome's rim and the cake."""
    best = min(_shell_gap(np.asarray(dm.to_local(ck.to_world(tuple(q))), float)) for q in _CK_SURF)
    for q in _DM_RIM:
        v = np.asarray(ck.to_local(dm.to_world(tuple(q))), float)
        dr = max(math.hypot(v[0], v[1]) - C_R, 0.0)
        dz = max(-v[2], v[2] - C_H, 0.0)
        best = min(best, math.hypot(dr, dz) - D_WALL / 2)
    return best


_w = {"hist": [], "still": False, "far": False, "bad": False, "cake0": None, "touch": None}


@on_reset
def _w_reset(env):
    _w.update(hist=[], still=False, far=False, bad=False, cake0=None, touch=None)


@on_step
def _watch(env):
    dm, em, pl, ck = (labkit._part_state(env, p) for p in (DOME, EMPTY, PLATE, CAKE))
    c = np.asarray(pl.to_local(ck.pos), float)
    if _w["cake0"] is None:
        _w["cake0"] = c
    rel_tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(np.dot(np.asarray(ck.rotation())[:, 2],
                                                                    np.asarray(pl.rotation())[:, 2]))))))
    if float(np.linalg.norm(c - _w["cake0"])) >= CAKE_MOVE or rel_tilt >= CAKE_TILT:
        _w["bad"] = True
    # A touch marks the cake even when it does not move it: any part of a finger within TOUCH_HAND of the cake, or
    # the dome's wall (inside or out, rim included) within TOUCH_DOME of it. Once touched, the trial scores 0.
    if _cake_gap_hand(ck, _finger_points(env)) < TOUCH_HAND:
        _w["bad"] = True
        _w["touch"] = _w.get("touch") or "hand"
    if _cake_gap_dome(ck, dm) < TOUCH_DOME:
        _w["bad"] = True
        _w["touch"] = _w.get("touch") or "dome"
    tips = _tips(env)
    pts = tips + [(tips[0] + tips[1]) / 2, (tips[2] + tips[3]) / 2]
    _w["far"] = _clearance([(dm, _DM_BOX), (em, _PL_BOX), (pl, _PL_BOX), (ck, _CK_BOX)], pts) >= RELEASE
    snap = np.array([np.asarray(p.to_world(tuple(q)), float)
                     for p, qs in ((dm, _DM_PTS), (em, _PL_PTS), (pl, _PL_PTS), (ck, _CK_PTS)) for q in qs])
    h = _w["hist"]
    h.append(snap)
    del h[:-STILL_STEPS]
    H = np.array(h)
    _w["still"] = (len(h) == STILL_STEPS
                   and float(np.max(np.linalg.norm(H[:, None] - H[None, :], axis=-1))) < STILL_TOL)


def _tier(dm, pl, ck):
    if math.hypot(*(np.asarray(pl.pos)[:2] - SPOT)) > SPOT_XY:
        return 0.0
    d = np.asarray(pl.to_local(dm.pos), float) - np.array([0.0, 0.0, P_T])
    k = np.asarray(dm.to_local(ck.pos), float)
    k_top = np.asarray(dm.to_local(ck.to_world((0.0, 0.0, C_H))), float)
    covering = (k[2] >= -0.005 and k_top[2] <= D_H - D_WALL  # the cake between the rim's plane and the dome's top
                and float(np.asarray(dm.pos)[2]) < float(np.asarray(ck.to_world((0.0, 0.0, C_H)))[2])
                and math.hypot(k[0], k[1]) + C_R < D_R - D_WALL and dm.tilt_deg() < COVER_TILT)
    if not covering:
        return 0.25
    seated = abs(d[2]) <= SEAT_Z and math.hypot(d[0], d[1]) <= SEAT_XY and dm.tilt_deg() < SEAT_TILT
    # The tier only; score() returns it only once the 21-sample stillness window holds (and the hands are clear).
    return 1.0 if seated else 0.5


def score(objects):
    if _w["bad"] or not (_w["far"] and _w["still"]):
        return 0.0
    return _tier(objects["dome"], objects["plate"], objects["cake"])


def success(objects):
    return score(objects) == 1.0


def distance(objects):
    return 0.0 if _tier(objects["dome"], objects["plate"], objects["cake"]) == 1.0 else 1.0
