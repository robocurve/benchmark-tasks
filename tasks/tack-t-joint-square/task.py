"""Tack-weld a T-joint square. A mild-steel base plate (150 x 100 x 6.35 mm) is clamped on a steel fixture table, with a
scribed joint line, a tick at each end of it and three white paint ticks beside it. A 3 x 3 x 1/4 in mild-steel coupon
(0.29 kg) lies flat on a steel block to the right, and a TIG torch lies on its rest to the left. One hand stands the
coupon on its edge on the line and holds it square to the base plate; the other makes three tack welds along the joint's
root with the torch (near each end, then the middle). The torch goes back on its rest and both hands move away, leaving
the coupon standing by its tacks.

Run it:  inspect-robots run --embodiment yam_tack -E task=task.py -E object_jitter=0 --policy yam_scripted --no-prompt \\
           --instruction "Stand the coupon on the scribed line and tack it square to the base plate with three TIG tacks, then put the torch back on its rest and take both hands away." --scorer task_score

`yam_tack` (tack_t_joint/embodiment.py) spawns the bench, coupon and torch, reports each gripper's fingertip midpoint,
and makes each tack a real joint between the coupon and the bench.

The tack is bookkeeping over rigid bodies (heat, the pool and shrinkage aren't simulated):
- Root: wherever one of the coupon's four edge faces lies on the base plate with a gap of 1 mm or less. The root lines are
  that face's two long corners, one each side of the coupon.
- Arc: the torch is in a hand, and its tungsten tip is 1.5-4.5 mm from the nearest root point. There is no arc anywhere else.
- Heat: while the arc is on, each 2 mm of that root line within 4 mm of the nearest point gains its arc time. A piece is
  fused at 1.5 s and overheated at 5 s.
- Tack: when a piece fuses, the coupon is joined to the bench at that point in the pose it has at that moment: a pin at
  the tack, held against bending by a stiff spring (0.5 N m per degree). Bent 2 degrees (1.0 N m) it cracks and lets go.
  One tack is a weak hinge; two or three tacks along the line can only fold about the line.
- Shrinkage, as a criterion (not thermal simulation): a single-sided tack pulls an unrestrained plate toward its own side as
  it cools. If a tack sets while no hand grips the coupon (both fingers of one gripper pressing on
  it, from the simulator's contact forces) and no earlier tack holds it, the joint is set 3 degrees toward
  the tack's side and its spring pulls the coupon there. Gripped, or already tacked, it is held in the pose it has.
- Contamination, latched: the tungsten tip within 0.5 mm of the base plate, the fixture table or the coupon. No arc after.
"""

import math
import os

import numpy as np

from tack_t_joint import mech as M
from tack_t_joint import tilt
from yam_arena import ik
from yam_arena.constants import FINGER_OPEN, READY
from yam_arena.demo import Waypoint, grip_rotation
from yam_arena.station import BASES
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]
BASE_Z = TOP + M.BT  # the base plate's top, world
Y_LINE = M.Y_LINE
MARKS = np.array(M.MARKS)

OBJECTS = [
    Box("bench", pos=(0.0, 0.0, TOP), size=(0.01, 0.01, 0.01)),
    Box("coupon", pos=(M.COUPON_START[0], M.COUPON_START[1], TOP + M.COUPON_START[2]), size=(0.01, 0.01, 0.01), mass=M.COUPON_MASS),
    Box("torch", pos=(M.TORCH_REST["x"], M.TORCH_REST["y"], TOP + M.TORCH_REST_Z), size=(0.01, 0.01, 0.01), mass=M.TORCH_TOTAL,
        yaw=M.TORCH_REST["lean"]),
]
PARTS = ("coupon", "torch")

# -- the tack's criteria: manual TIG (GTAW) fusion tacks on 1/4 in mild steel, 2.4 mm tungsten, about 130 A ----------------
ARC = (0.0015, 0.0045)  # arc length: tungsten tip to the root
TOUCH = 0.0005  # the tip this close to steel has dipped
GAP_MAX = 0.001  # a fusion tack bridges a root gap up to this
BIN = 0.002  # the root line is tracked in pieces this long
R_HEAT = 0.004  # the arc heats pieces within this of the nearest root point: a tack about 10 mm long
T_FUSE, T_BURN = 1.5, 5.0  # s of arc: fused; overheated (oversize, undercut)
JOIN = 0.008  # a piece fusing within this of an existing tack on the same root line grows that tack
BREAK = math.radians(M.TACK_BREAK_DEG)
SHRINK = math.radians(3.0)  # angular pull of one single-sided tack on an unrestrained 1/4 in plate (1 to 3 degrees is usual)
SETTLE_S = 1.0  # a new tack is not tested for cracking while the coupon moves to where it set
GRIP_N = 0.5  # a hand grips the coupon when each of its two fingers presses on it with this contact force (N) or more
ZONE = 0.006  # a tack counts for a mark when fused metal lies within this of it along the line
STRAY = 0.010  # fused metal further than this from every mark is stray
SIDE = 0.010  # and tacks lie within this of the scribed line
SQUARE = math.radians(2.0)
LINE_TOL, END_TOL = 0.0015, 0.003
TEE = math.radians(10.0), 0.010  # tacks count on a T this close to square and to the line
HELD = 0.025  # a gripper's fingertip midpoint within this of the torch's grip centre holds it
HANDS_CLEAR, LIN, ANG, STILL_S = 0.06, 0.01, 0.5, 1.0
REST_XY, REST_Z, REST_TILT, REST_TURN = 0.012, 0.006, math.radians(10.0), math.radians(20.0)
CAP = 0.4

# -- the coupon's four edge faces, in its own frame: (face, piece, side) ------------------------------------------------
NB = 38
_S = (np.arange(NB) - (NB - 1) / 2) * BIN
FACES = ((1, -1), (1, 1), (0, -1), (0, 1))  # (axis of the face's normal, sign): the rolled edges, then the saw-cut ends
_LAND = np.zeros((4, NB, 2, 3))  # the two edges of the flat each face stands on
_ROOT = np.zeros((4, NB, 2, 3))  # the two long corners of each face: the root line each side of the coupon
_NORMAL = np.zeros((4, 3))
for _f, (_ax, _sg) in enumerate(FACES):
    _al = 1 - _ax
    _NORMAL[_f, _ax] = _sg
    _half = M.COUPON_HALF[_ax]
    _lh = M.LAND / 2 if _ax == 1 else M.T / 2
    for _k, _s in enumerate(_S):
        for _j, _sd in enumerate((-1, 1)):
            _LAND[_f, _k, _j, _ax], _LAND[_f, _k, _j, _al], _LAND[_f, _k, _j, 2] = _sg * _half, _s, _sd * _lh
            _ROOT[_f, _k, _j, _ax], _ROOT[_f, _k, _j, _al], _ROOT[_f, _k, _j, 2] = _sg * _half, _s, _sd * M.T / 2


def _fresh():
    shape = (4, NB, 2)
    return {"heat": np.zeros(shape), "fused": np.zeros(shape, bool), "over": np.zeros(shape, bool), "tack": -np.ones(shape, int),
            "xy": np.zeros(shape + (2,)), "tacks": [], "commands": [], "dirty": False, "bad": False, "arc": False, "still": 0.0, "cracked": 0}


_trial = _fresh()
TRACE = None  # dev only: a list here gets each step's coupon pose and tungsten tip (dev/sim.py measures the arms' give from it)


def reset_latch():
    """Forget the trial: the embodiment calls this at every reset. Nothing a robot does clears a latch."""
    _trial.clear()
    _trial.update(_fresh())


def take_commands():
    """Joints to switch since the last call: {"do": "lock", "slot", "anchor", "pos", "quat"} or {"do": "free", "slot"}."""
    out = _trial["commands"]
    _trial["commands"] = []
    return out


def _rot(q):
    x, y, z, w = q
    return np.array(((1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)), (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
                     (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y))))


def _angle(Ra, Rb):
    return math.acos(min(1.0, max(-1.0, (np.trace(Ra.T @ Rb) - 1.0) / 2.0)))


def tungsten(objects):
    """The tungsten's tip, world."""
    t = objects["torch"]
    return np.asarray(t.pos, float) + _rot(t.quat_xyzw) @ M.TIP


def holder(objects):
    """Which hand ('left_tcp' or 'right_tcp') holds the torch: its fingertip midpoint within HELD of the grip's centre."""
    p = np.asarray(objects["torch"].pos, float)
    best = min(("left_tcp", "right_tcp"), key=lambda a: np.linalg.norm(np.asarray(objects[a].pos, float) - p))
    return best if np.linalg.norm(np.asarray(objects[best].pos, float) - p) <= HELD else None


def gripped(objects, but=None):
    """Is the coupon squeezed between the two fingers of a hand (other than ``but``, the torch's)? ``left_grip`` and
    ``right_grip`` carry the contact force (N) between the coupon and each of that gripper's two fingers, as PhysX reports
    it; a hand beside the coupon, or closed round it without touching, presses with nothing."""
    return any(f"{arm}_tcp" != but and min(objects[f"{arm}_grip"].pos[:2]) >= GRIP_N for arm in ("left", "right"))


def _axis_rot(axis, a):
    k = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array(((0, -k[2], k[1]), (k[2], 0, -k[0]), (-k[1], k[0], 0)))
    return np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * (K @ K)


def _box_dist(objects, part, lo, hi, p):
    d = _rot(objects[part].quat_xyzw).T @ (np.asarray(p, float) - np.asarray(objects[part].pos, float))
    return float(np.linalg.norm(d - np.clip(d, lo, hi)))


def clear_of(objects, part):
    """Both hands this far (HANDS_CLEAR) from a part: open fingers sit 4.7 cm out, so 6 cm means no finger is on it."""
    lo, hi = (-M.COUPON_HALF, M.COUPON_HALF) if part == "coupon" else M.TORCH_HULL
    return all(_box_dist(objects, part, lo, hi, objects[h].pos) >= HANDS_CLEAR for h in ("left_tcp", "right_tcp"))


def torch_rested(objects):
    t = objects["torch"]
    R, p = _rot(t.quat_xyzw), np.asarray(t.pos, float)
    turn = math.atan2(R[1, 0], R[0, 0]) - M.TORCH_REST["lean"]
    turn = abs(math.atan2(math.sin(turn), math.cos(turn)))
    return bool(math.hypot(p[0] - M.TORCH_REST["x"], p[1] - M.TORCH_REST["y"]) <= REST_XY and abs(p[2] - (TOP + M.TORCH_REST_Z)) <= REST_Z
                and R[2, 2] >= math.cos(REST_TILT) and turn <= REST_TURN)


def joint(objects):
    """The coupon against the base plate now: each piece's root gap and root points, and which pieces are a fitted root."""
    c = objects["coupon"]
    R, p = _rot(c.quat_xyzw), np.asarray(c.pos, float)
    land = _LAND @ R.T + p
    root = _ROOT @ R.T + p
    gap = np.maximum(land[..., 2].min(axis=2) - BASE_Z, 0.0)  # (4, NB)
    mid = land.mean(axis=2)
    over = (M.BASE["x"][0] <= mid[..., 0]) & (mid[..., 0] <= M.BASE["x"][1]) & (M.BASE["y"][0] <= mid[..., 1]) & (mid[..., 1] <= M.BASE["y"][1])
    down = (_NORMAL @ R.T)[:, 2] < -0.7
    fitted = down[:, None] & over & (gap <= GAP_MAX)
    root[..., 2] = BASE_Z
    return dict(R=R, p=p, gap=gap, fitted=fitted, root=root, down=down)


def fit(objects, j=None):
    """How the coupon stands: lean from square (rad), the widest root gap, its left face's distance from the scribed line
    at each end, its ends' distance from the end ticks. None if no edge face is down on the base plate."""
    j = j or joint(objects)
    if not j["fitted"].any():
        return None
    f = int(np.argmax(j["fitted"].sum(axis=1)))
    R = j["R"]
    lean = math.asin(min(1.0, abs(R[2, 2])))
    ends = j["root"][f][[0, -1]]  # (2 ends, 2 sides, 3)
    left = int(np.argmax(ends[..., 1].mean(axis=0)))
    line = float(np.abs(ends[:, left, 1] - Y_LINE).max())
    ax, sg = FACES[f]
    tips = [j["p"] + R @ (np.eye(3)[ax] * sg * M.COUPON_HALF[ax] + np.eye(3)[1 - ax] * e * M.COUPON_HALF[1 - ax]) for e in (-1, 1)]
    xs = sorted(t[0] for t in tips)
    return dict(face=f, lean=lean, gap=float(j["gap"][f].max()), line=line, ends=max(abs(xs[0] - M.X0), abs(xs[1] - M.X1)))


def note_step(objects, dt):
    """Called by the embodiment after every control step: contamination, the arc, heat, new tacks, cracked tacks, and the
    clock for everything at rest."""
    tr = _trial
    if any(objects[n].pos[2] < TOP - 0.01 for n in PARTS):
        tr["bad"] = True
    j = joint(objects)
    R, p = j["R"], j["p"]
    for tk in tr["tacks"]:  # a tack bent past its limit, or pulled off its point, cracks and lets go
        tk["age"] += dt
        if tk["intact"] and tk["age"] > SETTLE_S and (_angle(tk["R"], R) > BREAK or np.linalg.norm(p + R @ tk["local"] - tk["anchor"]) > 0.0015):
            tk["intact"] = False
            tr["cracked"] += 1
            mine = tr["tack"] == tk["id"]
            tr["fused"][mine], tr["heat"][mine], tr["tack"][mine] = False, 0.0, -1
            tr["commands"].append({"do": "free", "slot": tk["id"]})
    tip = tungsten(objects)
    over_slab = M.SLAB["x"][0] <= tip[0] <= M.SLAB["x"][1] and M.SLAB["y"][0] <= tip[1] <= M.SLAB["y"][1]
    over_base = M.BASE["x"][0] <= tip[0] <= M.BASE["x"][1] and M.BASE["y"][0] <= tip[1] <= M.BASE["y"][1]
    if (over_base and tip[2] - BASE_Z <= TOUCH) or (over_slab and tip[2] - (TOP + M.SLAB["t"]) <= TOUCH) \
            or _box_dist(objects, "coupon", -M.COUPON_HALF, M.COUPON_HALF, tip) <= TOUCH:
        tr["dirty"] = True
    arc = False
    if not tr["dirty"] and holder(objects) is not None and j["fitted"].any():
        d = np.where(j["fitted"][..., None], np.linalg.norm(j["root"] - tip, axis=3), np.inf)
        f, k, s = np.unravel_index(int(np.argmin(d)), d.shape)
        if ARC[0] <= d[f, k, s] <= ARC[1]:
            arc = True
            hot = j["fitted"][f] & (np.abs(_S - _S[k]) <= R_HEAT + 1e-9)
            tr["heat"][f, hot, s] += dt
            new = hot & (tr["heat"][f, :, s] >= T_FUSE) & ~tr["fused"][f, :, s]
            tr["over"][f, :, s] |= hot & (tr["heat"][f, :, s] >= T_BURN)
            if new.any():
                tr["fused"][f, new, s] = True
                tr["xy"][f, new, s] = j["root"][f, new, s, :2]
                at = float(_S[new].mean())
                near = [tk for tk in tr["tacks"] if tk["intact"] and tk["where"] == (f, s) and abs(tk["s"] - at) <= JOIN]
                if near:
                    tk = near[0]
                else:
                    anchor = j["root"][f, new, s].mean(axis=0)
                    Rt, pt = R, p
                    pulled = not gripped(objects, holder(objects)) and not any(k2["intact"] for k2 in tr["tacks"])
                    if pulled:  # shrinkage: the coupon's top goes SHRINK toward the tack's side, about the root line
                        ax, sg = FACES[f]
                        along, top, out = R[:, 1 - ax], -sg * R[:, ax], (1 if s else -1) * R[:, 2]
                        turn = _axis_rot(along * (1.0 if np.dot(np.cross(top, out), along) > 0 else -1.0), SHRINK)
                        Rt, pt = turn @ R, anchor + turn @ (p - anchor)
                    tk = {"id": len(tr["tacks"]), "where": (int(f), int(s)), "s": at, "intact": True, "anchor": anchor, "R": Rt.copy(),
                          "local": Rt.T @ (anchor - pt), "age": 0.0, "pulled": bool(pulled)}
                    tr["tacks"].append(tk)
                    tr["commands"].append({"do": "lock", "slot": tk["id"], "anchor": anchor, "pos": pt.copy(), "quat": np.asarray(ik.matrix_to_quat_xyzw(Rt), float)})
                tr["tack"][f, new, s] = tk["id"]
    tr["arc"] = arc
    if TRACE is not None:
        TRACE.append((p.copy(), R.copy(), tip.copy(), arc))
    still = all(np.linalg.norm(objects[n].lin_vel) <= LIN and np.linalg.norm(objects[n].ang_vel) <= ANG for n in PARTS)
    tr["still"] = tr["still"] + dt if still and clear_of(objects, "coupon") and clear_of(objects, "torch") else 0.0


def sound_tacks():
    """For each of the three marks: is there fused metal of an uncracked tack within ZONE of it, and nothing overheated there."""
    tr = _trial
    x, y = tr["xy"][..., 0], tr["xy"][..., 1]
    out = []
    for m in MARKS:
        here = (np.abs(x - m) <= ZONE) & (np.abs(y - Y_LINE) <= SIDE)
        out.append(bool((tr["fused"] & here).any() and not (tr["over"] & tr["fused"] & here).any()))
    return out


def faults():
    """What caps the score: 'contaminated', 'overheated' (a tack dwelt on for 5 s), 'stray' (fused metal away from the marks)."""
    tr = _trial
    out = []
    if tr["dirty"]:
        out.append("contaminated")
    if tr["over"].any():
        out.append("overheated")
    if tr["fused"].any():
        x, y = tr["xy"][..., 0][tr["fused"]], tr["xy"][..., 1][tr["fused"]]
        if (np.abs(x[:, None] - MARKS[None, :]).min(axis=1) > STRAY).any() or (np.abs(y - Y_LINE) > SIDE).any():
            out.append("stray")
    return out


def shown():
    """The effects to show now: a bead per uncracked tack, the arc's glow, soot on a contaminated cup."""
    tr = _trial
    on = {f"tack{tk['id']}" for tk in tr["tacks"] if tk["intact"]}
    if tr["arc"]:
        on.add("fx_arc")
    if tr["dirty"]:
        on.add("fx_soot")
    return on


def score(objects):
    """Read at the end of the run; there is no early end.
    0: the coupon or the torch off the table; no sound tack; or the coupon not a T on the joint (more than 10 degrees from
       square, more than 10 mm off the line, or no edge on the base plate).
    0.15 per sound tack (0.15, 0.30, 0.45): tacked, but the fit is out of tolerance or a hand is still on the coupon.
    0.2 + 0.2 per sound tack (0.4, 0.6, 0.8): the coupon stands by its tacks with both hands 6 cm clear of it, within
       2 degrees of square, root gap 1 mm or less, its left face within 1.5 mm of the line and its ends within 3 mm of the ticks.
    Capped at 0.4 after contamination, an overheated tack or stray fused metal.
    1.0: three sound tacks, the fit in tolerance, nothing capped, the torch back on its rest, both hands clear of the
       coupon and the torch, and everything at rest for the last 1 s."""
    tr = _trial
    if tr["bad"]:
        return 0.0
    ft = fit(objects)
    n = sum(sound_tacks())
    if ft is None or n == 0 or ft["lean"] > TEE[0] or ft["line"] > TEE[1]:
        return 0.0
    good = ft["lean"] <= SQUARE and ft["gap"] <= GAP_MAX and ft["line"] <= LINE_TOL and ft["ends"] <= END_TOL
    free = clear_of(objects, "coupon") and np.linalg.norm(objects["coupon"].lin_vel) <= LIN and np.linalg.norm(objects["coupon"].ang_vel) <= ANG
    capped = bool(faults())
    if n == 3 and good and free and not capped and torch_rested(objects) and clear_of(objects, "torch") and tr["still"] >= STILL_S:
        return 1.0
    s = 0.2 + 0.2 * n if good and free else 0.15 * n
    return round(min(s, CAP) if capped else s, 3)


def debug_line(objects):
    j = joint(objects)
    ft = fit(objects, j)
    tip = tungsten(objects)
    R = j["R"]
    d = np.where(j["fitted"][..., None], np.linalg.norm(j["root"] - tip, axis=3), np.inf).min() if j["fitted"].any() else float("inf")
    mm = lambda v: np.round(1000 * np.asarray(v, float), 2).tolist()  # noqa: E731
    lean = math.degrees(math.asin(max(-1.0, min(1.0, R[2, 2]))))
    turn = math.degrees(math.atan2(R[1, 0], R[0, 0]))
    roll = math.degrees(math.asin(max(-1.0, min(1.0, R[2, 0]))))
    fits = "none" if ft is None else f"gap {1000 * ft['gap']:.2f} line {1000 * ft['line']:.2f} ends {1000 * ft['ends']:.2f}"
    tr = _trial
    return (f"coupon {mm(j['p'])} lean {lean:+6.2f} turn {turn:+6.2f} roll {roll:+6.2f} fit {fits} | tip {mm(tip)} root {1000 * d:6.2f} arc {int(tr['arc'])} "
            f"hand {holder(objects)} grip {int(gripped(objects, holder(objects)))} {np.round(objects['left_grip'].pos[:2], 1).tolist()}{np.round(objects['right_grip'].pos[:2], 1).tolist()} heat {tr['heat'].max():.2f} tacks {[(tk['id'], int(tk['intact']), int(tk['pulled']), round(1000 * float(tk['anchor'][0]), 1)) for tk in tr['tacks']]} "
            f"sound {[int(v) for v in sound_tacks()]} faults {faults()} | L {mm(objects['left_tcp'].pos)} R {mm(objects['right_tcp'].pos)} torch {mm(objects['torch'].pos)} "
            f"rested {int(torch_rested(objects))} clear {int(clear_of(objects, 'coupon'))}{int(clear_of(objects, 'torch'))} still {tr['still']:.2f} score {score(objects):.3f}")


# -- scripted demo -----------------------------------------------------------------------------------------------------
# Open loop from the fixed start: it uses only where the rests, the base plate's line and its marks are, which a policy can
# see, and reads no simulator state. The aims carry fixed offsets against the arms' give under load (see HOLD_CAL, TORCH_CAL).
# TACK_DEMO picks a shortcut, for checking the score: "none" (do nothing), "one" (one tack, then let go), "lean" (hold the
# coupon 5 degrees out of square), "hover" (the torch held 10 mm from the root: no arc), "held" (never let go of the coupon),
# "stand" (let go of the coupon before tacking), "near" (the hand opens 4 mm clear of the coupon's ends and stays round it
# without touching while it is tacked).
FINGER_PER_M = 0.04695 / 0.094
ARMS = {"left": 1, "right": -1}
PARK = 0.12, 0.34, TOP + 0.18
SEGMENT = 0.03
PICK_TILT, HOLD_TILT = math.radians(-35.0), math.radians(55.0)  # the holding hand's lean toward +y: 90 degrees apart
HOLD_YAW, HOLD_HEADING = 0.5 * math.pi, 0.5 * math.pi  # its fingers close along x, on the coupon's two ends
PRESS = 0.001  # the coupon is aimed this far into the base plate, to seat it
GRIP_W = 0.070  # finger opening commanded on the 76.2 mm coupon
NEAR_W = 0.084  # the "near" shortcut's opening: 4 mm clear of each end
WORK = math.radians(45.0)  # the torch's work angle: rolled about its handle so the tungsten bisects the corner
TORCH_LEAN = M.TORCH_REST["lean"]  # the torch points along the joint (+x), as it lies on its rest: a 17.5 degree push angle
TORCH_YAW = -0.5 * math.pi  # the torch hand's fingers close along x, on the grip block's front and back
TORCH_TILT, TORCH_HEADING = WORK, -0.5 * math.pi  # so the hand rolls this far toward the coupon (-y)
DWELL = 2.6  # s at each tack
GRIP_DZ = 0.003  # TCP above the torch's grip centre when holding
# Where the hands end up against where they are aimed, measured from logged runs (dev/sim.py); the plan aims the other
# way by as much. HOLD_CAL: the holding hand's TCP (m), lean and turn (rad). TORCH_CAL: the tungsten tip's (x, y, z) at each mark (m).
HOLD_CAL = dict(x=0.0032, y=0.0006, z=0.0, tilt=math.radians(0.02), yaw=math.radians(-0.60))
TORCH_CAL = {0: (0.00584, -0.01181, 0.01727), 1: (0.00595, -0.01136, 0.01718), 2: (0.00607, -0.01095, 0.01720)}


def set_variant(name):
    """Pick the demo: "" (the full demo) or a shortcut."""
    global VARIANT, LEAN_OFF, ARC_AIM, ORDER
    VARIANT = name
    LEAN_OFF = math.radians(-5.0 if name == "lean" else 0.0)  # the coupon's top held this far toward the torch's side (negative: away from it)
    ARC_AIM = 0.010 if name == "hover" else 0.003  # tungsten tip to the root
    ORDER = (0,) if name == "one" else (0, 2, 1)  # near each end first, then the middle


set_variant(os.environ.get("TACK_DEMO", ""))


def _fingers(width):
    return -FINGER_PER_M * width


TORCH_OPEN, TORCH_GRIP = _fingers(M.GRIP["x"] + 0.03), _fingers(M.GRIP["x"] - 0.010)  # squeezed 5 mm a side: the rolled torch must not swing in the grip


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array(((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0)))


def _lean_matrix(tilt_a, heading):
    """The rotation that leans a straight-down hand ``tilt_a`` toward ``heading``."""
    return tilt.rotation(0.0, tilt_a, heading) @ grip_rotation(0.0).T


def torch_pose(tip, tilt_a=0.0):
    """The torch's rotation and grip centre with the tungsten tip at ``tip`` (world) and the hand rolled ``tilt_a``."""
    T = _lean_matrix(tilt_a, TORCH_HEADING)
    R = T @ _rz(TORCH_LEAN)
    return R, np.asarray(tip, float) - R @ M.TIP, T


def torch_tcp(tip, tilt_a=0.0):
    """The torch hand's TCP for the tungsten tip at ``tip`` (world), the hand rolled ``tilt_a`` toward the coupon."""
    _, g, T = torch_pose(tip, tilt_a)
    return g + T @ np.array((0.0, 0.0, GRIP_DZ))


class _Plan:
    def __init__(self):
        self.wps = {a: [] for a in ARMS}
        self.t = {a: 0.0 for a in ARMS}
        self.last = {}
        for a in ARMS:
            pos, _ = ik.tcp_pose(np.array(READY, float), np.array(BASES[a]))
            self.last[a] = (tuple(float(v) for v in pos), FINGER_OPEN, 0.0, 0.0, 0.0)

    def go(self, arm, dur, pos=None, finger=None, label="", yaw=None, tilt_a=None, heading=None):
        p0, f0, y0, t0, h0 = self.last[arm]
        p = p0 if pos is None else tuple(float(v) for v in pos)
        f = f0 if finger is None else finger
        y = y0 if yaw is None else yaw
        ta = t0 if tilt_a is None else tilt_a
        h = h0 if heading is None else heading
        n = max(1, math.ceil(np.linalg.norm(np.subtract(p, p0)) / SEGMENT), math.ceil(abs(y - y0) / math.radians(15)), math.ceil(abs(ta - t0) / math.radians(10)))
        if label == "down":
            n = 1
        for k in range(1, n + 1):
            q = tuple(float(v) for v in np.asarray(p0) + (np.asarray(p) - np.asarray(p0)) * k / n)
            self.wps[arm].append(Waypoint(dur / n, q, f0 + (f - f0) * k / n, label, tilt.leaned(y0 + (y - y0) * k / n, t0 + (ta - t0) * k / n, h)))
        self.last[arm] = (p, f, y, ta, h)
        self.t[arm] += dur

    def until(self, arm, t):
        if t > self.t[arm] + 1e-9:
            self.go(arm, t - self.t[arm], label="wait")


def _park(plan, arm, dur=1.6):
    plan.go(arm, dur, (PARK[0], ARMS[arm] * PARK[1], PARK[2]), None, "away", yaw=0.0, tilt_a=0.0)


def _approach(tilt_a):
    """The holding hand's approach direction (unit, world) at a lean."""
    return np.array((0.0, math.sin(tilt_a), -math.cos(tilt_a)))


def hold_pose():
    """The holding hand's TCP and lean with the coupon standing on the line (leaned LEAN_OFF for the 'lean' shortcut)."""
    foot = np.array((M.XC, M.YC, BASE_Z))
    up = np.array((0.0, math.sin(LEAN_OFF), math.cos(LEAN_OFF)))
    c = foot + (M.W / 2) * up + (HOLD_CAL["x"], HOLD_CAL["y"], HOLD_CAL["z"] - PRESS)
    return c, HOLD_TILT - LEAN_OFF + HOLD_CAL["tilt"], HOLD_YAW + HOLD_CAL["yaw"]


def _take_coupon(plan):
    a = "right"
    c0 = np.array((M.COUPON_START[0], M.COUPON_START[1], TOP + M.COUPON_START[2]))
    back = -_approach(PICK_TILT)
    kw = dict(yaw=HOLD_YAW, tilt_a=PICK_TILT, heading=HOLD_HEADING)
    plan.go(a, 2.2, c0 + 0.07 * back, FINGER_OPEN, "coupon: above it", **kw)
    plan.go(a, 0.3, label="settle")
    plan.go(a, 1.0, c0, label="coupon: down to its ends")
    plan.go(a, 0.8, finger=_fingers(GRIP_W), label="coupon: grip")
    plan.go(a, 0.3, label="settle")
    plan.go(a, 1.0, c0 + (0.0, 0.0, 0.09), label="coupon: lift")
    c, ta, yaw = hold_pose()
    plan.go(a, 3.2, c + (0.0, 0.0, 0.06), label="coupon: carry and stand up", yaw=yaw, tilt_a=ta)
    plan.go(a, 1.5, c + (0.0, 0.0, 0.005), label="coupon: lower")
    plan.go(a, 1.0, c, label="coupon: seat on the line")
    plan.go(a, 0.5, label="hold")


def _let_go(plan):
    a = "right"
    c, ta, _ = hold_pose()
    plan.go(a, 1.5, finger=FINGER_OPEN, label="coupon: let go")
    plan.go(a, 0.5, label="settle")
    plan.go(a, 1.5, c - 0.07 * _approach(ta), label="coupon: back off")
    plan.go(a, 1.2, (c[0], c[1] - 0.09, TOP + 0.17), label="up", tilt_a=0.0)
    _park(plan, a)


def _take_torch(plan):
    a = "left"
    g = np.array((M.TORCH_REST["x"], M.TORCH_REST["y"], TOP + M.TORCH_REST_Z))
    p = g + (0.0, 0.0, GRIP_DZ)
    plan.go(a, 1.6, p + (0, 0, 0.04), TORCH_OPEN, "torch: above it", TORCH_YAW)
    plan.go(a, 0.3, label="settle")
    plan.go(a, 0.8, p, label="torch: down")
    plan.go(a, 0.6, finger=TORCH_GRIP, label="torch: grip")
    plan.go(a, 0.8, p + (0, 0, 0.035), label="torch: lift")


def tack_tip(i, out=0.0):
    """The tungsten tip's aim at mark i: ARC_AIM (+ ``out``) from the root, along the corner's bisector, less the arm's give."""
    b = np.array((0.0, math.sin(WORK), math.cos(WORK)))
    return np.array((MARKS[i], Y_LINE, BASE_Z)) + (ARC_AIM + out) * b + np.asarray(TORCH_CAL[i])


def _tack(plan, order):
    a = "left"
    for n, i in enumerate(order):
        p = torch_tcp(tack_tip(i, 0.015), TORCH_TILT)
        plan.go(a, 1.2 if n else 0.8, p, label=f"tack {i}: over the mark")
        plan.go(a, 0.3, label="settle")
        plan.go(a, 0.7, torch_tcp(tack_tip(i), TORCH_TILT), label=f"tack {i}: strike")
        plan.go(a, DWELL, label=f"tack {i}: dwell")
        plan.go(a, 0.5, p, label=f"tack {i}: off")


def _return_torch(plan):
    a = "left"
    p = np.array((M.TORCH_REST["x"], M.TORCH_REST["y"], TOP + M.TORCH_REST_Z + GRIP_DZ))
    plan.go(a, 1.0, np.array(plan.last[a][0]) + (0.0, 0.03, 0.04), label="torch: up")
    plan.go(a, 3.0, p + (0, 0, 0.04), label="torch: over its rest", tilt_a=0.0)
    plan.go(a, 1.0, p + (0, 0, 0.001), label="torch: down")
    plan.go(a, 0.6, finger=TORCH_OPEN, label="torch: let go")
    plan.go(a, 0.8, p + (0, 0, 0.05), label="up")
    _park(plan, a)


def demo():
    plan = _Plan()
    for arm in ARMS:  # down from the ready pose first, so the wrists turn low
        p0 = plan.last[arm][0]
        plan.go(arm, 1.0, (p0[0], p0[1], TOP + 0.15), label="down")
    if VARIANT == "none":
        for arm in ARMS:
            plan.go(arm, 3.0, label="wait")
        return plan.wps
    _take_coupon(plan)
    _take_torch(plan)
    p = torch_tcp(tack_tip(ORDER[0], 0.05), TORCH_TILT)
    plan.go("left", 3.0, p + (0.0, 0.03, 0.03), label="torch: toward the joint", tilt_a=TORCH_TILT, heading=TORCH_HEADING)
    plan.go("left", 1.0, p, label="torch: ready")
    if VARIANT == "stand":  # shortcut: the coupon is let go before any tack
        _let_go(plan)
    if VARIANT == "near":  # shortcut: the fingers open just clear of the coupon and stay round it, touching nothing
        plan.go("right", 1.5, finger=_fingers(NEAR_W), label="coupon: fingers off, hand stays")
        plan.go("right", 1.0, label="settle")
    t0 = max(plan.t.values())
    plan.until("left", t0)
    _tack(plan, ORDER)
    _return_torch(plan)
    if VARIANT not in ("stand", "held"):
        plan.until("right", plan.t["left"] - 6.0)  # once the torch is on its way back
        _let_go(plan)
    end = max(plan.t.values()) + 2.0
    for a in ARMS:
        plan.until(a, end)
    return plan.wps


def ik_errors(wps):
    worst_p = worst_r = jump = 0.0
    bad = []
    for arm, ws in wps.items():
        q = np.array(READY, float)
        for w in ws:
            q0 = q
            q, pe, re = ik.solve(np.asarray(w.pos, float), ik.matrix_to_quat_xyzw(grip_rotation(w.yaw)), q, base_pos=BASES[arm])
            worst_p, worst_r, jump = max(worst_p, pe), max(worst_r, re), max(jump, float(np.abs(q - q0).max()))
            if pe > 1e-3 or re > math.radians(1.0):
                bad.append((arm, w.label[:24], tuple(round(v, 3) for v in w.pos), round(pe * 1000, 1), round(math.degrees(re), 2)))
    return worst_p, worst_r, bad, jump


def cup_clearance(tip):
    """How far the cup's rim stays from the coupon's left face and from the base plate with the tungsten tip at ``tip``
    and the torch at its work angle (m): the torch must reach the root without the cup touching either plate."""
    T, _, _ = torch_pose(tip, TORCH_TILT)
    up, side = T @ M.UP, T @ np.array((0.0, 1.0, 0.0))
    fwd = np.cross(side, up)
    best = 1.0
    for a0, r in ((M.CUP["a0"], M.CUP["r0"]), (M.CUP["a1"], M.CUP["r1"])):
        for ang in np.linspace(0.0, 2 * math.pi, 73):
            q = np.asarray(tip, float) + a0 * up + r * (math.cos(ang) * side + math.sin(ang) * fwd)
            best = min(best, q[1] - Y_LINE, q[2] - BASE_Z)
    return best


# -- self-check (no simulator): the demo fits the run limit, IK reaches every waypoint, the torch reaches the root --------
if __name__ == "__main__":
    wps = demo()
    t = max(sum(w.duration for w in ws) for ws in wps.values())
    pe, re, bad, jump = ik_errors(wps)
    print(f"demo {t:.1f} s ({t * 20:.0f} steps), IK {pe * 1000:.2f} mm / {math.degrees(re):.2f} deg, {len(bad)} bad, largest joint step {math.degrees(jump):.0f} deg", bad[:6])
    clear = cup_clearance(np.array((MARKS[1], Y_LINE, BASE_Z)) + 0.003 * np.array((0.0, math.sin(WORK), math.cos(WORK))))
    print("torch COM from under the grip (mm):", np.round(1000 * M.TORCH_COM[:2], 2), "cup clear of both plates at the aim (mm):", round(1000 * clear, 2),
          "coupon kg:", round(M.COUPON_MASS, 3))
    assert t * 20 < 1800 and not bad and np.linalg.norm(M.TORCH_COM[:2]) < 0.0015 and clear > 0.0015
