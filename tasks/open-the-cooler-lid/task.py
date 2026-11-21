"""On the station table: a small personal drinks cooler, empty and dry, standing upright on a rubber mat, its lid
shut. Where the cooler stands and which way it faces changes between trials.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402  (before lots.py, so its reset hooks run first)
import lots  # noqa: E402
import tilt  # noqa: E402
from yam_arena.demo import FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

JADE, WHITE, LINER, GREY, MAT_GREY = ((0.05, 0.50, 0.46), (0.94, 0.94, 0.92), (0.88, 0.90, 0.90), (0.72, 0.73, 0.75),
                                      (0.16, 0.16, 0.17))

# The cooler: an Igloo Retro Playmate Mini 4 qt (item 00048535), 246 x 175 x 246 mm, 0.69 kg empty (published).
# Part frame: origin at the middle of the body's underside on the mat, z up, y along the cooler's length (the release
# button's end at +y), x across it. The lid is a white tent-top shell whose two end plates hang down over the body's
# ends to 58 mm above the mat; it turns on pins through the middle of the end plates, 125 mm up, so it swings over to
# either long side. Every size below that is not one of the three published ones is an estimate from the maker's photos.
HALF_W, HALF_L = 0.0875, 0.117  # body: 175 mm across, 234 mm long (+ two 6 mm end plates = 246 mm)
RIM = 0.160  # the body's top edge
PIVOT_Z = 0.125
PLATE_Y0, PLATE_Y1 = 0.117, 0.123
PLATE_Z0 = 0.058
HOLE = 0.012  # half the square opening in the button's end plate
BUTTON_R, BUTTON_PROUD, BUTTON_TRAVEL = 0.009, 0.008, 0.007
SKIRT_X0, SKIRT_X1, SKIRT_Z1 = 0.087, 0.091, 0.192  # the lid's long sides: 4 mm walls, 2.5 to 3.5 mm proud of the body
BODY_MASS, LID_MASS, BUTTON_MASS = 0.475, 0.200, 0.015  # 0.69 kg in all

BUTTON_K, BUTTON_PRELOAD = 150.0, 0.0167  # N/m; rest point 16.7 mm out: 2.5 N to start the press, 3.6 N fully in
PRESSED = 0.005  # the catch lets the lid turn while the button is in 5 mm or more
OPEN_Q = math.radians(90.0)  # the lid's stops: hanging down beside the body on either side
LOCK_Q = math.radians(0.5)  # the catch holds the lid within half a degree of shut
# Pin and seal friction: the lid's drive pulls toward a rest point that the lid drags along with it once it is more
# than DRAG_Q away, so the lid resists turning with up to DRAG_K * DRAG_Q = 0.025 N m and stays wherever it is left
# until gravity on its high shell beats that (about 10 degrees off shut).
DRAG_K, DRAG_Q = 1.25, 0.02


def _box(lo, hi, color):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return parts.box(tuple(float(v) for v in hi - lo), at=tuple(float(v) for v in (lo + hi) / 2), color=color)


T = 0.008  # body wall
BODY_SHAPES = [
    _box((-HALF_W, -HALF_L, 0.0), (HALF_W, HALF_L, 0.012), JADE),  # floor and foot
    _box((-HALF_W, -HALF_L, 0.012), (-HALF_W + T, HALF_L, RIM), JADE),  # long sides
    _box((HALF_W - T, -HALF_L, 0.012), (HALF_W, HALF_L, RIM), JADE),
    _box((-HALF_W + T, -HALF_L, 0.012), (HALF_W - T, -HALF_L + T, RIM), JADE),  # ends
    _box((-HALF_W + T, HALF_L - T, 0.012), (HALF_W - T, HALF_L, RIM), JADE),
    _box((-HALF_W + T, -HALF_L + T, 0.012), (HALF_W - T, HALF_L - T, 0.014), LINER),  # white liner floor
]


def _plate(sign, hole):
    y0, y1 = (PLATE_Y0, PLATE_Y1) if sign > 0 else (-PLATE_Y1, -PLATE_Y0)
    s = [_box((-HALF_W, y0, PLATE_Z0), (HALF_W, y1, PIVOT_Z - HOLE), WHITE),
         _box((-HALF_W, y0, PIVOT_Z + HOLE), (HALF_W, y1, RIM), WHITE),
         _box((-0.062, y0, RIM), (0.062, y1, 0.205), WHITE),
         _box((-0.025, y0, 0.205), (0.025, y1, 0.243), WHITE)]
    if hole:
        s += [_box((-HALF_W, y0, PIVOT_Z - HOLE), (-HOLE, y1, PIVOT_Z + HOLE), WHITE),
              _box((HOLE, y0, PIVOT_Z - HOLE), (HALF_W, y1, PIVOT_Z + HOLE), WHITE)]
    else:
        s += [_box((-HALF_W, y0, PIVOT_Z - HOLE), (HALF_W, y1, PIVOT_Z + HOLE), WHITE)]
    return s


LID_SHAPES = [
    *_plate(1, True), *_plate(-1, False),
    _box((-SKIRT_X1, -HALF_L, RIM), (-SKIRT_X0, HALF_L, SKIRT_Z1), WHITE),  # long sides of the lid
    _box((SKIRT_X0, -HALF_L, RIM), (SKIRT_X1, HALF_L, SKIRT_Z1), WHITE),
    _box((-0.078, -HALF_L, SKIRT_Z1), (0.078, HALF_L, 0.206), WHITE),  # the tent top, in three steps
    _box((-0.060, -HALF_L, 0.206), (0.060, HALF_L, 0.220), WHITE),
    _box((-0.040, -HALF_L, 0.220), (0.040, HALF_L, 0.232), WHITE),
    _box((-0.012, -0.078, 0.232), (0.012, -0.060, 0.246), WHITE),  # the carry handle: two posts and a grip
    _box((-0.012, 0.060, 0.232), (0.012, 0.078, 0.246), WHITE),
    _box((-0.012, -0.060, 0.240), (0.012, 0.060, 0.246), WHITE),
]
BUTTON_SHAPES = [parts.cylinder(BUTTON_R, 0.014, at=(0.0, PLATE_Y1 + BUTTON_PROUD - 0.007, PIVOT_Z), color=GREY,
                                axis="y")]

LID = parts.Hinge("lid", shapes=LID_SHAPES, pivot=(0.0, 0.0, PIVOT_Z), axis=(0.0, 1.0, 0.0), lower=-OPEN_Q,
                  upper=OPEN_Q, mass=LID_MASS, spring=DRAG_K, damping=0.02)
BUTTON = parts.Slider("button", shapes=BUTTON_SHAPES, axis=(0.0, -1.0, 0.0), travel=BUTTON_TRAVEL, mass=BUTTON_MASS,
                      spring=BUTTON_K, damping=0.5)

MAT_T = 0.003
MAT = parts.Part("mat", pos=(0.32, 0.0, TOP), shapes=[_box((-0.19, -0.17, 0.0), (0.19, 0.17, MAT_T), MAT_GREY)],
                 friction=(1.0, 0.9))
# Lot A (the screenshot): button end toward the robot, the cooler turned 12 degrees so that end points a little to
# the robot's left. Lot B: further away and to the left, turned 15 degrees the other way.
LOT_A = ((0.31, -0.02, TOP + MAT_T), math.radians(90.0 - 12.0))
LOT_B = ((0.33, 0.025, TOP + MAT_T), math.radians(90.0 + 15.0))
COOLER = parts.Part("cooler", pos=LOT_A[0], yaw=LOT_A[1], mass=BODY_MASS, shapes=BODY_SHAPES, joints=[LID, BUTTON],
                    friction=(0.8, 0.7))
OBJECTS = [MAT, COOLER]

LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A", place={"cooler": LOT_A[0]}, yaw={"cooler": LOT_A[1]}),
    lots.Lot("B", place={"cooler": LOT_B[0]}, yaw={"cooler": LOT_B[1]}),
)

EPS = 1e-6
OPEN_MIN = math.radians(80.0)  # open: the lid turned at least 80 degrees off shut, either way
BUTTON_OUT = 0.001  # the button back out (within 1 mm of its rest)
UPRIGHT_END = math.radians(3.0)  # at the end: standing upright...
SHIFT_END = 0.010  # ...within 10 mm of where it started
TIPPED = math.radians(10.0)  # whole trial: never tilted more than 10 degrees...
LIFTED = 0.010  # ...nor lifted more than 10 mm
CLEAR = 0.005  # let go: every gripper part more than 5 mm from every part of the cooler
FACE_STEP = 0.003
HOLD_S = 2.0
STILL_V, STILL_W, STILL_Q, STILL_B = 0.005, 0.05, 0.05, 0.005  # m/s, rad/s, rad/s (lid), m/s (button)
_HAND_LINKS = ("gripper", "tip_left", "tip_right")


def _rot(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _rz(a) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


# -- the catch ---------------------------------------------------------------------------------------------------------
# The side button is a spring plunger on the lid's pivot. Out, it locks the lid to the body within half a degree of
# shut; pushed in 5 mm or more, it frees the lid to swing either way to its stops. Let go once the lid has moved off
# shut, the plunger rides on the lid's pin boss and drops back in when the lid comes back to shut ("swivel it back
# until it clicks"). In the simulator: "locked" sets the lid's limits to -0.5 .. +0.5 degrees, "free" to -90 .. +90.
_mech = {"state": "locked", "limits": None, "anchor": 0.0, "seated": False}
_LIMITS = {"locked": (-LOCK_Q, LOCK_Q), "free": (-OPEN_Q, OPEN_Q)}


def next_state(state: str, q: float, b: float) -> str:
    """The catch's state after a control step with the lid at q (rad) and the button in b (m)."""
    if b >= PRESSED - EPS:
        return "free"
    if state == "free" and abs(q) <= LOCK_Q + EPS:
        return "locked"
    return state


def drag_anchor(anchor: float, q: float) -> float:
    """The friction drive's rest point after a control step: dragged along once the lid is more than DRAG_Q off it."""
    if q - anchor > DRAG_Q:
        return q - DRAG_Q
    if q - anchor < -DRAG_Q:
        return q + DRAG_Q
    return anchor


def _write_limits(sim, state: str) -> None:
    import torch

    if _mech["limits"] == state:
        return
    _mech["limits"] = state
    base = sim.env.unwrapped
    cooler = base.scene["cooler"]
    ids = cooler.find_joints(["lid"], preserve_order=True)[0]
    lo, hi = _LIMITS[state]
    lim = torch.tensor([[[lo, hi]]], dtype=torch.float32, device=base.device)
    cooler.write_joint_position_limit_to_sim_index(limits=lim, joint_ids=ids, warn_limit_violation=False)


# -- the trial record --------------------------------------------------------------------------------------------------
# Kept on the shared parts module, so every copy of this file loaded in one process reads and writes the same record.
_rec = parts.__dict__.setdefault("cooler_lid_record", {})
if not _rec:
    _rec.update(fault=False, gap=None, t=None, since=None, held=False, prev=None, start=None)


def _tilt(k) -> float:
    return math.acos(float(np.clip(_rot(k.quat_xyzw)[2, 2], -1.0, 1.0)))


def _known(k) -> bool:
    vals = np.r_[k.pos, k.quat_xyzw, k.lin_vel, k.ang_vel, k.joints.get("lid", np.nan), k.joints.get("button", np.nan)]
    return bool(np.isfinite(vals).all() and np.linalg.norm(k.quat_xyzw) > 1e-6)


def fault(k, start) -> bool:
    """Whole-trial rule: the cooler tilted more than 10 degrees or its body lifted more than 10 mm."""
    return bool(_tilt(k) > TIPPED + EPS or float(k.pos[2]) > float(start[2]) + LIFTED + EPS)


def open_end(k, start) -> bool:
    """The end state, apart from let-go and stillness: lid at least 80 degrees open, button back out, the cooler
    upright and within 10 mm of where it started."""
    if not _known(k):
        return False
    q, b = float(k.joints["lid"]), float(k.joints["button"])
    shift = float(np.linalg.norm(np.asarray(k.pos[:2], float) - np.asarray(start[:2], float)))
    return bool(abs(q) >= OPEN_MIN - EPS and b <= BUTTON_OUT + EPS and _tilt(k) <= UPRIGHT_END + EPS
                and shift <= SHIFT_END + EPS)


def _face_points(hull, step=FACE_STEP) -> np.ndarray:
    """Points on every face of a convex hull, about `step` apart (corners included)."""
    pts = [hull.vertices]
    for n, d in zip(hull.planes[:, :3], hull.planes[:, 3]):
        on = hull.vertices[np.abs(hull.vertices @ n - d) < 1e-6]
        if len(on) < 3:
            continue
        c = on.mean(axis=0)
        e1 = on[0] - c
        if np.linalg.norm(e1) < 1e-9:
            continue
        e1 /= np.linalg.norm(e1)
        e2 = np.cross(n, e1)
        ring = on[np.argsort(np.arctan2((on - c) @ e2, (on - c) @ e1))]
        for i in range(len(ring)):
            a, b = ring[i], ring[(i + 1) % len(ring)]
            m = max(1, int(math.ceil(max(np.linalg.norm(a - c), np.linalg.norm(b - c), np.linalg.norm(a - b)) / step)))
            for s_ in range(m + 1):
                for t_ in range(m + 1 - s_):
                    pts.append((c + (a - c) * (s_ / m) + (b - c) * (t_ / m))[None, :])
    pts = np.concatenate(pts)
    _, keep = np.unique(np.round(pts / (step / 2)).astype(np.int64), axis=0, return_index=True)
    return np.concatenate([hull.vertices, pts[np.sort(keep)]])


_FACES: dict = {}


def _link_faces(name):
    """Each hull of a hand link (link frame) with its face sample points, computed once."""
    if name not in _FACES:
        from yam_arena import analysis

        _FACES[name] = [(h, _face_points(h)) for h in analysis.link_hulls().get(name, [])]
    return _FACES[name]


def _link_poses(k) -> dict:
    out = {"base": (np.asarray(k.pos, float), _rot(k.quat_xyzw))}
    for name, (p, qx) in getattr(k, "links", {}).items():
        out[name] = (np.asarray(p, float), _rot(qx))
    return out


def gap(k, hulls) -> float:
    """The closest any point on a gripper hull's faces (sampled about every 3 mm, corners included) comes to any shape
    of the cooler, lid and button included (m; negative inside)."""
    from yam_arena import analysis

    pts = np.concatenate([getattr(h, "face_points", h.vertices) for h in hulls])
    near = np.linalg.norm(pts - np.asarray(k.pos, float), axis=1) < 0.45
    if not near.any():
        return 0.05
    pts = pts[near]
    boxes = COOLER.world_boxes(_link_poses(k))
    return float(min(analysis.box_signed_distance(pts, c, r, s).min() for _, c, r, s in boxes))


class _WorldHull:
    """A hand hull in the world: corners, face planes and face sample points."""

    def __init__(self, hull, faces, m):
        rot, pos = m[:3, :3], m[:3, 3]
        self.vertices = hull.vertices @ rot.T + pos
        n_w = hull.planes[:, :3] @ rot.T
        self.planes = np.c_[n_w, hull.planes[:, 3] + n_w @ pos]
        self.face_points = faces @ rot.T + pos


def _hands(sim):
    """Both arms' gripper and finger tip hulls in the world, or None unless all six hand links were found."""
    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    hulls, found = [], 0
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        poses = parts._np(asset.data.body_link_pose_w)[0]
        for i, n in enumerate(asset.body_names):
            if n in _HAND_LINKS:
                m = np.eye(4)
                m[:3, :3] = _rot(poses[i, 3:])
                m[:3, 3] = poses[i, :3] - origin
                link = [_WorldHull(h, f, m) for h, f in _link_faces(n)]
                found += 1 if link else 0
                hulls += link
    return hulls if found == 2 * len(_HAND_LINKS) else None


def still(k, prev, dt) -> bool:
    """The cooler at rest: body under 5 mm/s and 0.05 rad/s, lid turning under 0.05 rad/s, button under 5 mm/s."""
    if prev is None or not dt > 0:
        return False
    dq = abs(float(k.joints["lid"]) - prev[0]) / dt
    db = abs(float(k.joints["button"]) - prev[1]) / dt
    return bool(np.linalg.norm(k.lin_vel) < STILL_V and np.linalg.norm(k.ang_vel) < STILL_W and dq < STILL_Q
                and db < STILL_B)


def _hold(k, t, ok) -> None:
    """Count the 2 s hold: open, let go and still at this control step, continuously since `since`."""
    if not ok:
        _rec.update(since=None, held=False)
        return
    if _rec["since"] is None:
        _rec["since"] = t
    _rec["held"] = t - _rec["since"] >= HOLD_S - EPS


@parts.on_reset
def _new_trial(sim) -> None:
    _mech.update(state="locked", limits=None, anchor=0.0, seated=False)
    _rec.update(fault=False, gap=None, t=None, since=None, held=False, prev=None, start=None)
    _write_limits(sim, "locked")
    sim.set_target("cooler", "button", -BUTTON_PRELOAD)
    sim.set_target("cooler", "lid", 0.0)


def _seat_base(base) -> None:
    """Stand the cooler exactly where its lot puts it, still (a lot turning a loose articulation keeps its centre of
    mass where it was, which would leave the body off its place)."""
    import torch

    cooler = base.scene["cooler"]
    lot = lots.current()
    x, y, z = lot.pos("cooler")
    a = lot.yaw_of("cooler")
    pose = torch.tensor([[x, y, z, 0.0, 0.0, math.sin(a / 2), math.cos(a / 2)]], dtype=torch.float32,
                        device=base.device)
    pose[:, :3] += base.scene.env_origins[0][:3].to(pose.dtype)
    cooler.write_root_pose_to_sim_index(root_pose=pose)
    cooler.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), device=base.device))


_lots_apply = lots._apply


def _apply_and_seat(emb) -> bool:
    """The lots' move for a new trial, then the cooler stood exactly at its lot's pose, settled and rendered again."""
    _lots_apply(emb)
    base = emb._base
    _seat_base(base)
    for _ in range(max(1, round(0.2 / base.step_dt))):
        emb._env.step(emb._ready)
    for _ in range(lots.RENDERS):
        base.sim.render()
    return True


lots._apply = _apply_and_seat


@parts.on_step
def _step(sim) -> None:
    if not _mech["seated"]:  # a guard: stand the cooler again at the first step if the reset left it 1 mm off
        _mech["seated"] = True
        k0 = sim.states()["cooler"]
        if np.linalg.norm(np.asarray(k0.pos)[:2] - np.asarray(lots.current().pos("cooler")[:2])) > 0.001:
            _seat_base(sim.env.unwrapped)
    k = sim.states()["cooler"]
    t = float(sim.t)
    if not _known(k):  # unreadable state: no credit, now or later
        _rec.update(fault=True, gap=None, since=None, held=False, t=t)
        return
    q, b = float(k.joints["lid"]), float(k.joints["button"])
    _mech["state"] = next_state(_mech["state"], q, b)
    _write_limits(sim, _mech["state"])
    _mech["anchor"] = drag_anchor(_mech["anchor"], q)
    sim.set_target("cooler", "lid", _mech["anchor"])
    if _rec["start"] is None:
        _rec["start"] = tuple(float(v) for v in lots.current().pos("cooler"))
    if fault(k, _rec["start"]):
        _rec["fault"] = True
    prev, t_prev = _rec["prev"], _rec["t"]
    _rec["prev"], _rec["t"] = (q, b), t
    hulls = _hands(sim)
    if hulls is None:  # the arms could not be read: no credit at this step
        _rec.update(gap=None, since=None, held=False)
        return
    _rec["gap"] = gap(k, hulls)
    dt = t - t_prev if t_prev is not None else 0.0
    ok = (_rec["gap"] > CLEAR + EPS and not _rec["fault"] and open_end(k, _rec["start"]) and still(k, prev, dt))
    _hold(k, t, ok)


# -- scoring -----------------------------------------------------------------------------------------------------------


def score(objects):
    """1.0 when the lid is open at least 80 degrees with the button back out, the cooler upright and within 10 mm of
    its start, both grippers more than 5 mm from it, and all of it still like that for 2 s; and the cooler never tilted
    more than 10 degrees or lifted more than 10 mm during the trial. Otherwise 0; with no measurement the score is 0."""
    try:
        k = objects["cooler"]
        start = _rec["start"]
        released = _rec["gap"] is not None and _rec["gap"] > CLEAR + EPS
        at_rest = bool(np.linalg.norm(k.lin_vel) < STILL_V and np.linalg.norm(k.ang_vel) < STILL_W)
        ok = bool(start is not None and _rec["held"] and released and not _rec["fault"] and open_end(k, start)
                  and at_rest)
        return 1.0 if ok else 0.0
    except (KeyError, TypeError, ValueError, AttributeError):
        return 0.0


def success(objects):
    """The lid stands open, let go, and the cooler is upright, in place and still for 2 s."""
    return score(objects) == 1.0


def distance(objects):
    return float(OPEN_Q - abs(objects["cooler"].joints["lid"]))


# -- demo --------------------------------------------------------------------------------------------------------------
# One arm presses the release button on the end that faces the robot, its finger tips leaned 45 degrees toward the
# button, and holds it in. The other arm, fingers closed and leaned 30 degrees toward the lid, pushes the lid's nearer long side low down,
# just above the body, and rolls the lid about 16 degrees over, away from itself. The button arm lets go once the lid
# has left shut; the pushing arm backs off and the lid falls over to its stop under its own weight. The demo uses
# only the cooler's start pose as the cameras show it: which way the button end points decides which arm presses it.
PRESS_TILT = 45.0
TIP_FRONT, TIP_UP = 0.0135, -0.004  # the leaned closed tips' leading point, ahead of and below the TCP (YAM hulls)
PRESS_IN = 0.026  # aimed this far past the button's face: about 6 mm of travel once the arm gives way
PUSH_TILT = 30.0
PUSH_FRONT, PUSH_UP = 0.0122, -0.007  # the pusher's leaned closed tips' leading point, from the TCP (YAM hulls)
PUSH_Z = 0.170  # where the leaned tips meet the lid's long side, above the mat (its lower edge is at 160 mm)
PUSH_AT = -0.045  # where along the lid's long side the pusher works (part y, m)
PUSH_LEAD = 0.010  # aimed this far into the lid while rolling it (the arm gives way)
PUSH_TO = math.radians(16.0)


def _layout():
    """The cooler's pose as the cameras show it, and which arm does what."""
    lot = lots.current()
    p = np.asarray(lot.pos("cooler"), float)
    r = _rz(lot.yaw_of("cooler"))
    end = r[:, 1]  # where the button end points
    presser = "left" if end[1] > 0 else "right"
    pusher = "right" if presser == "left" else "left"
    toward = 1.0 if pusher == "left" else -1.0
    side = 1.0 if r[1, 0] * toward > 0 else -1.0  # the part-x sign of the lid face that faces the pusher
    return p, r, presser, pusher, side


def _lid_point(local, q):
    """A point of the lid (part frame at q = 0) with the lid turned q about the pins."""
    c, s = math.cos(q), math.sin(q)
    x, y, z = local[0], local[1], local[2] - PIVOT_Z
    return np.array([c * x + s * z, y, -s * x + c * z + PIVOT_Z])


def demo():
    """The arm the button end faces presses and holds the release button; the other arm rolls the lid over off its
    nearer long side; the button arm lets go, the pusher backs off and the lid falls open."""
    p, r, presser, pusher, side = _layout()

    def w(local):
        return p + r @ np.asarray(local, float)

    # The presser: tips leaned 45 degrees toward the button, along the cooler's length.
    d = r @ np.array([0.0, -1.0, 0.0])
    heading = math.atan2(d[1], d[0])
    face = w((0.0, PLATE_Y1 + BUTTON_PROUD, PIVOT_Z))

    def press_tcp(into):
        return face + (into - TIP_FRONT) * d - np.array([0.0, 0.0, TIP_UP])

    lean = dict(tilt=PRESS_TILT, heading=heading)
    park_p = (0.18, 0.24 if presser == "left" else -0.24, TOP + 0.16)
    t_press = 2.0 + 2.0 + 1.5 + 1.2
    press = [
        tilt.Waypoint(2.0, tuple(press_tcp(-0.06) + np.array([0, 0, 0.06])), 0.0, "above the button end", heading, **lean),
        tilt.Waypoint(2.0, tuple(press_tcp(-0.03)), 0.0, "in front of the button", heading, **lean),
        tilt.Waypoint(1.5, tuple(press_tcp(-0.002)), 0.0, "fingertips on the button", heading, **lean),
        tilt.Waypoint(1.2, tuple(press_tcp(PRESS_IN)), 0.0, "press the button in", heading, **lean),
    ]
    # The pusher: fingers closed, tips leaned 30 degrees toward the lid's nearer long side, low down on it.
    n_out = side * r[:, 0]  # outward normal of the face it pushes
    push_heading = math.atan2(-n_out[1], -n_out[0])
    push_lean = dict(tilt=PUSH_TILT, heading=push_heading)
    contact0 = np.array([side * SKIRT_X1, PUSH_AT, PUSH_Z])

    def push_tcp(q, lead):
        c = _lid_point(contact0, -side * q)  # the lid turns away from the pusher
        return w(c) + n_out * (PUSH_FRONT - lead) - np.array([0.0, 0.0, PUSH_UP])

    park_q = (0.22, -0.24 if presser == "left" else 0.24, TOP + 0.14)
    t_ready = t_press + 0.8
    pushes = [
        Waypoint(t_ready - 3.5, park_q, 0.0, "wait, fingers closed", 0.0),
        tilt.Waypoint(2.0, tuple(push_tcp(0.0, -0.04) + np.array([0, 0, 0.05])), 0.0, "beside the lid", push_heading, **push_lean),
        tilt.Waypoint(1.5, tuple(push_tcp(0.0, -0.01)), 0.0, "down beside the lid's long side", push_heading, **push_lean),
    ]
    for i in range(1, 9):
        pushes.append(tilt.Waypoint(0.5, tuple(push_tcp(PUSH_TO * i / 8, PUSH_LEAD)), 0.0, "roll the lid over", push_heading, **push_lean))
    t_rolled = sum(x.duration for x in pushes)
    press += [
        tilt.Waypoint(t_rolled - t_press - 2.0, tuple(press_tcp(PRESS_IN)), 0.0, "hold the button in", heading, **lean),
        tilt.Waypoint(1.0, tuple(press_tcp(-0.03)), 0.0, "let the button out", heading, **lean),
        tilt.Waypoint(1.0, tuple(press_tcp(-0.06) + np.array([0, 0, 0.06])), 0.0, "back off", heading, **lean),
        Waypoint(2.0, park_p, FINGER_OPEN, "park", 0.0),
        Waypoint(6.0, park_p, FINGER_OPEN, "wait", 0.0),
    ]
    pushes += [
        tilt.Waypoint(1.0, tuple(push_tcp(PUSH_TO, -0.03)), 0.0, "back off the lid", push_heading, **push_lean),
        tilt.Waypoint(1.5, tuple(push_tcp(PUSH_TO, -0.06) + np.array([0, 0, 0.04])), 0.0, "away", push_heading, **push_lean),
        Waypoint(2.0, park_q, FINGER_OPEN, "park", 0.0),
        Waypoint(6.0, park_q, FINGER_OPEN, "wait", 0.0),
    ]
    return {presser: press, pusher: pushes}
