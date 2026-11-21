"""A cropped section of a small window is fixed on the bench in front of the robot: the wall below it with its
window stool (the inside sill board), the two jambs, the head, and the stop that frames the opening from behind, with
the glass beyond. Its aluminium insect screen leans back against the front edge of the stool, standing on the bench,
mesh toward the robot. Near each bottom corner, on its room-side face, the screen has a spring latch: a black nylon
housing with a fixed lug and a tab beside it. Where the screen stands along the stool changes between trials.
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402  (before lots.py, so its reset hooks run first)
import lots  # noqa: E402
import tilt  # noqa: E402  (the fingers lean in toward the screen, under the wrist)
from yam_arena.demo import FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
EPS = 1e-6

ALU = (0.78, 0.79, 0.80)  # mill-finish aluminium screen frame
MESH = (0.22, 0.23, 0.25)  # charcoal fibreglass mesh
NYLON = (0.05, 0.05, 0.05)  # black nylon latch housing, lug and tab
STEEL = (0.55, 0.56, 0.58)  # latch bolt
WHITE = (0.93, 0.93, 0.91)  # painted wood / white vinyl window parts
STOP_C = (0.70, 0.62, 0.50)  # the stop, a contrasting wood tone so the opening's outline reads from above
GLASS = (0.62, 0.74, 0.80)
WALL = (0.86, 0.84, 0.78)


def _f(v):
    """Plain Python floats: a position of NumPy floats crashes the simulator at reset."""
    return tuple(float(x) for x in v)


# -- the screen (part frame: origin at the middle of its bottom edge, mid-thickness; x toward the window) -------------
W, H = 0.406, 0.356  # 16 x 14 in outside size
T, FACE = 0.0111, 0.0191  # 7/16 x 3/4 in roll-formed frame profile
SCREEN_KG = 0.22  # estimate: 1.73 m of frame, mesh, corners and two latches
B_Z = 0.045  # latch bolts' centre height above the bottom edge
BOLT_D = 0.0056  # 7/32 in bolt
BOLT_OUT = 0.0095  # 3/8 in proud of the side edge when extended
TRAVEL = 0.0115  # the tab's travel; squeezed shut, the bolt sits 2 mm inside the side edge
TAB = (0.030, 0.006, 0.028)  # tab and lug: proud of the housing (x), thick (y), tall (z)
HOUSE_T = 0.003  # housing plate on the room-side face
Y_TAB = W / 2 - 0.019  # tab centre at rest (each side, mirrored)
Y_LUG = Y_TAB - TAB[1] - TRAVEL  # fixed lug centre: the tab closes onto it
LATCH_SPRING = 250.0  # N/m (estimate): about 1 N preload, 3.9 N squeezed shut
LATCH_PRELOAD = 0.004  # m past the extended stop the spring's rest point lies
FACE_X = -T / 2  # the room-side face
TAB_X = (FACE_X - HOUSE_T - TAB[0] / 2)  # tab/lug centre (x)


def _screen_shapes():
    out = []
    for s in (-1, 1):  # stiles
        out.append(parts.box((T, FACE, H), at=(0.0, s * (W / 2 - FACE / 2), H / 2), color=ALU))
    for z in (FACE / 2, H - FACE / 2):  # rails
        out.append(parts.box((T, W - 2 * FACE, FACE), at=(0.0, 0.0, z), color=ALU))
    out.append(parts.box((0.001, W - 2 * FACE, H - 2 * FACE), at=(0.0, 0.0, H / 2), color=MESH))
    for s in (-1, 1):  # latch housings and fixed lugs
        out.append(parts.box((HOUSE_T, 0.045, 0.065), at=(FACE_X - HOUSE_T / 2, s * (W / 2 - 0.0225), 0.0325),
                             color=NYLON))
        out.append(parts.box(TAB, at=(TAB_X, s * Y_LUG, B_Z), color=NYLON))
    return out


def _latch(side: int) -> parts.Slider:
    """One spring latch: tab and bolt on one slider; squeezing the tab toward the lug draws the bolt in."""
    bolt_len = BOLT_OUT + 0.020
    shapes = (parts.box(TAB, at=(TAB_X, side * Y_TAB, B_Z), color=NYLON),
              parts.box((BOLT_D, bolt_len, BOLT_D), at=(0.0, side * (W / 2 + BOLT_OUT - bolt_len / 2), B_Z),
                        color=STEEL))
    return parts.Slider("latch_l" if side > 0 else "latch_r", shapes=shapes, axis=(0.0, -float(side), 0.0),
                        travel=TRAVEL, mass=0.02, spring=LATCH_SPRING, damping=2.0, max_effort=60.0,
                        target=-LATCH_PRELOAD)


# -- the window section (world) --------------------------------------------------------------------------------------
X_STOP = 0.400  # the stop's room-side face: the screen's back face seats here
PLAY = 0.003  # side play between the screen and each jamb
HEAD_GAP = 0.010  # room above the seated screen's top edge
DEPTH = 0.045  # jamb depth from the wall face to the screen's room-side face
X_WALL = X_STOP - T - DEPTH  # room-side face of the jambs and head
STOOL = 0.060  # the stool projects this far in front of the wall
X_STOOL = X_WALL - STOOL
Z_SILL = TOP + 0.070  # stool top = the opening's sill
Z_HEAD = Z_SILL + H + HEAD_GAP
Y_JAMB = W / 2 + PLAY  # jambs' inner faces
JAMB_W = 0.040
X_BACK = X_STOP + 0.040  # back of the cropped section
STOP_LAP = 0.010  # the stop covers the screen's edge by this much on every side
STOP_T = 0.012
HOLE_X = X_STOP - T / 2  # latch holes in the jambs, centred on the seated bolts
HOLE_Z = Z_SILL + B_Z
HOLE_W, HOLE_H, HOLE_DEPTH = 0.010, 0.008, 0.012


def _box(lo, hi, color):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return parts.box(_f(hi - lo), at=_f((lo + hi) / 2 - WIN_ORIGIN), color=color)


WIN_ORIGIN = np.array([X_STOP, 0.0, Z_SILL])


def _window_shapes():
    out = [_box((X_STOOL, -0.37, TOP), (X_BACK, 0.37, Z_SILL), WALL)]  # wall below, its top the stool and sill
    out.append(_box((X_STOOL, -0.37, Z_SILL - 0.003), (X_WALL, 0.37, Z_SILL), WHITE))  # stool board (top layer)
    for s in (-1, 1):
        def jb(xl, xh, yl, yh, zl, zh, c=WHITE):
            ys = sorted((s * yl, s * yh))
            out.append(_box((xl, ys[0], zl), (xh, ys[1], zh), c))
        jb(X_WALL, X_BACK, Y_JAMB + HOLE_DEPTH, Y_JAMB + JAMB_W, Z_SILL, Z_HEAD)  # jamb body
        jb(X_WALL, HOLE_X - HOLE_W / 2, Y_JAMB, Y_JAMB + HOLE_DEPTH, Z_SILL, Z_HEAD)  # in front of the hole
        jb(HOLE_X + HOLE_W / 2, X_STOP, Y_JAMB, Y_JAMB + HOLE_DEPTH, Z_SILL, Z_HEAD)  # behind it, up to the stop
        jb(HOLE_X - HOLE_W / 2, HOLE_X + HOLE_W / 2, Y_JAMB, Y_JAMB + HOLE_DEPTH, Z_SILL, HOLE_Z - HOLE_H / 2)
        jb(HOLE_X - HOLE_W / 2, HOLE_X + HOLE_W / 2, Y_JAMB, Y_JAMB + HOLE_DEPTH, HOLE_Z + HOLE_H / 2, Z_HEAD)
        jb(X_STOP, X_STOP + STOP_T, W / 2 - STOP_LAP, Y_JAMB + HOLE_DEPTH, Z_SILL, Z_HEAD, STOP_C)  # side stop
        jb(X_STOP + STOP_T, X_BACK, Y_JAMB - 0.002, Y_JAMB + HOLE_DEPTH, Z_SILL, Z_HEAD)  # sash pocket wall
    out.append(_box((X_STOP, -Y_JAMB, Z_SILL), (X_STOP + STOP_T, Y_JAMB, Z_SILL + STOP_LAP), STOP_C))  # sill stop
    out.append(_box((X_STOP, -Y_JAMB, Z_SILL + H - STOP_LAP), (X_STOP + STOP_T, Y_JAMB, Z_HEAD), STOP_C))  # head stop
    out.append(_box((X_WALL, -(Y_JAMB + JAMB_W), Z_HEAD), (X_BACK, Y_JAMB + JAMB_W, Z_HEAD + 0.035), WHITE))  # head
    out.append(_box((X_STOP + 0.024, -Y_JAMB, Z_SILL), (X_STOP + 0.030, Y_JAMB, Z_HEAD), GLASS))  # glass
    return out


WINDOW = parts.Part("window", pos=_f(WIN_ORIGIN), shapes=_window_shapes(), friction=(0.6, 0.5),
                    contact_offset=0.002)

# -- start: the screen leans back against the stool's front edge ------------------------------------------------------
LEAN = math.radians(10.0)


def _ry(a) -> np.ndarray:
    return parts._rot((0.0, 1.0, 0.0), a)


def lean_origin(y=0.0, lean=LEAN) -> np.ndarray:
    """The screen's origin (bottom-edge middle) when it stands on the bench leaning back ``lean`` against the stool's
    front edge, centred on ``y``."""
    z0 = TOP + (T / 2) * math.sin(lean)
    s = (Z_SILL - z0 + (T / 2) * math.sin(lean)) / math.cos(lean)  # along the back face up to the stool edge
    x0 = X_STOOL - (T / 2) * math.cos(lean) - s * math.sin(lean) - 0.0005
    return np.array([x0, y, z0 + 0.0005])


def _quat(m) -> tuple:
    return tuple(float(v) for v in parts._matrix_to_quat(m))


START_A = lean_origin(0.0)
SCREEN = parts.Part("screen", pos=_f(START_A), quat=_quat(_ry(LEAN)), mass=SCREEN_KG, shapes=_screen_shapes(),
                    joints=[_latch(+1), _latch(-1)], friction=(0.6, 0.5), contact_offset=0.002)
SEAT = np.array([X_STOP - T / 2, 0.0, Z_SILL])  # the seated screen's origin

OBJECTS = [WINDOW, SCREEN]
LOTS = lots.declare(OBJECTS, lots.Lot("A", place={"screen": _f(START_A)}),
                    lots.Lot("B", place={"screen": _f(lean_origin(0.025))}),
                    lots.Lot("C", place={"screen": _f(lean_origin(-0.025))}))
SUCCESS_HOLD_S = 1.5  # the 1 s hold is enforced in score() itself; this only keeps a run going past success


# -- scoring geometry ----------------------------------------------------------------------------------------------
def _rot(q) -> np.ndarray:
    return parts._quat_to_matrix(q)


def _local(st, p) -> np.ndarray:
    """A screen-frame point in the world."""
    return np.asarray(st.pos, float) + _rot(st.quat_xyzw) @ np.asarray(p, float)


CORNERS = [(T / 2, s * W / 2, z) for s in (-1, 1) for z in (0.0, H)]  # back-face corners
FLUSH_TOL = 0.002  # every back corner within 2 mm of the stop's face
SILL_TOL = 0.0015  # bottom corners within 1.5 mm above the sill (1 mm below allowed for contact)
TILT_TOL = 2.0  # degrees from upright, square to the opening
LATCH_IN = 0.003  # a latched bolt reaches at least 3 mm into its hole past the jamb face (6.5 mm when centred)


def seat_measures(st):
    """(worst back-corner distance from the stop's face, m; bottom corners' height range above the sill, m;
    tilt from upright in the window's plane and out of it, deg; worst side-edge overrun past a jamb face, m)."""
    back = [float(_local(st, c)[0] - X_STOP) for c in CORNERS]
    bottom = [float(_local(st, c)[2] - Z_SILL) for c in CORNERS if c[2] == 0.0]
    up = _rot(st.quat_xyzw)[:, 2]
    tl = math.degrees(math.acos(max(-1.0, min(1.0, float(up[2])))))
    side = max(abs(float(_local(st, (x, s * W / 2, z))[1])) - Y_JAMB for x in (-T / 2, T / 2) for s in (-1, 1)
               for z in (0.0, H))
    return max(abs(b) for b in back), (min(bottom), max(bottom)), tl, side


def seated(st) -> bool:
    flush, (blo, bhi), tl, side = seat_measures(st)
    return (flush <= FLUSH_TOL + EPS and blo >= -0.001 - EPS and bhi <= SILL_TOL + EPS and tl <= TILT_TOL + EPS
            and side <= 0.0005 + EPS)


def bolt_depth(st, joint) -> float:
    """How far the bolt's end reaches into its jamb hole past the jamb's face (m), 0 if it is not in the hole."""
    side = 1 if joint == "latch_l" else -1
    q = float(st.joints[joint])
    tip = _local(st, (0.0, side * (W / 2 + BOLT_OUT - q), B_Z))
    in_hole = abs(tip[0] - HOLE_X) <= HOLE_W / 2 - BOLT_D / 2 + 0.0005 + EPS and \
        abs(tip[2] - HOLE_Z) <= HOLE_H / 2 - BOLT_D / 2 + 0.0005 + EPS
    depth = side * float(tip[1]) - Y_JAMB
    return depth if in_hole and depth > 0 else 0.0


def latched(st, joint) -> bool:
    return bolt_depth(st, joint) >= LATCH_IN - EPS


def tier(objects) -> float:
    """End-state tier before the release and stillness rules: 1.0 seated on all four sides with both latches in their
    holes; 0.5 seated with one latch in; 0 otherwise."""
    st = objects["screen"]
    if not seated(st):
        return 0.0
    n = sum(latched(st, j) for j in ("latch_l", "latch_r"))
    return {2: 1.0, 1: 0.5}.get(n, 0.0)


# -- trial record ----------------------------------------------------------------------------------------------------
CLEAR = 0.005
STILL_M, HOLD_S = 0.001, 1.0
DROP_V, DROP_GAP = 0.5, 0.020  # falling faster than 0.5 m/s with every hand point 2 cm or more away: dropped
_rec = parts.__dict__.setdefault("insect_screen_record", {})


def reset_record() -> None:
    _rec.clear()
    _rec.update(steps=0, known=True, reason="", gap=None, since=None, held=False, snap=None, t=0.0, tier=0.0,
                dropped=False, last_z=None, last_t=None)


if not _rec:
    reset_record()


def _snap(objects) -> np.ndarray:
    st = objects["screen"]
    pts = [_local(st, c) for c in CORNERS]
    return np.concatenate(pts + [np.array([float(st.joints["latch_l"]), float(st.joints["latch_r"])])])


def observe(objects, gap, t) -> None:
    """One control step. ``gap``: closest sampled hand point to the screen (m), or None when the hands could not be
    read, which ends all credit for the trial."""
    _rec["steps"] += 1
    try:
        st = objects["screen"]
        vals = [*st.pos, *st.quat_xyzw, st.joints["latch_l"], st.joints["latch_r"]]
        if not all(math.isfinite(float(v)) for v in vals):
            raise ValueError("screen state unknown")
        if gap is None or not math.isfinite(gap):
            raise ValueError("hand state unknown")
        z = float(st.pos[2])
        if _rec["last_z"] is not None and t > _rec["last_t"]:
            vz = (z - _rec["last_z"]) / (t - _rec["last_t"])
            if vz < -DROP_V and gap >= DROP_GAP - EPS:
                _rec["dropped"] = True
        _rec["last_z"], _rec["last_t"] = z, float(t)
        tr = tier(objects)
        snap = _snap(objects)
    except (KeyError, ValueError, TypeError, AttributeError, IndexError) as e:
        _rec.update(known=False, reason=str(e), since=None, held=False)
        return
    _rec["t"] = float(t)
    _rec["gap"] = float(gap)
    if tr <= 0.0 or gap <= CLEAR + EPS:
        _rec.update(since=None, held=False, snap=None, tier=tr)
    elif _rec["since"] is None or tr != _rec["tier"] or float(np.abs(snap - _rec["snap"]).max()) > STILL_M + EPS:
        _rec.update(since=float(t), held=False, snap=snap, tier=tr)
    else:
        _rec["held"] = float(t) - _rec["since"] >= HOLD_S - EPS


def score(objects):
    """0 for the whole trial if the hands or the screen could not be read at any step, or if the screen was ever
    dropped. Otherwise, with every hand point more than 5 mm from the screen and nothing moving more than 1 mm for
    1 s: 1.0 seated on all four sides with both latch bolts in their holes, 0.5 seated with one in, 0 otherwise."""
    try:
        if not _rec["known"] or _rec["dropped"] or _rec["steps"] == 0 or not _rec["held"]:
            return 0.0
        if _rec["gap"] is None or _rec["gap"] <= CLEAR + EPS:
            return 0.0
        tr = tier(objects)
        if tr != _rec["tier"]:
            return 0.0
        if float(np.abs(_snap(objects) - _rec["snap"]).max()) > STILL_M + EPS:
            return 0.0
        return tr
    except (KeyError, ValueError, TypeError, AttributeError, IndexError):
        return 0.0


def success(objects):
    return score(objects) >= 1.0 - 1e-9


def distance(objects):
    try:
        st = objects["screen"]
        return float(np.linalg.norm(np.asarray(st.pos, float) - SEAT))
    except (KeyError, ValueError, TypeError, AttributeError):
        return 1.0


# -- hands: hull faces sampled every 3 mm ----------------------------------------------------------------------------
_HAND_LINKS = ("gripper", "tip_left", "tip_right")
_FACES: dict = {}


def _hull_face_points(hull, step=0.003) -> np.ndarray:
    pts = [hull.vertices]
    for n, d in zip(hull.planes[:, :3], hull.planes[:, 3]):
        on = hull.vertices[np.abs(hull.vertices @ n - d) < 1e-6]
        if len(on) < 3:
            continue
        c = on.mean(axis=0)
        e1 = on[0] - c
        if np.linalg.norm(e1) < 1e-9:
            continue
        e1 = e1 / np.linalg.norm(e1)
        e2 = np.cross(n, e1)
        ring = on[np.argsort(np.arctan2((on - c) @ e2, (on - c) @ e1))]
        for i in range(len(ring)):
            a, b = ring[i], ring[(i + 1) % len(ring)]
            m = max(1, int(math.ceil(max(np.linalg.norm(a - c), np.linalg.norm(b - c), np.linalg.norm(a - b)) / step)))
            ii, jj = np.meshgrid(np.arange(m + 1), np.arange(m + 1), indexing="ij")
            keep = (ii + jj) <= m
            s_, t_ = ii[keep][:, None] / m, jj[keep][:, None] / m
            pts.append(c + (a - c) * s_ + (b - c) * t_)
    pts = np.concatenate(pts)
    _, idx = np.unique(np.round(pts / (step / 2)).astype(np.int64), axis=0, return_index=True)
    return np.concatenate([hull.vertices, pts[np.sort(idx)]])


def link_points(name) -> np.ndarray:
    if name not in _FACES:
        from yam_arena import analysis

        hs = analysis.link_hulls().get(name, [])
        if not hs:
            raise ValueError(f"no collision hull for {name}")
        _FACES[name] = np.concatenate([_hull_face_points(h) for h in hs])
    return _FACES[name]


def screen_boxes(st):
    poses = {n: (np.asarray(p, float), _rot(q)) for n, (p, q) in st.links.items()}
    poses["base"] = (np.asarray(st.pos, float), _rot(st.quat_xyzw))
    return [(c, r, s) for _, c, r, s in SCREEN.world_boxes(poses)]


def hand_gap(objects, pts) -> float:
    """Closest hand point to any part of the screen, latches included (m; negative inside)."""
    from yam_arena.analysis import box_signed_distance

    st = objects["screen"]
    near = np.linalg.norm(pts - np.asarray(st.pos, float), axis=1) < 0.70
    pts = pts[near]
    if len(pts) == 0:
        return 1.0
    return min(float(box_signed_distance(pts, c, r, s).min()) for c, r, s in screen_boxes(st))


def _hand_points(sim):
    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    out, found = [], 0
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        poses = parts._np(asset.data.body_link_pose_w)[0]
        for i, n in enumerate(asset.body_names):
            if n in _HAND_LINKS:
                out.append(link_points(n) @ _rot(poses[i, 3:]).T + (poses[i, :3] - origin))
                found += 1
    if found != 2 * len(_HAND_LINKS):
        return None
    pts = np.concatenate(out)
    return pts if np.isfinite(pts).all() else None


# -- simulator hooks -------------------------------------------------------------------------------------------------
@parts.on_reset
def _new_trial(sim) -> None:
    reset_record()
    _rec["hook_t"] = None


@parts.on_step
def _step(sim) -> None:
    if _rec.get("hook_t") == sim.t:
        return
    _rec["hook_t"] = sim.t
    try:
        objects = sim.states()
    except Exception:  # noqa: BLE001  unknown state: no credit
        objects = {}
    g = None
    try:
        pts = _hand_points(sim)
        g = hand_gap(objects, pts) if pts is not None else None
    except Exception:  # noqa: BLE001
        g = None
    observe(objects, g, float(sim.t))
    log = str(pathlib.Path(os.environ["TRACE_OUT"]).with_name("screen.jsonl")) if os.environ.get("TRACE_OUT") else None
    if log and _rec["steps"] % 5 == 0 and objects:
        try:
            st = objects["screen"]
            flush, (blo, bhi), tl, side = seat_measures(st)
            row = dict(t=round(float(sim.t), 3), lot=lots.current().name, score=score(objects), tier=tier(objects),
                       pos=np.round(np.asarray(st.pos), 4).tolist(), flush=round(flush, 4),
                       bottom=[round(blo, 4), round(bhi, 4)], tilt=round(tl, 2), side=round(side, 4),
                       latch=[round(float(st.joints["latch_l"]), 4), round(float(st.joints["latch_r"]), 4)],
                       depth=[round(bolt_depth(st, "latch_l"), 4), round(bolt_depth(st, "latch_r"), 4)],
                       gap=_rec["gap"], held=_rec["held"], dropped=_rec["dropped"], known=_rec["known"],
                       reason=_rec["reason"])
            base = sim.env.unwrapped
            origin = parts._np(base.scene.env_origins[0])
            for arm in ("left_robot", "right_robot"):
                a_ = base.scene[arm]
                poses = parts._np(a_.data.body_link_pose_w)[0]
                qj = parts._np(a_.data.joint_pos)[0]
                names = list(a_.joint_names)
                gi = list(a_.body_names).index("gripper")
                tcp = poses[gi, :3] - origin + _rot(poses[gi, 3:]) @ np.array([0.0, 0.0, 0.134])
                row[arm[0]] = dict(tcp=np.round(tcp, 4).tolist(),
                                   f=[round(float(qj[names.index(n)]), 4) for n in ("joint7", "joint8") if n in names])
            with open(log, "a") as f:
                f.write(json.dumps(row) + "\n")
        except Exception as e:  # noqa: BLE001
            with open(log, "a") as f:
                f.write(json.dumps({"err": str(e)}) + "\n")


# -- demo: open loop from the published layout --------------------------------------------------------------------
LEAN_IN = 40.0  # degrees the fingertips lean in toward the screen (keeps the wrist clear of the mesh above)
GRIP_DX, GRIP_DZ = -0.024, 0.008  # TCP from the pinch's centre, screen frame (room side, up)
F_OPEN, F_SQUEEZE, F_LET = -0.024, -0.0005, -0.017
Y_MID = (Y_LUG - TAB[1] / 2 + Y_TAB - TRAVEL + TAB[1] / 2) / 2  # middle of the squeezed tab and lug
LIFT = 0.006  # bottom edge carried this far above the sill while it goes in
PUSH = 0.0015  # pushed this far past the stop's face (the arms give a little)


def grip_pose(origin, lean, side, dx=0.0, dz=0.0, dy=0.0):
    """TCP position and rotation for the pinch on one latch with the screen's origin at ``origin``, leaning ``lean``
    (rad, top toward the window). dx, dz: extra offset in the screen frame; dy: toward the side edge."""
    r = _ry(lean)
    p = np.asarray(origin, float) + r @ np.array([TAB_X + GRIP_DX + dx, side * (Y_MID + dy), B_Z + GRIP_DZ + dz])
    rot = r @ tilt.rotation(0.0, LEAN_IN, 0.0)
    return p, rot


def _rest_pose(arm):
    from yam_arena import ik, station
    from yam_arena.demo import READY

    pos, rot = ik.tcp_pose(np.array(READY, float), station.BASES[arm])
    return tuple(float(v) for v in pos), tilt._TiltedYaw(0.0, rot)


def Wp(d, p, f, lab, rot):
    return Waypoint(float(d), _f(p), float(f), lab, tilt._TiltedYaw(0.0, rot))


def demo():
    lot = lots.current()
    start = np.asarray(lot.pos("screen"), float)
    y0 = float(start[1])
    up = np.array([0.0, 0.0, 1.0])
    hover = SEAT + np.array([-0.120, 0.0, 0.020])  # upright, in front of the opening, bottom over the stool
    pre = SEAT + np.array([-(DEPTH + T + 0.006), 0.0, LIFT])  # just short of the jambs and the head
    front = SEAT + np.array([-0.010, 0.0, LIFT])
    inside = SEAT + np.array([PUSH, 0.0, LIFT])
    down = SEAT + np.array([PUSH, 0.0, -0.001])
    plans = {}
    for arm, side in (("left", 1), ("right", -1)):
        g0, r0 = grip_pose(start, LEAN, side, dx=-0.030)
        g1, _ = grip_pose(start, LEAN, side, dy=0.002)
        g1b, _ = grip_pose(start + np.array([-0.010, 0.0, 0.035]), LEAN, side)
        lifted = start + np.array([-0.030, 0.0, 0.070])
        g2, r2 = grip_pose(lifted, 0.0, side)
        g3, _ = grip_pose(np.array([hover[0], y0, hover[2]]), 0.0, side)
        g4, _ = grip_pose(hover, 0.0, side)
        g4b, _ = grip_pose(pre, 0.0, side)
        g5, _ = grip_pose(front, 0.0, side)
        g6, _ = grip_pose(inside, 0.0, side)
        g7, _ = grip_pose(down, 0.0, side)
        g8, _ = grip_pose(SEAT, 0.0, side, dx=-0.040)
        plan = [
            Wp(1.5, g0 + 0.06 * up, F_OPEN, "above the latch, in front of it", r0),
            Wp(1.2, g0, F_OPEN, "down beside the tab and lug", r0),
            Wp(1.0, g1, F_OPEN, "in around them", r0),
            Wp(0.8, g1, F_SQUEEZE, "squeeze the tab to the lug", r0),
            Wp(0.4, g1, F_SQUEEZE, "settle", r0),
            Wp(1.5, g1b, F_SQUEEZE, "lift it off the bench, still leaning", r0),
            Wp(2.0, g2, F_SQUEEZE, "stand it upright in the air", r2),
            Wp(2.0, g3, F_SQUEEZE, "back from the stool, over it", r2),
            Wp(1.5, g4, F_SQUEEZE, "centred on the opening", r2),
            Wp(1.0, g4, F_SQUEEZE, "settle", r2),
            Wp(1.5, g4b, F_SQUEEZE, "down to just over the sill, short of the jambs", r2),
            Wp(2.0, g5, F_SQUEEZE, "into the opening between the jambs", r2),
            Wp(1.2, g6, F_SQUEEZE, "back against the stop", r2),
            Wp(1.0, g7, F_SQUEEZE, "down onto the sill", r2),
            Wp(0.6, g7, F_SQUEEZE, "settle", r2),
            Wp(0.8, g7, F_LET, "let the tab go", r2),
            Wp(0.5, g7, F_LET, "settle", r2),
            Wp(1.2, g8, F_LET, "draw back off the latch", r2),
            Wp(1.2, g8 + 0.10 * up + np.array([-0.06, 0.0, 0.0]), F_LET, "up and away", r2),
        ]
        rest, ryaw = _rest_pose(arm)
        plan += [Waypoint(2.0, rest, F_LET, "back to the rest pose", ryaw),
                 Waypoint(4.0, rest, F_LET, "wait", ryaw)]
        plan[:0] = [Waypoint(2.0, rest, FINGER_OPEN, "to the rest pose", ryaw),
                    Waypoint(0.5, rest, FINGER_OPEN, "settle at the rest pose", ryaw)]
        plans[arm] = plan
    return plans
