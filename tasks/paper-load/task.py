"""A Canon PIXMA TS3720 all-in-one printer stands on an IKEA LACK side table in a floor bay, its front toward the robot.
Its rear tray is empty (it ran out of paper mid-job) and its paper support is raised. Five fresh sheets of Letter copy
paper lie on its shut document cover, portrait, their near edge over the cover's front edge. The instruction is "The
printer is out of paper. Load the fresh sheets so it can print again."

Loaded as Canon's manual says: the sheets standing portrait in the rear tray against its right side, the paper guide
slid to their left edge, and OK pressed to clear the out-of-paper code.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
         --scorer task_score --max-steps 2000 ...
Demo:    python demo.py task.py demo_out
"""

from __future__ import annotations

import collections
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from scenekit import UsdBody, before_reset, floor_bay, on_reset, on_step, physics  # noqa: E402

import geometry as g  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0

INSTRUCTION = "The printer is out of paper. Load the fresh sheets so it can print again."

physics(g.PHYSICS_HZ)
floor_bay()

TOP = g.TOP
SHEET_JOINTS = tuple(f"h{i}" for i in range(g.N_STRIPS - 1))
ROOT_STRIP = g.N_STRIPS // 2
KEYS = ("btn_black", "btn_color", "btn_ok")
STAND = UsdBody("side_table", usd="assets/stand.usda", pos=(0.0, 0.0, g.FZ), size=(g.LACK[0], g.LACK[1], g.STAND_H),
                friction=(0.5, 0.45))
PRINTER = UsdBody("printer", usd="assets/printer.usda", pos=(0.0, 0.0, TOP),
                  size=(g.PR_X[1] - g.PR_X[0], g.PR_Y[1] - g.PR_Y[0], g.PR_H), articulation=True,
                  joint_pos=(("cover", 0.0), ("btn_black", 0.0), ("btn_color", 0.0), ("btn_ok", 0.0),
                             ("support", g.SUPPORT_START), ("guide", 0.0)),
                  drives=(("cover", 0.0, g.COVER_DAMP), ("btn_black", g.BTN_K, 2.0), ("btn_color", g.BTN_K, 2.0),
                          ("btn_ok", g.BTN_K, 2.0), ("support", 0.0, g.SUPPORT_DAMP), ("guide", 0.0, g.GUIDE_DAMP)),
                  friction=(g.PRINTER_MU, g.PRINTER_MU), friction_min=True, solver_iters=16)
SHEET = UsdBody("paper", usd="assets/sheet.usda", pos=tuple(g.start_strip_pos(ROOT_STRIP)),
                size=(g.SHEET_W, g.SHEET_L, g.COLL_T), articulation=True,
                joint_pos=tuple((j, 0.0) for j in SHEET_JOINTS),
                drives=((r"h\d", g.HINGE_K, g.HINGE_C),), friction=(g.SHEET_MU, g.SHEET_MU), solver_iters=32)
OBJECTS = [STAND, PRINTER, SHEET]
HAND_LINKS = ("gripper", "tip_left", "tip_right")

_STATE: dict = {}
_DEBUG = bool(os.environ.get("PC_DEBUG"))


def _np(x) -> np.ndarray:
    return getattr(x, "torch", x).detach().cpu().numpy().astype(np.float64)


def _rotation(xyzw) -> np.ndarray:
    x, y, z, w = (float(v) for v in xyzw)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def sheet_point(strips, a: float, b: float, z: float = 0.0) -> np.ndarray:
    """World position of the stack point (a, b) (m), ``z`` off its middle plane."""
    i = int(min(g.N_STRIPS - 1, max(0, math.floor(a / g.STRIP_W))))
    c, R = strips[i]
    return np.asarray(c, float) + R @ np.array([a - g.strip_centre_a(i), b, z])


def stack_corners(strips) -> np.ndarray:
    """Every strip's eight collider corners (world), (9 x 8, 3)."""
    out = []
    for c, R in strips:
        for sx in (-g.STRIP_W / 2, g.STRIP_W / 2):
            for sy in (-g.SHEET_L / 2, g.SHEET_L / 2):
                for sz in (-g.COLL_T / 2, g.COLL_T / 2):
                    out.append(np.asarray(c) + R @ np.array([sx, sy, sz]))
    return np.array(out)


def load_report(strips, guide_q: float, support_deg: float) -> dict:
    """Where the stack is against the rear tray, read as in a real trial:
    - in_tray: its lowest edge inside the slot, below the housing's top, and the stack's lowest RISE_SPAN rising at
      least 50 degrees (above the support its top may bend back over it);
    - seated: its lowest point within SEATED_TOL of the slot's floor;
    - right_gap: its right edge's gap to the tray's right side (the 0.5 mm feeler leaf);
    - guide_gap: the guide's face to its left edge;
    - support_open: the paper support raised past upright."""
    P = stack_corners(strips)
    low = P[np.argmin(P[:, 2])]
    floor_z = TOP + g.SLOT_FLOOR_H
    edge = lambda a: np.array([sheet_point(strips, a, b) for b in (-g.SHEET_L / 2, 0.0, g.SHEET_L / 2)])  # noqa: E731
    bottom, top = edge(g.SHEET_W), edge(0.0)
    if bottom[:, 2].mean() < top[:, 2].mean():
        lo_edge, hi_edge = bottom, edge(g.SHEET_W - g.RISE_SPAN)
    else:
        lo_edge, hi_edge = top, edge(g.RISE_SPAN)
    rise = math.atan2(float(hi_edge[:, 2].mean() - lo_edge[:, 2].mean()),
                      float(np.linalg.norm(hi_edge[:, :2].mean(0) - lo_edge[:, :2].mean(0))))
    in_slot = bool(np.all((lo_edge[:, 0] > g.SLOT_X[0] - 0.004) & (lo_edge[:, 0] < g.SLOT_X[1] + 0.004))
                   and lo_edge[:, 2].max() < TOP + g.PR_H)
    in_tray = bool(in_slot and rise > g.UPRIGHT_MIN)
    seated = bool(in_tray and low[2] - floor_z < g.SEATED_TOL)
    right_gap = float(P[:, 1].min() - g.RIGHT_Y)
    guide_face = g.GUIDE_OPEN_Y + guide_q
    guide_gap = float(guide_face - P[:, 1].max())
    return dict(in_tray=in_tray, seated=seated, rise_deg=round(math.degrees(rise), 1),
                low_mm=round((low[2] - floor_z) * 1000, 2), right_gap_mm=round(right_gap * 1000, 2),
                guide_gap_mm=round(guide_gap * 1000, 2), support_open=bool(support_deg > g.SUPPORT_OPEN_DEG))


def tier(rep: dict, ok_after: bool, folded: bool, released: bool) -> float:
    """The score from the end state (see score())."""
    if folded or not rep["in_tray"]:
        return 0.0
    if not rep["seated"] or rep["right_gap_mm"] > g.RIGHT_TOL * 1000:
        return 0.25
    if rep["guide_gap_mm"] > g.GUIDE_TOL * 1000:
        return 0.5
    if ok_after and released and rep["support_open"]:
        return 1.0
    return 0.75


# -- the grippers' touch: collision hulls of each gripper's body and fingers ------------------------------------------
def _hull_planes(points: np.ndarray):
    from scipy.spatial import ConvexHull

    h = ConvexHull(points)
    return h.equations[:, :3], -h.equations[:, 3]           # inside: n . x <= d


def _surface_samples(points: np.ndarray, step: float = 0.004) -> np.ndarray:
    from scipy.spatial import ConvexHull

    h = ConvexHull(points)
    out = [points[h.vertices]]
    for s in h.simplices:
        for i, j in ((0, 1), (1, 2), (2, 0)):
            a, b = points[s[i]], points[s[j]]
            k = max(2, int(np.linalg.norm(b - a) / step) + 1)
            out.append(a + np.linspace(0.0, 1.0, k)[:, None] * (b - a))
    return np.unique(np.round(np.concatenate(out), 6), axis=0)


def _box(lo, hi) -> np.ndarray:
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])


def printer_parts() -> dict:
    """The printer's touchable parts as box corners: 'body' in the printer's frame (on the table), the cover, keys,
    support and guide in their own frames."""
    X0, X1 = g.PR_X
    Y0, Y1 = g.PR_Y
    L = g.HINGE[0] - g.COVER_X[0]
    s = g.BTN / 2
    hw = (g.SLOT_X[1] - g.SLOT_X[0]) / 2 - 0.001
    key = [_box((-s, -s, -0.006), (s, s, 0.0))]
    return {"body": [_box((X0, Y0, 0.0), (g.LIP_X[1], Y1, g.LIP_FLOOR_H)), _box((g.LIP_X[1], Y0, 0.0), (X1, Y1, g.FRAME_H)),
                     _box((X0, g.COVER_Y[1], 0.0), (g.REAR_X[0], Y1, g.COVER_TOP_H)),
                     _box((g.REAR_X[0], Y0, 0.0), (g.SLOT_X[0], Y1, g.PR_H)),
                     _box((g.SLOT_X[0], Y0, 0.0), (g.SLOT_X[1], Y1, g.SLOT_FLOOR_H)),
                     _box((g.SLOT_X[1], Y0, 0.0), (X1, Y1, g.SUPPORT_HINGE[1] - 0.004)),
                     _box((g.TRAY_X[0], g.TRAY_Y[0], g.TRAY_H - 0.003), (X0, g.TRAY_Y[1], g.TRAY_H + 0.012))],
            "cover": [_box((-L, g.COVER_Y[0], -g.COVER_T), (0.0, g.COVER_Y[1], 0.0))],
            "btn_black": key, "btn_color": key, "btn_ok": key,
            "support": [_box((-g.SUPPORT_L, g.SUPPORT_Y[0], 0.0), (0.0, g.SUPPORT_Y[1], g.SUPPORT_T))],
            "guide": [_box((-hw, 0.0, g.GUIDE_H[0]), (hw, g.GUIDE_T, g.GUIDE_H[1])),
                      _box((-hw, 0.0, g.GUIDE_H[1] - g.GUIDE_GRIP_H), (hw, 0.014, g.GUIDE_H[1]))]}


def sheet_parts() -> list:
    w, L, t = g.STRIP_W / 2, g.SHEET_L / 2, g.COLL_T / 2
    return [_box((-w, -L, -t), (w, L, t))]


_LOCAL: dict = {}


def _local_parts():
    if not _LOCAL:
        for name, parts in printer_parts().items():
            _LOCAL[name] = [(_surface_samples(v), *_hull_planes(v)) for v in parts]
        _LOCAL["strip"] = [(_surface_samples(v, 0.006), *_hull_planes(v)) for v in sheet_parts()]
    return _LOCAL


def gap_to_hulls(hand_hulls, objects: list) -> float:
    """The smallest gap (m) from any of ``hand_hulls`` (yam_arena.analysis Hulls, world) to any of ``objects``, a list
    of (part name, pos, R) placing the parts of _local_parts() in the world. Both sides are checked: the parts' surface
    samples against each hull's planes and each hull's vertices against the parts' planes; outside both, the larger is
    a lower bound on the true gap, so the gap errs toward touching."""
    best = 1.0
    hv = [h.vertices for h in hand_hulls]
    lo_h = np.min([v.min(0) for v in hv], axis=0)
    hi_h = np.max([v.max(0) for v in hv], axis=0)
    for name, pos, R in objects:
        p = np.asarray(pos, float)
        for samples, n, d in _local_parts()[name]:
            S = p + samples @ R.T
            if np.any(S.min(0) > hi_h + 0.05) or np.any(S.max(0) < lo_h - 0.05):
                continue
            nw, dw = n @ R.T, d + (n @ R.T) @ p
            for h in hand_hulls:
                if np.any(S.min(0) > h.vertices.max(0) + 0.03) or np.any(S.max(0) < h.vertices.min(0) - 0.03):
                    continue
                pl = h.planes
                a = float((S @ pl[:, :3].T - pl[:, 3]).max(axis=1).min())
                b = float((h.vertices @ nw.T - dw).max(axis=1).min())
                best = min(best, max(a, b) if a > 0 and b > 0 else min(a, b))
    return best


def hand_hulls_from_poses(link_poses_by_side: dict) -> list:
    from yam_arena.analysis import world_hulls

    out = []
    for poses in link_poses_by_side.values():
        out.extend(h for n, h in world_hulls(poses) if n in HAND_LINKS)
    return out


def touch_objects(strips, printer_poses: dict) -> list:
    out = [("body", np.array([0.0, 0.0, TOP]), np.eye(3))]
    for n in ("cover", "support", "guide") + KEYS:
        out.append((n, *printer_poses[n]))
    out += [("strip", c, R) for c, R in strips]
    return out


# -- reading the simulator -----------------------------------------------------------------------------------------------
def read_scene(environment) -> dict:
    base = environment.unwrapped
    origin = _np(base.scene.env_origins[0])
    sh = base.scene["paper"]
    pose = _np(sh.data.body_link_pose_w)[0]
    names = list(sh.body_names)
    strips = []
    for i in range(g.N_STRIPS):
        k = names.index(f"strip_{i}")
        strips.append((pose[k, :3] - origin, _rotation(pose[k, 3:7])))
    q = _np(sh.data.joint_pos)[0]
    hinges = {n: float(q[i]) for i, n in enumerate(sh.joint_names)}
    pr = base.scene["printer"]
    qp = _np(pr.data.joint_pos)[0]
    joints = {n: float(qp[i]) for i, n in enumerate(pr.joint_names)}
    pp = _np(pr.data.body_link_pose_w)[0]
    pnames = list(pr.body_names)
    parts = {n: (pp[pnames.index(n), :3] - origin, _rotation(pp[pnames.index(n), 3:7]))
             for n in ("cover", "support", "guide") + KEYS}
    return dict(strips=strips, hinges=hinges, joints=joints, parts=parts)


# -- the monitor ---------------------------------------------------------------------------------------------------------
def _write_start(environment) -> None:
    from scenekit import set_joint_targets, write_joint_positions

    write_joint_positions(environment, "printer", {"cover": 0.0, "btn_black": 0.0, "btn_color": 0.0, "btn_ok": 0.0,
                                                   "support": g.SUPPORT_START, "guide": 0.0})
    set_joint_targets(environment, "printer", {k: 0.0 for k in KEYS})
    write_joint_positions(environment, "paper", {j: 0.0 for j in SHEET_JOINTS})


@before_reset
def _before(environment) -> None:
    _write_start(environment)


def fresh_state() -> dict:
    return dict(step=0, armed={k: True for k in KEYS}, presses=[], ok_after=False, folded=False, max_hinge_deg=0.0,
                rep={}, gap=1.0, touch_log=collections.deque(maxlen=round(g.FINAL_S * g.CONTROL_HZ)),
                released=False, tier=0.0, first_full=None, events=[], support_deg=0.0)


@on_reset
def _reset_state(environment=None) -> None:
    _STATE.clear()
    _STATE.update(fresh_state())


_reset_state()


def advance(S: dict, strips, guide_q: float, support_deg: float, depths: dict, hinge_deg: float, gap: float) -> None:
    """One control step of the monitor, from what was read: the strips' poses, the guide's travel (m, <= 0), the paper
    support's angle (degrees), how far each key is pushed in (m), the stack's largest hinge angle (degrees) and the
    grippers' smallest gap (m)."""
    S["step"] += 1
    step = S["step"]
    S["support_deg"] = round(support_deg, 1)
    S["max_hinge_deg"] = max(S["max_hinge_deg"], hinge_deg)
    S["folded"] |= hinge_deg > g.FOLDED_DEG
    rep = load_report(strips, guide_q, support_deg)
    S["rep"] = rep
    for k in KEYS:
        depth = depths.get(k, 0.0)
        if S["armed"][k] and depth > g.BTN_PRESSED:
            S["armed"][k] = False
            S["presses"].append((step, k))
            S["events"].append((step, f"press {k}"))
            if k == "btn_ok" and rep["in_tray"]:
                S["ok_after"] = True                      # OK pressed with paper in the tray: code 1000 cleared
        elif not S["armed"][k] and depth < g.BTN_RELEASED:
            S["armed"][k] = True
    S["gap"] = gap
    S["touch_log"].append(gap <= g.TOUCH)
    S["released"] = bool(len(S["touch_log"]) == S["touch_log"].maxlen and not any(S["touch_log"]))
    S["tier"] = tier(rep, S["ok_after"], S["folded"], S["released"])
    if S["tier"] >= 1.0 and S["first_full"] is None:
        S["first_full"] = step
        S["events"].append((step, "1.0"))


@on_step
def _monitor(environment) -> None:
    from scenekit import robot_link_poses

    S = _STATE
    sc = read_scene(environment)
    strips = sc["strips"]
    hinge = math.degrees(max(abs(v) for v in sc["hinges"].values()))
    hulls = hand_hulls_from_poses(robot_link_poses(environment))
    gap = gap_to_hulls(hulls, touch_objects(strips, sc["parts"]))
    depths = {k: -sc["joints"][k] for k in KEYS}
    advance(S, strips, sc["joints"]["guide"], math.degrees(sc["joints"]["support"]), depths, hinge, gap)
    if _DEBUG and S["step"] % 20 == 0:
        print(f"[pl] step {S['step']} support {S['support_deg']} hinge {hinge:.1f} rep {S['rep']} ok_after "
              f"{int(S['ok_after'])} gap {gap * 1000:.1f} released {int(S['released'])} tier {S['tier']}", flush=True)


def success(objects) -> bool:
    """The 1.0 state (see score()); the trial ends 1 s after it is reached."""
    return bool(_STATE.get("tier", 0.0) >= 1.0)


def score(objects) -> float:
    """From the end state, read every control step (20 Hz) from the stack's strips, the printer's joints and the
    grippers' collision hulls (see load_report()):
    0 if the stack is not standing in the rear tray (doing nothing included) or a hinge ever bent past 150 degrees
    (folded over);
    0.25: in the tray, but not down on its floor (within 3 mm) or not against its right side (within 0.5 mm);
    0.5: down and against the right side, but the guide more than 0.5 mm from the stack's left edge;
    0.75: down, right and the guide snug, but OK not pressed with the paper in, the paper support not raised, or a
    gripper touching the printer or the paper in the final 1 s (in a real trial, also OK pressed but code 1000 still
    shown, or the test page jamming; neither is simulated);
    1.0: all of it."""
    return float(_STATE.get("tier", 0.0))
