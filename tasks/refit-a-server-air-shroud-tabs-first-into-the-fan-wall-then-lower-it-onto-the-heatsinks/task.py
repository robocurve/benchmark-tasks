"""Refit a 2U rack server's one-piece ABS air shroud into its motherboard bay.

The 418.0 x 160.0 x 78.0 mm shroud starts right-way-up on the closed front staging deck of an open-top EIA-310-D 2U
server bay (430.0 x 88.1 x 400.0 mm), its two 10.0 mm front tabs pointing forward (+x) toward the slotted fan bulkhead.
The open motherboard bay between the 52.0 mm mid-bay divider (x = 30.75 cm) and the 83.0 mm fan bulkhead (x = 47.30 cm)
is 165.5 mm long, whereas the shroud plus its front tabs is 170.0 mm long: lowered flat, the front tabs strike the
83.0 mm fan-wall lip or the rear floor strikes the 52.0 mm divider. Both hands pinch the recessed 26.0 mm blue
touchpoint bridges inside the shroud's 78.0 mm roof envelope, tilt the shroud nose-down so both front tabs slide into
the two 22.0 x 7.5 mm slots beneath the fan-wall lip, and hinge the rear down between the two 78.0 mm Dynatron S2
LGA4677 heatsinks (2.5 mm clearance per side) and four JEDEC MO-329 DDR5 RDIMM banks until all four corners seat flat.

Pure 240 Hz PhysX rigid-body contact (contact_offset = 0.6 mm); the task welds, pushes and latches nothing.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import mechkit as mk  # noqa: E402
import parts as P  # noqa: E402
import scene  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 3.0  # both arms finish returning to READY so the final frame shows the seated shroud clear
INSTRUCTION = (
    "Seat the 2U server's air shroud flat in the motherboard bay with both front tabs engaged in the fan-wall slots, "
    "without touching the processor heatsinks or memory modules with the grippers, and let go."
)
RUN_LIMIT_STEPS = 1000  # 50 s at 20 Hz
TOP = TABLE_TOP[SCENE]
FLOOR_Z = TOP + P.WALL_T
SIDES = ("left", "right")
HAND_LINKS = ("gripper", "tip_left", "tip_right")

scene.build(TOP)
OBJECTS = list(mk.MECH.bodies.values())
STATE: dict = {}

_LOCAL_HULLS: dict = {}
_LINK_IDS: dict = {}
_FOUL_BOXES = P.foul_boxes_world(TOP)


def _make_foul_samples() -> dict[str, np.ndarray]:
    """Dense surface/edge points on each heatsink and DDR5 RDIMM bank so flat pad faces bridging an edge are caught."""
    out: dict[str, np.ndarray] = {}
    for name, c, sz in _FOUL_BOXES:
        hx, hy, hz = sz / 2.0
        xs = np.linspace(c[0] - hx, c[0] + hx, max(3, int(math.ceil(sz[0] / 0.008)) + 1))
        ys = np.linspace(c[1] - hy, c[1] + hy, max(3, int(math.ceil(sz[1] / 0.008)) + 1))
        zs = np.linspace(c[2] - hz, c[2] + hz, max(2, int(math.ceil(sz[2] / 0.012)) + 1))
        top_grid = np.array([[x, y, c[2] + hz] for x in xs for y in ys], dtype=float)
        side_grid = np.array([[x, c[1] + sy * hy, z] for x in xs for sy in (-1.0, 1.0) for z in zs], dtype=float)
        end_grid = np.array([[c[0] + sx * hx, y, z] for sx in (-1.0, 1.0) for y in ys for z in zs], dtype=float)
        out[name] = np.vstack([top_grid, side_grid, end_grid])
    return out


_FOUL_PTS = _make_foul_samples()


def begin(seed: int | None = None) -> dict:
    STATE.clear()
    STATE.update(
        seed=seed,
        zero=None,
        hold=0,
        hold_from=None,
        hands=None,
        hand_hulls=None,
        first_half=None,
        first_full=None,
    )
    return STATE


@mk.on_reset
def start(seed=None) -> None:
    begin(seed)


# --- hand collision hulls in the world ----------------------------------------------------------------------------------
def read_hands() -> None:
    """Populate STATE['hands'] (side -> vertices) and STATE['hand_hulls'] (side -> list of (normals, offsets))."""
    from yam_arena import analysis

    if not _LOCAL_HULLS:
        for link in HAND_LINKS:
            _LOCAL_HULLS[link] = [(h.vertices, h.planes) for h in analysis.link_hulls().get(link, [])]
    base = mk.MECH.environment.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(float)
    hands: dict[str, np.ndarray] = {}
    hulls: dict[str, list[tuple[np.ndarray, np.ndarray]]] = {}
    for side in SIDES:
        robot = base.scene[f"{side}_robot"]
        pts_list = []
        hull_list = []
        for link, local_items in _LOCAL_HULLS.items():
            key = (id(base), side, link)
            if key not in _LINK_IDS:
                found = robot.find_bodies(link)[0]
                if not found:
                    raise RuntimeError(f"the {side} arm has no body {link!r}")
                _LINK_IDS[key] = found[0]
            pose7 = robot.data.body_link_pose_w.torch[0, _LINK_IDS[key]].detach().cpu().numpy().astype(float)
            pos = pose7[:3] - origin
            rot = mk.quat_to_matrix(pose7[3:7])
            for v_loc, pl_loc in local_items:
                pts_list.append(v_loc @ rot.T + pos)
                n_w = pl_loc[:, :3] @ rot.T
                hull_list.append((n_w, pl_loc[:, 3] + n_w @ pos))
        hands[side] = np.vstack(pts_list)
        hulls[side] = hull_list
    STATE["hands"] = hands
    STATE["hand_hulls"] = hulls


def _points_to_box(points: np.ndarray, centre: np.ndarray, rot: np.ndarray, size: np.ndarray) -> float:
    local = (points - centre) @ rot
    q = np.abs(local) - size / 2.0
    outside = np.linalg.norm(np.maximum(q, 0.0), axis=1)
    inside = np.minimum(np.max(q, axis=1), 0.0)
    return float((outside + inside).min())


def foul_touched() -> str | None:
    """Which heatsink or DDR5 RDIMM bank a gripper touches now, or None."""
    hands = STATE.get("hands")
    if not hands:
        return None
    all_pts = np.vstack([hands[s] for s in SIDES if s in hands])
    hand_hulls = STATE.get("hand_hulls") or {}
    eye = np.eye(3)
    for name, c, sz in _FOUL_BOXES:
        if _points_to_box(all_pts, c, eye, sz) <= P.FOUL_GAP:
            return f"a gripper touched {name.replace('_', ' ')}"
        surf = _FOUL_PTS[name]
        for s in SIDES:
            for n_w, d_w in hand_hulls.get(s, ()):
                if float((surf @ n_w.T - d_w).max(axis=1).min()) <= P.FOUL_GAP:
                    return f"a gripper touched {name.replace('_', ' ')}"
    return None


def hands_clear() -> float:
    """Minimum distance (m) from any hull vertex of either hand to the shroud or bay internals."""
    hands = STATE.get("hands")
    if not hands:
        return 0.0
    all_pts = np.vstack([hands[s] for s in SIDES if s in hands])
    p_sh, r_sh = mk.pose("shroud")
    best = math.inf
    for shape in mk.MECH.bodies["shroud"].shapes:
        if not shape.solid:
            continue
        sc = p_sh + r_sh @ np.asarray(shape.pos, dtype=float)
        sr = r_sh @ mk.quat_to_matrix(shape.quat)
        best = min(best, _points_to_box(all_pts, sc, sr, np.asarray(shape.dims, dtype=float)))
    eye = np.eye(3)
    for _, c, sz in _FOUL_BOXES:
        best = min(best, _points_to_box(all_pts, c, eye, sz))
    return float(max(0.0, best))


# --- shroud geometry checks ---------------------------------------------------------------------------------------------
def shroud_pose() -> tuple[np.ndarray, np.ndarray]:
    return mk.pose("shroud")


def pitch_deg() -> float:
    """Nose-down pitch (deg) of the shroud (+x tilted down is positive)."""
    r = shroud_pose()[1]
    return float(math.degrees(math.asin(max(-1.0, min(1.0, -float(r[2, 0]))))))


def tilt_deg() -> float:
    """Total tilt from level (deg) of the shroud."""
    r = shroud_pose()[1]
    return float(math.degrees(math.acos(max(-1.0, min(1.0, float(r[2, 2]))))))


def yaw_deg() -> float:
    r = shroud_pose()[1]
    return float(math.degrees(math.atan2(float(r[1, 0]), float(r[0, 0]))))


def tab_engagements() -> dict[str, float]:
    """Insertion depth (m) of each front tab past the x = 47.30 cm bulkhead face inside its 22.0 x 7.5 mm slot."""
    p, r = shroud_pose()
    if float(r[2, 2]) < 0.7:
        return {"left": 0.0, "right": 0.0}
    out: dict[str, float] = {}
    for side, s in (("left", 1.0), ("right", -1.0)):
        root = p + r @ np.array([P.SHROUD_L / 2.0, s * P.SLOT_Y, P.TAB_Z_MID])
        tip = p + r @ np.array([P.SHROUD_L / 2.0 + P.TAB_L, s * P.SLOT_Y, P.TAB_Z_MID])
        dx = float(tip[0] - root[0])
        if tip[0] <= P.X_BULKHEAD or dx <= 1e-6:
            out[side] = 0.0
            continue
        # Where the tab centreline crosses the bulkhead front plane (or root if root is already past X_BULKHEAD)
        alpha = float(np.clip((P.X_BULKHEAD - root[0]) / dx, 0.0, 1.0))
        entry = root + alpha * (tip - root)
        y_ok = max(abs(float(tip[1]) - s * P.SLOT_Y), abs(float(entry[1]) - s * P.SLOT_Y)) <= P.SLOT_Y_HALF_TOL
        h_tip = float(tip[2] - FLOOR_Z)
        h_entry = float(entry[2] - FLOOR_Z)
        # Tab thickness is 3.0 mm: top & bottom faces must lie beneath the 78.0 mm slot roof and above the 70.5 mm floor
        half_t = P.TAB_T / 2.0 - 0.0008
        z_ok = (P.SLOT_Z_LO <= h_tip - half_t and h_tip + half_t <= P.SLOT_Z_HI
                and P.SLOT_Z_LO <= h_entry - half_t and h_entry + half_t <= P.SLOT_Z_HI)
        out[side] = float(tip[0] - P.X_BULKHEAD) if (y_ok and z_ok) else 0.0
    return out


def both_tabs_engaged() -> bool:
    eng = tab_engagements()
    return eng["left"] >= P.TAB_IN_MIN and eng["right"] >= P.TAB_IN_MIN


def corner_heights() -> dict[str, float]:
    """Height above the bay floor (m) of all four outer roof corners of the shroud."""
    p, r = shroud_pose()
    out: dict[str, float] = {}
    for x_tag, sx in (("front", 1.0), ("rear", -1.0)):
        for y_tag, sy in (("left", 1.0), ("right", -1.0)):
            pt = p + r @ np.array([sx * P.SHROUD_L / 2.0, sy * P.SHROUD_W / 2.0, P.SHROUD_H])
            out[f"{x_tag}_{y_tag}"] = float(pt[2] - FLOOR_Z)
    return out


def corners_seated() -> bool:
    """All four outer roof corners lie within 78.0 +/- 1.5 mm above the bay floor over the motherboard bay."""
    p, r = shroud_pose()
    if float(r[2, 2]) < 0.95 or not (P.X_DIV_REAR < float(p[0]) < P.X_BULKHEAD):
        return False
    return all(abs(h - P.ROOF_TARGET) <= P.ROOF_TOL for h in corner_heights().values())


def dropped_outside() -> bool:
    """Whether the shroud has fallen outside the 430.0 x 400.0 mm 2U bay footprint."""
    p, r = shroud_pose()
    if abs(float(p[1])) > P.CHASSIS_W / 2.0 or float(p[0]) < P.X_FRONT - 0.040 or float(p[0]) > P.X_REAR + 0.040:
        return True
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            foot = p + r @ np.array([sx * 0.062, sy * P.SHROUD_W / 2.0, 0.0])
            if float(foot[2]) < TOP + 0.0015 and (abs(float(foot[1])) > P.CHASSIS_W / 2.0 or float(foot[0]) < P.X_FRONT or float(foot[0]) > P.X_REAR):
                return True
    return False


def zero_now() -> str | None:
    foul = foul_touched()
    if foul is not None:
        return foul
    if dropped_outside():
        return "the shroud fell outside the 2U server bay"
    return None


# --- scoring ------------------------------------------------------------------------------------------------------------
def full_now() -> bool:
    return both_tabs_engaged() and corners_seated() and hands_clear() >= P.HANDS_CLEAR


@mk.on_step
def step() -> None:
    if mk.MECH.environment is not None:
        read_hands()
    if STATE.get("zero") is None:
        STATE["zero"] = zero_now()
    t_now = mk.now() if mk.MECH.environment is not None else 0.0
    if STATE.get("zero") is None and both_tabs_engaged() and STATE.get("first_half") is None:
        STATE["first_half"] = t_now
    if STATE.get("zero") is None and full_now():
        if STATE.get("hold", 0) == 0:
            STATE["hold_from"] = shroud_pose()[0].copy()
        STATE["hold"] = STATE.get("hold", 0) + 1
        if float(np.linalg.norm(shroud_pose()[0] - STATE["hold_from"])) > P.HOLD_DRIFT:
            STATE["hold_from"] = shroud_pose()[0].copy()
            STATE["hold"] = 1
        if STATE["hold"] >= P.HOLD_STEPS and STATE.get("first_full") is None:
            STATE["first_full"] = t_now
    else:
        STATE["hold"], STATE["hold_from"] = 0, None


def level() -> float:
    if STATE.get("zero") is not None or zero_now() is not None:
        return 0.0
    if STATE.get("hold", 0) >= P.HOLD_STEPS and full_now():
        return 1.0
    if both_tabs_engaged():
        return 0.5
    return 0.0


def score(objects) -> float:
    """0 for good if any part of either gripper ever touches a processor heatsink or DDR5 memory module, or if the
    shroud falls outside the 430 x 400 mm 2U bay. Otherwise read at the end: 1.0 when both front tabs project at least
    5.0 mm into their 22.0 x 7.5 mm fan-wall slots beneath the 78.0 mm slot roof, all four outer roof corners stand at
    78.0 +/- 1.5 mm above the bay floor, and both hands are at least 30 mm clear, held for 1.0 s (20 steps) with the
    shroud moving 2.0 mm at most; 0.5 when both front tabs project at least 5.0 mm into their slots beneath the lip;
    0 otherwise."""
    return level()


def success(objects) -> bool:
    return score(objects) == 1.0


def report() -> dict:
    out = {
        "zero": STATE.get("zero"),
        "hold steps": STATE.get("hold", 0),
        "first 0.5 (s)": STATE.get("first_half"),
        "first 1.0 (s)": STATE.get("first_full"),
    }
    if mk.MECH.poses:
        eng = tab_engagements()
        ch = corner_heights()
        out.update({
            "shroud pos (m)": [round(float(v), 4) for v in shroud_pose()[0]],
            "pitch (deg)": round(pitch_deg(), 2),
            "tilt (deg)": round(tilt_deg(), 2),
            "yaw (deg)": round(yaw_deg(), 2),
            "tab in slot (mm)": {k: round(1000.0 * v, 2) for k, v in eng.items()},
            "corner roof heights (mm)": {k: round(1000.0 * v, 2) for k, v in ch.items()},
            "chassis contact (N)": round(mk.contact_force("shroud", "chassis"), 2),
            "hands clear (mm)": round(1000.0 * hands_clear(), 1),
            "score": score(None),
        })
    return out
