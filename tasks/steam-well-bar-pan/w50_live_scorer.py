"""Live 21-Frame Stillness + 424-Sphere Full-Hand Release + Actual-Bar-Frame Scorer for Steam-Well (antigravity helper).

Combines:
1. `claude-cake` / Codex `0543` actual-bar-frame `bar_seated` (both end-land patches in `bar_R`) & `pan_ok` (inner flange
   strip at `+-80%` half-length transformed into `bar_R`, outer flange on W50 rim; `16 deg` yawed bar -> `0.0`; duplicate
   quadrant -> `0.0`).
2. Certified 424-sphere full-hand (`gripper` + `D405` + `tip_left` + `tip_right`) `>= 80.0 mm` release gate with Isaac Lab 3.0
   `XYZW` quaternion order and `.torch` Warp proxy unwrapping.
3. Strict 21-frame (`1.0 s` at `20 Hz`) `< 5.0 mm` linear displacement and `< 2.0 deg` orientation drift stillness gate across
   the `75020` bar and all placed quarter pans.
"""

from __future__ import annotations

import math
from collections import deque
import numpy as np

from hand_cover_data import (
    GRIPPER_D405_COVER_LOCAL,
    TIP_LEFT_COVER_NATIVE,
    TIP_RIGHT_COVER_NATIVE,
)
import w50_four_pan_waypoints as W50_F

HOLD_FRAMES = 21           # 20 intervals = 1.0 s at 20 Hz
HAND_RELEASE_M = 0.080     # >= 80.0 mm full-hand clearance to bar and all pans
STILL_POS_M = 0.0050       # < 5.0 mm max 3D excursion over 1.0 s window
STILL_ANG_DEG = 2.0        # < 2.0 deg max rotation excursion over 1.0 s window


def _unwrap_array_row0(raw) -> np.ndarray:
    if hasattr(raw, "torch"):
        raw = raw.torch() if callable(raw.torch) else raw.torch
    if hasattr(raw, "detach"):
        raw = raw.detach().cpu().numpy()
    arr = np.asarray(raw, dtype=np.float64)
    return arr[0] if arr.ndim == 3 else arr


def _quat_xyzw_to_rot(q_xyzw: np.ndarray) -> np.ndarray:
    x, y, z, w = [float(v) for v in q_xyzw]
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def extract_hand_424_world(env) -> np.ndarray:
    """Return `(848, 3)` certified hand + D405 + fingertip points in env-local world coordinates (Isaac Lab 3.0 XYZW)."""
    b = env.unwrapped
    orig = _unwrap_array_row0(b.scene.env_origins)
    pts_all = []
    for key in ("left_robot", "right_robot"):
        rob = b.scene[key]
        names = list(rob.body_names)
        poses = _unwrap_array_row0(rob.data.body_link_pose_w)
        for lname, cloud in (
            ("gripper", GRIPPER_D405_COVER_LOCAL),
            ("tip_left", TIP_LEFT_COVER_NATIVE),
            ("tip_right", TIP_RIGHT_COVER_NATIVE),
        ):
            idx = names.index(lname)
            p_w = poses[idx, :3] - orig
            R_w = _quat_xyzw_to_rot(poses[idx, 3:7])
            pts_all.append((R_w @ cloud.T).T + p_w)
    return np.vstack(pts_all)


def min_hand_to_parts_m(hand_pts_w: np.ndarray, bar_pose: tuple[np.ndarray, np.ndarray], pan_poses: list[tuple[np.ndarray, np.ndarray]]) -> float:
    """Exact Euclidean distance from 848 hand points to the 75020 bar OBB and all pan 9-box OBBs."""
    bp, Rb = bar_pose
    obbs = [(bp + Rb @ np.array([0.0, 0.0, W50_F.BAR_H / 2.0]), Rb, np.array([W50_F.BAR_W / 2.0, W50_F.BAR_L / 2.0, W50_F.BAR_H / 2.0]))]
    bw, bl = W50_F.Q_W - 2.0 * W50_F.Q_FL, W50_F.Q_L - 2.0 * W50_F.Q_FL
    raw_pan = [
        ((0.0, 0.0, W50_F.SHEET / 2.0), (bw, bl, W50_F.SHEET)),
        ((-bw / 2.0 + W50_F.SHEET / 2.0, 0.0, W50_F.Q_D / 2.0), (W50_F.SHEET, bl, W50_F.Q_D)),
        ((+bw / 2.0 - W50_F.SHEET / 2.0, 0.0, W50_F.Q_D / 2.0), (W50_F.SHEET, bl, W50_F.Q_D)),
        ((0.0, -bl / 2.0 + W50_F.SHEET / 2.0, W50_F.Q_D / 2.0), (bw, W50_F.SHEET, W50_F.Q_D)),
        ((0.0, +bl / 2.0 - W50_F.SHEET / 2.0, W50_F.Q_D / 2.0), (bw, W50_F.SHEET, W50_F.Q_D)),
        ((-W50_F.Q_W / 2.0 + W50_F.Q_FL / 2.0, 0.0, W50_F.Q_D - W50_F.SHEET / 2.0), (W50_F.Q_FL, W50_F.Q_L, W50_F.SHEET)),
        ((+W50_F.Q_W / 2.0 - W50_F.Q_FL / 2.0, 0.0, W50_F.Q_D - W50_F.SHEET / 2.0), (W50_F.Q_FL, W50_F.Q_L, W50_F.SHEET)),
        ((0.0, -W50_F.Q_L / 2.0 + W50_F.Q_FL / 2.0, W50_F.Q_D - W50_F.SHEET / 2.0), (bw, W50_F.Q_FL, W50_F.SHEET)),
        ((0.0, +W50_F.Q_L / 2.0 - W50_F.Q_FL / 2.0, W50_F.Q_D - W50_F.SHEET / 2.0), (bw, W50_F.Q_FL, W50_F.SHEET)),
    ]
    for pp, Rp in pan_poses:
        for cl, sl in raw_pan:
            obbs.append((pp + Rp @ np.asarray(cl, float), Rp, 0.5 * np.asarray(sl, float)))
    best = float("inf")
    for c, R, h in obbs:
        ex = np.maximum(np.abs((hand_pts_w - c) @ R) - h, 0.0)
        d = float(np.sqrt(np.min(np.sum(ex * ex, axis=1))))
        if d < best:
            best = d
    return best


class SteamWellLiveTracker:
    """Stateful 21-frame live scorer tracker (`reset()` in `@on_reset`, `step(...)` in `@on_step`)."""

    def __init__(self, wc_xy=(0.4800, 0.0000), rim_z=0.8261):
        self.wc_xy = tuple(wc_xy)
        self.rim_z = float(rim_z)
        self.buf: deque[dict] = deque(maxlen=HOLD_FRAMES)
        self.latest_score: float = 0.0

    def reset(self):
        self.buf.clear()
        self.latest_score = 0.0

    def record(self, bar_pose: tuple[np.ndarray, np.ndarray], pan_poses: list[tuple[np.ndarray, np.ndarray]], hand_pts_w: np.ndarray) -> dict:
        geom_score = W50_F.score_bar_frame(pan_poses, bar_pose, self.wc_xy, self.rim_z, any_duplicate_zero=True)
        hand_clear = min_hand_to_parts_m(hand_pts_w, bar_pose, pan_poses)
        snap = {
            "geom_score": geom_score,
            "hand_clear_m": hand_clear,
            "bar_p": np.asarray(bar_pose[0], float).copy(),
            "bar_R": np.asarray(bar_pose[1], float).copy(),
            "pans": [(np.asarray(p, float).copy(), np.asarray(R, float).copy()) for p, R in pan_poses],
        }
        self.buf.append(snap)
        if len(self.buf) < HOLD_FRAMES:
            self.latest_score = 0.0
            return {"score": 0.0, "reason": "hold_window_incomplete", "geom_score": geom_score, "hand_clear_mm": round(hand_clear * 1e3, 1)}

        min_geom = min(f["geom_score"] for f in self.buf)
        min_hand = min(f["hand_clear_m"] for f in self.buf)
        if min_geom <= 0.0 or min_hand < HAND_RELEASE_M:
            self.latest_score = 0.0
            return {"score": 0.0, "reason": "unseated_or_hand_within_80mm", "geom_score": geom_score, "hand_clear_mm": round(min_hand * 1e3, 1)}

        f0 = self.buf[0]
        max_dp = max(float(np.linalg.norm(f["bar_p"] - f0["bar_p"])) for f in self.buf)
        max_da = max(
            math.degrees(math.acos(float(np.clip((np.trace(f["bar_R"].T @ f0["bar_R"]) - 1.0) / 2.0, -1.0, 1.0))))
            for f in self.buf
        )
        for idx in range(len(pan_poses)):
            p0, R0 = f0["pans"][idx]
            # Only enforce stillness on pans inside the well footprint (or all pans)
            for f in self.buf:
                pi, Ri = f["pans"][idx]
                max_dp = max(max_dp, float(np.linalg.norm(pi - p0)))
                max_da = max(max_da, math.degrees(math.acos(float(np.clip((np.trace(Ri.T @ R0) - 1.0) / 2.0, -1.0, 1.0)))))

        if max_dp >= STILL_POS_M or max_da >= STILL_ANG_DEG:
            self.latest_score = 0.0
            return {"score": 0.0, "reason": "still_moving", "max_dp_mm": round(max_dp * 1e3, 2), "max_da_deg": round(max_da, 2)}

        self.latest_score = float(min_geom)
        return {"score": self.latest_score, "reason": "ok", "hand_clear_mm": round(min_hand * 1e3, 1), "max_dp_mm": round(max_dp * 1e3, 2)}
