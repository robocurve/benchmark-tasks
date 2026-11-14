"""hhkit: shared helpers for this task's task.py. Pure Python; Isaac Sim pieces are reached only through the env
handle the embodiment passes in during a run.

- ``Trial``: per-trial memory for latched safety gates. The embodiment calls success(), distance() and score() once
  each per control step with the same states and has no reset hook for task files, so importing a task wraps
  ``YamArenaEmbodiment._task_step`` (to count control steps and keep the env handle) and ``reset`` (to clear the
  memory). The wrappers are installed once on the class and call every registered Trial. yam_arena is not modified.
- ``grippers`` / ``robot_points``: the grippers' centres and fingertips, and the origin of every robot link, read
  from the simulator (offline tests set them by hand).
- ``teleport``: move a loose body to a pose and zero its velocity, as the embodiment's own seeded reset does; used
  for a person's hand that slides along the bench on a schedule.
- Geometry: box corners and footprints, convex footprint-to-rectangle gaps, point-to-box distances, tilt.
"""

from __future__ import annotations

import math

import numpy as np

CONTROL_HZ = 20.0
TCP_PAST_GRIPPER = 0.134  # yam_arena.constants.TCP_OFFSET: the TCP sits this far along the gripper's approach axis


class Trial:
    """Memory for one trial: the control step count, the env handle and anything a task latches."""

    def __init__(self, on_clear=None, on_step=None):
        self._on_clear, self._on_step = on_clear, on_step
        self.hz = CONTROL_HZ
        self.grippers_override = None  # offline tests: a list of {"tcp": xyz, "tips": [xyz, xyz]}
        self.points_override = None  # offline tests: robot link points, an (n, 3) array
        self.clear()
        _install(self)

    def clear(self) -> None:
        self.step, self.tracked, self.env = 0, -1, None
        self.mem: dict = {}
        if self._on_clear is not None:
            self._on_clear(self.mem)

    def new_step(self, env=None, hz=None) -> None:
        self.step += 1
        if env is not None:
            self.env = env
        if hz:
            self.hz = float(hz)
        if self._on_step is not None and self.env is not None:
            self._on_step(self)

    @property
    def t(self) -> float:
        """Seconds since the trial started (control steps / rate)."""
        return self.step / self.hz

    def once(self) -> bool:
        """True the first time it is called in each control step (so a latch advances once per step)."""
        if self.tracked == self.step:
            return False
        self.tracked = self.step
        return True

    def frames(self, seconds: float) -> int:
        return max(1, round(seconds * self.hz))

    def grippers(self) -> list[dict]:
        if self.grippers_override is not None:
            return self.grippers_override
        return grippers(self.env) if self.env is not None else []

    def robot_points(self) -> np.ndarray:
        if self.points_override is not None:
            return np.asarray(self.points_override, dtype=float)
        if self.env is None:
            return np.zeros((0, 3))
        return robot_points(self.env)


def _install(trial: Trial) -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover  (no inspect-robots: offline tests drive Trial.new_step themselves)
        return
    cls = ie.YamArenaEmbodiment
    if not hasattr(cls, "_hh_trials"):
        cls._hh_trials = []
        base_step, base_reset = cls._task_step, cls.reset

        def _task_step(self, *a, **k):
            for tr in cls._hh_trials:
                tr.new_step(self._env, self.info.control_hz)
            return base_step(self, *a, **k)

        def reset(self, *a, **k):
            for tr in cls._hh_trials:
                tr.clear()
            return base_reset(self, *a, **k)

        cls._task_step, cls.reset = _task_step, reset
    cls._hh_trials.append(trial)


# -- simulator readers (only called during a run) ---------------------------------------------------------------
def _bodies(env, key):
    base = env.unwrapped
    origin = base.scene.env_origins[0]
    art = base.scene[key]
    pos = (getattr(art.data.body_pos_w, "torch", art.data.body_pos_w)[0] - origin).detach().cpu().numpy()
    return list(art.body_names), pos.astype(np.float64)


def grippers(env) -> list[dict]:
    """Each gripper's centre between the finger pads ("tcp") and its two fingertips, world frame."""
    out = []
    for key in ("left_robot", "right_robot"):
        names, pos = _bodies(env, key)
        g = pos[names.index("gripper")]
        tips = [pos[names.index(n)] for n in ("tip_left", "tip_right")]
        mid = (tips[0] + tips[1]) / 2
        axis = (mid - g) / (np.linalg.norm(mid - g) + 1e-12)
        out.append({"tcp": g + axis * TCP_PAST_GRIPPER, "tips": tips})
    return out


def robot_points(env) -> np.ndarray:
    """The origin of every link of both arms, plus each gripper's centre and fingertips (world frame, m)."""
    pts = []
    for key in ("left_robot", "right_robot"):
        pts.extend(_bodies(env, key)[1])
    for g in grippers(env):
        pts.extend([g["tcp"], *g["tips"]])
    return np.asarray(pts, dtype=float)


def teleport(env, name: str, xyz) -> None:
    """Put a loose body's centre at ``xyz`` (world, m) in its starting orientation and stop it, the way the
    embodiment's seeded reset moves objects."""
    import torch

    base = env.unwrapped
    obj = base.scene[name]
    pose = obj.data.default_root_pose.torch.clone()
    pose[:, :3] = torch.tensor(np.asarray(xyz, dtype=float), dtype=pose.dtype, device=pose.device)
    pose[:, :3] += base.scene.env_origins
    obj.write_root_pose_to_sim_index(root_pose=pose)
    obj.write_root_velocity_to_sim_index(root_velocity=torch.zeros_like(obj.data.default_root_vel.torch))


# -- geometry ---------------------------------------------------------------------------------------------------
def rot(quat_xyzw) -> np.ndarray:
    x, y, z, w = (float(v) for v in quat_xyzw)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def yaw_quat(yaw: float) -> np.ndarray:
    return np.array([0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)])


def box_corners(state, size) -> np.ndarray:
    """The 8 corners of a box of ``size`` (x, y, z edges) centred on the state's position, world frame."""
    h = np.asarray(size, dtype=float) / 2
    local = np.array([[i * h[0], j * h[1], k * h[2]] for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)])
    return local @ rot(state.quat_xyzw).T + np.asarray(state.pos, dtype=float)


def hull(points) -> np.ndarray:
    """Convex hull of 2-D points, counterclockwise (monotone chain)."""
    pts = sorted({(round(float(p[0]), 9), round(float(p[1]), 9)) for p in points})
    if len(pts) <= 2:
        return np.array(pts)

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return np.array(lower[:-1] + upper[:-1])


def footprint(state, size) -> np.ndarray:
    """A box's outline seen from above (convex polygon)."""
    return hull(box_corners(state, size)[:, :2])


def circle_poly(x, y, r, n=16) -> np.ndarray:
    a = np.linspace(0, 2 * math.pi, n, endpoint=False)
    return np.stack([x + r * np.cos(a), y + r * np.sin(a)], axis=1)


def point_rect(p, rect) -> float:
    """Distance from a 2-D point to an axis-aligned rectangle (x0, x1, y0, y1); 0 inside."""
    x0, x1, y0, y1 = rect
    return math.hypot(max(x0 - p[0], 0.0, p[0] - x1), max(y0 - p[1], 0.0, p[1] - y1))


def point_segment(p, a, b) -> float:
    p, a, b = (np.asarray(v, dtype=float) for v in (p, a, b))
    ab, ap = b - a, p - a
    t = 0.0 if not ab.any() else min(1.0, max(0.0, float(ap @ ab) / float(ab @ ab)))
    return float(np.linalg.norm(ap - t * ab))


def inside(p, poly) -> bool:
    n = len(poly)
    return n >= 3 and all((poly[(i + 1) % n][0] - poly[i][0]) * (p[1] - poly[i][1])
                          - (poly[(i + 1) % n][1] - poly[i][1]) * (p[0] - poly[i][0]) >= 0 for i in range(n))


def poly_rect_gap(poly, rect) -> float:
    """Gap (m) between a convex polygon and an axis-aligned rectangle seen from above; 0 if they overlap. Exact:
    the closest pair of two disjoint convex shapes includes a corner of one of them."""
    x0, x1, y0, y1 = rect
    rc = np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]])
    if any(point_rect(v, rect) == 0.0 for v in poly) or any(inside(c, poly) for c in rc):
        return 0.0
    n = len(poly)
    gap = min(point_rect(v, rect) for v in poly)
    for c in rc:
        gap = min(gap, min(point_segment(c, poly[i], poly[(i + 1) % n]) for i in range(n)))
    return gap


def rect_of(center_xy, size_xy, grow=0.0):
    (cx, cy), (sx, sy) = center_xy, size_xy
    return (cx - sx / 2 - grow, cx + sx / 2 + grow, cy - sy / 2 - grow, cy + sy / 2 + grow)


def point_box(p, center, size, quat_xyzw=(0.0, 0.0, 0.0, 1.0)) -> float:
    """Distance (m) from a point to the surface of a box (0 inside it)."""
    local = (np.asarray(p, dtype=float) - np.asarray(center, dtype=float)) @ rot(quat_xyzw)
    return float(np.linalg.norm(np.maximum(np.abs(local) - np.asarray(size, dtype=float) / 2, 0.0)))


def point_cylinder(p, center, radius, height) -> float:
    """Distance (m) from a point to an upright cylinder (0 inside it)."""
    dr = max(math.hypot(p[0] - center[0], p[1] - center[1]) - radius, 0.0)
    dz = max(abs(p[2] - center[2]) - height / 2, 0.0)
    return math.hypot(dr, dz)


def tilt_deg(state) -> float:
    """Angle between the body's own z axis and the vertical (degrees)."""
    return math.degrees(math.acos(max(-1.0, min(1.0, float(rot(state.quat_xyzw)[2, 2])))))


def flat_tilt_deg(state) -> float:
    """How far a box is from sitting flat on any of its faces (degrees)."""
    up = np.abs(rot(state.quat_xyzw)[2])
    return math.degrees(math.acos(min(1.0, float(up.max()))))


def yaw_deg(state) -> float:
    r = rot(state.quat_xyzw)
    return math.degrees(math.atan2(r[1, 0], r[0, 0]))
