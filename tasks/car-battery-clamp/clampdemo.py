"""The scripted demo's eyes, for car_battery_clamp.py.

Every trial starts from a different layout (the seed moves the battery and the wrench rack and tips the clamp a
different way), so a fixed plan cannot work. On its first step the ``yam_scripted`` policy looks at the top camera's
first frame, finds the battery and the rack there, and plans from those estimates. It uses only what a policy is given:
the RGB image, the camera's published calibration, and the sizes in the task's setup text. While it runs it reads no
simulator state; the two edge-finding offsets below were calibrated once against known start layouts.

How it finds them: every pixel's ray is cut with the plane of a known top face (the battery's lid, the rack's top).
The battery's case is the dark blob; its back edge and its two ends fix its centre (its front face, seen from above
and behind, falls in front of the lid's plane, so it does not move those edges). The rack is the orange blob; its
back edge and the middle of its top fix its centre.
"""

from __future__ import annotations

import os

import numpy as np

_EST: dict = {}  # the last estimate, for the log
_PLAN_LOG = open(os.environ["CLAMP_DEBUG"] + ".plan", "w") if os.environ.get("CLAMP_DEBUG") else None


def _rotation(q) -> np.ndarray:
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])



def _camera():
    from yam_arena import station
    from yam_arena.constants import CAM_RENDER_HEIGHT, CAM_RENDER_WIDTH

    cam = station.top_camera("calibrated")
    return cam, cam.lens.resized(CAM_RENDER_WIDTH, CAM_RENDER_HEIGHT), CAM_RENDER_WIDTH, CAM_RENDER_HEIGHT


def ray(u, v, height: int, width: int) -> tuple[np.ndarray, np.ndarray]:
    """The top camera's centre and the ray direction (world) through image point(s) (u, v), in pixels of an image
    ``height`` x ``width`` resized from the 640 x 360 render as the embodiment does; u, v may be fractional."""
    cam, lens, rw, rh = _camera()
    ub = (np.asarray(u, float) + 0.5) * rw / width - 0.5  # the point in the render
    vb = (np.asarray(v, float) + 0.5) * rh / height - 0.5
    d = np.stack([(ub - lens.cx) / lens.fx, -(vb - lens.cy) / lens.fy, -np.ones_like(ub)], -1)  # OpenGL camera
    return np.asarray(cam.pos, float), d @ _rotation(cam.rot_xyzw).T


def to_plane(u, v, height: int, width: int, z: float) -> np.ndarray:
    """Where the ray through image point(s) (u, v) meets the level plane at height ``z``."""
    o, d = ray(u, v, height, width)
    t = (z - o[2]) / d[..., 2]
    return o + t[..., None] * d


def to_image(p, height: int, width: int) -> np.ndarray:
    """The image point (u, v) of world point(s) ``p``."""
    cam, lens, rw, rh = _camera()
    c = (np.asarray(p, float) - np.asarray(cam.pos, float)) @ _rotation(cam.rot_xyzw)
    ub = lens.fx * c[..., 0] / -c[..., 2] + lens.cx
    vb = -lens.fy * c[..., 1] / -c[..., 2] + lens.cy
    return np.stack([(ub + 0.5) * width / rw - 0.5, (vb + 0.5) * height / rh - 0.5], -1)


def on_plane(height: int, width: int, z: float) -> np.ndarray:
    """Where each pixel's ray meets the level plane at height ``z`` (H x W x 3)."""
    u, v = np.meshgrid(np.arange(width, dtype=float), np.arange(height, dtype=float))
    return to_plane(u, v, height, width, z)


def _crossings(profile: np.ndarray, level: float) -> list[float]:
    """Fractional indices where ``profile`` crosses ``level``."""
    s = profile - level
    idx = np.nonzero(np.sign(s[:-1]) != np.sign(s[1:]))[0]
    return [float(i + s[i] / (s[i] - s[i + 1])) for i in idx if s[i] != s[i + 1]]


def _edge(gray, h, w, p_from, p_to, z, level=None) -> np.ndarray | None:
    """The first point from ``p_from`` toward ``p_to`` (world, on the plane at ``z``) where the image brightness
    crosses ``level`` (default: halfway between the two ends' brightness), found to a fraction of a pixel."""
    a, b = to_image([[*p_from, z], [*p_to, z]], h, w)
    n = int(max(abs(b[0] - a[0]), abs(b[1] - a[1]))) * 6 + 2
    us, vs = np.linspace(a[0], b[0], n), np.linspace(a[1], b[1], n)
    prof = np.array([_bilinear(gray, uu, vv) for uu, vv in zip(us, vs)])
    if level is None:
        level = (float(np.median(prof[: n // 6])) + float(np.median(prof[-n // 6:]))) / 2
    cr = _crossings(prof, level)
    if not cr:
        return None
    k = cr[0]
    return to_plane(np.interp(k, np.arange(n), us), np.interp(k, np.arange(n), vs), h, w, z)


def find_battery(img: np.ndarray, m, avoid_y=()) -> np.ndarray:
    """The battery case's centre (x, y), from the two lid edges that no shadow touches: the front edge, where the lid
    meets the darker front face (seen from above and behind), and the right-hand end, where the lid meets the bench.
    Each is found to a fraction of a pixel along many short scan lines cut with the lid's plane; ``avoid_y`` lists
    y positions (m) where something stands in front of the front edge (the clamp, the wrench)."""
    h, w = img.shape[:2]
    gray = img.astype(float).mean(-1)
    z = m.BATT_TOP
    nx, ny = m.BATT_XY
    hx, hy = m.BATT[0] / 2, m.BATT[1] / 2
    fronts = []
    for y in np.linspace(ny - hy + 0.03, ny + hy - 0.03, 31):
        if any(abs(y - ay) < 0.035 for ay in avoid_y):
            continue
        q = _edge(gray, h, w, (nx - hx + 0.035, y), (nx - hx - 0.035, y), z)
        if q is not None:
            fronts.append(q)
    rights = []
    for x in np.linspace(nx - hx + 0.03, nx + hx - 0.03, 15):
        q = _edge(gray, h, w, (x, ny - hy - 0.04), (x, ny - hy + 0.04), z)
        if q is not None:
            rights.append(q)
    if len(fronts) < 6 or len(rights) < 6:
        raise RuntimeError(f"clampdemo: battery edges not found ({len(fronts)} front, {len(rights)} end)")
    front = float(np.median([q[0] for q in fronts]))
    right = float(np.median([q[1] for q in rights]))
    return np.array([front + hx, right + hy])


def _bilinear(img: np.ndarray, u: float, v: float) -> float:
    h, w = img.shape[:2]
    u, v = min(max(u, 0.0), w - 1.001), min(max(v, 0.0), h - 1.001)
    i, j = int(v), int(u)
    fv, fu = v - i, u - j
    return float(img[i, j] * (1 - fu) * (1 - fv) + img[i, j + 1] * fu * (1 - fv) + img[i + 1, j] * (1 - fu) * fv
                 + img[i + 1, j + 1] * fu * fv)
def find_rack(img: np.ndarray, m) -> np.ndarray:
    """The wrench holder's centre (x, y). A coarse centre from the orange pixels, then the holder's top face's two
    ends and back edge to a fraction of a pixel, from where the orange-ness (red minus blue) crosses halfway between
    holder and surroundings, cut with the top face's plane."""
    h, w = img.shape[:2]
    c = img.astype(float)
    z = m.RACK_TOP
    p = on_plane(h, w, z)
    ci = c.astype(int)
    orange = (ci[..., 0] > 120) & (ci[..., 0] - ci[..., 2] > 70) & (ci[..., 1] > 0.3 * ci[..., 0]) & (ci[..., 1] < 0.8 * ci[..., 0])
    near = (np.abs(p[..., 0] - m.RACK_XY[0]) < 0.06) & (np.abs(p[..., 1] - m.RACK_XY[1]) < 0.06)
    pts = p[orange & near]
    if len(pts) < 20:
        raise RuntimeError(f"clampdemo: found {len(pts)} rack pixels")
    coarse = np.array([np.percentile(pts[:, 0], 98) - m.RACK[0] / 2, float(np.median(pts[:, 1]))])
    redness = c[..., 0] - c[..., 2]
    inside = float(np.percentile(redness[orange & near], 75))
    half = np.array(m.RACK[:2]) / 2
    ends = {1: [], -1: []}
    for x in np.linspace(coarse[0] - half[0] + 0.008, coarse[0] + half[0] - 0.008, 7):
        for side in (1, -1):
            a = to_image([[x, coarse[1] + side * (half[1] + 0.02), z]], h, w)[0]
            b = to_image([[x, coarse[1] + side * (half[1] - 0.012), z]], h, w)[0]
            n = int(abs(b[0] - a[0])) * 6 + 2
            us, vs = np.linspace(a[0], b[0], n), np.linspace(a[1], b[1], n)
            prof = np.array([_bilinear(redness, uu, vv) for uu, vv in zip(us, vs)])
            level = (inside + float(prof[:3].mean())) / 2
            cr = _crossings(prof, level)
            if cr:
                k = cr[0]
                ends[side].append(to_plane(np.interp(k, np.arange(n), us), np.interp(k, np.arange(n), vs), h, w, z))
    backs = []
    for y in np.concatenate([np.linspace(coarse[1] - half[1] + 0.004, coarse[1] - 0.008, 3),
                             np.linspace(coarse[1] + 0.008, coarse[1] + half[1] - 0.004, 3)]):  # either side of the slot
        a = to_image([[coarse[0] + half[0] + 0.02, y, z]], h, w)[0]
        b = to_image([[coarse[0] + half[0] - 0.015, y, z]], h, w)[0]
        n = int(abs(b[1] - a[1])) * 6 + 2
        us, vs = np.linspace(a[0], b[0], n), np.linspace(a[1], b[1], n)
        prof = np.array([_bilinear(redness, uu, vv) for uu, vv in zip(us, vs)])
        level = (inside + float(prof[:3].mean())) / 2
        cr = _crossings(prof, level)
        if cr:
            k = cr[0]
            backs.append(to_plane(np.interp(k, np.arange(n), us), np.interp(k, np.arange(n), vs), h, w, z))
    if len(ends[1]) < 3 or len(ends[-1]) < 3 or len(backs) < 2:
        return coarse
    y = (float(np.median([q[1] for q in ends[1]])) + float(np.median([q[1] for q in ends[-1]]))) / 2
    x = float(np.median([q[0] for q in backs])) - half[0]
    return np.array([x, y])


# Edge-finding bias, measured once on the first frames of five seeded starts against their known layouts (the half-
# pixel convention of the resize and the soft edges of the render): subtracted from every estimate (m).
BATTERY_BIAS = np.array([0.0015, 0.0015])
RACK_BIAS = np.array([-0.0005, 0.0010])  # re-measured on four seeded starts: the old value left it 1.7 mm short in x


def plan_from_image(img: np.ndarray, m) -> dict:
    """Both hands' plans from the first top-camera frame."""
    rack = find_rack(img, m) - RACK_BIAS
    batt = find_battery(img, m, avoid_y=(m.POS_XY[1], rack[1])) - BATTERY_BIAS
    post = batt + (np.asarray(m.POS_XY) - np.asarray(m.BATT_XY))
    _EST.update(battery=batt, rack=rack, post=post)
    print(f"clampdemo: battery at ({batt[0]:.4f}, {batt[1]:.4f}), rack at ({rack[0]:.4f}, {rack[1]:.4f})", flush=True)
    return m.plans(tuple(post), tuple(rack))


# The simulated YAM's joints are soft (position gains of 80 N m/rad at the shoulder and 10 at the wrist), so under
# gravity and load each joint settles short of its target. The demo closes that gap from the joint angles it observes:
# a slow integral term per joint, bounded.
SAG_GAIN = 0.08  # of the remaining error added to the offset per control step
SAG_MAX = 0.12  # rad


def install() -> None:
    """Hook yam_scripted (if it is importable) so that, for this task, its first step re-plans from the image."""
    try:
        from inspect_robots.types import Action, ActionChunk
        from inspect_robots_yam.packing import STATE_KEY
        from yam_arena import scripted
        from yam_arena.demo import ArmPlan
    except Exception:  # the task file also loads where yam_scripted is not installed (the tests, the checker)
        return
    cls = scripted.YamScriptedPolicy
    if getattr(cls, "_clampdemo", False):
        return
    plain_reset, plain_act = cls.reset, cls.act
    arm_idx = np.r_[0:6, 7:13]

    def reset(self, scene):
        plain_reset(self, scene)
        self._clamp_look = self._task is not None and hasattr(self._task.module, "plan_from_image_hook")
        self._clamp_prev = None
        self._clamp_off = np.zeros(14)

    def act(self, observation):
        if not getattr(self, "_clamp_on", True):
            return plain_act(self, observation)
        if getattr(self, "_clamp_look", False):
            self._clamp_look = False
            m = self._task.module
            plan = plan_from_image(np.asarray(observation.images["top_cam"]), m)
            self._plans = {arm: ArmPlan(list(w), self._bases[arm]) for arm, w in plan.items()}
            self._end = max((p.duration for p in self._plans.values()), default=0.0)
        if self._task is None or not hasattr(self._task.module, "plan_from_image_hook"):
            return plain_act(self, observation)
        if _PLAN_LOG is not None and self._plans:
            t = float(observation.state_time)
            parts = []
            for arm, plan in self._plans.items():
                pos, _, finger, label = plan.target(t)
                parts.append(f"{arm}=({pos[0]:.4f},{pos[1]:.4f},{pos[2]:.4f}) f={finger:.4f} [{label}]")
            _PLAN_LOG.write(f"{int(round(t * 20)):4d} " + " ".join(parts) + "\n")
            _PLAN_LOG.flush()
        chunk = plain_act(self, observation)
        planned = np.asarray(chunk.actions[0].data, float).copy()
        measured = np.asarray(observation.state[STATE_KEY], float)
        if self._clamp_prev is not None:
            err = self._clamp_prev[arm_idx] - measured[arm_idx]
            self._clamp_off[arm_idx] = np.clip(self._clamp_off[arm_idx] + SAG_GAIN * err, -SAG_MAX, SAG_MAX)
        self._clamp_prev = planned
        out = planned + self._clamp_off
        return ActionChunk(actions=[Action(data=out, meta=chunk.actions[0].meta)], control_hz=chunk.control_hz)

    cls.reset, cls.act = reset, act
    cls._clampdemo = True
