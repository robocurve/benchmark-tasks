"""The scripted demo's eyes and steady hands, for wrist_form_cuffs.py.

Every trial starts from a different layout, so a fixed plan cannot work. The ``yam_scripted`` policy looks at the top
camera twice, each time with both arms at the ready pose: on its first step, to find cuff A, and again after cuff A
is on its form, to find cuff B (the chain drags it while cuff A is moved). It uses only what a policy is given: the RGB
image, the camera's published calibration, the sizes in the task's setup text, and the arms' own joint and gripper
readings. It reads no simulator state.

How a cuff is found: the nickel-plated steel is the only colourless thing on the oak table, so each pixel gets a
"colourless" value, and the cuff's known shape (lock case, double bar and rivet boss; not the swinging bar, whose
angle is unknown) is slid and turned over the image until it lies on colourless pixels best.

The simulated arm's joints are soft, so under gravity and load each settles short of its target. The demo closes
that gap from the joint angles it observes (a bounded integral correction of the TCP pose).
"""

from __future__ import annotations

import math
import os

import numpy as np

_EST: dict = {}  # the last estimates, for the log
_PLAN_LOG = open(os.environ["CUFF_DEBUG"] + ".plan", "w") if os.environ.get("CUFF_DEBUG") else None

SAG_GAIN = float(os.environ.get("CUFF_SAG_GAIN", "0.25"))  # of the TCP pose error per control step
SAG_MAX = float(os.environ.get("CUFF_SAG_MAX", "0.30"))  # rad per joint
UNWIND = ("ease off", "let go")  # labels during which a hand's built-up push runs down
UNWIND_RATE = 0.8


# -- the camera ----------------------------------------------------------------------------------------------------------
def _rotation(q) -> np.ndarray:
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


_CAM: dict = {}


def _camera():
    if not _CAM:
        from yam_arena import station
        from yam_arena.constants import CAM_RENDER_HEIGHT, CAM_RENDER_WIDTH

        cam = station.top_camera("calibrated")
        _CAM.update(pos=np.asarray(cam.pos, float), rot=_rotation(cam.rot_xyzw),
                    lens=cam.lens.resized(CAM_RENDER_WIDTH, CAM_RENDER_HEIGHT), rw=CAM_RENDER_WIDTH, rh=CAM_RENDER_HEIGHT)
    return _CAM


def to_image(p, height: int, width: int) -> np.ndarray:
    """The image point (u, v) of world point(s) ``p`` in an image ``height`` x ``width`` resized from the render."""
    cam = _camera()
    lens = cam["lens"]
    c = (np.asarray(p, float) - cam["pos"]) @ cam["rot"]
    ub = lens.fx * c[..., 0] / -c[..., 2] + lens.cx
    vb = -lens.fy * c[..., 1] / -c[..., 2] + lens.cy
    return np.stack([(ub + 0.5) * width / cam["rw"] - 0.5, (vb + 0.5) * height / cam["rh"] - 0.5], -1)


def colourless(img: np.ndarray) -> np.ndarray:
    """0..1 per pixel: 1 where red and blue are equal (steel, any brightness), 0 on the oak table, its shadows and the
    skin-coloured forms (red well above blue)."""
    a = img.astype(float)
    total = a.sum(-1) + 1.0
    chroma = (a[..., 0] - a[..., 2]) / total  # oak about 0.07, its shadows more, steel about 0
    return np.clip((0.05 - chroma) / 0.03, 0.0, 1.0)


def _sample(field: np.ndarray, uv: np.ndarray) -> np.ndarray:
    """``field`` at fractional pixel positions ``uv`` (..., 2), bilinear, 0 outside."""
    h, w = field.shape
    u, v = uv[..., 0], uv[..., 1]
    ok = (u >= 0) & (u <= w - 1.001) & (v >= 0) & (v <= h - 1.001)
    u, v = np.clip(u, 0, w - 1.001), np.clip(v, 0, h - 1.001)
    j, i = u.astype(int), v.astype(int)
    fu, fv = u - j, v - i
    out = (field[i, j] * (1 - fu) * (1 - fv) + field[i, j + 1] * fu * (1 - fv)
           + field[i + 1, j] * (1 - fu) * fv + field[i + 1, j + 1] * fu * fv)
    return np.where(ok, out, 0.0)


def model_points(m, c: str) -> np.ndarray:
    """Points through the lock case, the double bar and the rivet boss of cuff ``c`` (N x 3, relative to the rivet,
    for the cuff at its nominal heading)."""
    pts = []
    cx, cy = m.CASE_C
    for x in np.arange(-m.CASE[0] / 2 + 0.001, m.CASE[0] / 2, 0.002):
        for y in np.arange(-m.CASE[1] / 2 + 0.001, m.CASE[1] / 2, 0.002):
            for z in (-m.CASE[2] / 2, 0.0, m.CASE[2] / 2):
                pts.append((cx + x, cy + y, z))
    for a, b in zip(m.BOW_PTS[:-1], m.BOW_PTS[1:]):
        d = b - a
        n = np.array([-d[1], d[0]]) / np.linalg.norm(d)
        for t in np.arange(0.0, 1.0, 0.002 / np.linalg.norm(d)):
            for s in (-0.003, 0.0, 0.003):
                for z in (-m.BOW_T / 2, 0.0, m.BOW_T / 2):
                    p = a + t * d + s * n
                    pts.append((p[0], p[1], z))
    for x in np.arange(-m.BOSS_R, m.BOSS_R + 1e-9, 0.002):
        for y in np.arange(-m.BOSS_R, m.BOSS_R + 1e-9, 0.002):
            if x * x + y * y <= m.BOSS_R ** 2:
                for z in (-m.BOW_T / 2, m.BOW_T / 2):
                    pts.append((x, y, z))
    return np.array(pts) @ m.BASE[c].T


def fit_cuff(img: np.ndarray, m, c: str, prior_xy, prior_yaw: float = 0.0, reach=(0.024, 0.024, 12.0)):
    """Where cuff ``c``'s rivet is and how far the cuff is turned (rad), found near a prior within ``reach``
    (m, m, degrees): returns (xy, yaw, how much of the shape lies on colourless pixels)."""
    h, w = img.shape[:2]
    field = colourless(img)
    pts = model_points(m, c)
    z0 = m.TOP + 0.0058
    best = (-1.0, float(prior_xy[0]), float(prior_xy[1]), float(prior_yaw))
    spans = [(reach[0], reach[1], math.radians(reach[2]), 0.003, math.radians(2.0)),
             (0.003, 0.003, math.radians(2.0), 0.001, math.radians(0.5)),
             (0.001, 0.001, math.radians(0.5), 0.00025, math.radians(0.25))]
    for rx, ry, ryaw, step, ystep in spans:
        _, x0, y0, a0 = best
        dxs = np.arange(-rx, rx + 1e-9, step)
        dys = np.arange(-ry, ry + 1e-9, step)
        gx, gy = np.meshgrid(dxs, dys, indexing="ij")
        shift = np.stack([x0 + gx, y0 + gy, np.full_like(gx, z0)], -1).reshape(-1, 1, 3)
        for a in a0 + np.arange(-ryaw, ryaw + 1e-9, ystep):
            ca, sa = math.cos(a), math.sin(a)
            rot = np.array([[ca, -sa, 0.0], [sa, ca, 0.0], [0.0, 0.0, 1.0]])
            score = _sample(field, to_image(shift + (pts @ rot.T)[None], h, w)).mean(-1)
            k = int(np.argmax(score))
            if score[k] > best[0]:
                best = (float(score[k]), float(shift[k, 0, 0]), float(shift[k, 0, 1]), float(a))
    return np.array([best[1], best[2]]), best[3], best[0]


# -- the policy hooks ----------------------------------------------------------------------------------------------------
def install() -> None:
    """Hook yam_scripted (if it is importable): plan from the camera when the task asks for it, and correct each
    arm's sag from its own joint readings."""
    try:
        from inspect_robots.types import Action, ActionChunk
        from inspect_robots_yam.packing import STATE_KEY
        from yam_arena import ik, scripted
        from yam_arena.demo import ArmPlan, Waypoint
    except Exception:  # the task file also loads where yam_scripted is not installed (the checker)
        return
    cls = scripted.YamScriptedPolicy
    if getattr(cls, "_cuffdemo", False):
        return
    plain_reset, plain_act = cls.reset, cls.act

    def tcp_jacobian(q, base):
        p0, r0 = ik.tcp_pose(q, base)
        jac = np.empty((6, 6))
        for k in range(6):
            qp = np.array(q, float)
            qp[k] += 1e-4
            p, r = ik.tcp_pose(qp, base)
            jac[:3, k] = (p - p0) / 1e-4
            w = (r - r0) / 1e-4 @ r0.T
            jac[3:, k] = 0.5 * np.array([w[2, 1] - w[1, 2], w[0, 2] - w[2, 0], w[1, 0] - w[0, 1]])
        return jac

    def log_rot(r):
        c = np.clip((np.trace(r) - 1.0) / 2.0, -1.0, 1.0)
        th = math.acos(c)
        v = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])
        return 0.5 * v if th < 1e-7 else th / (2.0 * math.sin(th)) * v

    def _mine(self) -> bool:
        return self._task is not None and hasattr(self._task.module, "cuffdemo_task")

    def reset(self, scene):
        plain_reset(self, scene)
        self._cuff_stage = 0 if _mine(self) and getattr(self._task.module, "LOOKS", 0) else -1
        self._cuff_prev = None
        self._cuff_off = np.zeros(14)

    def _extend(self, plan: dict, t: float) -> None:
        """Add the next stretch of both hands' plans, starting now."""
        for arm, more in plan.items():
            have = self._plans[arm]
            gap = t + self._dt - have.duration
            last = have.waypoints[-1]
            if gap > 1e-6:
                have.waypoints.append(Waypoint(gap, last.pos, last.finger, "look", last.yaw))
            have.waypoints.extend(more)
        self._end = max((p.duration for p in self._plans.values()), default=0.0)

    def act(self, observation):
        if not _mine(self):
            return plain_act(self, observation)
        m = self._task.module
        t = float(observation.state_time)
        if self._cuff_stage >= 0 and self._cuff_stage < m.LOOKS and t + 1e-6 >= self._end - 0.05:
            # both arms are at the ready pose and still: look, and plan the next stretch from what the camera shows
            more = m.plan_stage(self._cuff_stage, np.asarray(observation.images["top_cam"]), m)
            self._cuff_stage += 1
            if more:
                _extend(self, more, t)
        labels = {arm: plan.target(t)[3] for arm, plan in self._plans.items()}
        chunk = plain_act(self, observation)
        planned = np.asarray(chunk.actions[0].data, float).copy()
        measured = np.asarray(observation.state[STATE_KEY], float)
        fields = [f"{int(round(t * 20)):4d}"]
        if self._cuff_prev is not None:
            for arm, idx in (("left", np.r_[0:6]), ("right", np.r_[7:13])):
                base = self._bases[arm]
                p_plan, r_plan = ik.tcp_pose(planned[idx], base)
                p_meas, r_meas = ik.tcp_pose(measured[idx], base)
                if labels.get(arm) in UNWIND:
                    self._cuff_off[idx] *= UNWIND_RATE
                else:
                    err = np.r_[p_plan - p_meas, log_rot(r_plan @ r_meas.T)]
                    jac = tcp_jacobian(measured[idx], base)
                    dq = jac.T @ np.linalg.solve(jac @ jac.T + 1e-4 * np.eye(6), err)
                    self._cuff_off[idx] = np.clip(self._cuff_off[idx] + SAG_GAIN * dq, -SAG_MAX, SAG_MAX)
                if _PLAN_LOG is not None:
                    fields.append(f"{arm}: plan=({p_plan[0]:.4f},{p_plan[1]:.4f},{p_plan[2]:.4f}) "
                                  f"meas=({p_meas[0]:.4f},{p_meas[1]:.4f},{p_meas[2]:.4f}) "
                                  f"grip={measured[idx[-1] + 1]:.3f} off={np.linalg.norm(self._cuff_off[idx]):.3f} "
                                  f"[{labels.get(arm)}]")
        if _PLAN_LOG is not None:
            _PLAN_LOG.write(" ".join(fields) + "\n")
            _PLAN_LOG.flush()
        self._cuff_prev = planned
        out = planned + self._cuff_off
        return ActionChunk(actions=[Action(data=out, meta=chunk.actions[0].meta)], control_hz=chunk.control_hz)

    cls.reset, cls.act = reset, act
    cls._cuffdemo = True
