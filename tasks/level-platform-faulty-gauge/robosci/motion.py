"""Scripted motion primitives for oracle policies (privileged, top-down grasps).

Each primitive yields 14-D actions at the control rate. One arm moves at a
time; the idle arm holds its last command.
"""

from __future__ import annotations

from collections.abc import Iterator

import mujoco
import numpy as np

from robosci import scene
from robosci.ik import ArmIK, topdown_rot

SAFE_Z = 0.25
YAWS = (0.0, np.pi, np.pi / 2, -np.pi / 2)  # prefer fingers along x
TILTS = (0.0, 0.25, -0.25, 0.5, -0.5, 0.7, -0.7)  # about the finger axis, rad


def _roty(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def grasp_rot(yaw: float, tilt: float) -> np.ndarray:
    """Top-down rotation tilted about the finger axis (pads stay parallel to faces)."""
    return topdown_rot(yaw) @ _roty(tilt)


# Inspect Robots' default guardrail limits each action dimension to 5% of its
# range per step; stay under 4% so the harness never alters a scripted command.
MAX_STEP_FRAC = 0.04


class Motion:
    """Keeps the current 14-D command and turns Cartesian goals into actions."""

    def __init__(self, env) -> None:  # noqa: ANN001  (SimEnv, avoids import cycle)
        self.env = env
        self.ik = {s: ArmIK(env.model, s) for s in scene.SIDES}
        self.cmd = scene.home_ctrl()
        self.yaw = {s: 0.0 for s in scene.SIDES}
        self.rot = {s: grasp_rot(0.0, 0.0) for s in scene.SIDES}
        self.tilt = {s: 0.0 for s in scene.SIDES}
        self.held: dict[str, tuple[np.ndarray, float] | None] = {s: None for s in scene.SIDES}
        lo = [env.model.jnt_range[env.model.joint(j).id] for j in scene.ARM_JOINTS["left"]]
        self.max_step = np.array([MAX_STEP_FRAC * (r[1] - r[0]) for r in lo])
        self.max_step_grip = MAX_STEP_FRAC * scene.GRIPPER_OPEN

    def _track(self, side: str, q: np.ndarray) -> Iterator[np.ndarray]:
        """Move the arm command to ``q``, splitting any step that exceeds the cap."""
        seg = self.cmd[self._seg(side)]
        while True:
            d = q - seg[:6]
            k = float(np.max(np.abs(d) / self.max_step))
            if k <= 1.0:
                seg[:6] = q
                yield self.cmd.copy()
                return
            seg[:6] = seg[:6] + d / np.ceil(k)
            yield self.cmd.copy()

    def _n_steps(self, seconds: float, dq: np.ndarray | float, cap: np.ndarray | float) -> int:
        """Steps for a cosine-profile move: at least ``seconds``, and peak step under ``cap``."""
        need = float(np.max(np.abs(dq) / cap)) * np.pi / 2  # cosine profile peaks at pi/2 x mean
        return max(2, int(seconds * scene.CONTROL_HZ), int(np.ceil(need)) + 1)

    def _seg(self, side: str) -> slice:
        k = scene.SIDES.index(side)
        return slice(7 * k, 7 * k + 7)

    def arm_q(self, side: str) -> np.ndarray:
        return self.cmd[self._seg(side)][:6].copy()

    def _ik(self, side: str, pos: np.ndarray, rot: np.ndarray, q0: np.ndarray) -> tuple[np.ndarray, float]:
        return self.ik[side].solve(np.asarray(pos, float), rot, q0, restarts=3)

    def plan_vertical(self, side: str, low: np.ndarray, high_z: float = SAFE_Z,
                      yaws: tuple[float, ...] = YAWS, tilts: tuple[float, ...] = TILTS,
                      offset: np.ndarray | None = None) -> np.ndarray:
        """Pick one grasp rotation that reaches ``low`` and the point above it.

        With ``offset`` (held object's centre in the tool frame), ``low`` is where the
        object's centre should go and the tool target is shifted to match.
        Returns the tool position for ``low``.
        """
        for tilt in tilts:
            for y in yaws:
                rot = grasp_rot(y, tilt)
                lo = np.asarray(low, float) - (rot @ offset if offset is not None else 0.0)
                q_lo, e_lo = self._ik(side, lo, rot, self.arm_q(side))
                if e_lo > 3e-3:
                    continue
                _, e_hi = self._ik(side, np.array([lo[0], lo[1], high_z]), rot, q_lo)
                if e_hi < 3e-3:
                    self.rot[side], self.yaw[side], self.tilt[side] = rot, y, tilt
                    return lo
        raise RuntimeError(f"{side} arm cannot reach {np.round(low, 3)} vertically")

    def _vertical_ok(self, side: str, lo: np.ndarray, rot: np.ndarray) -> bool:
        q_lo, e_lo = self._ik(side, lo, rot, self.arm_q(side))
        if e_lo > 3e-3:
            return False
        _, e_hi = self._ik(side, np.array([lo[0], lo[1], SAFE_Z]), rot, q_lo)
        return e_hi < 3e-3

    def plan_pick_place(self, side: str, grasp: np.ndarray, center: np.ndarray,
                        target: np.ndarray) -> tuple[float, float] | None:
        """(yaw, tilt) for a pick at ``grasp`` whose object centre can then be set at
        ``target`` with the same tilt (object stays upright). No side effects."""
        grasp, center, target = (np.asarray(v, float) for v in (grasp, center, target))
        for tilt in TILTS:
            for y in YAWS:
                rot = grasp_rot(y, tilt)
                if not self._vertical_ok(side, grasp, rot):
                    continue
                off = rot.T @ (center - grasp)
                for y2 in YAWS:
                    rot2 = grasp_rot(y2, tilt)
                    if self._vertical_ok(side, target - rot2 @ off, rot2):
                        return y, tilt
        return None

    def solve(self, side: str, pos: np.ndarray) -> np.ndarray:
        """Joints for ``pos`` at the arm's current grasp rotation."""
        q, err = self._ik(side, pos, self.rot[side], self.arm_q(side))
        if err > 5e-3:
            raise RuntimeError(f"{side} arm cannot reach {np.round(pos, 3)} (err {err:.3f})")
        return q

    def joint_move(self, side: str, q_goal: np.ndarray, seconds: float) -> Iterator[np.ndarray]:
        q0 = self.arm_q(side)
        n = self._n_steps(seconds, q_goal - q0, self.max_step)
        for i in range(1, n + 1):
            s = 0.5 - 0.5 * np.cos(np.pi * i / n)  # smooth start and stop
            seg = self.cmd[self._seg(side)]
            seg[:6] = q0 + s * (q_goal - q0)
            yield self.cmd.copy()

    def line(self, side: str, p0: np.ndarray, p1: np.ndarray, seconds: float) -> Iterator[np.ndarray]:
        """Straight Cartesian line at fixed yaw (for vertical approach and retreat)."""
        n = max(2, int(seconds * scene.CONTROL_HZ))
        for i in range(1, n + 1):
            s = 0.5 - 0.5 * np.cos(np.pi * i / n)
            q = self.solve(side, p0 + s * (p1 - p0))
            yield from self._track(side, q)

    def gripper(self, side: str, opening: float, seconds: float = 0.6) -> Iterator[np.ndarray]:
        g0 = self.cmd[self._seg(side)][6]
        n = self._n_steps(seconds, opening - g0, self.max_step_grip)
        for i in range(1, n + 1):
            self.cmd[self._seg(side)][6] = g0 + (opening - g0) * i / n
            yield self.cmd.copy()

    def hold(self, seconds: float) -> Iterator[np.ndarray]:
        for _ in range(max(1, int(seconds * scene.CONTROL_HZ))):
            yield self.cmd.copy()

    def tcp(self, side: str) -> np.ndarray:
        d = self.env.data
        return d.site_xpos[self.env.model.site(f"tcp_{side}").id].copy()

    def goto_above(self, side: str, xy: np.ndarray) -> Iterator[np.ndarray]:
        """Level straight-line transit at SAFE_Z (no dipping over loaded pans).

        Rotation is interpolated; if an intermediate pose is out of reach the
        rest of the move falls back to joint interpolation.
        """
        q_goal = self.solve(side, np.array([xy[0], xy[1], SAFE_Z]))
        r_goal = self.rot[side]
        q_now = self.arm_q(side)
        p0, r0 = self.ik[side].fk(q_now)
        p1 = np.array([xy[0], xy[1], SAFE_Z])
        if p0[2] < SAFE_Z - 0.03:  # start below transit height: go straight up first
            yield from self.joint_move(side, q_goal, 1.5)
            return
        n = max(2, int(1.5 * scene.CONTROL_HZ))
        qa, qb = np.zeros(4), np.zeros(4)
        mujoco.mju_mat2Quat(qa, r0.ravel())
        mujoco.mju_mat2Quat(qb, r_goal.ravel())
        if qa @ qb < 0:
            qb = -qb
        for i in range(1, n + 1):
            s = 0.5 - 0.5 * np.cos(np.pi * i / n)
            qi = (1 - s) * qa + s * qb
            qi /= np.linalg.norm(qi)
            ri = np.zeros(9)
            mujoco.mju_quat2Mat(ri, qi)
            q, err = self._ik(side, p0 + s * (p1 - p0), ri.reshape(3, 3), self.arm_q(side))
            if err > 5e-3:
                yield from self.joint_move(side, q_goal, 1.5 * (1 - s) + 0.2)
                return
            yield from self._track(side, q)

    def pick(self, side: str, pos: np.ndarray, grip: float, body: str | None = None,
             plan: tuple[float, float] | None = None, open_to: float = scene.GRIPPER_OPEN) -> Iterator[np.ndarray]:
        """Grasp at tool position ``pos``; ``grip`` = commanded closed opening (m).

        With ``body``, remember where the object sits in the hand so ``place`` can
        put its centre exactly and keep it upright. ``open_to`` limits the opening
        (slide, m) so the open fingers miss close neighbours.
        """
        pos = np.asarray(pos, float)
        self.open_to = open_to
        yield from self.gripper(side, open_to, 0.3)
        if plan is not None:
            pos = self.plan_vertical(side, pos, yaws=(plan[0],), tilts=(plan[1],))
        else:
            pos = self.plan_vertical(side, pos)
        yield from self.goto_above(side, pos[:2])
        top = np.array([pos[0], pos[1], SAFE_Z])
        yield from self.line(side, top, pos, 1.2)
        yield from self.gripper(side, grip, 0.8)
        yield from self.hold(0.3)
        yield from self.line(side, pos, top, 1.0)
        self.held[side] = None
        if body is not None:
            m, d = self.env.model, self.env.data
            site = m.site(f"tcp_{side}").id
            r = d.site_xmat[site].reshape(3, 3)
            off = r.T @ (d.xpos[m.body(body).id] - d.site_xpos[site])
            self.held[side] = (off, self.tilt[side])

    def place(self, side: str, pos: np.ndarray, center: bool = False) -> Iterator[np.ndarray]:
        """Lower and release. ``pos`` is the tool target, or with ``center`` the
        held object's centre target (same tilt as at pick, so it lands upright).
        """
        pos = np.asarray(pos, float)
        if center and self.held[side] is not None:
            off, tilt = self.held[side]
            try:
                pos = self.plan_vertical(side, pos, tilts=(tilt,), offset=off)
            except RuntimeError:  # last resort: any tilt (object may land slightly tipped)
                pos = self.plan_vertical(side, pos, offset=off)
        else:
            pos = self.plan_vertical(side, pos)
        yield from self.goto_above(side, pos[:2])
        top = np.array([pos[0], pos[1], SAFE_Z])
        yield from self.line(side, top, pos, 1.2)
        yield from self.gripper(side, getattr(self, "open_to", scene.GRIPPER_OPEN), 0.6)
        yield from self.line(side, pos, top, 0.8)
        self.held[side] = None

    def press_down(self, side: str, top: np.ndarray, depth: float, seconds: float = 0.8) -> Iterator[np.ndarray]:
        """Close the gripper and push straight down on a button whose top is at ``top``.
        Leaves the arm pressing; call ``press_up`` to release."""
        top = np.asarray(top, float)
        yield from self.gripper(side, 0.0, 0.3)
        p_touch = self.plan_vertical(side, top + np.array([0, 0, 0.005]))
        yield from self.goto_above(side, p_touch[:2])
        yield from self.line(side, np.array([p_touch[0], p_touch[1], SAFE_Z]), p_touch, 1.0)
        p_low = p_touch - np.array([0, 0, depth + 0.005])
        yield from self.line(side, p_touch, p_low, seconds)
        self._press = getattr(self, "_press", {})
        self._press[side] = (p_touch, p_low)

    def press_up(self, side: str) -> Iterator[np.ndarray]:
        p_touch, p_low = self._press[side]
        yield from self.line(side, p_low, np.array([p_low[0], p_low[1], SAFE_Z]), 1.0)

    def press(self, side: str, top: np.ndarray, depth: float, hold: float = 0.5) -> Iterator[np.ndarray]:
        """Tap a push button: down by ``depth`` (+5 mm), hold, back up to SAFE_Z."""
        yield from self.press_down(side, top, depth)
        yield from self.hold(hold)
        yield from self.press_up(side)

    def home(self, side: str) -> Iterator[np.ndarray]:
        yield from self.joint_move(side, scene.HOME_ARM[side].copy(), 1.5)
