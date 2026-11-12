"""Scoring logic for chainsaw-brake-and-scabbard-before-carrying. Pure Python and NumPy: task.py feeds it measured
states every control step, and the unit tests feed it made-up ones.

End-state scoring: the score is that of the current step. Only violations (the saw pushed or tipped, a hand on the
chain) are latched. Releasing the brake or pulling the scabbard off lowers the score again.
"""

from __future__ import annotations

import math

import numpy as np

import geometry as g

CONTROL_HZ = 20


def rotation(quat_xyzw) -> np.ndarray:
    x, y, z, w = (float(v) for v in quat_xyzw)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def tilt(quat_xyzw) -> float:
    """Angle (rad) between the body's z axis and the world z axis."""
    return math.acos(max(-1.0, min(1.0, float(rotation(quat_xyzw)[2, 2]))))


def yaw(quat_xyzw) -> float:
    R = rotation(quat_xyzw)
    return math.atan2(R[1, 0], R[0, 0])


def bar_axis_world() -> tuple[float, float]:
    """World (x, y) of the bar's root axis point: the saw's origin."""
    return g.SAW_X, g.SAW_Y


def brake_engaged(guard_angle: float) -> bool:
    """The chain brake is on when the hand guard has swung forward to or past BRAKE_ON (negative angle = forward)."""
    return guard_angle <= g.BRAKE_ON


def scabbard_cover(scab_pos, scab_quat, saw_pos=None) -> tuple[float, bool]:
    """How much of the visible bar the scabbard covers (0..1) and whether it is on the bar (aligned on its axis and
    mid-plane, flat, pointing along it). Its open end faces the housing; its centre is mid-length, mid-channel."""
    bx, by = (g.SAW_X, g.SAW_Y) if saw_pos is None else (float(saw_pos[0]), float(saw_pos[1]))
    bz = g.TOP + g.BAR_Z
    x, y, z = (float(v) for v in scab_pos)
    a, b = y - g.SCAB_LEN / 2, y + g.SCAB_LEN / 2    # the scabbard's two ends (its open end faces the housing)
    lo, hi = sorted((by - g.S * g.BAR_LEN, by))      # the bar's visible span, tip to the cover's edge
    covered = max(0.0, min(b, hi) - max(a, lo)) / g.BAR_LEN
    aligned = (abs(x - bx) <= g.COVER_DX and abs(z - bz) <= g.COVER_DZ and tilt(scab_quat) <= g.COVER_TILT
               and abs(math.remainder(yaw(scab_quat) - g.SCAB_YAW, 2 * math.pi)) <= g.COVER_YAW)
    return covered, aligned


def on_chain(tcp, saw_pos=None) -> bool:
    """A gripper tool point inside the bar-and-chain's zone (a margin round its outline)."""
    bx, by = (g.SAW_X, g.SAW_Y) if saw_pos is None else (float(saw_pos[0]), float(saw_pos[1]))
    x, y, z = (float(v) for v in tcp)
    lo, hi = sorted((by - g.S * (g.BAR_LEN + 0.010), by + g.S * 0.010))
    return abs(x - bx) <= g.CHAIN_ZONE["half_w"] and lo <= y <= hi and abs(z - (g.TOP + g.BAR_Z)) <= g.CHAIN_ZONE["dz"]


# the arms count as clear when every tool point is above this height or well outside the work area
CLEAR_Z = g.TOP + 0.14
WORK_BOX = dict(x=(0.12, 0.42), y=(-0.60, 0.30) if g.S > 0 else (-0.30, 0.60))      # 30 x 90 cm round the saw and cover


def arms_clear(tcps) -> bool:
    for tcp in tcps:
        x, y, z = (float(v) for v in tcp)
        inside = WORK_BOX["x"][0] <= x <= WORK_BOX["x"][1] and WORK_BOX["y"][0] <= y <= WORK_BOX["y"][1]
        if inside and z < CLEAR_Z:
            return False
    return True


class Scorer:
    """Feed :meth:`step` once per control step; read ``score``, ``success`` and ``details`` from the result."""

    def __init__(self, hold_s: float = g.HOLD_S, hz: int = CONTROL_HZ):
        self.hold_steps = int(round(hold_s * hz))
        self.reset()

    def reset(self) -> None:
        self.saw0 = None
        self.moved = False
        self.tipped = False
        self.chain_touched = False
        self.hold = 0
        self.why: list[str] = []

    def step(self, *, saw_pos, saw_quat, guard_angle, scab_pos, scab_quat, tcps) -> dict:
        saw_pos = np.asarray(saw_pos, float)
        if self.saw0 is None:
            self.saw0 = saw_pos.copy()
        if float(np.linalg.norm(saw_pos - self.saw0)) > g.SAW_MOVE_LIMIT and not self.moved:
            self.moved = True
            self.why.append("saw moved")
        if tilt(saw_quat) > g.SAW_TILT_LIMIT and not self.tipped:
            self.tipped = True
            self.why.append("saw tipped")
        brake = brake_engaged(guard_angle)
        covered, aligned = scabbard_cover(scab_pos, scab_quat, self.saw0)
        cover_full = aligned and covered >= g.COVER_FULL
        # a hand on the bar or chain (a tool point in the chain's zone) unless a scabbard on the bar is covering it
        for tcp in tcps:
            if on_chain(tcp, self.saw0):
                # a tool point holding the scabbard (beside its middle, within its length) is not on the chain
                sx, sy, sz = (float(v) for v in scab_pos)
                over = (abs(float(tcp[0]) - sx) <= 0.035 and abs(float(tcp[2]) - sz) <= 0.030
                        and sy - g.SCAB_LEN / 2 - 0.01 <= float(tcp[1]) <= sy + g.SCAB_LEN / 2 + 0.01)
                if not over and not self.chain_touched:
                    self.chain_touched = True
                    self.why.append("hand on the chain")
        clear = arms_clear(tcps)
        score = 0.0
        if brake or (aligned and covered >= 0.5):
            score = 0.25
        if cover_full:
            score = 0.5
        if brake and cover_full:
            score = 0.75
        holding = brake and cover_full and clear and not (self.moved or self.tipped or self.chain_touched)
        self.hold = self.hold + 1 if holding else 0
        success = self.hold >= self.hold_steps
        if success:
            score = 1.0
        if self.moved or self.tipped or self.chain_touched:
            score = min(score, 0.25)
        return {"score": score, "success": success, "brake": brake, "covered": covered, "aligned": aligned,
                "cover_full": cover_full, "arms_clear": clear, "hold_steps": self.hold,
                "moved": self.moved, "tipped": self.tipped, "chain_touched": self.chain_touched}
