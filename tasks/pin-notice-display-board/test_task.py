"""Offline checks for the pin-notice task: geometry, the tether's stability, the scoring rule and the notice
findings on hand-built states. No Isaac Sim needed (PYTHONPATH must hold yam_arena)."""
import math
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
os.environ.setdefault("PIN_DEBUG", "0")
import task  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402


def _card_pose(u_centre, y=task.CARD_Y, twist_deg=0.0, out=task.CARD_T / 2):
    """A notice pose (pos, quat xyzw) lying on the face, its centre ``u_centre`` up the face, turned by ``twist``."""
    c = task.on_face(u_centre, y, out)
    th = math.radians(twist_deg)
    r = task.QM @ np.array([[math.cos(th), -math.sin(th), 0], [math.sin(th), math.cos(th), 0], [0, 0, 1]]) @ task.QM.T
    r = r @ task.QM @ task.QM.T                       # world rotation of the box frame: r (twist about N) x identity
    q = _quat_xyzw(r)
    return np.array([*c, *q])


def _quat_xyzw(r):
    w = math.sqrt(max(0.0, 1 + r[0, 0] + r[1, 1] + r[2, 2])) / 2
    x = (r[2, 1] - r[1, 2]) / (4 * w)
    y = (r[0, 2] - r[2, 0]) / (4 * w)
    z = (r[1, 0] - r[0, 1]) / (4 * w)
    return x, y, z, w


def test_scene_geometry():
    assert task.W / 2 - 0.012 - 0.01 > task.CARD_Y + task.CARD_WID / 2 - task.YC  # ticks clear of the notice's path
    assert 0.006 < 2 * abs(FINGER_OPEN)                                             # the pin head fits the open gripper
    top0 = task.CARD_U0 + task.CARD_LEN / 2
    assert abs(task.U_LINE - top0 - 0.025) < 0.001                                  # the line is 2.5 cm above the notice
    assert task.SEAT_BACK < task.TRAVEL_N                                            # the needle can seat
    assert task.PINNED_BACK < task.SEAT_BACK


def test_tether_is_stable_at_240hz():
    dt, m = 1 / 240, task.CARD.mass
    izz = m * (task.CARD_LEN ** 2 + task.CARD_WID ** 2) / 12
    r2 = (task.CARD_LEN / 2 - 0.021) ** 2 + (task.CARD_WID / 2 - 0.015) ** 2      # the hole's offset from the centre
    m_eff = 1 / (1 / m + r2 / izz)
    assert task.K_PIV * dt ** 2 / m_eff < 2.0                                       # explicit spring, well under 4
    assert task.C_PIV * dt / m_eff < 1.0
    assert m * 9.81 * math.sin(task.TH) / task.K_PIV < 0.001                        # sag under the notice's weight < 1 mm


def test_rule():
    assert task.rule(False, False, False, True, True) == 0.0
    assert task.rule(True, False, False, True, True) == 0.25
    assert task.rule(True, True, False, True, True) == 0.5
    assert task.rule(True, True, True, False, True) == 0.75
    assert task.rule(True, True, True, True, False) == 0.75
    assert task.rule(True, True, True, True, True) == 1.0


def test_card_findings():
    u_top, twist, on_board, still, top = task.card_findings(_card_pose(task.CARD_U0))
    assert abs(u_top - (task.CARD_U0 + task.CARD_LEN / 2)) < 1e-6 and abs(twist) < 1e-6 and on_board
    assert not still                                                          # no history yet
    u_top, twist, on_board, _, _ = task.card_findings(_card_pose(task.CARD_U0 + 0.025, twist_deg=4.0))
    assert abs(u_top - task.U_LINE) < 0.002 and abs(twist - 4.0) < 0.2 and on_board
    _, _, on_board, _, _ = task.card_findings(_card_pose(task.CARD_U0, out=-0.2))   # fallen behind the board
    assert not on_board
    pose = _card_pose(task.CARD_U0)
    *_, still, top = task.card_findings(pose, [(top, 0.0)] * 10)
    assert still
    *_, still, _ = task.card_findings(pose, [(top + task.D * 0.003, 0.0)] + [(top, 0.0)] * 9)   # moved 3 mm
    assert not still
    *_, still, _ = task.card_findings(pose, [(top, 1.5)] + [(top, 0.0)] * 9)                   # turned 1.5 deg
    assert not still


def test_collar_shift():
    a = task.N.copy()
    collar = task.FACE0 + task.N * 0.0008 + task.D * 0.1                # collar 0.8 mm off the face, square
    shift, d = task.collar_shift(a, collar, task.N, task.FACE0)
    assert abs(d - 0.0008) < 1e-9 and shift == 0.0
    collar = task.FACE0 - task.N * 0.0005 + task.D * 0.1                # 0.5 mm into the face: move it out
    shift, d = task.collar_shift(a, collar, task.N, task.FACE0)
    assert abs(shift - (0.0005 + task.CLEAR)) < 1e-9
    tilted = math.cos(math.radians(10)) * task.N + math.sin(math.radians(10)) * task.D
    shift, _ = task.collar_shift(tilted, collar, task.N, task.FACE0)
    assert shift > 0.0005 + 0.0055 * math.sin(math.radians(10))           # the low edge of the collar clears too


def test_demo_plans():
    for variant in ("full", "nothing", "pin_only", "no_slide", "keep_holding"):
        os.environ["DEMO_VARIANT"] = variant
        plan = task.demo()
        assert set(plan) == {"right", "left"}
        for arm in plan.values():
            for w in arm:
                assert all(np.isfinite(w.pos)) and w.duration > 0
    os.environ["DEMO_VARIANT"] = "full"
    right = task.demo()["right"]
    press = [w for w in right if w.label.startswith("press cmd")]
    assert len(press) == 8
    s = task.on_face(task.PIN_U, task.PIN_YW, task.CARD_T)
    assert abs(float((np.asarray(right[-1].pos) - s) @ task.N) - (0.05 + 0.0245)) < 1e-6   # retreat: 7.45 cm off
    assert task.PIN_U < task.CARD_U0 + task.CARD_LEN / 2 + task.SLIP_UP - 0.003          # pin lands inside the notice
    assert task.PIN_YW > task.CARD_Y - task.CARD_WID / 2 + 0.003
