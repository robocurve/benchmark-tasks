"""Unit tests for restock-hinged-pastry-bin-tray (geometry, scorer, slam/spill hazards, and 20 Hz ArmPlan IK)."""

from __future__ import annotations

import importlib
import math
import os
import sys
import types
from pathlib import Path

import numpy as np

TASK_DIR = Path(__file__).resolve().parent


def _install_stubs():
    """Install lightweight pure-Python stubs when yam_arena is absent (local Windows unit run)."""
    if "yam_arena" in sys.modules:
        return
    ya = types.ModuleType("yam_arena")
    yc = types.ModuleType("yam_arena.constants")
    yd = types.ModuleType("yam_arena.demo")
    yt = types.ModuleType("yam_arena.task_file")

    yc.FINGER_OPEN = -0.04695
    yc.FINGER_CLOSED = 0.0
    yc.PAD_TIP_PAST_TCP = 0.013
    yc.TCP_OFFSET = (0.0, 0.0, 0.134)

    class Waypoint:
        def __init__(self, duration, pos, finger, label="", yaw=0.0):
            self.duration = duration
            self.pos = tuple(pos)
            self.finger = finger
            self.label = label
            self.yaw = yaw

    class _StubArmPlan:
        pass

    yd.Waypoint = Waypoint
    yd.ArmPlan = _StubArmPlan
    yd._IS_STUB = True
    yd.grip_rotation = staticmethod(lambda yaw=0.0: np.eye(3))
    yd.ik = types.SimpleNamespace(smoothstep=lambda u: u, slerp_matrix=lambda a, b, u: a)

    from dataclasses import dataclass

    @dataclass(frozen=True)
    class Box:
        name: str = "box"
        pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
        color: tuple[float, float, float] = (0.5, 0.5, 0.5)
        mass: float | None = None
        yaw: float = 0.0
        size: tuple[float, float, float] = (0.04, 0.04, 0.04)

        def _check(self):
            pass

        @property
        def free(self):
            return self.mass is not None

    @dataclass(frozen=True)
    class ObjectState:
        pos: np.ndarray
        quat_xyzw: np.ndarray
        lin_vel: np.ndarray
        ang_vel: np.ndarray
        low: np.ndarray
        high: np.ndarray

        def resting(self, max_speed=0.01, max_spin=0.5):
            return bool(np.linalg.norm(self.lin_vel) < max_speed and np.linalg.norm(self.ang_vel) < max_spin)

    yt.TABLE_TOP = {"station": 0.75, "tabletop": 0.75}
    yt.Box = Box
    yt.ObjectState = ObjectState

    sys.modules["yam_arena"] = ya
    sys.modules["yam_arena.constants"] = yc
    sys.modules["yam_arena.demo"] = yd
    sys.modules["yam_arena.task_file"] = yt


try:
    import yam_arena.demo  # noqa: F401
except ImportError:
    _install_stubs()
if str(TASK_DIR) not in sys.path:
    sys.path.insert(0, str(TASK_DIR))

import binkit as bk  # noqa: E402
from yam_arena.task_file import ObjectState  # noqa: E402


def _load_task(side: str = "L", env_overrides: dict | None = None):
    old = {k: os.environ.get(k) for k in ("BIN_SIDE", "DEMO_NO_HOLD", "DEMO_DROP_DOOR")}
    os.environ["BIN_SIDE"] = side
    os.environ.pop("DEMO_NO_HOLD", None)
    os.environ.pop("DEMO_DROP_DOOR", None)
    if env_overrides:
        os.environ.update(env_overrides)
    try:
        sys.modules.pop("task", None)
        import task  # noqa: E402

        return importlib.reload(task)
    finally:
        for k, v in old.items():
            if v is None:
                os.environ.pop(k, None)
            else:
                os.environ[k] = v


def _state(pos, tilt_deg=0.0, vel=(0, 0, 0), ang_vel=(0, 0, 0), low_z=None, size=(0.098, 0.072, 0.024)):
    pos = np.asarray(pos, float)
    rad = math.radians(tilt_deg)
    q = np.array([math.sin(rad / 2), 0.0, 0.0, math.cos(rad / 2)], float)
    half = np.asarray(size, float) / 2
    lz = pos[2] if low_z is None else float(low_z)
    low = np.array([pos[0] - half[0], pos[1] - half[1], lz], float)
    high = np.array([pos[0] + half[0], pos[1] + half[1], lz + size[2]], float)
    return ObjectState(pos, q, np.asarray(vel, float), np.asarray(ang_vel, float), low, high)


def _bin_state(t, angle_deg: float = 0.0, vel_rad_s: float = 0.0):
    ang = math.radians(angle_deg)
    dpos, dquat = bk.compute_door_frame(t.BIN_POS, (0.0, 0.0, 0.0, 1.0), t.HINGE_LOCAL, ang)
    lo, hi = bk.closed_bounds(t.PASTRY_BIN)
    return bk.BinState(
        pos=np.asarray(t.BIN_POS, float),
        quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
        lin_vel=np.zeros(3),
        ang_vel=np.zeros(3),
        low=lo,
        high=hi,
        angle=ang,
        door_vel=vel_rad_s,
        door_pos=dpos,
        door_quat_xyzw=dquat,
    )


def _grippers(t, pos_l=(0.0, 0.30, 0.96), pos_r=(0.0, -0.30, 0.96)):
    return {
        "left_gripper": _state(pos_l, size=(0.04, 0.04, 0.04)),
        "right_gripper": _state(pos_r, size=(0.04, 0.04, 0.04)),
    }


def _fresh_tracker(t):
    t.HAZARD.run, t.HAZARD.on, t.HAZARD._key, t.HAZARD._objs = 0, False, None, None
    t._T = t._Tracker()


def test_geometry_clearances():
    t = _load_task("L")
    # Tray clears bin sidewalls and hinged lid clears tray headroom while staying < 90 deg at stop
    assert t.TRAY_Y + 0.030 <= t.BIN_IN_W
    assert t.LID_W <= t.BIN_OUT_W
    assert t.LID_LEN < t.BIN_D
    assert t.HINGE_WORLD[2] > t.Z_BIN_FLOOR + t.TRAY_RIM_H + 0.02
    assert t.LID_UPPER_DEG < 90.0
    # Stocked tray and tray_mat have zero XY bounding overlap
    dx = abs(t.STOCK_START[0] - t.MAT_POS[0])
    dy = abs(t.STOCK_START[1] - t.MAT_POS[1])
    assert dx > 0.10 or dy > (t.TRAY_Y + t.MAT_SIZE[1]) / 2.0
    # At max open angle, front handle lifts well above the bin floor for top-down access
    open_handle = t._handle_world(math.radians(t.LID_UPPER_DEG), 0.0)
    assert open_handle[2] - (t.Z_BIN_FLOOR + t.TRAY_RIM_H) > 0.10


def test_score_tiers_and_slam_hazard():
    t = _load_task("L")
    _fresh_tracker(t)

    # Start -> 0.0
    s0 = {
        "pastry_bin": _bin_state(t, angle_deg=0.0, vel_rad_s=0.0),
        "empty_tray": _state((t.TRAY_BIN_X, 0.0, t.Z_BIN_FLOOR), low_z=t.Z_BIN_FLOOR),
        "stocked_tray": _state(t.STOCK_START, low_z=t.TOP),
        **_grippers(t),
    }
    assert t.score(s0) == 0.0

    # Extracted empty_tray out of bin -> 0.25
    s1 = {
        "pastry_bin": _bin_state(t, angle_deg=70.0, vel_rad_s=0.0),
        "empty_tray": _state((0.15, -0.05, t.TOP), low_z=t.TOP),
        "stocked_tray": _state(t.STOCK_START, low_z=t.TOP),
        **_grippers(t),
    }
    assert t.score(s1) == 0.25

    # Placed empty_tray flat on tray_mat -> 0.50
    s2 = {
        "pastry_bin": _bin_state(t, angle_deg=70.0, vel_rad_s=0.0),
        "empty_tray": _state((t.MAT_POS[0], t.MAT_POS[1], t.MAT_TOP_Z), low_z=t.MAT_TOP_Z),
        "stocked_tray": _state(t.STOCK_START, low_z=t.TOP),
        **_grippers(t),
    }
    assert t.score(s2) == 0.50

    # Both trays swapped, lid still open -> 0.75
    s3 = {
        "pastry_bin": _bin_state(t, angle_deg=65.0, vel_rad_s=-0.6),
        "empty_tray": _state((t.MAT_POS[0], t.MAT_POS[1], t.MAT_TOP_Z), low_z=t.MAT_TOP_Z),
        "stocked_tray": _state((t.TRAY_BIN_X, 0.0, t.Z_BIN_FLOOR), low_z=t.Z_BIN_FLOOR),
        **_grippers(t),
    }
    assert t.score(s3) == 0.75

    # Touching handle still blocks 1.0 even with lid near closed
    handle_w = t._handle_world(math.radians(2.0), t.HANDLE_GRIP_Y)
    for _ in range(t.HOLD_STEPS + 2):
        st_touch = {
            "pastry_bin": _bin_state(t, angle_deg=2.0, vel_rad_s=0.0),
            "empty_tray": _state((t.MAT_POS[0], t.MAT_POS[1], t.MAT_TOP_Z), low_z=t.MAT_TOP_Z),
            "stocked_tray": _state((t.TRAY_BIN_X, 0.0, t.Z_BIN_FLOOR), low_z=t.Z_BIN_FLOOR),
            **_grippers(t, pos_l=handle_w),
        }
        assert t.score(st_touch) < 1.0

    # Clear for HOLD_STEPS -> 1.0
    for _ in range(t.HOLD_STEPS):
        st_ok = {
            "pastry_bin": _bin_state(t, angle_deg=1.5, vel_rad_s=0.0),
            "empty_tray": _state((t.MAT_POS[0], t.MAT_POS[1], t.MAT_TOP_Z), low_z=t.MAT_TOP_Z),
            "stocked_tray": _state((t.TRAY_BIN_X, 0.0, t.Z_BIN_FLOOR), low_z=t.Z_BIN_FLOOR),
            **_grippers(t),
        }
        sc = t.score(st_ok)
    assert sc == 1.0

    # Lid slam hazard permanently zeroes score even if trays finish properly
    _fresh_tracker(t)
    slam = {
        "pastry_bin": _bin_state(t, angle_deg=12.0, vel_rad_s=-2.9),
        "empty_tray": _state((t.MAT_POS[0], t.MAT_POS[1], t.MAT_TOP_Z), low_z=t.MAT_TOP_Z),
        "stocked_tray": _state((t.TRAY_BIN_X, 0.0, t.Z_BIN_FLOOR), low_z=t.Z_BIN_FLOOR),
        **_grippers(t),
    }
    assert t.score(slam) == 0.0 and t.HAZARD.on
    assert t.score(st_ok) == 0.0


def test_symmetric_demo_and_ik():
    for side in ("L", "R"):
        t = _load_task(side)
        plan = t.demo()
        assert set(plan) == {"left", "right"}
        hold = "left" if side == "L" else "right"
        work = "right" if side == "L" else "left"
        assert any(w.label == "swing_open" for w in plan[hold])
        assert any(w.label == "lower_gentle" for w in plan[hold])
        assert any(w.label == "down_bin" for w in plan[work])

        from yam_arena import demo as ydemo  # noqa: E402

        if not getattr(ydemo, "_IS_STUB", False):
            from yam_arena import station  # noqa: E402

            for arm in ("left", "right"):
                ap = ydemo.ArmPlan(list(plan[arm]), station.BASES[arm])
                for step_t in np.linspace(0.0, ap.duration, int(round(ap.duration * 20)) + 1):
                    ap.action7(step_t)
                assert ap.worst_ik_mm < 0.5, f"{side} {arm} worst_ik_mm={ap.worst_ik_mm:.3f}"
                assert ap.worst_ik_deg < 0.5, f"{side} {arm} worst_ik_deg={ap.worst_ik_deg:.3f}"


def test_negative_control_flags():
    t_nh = _load_task("L")
    os.environ["DEMO_NO_HOLD"] = "1"
    try:
        p_nh = t_nh.demo()
        assert any(w.label == "drop_early" for w in p_nh["left"])
    finally:
        os.environ.pop("DEMO_NO_HOLD", None)

    t_dd = _load_task("R")
    os.environ["DEMO_DROP_DOOR"] = "1"
    try:
        p_dd = t_dd.demo()
        assert any(w.label == "release_high" for w in p_dd["right"])
    finally:
        os.environ.pop("DEMO_DROP_DOOR", None)


if __name__ == "__main__":
    test_geometry_clearances()
    test_score_tiers_and_slam_hazard()
    test_symmetric_demo_and_ik()
    test_negative_control_flags()
    print("All restock-hinged-pastry-bin-tray unit tests passed!")
