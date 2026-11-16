"""Offline unit tests for qc_met8 scoring, shortcuts, demo reach, and top-camera visibility.

Run without simulator:
    python -m pytest -q test_qc_met8.py
"""

import inspect
import json
import re
from pathlib import Path

import numpy as np
import pytest

from labkit import PartState
from yam_arena import ik, station
from yam_arena.constants import CONTROL_HZ
from yam_arena.demo import ArmPlan
from yam_arena.task_file import load

PKG_DIR = Path(__file__).resolve().parent
TASK = load(PKG_DIR / "qc_met8.py")
M = TASK.module


def _fresh_log():
    M._clear(None)
    M._TEST_ROBOTS = None


def _open_clear_robots():
    p_l = np.array([M._LEFT_PARK[0], M._LEFT_PARK[1], M._LEFT_PARK[2]])
    p_r = np.array([M._RIGHT_PARK[0], M._RIGHT_PARK[1], M._RIGHT_PARK[2]])
    return [
        (np.vstack([p_l, p_l, p_l]), True),
        (np.vstack([p_r, p_r, p_r]), True),
    ]


def _close_robots():
    p = np.array([M.SHAFT_X, 0.025, M.TOP + M.DRIVE_LEVEL_Z])
    return [
        (np.vstack([p, p, p]), True),
        (np.vstack([p + [0.05, -0.05, 0.0], p, p]), False),
    ]


def _pitch_quat(pitch_deg: float, yaw_deg: float = 0.0, roll_deg: float = 0.0) -> np.ndarray:
    """Build quaternion (xyzw) with exact rot[2,1]=sin(pitch), rot[2,0]=-sin(roll)cos(pitch), rot[0,1]."""
    # Construct R = Rz(yaw_rad_neg) @ Rx(pitch) @ Ry(roll) matching drive_metrics rotation conventions
    # In qc_met8.drive_metrics: pitch_deg = asin(rot[2,1]), roll_deg = |asin(rot[2,0])|, yaw_deg = |atan2(rot[0,1], rot[1,1])|
    p = np.radians(pitch_deg)
    r = np.radians(roll_deg)
    y = np.radians(yaw_deg)
    rx = np.array([[1, 0, 0], [0, np.cos(p), -np.sin(p)], [0, np.sin(p), np.cos(p)]])
    ry = np.array([[np.cos(r), 0, np.sin(r)], [0, 1, 0], [-np.sin(r), 0, np.cos(r)]])
    rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
    # Rx(p) alone gives rot[2, 1] = +sin(p) (nose up for p > 0)
    rot = rz @ rx @ ry
    tr = rot[0, 0] + rot[1, 1] + rot[2, 2]
    if tr > 0:
        s = 2.0 * np.sqrt(tr + 1.0)
        w = 0.25 * s
        x = (rot[2, 1] - rot[1, 2]) / s
        y_q = (rot[0, 2] - rot[2, 0]) / s
        z = (rot[1, 0] - rot[0, 1]) / s
    elif rot[0, 0] > rot[1, 1] and rot[0, 0] > rot[2, 2]:
        s = 2.0 * np.sqrt(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2])
        w = (rot[2, 1] - rot[1, 2]) / s
        x = 0.25 * s
        y_q = (rot[0, 1] + rot[1, 0]) / s
        z = (rot[0, 2] + rot[2, 0]) / s
    elif rot[1, 1] > rot[2, 2]:
        s = 2.0 * np.sqrt(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2])
        w = (rot[0, 2] - rot[2, 0]) / s
        x = (rot[0, 1] + rot[1, 0]) / s
        y_q = 0.25 * s
        z = (rot[1, 2] + rot[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1])
        w = (rot[1, 0] - rot[0, 1]) / s
        x = (rot[0, 2] + rot[2, 0]) / s
        y_q = (rot[1, 2] + rot[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y_q, z, w], dtype=float)
    return q / np.linalg.norm(q)


def _drive_seated(
    pitch_deg: float = 0.0,
    yaw_deg: float = 0.0,
    roll_deg: float = 0.0,
    dx: float = 0.0,
    dy: float = 0.0,
    dz: float = 0.0,
    lin_vel: tuple[float, float, float] = (0.0, 0.0, 0.0),
    rear_lift: float | None = None,
) -> PartState:
    if rear_lift is None:
        rear_lift = max(M.LIFT_MIN, min(M.LIFT_MAX, M.WHEELBASE * np.sin(np.radians(pitch_deg))))
    q = _pitch_quat(pitch_deg, yaw_deg=yaw_deg, roll_deg=roll_deg)
    # Solve drive origin so saddle crown stays on SHAFT_X, CROWN_Z despite pitch/rear_lift
    p0 = np.array([M.SHAFT_X + dx, 0.025 + dy, M.TOP + M.DRIVE_LEVEL_Z + dz])
    probe = PartState(p0, q, np.zeros(3), np.zeros(3), p0 - 0.1, p0 + 0.1, {})
    sad_w = probe.to_world((0.0, M.SADDLE_Y, M.STYLUS_LOCAL_Z))
    shift = np.array([M.SHAFT_X + dx - sad_w[0], dy, M.TOP + M.CROWN_Z + dz - sad_w[2]])
    pos = p0 + shift
    turn = (rear_lift - M.E0) / M.K_RAD
    joints = {"rear_lift": rear_lift, "rear_turn": turn, "stylus_slide": 0.0}
    return PartState(
        pos, q, np.array(lin_vel, float), np.zeros(3),
        pos - [0.02, 0.09, 0.03], pos + [0.02, 0.09, 0.03], joints
    )


def _static_part(xy, z_top, half):
    c = np.array([xy[0], xy[1], z_top - half[2]])
    return PartState(c, np.array([0.0, 0.0, 0.0, 1.0]), np.zeros(3), np.zeros(3), c - half, c + half, {})


def _make_scene(drive: PartState | None = None, start_on_pad: bool = False):
    if drive is None:
        if start_on_pad:
            p = np.array([M.STAGING_XY[0], M.STAGING_XY[1], M._DRIVE_START_Z])
            drive = PartState(
                p, np.array([0.0, 0.0, 0.0, 1.0]), np.zeros(3), np.zeros(3),
                p - [0.02, 0.09, 0.03], p + [0.02, 0.09, 0.03],
                {"rear_lift": M.E0, "rear_turn": 0.0, "stylus_slide": 0.0},
            )
        else:
            drive = _drive_seated()
    return {
        "shaft": _static_part((M.SHAFT_X, M.SHAFT_YC), M.TOP + M.CROWN_Z, np.array([0.03, 0.08, 0.03])),
        "rear_anvil": _static_part(M.ANVIL_XY, M.TOP + M.ANVIL_H, np.array([0.03, 0.025, 0.021])),
        "staging_pad": _static_part(M.STAGING_XY, M.TOP + M.STAGING_BASE_H, np.array([0.025, 0.085, 0.005])),
        "console": _static_part(M.CONSOLE_XY, M.TOP + M.CONSOLE_SIZE[2], np.array([0.08, 0.033, 0.026])),
        "drive_unit": drive,
    }


def test_task_loads_with_expected_objects_and_hooks():
    assert TASK.scene == "station" and TASK.name == "qc_met8"
    assert [o.name for o in TASK.objects] == ["shaft", "rear_anvil", "staging_pad", "console", "drive_unit",
                                              "prop_caliper", "prop_glove_box", "prop_safety_glasses", "prop_solvent_spray"]
    assert TASK.free_objects == ("drive_unit",)
    assert all(callable(f) for f in (TASK.score, TASK.success, TASK.demo))
    assert TASK.score_over == "end" and TASK.success_hold_s == pytest.approx(1.0)


def test_start_state_and_initial_retreat_pitch():
    _fresh_log()
    M._TEST_ROBOTS = _open_clear_robots()
    start_objs = _make_scene(start_on_pad=True)
    assert TASK.score(start_objs) == 0.0 and not TASK.success(start_objs)
    start_pitch = np.degrees(np.arcsin(M.E0 / M.WHEELBASE))
    assert 3.55 <= start_pitch <= 3.62  # +3.58 deg nose-up when placed with rear foot retracted 7.5 mm
    seated_initial = _drive_seated(pitch_deg=start_pitch, rear_lift=M.E0)
    m = M.drive_metrics(seated_initial)
    assert m["seated"] and m["pitch_deg"] > M.PITCH_PARTIAL_DEG


def test_all_documented_score_tiers():
    # Documented tiers in task.json: 0, 0.25, 0.35, 0.4, 0.5, 0.55, 0.7, 0.85, 1.0
    # 0.25: seated on crown + anvil, out-of-range pitch (+4.5 deg), grippers holding
    _fresh_log()
    M._TEST_ROBOTS = _close_robots()
    assert TASK.score(_make_scene(_drive_seated(pitch_deg=4.5))) == 0.25

    # 0.35: seated + partial pitch (2.0 deg), grippers holding
    _fresh_log()
    M._TEST_ROBOTS = _close_robots()
    assert TASK.score(_make_scene(_drive_seated(pitch_deg=2.0))) == 0.35

    # 0.40: seated, out-of-range pitch (+4.5 deg), grippers open & clear
    _fresh_log()
    M._TEST_ROBOTS = _open_clear_robots()
    assert TASK.score(_make_scene(_drive_seated(pitch_deg=4.5))) == 0.40

    # 0.50: seated + partial pitch (2.0 deg), grippers open & clear
    _fresh_log()
    M._TEST_ROBOTS = _open_clear_robots()
    assert TASK.score(_make_scene(_drive_seated(pitch_deg=2.0))) == 0.50

    # 0.55: seated + leveled (0.0 deg), no traverse, grippers close
    _fresh_log()
    M._TEST_ROBOTS = _close_robots()
    assert TASK.score(_make_scene(_drive_seated(pitch_deg=0.0))) == 0.55

    # 0.70: seated + leveled (0.0 deg), no traverse, grippers open & clear
    _fresh_log()
    M._TEST_ROBOTS = _open_clear_robots()
    assert TASK.score(_make_scene(_drive_seated(pitch_deg=0.0))) == 0.70

    # 0.85: seated + leveled + traverse_done, grippers close
    _fresh_log()
    M._log["traverse_done"] = True
    M._TEST_ROBOTS = _close_robots()
    assert TASK.score(_make_scene(_drive_seated(pitch_deg=0.0))) == 0.85

    # 1.00: full solved state
    _fresh_log()
    M._log["traverse_done"] = True
    M._TEST_ROBOTS = _open_clear_robots()
    solved = _make_scene(_drive_seated(pitch_deg=0.0))
    assert TASK.score(solved) == 1.0 and TASK.success(solved)


def test_shortcuts_and_violations_fail():
    # Knocked / tipped / shifted / skewed / non-resting -> 0.0
    cases = [
        _drive_seated(pitch_deg=14.0),               # tipped beyond TIP_DEG
        _drive_seated(dx=0.003),                     # lateral off crown > 1.5 mm
        _drive_seated(yaw_deg=4.0),                  # yaw skew > 2.5 deg
        _drive_seated(roll_deg=7.0),                 # roll skew > 5.0 deg
        _drive_seated(roll_deg=180.0),               # upside down
        _drive_seated(dy=-0.025),                    # rear foot off anvil
        _drive_seated(dz=0.008),                     # lifted off shaft
        _drive_seated(lin_vel=(0.25, 0.0, 0.0)),     # moving / not resting
    ]
    for drv in cases:
        _fresh_log()
        M._log["traverse_done"] = True
        M._TEST_ROBOTS = _open_clear_robots()
        objs = _make_scene(drv)
        assert TASK.score(objs) == 0.0 and not TASK.success(objs)

    # Undo / post_disturbed -> 0.0
    _fresh_log()
    M._log["traverse_done"] = True
    M._log["post_disturbed"] = True
    M._TEST_ROBOTS = _open_clear_robots()
    objs = _make_scene(_drive_seated())
    assert TASK.score(objs) == 0.0 and not TASK.success(objs)

    # Vibration fault voids traverse credit (+0.30 withheld)
    _fresh_log()
    M._log["traverse_done"] = True
    M._log["vibration_fault"] = True
    M._TEST_ROBOTS = _open_clear_robots()
    assert TASK.score(_make_scene(_drive_seated())) == 0.70
    assert not TASK.success(_make_scene(_drive_seated()))


def test_demo_is_pure_open_loop_from_fixed_layout():
    src = inspect.getsource(M.demo)
    for banned in ("_part_state", "labkit.env", "_log", "score(", "success(", "_TEST_ROBOTS", "objects"):
        assert banned not in src, banned
    assert M.demo() == M.demo()


@pytest.mark.parametrize("arm", ["left", "right"])
def test_demo_ik_limits_and_run_limit(arm):
    plan = ArmPlan(TASK.demo()[arm], station.BASES[arm])
    assert plan.worst_ik_mm < 0.5
    assert plan.worst_ik_deg < 0.1
    steps = round(plan.duration * CONTROL_HZ) + 5
    actions = np.array([plan.action7((k + 1) / CONTROL_HZ)[0] for k in range(steps)])
    assert np.all(actions[:, :6] >= ik.LOWER) and np.all(actions[:, :6] <= ik.UPPER)

    cfg = json.loads((PKG_DIR / "task.json").read_text())
    m = re.search(r"([\d,]+)", str(cfg["run_limit"]))
    limit = int(m.group(1).replace(",", "")) if m else 2400
    assert plan.duration * CONTROL_HZ < limit
    assert plan.duration * CONTROL_HZ < 2400


def _project(cam_lens, cam_pos, cam_rot, p_world):
    p = cam_rot.T @ (np.asarray(p_world, float) - np.asarray(cam_pos, float))
    return (cam_lens.cx + cam_lens.fx * p[0] / (-p[2]), cam_lens.cy - cam_lens.fy * p[1] / (-p[2]))


def test_scored_regions_visible_and_unoccluded_at_demo_end():
    from yam_arena.urdf import quat_xyzw_to_matrix

    cam = station.top_camera("calibrated")
    lens = cam.lens.resized(1280, 704)
    rot = quat_xyzw_to_matrix(np.array(cam.rot_xyzw))
    z = M.TOP
    regions = {
        "shaft": ((M.SHAFT_X, M.SHAFT_YC, z + M.SHAFT_AXIS_Z), 0.042),
        "drive_unit": ((M.SHAFT_X, 0.025, z + M.DRIVE_LEVEL_Z), M.BODY_L),
        "rear_thumbscrew": ((M.SHAFT_X, -0.045, z + M.DRIVE_LEVEL_Z + M.THUMBSCREW_LOCAL_Z), 2 * M.THUMBSCREW_R),
        "rear_anvil": ((M.ANVIL_XY[0], M.ANVIL_XY[1], z + M.ANVIL_H), 0.056),
        "console": ((M.CONSOLE_XY[0], M.CONSOLE_XY[1], z + M.CONSOLE_SIZE[2] / 2), M.CONSOLE_SIZE[0]),
        "lcd": ((M.CONSOLE_XY[0] + 0.026, M.CONSOLE_XY[1], z + M.CONSOLE_SIZE[2] + 0.001), 0.056),
        "start_key": ((M.START_WORLD_XY[0], M.START_WORLD_XY[1], z + M.CONSOLE_SIZE[2] + 0.005), np.hypot(0.024, 0.020)),
    }
    final_tcp = {arm: ArmPlan(TASK.demo()[arm], station.BASES[arm]).target(1e9)[0] for arm in ("left", "right")}
    for name, (pt, dia) in regions.items():
        u, v = _project(lens, cam.pos, rot, pt)
        assert 0.0 <= u < lens.width and 0.0 <= v < lens.height, (name, u, v)
        px = dia * lens.fx / float(np.linalg.norm(np.asarray(pt) - np.asarray(cam.pos)))
        assert px >= 20.0, (name, px)
        for arm, tcp in final_tcp.items():
            tu, tv = _project(lens, cam.pos, rot, tcp)
            reach_px = 0.06 * lens.fx / float(np.linalg.norm(tcp - np.asarray(cam.pos)))
            assert np.hypot(u - tu, v - tv) > reach_px, (name, arm)


def test_robot_observations_gpu_tensor_safe(monkeypatch):
    """Ensure _robot_observations converts tensors via .detach().cpu().numpy() (never direct np.asarray)."""
    import labkit

    class _FakeCudaTensor:
        def __init__(self, arr):
            self._arr = np.asarray(arr, dtype=np.float64)

        def __getitem__(self, idx):
            return _FakeCudaTensor(self._arr[idx])

        def __array__(self, dtype=None):
            raise TypeError("can't convert cuda:0 device type tensor to numpy. Use Tensor.cpu() first.")

        def detach(self):
            return self

        def cpu(self):
            return self

        def numpy(self):
            return self._arr

    class _FakeData:
        def __init__(self, body_pos, joint_pos):
            self.body_pos_w = _FakeCudaTensor(body_pos)
            self.joint_pos = _FakeCudaTensor(joint_pos)

    class _FakeArt:
        body_names = ["base", "gripper", "tip_left", "tip_right"]
        joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7", "joint8"]

        def __init__(self, y_off):
            bp = np.array([[[0.0, y_off, 0.75], [0.12, y_off, 0.88], [0.12, y_off + 0.02, 0.75], [0.12, y_off - 0.02, 0.75]]])
            jq = np.array([[0.0, 1.0, 1.0, -0.6, 0.0, 0.0, M.FINGER_OPEN, M.FINGER_OPEN]])
            self.data = _FakeData(bp, jq)

    class _FakeScene:
        env_origins = _FakeCudaTensor([[0.0, 0.0, 0.0]])
        _arts = {"left_robot": _FakeArt(0.26), "right_robot": _FakeArt(-0.27)}

        def __getitem__(self, key):
            return self._arts[key]

    class _FakeEnv:
        class _Unwrapped:
            scene = _FakeScene()

        unwrapped = _Unwrapped()

    _fresh_log()
    monkeypatch.setattr(labkit, "env", lambda: _FakeEnv())
    obs = M._robot_observations()
    assert obs is not None and len(obs) == 2
    assert all(f_open for _, f_open in obs)

