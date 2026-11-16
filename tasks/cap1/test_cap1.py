"""Unit tests for cap1 scoring, shortcut resistance, kinematics, and camera unocclusion."""

from __future__ import annotations

import json
import math
import pathlib
import re

import numpy as np
import pytest

import cap1
from yam_arena import YAM_URDF, demo as yam_demo, ik, station
from yam_arena.constants import ARM_JOINTS, CONTROL_HZ
from yam_arena.task_file import ObjectState
from yam_arena.urdf import forward_kinematics, quat_xyzw_to_matrix

PKG_DIR = pathlib.Path(__file__).resolve().parent


def _state(pos, quat_xyzw=(0.0, 0.0, 0.0, 1.0), lin=(0.0, 0.0, 0.0), ang=(0.0, 0.0, 0.0)) -> ObjectState:
    p = np.asarray(pos, dtype=float)
    return ObjectState(
        pos=p,
        quat_xyzw=np.asarray(quat_xyzw, dtype=float),
        lin_vel=np.asarray(lin, dtype=float),
        ang_vel=np.asarray(ang, dtype=float),
        low=p - 0.01,
        high=p + 0.01,
    )


def _initial_objects() -> dict[str, ObjectState]:
    objs = {
        "rack": _state((cap1.RACK_XY[0], cap1.RACK_XY[1], cap1.TOP)),
        "pad": _state((cap1.CAPS_XY[0], cap1.CAPS_XY[1], cap1.TOP)),
    }
    for i, y in enumerate(cap1.TUBE_YS):
        objs[f"tube_{i}"] = _state((cap1.RACK_XY[0], y, cap1.TOP))
    for i, p in enumerate(cap1.CAP_SPAWNS):
        objs[f"cap_{i}"] = _state(p)
    return objs


def _cap_at_engagement(tube_idx: int, engage_m: float, dx: float = 0.0, dy: float = 0.0) -> ObjectState:
    z = cap1.TOP + cap1.TUBE_H + cap1.REST_GAP - cap1.CAP_CAVITY_H - engage_m
    return _state((cap1.RACK_XY[0] + dx, cap1.TUBE_YS[tube_idx] + dy, z))


def test_start_score_zero():
    objs = _initial_objects()
    assert cap1.score(objs) == 0.0
    assert cap1.success(objs) is False


@pytest.mark.parametrize("n_vented,expected", [(0, 0.0), (1, 0.33), (2, 0.67), (3, 1.0)])
def test_partial_credit_and_full_success(n_vented, expected):
    objs = _initial_objects()
    for i in range(n_vented):
        objs[f"cap_{i}"] = _cap_at_engagement(i, 0.0050)
    assert cap1.score(objs) == expected
    assert cap1.success(objs) is (n_vented == 3)


def test_shortcut_resting_unengaged_scores_zero():
    objs = _initial_objects()
    for i in range(3):
        objs[f"cap_{i}"] = _cap_at_engagement(i, 0.0020)  # < 3 mm resting loosely on rim
    assert cap1.score(objs) == 0.0
    assert cap1.success(objs) is False


def test_shortcut_sealed_overpressed_scores_zero():
    objs = _initial_objects()
    for i in range(3):
        objs[f"cap_{i}"] = _cap_at_engagement(i, 0.0090)  # > 8 mm pushed to 2nd stop
    assert cap1.score(objs) == 0.0
    assert cap1.success(objs) is False


def _lip_open_end_height_on_roller(peak_r: float) -> float:
    """Independent check of the detent geometry: lower the cap's lip (chamfer face from LIP_R_OUT at the open end up
    to LIP_R_IN, then a vertical inner face) onto one roller (circle of BALL_R, crest at peak_r) and return the height
    of the cap's open end above the roller axis at first touch. Brute force over points on the lip profile."""
    rc = peak_r - cap1.BALL_R
    best = -math.inf
    for k in range(4001):
        u = k / 4000.0
        r = cap1.LIP_R_OUT + u * (cap1.LIP_R_IN - cap1.LIP_R_OUT)  # along the chamfer face
        h = u * cap1.LIP_H  # height of that point above the cap's open end
        dr = r - rc
        if abs(dr) <= cap1.BALL_R:
            best = max(best, math.sqrt(cap1.BALL_R**2 - dr * dr) - h)  # roller surface height minus point height
    return best


def test_detent_rows_put_rest_and_first_stop_where_the_scorer_expects():
    rest_bottom = cap1.TUBE_H + cap1.REST_GAP - cap1.CAP_CAVITY_H  # cap open end at 0 mm engagement (tube frame)
    on_row1 = cap1.ROW1_Z + _lip_open_end_height_on_roller(cap1.ROW1_PEAK_R)
    on_row2 = cap1.ROW2_Z + _lip_open_end_height_on_roller(cap1.ROW2_PEAK_R)
    assert abs(on_row1 - rest_bottom) < 5e-5  # resting on the upper row = 0 mm engagement
    first_stop = rest_bottom - on_row2
    assert abs(first_stop - 0.005) < 5e-5 and cap1.ENGAGE_MIN_M < first_stop < cap1.ENGAGE_MAX_M
    sealed = cap1.REST_GAP  # the cavity ceiling lands on the rim
    assert sealed >= cap1.SEALED_MIN_M and sealed > cap1.ENGAGE_MAX_M


def test_detent_is_springs_and_contact_only():
    """Every roller stands proud of the lip, fits inside the skirt, can retract clear of the lip, and is held only
    by its spring (guide friction is a small fraction of it); no on_step hook of the task changes the scene."""
    for peak in (cap1.ROW1_PEAK_R, cap1.ROW2_PEAK_R):
        assert cap1.LIP_R_IN < peak < cap1.CAP_SKIRT_R_IN
        assert peak - cap1.BALL_TRAVEL < cap1.LIP_R_IN
    assert cap1.LIP_R_IN > cap1.TUBE_R and cap1.LIP_R_OUT <= cap1.CAP_SKIRT_R_IN
    assert cap1.ROW2_K > cap1.ROW1_K and cap1.ROW2_PEAK_R > cap1.ROW1_PEAK_R  # the second stop needs the harder press
    tube = next(o for o in cap1.OBJECTS if o.name == "tube_0")
    assert len(tube.joints) == 8
    for j in tube.joints:
        assert j.kind == "slide" and j.axis in ("X", "Y") and j.stiffness > 0
        assert j.friction < 0.2 * j.stiffness * 0.0005  # guide friction well below the spring force at the lip: no latch
        assert j.target == 0.0 and j.initial == 0.0 and j.upper - j.lower == pytest.approx(cap1.BALL_TRAVEL)
    assert not hasattr(cap1, "_step_detents")
    assert [fn.__name__ for fn in cap1.labkit._STEP if fn.__module__ == cap1.__name__] == ["_watch_subactions"]
    src = (PKG_DIR / "cap1.py").read_text()
    assert "set_joint_position_target" not in src and "write_joint" not in src and "write_root" not in src


def test_shortcut_intermediate_gap_scores_zero():
    objs = _initial_objects()
    for i in range(3):
        objs[f"cap_{i}"] = _cap_at_engagement(i, 0.0070)  # outside 4-6 mm vented stop
    assert cap1.score(objs) == 0.0
    assert cap1.success(objs) is False


def test_shortcut_tipped_cap_and_tipped_rack_score_zero():
    objs = _initial_objects()
    for i in range(3):
        objs[f"cap_{i}"] = _cap_at_engagement(i, 0.0050)
    # 90-degree tipped cap_0
    objs["cap_0"] = _state(objs["cap_0"].pos, quat_xyzw=(math.sin(math.pi / 4), 0.0, 0.0, math.cos(math.pi / 4)))
    assert cap1.score(objs) == 0.67
    assert cap1.success(objs) is False

    # Tipped rack fails everything -> 0.0
    objs["cap_0"] = _cap_at_engagement(0, 0.0050)
    objs["rack"] = _state(objs["rack"].pos, quat_xyzw=(math.sin(math.pi / 4), 0.0, 0.0, math.cos(math.pi / 4)))
    assert cap1.score(objs) == 0.0
    assert cap1.success(objs) is False


def test_shortcut_off_center_and_moving_caps():
    objs = _initial_objects()
    for i in range(3):
        objs[f"cap_{i}"] = _cap_at_engagement(i, 0.0050)
    # Off-center cap_2 cannot be seated
    objs["cap_2"] = _cap_at_engagement(2, 0.0050, dx=0.010)
    assert cap1.score(objs) == 0.67
    assert cap1.success(objs) is False

    # Undo after success: pulling cap_1 off drops score back to 0.67
    objs["cap_2"] = _cap_at_engagement(2, 0.0050)
    assert cap1.success(objs) is True
    objs["cap_1"] = _state((0.220, 0.0, cap1.TOP))
    assert cap1.score(objs) == 0.67
    assert cap1.success(objs) is False

    # Unsettled cap velocity prevents premature success
    objs["cap_1"] = _state(_cap_at_engagement(1, 0.0050).pos, lin=(0.0, 0.0, 0.08))
    assert cap1.success(objs) is False
    assert cap1.score(objs) == 0.67


def _three_vented() -> dict[str, ObjectState]:
    objs = _initial_objects()
    for i in range(3):
        objs[f"cap_{i}"] = _cap_at_engagement(i, 0.0050)
    return objs


@pytest.mark.parametrize(
    "lin,ang,expected",
    [
        ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 1.0),
        ((0.029, 0.0, 0.0), (0.0, 0.0, 0.29), 1.0),  # just under 3 cm/s and 0.3 rad/s: still
        ((0.0, 0.03, 0.0), (0.0, 0.0, 0.0), 0.67),  # in band, moving at exactly 3 cm/s
        ((0.0, 0.0, 0.0), (0.0, 0.30, 0.0), 0.67),  # turning at exactly 0.3 rad/s
        ((0.0, 0.0, -0.2), (0.0, 0.0, 0.0), 0.67),
        ((0.0, 0.0, 0.0), (0.31, 0.0, 0.0), 0.67),  # in band but turning
    ],
)
def test_full_score_needs_all_three_caps_still(lin, ang, expected):
    for moving in range(3):
        objs = _three_vented()
        objs[f"cap_{moving}"] = _state(objs[f"cap_{moving}"].pos, lin=lin, ang=ang)
        assert cap1.score(objs) == expected
        assert cap1.success(objs) is (expected == 1.0)


@pytest.mark.parametrize("tilt_deg,cap_counts", [(0.0, True), (19.9, True), (19.99, True), (20.1, False), (30.0, False)])
def test_tilt_threshold_is_20_degrees(tilt_deg, cap_counts):
    half = math.radians(tilt_deg) / 2.0
    tilt = (math.sin(half), 0.0, 0.0, math.cos(half))
    objs = _three_vented()
    objs["cap_0"] = _state(objs["cap_0"].pos, quat_xyzw=tilt)
    assert cap1.score(objs) == (1.0 if cap_counts else 0.67)
    objs = _three_vented()
    objs["tube_1"] = _state(objs["tube_1"].pos, quat_xyzw=tilt)
    assert cap1.score(objs) == (1.0 if cap_counts else 0.0)
    objs = _three_vented()
    objs["rack"] = _state(objs["rack"].pos, quat_xyzw=tilt)
    assert cap1.score(objs) == (1.0 if cap_counts else 0.0)


def test_demo_ik_limits_and_duration():
    meta = json.loads((PKG_DIR / "task.json").read_text())
    m = re.search(r"([\d,]+)", str(meta["run_limit"]))
    max_steps = int(m.group(1).replace(",", ""))
    plans = cap1.demo()
    assert set(plans) == {"left", "right"}
    for arm in ("left", "right"):
        ap = yam_demo.ArmPlan(plans[arm], station.BASES[arm])
        assert ap.worst_ik_mm < 0.5, f"{arm} worst_ik_mm={ap.worst_ik_mm}"
        assert ap.worst_ik_deg < 0.1, f"{arm} worst_ik_deg={ap.worst_ik_deg}"
        assert ap.duration * 20.0 < max_steps
        n = int(math.ceil(ap.duration * CONTROL_HZ)) + 1
        for t in np.linspace(0.0, ap.duration, n):
            q = np.asarray(ap.action7(float(t))[0], dtype=float)[:6]
            assert np.all(q >= ik.LOWER - 1e-4) and np.all(q <= ik.UPPER + 1e-4)


def _project_top_cam(pt_w: np.ndarray) -> tuple[float, float, float]:
    """Project a 3D world point onto station top_cam (scaled cleanly to 640x480 base intrinsics)."""
    fx, fy, cx, cy = station.TOP_CAM_K
    w, x, y, z = station.TOP_CAM_WXYZ
    # TOP_CAM_WXYZ converts camera-local frame to world; world->cam is R^T.
    r_wc = quat_xyzw_to_matrix(np.array([x, y, z, w], dtype=float))
    # YAM arena camera conventions: OpenGL/USD camera looks along -Z, +Y up -> convert to CV (+Z forward, -Y up)
    p_cam_gl = r_wc.T @ (pt_w - np.asarray(station.TOP_CAM_POS, dtype=float))
    depth = -p_cam_gl[2]
    u = fx * (p_cam_gl[0] / depth) + cx
    v = fy * (-p_cam_gl[1] / depth) + cy
    return float(u), float(v), float(depth)


def test_final_pose_leaves_scored_regions_unoccluded():
    plans = cap1.demo()
    scored_pts = [np.array([cap1.RACK_XY[0], y, cap1.TOP + cap1.TUBE_H]) for y in cap1.TUBE_YS]
    scored_uvs = [_project_top_cam(p)[:2] for p in scored_pts]

    for arm in ("left", "right"):
        ap = yam_demo.ArmPlan(plans[arm], station.BASES[arm])
        q6 = np.asarray(ap.action7(ap.duration)[0], dtype=float)[:6]
        q_dict = {n: float(val) for n, val in zip(ARM_JOINTS, q6, strict=True)}
        fk = forward_kinematics(YAM_URDF, q_dict)
        base = np.asarray(station.BASES[arm], dtype=float)
        for lname in ("link4", "link5", "gripper"):
            p_link = fk[lname][:3, 3] + base
            lu, lv, _ = _project_top_cam(p_link)
            for su, sv in scored_uvs:
                dist_px = math.hypot(lu - su, lv - sv)
                assert dist_px > 40.0, f"{arm} {lname} occludes scored tube top (dist_px={dist_px:.1f})"


def _usda_extent(path: pathlib.Path) -> tuple[np.ndarray, np.ndarray]:
    """Union of every mesh ``extent`` in a visual .usda (part frame, metres)."""
    lo, hi = [], []
    for a, b in re.findall(r"float3\[\] extent = \[\(([^)]*)\), \(([^)]*)\)\]", path.read_text()):
        lo.append([float(v) for v in a.split(",")])
        hi.append([float(v) for v in b.split(",")])
    assert lo, f"no mesh extents in {path}"
    return np.min(lo, axis=0), np.max(hi, axis=0)


@pytest.mark.parametrize("spec", cap1.OBJECTS, ids=lambda s: s.name)
def test_visual_models_cover_their_colliders(spec):
    """Every part is drawn by a real-geometry visual model aligned to its (invisible) collider shapes."""
    import labkit

    visuals = list(spec.visuals) + [v for j in spec.joints for v in j.visuals]
    assert visuals, f"{spec.name} would render its primitive colliders"
    lo, hi = [], []
    for usd, at in visuals:
        assert pathlib.Path(usd).is_file() and pathlib.Path(usd).is_relative_to(PKG_DIR)
        vlo, vhi = _usda_extent(pathlib.Path(usd))
        lo.append(vlo + np.array(at))
        hi.append(vhi + np.array(at))
    vlo, vhi = np.min(lo, axis=0), np.max(hi, axis=0)
    clo, chi = labkit._extent(list(spec.boxes) + [b for j in spec.joints for b in j.boxes],
                              list(spec.cylinders) + [c for j in spec.joints for c in j.cylinders])
    assert np.all(np.abs(vlo[:2] - clo[:2]) <= 0.001) and np.all(np.abs(vhi[:2] - chi[:2]) <= 0.001), (vlo, vhi, clo, chi)
    top_allow = 0.0041 if spec.name == "rack" else 0.001  # rack end panels stand 4 mm proud of the deck (visual only)
    assert -0.001 <= vhi[2] - chi[2] <= top_allow, (vhi[2], chi[2])
