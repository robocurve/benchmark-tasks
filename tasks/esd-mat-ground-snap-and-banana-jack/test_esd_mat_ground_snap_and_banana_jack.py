"""Offline unit, parity, and negative-ablation contract tests for esd_mat_ground_snap_and_banana_jack."""

import json
import math
import pathlib
import re
import types

import numpy as np
import pytest

import esd_mat_ground_snap_and_banana_jack as t
import pitch_demo

PKG = pathlib.Path(__file__).resolve().parent


class _FakePose:
    def __init__(self, pos=(0.0, 0.0, 0.0), quat_xyzw=(0.0, 0.0, 0.0, 1.0), size=(0.02, 0.02, 0.02)):
        self.pos = np.asarray(pos, float)
        self.quat_xyzw = np.asarray(quat_xyzw, float)
        h = np.asarray(size, float) / 2.0
        self.low = self.pos - h
        self.high = self.pos + h


def _make_objects(
    tab_deg: float = 0.0,
    cap_xy_off=(0.0, 0.0),
    cap_dz: float = 0.0,
    jaw_open: float = 0.0,
    cord_in_zone: bool = False,
    plug_gap: float = 0.001,
    plug_lat: float = 0.0,
    plug_yaw_deg: float = 0.0,
    jl_open: float = 0.0,
    gb_shift=(0.0, 0.0),
    tcp_pos=(0.160, 0.300, t.TOP + 0.165),
):
    """Build synthetic stateless ObjectState dict + mocked mechkit joint/TCP readers."""
    t_orig_q = t.kit.q
    t_orig_pose = t.kit.pose
    t_orig_tcps = t.mk.tcps

    th_rad = math.radians(tab_deg)
    # Minimal mock poses in world frame matching scorer expectations
    gb_p = (t.GB_XY[0] + gb_shift[0], t.GB_XY[1] + gb_shift[1], t.GB_Z0)
    stud_w = (t.TAB_HINGE[0] + t.STUD_LOCAL[0], t.TAB_HINGE[1] + t.STUD_LOCAL[1] * math.cos(th_rad),
              t.TAB_HINGE[2] + t.STUD_LOCAL[1] * math.sin(th_rad) + 0.0034)
    cap_p = (stud_w[0] + cap_xy_off[0], stud_w[1] + cap_xy_off[1], stud_w[2] + cap_dz)

    # Plug shoulder in local coords is (0, -0.011, 0.012); ground block face is +GB_SIZE[1]/2 = +0.027
    face_y = gb_p[1] + t.GB_SIZE[1] / 2
    plug_p = (gb_p[0] + plug_lat, face_y + plug_gap + 0.011, gb_p[2])
    ry = math.radians(plug_yaw_deg) / 2.0
    pl_q = (0.0, 0.0, math.sin(ry), math.cos(ry))

    if cord_in_zone:
        cp_p = (0.205, -0.065, t.TOP + 0.005)
        cd_p = (0.220, -0.075, t.TOP + 0.005)
    else:
        cp_p = (0.340, -0.050, t.TOP + 0.005)
        cd_p = (0.360, -0.100, t.TOP + 0.005)

    poses = {
        "esd_bench": ((0.0, 0.0, t.TOP), (0.0, 0.0, 0.0, 1.0)),
        "snap_tab": (t.TAB_HINGE, (math.sin(th_rad / 2), 0.0, 0.0, math.cos(th_rad / 2))),
        "snap_jaw": (t._JAW_POS, (0.0, 0.0, 0.0, 1.0)),
        "snap_cap": (cap_p, (0.0, 0.0, 0.0, 1.0)),
        "cord_prox": (cp_p, (0.0, 0.0, 0.0, 1.0)),
        "cord_dist": (cd_p, (0.0, 0.0, 0.0, 1.0)),
        "banana_plug": (plug_p, pl_q),
        "ground_block": (gb_p, (0.0, 0.0, 0.0, 1.0)),
        "jack_latch": (t._JL_POS, (0.0, 0.0, 0.0, 1.0)),
    }
    qs = {"tab_hinge": th_rad, "jaw_slide": jaw_open, "jack_slide": jl_open}
    objs = {k: _FakePose(p, q) for k, (p, q) in poses.items()}

    # Mock cord_prox and cord_dist local-X offsets around part mid-length
    cp_part = t.kit.parts["cord_prox"]
    cd_part = t.kit.parts["cord_dist"]
    cp_mid = (cp_part.lo[0] + cp_part.hi[0]) / 2.0
    cd_mid = (cd_part.lo[0] + cd_part.hi[0]) / 2.0
    if cord_in_zone:
        poses["cord_prox"] = ((0.205 - cp_mid, -0.065, t.TOP + 0.005), (0.0, 0.0, 0.0, 1.0))
        poses["cord_dist"] = ((0.220 - cd_mid, -0.075, t.TOP + 0.005), (0.0, 0.0, 0.0, 1.0))
    else:
        poses["cord_prox"] = ((0.350 - cp_mid, -0.050, t.TOP + 0.005), (0.0, 0.0, 0.0, 1.0))
        poses["cord_dist"] = ((0.365 - cd_mid, -0.100, t.TOP + 0.005), (0.0, 0.0, 0.0, 1.0))

    t.kit.q = lambda _o, jn: qs[jn]
    t.kit.pose = lambda _o, nm: (np.asarray(poses[nm][0], float), np.asarray(poses[nm][1], float))
    t.mk.tcps = lambda: [np.asarray(tcp_pos, float), np.asarray((0.390, -0.320, t.TOP + 0.165), float)]
    return objs, (t_orig_q, t_orig_pose, t_orig_tcps)


def _restore(orig):
    t.kit.q, t.kit.pose, t.mk.tcps = orig


def test_import_order_and_dynamic_mass_floor():
    src = (PKG / "esd_mat_ground_snap_and_banana_jack.py").read_text()
    assert src.index("import mechkit") < src.index("import labkit")
    assert len(src.splitlines()) < 500
    for nm, part in t.kit.parts.items():
        if part.free:
            assert part.mass >= 0.025, f"{nm} mass {part.mass} < 0.025 kg"


def test_initial_state_and_perfect_score():
    objs, orig = _make_objects(tab_deg=14.0, cap_xy_off=(-0.18, -0.08), plug_gap=0.06)
    try:
        assert t.score(objs) == 0.0
    finally:
        _restore(orig)
    objs, orig = _make_objects()
    try:
        assert t.score(objs) == 1.0
        assert t.success(objs)
    finally:
        _restore(orig)


def test_negative_tab_curled_or_snap_missed():
    for kw in ({"tab_deg": 4.5}, {"cap_xy_off": (0.006, 0.0)}, {"jaw_open": 0.0018}, {"gb_shift": (0.010, 0.0)}):
        objs, orig = _make_objects(**kw)
        try:
            assert t.score(objs) == 0.0, f"failed to return 0.0 for {kw}"
        finally:
            _restore(orig)


def test_negative_cord_crosses_marked_work_zone():
    objs, orig = _make_objects(cord_in_zone=True)
    try:
        assert t.score(objs) == 0.50
        assert not t.success(objs)
    finally:
        _restore(orig)


def test_negative_partial_jack_and_pulled_out():
    objs, orig = _make_objects(plug_gap=0.010)
    try:
        assert t.score(objs) == 0.60
    finally:
        _restore(orig)
    objs, orig = _make_objects(plug_gap=0.025)
    try:
        assert t.score(objs) == 0.25
    finally:
        _restore(orig)


def test_negative_hands_not_clear():
    objs, orig = _make_objects(tcp_pos=(t.GB_XY[0], t.GB_XY[1] + 0.020, t.TOP + 0.030))
    try:
        assert t.score(objs) == 0.85
        assert not t.success(objs)
    finally:
        _restore(orig)


def test_demo_purity_reach_and_run_limit():
    plans = t.demo()
    assert set(plans) == {"left", "right"}
    errs = pitch_demo.check_plan(plans, t.station.BASES)
    for arm, (pos_mm, rot_deg) in errs.items():
        assert pos_mm <= 2.0 and rot_deg <= 2.0, f"{arm} IK error {(pos_mm, rot_deg)}"
    tj = json.loads((PKG / "task.json").read_text())
    m = re.search(r"(\d+)\s*steps", str(tj["run_limit"]))
    assert m is not None
    limit_steps = int(m.group(1))
    plan_s = max(sum(w.duration for w in wps) for wps in plans.values())
    assert limit_steps >= math.ceil(plan_s * 20 * 1.3)
    assert len(tj["instruction"].split()) <= 28


def test_shortcut_adversarial_and_possible_scores_enumerated():
    # Sweeping/lift-drop without snap stud seating scores 0.0; enumerates all documented Possible scores
    documented = {0.0, 0.25, 0.50, 0.60, 0.85, 1.0}
    seen = set()
    for kw in (
        {"tab_deg": 14.0, "cap_xy_off": (-0.18, -0.08), "plug_gap": 0.06},  # nothing/shortcut
        {"plug_gap": 0.025},                                                # 0.25
        {"cord_in_zone": True},                                             # 0.50
        {"plug_gap": 0.010},                                                # 0.60
        {"tcp_pos": (t.GB_XY[0], t.GB_XY[1] + 0.020, t.TOP + 0.030)},      # 0.85
        {},                                                                 # 1.0
    ):
        objs, orig = _make_objects(**kw)
        try:
            seen.add(t.score(objs))
        finally:
            _restore(orig)
    assert seen == documented


def test_threshold_parity_with_task_json():
    tj = json.loads((PKG / "task.json").read_text())
    scoring = tj["scoring"]
    for val in (t.TAB_FLAT_DEG, round(t.STUD_XY_TOL * 1e3, 1), round(t.SNAP_Z_TOL * 1e3, 1),
                round(t.JAW_CLOSED_M * 1e3, 1), round(t.JACK_HOME_GAP_M * 1e3, 1),
                round(t.JACK_PARTIAL_GAP_M * 1e3, 1), t.JACK_ALIGN_DEG, round(t.BLOCK_SHIFT_MAX * 1e3, 1),
                int(round(t.CLEAR_M * 1e3))):
        assert str(val) in scoring, f"Threshold {val} missing from task.json scoring"
    for s in ("0.0", "0.25", "0.50", "0.60", "0.85", "1.0"):
        assert s in scoring


def test_top_cam_part_footprints_visible_without_occlusion():
    # Every free/jointed assembly part must remain >= 15 mm in XY and unobstructed by grippers at completion
    for nm, part in t.kit.parts.items():
        if part.free:
            span = part.hi - part.lo
            assert max(span[0], span[1]) >= 0.015, f"{nm} too tiny {span}"
    objs, orig = _make_objects()
    try:
        assert t.components(objs)["hands_clear"], "Parked hands must not cause camera occlusion over workcell"
        assert t.CLEAR_M >= 0.045
    finally:
        _restore(orig)
    objs2, orig2 = _make_objects(tcp_pos=(t.TAB_HINGE[0], t.TAB_HINGE[1], t.TOP + 0.035))
    try:
        assert not t.components(objs2)["hands_clear"], "Hand occlusion over snap_tab must block 1.0 score"
        assert t.score(objs2) == 0.85
    finally:
        _restore(orig2)
