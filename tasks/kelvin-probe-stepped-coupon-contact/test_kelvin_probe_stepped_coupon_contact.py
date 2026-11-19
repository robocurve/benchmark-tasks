"""Offline unit, HOLD-ABLATION, occlusion, shortcut, PBR visual, and reachability tests."""
import json
import math
import pathlib
import re
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import kelvin_probe_stepped_coupon_contact as t  # noqa: E402
import pitch_demo  # noqa: E402
from yam_arena import station  # noqa: E402

Q_ID = np.array([0.0, 0.0, 0.0, 1.0], float)
OUT_AT = (np.array([t.CLIP_BAY_XY[0], t.CLIP_BAY_XY[1], t.BASE_Z0], float), Q_ID)
DOCK_AT = (np.array([t.BX + t.CLIP_DOCK_REL[0], t.BY + t.CLIP_DOCK_REL[1], t.BASE_Z0 + t.CLIP_DOCK_REL[2]], float), Q_ID)
PARK = [np.array([0.14, 0.29, t.TOP + 0.195]), np.array([0.14, -0.29, t.TOP + 0.195])]


def _eval(monkeypatch, *, q=None, moved=None, tcps=None):
    st = t.kit.fake_states(q=q or {}, moved=moved or {})
    monkeypatch.setattr(t.mk, "tcps", lambda: PARK if tcps is None else tcps)
    return t.score(st)


def seated_kw(**over):
    kw = dict(
        q={"coupon_slide": t.COUPON_TARGET, "probe_hinge": 0.0, "head_compliance": 0.0, "datum_slide": 0.002},
        moved={"transit_clip": OUT_AT},
        tcps=PARK,
    )
    kw.update(over)
    return kw


def test_initial_noop_scores_zero(monkeypatch):
    assert _eval(monkeypatch) == 0.0


def test_full_score_and_sub_tiers(monkeypatch):
    assert _eval(monkeypatch, **seated_kw()) == 1.0
    # 0.7: gripper inside 45 mm clearance
    assert _eval(monkeypatch, **seated_kw(tcps=[np.array([t.BX, 0.01, t.BASE_Z0 + 0.04]), PARK[1]])) == 0.7
    # 0.7: alignment error 7 mm (> 6 mm full, <= 8 mm)
    assert _eval(monkeypatch, **seated_kw(q={"coupon_slide": t.COUPON_TARGET - 0.007, "probe_hinge": 0.0})) == 0.7
    # 0.4: alignment error 10 mm (> 8 mm, <= 12 mm)
    assert _eval(monkeypatch, **seated_kw(q={"coupon_slide": t.COUPON_TARGET - 0.010, "probe_hinge": 0.0})) == 0.4
    # 0.0: alignment error > 12 mm
    assert _eval(monkeypatch, **seated_kw(q={"coupon_slide": t.COUPON_TARGET - 0.015, "probe_hinge": 0.0})) == 0.0


def test_without_hold_unheld_gate_ablation(monkeypatch):
    # HOLD-ABLATION: if coupon is unheld and recoils back to 0 under its 25 N/m return spring while
    # the probe arm lowers (or transit_clip remains unheld), score(unheld) == 0.0.
    unheld_coupon_recoil = seated_kw(q={"coupon_slide": 0.0, "probe_hinge": math.radians(-10.0)})
    assert _eval(monkeypatch, **unheld_coupon_recoil) == 0.0
    assert _eval(monkeypatch, **seated_kw(moved={"transit_clip": DOCK_AT})) == 0.0


def test_shortcut_ablation_clip_docked_and_tilted_head_scores_zero(monkeypatch):
    # Negative controls: transit clip docked, knocked outside bay, held mid-air, or tipped > 20 deg, or tilted head -> 0.0.
    assert _eval(monkeypatch, **seated_kw(moved={"transit_clip": DOCK_AT})) == 0.0
    outside_bay = (np.array([t.BX - 0.050, t.CLIP_BAY_XY[1], t.BASE_Z0], float), Q_ID)
    assert _eval(monkeypatch, **seated_kw(moved={"transit_clip": outside_bay})) == 0.0
    midair_bay = (np.array([t.CLIP_BAY_XY[0], t.CLIP_BAY_XY[1], t.BASE_Z0 + 0.030], float), Q_ID)
    assert _eval(monkeypatch, **seated_kw(moved={"transit_clip": midair_bay})) == 0.0
    tipped_bay = (np.array([t.CLIP_BAY_XY[0], t.CLIP_BAY_XY[1], t.BASE_Z0], float), np.array([0.0, 0.35, 0.0, 0.9367], float))
    assert _eval(monkeypatch, **seated_kw(moved={"transit_clip": tipped_bay})) == 0.0
    assert _eval(monkeypatch, **seated_kw(q={"coupon_slide": t.COUPON_TARGET, "probe_hinge": math.radians(-15.0)})) == 0.0


def test_negative_control_reversed_coupon_and_stage_knock_scores_zero(monkeypatch):
    # Negative controls: 180-deg reversed coupon loading or stage shifted/lifted > 15 mm in 3D scores 0.0.
    knocked = (np.array([t.BX + 0.020, t.BY + 0.010, t.BASE_Z0]), Q_ID)
    assert _eval(monkeypatch, **seated_kw(moved={"transit_clip": OUT_AT, "carrier_stage": knocked})) == 0.0
    lifted = (np.array([t.BX, t.BY, t.BASE_Z0 + 0.020]), Q_ID)
    assert _eval(monkeypatch, **seated_kw(moved={"transit_clip": OUT_AT, "carrier_stage": lifted})) == 0.0
    rev = (np.array([t.BX + 0.018, t.BY, t.COUPON_Z0]), np.array([0.0, 0.0, 1.0, 0.0]))
    assert _eval(monkeypatch, **seated_kw(moved={"transit_clip": OUT_AT, "stepped_coupon": rev})) == 0.0


def test_top_camera_line_of_sight_unoccluded():
    cam = np.array(station.top_camera("calibrated").pos, float)
    target = np.array([t.BX + 0.018, t.BY, t.BASE_Z0 + 0.028], float)
    ray = target - cam
    rdir = ray / np.linalg.norm(ray)
    for tcp in PARK:
        proj = float(np.dot(tcp - cam, rdir))
        perp = float(np.linalg.norm((tcp - cam) - proj * rdir))
        assert perp > 0.10 or proj < 0.0 or proj > float(np.linalg.norm(ray))


def test_pbr_visuals_and_contract():
    vis = HERE / "assets" / "kelvin-probe-stepped-coupon-contact"
    for name in ("bench_mat.usdc", "carrier_stage.usdc", "datum_fence.usdc", "stepped_coupon.usdc",
                 "probe_arm.usdc", "probe_head.usdc", "transit_clip.usdc"):
        assert (vis / name).is_file(), f"missing {name}"
    assert (vis / "textures").is_dir() and len(list((vis / "textures").glob("*.png"))) >= 30
    tj = json.loads((HERE / "task.json").read_text())
    assert len(tj["instruction"].split()) <= 22
    assert "possible scores: 0.0, 0.4, 0.7, 1.0" in tj["scoring"].lower()
    assert tj["real_trial_cost_usd"] == 76 and t.SUCCESS_HOLD_S == 3.0
    for p in t.kit.parts.values():
        assert p.mass is None or p.mass >= 0.040


def test_plan_kinematic_reach_and_demo_purity():
    plan = t.demo()
    assert sorted(plan) == ["left", "right"]
    reach = pitch_demo.check_plan(plan, station.BASES)
    for arm, (mm, deg) in reach.items():
        assert mm <= 5.0 and deg <= 8.0, (arm, reach[arm])
    assert not set(t.demo.__code__.co_names) & {"env", "scene", "objects", "score", "components"}
    steps = int(re.match(r"([\d,]+)\s*steps",
                         json.loads((HERE / "task.json").read_text())["run_limit"]).group(1).replace(",", ""))
    assert steps >= math.ceil(max(sum(w.duration for w in v) for v in plan.values()) * 20)
