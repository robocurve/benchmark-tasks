"""Offline scoring, 3D interlock, hold-ablation, occlusion, parity, and IK reachability tests."""
import inspect
import json
from pathlib import Path

import numpy as np
import pytest

import reticle_gate_vernier_keeper as t


@pytest.fixture(autouse=True)
def clear(monkeypatch):
    monkeypatch.setattr(t, "_tcps", lambda: [np.array([0.1, 0.3, 1.1]), np.array([0.1, -0.3, 1.1])])


def state(**q):
    values = dict(reticle_slide=t.RETICLE_STROKE, keeper_slide=t.KEEPER_STROKE, gate_hinge=0.0)
    values.update(q)
    return t.kit.fake_states(q=values)


def _part_aabbs_world(name: str, q_val: float, is_hinge: bool = False):
    part = t.kit.parts[name]
    j = t.kit.joints_[part.parent_joint]
    if is_hinge:
        r_j = t.mk.axis_angle(j.axis, q_val)
        p_j = j.anchor - r_j @ j.anchor
    else:
        r_j = np.eye(3)
        p_j = j.axis * q_val
    world_boxes = []
    for b in part.boxes:
        rb = r_j @ part.ref[:3, :3] @ t.mk.quat_to_mat(b.quat)
        cb = r_j @ (part.ref[:3, :3] @ np.array(b.at) + part.ref[:3, 3]) + p_j
        hx, hy, hz = np.array(b.size) / 2.0
        corners = np.array([[sx * hx, sy * hy, sz * hz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
        wc = corners @ rb.T + cb
        world_boxes.append((wc.min(axis=0), wc.max(axis=0)))
    return world_boxes


def _overlap(boxes_a, boxes_b) -> bool:
    return any(
        np.all(np.minimum(ahi, bhi) - np.maximum(alo, blo) > 1e-9)
        for alo, ahi in boxes_a
        for blo, bhi in boxes_b
    )


def test_noop_initial_reset_scores_zero():
    assert t.score(state()) == 1.0
    initial = t.kit.fake_states()
    assert not t.success(initial)
    assert t.components(initial)["reticle"] < t.RETICLE_PARTIAL
    assert t.score(initial) == 0.0


def test_without_hold_unheld_gate_ablation():
    """HOLD-ABLATION: without holding beam_gate open, the closed gate physically blocks reticle_slide < 76 mm -> 0.0."""
    # Geometric block: closed gate tooth intersects reticle north lug well before RETICLE_PARTIAL (0.076 m)
    gate_boxes = _part_aabbs_world("beam_gate", 0.0, is_hinge=True)
    blocked = [q for q in np.linspace(0.0, t.RETICLE_STROKE, 91) if _overlap(gate_boxes, _part_aabbs_world("reticle_carriage", q))]
    assert blocked and min(blocked) < 0.045
    # Unheld ablation state: gate at rest (0 deg) stalls reticle short of gate tooth -> score == 0.0 < full 1.0
    unheld = state(reticle_slide=0.023, keeper_slide=t.KEEPER_STROKE, gate_hinge=0.0)
    assert t.score(unheld) == 0.0
    assert t.score(unheld) < t.score(state())


def test_left_or_right_parked_shortcut_zero():
    assert t.score(state(reticle_slide=0.0)) == 0.0
    assert t.score(state(keeper_slide=0.0)) == 0.4


def test_displaced_or_tilted_stage_shortcut_zero():
    moved = t.kit.fake_states(
        q=dict(reticle_slide=t.RETICLE_STROKE, keeper_slide=t.KEEPER_STROKE, gate_hinge=0.0),
        moved={"stage": ((t.SX + 0.013, t.SY, t.Z_HOME + 0.009), (0, 0, 0, 1))},
    )
    assert t.components(moved)["base_off"] == pytest.approx(np.hypot(0.013, 0.009))
    assert t.score(moved) == 0.0
    tilted = t.kit.fake_states(
        q=dict(reticle_slide=t.RETICLE_STROKE, keeper_slide=t.KEEPER_STROKE, gate_hinge=0.0),
        moved={"stage": ((t.SX, t.SY, t.Z_HOME), t.mk.mat_to_quat(t.mk.axis_angle((1, 0, 0), np.radians(10.0))))},
    )
    assert t.score(tilted) == 0.0


def test_beam_gate_spring_self_closes_and_clears_seated_reticle():
    j = t.kit.joints_["gate_hinge"]
    assert j.stiffness > 0 and j.rest == j.lower == 0.0 and j.damping > 0
    assert j.stiffness * np.radians(t.GATE_CLOSED_DEG) > j.friction
    # Open gate at >= 44 deg clears the full 90 mm reticle sweep, and closing behind seated reticle has zero hit
    open_gate = _part_aabbs_world("beam_gate", np.radians(44.0), is_hinge=True)
    for q in np.linspace(0.0, t.RETICLE_STROKE, 46):
        assert not _overlap(open_gate, _part_aabbs_world("reticle_carriage", q))
    seated_reticle = _part_aabbs_world("reticle_carriage", t.RETICLE_SEATED)
    for deg in np.linspace(0.0, 44.0, 23):
        assert not _overlap(_part_aabbs_world("beam_gate", np.radians(deg), is_hinge=True), seated_reticle)


def test_3d_vernier_keeper_physically_locks_reticle_and_gate():
    """C23 real 3D physical engagement: vernier_keeper blocks unseating reticle & traps gate, and cannot engage if unseated."""
    keeper_locked = _part_aabbs_world("vernier_keeper", t.KEEPER_STROKE)
    seated_ret = _part_aabbs_world("reticle_carriage", t.RETICLE_STROKE)
    closed_gate = _part_aabbs_world("beam_gate", 0.0, is_hinge=True)
    assert not _overlap(keeper_locked, seated_ret)
    assert not _overlap(keeper_locked, closed_gate)
    # 1. Locked keeper physically blocks -X backout of seated reticle (-10 mm -> 0.080) and traps gate below 5 deg
    assert _overlap(keeper_locked, _part_aabbs_world("reticle_carriage", 0.080))
    assert _overlap(keeper_locked, _part_aabbs_world("beam_gate", np.radians(5.0), is_hinge=True))
    # 2. Unseated reticle (0.068 < RETICLE_PARTIAL) or open gate (5 deg) physically blocks keeper before KEEPER_ENGAGED
    keeper_engaged = _part_aabbs_world("vernier_keeper", t.KEEPER_ENGAGED)
    assert _overlap(keeper_engaged, _part_aabbs_world("reticle_carriage", 0.068))
    assert _overlap(keeper_engaged, _part_aabbs_world("beam_gate", np.radians(5.0), is_hinge=True))


def test_possible_scores():
    assert t.score(state()) == 1.0
    assert t.score(state(reticle_slide=0.078)) == 0.4
    assert t.score(state(keeper_slide=0.010)) == 0.4
    assert t.score(state(gate_hinge=np.radians(15.0))) == 0.4
    assert t.score(state(reticle_slide=0.050)) == 0.0


def test_occlusion_missing_or_single_tcp_fails_closed(monkeypatch):
    seated = state()
    assert t.score(seated) == 1.0
    monkeypatch.setattr(t, "_tcps", lambda: [])
    assert not t.components(seated)["clear"] and t.score(seated) == 0.7
    monkeypatch.setattr(t, "_tcps", lambda: [np.array([0.0, 0.0, 2.0])])
    assert t.score(seated) == 0.7


def test_proximity_hover_and_clearance_all_bodies(monkeypatch):
    s = state()
    for name in t.DYNAMIC_NAMES:
        monkeypatch.setattr(t, "_tcps", lambda n=name: [s[n].pos, np.array([0.0, 0.0, 2.0])])
        assert t.score(s) == 0.7


def test_stateless_order_and_zero_on_step():
    s = state()
    assert t.score(s) == 1.0
    assert t.score(t.kit.fake_states()) == 0.0
    assert t.score(s) == 1.0
    assert t.SUCCESS_HOLD_S == 3.0 and t.SCORE_OVER == "end"
    assert not t.kit._steps


def test_masses_geoms_and_pbr_usdc_assets():
    pkg = Path(__file__).parent
    assert all(p.mass >= 0.025 for p in t.kit.parts.values())
    boxes = [b for p in t.kit.parts.values() for b in p.boxes]
    assert len(boxes) >= 24 and len({b.color for b in boxes}) >= 7
    for name in ("stage", "beam_gate", "reticle_carriage", "vernier_keeper", "bench_mat"):
        assert (pkg / "assets" / t.SLUG / f"{name}.usdc").is_file()


def test_metadata_parity():
    data = json.loads((Path(__file__).parent / "task.json").read_text())
    assert len(data["title"]) <= 115 and len(data["instruction"].split()) <= 24
    assert len(data["fidelity"]) <= 460
    assert data["category"] == "Other"
    assert "Possible scores 0.0, 0.4, 0.7, 1.0" in data["scoring"]
    assert "caliper/ruler and digital protractor" in data["scoring"]
    assert "## Importance" in (Path(__file__).parent / "README.md").read_text()


def test_demo_purity_and_reachability():
    src = inspect.getsource(t.demo)
    assert all(x not in src for x in ("kit.pose", "kit.q", "objects", "_tcps"))
    p = t.demo()
    assert set(p) == {"left", "right"}
    assert max(sum(w.duration for w in a) for a in p.values()) <= 60.0
    for arm, plan in p.items():
        assert plan[-1].duration >= 3.0
        assert all(w.pos[1] >= 0.0 if arm == "left" else w.pos[1] <= 0.0 for w in plan)
    errors = t.pitch_demo.check_plan(p, t.station.BASES, dt=0.1)
    assert all(mm < 4.0 and deg < 4.0 for mm, deg in errors.values()), errors
