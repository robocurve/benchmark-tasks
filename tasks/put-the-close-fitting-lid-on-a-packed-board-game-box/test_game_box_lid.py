"""Pure offline tests (no simulator) for ``mech/tasks/game_box_lid``:
- scene invariants and Check-51 pure rigid-body contact (zero welds, zero hidden joints, zero computed pushes);
- Gate G1 offline checks across all three commercial Ravensburger box presets (5-wall collider bounds, IK <= 0.5 mm,
  joint margins >= 5 deg, TCP separation >= 200 mm, bench/skirt finger clearances >= 10 mm, top-camera visibility);
- wedging geometry and preset switching via ``GAME_BOX_PRESET``;
- every scoring rule, whole-trial fault latch, partial-credit boundary, 180-deg rectangular lid symmetry, gripper
  clearance, rest speed threshold, and 1.0 s continuous hold requirement.
"""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
import re
import sys

import numpy as np
import pytest
from yam_arena import task_file

HERE = Path(__file__).resolve().parent
TASK = HERE / "task.py"
OWN = ("mechkit", "kit_extra", "demo_kit", "parts", "checks", "demo", "trials")
DT = 0.05
AWAY = {
    "left": (np.array([0.15, +0.34, 1.02]), 1.0),
    "right": (np.array([0.15, -0.34, 1.02]), 1.0),
}


def axis_angle(axis, deg: float) -> np.ndarray:
    a = np.asarray(axis, dtype=float)
    a /= np.linalg.norm(a)
    rad = math.radians(deg)
    k = np.array([[0.0, -a[2], a[1]], [a[2], 0.0, -a[0]], [-a[1], a[0], 0.0]])
    return np.eye(3) + math.sin(rad) * k + (1.0 - math.cos(rad)) * (k @ k)


@pytest.fixture
def m():
    before = {name: sys.modules.pop(name, None) for name in OWN}
    old_preset = os.environ.pop("GAME_BOX_PRESET", None)
    sys.path.insert(0, str(HERE))
    try:
        module = task_file.load(TASK).module
        fresh(module)
        yield module
        module.mk.MECH.clear()
    finally:
        if old_preset is not None:
            os.environ["GAME_BOX_PRESET"] = old_preset
        else:
            os.environ.pop("GAME_BOX_PRESET", None)
        for name, held in before.items():
            sys.modules.pop(name, None)
            if held is not None:
                sys.modules[name] = held


def fresh(m) -> None:
    m.mk.MECH.new_episode()
    m.start()


class Bench:
    """Synthetic control-step harness driving ``task.py`` via ``mk.set_state`` without the simulator."""

    def __init__(self, m) -> None:
        self.m = m
        self.t = 0.0
        self.base_pos = np.array([m.P.BASE_AT[0], m.P.BASE_AT[1], m.TOP], dtype=float)
        self.base_rot = np.eye(3)
        self.lid_pos = np.array([m.P.LID_AT[0], m.P.LID_AT[1], m.TOP], dtype=float)
        self.lid_rot = np.eye(3)
        self.extra_poses: dict[str, tuple[np.ndarray, np.ndarray]] = {}
        self.velocities: dict[str, tuple[np.ndarray, np.ndarray]] = {}
        self.grippers = dict(AWAY)
        self.lid_base_force = 0.0

    def place_lid_on_base(
        self,
        *,
        roof_h: float | None = None,
        tilt_deg: float = 0.0,
        yaw_deg: float = 0.0,
        offset_xy: tuple[float, float] = (0.0, 0.0),
    ) -> None:
        """Place ``box_lid`` relative to ``box_base`` with exact ``roof_h``, ``tilt_deg`` (roll), ``yaw_deg``, and ``offset_xy``."""
        P = self.m.P
        target_roof_h = P.CLOSED_H if roof_h is None else roof_h
        rel_rot = axis_angle([0.0, 0.0, 1.0], yaw_deg) @ axis_angle([1.0, 0.0, 0.0], tilt_deg)
        lr = self.base_rot @ rel_rot
        # Put mid-height xy at offset_xy in B, and top outer roof center z at target_roof_h in B
        mid_local = rel_rot @ np.array([0.0, 0.0, 0.5 * P.LID_OUT_H])
        roof_local = rel_rot @ np.array([0.0, 0.0, P.LID_OUT_H])
        origin_in_b = np.array(
            [offset_xy[0] - mid_local[0], offset_xy[1] - mid_local[1], target_roof_h - roof_local[2]],
            dtype=float,
        )
        self.lid_pos = self.base_pos + self.base_rot @ origin_in_b
        self.lid_rot = lr

    def sync_contents_to_base(self) -> dict[str, tuple[np.ndarray, np.ndarray]]:
        cs = self.m.P.content_specs()
        out = {}
        for name in self.m.CONTENTS:
            lp = np.asarray(cs[name]["local_pos"], dtype=float)
            out[name] = (self.base_pos + self.base_rot @ lp, self.base_rot.copy())
        return out

    def hold(self, seconds: float = DT) -> float:
        steps = max(1, int(round(seconds / DT)))
        s = 0.0
        for _ in range(steps):
            self.t = round(self.t + DT, 6)
            poses = {
                "box_base": (self.base_pos.copy(), self.base_rot.copy()),
                "box_lid": (self.lid_pos.copy(), self.lid_rot.copy()),
                **self.sync_contents_to_base(),
                **self.extra_poses,
            }
            contacts = {("box_lid", "box_base"): self.lid_base_force}
            self.m.mk.set_state(poses, self.velocities, grippers=self.grippers, time=self.t, contacts=contacts)
            self.m.step()
            s = self.m.score(None)
        return s


# --- scene & Check 51 invariants ---


def test_scene_and_check51_pure_rigid_contact(m):
    P, mech = m.P, m.mk.MECH
    assert m.INSTRUCTION == (
        "Put the lid squarely onto the packed game box so it seats all the way down over the base "
        "without spilling the contents or moving the box off its marks."
    )
    assert (m.SCENE, m.SCORE_OVER, m.SUCCESS_HOLD_S, m.RUN_LIMIT_STEPS) == ("station", "end", 3.5, 1200)
    # Check 51: zero welds, zero hidden joints, zero computed mk.push loads
    assert len(mech.joints) == 0
    assert len(mech.pushed) == 0
    assert mech.sensed == {"box_lid": ("box_base",)}
    assert mech.bodies["box_base"].contact_offset == pytest.approx(0.0004)
    assert mech.bodies["box_lid"].contact_offset == pytest.approx(0.0004)
    assert mech.bodies["corner_marks"].mass is None
    assert sum(mech.bodies[c].mass for c in m.CONTENTS) == pytest.approx(P.CONTENTS_MASS, abs=1e-6)
    assert len(P.FIGURES) >= 7
    assert set(P.BOX_PRESETS) == {"ravensburger_20847", "ravensburger_275x192", "ravensburger_183x115"}
    assert P.BOX_PRESET == "ravensburger_20847"  # named product with the maker's box size is the default
    assert P.ACTIVE_BOX.source_kind == "maker"


def test_task_json_and_readme_checklist_invariants(m):
    spec = json.loads((HERE / "task.json").read_text())
    assert len(spec["title"]) <= 120
    assert len(spec["context"]) <= 1000
    assert len(spec["fidelity"]) <= 500
    assert "not modelled" in spec["fidelity"].lower() or "not modeled" in spec["fidelity"].lower()
    assert spec["category"] == "Societal relatability"
    assert spec["instruction"] == m.INSTRUCTION
    assert spec["run_limit"] == "1,200 steps (60 s at 20 Hz)"
    readme = (HERE / "README.md").read_text()
    task_py = (HERE / "task.py").read_text()
    # Checklist 52: length limits and zero review history / reviewer address
    assert len(readme) <= 10_000, len(readme)
    assert len(task_py) <= 15_000, len(task_py)
    assert "earlier review" not in readme.lower()
    assert "thank you" not in readme.lower()
    assert "1415158438" in readme
    assert "MetaWorld+: Box close" in readme
    assert "article 20847" in readme  # the named real product
    for p in HERE.iterdir():
        assert not p.is_symlink(), f"symlink not allowed in task folder: {p.name}"
        if p.suffix in {".json", ".md", ".py"}:
            txt = p.read_text()
            assert re.search(r"(?<![\w/])(#\d{3,4}|TK-[A-Z0-9]{6})\b", txt) is None, f"board ID leak in {p.name}"


def test_g1_offline_checks_across_all_three_presets(m):
    import checks

    res = checks.run_offline()
    assert set(res) == {"ravensburger_20847", "ravensburger_275x192", "ravensburger_183x115"}
    for key, data in res.items():
        ik_hulls = data["ik_and_hulls"]
        for side in ("left", "right"):
            assert ik_hulls["dry_run"][side]["worst_ik_mm"] <= 0.5, (key, side)
            assert ik_hulls["dry_run"][side]["smallest_margin_deg"] >= 5.0, (key, side)
            assert ik_hulls["dry_run"][side]["push_effort"] <= 1.0, (key, side)
        assert ik_hulls["min_tcp_sep_mm"] >= 200.0, key
        assert ik_hulls["min_bench_clear_mm"] >= 10.0, key
        assert ik_hulls["min_tip_above_lid_bottom_mm"] >= 10.0, key
        assert ik_hulls["max_wall_intrusion_mm"] <= 0.5, key
        assert data["top_cam"]["clear_of_left_arm_px"] > 10.0, key
        assert data["top_cam"]["clear_of_right_arm_px"] > 10.0, key


def test_demo_quick_dry_run_and_trials_imports(m):
    import demo
    import trials
    from yam_arena import station

    bases = {s: np.array(station.BASES[s], dtype=float) for s in ("left", "right")}
    initial, director, _ = demo.make_director(m, m.P.ACTIVE_BOX)
    dry = demo.kit.dry_run(initial, bases, director=director, tail_s=4.0, limit_s=m.P.RUN_LIMIT_S)
    for side in ("left", "right"):
        assert dry[side]["worst_ik_mm"] <= 0.5
        assert dry[side]["smallest_margin_deg"] >= 5.0
        assert dry[side]["push_effort"] <= 1.0
    shove = trials.side_shove_moves(m)
    dry_shove = demo.kit.dry_run(shove, bases, tail_s=1.0, limit_s=m.P.RUN_LIMIT_S)
    assert dry_shove["left"]["worst_ik_mm"] <= 0.5


@pytest.mark.parametrize("preset_name", ["ravensburger_20847", "ravensburger_275x192", "ravensburger_183x115"])
def test_preset_reload_and_wedge_geometry(preset_name):
    before = {name: sys.modules.pop(name, None) for name in OWN}
    old = os.environ.get("GAME_BOX_PRESET")
    os.environ["GAME_BOX_PRESET"] = preset_name
    sys.path.insert(0, str(HERE))
    try:
        mod = task_file.load(TASK).module
        P = mod.P
        assert P.LID_IN_L == pytest.approx(P.BASE_L + 2.0 * P.CLEAR_SIDE)
        assert P.LID_IN_W == pytest.approx(P.BASE_W + 2.0 * P.CLEAR_SIDE)
        assert P.LID_OUT_L == pytest.approx(P.LID_IN_L + 2.0 * P.BOARD_T)
        assert P.LID_OUT_W == pytest.approx(P.LID_IN_W + 2.0 * P.BOARD_T)
        assert P.START_GAP_X > 0.020
        assert 2.0 <= P.WEDGE_TILT_DEG <= 6.0
        # At 6.0 deg roll, high skirt side jams well short of full LID_IN_H depth
        assert P.wedge_stop_depth_m(6.0) < 0.60 * P.LID_IN_H
        mod.mk.MECH.clear()
    finally:
        if old is None:
            os.environ.pop("GAME_BOX_PRESET", None)
        else:
            os.environ["GAME_BOX_PRESET"] = old
        for name, held in before.items():
            sys.modules.pop(name, None)
            if held is not None:
                sys.modules[name] = held


# --- idle, faults, partial credit, and full score ---


def test_idle_start_scores_zero(m):
    b = Bench(m)
    assert b.hold(2.0) == 0.0
    assert m.STATE["fault"] is None
    assert m.spilled_contents() == []


def test_base_tilt_whole_trial_zero_threshold_and_latch(m):
    # 14.9 deg tilt does not latch fault -> recovers to 1.0 when leveled and seated
    b = Bench(m)
    b.base_rot = axis_angle([1.0, 0.0, 0.0], 14.9)
    assert b.hold(0.10) == 0.0
    assert m.STATE["fault"] is None
    b.base_rot = np.eye(3)
    b.place_lid_on_base()
    assert b.hold(1.05) == 1.0

    # 15.1 deg tilt latches fault permanently -> stays 0.0 even after perfect seating
    fresh(m)
    b2 = Bench(m)
    b2.base_rot = axis_angle([1.0, 0.0, 0.0], 15.1)
    assert b2.hold(0.05) == 0.0
    assert m.STATE["fault"] is not None
    b2.base_rot = np.eye(3)
    b2.place_lid_on_base()
    assert b2.hold(1.50) == 0.0


def test_base_corner_shift_whole_trial_zero_threshold_and_latch(m):
    # 24.9 mm shift stays inside 25.0 mm tolerance
    b = Bench(m)
    b.base_pos[0] += 0.0249
    b.place_lid_on_base()
    assert b.hold(1.05) == 1.0

    # 25.1 mm shift latches permanent 0.0 fault even if pushed back onto its marks
    fresh(m)
    b2 = Bench(m)
    b2.base_pos[0] += 0.0251
    assert b2.hold(0.05) == 0.0
    assert m.STATE["fault"] is not None
    b2.base_pos[0] = m.P.BASE_AT[0]
    b2.place_lid_on_base()
    assert b2.hold(1.50) == 0.0


def test_base_yaw_moving_corners_beyond_25mm_latches_zero(m):
    # Corner radius is hypot(112.2, 164.2) = 198.8 mm; 9.0 deg yaw moves corners ~31.2 mm > 25.0 mm
    b = Bench(m)
    b.base_rot = axis_angle([0.0, 0.0, 1.0], 9.0)
    assert b.hold(0.05) == 0.0
    assert m.STATE["fault"] is not None
    b.base_rot = np.eye(3)
    b.place_lid_on_base()
    assert b.hold(1.50) == 0.0


@pytest.mark.parametrize("piece", ["tile_stack", "card_deck", "wood_pawn"])
def test_spilled_content_latches_permanent_zero(m, piece):
    b = Bench(m)
    b.extra_poses[piece] = (np.array([0.25, 0.25, m.TOP + 0.015]), np.eye(3))
    assert b.hold(0.05) == 0.0
    assert m.STATE["fault"] is not None and piece in m.STATE["fault"][1]
    # Putting the spilled piece back inside never clears the whole-trial fault
    b.extra_poses.clear()
    b.place_lid_on_base()
    assert b.hold(1.50) == 0.0


@pytest.mark.parametrize(
    "case, kwargs, contact_n",
    [
        ("upside_down", {"roof_h": 0.0550, "tilt_deg": 180.0, "yaw_deg": 0.0, "offset_xy": (0.0, 0.0)}, 2.0),
        ("off_box_beside_base", {"roof_h": 0.0680, "tilt_deg": 0.0, "yaw_deg": 0.0, "offset_xy": (0.28, 0.05)}, 2.0),
        ("over_base_but_not_touching", {"roof_h": 0.0680, "tilt_deg": 0.0, "yaw_deg": 0.0, "offset_xy": (0.0, 0.0)}, 0.0),
        ("feather_touch_only", {"roof_h": 0.0680, "tilt_deg": 0.0, "yaw_deg": 0.0, "offset_xy": (0.0, 0.0)}, 0.4),
        ("tilt_over_15deg", {"roof_h": 0.0800, "tilt_deg": 15.2, "yaw_deg": 0.0, "offset_xy": (0.0, 0.0)}, 2.0),
    ],
)
def test_off_box_or_moving_lid_scores_zero(m, case, kwargs, contact_n):
    b = Bench(m)
    b.place_lid_on_base(**kwargs)
    b.lid_base_force = contact_n
    assert b.hold(1.20) == 0.0, case


def test_cocked_on_box_lid_scores_half(m):
    # The everyday half result: the lid rests ON the box, cocked, not seated -- at rest, hands clear,
    # over the base, pressing on it (contact >= 0.5 N), tilt <= 15 deg, corners proud of the closed height.
    b = Bench(m)
    b.place_lid_on_base(roof_h=0.076, tilt_deg=2.0, yaw_deg=0.2, offset_xy=(0.0002, 0.0002))
    b.lid_base_force = 2.0
    assert b.hold(0.95) == 0.0  # the on-box rest needs its 1.0 s hold before 0.5
    assert b.hold(0.10) == 0.5
    assert m.STATE["eng_held_s"] >= 1.0

    # Moving lid does NOT get 0.5 (on_box_now requires all_at_rest)
    b.velocities["box_lid"] = (np.array([0.012, 0.0, 0.0]), np.zeros(3))
    assert b.hold(0.05) == 0.0
    b.velocities.clear()
    b.place_lid_on_base(roof_h=0.076, tilt_deg=2.0, yaw_deg=0.2, offset_xy=(0.0002, 0.0002))
    assert b.hold(1.05) == 0.5

    # No roof cap and no straddle: a proud rest at CLOSED_H + 50 mm counts, and so does a lid cocked
    # with one skirt stepped inside the cavity (see test_cocked_with_skirt_inside_or_perched_scores_half)
    fresh(m)
    b2 = Bench(m)
    b2.lid_base_force = 2.0
    b2.place_lid_on_base(roof_h=m.P.CLOSED_H + 0.050, tilt_deg=0.0)
    assert b2.hold(1.10) == 0.5
    b2.place_lid_on_base(roof_h=0.0720, tilt_deg=1.5, offset_xy=(0.0, 0.0025))
    assert b2.hold(1.10) == 0.5


def test_seated_is_roof_height_only_walls_bound_centring(m):
    # Seated: EVERY roof corner within 1.0 mm of CLOSED_H, tilt <= 2 deg, lid over the box -- no offset or yaw test.
    # A lid ~1.4 mm off-centre at full height is fully seated (the walls bound it to <= 1.5 mm in reality).
    b = Bench(m)
    b.place_lid_on_base(offset_xy=(0.0010, 0.0010), yaw_deg=0.6)
    assert b.hold(1.05) == 1.0
    # Corner boundary: +0.9 mm every corner -> 1.0; +1.1 mm (with contact) -> half
    b.place_lid_on_base(roof_h=m.P.CLOSED_H + 0.0009)
    assert b.hold(1.05) == 1.0
    b.lid_base_force = 2.0
    b.place_lid_on_base(roof_h=m.P.CLOSED_H + 0.0011)
    assert b.hold(1.05) == 0.5
    # A lid down at closed height but NOT over the box (beside it) is not seated
    fresh(m)
    b3 = Bench(m)
    b3.place_lid_on_base(offset_xy=(0.28, 0.0))
    b3.lid_base_force = 2.0
    assert b3.hold(1.05) == 0.0


@pytest.mark.parametrize(
    "case, offset_xy, roof_h, tilt_deg, straddle",
    [
        ("skirt_perched_on_base_rim_wall", (0.0, 0.0025), 0.0720, 1.5, False),
        ("skirt_over_cavity_cocked", (0.0, 0.0055), 0.0720, 1.5, False),
    ],
)
def test_cocked_with_skirt_inside_or_perched_scores_half(m, case, offset_xy, roof_h, tilt_deg, straddle):
    # A lid cocked with one skirt inside the base / perched on the rim is the everyday half result:
    # it rests on the box (contact), not seated; straddle is deliberately not required for 0.5.
    # (A lid stepped inside the cavity AT closed height is geometrically impossible: the walls bound
    # centring to <= 1.5 mm, inside the 1.0 mm corner window -- see test_seated_is_roof_height_only.)
    b = Bench(m)
    b.place_lid_on_base(roof_h=roof_h, tilt_deg=tilt_deg, yaw_deg=0.0, offset_xy=offset_xy)
    b.lid_base_force = 2.0
    assert b.hold(1.20) == 0.5, case
    assert m.skirts_straddle_base() is straddle, case  # the dropped geometric condition, still computed


def test_full_seat_hold_duration_symmetry_and_tight_thresholds(m):
    b = Bench(m)
    b.place_lid_on_base()
    # 19 steps (0.95 s): seated but the 1.0 s hold is incomplete -> 0.0 (a metric-seated lid is not the 0.5 state)
    assert b.hold(0.95) == 0.0
    assert not m.success(None)
    assert b.hold(0.05) == 1.0
    assert m.success(None)

    # Lifting lid back to 76 mm drops 1.0 to the on-box rest, which needs its own 1.0 s hold before 0.5
    b.place_lid_on_base(roof_h=0.076, tilt_deg=1.5)
    b.lid_base_force = 2.0
    assert b.hold(0.05) == 0.0
    assert m.STATE["full_held_s"] == 0.0
    assert b.hold(1.00) == 0.5

    # Rectangular 180-deg yaw symmetry also scores 1.0
    fresh(m)
    b_sym = Bench(m)
    b_sym.place_lid_on_base(yaw_deg=180.0)
    assert b_sym.hold(1.00) == 1.0


@pytest.mark.parametrize(
    "knob, pass_val, fail_val, fail_expected",
    [
        ("corner_above", 0.0009, 0.0011, 0.5),
        ("gripper_clear_m", 0.031, 0.029, 0.0),
        ("lin_speed", 0.0095, 0.0105, 0.0),
        ("ang_speed", 0.095, 0.105, 0.0),
    ],
)
def test_every_full_score_limit_at_both_sides(m, knob, pass_val, fail_val, fail_expected):
    # (SEAT_TILT_MAX_DEG 2.0 is belt-and-braces: the 1.0 mm corner window alone caps tilt near 0.2 deg)
    assert m.P.SEAT_TILT_MAX_DEG == 2.0 and m.P.SEAT_CORNER_TOL == 0.001
    for val, want in ((pass_val, 1.0), (fail_val, fail_expected)):
        fresh(m)
        b = Bench(m)
        kwargs = {}
        if knob == "corner_above":
            kwargs["roof_h"] = m.P.CLOSED_H + val
        b.place_lid_on_base(**kwargs)
        b.lid_base_force = 2.0  # the fail side rests on the box -> half, not zero
        if knob == "gripper_clear_m":
            b.grippers["left"] = (
                np.array([m.P.BASE_AT[0], 0.5 * m.P.LID_OUT_L + val, m.TOP + 0.030]),
                0.5,
            )
        elif knob == "lin_speed":
            b.velocities["box_lid"] = (np.array([val, 0.0, 0.0]), np.zeros(3))
        elif knob == "ang_speed":
            b.velocities["box_lid"] = (np.zeros(3), np.array([0.0, 0.0, val]))
        assert b.hold(1.05) == want, (knob, val)
