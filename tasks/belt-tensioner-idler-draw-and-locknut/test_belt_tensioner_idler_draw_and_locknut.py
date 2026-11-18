"""Offline unit tests for belt-tensioner-idler-draw-and-locknut (CPU only; no simulator).

States are built synthetically via PartState plus monkeypatched mk.tcps(). Covers every documented score tier
(0.0, 0.4, 0.7, 1.0), pure end-state clamping, fail-closed TCP check, negative shortcut and ablation controls,
top-camera occlusion clearance, scoring numeric parity, demo reachability, and demo purity.
"""
import ast
import json
import math
import pathlib
import re
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import belt_tensioner_idler_draw_and_locknut as t  # noqa: E402
from labkit import PartState  # noqa: E402
from yam_arena import station  # noqa: E402

PARK = [np.array([0.20, 0.28, t.TOP + 0.175]), np.array([0.20, -0.28, t.TOP + 0.175])]


def frame(draw=None, advance=None, spin=None, dx=0.0, dy=0.0, tilt=0.0, clamped=True):
    """Build a synthetic actuator_frame PartState with draw, clamp, and spin joint values."""
    pos = np.array([t.FX + dx, t.FY + dy, t.TOP])
    half = math.radians(tilt) / 2.0
    quat = np.array([math.sin(half), 0.0, 0.0, math.cos(half)])
    if advance is None:
        advance = -t.NUT_GAP if clamped else 0.0
    if spin is None:
        spin = -math.radians(t.CLAMP_TURN + 30.0) if clamped else 0.0
    return PartState(
        pos=pos,
        quat_xyzw=quat,
        lin_vel=np.zeros(3),
        ang_vel=np.zeros(3),
        low=pos - 0.20,
        high=pos + 0.20,
        joints={
            "draw": t.SHIM_STOP if draw is None else draw,
            "clamp": advance,
            "spin": spin,
        },
    )


def gauge(at="seat"):
    """Setup gauge PartState in its frame pocket, back in its tray, or at a world point."""
    p = (
        {
            "seat": np.array([t.GX, t.GY, t.GZ]),
            "tray": np.array([t.TRAY_X, t.TRAY_Y, t.TOP + 0.072]),
            "far": np.array([t.FX - 0.130, t.FY + 0.138, t.GZ]),
        }[at]
        if isinstance(at, str)
        else np.asarray(at, float)
    )
    return PartState(
        pos=p,
        quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
        lin_vel=np.zeros(3),
        ang_vel=np.zeros(3),
        low=p - 0.03,
        high=p + 0.03,
        joints={},
    )


def sc(monkeypatch, fr=None, gp=None, clamped=True, pads=PARK):
    """Evaluate pure end-state score with monkeypatched gripper TCP coordinates."""
    monkeypatch.setattr(t.mk, "tcps", lambda: list(pads))
    fr_state = fr if fr is not None else frame(clamped=clamped)
    return t.score({"actuator_frame": fr_state, "setup_gauge": gp or gauge("tray")})


# -- constructed states & pure physical predicates ---------------------------------------------------------------------

def test_start_state_is_slack_and_gauge_in_tray():
    fr = frame(draw=0.0, advance=0.0, spin=0.0, clamped=False)
    k = t.components({"actuator_frame": fr, "setup_gauge": gauge("tray")})
    assert k["draw"] == 0.0 and not k["in_window"]
    assert not t.seated(gauge("tray").pos, fr.pos, fr.quat_xyzw)
    assert t.seated(gauge("seat").pos, fr.pos, fr.quat_xyzw)
    assert k["frame_ok"]


def test_clamp_seat_needs_half_a_turn_and_contact():
    assert t.clamp_seated(-math.radians(t.CLAMP_TURN), t.NUT_GAP)
    assert not t.clamp_seated(-math.radians(t.CLAMP_TURN - 5.0), t.NUT_GAP)
    assert not t.clamp_seated(-math.radians(t.CLAMP_TURN), 0.0003)


def test_draw_window_bounds():
    assert t.DRAW_WINDOW == (0.018, 0.024)
    assert t.DRAW_WINDOW[0] <= t.SHIM_STOP <= t.DRAW_WINDOW[1]
    assert abs(t.SPRING_N - 3.36) < 1e-6
    assert abs(t.K_SPRING * t.SHIM_STOP - t.SPRING_N) < 0.02


# -- score table & pure end-state transitions --------------------------------------------------------------------------

def test_ideal_completed_and_released_scores_one(monkeypatch):
    assert sc(monkeypatch, frame(clamped=True), gauge("tray")) == 1.0


def test_held_in_window_unclamped_scores_04(monkeypatch):
    assert sc(monkeypatch, frame(clamped=False), gauge("tray")) == t.S_HELD


def test_clamped_but_gauge_left_in_frame_scores_07(monkeypatch):
    assert sc(monkeypatch, frame(clamped=True), gauge("seat")) == t.S_SPILL


def test_clamped_but_gauge_far_from_tray_scores_07(monkeypatch):
    assert sc(monkeypatch, frame(clamped=True), gauge("far")) == t.S_SPILL


def test_proximity_hover_still_grasped_cap(monkeypatch):
    """Either gripper within 45 mm of nut crown, draw post, or gauge keeps score < 1.0."""
    on_nut = [np.array([t.FX, t.FY + t.STUD_Y, t.TOP + t.COLLAR_Z0 + 0.010]), PARK[1]]
    s_nut = sc(monkeypatch, frame(clamped=True), gauge("tray"), pads=on_nut)
    assert s_nut == t.S_SPILL and s_nut < 1.0
    on_post = [PARK[0], np.array([t.FX + 0.004, t.FY + t.POST_Y, t.TOP + 0.080])]
    s_post = sc(monkeypatch, frame(clamped=True), gauge("tray"), pads=on_post)
    assert s_post == t.S_SPILL and s_post < 1.0
    on_gauge = [np.array([t.TRAY_X, t.TRAY_Y, t.TOP + 0.080]), PARK[1]]
    s_gauge = sc(monkeypatch, frame(clamped=True), gauge("tray"), pads=on_gauge)
    assert s_gauge == t.S_SPILL and s_gauge < 1.0


def test_fail_closed_when_tcps_missing(monkeypatch):
    """Empty TCP readout in a unit test fails closed (clear=False -> capped at S_SPILL < 1.0)."""
    monkeypatch.setattr(t.mk, "tcps", lambda: [])
    monkeypatch.setattr(t.labkit, "env", lambda: None)
    s = t.score({"actuator_frame": frame(clamped=True), "setup_gauge": gauge("tray")})
    assert s == t.S_SPILL and s < 1.0


# -- negative controls & shortcuts (each must score 0.0 or < 1.0) ------------------------------------------------------

def test_negative_noop_scores_zero(monkeypatch):
    """Shortcut: doing nothing leaves draw at 0 and scores 0.0."""
    assert sc(monkeypatch, frame(draw=0.0, clamped=False), gauge("tray")) == 0.0


def test_negative_push_frame_without_draw_scores_zero(monkeypatch):
    """Shortcut: shoving the unbolted sub-frame without tensioning the carriage scores 0.0."""
    assert sc(monkeypatch, frame(draw=0.0, dx=0.040, clamped=False), gauge("tray")) == 0.0
    assert sc(monkeypatch, frame(draw=t.SHIM_STOP, dx=0.040, clamped=True), gauge("tray")) == 0.0


def test_negative_release_unclamped_spring_recoil_scores_zero(monkeypatch):
    """Shortcut: releasing carriage without clamping lets return spring recoil below 18 mm window."""
    assert sc(monkeypatch, frame(draw=0.005, clamped=False), gauge("tray")) == 0.0


def test_negative_overdraw_and_tilt_void_scores_zero(monkeypatch):
    """Over-drawing past 24 mm or tilting the frame > 8 deg voids the tension reading (0.0)."""
    assert sc(monkeypatch, frame(draw=0.030, clamped=True), gauge("tray")) == 0.0
    assert sc(monkeypatch, frame(tilt=20.0, clamped=True), gauge("tray")) == 0.0


def test_occlusion_top_camera_clear_when_parked(monkeypatch):
    """Parked grippers clear top-camera line of sight; hand blocking target corridor fails < 1.0."""
    cam = np.array([0.35, 0.0, 1.35])
    targets = [t.CROWN, np.array([t.FX + 0.004, t.FY + t.POST_Y, t.TOP + 0.0725]), gauge("tray").pos]
    for tgt in targets:
        seg = tgt - cam
        seg_n2 = float(np.dot(seg, seg))
        for p in PARK:
            u = float(np.clip(np.dot(p - cam, seg) / seg_n2, 0.0, 1.0))
            perp = float(np.linalg.norm(p - (cam + u * seg)))
            assert perp >= 0.06, f"parked TCP {p} occludes line-of-sight to {tgt} ({perp:.3f} m)"
    occluding = [targets[0] + np.array([0.0, 0.0, 0.02]), PARK[1]]
    assert sc(monkeypatch, frame(clamped=True), gauge("tray"), pads=occluding) < 1.0


# -- schema, numeric parity & purity -----------------------------------------------------------------------------------

def test_possible_scores_matches_documented_tiers(monkeypatch):
    tj = json.loads((HERE / "task.json").read_text())
    assert "Possible scores:" in tj["scoring"]
    assert "Possible scores:" in (t.score.__doc__ or "")
    doc_vals = {float(v) for v in re.findall(r"(?<![\d.])(\d\.\d+|[01])(?![\d.])", tj["scoring"])} & {0.0, 0.4, 0.7, 1.0}
    enumerated = {
        sc(monkeypatch, frame(draw=0.0, clamped=False), gauge("tray")),
        sc(monkeypatch, frame(clamped=False), gauge("tray")),
        sc(monkeypatch, frame(clamped=True), gauge("seat")),
        sc(monkeypatch, frame(clamped=True), gauge("tray")),
    }
    assert enumerated == {0.0, t.S_HELD, t.S_SPILL, t.S_FULL} == doc_vals


def test_scoring_text_numbers_appear_in_the_scorer():
    tj = json.loads((HERE / "task.json").read_text())
    src = (HERE / "belt_tensioner_idler_draw_and_locknut.py").read_text()
    lits = {
        round(float(n.value), 9)
        for n in ast.walk(ast.parse(src))
        if isinstance(n, ast.Constant) and isinstance(n.value, (int, float)) and not isinstance(n.value, bool)
    }
    units = {"mm": 1e-3, "cm": 1e-2, "m": 1.0, "n": 1.0, "s": 1.0, "deg": math.pi / 180.0, "g": 1.0, "kg": 1.0}
    miss = []
    for num, unit in re.findall(r"(\d+(?:\.\d+)?)\s*(mm|cm|m|N|s|deg|g|kg)\b", tj["scoring"]):
        v = float(num) * units[unit.lower()]
        if not any(abs(l - v) <= 1e-6 + 1e-6 * abs(v) for l in lits) and not any(
            abs(l - v / units[unit.lower()]) <= 1e-9 for l in lits
        ):
            miss.append(f"{num} {unit}")
    assert not miss, f"scoring numbers missing from the scorer: {miss}"


def test_plan_kinematic_reach():
    from pitch_demo import check_plan

    worst = check_plan(t.demo(), station.BASES)
    for arm, (mm, deg) in worst.items():
        assert mm <= 6.0, f"{arm} arm out of reach by {mm} mm"
        assert deg <= 6.0, f"{arm} wrist out of range by {deg} deg"


def test_demo_reads_constants_only_and_scorer_is_stateless():
    src = (HERE / "belt_tensioner_idler_draw_and_locknut.py").read_text()
    tree = ast.parse(src)
    fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "demo")
    names = {n.id.lower() for n in ast.walk(fn) if isinstance(n, ast.Name) and isinstance(n.ctx, ast.Load)}
    assert not names & {"objects", "env", "state", "scene", "obs", "trial"}
    assert not [
        n
        for n in ast.walk(fn)
        if isinstance(n, ast.Attribute) and n.attr.lower().startswith(("get_", "query", "read_"))
    ]
    for fn_name in ("components", "score"):
        sc_fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == fn_name)
        sc_names = {n.id for n in ast.walk(sc_fn) if isinstance(n, ast.Name)}
        assert not sc_names & {"_EP", "MEMORY", "_TEST"}


def test_demo_holds_clear_at_the_end():
    plan = t.demo()
    spots = [np.array([t.FX + 0.004, t.FY + t.POST_Y, t.TOP + 0.0725]), t.CROWN, np.array([t.TRAY_X, t.TRAY_Y, t.TOP + 0.072])]
    for arm in ("left", "right"):
        last = plan[arm][-1]
        assert last.finger == t.FINGER_OPEN
        assert last.duration >= 3.0, f"{arm} arm must hold clear for >= 3 s"
        assert min(float(np.linalg.norm(np.asarray(last.pos) - s)) for s in spots) >= t.HAND_CLEAR
    assert sum(w.duration for w in plan["left"]) >= 30.0
