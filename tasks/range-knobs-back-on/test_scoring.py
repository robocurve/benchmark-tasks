"""Offline checks of task.py (no GPU, no Isaac Sim): every scoring outcome from fabricated poses (including attempts to
score without doing the task), the runtime's ending, the geometry that makes the valve lock and the knob's D bore
work (sampled from the very shapes the simulator is given), the layout, the demo's reach, and that the task's text
states the numbers the code uses.

    python -m pytest -p no:cacheprovider test_scoring.py
"""

from __future__ import annotations

import importlib
import json
import math
import pathlib
import re
import sys

import numpy as np
import pytest

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from knobkit import BoxShape, LoftShape, rotation  # noqa: E402

T = importlib.import_module("task")
MM = 1e-3
FAR = np.array([[0.0, 0.40, 1.10], [0.0, -0.40, 1.10]])        # grippers well away from everything


def rz(deg):
    a = math.radians(deg)
    return np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1.0]])


def rx(deg):
    a = math.radians(deg)
    return np.array([[1.0, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])


def knob_at(i, gap, pointer=0.0, dx=0.0, r=None):
    """Knob on stem i: its mouth ``gap`` above the panel, its pointer ``pointer`` deg from OFF."""
    sx, sy = T.STEMS[i]
    return (np.array([sx + dx, sy, T.PANEL_Z + gap]), rz(pointer) if r is None else r)


def seated(i, **kw):
    return knob_at(i, T.SEATED_GAP, **kw)


def start_knobs():
    return [(np.array([x, y, T.PANEL_Z + 0.0005]), rz(a)) for x, y, a in T.KNOB_START]


def rest_stems():
    return [(np.array([sx, sy, T.PANEL_Z]), np.eye(3)) for sx, sy in T.STEMS]


class Episode:
    """Feeds fabricated steps to the task as the runtime does: observe() (the simulator hook), then success() and
    score(); keeps the runtime's own 1 s success count."""

    def __init__(self):
        T._reset_state()
        self.knobs = start_knobs()
        self.stems = rest_stems()
        self.grippers = FAR
        self.runtime_held = 0
        self.ended = False
        self.log = []

    def step(self, knobs=None, stems=None, grippers=None, n=1):
        if knobs is not None:
            self.knobs = list(knobs)
        if stems is not None:
            self.stems = list(stems)
        if grippers is not None:
            self.grippers = grippers
        s = None
        for _ in range(n):
            T.observe(self.knobs, self.stems, self.grippers)
            ok = T.success(None)
            s = T.score(None)
            assert not ok or s == 1.0, "success() true while score() is below 1.0"
            self.runtime_held = self.runtime_held + 1 if ok else 0
            self.ended = self.ended or self.runtime_held >= T.HOLD_STEPS
            self.log.append((s, ok, self.ended))
        return s


def turned_stem(i, deg, depth=0.0):
    sx, sy = T.STEMS[i]
    return (np.array([sx, sy, T.PANEL_Z - depth]), rz(deg))


# -- scoring outcomes ----------------------------------------------------------------------------------------------

def test_start_scores_zero():
    assert Episode().step(n=30) == 0.0


def test_one_knob_seated_scores_045_and_both_090_while_held():
    e = Episode()
    assert e.step(knobs=[seated(0), start_knobs()[1]], n=5) == pytest.approx(0.45)
    near = np.array([[T.STEMS[0][0], T.STEMS[0][1], T.PANEL_Z + 0.04]])     # a fingertip on the knob's fin
    assert e.step(knobs=[seated(0), seated(1)], grippers=near, n=60) == pytest.approx(0.9)
    assert not e.ended


def test_both_seated_let_go_and_still_scores_full_after_a_second_and_the_episode_ends_at_full():
    e = Episode()
    e.step(knobs=[seated(0), seated(1)], n=4 * T.HOLD_STEPS)
    first = next(k for k, (s, _, _) in enumerate(e.log) if s == 1.0)
    assert first == 2 * T.HOLD_STEPS - 1              # 1 s to show it is still, then 1 s held
    assert all(s == pytest.approx(0.9) for s, _, _ in e.log[:first])
    ended = next(k for k, (_, _, end) in enumerate(e.log) if end)
    assert ended == first + T.HOLD_STEPS - 1 and all(s == 1.0 for s, _, _ in e.log[first:])


def test_episode_can_only_end_at_full_score():
    e = Episode()
    e.step(knobs=[seated(0), seated(1)], n=40)
    e.step(knobs=[seated(0), knob_at(1, 0.0145, 30.0)], n=40)      # one knob knocked back up onto its tip
    assert all(s == 1.0 for s, ok, _ in e.log if ok)
    assert e.log[-1][0] == pytest.approx(0.45)


def test_knob_on_the_flat_but_not_home_scores_02():
    e = Episode()
    assert e.step(knobs=[knob_at(0, T.ENGAGED_GAP), knob_at(1, T.ENGAGED_GAP, pointer=-3.0)], n=5) == \
        pytest.approx(0.4)


def test_knob_resting_on_the_tip_unaligned_scores_nothing():
    assert Episode().step(knobs=[knob_at(0, 0.0145, 30.0), knob_at(1, 0.0145, -60.0)], n=5) == 0.0


def test_knob_beside_its_stem_or_tilted_scores_nothing():
    e = Episode()
    assert e.step(knobs=[seated(0, dx=2 * MM), knob_at(1, 0.004, r=rz(0) @ rx(8))], n=5) == 0.0


def test_seated_but_pointer_off_off_is_not_seated():
    e = Episode()
    assert e.step(knobs=[seated(0, pointer=7.0), seated(1, pointer=12.0)], n=5) == pytest.approx(0.2)


def test_any_stem_turned_off_off_zeroes_the_whole_trial_even_if_turned_back():
    e = Episode()
    e.step(knobs=[seated(0), seated(1)], n=5)
    e.step(stems=[turned_stem(0, 12.0, depth=T.UNLOCK), rest_stems()[1]], n=2)   # the gas was on
    assert e.log[-1][0] == 0.0
    e.step(stems=rest_stems(), grippers=FAR, n=80)
    assert all(s == 0.0 for s, _, _ in e.log[5:]) and not e.ended


def test_stem_pushed_in_but_not_turned_is_not_a_failure():
    e = Episode()
    assert e.step(knobs=[seated(0), seated(1)], stems=[turned_stem(0, 1.4, depth=T.TRAVEL), rest_stems()[1]],
                  n=5) == pytest.approx(0.9)


def test_hands_within_2_cm_never_score_full():
    sx, sy = T.STEMS[1]
    near = np.array([[sx + T.KNOB_R + 0.015, sy, T.PANEL_Z + 0.02]])
    e = Episode()
    e.step(knobs=[seated(0), seated(1)], grippers=near, n=80)
    assert max(s for s, _, _ in e.log) < 1.0
    assert e.step(grippers=np.array([[sx + T.KNOB_R + 0.021, sy, T.PANEL_Z + 0.02]]), n=T.HOLD_STEPS + 1) == 1.0


def test_moving_knob_never_scores_full():
    e = Episode()
    for k in range(80):
        e.step(knobs=[seated(0, pointer=0.4 * math.sin(k / 3.0)), seated(1)])    # pointer tip rocking about 0.6 mm
    assert max(s for s, _, _ in e.log) == 1.0                                    # under 1 mm: still
    e3 = Episode()
    for k in range(80):
        e3.step(knobs=[knob_at(0, T.SEATED_GAP - 0.0015 * (k % 2)), seated(1)])  # 1.5 mm: not still
    assert max(s for s, _, _ in e3.log) < 1.0


def test_both_knobs_on_one_stem_or_one_knob_twice_cannot_count_twice():
    e = Episode()
    assert e.step(knobs=[seated(0), seated(0)], n=40) == pytest.approx(0.45)


def test_knob_upside_down_or_off_the_cooktop_scores_nothing():
    sx, sy = T.STEMS[0]
    e = Episode()
    assert e.step(knobs=[(np.array([sx, sy, T.PANEL_Z + 0.004]), rx(180)), (np.array([0.3, -0.3, T.TOP]), rz(0))],
                  n=5) == 0.0


def test_success_only_at_full_score():
    e = Episode()
    e.step(knobs=[seated(0), knob_at(1, T.ENGAGED_GAP)], n=60)
    assert not any(ok for _, ok, _ in e.log)


# -- the mechanism's geometry, from the shapes the simulator is given ----------------------------------------------

def _lip_boxes(i):
    ck = T.OBJECTS[0]
    lx, ly = T.STEMS[i][0] - T.COOK_C[0], T.STEMS[i][1]
    return [b for b in ck.shapes if isinstance(b, BoxShape) and b.collide and b.centre[2] < -0.001
            and abs(b.centre[0] - lx) < 0.012 and abs(b.centre[1] - ly) < 0.012], lx, ly


def _lug_hits_lip(turn, push, i=0):
    boxes, lx, ly = _lip_boxes(i)
    lug = T.OBJECTS[1 + i].shapes[1]
    g = np.stack(np.meshgrid(*[np.linspace(c - s / 2, c + s / 2, n) for c, s, n in zip(lug.centre, lug.size,
                                                                                         (26, 9, 9))]), -1)
    p = g.reshape(-1, 3) @ rz(turn).T + np.array([lx, ly, -push])
    for b in boxes:
        loc = (p - np.array(b.centre)) @ rotation(b.quat())
        if np.all(np.abs(loc) < np.array(b.size) / 2 - 1e-6, axis=1).any():
            return True
    return False


def test_stem_at_rest_sits_in_its_notch_without_touching_it():
    for i in range(2):
        assert not _lug_hits_lip(0.0, 0.0, i)


def test_stem_cannot_turn_until_pushed_past_the_unlock_depth():
    free = [t for t in np.arange(0.0, T.MAX_TURN_DEG, 0.5) if not _lug_hits_lip(t, 0.0)]
    assert max(free) <= 2.0                                   # only the notch's play
    assert any(_lug_hits_lip(t, T.UNLOCK - 0.3 * MM) for t in range(5, 300, 5))
    assert not any(_lug_hits_lip(t, T.UNLOCK + 0.01 * MM) for t in range(0, 300, 2))
    assert T.UNLOCK_FORCE == pytest.approx(10.2)


def _stem_poly():
    return np.array(T.d_polygon(n=400))


def _flat_penetration(turn_deg):
    """How far the stem's section, turned ``turn_deg`` against the knob, reaches into the knob's moulded flat (m)."""
    knob = T.OBJECTS[3]
    flat = [s for s in knob.shapes if isinstance(s, LoftShape) and len(s.levels) == 3 and s.levels[0][0] > 1 * MM][0]
    x0 = min(x for x, _ in flat.levels[1][1])
    ymax = max(y for _, y in flat.levels[1][1])
    q = _stem_poly() @ rz(-turn_deg)[:2, :2].T
    m = np.abs(q[:, 1]) <= ymax
    return max(0.0, float(q[m, 0].max() - x0))


def test_knob_drops_onto_the_flat_only_within_about_5_degrees():
    window = [d for d in np.arange(-40, 40.01, 0.1) if _flat_penetration(d) <= 0]
    assert 3.0 <= max(window) <= 7.0 and -7.0 <= min(window) <= -3.0
    assert _flat_penetration(20.0) > 0.4 * MM and _flat_penetration(180.0) > 1.0 * MM


def test_bore_and_clip_fit():
    assert 0.1 * MM < T.BORE_R - T.STEM_R < 0.4 * MM                 # the round bore clears the stem
    assert T.STEM_FLAT - T.CLIP_X0 > T.BORE_R - T.STEM_R             # the clip still presses once the knob is pushed over
    assert T.SEATED_GAP == pytest.approx(4 * MM) and T.ENGAGED_GAP == pytest.approx(10 * MM)
    assert T.SEATED_GAP < T.GAP_SEATED < T.ENGAGED_GAP < T.GAP_ENGAGED < T.STEM_OUT - T.FLAT_Z[0]   # unaligned: 14.5 mm


def test_layout():
    names = [o.name for o in T.OBJECTS]
    assert names == ["cooktop", "stem_0", "stem_1", "knob_0", "clip_0", "knob_1", "clip_1"]
    assert abs(T.STEMS[0][1] - T.STEMS[1][1]) >= 0.09                # grippers over the two stems stay 9 cm apart
    for x, y, _ in T.KNOB_START:
        assert T.COOK_X[0] + T.KNOB_R < x and abs(y) + T.KNOB_R < T.COOK_Y
        for sx, sy in T.STEMS:
            assert math.hypot(x - sx, y - sy) > T.KNOB_R + 0.02
        for bx, by in T.BURNERS:
            assert math.hypot(x - bx, y - by) > T.KNOB_R + 0.05
    assert T.KNOB_R * 2 < 0.094                                     # the knob fits the 94 mm gripper
    for o in T.OBJECTS:
        if not o.kinematic:
            assert o.mass < 0.05


# -- the demo -----------------------------------------------------------------------------------------------------

def test_demo_reaches_every_move_and_fits_the_run_limit():
    demo = importlib.import_module("demo")
    from yam_arena import ik, station
    from yam_arena.constants import READY

    plan = demo.offline_plan(T)
    for side in ("left", "right"):
        q = np.array(READY, float)
        for pos, rot, label in plan[side]:
            q, pe, rerr = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), q, base_pos=station.BASES[side], iters=200)
            assert pe < 1e-3 and rerr < 0.01, (side, label, pe, rerr)
    run_limit = int(re.sub(r"\D", "", json.loads((HERE / "task.json").read_text())["run_limit"].split("steps")[0]))
    assert plan["steps"] + 3 * T.HOLD_STEPS < run_limit


def test_demo_reports_the_real_score():
    src = (HERE / "demo.py").read_text()
    assert 'info.get("score"' in src and "task_score\": final" in src
    assert "= 1.0" not in src.split("def run(")[1].replace("score\", 0.0) or 0.0) >= 1.0", "")


# -- the text ------------------------------------------------------------------------------------------------------

def test_task_json_states_the_numbers():
    j = json.loads((HERE / "task.json").read_text())
    limits = {"title": 120, "context": 1000, "scores": 300, "keywords": 200, "fidelity": 500, "setup": 4000,
              "scoring": 4000, "instruction": 1000, "run_limit": 200}
    for k, n in limits.items():
        v = j[k] if isinstance(j[k], str) else ", ".join(j[k])
        assert len(v) <= n, (k, len(v))
    assert j["instruction"] == T.INSTRUCTION
    sc = j["scoring"]
    for s in ("10 degrees", "1.5 mm", "3 degrees", "5 mm", "5 degrees", "12 mm", "10 degrees", "0.45", "0.2",
              "2 cm", "1 mm", "1 s"):
        assert s in sc, s
    st = j["setup"]
    for s in ("6.35 mm", "18 mm", "100 mm", "44 mm", "14 mm", "3.5 mm", "10 N"):
        assert s in st, s
    assert T.GAS_ON_DEG == 10 and T.CENTRE == 1.5 * MM and T.GAP_SEATED == 5 * MM and T.GAP_ENGAGED == 12 * MM


def test_no_identifying_text_or_bytecode():
    for f in ("task.json", "README.md"):
        text = (HERE / f).read_text()
        assert "/Users/" not in text and "/home/" not in text and "@" not in text, f
    assert not list(HERE.rglob("*.pyc")) and not (HERE / "__pycache__").exists()
