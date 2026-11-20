"""Offline checks of task.py (no GPU, no Isaac Sim): the scale against the pendulum's physics, every scoring outcome from
fabricated poses (including attempts to score without doing the task), the runtime's ending, the demo's reach, and that
the task's text states the numbers the code uses.

    PYTHONDONTWRITEBYTECODE=1 python -m pytest -p no:cacheprovider test_scoring.py
"""

from __future__ import annotations

import importlib
import json
import math
import pathlib
import sys

import numpy as np
import pytest

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
T = importlib.import_module("task")

I3 = np.eye(3)
CASE_POS = np.array([T.CASE_XY[0], T.CASE_XY[1], T.TOP])
PIVOT = CASE_POS + np.array([T.X_ROD, 0.0, T.Z_PIV])
FAR = np.array([[0.0, 0.40, 1.10], [0.0, -0.40, 1.10]])          # hands well away from everything
DT = 1.0 / T.CONTROL_HZ


def rx(deg):
    a = math.radians(deg)
    return np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])


def rz(deg):
    a = math.radians(deg)
    return np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])


class Episode:
    """Feeds fabricated steps to the task as the runtime does: observe() (the simulator hook), then success() and
    score(); keeps the runtime's own 1 s success count."""

    def __init__(self):
        T._reset_state()
        self.t = 0
        self.top = T.SL_START
        self.case = (CASE_POS.copy(), I3)
        self.runtime_held = 0
        self.ended = False
        self.log = []

    def step(self, swing=0.0, top=None, hands=FAR, case=None, n=1, wave=None):
        """``wave``: a function of time (s) giving the swing angle (degrees), instead of a fixed ``swing``."""
        if top is not None:
            self.top = top
        if case is not None:
            self.case = case
        out = None
        for _ in range(n):
            th = wave(self.t * DT) if wave is not None else swing
            case_pos, case_r = self.case
            rod_r = case_r @ rx(th)
            rod_pos = case_pos + case_r @ np.array([T.X_ROD, 0.0, T.Z_PIV])
            slider = rod_pos + rod_r @ np.array([0.0, 0.0, self.top - T.SL_H / 2])
            T.observe(case_pos, case_r, rod_pos, rod_r, slider, hands)
            ok = T.success(None)
            s = T.score(None)
            assert not ok or s == 1.0, "success() true while score() is below 1.0"
            self.runtime_held = self.runtime_held + 1 if ok else 0
            self.ended = self.runtime_held >= T.HOLD_STEPS
            self.log.append((s, ok, self.ended))
            self.t += 1
            out = s
            if self.ended:
                break
        return out


def ticking(b, amp=25.0, phase=0.3):
    """A free swing at ``b`` ticks per minute (two per period), ``amp`` degrees to each side."""
    return lambda t: amp * math.sin(2 * math.pi * b / 120.0 * t + phase)


SET = T.MARK_Z[60]
FULL = T.WINDOW + T.HOLD_STEPS + 5


# -- the scale and the physics ------------------------------------------------------------------------------------------

def test_scale_runs_40_at_the_top_to_208_at_the_bottom():
    z = [T.MARK_Z[b] for b in T.MARKS]
    assert all(a > b for a, b in zip(z, z[1:]))
    assert T.SL_TOP_MIN < T.MARK_Z[208] and T.MARK_Z[40] < T.SL_TOP_MAX <= T.ROD_L
    assert abs(T.bpm(T.MARK_Z[60]) - 60.0) < 1e-6 and abs(T.bpm(T.MARK_Z[120]) - 120.0) < 1e-6
    assert T.SL_START == pytest.approx(T.MARK_Z[120])
    gap_58, gap_63 = T.MARK_Z[58] - T.MARK_Z[60], T.MARK_Z[60] - T.MARK_Z[63]
    assert 0.0008 < gap_58 < 0.0025 and 0.0012 < gap_63 < 0.003     # the lines either side of 60: 1-2 mm away
    assert 0.09 < T.MARK_Z[40] - T.MARK_Z[208] < 0.11                # a 10 cm scale


def test_period_formula_matches_integrating_the_pendulum():
    """bpm() (the elliptic-integral period of the compound pendulum) against a direct integration of
    I theta'' = -k sin(theta), released from 30 degrees."""
    for b in (60, 120, 208, 40):
        inertia, k = T.pendulum(T.MARK_Z[b] - T.SL_H / 2)
        th, w, t, dt, last, passes = math.radians(T.A_CAL), 0.0, 0.0, 1e-4, None, []
        while len(passes) < 5:
            a = -k / inertia * math.sin(th)
            w += a * dt
            th_new = th + w * dt
            if th > 0 >= th_new or th < 0 <= th_new:
                passes.append(t + dt * th / (th - th_new))
            th, t = th_new, t + dt
        measured = 60.0 * (len(passes) - 1) / (passes[-1] - passes[0])
        assert measured == pytest.approx(b, rel=2e-3)


def test_heavier_than_the_weight_above_the_pivot():
    """The pendulum stands upright on its own wherever the weight is (the fixed weight below the pivot wins)."""
    for top in (T.SL_TOP_MIN, T.SL_TOP_MAX):
        assert T.pendulum(top - T.SL_H / 2)[1] > 0


def test_a_notch_at_every_mark_and_a_ridge_between_every_two():
    """Every mark has a V notch whose bottom is on the rod's face (two flanks meeting there), and the tooth seated in it
    touches both flanks within their length, so it rests with the weight's top edge exactly on that mark's line."""
    r = T.ridges()
    assert len(r) == len(T.MARKS) + 1                           # one more beyond each end notch
    zs = sorted(T.NOTCH_Z.values())
    for (lo, hi, h), a, b in zip(r[1:-1], zs, zs[1:]):
        assert (lo, hi) == (a, b) and h == pytest.approx((hi - lo) / 2 * math.tan(math.radians(T.FLANK_DEG)))
    reach = T.TOOTH_R * math.sin(math.radians(T.FLANK_DEG))     # along the rod, from the notch to where it touches
    assert reach < min(hi - lo for lo, hi, _ in r) / 2
    assert T.STANDOFF > max(h for *_, h in r) + 0.0005            # the weight's body clears every ridge


def test_click_over_force_and_holding():
    assert 2.5 < T.CLICK_FORCE < 3.5                               # about 3 N to push the weight to the next notch
    assert T.CLICK_FORCE > 10 * T.SL_M * 9.81                      # its own weight cannot move it
    assert T.CLIP_MU < math.tan(math.radians(T.FLANK_DEG))         # it slides down a flank into the notch by itself


def test_notch_at_is_the_nearest_line():
    for b in T.MARKS:
        assert T.notch_at(T.MARK_Z[b]) == b


def test_masses_add_up_to_the_metronome():
    total = sum(o.mass for o in T.OBJECTS)
    assert total == pytest.approx(0.505, abs=0.01)


# -- the start ------------------------------------------------------------------------------------------------------

def test_start_scores_zero():
    ep = Episode()
    assert ep.step(n=200) == 0.0


def test_started_at_the_old_tempo_scores_a_quarter():
    ep = Episode()
    assert ep.step(wave=ticking(120), n=FULL) == 0.25


# -- the weight ----------------------------------------------------------------------------------------------------

def test_weight_set_but_not_started_scores_a_half():
    ep = Episode()
    assert ep.step(top=SET, n=FULL) == 0.5


@pytest.mark.parametrize("off", [-1, 1])
def test_held_short_of_the_next_notch_still_counts_as_the_60_notch(off):
    """Held (not clicked in) a little off the 60 line, the weight is scored by the nearest line: it falls back into the
    60 notch when let go."""
    lo, hi = T.MARK_Z[63], T.MARK_Z[58]
    top = SET + off * 0.45 * ((hi - SET) if off > 0 else (SET - lo))
    ep = Episode()
    assert ep.step(top=top, n=5) == 0.5


@pytest.mark.parametrize("mark,score", [(58, 0.25), (63, 0.25), (56, 0.25), (66, 0.25), (52, 0.25), (69, 0.25),
                                        (50, 0.0), (72, 0.0)])
def test_weight_off_60(mark, score):
    ep = Episode()
    assert ep.step(top=T.MARK_Z[mark], n=5) == score


# -- the full score ---------------------------------------------------------------------------------------------------

def test_set_and_ticking_on_its_own_scores_one_and_ends_the_episode():
    ep = Episode()
    ep.step(top=SET, n=20)
    s = ep.step(wave=ticking(60), n=FULL)
    assert s == 1.0 and ep.ended
    first = next(i for i, (sc, _, _) in enumerate(ep.log) if sc == 1.0)
    assert first >= 20 + 3 * T.CONTROL_HZ      # not before four centre passes, 1 s apart


def test_full_score_needs_the_whole_window():
    ep = Episode()
    ep.step(top=SET, n=20)
    assert ep.step(wave=ticking(60), n=50) == 0.5             # 2.5 s: three passes at most


@pytest.mark.parametrize("mark", [58, 63])
def test_the_next_notch_is_not_full_score(mark):
    ep = Episode()
    assert ep.step(top=T.MARK_Z[mark], wave=ticking(T.bpm(T.MARK_Z[mark], 25.0)), n=FULL) == 0.75 and not ep.ended


def test_swinging_near_60_scores_three_quarters():
    ep = Episode()
    top = T.MARK_Z[66]
    assert ep.step(top=top, wave=ticking(T.bpm(top, 25.0)), n=FULL) == 0.75 and not ep.ended


def test_tempo_out_of_range_is_not_full_score():
    """The weight read as set but the pendulum ticking at 70 (it cannot in the simulation; a check on the code)."""
    ep = Episode()
    assert ep.step(top=SET, wave=ticking(70), n=FULL) == 0.75


def test_small_swing_does_not_count_as_started():
    ep = Episode()
    assert ep.step(top=SET, wave=ticking(60, amp=12.0), n=FULL) == 0.5


def test_swing_dying_away_does_not_count():
    ep = Episode()
    assert ep.step(top=SET, wave=lambda t: 25.0 * math.exp(-t / 2.5) * math.sin(math.pi * t + 0.3), n=FULL) == 0.5


# -- hands off ------------------------------------------------------------------------------------------------------

def test_hand_within_5_cm_resets_the_window():
    ep = Episode()
    ep.step(top=SET, wave=ticking(60), n=T.WINDOW - 20)
    near = np.array([PIVOT + np.array([-0.045, 0.0, 0.10])])     # 4.5 cm in front of the case's front face
    assert T.clearance(near, CASE_POS, I3, PIVOT, I3, PIVOT) < T.CLEAR
    ep.step(wave=ticking(60), hands=near)
    assert ep.step(wave=ticking(60), n=T.WINDOW - 1) == 0.5
    assert ep.step(wave=ticking(60), n=1) == 1.0


def test_driving_the_pendulum_by_hand_scores_no_more_than_set():
    """A hand holding the weight and swinging the pendulum at 60 is always within 5 cm of it."""
    ep = Episode()
    wave = ticking(60)

    def hand(t):
        th = wave(t)
        return np.array([PIVOT + rx(th) @ np.array([-0.015, 0.0, SET - T.SL_H / 2])])

    for _ in range(FULL):
        ep.step(top=SET, swing=wave(ep.t * DT), hands=hand(ep.t * DT))
    assert ep.log[-1][0] == 0.5 and not any(ok for _, ok, _ in ep.log)


def test_hand_5_cm_beside_the_swinging_weight_is_not_clear():
    th = 25.0
    rod_r = rx(th)
    slider = PIVOT + rod_r @ np.array([0.0, 0.0, SET - T.SL_H / 2])
    side = slider + rod_r @ np.array([-0.006, -(T.SL_W / 2 + 0.04), 0.0])
    assert T.clearance(np.array([side]), CASE_POS, I3, PIVOT, rod_r, slider) < T.CLEAR


# -- the case ---------------------------------------------------------------------------------------------------------

def test_tipping_the_metronome_scores_zero_for_good():
    ep = Episode()
    ep.step(top=SET, n=10)
    ep.step(case=(CASE_POS, rx(25.0)), n=3)
    assert ep.step(case=(CASE_POS, I3), wave=ticking(60), n=FULL) == 0.0


@pytest.mark.parametrize("case", [(CASE_POS + np.array([0.025, 0.0, 0.0]), I3), (CASE_POS, rz(12.0)),
                                  (CASE_POS, rx(6.0))])
def test_moved_turned_or_held_tilted_scores_zero(case):
    ep = Episode()
    assert ep.step(top=SET, case=case, wave=ticking(60), n=FULL) == 0.0


def test_moved_a_little_still_counts():
    ep = Episode()
    ep.step(top=SET, case=(CASE_POS + np.array([0.015, 0.0, 0.0]), rz(8.0)), wave=ticking(60), n=FULL)
    assert ep.ended


# -- the text states the code's numbers --------------------------------------------------------------------------------

def test_task_json_fields_and_numbers():
    j = json.loads((HERE / "task.json").read_text())
    limits = {"title": 120, "context": 1000, "scores": 300, "fidelity": 500, "setup": 4000, "scoring": 4000,
              "instruction": 1000, "run_limit": 200}
    for k, n in limits.items():
        assert len(j[k]) <= n, k
    assert len(", ".join(j["keywords"])) <= 200
    assert j["instruction"] == T.INSTRUCTION
    assert j["category"] == "Societal relatability"
    sc = j["scoring"]
    for s in ("60 notch", "52", "69", "15 degrees", "5 cm", "5 s", "56", "64", "20 degrees", "5 degrees", "2 cm",
              "10 degrees", "four", "1 s"):
        assert s in sc, s
    for s in ("22.5", "500 g", "70 g", "20 g", "3 N", "120", "notch"):
        assert s in j["setup"], s


def test_task_text_has_no_identifiers():
    for f in ("task.json", "README.md"):
        text = (HERE / f).read_text().lower()
        for bad in ("/users/", "/home/", "c:\\"):
            assert bad not in text, (f, bad)


# -- the demo -----------------------------------------------------------------------------------------------------------

def test_demo_reaches_every_pose_and_fits_the_run_limit():
    demo = importlib.import_module("demo")
    from yam_arena import station

    class _Ctx:
        left = demo.Arm("left", station.BASES["left"])
        right = demo.Arm("right", station.BASES["right"])

    ctx = _Ctx()
    steps = sum(1 for _ in demo.script(T, ctx))
    assert ctx.left.worst_mm < 1.0
    assert steps + T.WINDOW + T.HOLD_STEPS + 40 < 1600



def test_free_swing_with_the_simulations_own_slow_decay_counts():
    """Released at 30 degrees and dying away as the simulated pendulum does (its swing falls by about 1/e in 13 s):
    still over 15 degrees at the end of the window, so it scores."""
    ep = Episode()
    ep.step(top=SET, n=20)
    ep.step(wave=lambda t: 30.0 * math.exp(-t / 13.0) * math.sin(math.pi * t + 0.3), n=FULL)
    assert ep.ended
