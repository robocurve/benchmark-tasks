"""Scoring and demo checks for trim-clip that run without the simulator: python -m pytest -q test_trim_clip.py"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest
from yam_arena import station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

import trim_clip as M

HERE = Path(__file__).resolve().parent
SHEET_TOP = M.TOP + M.SHEET_TOP / 1000
PANEL_TOP = M.TOP + M.PANEL_TOP / 1000


def _quat(yaw_deg=0.0, roll_deg=0.0):
    cy, sy = math.cos(math.radians(yaw_deg) / 2), math.sin(math.radians(yaw_deg) / 2)
    cr, sr = math.cos(math.radians(roll_deg) / 2), math.sin(math.radians(roll_deg) / 2)
    return np.array([cy * sr, sy * sr, sy * cr, cy * cr])


def panel(xy=M.PANEL_LINED, bottom=SHEET_TOP, yaw=0.0, roll=0.0, moving=False) -> ObjectState:
    """As the simulator reports it: the centre of its bounding box, which is the plate's centre in x and y."""
    top = bottom + (M.PANEL[2] + M.LIP_H) / 1000
    v = np.array([0.02, 0, 0]) if moving else np.zeros(3)
    return ObjectState(pos=np.array([*xy, (bottom + top) / 2]), quat_xyzw=_quat(yaw, roll), lin_vel=v,
                       ang_vel=np.zeros(3), low=np.array([0, 0, bottom]), high=np.array([0, 0, top]))


def clip(xy=M.HOLE_W, head_above=0.0, tilt=0.0, moving=False) -> ObjectState:
    """The clip with its head's underside ``head_above`` (m) over the panel's top."""
    underside = PANEL_TOP + head_above
    low, high = underside - M.CLIP_LEN / 1000, underside + M.HEAD_T / 1000
    v = np.array([0.02, 0, 0]) if moving else np.zeros(3)
    return ObjectState(pos=np.array([*xy, (low + high) / 2]), quat_xyzw=_quat(0, tilt), lin_vel=v,
                       ang_vel=np.zeros(3), low=np.array([0, 0, low]), high=np.array([0, 0, high]))


def state(p=None, c=None) -> dict:
    return {"panel": p or panel(), "clip": c or clip()}


def test_the_task_file_loads():
    task = load(HERE / "trim_clip.py")
    assert set(task.free_objects) == set(M.NAMES)


# -- scoring --------------------------------------------------------------------------------------------------------

def test_the_start_scores_0():
    start_clip = clip(xy=M.TRAY_XY, head_above=(M.TRAY_H - M.PANEL_TOP) / 1000)
    s = state(panel(xy=M.PANEL_START), start_clip)
    assert M.panel_misalign(s["panel"]) > M.LINED_UP and M.score(s) == 0.0


def test_fastened_succeeds_and_scores_0_9_until_held():
    assert M.success(state()) and M.score(state()) == 0.9


def test_the_panel_lined_up_with_no_clip_scores_a_quarter():
    s = state(c=clip(xy=M.TRAY_XY, head_above=(M.TRAY_H - M.PANEL_TOP) / 1000))
    assert M.score(s) == 0.25


def test_a_clip_held_but_standing_proud_scores_half():
    assert M.score(state(c=clip(head_above=0.002))) == 0.5


def test_a_clip_not_past_the_catch_does_not_count():
    # the collar's top still above the jaws' underside: the catch doesn't hold it
    proud = (M.STEM_L - (M.PANEL_TOP - (M.JAW_Z - M.JAW_R))) / 1000 + 0.001
    assert M.score(state(c=clip(head_above=proud))) == 0.25


def test_the_panel_out_of_line_scores_0_even_with_the_clip_in():
    s = state(panel(xy=(M.PANEL_LINED[0] + 0.003, M.PANEL_LINED[1])), clip())
    assert M.score(s) == 0.0


def test_lined_up_means_within_1_5_mm():
    assert M.success(state(panel(xy=(M.PANEL_LINED[0] + 0.0014, M.PANEL_LINED[1]))))
    assert not M.success(state(panel(xy=(M.PANEL_LINED[0] + 0.0016, M.PANEL_LINED[1]))))


def test_a_turned_panel_measures_its_hole_where_it_is():
    # turned 2 degrees about its middle, its hole (7 cm out) moves 2.4 mm: out of line
    assert M.score(state(panel(yaw=2.0))) == 0.0


def test_a_tilted_or_lifted_panel_scores_0():
    assert M.score(state(panel(roll=5.0))) == 0.0
    assert M.score(state(panel(bottom=SHEET_TOP + 0.003))) == 0.0


def test_a_crooked_clip_does_not_count():
    assert M.score(state(c=clip(tilt=8.0))) == 0.25


def test_a_clip_beside_the_hole_does_not_count():
    assert M.score(state(c=clip(xy=(M.HOLE_W[0] + 0.003, M.HOLE_W[1])))) == 0.25


def test_moving_parts_do_not_succeed():
    assert not M.success(state(c=clip(moving=True))) and not M.success(state(p=panel(moving=True)))


# -- the scene ------------------------------------------------------------------------------------------------------

def test_the_collar_passes_the_holes_and_the_stem_passes_the_jaws():
    across_corners = M.COLLAR_W / math.cos(math.pi / 8)
    assert M.COLLAR_W < M.HOLE and across_corners < M.HOLE * math.sqrt(2)
    assert M.STEM_W / math.cos(math.pi / 8) < M.JAW_GAP < M.COLLAR_W  # the jaws let the stem by, catch the collar


def test_seated_the_collar_is_below_the_jaws():
    collar_top = M.PANEL_TOP - M.STEM_L
    assert collar_top < M.JAW_Z - M.JAW_R


def test_the_clip_stands_in_its_tray():
    assert M.CLIP_LEN < M.TRAY_H and M.COLLAR_W < M.HOLE


def test_the_panel_starts_out_of_line_but_on_the_flange():
    assert M.panel_misalign(panel(xy=M.PANEL_START)) > 2 * M.LINED_UP
    sy = M.SHEET[1] / 2000
    for y in (M.PANEL_START[1] - M.PANEL[1] / 2000, M.PANEL_START[1] + M.PANEL[1] / 2000):
        assert abs(y - M.BODY_XY[1]) < sy


def test_the_arms_work_well_apart():
    hole_y, lip_y = M.HOLE_W[1], M.lip_tcp(M.PANEL_LINED)[1]
    assert hole_y - lip_y > 0.16


def test_the_octagons_are_octagons():
    pieces = M._octagon(0.0, 10.0, 1.0)
    assert len(pieces) == 4 and all(p.size[0] == pytest.approx(0.010) for p in pieces)


# -- the demo -------------------------------------------------------------------------------------------------------

@pytest.mark.parametrize("arm", ["left", "right"])
def test_the_demo_is_within_reach(arm):
    plan = ArmPlan(M.demo()[arm], station.BASES[arm])
    for t in np.arange(0.0, plan.duration, 0.05):
        plan.action7(t)
    assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (plan.worst_ik_mm, plan.worst_ik_deg)


def test_the_left_arm_reaches_the_holes_only_once_the_panel_is_in_line():
    left, right = M.demo()["left"], M.demo()["right"]
    held = sum(w.duration for w in right[:5])  # pinched and slid into line
    t = 0.0
    for w in left:
        t += w.duration
        if w.label == "high over the lined-up holes":
            break
    assert t >= held - 1e-9


def test_the_right_arm_holds_until_the_clip_is_home():
    left, right = M.demo()["left"], M.demo()["right"]
    pushed = 0.0
    for w in left:
        pushed += w.duration
        if w.label == "hold":
            break
    held = 0.0
    for w in right:
        held += w.duration
        if w.label == "keep holding":
            break
    assert held >= pushed


def test_the_demo_finishes_well_within_the_run_limit():
    assert max(sum(w.duration for w in M.demo()[a]) for a in ("left", "right")) < 600 / 20


def test_the_demo_aims_unshifted():
    assert M.DEMO_AIM_SHIFT_MM == 0.0
