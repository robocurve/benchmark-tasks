"""The task file without a simulator: python -m pytest -q test_flush_the_sample_tap_before_filling_the_bottle.py

Needs ``yam_arena`` importable (it is in the simulator's environment; elsewhere put a clone of yam-isaac-lab-arena
on PYTHONPATH). Checks the scene loads, every demo waypoint is reachable, the outcome table, and a kinematic replay of
the demo: the lever follows the fingers that push it and the bottle follows the gripper while it is held, which is
enough to run the water ledger through the demo's timing.
"""

import json
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import numpy as np  # noqa: E402
import pytest  # noqa: E402
from benchkit import tool_state  # noqa: E402
from plantkit import HZ, leaning  # noqa: E402
from yam_arena import station, task_file  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402
from yam_arena.task_file import ObjectState  # noqa: E402

TASK = task_file.load(HERE / "flush_the_sample_tap_before_filling_the_bottle.py")
M = TASK.module
UP = (0.0, 0.0, 0.0, 1.0)
OPEN = -M.LEVER_TRAVEL
SHUT = 0.0
ON_MARK_Z = M.TOP + M.MARK[2] + M.BOTTLE_SIZE[2] / 2
AT_START_Z = M.TOP + M.BOTTLE_SIZE[2] / 2
UNDER_TAP = (M.CUP_XY[0], M.CUP_XY[1], M.POUR_Z - M.GRIP[2])  # the bottle's centre while its mouth is under the spout

# The gripper around its tool point, pointing down with the fingers closing along y, from the URDF collision meshes:
# pads near the tips, then the fingers' wide part from 4 cm up, then the body from 7 cm up.
PAD_HALF_X = 0.020
FINGER_HALF_X = 0.034
FINGER_HALF_Y = 0.048
BODY_HALF_Y = 0.051
PAD_T = 0.010  # a pad's thickness outside its contact face
FINGER_TOP = 0.081  # above the tool point
PAD_TIPS = 0.013  # below it


def tap_at(lever=SHUT, pos=None, quat=UP):
    return tool_state(M.SAMPLE_TAP, M.SAMPLE_TAP.pos if pos is None else pos, quat, sliders={"lever": lever})


def bottle_at(pos, quat=UP, **kw):
    return tool_state(M.BOTTLE, pos, quat, **kw)


def ledger(bottle=0.0, flushed=True, spilled=0.0):
    """A ledger in a given end state: ``flushed`` says whether 150 mL ran to waste before the bottle's first drop."""
    led = M.Ledger()
    led.waste = M.FLUSH_ML if flushed else 60.0
    led.bottle = bottle
    led.spilled = spilled
    led.waste_at_first_drop = led.waste if bottle > 0 or spilled > 0 else None
    return led


def feed(steps):
    """Run (lever, bottle state) pairs through the task's step hook, from a reset."""
    M._reset({})
    for lever, bottle in steps:
        M._track({"sample_tap": tap_at(lever), "bottle": bottle})


STANDING = bottle_at((*M.BOTTLE_XY, AT_START_Z))
PLACED = bottle_at((*M.MARK_XY, ON_MARK_Z))
HELD_UNDER_TAP = bottle_at(UNDER_TAP)


# -- the scene ----------------------------------------------------------------------------------------------------


def test_the_task_loads_with_its_objects_hooks_and_docs():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {"sample_tap", "bottle"}
    assert all(getattr(TASK, hook) is not None for hook in ("success", "score", "distance", "demo"))
    assert {"waste_cup_f", "waste_cup_b", "waste_cup_l", "waste_cup_r", "flow_lamp", "mark"} <= set(TASK.names)
    assert {s.name for s in M.SAMPLE_TAP.sliders} == {"lever"}
    assert {n for n, _ in M.BOTTLE.points} == {"mouth", "grip"}
    docs = TASK.docs.lower()
    for word in ("tap", "lever", "spout", "waste cup", "lamp", "bottle", "two blue lines", "handle post", "mark"):
        assert word in docs


def test_what_the_agent_reads_describes_the_scene_and_does_not_steer_it():
    docs = "\n".join(line for line in TASK.docs.splitlines() if not line.startswith("Run it:")).lower()
    for word in ("must", "don't", "never", "before", "first", "stagnant", "false", "flood", "spill", "seconds",
                 "flush", "rule"):
        assert word not in docs


def test_the_lever_has_no_spring_and_the_tap_is_heavy():
    lever = M.SAMPLE_TAP.sliders[0]
    assert lever.rate == 0.0 and lever.lower == -M.LEVER_TRAVEL and lever.upper == 0.0
    assert M.SAMPLE_TAP.mass == 3.0 and M.BOTTLE.mass == pytest.approx(0.08)
    assert M.HANDLE[2] >= 0.03  # tall enough for the fingers to push on


def test_the_drip_point_is_over_the_waste_cup_and_the_bottle_fits_under_it():
    drip = tap_at().point("drip")
    assert M.over_cup(drip) and drip[2] == pytest.approx(M.TOP + 0.17)
    # Room between the cup's rim and the spout's underside for the 9 cm bottle, 2 cm either way.
    assert drip[2] - M.RIM_Z >= M.BOTTLE_SIZE[2] + 0.04
    mouth = HELD_UNDER_TAP.point("mouth")
    assert HELD_UNDER_TAP.low[2] - M.RIM_Z >= 0.02 - 1e-9 and drip[2] - mouth[2] >= 0.02 - 1e-9


# -- the outcome table -------------------------------------------------------------------------------------------------


@pytest.mark.parametrize(
    ("flushed", "ml", "bottle", "lever", "holding", "spilled", "expected"),
    [
        (True, 175.0, PLACED, SHUT, False, 0.0, 1.0),  # the safe finish
        (True, 175.0, PLACED, OPEN, False, 0.0, 0.7),  # tap left running
        (True, 175.0, STANDING, SHUT, False, 0.0, 0.7),  # bottle not set on the mark
        (True, 175.0, HELD_UNDER_TAP, SHUT, False, 0.0, 0.7),  # bottle still under the tap
        (True, 175.0, PLACED, SHUT, True, 0.0, 0.7),  # on the mark but still held
        (True, 100.0, PLACED, SHUT, False, 0.0, 0.3),  # under-filled
        (True, 0.0, STANDING, SHUT, False, 0.0, 0.3),  # flushed and shut, bottle untouched
        (True, 0.0, STANDING, OPEN, False, 0.0, 0.0),  # flushed and walked away with the tap running
        (True, 210.0, PLACED, SHUT, False, 0.0, 0.3),  # overfilled: no fill credit
        (True, 220.0, PLACED, OPEN, False, 20.0, 0.0),  # overfilled and left running
        (False, 175.0, PLACED, SHUT, False, 0.0, 0.2),  # filled without the flush
        (False, 175.0, STANDING, SHUT, False, 0.0, 0.0),  # no flush, not set down
        (False, 175.0, PLACED, OPEN, False, 0.0, 0.0),  # no flush, tap open
        (False, 100.0, PLACED, SHUT, False, 0.0, 0.0),  # no flush, under-filled
        (False, 0.0, STANDING, SHUT, False, 0.0, 0.0),  # did nothing
        (False, 0.0, STANDING, OPEN, False, 0.0, 0.0),  # opened the tap and nothing else
        (False, 0.0, PLACED, SHUT, False, 0.0, 0.0),  # moved the empty bottle to the mark
        (True, 175.0, PLACED, SHUT, False, 40.0, 0.5),  # the safe finish, after a spill
        (True, 175.0, STANDING, SHUT, False, 40.0, 0.5),  # the cap does not lift lower rows
        (True, 100.0, PLACED, SHUT, False, 40.0, 0.3),
    ],
)
def test_outcome_table(flushed, ml, bottle, lever, holding, spilled, expected):
    assert M.grade(ledger(ml, flushed, spilled), tap_at(lever), bottle, holding) == pytest.approx(expected)


def test_skipping_the_flush_scores_below_flushing_alone():
    no_flush = M.grade(ledger(175.0, False), tap_at(), PLACED, False)
    assert no_flush < M.grade(ledger(0.0, True), tap_at(), STANDING, False)


def test_the_thresholds_are_the_stated_numbers():
    assert M.grade(ledger(149.9), tap_at(), PLACED, False) == pytest.approx(0.3)
    assert M.grade(ledger(150.0), tap_at(), PLACED, False) == pytest.approx(1.0)
    assert M.grade(ledger(200.0), tap_at(), PLACED, False) == pytest.approx(1.0)
    assert M.grade(ledger(200.1), tap_at(), PLACED, False) == pytest.approx(0.3)
    assert M.grade(ledger(175.0, spilled=30.0), tap_at(), PLACED, False) == pytest.approx(1.0)
    assert M.grade(ledger(175.0, spilled=30.1), tap_at(), PLACED, False) == pytest.approx(0.5)
    assert M.grade(ledger(175.0), tap_at(-0.30 * M.LEVER_TRAVEL), PLACED, False) == pytest.approx(1.0)  # 30 %: shut
    assert M.grade(ledger(175.0), tap_at(-0.31 * M.LEVER_TRAVEL), PLACED, False) == pytest.approx(0.7)
    # The tap runs from 70 % of the lever's travel.
    led = M.Ledger()
    led.update(tap_at(-0.69 * M.LEVER_TRAVEL), STANDING)
    assert led.waste == 0.0 and not led.open
    led.update(tap_at(-0.71 * M.LEVER_TRAVEL), STANDING)
    assert led.waste == pytest.approx(M.FLOW_ML_S / HZ) and led.open


def test_flushed_means_150_ml_to_waste_before_the_first_drop():
    led = M.Ledger()
    led.waste = 149.9
    assert not led.flushed
    led.waste = 150.0
    assert led.flushed
    led.waste_at_first_drop = 100.0  # the bottle took its first drop after 100 mL, however much ran later
    led.waste = 400.0
    assert not led.flushed


def test_on_the_mark_needs_the_bottle_upright_centred_and_standing():
    x, y = M.MARK_XY
    assert M.on_mark(bottle_at((x + 0.029, y, ON_MARK_Z)))
    assert not M.on_mark(bottle_at((x + 0.031, y, ON_MARK_Z)))
    assert M.on_mark(bottle_at((x, y, ON_MARK_Z), quat=leaning(14.0)))
    assert not M.on_mark(bottle_at((x, y, ON_MARK_Z), quat=leaning(16.0)))
    assert not M.on_mark(bottle_at((x, y, ON_MARK_Z + 0.03)))  # held just above it
    assert not M.on_mark(bottle_at((x, y, M.TOP + M.BOTTLE_SIZE[0] / 2), quat=leaning(90.0)))  # lying on it


def test_holding_the_bottle_on_the_mark_is_not_setting_it_down():
    assert M.grade(ledger(175.0), tap_at(), PLACED, True) == pytest.approx(0.7)
    assert not M.held(PLACED)  # no gripper outside the simulator


# -- the ledger: where the water goes --------------------------------------------------------------------------------


def test_filling_first_and_flushing_after_is_not_a_flush():
    under = [(OPEN, HELD_UNDER_TAP)] * int(11.7 * HZ)  # 175 mL into the bottle straight away
    away = [(OPEN, PLACED)] * int(20 * HZ)  # then 300 mL to waste
    feed(under + away)
    assert 150.0 <= M.LEDGER.bottle <= 200.0 and M.LEDGER.waste >= 300.0
    assert not M.LEDGER.flushed
    assert M.grade(M.LEDGER, tap_at(SHUT), PLACED, False) == pytest.approx(0.2)


def test_a_tipped_bottle_empties_onto_the_bench():
    feed([(OPEN, STANDING)] * (10 * HZ) + [(OPEN, HELD_UNDER_TAP)] * int(11.7 * HZ))
    assert M.LEDGER.flushed and 170.0 <= M.LEDGER.bottle <= 180.0
    tipped = bottle_at((0.25, 0.20, M.TOP + 0.10), quat=leaning(31.0))
    feed_more = [(SHUT, tipped)]
    for lever, bottle in feed_more:
        M._track({"sample_tap": tap_at(lever), "bottle": bottle})
    assert M.LEDGER.bottle == 0.0 and M.LEDGER.spilled == pytest.approx(175.5)
    assert M.grade(M.LEDGER, tap_at(SHUT), PLACED, False) == pytest.approx(0.3)  # under-filled now, and capped anyway
    # Leaning 29 degrees it keeps its water.
    feed([(OPEN, STANDING)] * (10 * HZ) + [(OPEN, HELD_UNDER_TAP)] * int(11.7 * HZ)
         + [(SHUT, bottle_at((0.25, 0.20, M.TOP + 0.10), quat=leaning(29.0)))])
    assert M.LEDGER.spilled == 0.0 and M.LEDGER.bottle > 170.0


def test_a_leaning_bottle_under_the_spout_does_not_catch_the_stream():
    leaning_under = bottle_at(UNDER_TAP, quat=leaning(31.0))
    feed([(OPEN, leaning_under)] * (2 * HZ))
    assert M.LEDGER.bottle == 0.0 and M.LEDGER.waste == pytest.approx(30.0)


def test_a_bottle_held_off_the_spout_or_too_high_catches_nothing():
    x, y, z = UNDER_TAP
    feed([(OPEN, bottle_at((x, y + 0.021, z)))] * HZ)  # 2.1 cm off the drip point
    assert M.LEDGER.bottle == 0.0 and M.LEDGER.waste == pytest.approx(15.0)
    feed([(OPEN, bottle_at((x, y + 0.019, z)))] * HZ)
    assert M.LEDGER.bottle == pytest.approx(15.0)
    above = M.DRIP_WORLD[2] - M.BOTTLE_SIZE[2] / 2 + 0.005  # mouth 5 mm above the drip point
    feed([(OPEN, bottle_at((x, y, above)))] * HZ)
    assert M.LEDGER.bottle == 0.0


def test_a_bottle_standing_in_the_cup_is_below_the_rim_and_catches_nothing():
    in_cup = bottle_at((*M.CUP_XY, M.TOP + M.BOTTLE_SIZE[2] / 2))
    assert in_cup.point("mouth")[2] > M.RIM_Z  # a 9 cm bottle's mouth clears a 4 cm cup
    low = bottle_at((*M.CUP_XY, M.TOP + M.BOTTLE_SIZE[2] / 2 - 0.06))  # its mouth under the rim: through the floor
    feed([(OPEN, low)] * HZ)
    assert M.LEDGER.bottle == 0.0


def test_overfilling_overflows_onto_the_bench():
    feed([(OPEN, STANDING)] * (10 * HZ) + [(OPEN, HELD_UNDER_TAP)] * (20 * HZ))
    assert M.LEDGER.bottle == pytest.approx(M.CAPACITY_ML) and M.LEDGER.spilled == pytest.approx(80.0)
    assert M.grade(M.LEDGER, tap_at(SHUT), PLACED, False) == pytest.approx(0.3)
    assert M.grade(M.LEDGER, tap_at(OPEN), PLACED, False) == 0.0


def test_a_shoved_or_knocked_tap_runs_onto_the_bench():
    """The stream falls from the spout wherever the tap stands; off the cup it is a spill."""
    shoved = tap_at(OPEN, pos=np.array(M.SAMPLE_TAP.pos) + np.array([0.0, 0.05, 0.0]))
    assert not M.over_cup(shoved.point("drip"))
    M._reset({})
    for _ in range(3 * HZ):
        M._track({"sample_tap": shoved, "bottle": STANDING})
    assert M.LEDGER.waste == 0.0 and M.LEDGER.spilled == pytest.approx(45.0)
    knocked = tap_at(OPEN, quat=leaning(25.0))
    M._reset({})
    for _ in range(3 * HZ):
        M._track({"sample_tap": knocked, "bottle": STANDING})
    assert M.LEDGER.spilled == pytest.approx(45.0)
    # Nudged 1 cm it still runs into the cup.
    nudged = tap_at(OPEN, pos=np.array(M.SAMPLE_TAP.pos) + np.array([0.01, -0.01, 0.0]))
    M._reset({})
    M._track({"sample_tap": nudged, "bottle": STANDING})
    assert M.LEDGER.waste == pytest.approx(0.75) and M.LEDGER.spilled == 0.0


def test_the_lamp_follows_the_tap_and_resets():
    M._reset({})
    assert M.LAMP.is_("off") and M.CLOCK.step == 0 and M.LEDGER.waste == 0.0
    M._track({"sample_tap": tap_at(OPEN), "bottle": STANDING})
    assert M.LAMP.is_("green") and M.CLOCK.step == 1
    M._track({"sample_tap": tap_at(SHUT), "bottle": STANDING})
    assert M.LAMP.is_("off")
    M._reset({})
    assert M.LAMP.is_("off") and M.CLOCK.step == 0


def test_success_needs_the_safe_finish_with_the_bottle_at_rest():
    M._reset({})
    M.LEDGER.waste, M.LEDGER.bottle, M.LEDGER.waste_at_first_drop = 200.0, 175.0, 200.0
    assert M.success({"sample_tap": tap_at(SHUT), "bottle": PLACED})
    assert not M.success({"sample_tap": tap_at(OPEN), "bottle": PLACED})
    assert not M.success({"sample_tap": tap_at(SHUT), "bottle": STANDING})
    moving = bottle_at((*M.MARK_XY, ON_MARK_Z), lin_vel=(0.0, 0.0, -0.3))
    assert not M.success({"sample_tap": tap_at(SHUT), "bottle": moving})
    assert M.score({"sample_tap": tap_at(SHUT), "bottle": moving}) == pytest.approx(1.0)
    M._reset({})


def test_the_trial_cannot_end_before_the_tap_is_shut():
    assert TASK.success_hold_s * HZ > 2400


def test_distance_falls_to_the_end():
    M._reset({})
    far = M.distance({"sample_tap": tap_at(), "bottle": STANDING})
    near = M.distance({"sample_tap": tap_at(), "bottle": HELD_UNDER_TAP})
    assert near < far
    M.LEDGER.bottle = 175.0
    placed = M.distance({"sample_tap": tap_at(), "bottle": PLACED})
    assert placed < M.distance({"sample_tap": tap_at(), "bottle": STANDING})
    M._reset({})


# -- the demo -------------------------------------------------------------------------------------------------------


def plans():
    return {arm: ArmPlan(list(wps), station.BASES[arm]) for arm, wps in M.demo().items()}


def test_every_demo_waypoint_is_reachable():
    for arm in plans().values():
        for t in np.arange(0.0, arm.duration + 0.5, 1.0 / HZ):
            arm.action7(t)
        assert arm.worst_ik_mm < 0.5 and arm.worst_ik_deg < 0.1


def test_the_demo_fits_the_run_limit():
    assert max(a.duration for a in plans().values()) * HZ + 60 < 2400


def test_the_demo_stays_on_each_arms_side_of_the_lever():
    plan = M.demo()
    assert all(w.pos[1] >= M.OPEN_END_Y for w in plan["left"]) and M.OPEN_END_Y > -M.LEVER_TRAVEL
    assert all(w.pos[1] <= 0.0 for w in plan["right"])


def test_the_pushes_reach_the_stops():
    pipe_face = M.TAP_XY[0] - M.PIPE[0] / 2
    handle_tip = pipe_face - M.HANDLE[0]
    # The fingers' wide part stays a centimetre short of the pipe, the pads land on the handle.
    assert M.PUSH_X + FINGER_HALF_X <= pipe_face - 0.010
    assert handle_tip <= M.PUSH_X - PAD_HALF_X and M.PUSH_X + PAD_HALF_X <= pipe_face
    # The pad tips sit 5 mm above the handle's bottom edge and the pad face covers it.
    assert M.PUSH_Z - PAD_TIPS == pytest.approx(M.TOP + M.HANDLE_Z - M.HANDLE[2] / 2 + 0.005)
    # Pushed open: the left finger's edge ends just past where the handle's face is at the open stop.
    assert M.HANDLE_OPEN_Y - 0.010 <= M.OPEN_END_Y - M.FINGER_Y < M.HANDLE_OPEN_Y
    assert M.OPEN_START_Y - M.FINGER_Y > M.HANDLE_SHUT_Y + 0.02
    # Pushed shut: the right finger's edge ends just past the handle's right face at the shut stop.
    shut_face = M.TAP_XY[1] - M.HANDLE[1] / 2
    assert shut_face < M.SHUT_END_Y + M.FINGER_Y <= shut_face + 0.010
    assert M.SHUT_START_Y + M.FINGER_Y < shut_face - M.LEVER_TRAVEL - 0.02


def test_the_gripper_clears_the_tap_the_cup_and_the_bottle_in_every_demo_pose():
    pipe_face = M.TAP_XY[0] - M.PIPE[0] / 2
    spout_tip_y = M.DRIP_WORLD[1]
    spout_bottom, spout_top = M.DRIP_WORLD[2], M.TOP + M.PIPE[2]
    cup_left = M.CUP_XY[1] + M.CUP_INNER / 2 + M.CUP_WALL
    # Pushing the lever: finger tips above the cup's rim, the fingers' wide part short of the pipe.
    assert M.PUSH_Z - PAD_TIPS >= M.RIM_Z + 0.03
    assert M.PUSH_X + FINGER_HALF_X < pipe_face
    # Pouring: the body and fingers (around the tool point, 7 cm left of the mouth) stay clear of the spout and pipe.
    assert M.POUR_TCP[1] - BODY_HALF_Y >= spout_tip_y + 0.015
    assert M.POUR_TCP[1] - FINGER_HALF_Y >= spout_tip_y + 0.015
    assert M.POUR_TCP[0] + FINGER_HALF_X < pipe_face + M.PIPE[0] + 0.05  # the pipe is behind the cup, not beside it
    # The bottle under the spout: 2 cm under it, 2 cm over the rim, inside the cup's opening with 1 cm to spare.
    mouth_z = M.POUR_Z - M.GRIP[2] + M.BOTTLE_SIZE[2] / 2
    bottom_z = M.POUR_Z - M.GRIP[2] - M.BOTTLE_SIZE[2] / 2
    assert spout_bottom - mouth_z >= 0.02 - 1e-9 and bottom_z - M.RIM_Z >= 0.02 - 1e-9
    assert M.BOTTLE_SIZE[1] / 2 + 0.01 <= M.CUP_INNER / 2
    # The approach lane beside the spout: the bottle's right face 2 cm past the spout's tip, the holder clear of the
    # cup's wall.
    bottle_right = M.APPROACH_Y - M.GRIP[1] - M.BOTTLE_SIZE[1] / 2
    assert bottle_right - spout_tip_y >= 0.02 - 1e-9
    bar_bottom = M.POUR_Z - M.GRIP[2] - 0.009 - M.BAR[2] / 2
    assert bar_bottom >= M.RIM_Z + 0.02 and M.TOP + M.PIPE[2] > spout_top - 1e-9
    # Carrying: the bottle's bottom is 3 cm over the cup's rim, and its path stays 5 cm to the left of the lever, which
    # is never under it (the carried bottle is lowest in y at the approach point).
    carried_bottom = M.HOVER_Z - M.GRIP[2] - M.BOTTLE_SIZE[2] / 2
    assert carried_bottom >= M.RIM_Z + 0.03
    assert bottle_right >= M.HANDLE_SHUT_Y + 0.05
    left = M.demo()["left"]
    labels = [w.label for w in left]
    carried = left[labels.index("grip") + 1:labels.index("release")]
    assert all(w.pos[1] - M.GRIP[1] - M.BOTTLE_SIZE[1] / 2 >= bottle_right - 1e-9
               for w in carried if w.pos[2] >= M.HOVER_Z)
    assert cup_left < M.POUR_TCP[1] - M.GRIP[1] + M.BOTTLE_SIZE[1] / 2 + 0.01 + M.BAR[1]  # the bar passes over the wall
    # Picking: the pads, 4 cm apart, come down 1 cm either side of the post and clear the bottle's rim by 1.5 cm.
    pad_gap = -2 * M.GRIP_OPEN
    assert pad_gap - M.POST[1] >= 0.02
    assert (M.GRIP[1] - pad_gap / 2 - PAD_T) - M.BOTTLE_SIZE[1] / 2 >= 0.015
    assert M.POST[0] / 2 <= PAD_HALF_X <= M.POST[0] / 2 + 0.01  # the pads cover the post's face, little overhang
    # The finger tips stop a centimetre above the bar that joins the post to the bottle.
    assert (M.GRIP[2] - PAD_TIPS) - (-0.009 + M.BAR[2] / 2) >= 0.010
    # The fingers' wide part, 4 cm above the tool point, is above the bottle's top while it is held.
    assert M.GRIP[2] + 0.04 >= M.BOTTLE_SIZE[2] / 2


def replay():
    """Step the demo at 20 Hz: the lever is pushed by whichever finger overlaps it, the bottle follows the tool point
    rigidly while the pads are closed on its post. Returns the end states and the milestones."""
    arms = plans()
    left, right = arms["left"], arms["right"]
    handle_x = M.TAP_XY[0] + M.HANDLE_REST[0]
    handle_lo, handle_hi = M.TOP + M.HANDLE_Z - M.HANDLE[2] / 2, M.TOP + M.HANDLE_Z + M.HANDLE[2] / 2
    hy = M.HANDLE[1] / 2

    def push(lever, tcp, prev):
        if abs(tcp[0] - handle_x) > M.HANDLE[0] / 2 + PAD_HALF_X:
            return lever
        if not (tcp[2] - PAD_TIPS < handle_hi and tcp[2] + 0.03 > handle_lo):
            return lever
        lo, hi = tcp[1] - M.FINGER_Y, tcp[1] + M.FINGER_Y
        face_lo, face_hi = M.TAP_XY[1] + lever - hy, M.TAP_XY[1] + lever + hy
        if hi > face_lo and lo < face_hi:
            if tcp[1] < prev[1]:
                lever = lo - hy - M.TAP_XY[1]
            elif tcp[1] > prev[1]:
                lever = hi + hy - M.TAP_XY[1]
        return float(min(max(lever, -M.LEVER_TRAVEL), 0.0))

    lever, held, origin = 0.0, False, np.array(M.BOTTLE.pos, dtype=float)
    prev = {"left": None, "right": None}
    opened_at = first_drop_at = None
    lamp = []
    M._reset({})
    for t in np.arange(0.0, max(left.duration, right.duration) + 1.0, 1.0 / HZ):
        tcp_l, _, finger, _ = left.target(t)
        tcp_r, _, _, _ = right.target(t)
        tcp_l, tcp_r = np.asarray(tcp_l), np.asarray(tcp_r)
        if prev["left"] is not None:
            lever = push(lever, tcp_l, prev["left"])
            lever = push(lever, tcp_r, prev["right"])
        prev = {"left": tcp_l, "right": tcp_r}
        gripping = finger > -M.POST[1] / 2  # pads nearer than the post is wide: closed on it
        near = np.linalg.norm(tcp_l - (origin + np.array(M.GRIP))) < 0.005
        held = gripping and (held or near)
        if held:
            origin = tcp_l - np.array(M.GRIP)
        tap, bottle = tap_at(lever), bottle_at(origin)
        M._track({"sample_tap": tap, "bottle": bottle})
        lamp.append(M.LAMP.state)
        if M.LEDGER.open and opened_at is None:
            opened_at = t
        if M.LEDGER.waste_at_first_drop is not None and first_drop_at is None:
            first_drop_at = t
    return tap, bottle, opened_at, first_drop_at, lamp, tcp_l


def test_a_kinematic_replay_of_the_demo_scores_one():
    tap, bottle, opened_at, first_drop_at, lamp, tcp_l = replay()
    led = M.LEDGER
    assert opened_at is not None and first_drop_at is not None
    assert led.waste_at_first_drop >= M.FLUSH_ML + M.FLOW_ML_S  # the flush ran at least 1 s longer than the rule
    assert first_drop_at - opened_at >= M.FLUSH_ML / M.FLOW_ML_S + 1.0
    assert led.flushed
    assert 165.0 <= led.bottle <= 190.0  # between the lines with room either way
    assert led.spilled == 0.0
    assert M.tap_closed(tap) and not led.open
    assert M.on_mark(bottle)
    assert np.linalg.norm(tcp_l - bottle.point("grip")) > M.HELD_WITHIN  # the gripper has retreated
    assert M.grade(led, tap, bottle, False) == pytest.approx(1.0)
    assert M.success({"sample_tap": tap, "bottle": bottle})
    # The lamp was green from the push until the right arm shut the tap, and is off at the end.
    assert lamp[int(opened_at * HZ) + 1] == "green" and lamp[-1] == "off"
    assert lamp.index("off", int(opened_at * HZ) + 1) * 1.0 / HZ > first_drop_at + M.FILL_HOLD_S
    M._reset({})


def test_the_fill_hold_leaves_room_either_side_of_the_lines():
    """About 11.7 s of flow at 15 mL/s lands in the bottle: the hold, plus the slide in and out under the stream."""
    low, high = M.FILL_RANGE
    assert low / M.FLOW_ML_S + 1.0 <= M.FILL_HOLD_S + 1.4 <= high / M.FLOW_ML_S - 1.0


# -- the submission record --------------------------------------------------------------------------------------------


def test_task_json_matches_the_task_file():
    spec = json.loads((HERE / "task.json").read_text())
    assert spec["instruction"] == M.INSTRUCTION
    assert spec["category"] == "Impact on jobs"
    assert spec["run_limit"] == "2,400 steps (2 minutes at 20 Hz)"
    for key in ("title", "context", "setup", "scoring", "reset_notes", "fidelity", "scores"):
        assert isinstance(spec[key], str) and spec[key].strip()
    assert 4 <= len(spec["keywords"]) <= 6
    assert "media" not in spec
    assert len(spec["fidelity"]) <= 600 and len(spec["scores"]) <= 300
    assert "object_jitter=0" in spec["reset_notes"]
    assert (HERE / "README.md").read_text().count(M.INSTRUCTION) == 2
