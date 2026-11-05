"""The task file without a simulator: python -m pytest -q test_flush_the_sample_tap_before_filling_the_bottle.py

Needs ``yam_arena`` importable (it is in the simulator's environment; elsewhere put a clone of yam-isaac-lab-arena
on PYTHONPATH). Checks the scene loads, every demo waypoint is reachable, the outcome table, and a kinematic replay of
the demo: the lever follows the pads that push it and the bottle follows the gripper while it is held, which is enough
to run the water ledger through the demo's timing. The replay also sweeps a box model of the gripper and its wrist
camera through every step and checks it stays clear of the tap, the cup, the lamp, the bottle and the other arm.
"""

import json
import math
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import numpy as np  # noqa: E402
import pytest  # noqa: E402
from benchkit import tool_state  # noqa: E402
from plantkit import HZ, leaning  # noqa: E402
from yam_arena import station, task_file  # noqa: E402
from yam_arena.constants import TCP_OFFSET  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402

TASK = task_file.load(HERE / "flush_the_sample_tap_before_filling_the_bottle.py")
M = TASK.module
UP = (0.0, 0.0, 0.0, 1.0)
OPEN = -M.LEVER_TRAVEL
SHUT = 0.0
ON_MARK_Z = M.TOP + M.MARK[2] + M.BOTTLE_SIZE[2] / 2
AT_START_Z = M.TOP + M.BOTTLE_SIZE[2] / 2
UNDER_TAP = (M.CUP_XY[0], M.CUP_XY[1], M.POUR_Z - M.GRIP[2])  # the bottle's centre while its mouth is under the spout
FILL_STEPS = 147  # control steps under the stream for a fill between the lines
GOOD_ML = FILL_STEPS * M.FLOW_ML_S / HZ  # 110.25 mL

# A box model of the gripper in its own frame (origin at the tool point; x the fingers' closing axis, -y the wrist
# camera's side, z towards the finger tips), from the URDF collision meshes: the pads near the tips, the finger
# carriages above them, the gripper body, the wrist link. Half-widths; ``q`` is the finger joint (0 closed, metres).
PAD_X = 0.010  # a pad's thickness outside its face, so pads span +-(q + PAD_X)
PAD_Y = 0.020  # the pads' half-length
PAD_Z = (-0.030, 0.013)
FINGER_X = 0.048  # the carriages when closed; opened they span +-(0.024 + q)
FINGER_Y = 0.041
FINGER_Z = (-0.081, -0.030)
BODY = (0.051, 0.0325, (-0.145, -0.070))
LINK5 = (0.0285, 0.0405, (-0.203, -0.134))
CAMERA_HALF = (0.025, 0.025, 0.020)  # a box round the camera's optical centre, bigger than a D405 and its mount


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


def flush_then_fill(fill_steps=FILL_STEPS, flush_s=10.0, bottle=None):
    return [(OPEN, STANDING)] * int(flush_s * HZ) + [(OPEN, bottle or HELD_UNDER_TAP)] * fill_steps


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


def test_the_volumes_come_from_the_bottles_geometry():
    area = (M.BOTTLE_SIZE[0] - 2 * M.BOTTLE_WALL) * (M.BOTTLE_SIZE[1] - 2 * M.BOTTLE_WALL) * 1e4  # cm2
    assert area == pytest.approx(19.36)
    assert M.FILL_RANGE[0] == pytest.approx(area * (5.0 - 0.3), abs=0.5) and M.FILL_RANGE[0] == 91.0
    assert M.FILL_RANGE[1] == pytest.approx(area * (7.0 - 0.3), abs=0.5) and M.FILL_RANGE[1] == 130.0
    assert M.CAPACITY_ML == pytest.approx(area * (9.0 - 0.3), abs=0.5) and M.CAPACITY_ML == 168.0
    assert M.FLUSH_ML == 10.0 * M.FLOW_ML_S


def test_the_holder_bar_joins_the_wall_to_the_post():
    assert M.BAR_Y0 < M.BOTTLE_SIZE[1] / 2 < M.POST_Y - M.POST[1] / 2 < M.BAR_Y1 < M.POST_Y + M.POST[1] / 2
    assert M.BAR == pytest.approx((0.015, 0.048, 0.012))


def _hull(points):
    """The convex hull of 2-D points, counter-clockwise (monotone chain)."""
    pts = sorted(set(points))

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def _inside_by(point, hull) -> float:
    """How far a 2-D point is inside a counter-clockwise hull (negative outside)."""
    out = []
    for a, b in zip(hull, hull[1:] + hull[:1]):
        edge, to = np.subtract(b, a), np.subtract(point, a)
        out.append(float((edge[0] * to[1] - edge[1] * to[0]) / np.linalg.norm(edge)))
    return min(out)


def _centre_of_mass(boxes):
    """Where the simulator puts a tool's centre of mass: its mass spread evenly over the volume of its boxes."""
    volume = np.array([np.prod(size) for _, size, _ in boxes])
    centres = np.array([centre for centre, _, _ in boxes])
    return (volume[:, None] * centres).sum(0) / volume.sum()


def test_the_bottle_stands_on_its_floor_and_the_posts_leg():
    """The post, 7.5 cm out on its bar, pulls the centre of mass 3 cm off the bottle's centre, past the edge of its
    5 cm floor: without the leg the bottle tips over onto its post when the trial starts (24 degrees in the simulator)
    and the open pads come down beside the post's old place. The leg puts the centre of mass 2 cm inside the feet."""
    bottom = -M.BOTTLE_SIZE[2] / 2
    feet = [(c, s) for c, s, _ in M.BOTTLE.boxes if abs(c[2] - s[2] / 2 - bottom) < 1e-9]
    assert [s for c, s in feet if abs(c[1] - M.POST_Y) < 1e-9] == [M.LEG]  # the leg reaches the table
    assert M.POST_Z - M.POST[2] / 2 == pytest.approx(M.LEG_Z + M.LEG[2] / 2)  # and meets the post
    footprint = _hull([(c[0] + sx * s[0] / 2, c[1] + sy * s[1] / 2) for c, s in feet for sx in (-1, 1)
                       for sy in (-1, 1)])
    com = _centre_of_mass(M.BOTTLE.boxes)
    assert _inside_by(tuple(com[:2]), footprint) >= 0.015
    without_leg = _centre_of_mass([b for b in M.BOTTLE.boxes if b[1] != M.LEG])
    assert without_leg[1] - M.BOTTLE_SIZE[1] / 2 > 0.004  # past the floor's edge: it tips
    # The leg is inside the bottle's bounds, so every clearance the sweep below measures holds for it too.
    assert M.BOTTLE.size == pytest.approx((M.BOTTLE_SIZE[0] + 0.002, M.POST_Y + M.POST[1] / 2 + M.BOTTLE_SIZE[1] / 2
                                           + 0.001, M.BOTTLE_SIZE[2]))


def test_on_the_mark_the_leg_stands_on_the_table_and_the_bottle_leans_a_little():
    """The leg lands beside the 3 mm plate, so the bottle rests on its floor's left edge and the leg, leaning about 3
    degrees towards the post: still upright and on the mark."""
    leg_out = M.POST_Y + M.LEG[1] / 2
    assert M.MARK_XY[1] + M.POST_Y - M.LEG[1] / 2 > M.MARK_XY[1] + M.MARK[1] / 2  # off the plate
    lean = math.asin(M.MARK[2] / (leg_out - M.BOTTLE_SIZE[1] / 2))
    assert math.degrees(lean) == pytest.approx(3.1, abs=0.1)
    assert _centre_of_mass(M.BOTTLE.boxes)[1] > M.BOTTLE_SIZE[1] / 2  # so it does lean onto the leg
    half_y, half_z = M.BOTTLE_SIZE[1] / 2, M.BOTTLE_SIZE[2] / 2
    pivot_y, pivot_z = M.MARK_XY[1] + half_y, M.TOP + M.MARK[2]
    centre = (M.MARK_XY[0], pivot_y - half_y * math.cos(lean) + half_z * math.sin(lean),
              pivot_z + half_y * math.sin(lean) + half_z * math.cos(lean))
    resting = bottle_at(centre, quat=leaning(-math.degrees(lean)))
    foot = resting.to_world((0.0, leg_out, -half_z))
    assert foot[2] == pytest.approx(M.TOP, abs=1e-4)
    assert M.on_mark(resting) and resting.tilt_deg() < M.UPRIGHT_DEG - 10.0
    assert M.grade(ledger(GOOD_ML), tap_at(), resting, False) == pytest.approx(1.0)


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
        (True, GOOD_ML, PLACED, SHUT, False, 0.0, 1.0),  # the safe finish
        (True, GOOD_ML, PLACED, OPEN, False, 0.0, 0.25),  # tap left running
        (True, GOOD_ML, STANDING, OPEN, False, 0.0, 0.25),  # tap left running, bottle not set down either
        (True, GOOD_ML, STANDING, SHUT, False, 0.0, 0.7),  # bottle not set on the mark
        (True, GOOD_ML, HELD_UNDER_TAP, SHUT, False, 0.0, 0.7),  # bottle still under the tap
        (True, GOOD_ML, PLACED, SHUT, True, 0.0, 0.7),  # on the mark but still held
        (True, 60.0, PLACED, SHUT, False, 0.0, 0.3),  # under-filled
        (True, 0.0, STANDING, SHUT, False, 0.0, 0.3),  # flushed and shut, bottle untouched
        (True, 0.0, STANDING, OPEN, False, 0.0, 0.0),  # flushed and walked away with the tap running
        (True, 60.0, PLACED, OPEN, False, 0.0, 0.0),  # under-filled and running
        (True, 140.0, PLACED, SHUT, False, 0.0, 0.3),  # overfilled: no fill credit
        (True, 168.0, PLACED, OPEN, False, 20.0, 0.0),  # overfilled and left running
        (False, GOOD_ML, PLACED, SHUT, False, 0.0, 0.2),  # filled without the flush
        (False, GOOD_ML, STANDING, SHUT, False, 0.0, 0.0),  # no flush, not set down
        (False, GOOD_ML, PLACED, OPEN, False, 0.0, 0.0),  # no flush, tap open
        (False, 60.0, PLACED, SHUT, False, 0.0, 0.0),  # no flush, under-filled
        (False, 0.0, STANDING, SHUT, False, 0.0, 0.0),  # did nothing
        (False, 0.0, STANDING, OPEN, False, 0.0, 0.0),  # opened the tap and nothing else
        (False, 0.0, PLACED, SHUT, False, 0.0, 0.0),  # moved the empty bottle to the mark
        (True, GOOD_ML, PLACED, SHUT, False, 40.0, 0.25),  # the safe finish, after a spill
        (True, GOOD_ML, STANDING, SHUT, False, 40.0, 0.25),
        (True, 60.0, PLACED, SHUT, False, 40.0, 0.25),  # the cap binds on the 0.3 row too
        (False, GOOD_ML, PLACED, SHUT, False, 40.0, 0.2),  # the cap does not lift lower rows
    ],
)
def test_outcome_table(flushed, ml, bottle, lever, holding, spilled, expected):
    assert M.grade(ledger(ml, flushed, spilled), tap_at(lever), bottle, holding) == pytest.approx(expected)


def test_the_caps_sit_below_the_honest_partial_results():
    assert M.NO_FLUSH < M.FLUSHED_ONLY and M.TAP_OPEN_CAP < M.FLUSHED_ONLY and M.SPILL_CAP < M.FLUSHED_ONLY
    no_flush = M.grade(ledger(GOOD_ML, False), tap_at(), PLACED, False)
    assert no_flush < M.grade(ledger(0.0, True), tap_at(), STANDING, False)


def test_a_knocked_or_shoved_tap_scores_zero_whatever_else_happened():
    knocked = tap_at(SHUT, quat=leaning(25.0))
    assert M.grade(ledger(GOOD_ML), knocked, PLACED, False) == 0.0
    shoved = tap_at(SHUT, pos=np.array(M.SAMPLE_TAP.pos) + np.array([0.0, 0.021, 0.0]))
    assert M.grade(ledger(GOOD_ML), shoved, PLACED, False) == 0.0
    nudged = tap_at(SHUT, pos=np.array(M.SAMPLE_TAP.pos) + np.array([0.01, -0.01, 0.0]))
    assert M.grade(ledger(GOOD_ML), nudged, PLACED, False) == pytest.approx(1.0)
    assert M.grade(ledger(GOOD_ML), tap_at(SHUT, quat=leaning(15.0)), PLACED, False) == pytest.approx(1.0)


def test_the_thresholds_are_the_stated_numbers():
    lo, hi = M.FILL_RANGE
    assert M.grade(ledger(lo - 0.1), tap_at(), PLACED, False) == pytest.approx(0.3)
    assert M.grade(ledger(lo), tap_at(), PLACED, False) == pytest.approx(1.0)
    assert M.grade(ledger(hi), tap_at(), PLACED, False) == pytest.approx(1.0)
    assert M.grade(ledger(hi + 0.1), tap_at(), PLACED, False) == pytest.approx(0.3)
    assert M.grade(ledger(GOOD_ML, spilled=30.0), tap_at(), PLACED, False) == pytest.approx(1.0)
    assert M.grade(ledger(GOOD_ML, spilled=30.1), tap_at(), PLACED, False) == pytest.approx(0.25)
    assert M.grade(ledger(GOOD_ML), tap_at(-0.30 * M.LEVER_TRAVEL), PLACED, False) == pytest.approx(1.0)  # shut
    assert M.grade(ledger(GOOD_ML), tap_at(-0.31 * M.LEVER_TRAVEL), PLACED, False) == pytest.approx(0.25)
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


def test_holding_the_bottle_anywhere_on_it_is_not_setting_it_down(monkeypatch):
    assert not M.held(PLACED)  # no gripper outside the simulator
    assert M.grade(ledger(GOOD_ML), tap_at(), PLACED, True) == pytest.approx(0.7)
    feed(flush_then_fill())  # the task's own ledger now holds a good, flushed sample
    post = PLACED.point("grip")
    body = PLACED.pos + np.array([0.0, 0.0, 0.01])  # inside the bottle's body
    rim = PLACED.pos + np.array([-M.BOTTLE_SIZE[0] / 2 - 0.008, 0.0, M.BOTTLE_SIZE[2] / 2])  # 8 mm off its rim
    away = PLACED.pos + np.array([0.0, 0.0, M.BOTTLE_SIZE[2] / 2 + 0.03])  # 3 cm above it
    for point, holding in ((post, True), (body, True), (rim, True), (away, False)):
        monkeypatch.setattr(M, "tool_points", lambda p=point: [p])
        assert M.held(PLACED) is holding
        assert M.score({"sample_tap": tap_at(), "bottle": PLACED}) == pytest.approx(0.7 if holding else 1.0)
    M._reset({})


# -- the ledger: where the water goes --------------------------------------------------------------------------------


def test_a_flush_then_a_fill_is_the_protocol():
    feed(flush_then_fill())
    assert M.LEDGER.flushed and M.LEDGER.waste_at_first_drop == pytest.approx(150.0)
    assert M.LEDGER.bottle == pytest.approx(GOOD_ML) and M.LEDGER.spilled == 0.0
    assert M.grade(M.LEDGER, tap_at(SHUT), PLACED, False) == pytest.approx(1.0)


def test_filling_first_and_flushing_after_is_not_a_flush():
    feed([(OPEN, HELD_UNDER_TAP)] * FILL_STEPS + [(OPEN, PLACED)] * (20 * HZ))
    assert M.filled(M.LEDGER) and M.LEDGER.waste >= 300.0
    assert not M.LEDGER.flushed
    assert M.grade(M.LEDGER, tap_at(SHUT), PLACED, False) == pytest.approx(0.2)


def test_a_bottle_wetted_during_the_flush_was_not_flushed():
    """A bottle under the stream but held badly, 2.1 cm off or leaning 35 degrees with its mouth under the drip point,
    takes no credited water, yet its first drop is recorded: centring it afterwards does not make a flush. The stream
    hitting the rim or the outside of the bottle wets nothing that counts."""
    x, y, z = UNDER_TAP
    off_centre = bottle_at((x, y + 0.021, z))
    feed([(OPEN, off_centre)] * (10 * HZ) + [(OPEN, HELD_UNDER_TAP)] * FILL_STEPS)
    assert M.LEDGER.waste_at_first_drop == pytest.approx(0.0) and not M.LEDGER.flushed
    assert M.filled(M.LEDGER) and M.grade(M.LEDGER, tap_at(SHUT), PLACED, False) == pytest.approx(0.2)
    # Leaning 35 degrees with its mouth's centre under the drip point: the stream enters the opening, uncredited.
    tilt = math.radians(35.0)
    mouth_under = bottle_at((x, y + M.BOTTLE_SIZE[2] / 2 * math.sin(tilt), z), quat=leaning(35.0))
    assert np.linalg.norm(mouth_under.point("mouth")[:2] - np.array(M.CUP_XY)) < 0.001
    assert M.stream_in_mouth(tap_at(OPEN), mouth_under) and not M.catching(tap_at(OPEN), mouth_under)
    feed([(OPEN, mouth_under)] * (10 * HZ) + [(OPEN, HELD_UNDER_TAP)] * FILL_STEPS)
    assert M.LEDGER.waste_at_first_drop == pytest.approx(0.0) and not M.LEDGER.flushed
    # Leaning 35 degrees with its centre under the drip point, the mouth swings 2.6 cm aside and the stream crosses
    # the tilted opening's plane 3.2 cm from its centre: it hits the outside, so the flush still counts.
    centre_under = bottle_at(UNDER_TAP, quat=leaning(35.0))
    assert np.linalg.norm(centre_under.point("mouth")[:2] - np.array(M.CUP_XY)) == pytest.approx(0.0258, abs=0.0005)
    assert not M.stream_in_mouth(tap_at(OPEN), centre_under)
    feed([(OPEN, centre_under)] * (10 * HZ) + [(OPEN, HELD_UNDER_TAP)] * FILL_STEPS)
    assert M.LEDGER.waste_at_first_drop == pytest.approx(150.0) and M.LEDGER.flushed
    # Upright and 2.4 cm off, the stream lands on the rim.
    feed([(OPEN, bottle_at((x, y + 0.024, z)))] * (10 * HZ) + [(OPEN, HELD_UNDER_TAP)] * FILL_STEPS)
    assert M.LEDGER.waste_at_first_drop == pytest.approx(150.0) and M.LEDGER.flushed


def test_a_tipped_bottle_empties_into_the_cup_or_onto_the_bench():
    feed(flush_then_fill())
    tipped_away = bottle_at((0.25, 0.20, M.TOP + 0.10), quat=leaning(31.0))
    M._track({"sample_tap": tap_at(SHUT), "bottle": tipped_away})
    assert M.LEDGER.bottle == 0.0 and M.LEDGER.spilled == pytest.approx(GOOD_ML)
    assert M.grade(M.LEDGER, tap_at(SHUT), PLACED, False) == pytest.approx(0.25)
    feed(flush_then_fill())
    tipped_over_cup = bottle_at((*M.CUP_XY, M.POUR_Z), quat=leaning(31.0))
    M._track({"sample_tap": tap_at(SHUT), "bottle": tipped_over_cup})
    assert M.LEDGER.bottle == 0.0 and M.LEDGER.spilled == 0.0 and M.LEDGER.waste == pytest.approx(150.0 + GOOD_ML)
    assert M.grade(M.LEDGER, tap_at(SHUT), PLACED, False) == pytest.approx(0.3)  # under-filled now
    feed(flush_then_fill() + [(SHUT, bottle_at((0.25, 0.20, M.TOP + 0.10), quat=leaning(29.0)))])
    assert M.LEDGER.spilled == 0.0 and M.LEDGER.bottle == pytest.approx(GOOD_ML)  # leaning 29 degrees it keeps it


def test_a_leaning_bottle_under_the_spout_does_not_catch_the_stream():
    feed([(OPEN, bottle_at(UNDER_TAP, quat=leaning(31.0)))] * (2 * HZ))
    assert M.LEDGER.bottle == 0.0 and M.LEDGER.waste == pytest.approx(30.0)


def test_a_bottle_held_off_the_spout_or_too_high_catches_nothing():
    x, y, z = UNDER_TAP
    feed([(OPEN, bottle_at((x, y + 0.021, z)))] * HZ)  # 2.1 cm off the drip point
    assert M.LEDGER.bottle == 0.0 and M.LEDGER.waste == pytest.approx(15.0)
    feed([(OPEN, bottle_at((x, y + 0.019, z)))] * HZ)
    assert M.LEDGER.bottle == pytest.approx(15.0)
    above = M.DRIP_WORLD[2] - M.BOTTLE_SIZE[2] / 2 + 0.005  # mouth 5 mm above the drip point
    feed([(OPEN, bottle_at((x, y, above)))] * HZ)
    assert M.LEDGER.bottle == 0.0 and M.LEDGER.waste_at_first_drop is None


def test_a_bottle_set_down_in_the_cup_rests_on_its_bar_and_still_fills():
    """The holder's bar lands on the cup's left wall, so the bottle leans about 11 degrees, post side up, with its far
    bottom edge on the table and its mouth still under the stream: resting it there fills it like holding it. Lying
    in the cup it catches nothing."""
    lean = math.radians(11.0)
    half_y, half_z = M.BOTTLE_SIZE[1] / 2, M.BOTTLE_SIZE[2] / 2
    centre_z = M.TOP + half_y * math.sin(lean) + half_z * math.cos(lean)  # the right bottom edge on the table
    propped = bottle_at((*M.CUP_XY, centre_z), quat=leaning(11.0))
    assert propped.low[2] == pytest.approx(M.TOP, abs=0.0005)  # the proud fill lines dip a fraction of a millimetre
    # The bar's underside, where it crosses the cup's left wall, rests just on the 4 cm rim.
    bar_under = M.BAR_Z - M.BAR[2] / 2
    y_local = (M.CUP_INNER / 2 + bar_under * math.sin(lean)) / math.cos(lean)
    bar_under_z = centre_z + y_local * math.sin(lean) + bar_under * math.cos(lean)
    assert M.RIM_Z <= bar_under_z <= M.RIM_Z + 0.001
    mouth = propped.point("mouth")
    assert mouth[2] > M.RIM_Z and np.linalg.norm(mouth[:2] - np.array(M.CUP_XY)) == pytest.approx(0.0086, abs=0.0005)
    assert M.stream_in_mouth(tap_at(OPEN), propped) and M.catching(tap_at(OPEN), propped)
    feed([(OPEN, STANDING)] * (10 * HZ) + [(OPEN, propped)] * FILL_STEPS)
    assert M.LEDGER.flushed and M.LEDGER.bottle == pytest.approx(GOOD_ML)
    lying = bottle_at((*M.CUP_XY, M.TOP + M.BOTTLE_SIZE[0] / 2), quat=leaning(90.0))
    feed([(OPEN, lying)] * HZ)
    assert M.LEDGER.bottle == 0.0 and M.LEDGER.waste_at_first_drop is None


def test_overfilling_overflows_into_the_cup_under_the_spout():
    feed(flush_then_fill(fill_steps=20 * HZ))
    assert M.LEDGER.bottle == pytest.approx(M.CAPACITY_ML) and M.LEDGER.spilled == 0.0
    assert M.LEDGER.waste == pytest.approx(150.0 + 300.0 - M.CAPACITY_ML)
    assert M.grade(M.LEDGER, tap_at(SHUT), PLACED, False) == pytest.approx(0.3)
    assert M.grade(M.LEDGER, tap_at(OPEN), PLACED, False) == 0.0


def test_a_shoved_or_knocked_tap_runs_onto_the_bench_and_fills_nothing():
    """The stream falls from the spout wherever the tap stands; off the cup it is a spill, and a tap carried to the
    bottle fills it with nothing that counts."""
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
    # Carried so that its spout stands over the bottle on the table: the stream enters the mouth but is not credited.
    drip = M.SAMPLE_TAP.points[0][1]
    over_bottle = tap_at(OPEN, pos=(M.BOTTLE_XY[0] - drip[0], M.BOTTLE_XY[1] - drip[1], M.TOP))
    M._reset({})
    for _ in range(10 * HZ):
        M._track({"sample_tap": over_bottle, "bottle": STANDING})
    assert M.stream_in_mouth(over_bottle, STANDING) and M.LEDGER.bottle == 0.0
    assert M.LEDGER.spilled == pytest.approx(150.0) and M.grade(M.LEDGER, over_bottle, STANDING, False) == 0.0
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
    feed(flush_then_fill())
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
    M.LEDGER.bottle = GOOD_ML
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


def test_each_arm_turns_its_wrist_the_one_way_it_can():
    plan = M.demo()
    assert all(w.yaw in (0.0, M.PUSH_YAW["left"]) for w in plan["left"])
    assert all(w.yaw == M.PUSH_YAW["right"] for w in plan["right"])
    assert M.PUSH_YAW["left"] == -M.PUSH_YAW["right"]


def test_the_left_arm_crosses_the_midline_only_at_the_open_stop_and_the_right_stays_on_its_side():
    plan = M.demo()
    assert all(w.pos[1] >= M.OPEN_END_Y for w in plan["left"]) and -0.01 <= M.OPEN_END_Y < 0.0
    assert all(w.pos[1] <= 0.0 for w in plan["right"])


def test_the_pushes_reach_the_stops():
    pipe_face = M.TAP_XY[0] - M.PIPE[0] / 2
    handle_tip = pipe_face - M.HANDLE[0]
    # With the gripper turned, the closed pads sit across the handle 1.5 cm in from its tip.
    assert handle_tip + 0.01 <= M.PUSH_X - PAD_X and M.PUSH_X + PAD_X <= pipe_face - 0.05
    # The pad tips sit 5 mm above the handle's bottom edge and the pads' long side reaches the handle's top.
    assert M.PUSH_Z - 0.013 == pytest.approx(M.TOP + M.HANDLE_Z - M.HANDLE[2] / 2 + 0.005)
    assert M.TOP + M.HANDLE_Z + M.HANDLE[2] / 2 <= M.PUSH_Z - PAD_Z[0]
    assert M.FINGER_PUSH == PAD_Y
    # Pushed open: the pads' leading edge ends just past where the handle's face is at the open stop.
    assert M.HANDLE_OPEN_Y - 0.010 <= M.OPEN_END_Y - M.FINGER_PUSH < M.HANDLE_OPEN_Y
    assert M.OPEN_START_Y - M.FINGER_PUSH >= M.HANDLE_SHUT_Y + 0.02
    # Pushed shut: the pads' leading edge ends just past the handle's right face at the shut stop.
    shut_face = M.TAP_XY[1] - M.HANDLE[1] / 2
    assert shut_face < M.SHUT_END_Y + M.FINGER_PUSH <= shut_face + 0.010
    assert M.SHUT_START_Y + M.FINGER_PUSH <= shut_face - M.LEVER_TRAVEL - 0.02


def test_the_fixed_geometry_leaves_the_room_the_demo_needs():
    spout_tip_y = M.DRIP_WORLD[1]
    # The bottle under the spout: 2 cm under it, 2 cm over the rim, inside the cup's opening with 1 cm to spare.
    mouth_z = M.POUR_Z - M.GRIP[2] + M.BOTTLE_SIZE[2] / 2
    bottom_z = M.POUR_Z - M.GRIP[2] - M.BOTTLE_SIZE[2] / 2
    assert M.DRIP_WORLD[2] - mouth_z >= 0.02 - 1e-9 and bottom_z - M.RIM_Z >= 0.02 - 1e-9
    assert M.BOTTLE_SIZE[1] / 2 + 0.01 <= M.CUP_INNER / 2
    # The holder's bar passes 2 cm over the cup's left wall while the bottle is under the spout.
    bar_bottom = M.POUR_Z - M.GRIP[2] + M.BAR_Z - M.BAR[2] / 2
    assert bar_bottom - M.RIM_Z >= 0.02
    assert M.BAR_Y0 < M.CUP_INNER / 2 + M.CUP_WALL < M.BAR_Y1  # it does cross the wall's line
    # The approach lane beside the spout: the bottle's right face 2 cm past the spout's tip.
    bottle_right = M.APPROACH_Y - M.GRIP[1] - M.BOTTLE_SIZE[1] / 2
    assert bottle_right - spout_tip_y >= 0.02 - 1e-9
    # Carrying: the bottle's bottom is 6 cm over the cup's rim, and its path stays 5 cm left of the lever.
    carried_bottom = M.HOVER_Z - M.GRIP[2] - M.BOTTLE_SIZE[2] / 2
    assert carried_bottom - M.RIM_Z >= 0.06 - 1e-9
    assert bottle_right >= M.HANDLE_SHUT_Y + 0.05
    left = M.demo()["left"]
    labels = [w.label for w in left]
    carried = left[labels.index("grip") + 1:labels.index("release")]
    assert all(w.pos[1] - M.GRIP[1] - M.BOTTLE_SIZE[1] / 2 >= bottle_right - 1e-9
               for w in carried if w.pos[2] >= M.HOVER_Z)
    # Picking: the pads, 4 cm apart, come down 1 cm either side of the post and 2 cm clear of the bottle's rim.
    pad_gap = -2 * M.GRIP_OPEN
    assert pad_gap - M.POST[1] >= 0.02
    assert (M.GRIP[1] - pad_gap / 2 - PAD_X) - M.BOTTLE_SIZE[1] / 2 >= 0.02 - 1e-9
    assert M.POST[0] / 2 <= PAD_Y <= M.POST[0] / 2 + 0.01  # the pads cover the post's face, little overhang
    # The finger tips stop a centimetre above the bar that joins the post to the bottle.
    assert (M.GRIP[2] - 0.013) - (M.BAR_Z + M.BAR[2] / 2) >= 0.010
    # The finger carriages, 3 cm above the tool point, are above the bottle's top while it is held.
    assert M.GRIP[2] - FINGER_Z[1] >= M.BOTTLE_SIZE[2] / 2


# -- a kinematic replay of the demo, with a box sweep of the gripper and its wrist camera ----------------------------


def aabb(centre, half):
    centre, half = np.asarray(centre, dtype=float), np.asarray(half, dtype=float)
    return centre - half, centre + half


def rotated_aabb(tcp, rot, lo, hi):
    """World bounds of a box given in the gripper frame (``lo``..``hi``) at a tool pose."""
    corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    world = np.asarray(tcp) + corners @ np.asarray(rot).T
    return world.min(0), world.max(0)


def gap(a, b) -> float:
    """Metres between two world boxes, 0 if they overlap."""
    d = np.maximum(np.maximum(a[0] - b[1], b[0] - a[1]), 0.0)
    return float(np.linalg.norm(d))


def gripper_boxes(tcp, rot, q: float, side: str) -> dict:
    """The gripper's box model at a tool pose: pads, finger carriages, body, wrist link and the wrist camera."""
    q = abs(q)
    fx = max(FINGER_X, 0.024 + q)
    cam = np.array(station.wrist_camera(side).pos) - np.array(TCP_OFFSET)  # the camera, in the gripper's frame
    local = {
        "pads": ((-(PAD_X + q), -PAD_Y, PAD_Z[0]), (PAD_X + q, PAD_Y, PAD_Z[1])),
        "fingers": ((-fx, -FINGER_Y, FINGER_Z[0]), (fx, FINGER_Y, FINGER_Z[1])),
        "body": ((-BODY[0], -BODY[1], BODY[2][0]), (BODY[0], BODY[1], BODY[2][1])),
        "link5": ((-LINK5[0], -LINK5[1], LINK5[2][0]), (LINK5[0], LINK5[1], LINK5[2][1])),
        "camera": (cam - CAMERA_HALF, cam + CAMERA_HALF),
    }
    return {name: rotated_aabb(tcp, rot, lo, hi) for name, (lo, hi) in local.items()}


def fixed_boxes() -> dict:
    """World boxes of everything that does not move: the tap's pipe, flange and spout, the cup, the lamp, the mark."""
    out = {}
    for i, (centre, size, _) in enumerate(M.SAMPLE_TAP.boxes):
        out[("tap", i)] = aabb(np.array(M.SAMPLE_TAP.pos) + centre, np.array(size) / 2)
    for obj in TASK.objects:
        if obj.name.startswith("waste_cup") or obj.name in ("flow_lamp", "mark"):
            out[(obj.name, 0)] = aabb(obj.pos, np.array(obj.size) / 2)
    return out


def handle_box(lever: float):
    centre = np.array(M.SAMPLE_TAP.pos) + np.array(M.HANDLE_REST) + np.array([0.0, lever, 0.0])
    return aabb(centre, np.array(M.HANDLE) / 2)


def replay():
    """Step the demo at 20 Hz: the lever is pushed by whichever pads overlap it, the bottle follows the tool point
    rigidly while the pads are closed on its post. Returns the end states, the milestones and the least clearance of
    each gripper part from each obstacle class."""
    arms = plans()
    left, right = arms["left"], arms["right"]
    handle_x = M.TAP_XY[0] + M.HANDLE_REST[0]
    handle_lo, handle_hi = M.TOP + M.HANDLE_Z - M.HANDLE[2] / 2, M.TOP + M.HANDLE_Z + M.HANDLE[2] / 2
    hy = M.HANDLE[1] / 2

    def push(lever, tcp, prev):
        if abs(tcp[0] - handle_x) > M.HANDLE[0] / 2 + PAD_X:
            return lever
        if not (tcp[2] + PAD_Z[0] < handle_hi and tcp[2] + PAD_Z[1] > handle_lo):
            return lever
        lo, hi = tcp[1] - M.FINGER_PUSH, tcp[1] + M.FINGER_PUSH
        face_lo, face_hi = M.TAP_XY[1] + lever - hy, M.TAP_XY[1] + lever + hy
        if hi > face_lo and lo < face_hi:
            if tcp[1] < prev[1]:
                lever = lo - hy - M.TAP_XY[1]
            elif tcp[1] > prev[1]:
                lever = hi + hy - M.TAP_XY[1]
        return float(min(max(lever, -M.LEVER_TRAVEL), 0.0))

    fixed = fixed_boxes()
    least = {}

    def note(key, value):
        least[key] = min(least.get(key, 9.0), value)

    lever, held, origin = 0.0, False, np.array(M.BOTTLE.pos, dtype=float)
    prev = {"left": None, "right": None}
    opened_at = first_drop_at = None
    lamp = []
    M._reset({})
    for t in np.arange(0.0, max(left.duration, right.duration) + 1.0, 1.0 / HZ):
        poses = {}
        for side, arm in (("left", left), ("right", right)):
            tcp, rot, finger, _ = arm.target(t)
            poses[side] = (np.asarray(tcp, dtype=float), np.asarray(rot), float(finger))
        if prev["left"] is not None:
            lever = push(lever, poses["left"][0], prev["left"])
            lever = push(lever, poses["right"][0], prev["right"])
        prev = {side: poses[side][0] for side in poses}
        tcp_l, _, finger_l = poses["left"]
        gripping = finger_l > -M.POST[1] / 2  # pads nearer than the post is wide: closed on it
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
        # The sweep: every gripper part against everything it must not touch.
        boxes = {side: gripper_boxes(*poses[side], side) for side in poses}
        handle = handle_box(lever)
        bottle_box = (bottle.low, bottle.high)
        for side, parts in boxes.items():
            # In the lane straight above the post, up to where the pad tips are a centimetre over the bottle's top,
            # the open pads straddle the post inside the bottle's hull on purpose.
            to_grip = poses[side][0] - bottle.point("grip")
            at_post = bool(np.linalg.norm(to_grip[:2]) < 0.025 and -0.005 < to_grip[2] < 0.05)
            for name, box in parts.items():
                for key, obstacle in fixed.items():
                    note((side, name, key[0]), gap(box, obstacle))
                if name != "pads":
                    note((side, name, "handle"), gap(box, handle))
                if not (held and side == "left") and not at_post:
                    note((side, name, "bottle"), gap(box, bottle_box))
                for other_name, other in boxes["right" if side == "left" else "left"].items():
                    note((side, name, "other arm"), gap(box, other))
        if held:
            for key, obstacle in fixed.items():
                note(("bottle", "", key[0]), gap(bottle_box, obstacle))
            note(("bottle", "", "handle"), gap(bottle_box, handle))
    led = M.LEDGER  # a snapshot: other tests reset the task's ledger
    outcome = {"waste": led.waste, "bottle": led.bottle, "spilled": led.spilled, "open": led.open,
               "waste_at_first_drop": led.waste_at_first_drop, "flushed": led.flushed,
               "grade": M.grade(led, tap, bottle, False), "success": M.success({"sample_tap": tap, "bottle": bottle})}
    M._reset({})
    return tap, bottle, opened_at, first_drop_at, lamp, tcp_l, least, outcome


REPLAY = replay()


def test_a_kinematic_replay_of_the_demo_scores_one():
    tap, bottle, opened_at, first_drop_at, lamp, tcp_l, _, led = REPLAY
    assert opened_at is not None and first_drop_at is not None
    assert led["waste_at_first_drop"] >= M.FLUSH_ML + M.FLOW_ML_S  # the flush ran at least 1 s longer than the rule
    assert first_drop_at - opened_at >= M.FLUSH_ML / M.FLOW_ML_S + 1.0
    assert led["flushed"]
    assert M.FILL_RANGE[0] + 15.0 <= led["bottle"] <= M.FILL_RANGE[1] - 15.0  # a second's flow short of each line
    assert led["spilled"] == 0.0
    assert M.tap_closed(tap) and not led["open"] and M.tap_standing(tap)
    assert M.on_mark(bottle)
    assert np.linalg.norm(tcp_l - bottle.point("grip")) > 0.05  # the gripper has retreated
    assert led["grade"] == pytest.approx(1.0) and led["success"]
    # The lamp was green from the push until the right arm shut the tap, and is off at the end.
    assert lamp[int(opened_at * HZ) + 1] == "green" and lamp[-1] == "off"
    assert lamp.index("off", int(opened_at * HZ) + 1) * 1.0 / HZ > first_drop_at + M.FILL_HOLD_S


def test_the_fill_hold_leaves_room_either_side_of_the_lines():
    """About 7.4 s of flow at 15 mL/s lands in the bottle: the hold, plus the slide in and out under the stream."""
    low, high = M.FILL_RANGE
    assert low / M.FLOW_ML_S + 1.0 <= M.FILL_HOLD_S + 1.4 <= high / M.FLOW_ML_S - 1.0


def test_the_wrist_cameras_stay_clear_of_the_tap_throughout_the_demo():
    """The reason the pushes are made with the gripper turned: at yaw 0 the camera sits 7 cm in front of the tool
    point, inside the pipe and spout at every lever pose."""
    least = REPLAY[6]
    for side in ("left", "right"):
        assert least[(side, "camera", "tap")] >= 0.03, (side, least[(side, "camera", "tap")])
        for obstacle in ("waste_cup_f", "waste_cup_b", "waste_cup_l", "waste_cup_r", "flow_lamp", "handle"):
            assert least[(side, "camera", obstacle)] >= 0.02, (side, obstacle, least[(side, "camera", obstacle)])
        assert least[(side, "camera", "other arm")] >= 0.02
    assert least[("left", "camera", "bottle")] >= 0.02


def test_the_gripper_stays_clear_of_the_tap_the_cup_and_the_bottle_throughout_the_demo():
    least = REPLAY[6]
    for side in ("left", "right"):
        for name in ("fingers", "body", "link5"):
            for obstacle in ("tap", "waste_cup_f", "waste_cup_b", "waste_cup_l", "waste_cup_r", "flow_lamp"):
                assert least[(side, name, obstacle)] >= 0.02, (side, name, obstacle, least[(side, name, obstacle)])
            # The finger carriages start 3 cm above the tool point and pass 1.3 cm over the handle's top as the pads
            # below them push it; the body and wrist link stay well clear.
            assert least[(side, name, "handle")] >= (0.01 if name == "fingers" else 0.015), (side, name)
            assert least[(side, name, "other arm")] >= 0.02
        assert least[(side, "pads", "tap")] >= 0.02 and least[(side, "pads", "other arm")] >= 0.02
        for obstacle in ("waste_cup_f", "waste_cup_b", "waste_cup_l", "waste_cup_r", "flow_lamp"):
            assert least[(side, "pads", obstacle)] >= 0.02
    # The left gripper never touches the bottle except at its post; the right never comes near it.
    assert least[("left", "pads", "bottle")] >= 0.01 and least[("left", "fingers", "bottle")] >= 0.01
    assert least[("left", "body", "bottle")] >= 0.01
    assert all(least[("right", name, "bottle")] >= 0.05 for name in ("pads", "fingers", "body", "link5", "camera"))
    # The carried bottle and its holder keep 1.5 cm from the tap, the cup's walls, the lamp and the lever.
    for obstacle in ("tap", "waste_cup_f", "waste_cup_b", "waste_cup_l", "waste_cup_r", "flow_lamp", "handle"):
        assert least[("bottle", "", obstacle)] >= 0.015, (obstacle, least[("bottle", "", obstacle)])


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
    for number in ("91", "130", "168", "0.25", "150 mL", "30 mL", "20 degrees", "2 cm"):
        assert number in spec["scoring"], number
    assert (HERE / "README.md").read_text().count(M.INSTRUCTION) == 2
