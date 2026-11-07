"""The task file without a simulator: python -m pytest -q test_fix_the_deck_to_match_the_setup_sheet.py

Needs ``yam_arena`` importable (it is in the simulator's environment; elsewhere put a clone of yam-isaac-lab-arena
on PYTHONPATH). Checks the scene loads, the demo is reachable, the outcome table, and that the sheet can be read in the
320 x 176 top camera.
"""

import itertools
import json
import math
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import numpy as np  # noqa: E402
import pytest  # noqa: E402
from yam_arena import station, task_file  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402
from yam_arena.task_file import ObjectState  # noqa: E402
from yam_arena.urdf import quat_xyzw_to_matrix  # noqa: E402

TASK = task_file.load(HERE / "fix_the_deck_to_match_the_setup_sheet.py")
M = TASK.module
UP = (0.0, 0.0, 0.0, 1.0)
HOME = M.OWN_POSITION[M.WRONG]
TABLE_LINEAR = (0.792, 0.524, 0.273)  # the station table's albedo in yam_arena.scene
NEAR_MISS = (M.POSITIONS[HOME][0], M.POSITIONS[HOME][1] - 0.08)  # beside position 6, on the side away from the pink plate


def yaw_quat(deg):
    a = math.radians(deg) / 2
    return (0.0, 0.0, math.sin(a), math.cos(a))


def roll_quat(deg):
    a = math.radians(deg) / 2
    return (math.sin(a), 0.0, 0.0, math.cos(a))


def plate_at(x, y, quat=UP, bottom=None):
    bottom = M.PAD_TOP if bottom is None else bottom
    half = np.array(M.PLATE) / 2
    pos = np.array([x, y, bottom + half[2]], dtype=float)
    return ObjectState(pos=pos, quat_xyzw=np.array(quat, dtype=float), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=pos - half, high=pos + half)


def on(position, dx=0.0, dy=0.0, **kw):
    x, y = M.POSITIONS[position]
    return plate_at(x + dx, y + dy, **kw)


def layout(mapping):
    return {M.plate_name(colour): on(pos) for pos, colour in mapping.items() if colour}


def start():
    return layout(M.START_LAYOUT)


def fixed():
    return layout(M.SHEET_LAYOUT)


def put(objects, colour, state):
    objects[M.plate_name(colour)] = state
    return objects


def test_the_scene_loads():
    assert set(TASK.free_objects) == {M.plate_name(c) for c in M.COLOURS}
    assert "setup sheet" in TASK.docs


def test_one_plate_is_wrong_and_its_home_is_the_hole():
    wrong = [n for n in M.POSITIONS if M.START_LAYOUT[n] != M.SHEET_LAYOUT[n]]
    assert wrong == sorted((M.WRONG_FROM, HOME))
    assert M.START_LAYOUT[M.WRONG_FROM] == M.WRONG and M.START_LAYOUT[HOME] is None
    assert M.SHEET_LAYOUT[M.WRONG_FROM] is None
    assert sorted(c for c in M.SHEET_LAYOUT.values() if c) == sorted(M.COLOURS)


def test_the_sheet_only_fits_the_way_it_is_laid():
    """Turned or mirrored, the sheet would call for more than one plate to move, so there is one way to read it."""
    grid = [[1, 2], [3, 4], [5, 6]]
    readings = {
        "turned": [row[::-1] for row in grid[::-1]],
        "mirrored left to right": [row[::-1] for row in grid],
        "mirrored front to back": grid[::-1],
    }
    for name, cells in readings.items():
        sheet = {n: M.SHEET_LAYOUT[c] for n, c in zip(itertools.chain(*grid), itertools.chain(*cells))}
        moved = {M.START_LAYOUT[n] for n in M.POSITIONS if M.START_LAYOUT[n] != sheet[n]} - {None}
        assert len(moved) >= 2, name


def test_fixing_the_deck_scores_one():
    assert M.grade(fixed()) == 1.0


def test_doing_nothing_scores_zero():
    assert M.grade(start()) == 0.0


def test_a_plate_placed_a_little_off_still_counts():
    assert M.grade(put(fixed(), M.WRONG, on(HOME, 0.019, -0.019))) == 1.0
    assert M.grade(put(fixed(), M.WRONG, on(HOME, quat=yaw_quat(14)))) == 1.0
    assert M.grade(put(fixed(), M.WRONG, on(HOME, quat=yaw_quat(180)))) == 1.0


def test_right_position_but_rough_is_partial():
    assert M.grade(put(fixed(), M.WRONG, on(HOME, 0.030))) == pytest.approx(0.7)
    assert M.grade(put(fixed(), M.WRONG, on(HOME, quat=yaw_quat(40)))) == pytest.approx(0.7)
    assert M.grade(put(fixed(), M.WRONG, on(HOME, quat=yaw_quat(90)))) == pytest.approx(0.7)


def test_setting_the_wrong_plate_down_beside_its_position_is_partial():
    assert M.grade(put(start(), M.WRONG, plate_at(*NEAR_MISS, bottom=M.TOP))) == pytest.approx(0.3)
    x, y = M.POSITIONS[HOME]
    assert M.grade(put(start(), M.WRONG, plate_at(x - 0.08, y, bottom=M.TOP))) == pytest.approx(0.3)


def test_shoving_the_wrong_plate_off_its_position_earns_nothing():
    x, y = M.POSITIONS[M.WRONG_FROM]
    for pushed in ((x, y + 0.045), (x + 0.06, y), (x, 0.30)):
        assert M.grade(put(start(), M.WRONG, plate_at(*pushed, bottom=M.TOP))) == 0.0


def test_knocking_the_wrong_plate_aside_earns_nothing_wherever_it_lands():
    for spot in ((0.30, -0.22), (0.21, 0.175), (0.45, 0.28), (0.10, 0.30)):
        assert M.grade(put(start(), M.WRONG, plate_at(*spot, bottom=M.TOP))) == 0.0


def test_near_miss_stops_at_ten_centimetres():
    x, y = M.POSITIONS[HOME]
    assert M.grade(put(start(), M.WRONG, plate_at(x - 0.099, y, bottom=M.TOP))) == pytest.approx(0.3)
    assert M.grade(put(start(), M.WRONG, plate_at(x - 0.101, y, bottom=M.TOP))) == 0.0


def test_no_nudge_can_reach_the_near_miss_from_the_start():
    """Every point on the yellow plate's own pad is further from position 6 than the near miss reaches."""
    x, y = M.POSITIONS[M.WRONG_FROM]
    gx, gy = M.POSITIONS[HOME]
    closest = np.hypot(x - M.PAD[0] / 2 - gx, y - M.PAD[1] / 2 - gy)
    assert closest > M.BESIDE


def test_nudging_a_correct_plate_halves_the_score():
    assert M.grade(put(fixed(), "green", on(3, 0.030))) == pytest.approx(0.5)
    objects = put(fixed(), M.WRONG, on(HOME, 0.030))
    assert M.grade(put(objects, "green", on(3, 0.030))) == pytest.approx(0.35)
    objects = put(start(), M.WRONG, plate_at(*NEAR_MISS, bottom=M.TOP))
    assert M.grade(put(objects, "blue", on(2, dy=0.030))) == pytest.approx(0.15)


def test_a_small_bump_to_a_correct_plate_is_free():
    """Open fingers clear the next plate by 1.7 cm, so a finger coming down a little wide doesn't cost anything."""
    assert M.grade(put(fixed(), "pink", on(5, dy=0.019))) == 1.0
    assert M.grade(put(fixed(), "green", on(3, quat=yaw_quat(14)))) == 1.0


def test_a_correct_plate_pushed_off_its_pad_scores_zero():
    assert M.grade(put(fixed(), "pink", on(5, dy=0.039))) == pytest.approx(0.5)
    assert M.grade(put(fixed(), "pink", on(5, dy=0.041))) == 0.0


def test_a_correct_plate_turned_on_its_pad_counts_as_nudged():
    assert M.grade(put(fixed(), "green", on(3, quat=yaw_quat(30)))) == pytest.approx(0.5)


def test_the_wrong_plate_moved_to_any_other_position_scores_zero():
    for n in M.POSITIONS:
        if n != HOME:
            assert M.grade(put(start(), M.WRONG, on(n))) == 0.0


def test_moving_a_correct_plate_into_the_hole_scores_zero():
    """The obvious shortcut: the hole is plain to see, without reading the sheet."""
    for colour in M.OTHERS:
        assert M.grade(put(start(), colour, on(HOME))) == 0.0


def test_fixing_the_plate_but_taking_another_off_the_deck_scores_zero():
    assert M.grade(put(fixed(), "orange", plate_at(0.30, -0.22, bottom=M.TOP))) == 0.0


def test_taking_a_correct_plate_away_instead_scores_zero():
    assert M.grade(put(start(), "orange", plate_at(0.30, -0.22, bottom=M.TOP))) == 0.0


def test_shuffling_the_whole_deck_scores_zero():
    swapped = {1: "blue", 2: None, 3: "green", 4: "orange", 5: "pink", 6: "yellow"}
    assert M.grade(layout(swapped)) == 0.0
    rotated = {1: "green", 2: "blue", 3: "orange", 4: "pink", 5: None, 6: "yellow"}
    assert M.grade(layout(rotated)) == 0.0


def test_stacking_plates_scores_zero():
    stacked = M.PAD_TOP + M.PLATE[2]
    assert M.grade(put(start(), M.WRONG, on(5, bottom=stacked))) == 0.0
    objects = fixed()
    assert M.grade(put(objects, "pink", on(HOME, bottom=stacked))) == 0.0


def test_knocking_a_plate_off_the_table_scores_zero():
    assert M.grade(put(start(), M.WRONG, plate_at(0.30, 0.60, bottom=M.TOP - 0.40))) == 0.0
    assert M.grade(put(fixed(), "blue", plate_at(0.30, 0.60, bottom=M.TOP - 0.40))) == 0.0


def test_a_plate_still_in_the_gripper_scores_zero():
    for height in (0.006, 0.02, 0.05):
        held = plate_at(*M.POSITIONS[HOME], bottom=M.PAD_TOP + height)
        assert M.grade(put(fixed(), M.WRONG, held)) == 0.0
    x, y = NEAR_MISS
    assert M.grade(put(start(), M.WRONG, plate_at(x, y, bottom=M.TOP + 0.012))) == 0.0


def test_a_plate_on_its_edge_earns_nothing():
    on_edge = roll_quat(90)
    assert M.grade(put(fixed(), M.WRONG, on(HOME, quat=on_edge))) == 0.0
    assert M.grade(put(start(), M.WRONG, plate_at(0.30, -0.22, quat=on_edge, bottom=M.TOP))) == 0.0


def test_the_score_never_passes_full_marks_for_less_than_a_fixed_deck():
    states = [fixed(), start(), put(fixed(), M.WRONG, on(HOME, 0.03)), put(fixed(), "blue", on(2, 0.03))]
    assert [M.grade(s) >= 1.0 for s in states] == [True, False, False, False]


def test_success_needs_the_plates_still():
    objects = fixed()
    assert M.success(objects)
    moving = objects[M.plate_name(M.WRONG)]
    objects[M.plate_name(M.WRONG)] = ObjectState(pos=moving.pos, quat_xyzw=moving.quat_xyzw,
                                                 lin_vel=np.array([0.1, 0.0, 0.0]), ang_vel=np.zeros(3),
                                                 low=moving.low, high=moving.high)
    assert not M.success(objects)


def test_distance_falls_to_zero_at_home():
    assert M.distance(start()) > 0.25
    assert M.distance(fixed()) == pytest.approx(0.0)


def test_nothing_overlaps_on_the_table():
    pads = list(M.POSITIONS.values())
    for a, b in itertools.combinations(pads, 2):
        assert abs(a[0] - b[0]) >= M.PAD[0] or abs(a[1] - b[1]) >= M.PAD[1]
    assert M.PLATE[0] + 2 * M.IN_PLACE < 2 * M.PAD[0]
    for box in M.OBJECTS:
        if box.name.startswith(("pad", "pip")):
            assert abs(box.pos[0] - M.DECK_CENTRE[0]) + box.size[0] / 2 <= M.DECK_SIZE[0] / 2, box.name
            assert abs(box.pos[1] - M.DECK_CENTRE[1]) + box.size[1] / 2 <= M.DECK_SIZE[1] / 2, box.name
    deck_right = M.DECK_CENTRE[1] - M.DECK_SIZE[1] / 2
    sheet_left = M.SHEET_XY[1] + M.SHEET_SIZE[1] / 2
    assert sheet_left < deck_right
    for pos in M.POSITIONS:
        cx, cy = M.sheet_cell(pos)
        assert abs(cx - M.SHEET_XY[0]) + M.BLOCK[0] / 2 <= M.SHEET_SIZE[0] / 2
        assert abs(cy - M.SHEET_XY[1]) + M.BLOCK[1] / 2 <= M.SHEET_SIZE[1] / 2


def test_open_fingers_clear_the_next_plate():
    finger_outer = 0.047 + 0.004  # 94 mm between the pads, 4 mm of finger behind each
    gap = abs(M.COLUMN_Y["left"] - M.COLUMN_Y["right"]) - M.PLATE[1] / 2 - finger_outer
    assert gap > 0.015


def _pixel(point, width=320, height=176):
    cam = station.top_camera("calibrated")
    rot = quat_xyzw_to_matrix(np.array(cam.rot_xyzw))
    p = rot.T @ (np.array(point) - np.array(cam.pos))
    lens = cam.lens.resized(width, height)
    return np.array([lens.cx + lens.fx * p[0] / -p[2], lens.cy - lens.fy * p[1] / -p[2]])


def test_the_sheet_reads_in_the_top_camera():
    z = M.TOP + M.SHEET_H
    for position in M.POSITIONS:
        cx, cy = M.sheet_cell(position)
        hx, hy = M.BLOCK[0] / 2, M.BLOCK[1] / 2
        across = np.linalg.norm(_pixel((cx, cy + hy, z)) - _pixel((cx, cy - hy, z)))
        along = np.linalg.norm(_pixel((cx + hx, cy, z)) - _pixel((cx - hx, cy, z)))
        assert across >= 8.0 and along >= 9.5  # px
    corners = [(M.SHEET_XY[0] + sx * M.SHEET_SIZE[0] / 2, M.SHEET_XY[1] + sy * M.SHEET_SIZE[1] / 2, z)
               for sx in (-1, 1) for sy in (-1, 1)]
    for corner in corners:
        u, v = _pixel(corner)
        assert 0 <= u < 320 and 0 <= v < 176


def test_the_sheet_shows_blocks_with_gaps_and_an_empty_cell():
    z = M.TOP + M.SHEET_H
    for a, b in ((1, 3), (3, 5), (2, 4), (4, 6)):
        ax, ay = M.sheet_cell(a)
        bx, by = M.sheet_cell(b)
        gap = abs(_pixel((ax - M.BLOCK[0] / 2, ay, z))[1] - _pixel((bx + M.BLOCK[0] / 2, by, z))[1])
        assert gap >= 1.5  # px
    blocks = {box.name: box.color for box in M.OBJECTS}
    assert blocks[f"sheet_block{M.WRONG_FROM}"] == M.PAD_GREY
    assert blocks[f"sheet_block{HOME}"] == M.COLOURS[M.WRONG]


def _linear(srgb):
    c = np.array(srgb, dtype=float)
    return np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4)


def _lit(linear):
    """A top-camera pixel for a matte albedo: an ACES tone curve with gain 2 and a veil of 0.05. An estimate, fitted to
    pixels of earlier station renders (the albedo 0.75, 0.08, 0.08 came out 214, 60, 61; the table 227, 211, 178)."""
    into = np.array([[0.59719, 0.35458, 0.04823], [0.07600, 0.90834, 0.01566], [0.02840, 0.13383, 0.83777]])
    out = np.array([[1.60475, -0.53108, -0.07367], [-0.10208, 1.10813, -0.00605], [-0.00327, -0.07276, 1.07602]])
    x = into @ (2.0 * np.array(linear) + 0.05)
    x = (x * (x + 0.0245786) - 0.000090537) / (x * (0.983729 * x + 0.4329510) + 0.238081)
    x = np.clip(out @ x, 0.0, 1.0)
    return np.where(x <= 0.0031308, 12.92 * x, 1.055 * x ** (1 / 2.4) - 0.055)


def _lab(srgb):
    linear = _linear(srgb)
    xyz = np.array([[0.4124, 0.3576, 0.1805], [0.2126, 0.7152, 0.0722], [0.0193, 0.1192, 0.9505]]) @ linear
    t = xyz / np.array([0.9505, 1.0, 1.089])
    f = np.where(t > 0.008856, np.cbrt(t), 7.787 * t + 16 / 116)
    return np.array([116 * f[1] - 16, 500 * (f[0] - f[1]), 200 * (f[1] - f[2])])


def test_the_colours_stay_apart_under_the_station_lamp():
    """Plate against plate, and each plate against what it is seen on: a pad, the sheet, or the table."""
    plates = {name: _lab(_lit(_linear(rgb))) for name, rgb in M.COLOURS.items()}
    for (a, la), (b, lb) in itertools.combinations(plates.items(), 2):
        assert np.linalg.norm(la - lb) > 55.0, (a, b)
    for name, lab in plates.items():
        for ground in (M.SHEET_WHITE, M.PAD_GREY):
            assert np.linalg.norm(lab - _lab(_lit(_linear(ground)))) > 55.0, name
        assert np.linalg.norm(lab - _lab(_lit(TABLE_LINEAR))) > 40.0, name


def test_every_demo_waypoint_is_reachable():
    plan = M.demo()
    assert set(plan) == {"left"}
    for arm, waypoints in plan.items():
        a = ArmPlan(list(waypoints), station.BASES[arm])
        for t in np.arange(0.0, a.duration + 0.5, 1 / 20):
            a.action7(t)
        assert a.worst_ik_mm < 0.5 and a.worst_ik_deg < 0.1
        assert a.duration * 20 + 60 < 1800


def test_every_position_can_be_reached_by_the_left_arm():
    """So no position is out of reach for a policy that moves another plate, and the layout can be changed."""
    for start_at, end_at in itertools.permutations(M.POSITIONS, 2):
        a = ArmPlan(M.move_plate(start_at, end_at), station.BASES["left"])
        for t in np.arange(0.0, a.duration + 0.5, 1 / 5):
            a.action7(t)
        assert a.worst_ik_mm < 0.5 and a.worst_ik_deg < 0.1, (start_at, end_at)


def test_left_arm_stays_on_its_side():
    for w in M.demo()["left"]:
        assert w.pos[1] >= 0.0


def test_the_scoring_text_gives_the_code_s_numbers():
    text = json.loads((HERE / "task.json").read_text())["scoring"]
    for phrase in (f"leaning under {M.FLAT_DEG:g} degrees", f"bottom under {M.DOWN_ON_DECK * 100:g} cm above it",
                   f"bottom under {M.SEATED * 100:g} cm above it", f"within {M.IN_PLACE * 100:g} cm",
                   f"turned under {M.SQUARE_DEG:g} degrees", f"bottom under {M.ON_TABLE * 100:g} cm above the table",
                   f"centre under {M.BESIDE * 100:g} cm from 6's centre"):
        assert phrase in text, phrase
    for value, milestone in ((M.MISSED * M.NUDGED, "beside 6, another not"), (M.MISSED, "beside 6, others"),
                             (M.ROUGH * M.NUDGED, "down on 6, not in place, another"), (M.FIXED * M.NUDGED, "in place on 6, another"),
                             (M.ROUGH, "down on 6, not in place, others"), (M.FIXED, "in place on 6, others")):
        assert f"{round(value, 2)}: yellow {milestone}" in text, milestone


def test_the_setup_text_gives_the_code_s_numbers():
    text = json.loads((HERE / "task.json").read_text())["setup"]
    for phrase in (f"{M.DECK_SIZE[0] * 100:g} x {M.DECK_SIZE[1] * 100:g} x {M.DECK_H * 100:g} cm",
                   f"{M.PAD[0] * 100:g} x {M.PAD[1] * 100:g} x {M.PAD_H * 100:g} cm",
                   f"{M.PLATE[0] * 100:g} x {M.PLATE[1] * 100:g} x {M.PLATE[2] * 100:g} cm", f"{M.PLATE_MASS * 1000:g} g",
                   f"{M.SHEET_SIZE[0] * 100:g} x {M.SHEET_SIZE[1] * 100:g} x {M.SHEET_H * 100:g} cm",
                   f"at x {M.SHEET_XY[0] * 100:g}, y {M.SHEET_XY[1] * 100:g}", f"{M.BLOCK[0] * 100:g} x {M.BLOCK[1] * 100:g} cm",
                   f"{M.SHEET_SCALE:g} scale"):
        assert phrase in text, phrase


def test_task_json_matches_the_task_file():
    spec = json.loads((HERE / "task.json").read_text())
    assert spec["instruction"] == M.INSTRUCTION
    assert spec["category"] == "Impact on jobs"
    assert spec["run_limit"] == "2,400 steps (2 minutes at 20 Hz)"
    assert len(spec["fidelity"]) <= 400 and len(spec["scores"]) <= 300
    assert len(spec["setup"]) <= 750 and len(spec["scoring"]) <= 750 and len(spec["context"]) < 450
    assert len(spec["title"]) < 55
    assert (HERE / "README.md").read_text().count(M.INSTRUCTION) == 2
