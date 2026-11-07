"""Offline checks (no GPU): the task file loads, the sheet's collider leaves every hole clear with the play the parts
need, the scorer covers each outcome, and the demo's pinch, clearances and reach. Run in the yam_arena environment:
python -m pytest -q test_laser_scrap_sheet_lift.py"""

from __future__ import annotations

import math
import pathlib

import numpy as np
import pytest
from yam_arena import ik
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "laser_scrap_sheet_lift.py")
T = TASK.module
import layout as L  # noqa: E402  (on the path once the task file is loaded)
import plate as P  # noqa: E402


def _state(pos, quat=(0.0, 0.0, 0.0, 1.0), vel=(0.0, 0.0, 0.0)):
    pos = np.asarray(pos, float)
    return ObjectState(pos=pos, quat_xyzw=np.asarray(quat, float), lin_vel=np.asarray(vel, float),
                       ang_vel=np.zeros(3), low=pos - 0.05, high=pos + 0.05)


IN_BIN = (L.BIN_XY[0], L.BIN_XY[1], L.TOP + L.BIN_FLOOR + L.SHEET_SIZE[2] / 2)
HELD = (L.SHEET_POS[0], L.SHEET_POS[1], L.SHEET_POS[2] + 0.05)


def _scene(sheet=L.SHEET_POS, moved=(), sheet_vel=(0, 0, 0), part_at=None):
    """The sheet at ``sheet``, every part at its start except those in ``moved`` (1 cm along y), or as given in
    ``part_at``."""
    out = {"sheet": _state(sheet, vel=sheet_vel)}
    for n in T.PART_NAMES:
        p = np.asarray(L.part_pos(n), float)
        if n in moved:
            p = p + np.array([0.0, 0.01, 0.0])
        if part_at and n in part_at:
            p = np.asarray(part_at[n], float)
        out[n] = _state(p)
    return out


# -- the scene -------------------------------------------------------------------------------------------------------


import cutter as C  # noqa: E402


def test_loads():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {"sheet", *T.PART_NAMES}


def test_masses_are_acrylic():
    assert T.SHEET.mass == pytest.approx(0.219, abs=0.003)
    assert sum(p.mass for p in T.PARTS) == pytest.approx(0.0388, abs=0.001)


def _boxes():
    out = {name: (np.asarray(c) - np.asarray(s) / 2, np.asarray(c) + np.asarray(s) / 2)
           for name, c, s in C.placed_body(L.XF, L.YC)}
    out.update({name: (np.asarray(c) - np.asarray(s) / 2, np.asarray(c) + np.asarray(s) / 2)
                for name, c, s in L.bin_parts()})
    return out


def test_sheet_overhangs_the_bed_and_bin_holds_it():
    assert T.BED_LO[0] - (L.SHEET_POS[0] - L.SHEET_SIZE[0] / 2) == pytest.approx(0.03)
    assert L.SHEET_SIZE[0] + 0.02 < L.BIN_INSIDE[0] and L.SHEET_SIZE[1] + 0.02 < L.BIN_INSIDE[1]
    boxes = _boxes()
    assert boxes["bin_right"][0][1] - boxes["cutter_left"][1][1] >= 0.009  # the bin stands clear of the machine
    bed = boxes["cutter_bed"]
    assert bed[1][2] == pytest.approx(L.BED_TOP)
    sheet_lo = np.asarray(L.SHEET_POS) - np.asarray(L.SHEET_SIZE) / 2
    sheet_hi = np.asarray(L.SHEET_POS) + np.asarray(L.SHEET_SIZE) / 2
    assert sheet_lo[1] >= bed[0][1] and sheet_hi[1] <= bed[1][1] and sheet_hi[0] <= bed[1][0]


def test_nothing_fixed_in_the_sheets_way_up():
    """Straight above the sheet, 1.5 cm round it, nothing fixed below the carry height but the bed."""
    lo = np.asarray(L.SHEET_POS) - np.asarray(L.SHEET_SIZE) / 2 - np.array([0.0, 0.015, 0.0])
    hi = np.asarray(L.SHEET_POS) + np.asarray(L.SHEET_SIZE) / 2 + np.array([0.0, 0.015, 0.0])
    hi[2] = T.CARRY_Z + 0.02
    lo[2] = L.BED_TOP + 0.001
    for name, (blo, bhi) in _boxes().items():
        overlap = np.minimum(bhi, hi) - np.maximum(blo, lo)
        assert np.any(overlap <= 0), name


def test_carry_clears_the_side_wall_and_the_bin():
    wall = _boxes()["cutter_left"][1][2]
    assert T.CARRY_Z - 0.055 > wall + 0.01  # the lower finger reaches 5.5 cm below the TCP
    assert T.CARRY_Z - 0.055 > _boxes()["cutter_rail_l"][1][2] + 0.01


def test_open_lid_pieces_inside_its_box_and_above_the_body():
    half = np.asarray(L.LID_OPEN_SIZE) / 2 + 1e-6
    a = math.radians(C.OPEN_DEG)
    for (c, size, yaw, pitch) in L.open_lid_pieces():
        assert pitch == pytest.approx(a) and yaw == 0.0
        for du in (-size[0] / 2, size[0] / 2):
            for dw in (-size[2] / 2, size[2] / 2):
                p = (c[0] + du * math.cos(a) + dw * math.sin(a), c[2] - du * math.sin(a) + dw * math.cos(a))
                assert abs(p[0]) <= half[0] and abs(p[1]) <= half[2]
    lo = L.LID_OPEN_POS[2] - L.LID_OPEN_SIZE[2] / 2
    assert lo > L.TOP + C.H - C.WALL  # it stands above the side walls and the gantry


def _segments_cross(a, b, c, d) -> bool:
    def side(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])

    return side(a, b, c) * side(a, b, d) < 0 and side(c, d, a) * side(c, d, b) < 0


def _gap(poly_a, poly_b) -> float:
    """Distance between two convex polygons' outlines (0 if they overlap)."""
    a, b = np.asarray(poly_a), np.asarray(poly_b)
    for poly, other in ((a, b), (b, a)):
        if any(P.outside(poly, p) < 0 for p in other):
            return 0.0
    for i in range(len(a)):
        for j in range(len(b)):
            if _segments_cross(a[i], a[(i + 1) % len(a)], b[j], b[(j + 1) % len(b)]):
                return 0.0

    def seg_point(p, q, r):
        d = q - p
        s = np.clip(np.dot(r - p, d) / np.dot(d, d), 0.0, 1.0)
        return float(np.linalg.norm(p + s * d - r))

    return min(min(seg_point(a[i], a[(i + 1) % len(a)], r) for i in range(len(a)) for r in b),
               min(seg_point(b[j], b[(j + 1) % len(b)], r) for j in range(len(b)) for r in a))


def test_collider_pieces_stay_in_the_sheet_and_out_of_the_holes():
    hx, hy = L.SHEET_SIZE[0] / 2, L.SHEET_SIZE[1] / 2
    assert 20 <= len(T.SHEET.parts) <= 120
    for piece in T.SHEET.parts:
        corners = P.piece_corners(piece)
        assert np.all(np.abs(corners[:, 0]) <= hx + 1e-9) and np.all(np.abs(corners[:, 1]) <= hy + 1e-9), piece
        assert piece[1][2] == pytest.approx(L.SHEET_SIZE[2])
        for name, hole in T.HOLES.items():
            assert _gap(corners, hole) >= -1e-9 and all(P.outside(hole, c) > -1e-9 for c in corners), (name, piece)
            # no piece reaches into the hole: every point of the piece is outside it
            for s in np.linspace(0, 1, 7):
                for t in np.linspace(0, 1, 7):
                    p = corners[0] + s * (corners[1] - corners[0]) + t * (corners[3] - corners[0])
                    assert P.outside(hole, p) >= -1e-6, (name, piece)


def test_every_hole_wall_is_solid():
    """Just outside every side of every hole, a point lies in some collider piece: the part cannot slip out sideways
    when the sheet moves."""
    def inside_piece(p):
        for (cx, cy, _), (lx, ly, _), yaw in T.SHEET.parts:
            c, s = math.cos(yaw), math.sin(yaw)
            u, v = c * (p[0] - cx) + s * (p[1] - cy), -s * (p[0] - cx) + c * (p[1] - cy)
            if abs(u) <= lx / 2 + 1e-9 and abs(v) <= ly / 2 + 1e-9:
                return True
        return False

    for name, hole in T.HOLES.items():
        a, b, t, n, length = P._edges(hole)
        for i in range(len(a)):
            for s in np.linspace(0.02, 0.98, 9):
                assert inside_piece(a[i] + s * (b[i] - a[i]) + 0.001 * n[i]), (name, i, s)


def test_parts_fit_their_holes_with_the_play():
    for part in T.PARTS:
        u, v = L.PART_UV[part.name]
        outline = getattr(part, "outline", None) or L.rect_outline(0, 0, part.size[0], part.size[1])
        world = [(u + x, v + y) for x, y in outline]
        gap = min(-P.outside(T.HOLES[part.name], p) for p in world)
        assert gap >= L.PLAY - 1e-6, part.name
        assert gap <= L.PLAY * 1.5, part.name  # and no looser than 0.75 mm at the closest corner


def test_holes_keep_their_distance():
    names = list(T.HOLES)
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            assert _gap(T.HOLES[names[i]], T.HOLES[names[j]]) >= 0.008, (names[i], names[j])
    for hole in T.HOLES.values():  # 2 cm of sheet at the sides, 1.2 cm at the far edge, the near 6 cm uncut
        pts = np.asarray(hole)
        assert pts[:, 0].min() >= -L.SHEET_SIZE[0] / 2 + 0.06
        assert pts[:, 0].max() <= L.SHEET_SIZE[0] / 2 - 0.012 + 1e-9
        assert np.abs(pts[:, 1]).max() <= L.SHEET_SIZE[1] / 2 - 0.02
    for name in ("disc", "rect_wide"):  # the two far parts, 1.2 cm from the far edge
        assert np.asarray(T.HOLES[name])[:, 0].max() == pytest.approx(L.SHEET_SIZE[0] / 2 - 0.012, abs=2e-4)


def test_tilted_lift_drags_the_far_parts():
    """Lifted by its near edge with the far edge on the bed, the sheet slides toward the hands by L (1 - cos a). The
    far holes' walls hold the far parts until they have risen past the parts' tops (5 mm at 1.2 cm from the far
    edge: 24.6 degrees), by when the sheet has slid 2 cm, three times the 6 mm a part may move."""
    t, d, length = L.SHEET_SIZE[2], 0.012, L.SHEET_SIZE[0]
    clear = math.asin(t / d)
    assert math.degrees(clear) > 20.0
    assert length * (1 - math.cos(clear)) - L.PLAY > 3 * T.PART_TOL


# -- scoring ---------------------------------------------------------------------------------------------------------


def test_start_scores_zero():
    o = _scene()
    assert T.score(o) == 0.0 and not T.success(o)


def test_sheet_in_bin_with_all_parts():
    o = _scene(sheet=IN_BIN)
    assert T.success(o) and T.score(o) == 1.0


def test_ladder_by_parts_left():
    assert T.score(_scene(sheet=IN_BIN, moved=("disc",))) == 0.75
    assert T.score(_scene(sheet=IN_BIN, moved=("disc", "triangle"))) == 0.5
    assert T.score(_scene(sheet=IN_BIN, moved=("disc", "triangle", "rect_long"))) == 0.0
    assert T.score(_scene(sheet=IN_BIN, moved=T.PART_NAMES)) == 0.0


def test_sheet_off_the_bed_not_in_bin():
    assert T.score(_scene(sheet=HELD)) == 0.25
    assert T.score(_scene(sheet=HELD, moved=("disc",))) == 0.25
    assert T.score(_scene(sheet=HELD, moved=("disc", "triangle"))) == 0.0
    on_table = (0.20, 0.0, L.TOP + 0.0025)
    assert T.score(_scene(sheet=on_table)) == 0.25


def test_sheet_slid_on_the_bed_scores_zero():
    o = _scene(sheet=(L.SHEET_POS[0] + 0.03, L.SHEET_POS[1], L.SHEET_POS[2] + 0.005))
    assert T.score(o) == 0.0


def test_parts_carried_into_the_bin_do_not_count():
    carried = {n: (IN_BIN[0], IN_BIN[1], IN_BIN[2] + 0.003) for n in ("disc", "triangle")}
    assert T.score(_scene(sheet=IN_BIN, part_at=carried)) == 0.5


def test_part_lifted_or_moved_does_not_count():
    up = np.asarray(L.part_pos("disc")) + np.array([0, 0, 0.004])
    assert not T.part_left(_scene(part_at={"disc": up}), "disc")
    near = np.asarray(L.part_pos("disc")) + np.array([0.004, 0.004, 0])
    assert T.part_left(_scene(part_at={"disc": near}), "disc")
    far = np.asarray(L.part_pos("disc")) + np.array([0.0045, 0.0045, 0])
    assert not T.part_left(_scene(part_at={"disc": far}), "disc")
    o = _scene()
    a = math.radians(12) / 2
    o["disc"] = _state(L.part_pos("disc"), quat=(math.sin(a), 0.0, 0.0, math.cos(a)))
    assert not T.part_left(o, "disc")  # flipped up on an edge


def test_moving_sheet_is_not_success_but_scores():
    o = _scene(sheet=IN_BIN, sheet_vel=(0.0, 0.0, -0.3))
    assert not T.success(o) and T.score(o) == 1.0


def test_fallen_or_broken_state_scores_zero():
    assert T.score(_scene(sheet=(0.4, 0.0, L.TOP - 0.3))) == 0.0
    o = _scene(sheet=IN_BIN)
    o["disc"] = _state((np.nan, 0.0, 0.0))
    assert T.score(o) == 0.0 and not T.success(o)


def test_sheet_above_the_rim_is_not_in_the_bin():
    assert not T.sheet_in_bin(_scene(sheet=(IN_BIN[0], IN_BIN[1], L.BIN_RIM + 0.01)))
    assert not T.sheet_in_bin(_scene(sheet=(IN_BIN[0], IN_BIN[1] + 0.13, IN_BIN[2])))


# -- the demo --------------------------------------------------------------------------------------------------------

DT = 0.05


def _play(arm, plan):
    p = ArmPlan(plan, BASES[arm])
    out = []
    t = 0.0
    while t < p.duration + DT:
        q, label = p.action7(t)
        pos, rot = ik.tcp_pose(q[:6], np.asarray(BASES[arm], float))
        out.append((t, np.asarray(pos), np.asarray(rot), q[6], label))
        t += DT
    return p, out


PLAYS = {arm: _play(arm, plan) for arm, plan in T.demo().items()}


@pytest.mark.parametrize("arm", ["left", "right"])
def test_demo_reach(arm):
    p, _ = PLAYS[arm]
    assert p.worst_ik_mm < 2.0 and p.worst_ik_deg < 2.0, (p.worst_ik_mm, p.worst_ik_deg)


def test_demo_arms_move_together():
    d = {arm: [w.duration for w in plan] for arm, plan in T.demo().items()}
    assert d["left"] == d["right"]


@pytest.mark.parametrize("arm", ["left", "right"])
def test_pinch_is_level_and_jaws_close_vertically(arm):
    _, out = PLAYS[arm]
    for t, pos, rot, grip, label in out:
        if label in ("pinch", "lift level", "hold", "carry over the bin"):
            assert abs(rot[2, 0]) > 0.999, (t, label)  # the closing axis (gripper x) vertical
            assert rot[0, 2] > 0.999, (t, label)  # pointing forward along +x


def test_left_wrist_camera_points_out():
    """The wrist camera sits on the gripper's -y face. The left one looks away from the right arm (the right one
    looks toward the left gripper, 16 cm away: its roll keeps joint 6 inside its limit when it tips the sheet)."""
    _, out = PLAYS["left"]
    for t, pos, rot, grip, label in out:
        if label == "pinch":
            assert -rot[1, 1] > 0.9


def test_wrists_stay_apart():
    (_, left), (_, right) = PLAYS["left"], PLAYS["right"]
    closest = min(np.linalg.norm((lp - 0.17 * lr[:, 2]) - (rp - 0.17 * rr[:, 2]))
                  for (_, lp, lr, _, _), (_, rp, rr, _, _) in zip(left, right))
    assert closest >= 0.15
    tcp = min(np.linalg.norm(lp - rp) for (_, lp, _, _, _), (_, rp, _, _, _) in zip(left, right))
    assert tcp >= 0.15


def test_pads_stop_short_of_the_bed():
    assert T.GRIP_X + 0.013 < T.BED_LO[0] - 0.005  # the pad tips 5 mm or more from the bed's near face
    assert T.GRIP_DEPTH + 0.013 <= L.OVERHANG - 0.005


def test_gripper_passes_over_the_front_wall():
    wall = _boxes()["cutter_front"][1][2]
    assert T.PINCH_Z - 0.055 > wall + 0.01  # the lower jaw and gripper body 5.5 cm below the TCP
