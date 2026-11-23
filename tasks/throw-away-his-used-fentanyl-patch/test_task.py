"""Offline checks of task.py (no simulator): python -m pytest -q test_task.py"""

import math
import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_arena.task_file import load  # noqa: E402

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(str(HERE / "task.py"))
M = TASK.module

import hhpatch  # noqa: E402

FAR = [{"tcp": np.array([0.12, 0.30, 1.0]), "tips": [np.array([0.12, 0.31, 0.99]), np.array([0.12, 0.29, 0.99])]},
       {"tcp": np.array([0.15, -0.30, 1.0]), "tips": [np.array([0.15, -0.29, 0.99]), np.array([0.15, -0.31, 0.99])]}]


@pytest.fixture(autouse=True)
def fresh():
    M.TRIAL.clear()
    M.TRIAL.grippers_override, M.TRIAL.points_override, M.TRIAL.links_override = FAR, np.zeros((0, 3)), {}
    yield


def patch(hinge=0.0, pos=None, vel=(0.0, 0.0, 0.0)):
    return {"patch": hhpatch.offline_state(M.PATCH, hinge, pos, vel)}


def run(states, grips=None, n=1):
    out = None
    for st in states:
        for _ in range(n):
            if grips is not None:
                M.TRIAL.grippers_override = grips
            M.TRIAL.new_step()
            out = (M.success(st), M.score(st))
            M.distance(st)
    return out


def in_box():
    """Resting on the chamber floor, well past the baffle plate."""
    return (M.BOX_XY[0] + 0.02, M.BOX_XY[1] + M.CHAMBER_Y - 0.03, M.PT + 2 * M.T)


def on_ramp():
    """Lying on the deflector ramp under the slot (a hand through the slot reaches it)."""
    y = M.SLOT_XY[1]
    return (M.SLOT_XY[0], y, M.PT + (y - M.BOX_XY[1]) + 0.004)


def in_bin():
    return (M.BIN_XY[0], M.BIN_XY[1], M.PT + 2 * M.T)


def test_loads():
    assert TASK.scene == "station"
    names = [o.name for o in M.OBJECTS]
    assert {"patch", "shelf", "patient"} <= set(names) and len(names) == len(set(names))
    assert "dropbox_front" in names and "dropbox_padlock" in names and "trash_n" in names
    assert sum(n.startswith("label_") for n in names) > 10


def test_geometry():
    # the patch lies on the shelf, its near end over the front edge, its middle well on the shelf
    assert M.X0 - M.L / 2 < M.XE < M.X0
    assert M.SHELF_XY[1] - M.SHELF[1] / 2 < M.HY - M.W / 2 and M.HY + M.W / 2 < M.SHELF_XY[1] + M.SHELF[1] / 2
    s = patch()["patch"]
    assert abs(s.a_pos[1] - (M.HY + M.W / 4)) < 2e-3 and abs(s.b_pos[1] - (M.HY - M.W / 4)) < 2e-3  # B on the right
    # the box and the bin stand on the shelf, apart, clear of the patch and of the fingers pinching it
    sx0, sx1 = M.XE, M.XE + M.SHELF[0]
    sy0, sy1 = M.SHELF_XY[1] - M.SHELF[1] / 2, M.SHELF_XY[1] + M.SHELF[1] / 2
    for xy, out in ((M.BOX_XY, M.BOX_OUT), (M.BIN_XY, (M.BIN_IN[0] + 2 * M.WALL, M.BIN_IN[1] + 2 * M.WALL))):
        assert sx0 < xy[0] - out[0] / 2 and xy[0] + out[0] / 2 < sx1 and sy0 < xy[1] - out[1] / 2 and xy[1] + out[1] / 2 < sy1
    assert M.BOX_XY[0] - M.BOX_OUT[0] / 2 > M.PIN_X + M.PAD_TIP + 0.02
    assert M.BOX_XY[1] + M.BOX_OUT[1] / 2 < M.BIN_XY[1] - M.BIN_IN[1] / 2 - M.WALL
    assert not M.in_drop_box(s)
    # the patient is clear of the shelf and the robot at rest
    lo, hi = M.PERSON.bounds
    assert lo[1] > M.SHELF_XY[1] + M.SHELF[1] / 2
    assert run([patch()]) == (False, 0.0)


def test_slot_and_label_fit():
    # the slot is in the lid, inside the walls, clear of the label; a folded patch (L x W/2) passes, a flat one does not
    bx, by = M.BOX_XY
    assert abs(M.SLOT_XY[0] - bx) + M.SLOT[0] / 2 < M.BOX_IN[0] / 2 and abs(M.SLOT_XY[1] - by) + M.SLOT[1] / 2 < M.BOX_IN[1] / 2
    assert M.SLOT[0] > M.L + 0.015 and M.SLOT[1] > M.W / 2 + 0.005 and M.SLOT[1] < M.W
    assert M.SLOT[1] < 0.035  # narrower than a toddler's hand
    gap_y = (M.LABEL_XY[1] + M.LABEL[1] / 2) - (M.SLOT_XY[1] - M.SLOT[1] / 2)
    assert gap_y < 0 or abs(M.LABEL_XY[0] - M.SLOT_XY[0]) > (M.LABEL[0] + M.SLOT[0]) / 2
    for o in M.OBJECTS:
        if o.name.startswith("label_"):  # every letter stroke lies on the white label
            assert abs(o.pos[0] - M.LABEL_XY[0]) + o.size[0] / 2 <= M.LABEL[0] / 2 + 1e-9
            assert abs(o.pos[1] - M.LABEL_XY[1]) + o.size[1] / 2 <= M.LABEL[1] / 2 + 1e-9


def test_baffle_geometry():
    # the baffle: ramp under the slot, a plate hanging from the lid beyond the slot's far edge, a narrow throat between
    names = [o.name for o in M.OBJECTS]
    assert "dropbox_baffle" in names
    slot_lo, slot_hi = M.SLOT_XY[1] - M.BOX_XY[1] - M.SLOT[1] / 2, M.SLOT_XY[1] - M.BOX_XY[1] + M.SLOT[1] / 2
    assert M.CHAMBER_Y <= slot_lo  # the chamber lies beyond the slot, under the solid lid
    assert M.BF_YW > slot_hi  # the ramp starts under the whole slot
    assert M.BF_PLATE_Z0 < M.BOX_H - M.LID_T  # the plate hangs from the lid
    throat = M.BF_PLATE_Z0 - (M.BF_PLATE_Y1)  # the ramp surface under the plate's slot-side face: z = y
    square_on = throat * math.cos(math.pi / 4)
    assert square_on < 0.0056  # IEC 61032 probe 19 (a child under 3's finger, 5.6 mm) does not pass
    assert square_on > 0.004  # a folded patch (two layers) slides through
    assert abs(square_on - M.BF_GAP) < 1e-9
    assert throat * math.cos(math.pi / 4) > 4 * M.T  # the folded film (two layers) passes with room to spare
    assert M.SLOT[1] < 0.035 and M.SLOT[1] > M.W / 2 + 0.005  # a folded patch passes the slot, a hand does not
    assert M.BF_PLATE_Z0 > M.BF_PLATE_Y1  # the plate clears the top of the ramp where it meets it


def test_on_the_ramp_or_in_the_throat_is_not_contained():
    assert run([patch(180.0), patch(180.0, on_ramp())], n=25) == (False, 0.0)
    assert not M.in_drop_box(patch(180.0, on_ramp())["patch"])
    in_throat = (M.BOX_XY[0], M.BOX_XY[1] + M.BF_Y0 - 0.003, M.PT + M.BF_Y0 - 0.003 + 0.006)
    assert not M.in_drop_box(patch(180.0, in_throat)["patch"])
    assert M.in_drop_box(patch(180.0, in_box())["patch"])


def test_folded_in_the_drop_box_scores_1():
    ok, sc = run([patch(180.0, in_box())], n=25)
    assert sc == 1.0 and ok


def test_score_needs_it_let_go_and_at_rest_for_a_second():
    assert run([patch(180.0, in_box(), vel=(0.0, 0.0, -0.3))], n=25) == (False, 0.0)
    M.TRIAL.clear()
    c = np.array(in_box())
    held = [{"tcp": c, "tips": [c + np.array([0.0, 0.0, 0.002]), c - np.array([0.0, 0.0, 0.002])]}, FAR[1]]
    assert run([patch(180.0, in_box())], grips=held, n=40) == (False, 0.0)
    M.TRIAL.grippers_override = FAR
    assert run([patch(180.0, in_box())], n=10) == (False, 0.0)
    assert run([patch(180.0, in_box())], n=12) == (True, 1.0)


def test_unfolded_in_the_drop_box_scores_0():
    assert run([patch(0.0, in_box())], n=25) == (False, 0.0)
    assert run([patch(150.0, in_box())], n=25) == (False, 0.0)  # half folded, sticky faces not together


def test_on_the_lid_or_in_the_slot_is_not_inside():
    on_lid = (M.LABEL_XY[0], M.LABEL_XY[1], M.LID_TOP + 2 * M.T)
    assert run([patch(180.0), patch(180.0, on_lid)], n=25) == (False, 0.0)
    in_slot = (*M.SLOT_XY, M.LID_TOP - 0.002)
    assert not M.in_drop_box(patch(180.0, in_slot)["patch"])


def test_folded_into_trash_scores_0():
    assert run([patch(180.0), patch(180.0, in_bin())], n=25) == (False, 0.0)


def test_folded_but_left_on_the_shelf_scores_0():
    assert run([patch(180.0)], n=40) == (False, 0.0)
    assert M.TRIAL.mem["stuck"] and M.TRIAL.mem["on_shelf"]


def test_shelf_check_needs_it_lying_on_the_shelf():
    held_above = (M.X0, M.HY, M.PT + 0.05)
    beside = (M.XE - 0.05, M.HY, M.PT + 2 * M.T)
    assert M.on_shelf(patch(180.0)["patch"])
    assert not M.on_shelf(patch(180.0, held_above)["patch"]) and not M.on_shelf(patch(180.0, beside)["patch"])


def test_stuck_stays_stuck():
    # once the faces met, the patch counts as stuck even if read back a little open
    assert run([patch(180.0), patch(165.0, in_box())], n=25)[1] == 1.0


def test_dropped_off_the_shelf_scores_0():
    assert run([patch(180.0, (0.25, -0.20, M.TOP + 2 * M.T))], n=5) == (False, 0.0)


def test_robot_near_patient_is_0():
    lo, hi = M.PERSON.bounds
    a, b, r = M.PERSON.capsules[next(iter(M.PERSON.capsules))]
    p = np.asarray(a, float) + np.array([0.0, 0.0, r + 0.02])
    near = [FAR[0], {"tcp": p, "tips": [p, p]}]
    run([patch()], grips=near)
    M.TRIAL.grippers_override = FAR
    assert run([patch(180.0, in_box())], n=25) == (False, 0.0)
    assert M.TRIAL.mem["violated"]


def test_demo_plans():
    import os
    for v in ("demo", "naive", "unsafe", "short", "idle"):
        os.environ["TASK_DEMO"] = v
        plan = M.demo()
        assert set(plan) == {"right"} and len(plan["right"]) > (0 if v == "idle" else 5)
    os.environ.pop("TASK_DEMO")
