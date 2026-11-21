"""Offline checks of coatkit (no simulator): python3 test_coatkit.py (or pytest)."""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import coatkit as ck  # noqa: E402

SLAB = ck.Slab((0.115, 0.035, 0.014))
BED = ck.Bed(center=(0.28, 0.0), size=(0.236, 0.160), floor_z=0.001, depth=0.010)
LAW = ck.Law()
FLAT = np.array([0.0, 0.0, 0.0, 1.0])
FLIP = ck.quat(np.diag([1.0, -1.0, -1.0]))  # turned over about its long axis
DOWN_Y = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])  # top-down, fingers closing along world y
IN_BED = np.array([0.28, 0.0, 0.001 + 0.007])


def press(c, pos, q, xs, steps=8, f=3.0):
    for x in xs:
        tcp = np.array([pos[0] + x, pos[1], pos[2] + 0.02])
        for _ in range(steps):
            c.physics(pos, q, f, [tcp], [])
        for _ in range(2):
            c.physics(pos, q, 0.5, [], [])


def grip_at(c, pos, q, x, steps=3, z=0.016):
    g = ck.Grip((pos[0] + x, pos[1], z), DOWN_Y, 0.0172)
    for _ in range(steps):
        c.physics(pos, q, 0.0, [], [g])


def test_lying_or_dropped_sets_nothing():
    c = ck.Coat(SLAB, BED, LAW)
    for _ in range(50):
        c.physics(IN_BED, FLAT, 0.49, [], [])  # its own weight
    for _ in range(2):
        c.physics(IN_BED, FLAT, 12.0, [], [])  # the impact of a drop
    assert c.coverage() == 0.0


def test_short_push_and_push_without_hand_set_nothing():
    c = ck.Coat(SLAB, BED, LAW)
    tcp = IN_BED + np.array([0, 0, 0.02])
    for _ in range(LAW.press_steps - 1):
        c.physics(IN_BED, FLAT, 5.0, [tcp], [])
    c.physics(IN_BED, FLAT, 0.4, [], [])
    for _ in range(20):
        c.physics(IN_BED, FLAT, 5.0, [], [])  # force but no gripper pushing (e.g. the pan alone)
    assert c.coverage() == 0.0


def test_press_sets_bottom_and_lower_sides_near_presser():
    c = ck.Coat(SLAB, BED, LAW)
    press(c, IN_BED, FLAT, [0.0])
    fc = c.face_coverage()
    assert fc["top"] == 0.0
    assert 0.5 < fc["bottom"] < 0.56  # 6 cm of 11.5
    assert 0.0 < fc["+y"] < fc["bottom"]  # lower 1 cm of 1.4 only
    c2 = ck.Coat(SLAB, BED, LAW)
    press(c2, IN_BED, FLAT, [-0.04, 0.0, 0.04])
    fc2 = c2.face_coverage()
    assert fc2["bottom"] == 1.0 and fc2["top"] == 0.0
    assert abs(fc2["+y"] - 4 / 6) < 0.01 and abs(fc2["+x"] - 4 / 6) < 0.01  # rows at or below 1.0 cm


def test_press_outside_the_bed_sets_nothing():
    c = ck.Coat(SLAB, BED, LAW)
    elsewhere = np.array([0.28, 0.195, 0.008])
    press(c, elsewhere, FLAT, [-0.04, 0.0, 0.04])
    assert c.coverage() == 0.0


def test_both_faces_pressed_cover_all():
    c = ck.Coat(SLAB, BED, LAW)
    press(c, IN_BED, FLAT, [-0.04, 0.0, 0.04])
    press(c, IN_BED, FLIP, [-0.04, 0.0, 0.04])
    assert c.coverage() > 0.999


def test_grip_wipes_under_pads_only():
    c = ck.Coat(SLAB, BED, LAW)
    press(c, IN_BED, FLAT, [-0.04, 0.0, 0.04])
    press(c, IN_BED, FLIP, [-0.04, 0.0, 0.04])
    lifted = IN_BED + np.array([0, 0, 0.05])
    grip_at(c, lifted, FLAT, -0.0375, z=lifted[2] + 0.008)
    cov = c.coverage()
    lost = 1 - cov
    assert 0.03 < lost < 0.09, lost  # one grip: the two finger tips' faces on the sides
    fc = c.face_coverage()
    assert fc["top"] == 1.0 and fc["bottom"] == 1.0 and fc["+x"] == 1.0
    grip_at(c, lifted, FLAT, -0.0375, z=lifted[2] + 0.008)  # the same spot again costs nothing more
    assert abs(c.coverage() - cov) < 1e-12
    grip_at(c, lifted, FLAT, 0.03, z=lifted[2] + 0.008)  # a fresh spot strips more
    assert c.coverage() < cov - 0.03


def test_repress_patches_a_wiped_spot():
    c = ck.Coat(SLAB, BED, LAW)
    press(c, IN_BED, FLAT, [-0.04, 0.0, 0.04])
    before = c.coverage()
    grip_at(c, IN_BED, FLAT, 0.0)
    assert c.coverage() < before
    press(c, IN_BED, FLAT, [0.0])
    assert abs(c.coverage() - before) < 1e-12


def test_gripped_cells_not_coated_while_pressing():
    c = ck.Coat(SLAB, BED, LAW)
    g = ck.Grip((IN_BED[0], 0.0, 0.016), DOWN_Y, 0.0172)
    for _ in range(10):
        c.physics(IN_BED, FLAT, 3.0, [np.array([IN_BED[0], 0.0, 0.016])], [g])
    fc = c.face_coverage()
    assert fc["bottom"] > 0.5
    p, n, _ = c.world(IN_BED, FLAT)
    under = ck.under_pads(p, n, g, LAW.pad_reach)
    assert under.any() and not (c.coated & under).any()


def test_classify_fist_vs_grip():
    poses = {"right_robot": ((0.28, 0.0, 0.03), DOWN_Y, 0.0)}
    fist = {("right_robot", "tip_left"): (0, 0, -1.5), ("right_robot", "tip_right"): (0, 0, -1.5)}
    p, g = ck.classify(fist, poses, LAW)
    assert len(p) == 1 and len(g) == 0
    pinch = {("right_robot", "tip_left"): (0, 1.0, 0.3), ("right_robot", "tip_right"): (0, -1.0, 0.3)}
    poses = {"right_robot": ((0.28, 0.0, 0.03), DOWN_Y, 0.017)}
    p, g = ck.classify(pinch, poses, LAW)
    assert len(p) == 0 and len(g) == 1
    light = {("right_robot", "tip_left"): (0, 0.2, 0), ("right_robot", "tip_right"): (0, -0.2, 0)}
    p, g = ck.classify(light, poses, LAW)
    assert not p and not g


def test_cells_and_area():
    c = ck.Coat(SLAB, BED, LAW)
    a = 2 * (0.115 * 0.035 + 0.115 * 0.014 + 0.035 * 0.014)
    assert abs(c.total_area() - a) < 1e-9
    assert len(c.c) > 1500


def test_setup_state_reset():
    ck.setup("tender", SLAB, BED, LAW, pan="breading_pan")
    s = ck.state()
    press(s, IN_BED, FLAT, [0.0])
    assert ck.state().coverage() > 0
    ck.reset()
    assert ck.state().coverage() == 0.0


def test_quats_match_quat():
    rng = np.random.default_rng(1)
    Rs = [ck.rotm(q) for q in rng.normal(size=(300, 4))] + [np.diag([1.0, -1.0, -1.0]), ck.rotm([1, -1, 0, 0])]
    for R, q in zip(Rs, ck.quats(np.array(Rs))):
        assert np.allclose(ck.rotm(q), R, atol=1e-6)


def test_parts_on_hinges():
    """A piece cut into three parts: posed straight part by part, its cells sit where the one-body pose puts them;
    bending a hinge moves only the cells beyond it, and a press there still sets them along that part."""
    cut = SLAB.size[0] / 6
    origins = ((-cut, 0, 0), (0, 0, 0), (cut, 0, 0))
    c = ck.Coat(SLAB, BED, LAW, cuts=(-cut, cut), origins=origins)
    q = FLAT
    pos = np.array([np.asarray(IN_BED) + np.asarray(o) for o in origins])
    a, _, _ = c.world(pos, np.array([q] * 3))
    b, _, _ = c.world(IN_BED, q)
    assert np.allclose(a, b)
    assert set(np.unique(c.part)) == {0, 1, 2}
    bent = np.array([0, math.sin(0.15), 0, math.cos(0.15)])  # far part turned 17 degrees about y
    p2, _, _ = c.world(pos, np.array([q, q, bent]))
    moved = np.linalg.norm(p2 - a, axis=1) > 1e-9
    assert moved.any() and (c.part[moved] == 2).all()


def test_tiles_draw_parts():
    """The flakes' drawing path, with a stand-in for the simulator's markers: three part poses, twice (the second
    call is skipped as unchanged), flakes only on coated cells, one shade index each."""
    cut = SLAB.size[0] / 6
    origins = ((-cut, 0, 0), (0, 0, 0), (cut, 0, 0))
    c = ck.Coat(SLAB, BED, LAW, cuts=(-cut, cut), origins=origins)
    c.coated[: len(c.c) // 3] = True

    class Markers:
        calls = []

        def set_visibility(self, v):
            pass

        def visualize(self, translations, orientations, scales, marker_indices):
            self.calls.append((translations.shape, orientations.shape, scales.shape, marker_indices.shape))

    t = ck.Tiles()
    t.markers = Markers()
    pos = np.array([np.asarray(IN_BED) + np.asarray(o) for o in origins])
    q = np.array([FLAT] * 3)
    t.draw(c, pos, q)
    t.draw(c, pos, q)
    n = int(c.coated.sum())
    assert not t.off and Markers.calls == [((n, 3), (n, 4), (n, 3), (n,))]


def test_tiles_offline_noop():
    t = ck.Tiles()
    c = ck.Coat(SLAB, BED, LAW)
    t.draw(c, IN_BED, FLAT)
    assert t.off


if __name__ == "__main__":
    n = 0
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok  ", name)
            n += 1
    print(f"{n} tests passed")
