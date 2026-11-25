"""Offline outcome and geometry checks. Run with python -m unittest -v."""

from dataclasses import replace
import unittest

import numpy as np

from geometry import (
    AXIS, Outcome, Pad, SETTLE_STEPS, STRAP_SEGMENT, STRAP_THICK, TABLE_Z,
    open_strap, pad_mates, press_samples, radial, score_outcome, strap_winding,
    engaged_cells, HOOK_FACE, mating_points, CELL_AREA, STRAP_WIDTH, PAD_LENGTH,
    PARTIAL_WRAP, set_down,
)


class UmbrellaChecks(unittest.TestCase):
    # gathered, wound 366 degrees, fastened, released, resting on the table after a lowered set-down
    finished = Outcome(0.030, np.deg2rad(366), True, True, True, True, False, True, True, True)

    def test_end_scores_and_release_hold(self):
        end = self.finished
        self.assertEqual(score_outcome(end, SETTLE_STEPS), 1.0)
        for changes, credit in [
            ({"fingers_clear": False}, 0.75),
            ({"both_clear": False}, 0.75),
            ({"still": False}, 0.75),
            ({"set_down": False}, 0.75),                       # dropped onto the table
            ({"latched": False}, 0.5),                         # strap not fastened
            ({"mated": False}, 0.5),                           # hook face no longer engaged
            ({"winding": np.deg2rad(364.9)}, 0.5),             # no overlap: not once round
            ({"winding": PARTIAL_WRAP - 0.01}, 0.25),
            ({"winding": 1.5, "latched": False, "mated": False}, 0.25),
            ({"pleat_radius": 0.040}, 0.0),                    # canopy not gathered
            ({"table_supported": False}, 0.0),                 # neither held nor on the table
            ({"table_supported": False, "handle_held": True}, 0.75),  # fastened but still in a hand
            ({"handle_held": True}, 1.0),
            ({"winding": float("nan")}, 0.0),
            ({"pleat_radius": float("nan")}, 0.0),
        ]:
            with self.subTest(changes=changes):
                self.assertEqual(score_outcome(replace(end, **changes), SETTLE_STEPS), credit)
        # Retention: the released wrap must stay gathered, wound and fastened for a full second.
        self.assertEqual(score_outcome(end, SETTLE_STEPS - 1), 0.75)

    def test_score_has_no_technique_terms(self):
        # The outcome records the end state only; no handle turn or hand assignment enters it.
        self.assertEqual(set(Outcome.__dataclass_fields__), {
            "pleat_radius", "winding", "mated", "latched", "fingers_clear", "still", "handle_held",
            "table_supported", "both_clear", "set_down"})

    def test_set_down_needs_table_contact_before_release(self):
        self.assertTrue(set_down(True, True, False))     # lowered onto the table, then let go
        self.assertFalse(set_down(True, False, False))   # let go above the table: dropped
        self.assertFalse(set_down(True, True, True))     # still held
        self.assertFalse(set_down(False, True, False))   # rolled or slid off afterwards

    def test_gathering_alone_cannot_complete_the_task(self):
        from geometry import RELAXED_ENVELOPE_RADIUS
        snug = replace(self.finished, pleat_radius=0.79 * RELAXED_ENVELOPE_RADIUS)
        loose = replace(self.finished, pleat_radius=0.81 * RELAXED_ENVELOPE_RADIUS)
        self.assertEqual(score_outcome(snug, SETTLE_STEPS), 1.0)
        self.assertEqual(score_outcome(loose, SETTLE_STEPS), 0.0)
        self.assertLess(score_outcome(replace(snug, latched=False), SETTLE_STEPS), 1.0)
        self.assertLess(score_outcome(replace(snug, table_supported=False), SETTLE_STEPS), 1.0)

    def test_hook_and_loop_requires_a_sustained_loaded_press(self):
        pressed = dict(mated=True, held=True, touching=True, normal_load=0.2)
        count = 0
        for expected in (1, 2, 3):
            count = press_samples(count, **pressed)
            self.assertEqual(count, expected)
        for change in (dict(normal_load=0.14), dict(normal_load=float("nan")), dict(mated=False),
                       dict(touching=False), dict(held=False)):
            self.assertEqual(press_samples(2, **(pressed | change)), 0)

    def test_patch_overlap_distance_and_orientation(self):
        base = Pad(np.zeros(3), np.eye(3))
        self.assertTrue(pad_mates(base, Pad(np.array([.003, 0, STRAP_THICK]), np.eye(3))))
        for point in ([0, 0, .009], [.018, 0, STRAP_THICK], [0, .012, STRAP_THICK],
                      [0, 0, -STRAP_THICK]):
            with self.subTest(point=point):
                self.assertFalse(pad_mates(base, Pad(np.array(point), np.eye(3))))
        self.assertFalse(pad_mates(base, Pad(np.array([0, 0, STRAP_THICK]), np.diag([1, -1, -1]))))

    def test_aligned_patch_area_and_clear_gap(self):
        base = Pad(np.zeros(3), np.eye(3))
        full_contact = Pad(np.array([0., 0., STRAP_THICK]), np.eye(3))
        separated = Pad(np.array([0., 0., 0.009]), np.eye(3))
        self.assertAlmostEqual(len(mating_points(base, full_contact)) * CELL_AREA,
                               STRAP_WIDTH * PAD_LENGTH)
        self.assertEqual(len(mating_points(base, separated)), 0)
        self.assertEqual(press_samples(2, mated=pad_mates(base, separated),
                                       held=True, touching=True, normal_load=1.0), 0)

    def test_initial_strip_is_open_above_the_table_and_constant_length(self):
        strap = open_strap()
        self.assertTrue(np.allclose(np.linalg.norm(np.diff(strap, axis=0), axis=1), STRAP_SEGMENT))
        self.assertGreater(min(strap[:, 2]) - TABLE_Z, STRAP_THICK)
        self.assertLess(strap_winding(strap, AXIS), 4.7)

    def test_bonded_rows_accumulate_and_peel(self):
        base = Pad(np.zeros(3),np.eye(3))
        tip = Pad(np.array([0.,0.,STRAP_THICK]),np.eye(3))
        hits = tip.pos + HOOK_FACE
        cells = engaged_cells({},base,tip,hits[:8])
        self.assertEqual(len(cells),8)
        cells = engaged_cells(cells,base,tip,hits[5:16])
        self.assertEqual(len(cells),16)
        away = Pad(tip.pos+[0.,0.,.002],tip.rotation)
        self.assertFalse(engaged_cells(cells,base,away,[]))
        slid = Pad(tip.pos+[.004,0.,0.],tip.rotation)
        self.assertFalse(engaged_cells(cells,base,slid,[]))

    def test_signed_winding_rejects_backtracking(self):
        theta = np.linspace(0.12, 6.28, 13)
        closed = np.array([AXIS + 0.030 * radial(a) for a in theta])
        self.assertGreater(strap_winding(closed, AXIS), 5.7)
        self.assertLess(strap_winding(closed[::-1], AXIS), 0.0)
        trace = np.concatenate([closed[:7], closed[:7][::-1]])
        self.assertAlmostEqual(strap_winding(trace, AXIS), 0.0)
