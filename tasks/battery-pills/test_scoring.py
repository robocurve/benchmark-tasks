"""scoring.py = the card's scorer.py (r7) + SimScorer. Checks: the card's 77 negative-control rows and 3 sweep checks
(the same as `python3 -I card_scorer.py`), the rule part byte-identical to the card's, and SimScorer on synthetic poses,
including the r7 trial-wide spill flag (review TK-8X2ZNN)."""
import hashlib
import math
import unittest
from pathlib import Path

import numpy as np

import geometry as G
import scoring as SC

HERE = Path(__file__).resolve().parent


class CardRows(unittest.TestCase):
    def test_rows(self):
        rows = SC.rows()
        self.assertEqual(len(rows), 77)
        for i, (name, st, exp, outline) in enumerate(rows, 1):
            st.pop('_slots', None)
            SC.set_outline(*(outline or (SC.OUTLINE['x'], SC.OUTLINE['y'])))
            try:
                self.assertAlmostEqual(SC.score(st), exp, msg=f'row {i}: {name}')
            finally:
                SC.set_outline(SC.OUTLINE['x'], SC.OUTLINE['y'])

    def test_sweep(self):
        draws = SC.blind_draws()
        best = max(sum(SC.score(SC.plan_end(p, d)) for d in draws) / len(draws) for p in SC.PLANS)
        self.assertLessEqual(best, SC.P['blind_sweep']['mean_limit'])
        self.assertAlmostEqual(best, 1 / 7, places=3)
        self.assertTrue(all(SC.score(SC.start_state(d)) == 0.0 for d in draws))
        self.assertTrue(all(SC.score(SC.full_state(d)) == 1.0 for d in draws))

    def test_rule_is_the_cards(self):
        card = (HERE / 'card_scorer.py').read_text()
        ours = (HERE / 'scoring.py').read_text()
        rule = card.split("if __name__ == '__main__':")[0].split('import json, math, os, sys', 1)[1]
        self.assertIn(rule, ours)
        self.assertEqual(hashlib.sha256((HERE / 'card_scorer.py').read_bytes()).hexdigest(),
                         '6f08be230464e33cc3201a39cc0ede4caf4d768fb3db83c892ad98acf3a8acf2')


def _poses(lay, cell=None, dish_shift=(0, 0, 0), jar_cell=False, tray_dish=False):
    """Synthetic world poses from a layout (mm): optionally the dish (with tablets) moved onto the tray and the cell
    dropped in the jar."""
    I = np.eye(3)
    p = {}
    tray = (*lay['tray'], 0.0)
    p['tray'] = (np.array(G.world(tray)), I)
    p['jar'] = (np.array(G.world((*lay['jar'], 0.0))), I)
    dish = np.array([*lay['dish'], 0.0])
    shift = np.zeros(3)
    if tray_dish:
        shift = np.array([lay['tray'][0] - 15 - dish[0], lay['tray'][1] - dish[1], G.TRAY_FLOOR])
    p['dish'] = (np.array(G.world(dish + shift)), I)
    for i, t in enumerate(lay['tablets']):
        p[f'tablet{i}'] = (np.array(G.world(np.array(t) + shift)), I)
    c = np.array(lay['cell'])
    if jar_cell:
        c = np.array([lay['jar'][0] + 3, lay['jar'][1] - 2, G.JAR_FLOOR + G.CELL_H / 2])
    elif tray_dish:
        c = c + shift
    p['cell'] = (np.array(G.world(c)), I)
    return p


class SimAdapter(unittest.TestCase):
    def _run(self, poses, steps=25, grip=(200.0, 200.0), before=None, gap_xy=(200.0, 200.0)):
        s = SC.SimScorer(20, G.TABLE_TOP)
        r = None
        for p in before or []:               # earlier steps (e.g. a tablet lying out of the dish), then the end poses
            s.update(p, lambda n: grip, read_robot=False, grip_xy=lambda n: gap_xy)
        for k in range(steps):
            r = s.update(poses, lambda n: grip, read_robot=(k % 10 == 0 or k == steps - 1), grip_xy=lambda n: gap_xy)
        return r

    def _out_then_back(self, gap_xy):
        """tablet0 lies on the table 80 mm from the dish axis for 5 steps, then everything ends served."""
        lay = G.layout(1)
        end = _poses(lay, jar_cell=True, tray_dish=True)
        out = dict(_poses(lay, jar_cell=True))
        dish = np.array([*lay['dish'], G.TAB_H / 2])
        out['tablet0'] = (np.array(G.world(dish + np.array([80.0, 0.0, 0.0]))), np.eye(3))
        return self._run(end, before=[out] * 5, gap_xy=gap_xy)

    def test_spill_caps_at_half(self):
        r = self._out_then_back((50.0, 200.0))
        self.assertTrue(r['state']['spilled'])
        self.assertEqual(r['score'], 0.5, r['conditions'])

    def test_carried_tablet_is_no_spill(self):
        r = self._out_then_back((3.0, 200.0))          # between or under the fingers in the top view
        self.assertFalse(r['state']['spilled'])
        self.assertEqual(r['score'], 1.0, r['conditions'])

    def test_spill_gap_boundary(self):
        g = SC.S['spill_gap_mm']
        self.assertEqual(self._out_then_back((g - 0.01, 200.0))['score'], 1.0)
        self.assertEqual(self._out_then_back((g, 200.0))['score'], 0.5)

    def test_start_scores_zero(self):
        for seed in (0, 1, 2, 3):
            r = self._run(_poses(G.layout(seed)))
            self.assertEqual(r['score'], 0.0)

    def test_full_scores_one(self):
        for seed in (0, 1, 2, 3, 4, 5):
            r = self._run(_poses(G.layout(seed), jar_cell=True, tray_dish=True))
            self.assertEqual(r['score'], 1.0, (seed, r['conditions']))

    def test_cell_in_dish_on_tray_scores_zero(self):
        r = self._run(_poses(G.layout(1), tray_dish=True))
        self.assertEqual(r['score'], 0.0)

    def test_stowed_only(self):
        r = self._run(_poses(G.layout(2), jar_cell=True))
        self.assertEqual(r['score'], 0.25)

    def test_gripper_near_scores_zero(self):
        r = self._run(_poses(G.layout(1), jar_cell=True, tray_dish=True), grip=(29.9, 200.0))
        self.assertEqual(r['score'], 0.0)

    def test_short_window_is_not_still(self):
        r = self._run(_poses(G.layout(1), jar_cell=True, tray_dish=True), steps=10)
        self.assertEqual(r['score'], 0.0)

    def test_tilted_dish_centre(self):
        # dish tipped 90 deg about x: its centre is 16 mm off the bottom along world -y
        lay = G.layout(0)
        poses = _poses(lay)
        a = math.radians(90)
        rot = np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])
        poses['dish'] = (poses['dish'][0], rot)
        r = self._run(poses)
        self.assertAlmostEqual(r['state']['dish']['y'], lay['dish'][1] - 16.0, places=6)


class Layout(unittest.TestCase):
    def test_slots_and_gaps(self):
        for seed in range(0, 40):
            lay = G.layout(seed)
            d = lay['draw']
            self.assertNotEqual(d['cell'], d['empty'])
            self.assertEqual(len(lay['tablets']), 5)
            cx, cy = lay['dish']
            for t in lay['tablets'] + [lay['cell']]:
                self.assertLessEqual(math.hypot(t[0] - cx, t[1] - cy) + G.TAB_R, G.DISH_FLOOR_R - 9.0)
            for t in lay['tablets']:
                self.assertGreaterEqual(math.dist(t[:2], lay['cell'][:2]) - G.TAB_R - G.CELL_R, 9.3 - 1e-9)
            self.assertTrue(-10 <= d['tray'][0] <= 10 and -15 <= d['tray'][1] <= 15)
            self.assertTrue(0 <= d['phi'] <= 60)

    def test_nominal_is_the_scorers(self):
        self.assertEqual(G.draws(0)['cell'], SC.NOM['cell'])
        self.assertEqual(G.draws(0)['empty'], SC.NOM['empty'])


if __name__ == '__main__':
    unittest.main()
