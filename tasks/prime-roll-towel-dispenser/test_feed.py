import math
import unittest
import numpy as np

from feed import Feed, LEADER_RESET, NIP_X, NIP_Z, LEAD_LENGTH


class FeedContract(unittest.TestCase):
    def setUp(self):
        self.feed = Feed()
        self.axes = np.array([np.diag([-1., 1., -1.]), np.diag([1., -1., -1.])])
        self.clear = np.array([[.25, .2, 1.1], [.25, -.2, 1.1]])
        self.open = np.array([.074, .074])
        self.closed = np.array([.002, .002])

    def tick(self, tcp=None, fingers=None, lever=0.):
        self.feed.update(self.clear if tcp is None else tcp, self.axes,
                         self.open if fingers is None else fingers, math.radians(lever))

    def start_paper(self):
        self.tick(LEADER_RESET)
        self.tick(LEADER_RESET, self.closed)

    def seat(self):
        self.start_paper()
        points = LEADER_RESET.copy()
        points[:, 0] = NIP_X
        points[:, 2] = NIP_Z + LEAD_LENGTH
        self.tick(points, self.closed)
        self.assertTrue(self.feed.seated)

    def test_empty_strokes_do_not_feed(self):
        for _ in range(10):
            self.tick(lever=-35.)
            self.tick()
        self.assertEqual(self.feed.travel, 0.)
        self.assertEqual(self.feed.score, 0.)

    def test_skewed_leader_cannot_seat(self):
        self.start_paper()
        points = LEADER_RESET.copy()
        points[:, 0] = NIP_X
        points[:, 2] = [NIP_Z + LEAD_LENGTH, NIP_Z + LEAD_LENGTH + .010]
        self.tick(points, self.closed)
        self.tick(lever=-35.)
        self.assertFalse(self.feed.seated)
        self.assertEqual(self.feed.travel, 0.)

    def test_feed_and_released_dwell(self):
        self.seat()
        self.assertEqual(self.feed.score, .25)
        for _ in range(3):
            self.tick(lever=-35.)
            self.tick()
        self.assertEqual(self.feed.score, .5)
        self.tick(lever=-35.)
        for _ in range(9):
            self.tick()
        self.assertEqual(self.feed.score, .5)
        self.tick()
        self.assertEqual(self.feed.score, 1.)

    def test_rattle_and_reverse_do_not_advance(self):
        self.seat()
        self.tick(lever=-5.)
        a = self.feed.travel
        for _ in range(10):
            self.tick(lever=-4.)
            self.tick(lever=-5.)
        self.assertEqual(self.feed.travel, a)
        self.tick()
        self.assertEqual(self.feed.travel, a)

    def test_overstrain_prevents_any_credit(self):
        self.start_paper()
        points = LEADER_RESET.copy()
        points[0, 1] += .060
        self.tick(points, self.closed)
        self.assertTrue(self.feed.damaged)
        self.assertEqual(self.feed.score, 0.)


if __name__ == '__main__':
    unittest.main()
