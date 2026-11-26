"""Focus on support cases that can look identical in a single side frame."""

import math
import unittest

import numpy as np

from geometry import PHYSICS_HZ, contact_height, pitch
from scoring import FRONT_CORNERS, HOLD_S, WEIGHT, TentScore


def sample_geometry():
    pitches = (-50., 231.25)
    hinge = np.array((0.34, -0.046, contact_height(*pitches)))
    poses, contacts = [], []
    for angle, corners in zip(pitches, FRONT_CORNERS, strict=True):
        half = math.radians(angle) / 2
        poses.append(np.r_[hinge, 0., math.sin(half), 0., math.cos(half)])
        points = corners @ pitch(angle).T + hinge
        forces = np.tile((0., 0., WEIGHT / 4), (2, 1))
        contacts.append((points, forces))
    return np.array(poses), contacts


class TentScorerTests(unittest.TestCase):
    def setUp(self):
        self.poses, self.contacts = sample_geometry()
        self.scorer = TentScore()

    def sample(self, *, contacts=None, robot_force=0., angle=281.25, steps=1):
        for _ in range(steps):
            self.scorer.sample(self.poses, np.zeros((2, 6)), angle,
                               self.contacts if contacts is None else contacts, robot_force, 0.)

    def test_continuous_hold(self):
        self.sample(steps=int(PHYSICS_HZ * HOLD_S))
        self.assertEqual(self.scorer.score, .5)
        self.sample()
        self.assertEqual(self.scorer.score, 1.)
        self.sample(robot_force=.06)
        self.assertEqual(self.scorer.score, 0.)
        self.sample(steps=PHYSICS_HZ // 2)
        self.assertEqual(self.scorer.score, 0.)

    def test_cannot_substitute_contact_elsewhere_on_the_case(self):
        points, forces = self.contacts[0]
        off_edge = points + pitch(-50.) @ [0.020, 0., 0.]
        self.sample(contacts=[(off_edge, forces), self.contacts[1]])
        self.assertTrue(self.scorer.metrics['shape'])
        self.assertFalse(self.scorer.metrics['bearing'])

    def test_two_edges_near_desk_with_only_one_load_bearing(self):
        points, forces = self.contacts[0]
        self.sample(contacts=[(points, 2 * forces), (points[:0], forces[:0])])
        self.assertTrue(self.scorer.metrics['shape'])
        self.assertFalse(self.scorer.metrics['bearing'])


if __name__ == '__main__':
    unittest.main()
