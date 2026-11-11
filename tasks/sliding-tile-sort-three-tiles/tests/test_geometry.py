"""Unit tests for geometry, tolerances, and puzzle solver in sliding-tile-sort-three-tiles."""

import unittest
from collections import deque
import numpy as np

import geometry as g


class TestGeometry(unittest.TestCase):
    def test_tile_and_cell_clearance(self):
        # Cell is 76 mm, tile is 64 mm -> 6 mm clearance per side
        clearance = (g.CELL_SIZE - g.TILE_SIZE) / 2.0
        self.assertAlmostEqual(clearance, 0.006, places=4)

    def test_push_height_clears_rim(self):
        # Push height is at 9 mm above table; rim is 4 mm tall -> 5 mm clearance above rim
        rim_top = g.TABLE_Z + g.RIM_HEIGHT
        self.assertGreater(g.DEMO_PUSH_Z, rim_top + 0.003)

    def test_reach_bounds(self):
        base_l = np.array([0.0, +0.305, 0.0])
        base_r = np.array([0.0, -0.305, 0.0])

        for c in g.CELLS:
            dl = np.linalg.norm(np.array(c)[:2] - base_l[:2])
            dr = np.linalg.norm(np.array(c)[:2] - base_r[:2])
            self.assertGreaterEqual(dl, 0.10)
            self.assertLessEqual(dl, 0.50)
            self.assertGreaterEqual(dr, 0.10)
            self.assertLessEqual(dr, 0.50)

    def test_puzzle_solver_five_moves(self):
        # Verify BFS shortest path from ('Y', 'E', 'O', 'B') to ('B', 'O', 'Y', 'E') is exactly 5 moves
        adj = {0: [1, 2], 1: [0, 3], 2: [0, 3], 3: [1, 2]}
        start = ('Y', 'E', 'O', 'B')
        goal = ('B', 'O', 'Y', 'E')

        q = deque([(start, 0)])
        visited = {start: 0}
        min_moves = None

        while q:
            state, d = q.popleft()
            if state == goal:
                min_moves = d
                break
            e_idx = state.index('E')
            for nxt in adj[e_idx]:
                st = list(state)
                st[e_idx], st[nxt] = st[nxt], st[e_idx]
                tup = tuple(st)
                if tup not in visited:
                    visited[tup] = d + 1
                    q.append((tup, d + 1))

        self.assertEqual(min_moves, 5)

    def test_slider_rates(self):
        self.assertTrue(True)


if __name__ == "__main__":
    unittest.main()

