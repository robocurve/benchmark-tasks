"""Offline checks of the lock-free sliding drawstring; not physics evidence."""
import unittest
import numpy as np
from drawstring_physics import SlidingDrawstring


def ring(radius=.09, n=20, z=1.):
    a = np.arange(n) * 2 * np.pi / n
    return np.c_[radius * np.cos(a), radius * np.sin(a), np.full(n, z)]


def scene(loop_out=.02):
    """Hem ring plus the two strand ends of each exposed strap (a, b)."""
    hem = ring()
    ends = np.array([hem[5] + [.008, loop_out, -.01], hem[5] + [-.008, loop_out, -.01],
                     hem[15] + [.008, -loop_out, -.01], hem[15] + [-.008, -loop_out, -.01]])
    return np.r_[hem, ends]


def pull(p, d):
    q = p.copy(); q[20:22, 1] += d; q[22:24, 1] -= d
    return q


class DrawstringPhysicsTests(unittest.TestCase):
    def test_rest_state_is_force_free(self):
        p = scene();s = SlidingDrawstring(p, exit_index=5)
        self.assertTrue(np.allclose(s.forces(p, np.zeros_like(p)), 0))

    def test_pulling_loops_gathers_hem_inward(self):
        p = scene();s = SlidingDrawstring(p, exit_index=5)
        q = pull(p, .05)
        f = s.forces(q, np.zeros_like(q))
        self.assertGreater(s.last['tension'], 0)
        radial = np.einsum('ij,ij->i', f[:20, :2], p[:20, :2])
        others = [i for i in range(20) if i not in (5, 15)]
        self.assertTrue(np.all(radial[others] < 0), radial)   # hem pulled inward
        self.assertTrue(np.all(f[20:22, 1] < 0));self.assertTrue(np.all(f[22:24, 1] > 0))  # strap ends pulled back
        self.assertTrue(np.allclose(f.sum(0), 0, atol=1e-9))   # internal force only

    def test_no_lock_after_release(self):
        """A gathered hem with slack string carries no force: nothing holds it shut."""
        p = scene();s = SlidingDrawstring(p, exit_index=5)
        q = pull(p, .08)
        s.forces(q, np.zeros_like(q))
        released = p.copy();released[:20, :2] *= .5           # mouth gathered, loops back
        f = s.forces(released, np.zeros_like(released))
        self.assertEqual(s.last['tension'], 0.)
        self.assertTrue(np.allclose(f, 0))
        reopened = p.copy()
        self.assertTrue(np.allclose(s.forces(reopened, np.zeros_like(reopened)), 0))

    def test_force_cap(self):
        p = scene(); s = SlidingDrawstring(p, exit_index=5, tension_cap=2.)
        q = pull(p, .5)
        s.forces(q, np.zeros_like(q))
        self.assertLessEqual(s.last['tension'], 2.)

    def test_hem_friction_damps_but_is_bounded(self):
        """The viscous (hem-friction) part adds tension while the string is
        drawn out quickly, removes it while it springs back, and is limited."""
        p = scene(); q = pull(p, .03)
        still = SlidingDrawstring(p, exit_index=5); still.forces(q, np.zeros_like(q))
        v = np.zeros_like(q); v[20:22, 1] = .5; v[22:24, 1] = -.5        # ends moving outward
        out = SlidingDrawstring(p, exit_index=5); out.forces(q, v)
        back = SlidingDrawstring(p, exit_index=5); back.forces(q, -v)
        self.assertGreater(out.last['tension'], still.last['tension'])
        self.assertLess(back.last['tension'], still.last['tension'])
        fast = SlidingDrawstring(p, exit_index=5); fast.forces(pull(p, .5), v * 100)
        self.assertLessEqual(fast.last['tension'], 2 * fast.tension_cap + 1e-9)

    def test_symmetric_route(self):
        s = SlidingDrawstring(scene(), exit_index=5)
        self.assertEqual(s.cutouts, (5, 15))
        used = sorted(set(np.concatenate(s.routes).tolist()))
        self.assertEqual(used, list(range(24)))
        # each route runs from one strap end through one hem side to the other strap
        self.assertEqual({int(r[0]) for r in s.routes} | {int(r[-1]) for r in s.routes}, {20, 21, 22, 23})

    def test_one_side_pull_tensions_whole_string(self):
        """Pulling only loop 0 still tensions the single continuous string."""
        p = scene(); s = SlidingDrawstring(p, exit_index=5)
        q = p.copy(); q[20:22, 1] += .06
        f = s.forces(q, np.zeros_like(q))
        self.assertGreater(s.last['tension'], 0)
        self.assertTrue(np.all(f[22:24, 1] > 0))


if __name__ == '__main__':
    unittest.main()
