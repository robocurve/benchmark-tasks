import unittest

from secondary_lock_checks import Unlock, evaluate


class InterlockCases(unittest.TestCase):
    def test_both_locks_must_clear_before_withdrawal(self):
        for safety, latch, tip_clear in ((0., 0., False), (.0016, .002, True),
                                         (.0032, .0008, False), (.0032, .002, False)):
            lock = Unlock()
            lock.update(safety, latch, .001, True, tip_clear, True)
            lock.update(safety, latch, .005, True, tip_clear, True)
            self.assertTrue(lock.invalid)
            self.assertFalse(lock.escaped)

    def test_returning_to_reset_requires_a_new_unlock(self):
        lock = Unlock()
        lock.update(.0032, .002, .001, True, True, True)
        lock.update(.0032, .002, .005, True, True, True)
        self.assertTrue(lock.escaped)
        lock.update(0., 0., 0., True, False, False)
        self.assertFalse(lock.escaped)
        lock.update(0., 0., .005, True, False, False)
        self.assertTrue(lock.invalid)

    def test_unloaded_latch_rebound_does_not_clear(self):
        lock = Unlock()
        self.assertEqual(lock.update(.0032, .0017, .001, True, True, False), (True, False))
        lock.update(.0032, .0017, .005, True, True, False)
        self.assertTrue(lock.invalid)

    def test_alignment_must_continue_through_the_shoulder(self):
        lock = Unlock()
        lock.update(.0032, .0017, .001, True, True, True)
        lock.update(.0032, .0017, .003, False, True, True)
        lock.update(.0032, .0017, .005, True, True, True)
        self.assertFalse(lock.escaped)
        self.assertTrue(lock.invalid)

    def test_progress_and_release(self):
        self.assertEqual(evaluate(False, False, 0., False, False, False, 0, False), 0.)
        self.assertEqual(evaluate(True, False, 0., False, False, False, 0, False), .25)
        self.assertEqual(evaluate(True, True, .005, False, False, False, 0, False), .50)
        self.assertEqual(evaluate(False, False, .060, True, True, True, 19, False), .75)
        self.assertEqual(evaluate(False, False, .060, True, True, True, 20, False), 1.)
        self.assertEqual(evaluate(False, False, .060, True, True, True, 20, True), 0.)

    def test_physical_locking_paths(self):
        from task import LATCH, SECONDARY, SOCKET_SHOULDERS, latch_points

        def bounds(shape, index):
            center, size, _ = shape
            return center[index] - size[index] / 2, center[index] + size[index] / 2

        rail_gate = SECONDARY.boxes[0]
        pressed_pad = latch_points(LATCH.boxes[-1:], LATCH.travel)
        self.assertGreater(bounds(rail_gate, 1)[1] - SECONDARY.travel / 2, pressed_pad[:, 1].min())
        self.assertLess(bounds(rail_gate, 1)[1] - SECONDARY.travel, pressed_pad[:, 1].min())
        self.assertLess(latch_points(LATCH.boxes[:2], 0.)[:, 2].min(), bounds(SOCKET_SHOULDERS[0], 2)[1])
        self.assertGreater(latch_points(LATCH.boxes[:2], LATCH.travel)[:, 2].min(), bounds(SOCKET_SHOULDERS[0], 2)[1])

    def test_named_housings_and_wire_exits(self):
        from task import PLUG, SOCKET
        self.assertEqual(PLUG.size, (.02533, .03670, .0268))
        self.assertEqual(SOCKET.size, (.02460, .04755, .02245))
        self.assertTrue(SOCKET.mounted)
        self.assertFalse(PLUG.mounted)
        for housing, direction in ((PLUG, -1), (SOCKET, 1)):
            self.assertEqual(len(housing.lead.colors), 4)
            self.assertGreater(direction * housing.lead.points[-1][1], .140)

    def test_mounted_clip_fits_reverse_channel(self):
        from task import MOUNT, SOCKET

        rails = SOCKET.boxes[6:8]
        tongue = MOUNT[-1]
        inside_width = rails[1][0][0] - rails[0][0][0] - rails[0][1][0]
        self.assertAlmostEqual(inside_width, .0098)
        self.assertLess(tongue.size[0], inside_width)
        self.assertLess(tongue.size[1], rails[0][1][1])
