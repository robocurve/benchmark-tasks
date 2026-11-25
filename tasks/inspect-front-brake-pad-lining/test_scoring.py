import unittest

from scoring import Measurement


class LiningMeasurementTests(unittest.TestCase):
    def update(self, state, tick, *, floor=False, roof=False, stop=False,
               intact=True, parked=False, recorded=False):
        return state.update(tick, floor, roof, stop, intact, parked, recorded)

    def test_fit_and_physical_stop_survive_park(self):
        state = Measurement()
        for tick in range(1, 5):
            self.assertEqual(self.update(state, tick, floor=True), .25)
        for tick in range(5, 9):
            result = self.update(state, tick, roof=True)
        self.assertEqual(result, .50)
        for tick in range(9, 13):
            result = self.update(state, tick, stop=True)
        self.assertEqual(result, .75)
        self.assertEqual(self.update(state, 13, parked=True), .75)
        self.assertEqual(self.update(state, 14, parked=True, recorded=True), 1.)

    def test_floor_contact_and_proximity_cannot_pass(self):
        state = Measurement()
        for tick in range(1, 20):
            self.update(state, tick, floor=True, parked=True)
        self.assertEqual(state.score, .25)

    def test_fit_cannot_credit_wide_fixture_without_stop(self):
        state = Measurement()
        for tick in range(1, 12):
            self.update(state, tick, floor=True, roof=True, parked=True)
        self.assertEqual(state.score, .50)

    def test_duplicate_samples_cannot_complete_contact_dwell(self):
        state = Measurement()
        self.update(state, 1, floor=True, roof=True, stop=True)
        for _ in range(8):
            self.update(state, 1, floor=True, roof=True, stop=True, parked=True)
        self.assertLess(state.score, .75)

    def test_lost_tool_revokes_reading(self):
        state = Measurement()
        for tick in range(1, 5):
            self.update(state, tick, floor=True, roof=True, stop=True)
        self.assertEqual(self.update(state, 5, intact=False, parked=True), 0.)


class InstalledAccessTests(unittest.TestCase):
    def test_only_an_outboard_crossing_in_a_spoke_opening_arms_the_witness(self):
        import numpy as np
        import geometry as geo

        access = geo.SpokeAccess()
        clear = np.array([.122, -.069, geo.WHEEL_FACE])
        self.assertTrue(geo.wheel_opening(clear))
        self.assertFalse(access.update(clear - [0., 0., .010]))
        self.assertFalse(access.update(clear + [0., 0., .010]))
        self.assertTrue(access.update(clear - [0., 0., .010]))
        self.assertFalse(access.update(clear + [0., 0., .010]))

        through_center_cap = np.array([.02, 0., geo.WHEEL_FACE])
        access.update(through_center_cap + [0., 0., .010])
        self.assertFalse(access.update(through_center_cap - [0., 0., .010]))

    def test_axial_gauge_checks_air_gap_and_rejects_a_spoke_or_stem_contact(self):
        from types import SimpleNamespace
        import numpy as np
        import task
        import geometry as geo

        half = .004
        head = np.array([task.PAD_RADIAL + .010, task.LANE, geo.ROTOR_HEIGHT + half + .0005])
        tool = SimpleNamespace(pos=task.hub_point(head), rotation=lambda: task.GAUGE_ROTATION)
        rotor = SimpleNamespace(pos=task.HUB, rotation=lambda: task.HUB_ROTATION)
        surface = task.HUB_ROTATION[:, 2]
        face = task.hub_point(head - [0., 0., half + .0005])
        backing = task.hub_point(head + [0., 0., task.LINING-half-.0005])
        stem = face - .014 * task.GAUGE_ROTATION[:, 1]
        native = SimpleNamespace(pairs=[[
            [(0.2, face, surface), (9.0, stem, surface)],
            [(0.3, backing, -surface)],
        ]])
        self.assertEqual(task.forces(0, tool, rotor, native), (.2, .3, 0.))


class InspectionRecordTests(unittest.TestCase):
    def test_only_released_marker_in_the_measured_slot_can_record_result(self):
        from report_board import BOARD_SIZE, RESULT, SLOTS, TOKEN_HEIGHT, record_is_set

        table = .7475
        height = table + BOARD_SIZE[2] + TOKEN_HEIGHT / 2
        stopped = (0., 0., 0.)
        for slot, xy in SLOTS.items():
            with self.subTest(slot=slot):
                self.assertEqual(record_is_set((*xy, height), stopped, stopped, table), slot == RESULT)
        self.assertFalse(record_is_set((*SLOTS[RESULT], height + .03), stopped, stopped, table))
        self.assertFalse(record_is_set((*SLOTS[RESULT], height), (.05, 0., 0.), stopped, table))

    def test_correct_marker_cannot_replace_the_missing_contact_witness(self):
        m = Measurement()
        for tick in range(1, 21):
            result = m.update(tick, True, True, False, True, True, True)
        self.assertEqual(result, .5)
