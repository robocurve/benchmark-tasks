"""Dependency-free tests: fake only import-time yam_arena API, not robot physics."""
from __future__ import annotations
import importlib.util
import pathlib
import sys
import types
import unittest

HERE = pathlib.Path(__file__).resolve().parents[1]
MODULE_PATH = HERE / "safe_edge_recovery.py"

def load_task():
    base = types.ModuleType("yam_arena")
    const = types.ModuleType("yam_arena.constants")
    const.FINGER_CLOSED = 0.0
    const.FINGER_OPEN = 1.0
    demo = types.ModuleType("yam_arena.demo")
    class Waypoint:
        def __init__(self, duration, pose, finger, label):
            self.duration, self.pose, self.finger, self.label = duration, pose, finger, label
    demo.Waypoint = Waypoint
    task_file = types.ModuleType("yam_arena.task_file")
    task_file.TABLE_TOP = {"station": 0.8}
    class Box:
        def __init__(self, name, **kwargs):
            self.name = name
            self.kwargs = kwargs
    task_file.Box = Box
    for name, mod in [
        ("yam_arena", base), ("yam_arena.constants", const),
        ("yam_arena.demo", demo), ("yam_arena.task_file", task_file)
    ]:
        sys.modules[name] = mod
    spec = importlib.util.spec_from_file_location("edge_candidate", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module

M = load_task()

class ContractTests(unittest.TestCase):
    def setUp(self):
        self.mon = M.EdgeRecoveryMonitor()

    def s(self, x, y=M.START[1], low=None, resting=False):
        if low is None:
            low = M.PLATFORM_Z
        return M.Sample((x, y, low + M.BLOCK_EDGE / 2), low, resting)

    def add(self, sample):
        self.mon.update(sample)
        return self.mon.grade(sample)

    def test_static_cube_zero(self):
        self.assertEqual(self.add(self.s(M.START[0], resting=True)), 0.0)

    def test_safe_order_finishes_one(self):
        self.assertEqual(self.add(self.s(M.START[0])), 0.0)
        self.assertEqual(self.add(self.s(.181)), .25)
        self.assertEqual(self.add(self.s(M.RECOVERY_X + .004)), .5)
        self.assertEqual(self.add(self.s(.23, low=M.PLATFORM_Z + .045)), .75)
        self.assertEqual(self.add(self.s(M.GOAL[0], low=M.PLATFORM_Z + .055)), .75)
        for _ in range(M.SETTLE_SAMPLES - 1):
            self.assertEqual(self.add(self.s(M.GOAL[0], low=M.PLATFORM_Z + M.TARGET_SIZE[2], resting=True)), .75)
        self.assertEqual(self.add(self.s(M.GOAL[0], low=M.PLATFORM_Z + M.TARGET_SIZE[2], resting=True)), 1.0)

    def test_early_lift_is_irreversible(self):
        self.add(self.s(M.START[0]))
        self.assertEqual(self.add(self.s(M.START[0], low=M.PLATFORM_Z + .025)), 0)
        self.assertEqual(self.mon.violation, 'premature_lift')
        self.add(self.s(.24))
        self.add(self.s(.24, low=M.PLATFORM_Z + .07))
        self.assertEqual(self.add(self.s(M.GOAL[0], low=M.PLATFORM_Z + .001, resting=True)), 0)

    def test_cannot_cross_edge_and_return(self):
        self.add(self.s(M.START[0]))
        self.assertEqual(self.add(self.s(M.XMIN)), 0)
        self.assertIn(self.mon.violation, ('edge_overhang', 'edge_sweep'))
        self.assertEqual(self.add(self.s(.23)), 0)

    def test_airborne_edge_crossing_is_irreversible(self):
        self.add(self.s(M.START[0]))
        self.add(self.s(M.RECOVERY_X + .005))
        self.assertEqual(self.add(self.s(.24, low=M.PLATFORM_Z + .055)), .75)
        self.assertEqual(self.add(self.s(M.XMAX - .002, low=M.PLATFORM_Z + .055)), 0)
        self.assertEqual(self.mon.violation, 'edge_overhang')
        self.assertEqual(self.add(self.s(M.GOAL[0], low=M.PLATFORM_Z + .001, resting=True)), 0)

    def test_fall_overrides_full_score(self):
        self.add(self.s(M.START[0]))
        self.add(self.s(.225))
        self.add(self.s(.225, low=M.PLATFORM_Z + .05))
        self.assertEqual(self.add(self.s(M.GOAL[0], low=M.PLATFORM_Z - .02)), 0)
        self.assertEqual(self.mon.violation, 'fall_below_platform')

    def test_lifted_without_recovery_never_gets_credit(self):
        self.assertEqual(self.add(self.s(.21, low=M.PLATFORM_Z + .08)), 0)
        self.assertEqual(self.mon.violation, 'premature_lift')

    def test_hover_seven_mm_cannot_claim_supported_recovery(self):
        """Recovery credit requires near-surface support, not low-altitude flight."""
        self.add(self.s(M.START[0]))
        self.assertEqual(self.add(self.s(M.RECOVERY_X + .01, low=M.PLATFORM_Z + .007)), 0)
        self.assertFalse(self.mon.recovered)
        self.assertFalse(self.mon.safe_inward_progress)
        self.assertEqual(self.add(self.s(M.RECOVERY_X + .01, low=M.PLATFORM_Z + .05)), 0)
        self.assertEqual(self.mon.violation, 'premature_lift')

    def test_missing_lift_cannot_get_full_credit(self):
        self.add(self.s(M.START[0]))
        self.add(self.s(.23))
        self.assertEqual(self.add(self.s(M.GOAL[0], low=M.PLATFORM_Z + .001, resting=True)), 0.5)

    def test_unsupported_goal_false(self):
        self.add(self.s(M.START[0]))
        self.add(self.s(.23))
        self.add(self.s(.23, low=M.PLATFORM_Z + .05))
        self.assertEqual(self.add(self.s(M.GOAL[0], low=M.PLATFORM_Z + .001, resting=False)), .75)

    def test_hover_above_pad_not_full_score(self):
        self.add(self.s(M.START[0]))
        self.add(self.s(M.RECOVERY_X + .003))
        self.add(self.s(.23, low=M.PLATFORM_Z + .05))
        for _ in range(M.SETTLE_SAMPLES + 2):
            score = self.add(self.s(M.GOAL[0], low=M.PLATFORM_Z + .009, resting=True))
        self.assertEqual(score, .75)
        self.assertEqual(self.mon.settle_samples, 0)

    def test_placement_dwell_must_be_consecutive(self):
        self.add(self.s(M.START[0]))
        self.add(self.s(M.RECOVERY_X + .004))
        self.add(self.s(M.RECOVERY_X + .004, low=M.PLATFORM_Z + .05))
        for _ in range(M.SETTLE_SAMPLES - 1):
            self.add(self.s(M.GOAL[0], low=M.PLATFORM_Z + M.TARGET_SIZE[2], resting=True))
        self.assertEqual(self.mon.settle_samples, M.SETTLE_SAMPLES - 1)
        self.assertEqual(self.add(self.s(M.GOAL[0], low=M.PLATFORM_Z + .003, resting=False)), .75)
        self.assertEqual(self.mon.settle_samples, 0)

    def test_no_motion_outward_is_still_safe_if_supported(self):
        self.add(self.s(M.START[0]))
        self.assertEqual(self.add(self.s(M.START[0] - .0005)), 0.0)
        self.assertIsNone(self.mon.violation)

    def test_initial_com_near_true_edge_not_outer_face(self):
        self.assertAlmostEqual(M.START[0] - M.XMIN, .003, places=8)
        self.assertGreater(M.XMIN - (M.START[0] - M.BLOCK_EDGE/2), .014)
        self.assertEqual(self.add(self.s(M.START[0])), 0.0)
        self.assertIsNone(self.mon.violation)

    def test_initial_overhang_is_allowed_but_more_is_not(self):
        self.add(self.s(M.START[0]))
        self.assertIsNone(self.mon.violation)
        self.add(self.s(M.START[0] - .0005))
        self.assertIsNone(self.mon.violation)
        self.assertEqual(self.add(self.s(M.START[0] - .0018)), 0)
        self.assertEqual(self.mon.violation, 'edge_overhang')

    def test_yaw_expanded_footprint_fails(self):
        self.add(self.s(M.START[0]))
        widened = M.Sample((M.START[0], M.START[1], M.PLATFORM_Z+M.BLOCK_EDGE/2),
                           M.PLATFORM_Z, False, (.026, .018), 0.0)
        self.assertEqual(self.add(widened), 0.0)
        self.assertEqual(self.mon.violation, 'edge_overhang')

    def test_sweep_does_not_combine_unobserved_pose_components(self):
        """Regression: actual Isaac PhysX frames 3.00/3.05 s are BOTH safe.

        The former max-half-width-at-prior-centre test falsely rejected the
        transition even though neither endpoint crosses the 1-mm limit.
        """
        a = (0.11258808523416519, 0.1305395)
        b = (0.1131009, 0.1304932)
        ah = (0.018371, 0.018334)
        bh = (0.018618, 0.018555)
        self.assertTrue(M.within_edge_envelope(a, ah))
        self.assertTrue(M.within_edge_envelope(b, bh))
        self.assertTrue(M.swept_supported(a, b, ah, bh))
        self.add(M.Sample((a[0], a[1], M.PLATFORM_Z + .018), M.PLATFORM_Z,
                          False, ah, 0.187))
        self.add(M.Sample((b[0], b[1], M.PLATFORM_Z + .018), M.PLATFORM_Z,
                          False, bh, 0.215))
        self.assertIsNone(self.mon.violation)

    def test_sweep_still_rejects_real_outward_overhang(self):
        """The fix must not relax the actual edge or AABB boundary."""
        a = (M.START[0], M.START[1])
        b = (M.START[0] - .0005, M.START[1])
        h = (M.BLOCK_EDGE / 2, M.BLOCK_EDGE / 2)
        expanded = (h[0] + .003, h[1])
        self.assertTrue(M.swept_supported(a, b, h, h))
        self.assertFalse(M.swept_supported(a, b, h, expanded))
        self.assertFalse(M.within_edge_envelope(b, expanded))

    def test_replay_original_318_sample_physics_trace(self):
        """Real user-uploaded Isaac v6 scripted pose trace; no synthetic path.

        Recorded platform top was z=0.8075 m. Test mocks have a different
        TABLE_TOP value, so rebase BOTH object centre and bottom elevations
        without altering x/y, AABB extents, tilt or resting observations.
        """
        import json
        path = HERE / 'tests/fixtures/v6_seed0_scripted_pose.jsonl'
        rows = [json.loads(line) for line in path.read_text().splitlines() if line]
        self.assertEqual(len(rows), 318)
        z_offset = M.PLATFORM_Z - .8075
        milestones = {}
        for row in rows:
            xyz = list(row['xyz'])
            xyz[2] += z_offset
            sample = M.Sample(tuple(xyz), row['low_z'] + z_offset,
                              row['resting'], tuple(row['half_xy']), row['tilt_deg'])
            result = self.add(sample)
            self.assertIsNone(self.mon.violation,
                              f"Unexpected failure at t={row['t']}: {self.mon.violation}")
            if result not in milestones:
                milestones[result] = row['t']
        self.assertEqual(result, 1.0)
        self.assertAlmostEqual(milestones[.25], 3.65, places=2)
        self.assertAlmostEqual(milestones[.5], 4.65, places=2)
        self.assertAlmostEqual(milestones[.75], 6.25, places=2)
        self.assertAlmostEqual(milestones[1.0], 11.0, places=2)

    def test_tilt_is_irreversible(self):
        self.add(self.s(M.START[0]))
        tip = M.Sample((M.START[0], M.START[1], M.PLATFORM_Z + .018),
                       M.PLATFORM_Z, False, (.018, .018), 25.0)
        self.assertEqual(self.add(tip), 0.0)
        self.assertEqual(self.mon.violation, 'unsafe_tilt')

    def test_reenter_hazard_after_recovery_fails(self):
        self.add(self.s(M.START[0]))
        self.add(self.s(M.RECOVERY_X + .005))
        self.assertTrue(self.mon.recovered)
        self.assertEqual(self.add(self.s(M.START[0] + .001)), 0.0)
        self.assertEqual(self.mon.violation, 'reentered_hazard_zone')

    def test_destination_above_cube(self):
        self.assertGreater(M.TARGET_SIZE[2], M.BLOCK_EDGE)
        self.assertLess(M.TARGET_SIZE[2], .060)

    def test_flat_platform_placement_wrong_height(self):
        self.add(self.s(M.START[0]))
        self.add(self.s(M.RECOVERY_X + .003))
        self.add(self.s(M.RECOVERY_X + .003, low=M.PLATFORM_Z + .07))
        for _ in range(M.SETTLE_SAMPLES + 1):
            result=self.add(self.s(M.GOAL[0], low=M.PLATFORM_Z+.001, resting=True))
        self.assertEqual(result, .75)

    def test_non_finite_fail_closed(self):
        self.assertEqual(self.add(M.Sample((float('nan'), 0., 0.), M.PLATFORM_Z)), 0.0)
        self.assertEqual(self.mon.violation, 'invalid_pose')

    def test_reset_clears_violation(self):
        self.add(self.s(M.XMIN))
        self.assertIsNotNone(self.mon.violation)
        self.mon.reset()
        self.assertIsNone(self.mon.violation)
        self.assertEqual(self.add(self.s(M.START[0])), 0)

    def test_runtime_success_score_same_tick_not_double_counted(self):
        M.MONITOR.reset()
        class State:
            def __init__(self, x):
                self.pos = (x, M.START[1], M.PLATFORM_Z + M.BLOCK_EDGE / 2)
                self.low = (x - M.BLOCK_EDGE / 2, M.START[1] - M.BLOCK_EDGE / 2, M.PLATFORM_Z)
            def resting(self):
                return True
        class Runner:
            def __init__(self):
                self.task = object()
                self._t = .05
            def step(self, x):
                obj = {'recovery_block': State(x)}
                return M.success(obj), M.score(obj)
        runner = Runner()
        succeeded, partial = runner.step(M.START[0])
        self.assertFalse(succeeded)
        self.assertEqual(partial, 0)
        self.assertEqual(M.MONITOR.samples, 1)
        runner._t = .10
        runner.step(M.RECOVERY_X)
        self.assertEqual(M.MONITOR.samples, 2)
        self.assertEqual(M.MONITOR.recovered, True)

    def test_runtime_episode_time_rollback_resets_violation(self):
        M.MONITOR.reset()
        class State:
            pos = (M.XMIN, M.START[1], M.PLATFORM_Z + M.BLOCK_EDGE / 2)
            low = (M.XMIN, M.START[1], M.PLATFORM_Z)
            def resting(self):
                return True
        class Runner:
            def __init__(self):
                self.task = object()
                self._t = .1
            def step(self):
                return M.score({'recovery_block': State()})
        runner = Runner()
        runner.step()
        self.assertEqual(M.MONITOR.violation, 'edge_overhang')
        trial = M.MONITOR.trial
        runner._t = 0.
        runner.step()
        self.assertEqual(M.MONITOR.trial, trial + 1)
        self.assertEqual(M.MONITOR.samples, 1)

    def test_yam_task_loader_import_without_sys_modules_registration(self):
        """YAM task_file.load() calls exec_module() without sys.modules registration.

        This reproduces the actual import mechanism rather than the looser
        import mechanism used by the rest of this test suite.
        """
        name = "yam_task_safe_edge_recovery"
        original = sys.modules.pop(name, None)
        try:
            spec = importlib.util.spec_from_file_location(name, MODULE_PATH)
            loaded = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(loaded)
            self.assertEqual(loaded.SCENE, "station")
            self.assertEqual(len(loaded.OBJECTS), 3)
            self.assertTrue(callable(loaded.demo))
            self.assertEqual(len(loaded.demo()["left"]), 11)
        finally:
            if original is not None:
                sys.modules[name] = original

    def test_demo_slides_before_lift(self):
        wp = M.demo()['left']
        self.assertTrue(any('slide' in w.label for w in wp))
        slide_idx = next(i for i,w in enumerate(wp) if 'slide' in w.label)
        lift_idx = next(i for i,w in enumerate(wp) if 'lift only' in w.label)
        self.assertLess(slide_idx, lift_idx)
        self.assertLess(sum(w.duration for w in wp)*20, 1200)

if __name__ == '__main__':
    unittest.main()
