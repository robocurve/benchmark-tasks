import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from cover_geometry import ORIGIN
from cover_observations import over_receiver, plate_marks
from cover_scoring import Trial, visual_faults


def observation(x=0, z=0.14):
    pose = [ORIGIN[0] + x, ORIGIN[1], ORIGIN[2] + z, 0, 0, 0, 1]
    corners = plate_marks(pose)
    return dict(
        corners=corners,
        over_receiver=over_receiver(corners),
        stroke=[0, 0],
        drive_travel=0,
        travel=0,
        bottom=z,
        limb_gap=0.08,
        gauge_gap=0.08,
        pin_depth=[-0.05] * 2,
        pin_margin=[0] * 2,
        stage_error=abs(x + 0.22),
        stage_loads=[0, 0],
        rests=[0, 0],
        tilt=0,
        other_support=False,
        contact=True,
        xy_error=abs(x),
        clearance=0.05,
        handles={"left/gripLeft": 1, "right/gripRight": 1},
    )


def frames(tr, row, count):
    for _ in range(count * 12):
        tr.observe(dict(row, tick=tr.tick + 1))


class CameraRules(unittest.TestCase):
    def setUp(self):
        self.tr = Trial()
        stage = observation(-0.22, 0.092)
        stage.update(limb_gap=-0.04, stage_loads=[14, 15])
        frames(self.tr, stage, 1)

    def test_geometric_entry_and_latch(self):
        for x, z, gap, expected in [
            (-0.22, 0.092, 0, False),
            (0, 0.105, 0, False),
            (0, 0.100, 0.035, False),
            (0, 0.100, 0.034, True),
            (-0.158, 0.100, 0, True),
            (-0.165, 0.100, 0, False),
        ]:
            r = observation(x, z)
            r["gauge_gap"] = gap
            self.assertEqual(visual_faults(r)[3], expected)
        frames(self.tr, dict(observation(0, 0.100), limb_gap=0.020), 1)
        frames(self.tr, observation(), 20)
        self.assertEqual(self.tr.reason, "low_plate_at_occupied_receiver")
        self.assertEqual(self.tr.score, 0)

    def test_optical_faults_and_clock(self):
        r = observation()
        r.update(skin_N=9999, finger_load=[999, 999])
        self.assertFalse(any(visual_faults(r)))
        self.assertTrue(visual_faults(dict(r, stroke=[0.003, 0]))[0])
        self.assertTrue(visual_faults(dict(r, stroke=[0, 0.003]))[1])
        for travel in [-0.010, 0.010]:
            self.assertTrue(visual_faults(dict(r, travel=travel))[2])
        self.assertTrue(visual_faults(dict(r, bottom=0.036))[4])
        with self.assertRaises(ValueError):
            self.tr.observe(dict(r, tick=self.tr.tick + 2))
        self.tr.observe(dict(r, stroke=[0.008, 0], tick=self.tr.tick + 1))
        self.assertEqual(self.tr.reason, "")

    def test_holds_seating_and_dwell_loss(self):
        r = observation()
        frames(self.tr, r, 10)
        self.assertEqual(self.tr.score, 0)
        frames(self.tr, r, 1)
        self.assertEqual(self.tr.score, 0.5)
        r = dict(
            observation(0, 0.069),
            rests=[15, 14],
            pin_depth=[0.006, 0.006],
            handles={},
            contact=False,
        )
        frames(self.tr, r, 30)
        self.assertEqual(self.tr.score, 0)
        frames(self.tr, r, 1)
        self.assertEqual(self.tr.score, 1)
        frames(self.tr, dict(r, rests=[1, 0]), 1)
        self.assertEqual(self.tr.score, 0)
        frames(self.tr, dict(r, gauge_gap=0), 1)
        self.assertEqual(self.tr.reason, "low_plate_at_occupied_receiver")

    def test_false_perch_and_motion(self):
        frames(self.tr, observation(), 11)
        r = dict(
            observation(0, 0.069),
            rests=[15, 14],
            contact=False,
            handles={},
            pin_depth=[0.006, 0.002],
        )
        frames(self.tr, r, 40)
        self.assertEqual(self.tr.score, 0)
        for i in range(40):
            frames(self.tr, observation(0.02 if i % 2 else 0), 1)
        self.assertEqual(self.tr.score, 0)


class SlideChecks(unittest.TestCase):
    def test_dry_cycles_and_rejection(self):
        from importlib.util import spec_from_file_location, module_from_spec

        spec = spec_from_file_location(
            "slide", Path(__file__).resolve().parents[1] / "controls/check_slide.py"
        )
        mod = module_from_spec(spec)
        spec.loader.exec_module(mod)
        for profile, start in [("early", 0), ("nominal", 170), ("continuing", 700)]:
            rows = []
            for i in range(601):
                u = max(0, min(1, (i - start) / 160))
                rows.append({"frame": i, "input_mm": 200 * u * u * (3 - 2 * u)})
            mod.check(rows, profile)
            rows[250]["input_mm"] += 15
            with self.assertRaises(ValueError):
                mod.check(rows, profile)
