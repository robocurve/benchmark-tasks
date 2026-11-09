"""Scoring boundary and chart-rotation checks; synthetic data is not optics proof."""
import copy
import json
import math
from pathlib import Path
import tempfile
import unittest

import numpy as np
from PIL import Image

try:
    from .inspection_measure import measure_group
    from .score_hold import audit_run
    from .scoring import REVIEW_CLAIMS, check_review, clearance_audit
except ImportError:
    from inspection_measure import measure_group
    from score_hold import audit_run
    from scoring import REVIEW_CLAIMS, check_review, clearance_audit


class NativePixelTests(unittest.TestCase):
    @staticmethod
    def chart(angle, scale=4, missing=False):
        theta = math.radians(angle)
        affine = np.array([[scale*math.cos(theta), scale*math.sin(theta)],
                           [-scale*math.sin(theta), scale*math.cos(theta)], [64., 64.]])
        yy, xx = np.indices((129, 129))
        world = (np.stack((xx, yy), axis=-1)-affine[2])@np.linalg.inv(affine[:2])
        gray = np.full((129, 129), .95)
        for i in range(5):
            if missing and i == 2:
                continue
            inside = (np.abs(world[..., 0]-(-4.8+i*2.4)) <= .6) & (np.abs(world[..., 1]) <= 1.6)
            gray[inside] = .05
        group = {"name": "vertical", "orientation": "vertical", "center_mm": [0., 0.], "size_mm": [10.8, 3.2]}
        return gray, group, affine

    def test_clear_pattern_at_camera_roll_angles(self):
        for angle in (0, 5, 15, 30, 44, 45, 46, 60, 89, 90, 135, 180, 225, 270):
            with self.subTest(angle=angle):
                gray, group, affine = self.chart(angle)
                self.assertTrue(measure_group(gray, group, affine, .05, .95)["readable"])

    def test_unresolved_native_pattern_cannot_gain_pixels_from_rotation(self):
        for angle in (0, 15, 45, 90):
            gray, group, affine = self.chart(angle, scale=.6)
            self.assertFalse(measure_group(gray, group, affine, .05, .95)["readable"])

    def test_missing_middle_bar_and_low_contrast_fail(self):
        for angle in (0, 30, 45, 90):
            gray, group, affine = self.chart(angle, missing=True)
            self.assertFalse(measure_group(gray, group, affine, .05, .95)["readable"])
            gray, group, affine = self.chart(angle)
            gray = .49+.02*(gray-.05)/.9
            self.assertFalse(measure_group(gray, group, affine, .05, .95)["readable"])

    def test_occlusion_on_only_one_transect_fails(self):
        gray, group, affine = self.chart(0)
        gray[66, 44:85] = .95
        self.assertFalse(measure_group(gray, group, affine, .05, .95)["readable"])


class PhysicalReviewTests(unittest.TestCase):
    @staticmethod
    def trace(z=.9):
        return [{"step": i, "time_s": i/20, "camera_eye": [0., 0., 1.],
                 "camera_rotation": np.eye(3).tolist(), "tool_pose_xyzw": [0., 0., z, 0., 0., 0., 1.]} for i in range(61)]

    def test_clearance_uses_lowest_point_not_handle_origin(self):
        trace = self.trace(.7475+.025)
        result = clearance_audit(trace, {"steps": 60})
        self.assertTrue(result["valid"])
        self.assertFalse(result["passes"])
        self.assertAlmostEqual(result["minimum_clearance_m"], .016)

    def test_clearance_fails_for_one_low_sample(self):
        trace = self.trace()
        trace[30]["tool_pose_xyzw"][2] = .775
        result = clearance_audit(trace, {"steps": 60})
        self.assertFalse(result["passes"])
        self.assertEqual(result["minimum_step"], 30)

    def test_tampered_reported_clearance_is_invalid(self):
        trace = self.trace()
        trace[20]["tool_table_clearance_m"] = 1.
        self.assertFalse(clearance_audit(trace, {"steps": 60})["valid"])

    def test_review_bound_to_evidence_and_known_observations(self):
        hashes = {"trace": "abc", "video": "def"}
        review = {"schema": "magnifier-physical-review-v1", "reviewer": "test reviewer",
                  "reviewed_full_trial": True, "source_sha256": hashes, "interval_steps": [0, 60],
                  "observations": dict.fromkeys(REVIEW_CLAIMS, True)}
        self.assertTrue(check_review(review, hashes, [0, 60])["passes"])
        tampered = copy.deepcopy(review); tampered["source_sha256"]["trace"] = "changed"
        self.assertFalse(check_review(tampered, hashes, [0, 60])["valid"])
        unknown = copy.deepcopy(review); unknown["observations"]["picked_up_by_handle"] = None
        self.assertFalse(check_review(unknown, hashes, [0, 60])["valid"])
        failed = copy.deepcopy(review); failed["observations"]["picked_up_by_handle"] = False
        result = check_review(failed, hashes, [0, 60])
        self.assertTrue(result["valid"])
        self.assertFalse(result["passes"])

    def test_complete_blank_camera_trial_is_valid_zero_but_missing_frame_is_not(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            raw = np.full((360, 640, 3), 128, np.uint8)
            policy = np.full((224, 224, 3), 128, np.uint8)
            trace = self.trace()
            record = {"complete": True, "steps": 60, "final_state": trace[-1],
                      "optical_comparison": {"same_camera_pose": True, "same_physics_step": 100,
                                              "actual_policy_resize_verified": True}}
            (root/"pickup.json").write_text(json.dumps(record))
            (root/"trace.jsonl").write_text("\n".join(json.dumps(s) for s in trace))
            target = {"fiducials": [{"name": name, "xy_mm": [i%2, i//2]} for i, name in enumerate(("red", "green", "blue", "magenta"))]}
            (root/"target.json").write_text(json.dumps(target))
            for label in ("baseline", "final"):
                np.savez(root/(label+".npz"), raw=raw, policy=policy)
                Image.fromarray(raw).save(root/(label+"-raw.png")); Image.fromarray(policy).save(root/(label+"-policy.png"))
            for i in range(61):
                for kind, data in (("raw", raw), ("policy", policy)):
                    Image.fromarray(data).save(root/f"hold-{i:04d}-{kind}.png")
            result = audit_run(root, root/"target.json")
            self.assertTrue(result["certified"], result["reasons"])
            self.assertEqual(result["photo_hold_score"], 0)
            self.assertEqual(len(result["unmeasurable_target_steps"]), 61)
            (root/"hold-0030-raw.png").unlink()
            result = audit_run(root, root/"target.json")
            self.assertFalse(result["certified"])
            self.assertEqual(result["photo_hold_score"], 0)


if __name__ == "__main__":
    unittest.main()
