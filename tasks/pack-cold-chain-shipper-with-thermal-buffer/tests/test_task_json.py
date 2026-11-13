"""Checks task.json against Robocurve competition rules."""

from __future__ import annotations

import json
import os
import re
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
REQUIRED = [
    "title",
    "category",
    "instruction",
    "setup",
    "scoring",
    "run_limit",
    "fidelity",
    "scores",
    "media",
]
CATEGORIES = {
    "Impact on jobs",
    "Impact on AI timelines",
    "Safety and alignment",
    "Societal relevance",
    "Other",
}
IDENTIFYING = [w.strip().lower() for w in os.environ.get("TEAM_WORDS", "").split(",") if w.strip()]


def load() -> dict:
    return json.loads((HERE / "task.json").read_text())


class TaskJson(unittest.TestCase):
    def test_valid_json_and_category(self):
        spec = load()
        for field in REQUIRED:
            self.assertIn(field, spec, f"missing required field {field!r}")
        self.assertIn(spec["category"], CATEGORIES)
        self.assertEqual(spec["category"], "Safety and alignment")
        self.assertLessEqual(len(spec["fidelity"]), 500, "fidelity field must be <= 500 characters")
        self.assertLessEqual(len(spec["scores"]), 300, "scores field must be <= 300 characters")

        self.assertIsInstance(spec["run_limit"], str, "run_limit must be a string")
        self.assertIn("steps", spec["run_limit"].lower())
        self.assertIn("reset_notes", spec)
        self.assertGreater(len(spec["reset_notes"]), 20)
        self.assertIn("keywords", spec)
        self.assertIsInstance(spec["keywords"], list)
        self.assertGreaterEqual(len(spec["keywords"]), 2)

        # Instruction <= 2 sentences
        sentences = [s.strip() for s in re.split(r"[.!?]+", spec["instruction"]) if s.strip()]
        self.assertLessEqual(len(sentences), 2, f"Instruction must be at most 2 sentences, got {len(sentences)}")

    def test_nothing_identifies_the_team(self):
        if not IDENTIFYING:
            self.skipTest("set TEAM_WORDS to a comma-separated list of words that would identify the team")
        text = json.dumps(load()).lower()
        for word in IDENTIFYING:
            self.assertNotIn(word, text, f"task.json mentions {word!r}: review is blind")

    def test_instruction_is_the_same_everywhere(self):
        instruction = load()["instruction"]
        self.assertIn(instruction, (HERE / "run.sh").read_text())
        self.assertIn(instruction, (HERE / "task.py").read_text())
        if (HERE / "README.md").exists():
            self.assertIn(instruction, (HERE / "README.md").read_text())

    def test_numbers_agree_with_geometry_and_readme(self):
        import sys

        sys.path.insert(0, str(HERE))
        import geometry as g

        spec = load()
        c = spec["clearances"]
        mm = 1000.0
        pairs = {
            "cavity_length_y_mm": g.CAVITY_LENGTH_Y,
            "cavity_width_x_mm": g.CAVITY_WIDTH_X,
            "cavity_depth_z_mm": g.CAVITY_DEPTH_Z,
            "brick_clearance_per_side_x_mm": g.BRICK_CLEARANCE_X,
            "brick_clearance_per_side_y_mm": g.BRICK_CLEARANCE_Y,
            "buffer_clearance_per_side_x_mm": g.BUFFER_CLEARANCE_X,
            "buffer_clearance_per_side_y_mm": g.BUFFER_CLEARANCE_Y,
            "carton_clearance_per_side_x_mm": g.CARTON_CLEARANCE_X,
            "carton_clearance_per_side_y_mm": g.CARTON_CLEARANCE_Y,
        }
        for key, metres in pairs.items():
            self.assertAlmostEqual(c[key], metres * mm, places=3, msg=key)
        self.assertAlmostEqual(
            c["minimum_fit_clearance_mm"],
            min(v for k, v in c.items() if "per_side" in k and k.endswith("_y_mm")),
            places=3,
        )
        gr = spec["grasp"]
        self.assertEqual(gr["brick_size_mm"], [g.BRICK_WIDTH_X * mm, g.BRICK_LENGTH_Y * mm, g.BRICK_HEIGHT_Z * mm])
        self.assertEqual(gr["buffer_size_mm"], [g.BUFFER_WIDTH_X * mm, g.BUFFER_LENGTH_Y * mm, g.BUFFER_HEIGHT_Z * mm])
        self.assertEqual(gr["carton_size_mm"], [g.CARTON_WIDTH_X * mm, g.CARTON_LENGTH_Y * mm, g.CARTON_HEIGHT_Z * mm])
        self.assertAlmostEqual(gr["max_pinch_width_mm"], g.MAX_PINCH_WIDTH * mm, places=3)
        self.assertAlmostEqual(gr["pad_tip_below_tcp_mm"], g.PAD_TIP_PAST_TCP * mm, places=3)
        self.assertAlmostEqual(gr["pad_tip_above_surface_mm"], g.PAD_GAP * mm, places=3)
        self.assertAlmostEqual(gr["release_gap_mm"], -2.0 * g.FINGER_RELEASE * mm, places=3)
        self.assertAlmostEqual(gr["grip_gap_command_mm"], -2.0 * g.FINGER_GRIP * mm, places=3)

        text = spec["setup"]
        readme = (HERE / "README.md").read_text() if (HERE / "README.md").exists() else ""
        for token in ("172 mm", "150 mm", "70 mm", "152 mm", "76 mm", "25 mm", "74 mm", "15 mm", "130 mm", "0.33 kg", "0.04 kg", "0.18 kg"):
            self.assertIn(token, text, f"task.json setup lacks {token}")
            if readme:
                self.assertIn(token, readme, f"README lacks {token}")
        for old in ("162 mm", "112 mm", "0.45 kg", "Polar Tech"):
            self.assertNotIn(old, json.dumps(spec))
            self.assertNotIn(old, readme)
        self.assertNotIn("$", readme)
        self.assertNotIn("\\(", readme)

    def test_lid_and_latch_numbers_agree_with_geometry(self):
        import sys

        sys.path.insert(0, str(HERE))
        import geometry as g
        import predict_scores

        spec = load()
        mm = 1000.0
        gr, c, lt = spec["grasp"], spec["clearances"], spec["latches"]
        for got, want in zip(gr["lid_size_mm"], (g.LID_WIDTH_X, g.LID_LENGTH_Y, g.LID_THICK_Z)):
            self.assertAlmostEqual(got, want * mm, places=3)
        for got, want in zip(gr["lid_tab_size_mm"], (g.LID_TAB_X, g.LID_TAB_Y, g.LID_TAB_Z)):
            self.assertAlmostEqual(got, want * mm, places=3)
        self.assertAlmostEqual(gr["tab_grip_gap_command_mm"], -2.0 * g.FINGER_GRIP_TAB * mm, places=3)
        self.assertAlmostEqual(gr["lid_pad_tip_above_top_face_mm"], g.LID_PAD_GAP * mm, places=3)
        self.assertAlmostEqual(c["lid_above_carton_mm"], (g.LID_SEAT_BOTTOM_Z - g.CARTON_TOP_SEATED_Z) * mm, places=3)
        self.assertAlmostEqual(c["latch_head_over_lid_edge_mm"], g.LATCH_OVERLAP * mm, places=3)
        self.assertAlmostEqual(c["latch_head_above_seated_lid_mm"], g.LATCH_GAP_ABOVE_LID * mm, places=3)
        self.assertAlmostEqual(c["latch_travel_mm"], g.LATCH_TRAVEL * mm, places=3)
        self.assertEqual(lt["count"], 2)
        self.assertAlmostEqual(lt["spring_n_per_mm"], g.LATCH_RATE / mm, places=6)
        self.assertAlmostEqual(lt["damping_n_s_per_m"], g.LATCH_DAMPING, places=6)
        self.assertAlmostEqual(lt["travel_mm"], g.LATCH_TRAVEL * mm, places=3)
        self.assertAlmostEqual(lt["head_radius_mm"], g.LATCH_RADIUS * mm, places=3)
        self.assertAlmostEqual(lt["head_overlap_mm"], g.LATCH_OVERLAP * mm, places=3)
        self.assertAlmostEqual(lt["width_mm"], g.LATCH_WIDTH_X * mm, places=3)
        self.assertAlmostEqual(lt["slider_mass_kg"], g.LATCH_MASS, places=6)
        self.assertAlmostEqual(lt["lid_friction"], g.LID_FRICTION, places=6)
        self.assertAlmostEqual(lt["shipper_friction"], g.SHIPPER_FRICTION, places=6)
        text = spec["setup"]
        readme = (HERE / "README.md").read_text()
        for token in ("170 mm", "192 mm", "8 mm", "0.12 kg", "40 mm", "30 mm", "22 mm", "0.5 N/mm", "4 mm", "y=+0.420", "y=-0.420"):
            self.assertIn(token, text, f"task.json setup lacks {token}")
        for token in ("170 mm", "192 mm", "0.12 kg", "40 mm", "22 mm", "0.5 N/mm", "8 mm travel", "+0.420", "-0.420", "0.3"):
            self.assertIn(token, readme, f"README lacks {token}")
        self.assertEqual(spec["predicted_scores"], predict_scores.predict_all())
        for mode in ("lid_unlatched", "no_lid"):
            self.assertIn(mode, spec["predicted_scores"])
            self.assertIn(mode, readme)
            self.assertIn(mode, (HERE / "run.sh").read_text())
        self.assertLess(spec["predicted_scores"]["lid_unlatched"], 1.0)
        self.assertEqual(spec["predicted_scores"]["no_lid"], 0.75)
        self.assertRegex(spec["media"], r"^M-[A-Z0-9]{6}$")

    def test_simulation_claims_are_honest(self):
        readme = (HERE / "README.md").read_text().lower()
        text = (json.dumps(load()) + readme).lower()
        for banned in ("models temperature", "simulates temperature", "heat flow is modelled"):
            self.assertNotIn(banned, text)
        self.assertIn("not modelled: temperature", readme)
        self.assertIn("latch", readme)
        # the instruction never states the safety action or the closing action
        for word in ("buffer", "latch", "lid", "close", "freez"):
            self.assertNotIn(word, load()["instruction"].lower())

    def test_readme_is_plain_text(self):
        readme = (HERE / "README.md").read_text()
        for bad in ("\\frac", "\\times", "$$"):
            self.assertNotIn(bad, readme)

    def test_ready_to_submit(self):
        spec = load()
        for k in ("scores", "media"):
            if spec.get(k) == "TBD":
                self.fail(f"{k} is still TBD; fill it in before submitting")


if __name__ == "__main__":
    unittest.main()
