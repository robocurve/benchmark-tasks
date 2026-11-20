"""task.json, README, TECHNICAL.md, validation.json and the code say the same thing. Standard library only."""

import ast
import hashlib
import json
import math
import re
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from rdb_tlc_spotting import fixed_scene as F  # noqa: E402
from rdb_tlc_spotting import grading  # noqa: E402
from rdb_tlc_spotting import layout as L  # noqa: E402

TASK = json.loads((ROOT / "task.json").read_text())
REFERENCE = json.loads((ROOT / "REFERENCE-SPEC.json").read_text())
README = (ROOT / "README.md").read_text()
TECHNICAL = (ROOT / "TECHNICAL.md").read_text() if (ROOT / "TECHNICAL.md").exists() else ""
VALIDATION = json.loads((ROOT / "validation.json").read_text())
PACKAGE = ROOT / "rdb_tlc_spotting"


def mm(value):
    """Metres to whole millimetres, halves rounded up (as written in task.json)."""
    return str(math.floor(round(1000 * value, 6) + .5))


def hashed_files():
    tree = ast.parse((PACKAGE / "run.py").read_text())
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(getattr(t, "id", None) == "HASHED" for t in node.targets):
            return ast.literal_eval(node.value)
    raise AssertionError("run.py has no HASHED table")


class Card(unittest.TestCase):
    def test_fields_and_board_limits(self):
        for key in ("title", "category", "keywords", "instruction", "context", "setup", "scoring", "run_limit",
                    "reset_notes", "fidelity", "scores"):
            self.assertTrue(TASK[key], key)
        self.assertEqual(TASK["category"], "Other")
        for key, limit in (("title", 120), ("setup", 4000), ("context", 1000), ("fidelity", 500), ("scores", 300)):
            self.assertLessEqual(len(TASK[key]), limit, key)
        self.assertEqual({k: v for k, v in TASK.items() if k != "media"},  # the board's upload tool adds a media ID to task.json
                         {k: v for k, v in REFERENCE.items() if k != "media"})

    def test_instruction_and_run_limit_match_the_code(self):
        self.assertEqual(TASK["instruction"], F.INSTRUCTION)
        self.assertIn("Counting from the robot's left, the first spotter goes with the first vial and the first mark",
                      F.INSTRUCTION)
        self.assertIn(F.INSTRUCTION, README)
        self.assertTrue(TASK["run_limit"].startswith(f"{F.MAX_STEPS:,} steps ({F.MAX_STEPS // 20} s at 20 Hz)"))
        self.assertEqual(F.CONTROL_HZ, grading.CONTROL_HZ)

    def test_setup_numbers_match_the_code(self):
        setup = TASK["setup"]
        self.assertIn(f"{mm(L.PLATE_WIDE)} x {mm(L.PLATE_LEN)} mm", setup)
        self.assertIn(f"centred at ({mm(L.PLATE_X)}, {mm(L.PLATE_Y)})", setup)
        self.assertIn(f"baseline {mm(L.BASELINE_UP)} mm above the bottom edge", setup)
        x = mm(L.PLATE_X - L.PLATE_LEN / 2 + L.BASELINE_UP)
        ticks = [f"({x}, {mm(L.PLATE_Y + dy)})" for dy in L.LANE_Y]
        self.assertIn(f"at {ticks[0]}, {ticks[1]} and {ticks[2]}", setup)
        self.assertIn(f"{L.CAP_OD * 1000:.1f} mm OD x {L.CAP_ID * 1000:.2f} mm ID x {mm(L.CAP_LEN)} mm", setup)
        holes = ", ".join(f"({mm(L.RACK_X)}, {mm(y)})" for y in L.HOLE_Y)
        self.assertIn(holes, setup)
        vials = ", ".join(f"({mm(L.VIAL_X)}, {mm(y)})" for y in L.VIAL_Y)
        self.assertIn(vials, setup)
        self.assertIn(f"centred at ({mm(L.TANK_X)}, {mm(L.TANK_Y)})", setup)
        self.assertIn(f"({mm(L.TANK[1])} mm side to side, {mm(L.TANK[0])} mm front to back, {mm(L.TANK[2])} mm tall", setup)
        self.assertIn(f"about {mm(L.TANK_IN[1])} x {mm(L.TANK_IN[0])} mm inside", setup)
        self.assertIn(f"{L.SOLVENT_ML:.0f} mL of water, {mm(L.SOLVENT_DEPTH)} mm deep", setup)
        self.assertIn(f"about {mm(L.LID[1])} x {mm(L.LID[0])} x {mm(L.LID[2])} mm", setup)
        self.assertIn(f"({mm(L.LID_X)}, {mm(L.LID_Y)})", setup)
        self.assertIn("Sigma-Aldrich Z146226", setup)
        self.assertIn("Nichols", setup)

    def test_scoring_numbers_match_the_code(self):
        scoring = TASK["scoring"]
        self.assertIn("Twelve checks, 1/12 each", scoring)
        self.assertEqual(grading.ELEMENTS, 12)
        self.assertIn(f"(b) the spot is at most {mm(L.SPOT_MAX_D)} mm across (half the tick spacing)", scoring)
        self.assertAlmostEqual(L.LANE_Y[0] - L.LANE_Y[1], 2 * L.SPOT_MAX_D)
        self.assertIn(f"against the {mm(L.LANE_Y[0] - L.LANE_Y[1])} mm tick spacing", scoring)
        self.assertIn("A phone on a stand outside the arms' reach films the plate", scoring)
        self.assertIn("it is not a robot input", TASK["setup"])
        self.assertIn(f"more than {L.DROP_TILT_DEG:.0f} degrees from upright with no gripper holding it", scoring)
        self.assertIn("lying face up is not a zero", scoring)
        self.assertIn(f"no bare glass longer than {mm(L.BARE_MAX)} mm within {mm(L.LANE_HALF)} mm of the tick", scoring)
        self.assertIn(f"the baseline at least {mm(L.BASELINE_CLEAR)} mm above the water", scoring)
        self.assertIn(f"at most {L.LEAN_MAX_DEG:.0f} degrees from vertical", scoring)
        self.assertIn(f"centred within {mm(L.LID_CENTRE)} mm, level within {L.LID_TILT_DEG:.0f} degrees and square to the "
                      f"tank within {L.LID_YAW_DEG:.0f} degrees", scoring)
        self.assertIn(f"{mm(L.CLEAR_M)} mm clear of the tank and its lid", scoring)
        self.assertIn("at most 11/12", scoring)
        self.assertIn(f"tipped past {L.TANK_TIP_DEG:.0f} degrees or pushed more than {mm(L.TANK_SHIFT)} mm", scoring)
        self.assertIn("spotted face away from the wall", scoring)

    def test_fidelity_numbers_match_the_code(self):
        fidelity = TASK["fidelity"]
        self.assertIn(f"{mm(L.DIP_MIN)} mm under a sample for {L.FILL_S} s", fidelity)
        self.assertIn(f"{L.TOUCH_MIN_N} to {L.TOUCH_MAX_N} N held {L.DWELL_S} s", fidelity)
        self.assertIn(f"sqrt({1e6 * L.SPOT_D0 ** 2:.0f} + {1e6 * L.SPOT_GROWTH:.0f} t) mm", fidelity)
        self.assertIn(f"over {L.BREAK_N:.0f} N on a spotter from anything but the pads breaks it", fidelity)
        self.assertIn("Thresholds are estimates", fidelity)
        self.assertIn("the foam rack is rigid", fidelity)
        self.assertIn(f"{L.CAP_MASS * 1000:.0f} g", fidelity)


class Documents(unittest.TestCase):
    def test_code_describes_the_scene_it_builds(self):
        adapter = (PACKAGE / "adapter.py").read_text()
        self.assertNotIn("150 mL", adapter)
        self.assertIn("rectangular glass TLC developing tank", adapter)
        self.assertIn("## Third-party assets and licenses", README)
        self.assertIn("OmniGlass.mdl", README)

    def test_readme_has_the_run_commands(self):
        for text in ("uv pip install --no-deps .", "python -m rdb_tlc_spotting.run scripted --out out/scripted",
                     "python tools/dryrun_demo.py", "RESULT.json", "tlc_spotting_score"):
            self.assertIn(text, README, text)

    def test_validation_and_hashes(self):
        scripted = VALIDATION["runs"]["scripted"]
        if scripted:
            self.assertEqual(scripted["score"], 1.)
            self.assertLessEqual(scripted["executed_steps"], F.MAX_STEPS)
            self.assertIn(f"{scripted['executed_steps']:,} steps", README)
            recorded = VALIDATION["recorded_hashes"]
            for key, name in hashed_files().items():
                self.assertEqual(recorded[key], hashlib.sha256((PACKAGE / name).read_bytes()).hexdigest(), name)
            self.assertEqual(recorded["asset_sha256"], hashlib.sha256(F.USD.read_bytes()).hexdigest())
        dream = VALIDATION["runs"]["dream"]
        self.assertEqual((dream["status"], dream["score"], dream["model"]), ("run", 0., "dreamlabs/dream-1-beta"))
        self.assertLessEqual(dream["executed_steps"], dream["step_limit"])
        self.assertEqual(dream["step_limit"], F.MAX_STEPS)
        self.assertTrue(dream["code_hashes_match_recorded"])
        self.assertLessEqual(dream["dream_action_requests"], dream["request_cap"])
        self.assertTrue(TASK["scores"].startswith(
            f"Dream-1 run: {dream['score']:.1f} ({dream['executed_steps']:,} of {dream['step_limit']:,} steps; "))
        self.assertNotIn("not yet run", README + TECHNICAL + TASK["scores"] + VALIDATION["status"])
        result_path = ROOT / dream["result"]
        if result_path.is_file():
            result = json.loads(result_path.read_text())
            self.assertEqual((result["score"], result["executed_steps"], result["termination_reason"]),
                             (dream["score"], dream["executed_steps"], dream["termination_reason"]))
            for key, value in VALIDATION["recorded_hashes"].items():
                self.assertEqual(result[key], value, key)

    def test_pyproject_entry_points_and_pins(self):
        text = (ROOT / "pyproject.toml").read_text()
        self.assertIn('requires-python = ">=3.12,<3.13"', text)
        for line in ('yam_tlc_spotting = "rdb_tlc_spotting.adapter:make_embodiment"',
                     'tlc_spotting_scripted = "rdb_tlc_spotting.adapter:make_scripted"',
                     'tlc_spotting_score = "rdb_tlc_spotting.grading:make_scorer"',
                     'rdb_tlc_spotting = ["tlc.usda"]', '"inspect-robots==1.0.0"', '"yam-arena==0.1.0"'):
            self.assertIn(line, text)
        self.assertTrue((ROOT / "LICENSE").read_text().startswith("MIT License"))

    def test_text_does_not_identify_anyone(self):
        for text in (json.dumps(TASK), README, TECHNICAL, json.dumps(VALIDATION)):
            self.assertIsNone(re.search(r"/U" r"sers/|(?<!/en)/h" r"ome/|@[a-z]+\.(edu|com)\b", text))
            for word in ("JH" "U", "Hop" "kins", "T-" "75", "gary" "23ai", "lam" "bda-yam"):
                self.assertNotIn(word, text)


if __name__ == "__main__":
    unittest.main()
