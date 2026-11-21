"""task.json, README, TECHNICAL.md, validation.json and the code say the same thing. Standard library only."""

import ast
import hashlib
import json
import re
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from rdb_sling_guards import fixed_scene as F  # noqa: E402
from rdb_sling_guards import layout as L  # noqa: E402

TASK = json.loads((ROOT / "task.json").read_text())
REFERENCE = json.loads((ROOT / "REFERENCE-SPEC.json").read_text())
README = (ROOT / "README.md").read_text()
TECHNICAL = (ROOT / "TECHNICAL.md").read_text()
VALIDATION = json.loads((ROOT / "validation.json").read_text())
PACKAGE = ROOT / "rdb_sling_guards"


def hashed_files():
    """run.py's HASHED table, read without importing run.py (it needs Inspect Robots)."""
    tree = ast.parse((PACKAGE / "run.py").read_text())
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(getattr(t, "id", None) == "HASHED" for t in node.targets):
            return ast.literal_eval(node.value)
    raise AssertionError("run.py has no HASHED table")


def run_constant(name):
    tree = ast.parse((PACKAGE / "run.py").read_text())
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(getattr(t, "id", None) == name for t in node.targets):
            return ast.literal_eval(node.value)
    raise AssertionError(f"run.py has no {name}")


class Card(unittest.TestCase):
    def test_fields_and_board_limits(self):
        for key in ("title", "category", "keywords", "instruction", "context", "setup", "scoring", "run_limit",
                    "reset_notes", "fidelity", "scores"):
            self.assertTrue(TASK[key], key)
        for key, limit in (("title", 120), ("setup", 4000), ("context", 1000), ("fidelity", 500), ("scores", 300)):
            self.assertLessEqual(len(TASK[key]), limit, key)

    def test_reference_spec_is_the_same_card(self):
        self.assertEqual({k: v for k, v in TASK.items() if k != "media"},
                         {k: v for k, v in REFERENCE.items() if k != "media"})

    def test_instruction_and_run_limit_match_the_code(self):
        self.assertEqual(TASK["instruction"], F.INSTRUCTION)
        self.assertIn(f"{F.MAX_STEPS:,} steps", TASK["run_limit"])
        self.assertIn(f"{F.MAX_STEPS // F.CONTROL_HZ} s at {F.CONTROL_HZ} Hz", TASK["run_limit"])

    def test_no_names_of_people_teams_or_organisations_in_the_task_text(self):
        text = " ".join(str(TASK[k]) for k in ("title", "instruction", "setup", "scoring")).lower()
        for word in ("university", "team", "@", "github.com/"):
            self.assertNotIn(word, text)

    def test_context_cites_the_osha_rule(self):
        self.assertIn("1910.184(c)(7)", TASK["context"])
        self.assertIn("padded or protected from the sharp edges of their loads", TASK["context"])

    def test_guards_are_not_labelled_as_safety_items(self):
        for path in (ROOT / "cad").glob("*.stl"):
            self.assertNotIn(b"SAFETY", path.read_bytes()[:80].upper())


class Code(unittest.TestCase):
    def test_hashed_files_exist(self):
        for key, name in hashed_files().items():
            self.assertTrue((PACKAGE / name).is_file(), name)

    def test_dream_entry_and_request_cap(self):
        self.assertEqual(run_constant("MODEL"), "dreamlabs/dream-1-beta")
        cap = run_constant("DREAM_REQUEST_CAP")
        self.assertGreaterEqual(cap * 24, F.MAX_STEPS)     # Dream-1 returns about 24 steps a request

    def test_pyproject_entry_points(self):
        text = (ROOT / "pyproject.toml").read_text()
        for entry in ("yam_sling_guards", "sling_guards_scripted", "sling_guards_score", "rigging.usda"):
            self.assertIn(entry, text)


class Docs(unittest.TestCase):
    def test_third_party_section(self):
        for text in (README, TECHNICAL):
            self.assertIn("## Third-party assets and licenses" if text is README else "Third-party", text)
        for asset in ("YAM arm model", "Station", "Overhead camera", "Side camera", "Isaac Sim 6.1", "Inspect Robots"):
            self.assertIn(asset, README)

    def test_readme_states_the_instruction(self):
        self.assertIn(F.INSTRUCTION[:60], README.replace("\n", " "))

    def test_guard_is_described_as_printed_and_why(self):
        for text in (README + TECHNICAL, TASK["fidelity"]):
            self.assertIn("6 in", text)

    def test_hook_is_the_published_part(self):
        for text in (TECHNICAL, TASK["setup"]):
            self.assertIn("CondorLift", text)


class Evidence(unittest.TestCase):
    def test_scripted_run_scores_full_marks(self):
        scripted = VALIDATION["runs"]["scripted"]
        self.assertEqual(scripted["score"], 1.0)
        self.assertLessEqual(scripted["executed_steps"], F.MAX_STEPS)
        self.assertTrue(scripted.get("repeat_trace_identical"))

    def test_controls_score_as_they_must(self):
        want = {"no_guards": 0., "top_only": 0., "guards_only": .4, "not_past_latch": .4, "one_fitting": .6}
        controls = VALIDATION["controls"]
        for name, score in want.items():
            self.assertAlmostEqual(controls[name]["score"], score, msg=name)

    def test_recorded_hashes_match_the_code(self):
        recorded = VALIDATION["recorded_hashes"]
        for key, name in hashed_files().items():
            self.assertEqual(recorded[key], hashlib.sha256((PACKAGE / name).read_bytes()).hexdigest(), name)
        self.assertEqual(recorded["asset_sha256"], hashlib.sha256(F.USD.read_bytes()).hexdigest())


if __name__ == "__main__":
    unittest.main()
