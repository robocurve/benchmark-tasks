"""Checks task.json against the board's rules (AGENTS.md)."""

import json
import os
import re
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
REQUIRED = ["title", "category", "instruction", "setup", "scoring", "run_limit", "fidelity", "scores", "media"]
CATEGORIES = {"Impact on jobs", "Impact on AI timelines", "Safety and alignment", "Societal relatability", "Other"}
IDENTIFYING = [w.strip().lower() for w in os.environ.get("TEAM_WORDS", "").split(",") if w.strip()]


def load():
    return json.loads((HERE / "task.json").read_text())


class TaskJson(unittest.TestCase):
    def test_valid_json_and_fields(self):
        spec = load()
        for field in REQUIRED:
            self.assertIn(field, spec, f"missing required field {field!r}")
        self.assertIn(spec["category"], CATEGORIES)
        self.assertEqual(spec["category"], "Safety and alignment")
        self.assertLessEqual(len(spec["fidelity"]), 500, "fidelity field must be <= 500 characters")
        self.assertIsInstance(spec["run_limit"], str)
        self.assertIn("steps", spec["run_limit"].lower())
        self.assertGreater(len(spec.get("reset_notes", "")), 20)
        self.assertGreaterEqual(len(spec.get("keywords", [])), 2)
        sentences = [s.strip() for s in re.split(r"[.!?]+", spec["instruction"]) if s.strip()]
        self.assertLessEqual(len(sentences), 2)

    def test_instruction_is_goal_only(self):
        text = load()["instruction"].lower()
        for word in ("lever", "rotate", "turn", "clamp", "angle", "bearing", "unlock", "adjust", "degrees"):
            if word == "angle":          # "angle grinder" is the object's name
                continue
            self.assertNotIn(word, text, f"the instruction names the method: {word!r}")

    def test_nothing_identifies_the_team(self):
        if not IDENTIFYING:
            self.skipTest("set TEAM_WORDS to a comma-separated list of words that would identify the team")
        for path in sorted(HERE.rglob("*")):
            if not path.is_file() or "__pycache__" in path.parts or path.suffix in (".png", ".mp4", ".pyc"):
                continue
            text = (path.read_text(errors="ignore") + " " + str(path.relative_to(HERE))).lower()
            for word in IDENTIFYING:
                self.assertNotIn(word, text, f"{path.name} mentions {word!r}: review is blind")

    def test_instruction_is_the_same_everywhere(self):
        instruction = load()["instruction"]
        self.assertIn(instruction, (HERE / "task.py").read_text())
        self.assertIn(instruction, (HERE / "README.md").read_text())
        self.assertIn(instruction, (HERE / "run.sh").read_text())

    def test_ready_to_submit(self):
        spec = load()
        for k in ("scores", "media"):
            if spec.get(k) == "TBD":
                self.fail(f"{k} is still TBD; fill it in before submitting")


if __name__ == "__main__":
    unittest.main()
