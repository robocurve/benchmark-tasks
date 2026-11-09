"""Checks task.json against Robocurve competition rules."""

import json
import os
import re
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
REQUIRED = ["title", "category", "instruction", "setup", "scoring", "run_limit", "fidelity", "scores", "media"]
CATEGORIES = {"Impact on jobs", "Impact on AI timelines", "Safety and alignment", "Societal relevance", "Other"}
IDENTIFYING = [w.strip().lower() for w in os.environ.get("TEAM_WORDS", "").split(",") if w.strip()]


def load():
    return json.loads((HERE / "task.json").read_text())


class TaskJson(unittest.TestCase):
    def test_valid_json_and_category(self):
        spec = load()
        for field in REQUIRED:
            self.assertIn(field, spec, f"missing required field {field!r}")
        self.assertIn(spec["category"], CATEGORIES)
        self.assertEqual(spec["category"], "Impact on jobs")
        self.assertLessEqual(len(spec["fidelity"]), 500, "fidelity field must be <= 500 characters")

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
        self.assertIn(instruction, (HERE / "README.md").read_text())

    def test_ready_to_submit(self):
        spec = load()
        for k in ("scores", "media"):
            if spec.get(k) == "TBD":
                self.fail(f"{k} is still TBD; fill it in before submitting")


if __name__ == "__main__":
    unittest.main()
