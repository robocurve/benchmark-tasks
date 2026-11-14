"""Checks task.json against the Robocurve rules (AGENTS.md) and against the other files."""

import json
import os
import re
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
REQUIRED = ["title", "category", "instruction", "setup", "scoring", "run_limit", "fidelity", "scores", "media"]
CATEGORIES = {"Impact on jobs", "Impact on AI timelines", "Safety and alignment", "Societal relatability", "Other"}
IDENTIFYING = [w.strip().lower() for w in (os.environ.get("TEAM_WORDS", "")).split(",")
               if w.strip()]


def load():
    return json.loads((HERE / "task.json").read_text())


class TaskJson(unittest.TestCase):
    def test_fields_and_category(self):
        spec = load()
        for field in REQUIRED:
            self.assertIn(field, spec, f"missing required field {field!r}")
        self.assertEqual(spec["category"], "Safety and alignment")
        self.assertLessEqual(len(spec["scores"]), 300)
        self.assertIn(spec["category"], CATEGORIES)
        self.assertLessEqual(len(spec["fidelity"]), 500)
        self.assertIn("steps", spec["run_limit"].lower())
        self.assertGreater(len(spec["reset_notes"]), 20)
        self.assertGreaterEqual(len(spec["keywords"]), 2)

    def test_instruction_is_goal_only_and_short(self):
        instruction = load()["instruction"]
        self.assertLessEqual(len([s for s in re.split(r"[.!?]+", instruction) if s.strip()]), 2)
        low = instruction.lower()
        for word in ("cap", "caps", "press", "push", "seat"):
            self.assertNotIn(word, low, f"the instruction must not give the method ({word!r})")

    def test_instruction_is_the_same_everywhere(self):
        instruction = load()["instruction"]
        for name in ("task.py", "README.md", "run.sh"):
            if (HERE / name).exists():
                self.assertIn(instruction, (HERE / name).read_text(), name)

    def test_nothing_identifies_the_team(self):
        for path in HERE.rglob("*"):
            if path.suffix not in {".json", ".md", ".py", ".sh", ".usda", ".txt"} or "__pycache__" in path.parts:
                continue
            if path.name == "test_task_json.py":
                continue
            text = path.read_text(errors="ignore").lower()
            for word in IDENTIFYING:
                self.assertNotIn(word, text, f"{path.name} mentions {word!r}: review is blind")

    def test_ready_to_submit(self):
        spec = load()
        for k in ("scores", "media"):
            if spec.get(k) == "TBD":
                self.fail(f"{k} is still TBD; fill it in before submitting")


if __name__ == "__main__":
    unittest.main()
