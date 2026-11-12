"""Unit tests validating task.json schema and fields for segregate-incompatible-cleaners-in-cabinet."""

from __future__ import annotations

import json
from pathlib import Path
import re
import unittest

HERE = Path(__file__).resolve().parent.parent
REQUIRED = {
    "title",
    "category",
    "instruction",
    "setup",
    "context",
    "scoring",
    "run_limit",
    "fidelity",
    "reset_notes",
    "keywords",
    "scores",
    "media",
}
CATEGORIES = {"Safety and alignment", "Impact on jobs", "Impact on AI timelines", "Societal relevance"}


class TaskJson(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with open(HERE / "task.json", encoding="utf-8") as f:
            cls.spec = json.load(f)

    def test_required_fields_present_and_well_formed(self):
        spec = self.spec
        for field in REQUIRED:
            self.assertIn(field, spec, f"missing required field {field!r}")
        self.assertIn(spec["category"], CATEGORIES)
        self.assertEqual(spec["category"], "Safety and alignment")
        self.assertLessEqual(len(spec["fidelity"]), 500, "fidelity field must be <= 500 characters")

        self.assertIsInstance(spec["run_limit"], str, "run_limit must be a string")
        self.assertRegex(spec["run_limit"], r"^\d+\s*steps?$", "run_limit must look like '1200 steps'")

        self.assertIsInstance(spec["keywords"], list)
        self.assertGreaterEqual(len(spec["keywords"]), 2)

        sentences = [s.strip() for s in re.split(r"[.!?]+", spec["instruction"]) if s.strip()]
        self.assertLessEqual(len(sentences), 2, f"Instruction must be at most 2 sentences, got {len(sentences)}")

        words = spec["instruction"].split()
        self.assertGreaterEqual(len(words), 8, f"Instruction too short: {spec['instruction']!r}")
        self.assertLessEqual(len(words), 40, f"Instruction too long: {spec['instruction']!r}")

    def test_no_team_words_in_text_fields(self):
        import os

        team = [w.strip() for w in os.environ.get("TEAM_WORDS", "").split(",") if w.strip()]
        if not team:
            return
        pattern = re.compile(r"\b(" + "|".join(re.escape(w) for w in team) + r")\b", re.IGNORECASE)
        for field in ("title", "instruction", "setup", "context", "scoring", "fidelity", "reset_notes"):
            val = self.spec.get(field, "")
            m = pattern.search(val)
            if m is not None:
                self.fail(f"Team word {m.group(0)!r} leaked into task.json {field!r}")

    def test_ready_to_submit(self):
        for k in ("scores", "media"):
            if self.spec.get(k) == "TBD":
                self.fail(f"{k} is still TBD; fill it in before submitting")


if __name__ == "__main__":
    unittest.main()
